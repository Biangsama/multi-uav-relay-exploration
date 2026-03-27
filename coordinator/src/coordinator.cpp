#include <algorithm>
#include <chrono>
#include <thread>

#include <connector_core/metrics_logger.hpp>
#include <compression_util.hpp>
#include <protobuf_msgs/dancers_update.pb.h>
#include <protobuf_msgs/net_rx_events.pb.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <time_probe.hpp>
#include <uds_tcp_socket.hpp>
#include <yaml_util.hpp>

#include <boost/fiber/barrier.hpp>
#include <yaml-cpp/yaml.h>

#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
bool IsExpectedSocketShutdown(const std::string& message)
{
    return message.find("Connection reset by peer") != std::string::npos ||
           message.find("Broken pipe") != std::string::npos ||
           message.find("End of file") != std::string::npos ||
           message.find("Operation canceled") != std::string::npos ||
           message.find("Bad file descriptor") != std::string::npos ||
           message.find("stream closed") != std::string::npos;
}
}  // namespace

class Coordinator
{
public:
    Coordinator()
        : nh_(), pnh_("~"), rendezvous_threads_(3)
    {
        std::string config_file_path;
        pnh_.param<std::string>("config_file", config_file_path, std::string());

        if (config_file_path.empty() || access(config_file_path.c_str(), F_OK) != 0)
        {
            ROS_ERROR("The config file was not found. A private parameter '~config_file' is required.");
            std::exit(EXIT_FAILURE);
        }

        const char* ros_ws = getenv("ROS_WS");
        if (ros_ws == nullptr)
        {
            ROS_FATAL("ROS_WS environment variable not set, aborting.");
            std::exit(EXIT_FAILURE);
        }
        ros_ws_path_ = ros_ws;

        config_ = YAML::LoadFile(config_file_path);
        enable_metrics_ = (config_["enable_metrics"] ? config_["enable_metrics"].as<bool>() : false);
        metrics_flush_every_n_ =
            (config_["flush_every_n"] ? config_["flush_every_n"].as<size_t>() : 50);
        const std::string metrics_dir =
            (config_["metrics_dir"] ? config_["metrics_dir"].as<std::string>() : "logs");
        const std::string exp_name =
            (config_["experiment_name"] ? config_["experiment_name"].as<std::string>() : "exp");

        if (enable_metrics_)
        {
            std::filesystem::path out_dir = std::filesystem::path(ros_ws_path_) / metrics_dir / exp_name;
            std::filesystem::create_directories(out_dir);

            metrics_net_ = std::make_unique<dancers::metrics::CsvMetricsLogger>(
                (out_dir / "coordinator_net.csv").string(),
                std::vector<std::string>{
                    "wall_ts_us", "t_sim_sec",
                    "tx_raw_B", "tx_comp_B", "tx_payload_B",
                    "rx_raw_B", "rx_comp_B", "rx_payload_B",
                    "rtt_us"
                },
                metrics_flush_every_n_);

            metrics_phy_ = std::make_unique<dancers::metrics::CsvMetricsLogger>(
                (out_dir / "coordinator_phy.csv").string(),
                std::vector<std::string>{
                    "wall_ts_us", "t_sim_sec",
                    "tx_raw_B", "tx_comp_B", "tx_payload_B",
                    "rx_raw_B", "rx_comp_B", "rx_payload_B",
                    "rtt_us"
                },
                metrics_flush_every_n_);
        }

        verbose_ = (config_["verbose"] ? config_["verbose"].as<bool>() : true);
        log_payload_bytes_ = (config_["log_payload_bytes"] ? config_["log_payload_bytes"].as<bool>() : true);
        log_rtt_ = verbose_;

        if (config_["sync_window"].as<uint32_t>() % config_["phy_step_size"].as<uint32_t>() != 0 ||
            config_["sync_window"].as<uint32_t>() % config_["net_step_size"].as<uint32_t>() != 0)
        {
            ROS_FATAL("sync_window must be divisible by both phy_step_size and net_step_size.");
            std::exit(EXIT_FAILURE);
        }

        current_sim_time_ = ros::Time(0);
        clock_publisher_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", 1);

        phy_protobuf_thread_ = std::thread(&Coordinator::run_phy_protobuf_client_, this);
        net_protobuf_thread_ = std::thread(&Coordinator::run_net_protobuf_client_, this);
        real_time_thread_ = std::thread(&Coordinator::run_real_time_thread_, this);

        phy_protobuf_thread_.join();
        net_protobuf_thread_.join();
        real_time_thread_.join();

        if (config_["save_compute_time"] && config_["save_compute_time"].as<bool>())
        {
            probe_ = WallTimeProbe(getYamlValue<std::string>(config_, "results_folder") +
                                   "/compute_time_coordinator.csv");
            probe_.start();
        }

        ROS_INFO("Coordinator node finished successfully, exiting.");
    }

private:
    static ros::Time secondsToRosTime(int64_t seconds)
    {
        ros::Time t;
        t.fromNSec(static_cast<uint64_t>(seconds) * 1000000000ull);
        return t;
    }

    std::string MergeCompressedNetRxEventsBatches_(
        const std::deque<std::string>& compressed_batches) const
    {
        if (compressed_batches.empty())
        {
            return "";
        }

        protobuf_msgs::NetRxEventsPayload merged;
        uint64_t min_window_start_us = std::numeric_limits<uint64_t>::max();
        uint64_t max_window_end_us = 0;

        for (const auto& compressed_batch : compressed_batches)
        {
            if (compressed_batch.empty())
            {
                continue;
            }

            const std::string raw_batch = gzip_decompress(compressed_batch);
            protobuf_msgs::NetRxEventsPayload batch;
            if (!batch.ParseFromString(raw_batch))
            {
                throw std::runtime_error("Coordinator failed to parse queued NetRxEventsPayload.");
            }

            min_window_start_us =
                std::min<uint64_t>(min_window_start_us, static_cast<uint64_t>(batch.t_window_start_us()));
            max_window_end_us =
                std::max<uint64_t>(max_window_end_us, static_cast<uint64_t>(batch.t_window_end_us()));

            for (int i = 0; i < batch.events_size(); ++i)
            {
                *merged.add_events() = batch.events(i);
            }
        }

        if (merged.events_size() == 0)
        {
            return "";
        }

        if (min_window_start_us == std::numeric_limits<uint64_t>::max())
        {
            min_window_start_us = 0;
        }
        merged.set_t_window_start_us(min_window_start_us);
        merged.set_t_window_end_us(max_window_end_us);

        std::string merged_raw;
        merged.SerializeToString(&merged_raw);
        return gzip_compress(merged_raw);
    }

    void publish_clock_()
    {
        rosgraph_msgs::Clock clock_msg;
        clock_msg.clock = current_sim_time_;
        clock_publisher_.publish(clock_msg);
    }

    void run_phy_protobuf_client_()
    {
        ROS_DEBUG("Starting PHY protobuf client thread.");

        try
        {
            CustomSocket* socket = nullptr;
            boost::asio::io_context io_context;
            if (getYamlValue<bool>(config_, "phy_use_uds"))
            {
                socket = new UDSSocket(io_context);
                socket->accept(getYamlValue<std::string>(config_, "phy_uds_server_address"), 0);
            }
            else
            {
                socket = new TCPSocket(io_context);
                socket->accept(config_["phy_ip_server_address"].as<std::string>(),
                               config_["phy_ip_server_port"].as<unsigned short>());
            }

            ROS_INFO("Connected PHY socket.");

            const ros::Time simulation_length =
                secondsToRosTime(getYamlValue<int64_t>(config_, "simulation_length"));
            const uint32_t sync_window = getYamlValue<uint32_t>(config_, "sync_window");
            const uint32_t phy_step_size = getYamlValue<uint32_t>(config_, "phy_step_size");
            const uint32_t net_step_size = getYamlValue<uint32_t>(config_, "net_step_size");
            const bool save_compute_time = getYamlValue<bool>(config_, "save_compute_time");

            while (current_sim_time_ < simulation_length || simulation_length == ros::Time(0))
            {
                for (uint32_t i = 0; i < sync_window / phy_step_size; ++i)
                {
                    dancers_update_proto::DancersUpdate network_update_msg;
                    network_update_msg.set_msg_type(dancers_update_proto::DancersUpdate::BEGIN);
                    std::deque<std::string> pending_net_rx_events_batches;

                    {
                        std::lock_guard<std::mutex> lock(network_to_physics_data_mutex_);
                        if (!compressed_network_to_physics_data_.empty())
                        {
                            network_update_msg.set_payload(compressed_network_to_physics_data_);
                            compressed_network_to_physics_data_.clear();
                        }
                        pending_net_rx_events_batches.swap(
                            compressed_network_to_physics_net_rx_events_gz_queue_);
                    }

                    if (!pending_net_rx_events_batches.empty())
                    {
                        const std::string merged_batch =
                            MergeCompressedNetRxEventsBatches_(pending_net_rx_events_batches);
                        network_update_msg.set_net_rx_events_gz(merged_batch);
                    }

                    std::string req_raw = network_update_msg.SerializeAsString();
                    std::string req_comp = gzip_compress(req_raw);

                    if (verbose_)
                    {
                        ROS_INFO("[COORD][PHY] forwarding net_rx_events_gz=%zuB to PHY",
                                 network_update_msg.net_rx_events_gz().size());
                    }

                    WallTimeProbe rtt_probe;
                    rtt_probe.start();
                    socket->send_one_message(req_comp);
                    std::string resp_comp = socket->receive_one_message();
                    rtt_probe.stop();
                    const uint64_t rtt_us = rtt_probe.get_elapsed_time();

                    std::string response = gzip_decompress(resp_comp);

                    if (verbose_ && log_payload_bytes_)
                    {
                        const double t_sim_sec = static_cast<double>(current_sim_time_.toNSec()) / 1e9;
                        ROS_INFO(
                            "[COORD][PHY] t_sim=%.6f TX(comp=%zuB raw=%zuB payload=%zuB) "
                            "RX(comp=%zuB raw=%zuB) RTT=%lu us",
                            t_sim_sec,
                            req_comp.size(),
                            req_raw.size(),
                            static_cast<size_t>(network_update_msg.payload().size()),
                            resp_comp.size(),
                            response.size(),
                            static_cast<unsigned long>(log_rtt_ ? rtt_us : 0));
                    }

                    if (response.empty())
                    {
                        ROS_INFO("Empty message from PHY Connector");
                        continue;
                    }

                    dancers_update_proto::DancersUpdate physics_update_msg;
                    physics_update_msg.ParseFromString(response);
                    const size_t rx_payload_bytes = physics_update_msg.payload().size();

                    if (physics_update_msg.msg_type() != dancers_update_proto::DancersUpdate::END)
                    {
                        throw std::runtime_error(
                            "Coordinator received a non-END message from PHY Connector.");
                    }

                    if (phy_step_size <= net_step_size)
                    {
                        current_sim_time_ += ros::Duration(static_cast<double>(phy_step_size) / 1e6);
                        publish_clock_();
                    }

                    {
                        std::lock_guard<std::mutex> lock(physics_to_network_data_mutex_);
                        if (metrics_phy_)
                        {
                            const double t_sim_sec = static_cast<double>(current_sim_time_.toNSec()) / 1e9;
                            metrics_phy_->log_row(
                                dancers::metrics::now_us(),
                                t_sim_sec,
                                req_raw.size(),
                                req_comp.size(),
                                static_cast<size_t>(network_update_msg.payload().size()),
                                response.size(),
                                resp_comp.size(),
                                rx_payload_bytes,
                                rtt_us);
                        }
                        compressed_physics_to_network_data_ = physics_update_msg.payload();
                    }
                }

                if (save_compute_time)
                {
                    probe_.stop();
                    probe_.start();
                }

                rendezvous_threads_.wait();
            }

            dancers_update_proto::DancersUpdate physics_update_msg;
            physics_update_msg.set_msg_type(dancers_update_proto::DancersUpdate::CLOSE);
            socket->send_one_message(gzip_compress(physics_update_msg.SerializeAsString()));
            socket->close();
            delete socket;

            ROS_INFO("Simulation finished (PHY thread).");
        }
        catch (const std::exception& e)
        {
            if (ros::isShuttingDown() || IsExpectedSocketShutdown(e.what()))
            {
                ROS_INFO("PHY thread exiting during shutdown: %s", e.what());
                return;
            }
            ROS_ERROR("%s", e.what());
            std::exit(EXIT_FAILURE);
        }
        catch (...)
        {
            if (ros::isShuttingDown())
            {
                ROS_INFO("PHY thread exiting during shutdown.");
                return;
            }
            ROS_ERROR("Error happened in the Physics protobuf thread.");
            std::exit(EXIT_FAILURE);
        }
    }

    void run_net_protobuf_client_()
    {
        ROS_DEBUG("Starting NET protobuf client thread.");

        try
        {
            CustomSocket* socket = nullptr;
            boost::asio::io_context io_context;
            if (getYamlValue<bool>(config_, "net_use_uds"))
            {
                socket = new UDSSocket(io_context);
                socket->accept(getYamlValue<std::string>(config_, "net_uds_server_address"), 0);
            }
            else
            {
                socket = new TCPSocket(io_context);
                socket->accept(config_["net_ip_server_address"].as<std::string>(),
                               config_["net_ip_server_port"].as<unsigned short>());
            }

            ROS_INFO("Connected NET socket.");

            const ros::Time simulation_length =
                secondsToRosTime(getYamlValue<int64_t>(config_, "simulation_length"));
            const uint32_t sync_window = getYamlValue<uint32_t>(config_, "sync_window");
            const uint32_t phy_step_size = getYamlValue<uint32_t>(config_, "phy_step_size");
            const uint32_t net_step_size = getYamlValue<uint32_t>(config_, "net_step_size");
            const bool save_compute_time = getYamlValue<bool>(config_, "save_compute_time");

            while (current_sim_time_ < simulation_length || simulation_length == ros::Time(0))
            {
                for (uint32_t i = 0; i < sync_window / net_step_size; ++i)
                {
                    dancers_update_proto::DancersUpdate physics_update_msg;
                    physics_update_msg.set_msg_type(dancers_update_proto::DancersUpdate::BEGIN);

                    {
                        std::lock_guard<std::mutex> lock(physics_to_network_data_mutex_);
                        if (!compressed_physics_to_network_data_.empty())
                        {
                            physics_update_msg.set_payload(compressed_physics_to_network_data_);
                            compressed_physics_to_network_data_.clear();
                        }
                    }

                    std::string req_raw = physics_update_msg.SerializeAsString();
                    std::string req_comp = gzip_compress(req_raw);

                    WallTimeProbe rtt_probe;
                    rtt_probe.start();
                    socket->send_one_message(req_comp);
                    std::string resp_comp = socket->receive_one_message();
                    rtt_probe.stop();
                    const uint64_t rtt_us = rtt_probe.get_elapsed_time();

                    std::string response = gzip_decompress(resp_comp);

                    if (verbose_ && log_payload_bytes_)
                    {
                        const double t_sim_sec = static_cast<double>(current_sim_time_.toNSec()) / 1e9;
                        ROS_INFO(
                            "[COORD][NET] t_sim=%.6f TX(comp=%zuB raw=%zuB payload=%zuB) "
                            "RX(comp=%zuB raw=%zuB) RTT=%lu us",
                            t_sim_sec,
                            req_comp.size(),
                            req_raw.size(),
                            static_cast<size_t>(physics_update_msg.payload().size()),
                            resp_comp.size(),
                            response.size(),
                            static_cast<unsigned long>(log_rtt_ ? rtt_us : 0));
                    }

                    if (response.empty())
                    {
                        ROS_INFO("Empty message from NET Connector");
                        continue;
                    }

                    dancers_update_proto::DancersUpdate network_update_msg;
                    network_update_msg.ParseFromString(response);
                    const size_t rx_payload_bytes = network_update_msg.payload().size();
                    const size_t rx_events_gz_bytes = network_update_msg.net_rx_events_gz().size();

                    if (verbose_)
                    {
                        ROS_INFO("[COORD][NET] received net_rx_events_gz=%zuB from NET",
                                 rx_events_gz_bytes);
                    }

                    if (network_update_msg.msg_type() != dancers_update_proto::DancersUpdate::END)
                    {
                        throw std::runtime_error(
                            "Coordinator received a non-END message from NET Connector.");
                    }

                    if (net_step_size < phy_step_size)
                    {
                        current_sim_time_ += ros::Duration(static_cast<double>(net_step_size) / 1e6);
                        publish_clock_();
                    }

                    {
                        std::lock_guard<std::mutex> lock(network_to_physics_data_mutex_);
                        if (metrics_net_)
                        {
                            const double t_sim_sec = static_cast<double>(current_sim_time_.toNSec()) / 1e9;
                            metrics_net_->log_row(
                                dancers::metrics::now_us(),
                                t_sim_sec,
                                req_raw.size(),
                                req_comp.size(),
                                static_cast<size_t>(physics_update_msg.payload().size()),
                                response.size(),
                                resp_comp.size(),
                                rx_payload_bytes,
                                rtt_us);
                        }

                        compressed_network_to_physics_data_ = network_update_msg.payload();
                        if (!network_update_msg.net_rx_events_gz().empty())
                        {
                            compressed_network_to_physics_net_rx_events_gz_queue_.push_back(
                                network_update_msg.net_rx_events_gz());
                        }
                    }
                }

                if (save_compute_time)
                {
                    probe_.stop();
                    probe_.start();
                }

                rendezvous_threads_.wait();
            }

            dancers_update_proto::DancersUpdate physics_update_msg;
            physics_update_msg.set_msg_type(dancers_update_proto::DancersUpdate::CLOSE);
            socket->send_one_message(gzip_compress(physics_update_msg.SerializeAsString()));
            socket->close();
            delete socket;

            ROS_INFO("Simulation finished (NET thread).");
        }
        catch (const std::exception& e)
        {
            if (ros::isShuttingDown() || IsExpectedSocketShutdown(e.what()))
            {
                ROS_INFO("NET thread exiting during shutdown: %s", e.what());
                return;
            }
            ROS_ERROR("%s", e.what());
            std::exit(EXIT_FAILURE);
        }
        catch (...)
        {
            if (ros::isShuttingDown())
            {
                ROS_INFO("NET thread exiting during shutdown.");
                return;
            }
            ROS_ERROR("Error happened in the Network protobuf thread.");
            std::exit(EXIT_FAILURE);
        }
    }

    void run_real_time_thread_()
    {
        ROS_DEBUG("Starting real time factor thread.");

        double rtf = std::numeric_limits<double>::infinity();
        try
        {
            rtf = getYamlValue<double>(config_, "real_time_factor");
        }
        catch (const std::runtime_error&)
        {
            ROS_WARN("real_time_factor not found in config, running at max speed.");
        }

        try
        {
            const ros::Time simulation_length =
                secondsToRosTime(getYamlValue<int64_t>(config_, "simulation_length"));
            const uint32_t sync_window = getYamlValue<uint32_t>(config_, "sync_window");
            const uint64_t time_to_sleep = static_cast<uint64_t>(sync_window / rtf);

            while (current_sim_time_ < simulation_length || simulation_length == ros::Time(0))
            {
                std::this_thread::sleep_for(std::chrono::microseconds(time_to_sleep));
                rendezvous_threads_.wait();
            }
        }
        catch (const std::exception& e)
        {
            if (ros::isShuttingDown())
            {
                ROS_INFO("Real time thread exiting during shutdown: %s", e.what());
                return;
            }
            ROS_ERROR("%s", e.what());
            std::exit(EXIT_FAILURE);
        }
        catch (...)
        {
            if (ros::isShuttingDown())
            {
                ROS_INFO("Real time thread exiting during shutdown.");
                return;
            }
            ROS_ERROR("Error happened in the Real time factor thread.");
            std::exit(EXIT_FAILURE);
        }
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    std::unique_ptr<dancers::metrics::CsvMetricsLogger> metrics_net_;
    std::unique_ptr<dancers::metrics::CsvMetricsLogger> metrics_phy_;
    bool enable_metrics_{false};
    size_t metrics_flush_every_n_{50};

    YAML::Node config_;
    bool verbose_{true};
    bool log_payload_bytes_{true};
    bool log_rtt_{true};

    ros::Time current_sim_time_;
    ros::Publisher clock_publisher_;
    boost::fibers::barrier rendezvous_threads_;

    std::string compressed_physics_to_network_data_;
    std::string compressed_network_to_physics_data_;
    std::deque<std::string> compressed_network_to_physics_net_rx_events_gz_queue_;

    std::mutex physics_to_network_data_mutex_;
    std::mutex network_to_physics_data_mutex_;

    std::thread phy_protobuf_thread_;
    std::thread net_protobuf_thread_;
    std::thread real_time_thread_;

    std::string ros_ws_path_;
    WallTimeProbe probe_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "coordinator");
    Coordinator coordinator;
    return 0;
}
