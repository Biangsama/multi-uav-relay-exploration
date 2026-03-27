#pragma once

#include <connector_core/metrics_logger.hpp>

#include <compression_util.hpp>
#include <protobuf_msgs/dancers_update.pb.h>
#include <ros/ros.h>
#include <time_probe.hpp>
#include <uds_tcp_socket.hpp>
#include <yaml_util.hpp>

#include <filesystem>
#include <memory>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

class Connector
{
public:
    explicit Connector(const std::string& node_name)
        : nh_(), pnh_("~"), node_name_(node_name)
    {
        std::string config_file_path;
        pnh_.param<std::string>("config_file", config_file_path, std::string());

        if (config_file_path.empty() || access(config_file_path.c_str(), F_OK) != 0)
        {
            throw std::runtime_error(
                "The config file was not found. A private parameter '~config_file' must be set.");
        }

        const char* ros_ws = getenv("ROS_WS");
        if (ros_ws == nullptr)
        {
            throw std::runtime_error("ROS_WS environment variable not set.");
        }
        ros_ws_path = ros_ws;

        config_ = YAML::LoadFile(config_file_path);

        it = 0;
        simulation_length = config_["simulation_length"].as<double>();
        sync_window = config_["sync_window"].as<unsigned int>() / 1000000.0f;

        verbose_ = (config_["verbose"] ? config_["verbose"].as<bool>() : false);
        log_payload_bytes_ =
            (config_["log_payload_bytes"] ? config_["log_payload_bytes"].as<bool>() : false);
        log_service_timing_ =
            (config_["log_service_timing"] ? config_["log_service_timing"].as<bool>() : false);
        enable_metrics_ = (config_["enable_metrics"] ? config_["enable_metrics"].as<bool>() : false);
        metrics_flush_every_n_ =
            (config_["flush_every_n"] ? config_["flush_every_n"].as<size_t>() : 50);

        save_compute_time =
            (config_["save_compute_time"] ? config_["save_compute_time"].as<bool>() : false);

        const std::string metrics_dir =
            (config_["metrics_dir"] ? config_["metrics_dir"].as<std::string>() : "logs");
        const std::string exp_name =
            (config_["experiment_name"] ? config_["experiment_name"].as<std::string>() : "exp");

        if (enable_metrics_)
        {
            std::filesystem::path out_dir = std::filesystem::path(ros_ws_path) / metrics_dir / exp_name;
            std::filesystem::path out_csv = out_dir / (node_name_ + std::string(".csv"));

            metrics_ = std::make_unique<dancers::metrics::CsvMetricsLogger>(
                out_csv.string(),
                std::vector<std::string>{
                    "wall_ts_us", "it", "t_sim_sec",
                    "rx_comp_B", "rx_raw_B", "rx_payload_B",
                    "step_us",
                    "tx_raw_B", "tx_comp_B", "tx_payload_B"
                },
                metrics_flush_every_n_);

            std::filesystem::path out_phy_csv = out_dir / "coordinator_phy.csv";
            metrics_phy_ = std::make_unique<dancers::metrics::CsvMetricsLogger>(
                out_phy_csv.string(),
                std::vector<std::string>{
                    "wall_ts_us", "t_sim_sec",
                    "tx_raw_B", "tx_comp_B", "tx_payload_B",
                    "rx_raw_B", "rx_comp_B", "rx_payload_B",
                    "rtt_us"
                },
                metrics_flush_every_n_);
        }
    }

    virtual ~Connector() = default;

protected:
    static bool IsExpectedSocketShutdown_(const std::string& message)
    {
        return message.find("Connection reset by peer") != std::string::npos ||
               message.find("Broken pipe") != std::string::npos ||
               message.find("End of file") != std::string::npos ||
               message.find("Operation canceled") != std::string::npos ||
               message.find("Bad file descriptor") != std::string::npos ||
               message.find("stream closed") != std::string::npos;
    }

    void Loop()
    {
        CustomSocket* socket = nullptr;
        boost::asio::io_context io_context;

        if (use_uds)
        {
            socket = new UDSSocket(io_context);
            socket->connect(uds_server_address);
        }
        else
        {
            socket = new TCPSocket(io_context);
            socket->connect(ip_server_address);
        }

        while (ros::ok())
        {
            std::string rx_comp;
            try
            {
                rx_comp = socket->receive_one_message();
            }
            catch (const std::exception& e)
            {
                if (ros::isShuttingDown() || IsExpectedSocketShutdown_(e.what()))
                {
                    ROS_INFO("[%s] receive interrupted during shutdown: %s",
                             node_name_.c_str(),
                             e.what());
                    break;
                }
                throw;
            }
            const size_t rx_comp_bytes = rx_comp.size();

            std::string received_data = gzip_decompress(rx_comp);
            const size_t rx_raw_bytes = received_data.size();

            const double t_sim = (step_size > 0.0) ? static_cast<double>(it) * step_size
                                                   : static_cast<double>(it) * sync_window;

            dancers_update_proto::DancersUpdate update_msg;
            update_msg.ParseFromString(received_data);

            if (verbose_ && log_payload_bytes_)
            {
                ROS_INFO("[%s] it=%lu t_sim=%.6f RX: comp=%zuB raw=%zuB update.payload=%zuB",
                         node_name_.c_str(),
                         static_cast<unsigned long>(it),
                         t_sim,
                         rx_comp_bytes,
                         rx_raw_bytes,
                         static_cast<size_t>(update_msg.payload().size()));
            }

            if (update_msg.msg_type() == dancers_update_proto::DancersUpdate::CLOSE)
            {
                break;
            }

            WallTimeProbe local_step_probe;
            local_step_probe.start();

            if (save_compute_time) probe.start();
            dancers_update_proto::DancersUpdate response_update_msg = StepSimulation(update_msg);
            if (save_compute_time) probe.stop();

            local_step_probe.stop();
            const uint64_t step_us = local_step_probe.get_elapsed_time();

            if (verbose_)
            {
                ROS_INFO("[%s] it=%lu StepSimulation wall_time=%lu us",
                         node_name_.c_str(),
                         static_cast<unsigned long>(it),
                         static_cast<unsigned long>(step_us));
            }

            ++it;

            std::string tx_raw;
            response_update_msg.SerializeToString(&tx_raw);
            const size_t tx_raw_bytes = tx_raw.size();

            std::string tx_comp = gzip_compress(tx_raw);
            const size_t tx_comp_bytes = tx_comp.size();

            if (verbose_ && log_payload_bytes_)
            {
                ROS_INFO("[%s] it=%lu t_sim=%.6f TX: raw=%zuB comp=%zuB resp.payload=%zuB",
                         node_name_.c_str(),
                         static_cast<unsigned long>(it - 1),
                         t_sim,
                         tx_raw_bytes,
                         tx_comp_bytes,
                         static_cast<size_t>(response_update_msg.payload().size()));
            }

            if (metrics_)
            {
                metrics_->log_row(
                    dancers::metrics::now_us(),
                    it,
                    t_sim,
                    rx_comp_bytes,
                    rx_raw_bytes,
                    static_cast<size_t>(update_msg.payload().size()),
                    step_us,
                    tx_raw_bytes,
                    tx_comp_bytes,
                    static_cast<size_t>(response_update_msg.payload().size()));
            }

            try
            {
                socket->send_one_message(tx_comp);
            }
            catch (const std::exception& e)
            {
                if (ros::isShuttingDown() || IsExpectedSocketShutdown_(e.what()))
                {
                    ROS_INFO("[%s] send interrupted during shutdown: %s",
                             node_name_.c_str(),
                             e.what());
                    break;
                }
                throw;
            }
        }

        try
        {
            socket->close();
        }
        catch (const std::exception& e)
        {
            if (!(ros::isShuttingDown() || IsExpectedSocketShutdown_(e.what())))
            {
                throw;
            }
        }
        delete socket;
    }

    virtual void ReadConfigFile() = 0;
    virtual dancers_update_proto::DancersUpdate StepSimulation(
        dancers_update_proto::DancersUpdate update_msg) = 0;

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    std::string node_name_;

    std::unique_ptr<dancers::metrics::CsvMetricsLogger> metrics_phy_;
    bool enable_metrics_{true};
    size_t metrics_flush_every_n_{50};
    std::unique_ptr<dancers::metrics::CsvMetricsLogger> metrics_;

    bool verbose_{true};
    bool log_payload_bytes_{true};
    bool log_service_timing_{true};

    YAML::Node config_;

    double sync_window{0.0};
    double step_size{0.0};
    double simulation_length{0.0};
    uint64_t it{0};
    uint64_t it_end_sim{0};

    std::thread loop_thread_;

    bool save_compute_time{false};
    std::string m_computation_time_file;
    WallTimeProbe probe;

    bool use_uds{true};
    std::string uds_server_address;
    std::string ip_server_address;
    unsigned int ip_server_port{0};

    std::string ros_ws_path;
};
