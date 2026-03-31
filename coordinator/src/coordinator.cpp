#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <unistd.h>

#include <compression_util.hpp>
#include <connector_core/metrics_logger.hpp>
#include <nav_msgs/Odometry.h>
#include <protobuf_msgs/agent_state_batch.pb.h>
#include <protobuf_msgs/dancers_update.pb.h>
#include <protobuf_msgs/net_rx_events.pb.h>
#include <protobuf_msgs/racer_swarm_msg.pb.h>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <rosgraph_msgs/Clock.h>
#include <time_probe.hpp>
#include <uds_tcp_socket.hpp>
#include <yaml-cpp/yaml.h>
#include <yaml_util.hpp>

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

double HeadingFromQuaternion(const geometry_msgs::Quaternion& q)
{
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

std::vector<int> ParseCsvInts(const std::string& csv)
{
    std::vector<int> values;
    std::stringstream ss(csv);
    std::string item;
    while (std::getline(ss, item, ','))
    {
        item.erase(std::remove_if(item.begin(), item.end(), ::isspace), item.end());
        if (item.empty())
        {
            continue;
        }
        values.push_back(std::stoi(item));
    }
    return values;
}
} // namespace

class Coordinator
{
public:
    Coordinator()
        : nh_(), pnh_("~")
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

        const uint32_t sync_window = getYamlValue<uint32_t>(config_, "sync_window");
        const uint32_t phy_step_size = getYamlValue<uint32_t>(config_, "phy_step_size");
        const uint32_t net_step_size = getYamlValue<uint32_t>(config_, "net_step_size");
        pnh_.param("strict_lockstep", strict_lockstep_, true);
        if (strict_lockstep_)
        {
            if (!(sync_window == phy_step_size && sync_window == net_step_size))
            {
                ROS_FATAL("strict_lockstep requires sync_window == phy_step_size == net_step_size.");
                std::exit(EXIT_FAILURE);
            }
        }
        else if (sync_window % phy_step_size != 0 || sync_window % net_step_size != 0)
        {
            ROS_FATAL("sync_window must be divisible by both phy_step_size and net_step_size.");
            std::exit(EXIT_FAILURE);
        }
        step_size_us_ = sync_window;

        current_sim_time_ = ros::Time(0);
        clock_publisher_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", 1);

        pnh_.param("trace_swarm_msg_enable", trace_swarm_msg_enable_, false);
        pnh_.param<std::string>("trace_swarm_msg_family", trace_swarm_msg_family_, std::string());
        pnh_.param("trace_swarm_msg_src_id", trace_swarm_msg_src_id_, -1);
        pnh_.param("trace_swarm_msg_bridge_seq", trace_swarm_msg_bridge_seq_, -1);
        pnh_.param("trace_agent_state_batch_enable", trace_agent_state_batch_enable_, false);
        pnh_.param("trace_agent_state_batch_every_n", trace_agent_state_batch_every_n_, 1);
        if (trace_agent_state_batch_every_n_ <= 0)
        {
            trace_agent_state_batch_every_n_ = 1;
        }

        pnh_.param<std::string>("odom_topic_prefix", odom_topic_prefix_, std::string("/relay_odom_"));
        pnh_.param("id_offset", id_offset_, -1);
        pnh_.param("real_state_stale_warn_sec", real_state_stale_warn_sec_, 1.0);
        int agent_state_max_age_us_param = static_cast<int>(std::max<uint64_t>(step_size_us_ * 25, 250000ull));
        pnh_.param("agent_state_max_age_us",
                   agent_state_max_age_us_param,
                   agent_state_max_age_us_param);
        agent_state_max_age_us_ = static_cast<uint64_t>(std::max(0, agent_state_max_age_us_param));
        int agent_state_future_tolerance_us_param =
            static_cast<int>(std::max<uint64_t>(step_size_us_, 1000ull));
        pnh_.param("agent_state_future_tolerance_us",
                   agent_state_future_tolerance_us_param,
                   agent_state_future_tolerance_us_param);
        agent_state_future_tolerance_us_ =
            static_cast<uint64_t>(std::max(0, agent_state_future_tolerance_us_param));
        std::string racer_ids_csv;
        pnh_.param<std::string>("racer_ids", racer_ids_csv, std::string());

        InitFallbackStates_();
        if (!racer_ids_csv.empty())
        {
            racer_ids_ = ParseCsvInts(racer_ids_csv);
        }
        if (racer_ids_.empty())
        {
            const unsigned int robots_number = getYamlValue<unsigned int>(config_, "robots_number");
            for (unsigned int i = 0; i < robots_number; ++i)
            {
                racer_ids_.push_back(static_cast<int>(i + 1));
            }
        }
        InitOdomSubscriptions_();

        WarmUpProtobufMessages_();

        ros::AsyncSpinner spinner(1);
        spinner.start();

        RunStrictLockstep_();

        spinner.stop();
        ROS_INFO("Coordinator node finished successfully, exiting.");
    }

private:
    struct CachedAgentState
    {
        double x{0.0};
        double y{0.0};
        double z{0.0};
        double vx{0.0};
        double vy{0.0};
        double vz{0.0};
        double heading{0.0};
        uint64_t source_stamp_us{0};
        bool has_real_state{false};
    };

    struct AgentStateFreshness
    {
        bool has_real_state_source{false};
        bool is_fresh{false};
        uint64_t age_us{0};
    };

    AgentStateFreshness EvaluateAgentStateFreshness_(
        const CachedAgentState& state,
        const uint64_t t_sample_us) const
    {
        AgentStateFreshness freshness;
        freshness.has_real_state_source = state.has_real_state && state.source_stamp_us > 0;
        if (!freshness.has_real_state_source)
        {
            return freshness;
        }

        if (state.source_stamp_us > t_sample_us)
        {
            const uint64_t lead_us = state.source_stamp_us - t_sample_us;
            freshness.age_us = lead_us;
            freshness.is_fresh = lead_us <= agent_state_future_tolerance_us_;
            return freshness;
        }

        freshness.age_us = t_sample_us - state.source_stamp_us;
        freshness.is_fresh = freshness.age_us <= agent_state_max_age_us_;
        return freshness;
    }

    bool ShouldTraceSwarmWrapper_(const relay_racer_proto::RacerSwarmMsg& wrapper) const
    {
        if (!trace_swarm_msg_enable_)
        {
            return false;
        }
        if (!trace_swarm_msg_family_.empty() && wrapper.family() != trace_swarm_msg_family_)
        {
            return false;
        }
        if (trace_swarm_msg_src_id_ > 0 &&
            static_cast<int>(wrapper.src_id()) != trace_swarm_msg_src_id_)
        {
            return false;
        }
        if (trace_swarm_msg_bridge_seq_ >= 0 &&
            wrapper.bridge_seq() != static_cast<uint64_t>(trace_swarm_msg_bridge_seq_))
        {
            return false;
        }
        return true;
    }

    void TraceSwarmWrapper_(
        const char* stage,
        const relay_racer_proto::RacerSwarmMsg& wrapper,
        const std::string& extra = std::string()) const
    {
        if (extra.empty())
        {
            ROS_INFO_STREAM("[SWARM_TRACE][" << stage << "] family=" << wrapper.family()
                            << " src_id=" << wrapper.src_id()
                            << " dst_id=" << wrapper.dst_id()
                            << " bridge_seq=" << wrapper.bridge_seq()
                            << " network_tx_id=" << wrapper.network_tx_id()
                            << " relay_hop_count=" << wrapper.relay_hop_count()
                            << " max_relay_hops=" << wrapper.max_relay_hops()
                            << " payload_bytes=" << wrapper.ros_payload().size());
            return;
        }

        ROS_INFO_STREAM("[SWARM_TRACE][" << stage << "] family=" << wrapper.family()
                        << " src_id=" << wrapper.src_id()
                        << " dst_id=" << wrapper.dst_id()
                        << " bridge_seq=" << wrapper.bridge_seq()
                        << " network_tx_id=" << wrapper.network_tx_id()
                        << " relay_hop_count=" << wrapper.relay_hop_count()
                        << " max_relay_hops=" << wrapper.max_relay_hops()
                        << " payload_bytes=" << wrapper.ros_payload().size()
                        << " " << extra);
    }

    void TraceDeliveredSwarmBatch_(const char* stage, const std::string& compressed_batch) const
    {
        if (!trace_swarm_msg_enable_ || compressed_batch.empty())
        {
            return;
        }

        std::string raw_batch;
        try
        {
            raw_batch = gzip_decompress(compressed_batch);
        }
        catch (const std::exception& e)
        {
            ROS_WARN("[SWARM_TRACE][%s] failed to decompress delivered batch: %s", stage, e.what());
            return;
        }

        protobuf_msgs::DeliveredSwarmPacketBatch batch;
        if (!batch.ParseFromString(raw_batch))
        {
            ROS_WARN("[SWARM_TRACE][%s] failed to parse delivered batch raw_bytes=%zu",
                     stage,
                     raw_batch.size());
            return;
        }

        for (int i = 0; i < batch.packets_size(); ++i)
        {
            const auto& packet = batch.packets(i);
            relay_racer_proto::RacerSwarmMsg wrapper;
            if (!wrapper.ParseFromString(packet.wrapped_msg()))
            {
                continue;
            }
            if (!ShouldTraceSwarmWrapper_(wrapper))
            {
                continue;
            }

            TraceSwarmWrapper_(
                stage,
                wrapper,
                "batch_packets=" + std::to_string(batch.packets_size()) +
                    " window_start_us=" + std::to_string(batch.t_window_start_us()) +
                    " window_end_us=" + std::to_string(batch.t_window_end_us()) +
                    " meta_src_id=" + std::to_string(packet.meta().src_id()) +
                    " meta_dst_id=" + std::to_string(packet.meta().dst_id()) +
                    " flow_id=" + std::to_string(packet.meta().flow_id()) +
                    " seq=" + std::to_string(packet.meta().seq()) +
                    " rx_time_us=" + std::to_string(packet.meta().rx_time_us()) +
                    " delay_us=" + std::to_string(packet.meta().delay_us()));
        }
    }

    void WarmUpProtobufMessages_() const
    {
        dancers_update_proto::DancersUpdate update;
        update.set_msg_type(dancers_update_proto::DancersUpdate::BEGIN);
        const std::string update_raw = update.SerializeAsString();
        dancers_update_proto::DancersUpdate update_roundtrip;
        if (!update_roundtrip.ParseFromString(update_raw))
        {
            throw std::runtime_error("Coordinator failed to warm up DancersUpdate protobuf.");
        }

        protobuf_msgs::NetRxEventsPayload net_events;
        net_events.set_t_window_start_us(0);
        net_events.set_t_window_end_us(0);
        const std::string net_events_raw = net_events.SerializeAsString();
        protobuf_msgs::NetRxEventsPayload net_events_roundtrip;
        if (!net_events_roundtrip.ParseFromString(net_events_raw))
        {
            throw std::runtime_error("Coordinator failed to warm up NetRxEventsPayload protobuf.");
        }

        protobuf_msgs::DeliveredSwarmPacketBatch delivered_batch;
        delivered_batch.set_t_window_start_us(0);
        delivered_batch.set_t_window_end_us(0);
        const std::string delivered_batch_raw = delivered_batch.SerializeAsString();
        protobuf_msgs::DeliveredSwarmPacketBatch delivered_batch_roundtrip;
        if (!delivered_batch_roundtrip.ParseFromString(delivered_batch_raw))
        {
            throw std::runtime_error("Coordinator failed to warm up DeliveredSwarmPacketBatch protobuf.");
        }

        protobuf_msgs::AgentStateBatch agent_batch;
        agent_batch.set_sample_seq(0);
        agent_batch.set_t_sample_us(0);
        agent_batch.set_window_idx(0);
        const std::string agent_batch_raw = agent_batch.SerializeAsString();
        protobuf_msgs::AgentStateBatch agent_batch_roundtrip;
        if (!agent_batch_roundtrip.ParseFromString(agent_batch_raw))
        {
            throw std::runtime_error("Coordinator failed to warm up AgentStateBatch protobuf.");
        }
    }

    static ros::Time secondsToRosTime(int64_t seconds)
    {
        ros::Time t;
        t.fromNSec(static_cast<uint64_t>(seconds) * 1000000000ull);
        return t;
    }

    static uint64_t ToUs(const ros::Time& stamp)
    {
        return static_cast<uint64_t>(stamp.toNSec() / 1000ull);
    }

    void publish_clock_()
    {
        rosgraph_msgs::Clock clock_msg;
        clock_msg.clock = current_sim_time_;
        clock_publisher_.publish(clock_msg);
    }

    void InitFallbackStates_()
    {
        const unsigned int robots_number = getYamlValue<unsigned int>(config_, "robots_number");
        const YAML::Node initial_positions = config_["initial_positions"];

        std::lock_guard<std::mutex> lock(agent_states_mutex_);
        if (initial_positions && initial_positions.IsSequence() &&
            initial_positions.size() >= robots_number)
        {
            for (unsigned int i = 0; i < robots_number; ++i)
            {
                const YAML::Node pose = initial_positions[i];
                CachedAgentState state;
                state.x = getYamlValue<double>(pose, "x");
                state.y = getYamlValue<double>(pose, "y");
                state.z = getYamlValue<double>(pose, "z");
                state.heading = pose["heading"] ? pose["heading"].as<double>() : 0.0;
                cached_agent_states_[i] = state;
            }
            return;
        }

        const int n_columns = std::max(1, static_cast<int>(std::sqrt(robots_number)));
        const double spacing = getYamlValue<double>(config_, "initial_spacing");
        const double initial_x = getYamlValue<double>(config_, "initial_x");
        const double initial_y = getYamlValue<double>(config_, "initial_y");
        const double initial_z = getYamlValue<double>(config_, "initial_z");
        const double initial_heading = getYamlValue<double>(config_, "initial_heading");
        const int n_rows = static_cast<int>(std::ceil(static_cast<double>(robots_number) / n_columns));
        const double x0 = initial_x - spacing * (n_columns - 1) * 0.5;
        const double y0 = initial_y - spacing * (n_rows - 1) * 0.5;

        for (unsigned int i = 0; i < robots_number; ++i)
        {
            const int row = static_cast<int>(i) / n_columns;
            const int col = static_cast<int>(i) % n_columns;
            CachedAgentState state;
            state.x = x0 + spacing * col;
            state.y = y0 + spacing * row;
            state.z = initial_z;
            state.heading = initial_heading;
            cached_agent_states_[i] = state;
        }
    }

    void InitOdomSubscriptions_()
    {
        odom_subscribers_.clear();
        odom_subscribers_.reserve(racer_ids_.size());
        for (const int racer_id : racer_ids_)
        {
            const std::string topic = odom_topic_prefix_ + std::to_string(racer_id);
            odom_subscribers_.push_back(
                nh_.subscribe<nav_msgs::Odometry>(
                    topic,
                    20,
                    [this, racer_id](const nav_msgs::OdometryConstPtr& msg) {
                        OdomCallback_(msg, racer_id);
                    }));
        }
    }

    void OdomCallback_(const nav_msgs::OdometryConstPtr& msg, const int racer_id)
    {
        const int platform_id_signed = racer_id + id_offset_;
        if (platform_id_signed < 0)
        {
            ROS_WARN_THROTTLE(2.0,
                              "[COORD] invalid platform_id from racer_id=%d id_offset=%d",
                              racer_id,
                              id_offset_);
            return;
        }
        const uint32_t platform_id = static_cast<uint32_t>(platform_id_signed);

        CachedAgentState state;
        state.x = msg->pose.pose.position.x;
        state.y = msg->pose.pose.position.y;
        state.z = msg->pose.pose.position.z;
        state.vx = msg->twist.twist.linear.x;
        state.vy = msg->twist.twist.linear.y;
        state.vz = msg->twist.twist.linear.z;
        state.heading = HeadingFromQuaternion(msg->pose.pose.orientation);
        state.source_stamp_us = ToUs(msg->header.stamp);
        state.has_real_state = true;

        std::lock_guard<std::mutex> lock(agent_states_mutex_);
        cached_agent_states_[platform_id] = state;
        last_real_state_update_wall_ = ros::WallTime::now();
    }

    std::string BuildAgentStatesGz_(const uint64_t sample_seq, const uint64_t t_sample_us) const
    {
        protobuf_msgs::AgentStateBatch batch;
        batch.set_sample_seq(sample_seq);
        batch.set_t_sample_us(t_sample_us);
        batch.set_window_idx(sample_seq);

        size_t fresh_state_count = 0;
        size_t stale_state_count = 0;
        size_t fallback_state_count = 0;
        std::vector<std::string> stale_state_details;
        {
            std::lock_guard<std::mutex> lock(agent_states_mutex_);
            for (const auto& [agent_id, state] : cached_agent_states_)
            {
                const AgentStateFreshness freshness =
                    EvaluateAgentStateFreshness_(state, t_sample_us);

                auto* sample = batch.add_states();
                sample->set_agent_id(agent_id);
                sample->set_x(state.x);
                sample->set_y(state.y);
                sample->set_z(state.z);
                sample->set_vx(state.vx);
                sample->set_vy(state.vy);
                sample->set_vz(state.vz);
                sample->set_heading(state.heading);
                sample->set_source_stamp_us(state.source_stamp_us);
                sample->set_has_real_state(freshness.has_real_state_source);
                sample->set_age_us(freshness.age_us);
                sample->set_is_fresh(freshness.is_fresh);

                if (!freshness.has_real_state_source)
                {
                    ++fallback_state_count;
                    continue;
                }

                if (freshness.is_fresh)
                {
                    ++fresh_state_count;
                }
                else
                {
                    ++stale_state_count;
                    if (stale_state_details.size() < 6)
                    {
                        stale_state_details.push_back(
                            std::to_string(agent_id) + ":" + std::to_string(freshness.age_us) + "us");
                    }
                }
            }
        }

        if (fallback_state_count > 0)
        {
            ROS_WARN_THROTTLE(2.0,
                              "[COORD] agent state batch contains %zu fallback states; waiting for real /relay_odom_* samples.",
                              fallback_state_count);
        }
        if (stale_state_count > 0)
        {
            std::ostringstream stale_stream;
            for (std::size_t i = 0; i < stale_state_details.size(); ++i)
            {
                if (i != 0)
                {
                    stale_stream << ",";
                }
                stale_stream << stale_state_details[i];
            }
            ROS_WARN_THROTTLE(1.0,
                              "[COORD] agent state batch contains %zu stale real states older than %luus (sample=%lu details=%s).",
                              stale_state_count,
                              static_cast<unsigned long>(agent_state_max_age_us_),
                              static_cast<unsigned long>(sample_seq),
                              stale_stream.str().c_str());
        }
        if (!last_real_state_update_wall_.isZero())
        {
            const double stale_sec = (ros::WallTime::now() - last_real_state_update_wall_).toSec();
            if (stale_sec > real_state_stale_warn_sec_)
            {
                ROS_WARN_THROTTLE(2.0,
                                  "[COORD] real odom updates globally stale for %.3fs while sampling strict-lockstep state batches.",
                                  stale_sec);
            }
        }

        if (trace_agent_state_batch_enable_ &&
            (sample_seq % static_cast<uint64_t>(trace_agent_state_batch_every_n_)) == 0)
        {
            ROS_INFO("[AGENT_STATE_TRACE][COORD] sample_seq=%lu t_sample_us=%lu states=%d fresh=%zu stale=%zu fallback=%zu",
                     static_cast<unsigned long>(sample_seq),
                     static_cast<unsigned long>(t_sample_us),
                     batch.states_size(),
                     fresh_state_count,
                     stale_state_count,
                     fallback_state_count);
        }

        std::string raw;
        batch.SerializeToString(&raw);
        return gzip_compress(raw);
    }

    size_t ComputePayloadBytes_(const dancers_update_proto::DancersUpdate& msg) const
    {
        return msg.payload().size() +
               msg.net_rx_events_gz().size() +
               msg.delivered_swarm_packets_gz().size() +
               msg.agent_states_gz().size();
    }

    std::string MergeCompressedDeliveredSwarmPacketBatches_(
        const std::deque<std::string>& compressed_batches) const
    {
        if (compressed_batches.empty())
        {
            return "";
        }

        protobuf_msgs::DeliveredSwarmPacketBatch merged;
        uint64_t min_window_start_us = std::numeric_limits<uint64_t>::max();
        uint64_t max_window_end_us = 0;

        for (const auto& compressed_batch : compressed_batches)
        {
            if (compressed_batch.empty())
            {
                continue;
            }

            const std::string raw_batch = gzip_decompress(compressed_batch);
            protobuf_msgs::DeliveredSwarmPacketBatch batch;
            if (!batch.ParseFromString(raw_batch))
            {
                throw std::runtime_error(
                    "Coordinator failed to parse queued DeliveredSwarmPacketBatch.");
            }

            min_window_start_us = std::min<uint64_t>(
                min_window_start_us, static_cast<uint64_t>(batch.t_window_start_us()));
            max_window_end_us = std::max<uint64_t>(
                max_window_end_us, static_cast<uint64_t>(batch.t_window_end_us()));

            for (int i = 0; i < batch.packets_size(); ++i)
            {
                *merged.add_packets() = batch.packets(i);
            }
        }

        if (merged.packets_size() == 0)
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

    CustomSocket* AcceptSocket_(
        boost::asio::io_context& io_context,
        const bool use_uds,
        const std::string& uds_address,
        const std::string& tcp_ip,
        const unsigned short tcp_port) const
    {
        CustomSocket* socket = nullptr;
        if (use_uds)
        {
            socket = new UDSSocket(io_context);
            socket->accept(uds_address, 0);
        }
        else
        {
            socket = new TCPSocket(io_context);
            socket->accept(tcp_ip, tcp_port);
        }
        return socket;
    }

    dancers_update_proto::DancersUpdate RoundTrip_(
        CustomSocket* socket,
        const dancers_update_proto::DancersUpdate& request_msg,
        const char* label,
        dancers::metrics::CsvMetricsLogger* metrics_logger) const
    {
        std::string req_raw = request_msg.SerializeAsString();
        std::string req_comp = gzip_compress(req_raw);

        WallTimeProbe rtt_probe;
        rtt_probe.start();
        socket->send_one_message(req_comp);
        std::string resp_comp = socket->receive_one_message();
        rtt_probe.stop();
        const uint64_t rtt_us = rtt_probe.get_elapsed_time();

        std::string response = gzip_decompress(resp_comp);
        if (response.empty())
        {
            ROS_WARN("[COORD][%s] Empty message from connector", label);
            dancers_update_proto::DancersUpdate empty_msg;
            empty_msg.set_msg_type(dancers_update_proto::DancersUpdate::END);
            return empty_msg;
        }

        dancers_update_proto::DancersUpdate response_msg;
        response_msg.ParseFromString(response);
        if (response_msg.msg_type() != dancers_update_proto::DancersUpdate::END)
        {
            throw std::runtime_error(
                std::string("Coordinator received a non-END message from ") + label + " Connector.");
        }

        if (verbose_ && log_payload_bytes_)
        {
            const double t_sim_sec = static_cast<double>(current_sim_time_.toNSec()) / 1e9;
            ROS_INFO(
                "[COORD][%s] t_sim=%.6f TX(comp=%zuB raw=%zuB payload=%zuB) RX(comp=%zuB raw=%zuB payload=%zuB) RTT=%lu us",
                label,
                t_sim_sec,
                req_comp.size(),
                req_raw.size(),
                ComputePayloadBytes_(request_msg),
                resp_comp.size(),
                response.size(),
                ComputePayloadBytes_(response_msg),
                static_cast<unsigned long>(log_rtt_ ? rtt_us : 0));
        }

        if (metrics_logger)
        {
            const double t_sim_sec = static_cast<double>(current_sim_time_.toNSec()) / 1e9;
            metrics_logger->log_row(
                dancers::metrics::now_us(),
                t_sim_sec,
                req_raw.size(),
                req_comp.size(),
                ComputePayloadBytes_(request_msg),
                response.size(),
                resp_comp.size(),
                ComputePayloadBytes_(response_msg),
                rtt_us);
        }

        return response_msg;
    }

    void SendClose_(CustomSocket* socket) const
    {
        if (socket == nullptr)
        {
            return;
        }
        dancers_update_proto::DancersUpdate close_msg;
        close_msg.set_msg_type(dancers_update_proto::DancersUpdate::CLOSE);
        socket->send_one_message(gzip_compress(close_msg.SerializeAsString()));
    }

    void RunStrictLockstep_()
    {
        ROS_INFO("[COORD] strict lockstep enabled: coordinator owns state sampling and per-step distribution.");

        boost::asio::io_context phy_io_context;
        boost::asio::io_context net_io_context;
        std::unique_ptr<CustomSocket> phy_socket;
        std::unique_ptr<CustomSocket> net_socket;

        try
        {
            phy_socket.reset(AcceptSocket_(
                phy_io_context,
                getYamlValue<bool>(config_, "phy_use_uds"),
                getYamlValue<std::string>(config_, "phy_uds_server_address"),
                config_["phy_ip_server_address"].as<std::string>(),
                config_["phy_ip_server_port"].as<unsigned short>()));
            ROS_INFO("Connected PHY socket.");

            net_socket.reset(AcceptSocket_(
                net_io_context,
                getYamlValue<bool>(config_, "net_use_uds"),
                getYamlValue<std::string>(config_, "net_uds_server_address"),
                config_["net_ip_server_address"].as<std::string>(),
                config_["net_ip_server_port"].as<unsigned short>()));
            ROS_INFO("Connected NET socket.");

            const ros::Time simulation_length =
                secondsToRosTime(getYamlValue<int64_t>(config_, "simulation_length"));

            double rtf = std::numeric_limits<double>::infinity();
            try
            {
                rtf = getYamlValue<double>(config_, "real_time_factor");
            }
            catch (const std::runtime_error&)
            {
                ROS_WARN("real_time_factor not found in config, running at max speed.");
            }

            publish_clock_();

            uint64_t step_idx = 0;
            auto next_deadline = std::chrono::steady_clock::now();
            const bool pace_real_time = std::isfinite(rtf) && rtf > 0.0;
            const auto step_wall_duration = std::chrono::duration<double>(
                static_cast<double>(step_size_us_) / 1e6 / (pace_real_time ? rtf : 1.0));

            while (current_sim_time_ < simulation_length || simulation_length == ros::Time(0))
            {
                const uint64_t t_sample_us = ToUs(current_sim_time_);
                const std::string agent_states_gz = BuildAgentStatesGz_(step_idx, t_sample_us);

                dancers_update_proto::DancersUpdate net_request;
                net_request.set_msg_type(dancers_update_proto::DancersUpdate::BEGIN);
                if (!agent_states_gz.empty())
                {
                    net_request.set_agent_states_gz(agent_states_gz);
                }

                dancers_update_proto::DancersUpdate net_response =
                    RoundTrip_(net_socket.get(), net_request, "NET", metrics_net_.get());

                dancers_update_proto::DancersUpdate phy_request;
                phy_request.set_msg_type(dancers_update_proto::DancersUpdate::BEGIN);
                if (!agent_states_gz.empty())
                {
                    phy_request.set_agent_states_gz(agent_states_gz);
                }
                if (!net_response.net_rx_events_gz().empty())
                {
                    phy_request.set_net_rx_events_gz(net_response.net_rx_events_gz());
                }
                if (!net_response.delivered_swarm_packets_gz().empty())
                {
                    std::deque<std::string> single_batch;
                    single_batch.push_back(net_response.delivered_swarm_packets_gz());
                    const std::string merged_batch =
                        MergeCompressedDeliveredSwarmPacketBatches_(single_batch);
                    phy_request.set_delivered_swarm_packets_gz(merged_batch);
                    TraceDeliveredSwarmBatch_("coord_phy_send_to_phy", merged_batch);
                }

                dancers_update_proto::DancersUpdate phy_response =
                    RoundTrip_(phy_socket.get(), phy_request, "PHY", metrics_phy_.get());
                (void)phy_response;

                current_sim_time_ += ros::Duration(static_cast<double>(step_size_us_) / 1e6);
                publish_clock_();
                ++step_idx;

                if (pace_real_time)
                {
                    next_deadline += std::chrono::duration_cast<std::chrono::steady_clock::duration>(step_wall_duration);
                    std::this_thread::sleep_until(next_deadline);
                }
            }

            SendClose_(phy_socket.get());
            SendClose_(net_socket.get());
            phy_socket->close();
            net_socket->close();
        }
        catch (const std::exception& e)
        {
            if (ros::isShuttingDown() || IsExpectedSocketShutdown(e.what()))
            {
                ROS_INFO("Coordinator exiting during shutdown: %s", e.what());
            }
            else
            {
                ROS_ERROR("%s", e.what());
                std::exit(EXIT_FAILURE);
            }
        }
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    std::string ros_ws_path_;
    YAML::Node config_;

    std::unique_ptr<dancers::metrics::CsvMetricsLogger> metrics_net_;
    std::unique_ptr<dancers::metrics::CsvMetricsLogger> metrics_phy_;
    bool enable_metrics_{false};
    size_t metrics_flush_every_n_{50};

    bool verbose_{true};
    bool log_payload_bytes_{true};
    bool log_rtt_{true};
    bool strict_lockstep_{true};

    ros::Time current_sim_time_;
    ros::Publisher clock_publisher_;
    uint64_t step_size_us_{0};

    std::string odom_topic_prefix_;
    int id_offset_{-1};
    double real_state_stale_warn_sec_{1.0};
    uint64_t agent_state_max_age_us_{100000};
    uint64_t agent_state_future_tolerance_us_{1000};
    std::vector<int> racer_ids_;
    std::vector<ros::Subscriber> odom_subscribers_;

    mutable std::mutex agent_states_mutex_;
    std::map<uint32_t, CachedAgentState> cached_agent_states_;
    ros::WallTime last_real_state_update_wall_;

    bool trace_swarm_msg_enable_{false};
    std::string trace_swarm_msg_family_;
    int trace_swarm_msg_src_id_{-1};
    int trace_swarm_msg_bridge_seq_{-1};
    bool trace_agent_state_batch_enable_{false};
    int trace_agent_state_batch_every_n_{1};
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "coordinator");
    Coordinator coordinator;
    (void)coordinator;
    return 0;
}
