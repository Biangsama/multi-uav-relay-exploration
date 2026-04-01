#include <connector_core/connector.hpp>

// ns-3 libs
#include <ns3/core-module.h>
#include <ns3/mobility-module.h>
#include <ns3/wifi-module.h>
#include <ns3/internet-module.h>
#include <ns3/propagation-loss-model.h>
#include <ns3/buildings-module.h>
#include <ns3/spectrum-module.h>
#include "ns3/three-gpp-v2v-propagation-loss-model.h"

// Home-made ns-3 apps
#include <applications/flocking-application.h>
#include <applications/swarm_message_application.h>

// Common DANCERS libs
#include <agent.hpp>
#include <yaml_util.hpp>

// C++ libs
#include <atomic>
#include <shared_mutex>
#include <unordered_map>
#include <thread>
#include <mutex>
#include <limits>
#include <cmath>
#include <vector>
#include <deque>
#include <algorithm>
#include <optional>
#include <string>
#include <exception>

// Custom ROS1 messages
#include <dancers_msgs/AgentStruct.h>
#include <dancers_msgs/AgentStructArray.h>
#include <dancers_msgs/GetAgentVelocities.h>
#include <std_msgs/UInt8MultiArray.h>

// Protobuf messages
#include <protobuf_msgs/pose_vector.pb.h>
#include <protobuf_msgs/state_payload.pb.h>   // <-- NEW: CmdPayload / NodeCommMetrics
#include <protobuf_msgs/net_rx_events.pb.h>  // <-- NEW: NetRxEventsPayload / RxEvent
#include <protobuf_msgs/agent_state_batch.pb.h>
#include <protobuf_msgs/racer_swarm_msg.pb.h>


// Rviz ROS1 messages
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

// CSV metrics (same style as coordinator)
#include <connector_core/metrics_logger.hpp>

#if __has_include(<rcpputils/filesystem_helper.hpp>)
  #include <rcpputils/filesystem_helper.hpp>
  namespace dancers_fs = rcpputils::fs;
#else
  #include <filesystem>
  namespace dancers_fs = std::filesystem;
#endif

#define RCLCPP_INFO(logger, ...) ROS_INFO(__VA_ARGS__)
#define RCLCPP_WARN(logger, ...) ROS_WARN(__VA_ARGS__)
#define RCLCPP_ERROR(logger, ...) ROS_ERROR(__VA_ARGS__)
#define RCLCPP_FATAL(logger, ...) ROS_FATAL(__VA_ARGS__)
#define RCLCPP_DEBUG(logger, ...) ROS_DEBUG(__VA_ARGS__)
#define RCLCPP_INFO_STREAM(logger, msg) ROS_INFO_STREAM(msg)
#define RCLCPP_WARN_STREAM(logger, msg) ROS_WARN_STREAM(msg)


using namespace ns3;
namespace {

// pack (rx_id, tx_id) into 64-bit key
static inline uint64_t PairKey(uint32_t rx_id, uint32_t tx_id)
{
    return (uint64_t(rx_id) << 32) | uint64_t(tx_id);
}

static inline uint64_t PairFlowKey(uint32_t rx_id, uint32_t tx_id, uint32_t flow_id)
{
    return (uint64_t(rx_id) << 48) ^ (uint64_t(tx_id) << 32) ^ uint64_t(flow_id);
}

static inline uint64_t AbsDiffU64(uint64_t a, uint64_t b)
{
    return (a > b) ? (a - b) : (b - a);
}

static inline double FriisReferenceLossDb(double frequency_hz, double reference_distance_m)
{
    constexpr double kSpeedOfLight = 299792458.0;
    constexpr double kPi = 3.14159265358979323846;
    const double wavelength = kSpeedOfLight / frequency_hz;
    const double term = (4.0 * kPi * reference_distance_m) / wavelength;
    return 20.0 * std::log10(term);
}

} // namespace

class BasicWifiAdhoc : public Connector
{
public:
    BasicWifiAdhoc()
    : Connector("basic_wifi_adhoc")
    {
        RCLCPP_INFO(this->get_logger(), "BasicWifiAdhoc (NET) started");

        // Read relevant information from the configuration file
        this->ReadConfigFile();

        // ---- Metrics switches (same YAML keys as coordinator) ----
        verbose_ = (config_["verbose"] ? config_["verbose"].as<bool>() : true);
        enable_metrics_ = (config_["enable_metrics"] ? config_["enable_metrics"].as<bool>() : false);
        metrics_flush_every_n_ = (config_["flush_every_n"] ? config_["flush_every_n"].as<size_t>() : 50);

        pnh_.param("enable_swarm_msg_transport", enable_swarm_msg_transport_, true);
        pnh_.param("swarm_msg_id_offset", swarm_msg_id_offset_, -1);
        int swarm_msg_port_param = static_cast<int>(swarm_msg_port_);
        int swarm_msg_flow_id_param = static_cast<int>(swarm_msg_flow_id_);
        pnh_.param("swarm_msg_port", swarm_msg_port_param, 4100);
        pnh_.param("swarm_msg_flow_id", swarm_msg_flow_id_param, 30);
        swarm_msg_port_ = static_cast<uint32_t>(std::max(0, swarm_msg_port_param));
        swarm_msg_flow_id_ = static_cast<uint32_t>(std::max(0, swarm_msg_flow_id_param));
        pnh_.param("swarm_msg_queue_limit", swarm_msg_queue_limit_, 0);
        pnh_.param("swarm_msg_queue_warn_threshold", swarm_msg_queue_warn_threshold_, 200);
        pnh_.param<std::string>(
            "swarm_msg_tx_topic",
            swarm_msg_tx_topic_,
            std::string("/relay_integration/racer_swarm_tx_bytes"));
        pnh_.param<std::string>(
            "swarm_msg_rx_topic",
            swarm_msg_rx_topic_,
            std::string("/relay_integration/racer_swarm_rx_bytes"));
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
        pnh_.param("ignore_stale_agent_states", ignore_stale_agent_states_, true);
        pnh_.param("warn_on_stale_agent_states", warn_on_stale_agent_states_, true);
        if (enable_swarm_msg_transport_)
        {
            swarm_tx_sub_ = nh_.subscribe(
                swarm_msg_tx_topic_,
                200,
                &BasicWifiAdhoc::swarmTxBytesCallback,
                this);
            RCLCPP_INFO(
                this->get_logger(),
                "[NET] enable_swarm_msg_transport=true tx_topic=%s delayed_rx_topic=%s port=%u flow_id=%u id_offset=%d queue_limit=%d warn_threshold=%d",
                swarm_msg_tx_topic_.c_str(),
                swarm_msg_rx_topic_.c_str(),
                swarm_msg_port_,
                swarm_msg_flow_id_,
                swarm_msg_id_offset_,
                swarm_msg_queue_limit_,
                swarm_msg_queue_warn_threshold_);
        }

        // Configure global ns-3 objects
        this->ConfigureNs3();

        // Initialize the agents and the ad-hoc Wi-Fi network
        this->InitAgents();

        // Init per-node stats storage after agents_ exists
        InitPerNodeStatsStorage_();

        // Init metrics CSV after agents_ exists
        this->InitMetricsCsv_();

        pnh_.param("prefer_real_agent_states", prefer_real_agent_states_, false);
        pnh_.param("real_agent_states_timeout_sec", real_agent_states_timeout_sec_, 1.0);
        pnh_.param<std::string>(
            "real_agent_states_topic",
            real_agent_states_topic_,
            std::string("/relay_integration/platform_agent_structs"));

        if (prefer_real_agent_states_)
        {
            real_agent_states_sub_ = nh_.subscribe(
                real_agent_states_topic_,
                10,
                &BasicWifiAdhoc::realAgentStatesCallback,
                this);
            RCLCPP_INFO(
                this->get_logger(),
                "[NET] prefer_real_agent_states=true topic=%s timeout=%.3fs",
                real_agent_states_topic_.c_str(),
                real_agent_states_timeout_sec_);
        }

        // Initiate publishers
        if (getYamlValue<bool>(this->config_, "display_network_edges"))
        {
            this->neighborhood_links_pub_ = nh_.advertise<visualization_msgs::Marker>("neighborhood_links", 10);
        }

        /* ----------- Service client ----------- */
        command_client_ = nh_.serviceClient<dancers_msgs::GetAgentVelocities>("get_agents_velocities");

        // Create timer for service calls
        this->req_cmds_timer_ = nh_.createTimer(
            ros::Duration(getYamlValue<uint32_t>(this->config_, "request_commands_period") / 1000000.0),
            [this](const ros::TimerEvent&) { this->RequestCommandsClbk(); });
    }

    void RunLoop()
    {
        Loop();
    }

private:
    /**
     * @brief Read config file and set variables
     */
    void ReadConfigFile() override
    {
        this->use_uds = getYamlValue<bool>(this->config_, "net_use_uds");
        this->uds_server_address = getYamlValue<std::string>(this->config_, "net_uds_server_address");
        this->ip_server_address = getYamlValue<std::string>(this->config_, "net_ip_server_address");
        this->ip_server_port = getYamlValue<unsigned int>(this->config_, "net_ip_server_port");

        // Verify sync window compatible with net step
        unsigned int sync_window_int = getYamlValue<unsigned int>(this->config_, "sync_window");
        unsigned int step_size_int = getYamlValue<unsigned int>(this->config_, "net_step_size");
        if (sync_window_int % step_size_int != 0)
        {
            RCLCPP_FATAL(this->get_logger(), "Sync window must be a multiple of the network step size, aborting.");
            exit(EXIT_FAILURE);
        }

        net_step_size_us_ = step_size_int;
        this->step_size = step_size_int / 1000000.0f;  // seconds
        this->it_end_sim = uint64_t(this->simulation_length / this->step_size);
    }

    /**
     * @brief Called at each step
     */
    dancers_update_proto::DancersUpdate StepSimulation(dancers_update_proto::DancersUpdate update_msg) override
    {
        // Reset per-window accumulators (one StepSimulation call == one window)
        ResetWindowAccumulators_();

        uint64_t t0_us = Simulator::Now().ToInteger(Time::US);

        this->UpdateAgentsPosition(update_msg);
        // Keep all ns-3 mobility mutations on the simulation thread.
        ApplyCachedAgentStatesToMobility_();
        DrainPendingSwarmTx_();

        // Advance step_size seconds in ns-3
        Simulator::Stop(Seconds(step_size));
        Simulator::Run();

        this->timeoutNeighbors();
        this->DisplayRviz();

        uint64_t t1_us = Simulator::Now().ToInteger(Time::US);
        uint64_t win_dur_us = (t1_us > t0_us) ? (t1_us - t0_us) : net_step_size_us_;
        if (win_dur_us == 0) win_dur_us = 1;

        // Finalize derived metrics and write CSV row(s) (if enabled)
        FinalizeAndLogWindow_(t1_us, win_dur_us);

        RCLCPP_DEBUG(this->get_logger(), "Finished iteration %ld", this->it);

        // NEW: wrap cmds + per-node metrics into CmdPayload
        return this->GenerateResponseProtobuf(t1_us, win_dur_us);
    }

    /**
     * @brief Generate the response protobuf message to send to the coordinator
     * NEW: send CmdPayload(payload=PoseVector bytes + node_metrics)
     */
    dancers_update_proto::DancersUpdate GenerateResponseProtobuf(uint64_t t_sim_us, uint64_t win_dur_us)
    {
        dancers_update_proto::DancersUpdate response_msg;
        response_msg.set_msg_type(dancers_update_proto::DancersUpdate::END);

        // 1) Build PoseVector command (legacy content)
        dancers_update_proto::PoseVector velocity_cmds;
        std::vector<uint32_t> agent_ids;
        {
            std::shared_lock lock(this->agents_mutex_);
            agent_ids.reserve(this->agents_.size());
            for (const auto& [agent_id, agent_ptr] : this->agents_)
            {
                dancers_update_proto::Pose* cmd = velocity_cmds.add_pose();
                cmd->set_vx(agent_ptr->cmd_velocity.x());
                cmd->set_vy(agent_ptr->cmd_velocity.y());
                cmd->set_vz(agent_ptr->cmd_velocity.z());
                cmd->set_agent_id(agent_id);
                agent_ids.push_back(agent_id);
            }
        }
        std::sort(agent_ids.begin(), agent_ids.end());

        std::string cmd_pv_bytes;
        velocity_cmds.SerializeToString(&cmd_pv_bytes);

        // 2) Wrap into CmdPayload with node comm metrics
        dancers_update_proto::CmdPayload cp;
        cp.set_seq((uint64_t)this->it);
        cp.set_t_sample_us(t_sim_us);     // 你也可以改成窗口开始时间
        cp.set_t_sim_us(t_sim_us);
        cp.set_window_idx((uint64_t)this->it);
        cp.set_cmd_pose_vector(cmd_pv_bytes);

        // 3) Attach per-node metrics
        for (uint32_t node_id : agent_ids)
        {
            EnsurePerNodeSize_(node_id);

            uint64_t rx_pkts = node_rx_pkts_[node_id];
            uint64_t lost_pkts = node_lost_pkts_[node_id];
            uint64_t expected_pkts = rx_pkts + lost_pkts;
            double pdr = (expected_pkts > 0) ? (double(rx_pkts) / double(expected_pkts)) : 1.0;

            uint64_t delay_mean_us = (rx_pkts > 0) ? (node_delay_sum_us_[node_id] / rx_pkts) : 0;
            uint64_t delay_max_us  = node_delay_max_us_[node_id];

            uint64_t js = node_jitter_samples_[node_id];
            uint64_t jitter_mean_us = (js > 0) ? (node_jitter_sum_us_[node_id] / js) : 0;

            uint64_t rx_bytes = node_rx_bytes_[node_id];
            double throughput_bps = (double(rx_bytes) * 8.0) * (1e6 / double(std::max<uint64_t>(1, win_dur_us)));

            uint64_t tx_pkts = node_tx_pkts_[node_id];
            uint64_t dup_pkts = node_dup_pkts_[node_id];

            double rxp_mean = (node_rx_power_samples_[node_id] > 0)
                ? (node_rx_power_sum_dbm_[node_id] / double(node_rx_power_samples_[node_id]))
                : 0.0;
            double rxp_min = std::isfinite(node_rx_power_min_dbm_[node_id]) ? node_rx_power_min_dbm_[node_id] : 0.0;
            double rxp_max = std::isfinite(node_rx_power_max_dbm_[node_id]) ? node_rx_power_max_dbm_[node_id] : 0.0;

            auto* nm = cp.add_node_metrics();
            nm->set_node_id(node_id);
            nm->set_expected_pkts(expected_pkts);
            nm->set_rx_pkts(rx_pkts);
            nm->set_lost_pkts(lost_pkts);
            nm->set_pdr(pdr);
            nm->set_delay_mean_us(delay_mean_us);
            nm->set_delay_max_us(delay_max_us);
            nm->set_jitter_mean_us(jitter_mean_us);
            nm->set_rx_bytes(rx_bytes);
            nm->set_throughput_bps(throughput_bps);
            nm->set_tx_pkts(tx_pkts);
            nm->set_dup_pkts(dup_pkts);
            nm->set_rx_power_dbm_mean(rxp_mean);
            nm->set_rx_power_dbm_min(rxp_min);
            nm->set_rx_power_dbm_max(rxp_max);
        }

        std::string cp_bytes;
        cp.SerializeToString(&cp_bytes);
        response_msg.set_payload(gzip_compress(cp_bytes));

        const uint64_t win_start_us = (t_sim_us > win_dur_us) ? (t_sim_us - win_dur_us) : 0;

        // Preserve the legacy packet-event batch for metrics/diagnostics consumers.
        // Always publish the current window snapshot, even when it contains zero events.
        protobuf_msgs::NetRxEventsPayload evp;
        evp.set_t_window_start_us(win_start_us);
        evp.set_t_window_end_us(t_sim_us);

        for (const auto& e : rx_events_window_)
        {
            auto* ev = evp.add_events();

            uint32_t src_id = (e.hdr_sender_id != 0) ? e.hdr_sender_id : e.tx_id;
            ev->set_src_id(src_id);
            ev->set_dst_id(e.rx_id);
            ev->set_flow_id(e.flow_id);
            ev->set_seq(e.seq);
            ev->set_tx_time_us(e.tx_time_us);
            ev->set_rx_time_us(e.rx_time_us);
            ev->set_delay_us(e.delay_us);
            ev->set_payload_bytes(e.pkt_bytes);

            int32_t rssi_x10 = 0;
            if (e.has_rx_power && std::isfinite(e.rx_power_dbm))
            {
                rssi_x10 = (int32_t)std::llround(e.rx_power_dbm * 10.0);
            }
            ev->set_rssi_dbm_x10(rssi_x10);
            ev->set_is_duplicate(e.is_duplicate);
            ev->set_has_packet_state(e.has_packet_state);
            ev->set_pos_x(e.pos_x);
            ev->set_pos_y(e.pos_y);
            ev->set_pos_z(e.pos_z);
            ev->set_vel_x(e.vel_x);
            ev->set_vel_y(e.vel_y);
            ev->set_vel_z(e.vel_z);
        }

        std::string ev_raw;
        evp.SerializeToString(&ev_raw);
        {
            std::lock_guard<std::mutex> lk(latest_comm_rx_events_mu_);
            latest_comm_rx_events_raw_ = ev_raw;
        }
        response_msg.set_net_rx_events_gz(gzip_compress(ev_raw));

        if (verbose_)
        {
            RCLCPP_INFO(this->get_logger(),
                        "[NET] net_rx_events_gz sent win=%lu events=%zu raw=%zuB gz=%zuB",
                        (unsigned long)window_idx_,
                        rx_events_window_.size(),
                        ev_raw.size(),
                        response_msg.net_rx_events_gz().size());
        }

        if (!delivered_swarm_packets_window_.empty())
        {
            protobuf_msgs::DeliveredSwarmPacketBatch batch;
            batch.set_t_window_start_us(win_start_us);
            batch.set_t_window_end_us(t_sim_us);

            for (const auto& item : delivered_swarm_packets_window_)
            {
                auto* packet = batch.add_packets();
                auto* meta = packet->mutable_meta();
                const auto& e = item.meta;
                const uint32_t src_id = (e.hdr_sender_id != 0) ? e.hdr_sender_id : e.tx_id;
                meta->set_src_id(src_id);
                meta->set_dst_id(e.rx_id);
                meta->set_flow_id(e.flow_id);
                meta->set_seq(e.seq);
                meta->set_tx_time_us(e.tx_time_us);
                meta->set_rx_time_us(e.rx_time_us);
                meta->set_delay_us(e.delay_us);
                meta->set_payload_bytes(e.pkt_bytes);
                meta->set_rssi_dbm_x10(item.rssi_dbm_x10);
                meta->set_is_duplicate(e.is_duplicate);
                meta->set_has_packet_state(e.has_packet_state);
                meta->set_pos_x(e.pos_x);
                meta->set_pos_y(e.pos_y);
                meta->set_pos_z(e.pos_z);
                meta->set_vel_x(e.vel_x);
                meta->set_vel_y(e.vel_y);
                meta->set_vel_z(e.vel_z);
                packet->set_wrapped_msg(item.wrapped_msg);

                if (trace_swarm_msg_enable_)
                {
                    relay_racer_proto::RacerSwarmMsg wrapper;
                    if (wrapper.ParseFromString(item.wrapped_msg) && ShouldTraceSwarmWrapper_(wrapper))
                    {
                        TraceSwarmWrapper_(
                            "net_send_to_coord",
                            wrapper,
                            "window_idx=" + std::to_string(window_idx_) +
                                " meta_src_id=" + std::to_string(meta->src_id()) +
                                " meta_dst_id=" + std::to_string(meta->dst_id()) +
                                " flow_id=" + std::to_string(meta->flow_id()) +
                                " seq=" + std::to_string(meta->seq()) +
                                " rx_time_us=" + std::to_string(meta->rx_time_us()) +
                                " delay_us=" + std::to_string(meta->delay_us()));
                    }
                }
            }

            std::string batch_raw;
            batch.SerializeToString(&batch_raw);
            response_msg.set_delivered_swarm_packets_gz(gzip_compress(batch_raw));

            if (verbose_)
            {
                RCLCPP_INFO(this->get_logger(),
                            "[NET] delivered_swarm_packets_gz sent win=%lu packets=%zu raw=%zuB gz=%zuB",
                            (unsigned long)window_idx_,
                            delivered_swarm_packets_window_.size(),
                            batch_raw.size(),
                            response_msg.delivered_swarm_packets_gz().size());
            }
        }

        return response_msg;
    }



    // ---- ns-3 setup / agents ----
    void ConfigureNs3();
    void InitAgents();
    void AddWifiNetworkStack(Ptr<ns3::Node> node, uint32_t agent_id);
    void AddFlockingApplication(Ptr<ns3::Node> node);
    void AddSwarmMessageApplication(Ptr<ns3::Node> node, uint32_t agent_id);
    void UpdateAgentsPosition(const dancers_update_proto::DancersUpdate& update_msg);
    void realAgentStatesCallback(const dancers_msgs::AgentStructArrayConstPtr& msg);
    void ApplyCachedAgentStatesToMobility_();
    void DisplayRviz();

    // ---- ROS2 service (controller) ----
    ros::Timer req_cmds_timer_;
    void RequestCommandsClbk();
    void DrainPendingSwarmTx_();
    bool ShouldTraceSwarmWrapper_(const relay_racer_proto::RacerSwarmMsg& wrapper) const;
    void TraceSwarmWrapper_(const char* stage,
                            const relay_racer_proto::RacerSwarmMsg& wrapper,
                            const std::string& extra = std::string()) const;

    // ---- neighbor maintenance ----
    void timeoutNeighbors();

    // Practical
    std::optional<uint32_t> FindAgentIdByNode(const Ptr<ns3::Node>& node) const;
    std::optional<uint32_t> GetNodeIndexFromAgentId_(uint32_t agent_id) const;

    // ROS2 callbacks
    void SpawnUavClbk(const dancers_msgs::AgentStruct& msg);

    // ROS2 Service client
    ros::ServiceClient command_client_;
    std::atomic<bool> command_request_in_flight_{false};
    ros::Subscriber real_agent_states_sub_;
    ros::Subscriber swarm_tx_sub_;

    // ns-3 Trace callbacks
    void flocking_broadcaster_clbk(std::string context, Ptr<const Packet> packet);
    void flocking_receiver_clbk(std::string context, Ptr<const Packet> packet, int peer_id);
    void swarmTxBytesCallback(const std_msgs::UInt8MultiArrayConstPtr& msg);
    void swarmMsgBroadcasterClbk(std::string context, Ptr<const Packet> packet);
    void swarmMsgReceiverClbk(std::string context, Ptr<const Packet> packet, int peer_id);

    // ROS2 Publisher
    ros::Publisher neighborhood_links_pub_;
    ros::Publisher swarm_rx_pub_;

    // Agents / nodes
    std::map<uint32_t, std::unique_ptr<Agent>> agents_;
    mutable std::shared_mutex agents_mutex_;

    std::map<uint32_t, Vector> current_waypoints_;
    NodeContainer nodes;
    std::map<uint32_t, Ptr<SwarmMessageBroadcaster>> swarm_msg_broadcasters_;

    // ns-3 helpers
    std::unique_ptr<WifiHelper> wifiHelper_;
    std::unique_ptr<WifiMacHelper> wifiMacHelper_;
    std::unique_ptr<WifiPhyHelper> wifiPhyHelper_;
    std::unique_ptr<InternetStackHelper> internetHelper_;
    std::unique_ptr<Ipv4AddressHelper> ipv4AddressHelper_;
    Ptr<UniformRandomVariable> random_gen_start_flocking_app;

    // RSSI-like
    Ptr<PropagationLossModel> propagationLossModel_{nullptr};
    double tx_power_dbm_{16.0};
    std::string propagation_model_name_{"three_gpp_v2v_urban"};
    bool use_building_obstacles_{true};

    // ---- Metrics CSV ----
    bool verbose_{true};
    bool enable_metrics_{false};
    size_t metrics_flush_every_n_{50};
    unsigned int net_step_size_us_{0};

    std::unique_ptr<dancers::metrics::CsvMetricsLogger> metrics_net_comm_total_;
    std::unique_ptr<dancers::metrics::CsvMetricsLogger> metrics_net_comm_nodes_;

        // ---- Per-packet RX events CSV (one row per received packet) ----
    std::unique_ptr<dancers::metrics::CsvMetricsLogger> metrics_net_rx_events_;

    struct RxEventRow
    {
        uint64_t rx_time_us{0};
        uint32_t rx_id{0};

        uint64_t tx_time_us{0};
        uint32_t tx_id{0};

        uint32_t hdr_sender_id{0};
        uint32_t flow_id{0};
        uint32_t seq{0};

        uint32_t pkt_bytes{0};
        uint64_t delay_us{0};

        double rx_power_dbm{std::numeric_limits<double>::quiet_NaN()};
        bool has_rx_power{false};

        bool is_duplicate{false};
        bool has_packet_state{false};
        double pos_x{0.0};
        double pos_y{0.0};
        double pos_z{0.0};
        double vel_x{0.0};
        double vel_y{0.0};
        double vel_z{0.0};
    };

    struct DeliveredSwarmPacketRow
    {
        RxEventRow meta;
        std::string wrapped_msg;
        int32_t rssi_dbm_x10{0};
    };

    std::vector<RxEventRow> rx_events_window_;
    std::vector<DeliveredSwarmPacketRow> delivered_swarm_packets_window_;
    mutable std::mutex latest_comm_rx_events_mu_;
    std::string latest_comm_rx_events_raw_;

    struct PendingSwarmTxMsg
    {
        uint32_t platform_src_id{0};
        std::string bytes;
    };

    bool enable_swarm_msg_transport_{true};
    int swarm_msg_id_offset_{-1};
    uint32_t swarm_msg_port_{4100};
    uint32_t swarm_msg_flow_id_{30};
    int swarm_msg_queue_limit_{0};
    int swarm_msg_queue_warn_threshold_{200};
    bool trace_swarm_msg_enable_{false};
    std::string trace_swarm_msg_family_;
    int trace_swarm_msg_src_id_{-1};
    int trace_swarm_msg_bridge_seq_{-1};
    bool trace_agent_state_batch_enable_{false};
    int trace_agent_state_batch_every_n_{1};
    bool ignore_stale_agent_states_{true};
    bool warn_on_stale_agent_states_{true};
    std::string swarm_msg_tx_topic_;
    std::string swarm_msg_rx_topic_;
    std::deque<PendingSwarmTxMsg> pending_swarm_tx_msgs_;
    uint64_t pending_swarm_tx_bytes_{0};
    std::mutex swarm_msg_tx_mutex_;
    uint64_t swarm_tx_ros_msgs_{0};
    uint64_t swarm_tx_ns3_msgs_{0};
    uint64_t swarm_rx_ns3_msgs_{0};
    uint64_t swarm_rx_ros_msgs_{0};

    void InitMetricsCsv_();

    uint64_t window_idx_{0};

    // ---- Total window accumulators ----
    uint64_t tx_pkts_total_{0};

    uint64_t rx_pkts_total_{0};
    uint64_t rx_bytes_total_{0};

    uint64_t lost_pkts_total_{0};
    uint64_t dup_pkts_total_{0};

    uint64_t delay_sum_us_{0};
    uint64_t delay_max_us_{0};

    uint64_t jitter_sum_us_{0};
    uint64_t jitter_samples_{0};

    double rx_power_sum_dbm_{0.0};
    uint64_t rx_power_samples_{0};
    double rx_power_min_dbm_{std::numeric_limits<double>::infinity()};
    double rx_power_max_dbm_{-std::numeric_limits<double>::infinity()};

    std::unordered_map<uint64_t, uint32_t> last_seq_by_pair_;
    std::unordered_map<uint64_t, uint64_t> last_delay_us_by_pair_;

    // ---- Per-node window accumulators ----
    void InitPerNodeStatsStorage_();
    void EnsurePerNodeSize_(uint32_t agent_id);

    std::vector<uint64_t> node_tx_pkts_;
    std::vector<uint64_t> node_rx_pkts_;
    std::vector<uint64_t> node_rx_bytes_;
    std::vector<uint64_t> node_lost_pkts_;
    std::vector<uint64_t> node_dup_pkts_;
    std::vector<uint64_t> node_delay_sum_us_;
    std::vector<uint64_t> node_delay_max_us_;
    std::vector<uint64_t> node_jitter_sum_us_;
    std::vector<uint64_t> node_jitter_samples_;

    std::vector<double>  node_rx_power_sum_dbm_;
    std::vector<uint64_t> node_rx_power_samples_;
    std::vector<double>  node_rx_power_min_dbm_;
    std::vector<double>  node_rx_power_max_dbm_;

    void ResetWindowAccumulators_();
    void FinalizeAndLogWindow_(uint64_t t_sim_us, uint64_t win_dur_us);
    bool hasFreshRealAgentStates_() const;

    bool prefer_real_agent_states_{false};
    double real_agent_states_timeout_sec_{1.0};
    std::string real_agent_states_topic_;
    ros::Time last_real_agent_states_update_;

};

/**
 * @brief Configure ns-3 helpers and set the random seed
 */
void BasicWifiAdhoc::ConfigureNs3()
{
    SeedManager::SetRun(getYamlValue<int>(this->config_, "seed"));

    propagation_model_name_ =
        (this->config_["propagation_model"] ? this->config_["propagation_model"].as<std::string>()
                                                : std::string("three_gpp_v2v_urban"));
    use_building_obstacles_ = (propagation_model_name_ == "three_gpp_v2v_urban");

    // Buildings / obstacles are only materialized for models that explicitly depend on them.
    if (use_building_obstacles_ && this->config_["obstacles"])
    {
        YAML::Node buildings = this->config_["obstacles"];
        for (const auto& building : buildings)
        {
            double x_min = building["x"].as<double>() - building["size_x"].as<double>() / 2;
            double y_min = building["y"].as<double>() - building["size_y"].as<double>() / 2;
            double z_min = 0.0;
            double x_max = building["x"].as<double>() + building["size_x"].as<double>() / 2;
            double y_max = building["y"].as<double>() + building["size_y"].as<double>() / 2;
            double z_max = building["size_z"].as<double>();

            Ptr<Building> ns3_building = CreateObject<Building>();
            ns3_building->SetBoundaries(Box(x_min, x_max, y_min, y_max, z_min, z_max));
            ns3_building->SetBuildingType(Building::Office);
            ns3_building->SetExtWallsType(Building::ConcreteWithWindows);
            ns3_building->SetNFloors(1);
            ns3_building->SetNRoomsX(1);
            ns3_building->SetNRoomsY(1);

            RCLCPP_DEBUG(this->get_logger(),
                "Created a building with boundaries (%f, %f, %f, %f) and height %f",
                x_min, y_min, x_max, y_max, z_max);
        }
    }

    this->nodes = NodeContainer();

    // --- Wi-Fi ---
    std::string wifi_standard = getYamlValue<std::string>(this->config_, "wifi_standard");
    this->wifiHelper_ = std::make_unique<WifiHelper>();

    if (wifi_standard == "802.11g")
        this->wifiHelper_->SetStandard(WIFI_STANDARD_80211g);
    else if (wifi_standard == "802.11n")
        this->wifiHelper_->SetStandard(WIFI_STANDARD_80211n);
    else if (wifi_standard == "802.11ax")
        this->wifiHelper_->SetStandard(WIFI_STANDARD_80211ax);
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Unsupported WiFi standard: %s", wifi_standard.c_str());
        exit(EXIT_FAILURE);
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Using WiFi standard: " << wifi_standard.c_str());

    std::string wifi_phy_mode = getYamlValue<std::string>(this->config_, "wifi_phy_mode");
    this->wifiHelper_->SetRemoteStationManager("ns3::ConstantRateWifiManager",
        "DataMode", StringValue(wifi_phy_mode),
        "ControlMode", StringValue(wifi_phy_mode));

    Config::SetDefault("ns3::WifiRemoteStationManager::NonUnicastMode", StringValue(wifi_phy_mode));
    Config::SetDefault("ns3::HtConfiguration::ShortGuardIntervalSupported",
        BooleanValue(getYamlValue<bool>(this->config_, "short_guard_interval_supported")));

    // --- MAC ---
    this->wifiMacHelper_ = std::make_unique<WifiMacHelper>();
    this->wifiMacHelper_->SetType("ns3::AdhocWifiMac");

    // --- PHY (Spectrum + configurable propagation model) ---
    this->wifiPhyHelper_ = std::make_unique<SpectrumWifiPhyHelper>();
    SpectrumWifiPhyHelper *spectrumPhyHelper = dynamic_cast<SpectrumWifiPhyHelper *>(this->wifiPhyHelper_.get());

    Ptr<MultiModelSpectrumChannel> spectrumChannel = CreateObject<MultiModelSpectrumChannel>();

    const bool shadowing_enabled =
        (this->config_["shadowing_enabled"] ? this->config_["shadowing_enabled"].as<bool>() : true);
    const double channel_condition_update_period_ms =
        (this->config_["channel_condition_update_period_ms"]
             ? this->config_["channel_condition_update_period_ms"].as<double>()
             : 20.0);
    tx_power_dbm_ =
        (this->config_["tx_power_dbm"] ? this->config_["tx_power_dbm"].as<double>() : tx_power_dbm_);
    const double frequency_hz = getYamlValue<double>(this->config_, "frequency");

    if (propagation_model_name_ == "three_gpp_v2v_urban")
    {
        Ptr<ChannelConditionModel> m_condModel = CreateObject<ThreeGppV2vUrbanChannelConditionModel>();
        m_condModel->SetAttribute(
            "UpdatePeriod", TimeValue(MilliSeconds(channel_condition_update_period_ms)));

        propagationLossModel_ = CreateObject<ThreeGppV2vUrbanPropagationLossModel>();
        propagationLossModel_->SetAttribute("Frequency", DoubleValue(frequency_hz));
        propagationLossModel_->SetAttribute("ShadowingEnabled", BooleanValue(shadowing_enabled));
        propagationLossModel_->SetAttribute("ChannelConditionModel", PointerValue(m_condModel));
        spectrumChannel->AddPropagationLossModel(propagationLossModel_);
        RCLCPP_INFO(this->get_logger(),
                    "[NET] propagation_model=%s tx_power_dbm=%.2f shadowing=%s channel_condition_update_period_ms=%.1f obstacles_as_buildings=%s",
                    propagation_model_name_.c_str(),
                    tx_power_dbm_,
                    shadowing_enabled ? "true" : "false",
                    channel_condition_update_period_ms,
                    use_building_obstacles_ ? "true" : "false");
    }
    else if (propagation_model_name_ == "log_distance" ||
             propagation_model_name_ == "log_distance_range")
    {
        const double reference_distance_m =
            (this->config_["log_distance_reference_distance_m"]
                 ? this->config_["log_distance_reference_distance_m"].as<double>()
                 : 1.0);
        const double path_loss_exponent =
            (this->config_["log_distance_exponent"]
                 ? this->config_["log_distance_exponent"].as<double>()
                 : 3.4);
        const double reference_loss_db =
            (this->config_["log_distance_reference_loss_db"]
                 ? this->config_["log_distance_reference_loss_db"].as<double>()
                 : FriisReferenceLossDb(frequency_hz, reference_distance_m));

        auto log_distance_loss = CreateObject<LogDistancePropagationLossModel>();
        log_distance_loss->SetReference(reference_distance_m, reference_loss_db);
        log_distance_loss->SetPathLossExponent(path_loss_exponent);
        propagationLossModel_ = log_distance_loss;

        double max_comm_range_m = -1.0;
        if (this->config_["max_comm_range_m"])
        {
            max_comm_range_m = this->config_["max_comm_range_m"].as<double>();
        }
        if (propagation_model_name_ == "log_distance_range" && max_comm_range_m > 0.0)
        {
            auto range_loss = CreateObject<RangePropagationLossModel>();
            range_loss->SetAttribute("MaxRange", DoubleValue(max_comm_range_m));
            propagationLossModel_->SetNext(range_loss);
        }

        spectrumChannel->AddPropagationLossModel(propagationLossModel_);
        RCLCPP_INFO(this->get_logger(),
                    "[NET] propagation_model=%s tx_power_dbm=%.2f exponent=%.2f ref_dist=%.2f ref_loss=%.2f max_range=%.2f obstacles_as_buildings=%s",
                    propagation_model_name_.c_str(),
                    tx_power_dbm_,
                    path_loss_exponent,
                    reference_distance_m,
                    reference_loss_db,
                    max_comm_range_m,
                    use_building_obstacles_ ? "true" : "false");
        if (shadowing_enabled)
        {
            RCLCPP_WARN(this->get_logger(),
                        "[NET] shadowing_enabled is ignored for propagation_model=%s",
                        propagation_model_name_.c_str());
        }
    }
    else
    {
        RCLCPP_FATAL(this->get_logger(),
                     "Unsupported propagation_model: %s",
                     propagation_model_name_.c_str());
        exit(EXIT_FAILURE);
    }

    Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel>();
    spectrumChannel->SetPropagationDelayModel(delayModel);

    spectrumPhyHelper->SetChannel(spectrumChannel);
    spectrumPhyHelper->SetErrorRateModel(getYamlValue<std::string>(this->config_, "error_rate_model"));

    // Tx power (dBm)
    spectrumPhyHelper->Set("TxPowerStart", DoubleValue(tx_power_dbm_));
    spectrumPhyHelper->Set("TxPowerEnd", DoubleValue(tx_power_dbm_));

    if (frequency_hz == 2.4e9)
        spectrumPhyHelper->Set("ChannelSettings", StringValue("{0, 0, BAND_2_4GHZ, 0}"));
    else if (frequency_hz == 5e9)
        spectrumPhyHelper->Set("ChannelSettings", StringValue("{0, 0, BAND_5GHZ, 0}"));

    // --- IP stack ---
    this->internetHelper_ = std::make_unique<InternetStackHelper>();

    // --- APP layer randomization ---
    this->random_gen_start_flocking_app = CreateObject<UniformRandomVariable>();
    this->random_gen_start_flocking_app->SetAttribute("Min", DoubleValue(0.0));
    this->random_gen_start_flocking_app->SetAttribute("Max", DoubleValue(
        getYamlValue<uint32_t>(this->config_["flocking_flow"], "interval") / 1000000.0f));
}

void BasicWifiAdhoc::InitAgents()
{
    for (unsigned int i = 0; i < getYamlValue<unsigned int>(this->config_, "robots_number"); i++)
    {
        dancers_msgs::AgentStruct agent_ros_msg;

        agent_ros_msg.agent_id = i;
        agent_ros_msg.state.position.x = float(i);
        agent_ros_msg.state.position.y = 0.0;
        agent_ros_msg.state.position.z = 1.0;
        agent_ros_msg.state.velocity.x = 0.0;
        agent_ros_msg.state.velocity.y = 0.0;
        agent_ros_msg.state.velocity.z = 0.0;
        agent_ros_msg.state.heading = 0.0;

        this->SpawnUavClbk(agent_ros_msg);
    }
}

void BasicWifiAdhoc::SpawnUavClbk(const dancers_msgs::AgentStruct& msg)
{
    {
        std::shared_lock lock(this->agents_mutex_);
        if (this->agents_.find(msg.agent_id) != this->agents_.end())
        {
            RCLCPP_WARN(this->get_logger(), "Agent %d already exists, skipping spawn.", msg.agent_id);
            return;
        }
    }

    uint32_t ns3_index = this->nodes.GetN();
    Ptr<ns3::Node> new_node = CreateObject<ns3::Node>();
    this->nodes.Add(new_node);

    Ptr<MobilityModel> nodeMob = CreateObject<ConstantVelocityMobilityModel>();
    nodeMob->GetObject<ConstantVelocityMobilityModel>()->SetPosition(Vector(msg.state.position.x, msg.state.position.y, msg.state.position.z));
    nodeMob->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(Vector(msg.state.velocity.x, msg.state.velocity.y, msg.state.velocity.z));
    new_node->AggregateObject(nodeMob);

    if (use_building_obstacles_)
    {
        BuildingsHelper::Install(new_node);
    }

    this->AddWifiNetworkStack(new_node, msg.agent_id);

    {
        std::unique_lock lock(this->agents_mutex_);
        auto [it, success] = this->agents_.emplace(msg.agent_id, std::make_unique<Agent>(AgentFromRosMsg(msg)));
        if (!success)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to insert new agent %d in the map. Aborting.", msg.agent_id);
            exit(EXIT_FAILURE);
        }
        auto& [id, agent_ptr] = *it;

        agent_ptr->node = std::make_shared<Ptr<ns3::Node>>(new_node);
        agent_ptr->node_container_index = ns3_index;
        agent_ptr->cmd_velocity = {0.0, 0.0, 0.0};
        agent_ptr->cmd_heading = 0.0;
        agent_ptr->channel_busy_time = 0.0;
    }

    // Ensure per-node arrays can hold this agent_id
    EnsurePerNodeSize_(msg.agent_id);

    this->AddFlockingApplication(new_node);
    if (enable_swarm_msg_transport_)
    {
        this->AddSwarmMessageApplication(new_node, msg.agent_id);
    }
}

void BasicWifiAdhoc::AddWifiNetworkStack(Ptr<ns3::Node> node, uint32_t agent_id)
{
    if (!this->wifiPhyHelper_ || !this->wifiMacHelper_ || !this->wifiHelper_ || !this->internetHelper_)
    {
        RCLCPP_FATAL(this->get_logger(), "Wifi helpers are not initialized, cannot add node to wifi network");
        return;
    }

    NetDeviceContainer new_net_device = (*this->wifiHelper_).Install((*this->wifiPhyHelper_), (*this->wifiMacHelper_), node);

    this->internetHelper_->Install(node);

    Ipv4InterfaceAddress nodeIpv4AddressInterface(
        Ipv4Address(("10.0.0." + std::to_string(agent_id + 1)).c_str()),
        Ipv4Mask("255.255.255.0")
    );

    Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
    int32_t interfaceIndex = ipv4->AddInterface(new_net_device.Get(0));
    if (!ipv4->AddAddress(interfaceIndex, nodeIpv4AddressInterface))
    {
        RCLCPP_FATAL(this->get_logger(), "Couldn't assign IPv4 address to node %d", agent_id);
    }
    ipv4->SetUp(interfaceIndex);
}

void BasicWifiAdhoc::AddFlockingApplication(Ptr<ns3::Node> node)
{
    if (!this->config_["flocking_flow"].IsDefined())
    {
        RCLCPP_FATAL(this->get_logger(), "Flocking flow is not defined in the configuration file, aborting.");
        exit(EXIT_FAILURE);
    }

    auto agent_id_opt = this->FindAgentIdByNode(node);
    if (!agent_id_opt)
    {
        RCLCPP_FATAL(this->get_logger(),
            "Could not find the agent associated to the node %d (ns-3 index), aborting.",
            node->GetId());
        exit(EXIT_FAILURE);
    }
    uint32_t agent_id = agent_id_opt.value();

    Ptr<FlockingBroadcaster> flocking_broadcaster = CreateObject<FlockingBroadcaster>();
    flocking_broadcaster->SetStartTime(Simulator::Now() + Seconds(this->random_gen_start_flocking_app->GetValue()));
    flocking_broadcaster->SetAttribute("PacketSize", UintegerValue(getYamlValue<uint32_t>(this->config_["flocking_flow"], "packet_size")));
    flocking_broadcaster->SetAttribute("Port", UintegerValue(getYamlValue<uint32_t>(this->config_["flocking_flow"], "port")));
    Ptr<ConstantRandomVariable> rand = CreateObject<ConstantRandomVariable>();
    rand->SetAttribute("Constant", DoubleValue(getYamlValue<uint32_t>(this->config_["flocking_flow"], "interval") / 1000000.0f));
    flocking_broadcaster->SetAttribute("Interval", PointerValue(rand));
    flocking_broadcaster->SetAttribute("FlowId", UintegerValue(getYamlValue<uint32_t>(this->config_["flocking_flow"], "flow_id")));

    Ptr<FlockingReceiver> flocking_receiver = CreateObject<FlockingReceiver>();
    flocking_receiver->SetStartTime(Seconds(0.0));
    flocking_receiver->SetAttribute("Port", UintegerValue(getYamlValue<uint32_t>(this->config_["flocking_flow"], "port")));

    node->AddApplication(flocking_broadcaster);
    node->AddApplication(flocking_receiver);

    flocking_broadcaster->TraceConnect("Tx", std::to_string(agent_id), MakeCallback(&BasicWifiAdhoc::flocking_broadcaster_clbk, this));
    flocking_receiver->TraceConnect("Rx", std::to_string(agent_id), MakeCallback(&BasicWifiAdhoc::flocking_receiver_clbk, this));
}

void BasicWifiAdhoc::AddSwarmMessageApplication(Ptr<ns3::Node> node, uint32_t agent_id)
{
    Ptr<SwarmMessageBroadcaster> swarm_broadcaster = CreateObject<SwarmMessageBroadcaster>();
    swarm_broadcaster->SetStartTime(Seconds(0.0));
    swarm_broadcaster->SetAttribute("Port", UintegerValue(swarm_msg_port_));
    swarm_broadcaster->SetAttribute("FlowId", UintegerValue(swarm_msg_flow_id_));
    swarm_broadcaster->SetAttribute("SenderId", UintegerValue(agent_id));

    Ptr<SwarmMessageReceiver> swarm_receiver = CreateObject<SwarmMessageReceiver>();
    swarm_receiver->SetStartTime(Seconds(0.0));
    swarm_receiver->SetAttribute("Port", UintegerValue(swarm_msg_port_));

    node->AddApplication(swarm_broadcaster);
    node->AddApplication(swarm_receiver);

    swarm_msg_broadcasters_[agent_id] = swarm_broadcaster;

    swarm_broadcaster->TraceConnect(
        "Tx", std::to_string(agent_id), MakeCallback(&BasicWifiAdhoc::swarmMsgBroadcasterClbk, this));
    swarm_receiver->TraceConnect(
        "Rx", std::to_string(agent_id), MakeCallback(&BasicWifiAdhoc::swarmMsgReceiverClbk, this));
}

std::optional<uint32_t> BasicWifiAdhoc::FindAgentIdByNode(const Ptr<ns3::Node>& node) const
{
    std::shared_lock lock(this->agents_mutex_);
    for (const auto& [id, agent] : this->agents_) {
        if (agent->node_container_index == node->GetId()) {
            return id;
        }
    }
    return std::nullopt;
}

std::optional<uint32_t> BasicWifiAdhoc::GetNodeIndexFromAgentId_(uint32_t agent_id) const
{
    std::shared_lock lock(this->agents_mutex_);
    auto it = this->agents_.find(agent_id);
    if (it == this->agents_.end()) return std::nullopt;
    return it->second->node_container_index;
}

// NOTE: PHY->NET still legacy PoseVector, so keep parse legacy.
void BasicWifiAdhoc::UpdateAgentsPosition(const dancers_update_proto::DancersUpdate& update_msg)
{
    if (!update_msg.agent_states_gz().empty())
    {
        protobuf_msgs::AgentStateBatch batch;
        try
        {
            if (!batch.ParseFromString(gzip_decompress(update_msg.agent_states_gz())))
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to parse coordinator agent state batch.");
                exit(EXIT_FAILURE);
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to decode coordinator agent state batch: %s", e.what());
            exit(EXIT_FAILURE);
        }

        size_t applied_states = 0;
        size_t skipped_stale_states = 0;
        size_t skipped_nonreal_states = 0;
        uint64_t max_skipped_age_us = 0;
        std::unique_lock lock(this->agents_mutex_);
        for (const auto& sample : batch.states())
        {
            auto agent = this->agents_.find(sample.agent_id());
            if (agent == this->agents_.end())
            {
                continue;
            }

            const bool sample_usable = sample.has_real_state() && sample.is_fresh();
            if (ignore_stale_agent_states_ && !sample_usable)
            {
                if (sample.has_real_state())
                {
                    ++skipped_stale_states;
                    max_skipped_age_us = std::max<uint64_t>(max_skipped_age_us, sample.age_us());
                }
                else
                {
                    ++skipped_nonreal_states;
                }
                continue;
            }

            agent->second->position = Eigen::Vector3d(sample.x(), sample.y(), sample.z());
            agent->second->velocity = Eigen::Vector3d(sample.vx(), sample.vy(), sample.vz());
            agent->second->heading = sample.heading();
            ++applied_states;

            if (!agent->second->initial_position_saved)
            {
                agent->second->initial_position = agent->second->position;
                agent->second->initial_position_saved = true;
            }
        }
        if (warn_on_stale_agent_states_ && (skipped_stale_states > 0 || skipped_nonreal_states > 0))
        {
            ROS_WARN_THROTTLE(1.0,
                              "[NET] skipped stale/non-real coordinator agent states sample_seq=%lu applied=%zu skipped_stale=%zu skipped_nonreal=%zu max_skipped_age_us=%lu",
                              static_cast<unsigned long>(batch.sample_seq()),
                              applied_states,
                              skipped_stale_states,
                              skipped_nonreal_states,
                              static_cast<unsigned long>(max_skipped_age_us));
        }
        if (trace_agent_state_batch_enable_ &&
            (batch.sample_seq() % static_cast<uint64_t>(trace_agent_state_batch_every_n_)) == 0)
        {
            ROS_INFO("[AGENT_STATE_TRACE][NET] sample_seq=%lu t_sample_us=%lu batch_states=%d applied=%zu skipped_stale=%zu skipped_nonreal=%zu",
                     static_cast<unsigned long>(batch.sample_seq()),
                     static_cast<unsigned long>(batch.t_sample_us()),
                     batch.states_size(),
                     applied_states,
                     skipped_stale_states,
                     skipped_nonreal_states);
        }
        return;
    }

    if (hasFreshRealAgentStates_())
    {
        ROS_INFO_THROTTLE(
            2.0,
            "[NET] ignoring legacy PHY PoseVector because fresh real agent states are available.");
        return;
    }

    if (update_msg.payload().empty())
        return;

    dancers_update_proto::PoseVector robots_positions;
    if (!robots_positions.ParseFromString(gzip_decompress(update_msg.payload())))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse robots positions from protobuf message (legacy PoseVector).");
        exit(EXIT_FAILURE);
    }

    for (const auto& position_msg : robots_positions.pose())
    {
        std::unique_lock lock(this->agents_mutex_);
        auto agent = this->agents_.find(position_msg.agent_id());
        if (agent != this->agents_.end())
        {
            agent->second->position = Eigen::Vector3d(position_msg.x(), position_msg.y(), position_msg.z());
            agent->second->velocity = Eigen::Vector3d(position_msg.vx(), position_msg.vy(), position_msg.vz());

            if (!agent->second->initial_position_saved)
            {
                agent->second->initial_position = agent->second->position;
                agent->second->initial_position_saved = true;
            }
        }
    }
}

void BasicWifiAdhoc::realAgentStatesCallback(const dancers_msgs::AgentStructArrayConstPtr& msg)
{
    if (msg->structs.empty())
        return;

    std::unique_lock lock(this->agents_mutex_);

    for (const auto& agent_struct : msg->structs)
    {
        auto agent_it = this->agents_.find(agent_struct.agent_id);
        if (agent_it == this->agents_.end())
            continue;

        auto& agent = agent_it->second;
        agent->position = Eigen::Vector3d(
            agent_struct.state.position.x,
            agent_struct.state.position.y,
            agent_struct.state.position.z);
        agent->velocity = Eigen::Vector3d(
            agent_struct.state.velocity.x,
            agent_struct.state.velocity.y,
            agent_struct.state.velocity.z);
        agent->heading = agent_struct.state.heading;

        if (!agent->initial_position_saved)
        {
            agent->initial_position = agent->position;
            agent->initial_position_saved = true;
        }

        // Do not touch ns-3 mobility objects from the ROS callback thread.
        // StepSimulation() applies the cached states on the simulation thread.
    }

    last_real_agent_states_update_ = ros::Time::now();
}

void BasicWifiAdhoc::ApplyCachedAgentStatesToMobility_()
{
    std::shared_lock lock(this->agents_mutex_);

    for (const auto& [agent_id, agent] : this->agents_)
    {
        if (!agent)
            continue;

        Ptr<ConstantVelocityMobilityModel> mobility =
            this->nodes.Get(agent->node_container_index)->GetObject<ConstantVelocityMobilityModel>();
        if (!mobility)
            continue;

        mobility->SetPosition(Vector(agent->position.x(), agent->position.y(), agent->position.z()));
        mobility->SetVelocity(Vector(agent->velocity.x(), agent->velocity.y(), agent->velocity.z()));
    }
}

void BasicWifiAdhoc::swarmTxBytesCallback(const std_msgs::UInt8MultiArrayConstPtr& msg)
{
    if (!enable_swarm_msg_transport_)
        return;

    ++swarm_tx_ros_msgs_;
    ROS_INFO_STREAM_THROTTLE(1.0, "[NET][SWARM] ros tx msgs=" << swarm_tx_ros_msgs_
                             << " ns3 tx msgs=" << swarm_tx_ns3_msgs_
                             << " ns3 rx msgs=" << swarm_rx_ns3_msgs_
                             << " ros rx msgs=" << swarm_rx_ros_msgs_);

    const std::string bytes(msg->data.begin(), msg->data.end());
    relay_racer_proto::RacerSwarmMsg wrapper;
    if (!wrapper.ParseFromString(bytes))
    {
        ROS_WARN_THROTTLE(1.0, "[NET] failed to parse RacerSwarmMsg from swarm tx bytes.");
        return;
    }

    const uint32_t network_tx_id =
        (wrapper.network_tx_id() != 0) ? wrapper.network_tx_id() : wrapper.src_id();
    const int platform_src_id = static_cast<int>(network_tx_id) + swarm_msg_id_offset_;
    if (platform_src_id < 0)
    {
        ROS_WARN_THROTTLE(
            1.0,
            "[NET] invalid platform source id for swarm network_tx_id=%u src_id=%u id_offset=%d",
            network_tx_id,
            wrapper.src_id(),
            swarm_msg_id_offset_);
        return;
    }

    std::lock_guard<std::mutex> lock(swarm_msg_tx_mutex_);
    if (swarm_msg_queue_limit_ > 0 &&
        pending_swarm_tx_msgs_.size() >= static_cast<size_t>(swarm_msg_queue_limit_))
    {
        if (ShouldTraceSwarmWrapper_(wrapper))
        {
            TraceSwarmWrapper_(
                "net_tx_queue_drop",
                wrapper,
                "platform_src_id=" + std::to_string(platform_src_id) +
                    " queue_limit=" + std::to_string(swarm_msg_queue_limit_) +
                    " pending_before=" + std::to_string(pending_swarm_tx_msgs_.size()));
        }
        pending_swarm_tx_bytes_ -= pending_swarm_tx_msgs_.front().bytes.size();
        pending_swarm_tx_msgs_.pop_front();
        ROS_WARN_THROTTLE(1.0, "[NET] swarm tx queue full, dropping oldest pending swarm packet.");
    }

    pending_swarm_tx_msgs_.push_back(
        PendingSwarmTxMsg{static_cast<uint32_t>(platform_src_id), bytes});
    pending_swarm_tx_bytes_ += bytes.size();

    if (swarm_msg_queue_warn_threshold_ > 0 &&
        pending_swarm_tx_msgs_.size() >= static_cast<size_t>(swarm_msg_queue_warn_threshold_))
    {
        ROS_WARN_THROTTLE(
            1.0,
            "[NET] swarm tx pending queue backlog size=%zu bytes=%lu limit=%d warn_threshold=%d",
            pending_swarm_tx_msgs_.size(),
            static_cast<unsigned long>(pending_swarm_tx_bytes_),
            swarm_msg_queue_limit_,
            swarm_msg_queue_warn_threshold_);
    }

    if (ShouldTraceSwarmWrapper_(wrapper))
    {
        TraceSwarmWrapper_(
            "net_tx_enqueue",
            wrapper,
            "platform_src_id=" + std::to_string(platform_src_id) +
                " pending_after=" + std::to_string(pending_swarm_tx_msgs_.size()));
    }
}

void BasicWifiAdhoc::DrainPendingSwarmTx_()
{
    if (!enable_swarm_msg_transport_)
        return;

    std::deque<PendingSwarmTxMsg> pending;
    {
        std::lock_guard<std::mutex> lock(swarm_msg_tx_mutex_);
        pending.swap(pending_swarm_tx_msgs_);
        pending_swarm_tx_bytes_ = 0;
    }

    for (const auto& item : pending)
    {
        auto it = swarm_msg_broadcasters_.find(item.platform_src_id);
        if (it == swarm_msg_broadcasters_.end())
        {
            ROS_WARN_THROTTLE(1.0, "[NET] no swarm broadcaster for platform agent_id=%u", item.platform_src_id);
            continue;
        }

        if (trace_swarm_msg_enable_)
        {
            relay_racer_proto::RacerSwarmMsg wrapper;
            if (wrapper.ParseFromString(item.bytes) && ShouldTraceSwarmWrapper_(wrapper))
            {
                TraceSwarmWrapper_(
                    "net_broadcast_enqueue",
                    wrapper,
                    "platform_src_id=" + std::to_string(item.platform_src_id) +
                        " batch_size=" + std::to_string(pending.size()));
            }
        }
        it->second->EnqueuePayload(item.bytes);
    }
}

void BasicWifiAdhoc::swarmMsgBroadcasterClbk(std::string context, Ptr<const Packet> /*packet*/)
{
    uint32_t tx_agent_id = static_cast<uint32_t>(std::stoi(context));
    EnsurePerNodeSize_(tx_agent_id);

    ++swarm_tx_ns3_msgs_;
    ROS_INFO_STREAM_THROTTLE(1.0, "[NET][SWARM] ns3 tx callback agent=" << tx_agent_id
                             << " ros tx msgs=" << swarm_tx_ros_msgs_
                             << " ns3 tx msgs=" << swarm_tx_ns3_msgs_
                             << " ns3 rx msgs=" << swarm_rx_ns3_msgs_
                             << " ros rx msgs=" << swarm_rx_ros_msgs_);

    tx_pkts_total_++;
    node_tx_pkts_[tx_agent_id]++;
}

void BasicWifiAdhoc::swarmMsgReceiverClbk(std::string context, Ptr<const Packet> packet, int peer_id)
{
    uint32_t rx_agent_id = static_cast<uint32_t>(std::stoi(context));
    ++swarm_rx_ns3_msgs_;
    ROS_INFO_STREAM_THROTTLE(1.0, "[NET][SWARM] ns3 rx callback agent=" << rx_agent_id
                             << " ros tx msgs=" << swarm_tx_ros_msgs_
                             << " ns3 tx msgs=" << swarm_tx_ns3_msgs_
                             << " ns3 rx msgs=" << swarm_rx_ns3_msgs_
                             << " ros rx msgs=" << swarm_rx_ros_msgs_);

    SwarmMessageHeader swarm_hdr;
    packet->PeekHeader(swarm_hdr);
    uint32_t tx_agent_id = swarm_hdr.GetSenderId();
    if (tx_agent_id == 0 && peer_id >= 0)
    {
        tx_agent_id = static_cast<uint32_t>(peer_id);
    }

    EnsurePerNodeSize_(rx_agent_id);
    EnsurePerNodeSize_(tx_agent_id);

    Ptr<Packet> payload_packet = packet->Copy();
    payload_packet->RemoveHeader(swarm_hdr);
    std::string payload_bytes(payload_packet->GetSize(), '\0');
    if (!payload_bytes.empty())
    {
        payload_packet->CopyData(reinterpret_cast<uint8_t*>(&payload_bytes[0]), payload_bytes.size());
    }

    rx_pkts_total_++;
    rx_bytes_total_ += packet->GetSize();
    node_rx_pkts_[rx_agent_id]++;
    node_rx_bytes_[rx_agent_id] += packet->GetSize();

    uint64_t now_us = Simulator::Now().ToInteger(Time::US);
    uint64_t tx_us = swarm_hdr.GetTxTimeUs();
    uint64_t delay_us = 0;
    if (tx_us > 0 && now_us >= tx_us)
    {
        delay_us = now_us - tx_us;
        delay_sum_us_ += delay_us;
        if (delay_us > delay_max_us_) delay_max_us_ = delay_us;

        node_delay_sum_us_[rx_agent_id] += delay_us;
        if (delay_us > node_delay_max_us_[rx_agent_id]) node_delay_max_us_[rx_agent_id] = delay_us;

        uint64_t key = PairFlowKey(rx_agent_id, tx_agent_id, swarm_hdr.GetFlowId());
        auto it_last = last_delay_us_by_pair_.find(key);
        if (it_last != last_delay_us_by_pair_.end())
        {
            uint64_t j = AbsDiffU64(delay_us, it_last->second);
            jitter_sum_us_ += j;
            jitter_samples_++;
            node_jitter_sum_us_[rx_agent_id] += j;
            node_jitter_samples_[rx_agent_id] += 1;
        }
        last_delay_us_by_pair_[key] = delay_us;
    }

    bool is_dup = false;
    {
        uint64_t key = PairFlowKey(rx_agent_id, tx_agent_id, swarm_hdr.GetFlowId());
        uint32_t seq = swarm_hdr.GetSeq();
        auto it_seq = last_seq_by_pair_.find(key);
        if (it_seq != last_seq_by_pair_.end())
        {
            uint32_t last = it_seq->second;
            if (seq == last)
            {
                is_dup = true;
                dup_pkts_total_++;
                node_dup_pkts_[rx_agent_id]++;
            }
            else if (seq > last + 1)
            {
                uint64_t miss = uint64_t(seq - last - 1);
                lost_pkts_total_ += miss;
                node_lost_pkts_[rx_agent_id] += miss;
                it_seq->second = seq;
            }
            else if (seq > last)
            {
                it_seq->second = seq;
            }
        }
        else
        {
            last_seq_by_pair_[key] = seq;
        }
    }

    double rx_dbm_for_event = std::numeric_limits<double>::quiet_NaN();
    bool has_rx_dbm_for_event = false;
    Ptr<MobilityModel> txMob = nullptr;

    if (propagationLossModel_)
    {
        auto tx_idx_opt = GetNodeIndexFromAgentId_(tx_agent_id);
        auto rx_idx_opt = GetNodeIndexFromAgentId_(rx_agent_id);
        if (tx_idx_opt && rx_idx_opt)
        {
            txMob = this->nodes.Get(tx_idx_opt.value())->GetObject<MobilityModel>();
            Ptr<MobilityModel> rxMob = this->nodes.Get(rx_idx_opt.value())->GetObject<MobilityModel>();
            if (txMob && rxMob)
            {
                double rx_dbm = propagationLossModel_->CalcRxPower(tx_power_dbm_, txMob, rxMob);
                rx_dbm_for_event = rx_dbm;
                has_rx_dbm_for_event = true;

                rx_power_sum_dbm_ += rx_dbm;
                rx_power_samples_++;
                if (rx_dbm < rx_power_min_dbm_) rx_power_min_dbm_ = rx_dbm;
                if (rx_dbm > rx_power_max_dbm_) rx_power_max_dbm_ = rx_dbm;

                node_rx_power_sum_dbm_[rx_agent_id] += rx_dbm;
                node_rx_power_samples_[rx_agent_id] += 1;
                if (rx_dbm < node_rx_power_min_dbm_[rx_agent_id]) node_rx_power_min_dbm_[rx_agent_id] = rx_dbm;
                if (rx_dbm > node_rx_power_max_dbm_[rx_agent_id]) node_rx_power_max_dbm_[rx_agent_id] = rx_dbm;
            }
        }
    }

    {
        RxEventRow e;
        e.rx_time_us = now_us;
        e.rx_id = rx_agent_id;
        e.tx_time_us = tx_us;
        e.tx_id = tx_agent_id;
        e.hdr_sender_id = swarm_hdr.GetSenderId();
        e.flow_id = swarm_hdr.GetFlowId();
        e.seq = swarm_hdr.GetSeq();
        e.pkt_bytes = packet->GetSize();
        e.delay_us = delay_us;
        e.rx_power_dbm = rx_dbm_for_event;
        e.has_rx_power = has_rx_dbm_for_event;
        e.is_duplicate = is_dup;
        if (txMob)
        {
            const Vector pos = txMob->GetPosition();
            const Vector vel = txMob->GetVelocity();
            e.has_packet_state = true;
            e.pos_x = pos.x;
            e.pos_y = pos.y;
            e.pos_z = pos.z;
            e.vel_x = vel.x;
            e.vel_y = vel.y;
            e.vel_z = vel.z;
        }
        rx_events_window_.push_back(e);
    }

    relay_racer_proto::RacerSwarmMsg wrapper;
    if (!wrapper.ParseFromString(payload_bytes))
    {
        ROS_WARN_THROTTLE(1.0, "[NET] failed to parse received RacerSwarmMsg payload.");
        return;
    }

    const int racer_rx_id = static_cast<int>(rx_agent_id) - swarm_msg_id_offset_;
    if (racer_rx_id <= 0)
    {
        ROS_WARN_THROTTLE(1.0, "[NET] invalid racer_rx_id from platform rx=%u id_offset=%d",
                          rx_agent_id, swarm_msg_id_offset_);
        return;
    }
    wrapper.set_dst_id(static_cast<uint32_t>(racer_rx_id));

    if (ShouldTraceSwarmWrapper_(wrapper))
    {
        TraceSwarmWrapper_(
            "net_rx_from_ns3",
            wrapper,
            "platform_rx_id=" + std::to_string(rx_agent_id) +
                " platform_tx_id=" + std::to_string(tx_agent_id) +
                " flow_id=" + std::to_string(swarm_hdr.GetFlowId()) +
                " seq=" + std::to_string(swarm_hdr.GetSeq()) +
                " delay_us=" + std::to_string(delay_us));
    }

    std::string delivered_bytes;
    if (!wrapper.SerializeToString(&delivered_bytes))
    {
        ROS_WARN_THROTTLE(1.0, "[NET] failed to serialize delivered RacerSwarmMsg.");
        return;
    }

    int32_t rssi_x10 = 0;
    if (has_rx_dbm_for_event && std::isfinite(rx_dbm_for_event))
    {
        rssi_x10 = static_cast<int32_t>(std::llround(rx_dbm_for_event * 10.0));
    }

    DeliveredSwarmPacketRow delivered;
    delivered.meta.rx_time_us = now_us;
    delivered.meta.rx_id = rx_agent_id;
    delivered.meta.tx_time_us = tx_us;
    delivered.meta.tx_id = tx_agent_id;
    delivered.meta.hdr_sender_id = swarm_hdr.GetSenderId();
    delivered.meta.flow_id = swarm_hdr.GetFlowId();
    delivered.meta.seq = swarm_hdr.GetSeq();
    delivered.meta.pkt_bytes = packet->GetSize();
    delivered.meta.delay_us = delay_us;
    delivered.meta.rx_power_dbm = rx_dbm_for_event;
    delivered.meta.has_rx_power = has_rx_dbm_for_event;
    delivered.meta.is_duplicate = is_dup;
    delivered.meta.has_packet_state = txMob != nullptr;
    if (txMob)
    {
        const Vector pos = txMob->GetPosition();
        const Vector vel = txMob->GetVelocity();
        delivered.meta.pos_x = pos.x;
        delivered.meta.pos_y = pos.y;
        delivered.meta.pos_z = pos.z;
        delivered.meta.vel_x = vel.x;
        delivered.meta.vel_y = vel.y;
        delivered.meta.vel_z = vel.z;
    }
    delivered.rssi_dbm_x10 = rssi_x10;
    delivered.wrapped_msg = std::move(delivered_bytes);

    if (ShouldTraceSwarmWrapper_(wrapper))
    {
        TraceSwarmWrapper_(
            "net_rx_batch_queue",
            wrapper,
            "window_pending_packets=" +
                std::to_string(delivered_swarm_packets_window_.size() + 1));
    }

    delivered_swarm_packets_window_.push_back(std::move(delivered));
}

bool BasicWifiAdhoc::ShouldTraceSwarmWrapper_(
    const relay_racer_proto::RacerSwarmMsg& wrapper) const
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

void BasicWifiAdhoc::TraceSwarmWrapper_(
    const char* stage,
    const relay_racer_proto::RacerSwarmMsg& wrapper,
    const std::string& extra) const
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

// ---- Per-node stats storage helpers ----
void BasicWifiAdhoc::InitPerNodeStatsStorage_()
{
    uint32_t max_id = 0;
    {
        std::shared_lock lock(this->agents_mutex_);
        for (const auto& [id, _] : this->agents_) max_id = std::max(max_id, id);
    }
    EnsurePerNodeSize_(max_id);
}

void BasicWifiAdhoc::EnsurePerNodeSize_(uint32_t agent_id)
{
    size_t need = static_cast<size_t>(agent_id) + 1;
    if (node_rx_pkts_.size() >= need) return;

    node_tx_pkts_.resize(need, 0);
    node_rx_pkts_.resize(need, 0);
    node_rx_bytes_.resize(need, 0);
    node_lost_pkts_.resize(need, 0);
    node_dup_pkts_.resize(need, 0);
    node_delay_sum_us_.resize(need, 0);
    node_delay_max_us_.resize(need, 0);
    node_jitter_sum_us_.resize(need, 0);
    node_jitter_samples_.resize(need, 0);

    node_rx_power_sum_dbm_.resize(need, 0.0);
    node_rx_power_samples_.resize(need, 0);
    node_rx_power_min_dbm_.resize(need, std::numeric_limits<double>::infinity());
    node_rx_power_max_dbm_.resize(need, -std::numeric_limits<double>::infinity());
}

// ---- Trace callbacks ----
void BasicWifiAdhoc::flocking_broadcaster_clbk(std::string context, Ptr<const Packet> /*packet*/)
{
    uint32_t tx_agent_id = static_cast<uint32_t>(std::stoi(context));
    EnsurePerNodeSize_(tx_agent_id);

    tx_pkts_total_++;
    node_tx_pkts_[tx_agent_id]++;
}

void BasicWifiAdhoc::flocking_receiver_clbk(std::string context, Ptr<const Packet> packet, int peer_id)
{
    uint32_t rx_agent_id = static_cast<uint32_t>(std::stoi(context));
    uint32_t tx_agent_id = static_cast<uint32_t>(peer_id);

    EnsurePerNodeSize_(rx_agent_id);
    EnsurePerNodeSize_(tx_agent_id);

    FlockingHeader fl_hdr;
    packet->PeekHeader(fl_hdr);

    Vector neighbor_position = fl_hdr.GetPosition();
    Vector neighbor_velocity = fl_hdr.GetVelocity();

    {
        std::unique_lock lock(this->agents_mutex_);
        if (this->agents_.at(rx_agent_id)->neighbors.find(tx_agent_id) == this->agents_.at(rx_agent_id)->neighbors.end())
        {
            this->agents_.at(rx_agent_id)->neighbors[tx_agent_id] = NeighborInfo();
        }

        this->agents_.at(rx_agent_id)->neighbors.at(tx_agent_id).position =
            Eigen::Vector3d(neighbor_position.x, neighbor_position.y, neighbor_position.z);
        this->agents_.at(rx_agent_id)->neighbors.at(tx_agent_id).velocity =
            Eigen::Vector3d(neighbor_velocity.x, neighbor_velocity.y, neighbor_velocity.z);
        this->agents_.at(rx_agent_id)->neighbors.at(tx_agent_id).id = tx_agent_id;
        this->agents_.at(rx_agent_id)->neighbors.at(tx_agent_id).last_seen = Simulator::Now().ToInteger(Time::US);
        this->agents_.at(rx_agent_id)->neighbors.at(tx_agent_id).link_type = LinkType::FlockingLink;

        this->agents_.at(rx_agent_id)->heartbeat_received++;
    }

    rx_pkts_total_++;
    rx_bytes_total_ += packet->GetSize();

    node_rx_pkts_[rx_agent_id]++;
    node_rx_bytes_[rx_agent_id] += packet->GetSize();

    uint64_t now_us = Simulator::Now().ToInteger(Time::US);
    uint64_t tx_us  = fl_hdr.GetTxTimeUs();

    // 计算时延
    uint64_t delay_us = 0;
    if (tx_us > 0 && now_us >= tx_us)
    {
        delay_us = now_us - tx_us;

        delay_sum_us_ += delay_us;
        if (delay_us > delay_max_us_) delay_max_us_ = delay_us;

        node_delay_sum_us_[rx_agent_id] += delay_us;
        if (delay_us > node_delay_max_us_[rx_agent_id]) node_delay_max_us_[rx_agent_id] = delay_us;

        uint64_t key = PairFlowKey(rx_agent_id, tx_agent_id, fl_hdr.GetFlowId());
        auto it_last = last_delay_us_by_pair_.find(key);
        if (it_last != last_delay_us_by_pair_.end())
        {
            uint64_t j = AbsDiffU64(delay_us, it_last->second);

            jitter_sum_us_ += j;
            jitter_samples_++;

            node_jitter_sum_us_[rx_agent_id] += j;
            node_jitter_samples_[rx_agent_id] += 1;
        }
        last_delay_us_by_pair_[key] = delay_us;
    }

    // 先判断 dup，再更新 last_seq
    bool is_dup = false;
    {
        uint64_t key = PairFlowKey(rx_agent_id, tx_agent_id, fl_hdr.GetFlowId());
        uint32_t seq = fl_hdr.GetSeq();

        auto it_seq = last_seq_by_pair_.find(key);
        if (it_seq != last_seq_by_pair_.end())
        {
            uint32_t last = it_seq->second;
            if (seq == last)
            {
                is_dup = true;
                dup_pkts_total_++;
                node_dup_pkts_[rx_agent_id]++;
            }
            else if (seq > last + 1)
            {
                uint64_t miss = uint64_t(seq - last - 1);
                lost_pkts_total_ += miss;
                node_lost_pkts_[rx_agent_id] += miss;
                it_seq->second = seq;
            }
            else if (seq > last)
            {
                it_seq->second = seq;
            }
        }
        else
        {
            last_seq_by_pair_[key] = seq;
        }
    }

    // For per-packet event row
    double rx_dbm_for_event = std::numeric_limits<double>::quiet_NaN();
    bool has_rx_dbm_for_event = false;

    if (propagationLossModel_)
    {
        auto tx_idx_opt = GetNodeIndexFromAgentId_(tx_agent_id);
        auto rx_idx_opt = GetNodeIndexFromAgentId_(rx_agent_id);
        if (tx_idx_opt && rx_idx_opt)
        {
            Ptr<MobilityModel> txMob = this->nodes.Get(tx_idx_opt.value())->GetObject<MobilityModel>();
            Ptr<MobilityModel> rxMob = this->nodes.Get(rx_idx_opt.value())->GetObject<MobilityModel>();

            if (txMob && rxMob)
            {
                double rx_dbm = propagationLossModel_->CalcRxPower(tx_power_dbm_, txMob, rxMob);

                rx_dbm_for_event = rx_dbm;
                has_rx_dbm_for_event = true;

                rx_power_sum_dbm_ += rx_dbm;
                rx_power_samples_++;
                if (rx_dbm < rx_power_min_dbm_) rx_power_min_dbm_ = rx_dbm;
                if (rx_dbm > rx_power_max_dbm_) rx_power_max_dbm_ = rx_dbm;

                node_rx_power_sum_dbm_[rx_agent_id] += rx_dbm;
                node_rx_power_samples_[rx_agent_id] += 1;
                if (rx_dbm < node_rx_power_min_dbm_[rx_agent_id]) node_rx_power_min_dbm_[rx_agent_id] = rx_dbm;
                if (rx_dbm > node_rx_power_max_dbm_[rx_agent_id]) node_rx_power_max_dbm_[rx_agent_id] = rx_dbm;
            }
        }
    }

    // ---- 关键修改：无论是否开 CSV，都把事件写进 rx_events_window_（用于发给 PHY） ----
    {
        RxEventRow e;
        e.rx_time_us = now_us;
        e.rx_id = rx_agent_id;

        e.tx_time_us = tx_us;
        e.tx_id = tx_agent_id;

        e.hdr_sender_id = fl_hdr.GetSenderId();
        e.flow_id = fl_hdr.GetFlowId();
        e.seq = fl_hdr.GetSeq();

        e.pkt_bytes = packet->GetSize();
        e.delay_us = delay_us;

        e.rx_power_dbm = rx_dbm_for_event;
        e.has_rx_power = has_rx_dbm_for_event;

        e.is_duplicate = is_dup;
        e.has_packet_state = true;
        e.pos_x = neighbor_position.x;
        e.pos_y = neighbor_position.y;
        e.pos_z = neighbor_position.z;
        e.vel_x = neighbor_velocity.x;
        e.vel_y = neighbor_velocity.y;
        e.vel_z = neighbor_velocity.z;

        rx_events_window_.push_back(e);
    }
}


// ---- Metrics helpers ----
void BasicWifiAdhoc::ResetWindowAccumulators_()
{
    window_idx_ = static_cast<uint64_t>(this->it);

    tx_pkts_total_ = 0;

    rx_pkts_total_ = 0;
    rx_bytes_total_ = 0;

    lost_pkts_total_ = 0;
    dup_pkts_total_ = 0;

    delay_sum_us_ = 0;
    delay_max_us_ = 0;

    jitter_sum_us_ = 0;
    jitter_samples_ = 0;

    rx_power_sum_dbm_ = 0.0;
    rx_power_samples_ = 0;
    rx_power_min_dbm_ = std::numeric_limits<double>::infinity();
    rx_power_max_dbm_ = -std::numeric_limits<double>::infinity();

    std::fill(node_tx_pkts_.begin(), node_tx_pkts_.end(), 0);
    std::fill(node_rx_pkts_.begin(), node_rx_pkts_.end(), 0);
    std::fill(node_rx_bytes_.begin(), node_rx_bytes_.end(), 0);
    std::fill(node_lost_pkts_.begin(), node_lost_pkts_.end(), 0);
    std::fill(node_dup_pkts_.begin(), node_dup_pkts_.end(), 0);
    std::fill(node_delay_sum_us_.begin(), node_delay_sum_us_.end(), 0);
    std::fill(node_delay_max_us_.begin(), node_delay_max_us_.end(), 0);
    std::fill(node_jitter_sum_us_.begin(), node_jitter_sum_us_.end(), 0);
    std::fill(node_jitter_samples_.begin(), node_jitter_samples_.end(), 0);

    std::fill(node_rx_power_sum_dbm_.begin(), node_rx_power_sum_dbm_.end(), 0.0);
    std::fill(node_rx_power_samples_.begin(), node_rx_power_samples_.end(), 0);
    std::fill(node_rx_power_min_dbm_.begin(), node_rx_power_min_dbm_.end(), std::numeric_limits<double>::infinity());
    std::fill(node_rx_power_max_dbm_.begin(), node_rx_power_max_dbm_.end(), -std::numeric_limits<double>::infinity());

    rx_events_window_.clear();
    delivered_swarm_packets_window_.clear();
}

bool BasicWifiAdhoc::hasFreshRealAgentStates_() const
{
    if (!prefer_real_agent_states_ || last_real_agent_states_update_.isZero())
        return false;

    return (ros::Time::now() - last_real_agent_states_update_).toSec() <=
           real_agent_states_timeout_sec_;
}

void BasicWifiAdhoc::FinalizeAndLogWindow_(uint64_t t_sim_us, uint64_t win_dur_us)
{
    double t_sim_sec = double(t_sim_us) / 1e6;

    if (metrics_net_comm_total_)
    {
        uint64_t expected_pkts_total = rx_pkts_total_ + lost_pkts_total_;
        double pdr = (expected_pkts_total > 0) ? (double(rx_pkts_total_) / double(expected_pkts_total)) : 1.0;

        uint64_t delay_mean_us = (rx_pkts_total_ > 0) ? (delay_sum_us_ / rx_pkts_total_) : 0;
        uint64_t jitter_mean_us = (jitter_samples_ > 0) ? (jitter_sum_us_ / jitter_samples_) : 0;

        double throughput_bps = (double(rx_bytes_total_) * 8.0) * (1e6 / double(win_dur_us));

        double rx_power_mean_dbm = (rx_power_samples_ > 0) ? (rx_power_sum_dbm_ / double(rx_power_samples_)) : 0.0;
        double rx_power_min = (std::isfinite(rx_power_min_dbm_) ? rx_power_min_dbm_ : 0.0);
        double rx_power_max = (std::isfinite(rx_power_max_dbm_) ? rx_power_max_dbm_ : 0.0);

        metrics_net_comm_total_->log_row(
            dancers::metrics::now_us(),
            t_sim_us,
            t_sim_sec,
            window_idx_,

            expected_pkts_total,
            rx_pkts_total_,
            lost_pkts_total_,
            pdr,

            delay_mean_us,
            delay_max_us_,
            jitter_mean_us,

            rx_bytes_total_,
            throughput_bps,

            tx_pkts_total_,
            dup_pkts_total_,

            rx_power_mean_dbm,
            rx_power_min,
            rx_power_max
        );

        if (verbose_)
        {
            RCLCPP_INFO(this->get_logger(),
                "[NET-TOTAL] t=%.6fs win=%lu exp=%lu rx=%lu lost=%lu pdr=%.3f "
                "delay_mean=%luus delay_max=%luus jitter_mean=%luus bytes=%lu thr=%.2fbps "
                "rxdBm(mean/min/max)=%.2f/%.2f/%.2f",
                t_sim_sec,
                (unsigned long)window_idx_,
                (unsigned long)expected_pkts_total,
                (unsigned long)rx_pkts_total_,
                (unsigned long)lost_pkts_total_,
                pdr,
                (unsigned long)delay_mean_us,
                (unsigned long)delay_max_us_,
                (unsigned long)jitter_mean_us,
                (unsigned long)rx_bytes_total_,
                throughput_bps,
                rx_power_mean_dbm, rx_power_min, rx_power_max
            );
        }
    }

    if (!metrics_net_comm_nodes_) return;

    std::vector<uint32_t> agent_ids;
    {
        std::shared_lock lock(this->agents_mutex_);
        agent_ids.reserve(this->agents_.size());
        for (const auto& [id, _] : this->agents_) agent_ids.push_back(id);
    }
    std::sort(agent_ids.begin(), agent_ids.end());

    for (uint32_t node_id : agent_ids)
    {
        EnsurePerNodeSize_(node_id);

        uint64_t rx_pkts = node_rx_pkts_[node_id];
        uint64_t lost_pkts = node_lost_pkts_[node_id];
        uint64_t expected_pkts = rx_pkts + lost_pkts;

        double pdr = (expected_pkts > 0) ? (double(rx_pkts) / double(expected_pkts)) : 1.0;

        uint64_t delay_mean_us = (rx_pkts > 0) ? (node_delay_sum_us_[node_id] / rx_pkts) : 0;
        uint64_t delay_max_us  = node_delay_max_us_[node_id];

        uint64_t js = node_jitter_samples_[node_id];
        uint64_t jitter_mean_us = (js > 0) ? (node_jitter_sum_us_[node_id] / js) : 0;

        uint64_t rx_bytes = node_rx_bytes_[node_id];
        double throughput_bps = (double(rx_bytes) * 8.0) * (1e6 / double(win_dur_us));

        uint64_t tx_pkts = node_tx_pkts_[node_id];
        uint64_t dup_pkts = node_dup_pkts_[node_id];

        double rxp_mean = (node_rx_power_samples_[node_id] > 0)
            ? (node_rx_power_sum_dbm_[node_id] / double(node_rx_power_samples_[node_id]))
            : 0.0;
        double rxp_min = std::isfinite(node_rx_power_min_dbm_[node_id]) ? node_rx_power_min_dbm_[node_id] : 0.0;
        double rxp_max = std::isfinite(node_rx_power_max_dbm_[node_id]) ? node_rx_power_max_dbm_[node_id] : 0.0;

        metrics_net_comm_nodes_->log_row(
            dancers::metrics::now_us(),
            t_sim_us,
            t_sim_sec,
            window_idx_,
            node_id,

            expected_pkts,
            rx_pkts,
            lost_pkts,
            pdr,

            delay_mean_us,
            delay_max_us,
            jitter_mean_us,

            rx_bytes,
            throughput_bps,

            tx_pkts,
            dup_pkts,

            rxp_mean,
            rxp_min,
            rxp_max
        );
    }

    // ---- Flush per-packet RX events for this window ----
    if (metrics_net_rx_events_ && !rx_events_window_.empty())
    {
        for (const auto& e : rx_events_window_)
        {
            double rx_time_sec = double(e.rx_time_us) / 1e6;
            double rxp = (std::isfinite(e.rx_power_dbm) ? e.rx_power_dbm : 0.0);

            metrics_net_rx_events_->log_row(
                dancers::metrics::now_us(),
                e.rx_time_us,
                rx_time_sec,
                window_idx_,

                e.rx_id,
                e.tx_id,
                e.hdr_sender_id,
                e.flow_id,
                e.seq,

                e.tx_time_us,
                e.delay_us,
                e.pkt_bytes,

                rxp,
                (e.has_rx_power ? 1 : 0)
            );
        }
    }
}

void BasicWifiAdhoc::InitMetricsCsv_()
{
    if (!enable_metrics_) return;

    std::string ros_ws_path = (getenv("ROS_WS") ? getenv("ROS_WS") : ".");
    std::string metrics_dir = (config_["metrics_dir"] ? config_["metrics_dir"].as<std::string>() : "logs");
    std::string exp_name = (config_["experiment_name"] ? config_["experiment_name"].as<std::string>() : "exp");

    dancers_fs::path out_dir = dancers_fs::path(ros_ws_path) / metrics_dir / exp_name;
    dancers_fs::create_directories(out_dir);

    {
        dancers_fs::path out_csv = out_dir / "net_comm_metrics.csv";

        metrics_net_comm_total_ = std::make_unique<dancers::metrics::CsvMetricsLogger>(
            out_csv.string(),
            std::vector<std::string>{
                "wall_ts_us",
                "t_sim_us",
                "t_sim_sec",
                "window_idx",

                "expected_pkts_total",
                "rx_pkts_total",
                "lost_pkts_total",
                "pdr_total",

                "delay_mean_us_total",
                "delay_max_us_total",
                "jitter_mean_us_total",

                "rx_bytes_total",
                "throughput_bps_total",

                "tx_pkts_total",
                "dup_pkts_total",

                "rx_power_dbm_mean_total",
                "rx_power_dbm_min_total",
                "rx_power_dbm_max_total"
            },
            metrics_flush_every_n_
        );

        RCLCPP_INFO(this->get_logger(), "NET total metrics CSV enabled: %s", out_csv.string().c_str());
    }

    {
        dancers_fs::path out_csv = out_dir / "net_comm_metrics_nodes.csv";

        metrics_net_comm_nodes_ = std::make_unique<dancers::metrics::CsvMetricsLogger>(
            out_csv.string(),
            std::vector<std::string>{
                "wall_ts_us",
                "t_sim_us",
                "t_sim_sec",
                "window_idx",
                "node_id",

                "expected_pkts",
                "rx_pkts",
                "lost_pkts",
                "pdr",

                "delay_mean_us",
                "delay_max_us",
                "jitter_mean_us",

                "rx_bytes",
                "throughput_bps",

                "tx_pkts",
                "dup_pkts",

                "rx_power_dbm_mean",
                "rx_power_dbm_min",
                "rx_power_dbm_max"
            },
            metrics_flush_every_n_
        );

        RCLCPP_INFO(this->get_logger(), "NET per-node metrics CSV enabled: %s", out_csv.string().c_str());
    }

    {
        dancers_fs::path out_csv = out_dir / "net_rx_events.csv";

        metrics_net_rx_events_ = std::make_unique<dancers::metrics::CsvMetricsLogger>(
            out_csv.string(),
            std::vector<std::string>{
                "wall_ts_us",
                "rx_time_us",
                "rx_time_sec",
                "window_idx",

                "rx_id",
                "tx_id",
                "hdr_sender_id",
                "flow_id",
                "seq",

                "tx_time_us",
                "delay_us",
                "pkt_bytes",

                "rx_power_dbm",
                "has_rx_power"
            },
            metrics_flush_every_n_
        );

        RCLCPP_INFO(this->get_logger(), "NET per-packet RX events CSV enabled: %s", out_csv.string().c_str());
    }
}

// ---- Controller service ----
void BasicWifiAdhoc::RequestCommandsClbk()
{
    if (command_request_in_flight_.load())
    {
        return;
    }

    while (!command_client_.exists() &&
           !command_client_.waitForExistence(ros::Duration(1.0)))
    {
        if (!ros::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_WARN_STREAM(this->get_logger(), "Service " << command_client_.getService() << " not available, waiting again...");
    }

    std::vector<dancers_msgs::AgentStruct> request_agents;

    {
        std::shared_lock lock(this->agents_mutex_);
        request_agents.reserve(this->agents_.size());
        for (const auto& [agent_id, agent_ptr] : this->agents_)
        {
            if (!agent_ptr->crashed)
            {
                dancers_msgs::AgentStruct agent_struct = RosMsgFromAgent(*agent_ptr);
                request_agents.push_back(std::move(agent_struct));
            }
        }
    }

    if (command_request_in_flight_.exchange(true))
    {
        return;
    }

    std::string latest_comm_rx_events_raw;
    {
        std::lock_guard<std::mutex> lk(latest_comm_rx_events_mu_);
        latest_comm_rx_events_raw = latest_comm_rx_events_raw_;
    }

    dancers_msgs::GetAgentVelocities srv;
    srv.request.agent_structs = std::move(request_agents);
    srv.request.comm_rx_events_bytes.assign(
        latest_comm_rx_events_raw.begin(), latest_comm_rx_events_raw.end());

    try
    {
        if (!command_client_.call(srv))
        {
            RCLCPP_WARN(this->get_logger(), "Failed to call service %s", command_client_.getService().c_str());
            command_request_in_flight_.store(false);
            return;
        }

        {
            std::unique_lock lock(this->agents_mutex_);
            for (const auto& velocity_heading : srv.response.velocity_headings.velocity_heading_array)
            {
                auto agent_it = this->agents_.find(velocity_heading.agent_id);
                if (agent_it != this->agents_.end())
                {
                    agent_it->second->cmd_velocity = Eigen::Vector3d(velocity_heading.velocity.x,
                                                                     velocity_heading.velocity.y,
                                                                     velocity_heading.velocity.z);
                    agent_it->second->cmd_heading = velocity_heading.heading;
                }
            }
        }
    }
    catch (const std::exception& ex)
    {
        RCLCPP_ERROR(this->get_logger(), "Exception while calling service %s: %s",
                     command_client_.getService().c_str(), ex.what());
    }
    catch (...)
    {
        RCLCPP_ERROR(this->get_logger(), "Unknown exception while calling service %s",
                     command_client_.getService().c_str());
    }

    command_request_in_flight_.store(false);
}

// ---- Neighbors timeout ----
void BasicWifiAdhoc::timeoutNeighbors()
{
    std::unique_lock lock(this->agents_mutex_);
    for (auto &[agent_id, agent_ptr] : this->agents_)
    {
        std::vector<uint32_t> NeighIDsToRemove;
        for (auto const &neighbor : agent_ptr->neighbors)
        {
            if ((Simulator::Now() - MicroSeconds(neighbor.second.last_seen)) >
                MicroSeconds(getYamlValue<uint32_t>(this->config_["flocking_flow"], "timeout")))
            {
                NeighIDsToRemove.push_back(neighbor.first);
                RCLCPP_INFO(this->get_logger(), "Agent %d not a neighbor of %d anymore", neighbor.first, agent_id);
            }
        }
        for (auto const &neighbor : NeighIDsToRemove)
        {
            agent_ptr->neighbors.erase(neighbor);
        }
    }
}

// ---- RViz ----
void BasicWifiAdhoc::DisplayRviz()
{
    if (!getYamlValue<bool>(this->config_, "display_network_edges") || !neighborhood_links_pub_)
        return;

    visualization_msgs::Marker network_marker{};
    network_marker.header.frame_id = "map";
    network_marker.id = 1011;
    network_marker.type = visualization_msgs::Marker::LINE_LIST;
    network_marker.action = visualization_msgs::Marker::MODIFY;
    network_marker.scale.x = 0.1;
    network_marker.color.r = 0.0;
    network_marker.color.g = 0.0;
    network_marker.color.b = 1.0;
    network_marker.color.a = 1.0;

    std::shared_lock lock(this->agents_mutex_);
    for (auto& [agent_id, agent_ptr] : this->agents_)
    {
        Eigen::Vector3d agent_pose(agent_ptr->position);
        for (auto& [neighbor_id, neighbor_struct] : agent_ptr->neighbors)
        {
            if (agent_id == neighbor_id) continue;

            Eigen::Vector3d other_agent_pose(this->agents_.at(neighbor_id)->position);

            geometry_msgs::Point p1{};
            p1.x = agent_pose.x();
            p1.y = agent_pose.y();
            p1.z = agent_pose.z();
            geometry_msgs::Point p2{};
            p2.x = other_agent_pose.x();
            p2.y = other_agent_pose.y();
            p2.z = other_agent_pose.z();

            network_marker.points.push_back(p1);
            network_marker.points.push_back(p2);
        }
    }

    this->neighborhood_links_pub_.publish(network_marker);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "basic_wifi_adhoc_with_controller");
    try
    {
        BasicWifiAdhoc node;
        ros::AsyncSpinner spinner(1);
        spinner.start();
        node.RunLoop();
        return 0;
    }
    catch (const std::exception& ex)
    {
        ROS_FATAL("basic_wifi_adhoc_with_controller terminated with exception: %s", ex.what());
    }
    catch (...)
    {
        ROS_FATAL("basic_wifi_adhoc_with_controller terminated with unknown non-std exception.");
    }
    return 1;
}
