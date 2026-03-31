#include <connector_core/connector.hpp>

#include <dancers_msgs/AgentStructArray.h>
#include <protobuf_msgs/agent_state_batch.pb.h>
#include <protobuf_msgs/net_rx_events.pb.h>
#include <protobuf_msgs/pose_vector.pb.h>
#include <protobuf_msgs/racer_swarm_msg.pb.h>
#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include <yaml_util.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace relay_racer_integration {

class RacerRealPhyBridge : public Connector {
 public:
  RacerRealPhyBridge()
      : Connector("racer_real_phy_bridge") {
    ReadConfigFile();
    pnh_.param<std::string>("agent_states_topic", agent_states_topic_,
                            std::string("/relay_integration/platform_agent_structs"));
    pnh_.param("subscribe_agent_states_topic", subscribe_agent_states_topic_, false);
    pnh_.param<std::string>("swarm_rx_topic", swarm_rx_topic_,
                            std::string("/relay_integration/racer_swarm_rx_bytes"));
    pnh_.param("swarm_rx_queue_size", swarm_rx_queue_size_, 200);
    pnh_.param("real_states_stale_warn_sec", real_states_stale_warn_sec_, 1.0);
    pnh_.param("trace_swarm_msg_enable", trace_swarm_msg_enable_, false);
    pnh_.param<std::string>("trace_swarm_msg_family", trace_swarm_msg_family_, std::string());
    pnh_.param("trace_swarm_msg_src_id", trace_swarm_msg_src_id_, -1);
    pnh_.param("trace_swarm_msg_bridge_seq", trace_swarm_msg_bridge_seq_, -1);
    pnh_.param("trace_agent_state_batch_enable", trace_agent_state_batch_enable_, false);
    pnh_.param("trace_agent_state_batch_every_n", trace_agent_state_batch_every_n_, 1);
    if (trace_agent_state_batch_every_n_ <= 0) {
      trace_agent_state_batch_every_n_ = 1;
    }

    InitFallbackStates();

    if (subscribe_agent_states_topic_) {
      agent_states_sub_ = nh_.subscribe(agent_states_topic_, 10,
                                        &RacerRealPhyBridge::agentStatesCallback, this);
    }
    swarm_rx_pub_ = nh_.advertise<std_msgs::UInt8MultiArray>(
        swarm_rx_topic_, std::max(1, swarm_rx_queue_size_));

    ROS_INFO("[PHY_BRIDGE] agent_state_source=%s topic=%s swarm_rx_topic=%s robots=%u phy_step=%.6fs "
             "processing_delay_us=%llu comm_ttl_us=%llu",
             subscribe_agent_states_topic_ ? "direct_ros_topic" : "coordinator_agent_states_gz",
             agent_states_topic_.c_str(), swarm_rx_topic_.c_str(), robots_number_, step_size,
             static_cast<unsigned long long>(comm_processing_delay_us_),
             static_cast<unsigned long long>(comm_ttl_us_));
  }

  void Run() { Loop(); }

 private:
  struct AgentSnapshot {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double vx{0.0};
    double vy{0.0};
    double vz{0.0};
    double heading{0.0};
    ros::Time stamp;
    bool has_real_state{false};
  };

  struct PendingDeliveredPacket {
    uint64_t deliver_us{0};
    uint64_t rx_time_us{0};
    uint32_t dst_id{0};
    uint32_t src_id{0};
    uint32_t flow_id{0};
    uint32_t seq{0};
    uint64_t bridge_seq{0};
    uint32_t network_tx_id{0};
    uint32_t relay_hop_count{0};
    uint32_t max_relay_hops{0};
    std::string family;
    bool has_wrapper{false};
    std::string wrapped_msg;
  };

  void ReadConfigFile() override {
    use_uds = getYamlValue<bool>(config_, "phy_use_uds");
    uds_server_address = getYamlValue<std::string>(config_, "phy_uds_server_address");
    ip_server_address = getYamlValue<std::string>(config_, "phy_ip_server_address");
    ip_server_port = getYamlValue<unsigned int>(config_, "phy_ip_server_port");

    const unsigned int sync_window_int = config_["sync_window"].as<unsigned int>();
    const unsigned int step_size_int = config_["phy_step_size"].as<unsigned int>();
    if (sync_window_int % step_size_int != 0) {
      ROS_FATAL("Sync window must be a multiple of the physics step size.");
      throw std::runtime_error("invalid sync_window/phy_step_size");
    }

    phy_step_us_ = getYamlValue<uint64_t>(config_, "phy_step_size");
    if (phy_step_us_ == 0) {
      ROS_FATAL("phy_step_size cannot be 0");
      throw std::runtime_error("phy_step_size cannot be 0");
    }
    comm_processing_delay_us_ =
        (config_["comm_processing_delay_us"] ? config_["comm_processing_delay_us"].as<uint64_t>()
                                             : 0ull);
    comm_ttl_us_ =
        (config_["comm_ttl_us"] ? config_["comm_ttl_us"].as<uint64_t>() : 5'000'000ull);

    step_size = static_cast<double>(phy_step_us_) / 1e6;
    it_end_sim = uint64_t(simulation_length / step_size);
  }

  void InitFallbackStates() {
    robots_number_ = getYamlValue<unsigned int>(config_, "robots_number");
    const YAML::Node initial_positions = config_["initial_positions"];

    if (initial_positions && initial_positions.IsSequence() &&
        initial_positions.size() >= robots_number_) {
      for (unsigned int i = 0; i < robots_number_; ++i) {
        const YAML::Node pose = initial_positions[i];
        AgentSnapshot state;
        state.x = getYamlValue<double>(pose, "x");
        state.y = getYamlValue<double>(pose, "y");
        state.z = getYamlValue<double>(pose, "z");
        state.heading = pose["heading"] ? pose["heading"].as<double>() : 0.0;
        snapshots_[i] = state;
      }
      return;
    }

    const int n_columns = std::max(1, static_cast<int>(std::sqrt(robots_number_)));
    const double spacing = getYamlValue<double>(config_, "initial_spacing");
    const double initial_x = getYamlValue<double>(config_, "initial_x");
    const double initial_y = getYamlValue<double>(config_, "initial_y");
    const double initial_z = getYamlValue<double>(config_, "initial_z");
    const double initial_heading = getYamlValue<double>(config_, "initial_heading");

    const int n_rows = static_cast<int>(std::ceil(static_cast<double>(robots_number_) / n_columns));
    const double x0 = initial_x - spacing * (n_columns - 1) * 0.5;
    const double y0 = initial_y - spacing * (n_rows - 1) * 0.5;

    for (unsigned int i = 0; i < robots_number_; ++i) {
      const int row = static_cast<int>(i) / n_columns;
      const int col = static_cast<int>(i) % n_columns;
      AgentSnapshot state;
      state.x = x0 + spacing * col;
      state.y = y0 + spacing * row;
      state.z = initial_z;
      state.heading = initial_heading;
      snapshots_[i] = state;
    }
  }

  void agentStatesCallback(const dancers_msgs::AgentStructArrayConstPtr& msg) {
    if (msg->structs.empty()) {
      return;
    }

    std::lock_guard<std::mutex> lock(snapshots_mutex_);
    const ros::Time stamp = ros::Time::now();
    for (const auto& agent_struct : msg->structs) {
      AgentSnapshot& state = snapshots_[agent_struct.agent_id];
      state.x = agent_struct.state.position.x;
      state.y = agent_struct.state.position.y;
      state.z = agent_struct.state.position.z;
      state.vx = agent_struct.state.velocity.x;
      state.vy = agent_struct.state.velocity.y;
      state.vz = agent_struct.state.velocity.z;
      state.heading = agent_struct.state.heading;
      state.stamp = stamp;
      state.has_real_state = true;
    }
    last_real_states_update_ = stamp;
  }

  void UpdateSnapshotsFromAgentStateBatch_(const std::string& compressed_batch) {
    if (compressed_batch.empty()) {
      return;
    }

    std::string raw_batch;
    try {
      raw_batch = gzip_decompress(compressed_batch);
    } catch (const std::exception& e) {
      ROS_ERROR("[PHY_BRIDGE] failed to decompress coordinator agent state batch: %s", e.what());
      throw;
    }

    protobuf_msgs::AgentStateBatch batch;
    if (!batch.ParseFromString(raw_batch)) {
      throw std::runtime_error("[PHY_BRIDGE] failed to parse coordinator agent state batch.");
    }

    size_t applied_states = 0;
    std::lock_guard<std::mutex> lock(snapshots_mutex_);
    for (const auto& sample : batch.states()) {
      AgentSnapshot& state = snapshots_[sample.agent_id()];
      state.x = sample.x();
      state.y = sample.y();
      state.z = sample.z();
      state.vx = sample.vx();
      state.vy = sample.vy();
      state.vz = sample.vz();
      state.heading = sample.heading();
      if (sample.source_stamp_us() > 0) {
        state.stamp.fromNSec(sample.source_stamp_us() * 1000ull);
      } else {
        state.stamp = ros::Time::now();
      }
      state.has_real_state = sample.has_real_state();
      ++applied_states;
    }
    last_real_states_update_ = ros::Time::now();
    if (trace_agent_state_batch_enable_ &&
        (batch.sample_seq() % static_cast<uint64_t>(trace_agent_state_batch_every_n_)) == 0) {
      ROS_INFO("[AGENT_STATE_TRACE][PHY] sample_seq=%lu t_sample_us=%lu batch_states=%d applied_states=%zu",
               static_cast<unsigned long>(batch.sample_seq()),
               static_cast<unsigned long>(batch.t_sample_us()),
               batch.states_size(),
               applied_states);
    }
  }

  static bool DeliveryOrder(const PendingDeliveredPacket& lhs,
                            const PendingDeliveredPacket& rhs) {
    if (lhs.deliver_us != rhs.deliver_us) {
      return lhs.deliver_us < rhs.deliver_us;
    }
    if (lhs.rx_time_us != rhs.rx_time_us) {
      return lhs.rx_time_us < rhs.rx_time_us;
    }
    if (lhs.dst_id != rhs.dst_id) {
      return lhs.dst_id < rhs.dst_id;
    }
    if (lhs.src_id != rhs.src_id) {
      return lhs.src_id < rhs.src_id;
    }
    if (lhs.flow_id != rhs.flow_id) {
      return lhs.flow_id < rhs.flow_id;
    }
    return lhs.seq < rhs.seq;
  }

  static uint64_t SaturatedAdd(const uint64_t lhs, const uint64_t rhs) {
    if (lhs > std::numeric_limits<uint64_t>::max() - rhs) {
      return std::numeric_limits<uint64_t>::max();
    }
    return lhs + rhs;
  }

  uint64_t CurrentStepEndSimUs() const {
    return phy_step_us_ * (it + 1);
  }

  void IngestDeliveredSwarmPackets_(const std::string& compressed_batch) {
    if (compressed_batch.empty()) {
      return;
    }

    std::string raw_batch;
    try {
      raw_batch = gzip_decompress(compressed_batch);
    } catch (const std::exception& e) {
      ROS_WARN_THROTTLE(1.0, "[PHY_BRIDGE] failed to decompress delivered_swarm_packets_gz: %s",
                        e.what());
      return;
    }

    protobuf_msgs::DeliveredSwarmPacketBatch batch;
    if (!batch.ParseFromString(raw_batch)) {
      ROS_WARN_THROTTLE(1.0,
                        "[PHY_BRIDGE] failed to parse DeliveredSwarmPacketBatch raw_bytes=%zu",
                        raw_batch.size());
      return;
    }

    for (const auto& packet : batch.packets()) {
      if (packet.wrapped_msg().empty()) {
        continue;
      }

      PendingDeliveredPacket pending;
      pending.rx_time_us = static_cast<uint64_t>(packet.meta().rx_time_us());
      pending.deliver_us = SaturatedAdd(pending.rx_time_us, comm_processing_delay_us_);
      pending.dst_id = packet.meta().dst_id();
      pending.src_id = packet.meta().src_id();
      pending.flow_id = packet.meta().flow_id();
      pending.seq = packet.meta().seq();
      pending.wrapped_msg = packet.wrapped_msg();

      relay_racer_proto::RacerSwarmMsg wrapper;
      if (wrapper.ParseFromString(pending.wrapped_msg)) {
        pending.family = wrapper.family();
        pending.bridge_seq = wrapper.bridge_seq();
        pending.network_tx_id = wrapper.network_tx_id();
        pending.relay_hop_count = wrapper.relay_hop_count();
        pending.max_relay_hops = wrapper.max_relay_hops();
        pending.has_wrapper = true;
        if (ShouldTracePending_(pending)) {
          TracePending_(
              "phy_ingest_queue",
              pending,
              "meta_src_id=" + std::to_string(packet.meta().src_id()) +
                  " meta_dst_id=" + std::to_string(packet.meta().dst_id()) +
                  " rx_time_us=" + std::to_string(packet.meta().rx_time_us()) +
                  " delay_us=" + std::to_string(packet.meta().delay_us()) +
                  " deliver_us=" + std::to_string(pending.deliver_us));
        }
      }

      pending_delivered_packets_.push_back(std::move(pending));
    }

    if (!batch.packets().empty()) {
      pending_deliveries_dirty_ = true;
      ROS_INFO_STREAM_THROTTLE(
          1.0, "[PHY_BRIDGE] queued delayed packets total=" << pending_delivered_packets_.size()
               << " batch=" << batch.packets_size()
               << " processing_delay_us=" << comm_processing_delay_us_);
    }
  }

  void PublishDeliveredPacket_(const PendingDeliveredPacket& packet) {
    std_msgs::UInt8MultiArray msg;
    msg.data.assign(packet.wrapped_msg.begin(), packet.wrapped_msg.end());
    if (ShouldTracePending_(packet)) {
      TracePending_("phy_publish_rx_bytes", packet,
                    "deliver_us=" + std::to_string(packet.deliver_us) +
                        " rx_time_us=" + std::to_string(packet.rx_time_us));
    }
    swarm_rx_pub_.publish(msg);
    ++published_delivered_packets_;
  }

  void ReleaseDeliveredPacketsUpTo_(const uint64_t step_end_us) {
    if (pending_delivered_packets_.empty()) {
      return;
    }

    if (pending_deliveries_dirty_) {
      std::sort(pending_delivered_packets_.begin(), pending_delivered_packets_.end(), DeliveryOrder);
      pending_deliveries_dirty_ = false;
    }

    std::size_t release_count = 0;
    std::size_t dropped_count = 0;
    while (release_count < pending_delivered_packets_.size() &&
           pending_delivered_packets_[release_count].deliver_us <= step_end_us) {
      const PendingDeliveredPacket& packet = pending_delivered_packets_[release_count];
      if (comm_ttl_us_ > 0 && step_end_us > packet.rx_time_us &&
          (step_end_us - packet.rx_time_us) > comm_ttl_us_) {
        if (ShouldTracePending_(packet)) {
          TracePending_("phy_drop_ttl", packet,
                        "step_end_us=" + std::to_string(step_end_us) +
                            " rx_time_us=" + std::to_string(packet.rx_time_us) +
                            " ttl_us=" + std::to_string(comm_ttl_us_));
        }
        ++dropped_delivered_packets_;
        ++dropped_count;
      } else {
        PublishDeliveredPacket_(packet);
      }
      ++release_count;
    }

    if (release_count > 0) {
      ROS_INFO_STREAM_THROTTLE(
          1.0, "[PHY_BRIDGE] released delayed packets count=" << release_count
               << " dropped_ttl=" << dropped_count
               << " remaining=" << (pending_delivered_packets_.size() - release_count)
               << " step_end_us=" << step_end_us
               << " total_published=" << published_delivered_packets_
               << " total_dropped_ttl=" << dropped_delivered_packets_);
      pending_delivered_packets_.erase(
          pending_delivered_packets_.begin(),
          pending_delivered_packets_.begin() + static_cast<std::ptrdiff_t>(release_count));
    }
  }

  dancers_update_proto::DancersUpdate StepSimulation(
      dancers_update_proto::DancersUpdate update_msg) override {
    if (!update_msg.agent_states_gz().empty()) {
      UpdateSnapshotsFromAgentStateBatch_(update_msg.agent_states_gz());
    }
    if (!update_msg.payload().empty()) {
      ROS_INFO_THROTTLE(
          5.0,
          "[PHY_BRIDGE] ignoring NET->PHY cmd payload (%zuB); physical truth is driven by coordinator agent state batches.",
          static_cast<size_t>(update_msg.payload().size()));
    }
    if (!update_msg.net_rx_events_gz().empty()) {
      ROS_INFO_THROTTLE(
          5.0,
          "[PHY_BRIDGE] received legacy net_rx_events_gz=%zuB for diagnostics.",
          static_cast<size_t>(update_msg.net_rx_events_gz().size()));
    }
    if (!update_msg.delivered_swarm_packets_gz().empty()) {
      IngestDeliveredSwarmPackets_(update_msg.delivered_swarm_packets_gz());
    }
    ReleaseDeliveredPacketsUpTo_(CurrentStepEndSimUs());
    if (subscribe_agent_states_topic_ && !last_real_states_update_.isZero() &&
        (ros::Time::now() - last_real_states_update_).toSec() > real_states_stale_warn_sec_) {
      ROS_WARN_THROTTLE(
          2.0,
          "[PHY_BRIDGE] direct ROS agent states are stale for %.3fs; returning last known/fallback positions.",
          (ros::Time::now() - last_real_states_update_).toSec());
    }
    return GenerateResponseProtobuf();
  }

  dancers_update_proto::DancersUpdate GenerateResponseProtobuf() const {
    dancers_update_proto::DancersUpdate response_msg;
    response_msg.set_msg_type(dancers_update_proto::DancersUpdate::END);

    dancers_update_proto::PoseVector robots_positions_msg;
    std::lock_guard<std::mutex> lock(snapshots_mutex_);
    for (const auto& kv : snapshots_) {
      const uint32_t agent_id = kv.first;
      const AgentSnapshot& state = kv.second;

      auto* robot_pose_msg = robots_positions_msg.add_pose();
      robot_pose_msg->set_agent_id(agent_id);
      robot_pose_msg->set_x(state.x);
      robot_pose_msg->set_y(state.y);
      robot_pose_msg->set_z(state.z);
      robot_pose_msg->set_vx(state.vx);
      robot_pose_msg->set_vy(state.vy);
      robot_pose_msg->set_vz(state.vz);

      const double half_heading = 0.5 * state.heading;
      robot_pose_msg->set_qx(0.0);
      robot_pose_msg->set_qy(0.0);
      robot_pose_msg->set_qz(std::sin(half_heading));
      robot_pose_msg->set_qw(std::cos(half_heading));
    }

    std::string serialized_msg;
    robots_positions_msg.SerializeToString(&serialized_msg);
    response_msg.set_payload(gzip_compress(serialized_msg));
    return response_msg;
  }

  bool ShouldTracePending_(const PendingDeliveredPacket& packet) const {
    if (!trace_swarm_msg_enable_ || !packet.has_wrapper) {
      return false;
    }
    if (!trace_swarm_msg_family_.empty() && packet.family != trace_swarm_msg_family_) {
      return false;
    }
    if (trace_swarm_msg_src_id_ > 0 && static_cast<int>(packet.src_id) != trace_swarm_msg_src_id_) {
      return false;
    }
    if (trace_swarm_msg_bridge_seq_ >= 0 &&
        packet.bridge_seq != static_cast<uint64_t>(trace_swarm_msg_bridge_seq_)) {
      return false;
    }
    return true;
  }

  void TracePending_(const char* stage,
                     const PendingDeliveredPacket& packet,
                     const std::string& extra = std::string()) const {
    if (extra.empty()) {
      ROS_INFO_STREAM("[SWARM_TRACE][" << stage << "] family=" << packet.family
                      << " src_id=" << packet.src_id
                      << " dst_id=" << packet.dst_id
                      << " bridge_seq=" << packet.bridge_seq
                      << " network_tx_id=" << packet.network_tx_id
                      << " relay_hop_count=" << packet.relay_hop_count
                      << " max_relay_hops=" << packet.max_relay_hops
                      << " flow_id=" << packet.flow_id
                      << " seq=" << packet.seq);
      return;
    }

    ROS_INFO_STREAM("[SWARM_TRACE][" << stage << "] family=" << packet.family
                    << " src_id=" << packet.src_id
                    << " dst_id=" << packet.dst_id
                    << " bridge_seq=" << packet.bridge_seq
                    << " network_tx_id=" << packet.network_tx_id
                    << " relay_hop_count=" << packet.relay_hop_count
                    << " max_relay_hops=" << packet.max_relay_hops
                    << " flow_id=" << packet.flow_id
                    << " seq=" << packet.seq
                    << " " << extra);
  }

  ros::Subscriber agent_states_sub_;
  bool subscribe_agent_states_topic_{false};
  ros::Publisher swarm_rx_pub_;
  std::string agent_states_topic_;
  std::string swarm_rx_topic_;
  int swarm_rx_queue_size_{200};
  double real_states_stale_warn_sec_{1.0};
  unsigned int robots_number_{0};
  uint64_t phy_step_us_{0};
  uint64_t comm_processing_delay_us_{0};
  uint64_t comm_ttl_us_{5'000'000};
  bool trace_swarm_msg_enable_{false};
  std::string trace_swarm_msg_family_;
  int trace_swarm_msg_src_id_{-1};
  int trace_swarm_msg_bridge_seq_{-1};
  bool trace_agent_state_batch_enable_{false};
  int trace_agent_state_batch_every_n_{1};
  ros::Time last_real_states_update_;

  mutable std::mutex snapshots_mutex_;
  std::map<uint32_t, AgentSnapshot> snapshots_;
  std::vector<PendingDeliveredPacket> pending_delivered_packets_;
  bool pending_deliveries_dirty_{false};
  uint64_t published_delivered_packets_{0};
  uint64_t dropped_delivered_packets_{0};
};

}  // namespace relay_racer_integration

int main(int argc, char** argv) {
  ros::init(argc, argv, "racer_real_phy_bridge_node");
  relay_racer_integration::RacerRealPhyBridge node;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  node.Run();
  return 0;
}
