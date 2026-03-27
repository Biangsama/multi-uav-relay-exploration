#include <connector_core/connector.hpp>

#include <dancers_msgs/AgentStructArray.h>
#include <protobuf_msgs/pose_vector.pb.h>
#include <ros/ros.h>
#include <yaml_util.hpp>

#include <cmath>
#include <algorithm>
#include <map>
#include <mutex>
#include <stdexcept>
#include <string>

namespace relay_racer_integration {

class RacerRealPhyBridge : public Connector {
 public:
  RacerRealPhyBridge()
      : Connector("racer_real_phy_bridge") {
    ReadConfigFile();
    pnh_.param<std::string>("agent_states_topic", agent_states_topic_,
                            std::string("/relay_integration/platform_agent_structs"));
    pnh_.param("real_states_stale_warn_sec", real_states_stale_warn_sec_, 1.0);

    InitFallbackStates();

    agent_states_sub_ = nh_.subscribe(agent_states_topic_, 10,
                                      &RacerRealPhyBridge::agentStatesCallback, this);

    ROS_INFO("[PHY_BRIDGE] topic=%s robots=%u phy_step=%.6fs", agent_states_topic_.c_str(),
             robots_number_, step_size);
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

    const uint64_t phy_step_us = getYamlValue<uint64_t>(config_, "phy_step_size");
    if (phy_step_us == 0) {
      ROS_FATAL("phy_step_size cannot be 0");
      throw std::runtime_error("phy_step_size cannot be 0");
    }

    step_size = static_cast<double>(phy_step_us) / 1e6;
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

  dancers_update_proto::DancersUpdate StepSimulation(
      dancers_update_proto::DancersUpdate update_msg) override {
    if (!update_msg.payload().empty()) {
      ROS_INFO_THROTTLE(
          5.0,
          "[PHY_BRIDGE] ignoring NET->PHY cmd payload (%zuB); physical truth is driven by RACER odom.",
          static_cast<size_t>(update_msg.payload().size()));
    }
    if (!update_msg.net_rx_events_gz().empty()) {
      ROS_INFO_THROTTLE(
          5.0,
          "[PHY_BRIDGE] received net_rx_events_gz=%zuB (kept for Stage C, not consumed in Stage B).",
          static_cast<size_t>(update_msg.net_rx_events_gz().size()));
    }
    if (!last_real_states_update_.isZero() &&
        (ros::Time::now() - last_real_states_update_).toSec() > real_states_stale_warn_sec_) {
      ROS_WARN_THROTTLE(
          2.0,
          "[PHY_BRIDGE] real agent states are stale for %.3fs; returning last known/fallback positions.",
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

  ros::Subscriber agent_states_sub_;
  std::string agent_states_topic_;
  double real_states_stale_warn_sec_{1.0};
  unsigned int robots_number_{0};
  ros::Time last_real_states_update_;

  mutable std::mutex snapshots_mutex_;
  std::map<uint32_t, AgentSnapshot> snapshots_;
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
