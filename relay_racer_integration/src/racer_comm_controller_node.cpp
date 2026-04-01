#include <dancers_msgs/AgentStruct.h>
#include <dancers_msgs/GetAgentVelocities.h>
#include <dancers_msgs/VelocityHeading.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/UInt8MultiArray.h>
#include <XmlRpcValue.h>

#include <protobuf_msgs/net_rx_events.pb.h>

#include <relay_racer_integration/RelayRoleCmd.h>
#include <relay_racer_integration/SwarmCommState.h>
#include <relay_racer_integration/id_utils.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace relay_racer_integration {
namespace {

enum class InputSource {
  kUnknown,
  kServiceChain,
  kRawTopic,
};

enum class RelayPolicyMode {
  kOff,
  kRule,
  kRl,
};

double Clamp01(const double value) {
  return std::max(0.0, std::min(1.0, value));
}

long long DirectedKey(const int src_id, const int dst_id) {
  return (static_cast<long long>(src_id) << 32) ^ static_cast<unsigned int>(dst_id);
}

int DirectedKeySrc(const long long key) {
  return static_cast<int>(key >> 32);
}

int DirectedKeyDst(const long long key) {
  return static_cast<int>(static_cast<unsigned int>(key));
}

double SquaredDistance(const geometry_msgs::Point& a, const geometry_msgs::Point& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return dx * dx + dy * dy + dz * dz;
}

struct ObstacleBox {
  double min_x;
  double max_x;
  double min_y;
  double max_y;
  double min_z;
  double max_z;
};

bool XmlRpcValueToDouble(const XmlRpc::XmlRpcValue& value, double* out) {
  if (out == nullptr) {
    return false;
  }
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    *out = static_cast<double>(value);
    return true;
  }
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    *out = static_cast<int>(value);
    return true;
  }
  return false;
}

bool ParseObstacleBoxes(const XmlRpc::XmlRpcValue& obstacles_param, std::vector<ObstacleBox>* obstacles) {
  if (obstacles == nullptr) {
    return false;
  }
  obstacles->clear();
  if (obstacles_param.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    return false;
  }

  for (int i = 0; i < obstacles_param.size(); ++i) {
    const auto& obstacle = obstacles_param[i];
    if (obstacle.getType() != XmlRpc::XmlRpcValue::TypeStruct || !obstacle.hasMember("x") ||
        !obstacle.hasMember("y") || !obstacle.hasMember("size_x") ||
        !obstacle.hasMember("size_y") || !obstacle.hasMember("size_z")) {
      continue;
    }

    double x = 0.0;
    double y = 0.0;
    double size_x = 0.0;
    double size_y = 0.0;
    double size_z = 0.0;
    if (!XmlRpcValueToDouble(obstacle["x"], &x) || !XmlRpcValueToDouble(obstacle["y"], &y) ||
        !XmlRpcValueToDouble(obstacle["size_x"], &size_x) ||
        !XmlRpcValueToDouble(obstacle["size_y"], &size_y) ||
        !XmlRpcValueToDouble(obstacle["size_z"], &size_z)) {
      continue;
    }

    ObstacleBox box;
    box.min_x = x - 0.5 * size_x;
    box.max_x = x + 0.5 * size_x;
    box.min_y = y - 0.5 * size_y;
    box.max_y = y + 0.5 * size_y;
    box.min_z = 0.0;
    box.max_z = std::max(0.0, size_z);
    obstacles->push_back(box);
  }
  return true;
}

bool IsInsideObstacle(const geometry_msgs::Point& point, const ObstacleBox& obstacle, const double clearance) {
  return point.x >= obstacle.min_x - clearance && point.x <= obstacle.max_x + clearance &&
         point.y >= obstacle.min_y - clearance && point.y <= obstacle.max_y + clearance &&
         point.z >= obstacle.min_z && point.z <= obstacle.max_z;
}

bool ProjectPointOutOfObstacle(
    const ObstacleBox& obstacle, const double clearance, geometry_msgs::Point* point) {
  if (point == nullptr || !IsInsideObstacle(*point, obstacle, clearance)) {
    return false;
  }

  const double min_x = obstacle.min_x - clearance;
  const double max_x = obstacle.max_x + clearance;
  const double min_y = obstacle.min_y - clearance;
  const double max_y = obstacle.max_y + clearance;
  const double dist_left = std::abs(point->x - min_x);
  const double dist_right = std::abs(max_x - point->x);
  const double dist_bottom = std::abs(point->y - min_y);
  const double dist_top = std::abs(max_y - point->y);

  int best_dir = 0;
  double best_dist = dist_left;
  if (dist_right < best_dist) {
    best_dist = dist_right;
    best_dir = 1;
  }
  if (dist_bottom < best_dist) {
    best_dist = dist_bottom;
    best_dir = 2;
  }
  if (dist_top < best_dist) {
    best_dir = 3;
  }

  const double eps = 1e-3;
  if (best_dir == 0) {
    point->x = min_x - eps;
  } else if (best_dir == 1) {
    point->x = max_x + eps;
  } else if (best_dir == 2) {
    point->y = min_y - eps;
  } else {
    point->y = max_y + eps;
  }
  return true;
}

const char* InputSourceName(const InputSource source) {
  switch (source) {
    case InputSource::kServiceChain:
      return "service_chain";
    case InputSource::kRawTopic:
      return "raw_topic";
    case InputSource::kUnknown:
    default:
      return "unknown";
  }
}

std::string NormalizePolicyName(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
      [](const unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  return value;
}

bool TryParseRelayPolicyMode(
    const std::string& raw_value, RelayPolicyMode* mode, std::string* normalized_value) {
  const std::string normalized = NormalizePolicyName(raw_value);
  if (normalized_value != nullptr) {
    *normalized_value = normalized;
  }

  if (normalized.empty() || normalized == "rule" || normalized == "rule_relay") {
    *mode = RelayPolicyMode::kRule;
    return true;
  }
  if (normalized == "off" || normalized == "disabled" || normalized == "no_relay") {
    *mode = RelayPolicyMode::kOff;
    return true;
  }
  if (normalized == "rl" || normalized == "rl_relay") {
    *mode = RelayPolicyMode::kRl;
    return true;
  }
  return false;
}

const char* RelayPolicyModeName(const RelayPolicyMode mode) {
  switch (mode) {
    case RelayPolicyMode::kOff:
      return "off";
    case RelayPolicyMode::kRl:
      return "rl";
    case RelayPolicyMode::kRule:
    default:
      return "rule";
  }
}

}  // namespace

class RacerCommControllerNode {
public:
  RacerCommControllerNode()
      : nh_(),
        pnh_("~"),
        relay_active_(false),
        relay_agent_id_(-1),
        relay_startup_grace_period_(0.0),
        prefer_service_input_(true),
        service_input_timeout_sec_(1.0),
        service_input_replaces_graph_(false),
        comm_component_requires_bidirectional_(false),
        last_input_source_(InputSource::kUnknown),
        relay_policy_mode_(RelayPolicyMode::kRule),
        relay_target_use_fixed_z_(true),
        relay_target_fixed_z_(1.0),
        relay_target_min_z_(0.9),
        relay_target_max_z_(1.3),
        publish_relay_role_cmds_(true) {
    pnh_.param("relay_trigger_age_threshold", relay_trigger_age_threshold_, 0.5);
    pnh_.param("relay_link_exit_age_threshold", relay_link_exit_age_threshold_, 0.8);
    pnh_.param("relay_trigger_persist_time", relay_trigger_persist_time_, 0.5);
    pnh_.param("relay_recover_time", relay_recover_time_, 1.0);
    pnh_.param("relay_min_hold_time", relay_min_hold_time_, 1.5);
    pnh_.param("relay_reenter_cooldown", relay_reenter_cooldown_, 1.0);
    pnh_.param("relay_startup_grace_period", relay_startup_grace_period_, 0.0);
    pnh_.param("relay_max_occupancy_ratio", relay_max_occupancy_ratio_, 0.5);
    pnh_.param("relay_no_improvement_timeout", relay_no_improvement_timeout_, 15.0);
    pnh_.param("relay_max_active_duration", relay_max_active_duration_, 30.0);
    pnh_.param("relay_failed_agent_cooldown", relay_failed_agent_cooldown_, 20.0);
    pnh_.param("relay_endgame_completion_ratio_threshold",
        relay_endgame_completion_ratio_threshold_, 0.99);
    pnh_.param("relay_endgame_frontier_cell_threshold",
        relay_endgame_frontier_cell_threshold_, 1200);
    pnh_.param("relay_endgame_frontier_cluster_threshold",
        relay_endgame_frontier_cluster_threshold_, 3);
    pnh_.param("relay_endgame_active_grid_threshold",
        relay_endgame_active_grid_threshold_, 4);
    pnh_.param("relay_candidate_speed_penalty_gain", relay_candidate_speed_penalty_gain_, 4.0);
    pnh_.param("relay_candidate_reuse_penalty_gain", relay_candidate_reuse_penalty_gain_, 0.08);
    pnh_.param("relay_candidate_small_component_penalty_gain",
        relay_candidate_small_component_penalty_gain_, 6.0);
    pnh_.param("relay_candidate_backtrack_penalty_gain", relay_candidate_backtrack_penalty_gain_, 8.0);
    pnh_.param("relay_suppress_balanced_split", relay_suppress_balanced_split_, true);
    pnh_.param("relay_balanced_min_component_size", relay_balanced_min_component_size_, 2);
    pnh_.param("relay_balanced_size_ratio_threshold", relay_balanced_size_ratio_threshold_, 0.75);
    pnh_.param("relay_balanced_min_separation_m", relay_balanced_min_separation_m_, 10.0);
    pnh_.param("relay_balanced_axis_ratio_threshold", relay_balanced_axis_ratio_threshold_, 1.8);
    pnh_.param("relay_balanced_outward_velocity_threshold",
        relay_balanced_outward_velocity_threshold_, 0.15);
    pnh_.param("relay_anchor_use_nearest_pair_midpoint", relay_anchor_use_nearest_pair_midpoint_, true);
    pnh_.param("relay_frontier_improvement_min_delta", relay_frontier_improvement_min_delta_, 200);
    pnh_.param("relay_frontier_regression_grace_sec", relay_frontier_regression_grace_sec_, 8.0);
    pnh_.param("relay_frontier_regression_abs_threshold",
        relay_frontier_regression_abs_threshold_, 1500);
    pnh_.param("relay_frontier_regression_ratio_threshold",
        relay_frontier_regression_ratio_threshold_, 1.25);
    pnh_.param("relay_target_use_fixed_z", relay_target_use_fixed_z_, true);
    pnh_.param("relay_target_fixed_z", relay_target_fixed_z_, 1.0);
    pnh_.param("relay_target_min_z", relay_target_min_z_, 0.9);
    pnh_.param("relay_target_max_z", relay_target_max_z_, 1.3);
    pnh_.param("relay_target_obstacle_clearance", relay_target_obstacle_clearance_, 0.6);
    std::string relay_policy_mode_param = "rule";
    pnh_.param<std::string>("relay_policy_mode", relay_policy_mode_param, "rule");
    pnh_.param("id_offset", id_offset_, -1);
    pnh_.param("prefer_service_input", prefer_service_input_, true);
    pnh_.param("service_input_timeout_sec", service_input_timeout_sec_, 1.0);
    pnh_.param("service_input_replaces_graph", service_input_replaces_graph_, false);
    pnh_.param("comm_component_requires_bidirectional", comm_component_requires_bidirectional_, false);
    pnh_.param("observed_agent_retention_sec", observed_agent_retention_sec_, 5.0);
    pnh_.param("agent_cache_timeout_sec", agent_cache_timeout_sec_, 3.0);
    pnh_.param("edge_state_prune_age_sec", edge_state_prune_age_sec_, 5.0);
    pnh_.param<std::string>("raw_comm_topic", raw_comm_topic_, "/comm/rx_events_bytes");
    pnh_.param<std::string>("relay_role_topic", relay_role_topic_, "/relay_integration/relay_role_cmd");
    pnh_.param<std::string>(
        "swarm_comm_state_topic", swarm_comm_state_topic_, "/relay_integration/swarm_comm_state");
    pnh_.param<std::string>("task_metrics_topic", task_metrics_topic_, "/relay_integration/task_metrics");
    pnh_.param<std::string>("service_name", service_name_, "get_agents_velocities");
    pnh_.param("publish_relay_role_cmds", publish_relay_role_cmds_, true);

    relay_link_exit_age_threshold_ = std::max(relay_link_exit_age_threshold_, relay_trigger_age_threshold_);
    relay_trigger_persist_time_ = std::max(0.0, relay_trigger_persist_time_);
    relay_recover_time_ = std::max(0.0, relay_recover_time_);
    relay_min_hold_time_ = std::max(0.0, relay_min_hold_time_);
    relay_reenter_cooldown_ = std::max(0.0, relay_reenter_cooldown_);
    relay_startup_grace_period_ = std::max(0.0, relay_startup_grace_period_);
    relay_no_improvement_timeout_ = std::max(0.0, relay_no_improvement_timeout_);
    relay_max_active_duration_ = std::max(0.0, relay_max_active_duration_);
    relay_failed_agent_cooldown_ = std::max(0.0, relay_failed_agent_cooldown_);
    relay_endgame_completion_ratio_threshold_ = Clamp01(relay_endgame_completion_ratio_threshold_);
    relay_endgame_frontier_cell_threshold_ = std::max(-1, relay_endgame_frontier_cell_threshold_);
    relay_endgame_frontier_cluster_threshold_ = std::max(-1, relay_endgame_frontier_cluster_threshold_);
    relay_endgame_active_grid_threshold_ = std::max(-1, relay_endgame_active_grid_threshold_);
    relay_candidate_speed_penalty_gain_ = std::max(0.0, relay_candidate_speed_penalty_gain_);
    relay_candidate_reuse_penalty_gain_ = std::max(0.0, relay_candidate_reuse_penalty_gain_);
    relay_candidate_small_component_penalty_gain_ = std::max(0.0, relay_candidate_small_component_penalty_gain_);
    relay_candidate_backtrack_penalty_gain_ = std::max(0.0, relay_candidate_backtrack_penalty_gain_);
    relay_balanced_min_component_size_ = std::max(1, relay_balanced_min_component_size_);
    relay_balanced_size_ratio_threshold_ = Clamp01(relay_balanced_size_ratio_threshold_);
    relay_balanced_min_separation_m_ = std::max(0.0, relay_balanced_min_separation_m_);
    relay_balanced_axis_ratio_threshold_ = std::max(1.0, relay_balanced_axis_ratio_threshold_);
    relay_balanced_outward_velocity_threshold_ = std::max(0.0, relay_balanced_outward_velocity_threshold_);
    relay_frontier_improvement_min_delta_ = std::max(0, relay_frontier_improvement_min_delta_);
    relay_frontier_regression_grace_sec_ = std::max(0.0, relay_frontier_regression_grace_sec_);
    relay_frontier_regression_abs_threshold_ = std::max(0, relay_frontier_regression_abs_threshold_);
    relay_frontier_regression_ratio_threshold_ = std::max(1.0, relay_frontier_regression_ratio_threshold_);
    observed_agent_retention_sec_ = std::max(0.0, observed_agent_retention_sec_);
    agent_cache_timeout_sec_ = std::max(0.0, agent_cache_timeout_sec_);
    edge_state_prune_age_sec_ = std::max(0.0, edge_state_prune_age_sec_);
    if (relay_target_min_z_ > relay_target_max_z_) {
      std::swap(relay_target_min_z_, relay_target_max_z_);
    }
    relay_target_obstacle_clearance_ = std::max(0.0, relay_target_obstacle_clearance_);
    XmlRpc::XmlRpcValue obstacles_param;
    if (pnh_.getParam("obstacles", obstacles_param)) {
      ParseObstacleBoxes(obstacles_param, &relay_obstacles_);
      ROS_INFO_STREAM("[RelayCtl] loaded obstacle boxes=" << relay_obstacles_.size()
                      << " relay_target_obstacle_clearance="
                      << relay_target_obstacle_clearance_);
    }
    std::string normalized_policy_mode;
    if (!TryParseRelayPolicyMode(relay_policy_mode_param, &relay_policy_mode_, &normalized_policy_mode)) {
      relay_policy_mode_ = RelayPolicyMode::kRule;
      ROS_WARN_STREAM("Unknown relay_policy_mode=\"" << relay_policy_mode_param
                      << "\". Falling back to rule.");
    }

    raw_comm_sub_ = nh_.subscribe(raw_comm_topic_, 10, &RacerCommControllerNode::rawCommCallback, this);
    task_metrics_sub_ =
        nh_.subscribe(task_metrics_topic_, 20, &RacerCommControllerNode::taskMetricsCallback, this);
    relay_role_pub_ = nh_.advertise<RelayRoleCmd>(relay_role_topic_, 20);
    swarm_comm_state_pub_ = nh_.advertise<SwarmCommState>(swarm_comm_state_topic_, 10, true);
    get_agent_velocities_srv_ =
        nh_.advertiseService(service_name_, &RacerCommControllerNode::getAgentVelocitiesCallback, this);

    relay_anchor_.x = 0.0;
    relay_anchor_.y = 0.0;
    relay_anchor_.z = 0.0;

    ROS_INFO_STREAM("racer_comm_controller_node serving " << service_name_
                    << ", raw_comm_topic=" << raw_comm_topic_
                    << ", relay_role_topic=" << relay_role_topic_
                    << ", task_metrics_topic=" << task_metrics_topic_
                    << ", prefer_service_input=" << std::boolalpha << prefer_service_input_
                    << ", service_input_timeout_sec=" << service_input_timeout_sec_
                    << ", service_input_replaces_graph=" << service_input_replaces_graph_
                    << ", comm_component_requires_bidirectional=" << std::boolalpha
                    << comm_component_requires_bidirectional_
                    << ", observed_agent_retention_sec=" << observed_agent_retention_sec_
                    << ", agent_cache_timeout_sec=" << agent_cache_timeout_sec_
                    << ", edge_state_prune_age_sec=" << edge_state_prune_age_sec_
                    << ", relay_trigger_age_threshold=" << relay_trigger_age_threshold_
                    << ", relay_link_exit_age_threshold=" << relay_link_exit_age_threshold_
                    << ", relay_trigger_persist_time=" << relay_trigger_persist_time_
                    << ", relay_recover_time=" << relay_recover_time_
                    << ", relay_min_hold_time=" << relay_min_hold_time_
                    << ", relay_reenter_cooldown=" << relay_reenter_cooldown_
                    << ", relay_startup_grace_period=" << relay_startup_grace_period_
                    << ", relay_no_improvement_timeout=" << relay_no_improvement_timeout_
                    << ", relay_max_active_duration=" << relay_max_active_duration_
                    << ", relay_failed_agent_cooldown=" << relay_failed_agent_cooldown_
                    << ", relay_endgame_completion_ratio_threshold="
                    << relay_endgame_completion_ratio_threshold_
                    << ", relay_endgame_frontier_cell_threshold="
                    << relay_endgame_frontier_cell_threshold_
                    << ", relay_endgame_frontier_cluster_threshold="
                    << relay_endgame_frontier_cluster_threshold_
                    << ", relay_endgame_active_grid_threshold="
                    << relay_endgame_active_grid_threshold_
                    << ", relay_candidate_speed_penalty_gain=" << relay_candidate_speed_penalty_gain_
                    << ", relay_candidate_reuse_penalty_gain=" << relay_candidate_reuse_penalty_gain_
                    << ", relay_candidate_small_component_penalty_gain="
                    << relay_candidate_small_component_penalty_gain_
                    << ", relay_candidate_backtrack_penalty_gain=" << relay_candidate_backtrack_penalty_gain_
                    << ", relay_suppress_balanced_split=" << std::boolalpha << relay_suppress_balanced_split_
                    << ", relay_balanced_min_component_size=" << relay_balanced_min_component_size_
                    << ", relay_balanced_size_ratio_threshold=" << relay_balanced_size_ratio_threshold_
                    << ", relay_balanced_min_separation_m=" << relay_balanced_min_separation_m_
                    << ", relay_balanced_axis_ratio_threshold=" << relay_balanced_axis_ratio_threshold_
                    << ", relay_balanced_outward_velocity_threshold=" << relay_balanced_outward_velocity_threshold_
                    << ", relay_anchor_use_nearest_pair_midpoint=" << relay_anchor_use_nearest_pair_midpoint_
                    << ", relay_frontier_improvement_min_delta=" << relay_frontier_improvement_min_delta_
                    << ", relay_frontier_regression_grace_sec=" << relay_frontier_regression_grace_sec_
                    << ", relay_frontier_regression_abs_threshold="
                    << relay_frontier_regression_abs_threshold_
                    << ", relay_frontier_regression_ratio_threshold="
                    << relay_frontier_regression_ratio_threshold_
                    << ", relay_policy_mode=" << RelayPolicyModeName(relay_policy_mode_)
                    << ", relay_target_use_fixed_z=" << relay_target_use_fixed_z_
                    << ", relay_target_fixed_z=" << relay_target_fixed_z_
                    << ", relay_target_min_z=" << relay_target_min_z_
                    << ", relay_target_max_z=" << relay_target_max_z_
                    << ", publish_relay_role_cmds=" << std::boolalpha << publish_relay_role_cmds_);
  }

private:
  struct AgentCacheEntry {
    int platform_id = -1;
    geometry_msgs::Point position;
    geometry_msgs::Vector3 velocity;
    double heading = 0.0;
    ros::Time stamp;
  };

  struct DirectedEdgeState {
    uint64_t last_rx_time_us = 0;
    float quality = 0.0f;
    double delay_sec = 0.0;
  };

  struct CommSnapshot {
    uint64_t window_start_us = 0;
    uint64_t window_end_us = 0;
    uint64_t horizon_time_us = 0;
    size_t event_count = 0;
    std::unordered_map<long long, DirectedEdgeState> edges;
    std::set<int> observed_agent_ids;
  };

  bool getAgentVelocitiesCallback(dancers_msgs::GetAgentVelocities::Request& req,
      dancers_msgs::GetAgentVelocities::Response& res) {
    updateAgentCache(req.agent_structs);

    CommSnapshot snapshot;
    const bool has_snapshot = decodeCommSnapshot(req.comm_rx_events_bytes, &snapshot);
    const ros::Time now = ros::Time::now();
    const bool applied_fresh_snapshot = has_snapshot &&
        applyCommSnapshot(snapshot, InputSource::kServiceChain, service_input_replaces_graph_, now);

    evaluateAndPublish(applied_fresh_snapshot ? InputSource::kServiceChain : InputSource::kUnknown);
    fillZeroVelocityResponse(req.agent_structs, res);
    return true;
  }

  void rawCommCallback(const std_msgs::UInt8MultiArrayConstPtr& msg) {
    if (msg->data.empty()) {
      return;
    }

    CommSnapshot snapshot;
    if (!decodeCommSnapshot(msg->data, &snapshot)) {
      return;
    }

    const ros::Time now = ros::Time::now();
    if (shouldPreferServiceInput(now, &snapshot)) {
      ROS_INFO_THROTTLE(
          2.0,
          "Skipping raw /comm/rx_events_bytes parsing and relay evaluation because a fresher service-chain snapshot is active.");
      return;
    }

    if (!applyCommSnapshot(snapshot, InputSource::kRawTopic, false, now)) {
      return;
    }

    last_raw_topic_update_ = now;
    evaluateAndPublish(InputSource::kRawTopic);
  }

  void taskMetricsCallback(const std_msgs::Int32MultiArrayConstPtr& msg) {
    saw_task_metrics_ = true;
    if (msg->data.size() >= 1) {
      unknown_cells_total_ = static_cast<int>(msg->data[0]);
      unknown_cells_peak_ = std::max(unknown_cells_peak_, std::max(0, unknown_cells_total_));
      if (unknown_cells_peak_ > 0) {
        completion_ratio_ = Clamp01(
            1.0 - static_cast<double>(unknown_cells_total_) / static_cast<double>(unknown_cells_peak_));
      }
    }
    if (msg->data.size() >= 2) {
      frontier_cluster_count_metric_ = static_cast<int>(msg->data[1]);
    }
    if (msg->data.size() >= 3) {
      frontier_cell_count_metric_ = static_cast<int>(msg->data[2]);
    }
    if (msg->data.size() >= 4) {
      active_grid_count_ = static_cast<int>(msg->data[3]);
    }
  }

  void noteObservedAgent(const int agent_id, const ros::Time& now) {
    if (agent_id <= 0) {
      return;
    }
    observed_agent_ids_.insert(agent_id);
    agent_last_observed_wall_time_[agent_id] = now;
  }

  void updateAgentCache(const std::vector<dancers_msgs::AgentStruct>& agents) {
    const ros::Time now = ros::Time::now();
    for (const auto& agent : agents) {
      const int platform_id = static_cast<int>(agent.agent_id);
      const int racer_id = RacerIdFromPlatformId(platform_id, id_offset_);
      if (racer_id <= 0) {
        continue;
      }

      AgentCacheEntry entry;
      entry.platform_id = platform_id;
      entry.position.x = agent.state.position.x;
      entry.position.y = agent.state.position.y;
      entry.position.z = agent.state.position.z;
      entry.velocity.x = agent.state.velocity.x;
      entry.velocity.y = agent.state.velocity.y;
      entry.velocity.z = agent.state.velocity.z;
      entry.heading = agent.state.heading;
      entry.stamp = now;
      agent_cache_[racer_id] = entry;
      noteObservedAgent(racer_id, now);
    }
  }

  uint64_t resolveEventTimeUs(
      const protobuf_msgs::RxEvent& event, const uint64_t snapshot_window_end_us) const {
    if (event.rx_time_us() != 0) {
      return event.rx_time_us();
    }
    if (event.tx_time_us() != 0 && event.delay_us() != 0) {
      return event.tx_time_us() + event.delay_us();
    }
    if (snapshot_window_end_us != 0) {
      return snapshot_window_end_us;
    }
    return event.tx_time_us();
  }

  float computeEdgeQuality(const protobuf_msgs::RxEvent& event, const double delay_sec) const {
    double quality = 1.0;
    if (delay_sec > 0.0) {
      quality = 1.0 / (1.0 + 10.0 * delay_sec);
    }
    if (event.rssi_dbm_x10() != 0) {
      quality += 0.001 * static_cast<double>(event.rssi_dbm_x10() + 900);
    }
    return static_cast<float>(Clamp01(quality));
  }

  bool decodeCommSnapshot(const std::vector<uint8_t>& bytes, CommSnapshot* snapshot) const {
    if (snapshot == nullptr || bytes.empty()) {
      return false;
    }

    protobuf_msgs::NetRxEventsPayload payload;
    if (!payload.ParseFromArray(bytes.data(), static_cast<int>(bytes.size()))) {
      ROS_WARN_THROTTLE(1.0, "Failed to parse NetRxEventsPayload bytes.");
      return false;
    }

    *snapshot = CommSnapshot();
    snapshot->window_start_us = payload.t_window_start_us();
    snapshot->window_end_us = payload.t_window_end_us();
    snapshot->horizon_time_us = snapshot->window_end_us;
    snapshot->event_count = static_cast<size_t>(payload.events_size());

    for (const auto& event : payload.events()) {
      const int src_id = RacerIdFromPlatformId(static_cast<int>(event.src_id()), id_offset_);
      const int dst_id = RacerIdFromPlatformId(static_cast<int>(event.dst_id()), id_offset_);
      if (src_id <= 0 || dst_id <= 0) {
        continue;
      }

      snapshot->observed_agent_ids.insert(src_id);
      snapshot->observed_agent_ids.insert(dst_id);

      DirectedEdgeState candidate;
      candidate.last_rx_time_us = resolveEventTimeUs(event, snapshot->window_end_us);
      candidate.delay_sec = static_cast<double>(event.delay_us()) * 1e-6;
      candidate.quality = computeEdgeQuality(event, candidate.delay_sec);
      if (candidate.last_rx_time_us > 0) {
        snapshot->horizon_time_us = std::max(snapshot->horizon_time_us, candidate.last_rx_time_us);
      }

      const long long edge_key = DirectedKey(src_id, dst_id);
      const auto existing_it = snapshot->edges.find(edge_key);
      if (existing_it == snapshot->edges.end() ||
          candidate.last_rx_time_us > existing_it->second.last_rx_time_us ||
          (candidate.last_rx_time_us == existing_it->second.last_rx_time_us &&
           candidate.quality > existing_it->second.quality)) {
        snapshot->edges[edge_key] = candidate;
      }
    }

    if (snapshot->horizon_time_us == 0) {
      ROS_WARN_THROTTLE(1.0,
          "NetRxEventsPayload missing usable timing metadata; dropping snapshot.");
      return false;
    }
    return true;
  }

  bool applyCommSnapshot(const CommSnapshot& snapshot, const InputSource source,
      const bool replace_graph, const ros::Time& now) {
    if (snapshot.horizon_time_us == 0 || snapshot.horizon_time_us <= latest_comm_horizon_us_) {
      return false;
    }

    if (replace_graph) {
      directed_edges_.clear();
      latest_link_up_.clear();
    }

    latest_comm_horizon_us_ = snapshot.horizon_time_us;
    for (const int agent_id : snapshot.observed_agent_ids) {
      noteObservedAgent(agent_id, now);
    }
    for (const auto& pair : snapshot.edges) {
      directed_edges_[pair.first] = pair.second;
    }

    if (source == InputSource::kServiceChain) {
      last_service_snapshot_horizon_us_ = snapshot.horizon_time_us;
      last_fresh_service_snapshot_wall_time_ = now;
    } else if (source == InputSource::kRawTopic) {
      last_raw_snapshot_horizon_us_ = snapshot.horizon_time_us;
    }
    return true;
  }

  bool shouldPreferServiceInput(
      const ros::Time& now, const CommSnapshot* incoming_snapshot = nullptr) const {
    if (!prefer_service_input_ || last_fresh_service_snapshot_wall_time_.isZero()) {
      return false;
    }
    if ((now - last_fresh_service_snapshot_wall_time_).toSec() > service_input_timeout_sec_) {
      return false;
    }
    if (incoming_snapshot != nullptr && incoming_snapshot->horizon_time_us > 0 &&
        incoming_snapshot->horizon_time_us > last_service_snapshot_horizon_us_) {
      return false;
    }
    return true;
  }

  void pruneStaleState(const ros::Time& now) {
    std::set<int> pruned_observed_ids;
    if (observed_agent_retention_sec_ > 0.0) {
      for (auto it = agent_last_observed_wall_time_.begin(); it != agent_last_observed_wall_time_.end();) {
        if (!it->second.isZero() && (now - it->second).toSec() > observed_agent_retention_sec_) {
          pruned_observed_ids.insert(it->first);
          it = agent_last_observed_wall_time_.erase(it);
        } else {
          ++it;
        }
      }
    }

    for (const int agent_id : pruned_observed_ids) {
      observed_agent_ids_.erase(agent_id);
      latest_components_.erase(agent_id);
      agent_cache_.erase(agent_id);
    }

    int pruned_agent_cache = 0;
    if (agent_cache_timeout_sec_ > 0.0) {
      for (auto it = agent_cache_.begin(); it != agent_cache_.end();) {
        if (!it->second.stamp.isZero() && (now - it->second.stamp).toSec() > agent_cache_timeout_sec_) {
          it = agent_cache_.erase(it);
          ++pruned_agent_cache;
        } else {
          ++it;
        }
      }
    }

    int pruned_edges = 0;
    for (auto it = directed_edges_.begin(); it != directed_edges_.end();) {
      const int src_id = DirectedKeySrc(it->first);
      const int dst_id = DirectedKeyDst(it->first);
      bool remove = pruned_observed_ids.count(src_id) != 0 || pruned_observed_ids.count(dst_id) != 0;
      if (!remove && edge_state_prune_age_sec_ > 0.0 && latest_comm_horizon_us_ > 0) {
        double edge_age_sec = std::numeric_limits<double>::infinity();
        if (it->second.last_rx_time_us != 0) {
          edge_age_sec = latest_comm_horizon_us_ <= it->second.last_rx_time_us
              ? 0.0
              : static_cast<double>(latest_comm_horizon_us_ - it->second.last_rx_time_us) * 1e-6;
        }
        if (edge_age_sec > edge_state_prune_age_sec_) {
          remove = true;
        }
      }
      if (remove) {
        it = directed_edges_.erase(it);
        ++pruned_edges;
      } else {
        ++it;
      }
    }

    for (auto it = latest_link_up_.begin(); it != latest_link_up_.end();) {
      const int src_id = DirectedKeySrc(it->first);
      const int dst_id = DirectedKeyDst(it->first);
      if (pruned_observed_ids.count(src_id) != 0 || pruned_observed_ids.count(dst_id) != 0) {
        it = latest_link_up_.erase(it);
      } else {
        ++it;
      }
    }

    if (!pruned_observed_ids.empty() || pruned_agent_cache > 0 || pruned_edges > 0) {
      ROS_INFO_STREAM_THROTTLE(1.0,
          "[RelayCtl] pruned stale state observed_agents=" << pruned_observed_ids.size()
          << " agent_cache=" << pruned_agent_cache
          << " edges=" << pruned_edges);
    }
  }

  void evaluateAndPublish(const InputSource input_source) {
    if (input_source != InputSource::kUnknown) {
      noteInputSource(input_source);
    }

    const ros::Time now = ros::Time::now();
    if (first_eval_time_.isZero()) {
      first_eval_time_ = now;
    }
    pruneStaleState(now);
    updateCommGraph(now);

    if (relay_active_ && relay_agent_id_ > 0) {
      const bool relay_agent_visible =
          std::find(current_comm_state_.agent_ids.begin(), current_comm_state_.agent_ids.end(),
                    relay_agent_id_) != current_comm_state_.agent_ids.end();
      const bool relay_agent_has_pose = agent_cache_.find(relay_agent_id_) != agent_cache_.end();
      if (!relay_agent_visible || !relay_agent_has_pose) {
        deactivateRelay(now, "relay_agent_stale");
      }
    }

    const RelayPolicyMode effective_policy_mode = effectiveRelayPolicyMode();
    if (effective_policy_mode == RelayPolicyMode::kOff) {
      if (relay_active_) {
        deactivateRelay(now, "policy_off");
      }
      relay_anchor_ = computeRelayAnchor();
    } else {
      if (relay_active_) {
        noteRelayProgress(now);
        relay_anchor_ = computeRelayAnchor();
        std::string exit_reason;
        if (evaluateRelayExit(now, &exit_reason)) {
          deactivateRelay(now, exit_reason);
        }
      } else if (evaluateRelayNeed(now)) {
        const geometry_msgs::Point anchor = computeRelayAnchor();
        const int relay_candidate = selectRelayAgent(anchor, now);
        if (relay_candidate > 0) {
          activateRelay(relay_candidate, anchor, now);
        }
      }
    }

    publishSwarmCommState(now);
    if (publish_relay_role_cmds_) {
      publishRelayRoleCommands(now);
    }
  }

  RelayPolicyMode effectiveRelayPolicyMode() const {
    if (relay_policy_mode_ == RelayPolicyMode::kRl) {
      ROS_WARN_THROTTLE(5.0,
          "relay_policy_mode=rl requested, but RL relay policy is not integrated yet. Falling back to rule policy.");
      return RelayPolicyMode::kRule;
    }
    return relay_policy_mode_;
  }

  void noteInputSource(const InputSource input_source) {
    if (input_source == last_input_source_) {
      return;
    }

    last_input_source_ = input_source;
    ROS_INFO_STREAM("racer_comm_controller_node now using " << InputSourceName(input_source)
                    << " as the primary relay decision input.");
  }

  void updateCommGraph(const ros::Time& now) {
    std::vector<int> agent_ids(observed_agent_ids_.begin(), observed_agent_ids_.end());
    std::sort(agent_ids.begin(), agent_ids.end());

    current_comm_state_.header.stamp = now;
    current_comm_state_.agent_ids.clear();
    current_comm_state_.component_ids.clear();
    current_comm_state_.link_src_ids.clear();
    current_comm_state_.link_dst_ids.clear();
    current_comm_state_.link_up.clear();
    current_comm_state_.link_quality.clear();
    current_comm_state_.link_age.clear();
    current_comm_state_.num_components = 0;

    latest_components_.clear();
    const auto previous_link_up = latest_link_up_;
    latest_link_up_.clear();

    if (agent_ids.empty()) {
      connected_since_ = ros::Time(0);
      fragmented_since_ = ros::Time(0);
      return;
    }

    current_comm_state_.agent_ids = agent_ids;

    std::map<int, std::vector<int>> adjacency;
    for (const int agent_id : agent_ids) {
      adjacency[agent_id] = std::vector<int>();
    }

    for (const int src_id : agent_ids) {
      for (const int dst_id : agent_ids) {
        if (src_id == dst_id) {
          continue;
        }

        const long long edge_key = DirectedKey(src_id, dst_id);
        const auto edge_it = directed_edges_.find(edge_key);
        const bool direct_seen = edge_it != directed_edges_.end();
        double link_age = std::numeric_limits<double>::infinity();
        if (direct_seen) {
          if (latest_comm_horizon_us_ == 0 || edge_it->second.last_rx_time_us == 0) {
            link_age = std::numeric_limits<double>::infinity();
          } else if (latest_comm_horizon_us_ <= edge_it->second.last_rx_time_us) {
            link_age = 0.0;
          } else {
            link_age = static_cast<double>(latest_comm_horizon_us_ - edge_it->second.last_rx_time_us) * 1e-6;
          }
        }
        const auto prev_it = previous_link_up.find(edge_key);
        const bool was_up = prev_it != previous_link_up.end() && prev_it->second;
        const double link_age_threshold = was_up ? relay_link_exit_age_threshold_ : relay_trigger_age_threshold_;
        const bool link_up = direct_seen && link_age <= link_age_threshold;
        const float quality = link_up ? edge_it->second.quality : 0.0f;
        const float age = std::isfinite(link_age) ? static_cast<float>(link_age)
                                                  : std::numeric_limits<float>::infinity();

        current_comm_state_.link_src_ids.push_back(src_id);
        current_comm_state_.link_dst_ids.push_back(dst_id);
        current_comm_state_.link_up.push_back(link_up);
        current_comm_state_.link_quality.push_back(quality);
        current_comm_state_.link_age.push_back(age);
        latest_link_up_[DirectedKey(src_id, dst_id)] = link_up;
      }
    }

    for (size_t i = 0; i < agent_ids.size(); ++i) {
      for (size_t j = i + 1; j < agent_ids.size(); ++j) {
        const int lhs = agent_ids[i];
        const int rhs = agent_ids[j];
        const bool lhs_rhs = latestLinkUp(lhs, rhs);
        const bool rhs_lhs = latestLinkUp(rhs, lhs);
        const bool undirected_link_up = comm_component_requires_bidirectional_
            ? (lhs_rhs && rhs_lhs)
            : (lhs_rhs || rhs_lhs);
        if (undirected_link_up) {
          adjacency[lhs].push_back(rhs);
          adjacency[rhs].push_back(lhs);
        }
      }
    }

    std::set<int> visited;
    int component_id = 0;
    for (const int agent_id : agent_ids) {
      if (visited.count(agent_id) != 0) {
        continue;
      }

      ++component_id;
      std::vector<int> stack{agent_id};
      while (!stack.empty()) {
        const int current = stack.back();
        stack.pop_back();
        if (visited.count(current) != 0) {
          continue;
        }

        visited.insert(current);
        latest_components_[current] = component_id;
        for (const int neighbor : adjacency[current]) {
          if (visited.count(neighbor) == 0) {
            stack.push_back(neighbor);
          }
        }
      }
    }

    current_comm_state_.num_components = component_id;
    for (const int agent_id : agent_ids) {
      current_comm_state_.component_ids.push_back(latest_components_[agent_id]);
    }

    if (component_id <= 1) {
      if (connected_since_.isZero()) {
        connected_since_ = now;
      }
      fragmented_since_ = ros::Time(0);
    } else {
      connected_since_ = ros::Time(0);
      if (fragmented_since_.isZero()) {
        fragmented_since_ = now;
      }
    }
  }

  bool shouldSuppressRelayInEndgame() const {
    if (!saw_task_metrics_) {
      return false;
    }

    const bool low_frontier_cells =
        relay_endgame_frontier_cell_threshold_ >= 0 && frontier_cell_count_metric_ >= 0 &&
        frontier_cell_count_metric_ <= relay_endgame_frontier_cell_threshold_;
    const bool low_active_grids =
        relay_endgame_active_grid_threshold_ >= 0 && active_grid_count_ >= 0 &&
        active_grid_count_ <= relay_endgame_active_grid_threshold_;
    const bool low_frontier_clusters =
        relay_endgame_frontier_cluster_threshold_ >= 0 && frontier_cluster_count_metric_ >= 0 &&
        frontier_cluster_count_metric_ <= relay_endgame_frontier_cluster_threshold_;
    const bool near_completion = completion_ratio_ >= relay_endgame_completion_ratio_threshold_;
    return low_frontier_cells || low_active_grids || (near_completion && low_frontier_clusters);
  }

  bool evaluateRelayNeed(const ros::Time& now) const {
    if (current_comm_state_.agent_ids.size() < 2) {
      return false;
    }
    if (!first_eval_time_.isZero() && relay_startup_grace_period_ > 0.0 &&
        (now - first_eval_time_).toSec() < relay_startup_grace_period_) {
      return false;
    }
    if (shouldSuppressRelayInEndgame()) {
      return false;
    }
    if (shouldSuppressRelayForBalancedSplit()) {
      ROS_INFO_STREAM_THROTTLE(2.0,
          "[RelayCtl] suppress relay for balanced independent split num_components="
          << current_comm_state_.num_components);
      return false;
    }
    if (current_comm_state_.num_components <= 1) {
      return false;
    }
    if (fragmented_since_.isZero()) {
      return false;
    }
    if ((now - fragmented_since_).toSec() < relay_trigger_persist_time_) {
      return false;
    }
    if (!relay_exit_time_.isZero() && (now - relay_exit_time_).toSec() < relay_reenter_cooldown_) {
      return false;
    }
    const double min_ratio = 1.0 / static_cast<double>(current_comm_state_.agent_ids.size());
    return min_ratio <= relay_max_occupancy_ratio_;
  }

  void activateRelay(
      const int relay_candidate, const geometry_msgs::Point& anchor, const ros::Time& now) {
    relay_active_ = true;
    relay_agent_id_ = relay_candidate;
    relay_anchor_ = anchor;
    relay_change_time_ = now;
    relay_start_num_components_ = std::max(0, current_comm_state_.num_components);
    relay_best_num_components_ = relay_start_num_components_;
    relay_start_frontier_cell_count_ = frontier_cell_count_metric_;
    relay_best_frontier_cell_count_ = frontier_cell_count_metric_;
    relay_last_improvement_time_ = now;
    ROS_INFO_STREAM("[RelayCtl] activate relay agent=" << relay_agent_id_
                    << " start_components=" << relay_start_num_components_
                    << " anchor=(" << relay_anchor_.x << ", " << relay_anchor_.y << ", "
                    << relay_anchor_.z << ")");
  }

  void deactivateRelay(const ros::Time& now, const std::string& reason) {
    if (!relay_active_ || relay_agent_id_ <= 0) {
      relay_active_ = false;
      relay_agent_id_ = -1;
      relay_change_time_ = now;
      relay_exit_time_ = now;
      relay_start_num_components_ = 0;
      relay_best_num_components_ = 0;
      relay_last_improvement_time_ = ros::Time(0);
      return;
    }

    const int previous_agent_id = relay_agent_id_;
    const double duration_sec = relay_change_time_.isZero() ? 0.0 : (now - relay_change_time_).toSec();
    relay_total_occupancy_by_agent_[previous_agent_id] += duration_sec;
    if (reason == "no_improvement" || reason == "max_active_duration" ||
        reason == "frontier_regression") {
      last_failed_relay_agent_id_ = previous_agent_id;
      relay_last_failed_time_ = now;
    }

    relay_active_ = false;
    relay_agent_id_ = -1;
    relay_change_time_ = now;
    relay_exit_time_ = now;
    relay_start_num_components_ = 0;
    relay_best_num_components_ = 0;
    relay_start_frontier_cell_count_ = -1;
    relay_best_frontier_cell_count_ = -1;
    relay_last_improvement_time_ = ros::Time(0);

    ROS_INFO_STREAM("[RelayCtl] deactivate relay agent=" << previous_agent_id
                    << " reason=" << reason
                    << " duration_sec=" << duration_sec
                    << " current_components=" << current_comm_state_.num_components);
  }

  void noteRelayProgress(const ros::Time& now) {
    if (!relay_active_ || relay_agent_id_ <= 0) {
      return;
    }
    if (current_comm_state_.num_components > 0 &&
        (relay_best_num_components_ <= 0 || current_comm_state_.num_components < relay_best_num_components_)) {
      relay_best_num_components_ = current_comm_state_.num_components;
      relay_last_improvement_time_ = now;
      ROS_INFO_STREAM("[RelayCtl] relay agent=" << relay_agent_id_
                      << " improved components to " << relay_best_num_components_
                      << " from start=" << relay_start_num_components_);
    }
    if (saw_task_metrics_ && frontier_cell_count_metric_ >= 0) {
      if (relay_best_frontier_cell_count_ < 0) {
        relay_best_frontier_cell_count_ = frontier_cell_count_metric_;
      } else if (frontier_cell_count_metric_ <=
          relay_best_frontier_cell_count_ - relay_frontier_improvement_min_delta_) {
        relay_best_frontier_cell_count_ = frontier_cell_count_metric_;
        relay_last_improvement_time_ = now;
        ROS_INFO_STREAM("[RelayCtl] relay agent=" << relay_agent_id_
                        << " improved frontier_cells to " << relay_best_frontier_cell_count_);
      }
    }
  }

  bool evaluateRelayExit(const ros::Time& now, std::string* reason) const {
    if (!relay_active_ || relay_agent_id_ <= 0) {
      return false;
    }

    const double active_duration_sec = (now - relay_change_time_).toSec();
    if (active_duration_sec < relay_min_hold_time_) {
      return false;
    }
    if (current_comm_state_.num_components <= 1 && !connected_since_.isZero() &&
        (now - connected_since_).toSec() >= relay_recover_time_) {
      if (reason != nullptr) {
        *reason = "recovered";
      }
      return true;
    }
    if (relay_no_improvement_timeout_ > 0.0 && !relay_last_improvement_time_.isZero() &&
        (now - relay_last_improvement_time_).toSec() >= relay_no_improvement_timeout_ &&
        current_comm_state_.num_components >= relay_best_num_components_) {
      if (reason != nullptr) {
        *reason = "no_improvement";
      }
      return true;
    }
    if (relay_frontier_regression_grace_sec_ > 0.0 && saw_task_metrics_ &&
        relay_start_frontier_cell_count_ >= 0 && relay_best_frontier_cell_count_ >= 0 &&
        frontier_cell_count_metric_ >= 0 &&
        active_duration_sec >= relay_frontier_regression_grace_sec_) {
      const int frontier_regression = frontier_cell_count_metric_ - relay_best_frontier_cell_count_;
      const double frontier_ratio = relay_start_frontier_cell_count_ > 0
          ? static_cast<double>(frontier_cell_count_metric_) /
              static_cast<double>(relay_start_frontier_cell_count_)
          : 1.0;
      if (frontier_regression >= relay_frontier_regression_abs_threshold_ &&
          frontier_ratio >= relay_frontier_regression_ratio_threshold_) {
        if (reason != nullptr) {
          *reason = "frontier_regression";
        }
        return true;
      }
    }
    if (relay_max_active_duration_ > 0.0 && active_duration_sec >= relay_max_active_duration_) {
      if (reason != nullptr) {
        *reason = "max_active_duration";
      }
      return true;
    }
    return false;
  }

  std::vector<std::vector<int>> orderedComponentsBySize() const {
    std::map<int, std::vector<int>> components;
    for (const auto& pair : latest_components_) {
      components[pair.second].push_back(pair.first);
    }

    std::vector<std::vector<int>> ordered_components;
    for (const auto& pair : components) {
      ordered_components.push_back(pair.second);
    }
    std::sort(ordered_components.begin(), ordered_components.end(),
        [](const std::vector<int>& lhs, const std::vector<int>& rhs) {
          return lhs.size() > rhs.size();
        });
    return ordered_components;
  }

  bool computeCentroidForAgents(const std::vector<int>& ids, geometry_msgs::Point* point) const {
    if (point == nullptr) {
      return false;
    }
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    int valid_count = 0;
    for (const int agent_id : ids) {
      const auto agent_it = agent_cache_.find(agent_id);
      if (agent_it == agent_cache_.end()) {
        continue;
      }
      x += agent_it->second.position.x;
      y += agent_it->second.position.y;
      z += agent_it->second.position.z;
      ++valid_count;
    }
    if (valid_count == 0) {
      return false;
    }
    point->x = x / static_cast<double>(valid_count);
    point->y = y / static_cast<double>(valid_count);
    point->z = z / static_cast<double>(valid_count);
    return true;
  }

  bool computeAverageVelocityForAgents(
      const std::vector<int>& ids, geometry_msgs::Vector3* velocity) const {
    if (velocity == nullptr) {
      return false;
    }
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    int valid_count = 0;
    for (const int agent_id : ids) {
      const auto agent_it = agent_cache_.find(agent_id);
      if (agent_it == agent_cache_.end()) {
        continue;
      }
      x += agent_it->second.velocity.x;
      y += agent_it->second.velocity.y;
      z += agent_it->second.velocity.z;
      ++valid_count;
    }
    if (valid_count == 0) {
      return false;
    }
    velocity->x = x / static_cast<double>(valid_count);
    velocity->y = y / static_cast<double>(valid_count);
    velocity->z = z / static_cast<double>(valid_count);
    return true;
  }

  bool computeNearestBridgeMidpoint(
      const std::vector<int>& lhs_ids,
      const std::vector<int>& rhs_ids,
      geometry_msgs::Point* anchor) const {
    if (anchor == nullptr) {
      return false;
    }
    double best_distance_sq = std::numeric_limits<double>::infinity();
    bool found = false;
    for (const int lhs_id : lhs_ids) {
      const auto lhs_it = agent_cache_.find(lhs_id);
      if (lhs_it == agent_cache_.end()) {
        continue;
      }
      for (const int rhs_id : rhs_ids) {
        const auto rhs_it = agent_cache_.find(rhs_id);
        if (rhs_it == agent_cache_.end()) {
          continue;
        }
        const double distance_sq = SquaredDistance(lhs_it->second.position, rhs_it->second.position);
        if (distance_sq >= best_distance_sq) {
          continue;
        }
        best_distance_sq = distance_sq;
        anchor->x = 0.5 * (lhs_it->second.position.x + rhs_it->second.position.x);
        anchor->y = 0.5 * (lhs_it->second.position.y + rhs_it->second.position.y);
        anchor->z = 0.5 * (lhs_it->second.position.z + rhs_it->second.position.z);
        found = true;
      }
    }
    return found;
  }

  bool computeWeightedCentroidForComponents(
      const std::vector<std::vector<int>>& components, geometry_msgs::Point* point) const {
    if (point == nullptr) {
      return false;
    }

    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double total_weight = 0.0;
    for (const auto& ids : components) {
      geometry_msgs::Point centroid;
      if (!computeCentroidForAgents(ids, &centroid)) {
        continue;
      }
      const double weight = static_cast<double>(std::max<size_t>(1, ids.size()));
      x += weight * centroid.x;
      y += weight * centroid.y;
      z += weight * centroid.z;
      total_weight += weight;
    }
    if (total_weight <= 0.0) {
      return false;
    }

    point->x = x / total_weight;
    point->y = y / total_weight;
    point->z = z / total_weight;
    return true;
  }

  double distanceToClosestAgent(
      const geometry_msgs::Point& point, const std::vector<int>& ids) const {
    double best_distance_sq = std::numeric_limits<double>::infinity();
    for (const int agent_id : ids) {
      const auto agent_it = agent_cache_.find(agent_id);
      if (agent_it == agent_cache_.end()) {
        continue;
      }
      best_distance_sq = std::min(best_distance_sq, SquaredDistance(point, agent_it->second.position));
    }
    if (!std::isfinite(best_distance_sq)) {
      return std::numeric_limits<double>::infinity();
    }
    return std::sqrt(best_distance_sq);
  }

  double scoreAnchorAgainstComponents(
      const geometry_msgs::Point& candidate,
      const std::vector<std::vector<int>>& components) const {
    double score = 0.0;
    bool found = false;
    for (const auto& ids : components) {
      const double distance = distanceToClosestAgent(candidate, ids);
      if (!std::isfinite(distance)) {
        continue;
      }
      score += static_cast<double>(std::max<size_t>(1, ids.size())) * distance;
      found = true;
    }
    return found ? score : std::numeric_limits<double>::infinity();
  }

  double computeBacktrackPenalty(
      const AgentCacheEntry& entry, const geometry_msgs::Point& anchor) const {
    const double vx = entry.velocity.x;
    const double vy = entry.velocity.y;
    const double vz = entry.velocity.z;
    const double speed = std::sqrt(vx * vx + vy * vy + vz * vz);
    if (speed < 1e-3) {
      return 0.0;
    }

    const double dx = anchor.x - entry.position.x;
    const double dy = anchor.y - entry.position.y;
    const double dz = anchor.z - entry.position.z;
    const double anchor_distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (anchor_distance < 1e-3) {
      return 0.0;
    }

    const double cosine =
        (vx * dx + vy * dy + vz * dz) / (speed * anchor_distance);
    return std::max(0.0, -cosine) * anchor_distance;
  }

  bool shouldSuppressRelayForBalancedSplit() const {
    if (!relay_suppress_balanced_split_ || current_comm_state_.num_components != 2) {
      return false;
    }

    const auto ordered_components = orderedComponentsBySize();
    if (ordered_components.size() != 2) {
      return false;
    }

    const int lhs_size = static_cast<int>(ordered_components[0].size());
    const int rhs_size = static_cast<int>(ordered_components[1].size());
    const int min_size = std::min(lhs_size, rhs_size);
    const int max_size = std::max(lhs_size, rhs_size);
    if (min_size < relay_balanced_min_component_size_ || max_size <= 0) {
      return false;
    }

    const double size_ratio = static_cast<double>(min_size) / static_cast<double>(max_size);
    if (size_ratio < relay_balanced_size_ratio_threshold_) {
      return false;
    }

    geometry_msgs::Point lhs_centroid;
    geometry_msgs::Point rhs_centroid;
    if (!computeCentroidForAgents(ordered_components[0], &lhs_centroid) ||
        !computeCentroidForAgents(ordered_components[1], &rhs_centroid)) {
      return false;
    }

    const double dx = rhs_centroid.x - lhs_centroid.x;
    const double dy = rhs_centroid.y - lhs_centroid.y;
    const double separation = std::sqrt(dx * dx + dy * dy);
    if (separation < relay_balanced_min_separation_m_) {
      return false;
    }

    const double major_axis = std::max(std::abs(dx), std::abs(dy));
    const double minor_axis = std::max(1e-3, std::min(std::abs(dx), std::abs(dy)));
    if ((major_axis / minor_axis) < relay_balanced_axis_ratio_threshold_) {
      return false;
    }

    geometry_msgs::Vector3 lhs_velocity;
    geometry_msgs::Vector3 rhs_velocity;
    if (!computeAverageVelocityForAgents(ordered_components[0], &lhs_velocity) ||
        !computeAverageVelocityForAgents(ordered_components[1], &rhs_velocity)) {
      return false;
    }

    geometry_msgs::Point midpoint;
    midpoint.x = 0.5 * (lhs_centroid.x + rhs_centroid.x);
    midpoint.y = 0.5 * (lhs_centroid.y + rhs_centroid.y);
    midpoint.z = 0.5 * (lhs_centroid.z + rhs_centroid.z);

    const auto outward_progress = [&midpoint](
        const geometry_msgs::Point& centroid, const geometry_msgs::Vector3& velocity) {
      const double cx = centroid.x - midpoint.x;
      const double cy = centroid.y - midpoint.y;
      const double cz = centroid.z - midpoint.z;
      const double cnorm = std::sqrt(cx * cx + cy * cy + cz * cz);
      if (cnorm < 1e-3) {
        return 0.0;
      }
      return (velocity.x * cx + velocity.y * cy + velocity.z * cz) / cnorm;
    };

    const double lhs_outward = outward_progress(lhs_centroid, lhs_velocity);
    const double rhs_outward = outward_progress(rhs_centroid, rhs_velocity);
    return lhs_outward >= relay_balanced_outward_velocity_threshold_ &&
        rhs_outward >= relay_balanced_outward_velocity_threshold_;
  }

  int selectRelayAgent(const geometry_msgs::Point& anchor, const ros::Time& now) const {
    std::unordered_map<int, int> component_sizes;
    for (const auto& pair : latest_components_) {
      ++component_sizes[pair.second];
    }

    double best_score = std::numeric_limits<double>::infinity();
    int best_agent_id = -1;
    double fallback_score = std::numeric_limits<double>::infinity();
    int fallback_agent_id = -1;
    const bool skip_recent_failed_agent =
        relay_failed_agent_cooldown_ > 0.0 && last_failed_relay_agent_id_ > 0 &&
        !relay_last_failed_time_.isZero() &&
        (now - relay_last_failed_time_).toSec() < relay_failed_agent_cooldown_;

    for (const auto& pair : agent_cache_) {
      const int racer_id = pair.first;
      const double anchor_distance = std::sqrt(SquaredDistance(pair.second.position, anchor));
      const double speed = std::sqrt(
          pair.second.velocity.x * pair.second.velocity.x +
          pair.second.velocity.y * pair.second.velocity.y +
          pair.second.velocity.z * pair.second.velocity.z);
      const double backtrack_penalty = computeBacktrackPenalty(pair.second, anchor);
      int component_size = 1;
      const auto component_it = latest_components_.find(racer_id);
      if (component_it != latest_components_.end()) {
        const auto component_size_it = component_sizes.find(component_it->second);
        if (component_size_it != component_sizes.end()) {
          component_size = std::max(1, component_size_it->second);
        }
      }
      double cumulative_occupancy_sec = 0.0;
      const auto occupancy_it = relay_total_occupancy_by_agent_.find(racer_id);
      if (occupancy_it != relay_total_occupancy_by_agent_.end()) {
        cumulative_occupancy_sec = occupancy_it->second;
      }
      const double score = anchor_distance +
          relay_candidate_speed_penalty_gain_ * speed +
          relay_candidate_reuse_penalty_gain_ * cumulative_occupancy_sec +
          relay_candidate_small_component_penalty_gain_ / static_cast<double>(component_size) +
          relay_candidate_backtrack_penalty_gain_ * backtrack_penalty;

      if (skip_recent_failed_agent && racer_id == last_failed_relay_agent_id_) {
        if (score < fallback_score) {
          fallback_score = score;
          fallback_agent_id = racer_id;
        }
        continue;
      }
      if (score < best_score) {
        best_score = score;
        best_agent_id = racer_id;
      }
    }

    if (best_agent_id > 0) {
      return best_agent_id;
    }
    return fallback_agent_id;
  }

  void sanitizeRelayAnchor(geometry_msgs::Point* anchor) const {
    if (anchor == nullptr) {
      return;
    }
    const geometry_msgs::Point raw_anchor = *anchor;
    if (relay_target_use_fixed_z_) {
      anchor->z = relay_target_fixed_z_;
    }
    anchor->z = std::max(relay_target_min_z_, std::min(relay_target_max_z_, anchor->z));

    bool adjusted = false;
    for (int pass = 0; pass < 8; ++pass) {
      bool moved = false;
      for (const auto& obstacle : relay_obstacles_) {
        if (ProjectPointOutOfObstacle(obstacle, relay_target_obstacle_clearance_, anchor)) {
          moved = true;
          adjusted = true;
        }
      }
      if (!moved) {
        break;
      }
    }

    if (adjusted) {
      ROS_WARN_STREAM_THROTTLE(1.0,
          "[RelayCtl] shifted relay target out of obstacle from (" << raw_anchor.x << ", "
          << raw_anchor.y << ", " << raw_anchor.z << ") to (" << anchor->x << ", "
          << anchor->y << ", " << anchor->z << ")");
    }
  }

  geometry_msgs::Point computeRelayAnchor() const {
    geometry_msgs::Point anchor;
    anchor.x = 0.0;
    anchor.y = 0.0;
    anchor.z = 0.0;

    const auto ordered_components = orderedComponentsBySize();
    if (ordered_components.size() >= 3) {
      // Score candidates against every component instead of only the largest two.
      std::vector<geometry_msgs::Point> candidates;
      geometry_msgs::Point weighted_centroid;
      if (computeWeightedCentroidForComponents(ordered_components, &weighted_centroid)) {
        candidates.push_back(weighted_centroid);
      }

      for (size_t i = 0; i < ordered_components.size(); ++i) {
        for (size_t j = i + 1; j < ordered_components.size(); ++j) {
          geometry_msgs::Point candidate;
          bool found_candidate = false;
          if (relay_anchor_use_nearest_pair_midpoint_) {
            found_candidate =
                computeNearestBridgeMidpoint(ordered_components[i], ordered_components[j], &candidate);
          }
          if (!found_candidate) {
            geometry_msgs::Point lhs;
            geometry_msgs::Point rhs;
            if (computeCentroidForAgents(ordered_components[i], &lhs) &&
                computeCentroidForAgents(ordered_components[j], &rhs)) {
              candidate.x = 0.5 * (lhs.x + rhs.x);
              candidate.y = 0.5 * (lhs.y + rhs.y);
              candidate.z = 0.5 * (lhs.z + rhs.z);
              found_candidate = true;
            }
          }
          if (found_candidate) {
            candidates.push_back(candidate);
          }
        }
      }

      double best_score = std::numeric_limits<double>::infinity();
      bool found_anchor = false;
      for (const auto& candidate : candidates) {
        const double score = scoreAnchorAgainstComponents(candidate, ordered_components);
        if (score >= best_score) {
          continue;
        }
        best_score = score;
        anchor = candidate;
        found_anchor = true;
      }

      if (found_anchor) {
        sanitizeRelayAnchor(&anchor);
        return anchor;
      }
    }

    if (ordered_components.size() >= 2) {
      if (relay_anchor_use_nearest_pair_midpoint_ &&
          computeNearestBridgeMidpoint(ordered_components[0], ordered_components[1], &anchor)) {
        sanitizeRelayAnchor(&anchor);
        return anchor;
      }

      geometry_msgs::Point lhs;
      geometry_msgs::Point rhs;
      if (computeCentroidForAgents(ordered_components[0], &lhs) &&
          computeCentroidForAgents(ordered_components[1], &rhs)) {
        anchor.x = 0.5 * (lhs.x + rhs.x);
        anchor.y = 0.5 * (lhs.y + rhs.y);
        anchor.z = 0.5 * (lhs.z + rhs.z);
        sanitizeRelayAnchor(&anchor);
        return anchor;
      }
    }

    std::vector<int> fallback_ids;
    for (const auto& pair : agent_cache_) {
      fallback_ids.push_back(pair.first);
    }
    computeCentroidForAgents(fallback_ids, &anchor);
    sanitizeRelayAnchor(&anchor);
    return anchor;
  }

  void publishSwarmCommState(const ros::Time& now) {
    current_comm_state_.header.stamp = now;
    swarm_comm_state_pub_.publish(current_comm_state_);
  }

  void publishRelayRoleCommands(const ros::Time& now) {
    for (const int agent_id : current_comm_state_.agent_ids) {
      RelayRoleCmd cmd;
      cmd.header.stamp = now;
      cmd.agent_id = agent_id;
      cmd.role = RelayRoleCmd::ROLE_EXPLORER;
      cmd.valid = false;
      cmd.relay_target = relay_anchor_;
      const double raw_target_z = cmd.relay_target.z;
      sanitizeRelayAnchor(&cmd.relay_target);
      cmd.relay_yaw = 0.0;

      if (relay_active_ && agent_id == relay_agent_id_) {
        cmd.role = RelayRoleCmd::ROLE_RELAY;
        cmd.valid = true;
        if (std::abs(raw_target_z - cmd.relay_target.z) > 1e-6) {
          ROS_WARN_STREAM_THROTTLE(1.0,
              "[RelayCtl] clamped relay target z from " << raw_target_z << " to "
              << cmd.relay_target.z);
        }
        ROS_INFO_STREAM_THROTTLE(1.0,
            "[RelayCtl] publish relay agent=" << cmd.agent_id << " target=("
            << cmd.relay_target.x << ", " << cmd.relay_target.y << ", "
            << cmd.relay_target.z << ")");
      }

      relay_role_pub_.publish(cmd);
    }
  }

  void fillZeroVelocityResponse(const std::vector<dancers_msgs::AgentStruct>& agents,
      dancers_msgs::GetAgentVelocities::Response& res) const {
    res.velocity_headings.velocity_heading_array.clear();
    for (const auto& agent : agents) {
      dancers_msgs::VelocityHeading velocity_heading;
      velocity_heading.agent_id = agent.agent_id;
      velocity_heading.heading = agent.state.heading;
      velocity_heading.velocity.x = 0.0;
      velocity_heading.velocity.y = 0.0;
      velocity_heading.velocity.z = 0.0;
      res.velocity_headings.velocity_heading_array.push_back(velocity_heading);
    }
  }

  bool latestLinkUp(const int src_id, const int dst_id) const {
    const auto it = latest_link_up_.find(DirectedKey(src_id, dst_id));
    return it != latest_link_up_.end() && it->second;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber raw_comm_sub_;
  ros::Subscriber task_metrics_sub_;
  ros::Publisher relay_role_pub_;
  ros::Publisher swarm_comm_state_pub_;
  ros::ServiceServer get_agent_velocities_srv_;

  double relay_trigger_age_threshold_;
  double relay_link_exit_age_threshold_;
  double relay_trigger_persist_time_;
  double relay_recover_time_;
  double relay_min_hold_time_;
  double relay_reenter_cooldown_;
  double relay_startup_grace_period_;
  double relay_max_occupancy_ratio_;
  double relay_no_improvement_timeout_;
  double relay_max_active_duration_;
  double relay_failed_agent_cooldown_;
  double relay_endgame_completion_ratio_threshold_ = 0.99;
  int relay_endgame_frontier_cell_threshold_ = 1200;
  int relay_endgame_frontier_cluster_threshold_ = 3;
  int relay_endgame_active_grid_threshold_ = 4;
  double relay_candidate_speed_penalty_gain_ = 4.0;
  double relay_candidate_reuse_penalty_gain_ = 0.08;
  double relay_candidate_small_component_penalty_gain_ = 6.0;
  double relay_candidate_backtrack_penalty_gain_ = 8.0;
  bool relay_suppress_balanced_split_ = true;
  int relay_balanced_min_component_size_ = 2;
  double relay_balanced_size_ratio_threshold_ = 0.75;
  double relay_balanced_min_separation_m_ = 10.0;
  double relay_balanced_axis_ratio_threshold_ = 1.8;
  double relay_balanced_outward_velocity_threshold_ = 0.15;
  bool relay_anchor_use_nearest_pair_midpoint_ = true;
  int relay_frontier_improvement_min_delta_ = 200;
  double relay_frontier_regression_grace_sec_ = 8.0;
  int relay_frontier_regression_abs_threshold_ = 1500;
  double relay_frontier_regression_ratio_threshold_ = 1.25;
  int id_offset_;
  bool prefer_service_input_;
  double service_input_timeout_sec_;
  bool service_input_replaces_graph_;
  bool comm_component_requires_bidirectional_;
  double observed_agent_retention_sec_ = 5.0;
  double agent_cache_timeout_sec_ = 3.0;
  double edge_state_prune_age_sec_ = 5.0;
  bool publish_relay_role_cmds_;
  std::string raw_comm_topic_;
  std::string relay_role_topic_;
  std::string swarm_comm_state_topic_;
  std::string task_metrics_topic_;
  std::string service_name_;

  std::unordered_map<int, AgentCacheEntry> agent_cache_;
  std::unordered_map<int, ros::Time> agent_last_observed_wall_time_;
  std::unordered_map<long long, DirectedEdgeState> directed_edges_;
  std::unordered_map<long long, bool> latest_link_up_;
  std::map<int, int> latest_components_;
  std::set<int> observed_agent_ids_;

  SwarmCommState current_comm_state_;
  bool relay_active_;
  int relay_agent_id_;
  geometry_msgs::Point relay_anchor_;
  ros::Time relay_change_time_;
  ros::Time connected_since_;
  ros::Time fragmented_since_;
  ros::Time relay_exit_time_;
  ros::Time relay_last_improvement_time_;
  ros::Time relay_last_failed_time_;
  ros::Time first_eval_time_;
  int relay_start_num_components_ = 0;
  int relay_best_num_components_ = 0;
  int last_failed_relay_agent_id_ = -1;
  int relay_start_frontier_cell_count_ = -1;
  int relay_best_frontier_cell_count_ = -1;
  bool saw_task_metrics_ = false;
  int unknown_cells_total_ = -1;
  int unknown_cells_peak_ = 0;
  int frontier_cluster_count_metric_ = -1;
  int frontier_cell_count_metric_ = -1;
  int active_grid_count_ = -1;
  double completion_ratio_ = 0.0;
  std::unordered_map<int, double> relay_total_occupancy_by_agent_;
  uint64_t latest_comm_horizon_us_ = 0;
  uint64_t last_service_snapshot_horizon_us_ = 0;
  uint64_t last_raw_snapshot_horizon_us_ = 0;
  ros::Time last_fresh_service_snapshot_wall_time_;
  ros::Time last_raw_topic_update_;
  InputSource last_input_source_;
  RelayPolicyMode relay_policy_mode_;
  bool relay_target_use_fixed_z_;
  double relay_target_fixed_z_;
  double relay_target_min_z_;
  double relay_target_max_z_;
  double relay_target_obstacle_clearance_ = 0.6;
  std::vector<ObstacleBox> relay_obstacles_;
};

}  // namespace relay_racer_integration

int main(int argc, char** argv) {
  ros::init(argc, argv, "racer_comm_controller_node");
  relay_racer_integration::RacerCommControllerNode node;
  ros::spin();
  return 0;
}
