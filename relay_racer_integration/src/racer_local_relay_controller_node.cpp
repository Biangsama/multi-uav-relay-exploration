#include <relay_racer_integration/RelayTaskState.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <XmlRpcValue.h>

#include <relay_racer_integration/RelayPeerState.h>
#include <relay_racer_integration/RelayProposal.h>
#include <relay_racer_integration/RelayRoleCmd.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace relay_racer_integration {
namespace {

enum class RelayPolicyMode {
  kOff,
  kRule,
  kRl,
};

double Clamp01(const double value) {
  return std::max(0.0, std::min(1.0, value));
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

}  // namespace

class RacerLocalRelayControllerNode {
 public:
  RacerLocalRelayControllerNode()
      : nh_(),
        pnh_("~"),
        relay_policy_mode_(RelayPolicyMode::kRule),
        relay_active_(false),
        have_self_odom_(false),
        queue_size_(50),
        control_period_sec_(0.5),
        peer_state_fresh_timeout_sec_(2.0),
        peer_state_stale_retention_sec_(20.0),
        proposal_timeout_sec_(3.0),
        task_state_fresh_timeout_sec_(2.0),
        relay_trigger_persist_time_(1.0),
        relay_winner_persist_time_(2.0),
        relay_recover_time_(2.0),
        relay_min_hold_time_(5.0),
        relay_reenter_cooldown_(2.0),
        relay_startup_grace_period_(25.0),
        relay_max_occupancy_ratio_(0.5),
        relay_no_improvement_timeout_(15.0),
        relay_max_active_duration_(30.0),
        relay_activation_min_stale_count_(1),
        relay_activation_score_margin_(0.0),
        relay_activation_min_votes_(2),
        relay_handoff_persist_time_(4.0),
        relay_handoff_min_vote_margin_(1),
        relay_candidate_speed_penalty_gain_(6.0),
        relay_candidate_reuse_penalty_gain_(0.15),
        relay_candidate_grid_load_penalty_gain_(3.0),
        relay_candidate_backtrack_penalty_gain_(8.0),
        relay_candidate_recent_safety_penalty_gain_(25.0),
        relay_candidate_recent_safety_timeout_sec_(8.0),
        relay_candidate_recent_plan_fail_penalty_gain_(18.0),
        relay_candidate_recent_plan_fail_timeout_sec_(8.0),
        relay_candidate_recent_plan_success_penalty_gain_(22.0),
        relay_candidate_recent_plan_success_timeout_sec_(10.0),
        relay_candidate_consecutive_plan_fail_penalty_gain_(4.0),
        relay_suppress_balanced_split_(true),
        relay_balanced_min_component_size_(2),
        relay_balanced_size_ratio_threshold_(0.75),
        relay_balanced_min_separation_m_(10.0),
        relay_balanced_axis_ratio_threshold_(1.8),
        relay_balanced_outward_velocity_threshold_(0.15),
        relay_anchor_use_nearest_pair_midpoint_(true),
        relay_target_use_fixed_z_(true),
        relay_target_fixed_z_(1.0),
        relay_target_min_z_(0.9),
        relay_target_max_z_(1.3) {
    pnh_.param("self_id", self_id_, 1);
    pnh_.param("queue_size", queue_size_, 50);
    pnh_.param("control_period_sec", control_period_sec_, 0.5);
    pnh_.param("peer_state_fresh_timeout_sec", peer_state_fresh_timeout_sec_, 2.0);
    pnh_.param("peer_state_stale_retention_sec", peer_state_stale_retention_sec_, 20.0);
    pnh_.param("proposal_timeout_sec", proposal_timeout_sec_, 3.0);
    pnh_.param("task_state_fresh_timeout_sec", task_state_fresh_timeout_sec_, 2.0);
    pnh_.param("relay_trigger_persist_time", relay_trigger_persist_time_, 1.0);
    pnh_.param("relay_winner_persist_time", relay_winner_persist_time_, 2.0);
    pnh_.param("relay_recover_time", relay_recover_time_, 2.0);
    pnh_.param("relay_min_hold_time", relay_min_hold_time_, 5.0);
    pnh_.param("relay_reenter_cooldown", relay_reenter_cooldown_, 2.0);
    pnh_.param("relay_startup_grace_period", relay_startup_grace_period_, 25.0);
    pnh_.param("relay_max_occupancy_ratio", relay_max_occupancy_ratio_, 0.5);
    pnh_.param("relay_no_improvement_timeout", relay_no_improvement_timeout_, 15.0);
    pnh_.param("relay_max_active_duration", relay_max_active_duration_, 30.0);
    pnh_.param("relay_activation_min_stale_count", relay_activation_min_stale_count_, 1);
    pnh_.param("relay_activation_score_margin", relay_activation_score_margin_, 0.0);
    pnh_.param("relay_activation_min_votes", relay_activation_min_votes_, 2);
    pnh_.param("relay_handoff_persist_time", relay_handoff_persist_time_, 4.0);
    pnh_.param("relay_handoff_min_vote_margin", relay_handoff_min_vote_margin_, 1);
    pnh_.param("relay_candidate_speed_penalty_gain", relay_candidate_speed_penalty_gain_, 6.0);
    pnh_.param("relay_candidate_reuse_penalty_gain", relay_candidate_reuse_penalty_gain_, 0.15);
    pnh_.param("relay_candidate_grid_load_penalty_gain", relay_candidate_grid_load_penalty_gain_, 3.0);
    pnh_.param("relay_candidate_backtrack_penalty_gain", relay_candidate_backtrack_penalty_gain_, 8.0);
    pnh_.param("relay_candidate_recent_safety_penalty_gain", relay_candidate_recent_safety_penalty_gain_, 25.0);
    pnh_.param("relay_candidate_recent_safety_timeout_sec", relay_candidate_recent_safety_timeout_sec_, 8.0);
    pnh_.param("relay_candidate_recent_plan_fail_penalty_gain", relay_candidate_recent_plan_fail_penalty_gain_, 18.0);
    pnh_.param("relay_candidate_recent_plan_fail_timeout_sec", relay_candidate_recent_plan_fail_timeout_sec_, 8.0);
    pnh_.param("relay_candidate_recent_plan_success_penalty_gain", relay_candidate_recent_plan_success_penalty_gain_, 22.0);
    pnh_.param("relay_candidate_recent_plan_success_timeout_sec", relay_candidate_recent_plan_success_timeout_sec_, 10.0);
    pnh_.param("relay_candidate_consecutive_plan_fail_penalty_gain", relay_candidate_consecutive_plan_fail_penalty_gain_, 4.0);
    pnh_.param("relay_suppress_balanced_split", relay_suppress_balanced_split_, true);
    pnh_.param("relay_balanced_min_component_size", relay_balanced_min_component_size_, 2);
    pnh_.param("relay_balanced_size_ratio_threshold", relay_balanced_size_ratio_threshold_, 0.75);
    pnh_.param("relay_balanced_min_separation_m", relay_balanced_min_separation_m_, 10.0);
    pnh_.param("relay_balanced_axis_ratio_threshold", relay_balanced_axis_ratio_threshold_, 1.8);
    pnh_.param("relay_balanced_outward_velocity_threshold", relay_balanced_outward_velocity_threshold_, 0.15);
    pnh_.param("relay_anchor_use_nearest_pair_midpoint", relay_anchor_use_nearest_pair_midpoint_, true);
    pnh_.param("relay_target_use_fixed_z", relay_target_use_fixed_z_, true);
    pnh_.param("relay_target_fixed_z", relay_target_fixed_z_, 1.0);
    pnh_.param("relay_target_min_z", relay_target_min_z_, 0.9);
    pnh_.param("relay_target_max_z", relay_target_max_z_, 1.3);
    pnh_.param("relay_target_obstacle_clearance", relay_target_obstacle_clearance_, 0.6);
    std::string relay_policy_mode_param = "rule";
    pnh_.param<std::string>("relay_policy_mode", relay_policy_mode_param, "rule");

    loadDroneIds();

    if (relay_target_min_z_ > relay_target_max_z_) {
      std::swap(relay_target_min_z_, relay_target_max_z_);
    }
    relay_target_obstacle_clearance_ = std::max(0.0, relay_target_obstacle_clearance_);
    XmlRpc::XmlRpcValue obstacles_param;
    if (pnh_.getParam("obstacles", obstacles_param)) {
      ParseObstacleBoxes(obstacles_param, &relay_obstacles_);
      ROS_INFO_STREAM("[LocalRelayCtl " << self_id_ << "] loaded obstacle boxes="
                      << relay_obstacles_.size()
                      << " relay_target_obstacle_clearance="
                      << relay_target_obstacle_clearance_);
    }
    relay_candidate_speed_penalty_gain_ = std::max(0.0, relay_candidate_speed_penalty_gain_);
    relay_candidate_reuse_penalty_gain_ = std::max(0.0, relay_candidate_reuse_penalty_gain_);
    relay_candidate_grid_load_penalty_gain_ = std::max(0.0, relay_candidate_grid_load_penalty_gain_);
    relay_candidate_backtrack_penalty_gain_ = std::max(0.0, relay_candidate_backtrack_penalty_gain_);
    relay_candidate_recent_safety_penalty_gain_ = std::max(0.0, relay_candidate_recent_safety_penalty_gain_);
    relay_candidate_recent_safety_timeout_sec_ = std::max(0.0, relay_candidate_recent_safety_timeout_sec_);
    relay_candidate_recent_plan_fail_penalty_gain_ = std::max(0.0, relay_candidate_recent_plan_fail_penalty_gain_);
    relay_candidate_recent_plan_fail_timeout_sec_ = std::max(0.0, relay_candidate_recent_plan_fail_timeout_sec_);
    relay_candidate_recent_plan_success_penalty_gain_ = std::max(0.0, relay_candidate_recent_plan_success_penalty_gain_);
    relay_candidate_recent_plan_success_timeout_sec_ = std::max(0.0, relay_candidate_recent_plan_success_timeout_sec_);
    relay_candidate_consecutive_plan_fail_penalty_gain_ = std::max(0.0, relay_candidate_consecutive_plan_fail_penalty_gain_);
    relay_balanced_min_component_size_ = std::max(1, relay_balanced_min_component_size_);
    relay_balanced_size_ratio_threshold_ = Clamp01(relay_balanced_size_ratio_threshold_);
    relay_balanced_min_separation_m_ = std::max(0.0, relay_balanced_min_separation_m_);
    relay_balanced_axis_ratio_threshold_ = std::max(1.0, relay_balanced_axis_ratio_threshold_);
    relay_balanced_outward_velocity_threshold_ = std::max(0.0, relay_balanced_outward_velocity_threshold_);

    std::string normalized_policy_mode;
    if (!TryParseRelayPolicyMode(relay_policy_mode_param, &relay_policy_mode_, &normalized_policy_mode)) {
      relay_policy_mode_ = RelayPolicyMode::kRule;
      ROS_WARN_STREAM("[LocalRelayCtl " << self_id_ << "] unknown relay_policy_mode=\""
                      << relay_policy_mode_param << "\". Falling back to rule.");
    }

    odom_topic_ = paramOrDefault("odom_topic", "/relay_odom_" + std::to_string(self_id_));
    peer_state_send_topic_ = paramOrDefault(
        "peer_state_send_topic", "/relay_integration/relay_peer_state_send_" + std::to_string(self_id_));
    peer_state_recv_topic_ = paramOrDefault(
        "peer_state_recv_topic", "/relay_integration/relay_peer_state_recv_" + std::to_string(self_id_));
    proposal_send_topic_ = paramOrDefault(
        "proposal_send_topic", "/relay_integration/relay_proposal_send_" + std::to_string(self_id_));
    proposal_recv_topic_ = paramOrDefault(
        "proposal_recv_topic", "/relay_integration/relay_proposal_recv_" + std::to_string(self_id_));
    relay_role_topic_ = paramOrDefault("relay_role_topic", "/relay_integration/relay_role_cmd");
    self_drone_state_topic_ = paramOrDefault(
        "self_drone_state_topic", "/relay_integration/relay_task_state_send_" + std::to_string(self_id_));
    peer_drone_state_topic_ = paramOrDefault(
        "peer_drone_state_topic", "/relay_integration/relay_task_state_recv_" + std::to_string(self_id_));

    odom_sub_ = nh_.subscribe(odom_topic_, queue_size_, &RacerLocalRelayControllerNode::odomCallback, this);
    peer_state_sub_ = nh_.subscribe(peer_state_recv_topic_, queue_size_,
                                    &RacerLocalRelayControllerNode::peerStateCallback, this);
    proposal_sub_ = nh_.subscribe(proposal_recv_topic_, queue_size_,
                                  &RacerLocalRelayControllerNode::proposalCallback, this);
    self_drone_state_sub_ = nh_.subscribe(
        self_drone_state_topic_, queue_size_, &RacerLocalRelayControllerNode::selfDroneStateCallback, this);
    peer_drone_state_sub_ = nh_.subscribe(
        peer_drone_state_topic_, queue_size_, &RacerLocalRelayControllerNode::peerDroneStateCallback, this);
    peer_state_pub_ = nh_.advertise<RelayPeerState>(peer_state_send_topic_, queue_size_);
    proposal_pub_ = nh_.advertise<RelayProposal>(proposal_send_topic_, queue_size_);
    relay_role_pub_ = nh_.advertise<RelayRoleCmd>(relay_role_topic_, queue_size_);
    tick_timer_ = nh_.createTimer(ros::Duration(control_period_sec_),
                                  &RacerLocalRelayControllerNode::tick, this);

    relay_anchor_.x = 0.0;
    relay_anchor_.y = 0.0;
    relay_anchor_.z = relay_target_fixed_z_;

    ROS_INFO_STREAM("[LocalRelayCtl " << self_id_ << "] policy=" << RelayPolicyModeName(relay_policy_mode_)
                    << " odom_topic=" << odom_topic_
                    << " peer_state_send_topic=" << peer_state_send_topic_
                    << " peer_state_recv_topic=" << peer_state_recv_topic_
                    << " proposal_send_topic=" << proposal_send_topic_
                    << " proposal_recv_topic=" << proposal_recv_topic_
                    << " relay_role_topic=" << relay_role_topic_
                    << " self_drone_state_topic=" << self_drone_state_topic_
                    << " peer_drone_state_topic=" << peer_drone_state_topic_);
  }

 private:
  struct PeerStateEntry {
    geometry_msgs::Point position;
    geometry_msgs::Vector3 velocity;
    double cumulative_relay_occupancy_sec = 0.0;
    ros::Time last_rx_time;
    bool valid = false;
  };

  struct TaskStateEntry {
    int relay_role = 0;
    bool task_assignable = true;
    size_t grid_count = 0;
    double last_safety_replan_stamp = 0.0;
    double last_plan_fail_stamp = 0.0;
    double last_plan_success_stamp = 0.0;
    int consecutive_plan_failures = 0;
    ros::Time last_rx_time;
    bool valid = false;
  };

  struct ProposalView {
    bool fragmented_observed = false;
    bool valid = false;
    int candidate_id = -1;
    geometry_msgs::Point relay_target;
    double score = 0.0;
    int fresh_peer_count = 0;
    int stale_peer_count = 0;
  };

  struct ProposalEntry {
    bool valid = false;
    int candidate_id = -1;
    double score = 0.0;
    geometry_msgs::Point relay_target;
    int fresh_peer_count = 0;
    int stale_peer_count = 0;
    int support_count = 0;
    ros::Time stamp;
  };

  std::string paramOrDefault(const std::string& key, const std::string& fallback) {
    std::string value = fallback;
    pnh_.param<std::string>(key, value, fallback);
    return value;
  }

  void loadDroneIds() {
    XmlRpc::XmlRpcValue drone_ids_param;
    if (pnh_.getParam("drone_ids", drone_ids_param) &&
        drone_ids_param.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      for (int i = 0; i < drone_ids_param.size(); ++i) {
        drone_ids_.push_back(static_cast<int>(drone_ids_param[i]));
      }
    }

    if (!drone_ids_.empty()) {
      return;
    }

    int drone_num = 1;
    int first_drone_id = 1;
    pnh_.param("drone_num", drone_num, 1);
    pnh_.param("first_drone_id", first_drone_id, 1);
    for (int i = 0; i < drone_num; ++i) {
      drone_ids_.push_back(first_drone_id + i);
    }
  }

  RelayPolicyMode effectiveRelayPolicyMode() const {
    if (relay_policy_mode_ == RelayPolicyMode::kRl) {
      ROS_WARN_THROTTLE(5.0,
          "[LocalRelayCtl %d] relay_policy_mode=rl requested, but local RL policy is not integrated yet. Falling back to rule.",
          self_id_);
      return RelayPolicyMode::kRule;
    }
    return relay_policy_mode_;
  }

  void odomCallback(const nav_msgs::OdometryConstPtr& msg) {
    self_position_ = msg->pose.pose.position;
    self_velocity_ = msg->twist.twist.linear;
    self_odom_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    have_self_odom_ = true;
  }

  void peerStateCallback(const RelayPeerStateConstPtr& msg) {
    if (!msg->valid || msg->agent_id == self_id_) {
      return;
    }
    PeerStateEntry& entry = peer_cache_[msg->agent_id];
    entry.position = msg->position;
    entry.velocity = msg->velocity;
    entry.cumulative_relay_occupancy_sec = msg->cumulative_relay_occupancy_sec;
    entry.last_rx_time = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    entry.valid = true;
  }

  void proposalCallback(const RelayProposalConstPtr& msg) {
    if (msg->proposer_id == self_id_) {
      return;
    }
    ProposalEntry& entry = proposal_cache_[msg->proposer_id];
    entry.valid = msg->valid;
    entry.candidate_id = msg->candidate_id;
    entry.score = msg->score;
    entry.relay_target = msg->relay_target;
    entry.fresh_peer_count = msg->fresh_peer_count;
    entry.stale_peer_count = msg->stale_peer_count;
    entry.support_count = 1;
    entry.stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
  }

  void selfDroneStateCallback(const relay_racer_integration::RelayTaskStateConstPtr& msg) {
    if (msg->agent_id != self_id_) {
      return;
    }
    updateTaskState(*msg, &self_task_state_);
  }

  void peerDroneStateCallback(const relay_racer_integration::RelayTaskStateConstPtr& msg) {
    if (msg->agent_id == self_id_) {
      return;
    }
    TaskStateEntry& entry = peer_task_cache_[msg->agent_id];
    updateTaskState(*msg, &entry);
  }

  void tick(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    if (first_tick_time_.isZero()) {
      first_tick_time_ = now;
    }

    pruneCaches(now);
    if (have_self_odom_) {
      publishSelfPeerState(now);
    }

    ProposalView self_view = computeSelfProposal(now);
    updateFragmentationState(self_view.fragmented_observed, now);
    publishSelfProposal(self_view, now);

    const RelayPolicyMode policy = effectiveRelayPolicyMode();
    if (policy == RelayPolicyMode::kOff || !have_self_odom_) {
      if (relay_active_) {
        deactivateRelay(now, policy == RelayPolicyMode::kOff ? "policy_off" : "no_self_odom");
      }
      publishRole(now, false, relay_anchor_);
      return;
    }

    int winner_id = -1;
    ProposalEntry winner;
    selectWinningProposal(self_view, now, &winner_id, &winner);
    updateWinnerHistory(winner_id, now);

    if (relay_active_) {
      if (self_view.valid) {
        relay_anchor_ = self_view.relay_target;
      }
      if (winner_id == self_id_ && winner.valid) {
        relay_active_support_count_ = std::max(1, winner.support_count);
      }
      noteRelayProgress(self_view, now);
      std::string exit_reason;
      if (shouldExitRelay(self_view, winner_id, winner, now, &exit_reason)) {
        deactivateRelay(now, exit_reason);
      }
    }

    if (!relay_active_ && shouldActivateRelay(self_view, winner_id, now)) {
      activateRelay(self_view, winner, now);
    }

    publishRole(now, relay_active_, relay_active_ ? relay_anchor_ : self_view.relay_target);
  }

  void pruneCaches(const ros::Time& now) {
    for (auto it = peer_cache_.begin(); it != peer_cache_.end();) {
      const double age = (now - it->second.last_rx_time).toSec();
      if (!it->second.valid || age > peer_state_stale_retention_sec_) {
        it = peer_cache_.erase(it);
      } else {
        ++it;
      }
    }
    for (auto it = proposal_cache_.begin(); it != proposal_cache_.end();) {
      const double age = (now - it->second.stamp).toSec();
      if (age > proposal_timeout_sec_) {
        it = proposal_cache_.erase(it);
      } else {
        ++it;
      }
    }
    for (auto it = peer_task_cache_.begin(); it != peer_task_cache_.end();) {
      const double age = (now - it->second.last_rx_time).toSec();
      if (!it->second.valid || age > peer_state_stale_retention_sec_) {
        it = peer_task_cache_.erase(it);
      } else {
        ++it;
      }
    }
  }

  void publishSelfPeerState(const ros::Time& now) {
    RelayPeerState msg;
    msg.header.stamp = now;
    msg.agent_id = self_id_;
    msg.position = self_position_;
    msg.velocity = self_velocity_;
    msg.cumulative_relay_occupancy_sec = currentRelayOccupancySec(now);
    msg.valid = have_self_odom_;
    peer_state_pub_.publish(msg);
  }

  void publishSelfProposal(const ProposalView& view, const ros::Time& now) {
    RelayProposal msg;
    msg.header.stamp = now;
    msg.proposer_id = self_id_;
    msg.candidate_id = view.valid ? view.candidate_id : -1;
    msg.valid = view.valid;
    msg.score = view.valid ? view.score : 0.0;
    msg.relay_target = view.relay_target;
    msg.fresh_peer_count = view.fresh_peer_count;
    msg.stale_peer_count = view.stale_peer_count;
    proposal_pub_.publish(msg);
  }

  void publishRole(const ros::Time& now, const bool active, geometry_msgs::Point target) {
    sanitizeRelayAnchor(&target);
    RelayRoleCmd cmd;
    cmd.header.stamp = now;
    cmd.agent_id = self_id_;
    cmd.role = active ? RelayRoleCmd::ROLE_RELAY : RelayRoleCmd::ROLE_EXPLORER;
    cmd.valid = active;
    cmd.relay_target = target;
    cmd.relay_yaw = 0.0f;
    relay_role_pub_.publish(cmd);
  }

  void updateFragmentationState(const bool fragmented, const ros::Time& now) {
    if (fragmented) {
      connected_since_ = ros::Time(0);
      if (fragmented_since_.isZero()) {
        fragmented_since_ = now;
      }
    } else {
      fragmented_since_ = ros::Time(0);
      if (connected_since_.isZero()) {
        connected_since_ = now;
      }
    }
  }

  ProposalView computeSelfProposal(const ros::Time& now) const {
    ProposalView view;
    view.relay_target = relay_anchor_;
    if (!have_self_odom_) {
      return view;
    }

    std::vector<int> fresh_ids;
    std::vector<int> stale_ids;
    for (const int drone_id : drone_ids_) {
      if (drone_id == self_id_) {
        continue;
      }
      const auto it = peer_cache_.find(drone_id);
      if (it == peer_cache_.end() || !it->second.valid) {
        continue;
      }
      const double age = (now - it->second.last_rx_time).toSec();
      if (age <= peer_state_fresh_timeout_sec_) {
        fresh_ids.push_back(drone_id);
      } else if (age <= peer_state_stale_retention_sec_) {
        stale_ids.push_back(drone_id);
      }
    }

    view.fragmented_observed = !stale_ids.empty();
    view.fresh_peer_count = static_cast<int>(fresh_ids.size());
    view.stale_peer_count = static_cast<int>(stale_ids.size());

    if (effectiveRelayPolicyMode() == RelayPolicyMode::kOff) {
      return view;
    }
    if (!first_tick_time_.isZero() && relay_startup_grace_period_ > 0.0 &&
        (now - first_tick_time_).toSec() < relay_startup_grace_period_) {
      return view;
    }
    if (stale_ids.empty()) {
      return view;
    }
    if (static_cast<int>(stale_ids.size()) < relay_activation_min_stale_count_) {
      return view;
    }
    const double occupancy_ratio = 1.0 / static_cast<double>(std::max<size_t>(1, drone_ids_.size()));
    if (occupancy_ratio > relay_max_occupancy_ratio_) {
      return view;
    }
    if (relay_suppress_balanced_split_ && shouldSuppressBalancedSplit(fresh_ids, stale_ids)) {
      ROS_INFO_STREAM_THROTTLE(2.0, "[LocalRelayCtl " << self_id_
                              << "] suppress relay for balanced independent local split");
      return view;
    }
    if (!shouldCoordinateLocalSide(fresh_ids, stale_ids)) {
      return view;
    }

    geometry_msgs::Point anchor;
    if (!computeRelayAnchor(fresh_ids, stale_ids, &anchor)) {
      return view;
    }

    std::vector<int> local_ids = fresh_ids;
    local_ids.push_back(self_id_);
    int local_best_id = -1;
    double local_best_score = std::numeric_limits<double>::infinity();
    if (!computeBestCandidate(local_ids, anchor, now, &local_best_id, &local_best_score)) {
      return view;
    }

    view.valid = true;
    view.candidate_id = local_best_id;
    view.relay_target = anchor;
    view.score = local_best_score;
    return view;
  }

  bool shouldActivateRelay(const ProposalView& view, const int winner_id, const ros::Time& now) const {
    if (!view.valid || winner_id != self_id_) {
      return false;
    }
    if (fragmented_since_.isZero()) {
      return false;
    }
    if ((now - fragmented_since_).toSec() < relay_trigger_persist_time_) {
      return false;
    }
    if (winner_id != stable_winner_id_ || stable_winner_since_.isZero() ||
        (now - stable_winner_since_).toSec() < relay_winner_persist_time_) {
      return false;
    }
    if (!relay_exit_time_.isZero() && (now - relay_exit_time_).toSec() < relay_reenter_cooldown_) {
      return false;
    }
    return true;
  }

  void activateRelay(const ProposalView& view, const ProposalEntry& winner, const ros::Time& now) {
    relay_active_ = true;
    relay_anchor_ = view.relay_target;
    relay_change_time_ = now;
    relay_last_improvement_time_ = now;
    relay_best_stale_peer_count_ = view.stale_peer_count;
    relay_best_fresh_peer_count_ = view.fresh_peer_count;
    relay_active_support_count_ = std::max(1, winner.support_count);
    ROS_INFO_STREAM("[LocalRelayCtl " << self_id_ << "] activate relay target=("
                    << relay_anchor_.x << ", " << relay_anchor_.y << ", " << relay_anchor_.z
                    << ") stale_peer_count=" << view.stale_peer_count
                    << " fresh_peer_count=" << view.fresh_peer_count
                    << " score=" << view.score
                    << " support_count=" << relay_active_support_count_);
  }

  void deactivateRelay(const ros::Time& now, const std::string& reason) {
    const double duration_sec = relay_change_time_.isZero() ? 0.0 : (now - relay_change_time_).toSec();
    cumulative_relay_occupancy_sec_ += std::max(0.0, duration_sec);
    relay_active_ = false;
    relay_change_time_ = ros::Time(0);
    relay_exit_time_ = now;
    relay_last_improvement_time_ = ros::Time(0);
    relay_best_stale_peer_count_ = std::numeric_limits<int>::max();
    relay_best_fresh_peer_count_ = 0;
    relay_active_support_count_ = 0;
    ROS_INFO_STREAM("[LocalRelayCtl " << self_id_ << "] deactivate relay reason=" << reason
                    << " duration_sec=" << duration_sec
                    << " cumulative_occupancy_sec=" << cumulative_relay_occupancy_sec_);
  }

  void noteRelayProgress(const ProposalView& view, const ros::Time& now) {
    if (!relay_active_) {
      return;
    }
    bool improved = false;
    if (view.stale_peer_count < relay_best_stale_peer_count_) {
      relay_best_stale_peer_count_ = view.stale_peer_count;
      improved = true;
    }
    if (view.fresh_peer_count > relay_best_fresh_peer_count_) {
      relay_best_fresh_peer_count_ = view.fresh_peer_count;
      improved = true;
    }
    if (improved) {
      relay_last_improvement_time_ = now;
      ROS_INFO_STREAM("[LocalRelayCtl " << self_id_ << "] relay improved stale_peer_count="
                      << relay_best_stale_peer_count_
                      << " fresh_peer_count=" << relay_best_fresh_peer_count_);
    }
  }

  bool shouldExitRelay(
      const ProposalView& view,
      const int winner_id,
      const ProposalEntry& winner,
      const ros::Time& now,
      std::string* reason) const {
    if (!relay_active_) {
      return false;
    }
    const double active_duration_sec = (now - relay_change_time_).toSec();
    if (active_duration_sec < relay_min_hold_time_) {
      return false;
    }
    if (!view.fragmented_observed && !connected_since_.isZero() &&
        (now - connected_since_).toSec() >= relay_recover_time_) {
      if (reason != nullptr) {
        *reason = "recovered";
      }
      return true;
    }
    if (!view.valid) {
      if (reason != nullptr) {
        *reason = "no_local_view";
      }
      return true;
    }
    if (winner_id > 0 && winner_id != self_id_) {
      const bool stable_handoff = (winner_id == stable_winner_id_) && !stable_winner_since_.isZero() &&
                                  ((now - stable_winner_since_).toSec() >= relay_handoff_persist_time_);
      const int required_support =
          std::max(1, relay_active_support_count_) + std::max(0, relay_handoff_min_vote_margin_);
      if (stable_handoff && winner.valid && winner.support_count >= required_support) {
        if (reason != nullptr) {
          *reason = "lost_election";
        }
        return true;
      }
    }
    if (relay_no_improvement_timeout_ > 0.0 && !relay_last_improvement_time_.isZero() &&
        (now - relay_last_improvement_time_).toSec() >= relay_no_improvement_timeout_ &&
        view.stale_peer_count >= relay_best_stale_peer_count_) {
      if (reason != nullptr) {
        *reason = "no_improvement";
      }
      return true;
    }
    if (relay_max_active_duration_ > 0.0 && active_duration_sec >= relay_max_active_duration_) {
      if (reason != nullptr) {
        *reason = "max_active_duration";
      }
      return true;
    }
    return false;
  }

  void selectWinningProposal(
      const ProposalView& self_view,
      const ros::Time& now,
      int* winner_id,
      ProposalEntry* winner) const {
    struct AggregateEntry {
      int candidate_id = -1;
      int votes = 0;
      double score_sum = 0.0;
      double best_score = std::numeric_limits<double>::infinity();
      geometry_msgs::Point relay_target;
      int fresh_peer_count = 0;
      int stale_peer_count = 0;
      int best_proposer_id = std::numeric_limits<int>::max();
      ros::Time stamp;
    };

    std::map<int, AggregateEntry> aggregate_by_candidate;
    const auto consider = [&](const int proposer_id, const ProposalEntry& candidate) {
      if (!candidate.valid || candidate.candidate_id <= 0) {
        return;
      }
      AggregateEntry& aggregate = aggregate_by_candidate[candidate.candidate_id];
      aggregate.candidate_id = candidate.candidate_id;
      aggregate.votes += 1;
      aggregate.score_sum += candidate.score;
      if (candidate.score < aggregate.best_score - 1e-6 ||
          (std::abs(candidate.score - aggregate.best_score) <= 1e-6 && proposer_id < aggregate.best_proposer_id)) {
        aggregate.best_score = candidate.score;
        aggregate.relay_target = candidate.relay_target;
        aggregate.fresh_peer_count = candidate.fresh_peer_count;
        aggregate.stale_peer_count = candidate.stale_peer_count;
        aggregate.best_proposer_id = proposer_id;
        aggregate.stamp = candidate.stamp;
      }
    };

    if (self_view.valid) {
      ProposalEntry self_entry;
      self_entry.valid = true;
      self_entry.candidate_id = self_view.candidate_id;
      self_entry.score = self_view.score;
      self_entry.relay_target = self_view.relay_target;
      self_entry.fresh_peer_count = self_view.fresh_peer_count;
      self_entry.stale_peer_count = self_view.stale_peer_count;
      self_entry.support_count = 1;
      self_entry.stamp = now;
      consider(self_id_, self_entry);
    }
    for (const auto& pair : proposal_cache_) {
      const double age = (now - pair.second.stamp).toSec();
      if (age > proposal_timeout_sec_) {
        continue;
      }
      consider(pair.first, pair.second);
    }

    int best_candidate_id = -1;
    ProposalEntry best;
    double best_average_score = std::numeric_limits<double>::infinity();
    for (const auto& pair : aggregate_by_candidate) {
      const AggregateEntry& aggregate = pair.second;
      if (aggregate.votes < relay_activation_min_votes_) {
        continue;
      }
      const double average_score = aggregate.score_sum / static_cast<double>(std::max(1, aggregate.votes));
      if (best_candidate_id < 0 || aggregate.votes > best.support_count ||
          (aggregate.votes == best.support_count && average_score < best_average_score - 1e-6) ||
          (aggregate.votes == best.support_count &&
           std::abs(average_score - best_average_score) <= 1e-6 &&
           aggregate.candidate_id < best_candidate_id)) {
        best_candidate_id = aggregate.candidate_id;
        best.valid = true;
        best.candidate_id = aggregate.candidate_id;
        best.score = aggregate.best_score;
        best.relay_target = aggregate.relay_target;
        best.fresh_peer_count = aggregate.fresh_peer_count;
        best.stale_peer_count = aggregate.stale_peer_count;
        best.support_count = aggregate.votes;
        best.stamp = aggregate.stamp;
        best_average_score = average_score;
      }
    }

    if (winner_id != nullptr) {
      *winner_id = best_candidate_id;
    }
    if (winner != nullptr) {
      *winner = best;
    }
  }

  void updateWinnerHistory(const int winner_id, const ros::Time& now) {
    if (winner_id != stable_winner_id_) {
      stable_winner_id_ = winner_id;
      stable_winner_since_ = now;
    }
  }

  double computeBacktrackPenalty(
      const geometry_msgs::Point& position,
      const geometry_msgs::Vector3& velocity,
      const geometry_msgs::Point& anchor) const {
    const double vx = velocity.x;
    const double vy = velocity.y;
    const double vz = velocity.z;
    const double speed = std::sqrt(vx * vx + vy * vy + vz * vz);
    if (speed < 1e-3) {
      return 0.0;
    }

    const double dx = anchor.x - position.x;
    const double dy = anchor.y - position.y;
    const double dz = anchor.z - position.z;
    const double anchor_distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (anchor_distance < 1e-3) {
      return 0.0;
    }

    const double cosine = (vx * dx + vy * dy + vz * dz) / (speed * anchor_distance);
    return std::max(0.0, -cosine) * anchor_distance;
  }

  double currentRelayOccupancySec(const ros::Time& now) const {
    double occupancy = cumulative_relay_occupancy_sec_;
    if (relay_active_ && !relay_change_time_.isZero()) {
      occupancy += std::max(0.0, (now - relay_change_time_).toSec());
    }
    return occupancy;
  }

  bool lookupRelayOccupancy(int agent_id, const ros::Time& now, double* occupancy) const {
    if (occupancy == nullptr) {
      return false;
    }
    if (agent_id == self_id_) {
      *occupancy = currentRelayOccupancySec(now);
      return have_self_odom_;
    }
    const auto it = peer_cache_.find(agent_id);
    if (it == peer_cache_.end() || !it->second.valid) {
      return false;
    }
    *occupancy = it->second.cumulative_relay_occupancy_sec;
    return true;
  }

  bool computeCandidateScore(
      int agent_id, const geometry_msgs::Point& anchor, const ros::Time& now, double* score) const {
    if (score == nullptr) {
      return false;
    }
    geometry_msgs::Point position;
    geometry_msgs::Vector3 velocity;
    double occupancy = 0.0;
    if (!lookupPoint(agent_id, &position) || !lookupVelocity(agent_id, &velocity) ||
        !lookupRelayOccupancy(agent_id, now, &occupancy)) {
      return false;
    }
    double task_penalty = 0.0;
    double recent_safety_penalty = 0.0;
    double recent_plan_fail_penalty = 0.0;
    double recent_plan_success_penalty = 0.0;
    double consecutive_plan_fail_penalty = 0.0;
    TaskStateEntry task_state;
    if (lookupTaskState(agent_id, now, &task_state)) {
      if (!task_state.task_assignable || task_state.relay_role == RelayRoleCmd::ROLE_RELAY) {
        return false;
      }
      task_penalty = relay_candidate_grid_load_penalty_gain_ *
                     static_cast<double>(task_state.grid_count);
      const double now_sec = now.toSec();
      if (relay_candidate_recent_safety_timeout_sec_ > 1e-6 &&
          task_state.last_safety_replan_stamp > 1e-6) {
        const double age = std::max(0.0, now_sec - task_state.last_safety_replan_stamp);
        if (age < relay_candidate_recent_safety_timeout_sec_) {
          recent_safety_penalty = relay_candidate_recent_safety_penalty_gain_ *
              (1.0 - age / relay_candidate_recent_safety_timeout_sec_);
        }
      }
      if (relay_candidate_recent_plan_fail_timeout_sec_ > 1e-6 &&
          task_state.last_plan_fail_stamp > 1e-6) {
        const double age = std::max(0.0, now_sec - task_state.last_plan_fail_stamp);
        if (age < relay_candidate_recent_plan_fail_timeout_sec_) {
          recent_plan_fail_penalty = relay_candidate_recent_plan_fail_penalty_gain_ *
              (1.0 - age / relay_candidate_recent_plan_fail_timeout_sec_);
        }
      }
      if (relay_candidate_recent_plan_success_timeout_sec_ > 1e-6 &&
          task_state.last_plan_success_stamp > 1e-6) {
        const double age = std::max(0.0, now_sec - task_state.last_plan_success_stamp);
        if (age < relay_candidate_recent_plan_success_timeout_sec_) {
          // Keep clearly busy multi-grid explorers on-task instead of reassigning them to relay.
          if (task_state.grid_count > 1) {
            return false;
          }
          recent_plan_success_penalty = relay_candidate_recent_plan_success_penalty_gain_ *
              (1.0 - age / relay_candidate_recent_plan_success_timeout_sec_);
        }
      }
      consecutive_plan_fail_penalty = relay_candidate_consecutive_plan_fail_penalty_gain_ *
          static_cast<double>(task_state.consecutive_plan_failures);
    }
    const double speed = std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y +
                                   velocity.z * velocity.z);
    const double anchor_distance = std::sqrt(SquaredDistance(position, anchor));
    const double backtrack_penalty = computeBacktrackPenalty(position, velocity, anchor);
    *score = anchor_distance +
             relay_candidate_speed_penalty_gain_ * speed +
             relay_candidate_reuse_penalty_gain_ * occupancy +
             relay_candidate_backtrack_penalty_gain_ * backtrack_penalty +
             task_penalty +
             recent_safety_penalty +
             recent_plan_fail_penalty +
             recent_plan_success_penalty +
             consecutive_plan_fail_penalty;
    return true;
  }

  void updateTaskState(const relay_racer_integration::RelayTaskState& msg, TaskStateEntry* entry) {
    if (entry == nullptr) {
      return;
    }
    entry->relay_role = static_cast<int>(msg.relay_role);
    entry->task_assignable = msg.task_assignable;
    entry->grid_count = static_cast<size_t>(std::max(0, msg.grid_count));
    entry->last_safety_replan_stamp = msg.last_safety_replan_stamp;
    entry->last_plan_fail_stamp = msg.last_plan_fail_stamp;
    entry->last_plan_success_stamp = msg.last_plan_success_stamp;
    entry->consecutive_plan_failures = std::max(0, msg.consecutive_plan_failures);
    entry->last_rx_time = msg.stamp > 1e-6 ? ros::Time(msg.stamp) : ros::Time::now();
    entry->valid = true;
  }

  bool lookupTaskState(int agent_id, const ros::Time& now, TaskStateEntry* state) const {
    if (state == nullptr) {
      return false;
    }
    if (agent_id == self_id_) {
      if (!self_task_state_.valid) {
        return false;
      }
      if ((now - self_task_state_.last_rx_time).toSec() > task_state_fresh_timeout_sec_) {
        return false;
      }
      *state = self_task_state_;
      return true;
    }
    const auto it = peer_task_cache_.find(agent_id);
    if (it == peer_task_cache_.end() || !it->second.valid) {
      return false;
    }
    if ((now - it->second.last_rx_time).toSec() > task_state_fresh_timeout_sec_) {
      return false;
    }
    *state = it->second;
    return true;
  }

  bool shouldCoordinateLocalSide(
      const std::vector<int>& fresh_ids, const std::vector<int>& stale_ids) const {
    if (stale_ids.empty()) {
      return true;
    }
    int local_min_id = self_id_;
    for (const int id : fresh_ids) {
      local_min_id = std::min(local_min_id, id);
    }
    int remote_min_id = stale_ids.front();
    for (const int id : stale_ids) {
      remote_min_id = std::min(remote_min_id, id);
    }
    return local_min_id <= remote_min_id;
  }

  bool computeBestCandidate(
      const std::vector<int>& ids, const geometry_msgs::Point& anchor, const ros::Time& now,
      int* best_id, double* best_score) const {
    int candidate_id = -1;
    double candidate_score = std::numeric_limits<double>::infinity();
    for (const int id : ids) {
      double score = 0.0;
      if (!computeCandidateScore(id, anchor, now, &score)) {
        continue;
      }
      if (candidate_id < 0 || score < candidate_score - 1e-6 ||
          (std::abs(score - candidate_score) <= 1e-6 && id < candidate_id)) {
        candidate_id = id;
        candidate_score = score;
      }
    }
    if (candidate_id < 0) {
      return false;
    }
    if (best_id != nullptr) {
      *best_id = candidate_id;
    }
    if (best_score != nullptr) {
      *best_score = candidate_score;
    }
    return true;
  }

  bool lookupPoint(int agent_id, geometry_msgs::Point* point) const {
    if (point == nullptr) {
      return false;
    }
    if (agent_id == self_id_) {
      if (!have_self_odom_) {
        return false;
      }
      *point = self_position_;
      return true;
    }
    const auto it = peer_cache_.find(agent_id);
    if (it == peer_cache_.end() || !it->second.valid) {
      return false;
    }
    *point = it->second.position;
    return true;
  }

  bool lookupVelocity(int agent_id, geometry_msgs::Vector3* velocity) const {
    if (velocity == nullptr) {
      return false;
    }
    if (agent_id == self_id_) {
      if (!have_self_odom_) {
        return false;
      }
      *velocity = self_velocity_;
      return true;
    }
    const auto it = peer_cache_.find(agent_id);
    if (it == peer_cache_.end() || !it->second.valid) {
      return false;
    }
    *velocity = it->second.velocity;
    return true;
  }

  bool computeCentroid(const std::vector<int>& ids, geometry_msgs::Point* point) const {
    if (point == nullptr || ids.empty()) {
      return false;
    }
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    int count = 0;
    for (const int id : ids) {
      geometry_msgs::Point pos;
      if (!lookupPoint(id, &pos)) {
        continue;
      }
      x += pos.x;
      y += pos.y;
      z += pos.z;
      ++count;
    }
    if (count == 0) {
      return false;
    }
    point->x = x / static_cast<double>(count);
    point->y = y / static_cast<double>(count);
    point->z = z / static_cast<double>(count);
    return true;
  }

  bool computeAverageVelocity(const std::vector<int>& ids, geometry_msgs::Vector3* velocity) const {
    if (velocity == nullptr || ids.empty()) {
      return false;
    }
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    int count = 0;
    for (const int id : ids) {
      geometry_msgs::Vector3 vel;
      if (!lookupVelocity(id, &vel)) {
        continue;
      }
      x += vel.x;
      y += vel.y;
      z += vel.z;
      ++count;
    }
    if (count == 0) {
      return false;
    }
    velocity->x = x / static_cast<double>(count);
    velocity->y = y / static_cast<double>(count);
    velocity->z = z / static_cast<double>(count);
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
      geometry_msgs::Point lhs;
      if (!lookupPoint(lhs_id, &lhs)) {
        continue;
      }
      for (const int rhs_id : rhs_ids) {
        geometry_msgs::Point rhs;
        if (!lookupPoint(rhs_id, &rhs)) {
          continue;
        }
        const double distance_sq = SquaredDistance(lhs, rhs);
        if (distance_sq >= best_distance_sq) {
          continue;
        }
        best_distance_sq = distance_sq;
        anchor->x = 0.5 * (lhs.x + rhs.x);
        anchor->y = 0.5 * (lhs.y + rhs.y);
        anchor->z = 0.5 * (lhs.z + rhs.z);
        found = true;
      }
    }
    return found;
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
          "[LocalRelayCtl " << self_id_ << "] shifted relay target out of obstacle from ("
          << raw_anchor.x << ", " << raw_anchor.y << ", " << raw_anchor.z
          << ") to (" << anchor->x << ", " << anchor->y << ", "
          << anchor->z << ")");
    }
  }

  bool computeRelayAnchor(
      const std::vector<int>& fresh_ids, const std::vector<int>& stale_ids, geometry_msgs::Point* anchor) const {
    if (anchor == nullptr || stale_ids.empty()) {
      return false;
    }
    std::vector<int> lhs_ids = fresh_ids;
    lhs_ids.push_back(self_id_);
    if (relay_anchor_use_nearest_pair_midpoint_ && computeNearestBridgeMidpoint(lhs_ids, stale_ids, anchor)) {
      sanitizeRelayAnchor(anchor);
      return true;
    }

    geometry_msgs::Point lhs_centroid;
    geometry_msgs::Point rhs_centroid;
    if (!computeCentroid(lhs_ids, &lhs_centroid) || !computeCentroid(stale_ids, &rhs_centroid)) {
      return false;
    }
    anchor->x = 0.5 * (lhs_centroid.x + rhs_centroid.x);
    anchor->y = 0.5 * (lhs_centroid.y + rhs_centroid.y);
    anchor->z = 0.5 * (lhs_centroid.z + rhs_centroid.z);
    sanitizeRelayAnchor(anchor);
    return true;
  }

  bool shouldSuppressBalancedSplit(
      const std::vector<int>& fresh_ids, const std::vector<int>& stale_ids) const {
    std::vector<int> lhs_ids = fresh_ids;
    lhs_ids.push_back(self_id_);
    if (stale_ids.empty() || lhs_ids.empty()) {
      return false;
    }
    const int lhs_size = static_cast<int>(lhs_ids.size());
    const int rhs_size = static_cast<int>(stale_ids.size());
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
    if (!computeCentroid(lhs_ids, &lhs_centroid) || !computeCentroid(stale_ids, &rhs_centroid)) {
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
    if (!computeAverageVelocity(lhs_ids, &lhs_velocity) || !computeAverageVelocity(stale_ids, &rhs_velocity)) {
      return false;
    }

    geometry_msgs::Point midpoint;
    midpoint.x = 0.5 * (lhs_centroid.x + rhs_centroid.x);
    midpoint.y = 0.5 * (lhs_centroid.y + rhs_centroid.y);
    midpoint.z = 0.5 * (lhs_centroid.z + rhs_centroid.z);

    const auto outward_progress = [&midpoint](const geometry_msgs::Point& centroid,
                                              const geometry_msgs::Vector3& velocity) {
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

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber odom_sub_;
  ros::Subscriber peer_state_sub_;
  ros::Subscriber proposal_sub_;
  ros::Subscriber self_drone_state_sub_;
  ros::Subscriber peer_drone_state_sub_;
  ros::Publisher peer_state_pub_;
  ros::Publisher proposal_pub_;
  ros::Publisher relay_role_pub_;
  ros::Timer tick_timer_;

  int self_id_{1};
  RelayPolicyMode relay_policy_mode_;
  bool relay_active_;
  bool have_self_odom_;
  int queue_size_;
  double control_period_sec_;
  double peer_state_fresh_timeout_sec_;
  double peer_state_stale_retention_sec_;
  double proposal_timeout_sec_;
  double task_state_fresh_timeout_sec_;
  double relay_trigger_persist_time_;
  double relay_winner_persist_time_;
  double relay_recover_time_;
  double relay_min_hold_time_;
  double relay_reenter_cooldown_;
  double relay_startup_grace_period_;
  double relay_max_occupancy_ratio_;
  double relay_no_improvement_timeout_;
  double relay_max_active_duration_;
  int relay_activation_min_stale_count_;
  double relay_activation_score_margin_;
  int relay_activation_min_votes_;
  double relay_handoff_persist_time_;
  int relay_handoff_min_vote_margin_;
  double relay_candidate_speed_penalty_gain_;
  double relay_candidate_reuse_penalty_gain_;
  double relay_candidate_grid_load_penalty_gain_;
  double relay_candidate_backtrack_penalty_gain_;
  double relay_candidate_recent_safety_penalty_gain_;
  double relay_candidate_recent_safety_timeout_sec_;
  double relay_candidate_recent_plan_fail_penalty_gain_;
  double relay_candidate_recent_plan_fail_timeout_sec_;
  double relay_candidate_recent_plan_success_penalty_gain_;
  double relay_candidate_recent_plan_success_timeout_sec_;
  double relay_candidate_consecutive_plan_fail_penalty_gain_;
  bool relay_suppress_balanced_split_;
  int relay_balanced_min_component_size_;
  double relay_balanced_size_ratio_threshold_;
  double relay_balanced_min_separation_m_;
  double relay_balanced_axis_ratio_threshold_;
  double relay_balanced_outward_velocity_threshold_;
  bool relay_anchor_use_nearest_pair_midpoint_;
  bool relay_target_use_fixed_z_;
  double relay_target_fixed_z_;
  double relay_target_min_z_;
  double relay_target_max_z_;
  double relay_target_obstacle_clearance_ = 0.6;
  std::vector<ObstacleBox> relay_obstacles_;

  std::vector<int> drone_ids_;
  std::unordered_map<int, PeerStateEntry> peer_cache_;
  std::unordered_map<int, TaskStateEntry> peer_task_cache_;
  std::unordered_map<int, ProposalEntry> proposal_cache_;

  geometry_msgs::Point self_position_;
  geometry_msgs::Vector3 self_velocity_;
  ros::Time self_odom_stamp_;
  TaskStateEntry self_task_state_;
  geometry_msgs::Point relay_anchor_;
  ros::Time first_tick_time_;
  ros::Time fragmented_since_;
  ros::Time connected_since_;
  ros::Time relay_change_time_;
  ros::Time relay_exit_time_;
  ros::Time relay_last_improvement_time_;
  ros::Time stable_winner_since_;
  int stable_winner_id_{-1};
  int relay_best_stale_peer_count_{std::numeric_limits<int>::max()};
  int relay_best_fresh_peer_count_{0};
  int relay_active_support_count_{0};
  double cumulative_relay_occupancy_sec_{0.0};

  std::string odom_topic_;
  std::string peer_state_send_topic_;
  std::string peer_state_recv_topic_;
  std::string proposal_send_topic_;
  std::string proposal_recv_topic_;
  std::string relay_role_topic_;
  std::string self_drone_state_topic_;
  std::string peer_drone_state_topic_;
};

}  // namespace relay_racer_integration

int main(int argc, char** argv) {
  ros::init(argc, argv, "racer_local_relay_controller_node");
  relay_racer_integration::RacerLocalRelayControllerNode node;
  ros::spin();
  return 0;
}
