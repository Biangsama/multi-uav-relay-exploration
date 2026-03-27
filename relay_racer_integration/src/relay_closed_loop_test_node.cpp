#include <bspline/Bspline.h>
#include <dancers_msgs/AgentStruct.h>
#include <dancers_msgs/GetAgentVelocities.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <protobuf_msgs/net_rx_events.pb.h>

#include <relay_racer_integration/RelayRoleCmd.h>

#include <cstdint>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace relay_racer_integration {
namespace {

dancers_msgs::AgentStruct MakeAgent(const uint32_t platform_id, const double x) {
  dancers_msgs::AgentStruct agent;
  agent.agent_role = dancers_msgs::AgentStruct::AGENT_ROLE_MISSION;
  agent.agent_id = platform_id;
  agent.state.position.x = x;
  agent.state.position.y = 0.0;
  agent.state.position.z = 1.0;
  agent.state.velocity.x = 0.0;
  agent.state.velocity.y = 0.0;
  agent.state.velocity.z = 0.0;
  agent.state.heading = 0.0;
  agent.heartbeat_received = 1;
  agent.heartbeat_sent = 1;
  return agent;
}

std::vector<int> ParseAgentIdsCsv(const std::string& csv) {
  std::vector<int> ids;
  std::stringstream ss(csv);
  std::string item;
  while (std::getline(ss, item, ',')) {
    if (item.empty()) continue;
    ids.push_back(std::stoi(item));
  }
  return ids;
}

std::vector<double> ParseDoublesCsv(const std::string& csv) {
  std::vector<double> values;
  std::stringstream ss(csv);
  std::string item;
  while (std::getline(ss, item, ',')) {
    if (item.empty()) continue;
    values.push_back(std::stod(item));
  }
  return values;
}

std::vector<std::pair<uint32_t, uint32_t>> ParseEdgePairsCsv(const std::string& csv) {
  std::vector<std::pair<uint32_t, uint32_t>> edges;
  std::stringstream ss(csv);
  std::string item;
  while (std::getline(ss, item, ',')) {
    if (item.empty()) continue;
    const std::size_t sep = item.find('-');
    if (sep == std::string::npos) {
      continue;
    }
    const int lhs = std::stoi(item.substr(0, sep));
    const int rhs = std::stoi(item.substr(sep + 1));
    if (lhs < 0 || rhs < 0 || lhs == rhs) {
      continue;
    }
    edges.emplace_back(static_cast<uint32_t>(lhs), static_cast<uint32_t>(rhs));
  }
  return edges;
}

std::string JoinAgentIds(const std::vector<int>& values) {
  std::ostringstream oss;
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i != 0) {
      oss << ",";
    }
    oss << values[i];
  }
  return oss.str();
}

std::string JoinDoubles(const std::vector<double>& values) {
  std::ostringstream oss;
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i != 0) {
      oss << ",";
    }
    oss << values[i];
  }
  return oss.str();
}

std::string JoinEdges(const std::vector<std::pair<uint32_t, uint32_t>>& values) {
  std::ostringstream oss;
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i != 0) {
      oss << ",";
    }
    oss << values[i].first << "-" << values[i].second;
  }
  return oss.str();
}

void AddBidirectionalEdge(const uint32_t lhs_platform_id, const uint32_t rhs_platform_id,
    protobuf_msgs::NetRxEventsPayload* payload) {
  auto* lhs_to_rhs = payload->add_events();
  lhs_to_rhs->set_src_id(lhs_platform_id);
  lhs_to_rhs->set_dst_id(rhs_platform_id);
  lhs_to_rhs->set_flow_id(1);
  lhs_to_rhs->set_seq(1);
  lhs_to_rhs->set_tx_time_us(1000);
  lhs_to_rhs->set_rx_time_us(1100);
  lhs_to_rhs->set_delay_us(100);
  lhs_to_rhs->set_payload_bytes(64);
  lhs_to_rhs->set_rssi_dbm_x10(-450);

  auto* rhs_to_lhs = payload->add_events();
  rhs_to_lhs->set_src_id(rhs_platform_id);
  rhs_to_lhs->set_dst_id(lhs_platform_id);
  rhs_to_lhs->set_flow_id(1);
  rhs_to_lhs->set_seq(2);
  rhs_to_lhs->set_tx_time_us(1200);
  rhs_to_lhs->set_rx_time_us(1300);
  rhs_to_lhs->set_delay_us(100);
  rhs_to_lhs->set_payload_bytes(64);
  rhs_to_lhs->set_rssi_dbm_x10(-450);
}

}  // namespace

class RelayClosedLoopTestNode {
public:
  RelayClosedLoopTestNode()
      : nh_(),
        pnh_("~"),
        state_(State::kInjectInput),
        exit_code_(0),
        service_call_deadline_(3.2),
        goal_deadline_(4.0),
        finish_deadline_(7.0),
        failure_deadline_(10.0),
        publish_fake_odom_(true),
        call_test_service_(true),
        wait_for_relay_cmd_(true),
        shutdown_on_finish_(true),
        republish_goal_without_relay_(false),
        goal_republish_period_(1.0),
        require_planner_ready_(false),
        planner_ready_min_messages_(1),
        planner_ready_hold_sec_(0.0),
        post_startup_service_enabled_(false),
        post_startup_service_require_goal_(true),
        post_startup_service_require_planning_artifact_(true),
        post_startup_service_delay_sec_(3.0),
        post_startup_service_repeat_count_(1),
        post_startup_service_repeat_period_sec_(0.5),
        planning_artifact_min_messages_(1),
        relay_role_seen_(false),
        relay_active_now_(false),
        goal_published_(false),
        planning_artifact_seen_(false),
        post_startup_service_calls_made_(0),
        goal_x_(1.0),
        goal_y_(0.0),
        goal_z_(1.0) {
    pnh_.param<std::string>("odom_topic", odom_topic_, "/relay_test/odom_1");
    pnh_.param<std::string>("goal_topic", goal_topic_, "/move_base_simple/goal");
    pnh_.param<std::string>("relay_role_topic", relay_role_topic_, "/relay_integration/relay_role_cmd");
    pnh_.param<std::string>("service_name", service_name_, "get_agents_velocities");
    pnh_.param("test_agent_id", test_agent_id_, 1);
    pnh_.param("service_call_deadline", service_call_deadline_, service_call_deadline_);
    pnh_.param("goal_deadline", goal_deadline_, goal_deadline_);
    pnh_.param("finish_deadline", finish_deadline_, finish_deadline_);
    pnh_.param("failure_deadline", failure_deadline_, failure_deadline_);
    pnh_.param("publish_fake_odom", publish_fake_odom_, publish_fake_odom_);
    pnh_.param("call_test_service", call_test_service_, call_test_service_);
    pnh_.param("wait_for_relay_cmd", wait_for_relay_cmd_, wait_for_relay_cmd_);
    pnh_.param("shutdown_on_finish", shutdown_on_finish_, shutdown_on_finish_);
    pnh_.param("republish_goal_without_relay", republish_goal_without_relay_,
        republish_goal_without_relay_);
    pnh_.param("goal_republish_period", goal_republish_period_, goal_republish_period_);
    pnh_.param("require_planner_ready", require_planner_ready_, require_planner_ready_);
    pnh_.param("planner_ready_min_messages", planner_ready_min_messages_, planner_ready_min_messages_);
    pnh_.param("planner_ready_hold_sec", planner_ready_hold_sec_, planner_ready_hold_sec_);
    pnh_.param<std::string>("planner_ready_agent_ids", planner_ready_agent_ids_csv_, "");
    pnh_.param("post_startup_service_enabled", post_startup_service_enabled_, post_startup_service_enabled_);
    pnh_.param("post_startup_service_require_goal", post_startup_service_require_goal_,
        post_startup_service_require_goal_);
    pnh_.param("post_startup_service_require_planning_artifact",
        post_startup_service_require_planning_artifact_,
        post_startup_service_require_planning_artifact_);
    pnh_.param("post_startup_service_delay_sec", post_startup_service_delay_sec_,
        post_startup_service_delay_sec_);
    pnh_.param("post_startup_service_repeat_count", post_startup_service_repeat_count_,
        post_startup_service_repeat_count_);
    pnh_.param("post_startup_service_repeat_period_sec", post_startup_service_repeat_period_sec_,
        post_startup_service_repeat_period_sec_);
    pnh_.param("planning_artifact_min_messages", planning_artifact_min_messages_,
        planning_artifact_min_messages_);
    pnh_.param<std::string>("planning_artifact_agent_ids", planning_artifact_agent_ids_csv_,
        planner_ready_agent_ids_csv_);
    pnh_.param<std::string>("service_agent_x_positions", service_agent_x_positions_csv_,
        std::string("0,6,12,30"));
    pnh_.param<std::string>("service_edges", service_edges_csv_, std::string("0-1,1-2"));
    pnh_.param("goal_x", goal_x_, goal_x_);
    pnh_.param("goal_y", goal_y_, goal_y_);
    pnh_.param("goal_z", goal_z_, goal_z_);

    planner_ready_agent_ids_ = ParseAgentIdsCsv(planner_ready_agent_ids_csv_);
    planning_artifact_agent_ids_ = ParseAgentIdsCsv(planning_artifact_agent_ids_csv_);
    if (planning_artifact_agent_ids_.empty()) {
      planning_artifact_agent_ids_ = planner_ready_agent_ids_;
      planning_artifact_agent_ids_csv_ = JoinAgentIds(planning_artifact_agent_ids_);
    }
    service_agent_x_positions_ = ParseDoublesCsv(service_agent_x_positions_csv_);
    if (service_agent_x_positions_.empty()) {
      service_agent_x_positions_ = {0.0, 6.0, 12.0, 30.0};
      service_agent_x_positions_csv_ = JoinDoubles(service_agent_x_positions_);
    }
    service_edges_ = ParseEdgePairsCsv(service_edges_csv_);
    if (service_edges_.empty()) {
      service_edges_.emplace_back(0, 1);
      service_edges_.emplace_back(1, 2);
      service_edges_csv_ = JoinEdges(service_edges_);
    }
    if (post_startup_service_repeat_count_ < 0) {
      post_startup_service_repeat_count_ = 0;
    }

    if (!call_test_service_) {
      state_ = State::kWaitRelayCmd;
    }

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 10);
    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(goal_topic_, 1, true);
    relay_role_sub_ =
        nh_.subscribe(relay_role_topic_, 20, &RelayClosedLoopTestNode::relayRoleCmdCallback, this);
    for (const int agent_id : planner_ready_agent_ids_) {
      planner_ready_subs_.push_back(nh_.subscribe<visualization_msgs::Marker>(
          "/planning_vis/frontier_" + std::to_string(agent_id), 20,
          boost::bind(&RelayClosedLoopTestNode::plannerFrontierCallback, this, _1, agent_id)));
    }
    for (const int agent_id : planning_artifact_agent_ids_) {
      planning_artifact_subs_.push_back(nh_.subscribe<bspline::Bspline>(
          "/planning/bspline_" + std::to_string(agent_id), 20,
          boost::bind(&RelayClosedLoopTestNode::planningBsplineCallback, this, _1, agent_id)));
    }
    service_client_ = nh_.serviceClient<dancers_msgs::GetAgentVelocities>(service_name_);
    timer_ = nh_.createTimer(ros::Duration(0.1), &RelayClosedLoopTestNode::timerCallback, this);

    start_time_ = ros::Time::now();
    ROS_INFO_STREAM("relay_closed_loop_test_node started, odom_topic=" << odom_topic_
                    << ", service_name=" << service_name_
                    << ", test_agent_id=" << test_agent_id_
                    << ", publish_fake_odom=" << std::boolalpha << publish_fake_odom_
                    << ", call_test_service=" << call_test_service_
                    << ", wait_for_relay_cmd=" << wait_for_relay_cmd_
                    << ", shutdown_on_finish=" << shutdown_on_finish_
                    << ", republish_goal_without_relay=" << republish_goal_without_relay_
                    << ", goal_republish_period=" << goal_republish_period_
                    << ", require_planner_ready=" << require_planner_ready_
                    << ", planner_ready_agent_ids=" << planner_ready_agent_ids_csv_
                    << ", planner_ready_min_messages=" << planner_ready_min_messages_
                    << ", planner_ready_hold_sec=" << planner_ready_hold_sec_
                    << ", post_startup_service_enabled=" << post_startup_service_enabled_
                    << ", post_startup_service_require_goal=" << post_startup_service_require_goal_
                    << ", post_startup_service_require_planning_artifact="
                    << post_startup_service_require_planning_artifact_
                    << ", post_startup_service_delay_sec=" << post_startup_service_delay_sec_
                    << ", post_startup_service_repeat_count=" << post_startup_service_repeat_count_
                    << ", post_startup_service_repeat_period_sec=" << post_startup_service_repeat_period_sec_
                    << ", planning_artifact_agent_ids=" << planning_artifact_agent_ids_csv_
                    << ", planning_artifact_min_messages=" << planning_artifact_min_messages_
                    << ", service_agent_x_positions=" << service_agent_x_positions_csv_
                    << ", service_edges=" << service_edges_csv_);
  }

  int exitCode() const {
    return exit_code_;
  }

private:
  enum class State { kInjectInput, kWaitRelayCmd, kWaitAfterGoal, kDone };

  void timerCallback(const ros::TimerEvent&) {
    if (publish_fake_odom_) {
      publishOdom();
    }

    const double elapsed = (ros::Time::now() - start_time_).toSec();
    if (elapsed >= failure_deadline_ && state_ != State::kDone) {
      if (wait_for_relay_cmd_ && !relay_role_seen_) {
        exit_code_ = 1;
        ROS_ERROR("relay_closed_loop_test_node timed out before observing the relay command.");
      } else {
        ROS_ERROR("relay_closed_loop_test_node timed out before finishing the trigger sequence.");
      }
      shutdown();
      return;
    }

    maybeRunPostStartupService();

    if (state_ == State::kInjectInput) {
      if (elapsed >= service_call_deadline_ && callRelayTriggerService("startup")) {
        state_ = State::kWaitRelayCmd;
      }
      return;
    }

    if (state_ == State::kWaitRelayCmd) {
      if (goal_published_) {
        state_ = State::kWaitAfterGoal;
        return;
      }

      if (elapsed < goal_deadline_) {
        return;
      }

      if (!plannersReady(elapsed)) {
        return;
      }

      if (!wait_for_relay_cmd_) {
        publishGoal();
        goal_published_ = true;
        state_ = State::kWaitAfterGoal;
        return;
      }

      if (relay_active_now_) {
        publishGoal();
        goal_published_ = true;
        state_ = State::kWaitAfterGoal;
      }
      return;
    }

    if (state_ == State::kWaitAfterGoal && elapsed >= finish_deadline_) {
      ROS_INFO("relay_closed_loop_test_node finished after observing relay command and publishing trigger goal.");
      if (shutdown_on_finish_) {
        shutdown();
      } else {
        state_ = State::kDone;
        timer_.stop();
        ROS_INFO("relay_closed_loop_test_node is holding the launch open for external validation.");
      }
      return;
    }

    if (state_ == State::kWaitAfterGoal && shouldRepublishGoal(elapsed)) {
      publishGoal(true);
    }
  }

  void publishOdom() {
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "world";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = 0.0;
    odom.pose.pose.position.y = 0.0;
    odom.pose.pose.position.z = 1.0;
    odom.pose.pose.orientation.w = 1.0;
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;
    odom_pub_.publish(odom);
  }

  void maybeRunPostStartupService() {
    if (!post_startup_service_enabled_ || post_startup_service_repeat_count_ <= 0) {
      return;
    }
    if (state_ == State::kDone) {
      return;
    }
    if (post_startup_service_calls_made_ >= post_startup_service_repeat_count_) {
      return;
    }
    if (post_startup_service_require_goal_ && !goal_published_) {
      return;
    }

    ros::Time reference_time = last_goal_publish_time_;
    if (post_startup_service_require_planning_artifact_) {
      if (!planning_artifact_seen_ || first_planning_artifact_time_.isZero()) {
        return;
      }
      reference_time = first_planning_artifact_time_;
    }

    if (reference_time.isZero()) {
      return;
    }

    const ros::Time now = ros::Time::now();
    if ((now - reference_time).toSec() < post_startup_service_delay_sec_) {
      return;
    }
    if (!last_post_startup_service_call_time_.isZero() &&
        (now - last_post_startup_service_call_time_).toSec() < post_startup_service_repeat_period_sec_) {
      return;
    }

    std::ostringstream context;
    context << "post_startup " << (post_startup_service_calls_made_ + 1)
            << "/" << post_startup_service_repeat_count_;
    if (callRelayTriggerService(context.str())) {
      last_post_startup_service_call_time_ = now;
      ++post_startup_service_calls_made_;
    }
  }

  bool callRelayTriggerService(const std::string& context) {
    if (!service_client_.waitForExistence(ros::Duration(0.5))) {
      ROS_WARN_THROTTLE(1.0, "Waiting for get_agents_velocities service.");
      return false;
    }

    dancers_msgs::GetAgentVelocities srv;
    for (std::size_t i = 0; i < service_agent_x_positions_.size(); ++i) {
      srv.request.agent_structs.push_back(MakeAgent(static_cast<uint32_t>(i), service_agent_x_positions_[i]));
    }

    protobuf_msgs::NetRxEventsPayload payload;
    for (const auto& edge : service_edges_) {
      if (edge.first >= service_agent_x_positions_.size() || edge.second >= service_agent_x_positions_.size()) {
        continue;
      }
      AddBidirectionalEdge(edge.first, edge.second, &payload);
    }

    std::string bytes;
    payload.SerializeToString(&bytes);
    srv.request.comm_rx_events_bytes.assign(bytes.begin(), bytes.end());

    if (!service_client_.call(srv)) {
      ROS_ERROR_STREAM("Failed to call get_agents_velocities for context=" << context);
      return false;
    }

    ROS_INFO_STREAM("Called get_agents_velocities for context=" << context
                    << " with agent_x_positions=" << service_agent_x_positions_csv_
                    << " edges=" << service_edges_csv_);
    return true;
  }

  bool plannersReady(const double elapsed) {
    if (!require_planner_ready_ || planner_ready_agent_ids_.empty()) {
      return true;
    }

    for (const int agent_id : planner_ready_agent_ids_) {
      if (planner_frontier_count_by_agent_[agent_id] < planner_ready_min_messages_) {
        ROS_WARN_THROTTLE(1.0, "Waiting for planner %d frontier warm-up (%d/%d).", agent_id,
            planner_frontier_count_by_agent_[agent_id], planner_ready_min_messages_);
        return false;
      }
    }

    if (!all_planners_ready_time_.isZero() && planner_ready_hold_sec_ > 0.0) {
      const double ready_hold = (ros::Time::now() - all_planners_ready_time_).toSec();
      if (ready_hold < planner_ready_hold_sec_) {
        ROS_WARN_THROTTLE(1.0, "Waiting planner-ready hold %.2f/%.2fs before publishing goal.",
            ready_hold, planner_ready_hold_sec_);
        return false;
      }
    }

    return true;
  }

  void plannerFrontierCallback(const visualization_msgs::MarkerConstPtr&, const int agent_id) {
    const int count = ++planner_frontier_count_by_agent_[agent_id];
    if (count == planner_ready_min_messages_) {
      planners_ready_.insert(agent_id);
      ROS_INFO_STREAM("Planner " << agent_id << " reached frontier warm-up count=" << count);
      if (planners_ready_.size() == planner_ready_agent_ids_.size() && all_planners_ready_time_.isZero()) {
        all_planners_ready_time_ = ros::Time::now();
        ROS_INFO_STREAM("All planners reached frontier warm-up at t="
                        << (all_planners_ready_time_ - start_time_).toSec());
      }
    }
  }

  void planningBsplineCallback(const bspline::BsplineConstPtr&, const int agent_id) {
    const int count = ++planning_bspline_count_by_agent_[agent_id];
    if (count == planning_artifact_min_messages_) {
      planning_artifact_agents_seen_.insert(agent_id);
      if (!planning_artifact_seen_) {
        planning_artifact_seen_ = true;
        first_planning_artifact_time_ = ros::Time::now();
        ROS_INFO_STREAM("Observed first planning artifact on planner " << agent_id
                        << " at t=" << (first_planning_artifact_time_ - start_time_).toSec());
      }
    }
  }

  void publishGoal(const bool is_republish = false) {
    geometry_msgs::PoseStamped goal;
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = "world";
    goal.pose.position.x = goal_x_;
    goal.pose.position.y = goal_y_;
    goal.pose.position.z = goal_z_;
    goal.pose.orientation.w = 1.0;
    goal_pub_.publish(goal);
    last_goal_publish_time_ = goal.header.stamp;
    if (is_republish) {
      ROS_INFO_STREAM("Republished /move_base_simple/goal while relay remains active at ("
                      << goal_x_ << ", " << goal_y_ << ", " << goal_z_ << ").");
    } else {
      ROS_INFO_STREAM("Published /move_base_simple/goal to trigger relay planning at ("
                      << goal_x_ << ", " << goal_y_ << ", " << goal_z_ << ").");
    }
  }

  bool shouldRepublishGoal(const double elapsed) const {
    if (goal_republish_period_ <= 0.0 || !goal_published_) {
      return false;
    }
    if (!relay_active_now_ && !republish_goal_without_relay_) {
      return false;
    }

    if ((finish_deadline_ - elapsed) <= goal_republish_period_) {
      return false;
    }

    if (last_goal_publish_time_.isZero()) {
      return true;
    }

    return (ros::Time::now() - last_goal_publish_time_).toSec() >= goal_republish_period_;
  }

  void relayRoleCmdCallback(const relay_racer_integration::RelayRoleCmdConstPtr& msg) {
    if (msg->agent_id != test_agent_id_) {
      return;
    }

    ROS_INFO_STREAM("Observed RelayRoleCmd for agent " << msg->agent_id << ": role=" << int(msg->role)
                    << ", valid=" << msg->valid << ", target=(" << msg->relay_target.x << ", "
                    << msg->relay_target.y << ", " << msg->relay_target.z << ")");

    const bool relay_active =
        msg->role == relay_racer_integration::RelayRoleCmd::ROLE_RELAY && msg->valid;
    relay_active_now_ = relay_active;

    if (relay_active) {
      relay_role_seen_ = true;
      const double elapsed = (ros::Time::now() - start_time_).toSec();
      if (state_ == State::kWaitRelayCmd && !goal_published_ && elapsed >= goal_deadline_
          && plannersReady(elapsed)) {
        publishGoal();
        goal_published_ = true;
        state_ = State::kWaitAfterGoal;
      }
    }
  }

  void shutdown() {
    state_ = State::kDone;
    timer_.stop();
    ros::shutdown();
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher odom_pub_;
  ros::Publisher goal_pub_;
  ros::Subscriber relay_role_sub_;
  ros::ServiceClient service_client_;
  ros::Timer timer_;

  State state_;
  int exit_code_;
  double service_call_deadline_;
  double goal_deadline_;
  double finish_deadline_;
  double failure_deadline_;
  bool publish_fake_odom_;
  bool call_test_service_;
  bool wait_for_relay_cmd_;
  bool shutdown_on_finish_;
  bool republish_goal_without_relay_;
  double goal_republish_period_;
  bool require_planner_ready_;
  int planner_ready_min_messages_;
  double planner_ready_hold_sec_;
  bool post_startup_service_enabled_;
  bool post_startup_service_require_goal_;
  bool post_startup_service_require_planning_artifact_;
  double post_startup_service_delay_sec_;
  int post_startup_service_repeat_count_;
  double post_startup_service_repeat_period_sec_;
  int planning_artifact_min_messages_;
  int test_agent_id_;
  bool relay_role_seen_;
  bool relay_active_now_;
  bool goal_published_;
  bool planning_artifact_seen_;
  int post_startup_service_calls_made_;
  double goal_x_;
  double goal_y_;
  double goal_z_;
  ros::Time start_time_;
  ros::Time last_goal_publish_time_;
  ros::Time all_planners_ready_time_;
  ros::Time first_planning_artifact_time_;
  ros::Time last_post_startup_service_call_time_;

  std::string odom_topic_;
  std::string goal_topic_;
  std::string relay_role_topic_;
  std::string service_name_;
  std::string planner_ready_agent_ids_csv_;
  std::string planning_artifact_agent_ids_csv_;
  std::string service_agent_x_positions_csv_;
  std::string service_edges_csv_;

  std::vector<int> planner_ready_agent_ids_;
  std::vector<int> planning_artifact_agent_ids_;
  std::vector<double> service_agent_x_positions_;
  std::vector<std::pair<uint32_t, uint32_t>> service_edges_;
  std::vector<ros::Subscriber> planner_ready_subs_;
  std::vector<ros::Subscriber> planning_artifact_subs_;
  std::set<int> planners_ready_;
  std::set<int> planning_artifact_agents_seen_;
  std::map<int, int> planner_frontier_count_by_agent_;
  std::map<int, int> planning_bspline_count_by_agent_;
};

}  // namespace relay_racer_integration

int main(int argc, char** argv) {
  ros::init(argc, argv, "relay_closed_loop_test_node");
  relay_racer_integration::RelayClosedLoopTestNode node;
  ros::spin();
  return node.exitCode();
}
