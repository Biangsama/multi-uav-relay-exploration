#ifndef _FAST_EXPLORATION_FSM_H_
#define _FAST_EXPLORATION_FSM_H_

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <exploration_manager/DroneState.h>
#include <exploration_manager/PairOpt.h>
#include <exploration_manager/PairOptResponse.h>
#include <exploration_manager/AssignmentPlan.h>
#include <exploration_manager/AssignmentAck.h>
#include <exploration_manager/AssignmentCommit.h>
#include <exploration_manager/AllocationRequest.h>
#include <exploration_manager/ReleaseRequest.h>
#include <bspline/Bspline.h>
#include <relay_racer_integration/RelayRoleCmd.h>
#include <relay_racer_integration/RelayTaskState.h>

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <vector>
#include <memory>
#include <unordered_map>
#include <string>
#include <thread>

using Eigen::Vector3d;
using std::shared_ptr;
using std::string;
using std::unique_ptr;
using std::vector;

namespace fast_planner {
class FastPlannerManager;
class FastExplorationManager;
class PlanningVisualization;
struct FSMParam;
struct FSMData;

enum EXPL_STATE { INIT, WAIT_TRIGGER, PLAN_TRAJ, PUB_TRAJ, EXEC_TRAJ, FINISH, IDLE };

class FastExplorationFSM {

public:
  FastExplorationFSM(/* args */) {
  }
  ~FastExplorationFSM() {
  }

  void init(ros::NodeHandle& nh);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  /* helper functions */
  int callExplorationPlanner();
  void transitState(EXPL_STATE new_state, string pos_call);
  void visualize(int content);
  void clearVisMarker();
  int getId();
  bool handleRemoteMapUpdate(const ros::Time& now);
  void findUnallocated(const vector<int>& actives, vector<int>& missed);
  void recoverAssignmentAfterRelayExit(const string& reason);
  void requestAggressiveReassign(const string& reason, bool immediate_claim = true);
  bool tryClaimUnallocatedGrids(const string& pos_call, bool require_empty_assignment);
  bool tryStartExploration(const string& pos_call, bool finish_if_no_frontier);
  void getEffectiveSelfGridIds(vector<int>& grid_ids) const;
  void stagePendingSelfRelease(const vector<int>& grid_ids);
  bool getPendingAllocationCandidates(vector<int>& grid_ids) const;
  void selectPlanFailureReleaseGrids(
      const vector<int>& effective_self_grid_ids, bool aggressive, vector<int>& release_grid_ids);
  void clearPendingAssignmentTxn();
  void clearLegacyPairOptState(const string& reason);
  void logOwnershipEvent(const string& chain, const string& event, uint64_t txn_id,
      uint64_t component_epoch, int leader_id, int requester_id, const vector<int>& grid_ids,
      const string& detail = string()) const;
  void logPlanFailureRecovery(const string& stage, int failure_count,
      const vector<int>& effective_self_grid_ids, const vector<int>& release_grid_ids,
      bool trigger_aggressive_reassign, const string& reason) const;
  bool publishAllocationRequest(const string& reason);
  bool publishReleaseRequest(const vector<int>& grid_ids, const string& reason);

  /* ROS functions */
  void FSMCallback(const ros::TimerEvent& e);
  void safetyCallback(const ros::TimerEvent& e);
  void frontierCallback(const ros::TimerEvent& e);
  void triggerCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
  void relayRoleCmdCallback(const relay_racer_integration::RelayRoleCmdConstPtr& msg);

  // Swarm
  void droneStateTimerCallback(const ros::TimerEvent& e);
  void droneStateMsgCallback(const exploration_manager::DroneStateConstPtr& msg);
  void assignmentPlanCallback(const exploration_manager::AssignmentPlanConstPtr& msg);
  void assignmentCommitCallback(const exploration_manager::AssignmentCommitConstPtr& msg);
  void optTimerCallback(const ros::TimerEvent& e);
  void optMsgCallback(const exploration_manager::PairOptConstPtr& msg);
  void optResMsgCallback(const exploration_manager::PairOptResponseConstPtr& msg);
  void swarmTrajCallback(const bspline::BsplineConstPtr& msg);
  void swarmTrajTimerCallback(const ros::TimerEvent& e);

  /* planning utils */
  shared_ptr<FastPlannerManager> planner_manager_;
  shared_ptr<FastExplorationManager> expl_manager_;
  shared_ptr<PlanningVisualization> visualization_;

  shared_ptr<FSMParam> fp_;
  shared_ptr<FSMData> fd_;
  EXPL_STATE state_;

  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, safety_timer_, vis_timer_, frontier_timer_;
  ros::Subscriber trigger_sub_, odom_sub_, relay_role_sub_;
  ros::Publisher replan_pub_, new_pub_, bspline_pub_;

  // Swarm state
  ros::Publisher drone_state_pub_, relay_task_state_pub_, component_state_pub_, allocation_request_pub_, release_request_pub_, assignment_ack_pub_,
      opt_pub_, opt_res_pub_, swarm_traj_pub_, grid_tour_pub_, hgrid_pub_;
  ros::Subscriber drone_state_sub_, assignment_plan_sub_, assignment_commit_sub_, opt_sub_, opt_res_sub_,
      swarm_traj_sub_;
  ros::Timer drone_state_timer_, opt_timer_, swarm_traj_timer_;

  int current_role_;
  bool have_relay_target_;
  Vector3d relay_target_;
  double relay_yaw_;
  vector<int> relay_suspended_grid_ids_;
  uint64_t component_epoch_;
  int component_leader_id_;
  bool component_members_initialized_;
  vector<int> last_component_member_ids_;
  std::unordered_map<int, int> component_owner_ids_;
  std::unordered_map<int, uint64_t> component_owner_versions_;
  // Transaction-local staged assignment: plan received and acked, but ownership not committed yet.
  bool pending_assignment_active_;
  uint64_t pending_assignment_txn_id_;
  uint64_t pending_assignment_component_epoch_;
  int pending_assignment_leader_id_;
  int pending_assignment_grid_id_;
  int pending_assignment_owner_id_;
  uint64_t pending_assignment_owner_version_;
  uint64_t next_allocation_request_txn_id_;
  uint64_t next_release_request_txn_id_;
};

}  // namespace fast_planner

#endif
