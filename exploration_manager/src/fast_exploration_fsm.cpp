
#include <plan_manage/planner_manager.h>
#include <exploration_manager/fast_exploration_manager.h>
#include <traj_utils/planning_visualization.h>

#include <exploration_manager/fast_exploration_fsm.h>
#include <exploration_manager/ComponentState.h>
#include <exploration_manager/AssignmentPlan.h>
#include <exploration_manager/AssignmentAck.h>
#include <exploration_manager/AssignmentCommit.h>
#include <exploration_manager/AllocationRequest.h>
#include <exploration_manager/ReleaseRequest.h>
#include <exploration_manager/expl_data.h>
#include <exploration_manager/HGrid.h>
#include <exploration_manager/GridTour.h>

#include <plan_env/edt_environment.h>
#include <plan_env/sdf_map.h>
#include <plan_env/multi_map_manager.h>
#include <active_perception/perception_utils.h>
#include <active_perception/hgrid.h>
// #include <active_perception/uniform_grid.h>
// #include <lkh_tsp_solver/lkh_interface.h>
// #include <lkh_mtsp_solver/lkh3_interface.h>

#include <cmath>
#include <fstream>
#include <limits>
#include <map>
#include <sstream>
#include <unordered_set>

using Eigen::Vector4d;

namespace fast_planner {
namespace {
constexpr int kFrontierVisClearCount = 256;
constexpr double kPeerStateFreshnessSec = 0.5;
constexpr int kPlanFailureSoftRetryThreshold = 4;
constexpr int kPlanFailurePartialReleaseThreshold = 8;
constexpr int kPlanFailureAggressiveThreshold = 12;
constexpr int kPlanFailureMaxPartialReliefRounds = 2;
constexpr double kAssignmentReliefCooldownSec = 2.0;
constexpr int kPairOptTransactionalRepairOnlyStatus = 5;
constexpr bool kPairOptTransactionalRepairOnly = true;

std::string idsToString(const vector<int>& ids) {
  std::ostringstream oss;
  oss << "[";
  for (int i = 0; i < static_cast<int>(ids.size()); ++i) {
    if (i != 0) {
      oss << ",";
    }
    oss << ids[i];
  }
  oss << "]";
  return oss.str();
}

std::string versionIdsToString(const vector<uint64_t>& ids) {
  std::ostringstream oss;
  oss << "[";
  for (int i = 0; i < static_cast<int>(ids.size()); ++i) {
    if (i != 0) {
      oss << ",";
    }
    oss << ids[i];
  }
  oss << "]";
  return oss.str();
}

bool needsImmediateAssignmentReplan(
    const vector<int>& prev_ids, const vector<int>& new_ids,
    const shared_ptr<FastExplorationManager>& expl_manager) {
  if (prev_ids.empty() != new_ids.empty()) {
    return true;
  }
  if (prev_ids.empty() || new_ids.empty()) {
    return false;
  }
  if (!expl_manager || !expl_manager->hgrid_) {
    return true;
  }
  return !expl_manager->hgrid_->isConsistent(prev_ids.front(), new_ids.front());
}
}
void FastExplorationFSM::init(ros::NodeHandle& nh) {
  fp_.reset(new FSMParam);
  fd_.reset(new FSMData);

  /*  Fsm param  */
  nh.param("fsm/thresh_replan1", fp_->replan_thresh1_, -1.0);
  nh.param("fsm/thresh_replan2", fp_->replan_thresh2_, -1.0);
  nh.param("fsm/thresh_replan3", fp_->replan_thresh3_, -1.0);
  nh.param("fsm/replan_time", fp_->replan_time_, -1.0);
  nh.param("fsm/idle_retry_interval", fp_->idle_retry_interval_, 100.0);
  nh.param("fsm/auto_start", fp_->auto_start_, false);
  nh.param("fsm/auto_start_retry_interval", fp_->auto_start_retry_interval_, 0.5);
  nh.param("fsm/remote_map_replan_interval", fp_->remote_map_replan_interval_, 1.0);
  nh.param("fsm/attempt_interval", fp_->attempt_interval_, 0.2);
  nh.param("fsm/pair_opt_interval", fp_->pair_opt_interval_, 1.0);
  nh.param("fsm/repeat_send_num", fp_->repeat_send_num_, 10);

  /* Initialize main modules */
  expl_manager_.reset(new FastExplorationManager);
  expl_manager_->initialize(nh);
  visualization_.reset(new PlanningVisualization(nh));

  planner_manager_ = expl_manager_->planner_manager_;
  state_ = EXPL_STATE::INIT;
  fd_->have_odom_ = false;
  fd_->state_str_ = { "INIT", "WAIT_TRIGGER", "PLAN_TRAJ", "PUB_TRAJ", "EXEC_TRAJ", "FINISH", "IDL"
                                                                                              "E" };
  fd_->static_state_ = true;
  fd_->trigger_ = false;
  fd_->avoid_collision_ = false;
  fd_->go_back_ = false;
  fd_->aggressive_reassign_requested_ = false;
  fd_->consecutive_plan_failures_ = 0;
  fd_->plan_failure_relief_rounds_ = 0;
  fd_->last_safety_replan_time_ = ros::Time(0);
  fd_->last_plan_fail_time_ = ros::Time(0);
  fd_->last_plan_success_time_ = ros::Time(0);
  fd_->last_assignment_relief_time_ = ros::Time(0);
  current_role_ = relay_racer_integration::RelayRoleCmd::ROLE_EXPLORER;
  have_relay_target_ = false;
  relay_target_.setZero();
  relay_yaw_ = 0.0;
  component_epoch_ = 0;
  component_leader_id_ = 0;
  component_members_initialized_ = false;
  last_component_member_ids_.clear();
  pending_assignment_active_ = false;
  pending_assignment_txn_id_ = 0;
  pending_assignment_component_epoch_ = 0;
  pending_assignment_leader_id_ = 0;
  pending_assignment_grid_id_ = 0;
  pending_assignment_owner_id_ = 0;
  pending_assignment_owner_version_ = 0;
  next_allocation_request_txn_id_ = 1;
  next_release_request_txn_id_ = 1;

  /* Ros sub, pub and timer */
  exec_timer_ = nh.createTimer(ros::Duration(0.01), &FastExplorationFSM::FSMCallback, this);
  safety_timer_ = nh.createTimer(ros::Duration(0.05), &FastExplorationFSM::safetyCallback, this);
  frontier_timer_ = nh.createTimer(ros::Duration(0.5), &FastExplorationFSM::frontierCallback, this);

  trigger_sub_ =
      nh.subscribe("/move_base_simple/goal", 1, &FastExplorationFSM::triggerCallback, this);
  odom_sub_ = nh.subscribe("/odom_world", 1, &FastExplorationFSM::odometryCallback, this);

  string relay_role_topic;
  nh.param("relay/role_cmd_topic", relay_role_topic, string("/relay_integration/relay_role_cmd"));
  relay_role_sub_ =
      nh.subscribe(relay_role_topic, 10, &FastExplorationFSM::relayRoleCmdCallback, this);

  replan_pub_ = nh.advertise<std_msgs::Empty>("/planning/replan", 10);
  new_pub_ = nh.advertise<std_msgs::Empty>("/planning/new", 10);
  bspline_pub_ = nh.advertise<bspline::Bspline>("/planning/bspline", 10);

  // Swarm, timer, pub and sub
  drone_state_timer_ =
      nh.createTimer(ros::Duration(0.04), &FastExplorationFSM::droneStateTimerCallback, this);
  drone_state_pub_ =
      nh.advertise<exploration_manager::DroneState>("/swarm_expl/drone_state_send", 10);
  relay_task_state_pub_ =
      nh.advertise<relay_racer_integration::RelayTaskState>("/relay_integration/relay_task_state_send", 10);
  component_state_pub_ =
      nh.advertise<exploration_manager::ComponentState>("/swarm_expl/component_state_send", 10);
  allocation_request_pub_ =
      nh.advertise<exploration_manager::AllocationRequest>("/swarm_expl/allocation_request", 10);
  release_request_pub_ =
      nh.advertise<exploration_manager::ReleaseRequest>("/swarm_expl/release_request", 10);
  assignment_ack_pub_ =
      nh.advertise<exploration_manager::AssignmentAck>("/swarm_expl/assignment_ack", 10);
  drone_state_sub_ = nh.subscribe(
      "/swarm_expl/drone_state_recv", 10, &FastExplorationFSM::droneStateMsgCallback, this);
  assignment_plan_sub_ = nh.subscribe(
      "/swarm_expl/assignment_plan", 10, &FastExplorationFSM::assignmentPlanCallback, this);
  assignment_commit_sub_ = nh.subscribe(
      "/swarm_expl/assignment_commit", 10, &FastExplorationFSM::assignmentCommitCallback, this);

  opt_timer_ = nh.createTimer(ros::Duration(0.05), &FastExplorationFSM::optTimerCallback, this);
  opt_pub_ = nh.advertise<exploration_manager::PairOpt>("/swarm_expl/pair_opt_send", 10);
  opt_sub_ = nh.subscribe("/swarm_expl/pair_opt_recv", 100, &FastExplorationFSM::optMsgCallback,
      this, ros::TransportHints().tcpNoDelay());

  opt_res_pub_ =
      nh.advertise<exploration_manager::PairOptResponse>("/swarm_expl/pair_opt_res_send", 10);
  opt_res_sub_ = nh.subscribe("/swarm_expl/pair_opt_res_recv", 100,
      &FastExplorationFSM::optResMsgCallback, this, ros::TransportHints().tcpNoDelay());

  swarm_traj_pub_ = nh.advertise<bspline::Bspline>("/planning/swarm_traj_send", 100);
  swarm_traj_sub_ =
      nh.subscribe("/planning/swarm_traj_recv", 100, &FastExplorationFSM::swarmTrajCallback, this);
  swarm_traj_timer_ =
      nh.createTimer(ros::Duration(0.1), &FastExplorationFSM::swarmTrajTimerCallback, this);

  hgrid_pub_ = nh.advertise<exploration_manager::HGrid>("/swarm_expl/hgrid_send", 10);
  grid_tour_pub_ = nh.advertise<exploration_manager::GridTour>("/swarm_expl/grid_tour_send", 10);
}

int FastExplorationFSM::getId() {
  return expl_manager_->ep_->drone_id_;
}

void FastExplorationFSM::getEffectiveSelfGridIds(vector<int>& grid_ids) const {
  grid_ids.clear();
  if (!expl_manager_ || !expl_manager_->ed_ || !expl_manager_->ep_) {
    return;
  }
  if (current_role_ == relay_racer_integration::RelayRoleCmd::ROLE_RELAY) {
    return;
  }

  const auto& self_state = expl_manager_->ed_->swarm_state_[expl_manager_->ep_->drone_id_ - 1];
  const auto& pending_release_ids = expl_manager_->ed_->pending_release_grid_ids_;
  if (pending_release_ids.empty()) {
    grid_ids = self_state.grid_ids_;
    return;
  }

  unordered_set<int> pending_release_map(
      pending_release_ids.begin(), pending_release_ids.end());
  for (const int grid_id : self_state.grid_ids_) {
    if (pending_release_map.find(grid_id) == pending_release_map.end()) {
      grid_ids.push_back(grid_id);
    }
  }
}

void FastExplorationFSM::stagePendingSelfRelease(const vector<int>& grid_ids) {
  if (!expl_manager_ || !expl_manager_->ed_ || grid_ids.empty()) {
    return;
  }

  auto& pending_release_ids = expl_manager_->ed_->pending_release_grid_ids_;
  unordered_set<int> pending_release_map(
      pending_release_ids.begin(), pending_release_ids.end());
  for (const int grid_id : grid_ids) {
    if (pending_release_map.insert(grid_id).second) {
      pending_release_ids.push_back(grid_id);
    }
  }
}

bool FastExplorationFSM::getPendingAllocationCandidates(vector<int>& grid_ids) const {
  grid_ids.clear();
  if (!expl_manager_ || !expl_manager_->ed_) {
    return false;
  }

  std::unordered_set<int> seen;
  auto append_unique = [&](const vector<int>& src) {
    for (const int grid_id : src) {
      if (seen.insert(grid_id).second) {
        grid_ids.push_back(grid_id);
      }
    }
  };

  append_unique(expl_manager_->ed_->pending_relay_recover_grid_ids_);
  append_unique(expl_manager_->ed_->pending_claim_grid_ids_);
  return !grid_ids.empty();
}

void FastExplorationFSM::selectPlanFailureReleaseGrids(
    const vector<int>& effective_self_grid_ids, bool aggressive, vector<int>& release_grid_ids) {
  release_grid_ids.clear();
  if (effective_self_grid_ids.empty()) {
    return;
  }
  if (aggressive || effective_self_grid_ids.size() <= 1 || !expl_manager_ || !expl_manager_->hgrid_) {
    release_grid_ids = effective_self_grid_ids;
    return;
  }

  int best_index = std::min<int>(1, effective_self_grid_ids.size() - 1);
  double best_cost = -1.0;
  for (int i = best_index; i < static_cast<int>(effective_self_grid_ids.size()); ++i) {
    const double grid_cost = expl_manager_->hgrid_->getCostDroneToGrid(
        fd_->odom_pos_, fd_->odom_vel_, effective_self_grid_ids[i], {});
    if (grid_cost > best_cost + 1e-6 ||
        (std::fabs(grid_cost - best_cost) <= 1e-6 &&
            effective_self_grid_ids[i] > effective_self_grid_ids[best_index])) {
      best_cost = grid_cost;
      best_index = i;
    }
  }
  release_grid_ids.push_back(effective_self_grid_ids[best_index]);
}

void FastExplorationFSM::clearPendingAssignmentTxn() {
  pending_assignment_active_ = false;
  pending_assignment_txn_id_ = 0;
  pending_assignment_component_epoch_ = 0;
  pending_assignment_leader_id_ = 0;
  pending_assignment_grid_id_ = 0;
  pending_assignment_owner_id_ = 0;
  pending_assignment_owner_version_ = 0;
}

void FastExplorationFSM::clearLegacyPairOptState(const string& reason) {
  if (!expl_manager_ || !expl_manager_->ed_) {
    return;
  }

  auto ed = expl_manager_->ed_;
  const bool had_state = !ed->ego_ids_.empty() || !ed->other_ids_.empty() ||
                         !ed->pending_pair_opt_grid_ids_.empty() ||
                         !ed->pending_pair_opt_peer_grid_ids_.empty() ||
                         !ed->pending_commit_grid_ids_.empty() ||
                         !ed->pending_commit_peer_grid_ids_.empty() ||
                         ed->wait_response_ || std::fabs(ed->pair_opt_stamp_) > 1e-6;
  if (had_state) {
    ROS_INFO_STREAM("[FSM]: Drone " << getId() << " cleared legacy pair-opt state reason="
                    << reason << ", ego=" << idsToString(ed->ego_ids_)
                    << ", other=" << idsToString(ed->other_ids_)
                    << ", pending_pair_opt_self=" << idsToString(ed->pending_pair_opt_grid_ids_)
                    << ", pending_pair_opt_peer="
                    << idsToString(ed->pending_pair_opt_peer_grid_ids_)
                    << ", pending_commit_self=" << idsToString(ed->pending_commit_grid_ids_)
                    << ", pending_commit_peer="
                    << idsToString(ed->pending_commit_peer_grid_ids_)
                    << ", wait_response=" << (ed->wait_response_ ? "true" : "false")
                    << ", pair_opt_stamp=" << ed->pair_opt_stamp_);
  }

  ed->ego_ids_.clear();
  ed->other_ids_.clear();
  ed->pending_pair_opt_grid_ids_.clear();
  ed->pending_pair_opt_peer_grid_ids_.clear();
  ed->pending_commit_grid_ids_.clear();
  ed->pending_commit_peer_grid_ids_.clear();
  ed->wait_response_ = false;
  ed->pair_opt_stamp_ = 0.0;
}

void FastExplorationFSM::logOwnershipEvent(const string& chain, const string& event,
    uint64_t txn_id, uint64_t component_epoch, int leader_id, int requester_id,
    const vector<int>& grid_ids, const string& detail) const {
  const int drone_id =
      expl_manager_ && expl_manager_->ep_ ? expl_manager_->ep_->drone_id_ : -1;
  std::ostringstream oss;
  oss << "[OWNERSHIP][" << chain << "][" << event << "] drone_id=" << drone_id
      << ", txn_id=" << txn_id << ", component_epoch=" << component_epoch
      << ", leader_id=" << leader_id << ", requester_id=" << requester_id
      << ", pending_active=" << (pending_assignment_active_ ? "true" : "false")
      << ", pending_txn_id=" << pending_assignment_txn_id_
      << ", pending_component_epoch=" << pending_assignment_component_epoch_
      << ", pending_leader_id=" << pending_assignment_leader_id_
      << ", staged_grid_id=" << pending_assignment_grid_id_
      << ", staged_owner_id=" << pending_assignment_owner_id_
      << ", staged_owner_version=" << pending_assignment_owner_version_
      << ", grids=" << idsToString(grid_ids);
  if (!detail.empty()) {
    oss << ", " << detail;
  }
  ROS_INFO_STREAM(oss.str());
}

void FastExplorationFSM::logPlanFailureRecovery(const string& stage, int failure_count,
    const vector<int>& effective_self_grid_ids, const vector<int>& release_grid_ids,
    bool trigger_aggressive_reassign, const string& reason) const {
  const int drone_id =
      expl_manager_ && expl_manager_->ep_ ? expl_manager_->ep_->drone_id_ : -1;
  const int remaining_after_release = std::max(
      0, static_cast<int>(effective_self_grid_ids.size()) - static_cast<int>(release_grid_ids.size()));
  std::ostringstream oss;
  oss << "[PLAN_FAILURE][" << stage << "] drone_id=" << drone_id
      << ", consecutive_failures=" << failure_count
      << ", relief_rounds=" << (fd_ ? fd_->plan_failure_relief_rounds_ : -1)
      << ", component_epoch=" << component_epoch_
      << ", leader_id=" << component_leader_id_
      << ", pending_assignment_active=" << (pending_assignment_active_ ? "true" : "false")
      << ", current_grids=" << idsToString(effective_self_grid_ids)
      << ", released_grids=" << idsToString(release_grid_ids)
      << ", remaining_after_release=" << remaining_after_release
      << ", aggressive_reassign=" << (trigger_aggressive_reassign ? "true" : "false")
      << ", reason=" << reason;
  ROS_WARN_STREAM(oss.str());
}

bool FastExplorationFSM::publishAllocationRequest(const string& reason) {
  if (!component_members_initialized_ || component_leader_id_ <= 0) {
    return false;
  }

  vector<int> request_grid_ids;
  if (!getPendingAllocationCandidates(request_grid_ids)) {
    return false;
  }

  if (pending_assignment_active_) {
    logOwnershipEvent("assignment_request", "deferred_pending_txn",
        pending_assignment_txn_id_, pending_assignment_component_epoch_,
        pending_assignment_leader_id_, getId(), request_grid_ids,
        "reason=" + reason + ", staged_claim=" +
            idsToString(expl_manager_->ed_->pending_claim_grid_ids_) +
            ", staged_relay_recover=" +
            idsToString(expl_manager_->ed_->pending_relay_recover_grid_ids_));
    return false;
  }

  clearLegacyPairOptState(std::string("publishAllocationRequest:") + reason);

  exploration_manager::AllocationRequest request_msg;
  request_msg.component_epoch = component_epoch_;
  request_msg.leader_id = component_leader_id_;
  request_msg.requester_id = getId();
  request_msg.txn_id = next_allocation_request_txn_id_++;
  request_msg.reason = reason;
  request_msg.stamp = ros::Time::now().toSec();
  request_msg.grid_ids = request_grid_ids;
  allocation_request_pub_.publish(request_msg);
  logOwnershipEvent("assignment_request", "published", request_msg.txn_id,
      request_msg.component_epoch, request_msg.leader_id, request_msg.requester_id,
      request_grid_ids,
      "reason=" + reason + ", staged_claim=" +
          idsToString(expl_manager_->ed_->pending_claim_grid_ids_) +
          ", staged_relay_recover=" +
          idsToString(expl_manager_->ed_->pending_relay_recover_grid_ids_));
  return true;
}

bool FastExplorationFSM::publishReleaseRequest(const vector<int>& grid_ids, const string& reason) {
  if (!component_members_initialized_ || component_leader_id_ <= 0 || grid_ids.empty()) {
    return false;
  }

  clearLegacyPairOptState(std::string("publishReleaseRequest:") + reason);

  exploration_manager::ReleaseRequest request_msg;
  request_msg.component_epoch = component_epoch_;
  request_msg.leader_id = component_leader_id_;
  request_msg.grid_ids = grid_ids;
  request_msg.txn_id = next_release_request_txn_id_++;
  request_msg.reason = reason;
  request_msg.stamp = ros::Time::now().toSec();
  request_msg.owner_ids.assign(grid_ids.size(), getId());
  request_msg.owner_versions.reserve(grid_ids.size());
  for (const int grid_id : grid_ids) {
    const auto version_it = component_owner_versions_.find(grid_id);
    request_msg.owner_versions.push_back(
        version_it == component_owner_versions_.end() ? 1 : version_it->second);
  }
  release_request_pub_.publish(request_msg);
  logOwnershipEvent("release_request", "published", request_msg.txn_id,
      request_msg.component_epoch, request_msg.leader_id, getId(), grid_ids,
      "reason=" + reason + ", owner_versions=" +
          versionIdsToString(request_msg.owner_versions) +
          ", staged_release=" + idsToString(expl_manager_->ed_->pending_release_grid_ids_));
  return true;
}

bool FastExplorationFSM::handleRemoteMapUpdate(const ros::Time& now) {
  if (!fd_->have_odom_ || current_role_ == relay_racer_integration::RelayRoleCmd::ROLE_RELAY) {
    return false;
  }
  if (!expl_manager_ || !expl_manager_->sdf_map_ || !expl_manager_->sdf_map_->mm_ ||
      !expl_manager_->sdf_map_->mm_->hasRemoteChunkUpdate()) {
    return false;
  }
  if (!fd_->last_remote_map_replan_time_.isZero() &&
      (now - fd_->last_remote_map_replan_time_).toSec() < fp_->remote_map_replan_interval_) {
    return false;
  }

  expl_manager_->sdf_map_->mm_->consumeRemoteChunkUpdate();
  fd_->last_remote_map_replan_time_ = now;

  const int frontier_status = expl_manager_->updateFrontierStruct(fd_->odom_pos_);
  const bool frontier_changed =
      expl_manager_->frontier_finder_ && expl_manager_->frontier_finder_->hasFrontierStructureChange();

  auto ed = expl_manager_->ed_;
  auto& self_state = ed->swarm_state_[getId() - 1];
  vector<int> effective_self_grid_ids;
  getEffectiveSelfGridIds(effective_self_grid_ids);
  const int prev_grid_count = static_cast<int>(effective_self_grid_ids.size());
  vector<int> active_grids;
  vector<int> remaining_grid_ids;
  expl_manager_->hgrid_->getActiveGrids(active_grids);
  if (prev_grid_count > 0) {
    unordered_map<int, char> active_grid_map;
    for (const int grid_id : active_grids) {
      active_grid_map[grid_id] = 1;
    }

    vector<int> invalidated_grid_ids;
    remaining_grid_ids.reserve(effective_self_grid_ids.size());
    invalidated_grid_ids.reserve(effective_self_grid_ids.size());
    vector<int> single_grid(1, -1);
    vector<int> frontier_ids;
    int removed_inactive_grid_count = 0;
    int removed_frontierless_grid_count = 0;
    for (const int grid_id : effective_self_grid_ids) {
      if (active_grid_map.find(grid_id) == active_grid_map.end()) {
        invalidated_grid_ids.push_back(grid_id);
        ++removed_inactive_grid_count;
        continue;
      }

      if (expl_manager_->hgrid_->isFineGrid(grid_id)) {
        frontier_ids.clear();
        single_grid[0] = grid_id;
        expl_manager_->hgrid_->getFrontiersInGrid(single_grid, frontier_ids);
        if (frontier_ids.empty()) {
          invalidated_grid_ids.push_back(grid_id);
          ++removed_frontierless_grid_count;
          continue;
        }
      }

      remaining_grid_ids.push_back(grid_id);
    }

    if (!invalidated_grid_ids.empty()) {
      ed->pending_invalidated_grid_ids_ = invalidated_grid_ids;
      stagePendingSelfRelease(invalidated_grid_ids);
      publishReleaseRequest(ed->pending_invalidated_grid_ids_, "remoteMapInvalidation");
      ROS_INFO_STREAM("[FSM]: Drone " << getId() << " staged release of "
                      << (removed_inactive_grid_count + removed_frontierless_grid_count)
                      << " stale grids after remote map update (inactive="
                      << removed_inactive_grid_count << ", frontierless="
                      << removed_frontierless_grid_count << ").");
    } else {
      ed->pending_invalidated_grid_ids_.clear();
    }
  } else {
    ed->pending_invalidated_grid_ids_.clear();
  }

  const bool lost_assignment_after_prune = prev_grid_count > 0 && remaining_grid_ids.empty();
  if (lost_assignment_after_prune) {
    fd_->aggressive_reassign_requested_ = true;
  }

  ROS_INFO_STREAM("[FSM]: Drone " << getId()
                  << " refreshed frontier after remote map update, status="
                  << frontier_status << ", changed=" << frontier_changed
                  << ", active_grids=" << active_grids.size()
                  << ", assigned_grids=" << remaining_grid_ids.size());

  if (state_ == WAIT_TRIGGER && fp_->auto_start_ && frontier_status != 0) {
    fd_->trigger_ = true;
    fd_->start_pos_ = fd_->odom_pos_;
    transitState(PLAN_TRAJ, "remoteMapUpdate");
    return true;
  }

  if (state_ == IDLE) {
    fd_->last_check_frontier_time_ = now;
    if (tryClaimUnallocatedGrids("remoteMapUpdate", true)) {
      return true;
    }
    if (frontier_status != 0) {
      transitState(PLAN_TRAJ, "remoteMapUpdate");
      return true;
    }
    return false;
  }

  if ((state_ == EXEC_TRAJ || state_ == PUB_TRAJ) &&
      (frontier_status != 0 || frontier_changed || lost_assignment_after_prune)) {
    replan_pub_.publish(std_msgs::Empty());
    transitState(PLAN_TRAJ, "remoteMapUpdate");
    return true;
  }

  return false;
}

void FastExplorationFSM::FSMCallback(const ros::TimerEvent& e) {
  ROS_INFO_STREAM_THROTTLE(
      1.0, "[FSM]: Drone " << getId() << " state: " << fd_->state_str_[int(state_)]);

  if (state_ != INIT && handleRemoteMapUpdate(ros::Time::now())) {
    return;
  }

  switch (state_) {
    case INIT: {
      // Wait for odometry ready
      if (!fd_->have_odom_) {
        ROS_WARN_THROTTLE(1.0, "no odom");
        return;
      }
      if ((ros::Time::now() - fd_->fsm_init_time_).toSec() < 2.0) {
        ROS_WARN_THROTTLE(1.0, "wait for init");
        return;
      }
      // Go to wait trigger when odom is ok
      transitState(WAIT_TRIGGER, "FSM");
      break;
    }

    case WAIT_TRIGGER: {
      if (fp_->auto_start_) {
        const double retry_interval = std::max(0.05, fp_->auto_start_retry_interval_);
        const double check_interval = (ros::Time::now() - fd_->last_check_frontier_time_).toSec();
        if (check_interval >= retry_interval) {
          if (!tryStartExploration("autoStart", false)) {
            fd_->last_check_frontier_time_ = ros::Time::now();
            ROS_WARN_STREAM_THROTTLE(1.0, "auto-start waiting for frontier.");
          }
        }
        break;
      }

      ROS_WARN_THROTTLE(1.0, "wait for trigger.");
      break;
    }

    case FINISH: {
      ROS_INFO_THROTTLE(1.0, "finish exploration.");
      break;
    }

    case IDLE: {
      if (tryClaimUnallocatedGrids("idleRecovery", true)) {
        break;
      }

      double check_interval = (ros::Time::now() - fd_->last_check_frontier_time_).toSec();
      if (check_interval > fp_->idle_retry_interval_) {
        const int frontier_status = expl_manager_->updateFrontierStruct(fd_->odom_pos_);
        if (frontier_status != 0) {
          ROS_WARN_STREAM("Retry frontier planning after idle wait of "
                          << fp_->idle_retry_interval_ << " s");
          fd_->go_back_ = false;
          transitState(PLAN_TRAJ, "FSM");
        } else {
          fd_->last_check_frontier_time_ = ros::Time::now();
          ROS_WARN_STREAM("Still no frontier after idle wait of "
                          << fp_->idle_retry_interval_ << " s");
        }
      }
      break;
    }

    case PLAN_TRAJ: {
      if (fd_->static_state_) {
        // Plan from static state (hover)
        fd_->start_pt_ = fd_->odom_pos_;
        fd_->start_vel_ = fd_->odom_vel_;
        fd_->start_acc_.setZero();
        fd_->start_yaw_ << fd_->odom_yaw_, 0, 0;
      } else {
        // Replan from non-static state, starting from 'replan_time' seconds later
        LocalTrajData* info = &planner_manager_->local_data_;
        double t_r = (ros::Time::now() - info->start_time_).toSec() + fp_->replan_time_;
        fd_->start_pt_ = info->position_traj_.evaluateDeBoorT(t_r);
        fd_->start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_r);
        fd_->start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_r);
        fd_->start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_r)[0];
        fd_->start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_r)[0];
        fd_->start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_r)[0];
      }
      // Inform traj_server the replanning
      replan_pub_.publish(std_msgs::Empty());
      int res = callExplorationPlanner();
      if (res == SUCCEED) {
        fd_->consecutive_plan_failures_ = 0;
        fd_->plan_failure_relief_rounds_ = 0;
        fd_->last_assignment_relief_time_ = ros::Time(0);
        fd_->last_plan_success_time_ = ros::Time::now();
        transitState(PUB_TRAJ, "FSM");
      } else if (res == FAIL) {  // Keep trying to replan
        fd_->static_state_ = true;
        fd_->last_plan_fail_time_ = ros::Time::now();
        ++fd_->consecutive_plan_failures_;
        ROS_WARN_STREAM("Plan fail, consecutive_failures=" << fd_->consecutive_plan_failures_);

        if (current_role_ != relay_racer_integration::RelayRoleCmd::ROLE_RELAY) {
          vector<int> effective_self_grid_ids;
          getEffectiveSelfGridIds(effective_self_grid_ids);
          const int failure_count = fd_->consecutive_plan_failures_;

          if (!effective_self_grid_ids.empty() &&
              failure_count == kPlanFailureSoftRetryThreshold) {
            logPlanFailureRecovery("level1_local_retry", failure_count, effective_self_grid_ids, {},
                false, "holding ownership and retrying local recovery first");
          }

          if (!effective_self_grid_ids.empty() &&
              failure_count >= kPlanFailurePartialReleaseThreshold) {
            if (pending_assignment_active_) {
              logPlanFailureRecovery("deferred_pending_assignment_txn", failure_count,
                  effective_self_grid_ids, {}, false,
                  "staged assignment txn is still active; deferring failure relief");
            } else {
              const ros::Time now = ros::Time::now();
              const bool relief_cooldown_active =
                  !fd_->last_assignment_relief_time_.isZero() &&
                  (now - fd_->last_assignment_relief_time_).toSec() < kAssignmentReliefCooldownSec;
              const bool single_grid_assignment = effective_self_grid_ids.size() <= 1;
              const bool aggressive_reassign_stage =
                  failure_count >= kPlanFailureAggressiveThreshold &&
                  (fd_->plan_failure_relief_rounds_ >= kPlanFailureMaxPartialReliefRounds ||
                      single_grid_assignment);
              if (relief_cooldown_active) {
                logPlanFailureRecovery("cooldown_hold", failure_count, effective_self_grid_ids, {},
                    false, "within assignment-relief cooldown");
              } else if (aggressive_reassign_stage) {
                vector<int> release_grid_ids;
                selectPlanFailureReleaseGrids(effective_self_grid_ids, true, release_grid_ids);
                if (!release_grid_ids.empty()) {
                  auto ed = expl_manager_->ed_;
                  ed->pending_plan_fail_release_grid_ids_ = release_grid_ids;
                  const bool published = publishReleaseRequest(
                      ed->pending_plan_fail_release_grid_ids_, "repeatedPlanFailureAggressive");
                  if (!published) {
                    ed->pending_plan_fail_release_grid_ids_.clear();
                    logPlanFailureRecovery("level3_full_release_deferred", failure_count,
                        effective_self_grid_ids, release_grid_ids, false,
                        "publishReleaseRequest failed for aggressive stage");
                  } else {
                    stagePendingSelfRelease(release_grid_ids);
                    fd_->last_assignment_relief_time_ = now;
                    fd_->last_check_frontier_time_ = now;
                    fd_->consecutive_plan_failures_ = 0;
                    fd_->plan_failure_relief_rounds_ = 0;
                    logPlanFailureRecovery("level3_full_release_aggressive", failure_count,
                        effective_self_grid_ids, release_grid_ids, true,
                        single_grid_assignment
                            ? "single remaining grid kept failing; escalated to full release"
                            : "partial relief exhausted; escalated to full release");
                    transitState(IDLE, "repeatedPlanFailAggressive");
                    requestAggressiveReassign(
                        "repeatedPlanFailureAggressive", false /* immediate_claim */);
                    visualize(1);
                  }
                }
              } else if (single_grid_assignment) {
                logPlanFailureRecovery("level2_hold_single_grid", failure_count,
                    effective_self_grid_ids, {}, false,
                    "single remaining grid kept local until aggressive threshold is reached");
              } else {
                vector<int> release_grid_ids;
                selectPlanFailureReleaseGrids(effective_self_grid_ids, false, release_grid_ids);
                if (!release_grid_ids.empty()) {
                  auto ed = expl_manager_->ed_;
                  ed->pending_plan_fail_release_grid_ids_ = release_grid_ids;
                  const bool published = publishReleaseRequest(
                      ed->pending_plan_fail_release_grid_ids_, "repeatedPlanFailurePartial");
                  if (!published) {
                    ed->pending_plan_fail_release_grid_ids_.clear();
                    logPlanFailureRecovery("level2_partial_release_deferred", failure_count,
                        effective_self_grid_ids, release_grid_ids, false,
                        "publishReleaseRequest failed for partial release");
                  } else {
                    stagePendingSelfRelease(release_grid_ids);
                    fd_->last_assignment_relief_time_ = now;
                    fd_->last_check_frontier_time_ = now;
                    ++fd_->plan_failure_relief_rounds_;
                    fd_->consecutive_plan_failures_ = kPlanFailureSoftRetryThreshold;
                    logPlanFailureRecovery("level2_partial_release", failure_count,
                        effective_self_grid_ids, release_grid_ids, false,
                        "released the most expensive local subset and kept local replanning");
                  }
                }
              }
            }
          }
        }
      } else if (res == NO_GRID) {
        fd_->consecutive_plan_failures_ = 0;
        fd_->plan_failure_relief_rounds_ = 0;
        fd_->last_assignment_relief_time_ = ros::Time(0);
        fd_->static_state_ = true;
        fd_->last_check_frontier_time_ = ros::Time::now();
        ROS_WARN_STREAM("No grid, enter IDLE and retry after " << fp_->idle_retry_interval_ << " s");
        transitState(IDLE, "FSM");
        visualize(1);
        // clearVisMarker();
      }
      break;
    }

    case PUB_TRAJ: {
      double dt = (ros::Time::now() - fd_->newest_traj_.start_time).toSec();
      if (dt > 0) {
        bspline_pub_.publish(fd_->newest_traj_);
        fd_->static_state_ = false;

        // fd_->newest_traj_.drone_id = planner_manager_->swarm_traj_data_.drone_id_;
        fd_->newest_traj_.drone_id = expl_manager_->ep_->drone_id_;
        swarm_traj_pub_.publish(fd_->newest_traj_);

        thread vis_thread(&FastExplorationFSM::visualize, this, 2);
        vis_thread.detach();
        transitState(EXEC_TRAJ, "FSM");
      }
      break;
    }

    case EXEC_TRAJ: {
      auto tn = ros::Time::now();
      // Check whether replan is needed
      LocalTrajData* info = &planner_manager_->local_data_;
      double t_cur = (tn - info->start_time_).toSec();

      if (current_role_ == relay_racer_integration::RelayRoleCmd::ROLE_RELAY) {
        if (t_cur > fp_->replan_thresh3_ || info->duration_ - t_cur < fp_->replan_thresh1_) {
          replan_pub_.publish(std_msgs::Empty());
          transitState(PLAN_TRAJ, "FSM");
        }
      } else if (!fd_->go_back_) {
        bool need_replan = false;
        if (t_cur > fp_->replan_thresh2_ && expl_manager_->frontier_finder_->isFrontierCovered()) {
          ROS_WARN("Replan: cluster covered=====================================");
          need_replan = true;
        } else if (info->duration_ - t_cur < fp_->replan_thresh1_) {
          // Replan if traj is almost fully executed
          ROS_WARN("Replan: traj fully executed=================================");
          need_replan = true;
        } else if (t_cur > fp_->replan_thresh3_) {
          // Replan after some time
          ROS_WARN("Replan: periodic call=======================================");
          need_replan = true;
        }

        if (need_replan) {
          if (expl_manager_->updateFrontierStruct(fd_->odom_pos_) != 0) {
            // Update frontier and plan new motion
            thread vis_thread(&FastExplorationFSM::visualize, this, 1);
            vis_thread.detach();
            transitState(PLAN_TRAJ, "FSM");
          } else {
            // No frontier detected, finish exploration
            fd_->last_check_frontier_time_ = ros::Time::now();
            transitState(IDLE, "FSM");
            ROS_WARN("Idle since no frontier is detected");
            fd_->static_state_ = true;
            replan_pub_.publish(std_msgs::Empty());
            // clearVisMarker();
            visualize(1);
          }
        }
      } else {
        // Check if reach goal
        auto pos = info->position_traj_.evaluateDeBoorT(t_cur);
        if ((pos - expl_manager_->ed_->next_pos_).norm() < 1.0) {
          replan_pub_.publish(std_msgs::Empty());
          clearVisMarker();
          transitState(FINISH, "FSM");
          return;
        }
        if (t_cur > fp_->replan_thresh3_ || info->duration_ - t_cur < fp_->replan_thresh1_) {
          // Replan for going back
          replan_pub_.publish(std_msgs::Empty());
          transitState(PLAN_TRAJ, "FSM");
          thread vis_thread(&FastExplorationFSM::visualize, this, 1);
          vis_thread.detach();
        }
      }

      break;
    }
  }
}

int FastExplorationFSM::callExplorationPlanner() {
  ros::Time time_r = ros::Time::now() + ros::Duration(fp_->replan_time_);

  int res;
  if (fd_->avoid_collision_ || fd_->go_back_) {  // Only replan trajectory
    res = expl_manager_->planTrajToView(fd_->start_pt_, fd_->start_vel_, fd_->start_acc_,
        fd_->start_yaw_, expl_manager_->ed_->next_pos_, expl_manager_->ed_->next_yaw_);
    fd_->avoid_collision_ = false;
  } else if (current_role_ == relay_racer_integration::RelayRoleCmd::ROLE_RELAY) {
    // Stage-1 relay mode reuses the point-to-view planner as a thin adapter hook.
    ROS_INFO_STREAM("[Relay]: Drone " << getId() << " planning toward relay target "
                    << relay_target_.transpose());
    Vector3d relay_goal = relay_target_;
    double relay_goal_yaw = relay_yaw_;
    if (!have_relay_target_) {
      ROS_WARN_THROTTLE(1.0, "Relay role active without a valid relay target, holding current pose.");
      relay_goal = fd_->start_pt_;
      relay_goal_yaw = fd_->start_yaw_(0);
    }

    expl_manager_->ed_->next_pos_ = relay_goal;
    expl_manager_->ed_->next_yaw_ = relay_goal_yaw;
    res = expl_manager_->planTrajToView(fd_->start_pt_, fd_->start_vel_, fd_->start_acc_,
        fd_->start_yaw_, relay_goal, relay_goal_yaw);
  } else {  // Do full planning normally
    res = expl_manager_->planExploreMotion(
        fd_->start_pt_, fd_->start_vel_, fd_->start_acc_, fd_->start_yaw_);
  }

  if (res == SUCCEED) {
    auto info = &planner_manager_->local_data_;
    info->start_time_ = (ros::Time::now() - time_r).toSec() > 0 ? ros::Time::now() : time_r;

    bspline::Bspline bspline;
    bspline.order = planner_manager_->pp_.bspline_degree_;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;
    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }
    Eigen::VectorXd knots = info->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }
    Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
    for (int i = 0; i < yaw_pts.rows(); ++i) {
      double yaw = yaw_pts(i, 0);
      bspline.yaw_pts.push_back(yaw);
    }
    bspline.yaw_dt = info->yaw_traj_.getKnotSpan();
    fd_->newest_traj_ = bspline;
  }
  return res;
}

void FastExplorationFSM::visualize(int content) {
  // content 1: frontier; 2 paths & trajs
  auto info = &planner_manager_->local_data_;
  auto plan_data = &planner_manager_->plan_data_;
  auto ed_ptr = expl_manager_->ed_;

  auto getColorVal = [&](const int& id, const int& num, const int& drone_id) {
    double a = (drone_id - 1) / double(num + 1);
    double b = 1 / double(num + 1);
    return a + b * double(id) / ed_ptr->frontiers_.size();
  };

  if (content == 1) {
    // Draw frontier
    static int last_ftr_num = 0;
    static int last_dftr_num = 0;
    for (int i = 0; i < ed_ptr->frontiers_.size(); ++i) {
      visualization_->drawCubes(ed_ptr->frontiers_[i], 0.1,
          visualization_->getColor(double(i) / ed_ptr->frontiers_.size(), 0.4), "frontier", i, 4);

      // getColorVal(i, expl_manager_->ep_->drone_num_, expl_manager_->ep_->drone_id_)
      // double(i) / ed_ptr->frontiers_.size()

      // visualization_->drawBox(ed_ptr->frontier_boxes_[i].first,
      // ed_ptr->frontier_boxes_[i].second,
      //     Vector4d(0.5, 0, 1, 0.3), "frontier_boxes", i, 4);
    }
    for (int i = ed_ptr->frontiers_.size(); i < last_ftr_num; ++i) {
      visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i, 4);
      // visualization_->drawBox(Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector4d(1, 0, 0, 0.3),
      // "frontier_boxes", i, 4);
    }
    last_ftr_num = ed_ptr->frontiers_.size();

    // for (int i = 0; i < ed_ptr->dead_frontiers_.size(); ++i)
    //   visualization_->drawCubes(
    //       ed_ptr->dead_frontiers_[i], 0.1, Vector4d(0, 0, 0, 0.5), "dead_frontier", i, 4);
    // for (int i = ed_ptr->dead_frontiers_.size(); i < last_dftr_num; ++i)
    //   visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 0.5), "dead_frontier", i, 4);
    // last_dftr_num = ed_ptr->dead_frontiers_.size();

    // // Draw updated box
    // Vector3d bmin, bmax;
    // planner_manager_->edt_environment_->sdf_map_->getUpdatedBox(bmin, bmax, false);
    // visualization_->drawBox(
    //     (bmin + bmax) / 2.0, bmax - bmin, Vector4d(0, 1, 0, 0.3), "updated_box", 0, 4);

    // vector<Eigen::Vector3d> bmins, bmaxs;
    // planner_manager_->edt_environment_->sdf_map_->mm_->getChunkBoxes(bmins, bmaxs, false);
    // for (int i = 0; i < bmins.size(); ++i) {
    //   visualization_->drawBox((bmins[i] + bmaxs[i]) / 2.0, bmaxs[i] - bmins[i],
    //       Vector4d(0, 1, 1, 0.3), "updated_box", i + 1, 4);
    // }

  } else if (content == 2) {

    // Hierarchical grid and global tour --------------------------------
    // vector<Eigen::Vector3d> pts1, pts2;
    // expl_manager_->uniform_grid_->getPath(pts1, pts2);
    // visualization_->drawLines(pts1, pts2, 0.05, Eigen::Vector4d(1, 0.3, 0, 1), "partition", 0,
    // 6);

    if (expl_manager_->ep_->drone_id_ == 1) {
      vector<Eigen::Vector3d> pts1, pts2;
      expl_manager_->hgrid_->getGridMarker(pts1, pts2);
      visualization_->drawLines(pts1, pts2, 0.05, Eigen::Vector4d(1, 0, 1, 0.5), "partition", 1, 6);

      vector<Eigen::Vector3d> pts;
      vector<string> texts;
      expl_manager_->hgrid_->getGridMarker2(pts, texts);
      static int last_text_num = 0;
      for (int i = 0; i < pts.size(); ++i) {
        visualization_->drawText(pts[i], texts[i], 1, Eigen::Vector4d(0, 0, 0, 1), "text", i, 6);
      }
      for (int i = pts.size(); i < last_text_num; ++i) {
        visualization_->drawText(
            Eigen::Vector3d(0, 0, 0), string(""), 1, Eigen::Vector4d(0, 0, 0, 1), "text", i, 6);
      }
      last_text_num = pts.size();

      // // Pub hgrid to ground node
      // exploration_manager::HGrid hgrid;
      // hgrid.stamp = ros::Time::now().toSec();
      // for (int i = 0; i < pts1.size(); ++i) {
      //   geometry_msgs::Point pt1, pt2;
      //   pt1.x = pts1[i][0];
      //   pt1.y = pts1[i][1];
      //   pt1.z = pts1[i][2];
      //   hgrid.points1.push_back(pt1);
      //   pt2.x = pts2[i][0];
      //   pt2.y = pts2[i][1];
      //   pt2.z = pts2[i][2];
      //   hgrid.points2.push_back(pt2);
      // }
      // hgrid_pub_.publish(hgrid);
    }

    auto grid_tour = expl_manager_->ed_->grid_tour_;
    // auto grid_tour = expl_manager_->ed_->grid_tour2_;
    // for (auto& pt : grid_tour) pt = pt + trans;

    visualization_->drawLines(grid_tour, 0.05,
        PlanningVisualization::getColor(
            (expl_manager_->ep_->drone_id_ - 1) / double(expl_manager_->ep_->drone_num_)),
        "grid_tour", 0, 6);

    // Publish grid tour to ground node
    exploration_manager::GridTour tour;
    for (int i = 0; i < grid_tour.size(); ++i) {
      geometry_msgs::Point point;
      point.x = grid_tour[i][0];
      point.y = grid_tour[i][1];
      point.z = grid_tour[i][2];
      tour.points.push_back(point);
    }
    tour.drone_id = expl_manager_->ep_->drone_id_;
    tour.stamp = ros::Time::now().toSec();
    grid_tour_pub_.publish(tour);

    // visualization_->drawSpheres(
    //     expl_manager_->ed_->grid_tour_, 0.3, Eigen::Vector4d(0, 1, 0, 1), "grid_tour", 1, 6);
    // visualization_->drawLines(
    //     expl_manager_->ed_->grid_tour2_, 0.05, Eigen::Vector4d(0, 1, 0, 0.5), "grid_tour", 2, 6);

    // Top viewpoints and frontier tour-------------------------------------

    // visualization_->drawSpheres(ed_ptr->points_, 0.2, Vector4d(0, 0.5, 0, 1), "points", 0, 6);
    // visualization_->drawLines(
    //     ed_ptr->points_, ed_ptr->views_, 0.05, Vector4d(0, 1, 0.5, 1), "view", 0, 6);
    // visualization_->drawLines(
    //     ed_ptr->points_, ed_ptr->averages_, 0.03, Vector4d(1, 0, 0, 1), "point-average", 0, 6);

    // auto frontier = ed_ptr->frontier_tour_;
    // for (auto& pt : frontier) pt = pt + trans;
    // visualization_->drawLines(frontier, 0.07,
    //     PlanningVisualization::getColor(
    //         (expl_manager_->ep_->drone_id_ - 1) / double(expl_manager_->ep_->drone_num_), 0.6),
    //     "frontier_tour", 0, 6);

    // for (int i = 0; i < ed_ptr->other_tours_.size(); ++i) {
    //   visualization_->drawLines(
    //       ed_ptr->other_tours_[i], 0.07, Eigen::Vector4d(0, 0, 1, 1), "other_tours", i, 6);
    // }

    // Locally refined viewpoints and refined tour-------------------------------

    // visualization_->drawSpheres(
    //     ed_ptr->refined_points_, 0.2, Vector4d(0, 0, 1, 1), "refined_pts", 0, 6);
    // visualization_->drawLines(
    //     ed_ptr->refined_points_, ed_ptr->refined_views_, 0.05, Vector4d(0.5, 0, 1, 1),
    //     "refined_view", 0, 6);
    // visualization_->drawLines(
    //     ed_ptr->refined_tour_, 0.07,
    //     PlanningVisualization::getColor(
    //         (expl_manager_->ep_->drone_id_ - 1) / double(expl_manager_->ep_->drone_num_), 0.6),
    //     "refined_tour", 0, 6);

    // visualization_->drawLines(ed_ptr->refined_views1_, ed_ptr->refined_views2_, 0.04, Vector4d(0,
    // 0, 0, 1),
    //                           "refined_view", 0, 6);
    // visualization_->drawLines(ed_ptr->refined_points_, ed_ptr->unrefined_points_, 0.05,
    // Vector4d(1, 1, 0, 1),
    //                           "refine_pair", 0, 6);
    // for (int i = 0; i < ed_ptr->n_points_.size(); ++i)
    //   visualization_->drawSpheres(ed_ptr->n_points_[i], 0.1,
    //                               visualization_->getColor(double(ed_ptr->refined_ids_[i]) /
    //                               ed_ptr->frontiers_.size()),
    //                               "n_points", i, 6);
    // for (int i = ed_ptr->n_points_.size(); i < 15; ++i)
    //   visualization_->drawSpheres({}, 0.1, Vector4d(0, 0, 0, 1), "n_points", i, 6);

    // Trajectory-------------------------------------------

    // visualization_->drawSpheres(
    //     { ed_ptr->next_goal_ /* + trans */ }, 0.3, Vector4d(0, 0, 1, 1), "next_goal", 0, 6);

    // vector<Eigen::Vector3d> next_yaw_vis;
    // next_yaw_vis.push_back(ed_ptr->next_goal_ /* + trans */);
    // next_yaw_vis.push_back(
    //     ed_ptr->next_goal_ /* + trans */ +
    //     2.0 * Eigen::Vector3d(cos(ed_ptr->next_yaw_), sin(ed_ptr->next_yaw_), 0));
    // visualization_->drawLines(next_yaw_vis, 0.1, Eigen::Vector4d(0, 0, 1, 1), "next_goal", 1, 6);
    // visualization_->drawSpheres(
    //     { ed_ptr->next_pos_ /* + trans */ }, 0.3, Vector4d(0, 1, 0, 1), "next_pos", 0, 6);

    // Eigen::MatrixXd ctrl_pt = info->position_traj_.getControlPoint();
    // for (int i = 0; i < ctrl_pt.rows(); ++i) {
    //   for (int j = 0; j < 3; ++j) ctrl_pt(i, j) = ctrl_pt(i, j) + trans[j];
    // }
    // NonUniformBspline position_traj(ctrl_pt, 3, info->position_traj_.getKnotSpan());

    visualization_->drawBspline(info->position_traj_, 0.1,
        PlanningVisualization::getColor(
            (expl_manager_->ep_->drone_id_ - 1) / double(expl_manager_->ep_->drone_num_)),
        false, 0.15, Vector4d(1, 1, 0, 1));

    // visualization_->drawLines(
    //     expl_manager_->ed_->path_next_goal_, 0.1, Eigen::Vector4d(0, 1, 0, 1), "astar", 0, 6);
    // visualization_->drawSpheres(
    //     expl_manager_->ed_->kino_path_, 0.1, Eigen::Vector4d(0, 0, 1, 1), "kino", 0, 6);
    // visualization_->drawSpheres(plan_data->kino_path_, 0.1, Vector4d(1, 0, 1, 1), "kino_path", 0,
    // 0); visualization_->drawLines(ed_ptr->path_next_goal_, 0.05, Vector4d(0, 1, 1, 1),
    // "next_goal", 1, 6);

    // // Draw trajs of other drones
    // vector<NonUniformBspline> trajs;
    // planner_manager_->swarm_traj_data_.getValidTrajs(trajs);
    // for (int k = 0; k < trajs.size(); ++k) {
    //   visualization_->drawBspline(trajs[k], 0.1, Eigen::Vector4d(1, 1, 0, 1), false, 0.15,
    //       Eigen::Vector4d(0, 0, 1, 1), k + 1);
    // }
  }
}

void FastExplorationFSM::clearVisMarker() {
  for (int i = 0; i < kFrontierVisClearCount; ++i) {
    visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i, 4);
    // visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "dead_frontier", i, 4);
    // visualization_->drawBox(Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector4d(1, 0, 0, 0.3),
    // "frontier_boxes", i, 4);
  }
  // visualization_->drawSpheres({}, 0.2, Vector4d(0, 0.5, 0, 1), "points", 0, 6);
  visualization_->drawLines({}, 0.07, Vector4d(0, 0.5, 0, 1), "frontier_tour", 0, 6);
  visualization_->drawLines({}, 0.07, Vector4d(0, 0.5, 0, 1), "grid_tour", 0, 6);
  // visualization_->drawSpheres({}, 0.2, Vector4d(0, 0, 1, 1), "refined_pts", 0, 6);
  // visualization_->drawLines({}, {}, 0.05, Vector4d(0.5, 0, 1, 1), "refined_view", 0, 6);
  // visualization_->drawLines({}, 0.07, Vector4d(0, 0, 1, 1), "refined_tour", 0, 6);
  visualization_->drawSpheres({}, 0.1, Vector4d(0, 0, 1, 1), "B-Spline", 0, 0);

  // visualization_->drawLines({}, {}, 0.03, Vector4d(1, 0, 0, 1), "current_pose", 0, 6);
}

void FastExplorationFSM::frontierCallback(const ros::TimerEvent& e) {
  if (state_ == WAIT_TRIGGER) {
    auto ft = expl_manager_->frontier_finder_;
    auto ed = expl_manager_->ed_;

    auto getColorVal = [&](const int& id, const int& num, const int& drone_id) {
      double a = (drone_id - 1) / double(num + 1);
      double b = 1 / double(num + 1);
      return a + b * double(id) / ed->frontiers_.size();
    };

    // ft->searchFrontiers();
    // ft->computeFrontiersToVisit();
    // ft->updateFrontierCostMatrix();

    // ft->getFrontiers(ed->frontiers_);
    // ft->getFrontierBoxes(ed->frontier_boxes_);

    expl_manager_->updateFrontierStruct(fd_->odom_pos_);

    cout << "odom: " << fd_->odom_pos_.transpose() << endl;
    vector<int> tmp_id1;
    vector<vector<int>> tmp_id2;
    bool status = expl_manager_->findGlobalTourOfGridFromSwarm(
        fd_->odom_pos_, fd_->odom_vel_, tmp_id1, tmp_id2, true);

    // Draw frontier and bounding box
    for (int i = 0; i < ed->frontiers_.size(); ++i) {
      visualization_->drawCubes(ed->frontiers_[i], 0.1,
          visualization_->getColor(double(i) / ed->frontiers_.size(), 0.4), "frontier", i, 4);
      // getColorVal(i, expl_manager_->ep_->drone_num_, expl_manager_->ep_->drone_id_)
      // double(i) / ed->frontiers_.size()
      // visualization_->drawBox(ed->frontier_boxes_[i].first, ed->frontier_boxes_[i].second,
      // Vector4d(0.5, 0, 1, 0.3),
      //                         "frontier_boxes", i, 4);
    }
    for (int i = ed->frontiers_.size(); i < kFrontierVisClearCount; ++i) {
      visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i, 4);
      // visualization_->drawBox(Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector4d(1, 0, 0, 0.3),
      // "frontier_boxes", i, 4);
    }
    if (status)
      visualize(2);
    else
      visualization_->drawLines({}, 0.07, Vector4d(0, 0.5, 0, 1), "grid_tour", 0, 6);

    // Draw grid tour
  }
}

void FastExplorationFSM::triggerCallback(const geometry_msgs::PoseStampedConstPtr& msg) {

  // // Debug traj planner
  // Eigen::Vector3d pos;
  // pos << msg->pose.position.x, msg->pose.position.y, 1;
  // expl_manager_->ed_->next_pos_ = pos;

  // Eigen::Vector3d dir = pos - fd_->odom_pos_;
  // expl_manager_->ed_->next_yaw_ = atan2(dir[1], dir[0]);
  // fd_->go_back_ = true;
  // transitState(PLAN_TRAJ, "triggerCallback");
  // return;

  if (state_ != WAIT_TRIGGER) return;
  cout << "Triggered!" << endl;
  tryStartExploration("triggerCallback", true);
}

bool FastExplorationFSM::tryStartExploration(const string& pos_call, bool finish_if_no_frontier) {
  if (state_ != WAIT_TRIGGER) return false;

  if (current_role_ == relay_racer_integration::RelayRoleCmd::ROLE_RELAY) {
    fd_->trigger_ = true;
    fd_->start_pos_ = fd_->odom_pos_;
    ROS_WARN_STREAM("Start expl pos: " << fd_->start_pos_.transpose());
    ROS_INFO_STREAM("[Relay]: Drone " << getId() << " entering relay planning from " << pos_call
                                      << ".");
    transitState(PLAN_TRAJ, pos_call);
    return true;
  }

  if (expl_manager_->updateFrontierStruct(fd_->odom_pos_) != 0) {
    fd_->trigger_ = true;
    fd_->start_pos_ = fd_->odom_pos_;
    ROS_WARN_STREAM("Start expl pos: " << fd_->start_pos_.transpose());
    transitState(PLAN_TRAJ, pos_call);
    return true;
  }

  if (finish_if_no_frontier) {
    fd_->trigger_ = true;
    fd_->start_pos_ = fd_->odom_pos_;
    ROS_WARN_STREAM("Start expl pos: " << fd_->start_pos_.transpose());
    transitState(FINISH, pos_call);
    return true;
  }

  return false;
}

void FastExplorationFSM::safetyCallback(const ros::TimerEvent& e) {
  if (state_ == EXPL_STATE::EXEC_TRAJ) {
    // Check safety and trigger replan if necessary
    double dist;
    bool safe = planner_manager_->checkTrajCollision(dist);
    if (!safe) {
      ROS_WARN("Replan: collision detected==================================");
      fd_->avoid_collision_ = true;
      fd_->last_safety_replan_time_ = ros::Time::now();
      transitState(PLAN_TRAJ, "safetyCallback");
    }
  }
}

void FastExplorationFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  fd_->odom_pos_(0) = msg->pose.pose.position.x;
  fd_->odom_pos_(1) = msg->pose.pose.position.y;
  fd_->odom_pos_(2) = msg->pose.pose.position.z;

  fd_->odom_vel_(0) = msg->twist.twist.linear.x;
  fd_->odom_vel_(1) = msg->twist.twist.linear.y;
  fd_->odom_vel_(2) = msg->twist.twist.linear.z;

  fd_->odom_orient_.w() = msg->pose.pose.orientation.w;
  fd_->odom_orient_.x() = msg->pose.pose.orientation.x;
  fd_->odom_orient_.y() = msg->pose.pose.orientation.y;
  fd_->odom_orient_.z() = msg->pose.pose.orientation.z;

  Eigen::Vector3d rot_x = fd_->odom_orient_.toRotationMatrix().block<3, 1>(0, 0);
  fd_->odom_yaw_ = atan2(rot_x(1), rot_x(0));

  if (!fd_->have_odom_) {
    fd_->have_odom_ = true;
    fd_->fsm_init_time_ = ros::Time::now();
  }
}

void FastExplorationFSM::relayRoleCmdCallback(
    const relay_racer_integration::RelayRoleCmdConstPtr& msg) {
  if (msg->agent_id != getId()) return;

  const int previous_role = current_role_;

  ROS_INFO_STREAM("[Relay]: Drone " << getId() << " received role=" << int(msg->role)
                  << ", valid=" << msg->valid << ", target=("
                  << msg->relay_target.x << ", " << msg->relay_target.y << ", "
                  << msg->relay_target.z << ")");

  current_role_ = msg->role;
  have_relay_target_ = msg->valid;
  relay_target_(0) = msg->relay_target.x;
  relay_target_(1) = msg->relay_target.y;
  relay_target_(2) = msg->relay_target.z;
  relay_yaw_ = msg->relay_yaw;
  fd_->consecutive_plan_failures_ = 0;
  fd_->plan_failure_relief_rounds_ = 0;
  fd_->last_assignment_relief_time_ = ros::Time(0);

  auto& self_state = expl_manager_->ed_->swarm_state_[getId() - 1];
  self_state.relay_role_ = current_role_;
  self_state.task_assignable_ =
      (current_role_ != relay_racer_integration::RelayRoleCmd::ROLE_RELAY);

  const bool entering_relay =
      previous_role != relay_racer_integration::RelayRoleCmd::ROLE_RELAY &&
      current_role_ == relay_racer_integration::RelayRoleCmd::ROLE_RELAY;
  const bool leaving_relay =
      previous_role == relay_racer_integration::RelayRoleCmd::ROLE_RELAY &&
      current_role_ != relay_racer_integration::RelayRoleCmd::ROLE_RELAY;

  if (entering_relay) {
    auto ed = expl_manager_->ed_;
    const size_t released_grid_count = self_state.grid_ids_.size();
    relay_suspended_grid_ids_ = self_state.grid_ids_;
    ed->pending_relay_enter_release_grid_ids_ = relay_suspended_grid_ids_;
    stagePendingSelfRelease(relay_suspended_grid_ids_);
    publishReleaseRequest(ed->pending_relay_enter_release_grid_ids_, "relayEnter");
    ed->pending_claim_grid_ids_.clear();
    ed->pending_relay_recover_grid_ids_.clear();
    ed->pending_grid_ids_.clear();
    if (pending_assignment_active_) {
      logOwnershipEvent("assignment_txn", "cleared_enter_relay", pending_assignment_txn_id_,
          pending_assignment_component_epoch_, pending_assignment_leader_id_, getId(),
          vector<int>{ pending_assignment_grid_id_ }, "role_transition=enter_relay");
      clearPendingAssignmentTxn();
    }
    clearLegacyPairOptState("relayRoleCmdCallback:enterRelay");
    self_state.grid_ids_.clear();
    ROS_WARN_STREAM("[Relay]: Drone " << getId() << " entering relay mode, releasing "
                                      << released_grid_count
                                      << " grids, staging relay-enter release, and replanning immediately.");
    self_state.recent_attempt_time_ = ros::Time::now().toSec();
    expl_manager_->ed_->last_grid_ids_.clear();
    expl_manager_->ed_->reallocated_ = true;
    expl_manager_->ed_->wait_response_ = false;
    fd_->aggressive_reassign_requested_ = false;
    fd_->go_back_ = false;
    fd_->avoid_collision_ = false;
    fd_->static_state_ = true;
    clearVisMarker();

    if (state_ == WAIT_TRIGGER && fd_->have_odom_) {
      fd_->trigger_ = true;
      fd_->start_pos_ = fd_->odom_pos_;
      transitState(PLAN_TRAJ, "relayRoleCmdCallback");
    } else if (state_ != INIT && state_ != FINISH && state_ != PLAN_TRAJ) {
      replan_pub_.publish(std_msgs::Empty());
      transitState(PLAN_TRAJ, "relayRoleCmdCallback");
    }
  } else if (leaving_relay) {
    recoverAssignmentAfterRelayExit("relayRoleCmdCallback");
  }
}

void FastExplorationFSM::transitState(EXPL_STATE new_state, string pos_call) {
  int pre_s = int(state_);
  state_ = new_state;
  ROS_INFO_STREAM("[" + pos_call + "]: Drone "
                  << getId()
                  << " from " + fd_->state_str_[pre_s] + " to " + fd_->state_str_[int(new_state)]);
}

void FastExplorationFSM::droneStateTimerCallback(const ros::TimerEvent& e) {
  // Broadcast own state periodically
  exploration_manager::DroneState msg;
  msg.drone_id = getId();

  auto& state = expl_manager_->ed_->swarm_state_[msg.drone_id - 1];

  if (fd_->static_state_) {
    state.pos_ = fd_->odom_pos_;
    state.vel_ = fd_->odom_vel_;
    state.yaw_ = fd_->odom_yaw_;
  } else {
    LocalTrajData* info = &planner_manager_->local_data_;
    double t_r = (ros::Time::now() - info->start_time_).toSec();
    state.pos_ = info->position_traj_.evaluateDeBoorT(t_r);
    state.vel_ = info->velocity_traj_.evaluateDeBoorT(t_r);
    state.yaw_ = info->yaw_traj_.evaluateDeBoorT(t_r)[0];
  }
  state.stamp_ = ros::Time::now().toSec();
  state.relay_role_ = current_role_;
  state.task_assignable_ = (current_role_ != relay_racer_integration::RelayRoleCmd::ROLE_RELAY);
  vector<int> published_grid_ids;
  getEffectiveSelfGridIds(published_grid_ids);
  msg.pos = { float(state.pos_[0]), float(state.pos_[1]), float(state.pos_[2]) };
  msg.vel = { float(state.vel_[0]), float(state.vel_[1]), float(state.vel_[2]) };
  msg.yaw = state.yaw_;
  for (auto id : published_grid_ids) msg.grid_ids.push_back(id);
  msg.recent_attempt_time = state.recent_attempt_time_;
  msg.stamp = state.stamp_;
  msg.relay_role = state.relay_role_;
  msg.task_assignable = state.task_assignable_;

  relay_racer_integration::RelayTaskState relay_task_msg;
  relay_task_msg.header.stamp = ros::Time::now();
  relay_task_msg.agent_id = msg.drone_id;
  relay_task_msg.stamp = msg.stamp;
  relay_task_msg.relay_role = msg.relay_role;
  relay_task_msg.task_assignable = msg.task_assignable;
  relay_task_msg.grid_count = static_cast<int32_t>(published_grid_ids.size());
  relay_task_msg.last_safety_replan_stamp =
      fd_->last_safety_replan_time_.isZero() ? 0.0 : fd_->last_safety_replan_time_.toSec();
  relay_task_msg.last_plan_fail_stamp =
      fd_->last_plan_fail_time_.isZero() ? 0.0 : fd_->last_plan_fail_time_.toSec();
  relay_task_msg.last_plan_success_stamp =
      fd_->last_plan_success_time_.isZero() ? 0.0 : fd_->last_plan_success_time_.toSec();
  relay_task_msg.consecutive_plan_failures = fd_->consecutive_plan_failures_;

  exploration_manager::ComponentState component_msg;
  component_msg.stamp = msg.stamp;
  component_msg.source_id = getId();

  const double now_sec = msg.stamp;
  const auto& states = expl_manager_->ed_->swarm_state_;
  const auto& observed_peer_grid_ids = expl_manager_->ed_->observed_peer_grid_ids_;
  vector<int> component_member_ids;
  std::map<int, int> owner_by_grid;
  vector<int> active_grids;
  expl_manager_->hgrid_->getActiveGrids(active_grids);
  for (int i = 0; i < static_cast<int>(states.size()); ++i) {
    const int agent_id = i + 1;
    const auto& candidate = states[i];
    const bool self_visible = (agent_id == getId());
    const bool peer_visible = !self_visible && candidate.stamp_ > 1e-4 &&
                              now_sec - candidate.stamp_ <= kPeerStateFreshnessSec;
    if (!self_visible && !peer_visible) {
      continue;
    }
    if (!candidate.task_assignable_ ||
        candidate.relay_role_ == relay_racer_integration::RelayRoleCmd::ROLE_RELAY) {
      continue;
    }

    component_member_ids.push_back(agent_id);
    const vector<int>* owner_grid_ids = &candidate.grid_ids_;
    if (self_visible) {
      owner_grid_ids = &published_grid_ids;
    } else if (i < static_cast<int>(observed_peer_grid_ids.size()) && candidate.stamp_ > 1e-4) {
      owner_grid_ids = &observed_peer_grid_ids[i];
    }

    for (const int grid_id : *owner_grid_ids) {
      owner_by_grid.emplace(grid_id, agent_id);
    }
  }

  if (!component_members_initialized_) {
    last_component_member_ids_ = component_member_ids;
    component_members_initialized_ = true;
  } else if (last_component_member_ids_ != component_member_ids) {
    ++component_epoch_;
    last_component_member_ids_ = component_member_ids;
  }

  component_msg.component_epoch = component_epoch_;
  component_msg.leader_id = component_member_ids.empty() ? 0 : component_member_ids.front();
  component_leader_id_ = component_msg.leader_id;
  if (pending_assignment_active_ &&
      (pending_assignment_component_epoch_ != component_msg.component_epoch ||
       pending_assignment_leader_id_ != component_leader_id_)) {
    logOwnershipEvent("assignment_txn", "cleared_component_view_change",
        pending_assignment_txn_id_, pending_assignment_component_epoch_,
        pending_assignment_leader_id_, getId(), vector<int>{ pending_assignment_grid_id_ },
        "published_component_epoch=" + std::to_string(component_msg.component_epoch) +
            ", published_leader_id=" + std::to_string(component_leader_id_));
    clearPendingAssignmentTxn();
  }

  std::unordered_map<int, int> next_component_owner_ids;
  std::unordered_map<int, uint64_t> next_component_owner_versions;
  for (const auto& owner_entry : owner_by_grid) {
    const int grid_id = owner_entry.first;
    const int owner_id = owner_entry.second;
    uint64_t owner_version = 1;
    const auto last_owner_it = component_owner_ids_.find(grid_id);
    const auto last_version_it = component_owner_versions_.find(grid_id);
    if (last_owner_it != component_owner_ids_.end() && last_version_it != component_owner_versions_.end()) {
      owner_version = last_version_it->second;
      if (last_owner_it->second != owner_id) {
        owner_version += 1;
      }
    }

    next_component_owner_ids[grid_id] = owner_id;
    next_component_owner_versions[grid_id] = owner_version;
    component_msg.grid_ids.push_back(grid_id);
    component_msg.owner_ids.push_back(owner_id);
    component_msg.owner_versions.push_back(owner_version);
  }
  component_owner_ids_.swap(next_component_owner_ids);
  component_owner_versions_.swap(next_component_owner_versions);

  std::unordered_set<int> owned_grid_ids;
  for (const auto& owner_entry : owner_by_grid) {
    owned_grid_ids.insert(owner_entry.first);
  }
  for (const int grid_id : active_grids) {
    if (owned_grid_ids.find(grid_id) == owned_grid_ids.end()) {
      component_msg.free_grid_ids.push_back(grid_id);
    }
  }

  drone_state_pub_.publish(msg);
  relay_task_state_pub_.publish(relay_task_msg);
  component_state_pub_.publish(component_msg);
}

void FastExplorationFSM::assignmentPlanCallback(
    const exploration_manager::AssignmentPlanConstPtr& msg) {
  if (msg->txn_id == 0 || msg->component_epoch != component_epoch_ ||
      msg->leader_id != component_leader_id_) {
    return;
  }

  const size_t entry_count = std::min(
      msg->grid_ids.size(), std::min(msg->owner_ids.size(), msg->owner_versions.size()));
  int match_index = -1;
  for (size_t i = 0; i < entry_count; ++i) {
    if (msg->owner_ids[i] == getId()) {
      match_index = static_cast<int>(i);
      break;
    }
  }
  if (match_index < 0) {
    return;
  }

  auto ed = expl_manager_->ed_;
  const int grid_id = msg->grid_ids[match_index];
  const uint64_t owner_version = msg->owner_versions[match_index];
  const vector<int> staged_grid_ids = { grid_id };
  auto has_grid = [&](const vector<int>& ids) {
    return std::find(ids.begin(), ids.end(), grid_id) != ids.end();
  };
  std::string staged_source = "external_plan";
  if (has_grid(ed->pending_claim_grid_ids_)) {
    staged_source = "pending_claim";
  } else if (has_grid(ed->pending_relay_recover_grid_ids_)) {
    staged_source = "pending_relay_recover";
  } else if (has_grid(ed->pending_grid_ids_)) {
    staged_source = "pending_global_alloc";
  }

  const auto& self_state = ed->swarm_state_[getId() - 1];
  if (current_role_ == relay_racer_integration::RelayRoleCmd::ROLE_RELAY ||
      !self_state.task_assignable_) {
    exploration_manager::AssignmentAck reject_msg;
    reject_msg.component_epoch = msg->component_epoch;
    reject_msg.leader_id = msg->leader_id;
    reject_msg.grid_id = grid_id;
    reject_msg.owner_id = getId();
    reject_msg.owner_version = owner_version;
    reject_msg.txn_id = msg->txn_id;
    reject_msg.accept = false;
    reject_msg.reason = std::string("notAssignable:") + msg->reason;
    reject_msg.stamp = ros::Time::now().toSec();
    assignment_ack_pub_.publish(reject_msg);
    logOwnershipEvent("assignment_ack", "published_reject_not_assignable",
        reject_msg.txn_id, reject_msg.component_epoch, reject_msg.leader_id, getId(),
        staged_grid_ids,
        "reason=" + reject_msg.reason + ", staged_source=" + staged_source +
            ", owner_version=" + std::to_string(owner_version));
    return;
  }

  if (pending_assignment_active_) {
    if (pending_assignment_txn_id_ != msg->txn_id ||
        pending_assignment_component_epoch_ != msg->component_epoch ||
        pending_assignment_leader_id_ != msg->leader_id ||
        pending_assignment_grid_id_ != grid_id ||
        pending_assignment_owner_id_ != getId() ||
        pending_assignment_owner_version_ != owner_version) {
      logOwnershipEvent("assignment_plan", "ignored_conflicting_pending_txn", msg->txn_id,
          msg->component_epoch, msg->leader_id, getId(), staged_grid_ids,
          "reason=" + msg->reason + ", staged_source=" + staged_source +
              ", existing_txn_id=" + std::to_string(pending_assignment_txn_id_));
      return;
    }

    logOwnershipEvent("assignment_plan", "duplicate_staged", msg->txn_id,
        msg->component_epoch, msg->leader_id, getId(), staged_grid_ids,
        "reason=" + msg->reason + ", staged_source=" + staged_source +
            ", owner_version=" + std::to_string(owner_version));
  } else {
    clearLegacyPairOptState(
        std::string("assignmentPlanCallback:txn=") + std::to_string(msg->txn_id));
    pending_assignment_active_ = true;
    pending_assignment_txn_id_ = msg->txn_id;
    pending_assignment_component_epoch_ = msg->component_epoch;
    pending_assignment_leader_id_ = msg->leader_id;
    pending_assignment_grid_id_ = grid_id;
    pending_assignment_owner_id_ = getId();
    pending_assignment_owner_version_ = owner_version;
    logOwnershipEvent("assignment_plan", "staged", msg->txn_id, msg->component_epoch,
        msg->leader_id, getId(), staged_grid_ids,
        "reason=" + msg->reason + ", staged_source=" + staged_source +
            ", owner_version=" + std::to_string(owner_version));
  }

  exploration_manager::AssignmentAck ack_msg;
  ack_msg.component_epoch = msg->component_epoch;
  ack_msg.leader_id = msg->leader_id;
  ack_msg.grid_id = grid_id;
  ack_msg.owner_id = getId();
  ack_msg.owner_version = owner_version;
  ack_msg.txn_id = msg->txn_id;
  ack_msg.accept = true;
  ack_msg.reason = msg->reason;
  ack_msg.stamp = ros::Time::now().toSec();
  assignment_ack_pub_.publish(ack_msg);
  logOwnershipEvent("assignment_ack", "published_accept", ack_msg.txn_id,
      ack_msg.component_epoch, ack_msg.leader_id, getId(), staged_grid_ids,
      "reason=" + ack_msg.reason + ", staged_source=" + staged_source +
          ", owner_version=" + std::to_string(owner_version));
}

void FastExplorationFSM::assignmentCommitCallback(
    const exploration_manager::AssignmentCommitConstPtr& msg) {
  const size_t entry_count = std::min(
      msg->grid_ids.size(), std::min(msg->owner_ids.size(), msg->owner_versions.size()));
  vector<int> committed_to_self_grid_ids;
  for (size_t i = 0; i < entry_count; ++i) {
    if (msg->owner_ids[i] == getId()) {
      committed_to_self_grid_ids.push_back(msg->grid_ids[i]);
    }
  }

  if (!pending_assignment_active_ || msg->txn_id == 0 ||
      msg->component_epoch != component_epoch_ || msg->leader_id != component_leader_id_ ||
      msg->txn_id != pending_assignment_txn_id_ ||
      msg->component_epoch != pending_assignment_component_epoch_ ||
      msg->leader_id != pending_assignment_leader_id_) {
    if (!committed_to_self_grid_ids.empty() && msg->txn_id != 0 &&
        msg->component_epoch == component_epoch_ && msg->leader_id == component_leader_id_) {
      logOwnershipEvent("assignment_commit", "ignored_without_matching_staged_txn",
          msg->txn_id, msg->component_epoch, msg->leader_id, getId(),
          committed_to_self_grid_ids,
          "reason=" + msg->reason + ", pending_active=" +
              std::string(pending_assignment_active_ ? "true" : "false"));
    }
    return;
  }

  const vector<int> staged_grid_ids = { pending_assignment_grid_id_ };
  bool matched_commit = false;
  for (size_t i = 0; i < entry_count; ++i) {
    if (msg->grid_ids[i] == pending_assignment_grid_id_ &&
        msg->owner_ids[i] == pending_assignment_owner_id_ &&
        msg->owner_versions[i] == pending_assignment_owner_version_) {
      matched_commit = true;
      break;
    }
  }
  if (!matched_commit) {
    logOwnershipEvent("assignment_commit", "ignored_missing_staged_entry", msg->txn_id,
        msg->component_epoch, msg->leader_id, getId(), staged_grid_ids,
        "reason=" + msg->reason + ", committed_to_self=" +
            idsToString(committed_to_self_grid_ids));
    return;
  }

  clearLegacyPairOptState(
      std::string("assignmentCommitCallback:txn=") + std::to_string(msg->txn_id));

  auto ed = expl_manager_->ed_;
  auto& self_state = ed->swarm_state_[getId() - 1];
  const vector<int> prev_self_grid_ids = self_state.grid_ids_;
  if (std::find(self_state.grid_ids_.begin(), self_state.grid_ids_.end(),
          pending_assignment_grid_id_) == self_state.grid_ids_.end()) {
    self_state.grid_ids_.push_back(pending_assignment_grid_id_);
  }

  auto erase_grid = [&](vector<int>& grid_ids) {
    grid_ids.erase(std::remove(grid_ids.begin(), grid_ids.end(), pending_assignment_grid_id_),
        grid_ids.end());
  };
  erase_grid(ed->pending_grid_ids_);
  erase_grid(ed->pending_claim_grid_ids_);
  erase_grid(ed->pending_relay_recover_grid_ids_);
  erase_grid(ed->pending_pair_opt_grid_ids_);
  erase_grid(ed->pending_release_grid_ids_);
  erase_grid(ed->pending_invalidated_grid_ids_);
  erase_grid(ed->pending_plan_fail_release_grid_ids_);
  erase_grid(ed->pending_commit_grid_ids_);
  erase_grid(ed->pending_relay_enter_release_grid_ids_);

  ed->last_grid_ids_.clear();
  ed->reallocated_ = true;
  ed->wait_response_ = false;
  fd_->plan_failure_relief_rounds_ = 0;
  fd_->last_assignment_relief_time_ = ros::Time(0);

  const bool self_assignment_changed = prev_self_grid_ids != self_state.grid_ids_;
  const bool need_immediate_replan =
      needsImmediateAssignmentReplan(prev_self_grid_ids, self_state.grid_ids_, expl_manager_);

  logOwnershipEvent("assignment_commit", "applied", msg->txn_id, msg->component_epoch,
      msg->leader_id, getId(), staged_grid_ids,
      "reason=" + msg->reason + ", prev_self=" + idsToString(prev_self_grid_ids) +
          ", new_self=" + idsToString(self_state.grid_ids_) +
          ", self_assignment_changed=" +
          std::string(self_assignment_changed ? "true" : "false"));

  clearPendingAssignmentTxn();

  vector<int> remaining_candidates;
  if (getPendingAllocationCandidates(remaining_candidates)) {
    publishAllocationRequest(std::string("continueStagedClaim:") + msg->reason);
  }

  if (!self_assignment_changed || self_state.grid_ids_.empty()) {
    return;
  }

  if ((state_ == IDLE || state_ == WAIT_TRIGGER) && fd_->have_odom_) {
    fd_->trigger_ = true;
    fd_->go_back_ = false;
    fd_->static_state_ = true;
    fd_->start_pos_ = fd_->odom_pos_;
    transitState(PLAN_TRAJ, "assignmentCommitCallback");
  } else if ((state_ == EXEC_TRAJ || state_ == PUB_TRAJ) && need_immediate_replan) {
    replan_pub_.publish(std_msgs::Empty());
    transitState(PLAN_TRAJ, "assignmentCommitCallback");
  }
}

void FastExplorationFSM::droneStateMsgCallback(const exploration_manager::DroneStateConstPtr& msg) {
  // Update other drones' states
  if (msg->drone_id == getId()) return;

  // Simulate swarm communication loss
  Eigen::Vector3d msg_pos(msg->pos[0], msg->pos[1], msg->pos[2]);
  // if ((msg_pos - fd_->odom_pos_).norm() > 6.0) return;

  auto& drone_state = expl_manager_->ed_->swarm_state_[msg->drone_id - 1];
  if (drone_state.stamp_ + 1e-4 >= msg->stamp) return;  // Avoid unordered msg

  const bool previous_assignable = drone_state.task_assignable_;
  const int previous_role = drone_state.relay_role_;

  drone_state.pos_ = Eigen::Vector3d(msg->pos[0], msg->pos[1], msg->pos[2]);
  drone_state.vel_ = Eigen::Vector3d(msg->vel[0], msg->vel[1], msg->vel[2]);
  drone_state.yaw_ = msg->yaw;
  auto& observed_peer_grid_ids = expl_manager_->ed_->observed_peer_grid_ids_;
  if (observed_peer_grid_ids.size() < expl_manager_->ed_->swarm_state_.size()) {
    observed_peer_grid_ids.resize(expl_manager_->ed_->swarm_state_.size());
  }
  observed_peer_grid_ids[msg->drone_id - 1].assign(msg->grid_ids.begin(), msg->grid_ids.end());
  drone_state.stamp_ = msg->stamp;
  drone_state.recent_attempt_time_ = msg->recent_attempt_time;
  drone_state.relay_role_ = msg->relay_role;
  drone_state.task_assignable_ = msg->task_assignable;

  const bool peer_entered_relay =
      previous_assignable && !drone_state.task_assignable_ &&
      drone_state.relay_role_ == relay_racer_integration::RelayRoleCmd::ROLE_RELAY;
  if (peer_entered_relay) {
    ROS_INFO_STREAM("[FSM]: Drone " << getId() << " observed drone " << msg->drone_id
                    << " switch from role " << previous_role << " to relay and release tasks.");
    requestAggressiveReassign("peerRelayRelease");
  }

  // std::cout << "Drone " << getId() << " get drone " << int(msg->drone_id) << "'s state" <<
  // std::endl; std::cout << drone_state.pos_.transpose() << std::endl;
}

void FastExplorationFSM::optTimerCallback(const ros::TimerEvent& e) {
  if (state_ == INIT) return;

  // Select nearby drone not interacting with recently
  auto& states = expl_manager_->ed_->swarm_state_;
  auto& state1 = states[getId() - 1];
  if (current_role_ == relay_racer_integration::RelayRoleCmd::ROLE_RELAY || !state1.task_assignable_) {
    return;
  }
  vector<int> effective_self_grid_ids;
  getEffectiveSelfGridIds(effective_self_grid_ids);
  const auto& observed_peer_grid_ids = expl_manager_->ed_->observed_peer_grid_ids_;
  if (pending_assignment_active_) {
    clearLegacyPairOptState("optTimerCallback:pending_assignment_txn");
    ROS_INFO_STREAM_THROTTLE(1.0, "[FSM]: Drone " << getId()
                                    << " skips pair-opt/local-repair because assignment txn "
                                    << pending_assignment_txn_id_ << " is still staged.");
    return;
  }
  if (effective_self_grid_ids.empty() && tryClaimUnallocatedGrids("optTimerCallback", true)) {
    return;
  }
  auto tn = ros::Time::now().toSec();

  // Find missed grids before local repair so an idle explorer can pull work released by a relay.
  vector<int> actives, missed;
  expl_manager_->hgrid_->getActiveGrids(actives);
  findUnallocated(actives, missed);
  const bool have_missed = !missed.empty();
  const bool aggressive_reassign = fd_->aggressive_reassign_requested_ && have_missed;

  if (kPairOptTransactionalRepairOnly) {
    clearLegacyPairOptState("optTimerCallback:transactional_repair_only");
    if (have_missed) {
      logOwnershipEvent("pair_opt_repair", "route_to_assignment_request", 0,
          component_epoch_, component_leader_id_, getId(), missed,
          "aggressive_reassign=" + std::string(aggressive_reassign ? "true" : "false") +
              ", effective_self=" + idsToString(effective_self_grid_ids));
    }
    if (have_missed && tryClaimUnallocatedGrids("assignmentTxnRepair", false)) {
      return;
    }
    if (!have_missed) {
      fd_->aggressive_reassign_requested_ = false;
    }
    if (have_missed) {
      ROS_INFO_STREAM_THROTTLE(1.0, "[FSM]: Drone " << getId()
                                      << " keeps pair-opt in local-repair-only mode; "
                                      << "ownership changes stay on assignment_plan/commit/ack.");
    }
    return;
  }

  // Avoid frequent attempt
  if (!aggressive_reassign && tn - state1.recent_attempt_time_ < fp_->attempt_interval_) return;

  int select_id = -1;
  double max_interval = -1.0;
  int best_peer_load = -1;
  const bool self_idle = effective_self_grid_ids.empty();
  for (int i = 0; i < states.size(); ++i) {
    const int candidate_id = i + 1;
    if (candidate_id == getId()) continue;
    // Check if have communication recently
    // or the drone just experience another opt
    // or the drone is interacted with recently /* !urgent &&  */
    // or the candidate drone dominates enough grids
    if (tn - states[i].stamp_ > 0.2) continue;
    if (!states[i].task_assignable_) continue;
    if (!aggressive_reassign && tn - states[i].recent_attempt_time_ < fp_->attempt_interval_) {
      continue;
    }
    if (!aggressive_reassign && tn - states[i].recent_interact_time_ < fp_->pair_opt_interval_) {
      continue;
    }

    const vector<int>& peer_grid_ids =
        (i < static_cast<int>(observed_peer_grid_ids.size()) && states[i].stamp_ > 1e-4)
            ? observed_peer_grid_ids[i]
            : states[i].grid_ids_;
    const int peer_load = static_cast<int>(peer_grid_ids.size());
    if (!have_missed && peer_load == 0 && self_idle) continue;
    if (!have_missed && peer_load == 0 && !self_idle) continue;

    double interval = tn - states[i].recent_interact_time_;
    if (self_idle || aggressive_reassign) {
      if (peer_load > best_peer_load ||
          (peer_load == best_peer_load && interval > max_interval + 1e-6) ||
          (peer_load == best_peer_load && std::fabs(interval - max_interval) <= 1e-6 &&
              (select_id < 0 || candidate_id < select_id))) {
        select_id = candidate_id;
        max_interval = interval;
        best_peer_load = peer_load;
      }
      continue;
    }

    if (interval > max_interval + 1e-6 ||
        (std::fabs(interval - max_interval) <= 1e-6 &&
            (select_id < 0 || candidate_id < select_id))) {
      select_id = candidate_id;
      max_interval = interval;
    }
  }
  if (select_id == -1) {
    if (!have_missed) {
      fd_->aggressive_reassign_requested_ = false;
    }
    return;
  }

  std::cout << "\nSelect: " << select_id << std::endl;
  ROS_WARN("Pair opt %d & %d", getId(), select_id);

  // Do pairwise optimization with selected drone, allocate the union of their domiance grids
  unordered_map<int, char> opt_ids_map;
  auto& state2 = states[select_id - 1];
  const vector<int>& selected_peer_grid_ids =
      (select_id - 1 < static_cast<int>(observed_peer_grid_ids.size()) && state2.stamp_ > 1e-4)
          ? observed_peer_grid_ids[select_id - 1]
          : state2.grid_ids_;
  for (auto id : effective_self_grid_ids) opt_ids_map[id] = 1;
  for (auto id : selected_peer_grid_ids) opt_ids_map[id] = 1;
  vector<int> opt_ids;
  for (auto pair : opt_ids_map) opt_ids.push_back(pair.first);

  std::cout << "Pair Opt id: ";
  for (auto id : opt_ids) std::cout << id << ", ";
  std::cout << "" << std::endl;

  // Find missed grids to reallocated them
  std::cout << "Missed: ";
  for (auto id : missed) std::cout << id << ", ";
  std::cout << "" << std::endl;
  opt_ids.insert(opt_ids.end(), missed.begin(), missed.end());

  // Do partition of the grid
  vector<Eigen::Vector3d> positions = { state1.pos_, state2.pos_ };
  vector<Eigen::Vector3d> velocities = { state1.vel_, state2.vel_ };
  vector<int> first_ids1, second_ids1, first_ids2, second_ids2;
  if (state_ != WAIT_TRIGGER) {
    expl_manager_->hgrid_->getConsistentGrid(
        effective_self_grid_ids, effective_self_grid_ids, first_ids1, second_ids1);
    expl_manager_->hgrid_->getConsistentGrid(
        selected_peer_grid_ids, selected_peer_grid_ids, first_ids2, second_ids2);
  }

  auto t1 = ros::Time::now();

  vector<int> alloc_drone_ids = { getId(), select_id };
  vector<vector<int>> allocated_grid_ids;
  expl_manager_->allocateGrids(positions, velocities, alloc_drone_ids, { first_ids1, first_ids2 },
      { second_ids1, second_ids2 }, opt_ids, allocated_grid_ids);

  vector<int> ego_ids, other_ids;
  if (!allocated_grid_ids.empty()) ego_ids = allocated_grid_ids[0];
  if (allocated_grid_ids.size() > 1) other_ids = allocated_grid_ids[1];
  if (allocated_grid_ids.size() != alloc_drone_ids.size()) {
    ROS_WARN_STREAM("[PAIR_ALLOC]: expected " << alloc_drone_ids.size()
                    << " allocation buckets but got " << allocated_grid_ids.size());
  }

  double alloc_time = (ros::Time::now() - t1).toSec();

  std::cout << "Ego1  : ";
  for (auto id : effective_self_grid_ids) std::cout << id << ", ";
  std::cout << "\nOther1: ";
  for (auto id : selected_peer_grid_ids) std::cout << id << ", ";
  std::cout << "\nEgo2  : ";
  for (auto id : ego_ids) std::cout << id << ", ";
  std::cout << "\nOther2: ";
  for (auto id : other_ids) std::cout << id << ", ";
  std::cout << "" << std::endl;

  // Check results. When this pair-opt wakes an idle drone up, reducing the max per-drone
  // remaining workload is more important than keeping the summed path cost strictly smaller.
  double prev_app1 = expl_manager_->computeGridPathCost(state1.pos_, state1.vel_, effective_self_grid_ids, first_ids1,
      { first_ids1, first_ids2 }, { second_ids1, second_ids2 }, true);
  double prev_app2 = expl_manager_->computeGridPathCost(state2.pos_, state2.vel_, selected_peer_grid_ids, first_ids2,
      { first_ids1, first_ids2 }, { second_ids1, second_ids2 }, true);
  const double prev_total_cost = prev_app1 + prev_app2;
  const double prev_max_cost = std::max(prev_app1, prev_app2);
  std::cout << "prev cost: " << prev_app1 << ", " << prev_app2 << ", " << prev_total_cost
            << std::endl;
  double cur_app1 = expl_manager_->computeGridPathCost(state1.pos_, state1.vel_, ego_ids, first_ids1,
      { first_ids1, first_ids2 }, { second_ids1, second_ids2 }, true);
  double cur_app2 = expl_manager_->computeGridPathCost(state2.pos_, state2.vel_, other_ids, first_ids2,
      { first_ids1, first_ids2 }, { second_ids1, second_ids2 }, true);
  const double cur_total_cost = cur_app1 + cur_app2;
  const double cur_max_cost = std::max(cur_app1, cur_app2);
  const int prev_active_agents = (!effective_self_grid_ids.empty() ? 1 : 0) +
                                 (!selected_peer_grid_ids.empty() ? 1 : 0);
  const int cur_active_agents = (!ego_ids.empty() ? 1 : 0) + (!other_ids.empty() ? 1 : 0);
  const double total_cost_increase = cur_total_cost - prev_total_cost;
  const double max_cost_reduction = prev_max_cost - cur_max_cost;
  const bool activated_idle_agent = cur_active_agents > prev_active_agents;
  const bool accept_idle_activation =
      activated_idle_agent && max_cost_reduction > 1.0 &&
      total_cost_increase <= max_cost_reduction + 1.0;
  const bool accept_aggressive_reactivation =
      fd_->aggressive_reassign_requested_ && effective_self_grid_ids.empty() &&
      !ego_ids.empty() && activated_idle_agent;
  std::cout << "cur cost : " << cur_app1 << ", " << cur_app2 << ", " << cur_total_cost
            << std::endl;
  if (cur_total_cost > prev_total_cost + 0.1 &&
      !accept_idle_activation && !accept_aggressive_reactivation) {
    ROS_ERROR("Larger cost after reallocation");
    if (state_ != WAIT_TRIGGER) {
      return;
    }
  }
  if (accept_idle_activation) {
    ROS_WARN_STREAM("Accept pair opt with higher total cost because it activates an idle drone: "
                    << "prev_total=" << prev_total_cost << ", cur_total=" << cur_total_cost
                    << ", prev_max=" << prev_max_cost << ", cur_max=" << cur_max_cost);
  } else if (accept_aggressive_reactivation) {
    ROS_WARN_STREAM("Accept pair opt with higher total cost to reactivate drone " << getId()
                    << " after relay release: prev_total=" << prev_total_cost
                    << ", cur_total=" << cur_total_cost);
  }

  if (!effective_self_grid_ids.empty() && !ego_ids.empty() &&
      !expl_manager_->hgrid_->isConsistent(effective_self_grid_ids[0], ego_ids[0])) {
    ROS_ERROR("Path 1 inconsistent");
  }
  if (!selected_peer_grid_ids.empty() && !other_ids.empty() &&
      !expl_manager_->hgrid_->isConsistent(selected_peer_grid_ids[0], other_ids[0])) {
    ROS_ERROR("Path 2 inconsistent");
  }

  // Update ego and other dominace grids
  auto last_ids2 = selected_peer_grid_ids;

  // Send the result to selected drone and wait for confirmation
  exploration_manager::PairOpt opt;
  opt.from_drone_id = getId();
  opt.to_drone_id = select_id;
  // opt.msg_type = 1;
  opt.stamp = tn;
  for (auto id : ego_ids) opt.ego_ids.push_back(id);
  for (auto id : other_ids) opt.other_ids.push_back(id);

  for (int i = 0; i < fp_->repeat_send_num_; ++i) opt_pub_.publish(opt);

  ROS_WARN("Drone %d send opt request to %d, pair opt t: %lf, allocate t: %lf", getId(), select_id,
      ros::Time::now().toSec() - tn, alloc_time);

  // Reserve the result and wait...
  auto ed = expl_manager_->ed_;
  ed->ego_ids_ = ego_ids;
  ed->other_ids_ = other_ids;
  ed->pair_opt_stamp_ = opt.stamp;
  ed->wait_response_ = true;
  state1.recent_attempt_time_ = tn;
  fd_->aggressive_reassign_requested_ = false;
}

void FastExplorationFSM::findUnallocated(const vector<int>& actives, vector<int>& missed) {
  // Only frontier-backed active grids should enter the shared unallocated pool.
  // Otherwise residual unknown cells can pull explorers back into already cleared regions.
  unordered_map<int, char> active_map;
  vector<int> frontier_ids;
  vector<int> single_grid(1, -1);
  for (auto ativ : actives) {
    if (expl_manager_->hgrid_->isFineGrid(ativ)) {
      frontier_ids.clear();
      single_grid[0] = ativ;
      expl_manager_->hgrid_->getFrontiersInGrid(single_grid, frontier_ids);
      if (frontier_ids.empty()) continue;
    }
    active_map[ativ] = 1;
  }

  // Remove allocated ones. Grids currently held by an active relay stay available so other
  // explorers can pick them up through assignment transactions triggered by local repair.
  vector<int> effective_self_grid_ids;
  getEffectiveSelfGridIds(effective_self_grid_ids);
  const auto& observed_peer_grid_ids = expl_manager_->ed_->observed_peer_grid_ids_;
  for (int i = 0; i < static_cast<int>(expl_manager_->ed_->swarm_state_.size()); ++i) {
    const auto& state = expl_manager_->ed_->swarm_state_[i];
    if (!state.task_assignable_) {
      continue;
    }

    const vector<int>* grid_ids = &state.grid_ids_;
    if (i + 1 == getId()) {
      grid_ids = &effective_self_grid_ids;
    } else if (i < static_cast<int>(observed_peer_grid_ids.size()) && state.stamp_ > 1e-4) {
      grid_ids = &observed_peer_grid_ids[i];
    }

    for (auto id : *grid_ids) {
      if (active_map.find(id) != active_map.end()) {
        active_map.erase(id);
      } else {
        // ROS_ERROR("Inactive grid %d is allocated.", id);
      }
    }
  }

  missed.clear();
  for (auto p : active_map) {
    missed.push_back(p.first);
  }
}

void FastExplorationFSM::recoverAssignmentAfterRelayExit(const string& reason) {
  if (!fd_->have_odom_ || current_role_ == relay_racer_integration::RelayRoleCmd::ROLE_RELAY) {
    return;
  }

  auto ed = expl_manager_->ed_;
  auto& states = expl_manager_->ed_->swarm_state_;
  auto& self_state = states[getId() - 1];
  const ros::Time now = ros::Time::now();
  const double now_sec = now.toSec();

  self_state.pos_ = fd_->odom_pos_;
  self_state.vel_ = fd_->odom_vel_;
  self_state.yaw_ = fd_->odom_yaw_;
  self_state.stamp_ = now_sec;
  self_state.relay_role_ = current_role_;
  self_state.task_assignable_ = true;

  vector<int> recoverable_grid_ids;
  if (!relay_suspended_grid_ids_.empty()) {
    vector<int> actives, missed;
    expl_manager_->hgrid_->getActiveGrids(actives);
    findUnallocated(actives, missed);
    std::unordered_set<int> missed_set(missed.begin(), missed.end());
    recoverable_grid_ids.reserve(relay_suspended_grid_ids_.size());
    for (const int grid_id : relay_suspended_grid_ids_) {
      if (missed_set.find(grid_id) != missed_set.end()) {
        recoverable_grid_ids.push_back(grid_id);
      }
    }
  }
  const size_t handed_off_grid_count =
      relay_suspended_grid_ids_.size() - recoverable_grid_ids.size();
  ed->pending_relay_recover_grid_ids_ = recoverable_grid_ids;
  self_state.recent_attempt_time_ = now_sec;
  fd_->go_back_ = false;
  fd_->avoid_collision_ = false;
  fd_->static_state_ = true;
  fd_->last_check_frontier_time_ = now;
  fd_->consecutive_plan_failures_ = 0;
  fd_->plan_failure_relief_rounds_ = 0;
  fd_->last_assignment_relief_time_ = ros::Time(0);

  if (!recoverable_grid_ids.empty()) {
    fd_->aggressive_reassign_requested_ = false;
    ed->pending_claim_grid_ids_.clear();
    publishAllocationRequest(std::string("relayRecover:") + reason);
    ROS_WARN_STREAM("[Relay]: Drone " << getId()
                    << " leaving relay mode and staging " << recoverable_grid_ids.size()
                    << " suspended grids; " << handed_off_grid_count
                    << " already handed off. Recovery now goes through assignment transactions.");

    if (state_ != INIT && state_ != IDLE) {
      transitState(IDLE, reason);
    }
    return;
  }

  ed->pending_relay_recover_grid_ids_.clear();
  ROS_WARN_STREAM("[Relay]: Drone " << getId()
                  << " leaving relay mode with no recoverable suspended grids; staging fresh assignment candidate.");

  fd_->aggressive_reassign_requested_ = true;
  if (tryClaimUnallocatedGrids(reason, true)) {
    if (state_ != INIT && state_ != IDLE) {
      transitState(IDLE, reason);
    }
    return;
  }

  publishAllocationRequest(std::string("relayExitNeedTask:") + reason);

  if (state_ != INIT && state_ != IDLE) {
    transitState(IDLE, reason);
  }
}

void FastExplorationFSM::requestAggressiveReassign(
    const string& reason, bool immediate_claim) {
  if (!fd_->have_odom_ ||
      current_role_ == relay_racer_integration::RelayRoleCmd::ROLE_RELAY) {
    return;
  }

  fd_->aggressive_reassign_requested_ = true;
  ROS_INFO_STREAM("[FSM]: Drone " << getId()
                  << " scheduling aggressive reassignment after " << reason
                  << ", immediate_claim=" << (immediate_claim ? "true" : "false")
                  << ", consecutive_failures=" << fd_->consecutive_plan_failures_
                  << ", relief_rounds=" << fd_->plan_failure_relief_rounds_);
  if (!immediate_claim) {
    return;
  }

  const bool claim_started = tryClaimUnallocatedGrids(reason, true);
  ROS_INFO_STREAM("[FSM]: Drone " << getId()
                  << " aggressive reassignment immediate claim_started="
                  << (claim_started ? "true" : "false")
                  << ", reason=" << reason);
}

bool FastExplorationFSM::tryClaimUnallocatedGrids(
    const string& pos_call, bool require_empty_assignment) {
  if (!fd_->have_odom_ ||
      current_role_ == relay_racer_integration::RelayRoleCmd::ROLE_RELAY) {
    return false;
  }

  auto& states = expl_manager_->ed_->swarm_state_;
  auto& state1 = states[getId() - 1];
  state1.pos_ = fd_->odom_pos_;
  state1.vel_ = fd_->odom_vel_;
  state1.yaw_ = fd_->odom_yaw_;
  state1.stamp_ = ros::Time::now().toSec();
  state1.relay_role_ = current_role_;
  state1.task_assignable_ = true;

  vector<int> effective_self_grid_ids;
  getEffectiveSelfGridIds(effective_self_grid_ids);
  if (require_empty_assignment && !effective_self_grid_ids.empty()) {
    return false;
  }

  const ros::Time now = ros::Time::now();
  const auto& observed_peer_grid_ids = expl_manager_->ed_->observed_peer_grid_ids_;
  const double retry_interval = std::max(0.2, std::min(fp_->idle_retry_interval_, 1.0));
  if (!fd_->last_idle_recovery_time_.isZero() &&
      (now - fd_->last_idle_recovery_time_).toSec() < retry_interval) {
    return false;
  }
  fd_->last_idle_recovery_time_ = now;

  if (expl_manager_->updateFrontierStruct(fd_->odom_pos_) == 0) {
    return false;
  }

  vector<int> actives, missed;
  expl_manager_->hgrid_->getActiveGrids(actives);
  findUnallocated(actives, missed);
  if (missed.empty()) {
    return false;
  }

  vector<int> assignable_ids, idle_ids;
  const double now_sec = now.toSec();
  for (int i = 0; i < states.size(); ++i) {
    const int agent_id = i + 1;
    const auto& candidate = states[i];
    if (!candidate.task_assignable_) continue;
    if (agent_id != getId() && now_sec - candidate.stamp_ > kPeerStateFreshnessSec) continue;
    assignable_ids.push_back(agent_id);
    const vector<int>* candidate_grid_ids = &candidate.grid_ids_;
    if (agent_id == getId()) {
      candidate_grid_ids = &effective_self_grid_ids;
    } else if (i < static_cast<int>(observed_peer_grid_ids.size()) && candidate.stamp_ > 1e-4) {
      candidate_grid_ids = &observed_peer_grid_ids[i];
    }
    if (candidate_grid_ids->empty()) {
      idle_ids.push_back(agent_id);
    }
  }

  const vector<int> candidate_ids = idle_ids.empty() ? assignable_ids : idle_ids;
  if (candidate_ids.empty()) {
    return false;
  }

  unordered_map<int, vector<int>> claimed_ids;
  unordered_map<int, int> assigned_loads;
  for (const int agent_id : candidate_ids) {
    if (agent_id == getId()) {
      assigned_loads[agent_id] = static_cast<int>(effective_self_grid_ids.size());
      continue;
    }
    const int peer_index = agent_id - 1;
    const vector<int>* candidate_grid_ids = &states[peer_index].grid_ids_;
    if (peer_index < static_cast<int>(observed_peer_grid_ids.size()) &&
        states[peer_index].stamp_ > 1e-4) {
      candidate_grid_ids = &observed_peer_grid_ids[peer_index];
    }
    assigned_loads[agent_id] = static_cast<int>(candidate_grid_ids->size());
  }

  for (const int grid_id : missed) {
    double best_score = std::numeric_limits<double>::infinity();
    int best_agent = -1;
    int best_load = std::numeric_limits<int>::max();

    for (const int agent_id : candidate_ids) {
      const auto& candidate = states[agent_id - 1];
      const int candidate_load = assigned_loads[agent_id];
      double score = expl_manager_->hgrid_->getCostDroneToGrid(candidate.pos_, grid_id, {});
      score += 2.0 * static_cast<double>(candidate_load);

      if (score + 1e-3 < best_score ||
          (std::fabs(score - best_score) <= 1e-3 &&
              (candidate_load < best_load ||
                  (candidate_load == best_load && (best_agent < 0 || agent_id < best_agent))))) {
        best_score = score;
        best_agent = agent_id;
        best_load = candidate_load;
      }
    }

    if (best_agent > 0) {
      claimed_ids[best_agent].push_back(grid_id);
      assigned_loads[best_agent] += 1;
    }
  }

  const auto self_it = claimed_ids.find(getId());
  if (self_it == claimed_ids.end() || self_it->second.empty()) {
    return false;
  }

  auto ed = expl_manager_->ed_;
  ed->pending_claim_grid_ids_ = self_it->second;
  state1.recent_attempt_time_ = now_sec;
  fd_->aggressive_reassign_requested_ = false;
  fd_->last_check_frontier_time_ = now;

  ROS_WARN_STREAM("[FSM]: Drone " << getId() << " staged claim candidate of "
                  << ed->pending_claim_grid_ids_.size() << " missed grids after " << pos_call
                  << ", active grids=" << actives.size() << ", missed grids=" << missed.size()
                  << ", require_empty_assignment=" << require_empty_assignment
                  << ". Ownership is not applied yet.");

  if (!publishAllocationRequest(std::string("pendingClaim:") + pos_call)) {
    ROS_WARN_STREAM("[FSM]: Drone " << getId()
                    << " staged claim candidate but could not publish allocation request yet.");
  }

  return true;
}

void FastExplorationFSM::optMsgCallback(const exploration_manager::PairOptConstPtr& msg) {
  if (msg->from_drone_id == getId() || msg->to_drone_id != getId()) return;

  // Check stamp to avoid unordered/repeated msg
  if (msg->stamp <= expl_manager_->ed_->pair_opt_stamps_[msg->from_drone_id - 1] + 1e-4) return;
  expl_manager_->ed_->pair_opt_stamps_[msg->from_drone_id - 1] = msg->stamp;

  auto& state1 = expl_manager_->ed_->swarm_state_[msg->from_drone_id - 1];
  auto& state2 = expl_manager_->ed_->swarm_state_[getId() - 1];
  const vector<int> requested_self_grid_ids(msg->other_ids.begin(), msg->other_ids.end());
  const vector<int> requested_peer_grid_ids(msg->ego_ids.begin(), msg->ego_ids.end());

  exploration_manager::PairOptResponse response;
  response.from_drone_id = msg->to_drone_id;
  response.to_drone_id = msg->from_drone_id;
  response.stamp = msg->stamp;  // reply with the same stamp for verification

  string reject_reason;
  if (current_role_ == relay_racer_integration::RelayRoleCmd::ROLE_RELAY ||
      state2.task_assignable_ == false) {
    ROS_WARN("Reject pair opt while acting as relay");
    response.status = 3;
    reject_reason = "receiver_not_assignable";
  } else if (state1.task_assignable_ == false) {
    ROS_WARN("Reject pair opt from non-assignable peer");
    response.status = 4;
    reject_reason = "sender_not_assignable";
  } else if (msg->stamp - state2.recent_attempt_time_ < fp_->attempt_interval_) {
    ROS_WARN("Reject frequent attempt");
    response.status = 2;
    reject_reason = "receiver_recent_attempt";
  } else {
    response.status = kPairOptTransactionalRepairOnlyStatus;
    reject_reason = "transactional_repair_only";
    state2.recent_attempt_time_ = ros::Time::now().toSec();
    ROS_WARN_STREAM("[FSM]: Drone " << getId() << " rejected direct pair-opt transfer from drone "
                    << msg->from_drone_id
                    << " because ownership changes now flow only through assignment transactions.");
  }

  clearLegacyPairOptState("optMsgCallback:" + reject_reason);
  logOwnershipEvent("pair_opt", "direct_transfer_rejected", 0, component_epoch_,
      component_leader_id_, msg->from_drone_id, requested_self_grid_ids,
      "status=" + std::to_string(response.status) + ", reason=" + reject_reason +
          ", sender_id=" + std::to_string(msg->from_drone_id) +
          ", self_candidate=" + idsToString(requested_self_grid_ids) +
          ", peer_candidate=" + idsToString(requested_peer_grid_ids));

  for (int i = 0; i < fp_->repeat_send_num_; ++i) {
    opt_res_pub_.publish(response);
  }
}

void FastExplorationFSM::optResMsgCallback(
    const exploration_manager::PairOptResponseConstPtr& msg) {
  if (msg->from_drone_id == getId() || msg->to_drone_id != getId()) return;

  // Check stamp to avoid unordered/repeated msg
  if (msg->stamp <= expl_manager_->ed_->pair_opt_res_stamps_[msg->from_drone_id - 1] + 1e-4) return;
  expl_manager_->ed_->pair_opt_res_stamps_[msg->from_drone_id - 1] = msg->stamp;

  auto ed = expl_manager_->ed_;
  const bool waiting_response = ed->wait_response_;
  const double reserved_stamp = ed->pair_opt_stamp_;
  const bool stamp_matches = waiting_response && std::fabs(reserved_stamp - msg->stamp) <= 1e-5;

  logOwnershipEvent("pair_opt", "response_ignored", 0, component_epoch_, component_leader_id_,
      msg->from_drone_id, ed->ego_ids_,
      "status=" + std::to_string(int(msg->status)) +
          ", waiting_response=" + std::string(waiting_response ? "true" : "false") +
          ", stamp_matches=" + std::string(stamp_matches ? "true" : "false") +
          ", reserved_stamp=" + std::to_string(reserved_stamp) +
          ", response_stamp=" + std::to_string(msg->stamp) +
          ", reserved_self=" + idsToString(ed->ego_ids_) +
          ", reserved_peer=" + idsToString(ed->other_ids_) +
          ", ownership_path=assignment_transactions_only");
  clearLegacyPairOptState("optResMsgCallback:status=" + std::to_string(int(msg->status)));
}

void FastExplorationFSM::swarmTrajCallback(const bspline::BsplineConstPtr& msg) {
  // Get newest trajs from other drones, for inter-drone collision avoidance
  auto& sdat = planner_manager_->swarm_traj_data_;

  // Ignore self trajectory
  if (msg->drone_id == sdat.drone_id_) return;

  // Ignore outdated trajectory
  if (sdat.receive_flags_[msg->drone_id - 1] == true &&
      msg->start_time.toSec() <= sdat.swarm_trajs_[msg->drone_id - 1].start_time_ + 1e-3)
    return;

  // Convert the msg to B-spline
  Eigen::MatrixXd pos_pts(msg->pos_pts.size(), 3);
  Eigen::VectorXd knots(msg->knots.size());
  for (int i = 0; i < msg->knots.size(); ++i) knots(i) = msg->knots[i];

  for (int i = 0; i < msg->pos_pts.size(); ++i) {
    pos_pts(i, 0) = msg->pos_pts[i].x;
    pos_pts(i, 1) = msg->pos_pts[i].y;
    pos_pts(i, 2) = msg->pos_pts[i].z;
  }

  // // Transform of drone's basecoor, optional step (skip if use swarm_pilot)
  // Eigen::Vector4d tf;
  // planner_manager_->edt_environment_->sdf_map_->getBaseCoor(msg->drone_id, tf);
  // double yaw = tf[3];
  // Eigen::Matrix3d rot;
  // rot << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;
  // Eigen::Vector3d trans = tf.head<3>();
  // for (int i = 0; i < pos_pts.rows(); ++i) {
  //   Eigen::Vector3d tmp = pos_pts.row(i);
  //   tmp = rot * tmp + trans;
  //   pos_pts.row(i) = tmp;
  // }

  sdat.swarm_trajs_[msg->drone_id - 1].setUniformBspline(pos_pts, msg->order, 0.1);
  sdat.swarm_trajs_[msg->drone_id - 1].setKnot(knots);
  sdat.swarm_trajs_[msg->drone_id - 1].start_time_ = msg->start_time.toSec();
  sdat.receive_flags_[msg->drone_id - 1] = true;

  if (state_ == EXEC_TRAJ) {
    // Check collision with received trajectory
    if (!planner_manager_->checkSwarmCollision(msg->drone_id)) {
      ROS_ERROR("Drone %d collide with drone %d.", sdat.drone_id_, msg->drone_id);
      fd_->avoid_collision_ = true;
      transitState(PLAN_TRAJ, "swarmTrajCallback");
    }
  }
}

void FastExplorationFSM::swarmTrajTimerCallback(const ros::TimerEvent& e) {
  // Broadcast newest traj of this drone to others
  if (state_ == EXEC_TRAJ) {
    swarm_traj_pub_.publish(fd_->newest_traj_);

  } else if (state_ == WAIT_TRIGGER) {
    // Publish a virtual traj at current pose, to avoid collision
    bspline::Bspline bspline;
    bspline.order = planner_manager_->pp_.bspline_degree_;
    bspline.start_time = ros::Time::now();
    bspline.traj_id = planner_manager_->local_data_.traj_id_;

    Eigen::MatrixXd pos_pts(4, 3);
    for (int i = 0; i < 4; ++i) pos_pts.row(i) = fd_->odom_pos_.transpose();

    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }

    NonUniformBspline tmp(pos_pts, planner_manager_->pp_.bspline_degree_, 1.0);
    Eigen::VectorXd knots = tmp.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }
    bspline.drone_id = expl_manager_->ep_->drone_id_;
    swarm_traj_pub_.publish(bspline);
  }
}

}  // namespace fast_planner
