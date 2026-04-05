#include <ros/ros.h>
#include <exploration_manager/AllocationRequest.h>
#include <exploration_manager/ComponentState.h>
#include <exploration_manager/AssignmentPlan.h>
#include <exploration_manager/AssignmentAck.h>
#include <exploration_manager/AssignmentCommit.h>
#include <exploration_manager/ReleaseRequest.h>

#include <algorithm>
#include <cstdint>
#include <string>
#include <unordered_map>

#include <plan_manage/backward.hpp>
namespace backward {
backward::SignalHandling sh;
}

namespace fast_planner {

struct OwnershipRecord {
  int owner_id;
  uint64_t owner_version;
};

using ComponentSnapshot = exploration_manager::ComponentState;

class ComponentAllocationCoordinator {
public:
  void init(ros::NodeHandle& nh) {
    nh.param("exploration/drone_id", drone_id_, 1);
    nh.param("exploration/assignment_plan_timeout_sec", pending_plan_timeout_sec_, 1.0);
    nh.param("exploration/component_snapshot_timeout_sec", component_snapshot_timeout_sec_, 1.0);
    nh.param("exploration/local_override_timeout_sec", local_override_timeout_sec_, 1.0);

    component_state_sub_ = nh.subscribe(
        "/swarm_expl/component_state_send", 10,
        &ComponentAllocationCoordinator::componentStateCallback, this);
    allocation_request_sub_ = nh.subscribe(
        "/swarm_expl/allocation_request", 10,
        &ComponentAllocationCoordinator::allocationRequestCallback, this);
    release_request_sub_ = nh.subscribe(
        "/swarm_expl/release_request", 10,
        &ComponentAllocationCoordinator::releaseRequestCallback, this);
    assignment_ack_sub_ = nh.subscribe(
        "/swarm_expl/assignment_ack", 10,
        &ComponentAllocationCoordinator::assignmentAckCallback, this);
    assignment_plan_pub_ =
        nh.advertise<exploration_manager::AssignmentPlan>("/swarm_expl/assignment_plan", 10);
    assignment_commit_pub_ =
        nh.advertise<exploration_manager::AssignmentCommit>("/swarm_expl/assignment_commit", 10);
    pending_plan_timer_ = nh.createTimer(
        ros::Duration(0.2), &ComponentAllocationCoordinator::pendingPlanTimerCallback, this);
  }

private:
  void clearPendingPlan() {
    pending_plan_active_ = false;
    pending_plan_txn_id_ = 0;
    pending_plan_grid_id_ = 0;
    pending_plan_owner_id_ = 0;
    pending_plan_owner_version_ = 0;
    pending_plan_reason_.clear();
    pending_plan_stamp_ = 0.0;
  }

  bool snapshotMatchesAuthoritativeView(const ComponentSnapshot& snapshot) const {
    if (leader_id_ <= 0 || component_epoch_ == 0) {
      return true;
    }
    return snapshot.leader_id == leader_id_ && snapshot.component_epoch == component_epoch_;
  }

  bool authoritativeSourceTimedOut() const {
    if (component_stamp_ <= 0.0) {
      return true;
    }
    return ros::Time::now().toSec() - component_stamp_ > component_snapshot_timeout_sec_;
  }

  std::string authoritativeAcceptanceReason(const ComponentSnapshot& snapshot) const {
    if (snapshot.source_id != snapshot.leader_id) {
      return std::string();
    }
    if (leader_id_ == 0) {
      return "leader_id_unset";
    }
    if (component_epoch_ == 0) {
      return "component_epoch_unset";
    }
    if (snapshot.source_id == leader_id_) {
      return "same_leader_refresh";
    }
    if (snapshot.component_epoch > component_epoch_) {
      return "newer_component_epoch";
    }
    if (authoritativeSourceTimedOut()) {
      return "authoritative_source_timeout";
    }
    return std::string();
  }

  bool pruneExpiredSnapshots() {
    const double now_sec = ros::Time::now().toSec();
    bool removed_snapshot = false;
    for (auto it = last_component_snapshots_by_source_.begin();
         it != last_component_snapshots_by_source_.end();) {
      if (it->second.stamp > 0.0 &&
          now_sec - it->second.stamp <= component_snapshot_timeout_sec_) {
        ++it;
        continue;
      }
      it = last_component_snapshots_by_source_.erase(it);
      removed_snapshot = true;
    }
    if (removed_snapshot) {
      rebuildAggregatedState();
    }
    return removed_snapshot;
  }

  bool pruneExpiredLocalOverrides() {
    const double now_sec = ros::Time::now().toSec();
    bool removed_override = false;
    for (auto it = local_override_stamps_.begin(); it != local_override_stamps_.end();) {
      if (now_sec - it->second <= local_override_timeout_sec_) {
        ++it;
        continue;
      }
      local_override_owner_ids_.erase(it->first);
      local_override_versions_.erase(it->first);
      it = local_override_stamps_.erase(it);
      removed_override = true;
    }
    if (removed_override) {
      rebuildAggregatedState();
    }
    return removed_override;
  }

  void pendingPlanTimerCallback(const ros::TimerEvent&) {
    pruneExpiredSnapshots();
    pruneExpiredLocalOverrides();
    if (!pending_plan_active_) {
      return;
    }
    if (pending_plan_stamp_ <= 0.0) {
      return;
    }
    if (ros::Time::now().toSec() - pending_plan_stamp_ < pending_plan_timeout_sec_) {
      return;
    }
    clearPendingPlan();
  }

  void rebuildAggregatedState() {
    ownership_table_.clear();
    free_grid_pool_.clear();
    std::unordered_map<int, int> ownership_source_ids;

    for (const auto& snapshot_entry : last_component_snapshots_by_source_) {
      const int source_id = snapshot_entry.first;
      const auto& snapshot = snapshot_entry.second;
      if (!snapshotMatchesAuthoritativeView(snapshot)) {
        continue;
      }
      const size_t owner_count = std::min(snapshot.grid_ids.size(),
          std::min(snapshot.owner_ids.size(), snapshot.owner_versions.size()));
      for (size_t i = 0; i < owner_count; ++i) {
        const int grid_id = snapshot.grid_ids[i];
        const int owner_id = snapshot.owner_ids[i];
        const uint64_t owner_version = snapshot.owner_versions[i];
        const auto override_version_it = local_override_versions_.find(grid_id);
        if (override_version_it != local_override_versions_.end()) {
          const double override_stamp = local_override_stamps_[grid_id];
          if (owner_version < override_version_it->second ||
              (owner_version == override_version_it->second &&
               snapshot.stamp < override_stamp)) {
            continue;
          }
        }

        auto latest_version_it = latest_owner_versions_.find(grid_id);
        if (latest_version_it == latest_owner_versions_.end() ||
            latest_version_it->second < owner_version) {
          latest_owner_versions_[grid_id] = owner_version;
        }

        auto ownership_it = ownership_table_.find(grid_id);
        if (ownership_it == ownership_table_.end() ||
            ownership_it->second.owner_version < owner_version) {
          ownership_table_[grid_id] = OwnershipRecord{ owner_id, owner_version };
          ownership_source_ids[grid_id] = source_id;
          continue;
        }
        if (ownership_it->second.owner_version != owner_version) {
          continue;
        }

        const int current_source_id = ownership_source_ids[grid_id];
        // Deterministic tie-break for same-version conflicts: local override wins earlier;
        // otherwise the smaller source_id wins.
        if (ownership_it->second.owner_id != owner_id && source_id < current_source_id) {
          ownership_table_[grid_id] = OwnershipRecord{ owner_id, owner_version };
          ownership_source_ids[grid_id] = source_id;
        } else if (ownership_it->second.owner_id == owner_id && source_id < current_source_id) {
          ownership_source_ids[grid_id] = source_id;
        }
      }
    }

    for (const auto& snapshot_entry : last_component_snapshots_by_source_) {
      const auto& snapshot = snapshot_entry.second;
      if (!snapshotMatchesAuthoritativeView(snapshot)) {
        continue;
      }
      for (const int grid_id : snapshot.free_grid_ids) {
        const auto override_version_it = local_override_versions_.find(grid_id);
        if (override_version_it != local_override_versions_.end() &&
            snapshot.stamp < local_override_stamps_[grid_id]) {
          continue;
        }
        if (ownership_table_.find(grid_id) != ownership_table_.end()) {
          continue;
        }
        const auto latest_version_it = latest_owner_versions_.find(grid_id);
        const uint64_t owner_version =
            latest_version_it == latest_owner_versions_.end() ? 0 : latest_version_it->second;
        auto free_it = free_grid_pool_.find(grid_id);
        if (free_it == free_grid_pool_.end() || free_it->second < owner_version) {
          free_grid_pool_[grid_id] = owner_version;
        }
      }
    }

    for (const auto& override_entry : local_override_versions_) {
      const int grid_id = override_entry.first;
      const uint64_t owner_version = override_entry.second;
      const int owner_id = local_override_owner_ids_[grid_id];
      auto latest_version_it = latest_owner_versions_.find(grid_id);
      if (latest_version_it == latest_owner_versions_.end() ||
          latest_version_it->second < owner_version) {
        latest_owner_versions_[grid_id] = owner_version;
      }
      if (owner_id > 0) {
        ownership_table_[grid_id] = OwnershipRecord{ owner_id, owner_version };
        free_grid_pool_.erase(grid_id);
      } else {
        ownership_table_.erase(grid_id);
        auto free_it = free_grid_pool_.find(grid_id);
        if (free_it == free_grid_pool_.end() || free_it->second < owner_version) {
          free_grid_pool_[grid_id] = owner_version;
        }
      }
    }
  }

  void componentStateCallback(const exploration_manager::ComponentStateConstPtr& msg) {
    if (msg->source_id <= 0) {
      return;
    }

    const std::string authoritative_accept_reason =
        authoritativeAcceptanceReason(*msg);
    const bool is_authoritative_source = !authoritative_accept_reason.empty();
    const bool authoritative_view_changed =
        component_epoch_ != msg->component_epoch || leader_id_ != msg->leader_id;

    if (is_authoritative_source) {
      ROS_INFO_STREAM("[ALLOC_COORD][accepted_authoritative_component_state]"
                      << " drone_id_=" << drone_id_
                      << " local_leader_id_=" << leader_id_
                      << " incoming_leader_id=" << msg->leader_id
                      << " local_component_epoch_=" << component_epoch_
                      << " incoming_component_epoch=" << msg->component_epoch
                      << " pending_plan_active_=" << pending_plan_active_
                      << " pending_plan_txn_id_=" << pending_plan_txn_id_
                      << " pending_plan_grid_id_=" << pending_plan_grid_id_
                      << " pending_plan_owner_id_=" << pending_plan_owner_id_
                      << " free_grid_pool_size=" << free_grid_pool_.size()
                      << " requested_grid_id=-1"
                      << " requested_grid_in_free_pool=false"
                      << " source_id=" << msg->source_id
                      << " authoritative_accept_reason=" << authoritative_accept_reason
                      << " authoritative_source_timed_out=" << authoritativeSourceTimedOut()
                      << " view_changed=" << authoritative_view_changed);
      if (authoritative_view_changed) {
        clearPendingPlan();
        last_component_snapshots_by_source_.clear();
        ownership_table_.clear();
        free_grid_pool_.clear();
        latest_owner_versions_.clear();
        local_override_owner_ids_.clear();
        local_override_versions_.clear();
        local_override_stamps_.clear();
      }

      component_epoch_ = msg->component_epoch;
      leader_id_ = msg->leader_id;
      component_stamp_ = msg->stamp;
      is_component_leader_ = (leader_id_ > 0 && leader_id_ == drone_id_);
    } else {
      ROS_INFO_STREAM("[ALLOC_COORD][cached_non_authoritative_snapshot]"
                      << " drone_id_=" << drone_id_
                      << " authoritative_leader_id_=" << leader_id_
                      << " incoming_leader_id=" << msg->leader_id
                      << " authoritative_component_epoch_=" << component_epoch_
                      << " incoming_component_epoch=" << msg->component_epoch
                      << " pending_plan_active_=" << pending_plan_active_
                      << " pending_plan_txn_id_=" << pending_plan_txn_id_
                      << " pending_plan_grid_id_=" << pending_plan_grid_id_
                      << " pending_plan_owner_id_=" << pending_plan_owner_id_
                      << " free_grid_pool_size=" << free_grid_pool_.size()
                      << " requested_grid_id=-1"
                      << " requested_grid_in_free_pool=false"
                      << " source_id=" << msg->source_id
                      << " matches_authoritative_view="
                      << snapshotMatchesAuthoritativeView(*msg));
    }

    last_component_snapshots_by_source_[msg->source_id] = *msg;
    rebuildAggregatedState();
  }

  void allocationRequestCallback(const exploration_manager::AllocationRequestConstPtr& msg) {
    last_allocation_request_txn_id_ = msg->txn_id;
    last_allocation_requester_id_ = msg->requester_id;
    const int requested_grid_id = msg->grid_ids.empty() ? -1 : msg->grid_ids.front();
    const bool requested_grid_in_free_pool =
        requested_grid_id >= 0 && free_grid_pool_.find(requested_grid_id) != free_grid_pool_.end();

    if (!is_component_leader_) {
      ROS_WARN_STREAM("[ALLOC_COORD][drop_request:not_component_leader]"
                      << " drone_id_=" << drone_id_
                      << " local_leader_id_=" << leader_id_
                      << " request_leader_id=" << msg->leader_id
                      << " local_component_epoch_=" << component_epoch_
                      << " request_component_epoch=" << msg->component_epoch
                      << " pending_plan_active_=" << pending_plan_active_
                      << " pending_plan_txn_id_=" << pending_plan_txn_id_
                      << " pending_plan_grid_id_=" << pending_plan_grid_id_
                      << " pending_plan_owner_id_=" << pending_plan_owner_id_
                      << " free_grid_pool_size=" << free_grid_pool_.size()
                      << " requested_grid_id=" << requested_grid_id
                      << " requested_grid_in_free_pool=" << requested_grid_in_free_pool);
      return;
    }
    if (msg->leader_id != leader_id_ || msg->component_epoch != component_epoch_) {
      ROS_WARN_STREAM("[ALLOC_COORD][drop_request:epoch_or_leader_mismatch]"
                      << " drone_id_=" << drone_id_
                      << " local_leader_id_=" << leader_id_
                      << " request_leader_id=" << msg->leader_id
                      << " local_component_epoch_=" << component_epoch_
                      << " request_component_epoch=" << msg->component_epoch
                      << " pending_plan_active_=" << pending_plan_active_
                      << " pending_plan_txn_id_=" << pending_plan_txn_id_
                      << " pending_plan_grid_id_=" << pending_plan_grid_id_
                      << " pending_plan_owner_id_=" << pending_plan_owner_id_
                      << " free_grid_pool_size=" << free_grid_pool_.size()
                      << " requested_grid_id=" << requested_grid_id
                      << " requested_grid_in_free_pool=" << requested_grid_in_free_pool);
      return;
    }
    if (pending_plan_active_) {
      ROS_WARN_STREAM("[ALLOC_COORD][drop_request:pending_plan_active]"
                      << " drone_id_=" << drone_id_
                      << " local_leader_id_=" << leader_id_
                      << " request_leader_id=" << msg->leader_id
                      << " local_component_epoch_=" << component_epoch_
                      << " request_component_epoch=" << msg->component_epoch
                      << " pending_plan_active_=" << pending_plan_active_
                      << " pending_plan_txn_id_=" << pending_plan_txn_id_
                      << " pending_plan_grid_id_=" << pending_plan_grid_id_
                      << " pending_plan_owner_id_=" << pending_plan_owner_id_
                      << " free_grid_pool_size=" << free_grid_pool_.size()
                      << " requested_grid_id=" << requested_grid_id
                      << " requested_grid_in_free_pool=" << requested_grid_in_free_pool);
      return;
    }

    bool found_free_grid = false;
    int selected_grid_id = 0;
    OwnershipRecord selected_record{ 0, 0 };
    if (!msg->grid_ids.empty()) {
      for (const int requested_id : msg->grid_ids) {
        const auto free_grid_it = free_grid_pool_.find(requested_id);
        if (free_grid_it == free_grid_pool_.end()) {
          continue;
        }
        found_free_grid = true;
        selected_grid_id = requested_id;
        selected_record = OwnershipRecord{ 0, free_grid_it->second };
        break;
      }
      if (!found_free_grid) {
        ROS_WARN_STREAM("[ALLOC_COORD][drop_request:requested_grids_unavailable]"
                        << " drone_id_=" << drone_id_
                        << " local_leader_id_=" << leader_id_
                        << " request_leader_id=" << msg->leader_id
                        << " local_component_epoch_=" << component_epoch_
                        << " request_component_epoch=" << msg->component_epoch
                        << " pending_plan_active_=" << pending_plan_active_
                        << " pending_plan_txn_id_=" << pending_plan_txn_id_
                        << " pending_plan_grid_id_=" << pending_plan_grid_id_
                        << " pending_plan_owner_id_=" << pending_plan_owner_id_
                        << " free_grid_pool_size=" << free_grid_pool_.size()
                        << " requested_grid_id=" << requested_grid_id
                        << " requested_grid_in_free_pool=" << requested_grid_in_free_pool
                        << " requested_grid_count=" << msg->grid_ids.size());
        return;
      }
    } else {
      for (const auto& free_grid_entry : free_grid_pool_) {
        if (!found_free_grid || free_grid_entry.first < selected_grid_id) {
          found_free_grid = true;
          selected_grid_id = free_grid_entry.first;
          selected_record = OwnershipRecord{ 0, free_grid_entry.second };
        }
      }
    }
    if (!found_free_grid) {
      ROS_WARN_STREAM("[ALLOC_COORD][drop_request:no_free_grid]"
                      << " drone_id_=" << drone_id_
                      << " local_leader_id_=" << leader_id_
                      << " request_leader_id=" << msg->leader_id
                      << " local_component_epoch_=" << component_epoch_
                      << " request_component_epoch=" << msg->component_epoch
                      << " pending_plan_active_=" << pending_plan_active_
                      << " pending_plan_txn_id_=" << pending_plan_txn_id_
                      << " pending_plan_grid_id_=" << pending_plan_grid_id_
                      << " pending_plan_owner_id_=" << pending_plan_owner_id_
                      << " free_grid_pool_size=" << free_grid_pool_.size()
                      << " requested_grid_id=" << requested_grid_id
                      << " requested_grid_in_free_pool=" << requested_grid_in_free_pool);
      return;
    }

    pending_plan_active_ = true;
    pending_plan_txn_id_ = msg->txn_id;
    pending_plan_grid_id_ = selected_grid_id;
    pending_plan_owner_id_ = msg->requester_id;
    pending_plan_owner_version_ = selected_record.owner_version + 1;
    pending_plan_reason_ = msg->reason;
    pending_plan_stamp_ = ros::Time::now().toSec();

    exploration_manager::AssignmentPlan plan_msg;
    plan_msg.component_epoch = component_epoch_;
    plan_msg.leader_id = leader_id_;
    plan_msg.grid_ids.push_back(pending_plan_grid_id_);
    plan_msg.owner_ids.push_back(pending_plan_owner_id_);
    plan_msg.owner_versions.push_back(pending_plan_owner_version_);
    plan_msg.txn_id = pending_plan_txn_id_;
    plan_msg.reason = pending_plan_reason_;
    plan_msg.stamp = ros::Time::now().toSec();
    assignment_plan_pub_.publish(plan_msg);
  }

  void releaseRequestCallback(const exploration_manager::ReleaseRequestConstPtr& msg) {
    last_release_request_txn_id_ = msg->txn_id;
    if (!is_component_leader_) {
      return;
    }
    if (msg->leader_id != leader_id_ || msg->component_epoch != component_epoch_) {
      return;
    }

    const size_t release_count = std::min(
        msg->grid_ids.size(), std::min(msg->owner_ids.size(), msg->owner_versions.size()));
    for (size_t i = 0; i < release_count; ++i) {
      const auto ownership_it = ownership_table_.find(msg->grid_ids[i]);
      if (ownership_it == ownership_table_.end()) {
        continue;
      }
      if (ownership_it->second.owner_id != msg->owner_ids[i] ||
          ownership_it->second.owner_version != msg->owner_versions[i]) {
        continue;
      }

      latest_owner_versions_[msg->grid_ids[i]] = ownership_it->second.owner_version;
      ownership_table_.erase(ownership_it);
      free_grid_pool_[msg->grid_ids[i]] = msg->owner_versions[i];
      local_override_owner_ids_[msg->grid_ids[i]] = 0;
      local_override_versions_[msg->grid_ids[i]] = msg->owner_versions[i];
      local_override_stamps_[msg->grid_ids[i]] = ros::Time::now().toSec();
    }
  }

  void assignmentAckCallback(const exploration_manager::AssignmentAckConstPtr& msg) {
    if (!is_component_leader_) {
      return;
    }
    if (!pending_plan_active_) {
      return;
    }
    if (msg->leader_id != leader_id_ || msg->component_epoch != component_epoch_ ||
        msg->txn_id != pending_plan_txn_id_) {
      return;
    }
    if (msg->grid_id != pending_plan_grid_id_ || msg->owner_id != pending_plan_owner_id_ ||
        msg->owner_version != pending_plan_owner_version_) {
      return;
    }
    if (!msg->accept) {
      return;
    }

    exploration_manager::AssignmentCommit commit_msg;
    commit_msg.component_epoch = component_epoch_;
    commit_msg.leader_id = leader_id_;
    commit_msg.grid_ids.push_back(pending_plan_grid_id_);
    commit_msg.owner_ids.push_back(pending_plan_owner_id_);
    commit_msg.owner_versions.push_back(pending_plan_owner_version_);
    commit_msg.txn_id = pending_plan_txn_id_;
    commit_msg.reason = pending_plan_reason_;
    commit_msg.stamp = ros::Time::now().toSec();
    assignment_commit_pub_.publish(commit_msg);

    ownership_table_[pending_plan_grid_id_] = OwnershipRecord{
        pending_plan_owner_id_, pending_plan_owner_version_ };
    latest_owner_versions_[pending_plan_grid_id_] = pending_plan_owner_version_;
    free_grid_pool_.erase(pending_plan_grid_id_);
    local_override_owner_ids_[pending_plan_grid_id_] = pending_plan_owner_id_;
    local_override_versions_[pending_plan_grid_id_] = pending_plan_owner_version_;
    local_override_stamps_[pending_plan_grid_id_] = ros::Time::now().toSec();
    clearPendingPlan();
  }

  ros::Subscriber component_state_sub_, allocation_request_sub_, release_request_sub_,
      assignment_ack_sub_;
  ros::Publisher assignment_plan_pub_, assignment_commit_pub_;
  ros::Timer pending_plan_timer_;

  int drone_id_ = 1;
  int leader_id_ = 0;
  bool is_component_leader_ = false;
  uint64_t component_epoch_ = 0;
  double component_stamp_ = 0.0;
  uint64_t last_allocation_request_txn_id_ = 0;
  int last_allocation_requester_id_ = 0;
  uint64_t last_release_request_txn_id_ = 0;
  bool pending_plan_active_ = false;
  uint64_t pending_plan_txn_id_ = 0;
  int pending_plan_grid_id_ = 0;
  int pending_plan_owner_id_ = 0;
  uint64_t pending_plan_owner_version_ = 0;
  double pending_plan_stamp_ = 0.0;
  double pending_plan_timeout_sec_ = 1.0;
  double component_snapshot_timeout_sec_ = 1.0;
  double local_override_timeout_sec_ = 1.0;
  std::string pending_plan_reason_;
  std::unordered_map<int, ComponentSnapshot> last_component_snapshots_by_source_;
  std::unordered_map<int, uint64_t> free_grid_pool_;
  std::unordered_map<int, uint64_t> latest_owner_versions_;
  std::unordered_map<int, int> local_override_owner_ids_;
  std::unordered_map<int, uint64_t> local_override_versions_;
  std::unordered_map<int, double> local_override_stamps_;
  std::unordered_map<int, OwnershipRecord> ownership_table_;
};

}  // namespace fast_planner

int main(int argc, char** argv) {
  ros::init(argc, argv, "component_allocation_coordinator_node");
  ros::NodeHandle nh("~");

  fast_planner::ComponentAllocationCoordinator coordinator;
  coordinator.init(nh);

  ros::Duration(1.0).sleep();
  ros::spin();

  return 0;
}
