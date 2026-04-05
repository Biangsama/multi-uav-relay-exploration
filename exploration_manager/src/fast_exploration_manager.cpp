// #include <fstream>
#include <exploration_manager/fast_exploration_manager.h>
#include <thread>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <sstream>
#include <unordered_set>
#include <active_perception/graph_node.h>
#include <active_perception/graph_search.h>
#include <active_perception/perception_utils.h>
#include <active_perception/frontier_finder.h>
// #include <active_perception/uniform_grid.h>
#include <active_perception/hgrid.h>
#include <plan_env/raycast.h>
#include <plan_env/sdf_map.h>
#include <plan_env/edt_environment.h>
#include <plan_manage/planner_manager.h>
// #include <lkh_tsp_solver/lkh_interface.h>
// #include <lkh_mtsp_solver/lkh3_interface.h>
#include <lkh_tsp_solver/SolveTSP.h>
#include <lkh_mtsp_solver/SolveMTSP.h>

#include <exploration_manager/expl_data.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Int32MultiArray.h>

using namespace Eigen;

namespace fast_planner {
namespace {
constexpr double kGlobalAllocPeerStateFreshnessSec = 0.5;

struct AllocationSolverDispatch {
  int prob_type;
  int request_prob;
  const char* dispatch_mode;
  const char* type_name;
  const char* file_stem;
  const char* client_name;
  const char* service_name_prefix;
  const char* backend_name;
  bool use_acvrp_client;
};

const char* allocationProbTypeLabel(const int prob_type) {
  switch (prob_type) {
    case 1:
      return "ATSP family";
    case 2:
      return "ACVRP family";
    default:
      return "unknown";
  }
}

const char* allocationRequestProbLabel(const int request_prob) {
  switch (request_prob) {
    case 1:
      return "amtsp / ATSP";
    case 2:
      return "amtsp2 / ATSP";
    case 3:
      return "amtsp3 / ACVRP";
    default:
      return "unknown";
  }
}

// Solver family selection only applies after small-grid direct-assignment shortcuts.
// Single-drone allocation keeps the ATSP path, while multi-drone general allocation
// uses the capacity-aware ACVRP path.
AllocationSolverDispatch selectAllocationSolverDispatch(const int drone_num) {
  if (drone_num > 1) {
    return { 2, 3, "multi-drone-general", "ACVRP", "amtsp3", "acvrp_client_",
        "/solve_acvrp_", "/usr/local/bin/LKH", true };
  }
  return { 1, 1, "single-drone-general", "ATSP", "amtsp", "tsp_client_",
      "/solve_tsp_", "solveMTSPWithLKH3", false };
}

constexpr int kMinScaledDemandBudget = 256;
constexpr int kScaledDemandUnitsPerNonEmptyGrid = 8;

struct AllocationDemandModel {
  vector<int> demands;
  int total_unknown;
  int positive_grid_num;
  int zero_grid_num;
  int target_total_demand;
  int total_demand;
  int max_demand;
  int min_nonzero_demand;
  bool scaled;
  double scale_ratio;
};

AllocationDemandModel buildAllocationDemandModel(const vector<int>& unknown_nums) {
  AllocationDemandModel model;
  model.demands.assign(unknown_nums.size(), 0);
  model.total_unknown = 0;
  model.positive_grid_num = 0;
  model.zero_grid_num = 0;
  model.target_total_demand = 0;
  model.total_demand = 0;
  model.max_demand = 0;
  model.min_nonzero_demand = 0;
  model.scaled = false;
  model.scale_ratio = 1.0;

  vector<int> nonempty_indices;
  nonempty_indices.reserve(unknown_nums.size());
  for (int i = 0; i < static_cast<int>(unknown_nums.size()); ++i) {
    const int unknown_num = std::max(0, unknown_nums[i]);
    model.total_unknown += unknown_num;
    if (unknown_num > 0) {
      nonempty_indices.push_back(i);
    }
  }

  model.positive_grid_num = static_cast<int>(nonempty_indices.size());
  model.zero_grid_num = static_cast<int>(unknown_nums.size()) - model.positive_grid_num;
  if (model.total_unknown <= 0 || nonempty_indices.empty()) {
    return model;
  }

  const int scaling_budget = std::max(kMinScaledDemandBudget,
      model.positive_grid_num * kScaledDemandUnitsPerNonEmptyGrid);
  model.target_total_demand = std::min(model.total_unknown, scaling_budget);
  model.scaled = model.target_total_demand < model.total_unknown;
  model.scale_ratio = static_cast<double>(model.target_total_demand) /
                      static_cast<double>(model.total_unknown);

  if (!model.scaled) {
    model.demands = unknown_nums;
  } else {
    for (const int idx : nonempty_indices) {
      model.demands[idx] = 1;
    }

    const int remaining_budget = model.target_total_demand - model.positive_grid_num;
    const int scalable_unknown = model.total_unknown - model.positive_grid_num;
    struct RemainderEntry {
      double fractional;
      int idx;
      int unknown_num;
    };

    vector<RemainderEntry> remainders;
    remainders.reserve(nonempty_indices.size());
    int assigned_extra = 0;
    if (remaining_budget > 0 && scalable_unknown > 0) {
      for (const int idx : nonempty_indices) {
        const double exact_extra = static_cast<double>(remaining_budget) *
            static_cast<double>(unknown_nums[idx] - 1) / static_cast<double>(scalable_unknown);
        const int extra_floor = static_cast<int>(std::floor(exact_extra));
        model.demands[idx] += extra_floor;
        assigned_extra += extra_floor;
        remainders.push_back({ exact_extra - static_cast<double>(extra_floor), idx, unknown_nums[idx] });
      }

      const int extras_left = remaining_budget - assigned_extra;
      std::sort(remainders.begin(), remainders.end(), [](const RemainderEntry& lhs,
                                                            const RemainderEntry& rhs) {
        if (std::fabs(lhs.fractional - rhs.fractional) > 1e-12) {
          return lhs.fractional > rhs.fractional;
        }
        if (lhs.unknown_num != rhs.unknown_num) {
          return lhs.unknown_num > rhs.unknown_num;
        }
        return lhs.idx < rhs.idx;
      });

      for (int i = 0; i < extras_left && i < static_cast<int>(remainders.size()); ++i) {
        model.demands[remainders[i].idx] += 1;
      }
    }
  }

  model.min_nonzero_demand = std::numeric_limits<int>::max();
  for (const int demand : model.demands) {
    model.total_demand += demand;
    model.max_demand = std::max(model.max_demand, demand);
    if (demand > 0) {
      model.min_nonzero_demand = std::min(model.min_nonzero_demand, demand);
    }
  }
  if (model.min_nonzero_demand == std::numeric_limits<int>::max()) {
    model.min_nonzero_demand = 0;
  }

  return model;
}
}
// SECTION interfaces for setup and query

FastExplorationManager::FastExplorationManager() {
}

FastExplorationManager::~FastExplorationManager() {
  ViewNode::astar_.reset();
  ViewNode::caster_.reset();
  ViewNode::map_.reset();
}

bool FastExplorationManager::tryPlanGoalCandidate(const Vector3d& start_pos,
    const Vector3d& candidate, bool optimistic, Vector3d& safe_goal, vector<Vector3d>& safe_path) {
  if (!sdf_map_ || !planner_manager_ || !planner_manager_->path_finder_) {
    return false;
  }
  if (!sdf_map_->isInBox(candidate)) {
    return false;
  }
  if (sdf_map_->getInflateOccupancy(candidate) == 1 ||
      sdf_map_->getOccupancy(candidate) == SDFMap::OCCUPIED) {
    return false;
  }

  planner_manager_->path_finder_->reset();
  if (planner_manager_->path_finder_->search(start_pos, candidate, optimistic) != Astar::REACH_END) {
    return false;
  }

  safe_goal = candidate;
  safe_path = planner_manager_->path_finder_->getPath();
  return !safe_path.empty();
}

bool FastExplorationManager::findSafeReachableGoal(const Vector3d& start_pos,
    const Vector3d& raw_goal, bool optimistic, Vector3d& safe_goal, vector<Vector3d>& safe_path) {
  if (!sdf_map_) {
    return false;
  }

  const double res = sdf_map_->getResolution();
  Vector3d box_min, box_max;
  sdf_map_->getBox(box_min, box_max);

  auto clamp_to_box = [&](const Vector3d& pos) {
    Vector3d clamped = pos;
    for (int i = 0; i < 3; ++i) {
      clamped[i] = std::max(box_min[i] + res, std::min(box_max[i] - res, clamped[i]));
    }
    return clamped;
  };

  const Vector3d clamped_goal = clamp_to_box(raw_goal);
  const Vector3d goal_dir = clamped_goal - start_pos;
  const double goal_dir_norm = goal_dir.norm();
  const double goal_dir_xy_norm = std::hypot(goal_dir.x(), goal_dir.y());
  Vector3d lateral_dir = Vector3d::Zero();
  if (goal_dir_xy_norm > 1e-3) {
    lateral_dir = Vector3d(-goal_dir.y() / goal_dir_xy_norm, goal_dir.x() / goal_dir_xy_norm, 0.0);
  }

  const auto try_mode = [&](const bool search_optimistic) {
    if (tryPlanGoalCandidate(start_pos, clamped_goal, search_optimistic, safe_goal, safe_path)) {
      return true;
    }

    if (goal_dir_norm > 1e-3) {
      for (int step = 19; step >= 1; --step) {
        const double frac = 0.05 * static_cast<double>(step);
        const Vector3d candidate = clamp_to_box(start_pos + frac * goal_dir);
        if (tryPlanGoalCandidate(start_pos, candidate, search_optimistic, safe_goal, safe_path)) {
          return true;
        }
      }
    }

    Eigen::Vector3d grad;
    if (sdf_map_->getDistWithGrad(clamped_goal, grad) > 0.0 && grad.norm() > 1e-3) {
      const Vector3d pushed_goal =
          clamp_to_box(clamped_goal + grad.normalized() * std::max(0.6, 2.0 * res));
      if (tryPlanGoalCandidate(start_pos, pushed_goal, search_optimistic, safe_goal, safe_path)) {
        return true;
      }
    }

    if (goal_dir_xy_norm > 1e-3) {
      const std::array<double, 4> lateral_fractions = {0.8, 0.6, 0.4, 0.2};
      const std::array<double, 3> lateral_offsets = {
          std::max(0.6, 2.0 * res), std::max(1.2, 4.0 * res), std::max(1.8, 6.0 * res)};
      for (const double frac : lateral_fractions) {
        Vector3d on_line = start_pos + frac * goal_dir;
        on_line.z() = start_pos.z() + frac * (clamped_goal.z() - start_pos.z());
        for (const double offset : lateral_offsets) {
          for (const double sign : {-1.0, 1.0}) {
            const Vector3d candidate = clamp_to_box(on_line + sign * offset * lateral_dir);
            if (tryPlanGoalCandidate(start_pos, candidate, search_optimistic, safe_goal, safe_path)) {
              return true;
            }
          }
        }
      }
    }

    const std::array<Eigen::Vector2d, 8> directions = {
        Eigen::Vector2d(1.0, 0.0), Eigen::Vector2d(-1.0, 0.0), Eigen::Vector2d(0.0, 1.0),
        Eigen::Vector2d(0.0, -1.0), Eigen::Vector2d(0.70710678, 0.70710678),
        Eigen::Vector2d(0.70710678, -0.70710678), Eigen::Vector2d(-0.70710678, 0.70710678),
        Eigen::Vector2d(-0.70710678, -0.70710678)};
      const std::array<double, 5> radii = {std::max(0.6, 2.0 * res), std::max(1.2, 4.0 * res),
          std::max(1.8, 6.0 * res), std::max(2.4, 8.0 * res), std::max(3.0, 10.0 * res)};
    const std::array<double, 3> z_offsets = {0.0, res, -res};
    for (const double radius : radii) {
      for (const double z_offset : z_offsets) {
        for (const auto& dir : directions) {
          Vector3d candidate = clamped_goal;
          candidate.x() += radius * dir.x();
          candidate.y() += radius * dir.y();
          candidate.z() += z_offset;
          candidate = clamp_to_box(candidate);
          if (tryPlanGoalCandidate(start_pos, candidate, search_optimistic, safe_goal, safe_path)) {
            return true;
          }
        }
      }
    }

    return false;
  };

  if (try_mode(optimistic)) {
    return true;
  }
  if (!optimistic && try_mode(true)) {
    return true;
  }

  return false;
}

void FastExplorationManager::initialize(ros::NodeHandle& nh) {
  planner_manager_.reset(new FastPlannerManager);
  planner_manager_->initPlanModules(nh);

  edt_environment_ = planner_manager_->edt_environment_;
  sdf_map_ = edt_environment_->sdf_map_;
  frontier_finder_.reset(new FrontierFinder(edt_environment_, nh));
  // uniform_grid_.reset(new UniformGrid(edt_environment_, nh));
  hgrid_.reset(new HGrid(edt_environment_, nh));
  // view_finder_.reset(new ViewFinder(edt_environment_, nh));

  ed_.reset(new ExplorationData);
  ep_.reset(new ExplorationParam);

  nh.param("exploration/refine_local", ep_->refine_local_, true);
  nh.param("exploration/no_grid_keep_moving", ep_->no_grid_keep_moving_, false);
  nh.param("exploration/refined_num", ep_->refined_num_, -1);
  nh.param("exploration/refined_radius", ep_->refined_radius_, -1.0);
  nh.param("exploration/top_view_num", ep_->top_view_num_, -1);
  nh.param("exploration/max_decay", ep_->max_decay_, -1.0);
  nh.param("exploration/tsp_dir", ep_->tsp_dir_, string("null"));
  nh.param("exploration/mtsp_dir", ep_->mtsp_dir_, string("null"));
  nh.param("exploration/relax_time", ep_->relax_time_, 1.0);
  nh.param("exploration/drone_num", ep_->drone_num_, 1);
  nh.param("exploration/drone_id", ep_->drone_id_, 1);
  nh.param("exploration/init_plan_num", ep_->init_plan_num_, 2);

  ed_->swarm_state_.resize(ep_->drone_num_);
  ed_->pair_opt_stamps_.resize(ep_->drone_num_);
  ed_->pair_opt_res_stamps_.resize(ep_->drone_num_);
  for (int i = 0; i < ep_->drone_num_; ++i) {
    ed_->swarm_state_[i].stamp_ = 0.0;
    ed_->swarm_state_[i].relay_role_ = 0;
    ed_->swarm_state_[i].task_assignable_ = true;
    ed_->pair_opt_stamps_[i] = 0.0;
    ed_->pair_opt_res_stamps_[i] = 0.0;
  }
  planner_manager_->swarm_traj_data_.init(ep_->drone_id_, ep_->drone_num_);

  nh.param("exploration/vm", ViewNode::vm_, -1.0);
  nh.param("exploration/am", ViewNode::am_, -1.0);
  nh.param("exploration/yd", ViewNode::yd_, -1.0);
  nh.param("exploration/ydd", ViewNode::ydd_, -1.0);
  nh.param("exploration/w_dir", ViewNode::w_dir_, -1.0);

  ViewNode::astar_.reset(new Astar);
  ViewNode::astar_->init(nh, edt_environment_);
  ViewNode::map_ = sdf_map_;

  double resolution_ = sdf_map_->getResolution();
  Eigen::Vector3d origin, size;
  sdf_map_->getRegion(origin, size);
  ViewNode::caster_.reset(new RayCaster);
  ViewNode::caster_->setParams(resolution_, origin);

  planner_manager_->path_finder_->lambda_heu_ = 1.0;
  // planner_manager_->path_finder_->max_search_time_ = 0.05;
  planner_manager_->path_finder_->max_search_time_ = 1.0;

  tsp_client_ =
      nh.serviceClient<lkh_mtsp_solver::SolveMTSP>("/solve_tsp_" + to_string(ep_->drone_id_), true);
  acvrp_client_ = nh.serviceClient<lkh_mtsp_solver::SolveMTSP>(
      "/solve_acvrp_" + to_string(ep_->drone_id_), true);
  task_metrics_pub_ = nh.advertise<std_msgs::Int32MultiArray>("/relay_integration/task_metrics", 10);

  // Swarm
  for (auto& state : ed_->swarm_state_) {
    state.stamp_ = 0.0;
    state.recent_interact_time_ = 0.0;
    state.recent_attempt_time_ = 0.0;
  }
  ed_->last_grid_ids_ = {};
  ed_->reallocated_ = true;
  ed_->pair_opt_stamp_ = 0.0;
  ed_->wait_response_ = false;
  ed_->pending_grid_ids_.clear();
  ed_->pending_claim_grid_ids_.clear();
  ed_->pending_relay_recover_grid_ids_.clear();
  ed_->pending_pair_opt_grid_ids_.clear();
  ed_->pending_pair_opt_peer_grid_ids_.clear();
  ed_->pending_release_grid_ids_.clear();
  ed_->pending_invalidated_grid_ids_.clear();
  ed_->pending_plan_fail_release_grid_ids_.clear();
  ed_->pending_commit_grid_ids_.clear();
  ed_->pending_commit_peer_grid_ids_.clear();
  ed_->pending_relay_enter_release_grid_ids_.clear();
  ed_->plan_num_ = 0;

  // Analysis
  // ofstream fout;
  // fout.open("/home/boboyu/Desktop/RAL_Time/frontier.txt");
  // fout.close();
}

int FastExplorationManager::planExploreMotion(
    const Vector3d& pos, const Vector3d& vel, const Vector3d& acc, const Vector3d& yaw) {
  ros::Time t1 = ros::Time::now();
  auto t2 = t1;

  std::cout << "start pos: " << pos.transpose() << ", vel: " << vel.transpose()
            << ", acc: " << acc.transpose() << std::endl;

  // Do global and local tour planning and retrieve the next viewpoint

  ed_->frontier_tour_.clear();
  Vector3d next_pos;
  double next_yaw;
  // Find the tour passing through viewpoints
  // Optimal tour is returned as indices of frontier
  vector<int> grid_ids, frontier_ids;
  // findGlobalTour(pos, vel, yaw, indices);
  findGridAndFrontierPath(pos, vel, yaw, grid_ids, frontier_ids);

  if (grid_ids.empty()) {
    if (!ep_->no_grid_keep_moving_ || ed_->points_.empty() || ed_->yaws_.size() != ed_->points_.size()) {
      return NO_GRID;
    }

    // Keep an unallocated drone moving toward the closest frontier viewpoint instead of
    // hovering in place when the current grid partition leaves it empty-handed.
    ROS_WARN("Empty grid, fallback to nearest frontier viewpoint");

    double min_cost = 100000.0;
    int min_cost_id = -1;
    vector<Vector3d> tmp_path;
    for (int i = 0; i < static_cast<int>(ed_->points_.size()); ++i) {
      const double tmp_cost =
          ViewNode::computeCost(pos, ed_->points_[i], yaw[0], ed_->yaws_[i], vel, yaw[1], tmp_path);
      if (tmp_cost < min_cost) {
        min_cost = tmp_cost;
        min_cost_id = i;
      }
    }

    if (min_cost_id < 0) {
      return NO_GRID;
    }

    next_pos = ed_->points_[min_cost_id];
    next_yaw = ed_->yaws_[min_cost_id];
    ed_->refined_ids_.clear();
    ed_->n_points_.clear();
    ed_->unrefined_points_ = { next_pos };
    ed_->refined_points_ = { next_pos };
    ed_->refined_views_ = { next_pos + 2.0 * Vector3d(cos(next_yaw), sin(next_yaw), 0) };
    ed_->refined_views1_.clear();
    ed_->refined_views2_.clear();

  } else if (frontier_ids.size() == 0) {
    // // The assigned grid contains no frontier, find the one closest to the grid
    // ROS_WARN("No frontier in grid");

    Eigen::Vector3d grid_center = ed_->grid_tour_[1];

    double min_cost = 100000;
    int min_cost_id = -1;
    for (int i = 0; i < ed_->points_.size(); ++i) {
      // double cost = (grid_center - ed_->averages_[i]).norm();
      vector<Eigen::Vector3d> path;
      double cost = ViewNode::computeCost(
          grid_center, ed_->averages_[i], 0, 0, Eigen::Vector3d(0, 0, 0), 0, path);
      if (cost < min_cost) {
        min_cost = cost;
        min_cost_id = i;
      }
    }
    next_pos = ed_->points_[min_cost_id];
    next_yaw = ed_->yaws_[min_cost_id];

    // // Simply go to the center of the unknown grid
    // next_pos = grid_center;
    // Eigen::Vector3d dir = grid_center - pos;
    // next_yaw = atan2(dir[1], dir[0]);

  } else if (frontier_ids.size() == 1) {
    // ROS_WARN("Single frontier");
    if (ep_->refine_local_) {
      // Single frontier, find the min cost viewpoint for it
      ed_->refined_ids_ = { frontier_ids[0] };
      ed_->unrefined_points_ = { ed_->points_[frontier_ids[0]] };
      ed_->n_points_.clear();
      vector<vector<double>> n_yaws;
      frontier_finder_->getViewpointsInfo(
          pos, { frontier_ids[0] }, ep_->top_view_num_, ep_->max_decay_, ed_->n_points_, n_yaws);

      if (grid_ids.size() <= 1) {
        // Only one grid is assigned
        double min_cost = 100000;
        int min_cost_id = -1;
        vector<Vector3d> tmp_path;
        for (int i = 0; i < ed_->n_points_[0].size(); ++i) {
          auto tmp_cost = ViewNode::computeCost(
              pos, ed_->n_points_[0][i], yaw[0], n_yaws[0][i], vel, yaw[1], tmp_path);
          if (tmp_cost < min_cost) {
            min_cost = tmp_cost;
            min_cost_id = i;
          }
        }
        next_pos = ed_->n_points_[0][min_cost_id];
        next_yaw = n_yaws[0][min_cost_id];
      } else {
        // More than one grid, the next grid is considered for path planning
        // vector<Eigen::Vector3d> grid_pos = { ed_->grid_tour_[2] };
        // Eigen::Vector3d dir = ed_->grid_tour_[2] - ed_->grid_tour_[1];
        // vector<double> grid_yaw = { atan2(dir[1], dir[0]) };

        Eigen::Vector3d grid_pos;
        double grid_yaw;
        if (hgrid_->getNextGrid(grid_ids, grid_pos, grid_yaw)) {
          ed_->n_points_.push_back({ grid_pos });
          n_yaws.push_back({ grid_yaw });
        }

        ed_->refined_points_.clear();
        ed_->refined_views_.clear();
        vector<double> refined_yaws;
        refineLocalTour(pos, vel, yaw, ed_->n_points_, n_yaws, ed_->refined_points_, refined_yaws);
        next_pos = ed_->refined_points_[0];
        next_yaw = refined_yaws[0];
      }
      ed_->refined_points_ = { next_pos };
      ed_->refined_views_ = { next_pos + 2.0 * Vector3d(cos(next_yaw), sin(next_yaw), 0) };
    }
  } else {
    // ROS_WARN("Multiple frontier");
    // More than two frontiers are assigned
    // Do refinement for the next few viewpoints in the global tour
    t1 = ros::Time::now();

    ed_->refined_ids_.clear();
    ed_->unrefined_points_.clear();
    int knum = min(int(frontier_ids.size()), ep_->refined_num_);
    for (int i = 0; i < knum; ++i) {
      auto tmp = ed_->points_[frontier_ids[i]];
      ed_->unrefined_points_.push_back(tmp);
      ed_->refined_ids_.push_back(frontier_ids[i]);
      if ((tmp - pos).norm() > ep_->refined_radius_ && ed_->refined_ids_.size() >= 2) break;
    }

    // Get top N viewpoints for the next K frontiers
    ed_->n_points_.clear();
    vector<vector<double>> n_yaws;
    frontier_finder_->getViewpointsInfo(
        pos, ed_->refined_ids_, ep_->top_view_num_, ep_->max_decay_, ed_->n_points_, n_yaws);

    ed_->refined_points_.clear();
    ed_->refined_views_.clear();
    vector<double> refined_yaws;
    refineLocalTour(pos, vel, yaw, ed_->n_points_, n_yaws, ed_->refined_points_, refined_yaws);
    next_pos = ed_->refined_points_[0];
    next_yaw = refined_yaws[0];

    // Get marker for view visualization
    for (int i = 0; i < ed_->refined_points_.size(); ++i) {
      Vector3d view =
          ed_->refined_points_[i] + 2.0 * Vector3d(cos(refined_yaws[i]), sin(refined_yaws[i]), 0);
      ed_->refined_views_.push_back(view);
    }
    ed_->refined_views1_.clear();
    ed_->refined_views2_.clear();
    for (int i = 0; i < ed_->refined_points_.size(); ++i) {
      vector<Vector3d> v1, v2;
      frontier_finder_->percep_utils_->setPose(ed_->refined_points_[i], refined_yaws[i]);
      frontier_finder_->percep_utils_->getFOV(v1, v2);
      ed_->refined_views1_.insert(ed_->refined_views1_.end(), v1.begin(), v1.end());
      ed_->refined_views2_.insert(ed_->refined_views2_.end(), v2.begin(), v2.end());
    }
    double local_time = (ros::Time::now() - t1).toSec();
    ROS_INFO("Local refine time: %lf", local_time);
  }

  std::cout << "Next view: " << next_pos.transpose() << ", " << next_yaw << std::endl;
  ed_->next_pos_ = next_pos;
  ed_->next_yaw_ = next_yaw;

  if (planTrajToView(pos, vel, acc, yaw, next_pos, next_yaw) == FAIL) {
    return FAIL;
  }

  double total = (ros::Time::now() - t2).toSec();
  ROS_INFO("Total time: %lf", total);
  ROS_ERROR_COND(total > 0.1, "Total time too long!!!");

  return SUCCEED;
}

int FastExplorationManager::planTrajToView(const Vector3d& pos, const Vector3d& vel,
    const Vector3d& acc, const Vector3d& yaw, const Vector3d& next_pos, const double& next_yaw) {

  // Plan trajectory (position and yaw) to the next viewpoint
  auto t1 = ros::Time::now();

  // Compute time lower bound of yaw and use in trajectory generation
  double diff0 = next_yaw - yaw[0];
  double diff1 = fabs(diff0);
  double time_lb = min(diff1, 2 * M_PI - diff1) / ViewNode::yd_;

  bool optimistic = ed_->plan_num_ < ep_->init_plan_num_;
  Vector3d safe_goal;
  vector<Vector3d> safe_path;
  if (!findSafeReachableGoal(pos, next_pos, optimistic, safe_goal, safe_path)) {
    ROS_ERROR_STREAM("No safe reachable goal near target " << next_pos.transpose());
    return FAIL;
  }
  if ((safe_goal - next_pos).norm() > 1e-3) {
    ROS_WARN_STREAM("Adjusted goal from " << next_pos.transpose() << " to "
                    << safe_goal.transpose() << " before trajectory planning");
  }

  ed_->next_pos_ = safe_goal;
  ed_->path_next_goal_ = safe_path;
  shortenPath(ed_->path_next_goal_);
  ed_->kino_path_.clear();

  const double radius_far = 7.0;
  const double radius_close = 1.5;
  const double len = Astar::pathLength(ed_->path_next_goal_);
  if (len < radius_close || optimistic) {
    // Next viewpoint is very close, no need to search kinodynamic path, just use waypoints-based
    // optimization
    planner_manager_->planExploreTraj(ed_->path_next_goal_, vel, acc, time_lb);
    ed_->next_goal_ = safe_goal;
    // std::cout << "Close goal." << std::endl;
    if (ed_->plan_num_ < ep_->init_plan_num_) {
      ed_->plan_num_++;
      ROS_WARN("init plan.");
    }
  } else if (len > radius_far) {
    // Next viewpoint is far away, select intermediate goal on geometric path (this also deal with
    // dead end)
    std::cout << "Far goal." << std::endl;
    double len2 = 0.0;
    vector<Eigen::Vector3d> truncated_path = { ed_->path_next_goal_.front() };
    for (int i = 1; i < ed_->path_next_goal_.size() && len2 < radius_far; ++i) {
      auto cur_pt = ed_->path_next_goal_[i];
      len2 += (cur_pt - truncated_path.back()).norm();
      truncated_path.push_back(cur_pt);
    }
    ed_->next_goal_ = truncated_path.back();
    planner_manager_->planExploreTraj(truncated_path, vel, acc, time_lb);
  } else {
    // Search kino path to exactly next viewpoint and optimize
    std::cout << "Mid goal" << std::endl;
    ed_->next_goal_ = safe_goal;

    if (!planner_manager_->kinodynamicReplan(
            pos, vel, acc, ed_->next_goal_, Vector3d(0, 0, 0), time_lb))
      return FAIL;
    ed_->kino_path_ = planner_manager_->kino_path_finder_->getKinoTraj(0.02);
  }

  if (planner_manager_->local_data_.position_traj_.getTimeSum() < time_lb - 0.5)
    ROS_ERROR("Lower bound not satified!");

  double traj_plan_time = (ros::Time::now() - t1).toSec();

  t1 = ros::Time::now();
  planner_manager_->planYawExplore(yaw, next_yaw, true, ep_->relax_time_);
  double yaw_time = (ros::Time::now() - t1).toSec();
  ROS_INFO("Traj: %lf, yaw: %lf", traj_plan_time, yaw_time);

  return SUCCEED;
}

void FastExplorationManager::publishTaskMetrics() {
  if (!task_metrics_pub_) return;

  vector<int> active_grid_ids;
  hgrid_->getActiveGrids(active_grid_ids);

  int total_unknown_cells = 0;
  for (const int grid_id : active_grid_ids) {
    total_unknown_cells += hgrid_->getUnknownCellsNum(grid_id);
  }

  const bool have_coverable_frontier = !ed_->frontiers_.empty();
  int total_frontier_cells = 0;
  if (have_coverable_frontier) {
    for (const auto& frontier : ed_->frontiers_) {
      total_frontier_cells += frontier.size();
    }
  }

  std_msgs::Int32MultiArray msg;
  msg.data.push_back(total_unknown_cells);
  msg.data.push_back(have_coverable_frontier ? static_cast<int>(ed_->frontiers_.size()) : 0);
  msg.data.push_back(total_frontier_cells);
  msg.data.push_back(have_coverable_frontier ? static_cast<int>(active_grid_ids.size()) : 0);
  msg.data.push_back(ep_->drone_id_);
  task_metrics_pub_.publish(msg);
}

int FastExplorationManager::updateFrontierStruct(const Eigen::Vector3d& pos) {

  auto t1 = ros::Time::now();
  auto t2 = t1;
  ed_->views_.clear();

  // Search frontiers and group them into clusters
  frontier_finder_->searchFrontiers();

  double frontier_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Find viewpoints (x,y,z,yaw) for all clusters; find the informative ones
  frontier_finder_->computeFrontiersToVisit();

  // Retrieve the updated info
  frontier_finder_->getFrontiers(ed_->frontiers_);
  frontier_finder_->getDormantFrontiers(ed_->dead_frontiers_);
  frontier_finder_->getFrontierBoxes(ed_->frontier_boxes_);

  frontier_finder_->getTopViewpointsInfo(pos, ed_->points_, ed_->yaws_, ed_->averages_);
  for (int i = 0; i < ed_->points_.size(); ++i)
    ed_->views_.push_back(
        ed_->points_[i] + 2.0 * Vector3d(cos(ed_->yaws_[i]), sin(ed_->yaws_[i]), 0));

  publishTaskMetrics();

  if (ed_->frontiers_.empty()) {
    ROS_WARN("No coverable frontier.");
    return 0;
  }

  double view_time = (ros::Time::now() - t1).toSec();

  t1 = ros::Time::now();
  frontier_finder_->updateFrontierCostMatrix();

  double mat_time = (ros::Time::now() - t1).toSec();
  double total_time = frontier_time + view_time + mat_time;
  ROS_INFO("Drone %d: frontier t: %lf, viewpoint t: %lf, mat: %lf", ep_->drone_id_, frontier_time,
      view_time, mat_time);

  ROS_INFO("Total t: %lf", (ros::Time::now() - t2).toSec());
  return ed_->frontiers_.size();
}

void FastExplorationManager::findGridAndFrontierPath(const Vector3d& cur_pos,
    const Vector3d& cur_vel, const Vector3d& cur_yaw, vector<int>& grid_ids,
    vector<int>& frontier_ids) {
  auto t1 = ros::Time::now();

  // Partitioning-based tour planning
  vector<int> ego_ids;
  vector<vector<int>> other_ids;
  if (!findGlobalTourOfGridFromSwarm(cur_pos, cur_vel, ego_ids, other_ids)) {
    grid_ids = {};
    return;
  }
  grid_ids = ego_ids;

  if (grid_ids.empty()) {
    frontier_ids.clear();
    ROS_WARN_STREAM("[GLOBAL_ALLOC]: drone " << ep_->drone_id_
                    << " received no grids from the global allocator.");
    return;
  }

  double grid_time = (ros::Time::now() - t1).toSec();

  // Frontier-based single drone tour planning
  // Restrict frontier within the first visited grid
  t1 = ros::Time::now();

  vector<int> ftr_ids;
  // uniform_grid_->getFrontiersInGrid(ego_ids[0], ftr_ids);
  hgrid_->getFrontiersInGrid(ego_ids, ftr_ids);
  ROS_INFO("Find frontier tour, %d involved------------", ftr_ids.size());

  if (ftr_ids.empty()) {
    frontier_ids = {};
    return;
  }

  // Consider next grid in frontier tour planning
  Eigen::Vector3d grid_pos;
  double grid_yaw;
  vector<Eigen::Vector3d> grid_pos_vec;
  if (hgrid_->getNextGrid(ego_ids, grid_pos, grid_yaw)) {
    grid_pos_vec = { grid_pos };
  }

  findTourOfFrontier(cur_pos, cur_vel, cur_yaw, ftr_ids, grid_pos_vec, frontier_ids);
  double ftr_time = (ros::Time::now() - t1).toSec();
  ROS_INFO("Grid tour t: %lf, frontier tour t: %lf.", grid_time, ftr_time);
}

void FastExplorationManager::buildSwarmGlobalAllocationInputs(const Vector3d& self_pos,
    const Vector3d& self_vel, vector<int>& drone_ids, vector<Eigen::Vector3d>& positions,
    vector<Eigen::Vector3d>& velocities) const {
  drone_ids.clear();
  positions.clear();
  velocities.clear();

  if (!ed_ || !ep_) {
    return;
  }

  drone_ids.push_back(ep_->drone_id_);
  positions.push_back(self_pos);
  velocities.push_back(self_vel);

  const int self_index = std::max(0, ep_->drone_id_ - 1);
  const double now_sec = ros::Time::now().toSec();
  for (int i = 0; i < static_cast<int>(ed_->swarm_state_.size()); ++i) {
    if (i == self_index) {
      continue;
    }

    const auto& candidate = ed_->swarm_state_[i];
    if (!candidate.task_assignable_ || candidate.stamp_ <= 1e-4) {
      continue;
    }
    if (now_sec - candidate.stamp_ > kGlobalAllocPeerStateFreshnessSec) {
      continue;
    }

    drone_ids.push_back(i + 1);
    positions.push_back(candidate.pos_);
    velocities.push_back(candidate.vel_);
  }
}

bool FastExplorationManager::findGlobalTourOfGridFromSwarm(const Vector3d& self_pos,
    const Vector3d& self_vel, vector<int>& indices, vector<vector<int>>& others, bool init) {
  vector<int> drone_ids;
  vector<Eigen::Vector3d> positions, velocities;
  buildSwarmGlobalAllocationInputs(self_pos, self_vel, drone_ids, positions, velocities);

  std::ostringstream drone_stream;
  for (int i = 0; i < static_cast<int>(drone_ids.size()); ++i) {
    if (i != 0) {
      drone_stream << ",";
    }
    drone_stream << drone_ids[i];
  }
  ROS_INFO_STREAM("[GLOBAL_ALLOC]: assembled swarm input drones=" << drone_ids.size()
                  << ", ids=[" << drone_stream.str() << "]");

  return findGlobalTourOfGrid(positions, velocities, drone_ids, indices, others, init);
}

void FastExplorationManager::shortenPath(vector<Vector3d>& path) {
  if (path.empty()) {
    ROS_ERROR("Empty path to shorten");
    return;
  }
  // Shorten the tour, only critical intermediate points are reserved.
  const double dist_thresh = 3.0;
  vector<Vector3d> short_tour = { path.front() };
  for (int i = 1; i < path.size() - 1; ++i) {
    if ((path[i] - short_tour.back()).norm() > dist_thresh)
      short_tour.push_back(path[i]);
    else {
      // Add waypoints to shorten path only to avoid collision
      ViewNode::caster_->input(short_tour.back(), path[i + 1]);
      Eigen::Vector3i idx;
      while (ViewNode::caster_->nextId(idx) && ros::ok()) {
        if (edt_environment_->sdf_map_->getInflateOccupancy(idx) == 1 ||
            edt_environment_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN) {
          short_tour.push_back(path[i]);
          break;
        }
      }
    }
  }
  if ((path.back() - short_tour.back()).norm() > 1e-3) short_tour.push_back(path.back());

  // Ensure at least three points in the path
  if (short_tour.size() == 2)
    short_tour.insert(short_tour.begin() + 1, 0.5 * (short_tour[0] + short_tour[1]));
  path = short_tour;
}

void FastExplorationManager::findGlobalTour(const Vector3d& cur_pos, const Vector3d& cur_vel,
    const Vector3d cur_yaw, vector<int>& indices) {
  auto t1 = ros::Time::now();

  // Get cost matrix for current state and clusters
  Eigen::MatrixXd cost_mat;
  frontier_finder_->getFullCostMatrix(cur_pos, cur_vel, cur_yaw, cost_mat);
  const int dimension = cost_mat.rows();
  std::cout << "mat:   " << cost_mat.rows() << std::endl;

  double mat_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Initialize TSP par file
  ofstream par_file(ep_->tsp_dir_ + "/drone_" + to_string(ep_->drone_id_) + ".par");
  par_file << "PROBLEM_FILE = " << ep_->tsp_dir_ + "/drone_" + to_string(ep_->drone_id_) + ".tsp\n";
  par_file << "GAIN23 = NO\n";
  par_file << "OUTPUT_TOUR_FILE ="
           << ep_->tsp_dir_ + "/drone_" + to_string(ep_->drone_id_) + ".tou"
                                                                      "r\n";
  par_file << "RUNS = 1\n";
  par_file.close();

  // Write params and cost matrix to problem file
  ofstream prob_file(ep_->tsp_dir_ + "/drone_" + to_string(ep_->drone_id_) + ".tsp");
  // Problem specification part, follow the format of TSPLIB
  string prob_spec;
  prob_spec = "NAME : single\nTYPE : ATSP\nDIMENSION : " + to_string(dimension) +
              "\nEDGE_WEIGHT_TYPE : "
              "EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\nEDGE_WEIGHT_SECTION\n";
  prob_file << prob_spec;
  // prob_file << "TYPE : TSP\n";
  // prob_file << "EDGE_WEIGHT_FORMAT : LOWER_ROW\n";
  // Problem data part
  const int scale = 100;
  for (int i = 0; i < dimension; ++i) {
    for (int j = 0; j < dimension; ++j) {
      int int_cost = cost_mat(i, j) * scale;
      prob_file << int_cost << " ";
    }
    prob_file << "\n";
  }
  prob_file << "EOF";
  prob_file.close();

  // solveTSPLKH((ep_->tsp_dir_ + "/drone_" + to_string(ep_->drone_id_) + ".par").c_str());
  lkh_tsp_solver::SolveTSP srv;
  if (!tsp_client_.call(srv)) {
    ROS_ERROR("Fail to solve TSP.");
    return;
  }

  // Read optimal tour from the tour section of result file
  ifstream res_file(ep_->tsp_dir_ + "/drone_" + to_string(ep_->drone_id_) + ".tour");
  string res;
  while (getline(res_file, res)) {
    // Go to tour section
    if (res.compare("TOUR_SECTION") == 0) break;
  }

  // Read path for ATSP formulation
  while (getline(res_file, res)) {
    // Read indices of frontiers in optimal tour
    int id = stoi(res);
    if (id == 1)  // Ignore the current state
      continue;
    if (id == -1) break;
    indices.push_back(id - 2);  // Idx of solver-2 == Idx of frontier
  }

  res_file.close();

  std::cout << "Tour " << ep_->drone_id_ << ": ";
  for (auto id : indices) std::cout << id << ", ";
  std::cout << "" << std::endl;

  // Get the path of optimal tour from path matrix
  frontier_finder_->getPathForTour(cur_pos, indices, ed_->frontier_tour_);

  double tsp_time = (ros::Time::now() - t1).toSec();
  ROS_INFO("Cost mat: %lf, TSP: %lf", mat_time, tsp_time);

  // if (tsp_time > 0.1) ROS_BREAK();
}

void FastExplorationManager::refineLocalTour(const Vector3d& cur_pos, const Vector3d& cur_vel,
    const Vector3d& cur_yaw, const vector<vector<Vector3d>>& n_points,
    const vector<vector<double>>& n_yaws, vector<Vector3d>& refined_pts,
    vector<double>& refined_yaws) {
  double create_time, search_time, parse_time;
  auto t1 = ros::Time::now();

  // Create graph for viewpoints selection
  GraphSearch<ViewNode> g_search;
  vector<ViewNode::Ptr> last_group, cur_group;

  // Add the current state
  ViewNode::Ptr first(new ViewNode(cur_pos, cur_yaw[0]));
  first->vel_ = cur_vel;
  g_search.addNode(first);
  last_group.push_back(first);
  ViewNode::Ptr final_node;

  // Add viewpoints
  std::cout << "Local refine graph size: 1, ";
  for (int i = 0; i < n_points.size(); ++i) {
    // Create nodes for viewpoints of one frontier
    for (int j = 0; j < n_points[i].size(); ++j) {
      ViewNode::Ptr node(new ViewNode(n_points[i][j], n_yaws[i][j]));
      g_search.addNode(node);
      // Connect a node to nodes in last group
      for (auto nd : last_group) g_search.addEdge(nd->id_, node->id_);
      cur_group.push_back(node);

      // Only keep the first viewpoint of the last local frontier
      if (i == n_points.size() - 1) {
        final_node = node;
        break;
      }
    }
    // Store nodes for this group for connecting edges
    std::cout << cur_group.size() << ", ";
    last_group = cur_group;
    cur_group.clear();
  }
  std::cout << "" << std::endl;
  create_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Search optimal sequence
  vector<ViewNode::Ptr> path;
  g_search.DijkstraSearch(first->id_, final_node->id_, path);

  search_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Return searched sequence
  for (int i = 1; i < path.size(); ++i) {
    refined_pts.push_back(path[i]->pos_);
    refined_yaws.push_back(path[i]->yaw_);
  }

  // Extract optimal local tour (for visualization)
  ed_->refined_tour_.clear();
  ed_->refined_tour_.push_back(cur_pos);
  ViewNode::astar_->lambda_heu_ = 1.0;
  ViewNode::astar_->setResolution(0.2);
  for (auto pt : refined_pts) {
    vector<Vector3d> path;
    if (ViewNode::searchPath(ed_->refined_tour_.back(), pt, path))
      ed_->refined_tour_.insert(ed_->refined_tour_.end(), path.begin(), path.end());
    else
      ed_->refined_tour_.push_back(pt);
  }
  ViewNode::astar_->lambda_heu_ = 10000;

  parse_time = (ros::Time::now() - t1).toSec();
  // ROS_WARN("create: %lf, search: %lf, parse: %lf", create_time, search_time, parse_time);
}

void FastExplorationManager::allocateGrids(const vector<Eigen::Vector3d>& positions,
    const vector<Eigen::Vector3d>& velocities, const vector<int>& drone_ids,
    const vector<vector<int>>& first_ids, const vector<vector<int>>& second_ids,
    const vector<int>& grid_ids, vector<vector<int>>& assigned_grid_ids) {
  assigned_grid_ids.clear();

  auto idsToString = [](const vector<int>& ids) {
    std::ostringstream oss;
    oss << "[";
    for (int i = 0; i < static_cast<int>(ids.size()); ++i) {
      if (i != 0) oss << ",";
      oss << ids[i];
    }
    oss << "]";
    return oss.str();
  };
  auto assignmentsToString = [&](const vector<int>& ids,
                                 const vector<vector<int>>& assignments) {
    std::ostringstream oss;
    oss << "{";
    for (int i = 0; i < static_cast<int>(ids.size()); ++i) {
      if (i != 0) oss << ", ";
      oss << ids[i] << ":";
      if (i < static_cast<int>(assignments.size())) {
        oss << idsToString(assignments[i]);
      } else {
        oss << "[]";
      }
    }
    oss << "}";
    return oss.str();
  };
  auto demandMapToString = [](const vector<int>& grid_ids, const vector<int>& unknown_nums,
                              const vector<int>& demands) {
    const int count = std::min(static_cast<int>(grid_ids.size()),
        std::min(static_cast<int>(unknown_nums.size()), static_cast<int>(demands.size())));
    std::ostringstream oss;
    oss << "[";
    for (int i = 0; i < count; ++i) {
      if (i != 0) oss << ", ";
      oss << grid_ids[i] << ":" << unknown_nums[i] << "->" << demands[i];
    }
    oss << "]";
    return oss.str();
  };
  auto demandHistogramToString = [](const vector<int>& demands) {
    vector<int> nonzero_demands;
    nonzero_demands.reserve(demands.size());
    for (const int demand : demands) {
      if (demand > 0) nonzero_demands.push_back(demand);
    }
    if (nonzero_demands.empty()) {
      return std::string("[]");
    }

    std::sort(nonzero_demands.begin(), nonzero_demands.end());
    std::ostringstream oss;
    oss << "[";
    int current = nonzero_demands.front();
    int count = 0;
    for (const int demand : nonzero_demands) {
      if (demand == current) {
        ++count;
        continue;
      }
      oss << current << "x" << count << ", ";
      current = demand;
      count = 1;
    }
    oss << current << "x" << count << "]";
    return oss.str();
  };

  if (positions.empty() || velocities.size() != positions.size() ||
      drone_ids.size() != positions.size()) {
    ROS_WARN_STREAM("[ALLOC]: invalid inputs positions=" << positions.size()
                    << ", velocities=" << velocities.size() << ", drone_ids="
                    << drone_ids.size() << ", active_grids=" << idsToString(grid_ids));
    return;
  }

  const int drone_num = positions.size();
  assigned_grid_ids.resize(drone_num);

  vector<vector<int>> normalized_first_ids(drone_num), normalized_second_ids(drone_num);
  for (int i = 0; i < drone_num; ++i) {
    if (i < static_cast<int>(first_ids.size())) normalized_first_ids[i] = first_ids[i];
    if (i < static_cast<int>(second_ids.size())) normalized_second_ids[i] = second_ids[i];
  }
  if (first_ids.size() != positions.size() || second_ids.size() != positions.size()) {
    ROS_WARN_STREAM("[ALLOC]: padded frontier hints first_ids=" << first_ids.size()
                    << ", second_ids=" << second_ids.size() << ", drones=" << drone_num);
  }

  ROS_INFO_STREAM("[ALLOC]: input drones=" << idsToString(drone_ids)
                  << ", active_grids=" << idsToString(grid_ids));

  if (grid_ids.empty()) {
    ROS_WARN_STREAM("[ALLOC]: skip allocation because active_grids is empty for drones="
                    << idsToString(drone_ids));
    return;
  }

  auto logAssignments = [&](const std::string& tag, const double matrix_sec,
                            const double solve_sec) {
    ROS_INFO_STREAM("[ALLOC]: " << tag << " result "
                    << assignmentsToString(drone_ids, assigned_grid_ids));
    for (int i = 0; i < drone_num; ++i) {
      ROS_INFO_STREAM("[ALLOC]: drone " << drone_ids[i] << " <- "
                      << idsToString(assigned_grid_ids[i]));
    }
    if (matrix_sec >= 0.0 || solve_sec >= 0.0) {
      ROS_INFO_STREAM("[ALLOC]: " << tag << " matrix_sec=" << matrix_sec
                      << ", solve_sec=" << solve_sec);
    }
  };

  auto assignGridByBestCost = [&](const int grid_id, const std::string& tag) {
    int best_drone_idx = -1;
    double best_cost = std::numeric_limits<double>::infinity();
    for (int i = 0; i < drone_num; ++i) {
      const double cost = hgrid_->getCostDroneToGrid(
          positions[i], velocities[i], grid_id, normalized_first_ids[i]);
      if (cost + 1e-6 < best_cost ||
          (std::fabs(cost - best_cost) <= 1e-6 &&
              (best_drone_idx < 0 || drone_ids[i] < drone_ids[best_drone_idx]))) {
        best_cost = cost;
        best_drone_idx = i;
      }
    }
    if (best_drone_idx >= 0) {
      assigned_grid_ids[best_drone_idx].push_back(grid_id);
      ROS_INFO_STREAM("[ALLOC]: " << tag << " grid=" << grid_id << " -> drone "
                      << drone_ids[best_drone_idx] << " (slot=" << best_drone_idx
                      << ", cost=" << best_cost << ")");
    }
  };

  auto t1 = ros::Time::now();

  if (grid_ids.size() == 1) {
    ROS_INFO_STREAM("[ALLOC]: dispatch shortcut=single-grid-best-cost, drone_num=" << drone_num
                    << ", grid_num=" << grid_ids.size() << ", prob_type=n/a, TYPE=n/a"
                    << ", request_prob=n/a, client=direct-best-cost, service=n/a"
                    << ", backend=no-solver, par_file=n/a, active_grids="
                    << idsToString(grid_ids));
    assignGridByBestCost(grid_ids.front(), "single-grid shortcut");
    logAssignments("single-grid shortcut", 0.0, 0.0);
    return;
  }

  if (drone_num > 1 && grid_ids.size() < 3) {
    ROS_INFO_STREAM("[ALLOC]: dispatch shortcut=small-grid-best-cost, drone_num=" << drone_num
                    << ", grid_num=" << grid_ids.size() << ", prob_type=n/a, TYPE=n/a"
                    << ", request_prob=n/a, client=direct-best-cost, service=n/a"
                    << ", backend=no-solver, par_file=n/a, active_grids="
                    << idsToString(grid_ids));
    for (const int grid_id : grid_ids) {
      assignGridByBestCost(grid_id, "small-grid shortcut");
    }
    logAssignments("small-grid shortcut", 0.0, 0.0);
    return;
  }

  Eigen::MatrixXd mat;
  hgrid_->getCostMatrix(
      positions, velocities, normalized_first_ids, normalized_second_ids, grid_ids, mat);
  const double mat_time = (ros::Time::now() - t1).toSec();

  t1 = ros::Time::now();
  const int dimension = mat.rows();

  vector<int> unknown_nums;
  unknown_nums.reserve(grid_ids.size());
  int max_unknown = 0;
  for (int i = 0; i < static_cast<int>(grid_ids.size()); ++i) {
    const int unum = std::max(0, hgrid_->getUnknownCellsNum(grid_ids[i]));
    unknown_nums.push_back(unum);
    max_unknown = std::max(max_unknown, unum);
  }
  const AllocationDemandModel demand_model = buildAllocationDemandModel(unknown_nums);
  const int total_unknown = demand_model.total_unknown;
  const vector<int>& demands = demand_model.demands;
  const int total_demand = demand_model.total_demand;
  const int max_demand = demand_model.max_demand;
  const int capacity_from_average = static_cast<int>(std::ceil(static_cast<double>(total_demand) /
                                 static_cast<double>(std::max(1, drone_num))));
  const int capacity = std::max(1, std::max(max_demand, capacity_from_average));
  const int total_capacity = capacity * drone_num;
  const AllocationSolverDispatch dispatch = selectAllocationSolverDispatch(drone_num);
  const string solver_file_prefix =
      ep_->mtsp_dir_ + "/" + string(dispatch.file_stem) + "_" + to_string(ep_->drone_id_);
  const string solver_problem_file = solver_file_prefix + ".atsp";
  const string solver_par_file = solver_file_prefix + ".par";
  const string solver_tour_file = solver_file_prefix + ".tour";
  const string solver_service_name =
      string(dispatch.service_name_prefix) + to_string(ep_->drone_id_);

  ROS_INFO_STREAM("[ALLOC]: dispatch mode=" << dispatch.dispatch_mode << ", drone_num="
                  << drone_num << ", grid_num=" << grid_ids.size() << ", prob_type="
                  << dispatch.prob_type << " (" << allocationProbTypeLabel(dispatch.prob_type)
                  << "), TYPE=" << dispatch.type_name << ", active_grids="
                  << idsToString(grid_ids) << ", request_prob=" << dispatch.request_prob << " ("
                  << allocationRequestProbLabel(dispatch.request_prob) << "), client="
                  << dispatch.client_name << ", service=" << solver_service_name
                  << ", backend=" << dispatch.backend_name << ", problem_file="
                  << solver_problem_file << ", par_file=" << solver_par_file
                  << ", tour_file=" << solver_tour_file << ", total_unknown="
                  << total_unknown << ", max_unknown=" << max_unknown
                  << ", target_total_demand=" << demand_model.target_total_demand
                  << ", total_demand=" << total_demand << ", capacity=" << capacity
                  << ", total_capacity=" << total_capacity
                  << ", demand_scaled=" << (demand_model.scaled ? "true" : "false"));

  if (dispatch.prob_type == 2) {
    const double avg_unknown_per_nonempty = demand_model.positive_grid_num > 0
        ? static_cast<double>(total_unknown) / static_cast<double>(demand_model.positive_grid_num)
        : 0.0;
    const double avg_demand_per_nonempty = demand_model.positive_grid_num > 0
        ? static_cast<double>(total_demand) / static_cast<double>(demand_model.positive_grid_num)
        : 0.0;
    const double demand_load_factor = total_capacity > 0
        ? static_cast<double>(total_demand) / static_cast<double>(total_capacity)
        : 0.0;
    ROS_INFO_STREAM("[ALLOC]: ACVRP demand stats total_unknown=" << total_unknown
                    << ", target_total_demand=" << demand_model.target_total_demand
                    << ", total_demand=" << total_demand
                    << ", capacity_per_vehicle=" << capacity
                    << ", total_capacity=" << total_capacity
                    << ", nonempty_grids=" << demand_model.positive_grid_num
                    << ", zero_grids=" << demand_model.zero_grid_num
                    << ", min_nonzero_demand=" << demand_model.min_nonzero_demand
                    << ", max_demand=" << max_demand
                    << ", avg_unknown_per_nonempty=" << avg_unknown_per_nonempty
                    << ", avg_demand_per_nonempty=" << avg_demand_per_nonempty
                    << ", scale_ratio=" << demand_model.scale_ratio
                    << ", load_factor=" << demand_load_factor
                    << ", demand_histogram=" << demandHistogramToString(demands));
    ROS_INFO_STREAM("[ALLOC]: ACVRP demand mapping "
                    << demandMapToString(grid_ids, unknown_nums, demands));
  }

  ofstream file(solver_problem_file);
  file << "NAME : pairopt\n";
  file << "TYPE : " << dispatch.type_name << "\n";
  file << "DIMENSION : " + to_string(dimension) + "\n";
  file << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
  file << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";

  if (dispatch.prob_type == 2) {
    file << "CAPACITY : " + to_string(capacity) + "\n";
    file << "VEHICLES : " + to_string(drone_num) + "\n";
  }

  file << "EDGE_WEIGHT_SECTION\n";
  for (int i = 0; i < dimension; ++i) {
    for (int j = 0; j < dimension; ++j) {
      const int int_cost = static_cast<int>(std::lround(100.0 * mat(i, j)));
      file << int_cost << " ";
    }
    file << "\n";
  }

  if (dispatch.prob_type == 2) {
    file << "DEMAND_SECTION\n";
    file << "1 0\n";
    for (int i = 0; i < drone_num; ++i) {
      file << to_string(i + 2) + " 0\n";
    }
    for (int i = 0; i < static_cast<int>(grid_ids.size()); ++i) {
      file << to_string(i + 2 + drone_num) + " " + to_string(demands[i]) + "\n";
    }
    file << "DEPOT_SECTION\n";
    file << "1\n";
    file << "EOF";
  }

  file.close();

  file.open(solver_par_file);
  file << "SPECIAL\n";
  file << "PROBLEM_FILE = " << solver_problem_file << "\n";
  if (dispatch.prob_type == 1) {
    file << "SALESMEN = " << to_string(drone_num) << "\n";
    file << "MTSP_OBJECTIVE = MINSUM\n";
    file << "TRACE_LEVEL = 0\n";
  } else if (dispatch.prob_type == 2) {
    file << "TRACE_LEVEL = 1\n";
    file << "SEED = 0\n";
  }
  file << "RUNS = 1\n";
  file << "TOUR_FILE = " << solver_tour_file << "\n";

  file.close();

  t1 = ros::Time::now();

  lkh_mtsp_solver::SolveMTSP srv;
  srv.request.prob = dispatch.request_prob;
  const bool solver_ok =
      dispatch.use_acvrp_client ? acvrp_client_.call(srv) : tsp_client_.call(srv);
  if (!solver_ok) {
    ROS_ERROR_STREAM("[ALLOC]: solver dispatch failed mode=" << dispatch.dispatch_mode
                     << ", drone_num=" << drone_num << ", grid_num=" << grid_ids.size()
                     << ", prob_type=" << dispatch.prob_type << " ("
                     << allocationProbTypeLabel(dispatch.prob_type) << "), TYPE="
                     << dispatch.type_name << ", request_prob=" << dispatch.request_prob << " ("
                     << allocationRequestProbLabel(dispatch.request_prob) << "), client="
                     << dispatch.client_name << ", service=" << solver_service_name
                     << ", backend=" << dispatch.backend_name << ", par_file="
                     << solver_par_file);
    return;
  }

  const double mtsp_time = (ros::Time::now() - t1).toSec();
  ROS_INFO_STREAM("[ALLOC]: solver call completed mode=" << dispatch.dispatch_mode
                  << ", drone_num=" << drone_num << ", grid_num=" << grid_ids.size()
                  << ", prob_type=" << dispatch.prob_type << " ("
                  << allocationProbTypeLabel(dispatch.prob_type) << "), TYPE="
                  << dispatch.type_name << ", request_prob=" << dispatch.request_prob << " ("
                  << allocationRequestProbLabel(dispatch.request_prob) << "), client="
                  << dispatch.client_name << ", service=" << solver_service_name
                  << ", backend=" << dispatch.backend_name << ", par_file="
                  << solver_par_file << ", solve_sec=" << mtsp_time);
  std::cout << "Allocation time: " << mtsp_time << std::endl;

  ifstream fin(solver_tour_file);
  string res;
  vector<int> ids;
  while (getline(fin, res)) {
    if (res.compare("TOUR_SECTION") == 0) break;
  }
  while (getline(fin, res)) {
    int id = stoi(res);
    ids.push_back(id - 1);
    if (id == -1) break;
  }
  fin.close();

  vector<vector<int>> tours;
  vector<int> tour;
  for (const int id : ids) {
    if (id > 0 && id <= drone_num) {
      if (!tour.empty()) {
        tours.push_back(tour);
      }
      tour.clear();
      tour.push_back(id);
    } else if (id >= dimension || id <= 0) {
      if (!tour.empty()) {
        tours.push_back(tour);
        tour.clear();
      }
    } else if (!tour.empty()) {
      tour.push_back(id);
    }
  }
  if (!tour.empty()) {
    tours.push_back(tour);
  }

  int assigned_count = 0;
  for (const auto& parsed_tour : tours) {
    if (parsed_tour.empty()) continue;
    const int drone_slot = parsed_tour.front() - 1;
    if (drone_slot < 0 || drone_slot >= drone_num) continue;
    for (int i = 1; i < static_cast<int>(parsed_tour.size()); ++i) {
      const int grid_slot = parsed_tour[i] - 1 - drone_num;
      if (grid_slot < 0 || grid_slot >= static_cast<int>(grid_ids.size())) {
        ROS_WARN_STREAM("[ALLOC]: skip invalid grid slot " << grid_slot
                        << " while parsing tour for drone " << drone_ids[drone_slot]);
        continue;
      }
      assigned_grid_ids[drone_slot].push_back(grid_ids[grid_slot]);
      ++assigned_count;
    }
  }

  if (assigned_count != static_cast<int>(grid_ids.size())) {
    ROS_WARN_STREAM("[ALLOC]: parsed " << assigned_count << " assigned grids from "
                    << grid_ids.size() << " active grids. Result="
                    << assignmentsToString(drone_ids, assigned_grid_ids));
  }

  logAssignments("solver", mat_time, mtsp_time);
}

double FastExplorationManager::computeGridPathCost(const Eigen::Vector3d& pos,
    const Eigen::Vector3d& vel, const vector<int>& grid_ids, const vector<int>& first,
    const vector<vector<int>>& firsts,
    const vector<vector<int>>& seconds, const double& w_f) {
  if (grid_ids.empty()) return 0.0;

  double cost = 0.0;
  vector<Eigen::Vector3d> path;
  cost += hgrid_->getCostDroneToGrid(pos, vel, grid_ids[0], first);
  for (int i = 0; i < grid_ids.size() - 1; ++i) {
    cost += hgrid_->getCostGridToGrid(grid_ids[i], grid_ids[i + 1], firsts, seconds, firsts.size());
  }
  return cost;
}

bool FastExplorationManager::findGlobalTourOfGrid(const vector<Eigen::Vector3d>& positions,
    const vector<Eigen::Vector3d>& velocities, const vector<int>& drone_ids, vector<int>& indices,
    vector<vector<int>>& others, bool init) {

  ROS_INFO("Find grid tour---------------");

  auto t1 = ros::Time::now();

  indices.clear();
  others.clear();
  if (positions.empty() || velocities.size() != positions.size() ||
      drone_ids.size() != positions.size()) {
    ROS_WARN_STREAM("[GLOBAL_ALLOC]: invalid swarm inputs positions=" << positions.size()
                    << ", velocities=" << velocities.size() << ", drone_ids="
                    << drone_ids.size());
    return false;
  }

  vector<int> self_grid_ids = ed_->swarm_state_[ep_->drone_id_ - 1].grid_ids_;
  if (!ed_->pending_release_grid_ids_.empty()) {
    std::unordered_set<int> pending_release_ids(
        ed_->pending_release_grid_ids_.begin(), ed_->pending_release_grid_ids_.end());
    vector<int> filtered_grid_ids;
    filtered_grid_ids.reserve(self_grid_ids.size());
    for (const int grid_id : self_grid_ids) {
      if (pending_release_ids.find(grid_id) == pending_release_ids.end()) {
        filtered_grid_ids.push_back(grid_id);
      }
    }
    self_grid_ids.swap(filtered_grid_ids);
  }

  // hgrid_->updateBaseCoor();  // Use the latest basecoor transform of swarm

  vector<int> first_ids, second_ids;
  vector<int> frontier_cell_nums;
  frontier_cell_nums.reserve(ed_->frontiers_.size());
  for (const auto& frontier : ed_->frontiers_) {
    frontier_cell_nums.push_back(static_cast<int>(frontier.size()));
  }
  hgrid_->inputFrontiers(ed_->averages_, frontier_cell_nums);

  hgrid_->updateGridData(ep_->drone_id_, self_grid_ids, ed_->reallocated_, ed_->last_grid_ids_,
      first_ids, second_ids);
  publishTaskMetrics();

  vector<int> active_grid_ids;
  hgrid_->getActiveGrids(active_grid_ids);
  if (!ed_->pending_release_grid_ids_.empty()) {
    std::unordered_set<int> pending_release_ids(
        ed_->pending_release_grid_ids_.begin(), ed_->pending_release_grid_ids_.end());
    vector<int> filtered_active_grid_ids;
    filtered_active_grid_ids.reserve(active_grid_ids.size());
    for (const int grid_id : active_grid_ids) {
      if (pending_release_ids.find(grid_id) == pending_release_ids.end()) {
        filtered_active_grid_ids.push_back(grid_id);
      }
    }
    active_grid_ids.swap(filtered_active_grid_ids);
  }

  if (active_grid_ids.empty()) {
    ROS_WARN("[GLOBAL_ALLOC]: no active grids available for global allocation.");
    ed_->grid_tour_.clear();
    ed_->grid_tour2_.clear();
    return false;
  }

  vector<vector<int>> all_first_ids, all_second_ids;
  all_first_ids.reserve(drone_ids.size());
  all_second_ids.reserve(drone_ids.size());
  if (!init) {
    all_first_ids.push_back(first_ids);
    all_second_ids.push_back(second_ids);
    for (int i = 1; i < static_cast<int>(drone_ids.size()); ++i) {
      const int drone_index = drone_ids[i] - 1;
      vector<int> peer_grid_ids;
      if (drone_index >= 0 && drone_index < static_cast<int>(ed_->observed_peer_grid_ids_.size()) &&
          !ed_->observed_peer_grid_ids_[drone_index].empty()) {
        peer_grid_ids = ed_->observed_peer_grid_ids_[drone_index];
      } else if (drone_index >= 0 && drone_index < static_cast<int>(ed_->swarm_state_.size())) {
        peer_grid_ids = ed_->swarm_state_[drone_index].grid_ids_;
      }

      vector<int> peer_first_ids, peer_second_ids;
      if (!peer_grid_ids.empty()) {
        hgrid_->getConsistentGrid(
            peer_grid_ids, active_grid_ids, peer_first_ids, peer_second_ids);
      }
      all_first_ids.push_back(peer_first_ids);
      all_second_ids.push_back(peer_second_ids);
    }
  } else {
    all_first_ids.assign(drone_ids.size(), vector<int>());
    all_second_ids.assign(drone_ids.size(), vector<int>());
  }

  std::ostringstream drone_stream;
  for (int i = 0; i < static_cast<int>(drone_ids.size()); ++i) {
    if (i != 0) {
      drone_stream << ",";
    }
    drone_stream << drone_ids[i];
  }
  ROS_INFO_STREAM("[GLOBAL_ALLOC]: solving with drones=" << drone_ids.size() << " ids=["
                  << drone_stream.str() << "], active_grids=" << active_grid_ids.size()
                  << ", self_prev_grids=" << self_grid_ids.size() << ", init=" << init);

  Eigen::MatrixXd mat;
  hgrid_->getCostMatrix(positions, velocities, all_first_ids, all_second_ids, active_grid_ids, mat);

  double mat_time = (ros::Time::now() - t1).toSec();

  // Find optimal path through ATSP
  t1 = ros::Time::now();
  const int dimension = mat.rows();
  const int drone_num = positions.size();

  // Create problem file
  ofstream file(ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".atsp");
  file << "NAME : amtsp\n";
  file << "TYPE : ATSP\n";
  file << "DIMENSION : " + to_string(dimension) + "\n";
  file << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
  file << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";
  file << "EDGE_WEIGHT_SECTION\n";
  for (int i = 0; i < dimension; ++i) {
    for (int j = 0; j < dimension; ++j) {
      int int_cost = 100 * mat(i, j);
      file << int_cost << " ";
    }
    file << "\n";
  }
  file.close();

  // Create par file
  file.open(ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".par");
  file << "SPECIAL\n";
  file << "PROBLEM_FILE = " + ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".atsp\n";
  file << "SALESMEN = " << to_string(drone_num) << "\n";
  file << "MTSP_OBJECTIVE = MINSUM\n";
  // file << "MTSP_MIN_SIZE = " << to_string(min(int(ed_->frontiers_.size()) / drone_num, 4)) <<
  // "\n"; file << "MTSP_MAX_SIZE = "
  //      << to_string(max(1, int(ed_->frontiers_.size()) / max(1, drone_num - 1))) << "\n";
  file << "RUNS = 1\n";
  file << "TRACE_LEVEL = 0\n";
  file << "TOUR_FILE = " + ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".tour\n";
  file.close();

  auto par_dir = ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".atsp";
  t1 = ros::Time::now();

  lkh_mtsp_solver::SolveMTSP srv;
  srv.request.prob = 2;
  if (!tsp_client_.call(srv)) {
    ROS_ERROR("Fail to solve ATSP.");
    return false;
  }

  double mtsp_time = (ros::Time::now() - t1).toSec();

  // Read results
  t1 = ros::Time::now();

  ifstream fin(ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".tour");
  string res;
  vector<int> ids;
  while (getline(fin, res)) {
    if (res.compare("TOUR_SECTION") == 0) break;
  }
  while (getline(fin, res)) {
    int id = stoi(res);
    ids.push_back(id - 1);
    if (id == -1) break;
  }
  fin.close();

  // Parse the m-tour of grid
  vector<vector<int>> tours;
  vector<int> tour;
  for (auto id : ids) {
    if (id > 0 && id <= drone_num) {
      tour.clear();
      tour.push_back(id);
    } else if (id >= dimension || id <= 0) {
      tours.push_back(tour);
    } else {
      tour.push_back(id);
    }
  }

  others.resize(std::max(0, drone_num - 1));
  for (int i = 1; i < static_cast<int>(tours.size()); ++i) {
    if (tours[i].empty()) {
      continue;
    }
    if (tours[i][0] == 1) {
      indices.insert(indices.end(), tours[i].begin() + 1, tours[i].end());
    } else if (tours[i][0] > 1 && tours[i][0] <= drone_num) {
      others[tours[i][0] - 2].insert(
          others[tours[i][0] - 2].end(), tours[i].begin() + 1, tours[i].end());
    }
  }
  for (auto& id : indices) {
    id -= 1 + drone_num;
  }
  for (auto& other : others) {
    for (auto& id : other) {
      id -= 1 + drone_num;
    }
  }

  std::ostringstream self_stream;
  std::cout << "Grid tour: ";
  for (int i = 0; i < static_cast<int>(indices.size()); ++i) {
    indices[i] = active_grid_ids[indices[i]];
    if (i != 0) {
      self_stream << ",";
    }
    self_stream << indices[i];
    std::cout << indices[i] << ", ";
  }
  std::cout << "" << std::endl;
  ROS_INFO_STREAM("[GLOBAL_ALLOC]: result drone " << ep_->drone_id_ << " <- ["
                  << self_stream.str() << "]");

  for (int i = 0; i < static_cast<int>(others.size()); ++i) {
    std::ostringstream peer_stream;
    for (int j = 0; j < static_cast<int>(others[i].size()); ++j) {
      others[i][j] = active_grid_ids[others[i][j]];
      if (j != 0) {
        peer_stream << ",";
      }
      peer_stream << others[i][j];
    }
    ROS_INFO_STREAM("[GLOBAL_ALLOC]: result drone " << drone_ids[i + 1] << " <- ["
                    << peer_stream.str() << "]");
  }

  ed_->pending_grid_ids_ = indices;
  if (!indices.empty()) {
    hgrid_->getGridTour(indices, positions[0], ed_->grid_tour_, ed_->grid_tour2_);
  } else {
    ed_->grid_tour_.clear();
    ed_->grid_tour2_.clear();
  }
  ROS_WARN_STREAM("[GLOBAL_ALLOC]: drone " << ep_->drone_id_ << " staged planner candidate with "
                  << indices.size() << " self grids from " << active_grid_ids.size()
                  << " active grids across " << drone_num << " drones.");
  ROS_INFO_STREAM("[GLOBAL_ALLOC]: matrix_sec=" << mat_time << ", solve_sec=" << mtsp_time);

  return true;
}

void FastExplorationManager::findTourOfFrontier(const Vector3d& cur_pos, const Vector3d& cur_vel,
    const Vector3d& cur_yaw, const vector<int>& ftr_ids, const vector<Eigen::Vector3d>& grid_pos,
    vector<int>& indices) {

  auto t1 = ros::Time::now();

  vector<Eigen::Vector3d> positions = { cur_pos };
  vector<Eigen::Vector3d> velocities = { cur_vel };
  vector<double> yaws = { cur_yaw[0] };

  // frontier_finder_->getSwarmCostMatrix(positions, velocities, yaws, mat);
  Eigen::MatrixXd mat;
  frontier_finder_->getSwarmCostMatrix(positions, velocities, yaws, ftr_ids, grid_pos, mat);
  const int dimension = mat.rows();
  // std::cout << "dim of frontier TSP mat: " << dimension << std::endl;

  double mat_time = (ros::Time::now() - t1).toSec();
  // ROS_INFO("mat time: %lf", mat_time);

  // Find optimal allocation through AmTSP
  t1 = ros::Time::now();

  // Create problem file
  ofstream file(ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".atsp");
  file << "NAME : amtsp\n";
  file << "TYPE : ATSP\n";
  file << "DIMENSION : " + to_string(dimension) + "\n";
  file << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
  file << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";
  file << "EDGE_WEIGHT_SECTION\n";
  for (int i = 0; i < dimension; ++i) {
    for (int j = 0; j < dimension; ++j) {
      int int_cost = 100 * mat(i, j);
      file << int_cost << " ";
    }
    file << "\n";
  }
  file.close();

  // Create par file
  const int drone_num = 1;

  file.open(ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".par");
  file << "SPECIAL\n";
  file << "PROBLEM_FILE = " + ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".atsp\n";
  file << "SALESMEN = " << to_string(drone_num) << "\n";
  file << "MTSP_OBJECTIVE = MINSUM\n";
  file << "MTSP_MIN_SIZE = " << to_string(min(int(ed_->frontiers_.size()) / drone_num, 4)) << "\n";
  file << "MTSP_MAX_SIZE = "
       << to_string(max(1, int(ed_->frontiers_.size()) / max(1, drone_num - 1))) << "\n";
  file << "RUNS = 1\n";
  file << "TRACE_LEVEL = 0\n";
  file << "TOUR_FILE = " + ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".tour\n";
  file.close();

  auto par_dir = ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".atsp";
  t1 = ros::Time::now();

  lkh_mtsp_solver::SolveMTSP srv;
  srv.request.prob = 1;
  if (!tsp_client_.call(srv)) {
    ROS_ERROR("Fail to solve ATSP.");
    return;
  }

  double mtsp_time = (ros::Time::now() - t1).toSec();
  // ROS_INFO("AmTSP time: %lf", mtsp_time);

  // Read results
  t1 = ros::Time::now();

  ifstream fin(ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".tour");
  string res;
  vector<int> ids;
  while (getline(fin, res)) {
    if (res.compare("TOUR_SECTION") == 0) break;
  }
  while (getline(fin, res)) {
    int id = stoi(res);
    ids.push_back(id - 1);
    if (id == -1) break;
  }
  fin.close();

  // Parse the m-tour
  vector<vector<int>> tours;
  vector<int> tour;
  for (auto id : ids) {
    if (id > 0 && id <= drone_num) {
      tour.clear();
      tour.push_back(id);
    } else if (id >= dimension || id <= 0) {
      tours.push_back(tour);
    } else {
      tour.push_back(id);
    }
  }

  vector<vector<int>> others(drone_num - 1);
  for (int i = 1; i < tours.size(); ++i) {
    if (tours[i][0] == 1) {
      indices.insert(indices.end(), tours[i].begin() + 1, tours[i].end());
    }
    // else {
    //   others[tours[i][0] - 2].insert(
    //       others[tours[i][0] - 2].end(), tours[i].begin() + 1, tours[i].end());
    // }
  }
  for (auto& id : indices) {
    id -= 1 + drone_num;
  }
  // for (auto& other : others) {
  //   for (auto& id : other)
  //     id -= 1 + drone_num;
  // }

  if (ed_->grid_tour_.size() > 2) {  // Remove id for next grid, since it is considered in the TSP
    indices.pop_back();
  }
  // Subset of frontier inside first grid
  for (int i = 0; i < indices.size(); ++i) {
    indices[i] = ftr_ids[indices[i]];
  }

  // Get the path of optimal tour from path matrix
  frontier_finder_->getPathForTour(cur_pos, indices, ed_->frontier_tour_);
  if (!grid_pos.empty()) {
    ed_->frontier_tour_.push_back(grid_pos[0]);
  }

  // ed_->other_tours_.clear();
  // for (int i = 1; i < positions.size(); ++i) {
  //   ed_->other_tours_.push_back({});
  //   frontier_finder_->getPathForTour(positions[i], others[i - 1], ed_->other_tours_[i - 1]);
  // }

  double parse_time = (ros::Time::now() - t1).toSec();
  // ROS_INFO("Cost mat: %lf, TSP: %lf, parse: %f, %d frontiers assigned.", mat_time, mtsp_time,
  //     parse_time, indices.size());
}

}  // namespace fast_planner
