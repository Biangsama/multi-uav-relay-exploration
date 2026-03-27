# multi-uav-relay-exploration

Source snapshot of a ROS Noetic / Catkin `src` tree for multi-UAV exploration with relay-assisted communication, task reassignment, and evaluation tooling.

This repository packages the source-side integration of three main parts:
- a RACER-based swarm exploration stack
- a relay / communication integration layer
- supporting communication-platform packages used by the relay experiments

## Repository Scope

This repository is a **source-only** snapshot prepared from a larger local workspace. It contains the Catkin `src` tree and excludes local build artifacts such as `build/`, `devel/`, logs, editor metadata, and Python cache files.

The repository root is intended to be used as the **`src/` directory of a Catkin workspace**.

## Main Capabilities

- Frontier-based multi-UAV exploration and local trajectory planning
- Relay role selection and relay-target generation for fragmented communication conditions
- Task reassignment through pair-wise reallocation and missed-grid claiming
- Relay-aware monitoring, validation, and RViz overlays
- Real-input launch files for 4-UAV and 6-UAV experiments

## Top-Level Package Overview

- `exploration_manager`: swarm exploration FSM, task allocation, pair-opt, planner integration
- `active_perception`: hierarchical grid, frontier-to-grid mapping, grid relevance logic
- `plan_env`, `plan_manage`, `path_searching`, `bspline`, `bspline_opt`, `traj_utils`, `poly_traj`: planning and trajectory stack
- `relay_racer_integration`: relay controllers, communication bridges, monitoring scripts, RViz overlays, experiment launch files
- `connector_core`, `coordinator`, `mini_dancers`, `ns3_connector`, `dancers_msgs`, `protobuf_msgs`: communication / integration-side packages
- `local_sensing_node`, `map_generator`, `odom_visualization`, `pose_utils`, `poscmd_2_odom`, `quadrotor_msgs`: simulator and sensing-side support packages
- `lkh_tsp_solver`, `lkh_mtsp_solver`: TSP / MTSP / ACVRP related solvers used by allocation and tour planning

## Important Code Entry Points

- `exploration_manager/src/fast_exploration_fsm.cpp`
- `exploration_manager/src/fast_exploration_manager.cpp`
- `active_perception/src/uniform_grid.cpp`
- `active_perception/src/hgrid.cpp`
- `relay_racer_integration/src/racer_local_relay_controller_node.cpp`
- `relay_racer_integration/src/racer_comm_controller_node.cpp`
- `relay_racer_integration/scripts/relay_efficiency_eval.py`
- `relay_racer_integration/scripts/relay_validation_monitor.py`
- `relay_racer_integration/scripts/relay_rviz_overlay.py`

## Launch Files

Main launch files are under `relay_racer_integration/launch/`.

Common entry points:
- `relay_real_input_6planner_min.launch`: 6-UAV relay/exploration experiment
- `relay_real_input_4planner_min.launch`: 4-UAV relay/exploration experiment
- `relay_real_input_min.launch`: smaller real-input setup
- `relay_rviz.launch`: relay visualization overlay in RViz
- `relay_closed_loop_min.launch`: closed-loop relay experiment
- `racer_platform_min.launch`: minimal platform launch

## Scripts

Main scripts are under `relay_racer_integration/scripts/`.

Important utilities:
- `relay_efficiency_eval.py`: batch evaluation for `no_relay` / `rule_relay` experiments
- `relay_validation_monitor.py`: runtime/final validation and report generation
- `relay_rviz_overlay.py`: RViz markers for relay role, bridge components, and target visualization
- `racer_real_odom_bridge.py`: real-odom bridging utility
- `planner_sensor_gate.py`: planner/sensor gating helper

## Environment Requirements

Recommended baseline:
- Ubuntu 20.04
- ROS Noetic
- Catkin workspace tooling (`catkin_make` or equivalent)
- Standard ROS message packages used in the package manifests
- OpenCV / `cv_bridge`
- RViz
- Protobuf support used by the integration layer

Because this repository is a source snapshot from a larger experiment workspace, you may need to install additional ROS dependencies depending on which launch files or packages you actually build and run.

## How To Use This Repository

### Option 1: Clone directly as the `src` directory of a Catkin workspace

```bash
mkdir -p ~/multi_uav_ws
cd ~/multi_uav_ws
git clone https://github.com/Biangsama/multi-uav-relay-exploration.git src
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

### Option 2: Replace an existing workspace `src` tree

If you already have a Catkin workspace, replace or merge its `src/` directory with this repository content, then rebuild from the workspace root:

```bash
cd <your_catkin_ws>
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

## Minimal Run Examples

Build selected packages:

```bash
cd <your_catkin_ws>
catkin_make --pkg exploration_manager relay_racer_integration -j1
source devel/setup.bash
```

Run a 6-UAV relay experiment:

```bash
roslaunch relay_racer_integration relay_real_input_6planner_min.launch
```

Run relay RViz overlay:

```bash
roslaunch relay_racer_integration relay_rviz.launch
```

Run a batch comparison:

```bash
python3 src/relay_racer_integration/scripts/relay_efficiency_eval.py \
  --modes no_relay rule_relay \
  --mainline-launch relay_real_input_6planner_min.launch \
  --output-dir /tmp/relay_efficiency_eval
```

## Notes On Current Research Focus

This codebase has been used to study:
- relay-assisted coordination under fragmented communication
- exploration task handoff after role switching
- repeated-frontier / rediscovery suppression
- relay-aware monitoring and visualization
- comparisons between `no_relay` and `rule_relay` modes

## Practical Notes

- This repository is not a fully packaged binary release; it is a research-code source tree.
- Some package manifests still contain placeholder metadata inherited from upstream code.
- If you plan to make the repository public, review licenses and maintainer metadata before broader release.
- If you only need a subset of the system, `exploration_manager`, `active_perception`, and `relay_racer_integration` are the main packages to start from.

## Maintainer

Prepared from a local research workspace snapshot for GitHub publication.
