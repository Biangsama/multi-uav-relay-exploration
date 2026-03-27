#!/usr/bin/env python3

import os
import re
import math
from collections import defaultdict
from functools import partial

import rospy
from bspline.msg import Bspline
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from relay_racer_integration.msg import RelayRoleCmd, SwarmCommState
from std_msgs.msg import Empty, Int32MultiArray
from visualization_msgs.msg import Marker


ROLE_NAMES = {
    RelayRoleCmd.ROLE_EXPLORER: "ROLE_EXPLORER",
    RelayRoleCmd.ROLE_RELAY: "ROLE_RELAY",
}


class RelayValidationMonitor:
    def __init__(self):
        self.agent_id = int(rospy.get_param("~agent_id", 2))
        self.relay_role_topic = rospy.get_param("~relay_role_topic", "/relay_integration/relay_role_cmd")
        self.swarm_comm_state_topic = rospy.get_param(
            "~swarm_comm_state_topic", "/relay_integration/swarm_comm_state"
        )
        self.task_metrics_topic = rospy.get_param("~task_metrics_topic", "/relay_integration/task_metrics")
        self.frontier_topic = rospy.get_param("~frontier_topic", "/planning_vis/frontier_2")
        self.report_path = rospy.get_param("~report_path", "")
        self.write_runtime_report = bool(rospy.get_param("~write_runtime_report", True))
        self.events_path = rospy.get_param("~events_path", "")
        self.summary_period = float(rospy.get_param("~summary_period", 2.0))
        self.metric_sample_period = max(0.1, float(rospy.get_param("~metric_sample_period", 0.5)))
        self.pre_window_sec = max(0.0, float(rospy.get_param("~pre_window_sec", 5.0)))
        self.post_window_sec = max(0.0, float(rospy.get_param("~post_window_sec", 5.0)))
        self.require_pos_cmd = bool(rospy.get_param("~require_pos_cmd", True))
        self.frontier_completion_threshold = int(rospy.get_param("~frontier_completion_threshold", -1))
        self.completion_confirm_sec = max(0.0, float(rospy.get_param("~completion_confirm_sec", 5.0)))
        self.completion_frontier_cluster_threshold = int(
            rospy.get_param("~completion_frontier_cluster_threshold", 0)
        )
        self.completion_active_grid_threshold = int(
            rospy.get_param("~completion_active_grid_threshold", -1)
        )
        self.planning_agent_ids = self._normalize_int_list(
            rospy.get_param("~planning_agent_ids", [self.agent_id]), default=[self.agent_id]
        )
        self.frontier_topics_by_agent = self._resolve_frontier_topics(
            self.frontier_topic, self.planning_agent_ids, self.agent_id
        )
        self.odom_topic_prefix = rospy.get_param("~odom_topic_prefix", "/relay_odom_")
        self.odom_agent_ids = self._normalize_int_list(
            rospy.get_param("~odom_agent_ids", self.planning_agent_ids), default=self.planning_agent_ids
        )
        self.odom_max_gap_sec = max(0.0, float(rospy.get_param("~odom_max_gap_sec", 2.0)))
        self.low_speed_threshold = max(0.0, float(rospy.get_param("~low_speed_threshold", 0.05)))
        self.task_metrics_agent_timeout_sec = max(
            0.5,
            float(
                rospy.get_param(
                    "~task_metrics_agent_timeout_sec",
                    max(2.0, self.completion_confirm_sec + self.summary_period, 4.0 * self.metric_sample_period),
                )
            ),
        )

        if not self.events_path and self.report_path:
            report_root, _ = os.path.splitext(self.report_path)
            self.events_path = report_root + ".events.tsv"

        self.task_completed = 0
        self.completion_elapsed_sec = -1.0
        self.runtime_sec = 0.0

        self.team_agent_ids = set()
        self.team_agent_count = 0
        self.comm_fragmentation_observed = False
        self.num_components_peak = 0

        self.relay_role_count = 0
        self.tracked_relay_role_count = 0
        self.planning_new_count = 0
        self.planning_bspline_count = 0
        self.planning_pos_cmd_count = 0
        self.frontier_message_count = 0
        self.frontier_message_count_by_agent = defaultdict(int)
        self.task_metrics_count = 0

        self.role_switch_count = 0
        self.relay_drop_count = 0
        self.success_logged = False

        self.saw_relay_role = False
        self.saw_planning_new = False
        self.saw_planning_bspline = False
        self.saw_planning_pos_cmd = False
        self.saw_frontier = False
        self.saw_task_metrics = False

        self.first_relay_time = ""
        self.first_planning_new_time = ""
        self.first_planning_bspline_time = ""
        self.first_planning_pos_cmd_time = ""
        self.first_frontier_time = ""
        self.first_frontier_positive_time = ""
        self.success_time = ""
        self.completion_time = ""
        self.success_elapsed_sec = None
        self.first_frontier_positive_elapsed_sec = None

        self.last_role = ""
        self.last_valid = 0
        self.last_target = (0.0, 0.0, 0.0)
        self.last_bspline_traj_id = -1
        self.last_bspline_knot_count = 0
        self.last_pos_cmd_traj_id = -1
        self.last_pos_cmd_position = (0.0, 0.0, 0.0)
        self.last_pos_cmd_velocity = (0.0, 0.0, 0.0)
        self.last_num_components = 0
        self.last_information_age = 0.0
        self.max_information_age = 0.0
        self.mean_information_age = 0.0
        self.unknown_cells_total = 0
        self.unknown_cells_peak = 0
        self.unknown_cells_min = -1
        self.frontier_cluster_count_metric = 0
        self.frontier_cell_count_metric = 0
        self.active_grid_count = 0
        self.task_metrics_fresh_agent_count = 0
        self.task_metrics_fresh_agent_ids = set()
        self._task_metrics_by_agent = {}
        self._task_metrics_elapsed_by_agent = {}

        self.frontier_counts_by_agent = defaultdict(dict)
        self.marker_frontier_remaining_by_agent = {}
        self.marker_frontier_remaining = 0
        self.marker_frontier_peak_count = 0
        self.marker_frontier_min_count = -1
        self.marker_frontier_completion_ratio = 0.0

        self.frontier_remaining = 0
        self.frontier_peak_count = 0
        self.frontier_min_count = -1
        self.frontier_completion_ratio = 0.0
        self.frontier_progress_source = "task_metrics"
        self.completion_ratio = 0.0

        self.relay_active_duration = 0.0
        self.relay_total_occupancy_sec = 0.0
        self.relay_occupancy_cost = 0.0
        self.relay_event_count = 0
        self.relay_agent_list = set()
        self.planning_artifact_agent_list = set()

        self.relay_role_count_by_agent = defaultdict(int)
        self.relay_event_count_by_agent = defaultdict(int)
        self.relay_active_duration_by_agent = defaultdict(float)
        self._relay_last_role_state_by_agent = {}
        self._relay_last_state_stamp_by_agent = {}
        self._relay_active_by_agent = {}
        self._relay_last_target_by_agent = {}
        self._open_relay_events = {}
        self._relay_events = []

        self.planning_new_count_by_agent = defaultdict(int)
        self.planning_bspline_count_by_agent = defaultdict(int)
        self.planning_pos_cmd_count_by_agent = defaultdict(int)
        self._planning_new_seen_agents = set()
        self._planning_bspline_seen_agents = set()
        self._planning_pos_cmd_seen_agents = set()

        self.team_total_path_length_m = 0.0
        self.relay_total_path_length_m = 0.0
        self.relay_path_fraction = 0.0
        self.completion_ratio_per_meter = None
        self.frontier_completion_ratio_per_meter = None
        self.team_low_speed_ratio = 0.0
        self.path_length_by_agent = defaultdict(float)
        self.relay_path_length_by_agent = defaultdict(float)
        self.low_speed_duration_by_agent = defaultdict(float)
        self.low_speed_ratio_by_agent = defaultdict(float)
        self._last_odom_state_by_agent = {}

        self._start_time = None
        self._information_age_sum = 0.0
        self._information_age_samples = 0
        self._metric_samples = []
        self._last_sample_elapsed = None
        self._events_finalized = False
        self._completion_candidate_since = None

        rospy.Subscriber(self.relay_role_topic, RelayRoleCmd, self.relay_role_callback, queue_size=50)
        rospy.Subscriber(self.swarm_comm_state_topic, SwarmCommState, self.swarm_comm_state_callback, queue_size=50)
        rospy.Subscriber(self.task_metrics_topic, Int32MultiArray, self.task_metrics_callback, queue_size=50)
        self._frontier_subscribers = []
        for frontier_agent_id, frontier_topic in sorted(self.frontier_topics_by_agent.items()):
            self._frontier_subscribers.append(
                rospy.Subscriber(
                    frontier_topic,
                    Marker,
                    partial(self.frontier_callback, frontier_agent_id),
                    queue_size=200,
                )
            )

        for planning_agent_id in self.planning_agent_ids:
            rospy.Subscriber(
                "/planning/new_{}".format(planning_agent_id),
                Empty,
                partial(self.planning_new_callback, planning_agent_id),
                queue_size=20,
            )
            rospy.Subscriber(
                "/planning/bspline_{}".format(planning_agent_id),
                Bspline,
                partial(self.planning_bspline_callback, planning_agent_id),
                queue_size=20,
            )
            rospy.Subscriber(
                "/planning/pos_cmd_{}".format(planning_agent_id),
                PositionCommand,
                partial(self.planning_pos_cmd_callback, planning_agent_id),
                queue_size=50,
            )

        for odom_agent_id in self.odom_agent_ids:
            rospy.Subscriber(
                "{}{}".format(self.odom_topic_prefix, odom_agent_id),
                Odometry,
                partial(self.odom_callback, odom_agent_id),
                queue_size=100,
            )

        rospy.Timer(rospy.Duration(self.metric_sample_period), self.metric_sample_timer_callback)
        rospy.Timer(rospy.Duration(self.summary_period), self.summary_timer_callback)
        rospy.on_shutdown(self.write_report)

        frontier_topics_csv = ",".join(
            "{}:{}".format(agent_id, topic)
            for agent_id, topic in sorted(self.frontier_topics_by_agent.items())
        )
        rospy.loginfo(
            "[BaselineMonitor] watching relay_role_topic=%s swarm_comm_state_topic=%s task_metrics_topic=%s frontier_topics=%s planning_agent_ids=%s require_pos_cmd=%s frontier_completion_threshold=%d completion_confirm_sec=%.2f completion_frontier_cluster_threshold=%d completion_active_grid_threshold=%d report_path=%s events_path=%s",
            self.relay_role_topic,
            self.swarm_comm_state_topic,
            self.task_metrics_topic,
            frontier_topics_csv,
            self.planning_agent_ids,
            self.require_pos_cmd,
            self.frontier_completion_threshold,
            self.completion_confirm_sec,
            self.completion_frontier_cluster_threshold,
            self.completion_active_grid_threshold,
            self.report_path if self.report_path else "<disabled>",
            self.events_path if self.events_path else "<disabled>",
        )

    @staticmethod
    def _normalize_int_list(value, default=None):
        if default is None:
            default = []

        if isinstance(value, int):
            return [int(value)]

        if isinstance(value, str):
            raw_items = [item.strip() for item in value.split(",") if item.strip()]
            if not raw_items:
                return list(default)
            return [int(item) for item in raw_items]

        if isinstance(value, (list, tuple)):
            items = []
            for item in value:
                if item is None:
                    continue
                items.append(int(item))
            return items if items else list(default)

        return list(default)

    @staticmethod
    def _resolve_frontier_topics(frontier_topic, planning_agent_ids, default_agent_id):
        match = re.match(r"^(.*_)(\d+)$", frontier_topic)
        if match and planning_agent_ids:
            topic_prefix = match.group(1)
            return {int(agent_id): "{}{}".format(topic_prefix, int(agent_id)) for agent_id in planning_agent_ids}
        return {int(default_agent_id): frontier_topic}

    @staticmethod
    def _csv_from_int_set(values):
        return ",".join(str(value) for value in sorted(values))

    @staticmethod
    def _csv_from_mapping(mapping, precision=6):
        items = []
        for key in sorted(mapping):
            value = mapping[key]
            if isinstance(value, float):
                items.append("{}:{:.{precision}f}".format(key, value, precision=precision))
            else:
                items.append("{}:{}".format(key, value))
        return ",".join(items)

    def stamp_now(self):
        now = rospy.get_time()
        if now <= 0.0:
            return ""
        if self._start_time is None:
            self._start_time = now
        return "{:.3f}".format(now)

    def elapsed_now(self):
        now = rospy.get_time()
        if now <= 0.0:
            return 0.0
        if self._start_time is None:
            self._start_time = now
        return max(0.0, now - self._start_time)

    @staticmethod
    def format_optional(value, precision=6):
        if value is None:
            return "NA"
        if isinstance(value, float):
            if not math.isfinite(value):
                return "NA"
            return "{:.{precision}f}".format(value, precision=precision)
        return str(value)

    @staticmethod
    def _distance_between_positions(lhs, rhs):
        dx = float(lhs[0]) - float(rhs[0])
        dy = float(lhs[1]) - float(rhs[1])
        dz = float(lhs[2]) - float(rhs[2])
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _task_metrics_agent_id_from_msg(self, msg):
        if len(msg.data) >= 5:
            try:
                return int(msg.data[4])
            except (TypeError, ValueError):
                pass
        return self.agent_id

    def _fresh_task_metrics_agent_ids(self):
        if not self._task_metrics_by_agent:
            return []

        now_elapsed = self.elapsed_now()
        if now_elapsed <= 0.0:
            return sorted(self._task_metrics_by_agent.keys())

        fresh_agent_ids = []
        for agent_id, sample_elapsed in self._task_metrics_elapsed_by_agent.items():
            if now_elapsed - sample_elapsed <= self.task_metrics_agent_timeout_sec:
                fresh_agent_ids.append(agent_id)
        return sorted(fresh_agent_ids)

    def _refresh_task_metrics_aggregate(self):
        fresh_agent_ids = self._fresh_task_metrics_agent_ids()
        self.task_metrics_fresh_agent_ids = set(fresh_agent_ids)
        self.task_metrics_fresh_agent_count = len(fresh_agent_ids)
        if not fresh_agent_ids:
            return False

        fresh_metrics = [self._task_metrics_by_agent[agent_id] for agent_id in fresh_agent_ids]
        self.unknown_cells_total = max(metric["unknown_cells_total"] for metric in fresh_metrics)
        self.frontier_cluster_count_metric = max(
            metric["frontier_cluster_count_metric"] for metric in fresh_metrics
        )
        self.frontier_cell_count_metric = max(metric["frontier_cell_count_metric"] for metric in fresh_metrics)
        self.active_grid_count = max(metric["active_grid_count"] for metric in fresh_metrics)

        self.unknown_cells_peak = max(self.unknown_cells_peak, self.unknown_cells_total)
        if self.unknown_cells_min < 0:
            self.unknown_cells_min = self.unknown_cells_total
        else:
            self.unknown_cells_min = min(self.unknown_cells_min, self.unknown_cells_total)

        if self.unknown_cells_peak > 0:
            self.completion_ratio = max(
                0.0,
                min(1.0, 1.0 - float(self.unknown_cells_total) / float(self.unknown_cells_peak)),
            )
        return True

    def record_metric_sample(self, force=False):
        elapsed = self.elapsed_now()
        if elapsed <= 0.0 and not force:
            return

        if (
            not force
            and self._last_sample_elapsed is not None
            and elapsed - self._last_sample_elapsed < max(0.05, 0.5 * self.metric_sample_period)
        ):
            return

        sample = self.snapshot_metrics()
        sample["elapsed_sec"] = elapsed
        self._metric_samples.append(sample)
        self._last_sample_elapsed = elapsed

    def metric_sample_timer_callback(self, _event):
        self.record_metric_sample()

    def _window_samples(self, start_elapsed, end_elapsed, window_name):
        if window_name == "pre":
            low = max(0.0, start_elapsed - self.pre_window_sec)
            high = start_elapsed
            return [
                sample
                for sample in self._metric_samples
                if low <= sample.get("elapsed_sec", 0.0) < high
            ]

        if window_name == "during":
            return [
                sample
                for sample in self._metric_samples
                if start_elapsed <= sample.get("elapsed_sec", 0.0) <= end_elapsed
            ]

        low = end_elapsed
        high = end_elapsed + self.post_window_sec
        return [
            sample
            for sample in self._metric_samples
            if low < sample.get("elapsed_sec", 0.0) <= high
        ]

    @staticmethod
    def _mean_metric(samples, key):
        if not samples:
            return None
        values = [float(sample.get(key, 0.0)) for sample in samples]
        if not values:
            return None
        return sum(values) / float(len(values))

    def _annotate_relay_events(self):
        success_anchor = self.success_elapsed_sec
        frontier_anchor = self.first_frontier_positive_elapsed_sec

        for event in self._relay_events:
            start_elapsed = max(0.0, float(event["start_elapsed_sec"]))
            end_elapsed = float(event["end_elapsed_sec"])
            if end_elapsed < start_elapsed:
                end_elapsed = start_elapsed

            by_planning = success_anchor is not None and start_elapsed >= success_anchor
            by_frontier = frontier_anchor is not None and start_elapsed >= frontier_anchor
            by_all = (
                success_anchor is not None
                and frontier_anchor is not None
                and start_elapsed >= max(success_anchor, frontier_anchor)
            )

            event["post_startup_by_planning_success"] = 1 if by_planning else 0
            event["post_startup_by_frontier_positive"] = 1 if by_frontier else 0
            event["post_startup_any"] = 1 if (by_planning or by_frontier) else 0
            event["post_startup_all"] = 1 if by_all else 0

            for window_name in ("pre", "during", "post"):
                samples = self._window_samples(start_elapsed, end_elapsed, window_name)
                event["{}_window_sample_count".format(window_name)] = len(samples)
                for metric_name in (
                    "num_components",
                    "completion_ratio",
                    "frontier_completion_ratio",
                    "frontier_remaining",
                    "information_age",
                ):
                    event["{}_{}_mean".format(window_name, metric_name)] = self._mean_metric(samples, metric_name)

    def finalize_events(self):
        if self._events_finalized:
            return

        self.update_all_relay_durations()
        self.close_open_relay_events("shutdown")
        self.update_all_relay_durations()
        self.refresh_frontier_progress()
        self.record_metric_sample(force=True)
        self._annotate_relay_events()
        self._events_finalized = True

    def snapshot_metrics(self):
        return {
            "elapsed_sec": self.elapsed_now(),
            "num_components": self.last_num_components,
            "completion_ratio": self.completion_ratio,
            "frontier_remaining": self.frontier_remaining,
            "frontier_completion_ratio": self.frontier_completion_ratio,
            "information_age": self.last_information_age,
            "unknown_cells_total": self.unknown_cells_total,
        }

    def refresh_runtime_metrics(self):
        self.update_all_relay_durations()
        self.team_total_path_length_m = sum(self.path_length_by_agent.values())
        self.relay_total_path_length_m = sum(self.relay_path_length_by_agent.values())
        if self.team_total_path_length_m > 0.0:
            self.relay_path_fraction = max(0.0, min(1.0, self.relay_total_path_length_m / self.team_total_path_length_m))
            self.completion_ratio_per_meter = self.completion_ratio / self.team_total_path_length_m
            self.frontier_completion_ratio_per_meter = (
                self.frontier_completion_ratio / self.team_total_path_length_m
            )
        else:
            self.relay_path_fraction = 0.0
            self.completion_ratio_per_meter = None
            self.frontier_completion_ratio_per_meter = None

        elapsed = self.elapsed_now()
        if elapsed > 0.0:
            self.runtime_sec = elapsed
            self.relay_occupancy_cost = self.relay_total_occupancy_sec / elapsed
            total_low_speed_duration = 0.0
            for agent_id in self.odom_agent_ids:
                agent_low_speed = self.low_speed_duration_by_agent[agent_id]
                self.low_speed_ratio_by_agent[agent_id] = max(0.0, min(1.0, agent_low_speed / elapsed))
                total_low_speed_duration += agent_low_speed
            denom = elapsed * float(len(self.odom_agent_ids)) if self.odom_agent_ids else 0.0
            self.team_low_speed_ratio = (
                max(0.0, min(1.0, total_low_speed_duration / denom)) if denom > 0.0 else 0.0
            )

    def build_report_lines(self, finalized):
        planning_artifact_success = 1 if self.success_logged else 0
        planning_success_agents_csv = self._csv_from_int_set(self.planning_artifact_agent_list)
        relay_agent_list_csv = self._csv_from_int_set(self.relay_agent_list)
        team_agent_ids_csv = self._csv_from_int_set(self.team_agent_ids)
        per_agent_relay_occupancy_sec_csv = self._csv_from_mapping(self.relay_active_duration_by_agent)
        per_agent_relay_event_count_csv = self._csv_from_mapping(self.relay_event_count_by_agent, precision=0)
        per_agent_relay_role_count_csv = self._csv_from_mapping(self.relay_role_count_by_agent, precision=0)
        per_agent_path_length_m_csv = self._csv_from_mapping(self.path_length_by_agent)
        per_agent_relay_path_length_m_csv = self._csv_from_mapping(self.relay_path_length_by_agent)
        per_agent_low_speed_duration_sec_csv = self._csv_from_mapping(self.low_speed_duration_by_agent)
        per_agent_low_speed_ratio_csv = self._csv_from_mapping(self.low_speed_ratio_by_agent)

        relay_events = self._relay_events if finalized else list(self._relay_events)
        post_startup_events = [event for event in relay_events if int(event.get("post_startup_any", 0)) == 1]
        post_startup_by_planning_events = [
            event for event in relay_events if int(event.get("post_startup_by_planning_success", 0)) == 1
        ]
        post_startup_by_frontier_events = [
            event for event in relay_events if int(event.get("post_startup_by_frontier_positive", 0)) == 1
        ]
        post_startup_all_events = [event for event in relay_events if int(event.get("post_startup_all", 0)) == 1]
        post_startup_agents_csv = self._csv_from_int_set({int(event["agent_id"]) for event in post_startup_events})
        post_startup_total_occupancy_sec = sum(float(event["duration_sec"]) for event in post_startup_events)

        lines = [
            "report_finalized={}".format(1 if finalized else 0),
            "agent_id={}".format(self.agent_id),
            "team_agent_count={}".format(self.team_agent_count),
            "team_agent_ids_csv={}".format(team_agent_ids_csv),
            "planning_agent_ids_csv={}".format(self._csv_from_int_set(set(self.planning_agent_ids))),
            "odom_agent_ids_csv={}".format(self._csv_from_int_set(set(self.odom_agent_ids))),
            "metric_sample_period={:.6f}".format(self.metric_sample_period),
            "pre_window_sec={:.6f}".format(self.pre_window_sec),
            "post_window_sec={:.6f}".format(self.post_window_sec),
            "relay_role_count={}".format(self.relay_role_count),
            "tracked_relay_role_count={}".format(self.tracked_relay_role_count),
            "planning_new_count={}".format(self.planning_new_count),
            "planning_bspline_count={}".format(self.planning_bspline_count),
            "planning_pos_cmd_count={}".format(self.planning_pos_cmd_count),
            "frontier_message_count={}".format(self.frontier_message_count),
            "frontier_message_count_by_agent_csv={}".format(
                self._csv_from_mapping(self.frontier_message_count_by_agent)
            ),
            "task_metrics_count={}".format(self.task_metrics_count),
            "task_metrics_agent_timeout_sec={:.6f}".format(self.task_metrics_agent_timeout_sec),
            "task_metrics_fresh_agent_count={}".format(self.task_metrics_fresh_agent_count),
            "task_metrics_fresh_agent_ids_csv={}".format(
                self._csv_from_int_set(self.task_metrics_fresh_agent_ids)
            ),
            "saw_relay_role={}".format(1 if self.saw_relay_role else 0),
            "saw_planning_new={}".format(1 if self.saw_planning_new else 0),
            "saw_planning_bspline={}".format(1 if self.saw_planning_bspline else 0),
            "saw_planning_pos_cmd={}".format(1 if self.saw_planning_pos_cmd else 0),
            "saw_frontier={}".format(1 if self.saw_frontier else 0),
            "saw_task_metrics={}".format(1 if self.saw_task_metrics else 0),
            "planning_artifact_success={}".format(planning_artifact_success),
            "team_planning_artifact_success={}".format(planning_artifact_success),
            "planning_artifact_agent_list_csv={}".format(planning_success_agents_csv),
            "success={}".format(planning_artifact_success),
            "require_pos_cmd={}".format(1 if self.require_pos_cmd else 0),
            "task_completed={}".format(self.task_completed),
            "frontier_completion_threshold={}".format(self.frontier_completion_threshold),
            "completion_confirm_sec={:.6f}".format(self.completion_confirm_sec),
            "completion_frontier_cluster_threshold={}".format(self.completion_frontier_cluster_threshold),
            "completion_active_grid_threshold={}".format(self.completion_active_grid_threshold),
            "role_switch_count={}".format(self.role_switch_count),
            "relay_drop_count={}".format(self.relay_drop_count),
            "relay_event_count={}".format(self.relay_event_count),
            "relay_agent_list_csv={}".format(relay_agent_list_csv),
            "relay_total_occupancy_sec={:.6f}".format(self.relay_total_occupancy_sec),
            "post_startup_event_count={}".format(len(post_startup_events)),
            "post_startup_event_run={}".format(1 if post_startup_events else 0),
            "post_startup_relay_agent_list_csv={}".format(post_startup_agents_csv),
            "post_startup_relay_total_occupancy_sec={:.6f}".format(post_startup_total_occupancy_sec),
            "post_startup_by_planning_event_count={}".format(len(post_startup_by_planning_events)),
            "post_startup_by_frontier_event_count={}".format(len(post_startup_by_frontier_events)),
            "post_startup_all_event_count={}".format(len(post_startup_all_events)),
            "frontier_progress_source={}".format(self.frontier_progress_source),
            "frontier_remaining={}".format(self.frontier_remaining),
            "frontier_peak_count={}".format(self.frontier_peak_count),
            "frontier_min_count={}".format(self.frontier_min_count),
            "frontier_completion_ratio={:.6f}".format(self.frontier_completion_ratio),
            "marker_frontier_remaining={}".format(self.marker_frontier_remaining),
            "marker_frontier_remaining_by_agent_csv={}".format(
                self._csv_from_mapping(self.marker_frontier_remaining_by_agent)
            ),
            "marker_frontier_peak_count={}".format(self.marker_frontier_peak_count),
            "marker_frontier_min_count={}".format(self.marker_frontier_min_count),
            "marker_frontier_completion_ratio={:.6f}".format(self.marker_frontier_completion_ratio),
            "unknown_cells_total={}".format(self.unknown_cells_total),
            "unknown_cells_peak={}".format(self.unknown_cells_peak),
            "unknown_cells_min={}".format(self.unknown_cells_min),
            "frontier_cluster_count_metric={}".format(self.frontier_cluster_count_metric),
            "frontier_cell_count_metric={}".format(self.frontier_cell_count_metric),
            "active_grid_count={}".format(self.active_grid_count),
            "completion_ratio={:.6f}".format(self.completion_ratio),
            "team_completion_ratio={:.6f}".format(self.completion_ratio),
            "relay_active_duration={:.6f}".format(self.relay_total_occupancy_sec),
            "runtime_sec={:.6f}".format(self.runtime_sec),
            "team_total_path_length_m={:.6f}".format(self.team_total_path_length_m),
            "relay_total_path_length_m={:.6f}".format(self.relay_total_path_length_m),
            "relay_path_fraction={:.6f}".format(self.relay_path_fraction),
            "completion_ratio_per_meter={}".format(self.format_optional(self.completion_ratio_per_meter)),
            "frontier_completion_ratio_per_meter={}".format(
                self.format_optional(self.frontier_completion_ratio_per_meter)
            ),
            "team_low_speed_ratio={:.6f}".format(self.team_low_speed_ratio),
            "relay_occupancy_cost={:.6f}".format(self.relay_occupancy_cost),
            "last_num_components={}".format(self.last_num_components),
            "num_components_peak={}".format(self.num_components_peak),
            "comm_fragmentation_observed={}".format(1 if self.comm_fragmentation_observed else 0),
            "last_information_age={:.6f}".format(self.last_information_age),
            "mean_information_age={:.6f}".format(self.mean_information_age),
            "max_information_age={:.6f}".format(self.max_information_age),
            "first_relay_time={}".format(self.first_relay_time),
            "first_planning_new_time={}".format(self.first_planning_new_time),
            "first_planning_bspline_time={}".format(self.first_planning_bspline_time),
            "first_planning_pos_cmd_time={}".format(self.first_planning_pos_cmd_time),
            "first_frontier_time={}".format(self.first_frontier_time),
            "first_frontier_positive_time={}".format(self.first_frontier_positive_time),
            "success_elapsed_sec={}".format(self.format_optional(self.success_elapsed_sec)),
            "first_frontier_positive_elapsed_sec={}".format(self.format_optional(self.first_frontier_positive_elapsed_sec)),
            "completion_time={}".format(self.completion_time),
            "completion_elapsed_sec={:.6f}".format(self.completion_elapsed_sec),
            "success_time={}".format(self.success_time),
            "last_role={}".format(self.last_role),
            "last_valid={}".format(self.last_valid),
            "last_target_x={:.6f}".format(self.last_target[0]),
            "last_target_y={:.6f}".format(self.last_target[1]),
            "last_target_z={:.6f}".format(self.last_target[2]),
            "last_bspline_traj_id={}".format(self.last_bspline_traj_id),
            "last_bspline_knot_count={}".format(self.last_bspline_knot_count),
            "last_pos_cmd_traj_id={}".format(self.last_pos_cmd_traj_id),
            "last_pos_cmd_x={:.6f}".format(self.last_pos_cmd_position[0]),
            "last_pos_cmd_y={:.6f}".format(self.last_pos_cmd_position[1]),
            "last_pos_cmd_z={:.6f}".format(self.last_pos_cmd_position[2]),
            "last_pos_cmd_vx={:.6f}".format(self.last_pos_cmd_velocity[0]),
            "last_pos_cmd_vy={:.6f}".format(self.last_pos_cmd_velocity[1]),
            "last_pos_cmd_vz={:.6f}".format(self.last_pos_cmd_velocity[2]),
            "per_agent_relay_occupancy_sec_csv={}".format(per_agent_relay_occupancy_sec_csv),
            "per_agent_relay_event_count_csv={}".format(per_agent_relay_event_count_csv),
            "per_agent_relay_role_count_csv={}".format(per_agent_relay_role_count_csv),
            "per_agent_path_length_m_csv={}".format(per_agent_path_length_m_csv),
            "per_agent_relay_path_length_m_csv={}".format(per_agent_relay_path_length_m_csv),
            "per_agent_low_speed_duration_sec_csv={}".format(per_agent_low_speed_duration_sec_csv),
            "per_agent_low_speed_ratio_csv={}".format(per_agent_low_speed_ratio_csv),
            "events_path={}".format(self.events_path),
        ]
        return lines

    def write_report_snapshot(self):
        if not self.report_path or not self.write_runtime_report:
            return

        self.refresh_frontier_progress()
        self.refresh_runtime_metrics()
        report_dir = os.path.dirname(self.report_path)
        if report_dir:
            os.makedirs(report_dir, exist_ok=True)

        with open(self.report_path, "w", encoding="ascii") as report_file:
            report_file.write("\n".join(self.build_report_lines(finalized=False)) + "\n")

    def maybe_log_success(self):
        if self.success_logged:
            return

        success = bool(self._planning_bspline_seen_agents)
        if self.require_pos_cmd:
            success = success and bool(self._planning_pos_cmd_seen_agents)

        if success:
            self.success_logged = True
            self.success_time = self.stamp_now()
            self.success_elapsed_sec = self.elapsed_now()
            self.planning_artifact_agent_list = set(self._planning_bspline_seen_agents)
            if self.require_pos_cmd:
                self.planning_artifact_agent_list &= set(self._planning_pos_cmd_seen_agents)
            self.record_metric_sample(force=True)
            rospy.loginfo(
                "[BaselineMonitor] success: planning artifacts observed on agents=%s completion_ratio=%.3f frontier_remaining=%d relay_events=%d",
                sorted(self.planning_artifact_agent_list),
                self.completion_ratio,
                self.frontier_remaining,
                self.relay_event_count,
            )
            self.write_report_snapshot()

    def update_agent_relay_duration(self, agent_id):
        now = rospy.get_time()
        if now <= 0.0:
            return

        last_stamp = self._relay_last_state_stamp_by_agent.get(agent_id)
        if last_stamp is None:
            self._relay_last_state_stamp_by_agent[agent_id] = now
            return

        dt = max(0.0, now - last_stamp)
        if self._relay_active_by_agent.get(agent_id, False):
            self.relay_active_duration_by_agent[agent_id] += dt
        self._relay_last_state_stamp_by_agent[agent_id] = now

    def update_all_relay_durations(self):
        for agent_id in list(self._relay_last_state_stamp_by_agent.keys()):
            self.update_agent_relay_duration(agent_id)
        self.relay_total_occupancy_sec = sum(self.relay_active_duration_by_agent.values())
        self.relay_active_duration = self.relay_total_occupancy_sec

    def begin_relay_event(self, agent_id, target):
        self.record_metric_sample(force=True)
        snapshot = self.snapshot_metrics()
        event = {
            "event_id": len(self._relay_events) + len(self._open_relay_events) + 1,
            "agent_id": int(agent_id),
            "start_time": self.stamp_now(),
            "start_elapsed_sec": snapshot["elapsed_sec"],
            "start_target_x": target[0],
            "start_target_y": target[1],
            "start_target_z": target[2],
            "start_num_components": snapshot["num_components"],
            "start_completion_ratio": snapshot["completion_ratio"],
            "start_frontier_remaining": snapshot["frontier_remaining"],
            "start_frontier_completion_ratio": snapshot["frontier_completion_ratio"],
            "start_information_age": snapshot["information_age"],
            "start_unknown_cells_total": snapshot["unknown_cells_total"],
            "end_time": "",
            "end_elapsed_sec": -1.0,
            "end_target_x": target[0],
            "end_target_y": target[1],
            "end_target_z": target[2],
            "end_num_components": snapshot["num_components"],
            "end_completion_ratio": snapshot["completion_ratio"],
            "end_frontier_remaining": snapshot["frontier_remaining"],
            "end_frontier_completion_ratio": snapshot["frontier_completion_ratio"],
            "end_information_age": snapshot["information_age"],
            "end_unknown_cells_total": snapshot["unknown_cells_total"],
            "duration_sec": 0.0,
            "end_reason": "open",
        }
        self._open_relay_events[agent_id] = event
        self.relay_event_count += 1
        self.relay_event_count_by_agent[agent_id] += 1
        self.relay_agent_list.add(agent_id)
        rospy.loginfo(
            "[BaselineMonitor] relay_start event=%d agent=%d target=(%.3f, %.3f, %.3f) components=%d completion_ratio=%.3f frontier_remaining=%d",
            event["event_id"],
            agent_id,
            target[0],
            target[1],
            target[2],
            snapshot["num_components"],
            snapshot["completion_ratio"],
            snapshot["frontier_remaining"],
        )

    def end_relay_event(self, agent_id, target, end_reason):
        event = self._open_relay_events.pop(agent_id, None)
        if event is None:
            return

        self.update_agent_relay_duration(agent_id)
        self.record_metric_sample(force=True)
        snapshot = self.snapshot_metrics()
        event["end_time"] = self.stamp_now()
        event["end_elapsed_sec"] = snapshot["elapsed_sec"]
        event["end_target_x"] = target[0]
        event["end_target_y"] = target[1]
        event["end_target_z"] = target[2]
        event["end_num_components"] = snapshot["num_components"]
        event["end_completion_ratio"] = snapshot["completion_ratio"]
        event["end_frontier_remaining"] = snapshot["frontier_remaining"]
        event["end_frontier_completion_ratio"] = snapshot["frontier_completion_ratio"]
        event["end_information_age"] = snapshot["information_age"]
        event["end_unknown_cells_total"] = snapshot["unknown_cells_total"]
        event["duration_sec"] = max(0.0, event["end_elapsed_sec"] - event["start_elapsed_sec"])
        event["end_reason"] = end_reason
        self._relay_events.append(event)
        rospy.loginfo(
            "[BaselineMonitor] relay_end event=%d agent=%d duration=%.3fs components=%d->%d completion_ratio=%.3f->%.3f frontier_remaining=%d->%d reason=%s",
            event["event_id"],
            agent_id,
            event["duration_sec"],
            event["start_num_components"],
            event["end_num_components"],
            event["start_completion_ratio"],
            event["end_completion_ratio"],
            event["start_frontier_remaining"],
            event["end_frontier_remaining"],
            end_reason,
        )

    def close_open_relay_events(self, end_reason):
        open_agent_ids = sorted(self._open_relay_events.keys())
        for agent_id in open_agent_ids:
            target = self._relay_last_target_by_agent.get(agent_id, (0.0, 0.0, 0.0))
            self.end_relay_event(agent_id, target, end_reason)

    def _completion_candidate_active(self):
        if self.frontier_completion_threshold < 0 or self.frontier_peak_count <= 0:
            return False
        if self.frontier_remaining > self.frontier_completion_threshold:
            return False
        if self.task_metrics_count > 0:
            if self.task_metrics_fresh_agent_count <= 0:
                return False
            if (
                self.completion_frontier_cluster_threshold >= 0
                and self.frontier_cluster_count_metric > self.completion_frontier_cluster_threshold
            ):
                return False
            if (
                self.completion_active_grid_threshold >= 0
                and self.active_grid_count > self.completion_active_grid_threshold
            ):
                return False
        return True

    def _update_task_completion_status(self):
        if self.task_completed:
            return

        if not self._completion_candidate_active():
            self._completion_candidate_since = None
            return

        now_elapsed = self.elapsed_now()
        if self._completion_candidate_since is None:
            self._completion_candidate_since = now_elapsed
            rospy.loginfo(
                "[BaselineMonitor] completion candidate entered: frontier_remaining=%d threshold=%d frontier_clusters=%d active_grids=%d confirm_sec=%.2f.",
                self.frontier_remaining,
                self.frontier_completion_threshold,
                self.frontier_cluster_count_metric,
                self.active_grid_count,
                self.completion_confirm_sec,
            )
            return

        if now_elapsed - self._completion_candidate_since < self.completion_confirm_sec:
            return

        self.completion_time = self.stamp_now()
        self.completion_elapsed_sec = now_elapsed
        self.task_completed = 1
        rospy.loginfo(
            "[BaselineMonitor] frontier completion confirmed: frontier_remaining=%d threshold=%d frontier_clusters=%d active_grids=%d confirm_sec=%.2f.",
            self.frontier_remaining,
            self.frontier_completion_threshold,
            self.frontier_cluster_count_metric,
            self.active_grid_count,
            self.completion_confirm_sec,
        )
        self.write_report_snapshot()

    def update_frontier_progress(self, frontier_remaining):
        self.frontier_remaining = int(frontier_remaining)
        self.frontier_peak_count = max(self.frontier_peak_count, self.frontier_remaining)
        if self.frontier_min_count < 0:
            self.frontier_min_count = self.frontier_remaining
        else:
            self.frontier_min_count = min(self.frontier_min_count, self.frontier_remaining)

        if self.frontier_peak_count > 0:
            self.frontier_completion_ratio = max(
                0.0,
                min(1.0, 1.0 - float(self.frontier_remaining) / float(self.frontier_peak_count)),
            )

        if self.first_frontier_positive_elapsed_sec is None and self.frontier_remaining > 0:
            self.first_frontier_positive_time = self.stamp_now()
            self.first_frontier_positive_elapsed_sec = self.elapsed_now()

        self._update_task_completion_status()

    def refresh_frontier_progress(self):
        self._refresh_task_metrics_aggregate()

        if self.saw_frontier:
            self.frontier_progress_source = "marker"
            self.update_frontier_progress(self.marker_frontier_remaining)
            return

        if self.task_metrics_count > 0 and self.task_metrics_fresh_agent_count > 0:
            self.frontier_progress_source = "task_metrics"
            self.update_frontier_progress(self.frontier_cell_count_metric)
            return

        self.frontier_progress_source = "none"

    def relay_role_callback(self, msg):
        agent_id = int(msg.agent_id)
        role_name = ROLE_NAMES.get(msg.role, "ROLE_{}".format(int(msg.role)))
        target = (msg.relay_target.x, msg.relay_target.y, msg.relay_target.z)
        active_relay = msg.role == RelayRoleCmd.ROLE_RELAY and msg.valid

        self.team_agent_ids.add(agent_id)
        self.team_agent_count = max(self.team_agent_count, len(self.team_agent_ids))

        self.update_agent_relay_duration(agent_id)

        self.relay_role_count += 1
        self.relay_role_count_by_agent[agent_id] += 1
        self.saw_relay_role = True
        if not self.first_relay_time:
            self.first_relay_time = self.stamp_now()

        previous_state = self._relay_last_role_state_by_agent.get(agent_id)
        previous_active = previous_state == (RelayRoleCmd.ROLE_RELAY, True)
        current_state = (int(msg.role), bool(msg.valid))

        if previous_state is not None and current_state != previous_state:
            self.role_switch_count += 1
            if previous_active and not active_relay:
                self.relay_drop_count += 1

        if previous_active and not active_relay:
            self.end_relay_event(agent_id, target, "role_change")
        elif active_relay and not previous_active:
            self.begin_relay_event(agent_id, target)

        self._relay_last_role_state_by_agent[agent_id] = current_state
        self._relay_active_by_agent[agent_id] = active_relay
        self._relay_last_target_by_agent[agent_id] = target

        if agent_id == self.agent_id:
            self.tracked_relay_role_count += 1
            self.last_role = role_name
            self.last_valid = 1 if msg.valid else 0
            self.last_target = target
            if self.tracked_relay_role_count == 1:
                rospy.loginfo(
                    "[BaselineMonitor] observed tracked relay role cmd: role=%s valid=%s target=(%.3f, %.3f, %.3f)",
                    role_name,
                    msg.valid,
                    target[0],
                    target[1],
                    target[2],
                )

        self.maybe_log_success()

    def swarm_comm_state_callback(self, msg):
        self.team_agent_ids.update(int(agent_id) for agent_id in msg.agent_ids)
        self.team_agent_count = max(self.team_agent_count, len(msg.agent_ids))
        self.last_num_components = int(msg.num_components)
        self.num_components_peak = max(self.num_components_peak, self.last_num_components)
        if self.last_num_components > 1:
            self.comm_fragmentation_observed = True

        finite_link_ages = []
        for link_up, link_age in zip(msg.link_up, msg.link_age):
            if not link_up:
                continue
            age = float(link_age)
            if math.isfinite(age) and age >= 0.0:
                finite_link_ages.append(age)

        if not finite_link_ages:
            self.record_metric_sample()
            return

        mean_age = sum(finite_link_ages) / float(len(finite_link_ages))
        self.last_information_age = mean_age
        self.max_information_age = max(self.max_information_age, max(finite_link_ages))
        self._information_age_sum += mean_age
        self._information_age_samples += 1
        self.mean_information_age = self._information_age_sum / float(self._information_age_samples)
        self.record_metric_sample()

    def task_metrics_callback(self, msg):
        self.task_metrics_count += 1
        self.saw_task_metrics = True

        agent_id = self._task_metrics_agent_id_from_msg(msg)
        self.team_agent_ids.add(agent_id)
        self.team_agent_count = max(self.team_agent_count, len(self.team_agent_ids))

        metrics = {
            "unknown_cells_total": int(msg.data[0]) if len(msg.data) >= 1 else 0,
            "frontier_cluster_count_metric": int(msg.data[1]) if len(msg.data) >= 2 else 0,
            "frontier_cell_count_metric": int(msg.data[2]) if len(msg.data) >= 3 else 0,
            "active_grid_count": int(msg.data[3]) if len(msg.data) >= 4 else 0,
        }
        self._task_metrics_by_agent[agent_id] = metrics
        self._task_metrics_elapsed_by_agent[agent_id] = self.elapsed_now()

        self.refresh_frontier_progress()
        self.record_metric_sample()

    def odom_callback(self, odom_agent_id, msg):
        stamp = msg.header.stamp.to_sec() if msg.header.stamp and not msg.header.stamp.is_zero() else rospy.get_time()
        if stamp <= 0.0:
            return

        position = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
        velocity = msg.twist.twist.linear
        speed = math.sqrt(velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z)
        last_state = self._last_odom_state_by_agent.get(odom_agent_id)
        if last_state is not None:
            dt = max(0.0, stamp - float(last_state["stamp"]))
            if dt > 0.0 and (self.odom_max_gap_sec <= 0.0 or dt <= self.odom_max_gap_sec):
                segment = self._distance_between_positions(position, last_state["position"])
                if math.isfinite(segment):
                    self.path_length_by_agent[odom_agent_id] += segment
                    if self._relay_active_by_agent.get(odom_agent_id, False):
                        self.relay_path_length_by_agent[odom_agent_id] += segment
                if speed <= self.low_speed_threshold:
                    self.low_speed_duration_by_agent[odom_agent_id] += dt

        self._last_odom_state_by_agent[odom_agent_id] = {
            "stamp": stamp,
            "position": position,
        }

    def planning_new_callback(self, planning_agent_id, _msg):
        self.planning_new_count += 1
        self.planning_new_count_by_agent[planning_agent_id] += 1
        self._planning_new_seen_agents.add(planning_agent_id)
        self.saw_planning_new = True
        if not self.first_planning_new_time:
            self.first_planning_new_time = self.stamp_now()
            rospy.loginfo(
                "[BaselineMonitor] observed planning trigger on /planning/new_%d.", planning_agent_id
            )
        self.maybe_log_success()

    def planning_bspline_callback(self, planning_agent_id, msg):
        self.planning_bspline_count += 1
        self.planning_bspline_count_by_agent[planning_agent_id] += 1
        self._planning_bspline_seen_agents.add(planning_agent_id)
        self.saw_planning_bspline = True

        if planning_agent_id == self.agent_id:
            self.last_bspline_traj_id = int(msg.traj_id)
            self.last_bspline_knot_count = len(msg.knots)

        if not self.first_planning_bspline_time:
            self.first_planning_bspline_time = self.stamp_now()
            rospy.loginfo(
                "[BaselineMonitor] observed bspline output on /planning/bspline_%d: traj_id=%d knots=%d pos_pts=%d.",
                planning_agent_id,
                int(msg.traj_id),
                len(msg.knots),
                len(msg.pos_pts),
            )
        self.maybe_log_success()

    def planning_pos_cmd_callback(self, planning_agent_id, msg):
        self.planning_pos_cmd_count += 1
        self.planning_pos_cmd_count_by_agent[planning_agent_id] += 1
        self._planning_pos_cmd_seen_agents.add(planning_agent_id)
        self.saw_planning_pos_cmd = True

        if planning_agent_id == self.agent_id:
            self.last_pos_cmd_traj_id = int(msg.trajectory_id)
            self.last_pos_cmd_position = (msg.position.x, msg.position.y, msg.position.z)
            self.last_pos_cmd_velocity = (msg.velocity.x, msg.velocity.y, msg.velocity.z)

        if not self.first_planning_pos_cmd_time:
            self.first_planning_pos_cmd_time = self.stamp_now()
            rospy.loginfo(
                "[BaselineMonitor] observed position command on /planning/pos_cmd_%d: traj_id=%d pos=(%.3f, %.3f, %.3f).",
                planning_agent_id,
                int(msg.trajectory_id),
                msg.position.x,
                msg.position.y,
                msg.position.z,
            )
        self.maybe_log_success()

    def frontier_callback(self, frontier_agent_id, msg):
        self.frontier_message_count += 1
        self.frontier_message_count_by_agent[frontier_agent_id] += 1
        self.saw_frontier = True
        if not self.first_frontier_time:
            self.first_frontier_time = self.stamp_now()

        frontier_counts = self.frontier_counts_by_agent[frontier_agent_id]
        if msg.action == Marker.DELETEALL:
            frontier_counts.clear()
        elif msg.action == Marker.DELETE:
            frontier_counts.pop(int(msg.id), None)
        else:
            frontier_counts[int(msg.id)] = len(msg.points)

        self.marker_frontier_remaining_by_agent[frontier_agent_id] = sum(frontier_counts.values())
        self.marker_frontier_remaining = max(self.marker_frontier_remaining_by_agent.values(), default=0)
        self.marker_frontier_peak_count = max(self.marker_frontier_peak_count, self.marker_frontier_remaining)
        if self.marker_frontier_min_count < 0:
            self.marker_frontier_min_count = self.marker_frontier_remaining
        else:
            self.marker_frontier_min_count = min(self.marker_frontier_min_count, self.marker_frontier_remaining)

        if self.marker_frontier_peak_count > 0:
            self.marker_frontier_completion_ratio = max(
                0.0,
                min(1.0, 1.0 - float(self.marker_frontier_remaining) / float(self.marker_frontier_peak_count)),
            )

        self.refresh_frontier_progress()

        self.record_metric_sample()

    def summary_timer_callback(self, _event):
        self.refresh_frontier_progress()
        self.record_metric_sample()
        self.refresh_runtime_metrics()
        rospy.loginfo(
            "[BaselineMonitor] summary agents=%s relay_events=%d relay_agents=%s bspline=%d pos_cmd=%d success=%s completion_ratio=%.3f frontier_remaining=%d frontier_completion_ratio=%.3f team_path=%.3fm relay_path=%.3fm relay_total=%.3fs relay_cost=%.3f role_switches=%d relay_drops=%d num_components=%d mean_info_age=%.3f",
            sorted(self.team_agent_ids),
            self.relay_event_count,
            sorted(self.relay_agent_list),
            self.planning_bspline_count,
            self.planning_pos_cmd_count,
            self.success_logged,
            self.completion_ratio,
            self.frontier_remaining,
            self.frontier_completion_ratio,
            self.team_total_path_length_m,
            self.relay_total_path_length_m,
            self.relay_total_occupancy_sec,
            self.relay_occupancy_cost,
            self.role_switch_count,
            self.relay_drop_count,
            self.last_num_components,
            self.mean_information_age,
        )
        self.write_report_snapshot()

    def write_events_file(self):
        self.finalize_events()
        if not self.events_path:
            return

        events_dir = os.path.dirname(self.events_path)
        if events_dir:
            os.makedirs(events_dir, exist_ok=True)

        header = [
            "event_id",
            "agent_id",
            "start_time",
            "end_time",
            "start_elapsed_sec",
            "end_elapsed_sec",
            "duration_sec",
            "start_num_components",
            "end_num_components",
            "start_completion_ratio",
            "end_completion_ratio",
            "start_frontier_remaining",
            "end_frontier_remaining",
            "start_frontier_completion_ratio",
            "end_frontier_completion_ratio",
            "start_information_age",
            "end_information_age",
            "start_unknown_cells_total",
            "end_unknown_cells_total",
            "start_target_x",
            "start_target_y",
            "start_target_z",
            "end_target_x",
            "end_target_y",
            "end_target_z",
            "end_reason",
            "post_startup_by_planning_success",
            "post_startup_by_frontier_positive",
            "post_startup_any",
            "post_startup_all",
            "pre_window_sample_count",
            "during_window_sample_count",
            "post_window_sample_count",
        ]
        for window_name in ("pre", "during", "post"):
            for metric_name in (
                "num_components",
                "completion_ratio",
                "frontier_completion_ratio",
                "frontier_remaining",
                "information_age",
            ):
                header.append("{}_{}_mean".format(window_name, metric_name))

        with open(self.events_path, "w", encoding="ascii") as events_file:
            events_file.write("\t".join(header) + "\n")
            for event in self._relay_events:
                row = [
                    str(event["event_id"]),
                    str(event["agent_id"]),
                    str(event["start_time"]),
                    str(event["end_time"]),
                    self.format_optional(event["start_elapsed_sec"]),
                    self.format_optional(event["end_elapsed_sec"]),
                    self.format_optional(event["duration_sec"]),
                    str(event["start_num_components"]),
                    str(event["end_num_components"]),
                    self.format_optional(event["start_completion_ratio"]),
                    self.format_optional(event["end_completion_ratio"]),
                    str(event["start_frontier_remaining"]),
                    str(event["end_frontier_remaining"]),
                    self.format_optional(event["start_frontier_completion_ratio"]),
                    self.format_optional(event["end_frontier_completion_ratio"]),
                    self.format_optional(event["start_information_age"]),
                    self.format_optional(event["end_information_age"]),
                    str(event["start_unknown_cells_total"]),
                    str(event["end_unknown_cells_total"]),
                    self.format_optional(event["start_target_x"]),
                    self.format_optional(event["start_target_y"]),
                    self.format_optional(event["start_target_z"]),
                    self.format_optional(event["end_target_x"]),
                    self.format_optional(event["end_target_y"]),
                    self.format_optional(event["end_target_z"]),
                    str(event["end_reason"]),
                    str(event.get("post_startup_by_planning_success", 0)),
                    str(event.get("post_startup_by_frontier_positive", 0)),
                    str(event.get("post_startup_any", 0)),
                    str(event.get("post_startup_all", 0)),
                    str(event.get("pre_window_sample_count", 0)),
                    str(event.get("during_window_sample_count", 0)),
                    str(event.get("post_window_sample_count", 0)),
                ]
                for window_name in ("pre", "during", "post"):
                    for metric_name in (
                        "num_components",
                        "completion_ratio",
                        "frontier_completion_ratio",
                        "frontier_remaining",
                        "information_age",
                    ):
                        row.append(self.format_optional(event.get("{}_{}_mean".format(window_name, metric_name))))
                events_file.write("\t".join(row) + "\n")

    def write_report(self):
        if not self.report_path:
            return

        self.finalize_events()
        self.refresh_frontier_progress()
        self.refresh_runtime_metrics()

        report_dir = os.path.dirname(self.report_path)
        if report_dir:
            os.makedirs(report_dir, exist_ok=True)

        with open(self.report_path, "w", encoding="ascii") as report_file:
            report_file.write("\n".join(self.build_report_lines(finalized=True)) + "\n")

        self.write_events_file()


if __name__ == "__main__":

    rospy.init_node("relay_validation_monitor")
    RelayValidationMonitor()
    rospy.spin()
