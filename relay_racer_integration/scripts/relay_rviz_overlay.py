#!/usr/bin/env python3
import copy
import math
import time
from collections import defaultdict

import rospy
from geometry_msgs.msg import Point, PoseArray
from relay_racer_integration.msg import RelayRoleCmd, SwarmCommState
from visualization_msgs.msg import Marker, MarkerArray


ROLE_LABELS = {
    RelayRoleCmd.ROLE_EXPLORER: "EXPLORER",
    RelayRoleCmd.ROLE_RELAY: "RELAY",
}

COMPONENT_COLORS = [
    (0.18, 0.67, 1.00),
    (0.00, 0.82, 0.55),
    (1.00, 0.62, 0.10),
    (1.00, 0.35, 0.35),
    (0.72, 0.45, 1.00),
    (1.00, 0.85, 0.20),
]


class RelayRvizOverlay:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.relay_role_topic = rospy.get_param(
            "~relay_role_topic", "/relay_integration/relay_role_cmd"
        )
        self.swarm_comm_state_topic = rospy.get_param(
            "~swarm_comm_state_topic", "/relay_integration/swarm_comm_state"
        )
        self.agent_pose_topic = rospy.get_param("~agent_pose_topic", "/agent_poses")
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 5.0))
        self.marker_lifetime_sec = float(rospy.get_param("~marker_lifetime_sec", 0.6))
        self.stale_timeout_sec = float(rospy.get_param("~stale_timeout_sec", 2.0))
        self.relay_hold_sec = float(rospy.get_param("~relay_hold_sec", 5.0))
        self.role_text_height = float(rospy.get_param("~role_text_height", 1.2))
        self.target_text_height = float(rospy.get_param("~target_text_height", 0.8))
        self.line_width = float(rospy.get_param("~line_width", 0.12))
        self.target_radius = float(rospy.get_param("~target_radius", 0.5))
        self.comm_line_width = float(rospy.get_param("~comm_line_width", 0.08))
        self.component_radius = float(rospy.get_param("~component_radius", 0.45))
        self.component_text_height = float(rospy.get_param("~component_text_height", 0.7))
        self.bridge_line_width = float(rospy.get_param("~bridge_line_width", 0.09))
        self.bridge_gap_text_height = float(rospy.get_param("~bridge_gap_text_height", 0.65))

        self._latest_role_cmd = {}
        self._latest_comm_state = None
        self._latest_pose_array = None
        self._agent_positions = {}
        self._last_active_relay_cmd = None
        self._last_active_relay_wall_time = None

        self.role_marker_pub = rospy.Publisher(
            "/relay_integration/role_markers", MarkerArray, queue_size=1, latch=True
        )
        self.target_marker_pub = rospy.Publisher(
            "/relay_integration/relay_target_markers", MarkerArray, queue_size=1, latch=True
        )
        self.comm_links_pub = rospy.Publisher(
            "/relay_integration/comm_state_links", Marker, queue_size=1, latch=True
        )

        rospy.Subscriber(self.relay_role_topic, RelayRoleCmd, self._relay_role_callback, queue_size=20)
        rospy.Subscriber(
            self.swarm_comm_state_topic, SwarmCommState, self._swarm_comm_state_callback, queue_size=20
        )
        rospy.Subscriber(self.agent_pose_topic, PoseArray, self._agent_pose_callback, queue_size=20)

        rospy.Timer(rospy.Duration(1.0 / max(self.publish_rate_hz, 1.0)), self._publish_markers)

        rospy.loginfo(
            "[relay_rviz_overlay] frame_id=%s relay_role_topic=%s swarm_comm_state_topic=%s agent_pose_topic=%s relay_hold_sec=%.2f",
            self.frame_id,
            self.relay_role_topic,
            self.swarm_comm_state_topic,
            self.agent_pose_topic,
            self.relay_hold_sec,
        )

    def _relay_role_callback(self, msg):
        self._latest_role_cmd[msg.agent_id] = msg
        if msg.role == RelayRoleCmd.ROLE_RELAY and msg.valid:
            self._last_active_relay_cmd = copy.deepcopy(msg)
            self._last_active_relay_wall_time = time.monotonic()
        self._publish_markers(None)

    def _swarm_comm_state_callback(self, msg):
        self._latest_comm_state = msg
        self._refresh_agent_positions()
        self._publish_markers(None)

    def _agent_pose_callback(self, msg):
        self._latest_pose_array = msg
        self._refresh_agent_positions()
        self._publish_markers(None)

    @staticmethod
    def _copy_point(point):
        copied = Point()
        copied.x = point.x
        copied.y = point.y
        copied.z = point.z
        return copied

    @staticmethod
    def _make_point(x, y, z):
        point = Point()
        point.x = x
        point.y = y
        point.z = z
        return point

    @staticmethod
    def _distance(p1, p2):
        return math.sqrt(
            (p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2 + (p1.z - p2.z) ** 2
        )

    @staticmethod
    def _midpoint(p1, p2):
        return RelayRvizOverlay._make_point(
            0.5 * (p1.x + p2.x),
            0.5 * (p1.y + p2.y),
            0.5 * (p1.z + p2.z),
        )

    @staticmethod
    def _component_color(component_id):
        return COMPONENT_COLORS[int(component_id) % len(COMPONENT_COLORS)]

    def _refresh_agent_positions(self):
        if self._latest_comm_state is None or self._latest_pose_array is None:
            return

        agent_ids = list(self._latest_comm_state.agent_ids)
        poses = list(self._latest_pose_array.poses)
        if len(agent_ids) != len(poses):
            rospy.logwarn_throttle(
                2.0,
                "[relay_rviz_overlay] agent_ids size (%d) != pose count (%d); skip pose/id alignment",
                len(agent_ids),
                len(poses),
            )
            return

        positions = {}
        for agent_id, pose in zip(agent_ids, poses):
            positions[int(agent_id)] = self._copy_point(pose.position)
        self._agent_positions = positions

    def _publish_markers(self, _event):
        self.role_marker_pub.publish(self._build_role_markers())
        self.target_marker_pub.publish(self._build_target_markers())
        self.comm_links_pub.publish(self._build_comm_state_links())

    def _current_held_relay(self):
        if self._last_active_relay_cmd is None or self._last_active_relay_wall_time is None:
            return None
        if (time.monotonic() - self._last_active_relay_wall_time) > self.relay_hold_sec:
            return None
        return self._last_active_relay_cmd

    def _effective_cmd_for_agent(self, agent_id):
        cmd = self._latest_role_cmd.get(agent_id)
        if cmd is not None and not self._is_stale(cmd.header.stamp):
            return cmd, False

        held = self._current_held_relay()
        if held is not None and int(held.agent_id) == int(agent_id):
            return held, True

        return cmd, False

    def _component_map(self):
        component_map = {}
        if self._latest_comm_state is None:
            return component_map
        for agent_id, component_id in zip(
            self._latest_comm_state.agent_ids, self._latest_comm_state.component_ids
        ):
            component_map[int(agent_id)] = int(component_id)
        return component_map

    def _component_centroids(self):
        component_map = self._component_map()
        grouped = defaultdict(list)
        for agent_id, point in self._agent_positions.items():
            component_id = component_map.get(int(agent_id))
            if component_id is None:
                continue
            grouped[int(component_id)].append(self._copy_point(point))

        centroids = {}
        counts = {}
        for component_id, points in grouped.items():
            if not points:
                continue
            counts[component_id] = len(points)
            centroids[component_id] = self._make_point(
                sum(point.x for point in points) / float(len(points)),
                sum(point.y for point in points) / float(len(points)),
                sum(point.z for point in points) / float(len(points)),
            )
        return centroids, counts

    def _select_bridge_components(self, relay_cmd, centroids):
        if relay_cmd is None or len(centroids) < 2:
            return None

        target = relay_cmd.relay_target
        ranked = sorted(
            centroids.items(),
            key=lambda item: (self._distance(item[1], target), int(item[0])),
        )
        if len(ranked) < 2:
            return None

        (comp_a, centroid_a), (comp_b, centroid_b) = ranked[0], ranked[1]
        return {
            "comp_a": int(comp_a),
            "comp_b": int(comp_b),
            "centroid_a": self._copy_point(centroid_a),
            "centroid_b": self._copy_point(centroid_b),
            "gap_m": self._distance(centroid_a, centroid_b),
            "target_to_a_m": self._distance(target, centroid_a),
            "target_to_b_m": self._distance(target, centroid_b),
        }

    def _build_role_markers(self):
        array = MarkerArray()
        array.markers.append(self._delete_all_marker("role_cleanup", 0))

        component_map = self._component_map()
        relay_cmd, _ = self._select_target_cmd()
        centroids, _ = self._component_centroids()
        bridge_info = self._select_bridge_components(relay_cmd, centroids)

        for agent_id in sorted(self._agent_positions.keys()):
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = "relay_roles"
            marker.id = int(agent_id)
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.scale.z = self.role_text_height
            marker.pose.position = self._copy_point(self._agent_positions[agent_id])
            marker.pose.position.z += 1.8
            marker.pose.orientation.w = 1.0
            marker.lifetime = rospy.Duration(self.marker_lifetime_sec)

            cmd, held = self._effective_cmd_for_agent(agent_id)
            role_name = "UNKNOWN"
            bridge_suffix = ""
            if cmd is None or (not held and self._is_stale(cmd.header.stamp)):
                marker.color.r = 0.6
                marker.color.g = 0.6
                marker.color.b = 0.6
                marker.color.a = 0.9
            else:
                role_name = ROLE_LABELS.get(cmd.role, f"ROLE_{cmd.role}")
                if held:
                    role_name += "*"
                if cmd.role == RelayRoleCmd.ROLE_RELAY and cmd.valid:
                    marker.color.r = 1.0
                    marker.color.g = 0.45
                    marker.color.b = 0.0
                    marker.color.a = 1.0
                    if bridge_info is not None:
                        bridge_suffix = (
                            f" bridge C{bridge_info['comp_a']}<->C{bridge_info['comp_b']}"
                        )
                else:
                    marker.color.r = 0.1
                    marker.color.g = 0.85
                    marker.color.b = 0.25
                    marker.color.a = 1.0

            component_suffix = ""
            if agent_id in component_map:
                component_suffix = f" C{component_map[agent_id]}"
            marker.text = f"{agent_id} {role_name}{component_suffix}{bridge_suffix}"
            array.markers.append(marker)

        return array

    def _append_component_markers(self, array, centroids, component_sizes):
        for component_id in sorted(centroids.keys()):
            centroid = centroids[component_id]
            color_r, color_g, color_b = self._component_color(component_id)

            sphere = Marker()
            sphere.header.frame_id = self.frame_id
            sphere.header.stamp = rospy.Time.now()
            sphere.ns = "relay_components"
            sphere.id = 1000 + int(component_id)
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position = self._copy_point(centroid)
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = self.component_radius
            sphere.scale.y = self.component_radius
            sphere.scale.z = self.component_radius
            sphere.lifetime = rospy.Duration(self.marker_lifetime_sec)
            sphere.color.r = color_r
            sphere.color.g = color_g
            sphere.color.b = color_b
            sphere.color.a = 0.55
            array.markers.append(sphere)

            label = Marker()
            label.header.frame_id = self.frame_id
            label.header.stamp = rospy.Time.now()
            label.ns = "relay_component_labels"
            label.id = 2000 + int(component_id)
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position = self._copy_point(centroid)
            label.pose.position.z += 1.0
            label.pose.orientation.w = 1.0
            label.scale.z = self.component_text_height
            label.lifetime = rospy.Duration(self.marker_lifetime_sec)
            label.color.r = color_r
            label.color.g = color_g
            label.color.b = color_b
            label.color.a = 0.95
            label.text = f"C{component_id} ({component_sizes.get(component_id, 0)})"
            array.markers.append(label)

    def _build_target_markers(self):
        array = MarkerArray()
        array.markers.append(self._delete_all_marker("target_cleanup", 0))

        centroids, component_sizes = self._component_centroids()
        self._append_component_markers(array, centroids, component_sizes)

        role_cmd, held = self._select_target_cmd()
        if role_cmd is None:
            return array

        target = self._copy_point(role_cmd.relay_target)
        bridge_info = self._select_bridge_components(role_cmd, centroids)

        sphere = Marker()
        sphere.header.frame_id = self.frame_id
        sphere.header.stamp = rospy.Time.now()
        sphere.ns = "relay_target"
        sphere.id = 1
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position = self._copy_point(target)
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = self.target_radius
        sphere.scale.y = self.target_radius
        sphere.scale.z = self.target_radius
        sphere.lifetime = rospy.Duration(self.marker_lifetime_sec)
        sphere.color.r = 1.0
        sphere.color.g = 0.45
        sphere.color.b = 0.0
        sphere.color.a = 0.9
        array.markers.append(sphere)

        label = Marker()
        label.header.frame_id = self.frame_id
        label.header.stamp = rospy.Time.now()
        label.ns = "relay_target_label"
        label.id = 2
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position = self._copy_point(target)
        label.pose.position.z += 1.2
        label.pose.orientation.w = 1.0
        label.scale.z = self.target_text_height
        label.lifetime = rospy.Duration(self.marker_lifetime_sec)
        label.color.r = sphere.color.r
        label.color.g = sphere.color.g
        label.color.b = sphere.color.b
        label.color.a = 1.0
        if bridge_info is None:
            label.text = f"relay#{int(role_cmd.agent_id)}{'*' if held else ''}"
        else:
            label.text = (
                f"relay#{int(role_cmd.agent_id)}{'*' if held else ''} "
                f"C{bridge_info['comp_a']}<->C{bridge_info['comp_b']} "
                f"gap={bridge_info['gap_m']:.1f}m"
            )
        array.markers.append(label)

        if role_cmd.agent_id in self._agent_positions:
            line = Marker()
            line.header.frame_id = self.frame_id
            line.header.stamp = rospy.Time.now()
            line.ns = "relay_target_line"
            line.id = int(role_cmd.agent_id)
            line.type = Marker.LINE_LIST
            line.action = Marker.ADD
            line.scale.x = self.line_width
            line.lifetime = rospy.Duration(self.marker_lifetime_sec)
            line.color.r = 1.0
            line.color.g = 0.45
            line.color.b = 0.0
            line.color.a = 0.95
            line.points = [self._copy_point(self._agent_positions[role_cmd.agent_id]), self._copy_point(target)]
            array.markers.append(line)

        if bridge_info is None:
            return array

        rays = Marker()
        rays.header.frame_id = self.frame_id
        rays.header.stamp = rospy.Time.now()
        rays.ns = "relay_bridge_rays"
        rays.id = 3001
        rays.type = Marker.LINE_LIST
        rays.action = Marker.ADD
        rays.scale.x = self.bridge_line_width
        rays.lifetime = rospy.Duration(self.marker_lifetime_sec)
        rays.color.r = 1.0
        rays.color.g = 0.85
        rays.color.b = 0.15
        rays.color.a = 0.95
        rays.points = [
            self._copy_point(target),
            self._copy_point(bridge_info["centroid_a"]),
            self._copy_point(target),
            self._copy_point(bridge_info["centroid_b"]),
        ]
        array.markers.append(rays)

        gap_line = Marker()
        gap_line.header.frame_id = self.frame_id
        gap_line.header.stamp = rospy.Time.now()
        gap_line.ns = "relay_bridge_gap"
        gap_line.id = 3002
        gap_line.type = Marker.LINE_LIST
        gap_line.action = Marker.ADD
        gap_line.scale.x = self.bridge_line_width * 1.15
        gap_line.lifetime = rospy.Duration(self.marker_lifetime_sec)
        gap_line.color.r = 1.0
        gap_line.color.g = 0.0
        gap_line.color.b = 0.8
        gap_line.color.a = 0.9
        gap_line.points = [
            self._copy_point(bridge_info["centroid_a"]),
            self._copy_point(bridge_info["centroid_b"]),
        ]
        array.markers.append(gap_line)

        gap_label = Marker()
        gap_label.header.frame_id = self.frame_id
        gap_label.header.stamp = rospy.Time.now()
        gap_label.ns = "relay_bridge_gap_label"
        gap_label.id = 3003
        gap_label.type = Marker.TEXT_VIEW_FACING
        gap_label.action = Marker.ADD
        gap_label.pose.position = self._midpoint(
            bridge_info["centroid_a"], bridge_info["centroid_b"]
        )
        gap_label.pose.position.z += 1.4
        gap_label.pose.orientation.w = 1.0
        gap_label.scale.z = self.bridge_gap_text_height
        gap_label.lifetime = rospy.Duration(self.marker_lifetime_sec)
        gap_label.color.r = 1.0
        gap_label.color.g = 0.0
        gap_label.color.b = 0.8
        gap_label.color.a = 0.95
        gap_label.text = (
            f"bridge C{bridge_info['comp_a']}<->C{bridge_info['comp_b']} "
            f"gap={bridge_info['gap_m']:.1f}m"
        )
        array.markers.append(gap_label)

        return array

    def _build_comm_state_links(self):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "comm_state_links"
        marker.id = 1
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = self.comm_line_width
        marker.lifetime = rospy.Duration(self.marker_lifetime_sec)
        marker.color.r = 0.0
        marker.color.g = 0.9
        marker.color.b = 0.95
        marker.color.a = 0.95

        if self._latest_comm_state is None:
            return marker

        seen = set()
        for src, dst, up in zip(
            self._latest_comm_state.link_src_ids,
            self._latest_comm_state.link_dst_ids,
            self._latest_comm_state.link_up,
        ):
            if not up:
                continue
            src = int(src)
            dst = int(dst)
            if src == dst:
                continue
            if src not in self._agent_positions or dst not in self._agent_positions:
                continue
            edge = tuple(sorted((src, dst)))
            if edge in seen:
                continue
            seen.add(edge)
            marker.points.append(self._copy_point(self._agent_positions[src]))
            marker.points.append(self._copy_point(self._agent_positions[dst]))

        return marker

    def _select_target_cmd(self):
        fresh_cmds = [
            cmd
            for cmd in self._latest_role_cmd.values()
            if not self._is_stale(cmd.header.stamp)
        ]
        active_relays = [
            cmd
            for cmd in fresh_cmds
            if cmd.role == RelayRoleCmd.ROLE_RELAY and cmd.valid
        ]
        if active_relays:
            return sorted(active_relays, key=lambda cmd: cmd.agent_id)[0], False

        held = self._current_held_relay()
        if held is not None:
            return held, True

        return None, False

    def _is_stale(self, stamp):
        if stamp is None or stamp.is_zero():
            return True
        return (rospy.Time.now() - stamp).to_sec() > self.stale_timeout_sec

    def _delete_all_marker(self, ns, marker_id):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = ns
        marker.id = marker_id
        marker.action = Marker.DELETEALL
        return marker


if __name__ == "__main__":
    rospy.init_node("relay_rviz_overlay")
    RelayRvizOverlay()
    rospy.spin()
