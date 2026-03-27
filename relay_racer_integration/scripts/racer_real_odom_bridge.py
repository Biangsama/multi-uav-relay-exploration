#!/usr/bin/env python3

import math

import rospy
from dancers_msgs.msg import AgentStruct, AgentStructArray
from geometry_msgs.msg import Pose, PoseArray, Point, Quaternion
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray


class RacerRealOdomBridge:
    def __init__(self):
        self.racer_ids = self._normalize_int_list(
            rospy.get_param("~racer_ids", [1, 2, 3, 4]), default=[1, 2, 3, 4]
        )
        self.id_offset = int(rospy.get_param("~id_offset", -1))
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.odom_topic_prefix = rospy.get_param("~odom_topic_prefix", "/relay_odom_")
        self.agent_pose_topic = rospy.get_param(
            "~agent_pose_topic", "/agent_poses"
        )
        self.id_marker_topic = rospy.get_param(
            "~id_marker_topic", "/id_markers"
        )
        self.agent_states_topic = rospy.get_param(
            "~agent_states_topic", "/relay_integration/platform_agent_structs"
        )
        self.publish_rate_hz = max(1.0, float(rospy.get_param("~publish_rate_hz", 10.0)))
        self.label_z_offset = float(rospy.get_param("~label_z_offset", 1.0))
        self.label_scale_z = float(rospy.get_param("~label_scale_z", 1.1))
        self.marker_lifetime_sec = float(rospy.get_param("~marker_lifetime_sec", 0.5))

        self._latest_odom_by_racer_id = {}

        self.agent_pose_pub = rospy.Publisher(
            self.agent_pose_topic, PoseArray, queue_size=1, latch=True
        )
        self.id_marker_pub = rospy.Publisher(
            self.id_marker_topic, MarkerArray, queue_size=1, latch=True
        )
        self.agent_states_pub = rospy.Publisher(
            self.agent_states_topic, AgentStructArray, queue_size=1, latch=True
        )

        for racer_id in self.racer_ids:
            rospy.Subscriber(
                "{}{}".format(self.odom_topic_prefix, racer_id),
                Odometry,
                self._odom_callback,
                callback_args=racer_id,
                queue_size=20,
            )

        rospy.Timer(rospy.Duration(1.0 / self.publish_rate_hz), self._publish)

        rospy.loginfo(
            "[racer_real_odom_bridge] racer_ids=%s id_offset=%d odom_topic_prefix=%s agent_pose_topic=%s id_marker_topic=%s agent_states_topic=%s",
            self.racer_ids,
            self.id_offset,
            self.odom_topic_prefix,
            self.agent_pose_topic,
            self.id_marker_topic,
            self.agent_states_topic,
        )

    @staticmethod
    def _normalize_int_list(value, default=None):
        if default is None:
            default = []

        if isinstance(value, int):
            return [int(value)]

        if isinstance(value, str):
            items = [item.strip() for item in value.split(",") if item.strip()]
            return [int(item) for item in items] if items else list(default)

        if isinstance(value, (list, tuple)):
            items = [int(item) for item in value if item is not None]
            return items if items else list(default)

        return list(default)

    @staticmethod
    def _heading_from_quaternion(orientation):
        siny_cosp = 2.0 * (
            orientation.w * orientation.z + orientation.x * orientation.y
        )
        cosy_cosp = 1.0 - 2.0 * (
            orientation.y * orientation.y + orientation.z * orientation.z
        )
        return math.atan2(siny_cosp, cosy_cosp)

    def _platform_id_from_racer_id(self, racer_id):
        return int(racer_id) + self.id_offset

    def _resolved_frame_id(self, odom_msg):
        if self.frame_id:
            return self.frame_id
        if odom_msg.header.frame_id:
            return odom_msg.header.frame_id
        return "map"

    @staticmethod
    def _copy_point(point):
        copied = Point()
        copied.x = point.x
        copied.y = point.y
        copied.z = point.z
        return copied

    @classmethod
    def _copy_pose(cls, pose):
        copied = Pose()
        copied.position = cls._copy_point(pose.position)
        copied.orientation = Quaternion(
            x=pose.orientation.x,
            y=pose.orientation.y,
            z=pose.orientation.z,
            w=pose.orientation.w,
        )
        return copied

    def _odom_callback(self, msg, racer_id):
        self._latest_odom_by_racer_id[int(racer_id)] = msg

    def _publish(self, _event):
        if not self._latest_odom_by_racer_id:
            return

        self._publish_agent_states()
        self._publish_pose_array_and_markers_if_ready()

    def _publish_agent_states(self):
        agent_states = AgentStructArray()

        for racer_id in self.racer_ids:
            odom_msg = self._latest_odom_by_racer_id.get(racer_id)
            if odom_msg is None:
                continue

            agent_msg = AgentStruct()
            agent_msg.agent_id = self._platform_id_from_racer_id(racer_id)
            agent_msg.agent_role = AgentStruct.AGENT_ROLE_UNDEFINED
            agent_msg.state.position.x = odom_msg.pose.pose.position.x
            agent_msg.state.position.y = odom_msg.pose.pose.position.y
            agent_msg.state.position.z = odom_msg.pose.pose.position.z
            agent_msg.state.velocity.x = odom_msg.twist.twist.linear.x
            agent_msg.state.velocity.y = odom_msg.twist.twist.linear.y
            agent_msg.state.velocity.z = odom_msg.twist.twist.linear.z
            agent_msg.state.heading = self._heading_from_quaternion(
                odom_msg.pose.pose.orientation
            )
            agent_states.structs.append(agent_msg)

        if agent_states.structs:
            self.agent_states_pub.publish(agent_states)

    def _publish_pose_array_and_markers_if_ready(self):
        if any(racer_id not in self._latest_odom_by_racer_id for racer_id in self.racer_ids):
            return

        pose_array = PoseArray()
        marker_array = MarkerArray()

        latest_stamp = rospy.Time(0)
        frame_id = self.frame_id

        cleanup_marker = Marker()
        cleanup_marker.header.frame_id = frame_id if frame_id else "map"
        cleanup_marker.header.stamp = rospy.Time.now()
        cleanup_marker.ns = "relay_real_id_markers_cleanup"
        cleanup_marker.id = 0
        cleanup_marker.action = Marker.DELETEALL
        marker_array.markers.append(cleanup_marker)

        for racer_id in self.racer_ids:
            odom_msg = self._latest_odom_by_racer_id[racer_id]
            if odom_msg.header.stamp > latest_stamp:
                latest_stamp = odom_msg.header.stamp
            if not frame_id:
                frame_id = self._resolved_frame_id(odom_msg)

            pose_copy = self._copy_pose(odom_msg.pose.pose)
            pose_array.poses.append(pose_copy)

            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = "relay_real_id_markers"
            marker.id = int(racer_id)
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position = self._copy_point(odom_msg.pose.pose.position)
            marker.pose.position.z += self.label_z_offset
            marker.pose.orientation.w = 1.0
            marker.scale.z = self.label_scale_z
            marker.color.r = 0.10
            marker.color.g = 0.70
            marker.color.b = 1.00
            marker.color.a = 1.00
            marker.text = str(racer_id)
            marker.lifetime = rospy.Duration(self.marker_lifetime_sec)
            marker_array.markers.append(marker)

        pose_array.header.frame_id = frame_id if frame_id else "map"
        pose_array.header.stamp = latest_stamp if latest_stamp != rospy.Time(0) else rospy.Time.now()

        self.agent_pose_pub.publish(pose_array)
        self.id_marker_pub.publish(marker_array)


if __name__ == "__main__":
    rospy.init_node("racer_real_odom_bridge")
    RacerRealOdomBridge()
    rospy.spin()
