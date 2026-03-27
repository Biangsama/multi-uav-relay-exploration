#!/usr/bin/env python3
import os
import time

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image


class PlannerSensorGate:
    def __init__(self):
        self.agent_id = int(rospy.get_param('~agent_id', 1))
        self.raw_pose_topic = rospy.get_param('~raw_pose_topic')
        self.raw_depth_topic = rospy.get_param('~raw_depth_topic')
        self.raw_odom_topic = rospy.get_param('~raw_odom_topic')
        self.gated_pose_topic = rospy.get_param('~gated_pose_topic')
        self.gated_depth_topic = rospy.get_param('~gated_depth_topic')
        self.stable_pair_count_required = int(rospy.get_param('~stable_pair_count', 5))
        self.sync_tolerance_sec = float(rospy.get_param('~sync_tolerance_sec', 0.02))
        self.max_sane_stamp_sec = int(rospy.get_param('~max_sane_stamp_sec', 1000000))
        self.sample_limit = int(rospy.get_param('~sample_limit', 40))
        self.log_path = rospy.get_param('~log_path', '')

        self.pose_pub = rospy.Publisher(self.gated_pose_topic, PoseStamped, queue_size=20)
        self.depth_pub = rospy.Publisher(self.gated_depth_topic, Image, queue_size=20)

        self.start_wall = time.time()
        self.odom_seen = False
        self.gate_open = False
        self.gate_open_wall_sec = None
        self.gate_open_stamp_ns = None

        self.raw_counts = {'odom': 0, 'pose': 0, 'depth': 0}
        self.drop_counts = {'pose': 0, 'depth': 0}
        self.bad_reason_counts = {}
        self.first_stamp_ns = {}
        self.last_stamp_ns = {'odom': None, 'pose': None, 'depth': None}
        self.last_forward_stamp_ns = {'pose': None, 'depth': None}
        self.latest_pose_msg = None
        self.latest_depth_msg = None
        self.latest_pose_ns = None
        self.latest_depth_ns = None
        self.stable_pair_count = 0
        self.last_counted_pair_ns = None
        self.samples = []

        rospy.Subscriber(self.raw_odom_topic, Odometry, self.odom_callback, queue_size=100)
        rospy.Subscriber(self.raw_pose_topic, PoseStamped, self.pose_callback, queue_size=100)
        rospy.Subscriber(self.raw_depth_topic, Image, self.depth_callback, queue_size=100)
        rospy.on_shutdown(self.on_shutdown)

        rospy.loginfo(
            'planner_sensor_gate started for agent=%d raw_pose=%s raw_depth=%s raw_odom=%s gated_pose=%s gated_depth=%s stable_pair_count=%d sync_tolerance_sec=%.6f',
            self.agent_id,
            self.raw_pose_topic,
            self.raw_depth_topic,
            self.raw_odom_topic,
            self.gated_pose_topic,
            self.gated_depth_topic,
            self.stable_pair_count_required,
            self.sync_tolerance_sec,
        )

    @staticmethod
    def stamp_to_ns(stamp):
        return int(stamp.secs) * 1000000000 + int(stamp.nsecs)

    @staticmethod
    def stamp_to_string(stamp):
        return f"{int(stamp.secs)}.{int(stamp.nsecs):09d}"

    def add_sample(self, topic_name, stamp, reason, extra=''):
        if len(self.samples) >= self.sample_limit:
            return
        suffix = f' {extra}' if extra else ''
        self.samples.append(
            f"topic={topic_name} stamp={self.stamp_to_string(stamp)} reason={reason}{suffix}"
        )

    def count_bad_reason(self, reason):
        self.bad_reason_counts[reason] = self.bad_reason_counts.get(reason, 0) + 1

    def classify_stamp(self, topic_name, stamp):
        if stamp.secs == 0 and stamp.nsecs == 0:
            return False, 'zero_stamp'
        if int(stamp.secs) > self.max_sane_stamp_sec:
            return False, 'huge_stamp'
        ns = self.stamp_to_ns(stamp)
        last_ns = self.last_stamp_ns[topic_name]
        if last_ns is not None and ns < last_ns:
            return False, 'non_monotonic'
        return True, 'ok'

    def odom_callback(self, msg):
        stamp = msg.header.stamp
        ns = self.stamp_to_ns(stamp)
        self.raw_counts['odom'] += 1
        if 'odom' not in self.first_stamp_ns:
            self.first_stamp_ns['odom'] = ns
        self.last_stamp_ns['odom'] = ns
        self.odom_seen = True
        self.add_sample('odom', stamp, 'ok')

    def pose_callback(self, msg):
        self.handle_sensor_msg('pose', msg, self.pose_pub)

    def depth_callback(self, msg):
        self.handle_sensor_msg('depth', msg, self.depth_pub)

    def handle_sensor_msg(self, topic_name, msg, pub):
        stamp = msg.header.stamp
        ns = self.stamp_to_ns(stamp)
        self.raw_counts[topic_name] += 1
        if topic_name not in self.first_stamp_ns:
            self.first_stamp_ns[topic_name] = ns

        sane, reason = self.classify_stamp(topic_name, stamp)
        odom_state = 'odom_seen=1' if self.odom_seen else 'odom_seen=0'
        self.add_sample(topic_name, stamp, reason, odom_state)

        if not sane:
            self.count_bad_reason(f'{topic_name}:{reason}')
        self.last_stamp_ns[topic_name] = ns

        if topic_name == 'pose':
            self.latest_pose_msg = msg
            self.latest_pose_ns = ns
        else:
            self.latest_depth_msg = msg
            self.latest_depth_ns = ns

        self.maybe_open_gate()

        if self.gate_open and sane and self.is_forward_monotonic(topic_name, ns):
            pub.publish(msg)
            self.last_forward_stamp_ns[topic_name] = ns
        else:
            self.drop_counts[topic_name] += 1

    def is_forward_monotonic(self, topic_name, ns):
        last_ns = self.last_forward_stamp_ns[topic_name]
        return last_ns is None or ns >= last_ns

    def maybe_open_gate(self):
        if self.gate_open or not self.odom_seen:
            return
        if self.latest_pose_msg is None or self.latest_depth_msg is None:
            return

        pose_stamp = self.latest_pose_msg.header.stamp
        depth_stamp = self.latest_depth_msg.header.stamp
        pose_ok, _ = self.classify_stamp('pose', pose_stamp)
        depth_ok, _ = self.classify_stamp('depth', depth_stamp)
        if not pose_ok or not depth_ok:
            return

        pair_dt = abs(self.latest_pose_ns - self.latest_depth_ns) / 1e9
        if pair_dt > self.sync_tolerance_sec:
            self.count_bad_reason('pair:unsynced')
            return

        pair_ns = max(self.latest_pose_ns, self.latest_depth_ns)
        if self.last_counted_pair_ns == pair_ns:
            return

        self.last_counted_pair_ns = pair_ns
        self.stable_pair_count += 1
        if self.stable_pair_count < self.stable_pair_count_required:
            return

        self.gate_open = True
        self.gate_open_wall_sec = time.time() - self.start_wall
        self.gate_open_stamp_ns = pair_ns
        self.pose_pub.publish(self.latest_pose_msg)
        self.depth_pub.publish(self.latest_depth_msg)
        self.last_forward_stamp_ns['pose'] = self.latest_pose_ns
        self.last_forward_stamp_ns['depth'] = self.latest_depth_ns
        rospy.loginfo(
            'planner_sensor_gate opened for agent=%d after stable_pair_count=%d wall_sec=%.3f gate_stamp=%s',
            self.agent_id,
            self.stable_pair_count,
            self.gate_open_wall_sec,
            self.stamp_to_string(self.latest_pose_msg.header.stamp),
        )

    def on_shutdown(self):
        if not self.log_path:
            return
        try:
            parent = os.path.dirname(self.log_path)
            if parent:
                os.makedirs(parent, exist_ok=True)
            with open(self.log_path, 'w') as f:
                def write_kv(key, value):
                    f.write(f'{key}={value}\\n')
                write_kv('agent_id', self.agent_id)
                write_kv('raw_odom_count', self.raw_counts['odom'])
                write_kv('raw_pose_count', self.raw_counts['pose'])
                write_kv('raw_depth_count', self.raw_counts['depth'])
                write_kv('drop_pose_count', self.drop_counts['pose'])
                write_kv('drop_depth_count', self.drop_counts['depth'])
                write_kv('stable_pair_count', self.stable_pair_count)
                write_kv('stable_pair_count_required', self.stable_pair_count_required)
                write_kv('gate_open', int(self.gate_open))
                write_kv('gate_open_wall_sec', '' if self.gate_open_wall_sec is None else f'{self.gate_open_wall_sec:.6f}')
                write_kv('gate_open_stamp', '' if self.gate_open_stamp_ns is None else f'{self.gate_open_stamp_ns // 1000000000}.{self.gate_open_stamp_ns % 1000000000:09d}')
                write_kv('first_odom_stamp', '' if 'odom' not in self.first_stamp_ns else f'{self.first_stamp_ns["odom"] // 1000000000}.{self.first_stamp_ns["odom"] % 1000000000:09d}')
                write_kv('first_pose_stamp', '' if 'pose' not in self.first_stamp_ns else f'{self.first_stamp_ns["pose"] // 1000000000}.{self.first_stamp_ns["pose"] % 1000000000:09d}')
                write_kv('first_depth_stamp', '' if 'depth' not in self.first_stamp_ns else f'{self.first_stamp_ns["depth"] // 1000000000}.{self.first_stamp_ns["depth"] % 1000000000:09d}')
                for key in sorted(self.bad_reason_counts):
                    write_kv(f'bad_reason_{key.replace(":", "_")}', self.bad_reason_counts[key])
                f.write('samples_begin\\n')
                for sample in self.samples:
                    f.write(sample + '\\n')
                f.write('samples_end\\n')
        except Exception as exc:
            rospy.logwarn('planner_sensor_gate failed to write log: %s', exc)


if __name__ == '__main__':
    rospy.init_node('planner_sensor_gate')
    PlannerSensorGate()
    rospy.spin()
