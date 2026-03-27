#!/usr/bin/env python3
import argparse
import json
import math
import time
from collections import defaultdict

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import UInt8MultiArray

from bspline.msg import Bspline
from exploration_manager.msg import DroneState, PairOpt, PairOptResponse
from plan_env.msg import ChunkData, ChunkStamps, IdxList


class StageCTransportProbe:
    def __init__(self, args):
        self.args = args
        self.tx_bytes_count = 0
        self.rx_bytes_count = 0
        self.state = {}
        self.family_specs = self._build_family_specs()

        rospy.init_node("stagec_transport_probe", anonymous=True)
        rospy.Subscriber(
            "/relay_integration/racer_swarm_tx_bytes",
            UInt8MultiArray,
            self._tx_bytes_callback,
            queue_size=200,
        )
        rospy.Subscriber(
            "/relay_integration/racer_swarm_rx_bytes",
            UInt8MultiArray,
            self._rx_bytes_callback,
            queue_size=200,
        )

        self.publishers = {}
        self._setup_family_subscribers()
        self._setup_publishers()

    def _build_family_specs(self):
        return {
            "drone_state": {
                "msg_type": DroneState,
                "send_topic": f"/swarm_expl/drone_state_send_{self.args.src_id}",
                "recv_topics": [
                    f"/swarm_expl/drone_state_recv_{dst}"
                    for dst in self.args.recv_ids
                    if dst != self.args.src_id
                ],
                "message": self._make_drone_state_probe(),
                "matcher": self._match_drone_state,
                "summarizer": self._summarize_drone_state,
            },
            "pair_opt": {
                "msg_type": PairOpt,
                "send_topic": f"/swarm_expl/pair_opt_send_{self.args.src_id}",
                "recv_topics": [
                    f"/swarm_expl/pair_opt_recv_{dst}"
                    for dst in self.args.recv_ids
                    if dst != self.args.src_id
                ],
                "message": self._make_pair_opt_probe(),
                "matcher": self._match_pair_opt,
                "summarizer": self._summarize_pair_opt,
            },
            "pair_opt_res": {
                "msg_type": PairOptResponse,
                "send_topic": f"/swarm_expl/pair_opt_res_send_{self.args.res_src_id}",
                "recv_topics": [
                    f"/swarm_expl/pair_opt_res_recv_{dst}"
                    for dst in self.args.recv_ids
                    if dst != self.args.res_src_id
                ],
                "message": self._make_pair_opt_res_probe(),
                "matcher": self._match_pair_opt_res,
                "summarizer": self._summarize_pair_opt_res,
            },
            "swarm_traj": {
                "msg_type": Bspline,
                "send_topic": f"/planning/swarm_traj_send_{self.args.src_id}",
                "recv_topics": [
                    f"/planning/swarm_traj_recv_{dst}"
                    for dst in self.args.recv_ids
                    if dst != self.args.src_id
                ],
                "message": self._make_swarm_traj_probe(),
                "matcher": self._match_swarm_traj,
                "summarizer": self._summarize_swarm_traj,
            },
            "chunk_stamps": {
                "msg_type": ChunkStamps,
                "send_topic": f"/multi_map_manager/chunk_stamps_send_{self.args.src_id}",
                "recv_topics": [
                    f"/multi_map_manager/chunk_stamps_recv_{dst}"
                    for dst in self.args.recv_ids
                    if dst != self.args.src_id
                ],
                "message": self._make_chunk_stamps_probe(),
                "matcher": self._match_chunk_stamps,
                "summarizer": self._summarize_chunk_stamps,
            },
            "chunk_data": {
                "msg_type": ChunkData,
                "send_topic": f"/multi_map_manager/chunk_data_send_{self.args.src_id}",
                "recv_topics": [
                    f"/multi_map_manager/chunk_data_recv_{dst}"
                    for dst in self.args.recv_ids
                    if dst != self.args.src_id
                ],
                "message": self._make_chunk_data_probe(),
                "matcher": self._match_chunk_data,
                "summarizer": self._summarize_chunk_data,
            },
        }

    def _setup_family_subscribers(self):
        for family in self.args.families:
            spec = self.family_specs[family]
            self.state[family] = {
                "total_recv": 0,
                "matched_recv": 0,
                "matched_topics": defaultdict(int),
                "last_match": None,
            }
            for topic in spec["recv_topics"]:
                rospy.Subscriber(
                    topic,
                    spec["msg_type"],
                    self._make_recv_callback(family, topic),
                    queue_size=200,
                )

    def _setup_publishers(self):
        for family in self.args.families:
            spec = self.family_specs[family]
            self.publishers[family] = rospy.Publisher(
                spec["send_topic"], spec["msg_type"], queue_size=50
            )

    def _make_recv_callback(self, family, topic):
        spec = self.family_specs[family]

        def callback(msg):
            family_state = self.state[family]
            family_state["total_recv"] += 1
            if spec["matcher"](msg):
                family_state["matched_recv"] += 1
                family_state["matched_topics"][topic] += 1
                family_state["last_match"] = spec["summarizer"](msg)

        return callback

    def _tx_bytes_callback(self, _msg):
        self.tx_bytes_count += 1

    def _rx_bytes_callback(self, _msg):
        self.rx_bytes_count += 1

    def _wait_for_connections(self):
        deadline = time.time() + self.args.connection_wait
        while time.time() < deadline and not rospy.is_shutdown():
            ready = True
            for family in self.args.families:
                if self.publishers[family].get_num_connections() == 0:
                    ready = False
                    break
            if ready:
                return True
            time.sleep(0.1)
        return False

    def _publish_probe_messages(self, families):
        for family in families:
            spec = self.family_specs[family]
            pub = self.publishers[family]
            for _ in range(self.args.probe_count):
                pub.publish(spec["message"])
                time.sleep(self.args.publish_interval)

    def _pending_families(self):
        return [
            family for family in self.args.families if self.state[family]["matched_recv"] == 0
        ]

    def run(self):
        if self.args.startup_delay > 0:
            time.sleep(self.args.startup_delay)

        connected = self._wait_for_connections()
        time.sleep(self.args.pre_publish_settle)

        pending_families = list(self.args.families)
        rounds_used = 0
        while pending_families and rounds_used < self.args.rounds and not rospy.is_shutdown():
            self._publish_probe_messages(pending_families)
            time.sleep(self.args.post_publish_wait)
            pending_families = self._pending_families()
            rounds_used += 1

        result = {
            "connected": connected,
            "rounds_used": rounds_used,
            "pending_after_rounds": pending_families,
            "tx_bytes_count": self.tx_bytes_count,
            "rx_bytes_count": self.rx_bytes_count,
            "families": {},
        }
        success = True
        for family in self.args.families:
            family_state = self.state[family]
            family_result = {
                "total_recv": family_state["total_recv"],
                "matched_recv": family_state["matched_recv"],
                "matched_topics": dict(family_state["matched_topics"]),
                "last_match": family_state["last_match"],
                "success": family_state["matched_recv"] > 0,
            }
            result["families"][family] = family_result
            success = success and family_result["success"]

        result["success"] = success
        print(json.dumps(result, sort_keys=True, indent=2))
        return 0 if success else 1

    def _make_drone_state_probe(self):
        msg = DroneState()
        msg.drone_id = self.args.src_id
        msg.grid_ids = [91, 92, 93]
        msg.recent_attempt_time = 910002.0
        msg.stamp = 910001.0
        msg.pos = [91.0, 92.0, 93.0]
        msg.vel = [0.91, 0.92, 0.93]
        msg.yaw = 0.915
        return msg

    def _match_drone_state(self, msg):
        return (
            msg.drone_id == self.args.src_id
            and list(msg.grid_ids) == [91, 92, 93]
            and self._close(msg.stamp, 910001.0)
        )

    def _summarize_drone_state(self, msg):
        return {
            "drone_id": msg.drone_id,
            "grid_ids": list(msg.grid_ids),
            "stamp": msg.stamp,
            "yaw": msg.yaw,
        }

    def _make_pair_opt_probe(self):
        msg = PairOpt()
        msg.from_drone_id = self.args.src_id
        msg.to_drone_id = self.args.logical_target_id
        msg.stamp = 920001.0
        msg.ego_ids = [11, 12, 13]
        msg.other_ids = [21, 22, 23]
        return msg

    def _match_pair_opt(self, msg):
        return (
            msg.from_drone_id == self.args.src_id
            and msg.to_drone_id == self.args.logical_target_id
            and self._close(msg.stamp, 920001.0)
            and list(msg.ego_ids) == [11, 12, 13]
            and list(msg.other_ids) == [21, 22, 23]
        )

    def _summarize_pair_opt(self, msg):
        return {
            "from_drone_id": msg.from_drone_id,
            "to_drone_id": msg.to_drone_id,
            "stamp": msg.stamp,
            "ego_ids": list(msg.ego_ids),
            "other_ids": list(msg.other_ids),
        }

    def _make_pair_opt_res_probe(self):
        msg = PairOptResponse()
        msg.from_drone_id = self.args.res_src_id
        msg.to_drone_id = self.args.res_target_id
        msg.status = 9307
        msg.stamp = 930001.0
        return msg

    def _match_pair_opt_res(self, msg):
        return (
            msg.from_drone_id == self.args.res_src_id
            and msg.to_drone_id == self.args.res_target_id
            and msg.status == 9307
            and self._close(msg.stamp, 930001.0)
        )

    def _summarize_pair_opt_res(self, msg):
        return {
            "from_drone_id": msg.from_drone_id,
            "to_drone_id": msg.to_drone_id,
            "status": msg.status,
            "stamp": msg.stamp,
        }

    def _make_swarm_traj_probe(self):
        msg = Bspline()
        msg.drone_id = self.args.src_id
        msg.order = 3
        msg.traj_id = 940001
        msg.start_time = rospy.Time.from_sec(9400.25)
        msg.knots = [0.0, 0.4, 0.8, 1.2, 1.6]
        msg.pos_pts = [
            self._point(94.0, 10.0, 1.0),
            self._point(94.5, 10.5, 1.1),
            self._point(95.0, 11.0, 1.2),
        ]
        msg.yaw_pts = [0.1, 0.2, 0.3]
        msg.yaw_dt = 0.4
        return msg

    def _match_swarm_traj(self, msg):
        return (
            msg.drone_id == self.args.src_id
            and msg.traj_id == 940001
            and msg.order == 3
            and self._close(msg.start_time.to_sec(), 9400.25)
            and len(msg.knots) == 5
            and len(msg.pos_pts) == 3
        )

    def _summarize_swarm_traj(self, msg):
        return {
            "drone_id": msg.drone_id,
            "traj_id": msg.traj_id,
            "order": msg.order,
            "start_time": msg.start_time.to_sec(),
            "knot_count": len(msg.knots),
            "pos_pt_count": len(msg.pos_pts),
        }

    def _make_chunk_stamps_probe(self):
        msg = ChunkStamps()
        msg.from_drone_id = self.args.src_id
        first = IdxList()
        first.ids = [9501, 9502]
        second = IdxList()
        second.ids = [9511, 9512, 9513]
        msg.idx_lists = [first, second]
        msg.time = 950001.0
        return msg

    def _match_chunk_stamps(self, msg):
        if msg.from_drone_id != self.args.src_id or not self._close(msg.time, 950001.0):
            return False
        got = [list(item.ids) for item in msg.idx_lists]
        return got == [[9501, 9502], [9511, 9512, 9513]]

    def _summarize_chunk_stamps(self, msg):
        return {
            "from_drone_id": msg.from_drone_id,
            "time": msg.time,
            "idx_lists": [list(item.ids) for item in msg.idx_lists],
        }

    def _make_chunk_data_probe(self):
        msg = ChunkData()
        msg.from_drone_id = self.args.src_id
        msg.to_drone_id = self.args.logical_target_id
        msg.chunk_drone_id = self.args.src_id
        msg.voxel_adrs = [9601, 9602, 9603]
        msg.voxel_occ_ = [1, 0, 1]
        msg.idx = 960001
        msg.latest_idx = 960009
        msg.pos_x = 96.1
        msg.pos_y = 96.2
        msg.pos_z = 96.3
        return msg

    def _match_chunk_data(self, msg):
        return (
            msg.from_drone_id == self.args.src_id
            and msg.to_drone_id == self.args.logical_target_id
            and msg.chunk_drone_id == self.args.src_id
            and list(msg.voxel_adrs) == [9601, 9602, 9603]
            and list(msg.voxel_occ_) == [1, 0, 1]
            and msg.idx == 960001
            and msg.latest_idx == 960009
            and self._close(msg.pos_x, 96.1)
            and self._close(msg.pos_y, 96.2)
            and self._close(msg.pos_z, 96.3)
        )

    def _summarize_chunk_data(self, msg):
        return {
            "from_drone_id": msg.from_drone_id,
            "to_drone_id": msg.to_drone_id,
            "chunk_drone_id": msg.chunk_drone_id,
            "idx": msg.idx,
            "latest_idx": msg.latest_idx,
            "voxel_adrs": list(msg.voxel_adrs),
            "voxel_occ_": list(msg.voxel_occ_),
            "pos": [msg.pos_x, msg.pos_y, msg.pos_z],
        }

    @staticmethod
    def _point(x, y, z):
        point = Point()
        point.x = x
        point.y = y
        point.z = z
        return point

    @staticmethod
    def _close(a, b, tol=1e-6):
        return math.fabs(a - b) <= tol


def parse_args():
    parser = argparse.ArgumentParser(description="External Stage C transport probe for RACER swarm topics.")
    parser.add_argument(
        "--families",
        nargs="+",
        default=["swarm_traj", "chunk_stamps", "chunk_data"],
        choices=["drone_state", "pair_opt", "pair_opt_res", "swarm_traj", "chunk_stamps", "chunk_data"],
        help="Topic families to inject and verify.",
    )
    parser.add_argument("--src-id", type=int, default=1, help="Primary source agent id for most probes.")
    parser.add_argument("--res-src-id", type=int, default=2, help="Source agent id for pair_opt_res probes.")
    parser.add_argument("--logical-target-id", type=int, default=2, help="Logical target id embedded in pair_opt and chunk_data probes.")
    parser.add_argument("--res-target-id", type=int, default=1, help="Logical target id embedded in pair_opt_res probes.")
    parser.add_argument("--recv-ids", nargs="+", type=int, default=[1, 2, 3, 4], help="Agent ids whose recv topics should be observed.")
    parser.add_argument("--startup-delay", type=float, default=10.0, help="Wall-clock delay before probing to let the operational stack start.")
    parser.add_argument("--connection-wait", type=float, default=6.0, help="Wall-clock time to wait for tx bridge subscriptions.")
    parser.add_argument("--pre-publish-settle", type=float, default=1.0, help="Extra wall-clock settle time before publishing probes.")
    parser.add_argument("--post-publish-wait", type=float, default=6.0, help="Wall-clock wait after publishing to collect recv messages.")
    parser.add_argument("--probe-count", type=int, default=4, help="How many times to publish each probe message.")
    parser.add_argument("--publish-interval", type=float, default=0.2, help="Wall-clock interval between repeated probe publishes.")
    parser.add_argument("--rounds", type=int, default=2, help="How many retry rounds to run for families that still have no matched recv messages.")
    return parser.parse_args()


def main():
    args = parse_args()
    probe = StageCTransportProbe(args)
    raise SystemExit(probe.run())


if __name__ == "__main__":
    main()
