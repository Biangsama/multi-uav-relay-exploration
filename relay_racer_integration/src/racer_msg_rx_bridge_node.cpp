#include <ros/ros.h>
#include <ros/serialization.h>
#include <std_msgs/UInt8MultiArray.h>
#include <topic_tools/shape_shifter.h>
#include <XmlRpcValue.h>

#include <protobuf_msgs/racer_swarm_msg.pb.h>

#include <cstdint>
#include <map>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace relay_racer_integration {

class RacerMsgRxBridgeNode {
 public:
  RacerMsgRxBridgeNode() : nh_(), pnh_("~") {
    pnh_.param("queue_size", queue_size_, 50);
    pnh_.param("dedupe_ttl_sec", dedupe_ttl_sec_, 15.0);
    pnh_.param<std::string>("rx_topic", rx_topic_,
                            std::string("/relay_integration/racer_swarm_rx_bytes"));
    pnh_.param("trace_swarm_msg_enable", trace_swarm_msg_enable_, false);
    pnh_.param<std::string>("trace_swarm_msg_family", trace_swarm_msg_family_, std::string());
    pnh_.param("trace_swarm_msg_src_id", trace_swarm_msg_src_id_, -1);
    pnh_.param("trace_swarm_msg_bridge_seq", trace_swarm_msg_bridge_seq_, -1);
    loadDroneIds();
    loadFamilies();

    sub_ = nh_.subscribe(rx_topic_, queue_size_, &RacerMsgRxBridgeNode::bytesCallback, this);
  }

 private:
  void loadDroneIds() {
    XmlRpc::XmlRpcValue drone_ids_param;
    if (pnh_.getParam("drone_ids", drone_ids_param) &&
        drone_ids_param.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      for (int i = 0; i < drone_ids_param.size(); ++i) {
        drone_ids_.push_back(static_cast<int>(drone_ids_param[i]));
      }
    }

    if (!drone_ids_.empty()) {
      return;
    }

    int drone_num = 1;
    int first_drone_id = 1;
    pnh_.param("drone_num", drone_num, 1);
    pnh_.param("first_drone_id", first_drone_id, 1);
    for (int i = 0; i < drone_num; ++i) {
      drone_ids_.push_back(first_drone_id + i);
    }
  }

  void loadFamilies() {
    families_["drone_state"] = "/swarm_expl/drone_state";
    families_["pair_opt"] = "/swarm_expl/pair_opt";
    families_["pair_opt_res"] = "/swarm_expl/pair_opt_res";
    families_["swarm_traj"] = "/planning/swarm_traj";
    families_["chunk_stamps"] = "/multi_map_manager/chunk_stamps";
    families_["chunk_data"] = "/multi_map_manager/chunk_data";
    families_["relay_peer_state"] = "/relay_integration/relay_peer_state";
    families_["relay_proposal"] = "/relay_integration/relay_proposal";
    families_["relay_task_state"] = "/relay_integration/relay_task_state";
  }

  void bytesCallback(const std_msgs::UInt8MultiArrayConstPtr& msg) {
    ++rx_wrapper_msgs_;
    const std::string bytes(msg->data.begin(), msg->data.end());
    relay_racer_proto::RacerSwarmMsg wrapper;
    if (!wrapper.ParseFromString(bytes)) {
      ROS_WARN_THROTTLE(1.0, "Failed to parse RacerSwarmMsg from rx bytes.");
      return;
    }

    if (shouldTrace(wrapper)) {
      traceWrapper("rx_bridge_bytes", wrapper,
                   "rx_wrapper_msgs=" + std::to_string(rx_wrapper_msgs_));
    }

    const auto family_it = families_.find(wrapper.family());
    if (family_it == families_.end()) {
      ROS_WARN_THROTTLE(1.0, "Unknown RacerSwarmMsg family=%s", wrapper.family().c_str());
      return;
    }
    if (wrapper.dst_id() == 0) {
      if (shouldTrace(wrapper)) {
        traceWrapper("rx_bridge_drop_unresolved", wrapper);
      }
      ROS_WARN_THROTTLE(1.0, "Dropping RacerSwarmMsg family=%s with unresolved dst_id=0",
                        wrapper.family().c_str());
      return;
    }
    if (isDuplicateAndUpdate(wrapper)) {
      ++deduped_msgs_;
      if (shouldTrace(wrapper)) {
        traceWrapper("rx_bridge_drop_duplicate", wrapper,
                     "deduped_msgs=" + std::to_string(deduped_msgs_));
      }
      ROS_INFO_STREAM_THROTTLE(1.0,
                               "[RX_BRIDGE] dropping duplicate wrapper family=" << wrapper.family()
                               << " src_id=" << wrapper.src_id()
                               << " dst_id=" << wrapper.dst_id()
                               << " bridge_seq=" << wrapper.bridge_seq()
                               << " deduped=" << deduped_msgs_);
      return;
    }

    std::vector<uint8_t> payload(wrapper.ros_payload().begin(), wrapper.ros_payload().end());
    topic_tools::ShapeShifter shifter;
    shifter.morph(wrapper.ros_md5(), wrapper.ros_datatype(), wrapper.ros_msg_def(),
                  wrapper.ros_latching());
    ros::serialization::IStream istream(payload.data(), payload.size());
    shifter.read(istream);

    const std::string output_topic =
        family_it->second + "_recv_" + std::to_string(wrapper.dst_id());
    ros::Publisher& pub = publishers_[output_topic];
    if (pub.getTopic().empty()) {
      pub = shifter.advertise(nh_, output_topic, queue_size_, true);
      ROS_INFO_STREAM("[RX_BRIDGE] advertised latched recv topic " << output_topic
                      << " family=" << wrapper.family() << " dst_id=" << wrapper.dst_id());
    }

    pub.publish(shifter);
    ++published_msgs_;
    ++published_per_topic_[output_topic];
    if (shouldTrace(wrapper)) {
      traceWrapper("rx_bridge_publish", wrapper,
                   "topic=" + output_topic +
                       " topic_count=" + std::to_string(published_per_topic_[output_topic]));
    }
    ROS_INFO_STREAM_THROTTLE(1.0,
                             "[RX_BRIDGE] wrapper msgs=" << rx_wrapper_msgs_
                             << " published msgs=" << published_msgs_
                             << " last family=" << wrapper.family()
                             << " src_id=" << wrapper.src_id()
                             << " dst_id=" << wrapper.dst_id()
                             << " hop_count=" << wrapper.relay_hop_count()
                             << " topic=" << output_topic
                             << " topic_count=" << published_per_topic_[output_topic]);
  }

  bool isDuplicateAndUpdate(const relay_racer_proto::RacerSwarmMsg& wrapper) {
    pruneDedupeCache();
    const std::string key = wrapper.family() + "|" + std::to_string(wrapper.src_id()) + "|" +
                            std::to_string(wrapper.bridge_seq()) + "|" +
                            std::to_string(wrapper.dst_id());
    const uint64_t now_us = static_cast<uint64_t>(ros::WallTime::now().toNSec() / 1000ull);
    auto it = delivered_cache_us_.find(key);
    if (it != delivered_cache_us_.end() && now_us >= it->second &&
        (now_us - it->second) <= static_cast<uint64_t>(dedupe_ttl_sec_ * 1e6)) {
      it->second = now_us;
      return true;
    }
    delivered_cache_us_[key] = now_us;
    return false;
  }

  void pruneDedupeCache() {
    const uint64_t now_us = static_cast<uint64_t>(ros::WallTime::now().toNSec() / 1000ull);
    if (now_us <= last_prune_us_ + 1000000ull) {
      return;
    }
    const uint64_t ttl_us = static_cast<uint64_t>(dedupe_ttl_sec_ * 1e6);
    for (auto it = delivered_cache_us_.begin(); it != delivered_cache_us_.end();) {
      if (now_us >= it->second && (now_us - it->second) > ttl_us) {
        it = delivered_cache_us_.erase(it);
      } else {
        ++it;
      }
    }
    last_prune_us_ = now_us;
  }

  bool shouldTrace(const relay_racer_proto::RacerSwarmMsg& wrapper) const {
    if (!trace_swarm_msg_enable_) {
      return false;
    }
    if (!trace_swarm_msg_family_.empty() && wrapper.family() != trace_swarm_msg_family_) {
      return false;
    }
    if (trace_swarm_msg_src_id_ > 0 &&
        static_cast<int>(wrapper.src_id()) != trace_swarm_msg_src_id_) {
      return false;
    }
    if (trace_swarm_msg_bridge_seq_ >= 0 &&
        wrapper.bridge_seq() != static_cast<uint64_t>(trace_swarm_msg_bridge_seq_)) {
      return false;
    }
    return true;
  }

  void traceWrapper(const char* stage,
                    const relay_racer_proto::RacerSwarmMsg& wrapper,
                    const std::string& extra = std::string()) const {
    if (extra.empty()) {
      ROS_INFO_STREAM("[SWARM_TRACE][" << stage << "] family=" << wrapper.family()
                      << " src_id=" << wrapper.src_id()
                      << " dst_id=" << wrapper.dst_id()
                      << " bridge_seq=" << wrapper.bridge_seq()
                      << " network_tx_id=" << wrapper.network_tx_id()
                      << " relay_hop_count=" << wrapper.relay_hop_count()
                      << " max_relay_hops=" << wrapper.max_relay_hops()
                      << " payload_bytes=" << wrapper.ros_payload().size());
      return;
    }

    ROS_INFO_STREAM("[SWARM_TRACE][" << stage << "] family=" << wrapper.family()
                    << " src_id=" << wrapper.src_id()
                    << " dst_id=" << wrapper.dst_id()
                    << " bridge_seq=" << wrapper.bridge_seq()
                    << " network_tx_id=" << wrapper.network_tx_id()
                    << " relay_hop_count=" << wrapper.relay_hop_count()
                    << " max_relay_hops=" << wrapper.max_relay_hops()
                    << " payload_bytes=" << wrapper.ros_payload().size()
                    << " " << extra);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_;
  std::map<std::string, std::string> families_;
  std::map<std::string, ros::Publisher> publishers_;
  std::map<std::string, uint64_t> published_per_topic_;
  std::vector<int> drone_ids_;
  int queue_size_{50};
  uint64_t rx_wrapper_msgs_{0};
  uint64_t published_msgs_{0};
  uint64_t deduped_msgs_{0};
  double dedupe_ttl_sec_{15.0};
  bool trace_swarm_msg_enable_{false};
  std::string trace_swarm_msg_family_;
  int trace_swarm_msg_src_id_{-1};
  int trace_swarm_msg_bridge_seq_{-1};
  uint64_t last_prune_us_{0};
  std::unordered_map<std::string, uint64_t> delivered_cache_us_;
  std::string rx_topic_;
};

}  // namespace relay_racer_integration

int main(int argc, char** argv) {
  ros::init(argc, argv, "racer_msg_rx_bridge_node");
  relay_racer_integration::RacerMsgRxBridgeNode node;
  ros::spin();
  return 0;
}
