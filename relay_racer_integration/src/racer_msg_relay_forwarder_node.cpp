#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>

#include <protobuf_msgs/racer_swarm_msg.pb.h>
#include <relay_racer_integration/RelayRoleCmd.h>

#include <cstdint>
#include <string>
#include <unordered_map>

namespace relay_racer_integration {

class RacerMsgRelayForwarderNode {
 public:
  RacerMsgRelayForwarderNode() : nh_(), pnh_("~") {
    pnh_.param("queue_size", queue_size_, 100);
    pnh_.param("forward_cache_ttl_sec", forward_cache_ttl_sec_, 20.0);
    pnh_.param<std::string>("rx_topic", rx_topic_,
                            std::string("/relay_integration/racer_swarm_rx_bytes"));
    pnh_.param<std::string>("tx_topic", tx_topic_,
                            std::string("/relay_integration/racer_swarm_tx_bytes"));
    pnh_.param<std::string>("relay_role_topic", relay_role_topic_,
                            std::string("/relay_integration/relay_role_cmd"));

    tx_pub_ = nh_.advertise<std_msgs::UInt8MultiArray>(tx_topic_, queue_size_);
    rx_sub_ = nh_.subscribe(rx_topic_, queue_size_,
                            &RacerMsgRelayForwarderNode::rxBytesCallback, this);
    role_sub_ = nh_.subscribe(relay_role_topic_, 50,
                              &RacerMsgRelayForwarderNode::relayRoleCallback, this);
  }

 private:
  void relayRoleCallback(const RelayRoleCmdConstPtr& msg) {
    relay_active_by_agent_[msg->agent_id] =
        (msg->role == RelayRoleCmd::ROLE_RELAY) && msg->valid;
  }

  void rxBytesCallback(const std_msgs::UInt8MultiArrayConstPtr& msg) {
    relay_racer_proto::RacerSwarmMsg wrapper;
    const std::string bytes(msg->data.begin(), msg->data.end());
    if (!wrapper.ParseFromString(bytes)) {
      ROS_WARN_THROTTLE(1.0, "[RELAY_FWD] failed to parse RacerSwarmMsg from rx bytes.");
      return;
    }

    const int relay_id = static_cast<int>(wrapper.dst_id());
    if (relay_id <= 0 || !isRelayActive(relay_id)) {
      return;
    }
    if (wrapper.max_relay_hops() == 0 || wrapper.relay_hop_count() >= wrapper.max_relay_hops()) {
      return;
    }
    if (wrapper.network_tx_id() == wrapper.dst_id()) {
      return;
    }
    if (seenAndRefresh(relay_id, wrapper)) {
      return;
    }

    wrapper.set_network_tx_id(static_cast<uint32_t>(relay_id));
    wrapper.set_relay_hop_count(wrapper.relay_hop_count() + 1);
    wrapper.set_dst_id(0);

    std::string forwarded_bytes;
    if (!wrapper.SerializeToString(&forwarded_bytes)) {
      ROS_WARN_THROTTLE(1.0, "[RELAY_FWD] failed to serialize forwarded RacerSwarmMsg.");
      return;
    }

    std_msgs::UInt8MultiArray out;
    out.data.assign(forwarded_bytes.begin(), forwarded_bytes.end());
    tx_pub_.publish(out);
    ++forwarded_msgs_;
    ROS_INFO_STREAM_THROTTLE(1.0,
                             "[RELAY_FWD] forwarded msgs=" << forwarded_msgs_
                             << " relay_id=" << relay_id
                             << " family=" << wrapper.family()
                             << " src_id=" << wrapper.src_id()
                             << " bridge_seq=" << wrapper.bridge_seq()
                             << " hop_count=" << wrapper.relay_hop_count());
  }

  bool isRelayActive(int agent_id) const {
    const auto it = relay_active_by_agent_.find(agent_id);
    return it != relay_active_by_agent_.end() && it->second;
  }

  bool seenAndRefresh(int relay_id, const relay_racer_proto::RacerSwarmMsg& wrapper) {
    pruneSeenCache();
    const std::string key = std::to_string(relay_id) + "|" + wrapper.family() + "|" +
                            std::to_string(wrapper.src_id()) + "|" +
                            std::to_string(wrapper.bridge_seq());
    const uint64_t now_us = static_cast<uint64_t>(ros::WallTime::now().toNSec() / 1000ull);
    auto it = seen_forward_us_.find(key);
    if (it != seen_forward_us_.end() && now_us >= it->second &&
        (now_us - it->second) <= static_cast<uint64_t>(forward_cache_ttl_sec_ * 1e6)) {
      it->second = now_us;
      return true;
    }
    seen_forward_us_[key] = now_us;
    return false;
  }

  void pruneSeenCache() {
    const uint64_t now_us = static_cast<uint64_t>(ros::WallTime::now().toNSec() / 1000ull);
    if (now_us <= last_prune_us_ + 1000000ull) {
      return;
    }
    const uint64_t ttl_us = static_cast<uint64_t>(forward_cache_ttl_sec_ * 1e6);
    for (auto it = seen_forward_us_.begin(); it != seen_forward_us_.end();) {
      if (now_us >= it->second && (now_us - it->second) > ttl_us) {
        it = seen_forward_us_.erase(it);
      } else {
        ++it;
      }
    }
    last_prune_us_ = now_us;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher tx_pub_;
  ros::Subscriber rx_sub_;
  ros::Subscriber role_sub_;
  std::unordered_map<int, bool> relay_active_by_agent_;
  std::unordered_map<std::string, uint64_t> seen_forward_us_;
  int queue_size_{100};
  double forward_cache_ttl_sec_{20.0};
  uint64_t last_prune_us_{0};
  uint64_t forwarded_msgs_{0};
  std::string rx_topic_;
  std::string tx_topic_;
  std::string relay_role_topic_;
};

}  // namespace relay_racer_integration

int main(int argc, char** argv) {
  ros::init(argc, argv, "racer_msg_relay_forwarder_node");
  relay_racer_integration::RacerMsgRelayForwarderNode node;
  ros::spin();
  return 0;
}
