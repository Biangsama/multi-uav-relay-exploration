#include <ros/ros.h>
#include <ros/serialization.h>
#include <std_msgs/UInt8MultiArray.h>
#include <topic_tools/shape_shifter.h>
#include <XmlRpcValue.h>
#include <boost/bind.hpp>

#include <protobuf_msgs/racer_swarm_msg.pb.h>

#include <map>
#include <string>
#include <utility>
#include <vector>

namespace relay_racer_integration {

class RacerMsgTxBridgeNode {
 public:
  RacerMsgTxBridgeNode() : nh_(), pnh_("~") {
    pnh_.param("queue_size", queue_size_, 50);
    pnh_.param("default_max_relay_hops", default_max_relay_hops_, 1);
    pnh_.param<std::string>("tx_topic", tx_topic_,
                            std::string("/relay_integration/racer_swarm_tx_bytes"));
    loadDroneIds();
    loadFamilies();

    tx_pub_ = nh_.advertise<std_msgs::UInt8MultiArray>(tx_topic_, queue_size_);

    for (const auto& family : families_) {
      for (const int src_id : drone_ids_) {
        const std::string input_topic = family.second + "_send_" + std::to_string(src_id);
        subscribers_.push_back(nh_.subscribe<topic_tools::ShapeShifter>(
            input_topic, queue_size_,
            boost::bind(&RacerMsgTxBridgeNode::messageCallback, this, _1, src_id, family.first)));
        ROS_INFO_STREAM("racer_msg_tx_bridge subscribing to " << input_topic << " family="
                        << family.first);
      }
    }
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
    families_.push_back({"drone_state", "/swarm_expl/drone_state"});
    families_.push_back({"pair_opt", "/swarm_expl/pair_opt"});
    families_.push_back({"pair_opt_res", "/swarm_expl/pair_opt_res"});
    families_.push_back({"swarm_traj", "/planning/swarm_traj"});
    families_.push_back({"chunk_stamps", "/multi_map_manager/chunk_stamps"});
    families_.push_back({"chunk_data", "/multi_map_manager/chunk_data"});
    families_.push_back({"relay_peer_state", "/relay_integration/relay_peer_state"});
    families_.push_back({"relay_proposal", "/relay_integration/relay_proposal"});
    families_.push_back({"relay_task_state", "/relay_integration/relay_task_state"});
  }

  void messageCallback(const topic_tools::ShapeShifter::ConstPtr& msg,
                       const int src_id,
                       const std::string& family) {
    const uint32_t serialized_len = ros::serialization::serializationLength(*msg);
    std::vector<uint8_t> payload(serialized_len);
    ros::serialization::OStream ostream(payload.data(), payload.size());
    ros::serialization::serialize(ostream, *msg);

    relay_racer_proto::RacerSwarmMsg wrapper;
    wrapper.set_family(family);
    wrapper.set_src_id(static_cast<uint32_t>(src_id));
    wrapper.set_dst_id(0);
    wrapper.set_bridge_seq(nextSeq(family, src_id));
    wrapper.set_stamp_us(static_cast<uint64_t>(ros::Time::now().toNSec() / 1000ull));
    wrapper.set_ros_datatype(msg->getDataType());
    wrapper.set_ros_md5(msg->getMD5Sum());
    wrapper.set_ros_msg_def(msg->getMessageDefinition());
    wrapper.set_ros_latching("0");
    wrapper.set_ros_payload(payload.data(), payload.size());
    wrapper.set_network_tx_id(static_cast<uint32_t>(src_id));
    wrapper.set_relay_hop_count(0);
    wrapper.set_max_relay_hops(static_cast<uint32_t>(std::max(0, default_max_relay_hops_)));

    std::string bytes;
    if (!wrapper.SerializeToString(&bytes)) {
      ROS_WARN_THROTTLE(1.0, "Failed to serialize RacerSwarmMsg for family=%s src=%d",
                        family.c_str(), src_id);
      return;
    }

    std_msgs::UInt8MultiArray out;
    out.data.assign(bytes.begin(), bytes.end());
    tx_pub_.publish(out);
  }

  uint64_t nextSeq(const std::string& family, const int src_id) {
    std::pair<std::string, int> key(family, src_id);
    uint64_t& seq = next_seq_[key];
    const uint64_t current = seq;
    ++seq;
    return current;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher tx_pub_;
  std::vector<ros::Subscriber> subscribers_;
  std::vector<int> drone_ids_;
  std::vector<std::pair<std::string, std::string>> families_;
  std::map<std::pair<std::string, int>, uint64_t> next_seq_;
  int queue_size_{50};
  int default_max_relay_hops_{1};
  std::string tx_topic_;
};

}  // namespace relay_racer_integration

int main(int argc, char** argv) {
  ros::init(argc, argv, "racer_msg_tx_bridge_node");
  relay_racer_integration::RacerMsgTxBridgeNode node;
  ros::spin();
  return 0;
}
