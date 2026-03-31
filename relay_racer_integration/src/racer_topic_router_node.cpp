#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include <XmlRpcValue.h>
#include <boost/bind.hpp>

#include <relay_racer_integration/SwarmCommState.h>

#include <algorithm>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

namespace relay_racer_integration {
namespace {

long long DirectedKey(const int src_id, const int dst_id) {
  return (static_cast<long long>(src_id) << 32) ^ static_cast<unsigned int>(dst_id);
}

}  // namespace

class RacerTopicRouterNode {
public:
  RacerTopicRouterNode()
      : nh_(), pnh_("~"), has_comm_state_(false) {
    pnh_.param("fail_open_routing", fail_open_routing_, true);
    pnh_.param("queue_size", queue_size_, 50);
    pnh_.param<std::string>(
        "swarm_comm_state_topic", swarm_comm_state_topic_, "/relay_integration/swarm_comm_state");

    loadDroneIds();
    comm_state_sub_ = nh_.subscribe(
        swarm_comm_state_topic_, 10, &RacerTopicRouterNode::commStateCallback, this);

    registerFamily("drone_state", "/swarm_expl/drone_state");
    registerFamily("pair_opt", "/swarm_expl/pair_opt");
    registerFamily("pair_opt_res", "/swarm_expl/pair_opt_res");
    registerFamily("swarm_traj", "/planning/swarm_traj");
    registerFamily("chunk_stamps", "/multi_map_manager/chunk_stamps");
    registerFamily("chunk_data", "/multi_map_manager/chunk_data");
    registerFamily("relay_peer_state", "/relay_integration/relay_peer_state");
    registerFamily("relay_proposal", "/relay_integration/relay_proposal");
    registerFamily("relay_task_state", "/relay_integration/relay_task_state");

    ROS_WARN("[LOCAL_ROUTER] enabled local topic routing bypass. This path does not model the "
             "coordinator/PHY delayed-delivery transport and should only be used as a fallback.");
  }

private:
  void loadDroneIds() {
    XmlRpc::XmlRpcValue drone_ids_param;
    if (pnh_.getParam("drone_ids", drone_ids_param) && drone_ids_param.getType() == XmlRpc::XmlRpcValue::TypeArray) {
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

  void registerFamily(const std::string& family_name, const std::string& topic_base) {
    for (const int src_id : drone_ids_) {
      const std::string input_topic = topic_base + "_send_" + std::to_string(src_id);
      routed_subscribers_.push_back(nh_.subscribe<topic_tools::ShapeShifter>(input_topic, queue_size_,
          boost::bind(&RacerTopicRouterNode::routeMessage, this, _1, src_id, topic_base)));
      ROS_INFO_STREAM("racer_topic_router_node subscribing to " << input_topic << " for family "
                      << family_name);
    }
  }

  void routeMessage(
      const topic_tools::ShapeShifter::ConstPtr& msg, const int src_id, const std::string& topic_base) {
    for (const int dst_id : drone_ids_) {
      if (dst_id == src_id) {
        continue;
      }
      if (!shouldForward(src_id, dst_id)) {
        continue;
      }

      const std::string output_topic = topic_base + "_recv_" + std::to_string(dst_id);
      ros::Publisher& publisher = output_publishers_[output_topic];
      if (publisher.getTopic().empty()) {
        publisher = msg->advertise(nh_, output_topic, queue_size_, false);
      }
      publisher.publish(msg);
    }
  }

  void commStateCallback(const SwarmCommStateConstPtr& msg) {
    has_comm_state_ = true;
    link_up_map_.clear();

    const size_t edge_count = std::min(
        std::min(msg->link_src_ids.size(), msg->link_dst_ids.size()), msg->link_up.size());
    for (size_t i = 0; i < edge_count; ++i) {
      link_up_map_[DirectedKey(msg->link_src_ids[i], msg->link_dst_ids[i])] = msg->link_up[i];
    }
  }

  bool shouldForward(const int src_id, const int dst_id) const {
    if (!has_comm_state_) {
      return fail_open_routing_;
    }
    const auto it = link_up_map_.find(DirectedKey(src_id, dst_id));
    if (it == link_up_map_.end()) {
      return fail_open_routing_;
    }
    return it->second;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber comm_state_sub_;
  std::vector<ros::Subscriber> routed_subscribers_;
  std::map<std::string, ros::Publisher> output_publishers_;

  bool has_comm_state_;
  bool fail_open_routing_;
  int queue_size_;
  std::string swarm_comm_state_topic_;
  std::vector<int> drone_ids_;
  std::unordered_map<long long, bool> link_up_map_;
};

}  // namespace relay_racer_integration

int main(int argc, char** argv) {
  ros::init(argc, argv, "racer_topic_router_node");
  relay_racer_integration::RacerTopicRouterNode node;
  ros::spin();
  return 0;
}
