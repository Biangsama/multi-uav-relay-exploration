#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>

#include <protobuf_msgs/pose_vector.pb.h>

#include <relay_racer_integration/id_utils.h>

#include <string>

namespace relay_racer_integration {

class RacerPhyConnectorNode {
public:
  RacerPhyConnectorNode()
      : nh_(), pnh_("~") {
    pnh_.param("racer_id", racer_id_, 1);
    pnh_.param("id_offset", id_offset_, -1);
    pnh_.param("platform_id", platform_id_, PlatformIdFromRacerId(racer_id_, id_offset_));
    pnh_.param<std::string>("odom_topic", odom_topic_, "/odom_world");
    pnh_.param<std::string>(
        "pose_vector_bytes_topic", pose_vector_bytes_topic_, "/relay_integration/platform_pose_vector_bytes");

    odom_sub_ = nh_.subscribe(odom_topic_, 10, &RacerPhyConnectorNode::odomCallback, this);
    pose_vector_pub_ = nh_.advertise<std_msgs::UInt8MultiArray>(pose_vector_bytes_topic_, 10);

    ROS_INFO_STREAM("racer_phy_connector_node listening on " << odom_topic_ << ", racer_id="
                    << racer_id_ << ", platform_id=" << platform_id_ << ", id_offset=" << id_offset_);
  }

private:
  void odomCallback(const nav_msgs::OdometryConstPtr& msg) {
    dancers_update_proto::PoseVector pose_vector;
    auto* pose = pose_vector.add_pose();
    pose->set_x(msg->pose.pose.position.x);
    pose->set_y(msg->pose.pose.position.y);
    pose->set_z(msg->pose.pose.position.z);
    pose->set_qx(msg->pose.pose.orientation.x);
    pose->set_qy(msg->pose.pose.orientation.y);
    pose->set_qz(msg->pose.pose.orientation.z);
    pose->set_qw(msg->pose.pose.orientation.w);
    pose->set_vx(msg->twist.twist.linear.x);
    pose->set_vy(msg->twist.twist.linear.y);
    pose->set_vz(msg->twist.twist.linear.z);
    pose->set_agent_id(static_cast<unsigned int>(platform_id_));

    std::string bytes;
    if (!pose_vector.SerializeToString(&bytes)) {
      ROS_WARN_THROTTLE(1.0, "Failed to serialize PoseVector for platform bridge.");
      return;
    }

    std_msgs::UInt8MultiArray payload;
    payload.data.assign(bytes.begin(), bytes.end());
    pose_vector_pub_.publish(payload);

    // TODO(stage-1): replace the debug byte publisher with the coordinator/PoseVector socket bridge.
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber odom_sub_;
  ros::Publisher pose_vector_pub_;

  int racer_id_;
  int platform_id_;
  int id_offset_;
  std::string odom_topic_;
  std::string pose_vector_bytes_topic_;
};

}  // namespace relay_racer_integration

int main(int argc, char** argv) {
  ros::init(argc, argv, "racer_phy_connector_node");
  relay_racer_integration::RacerPhyConnectorNode node;
  ros::spin();
  return 0;
}
