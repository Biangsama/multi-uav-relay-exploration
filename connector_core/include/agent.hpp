#pragma once

#include <Eigen/Dense>
#include <dancers_msgs/AgentStruct.h>
#include <dancers_msgs/Neighbor.h>

#include <map>
#include <memory>
#include <string>

enum AgentRoleType
{
    Undefined = 0,
    Mission,
    Potential,
    Idle
};

enum LinkType
{
    DataLink = 0,
    FlockingLink
};

struct NeighborInfo
{
    int id{};
    double link_quality{};
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    AgentRoleType role{Undefined};
    LinkType link_type{DataLink};
    uint32_t last_seen{};  // us
};

namespace ns3 {
template <typename T>
class Ptr;
class Node;
}  // namespace ns3
namespace mrs_multirotor_simulator {
class UavSystem;
}  // namespace mrs_multirotor_simulator

struct Agent
{
    uint32_t id{};
    AgentRoleType role{Undefined};

    std::shared_ptr<mrs_multirotor_simulator::UavSystem> uav_system;
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d initial_position{Eigen::Vector3d::Zero()};
    bool initial_position_saved{false};
    double heading{};
    bool crashed{false};
    Eigen::Vector3d cmd_velocity{Eigen::Vector3d::Zero()};
    double cmd_heading{};

    std::shared_ptr<ns3::Ptr<ns3::Node>> node;
    uint32_t heartbeat_received{};
    uint32_t heartbeat_sent{};
    std::map<uint32_t, NeighborInfo> neighbors;
    uint32_t node_container_index{};
    double channel_busy_time{};

    std::string getNeighborsString()
    {
        std::string s = "Neighbors:";
        for (const auto& pair : neighbors)
        {
            s += "[" + std::to_string(pair.first) + "]\n";
        }
        return s;
    }
};

inline Agent AgentFromRosMsg(const dancers_msgs::AgentStruct& agent)
{
    Agent a;
    a.id = agent.agent_id;
    a.role = static_cast<AgentRoleType>(agent.agent_role);
    a.position =
        Eigen::Vector3d(agent.state.position.x, agent.state.position.y, agent.state.position.z);
    a.velocity =
        Eigen::Vector3d(agent.state.velocity.x, agent.state.velocity.y, agent.state.velocity.z);
    a.heading = agent.state.heading;
    a.heartbeat_received = agent.heartbeat_received;
    a.heartbeat_sent = agent.heartbeat_sent;

    for (const auto& neighbor : agent.neighbor_array.neighbors)
    {
        NeighborInfo n;
        n.id = neighbor.agent_id;
        n.link_quality = neighbor.link_quality;
        n.position = Eigen::Vector3d(neighbor.position.x, neighbor.position.y, neighbor.position.z);
        n.velocity = Eigen::Vector3d(neighbor.velocity.x, neighbor.velocity.y, neighbor.velocity.z);
        n.role = static_cast<AgentRoleType>(neighbor.agent_role);
        a.neighbors[neighbor.agent_id] = n;
    }

    return a;
}

inline dancers_msgs::AgentStruct RosMsgFromAgent(const Agent& agent)
{
    dancers_msgs::AgentStruct msg;
    msg.agent_id = agent.id;
    msg.agent_role = static_cast<uint8_t>(agent.role);
    msg.state.position.x = agent.position.x();
    msg.state.position.y = agent.position.y();
    msg.state.position.z = agent.position.z();
    msg.state.velocity.x = agent.velocity.x();
    msg.state.velocity.y = agent.velocity.y();
    msg.state.velocity.z = agent.velocity.z();
    msg.state.heading = agent.heading;
    msg.heartbeat_received = agent.heartbeat_received;
    msg.heartbeat_sent = agent.heartbeat_sent;

    for (const auto& pair : agent.neighbors)
    {
        const NeighborInfo& n = pair.second;
        dancers_msgs::Neighbor neighbor_msg;
        neighbor_msg.agent_id = n.id;
        neighbor_msg.link_quality = n.link_quality;
        neighbor_msg.position.x = n.position.x();
        neighbor_msg.position.y = n.position.y();
        neighbor_msg.position.z = n.position.z();
        neighbor_msg.velocity.x = n.velocity.x();
        neighbor_msg.velocity.y = n.velocity.y();
        neighbor_msg.velocity.z = n.velocity.z();
        neighbor_msg.agent_role = static_cast<uint8_t>(n.role);
        msg.neighbor_array.neighbors.push_back(neighbor_msg);
    }

    return msg;
}
