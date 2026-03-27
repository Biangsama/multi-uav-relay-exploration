#include <connector_core/connector.hpp>

#include <ns3/buildings-module.h>
#include <ns3/core-module.h>
#include <ns3/internet-module.h>
#include <ns3/mobility-module.h>
#include <ns3/propagation-loss-model.h>
#include <ns3/wifi-module.h>

#include <applications/waypoint_broadcast.h>

#include <agent.hpp>
#include <protobuf_msgs/net_rx_events.pb.h>
#include <protobuf_msgs/pose_vector.pb.h>
#include <ros/ros.h>
#include <yaml_util.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <shared_mutex>
#include <string>
#include <vector>

using namespace ns3;

class BasicWifiAdhoc : public Connector
{
public:
    BasicWifiAdhoc()
        : Connector("basic_wifi_adhoc")
    {
        ROS_INFO("BasicWifiAdhoc (NET) started");

        ReadConfigFile();
        ConfigureNs3();
        InitAgents();

        auto leader_app = AddWaypointBroadcaster(nodes_.Get(0), 0);
        for (uint32_t i = 1; i < nodes_.GetN(); ++i)
        {
            AddWaypointReceiver(nodes_.Get(i));
        }

        if (config_["square_waypoint"])
        {
            const auto& square = config_["square_waypoint"];
            const double side_length = getYamlValue<double>(square, "side_length");
            const double half_side = side_length * 0.5;
            const bool clockwise = square["clockwise"] ? square["clockwise"].as<bool>() : true;
            const uint32_t start_corner_index =
                square["start_corner_index"] ? square["start_corner_index"].as<uint32_t>() : 0u;
            const Time interval = MicroSeconds(getYamlValue<unsigned int>(square, "new_waypoint_interval"));

            const Vector center(
                getYamlValue<double>(square["center"], "x"),
                getYamlValue<double>(square["center"], "y"),
                getYamlValue<double>(square["center"], "z"));

            std::array<Vector, 4> corners = {
                Vector(center.x - half_side, center.y - half_side, center.z),
                Vector(center.x + half_side, center.y - half_side, center.z),
                Vector(center.x + half_side, center.y + half_side, center.z),
                Vector(center.x - half_side, center.y + half_side, center.z),
            };

            if (!clockwise)
            {
                std::swap(corners[1], corners[3]);
            }

            SquareWaypointGeneration(
                leader_app,
                interval,
                corners,
                start_corner_index % static_cast<uint32_t>(corners.size()));
        }
        else if (config_["random_waypoint"])
        {
            RandomWaypointGeneration(
                leader_app,
                MicroSeconds(getYamlValue<unsigned int>(config_["random_waypoint"], "new_waypoint_interval")),
                Vector(
                    getYamlValue<double>(config_["random_waypoint"]["arena_corner_1"], "x"),
                    getYamlValue<double>(config_["random_waypoint"]["arena_corner_1"], "y"),
                    getYamlValue<double>(config_["random_waypoint"]["arena_corner_1"], "z")),
                Vector(
                    getYamlValue<double>(config_["random_waypoint"]["arena_corner_2"], "x"),
                    getYamlValue<double>(config_["random_waypoint"]["arena_corner_2"], "y"),
                    getYamlValue<double>(config_["random_waypoint"]["arena_corner_2"], "z")));
        }
        else
        {
            throw std::runtime_error(
                "Neither square_waypoint nor random_waypoint section found in Tutorial 3 config.");
        }
    }

    void RunLoop()
    {
        Loop();
    }

private:
    struct RxEventRow
    {
        uint64_t rx_time_us{0};
        uint32_t rx_id{0};
        uint64_t tx_time_us{0};
        uint32_t tx_id{0};
        uint32_t sender_id{0};
        uint32_t flow_id{0};
        uint32_t seq{0};
        uint32_t pkt_bytes{0};
        uint64_t delay_us{0};
        double wp_x{0.0};
        double wp_y{0.0};
        double wp_z{0.0};
        bool is_duplicate{false};
    };

    void ReadConfigFile() override
    {
        use_uds = getYamlValue<bool>(config_, "net_use_uds");
        uds_server_address = getYamlValue<std::string>(config_, "net_uds_server_address");
        ip_server_address = getYamlValue<std::string>(config_, "net_ip_server_address");
        ip_server_port = getYamlValue<unsigned int>(config_, "net_ip_server_port");

        const unsigned int sync_window_int = getYamlValue<unsigned int>(config_, "sync_window");
        const unsigned int step_size_int = getYamlValue<unsigned int>(config_, "net_step_size");
        if ((sync_window_int % step_size_int) != 0)
        {
            throw std::runtime_error("Sync window must be a multiple of the network step size.");
        }

        step_size = static_cast<double>(step_size_int) / 1000000.0;
        it_end_sim = static_cast<uint64_t>(simulation_length / step_size);
    }

    dancers_update_proto::DancersUpdate StepSimulation(dancers_update_proto::DancersUpdate update_msg) override
    {
        rx_events_window_.clear();
        UpdateAgentsPosition(update_msg);

        Simulator::Stop(Seconds(step_size));
        Simulator::Run();

        return GenerateResponse_();
    }

    dancers_update_proto::DancersUpdate GenerateResponse_()
    {
        dancers_update_proto::DancersUpdate response;
        response.set_msg_type(dancers_update_proto::DancersUpdate::END);

        dancers_update_proto::PoseVector waypoints;
        {
            std::shared_lock<std::shared_mutex> lk(agents_mutex_);
            for (const auto& kv : current_waypoints_)
            {
                const uint32_t agent_id = kv.first;
                const auto agent_it = agents_.find(agent_id);
                if (agent_it == agents_.end())
                {
                    continue;
                }

                const Vector waypoint_world = WaypointToWorldFrame(agent_id, kv.second);
                auto* msg = waypoints.add_pose();
                msg->set_agent_id(agent_id);
                msg->set_x(waypoint_world.x);
                msg->set_y(waypoint_world.y);
                msg->set_z(waypoint_world.z);
            }
        }

        std::string waypoint_bytes;
        waypoints.SerializeToString(&waypoint_bytes);
        response.set_payload(gzip_compress(waypoint_bytes));

        if (!rx_events_window_.empty())
        {
            protobuf_msgs::NetRxEventsPayload batch;
            const uint64_t t_end_us = Simulator::Now().ToInteger(Time::US);
            const uint64_t win_dur_us = static_cast<uint64_t>(std::llround(step_size * 1e6));
            const uint64_t t_start_us = (t_end_us > win_dur_us) ? (t_end_us - win_dur_us) : 0;

            batch.set_t_window_start_us(t_start_us);
            batch.set_t_window_end_us(t_end_us);

            for (const auto& e : rx_events_window_)
            {
                auto* ev = batch.add_events();
                ev->set_src_id(e.sender_id);
                ev->set_dst_id(e.rx_id);
                ev->set_flow_id(e.flow_id);
                ev->set_seq(e.seq);
                ev->set_tx_time_us(e.tx_time_us);
                ev->set_rx_time_us(e.rx_time_us);
                ev->set_delay_us(e.delay_us);
                ev->set_payload_bytes(e.pkt_bytes);
                ev->set_rssi_dbm_x10(0);
                ev->set_is_duplicate(e.is_duplicate);
                ev->set_wp_x(e.wp_x);
                ev->set_wp_y(e.wp_y);
                ev->set_wp_z(e.wp_z);
            }

            std::string raw;
            batch.SerializeToString(&raw);
            response.set_net_rx_events_gz(gzip_compress(raw));
            ROS_INFO("[NET] net_rx_events_gz sent events=%zu raw=%zuB gz=%zuB",
                     rx_events_window_.size(),
                     raw.size(),
                     response.net_rx_events_gz().size());
        }

        return response;
    }

    void ConfigureNs3()
    {
        SeedManager::SetRun(getYamlValue<int>(config_, "seed"));

        const std::string wifi_standard = getYamlValue<std::string>(config_, "wifi_standard");
        wifi_helper_ = std::make_unique<WifiHelper>();
        if (wifi_standard == "802.11g")
        {
            wifi_helper_->SetStandard(WIFI_STANDARD_80211g);
        }
        else if (wifi_standard == "802.11n")
        {
            wifi_helper_->SetStandard(WIFI_STANDARD_80211n);
        }
        else if (wifi_standard == "802.11ax")
        {
            wifi_helper_->SetStandard(WIFI_STANDARD_80211ax);
        }
        else
        {
            throw std::runtime_error("Unsupported WiFi standard: " + wifi_standard);
        }

        const std::string wifi_phy_mode = getYamlValue<std::string>(config_, "wifi_phy_mode");
        wifi_helper_->SetRemoteStationManager(
            "ns3::ConstantRateWifiManager",
            "DataMode", StringValue(wifi_phy_mode),
            "ControlMode", StringValue(wifi_phy_mode));
        Config::SetDefault("ns3::WifiRemoteStationManager::NonUnicastMode", StringValue(wifi_phy_mode));
        Config::SetDefault(
            "ns3::HtConfiguration::ShortGuardIntervalSupported",
            BooleanValue(getYamlValue<bool>(config_, "short_guard_interval_supported")));

        wifi_mac_helper_ = std::make_unique<WifiMacHelper>();
        wifi_mac_helper_->SetType("ns3::AdhocWifiMac");

        wifi_phy_helper_ = std::make_unique<YansWifiPhyHelper>();
        YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
        wifi_phy_helper_->SetChannel(channel.Create());

        internet_helper_ = std::make_unique<InternetStackHelper>();
    }

    void InitAgents()
    {
        const unsigned int robots = getYamlValue<unsigned int>(config_, "robots_number");
        std::vector<Vector> initial_positions;
        initial_positions.reserve(robots);

        if (config_["ns3_initial_positions"])
        {
            const YAML::Node& positions = config_["ns3_initial_positions"];
            if (!positions.IsSequence())
            {
                throw std::runtime_error("ns3_initial_positions must be a YAML sequence.");
            }
            if (positions.size() < robots)
            {
                throw std::runtime_error("ns3_initial_positions has fewer entries than robots_number.");
            }

            for (unsigned int i = 0; i < robots; ++i)
            {
                const YAML::Node& pose = positions[i];
                initial_positions.emplace_back(
                    getYamlValue<double>(pose, "x"),
                    getYamlValue<double>(pose, "y"),
                    getYamlValue<double>(pose, "z"));
            }
        }
        else
        {
            const double initial_x =
                config_["initial_x"] ? getYamlValue<double>(config_, "initial_x") : 0.0;
            const double initial_y =
                config_["initial_y"] ? getYamlValue<double>(config_, "initial_y") : 0.0;
            const double initial_z =
                config_["initial_z"] ? getYamlValue<double>(config_, "initial_z") : 1.0;
            const double initial_spacing =
                config_["initial_spacing"] ? getYamlValue<double>(config_, "initial_spacing") : 1.0;

            for (unsigned int i = 0; i < robots; ++i)
            {
                initial_positions.emplace_back(
                    initial_x + static_cast<double>(i) * initial_spacing,
                    initial_y,
                    initial_z);
            }
        }

        for (unsigned int i = 0; i < robots; ++i)
        {
            dancers_msgs::AgentStruct msg;
            msg.agent_id = i;
            msg.state.position.x = initial_positions[i].x;
            msg.state.position.y = initial_positions[i].y;
            msg.state.position.z = initial_positions[i].z;
            msg.state.velocity.x = 0.0;
            msg.state.velocity.y = 0.0;
            msg.state.velocity.z = 0.0;
            msg.state.heading = 0.0;
            SpawnUavClbk(msg);
        }
    }

    void SpawnUavClbk(const dancers_msgs::AgentStruct& msg)
    {
        {
            std::shared_lock<std::shared_mutex> lk(agents_mutex_);
            if (agents_.count(msg.agent_id) > 0)
            {
                ROS_WARN("Agent %d already exists, skipping spawn.", msg.agent_id);
                return;
            }
        }

        const uint32_t ns3_index = nodes_.GetN();
        Ptr<Node> node = CreateObject<Node>();
        nodes_.Add(node);

        Ptr<ConstantVelocityMobilityModel> mobility = CreateObject<ConstantVelocityMobilityModel>();
        mobility->SetPosition(Vector(msg.state.position.x, msg.state.position.y, msg.state.position.z));
        mobility->SetVelocity(Vector(msg.state.velocity.x, msg.state.velocity.y, msg.state.velocity.z));
        node->AggregateObject(mobility);

        BuildingsHelper::Install(node);
        AddWifiNetworkStack(node, msg.agent_id);

        std::unique_lock<std::shared_mutex> lk(agents_mutex_);
        auto [it, inserted] = agents_.emplace(msg.agent_id, std::make_unique<Agent>(AgentFromRosMsg(msg)));
        if (!inserted)
        {
            throw std::runtime_error("Failed to insert spawned agent.");
        }
        it->second->node = std::make_shared<Ptr<Node>>(node);
        it->second->node_container_index = ns3_index;
        it->second->cmd_velocity = Eigen::Vector3d::Zero();
        it->second->cmd_heading = 0.0;
        it->second->channel_busy_time = 0.0;
    }

    void AddWifiNetworkStack(Ptr<Node> node, uint32_t agent_id)
    {
        NetDeviceContainer net_device = wifi_helper_->Install(*wifi_phy_helper_, *wifi_mac_helper_, node);
        internet_helper_->Install(node);

        const std::string ip = "10.0.0." + std::to_string(agent_id + 1);
        Ipv4InterfaceAddress address(Ipv4Address(ip.c_str()), Ipv4Mask("255.255.255.0"));
        Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
        const int32_t interface_index = ipv4->AddInterface(net_device.Get(0));
        if (!ipv4->AddAddress(interface_index, address))
        {
            throw std::runtime_error("Could not assign IPv4 address to node.");
        }
        ipv4->SetUp(interface_index);
    }

    Ptr<WaypointBroadcaster> AddWaypointBroadcaster(Ptr<Node> node, uint32_t agent_id)
    {
        if (!config_["waypoint_flow"])
        {
            throw std::runtime_error("waypoint_flow section missing from Tutorial 3 config.");
        }

        Ptr<WaypointBroadcaster> app = CreateObject<WaypointBroadcaster>();
        app->SetStartTime(Seconds(0.1));
        app->SetStopTime(Seconds(simulation_length));
        app->SetAttribute("AdditionalSize", UintegerValue(0));
        app->SetAttribute("Port", UintegerValue(getYamlValue<uint32_t>(config_["waypoint_flow"], "port")));
        app->SetAttribute("FlowId", UintegerValue(getYamlValue<uint32_t>(config_["waypoint_flow"], "flow_id")));
        app->SetAttribute("SenderId", UintegerValue(agent_id));

        Ptr<ConstantRandomVariable> interval = CreateObject<ConstantRandomVariable>();
        interval->SetAttribute(
            "Constant",
            DoubleValue(getYamlValue<unsigned int>(config_["waypoint_flow"], "interval") / 1000000.0));
        app->SetAttribute("Interval", PointerValue(interval));

        node->AddApplication(app);
        app->TraceConnectWithoutContext("Tx", MakeCallback(&BasicWifiAdhoc::WaypointBroadcasterCb_, this));
        return app;
    }

    Ptr<WaypointReceiver> AddWaypointReceiver(Ptr<Node> node)
    {
        const auto agent_id = FindAgentIdByNode(node);
        if (!agent_id.has_value())
        {
            throw std::runtime_error("Could not map ns-3 node back to agent id.");
        }

        Ptr<WaypointReceiver> app = CreateObject<WaypointReceiver>();
        app->SetStartTime(Simulator::Now());
        app->SetStopTime(Seconds(simulation_length));
        app->SetAttribute("Port", UintegerValue(getYamlValue<uint32_t>(config_["waypoint_flow"], "port")));
        node->AddApplication(app);
        app->TraceConnect("Rx", std::to_string(agent_id.value()), MakeCallback(&BasicWifiAdhoc::WaypointReceiverCb_, this));
        return app;
    }

    void RandomWaypointGeneration(Ptr<WaypointBroadcaster> app, Time interval, Vector corner1, Vector corner2)
    {
        const double min_x = std::min(corner1.x, corner2.x);
        const double max_x = std::max(corner1.x, corner2.x);
        const double min_y = std::min(corner1.y, corner2.y);
        const double max_y = std::max(corner1.y, corner2.y);
        const double min_z = std::min(corner1.z, corner2.z);
        const double max_z = std::max(corner1.z, corner2.z);

        const double x = min_x + (max_x - min_x) * static_cast<double>(std::rand()) / RAND_MAX;
        const double y = min_y + (max_y - min_y) * static_cast<double>(std::rand()) / RAND_MAX;
        const double z = min_z + (max_z - min_z) * static_cast<double>(std::rand()) / RAND_MAX;
        app->SetWaypoint(Vector(x, y, z));

        ROS_INFO("New Tutorial 3 waypoint generated: (%.3f, %.3f, %.3f)", x, y, z);
        Simulator::Schedule(
            interval, &BasicWifiAdhoc::RandomWaypointGeneration, this, app, interval, corner1, corner2);
    }

    void SquareWaypointGeneration(
        Ptr<WaypointBroadcaster> app,
        Time interval,
        const std::array<Vector, 4>& corners,
        uint32_t corner_index)
    {
        const Vector& waypoint = corners.at(corner_index % static_cast<uint32_t>(corners.size()));
        app->SetWaypoint(waypoint);

        ROS_INFO("New Tutorial 3 square waypoint[%u]: (%.3f, %.3f, %.3f)",
                 corner_index % static_cast<uint32_t>(corners.size()),
                 waypoint.x,
                 waypoint.y,
                 waypoint.z);

        const uint32_t next_corner = (corner_index + 1) % static_cast<uint32_t>(corners.size());
        Simulator::Schedule(
            interval, &BasicWifiAdhoc::SquareWaypointGeneration, this, app, interval, corners, next_corner);
    }

    void UpdateAgentsPosition(const dancers_update_proto::DancersUpdate& update_msg)
    {
        if (update_msg.payload().empty())
        {
            return;
        }

        dancers_update_proto::PoseVector robots_positions;
        if (!robots_positions.ParseFromString(gzip_decompress(update_msg.payload())))
        {
            throw std::runtime_error("Failed to parse robots positions from coordinator payload.");
        }

        for (const auto& pose : robots_positions.pose())
        {
            std::unique_lock<std::shared_mutex> lk(agents_mutex_);
            auto it = agents_.find(pose.agent_id());
            if (it == agents_.end())
            {
                continue;
            }

            it->second->position = Eigen::Vector3d(pose.x(), pose.y(), pose.z());
            if (!it->second->initial_position_saved)
            {
                it->second->initial_position = it->second->position;
                it->second->initial_position_saved = true;
            }

            Ptr<ConstantVelocityMobilityModel> mobility =
                nodes_.Get(it->second->node_container_index)->GetObject<ConstantVelocityMobilityModel>();
            mobility->SetPosition(Vector(pose.x(), pose.y(), pose.z()));
            mobility->SetVelocity(Vector(pose.vx(), pose.vy(), pose.vz()));
        }
    }

    void WaypointReceiverCb_(std::string context, Ptr<const Packet> packet, int peer_id)
    {
        const uint32_t rx_id = static_cast<uint32_t>(std::stoul(context));
        const uint32_t tx_id_from_ip = static_cast<uint32_t>(peer_id);

        WaypointHeader header;
        packet->PeekHeader(header);

        const uint32_t tx_id = header.GetSenderId();
        const uint32_t flow_id = header.GetFlowId();
        const uint32_t seq = header.GetSeq();
        const uint64_t rx_us = Simulator::Now().ToInteger(Time::US);
        const uint64_t tx_us = header.GetTxTimeUs();
        const uint64_t delay_us = (tx_us > 0 && rx_us >= tx_us) ? (rx_us - tx_us) : 0;

        const Vector waypoint = header.GetPosition();
        current_waypoints_[rx_id] = waypoint;
        const Vector world = WaypointToWorldFrame(rx_id, waypoint);

        RxEventRow ev;
        ev.rx_time_us = rx_us;
        ev.rx_id = rx_id;
        ev.tx_time_us = tx_us;
        ev.tx_id = tx_id;
        ev.sender_id = tx_id;
        ev.flow_id = flow_id;
        ev.seq = seq;
        ev.pkt_bytes = packet->GetSize();
        ev.delay_us = delay_us;
        ev.wp_x = world.x;
        ev.wp_y = world.y;
        ev.wp_z = world.z;
        ev.is_duplicate = false;
        rx_events_window_.push_back(ev);

        if (tx_id != tx_id_from_ip)
        {
            ROS_WARN("Header sender_id=%u but IP-derived peer_id=%u", tx_id, tx_id_from_ip);
        }
    }

    void WaypointBroadcasterCb_(Ptr<const Packet> packet)
    {
        WaypointHeader header;
        packet->PeekHeader(header);

        const Vector waypoint = header.GetPosition();
        current_waypoints_[0] = waypoint;

        RxEventRow ev;
        ev.rx_time_us = Simulator::Now().ToInteger(Time::US);
        ev.rx_id = 0;
        ev.tx_time_us = header.GetTxTimeUs();
        ev.tx_id = header.GetSenderId();
        ev.sender_id = header.GetSenderId();
        ev.flow_id = header.GetFlowId();
        ev.seq = header.GetSeq();
        ev.pkt_bytes = packet->GetSize();
        ev.delay_us = 0;

        const Vector world = WaypointToWorldFrame(0, waypoint);
        ev.wp_x = world.x;
        ev.wp_y = world.y;
        ev.wp_z = world.z;
        rx_events_window_.push_back(ev);
    }

    std::optional<uint32_t> FindAgentIdByNode(const Ptr<Node>& node) const
    {
        std::shared_lock<std::shared_mutex> lk(agents_mutex_);
        for (const auto& kv : agents_)
        {
            if (kv.second->node_container_index == node->GetId())
            {
                return kv.first;
            }
        }
        return std::nullopt;
    }

    Vector WaypointToWorldFrame(uint32_t agent_id, const Vector& waypoint) const
    {
        Vector world = waypoint;
        std::shared_lock<std::shared_mutex> lk(agents_mutex_);
        const auto it = agents_.find(agent_id);
        if (it != agents_.end() && it->second->initial_position_saved)
        {
            world.x += it->second->initial_position.x();
            world.y += it->second->initial_position.y();
            world.z += it->second->initial_position.z();
        }
        return world;
    }

    std::map<uint32_t, std::unique_ptr<Agent>> agents_;
    mutable std::shared_mutex agents_mutex_;
    std::map<uint32_t, Vector> current_waypoints_;
    std::vector<RxEventRow> rx_events_window_;

    NodeContainer nodes_;
    std::unique_ptr<WifiHelper> wifi_helper_;
    std::unique_ptr<WifiMacHelper> wifi_mac_helper_;
    std::unique_ptr<YansWifiPhyHelper> wifi_phy_helper_;
    std::unique_ptr<InternetStackHelper> internet_helper_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "basic_wifi_adhoc");
    BasicWifiAdhoc node;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    node.RunLoop();
    return 0;
}
