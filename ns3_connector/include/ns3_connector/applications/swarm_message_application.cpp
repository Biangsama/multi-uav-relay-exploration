#include "applications/swarm_message_application.h"

#include "ns3/core-module.h"
#include "ns3/flow-id-tag.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include <protobuf_msgs/racer_swarm_msg.pb.h>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("SwarmMessageApplication");

namespace {

bool ShouldTraceOwnershipFamily(const relay_racer_proto::RacerSwarmMsg& wrapper)
{
  return wrapper.family() == "allocation_request" || wrapper.family() == "release_request";
}

bool DecodeSwarmPacketPayload(Ptr<const Packet> packet,
                              SwarmMessageHeader* header,
                              relay_racer_proto::RacerSwarmMsg* wrapper,
                              uint32_t* payload_bytes)
{
  if (!packet)
  {
    return false;
  }

  Ptr<Packet> copy = packet->Copy();
  SwarmMessageHeader local_header;
  if (copy->GetSize() < local_header.GetSerializedSize())
  {
    return false;
  }

  copy->RemoveHeader(local_header);
  const uint32_t local_payload_bytes = copy->GetSize();
  std::string payload(local_payload_bytes, '\0');
  if (local_payload_bytes > 0)
  {
    copy->CopyData(reinterpret_cast<uint8_t*>(&payload[0]), local_payload_bytes);
  }

  if (header)
  {
    *header = local_header;
  }
  if (payload_bytes)
  {
    *payload_bytes = local_payload_bytes;
  }

  return wrapper && wrapper->ParseFromString(payload);
}

void LogOwnershipSendPayload(const relay_racer_proto::RacerSwarmMsg& wrapper,
                             uint32_t sender_id,
                             uint32_t flow_id,
                             uint32_t dest_port,
                             uint32_t seq,
                             uint32_t packet_bytes,
                             int send_ret)
{
  if (!ShouldTraceOwnershipFamily(wrapper))
  {
    return;
  }

  NS_LOG_UNCOND("[SWARM_APP][send_payload]"
                << " family=" << wrapper.family()
                << " src_id=" << wrapper.src_id()
                << " dst_id=" << wrapper.dst_id()
                << " network_tx_id=" << wrapper.network_tx_id()
                << " payload_bytes=" << wrapper.ros_payload().size()
                << " packet_bytes=" << packet_bytes
                << " sender_id=" << sender_id
                << " flow_id=" << flow_id
                << " dest_port=" << dest_port
                << " seq=" << seq
                << " send_ret=" << send_ret);
}

void LogOwnershipRecvPacket(const relay_racer_proto::RacerSwarmMsg& wrapper,
                            const SwarmMessageHeader& header,
                            uint32_t local_port,
                            uint32_t peer_id,
                            const InetSocketAddress& remote,
                            uint32_t payload_bytes,
                            uint32_t packet_bytes)
{
  if (!ShouldTraceOwnershipFamily(wrapper))
  {
    return;
  }

  NS_LOG_UNCOND("[SWARM_APP][recv_packet]"
                << " family=" << wrapper.family()
                << " src_id=" << wrapper.src_id()
                << " dst_id=" << wrapper.dst_id()
                << " network_tx_id=" << wrapper.network_tx_id()
                << " payload_bytes=" << payload_bytes
                << " packet_bytes=" << packet_bytes
                << " sender_id=" << header.GetSenderId()
                << " flow_id=" << header.GetFlowId()
                << " seq=" << header.GetSeq()
                << " local_port=" << local_port
                << " from=" << remote.GetIpv4()
                << ":" << remote.GetPort()
                << " peer_id=" << peer_id);
}

void LogOwnershipEnqueuePayload(const relay_racer_proto::RacerSwarmMsg& wrapper,
                                uint32_t sender_id,
                                uint32_t flow_id,
                                uint32_t dest_port,
                                size_t pending_before,
                                size_t pending_after,
                                bool running)
{
  if (!ShouldTraceOwnershipFamily(wrapper))
  {
    return;
  }

  NS_LOG_UNCOND("[SWARM_APP][enqueue_payload]"
                << " family=" << wrapper.family()
                << " src_id=" << wrapper.src_id()
                << " dst_id=" << wrapper.dst_id()
                << " network_tx_id=" << wrapper.network_tx_id()
                << " payload_bytes=" << wrapper.ros_payload().size()
                << " sender_id=" << sender_id
                << " flow_id=" << flow_id
                << " dest_port=" << dest_port
                << " pending_before=" << pending_before
                << " pending_after=" << pending_after
                << " running=" << running);
}

void LogOwnershipFlushQueue(const relay_racer_proto::RacerSwarmMsg& wrapper,
                            uint32_t sender_id,
                            uint32_t flow_id,
                            uint32_t dest_port,
                            size_t pending_before,
                            bool running,
                            bool has_socket)
{
  if (!ShouldTraceOwnershipFamily(wrapper))
  {
    return;
  }

  NS_LOG_UNCOND("[SWARM_APP][flush_queue]"
                << " family=" << wrapper.family()
                << " src_id=" << wrapper.src_id()
                << " dst_id=" << wrapper.dst_id()
                << " network_tx_id=" << wrapper.network_tx_id()
                << " payload_bytes=" << wrapper.ros_payload().size()
                << " sender_id=" << sender_id
                << " flow_id=" << flow_id
                << " dest_port=" << dest_port
                << " pending_before=" << pending_before
                << " running=" << running
                << " has_socket=" << has_socket);
}

void LogOwnershipSendPayloadBegin(const relay_racer_proto::RacerSwarmMsg& wrapper,
                                  uint32_t sender_id,
                                  uint32_t flow_id,
                                  uint32_t dest_port,
                                  uint32_t seq,
                                  uint32_t packet_bytes)
{
  if (!ShouldTraceOwnershipFamily(wrapper))
  {
    return;
  }

  NS_LOG_UNCOND("[SWARM_APP][send_payload_begin]"
                << " family=" << wrapper.family()
                << " src_id=" << wrapper.src_id()
                << " dst_id=" << wrapper.dst_id()
                << " network_tx_id=" << wrapper.network_tx_id()
                << " payload_bytes=" << wrapper.ros_payload().size()
                << " packet_bytes=" << packet_bytes
                << " sender_id=" << sender_id
                << " flow_id=" << flow_id
                << " dest_port=" << dest_port
                << " seq=" << seq);
}

}  // namespace

TypeId SwarmMessageBroadcaster::GetTypeId()
{
  static TypeId tid =
      TypeId("ns3::SwarmMessageBroadcaster")
          .SetParent<Application>()
          .AddConstructor<SwarmMessageBroadcaster>()
          .AddAttribute("Port",
                        "Destination app port.",
                        UintegerValue(4100),
                        MakeUintegerAccessor(&SwarmMessageBroadcaster::m_destPort),
                        MakeUintegerChecker<uint32_t>())
          .AddAttribute("FlowId",
                        "Flow Id stored in SwarmMessageHeader.",
                        UintegerValue(30),
                        MakeUintegerAccessor(&SwarmMessageBroadcaster::m_flowId),
                        MakeUintegerChecker<uint32_t>())
          .AddAttribute("SenderId",
                        "Logical sender id used by the swarm message transport.",
                        UintegerValue(0),
                        MakeUintegerAccessor(&SwarmMessageBroadcaster::m_senderId),
                        MakeUintegerChecker<uint32_t>())
          .AddTraceSource("Tx",
                          "A queued swarm packet is sent",
                          MakeTraceSourceAccessor(&SwarmMessageBroadcaster::m_txTrace),
                          "ns3::Packet::TracedCallback");
  return tid;
}

SwarmMessageBroadcaster::SwarmMessageBroadcaster() = default;
SwarmMessageBroadcaster::~SwarmMessageBroadcaster() = default;

void SwarmMessageBroadcaster::DoDispose()
{
  Simulator::Cancel(m_flushEvent);
  m_socket = nullptr;
  Application::DoDispose();
}

void SwarmMessageBroadcaster::StartApplication()
{
  if (!m_socket)
  {
    Ptr<SocketFactory> socketFactory =
        GetNode()->GetObject<SocketFactory>(UdpSocketFactory::GetTypeId());
    m_socket = socketFactory->CreateSocket();
    m_socket->Bind();
    m_socket->SetAllowBroadcast(true);
  }

  m_running = true;
  if (!m_pendingPayloads.empty())
  {
    Simulator::Cancel(m_flushEvent);
    m_flushEvent = Simulator::ScheduleNow(&SwarmMessageBroadcaster::FlushQueue, this);
  }
}

void SwarmMessageBroadcaster::StopApplication()
{
  m_running = false;
  Simulator::Cancel(m_flushEvent);
}

void SwarmMessageBroadcaster::EnqueuePayload(const std::string& payload)
{
  relay_racer_proto::RacerSwarmMsg wrapper;
  const bool parsed = wrapper.ParseFromString(payload);
  const size_t pending_before = m_pendingPayloads.size();
  m_pendingPayloads.push_back(payload);
  if (parsed)
  {
    LogOwnershipEnqueuePayload(
        wrapper, m_senderId, m_flowId, m_destPort, pending_before, m_pendingPayloads.size(), m_running);
  }
  if (m_running)
  {
    Simulator::Cancel(m_flushEvent);
    m_flushEvent = Simulator::ScheduleNow(&SwarmMessageBroadcaster::FlushQueue, this);
  }
}

void SwarmMessageBroadcaster::FlushQueue()
{
  while (m_running && m_socket && !m_pendingPayloads.empty())
  {
    relay_racer_proto::RacerSwarmMsg wrapper;
    if (wrapper.ParseFromString(m_pendingPayloads.front()))
    {
      LogOwnershipFlushQueue(
          wrapper, m_senderId, m_flowId, m_destPort, m_pendingPayloads.size(), m_running, m_socket != nullptr);
    }
    SendPayload(m_pendingPayloads.front());
    m_pendingPayloads.pop_front();
  }
}

void SwarmMessageBroadcaster::SendPayload(const std::string& payload)
{
  relay_racer_proto::RacerSwarmMsg wrapper;
  const bool parsed = wrapper.ParseFromString(payload);
  Ptr<Packet> packet = payload.empty()
      ? Create<Packet>()
      : Create<Packet>(reinterpret_cast<const uint8_t*>(payload.data()), payload.size());

  SwarmMessageHeader header;
  header.SetSenderId(m_senderId);
  header.SetFlowId(m_flowId);
  header.SetSeq(m_seq++);
  header.SetTxTimeUs(Simulator::Now().ToInteger(Time::US));
  packet->AddHeader(header);

  FlowIdTag flow_id;
  flow_id.SetFlowId(m_flowId);
  packet->AddPacketTag(flow_id);

  const uint32_t seq = header.GetSeq();
  if (parsed)
  {
    LogOwnershipSendPayloadBegin(wrapper, m_senderId, m_flowId, m_destPort, seq, packet->GetSize());
  }
  const int send_ret =
      m_socket->SendTo(packet, 0, InetSocketAddress(Ipv4Address::GetBroadcast(), m_destPort));
  if (parsed)
  {
    LogOwnershipSendPayload(
        wrapper, m_senderId, m_flowId, m_destPort, seq, packet->GetSize(), send_ret);
  }
  else
  {
    NS_LOG_UNCOND("[SWARM_APP][send_payload_unparsed]"
                  << " payload_bytes=" << payload.size()
                  << " packet_bytes=" << packet->GetSize()
                  << " sender_id=" << m_senderId
                  << " flow_id=" << m_flowId
                  << " dest_port=" << m_destPort
                  << " seq=" << seq
                  << " send_ret=" << send_ret);
  }
  m_txTrace(packet);
  ++m_sent;
}

uint64_t SwarmMessageBroadcaster::GetSent() const
{
  return m_sent;
}

typedef void (*SwarmMessageRxTraceCallback)(Ptr<const Packet>, int);

TypeId SwarmMessageReceiver::GetTypeId()
{
  static TypeId tid =
      TypeId("ns3::SwarmMessageReceiver")
          .SetParent<Application>()
          .AddConstructor<SwarmMessageReceiver>()
          .AddAttribute("Port",
                        "Listening port.",
                        UintegerValue(4100),
                        MakeUintegerAccessor(&SwarmMessageReceiver::m_port),
                        MakeUintegerChecker<uint32_t>())
          .AddTraceSource("Rx",
                          "A swarm packet is received",
                          MakeTraceSourceAccessor(&SwarmMessageReceiver::m_rxTrace),
                          "ns3::SwarmMessageRxTraceCallback");
  return tid;
}

SwarmMessageReceiver::SwarmMessageReceiver() = default;
SwarmMessageReceiver::~SwarmMessageReceiver() = default;

void SwarmMessageReceiver::DoDispose()
{
  m_socket = nullptr;
  Application::DoDispose();
}

void SwarmMessageReceiver::StartApplication()
{
  if (!m_socket)
  {
    Ptr<SocketFactory> socketFactory =
        GetNode()->GetObject<SocketFactory>(UdpSocketFactory::GetTypeId());
    m_socket = socketFactory->CreateSocket();
    InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(), m_port);
    if (m_socket->Bind(local) == -1)
    {
      NS_FATAL_ERROR("SwarmMessageReceiver: Failed to bind socket");
    }
  }

  m_socket->SetRecvCallback(MakeCallback(&SwarmMessageReceiver::Receive, this));
}

void SwarmMessageReceiver::StopApplication()
{
  if (m_socket)
  {
    m_socket->SetRecvCallback(MakeNullCallback<void, Ptr<Socket>>());
  }
}

void SwarmMessageReceiver::Receive(Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  Address from;
  uint32_t drained = 0;

  NS_LOG_UNCOND("[SWARM_APP][recv_poll_begin]"
                << " local_port=" << m_port
                << " available_before=" << socket->GetRxAvailable());

  while ((packet = socket->RecvFrom(from)))
  {
    ++drained;
    if (!InetSocketAddress::IsMatchingType(from))
    {
      continue;
    }

    const InetSocketAddress remote = InetSocketAddress::ConvertFrom(from);
    Ipv4Address peer_address = remote.GetIpv4();
    uint32_t peer_id = peer_address.CombineMask("0.0.0.255").Get() - 1;
    SwarmMessageHeader header;
    relay_racer_proto::RacerSwarmMsg wrapper;
    uint32_t payload_bytes = 0;
    if (DecodeSwarmPacketPayload(packet, &header, &wrapper, &payload_bytes))
    {
      LogOwnershipRecvPacket(
          wrapper, header, m_port, peer_id, remote, payload_bytes, packet->GetSize());
    }
    else
    {
      NS_LOG_UNCOND("[SWARM_APP][recv_packet_unparsed]"
                    << " local_port=" << m_port
                    << " packet_bytes=" << packet->GetSize()
                    << " from=" << remote.GetIpv4()
                    << ":" << remote.GetPort()
                    << " peer_id=" << peer_id);
    }

    m_rxTrace(packet, static_cast<int>(peer_id));
    ++m_received;
  }

  NS_LOG_UNCOND("[SWARM_APP][recv_poll_end]"
                << " local_port=" << m_port
                << " drained=" << drained
                << " available_after=" << socket->GetRxAvailable());
}

uint64_t SwarmMessageReceiver::GetReceived() const
{
  return m_received;
}

SwarmMessageHeader::SwarmMessageHeader() = default;
SwarmMessageHeader::~SwarmMessageHeader() = default;

TypeId SwarmMessageHeader::GetTypeId()
{
  static TypeId tid = TypeId("ns3::SwarmMessageHeader")
      .SetParent<Header>()
      .AddConstructor<SwarmMessageHeader>();
  return tid;
}

TypeId SwarmMessageHeader::GetInstanceTypeId() const
{
  return GetTypeId();
}

void SwarmMessageHeader::Print(std::ostream& os) const
{
  os << "SwarmMessageHeader{sender_id=" << m_sender_id
     << ", flow_id=" << m_flow_id
     << ", seq=" << m_seq
     << ", tx_us=" << m_tx_time_us << "}";
}

uint32_t SwarmMessageHeader::GetSerializedSize() const
{
  return 20;
}

void SwarmMessageHeader::Serialize(Buffer::Iterator start) const
{
  Buffer::Iterator i = start;
  i.WriteHtonU32(m_sender_id);
  i.WriteHtonU32(m_flow_id);
  i.WriteHtonU32(m_seq);
  i.WriteHtonU64(m_tx_time_us);
}

uint32_t SwarmMessageHeader::Deserialize(Buffer::Iterator start)
{
  Buffer::Iterator i = start;
  m_sender_id = i.ReadNtohU32();
  m_flow_id = i.ReadNtohU32();
  m_seq = i.ReadNtohU32();
  m_tx_time_us = i.ReadNtohU64();
  return GetSerializedSize();
}

void SwarmMessageHeader::SetSenderId(uint32_t sender_id)
{
  m_sender_id = sender_id;
}

uint32_t SwarmMessageHeader::GetSenderId() const
{
  return m_sender_id;
}

void SwarmMessageHeader::SetFlowId(uint32_t flow_id)
{
  m_flow_id = flow_id;
}

uint32_t SwarmMessageHeader::GetFlowId() const
{
  return m_flow_id;
}

void SwarmMessageHeader::SetSeq(uint32_t seq)
{
  m_seq = seq;
}

uint32_t SwarmMessageHeader::GetSeq() const
{
  return m_seq;
}

void SwarmMessageHeader::SetTxTimeUs(uint64_t tx_time_us)
{
  m_tx_time_us = tx_time_us;
}

uint64_t SwarmMessageHeader::GetTxTimeUs() const
{
  return m_tx_time_us;
}
