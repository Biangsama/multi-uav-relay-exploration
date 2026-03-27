#include "applications/waypoint_broadcast.h"

#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/stats-module.h"
#include "ns3/flow-id-tag.h"

#include <cstdint>
#include <cstring>

using namespace ns3;

#define WAYPOINT_HEADER_SIZE 32

NS_LOG_COMPONENT_DEFINE("WiFiAppWaypointBroadcast");

// ----------------------
// Helpers: float <-> u32 network order
// ----------------------
static inline void
WriteFloatHton(Buffer::Iterator& i, float v)
{
  uint32_t u = 0;
  static_assert(sizeof(float) == sizeof(uint32_t), "float must be 32-bit");
  std::memcpy(&u, &v, sizeof(uint32_t));
  i.WriteHtonU32(u);
}

static inline float
ReadFloatNtoh(Buffer::Iterator& i)
{
  uint32_t u = i.ReadNtohU32();
  float v = 0.0f;
  std::memcpy(&v, &u, sizeof(uint32_t));
  return v;
}

static inline void
WriteU64Hton(Buffer::Iterator& i, uint64_t v)
{
  i.WriteHtonU64(v);
}

static inline uint64_t
ReadU64Ntoh(Buffer::Iterator& i)
{
  return i.ReadNtohU64();
}

//----------------------------------------------------------------------
//-- WaypointBroadcaster
//------------------------------------------------------
TypeId
WaypointBroadcaster::GetTypeId()
{
  static TypeId tid =
      TypeId("ns3::WaypointBroadcaster")
          .SetParent<Application>()
          .AddConstructor<WaypointBroadcaster>()
          .AddAttribute("AdditionalSize",
                        "The size in bytes of additional data appended after the waypoint header.",
                        UintegerValue(0),
                        MakeUintegerAccessor(&WaypointBroadcaster::m_pktSize),
                        MakeUintegerChecker<uint32_t>())
          .AddAttribute("Port",
                        "Destination app port.",
                        UintegerValue(4000),
                        MakeUintegerAccessor(&WaypointBroadcaster::m_destPort),
                        MakeUintegerChecker<uint32_t>())
          .AddAttribute("Interval",
                        "Delay between transmissions in seconds.",
                        StringValue("ns3::ConstantRandomVariable[Constant=0.5]"),
                        MakePointerAccessor(&WaypointBroadcaster::m_interval),
                        MakePointerChecker<RandomVariableStream>())
          .AddAttribute("FlowId",
                        "Optional FlowIdTag set on packets",
                        UintegerValue(2),
                        MakeUintegerAccessor(&WaypointBroadcaster::m_flowId),
                        MakeUintegerChecker<uint32_t>())
          .AddAttribute("SenderId",
                        "Logical sender id (agent id) written into WaypointHeader.",
                        UintegerValue(0),
                        MakeUintegerAccessor(&WaypointBroadcaster::m_senderId),
                        MakeUintegerChecker<uint32_t>())
          .AddTraceSource("Tx",
                          "A new packet is created and is sent",
                          MakeTraceSourceAccessor(&WaypointBroadcaster::m_txTrace),
                          "ns3::Packet::TracedCallback");
  return tid;
}

WaypointBroadcaster::WaypointBroadcaster()
{
  NS_LOG_FUNCTION_NOARGS();
  m_interval = CreateObject<ConstantRandomVariable>();
  m_socket = nullptr;
  m_sent = 0;
  m_count = 0;
  m_waypoint = Vector(0.0, 0.0, 0.0);
}

WaypointBroadcaster::~WaypointBroadcaster()
{
  NS_LOG_FUNCTION_NOARGS();
}

void
WaypointBroadcaster::DoDispose()
{
  NS_LOG_FUNCTION_NOARGS();
  m_socket = nullptr;
  Application::DoDispose();
}

void
WaypointBroadcaster::StartApplication()
{
  NS_LOG_FUNCTION_NOARGS();

  if (!m_socket)
  {
    Ptr<SocketFactory> socketFactory =
        GetNode()->GetObject<SocketFactory>(UdpSocketFactory::GetTypeId());
    m_socket = socketFactory->CreateSocket();
    m_socket->Bind();
    m_socket->SetAllowBroadcast(true);
  }

  m_count = 0;
  Simulator::Cancel(m_sendEvent);
  m_sendEvent = Simulator::ScheduleNow(&WaypointBroadcaster::SendPacket, this);
}

void
WaypointBroadcaster::StopApplication()
{
  NS_LOG_FUNCTION_NOARGS();
  Simulator::Cancel(m_sendEvent);
}

void
WaypointBroadcaster::SendPacket()
{
  NS_LOG_INFO("WaypointBroadcaster sending at " << Simulator::Now().GetSeconds() << " s");

  Ptr<Packet> packet = (m_pktSize > 0) ? Create<Packet>(m_pktSize) : Create<Packet>();

  // Header
  WaypointHeader waypoint_header;
  waypoint_header.SetPosition(m_waypoint);
  waypoint_header.SetSenderId(m_senderId);
  waypoint_header.SetFlowId(m_flowId);
  waypoint_header.SetSeq(m_count);
  waypoint_header.SetTxTimeUs(Simulator::Now().ToInteger(Time::US));
  packet->AddHeader(waypoint_header);

  // Optional tag
  FlowIdTag flow_id;
  flow_id.SetFlowId(m_flowId);
  packet->AddPacketTag(flow_id);

  // Broadcast
  m_socket->SendTo(packet, 0, InetSocketAddress(Ipv4Address::GetBroadcast(), m_destPort));

  // Trace
  m_txTrace(packet);

  m_sent++;
  m_count++;

  // Next
  m_sendEvent = Simulator::Schedule(Seconds(m_interval->GetValue()),
                                    &WaypointBroadcaster::SendPacket,
                                    this);
}

uint64_t
WaypointBroadcaster::GetSent() const
{
  return m_sent;
}

void
WaypointBroadcaster::SetWaypoint(Vector waypoint)
{
  m_waypoint = waypoint;
}

//----------------------------------------------------------------------
//-- WaypointReceiver
//------------------------------------------------------

// 更明确的 Trace 签名字符串（不影响编译，只影响 ns-3 trace 标识）
typedef void (*WaypointRxTraceCallback)(Ptr<const Packet>, int);

TypeId
WaypointReceiver::GetTypeId()
{
  static TypeId tid =
      TypeId("ns3::WaypointReceiver")
          .SetParent<Application>()
          .AddConstructor<WaypointReceiver>()
          .AddAttribute("Port",
                        "Listening port.",
                        UintegerValue(4000),
                        MakeUintegerAccessor(&WaypointReceiver::m_port),
                        MakeUintegerChecker<uint32_t>())
          .AddTraceSource("Rx",
                          "A new packet is received",
                          MakeTraceSourceAccessor(&WaypointReceiver::m_rxTrace),
                          "ns3::WaypointRxTraceCallback");
  return tid;
}

WaypointReceiver::WaypointReceiver()
  : m_received(0)
{
  NS_LOG_FUNCTION_NOARGS();
  m_socket = nullptr;
}

WaypointReceiver::~WaypointReceiver()
{
  NS_LOG_FUNCTION_NOARGS();
}

void
WaypointReceiver::DoDispose()
{
  NS_LOG_FUNCTION_NOARGS();
  m_socket = nullptr;
  Application::DoDispose();
}

void
WaypointReceiver::StartApplication()
{
  NS_LOG_FUNCTION_NOARGS();

  if (!m_socket)
  {
    Ptr<SocketFactory> socketFactory =
        GetNode()->GetObject<SocketFactory>(UdpSocketFactory::GetTypeId());
    m_socket = socketFactory->CreateSocket();
    InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(), m_port);
    if (m_socket->Bind(local) == -1)
    {
      NS_FATAL_ERROR("WaypointReceiver: Failed to bind socket");
    }
  }

  m_socket->SetRecvCallback(MakeCallback(&WaypointReceiver::Receive, this));
}

void
WaypointReceiver::StopApplication()
{
  NS_LOG_FUNCTION_NOARGS();
  if (m_socket)
  {
    m_socket->SetRecvCallback(MakeNullCallback<void, Ptr<Socket>>());
  }
}

void
WaypointReceiver::Receive(Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  Address from;

  while ((packet = socket->RecvFrom(from)))
  {
    if (!InetSocketAddress::IsMatchingType(from))
    {
      continue;
    }

    Ipv4Address peer_address = InetSocketAddress::ConvertFrom(from).GetIpv4();
    // /24 addresses with last octet = peer_id + 1 (10.0.0.<id+1>)
    uint32_t peer_id = peer_address.CombineMask("0.0.0.255").Get() - 1;

    // Trace
    m_rxTrace(packet, static_cast<int>(peer_id));
    m_received++;
  }
}

uint64_t
WaypointReceiver::GetReceived() const
{
  return m_received;
}

//----------------------------------------------------------------------
//-- WaypointHeader
//------------------------------------------------------
WaypointHeader::WaypointHeader()
  : m_position(0.0, 0.0, 0.0)
{
}

WaypointHeader::~WaypointHeader() {}

TypeId
WaypointHeader::GetTypeId()
{
  static TypeId tid =
      TypeId("ns3::WaypointHeader")
          .SetParent<Header>()
          .AddConstructor<WaypointHeader>();
  return tid;
}

TypeId
WaypointHeader::GetInstanceTypeId() const
{
  return GetTypeId();
}

uint32_t
WaypointHeader::GetSerializedSize() const
{
  return WAYPOINT_HEADER_SIZE;
}

void
WaypointHeader::Print(std::ostream& os) const
{
  os << "Waypoint: (" << m_position.x << ", " << m_position.y << ", " << m_position.z << ")"
     << " sender=" << m_sender_id
     << " flow=" << m_flow_id
     << " seq=" << m_seq
     << " tx_us=" << m_tx_time_us;
}

void
WaypointHeader::Serialize(Buffer::Iterator start) const
{
  Buffer::Iterator i = start;
  WriteFloatHton(i, static_cast<float>(m_position.x));
  WriteFloatHton(i, static_cast<float>(m_position.y));
  WriteFloatHton(i, static_cast<float>(m_position.z));

  i.WriteHtonU32(m_sender_id);
  i.WriteHtonU32(m_flow_id);
  i.WriteHtonU32(m_seq);
  WriteU64Hton(i, m_tx_time_us);
}

uint32_t
WaypointHeader::Deserialize(Buffer::Iterator start)
{
  Buffer::Iterator i = start;
  m_position.x = ReadFloatNtoh(i);
  m_position.y = ReadFloatNtoh(i);
  m_position.z = ReadFloatNtoh(i);

  m_sender_id = i.ReadNtohU32();
  m_flow_id = i.ReadNtohU32();
  m_seq = i.ReadNtohU32();
  m_tx_time_us = ReadU64Ntoh(i);
  return GetSerializedSize();
}

void
WaypointHeader::SetPosition(Vector position)
{
  m_position = position;
}

Vector
WaypointHeader::GetPosition() const
{
  return m_position;
}

void
WaypointHeader::SetSenderId(uint32_t id)
{
  m_sender_id = id;
}

uint32_t
WaypointHeader::GetSenderId() const
{
  return m_sender_id;
}

void
WaypointHeader::SetFlowId(uint32_t id)
{
  m_flow_id = id;
}

uint32_t
WaypointHeader::GetFlowId() const
{
  return m_flow_id;
}

void
WaypointHeader::SetSeq(uint32_t s)
{
  m_seq = s;
}

uint32_t
WaypointHeader::GetSeq() const
{
  return m_seq;
}

void
WaypointHeader::SetTxTimeUs(uint64_t t_us)
{
  m_tx_time_us = t_us;
}

uint64_t
WaypointHeader::GetTxTimeUs() const
{
  return m_tx_time_us;
}
