#include "applications/flocking-application.h"

#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/stats-module.h"
#include "ns3/mobility-module.h"

#include <cstdint>
#include <cstring>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("FlockingApplication");

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

//----------------------------------------------------------------------
//-- FlockingBroadcaster
//------------------------------------------------------
TypeId
FlockingBroadcaster::GetTypeId()
{
  static TypeId tid =
      TypeId("ns3::FlockingBroadcaster")
          .SetParent<Application>()
          .AddConstructor<FlockingBroadcaster>()
          .AddAttribute("PacketSize",
                        "The packet size in bytes (payload size, excluding headers).",
                        UintegerValue(0),
                        MakeUintegerAccessor(&FlockingBroadcaster::m_pktSize),
                        MakeUintegerChecker<uint32_t>())
          .AddAttribute("Port",
                        "Destination app port.",
                        UintegerValue(4000),
                        MakeUintegerAccessor(&FlockingBroadcaster::m_destPort),
                        MakeUintegerChecker<uint32_t>())
          .AddAttribute("Interval",
                        "Delay between transmissions in seconds.",
                        StringValue("ns3::ConstantRandomVariable[Constant=0.1]"),
                        MakePointerAccessor(&FlockingBroadcaster::m_interval),
                        MakePointerChecker<RandomVariableStream>())
          .AddAttribute("FlowId",
                        "Flow Id stored in FlockingHeader.",
                        UintegerValue(2),
                        MakeUintegerAccessor(&FlockingBroadcaster::m_flowId),
                        MakeUintegerChecker<uint32_t>())
          .AddTraceSource("Tx",
                          "A new packet is created and is sent",
                          MakeTraceSourceAccessor(&FlockingBroadcaster::m_txTrace),
                          "ns3::Packet::TracedCallback");
  return tid;
}

FlockingBroadcaster::FlockingBroadcaster()
{
  NS_LOG_FUNCTION_NOARGS();
  m_interval = CreateObject<ConstantRandomVariable>();
  m_socket = nullptr;
  m_sent = 0;
  m_count = 0;
  m_seq = 0;
}

FlockingBroadcaster::~FlockingBroadcaster()
{
  NS_LOG_FUNCTION_NOARGS();
}

void
FlockingBroadcaster::DoDispose()
{
  NS_LOG_FUNCTION_NOARGS();
  m_socket = nullptr;
  Application::DoDispose();
}

void
FlockingBroadcaster::StartApplication()
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
  m_sendEvent = Simulator::ScheduleNow(&FlockingBroadcaster::SendPacket, this);
}

void
FlockingBroadcaster::StopApplication()
{
  NS_LOG_FUNCTION_NOARGS();
  Simulator::Cancel(m_sendEvent);
}

void
FlockingBroadcaster::SendPacket()
{
  NS_LOG_INFO("FlockingBroadcaster sending at " << Simulator::Now().GetSeconds() << " s");

  Ptr<Packet> packet = (m_pktSize > 0) ? Create<Packet>(m_pktSize) : Create<Packet>();

  // Header: position/velocity + (sender_id, flow_id, seq, tx_time_us)
  FlockingHeader hdr;

  Ptr<MobilityModel> mob = GetNode()->GetObject<MobilityModel>();
  if (mob)
  {
    hdr.SetPosition(mob->GetPosition());
    hdr.SetVelocity(mob->GetVelocity());
  }
  else
  {
    hdr.SetPosition(Vector(0.0, 0.0, 0.0));
    hdr.SetVelocity(Vector(0.0, 0.0, 0.0));
  }

  // scheme-1 fields
  hdr.SetSenderId(GetNode()->GetId());                   // best-effort stable node id
  hdr.SetFlowId(m_flowId);
  hdr.SetSeq(m_seq++);
  hdr.SetTxTimeUs(Simulator::Now().ToInteger(Time::US)); // tx timestamp in us

  packet->AddHeader(hdr);

  // Broadcast
  m_socket->SendTo(packet, 0, InetSocketAddress(Ipv4Address::GetBroadcast(), m_destPort));

  // Trace
  m_txTrace(packet);

  m_sent++;
  m_count++;

  // Next
  m_sendEvent = Simulator::Schedule(Seconds(m_interval->GetValue()),
                                    &FlockingBroadcaster::SendPacket,
                                    this);
}

uint64_t
FlockingBroadcaster::GetSent() const
{
  return m_sent;
}

//----------------------------------------------------------------------
//-- FlockingReceiver
//------------------------------------------------------
typedef void (*FlockingRxTraceCallback)(Ptr<const Packet>, int);

TypeId
FlockingReceiver::GetTypeId()
{
  static TypeId tid =
      TypeId("ns3::FlockingReceiver")
          .SetParent<Application>()
          .AddConstructor<FlockingReceiver>()
          .AddAttribute("Port",
                        "Listening port.",
                        UintegerValue(4000),
                        MakeUintegerAccessor(&FlockingReceiver::m_port),
                        MakeUintegerChecker<uint32_t>())
          .AddTraceSource("Rx",
                          "A new packet is received",
                          MakeTraceSourceAccessor(&FlockingReceiver::m_rxTrace),
                          "ns3::FlockingRxTraceCallback");
  return tid;
}

FlockingReceiver::FlockingReceiver()
  : m_received(0)
{
  NS_LOG_FUNCTION_NOARGS();
  m_socket = nullptr;
  m_calc = nullptr;
  m_delay = nullptr;
}

FlockingReceiver::~FlockingReceiver()
{
  NS_LOG_FUNCTION_NOARGS();
}

void
FlockingReceiver::SetCounter(Ptr<CounterCalculator<>> calc)
{
  m_calc = calc;
}

void
FlockingReceiver::SetDelayTracker(Ptr<TimeMinMaxAvgTotalCalculator> delay)
{
  m_delay = delay;
}

uint64_t
FlockingReceiver::GetReceived() const
{
  return m_received;
}

void
FlockingReceiver::DoDispose()
{
  NS_LOG_FUNCTION_NOARGS();
  m_socket = nullptr;
  Application::DoDispose();
}

void
FlockingReceiver::StartApplication()
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
      NS_FATAL_ERROR("FlockingReceiver: Failed to bind socket");
    }
  }

  m_socket->SetRecvCallback(MakeCallback(&FlockingReceiver::Receive, this));
}

void
FlockingReceiver::StopApplication()
{
  NS_LOG_FUNCTION_NOARGS();
  if (m_socket)
  {
    m_socket->SetRecvCallback(MakeNullCallback<void, Ptr<Socket>>());
  }
}

void
FlockingReceiver::Receive(Ptr<Socket> socket)
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

    // Optional delay tracker based on tx_time_us in header
    FlockingHeader hdr;
    packet->PeekHeader(hdr);

    uint64_t rx_us = Simulator::Now().ToInteger(Time::US);
    uint64_t tx_us = hdr.GetTxTimeUs();
    if (tx_us > 0 && rx_us >= tx_us)
    {
      Time delay = MicroSeconds(rx_us - tx_us);
      if (m_delay)
      {
        m_delay->Update(delay);
      }
    }

    if (m_calc)
    {
      m_calc->Update(1);
    }

    // Trace to NET connector (context-based TraceConnect 会把 agent_id 传给回调)
    m_rxTrace(packet, static_cast<int>(peer_id));
    m_received++;
  }
}

//----------------------------------------------------------------------
//-- FlockingHeader
//------------------------------------------------------
//
// Serialized layout (fixed order, scheme-1):
//   role(u8)
//   position: x,y,z (float32 x3)
//   velocity: vx,vy,vz (float32 x3)
//   sender_id(u32)
//   flow_id(u32)
//   seq(u32)
//   tx_time_us(u64)
// Total bytes: 1 + 12 + 12 + 4 + 4 + 4 + 8 = 45
//

FlockingHeader::FlockingHeader()
  : m_role(Undefined),
    m_position(0.0, 0.0, 0.0),
    m_velocity(0.0, 0.0, 0.0),
    m_sender_id(0),
    m_flow_id(0),
    m_seq(0),
    m_tx_time_us(0)
{
}

FlockingHeader::~FlockingHeader() {}

TypeId
FlockingHeader::GetTypeId()
{
  static TypeId tid =
      TypeId("ns3::FlockingHeader")
          .SetParent<Header>()
          .AddConstructor<FlockingHeader>();
  return tid;
}

TypeId
FlockingHeader::GetInstanceTypeId() const
{
  return GetTypeId();
}

void
FlockingHeader::Print(std::ostream& os) const
{
  os << "FlockingHeader{"
     << "role=" << static_cast<uint32_t>(m_role)
     << ", pos=(" << m_position.x << "," << m_position.y << "," << m_position.z << ")"
     << ", vel=(" << m_velocity.x << "," << m_velocity.y << "," << m_velocity.z << ")"
     << ", sender_id=" << m_sender_id
     << ", flow_id=" << m_flow_id
     << ", seq=" << m_seq
     << ", tx_us=" << m_tx_time_us
     << "}";
}

uint32_t
FlockingHeader::GetSerializedSize() const
{
  return 45;
}

void
FlockingHeader::Serialize(Buffer::Iterator start) const
{
  Buffer::Iterator i = start;

  i.WriteU8(static_cast<uint8_t>(m_role));

  WriteFloatHton(i, static_cast<float>(m_position.x));
  WriteFloatHton(i, static_cast<float>(m_position.y));
  WriteFloatHton(i, static_cast<float>(m_position.z));

  WriteFloatHton(i, static_cast<float>(m_velocity.x));
  WriteFloatHton(i, static_cast<float>(m_velocity.y));
  WriteFloatHton(i, static_cast<float>(m_velocity.z));

  i.WriteHtonU32(m_sender_id);
  i.WriteHtonU32(m_flow_id);
  i.WriteHtonU32(m_seq);
  i.WriteHtonU64(m_tx_time_us);
}

uint32_t
FlockingHeader::Deserialize(Buffer::Iterator start)
{
  Buffer::Iterator i = start;

  m_role = static_cast<FlockingRole>(i.ReadU8());

  m_position.x = ReadFloatNtoh(i);
  m_position.y = ReadFloatNtoh(i);
  m_position.z = ReadFloatNtoh(i);

  m_velocity.x = ReadFloatNtoh(i);
  m_velocity.y = ReadFloatNtoh(i);
  m_velocity.z = ReadFloatNtoh(i);

  m_sender_id  = i.ReadNtohU32();
  m_flow_id    = i.ReadNtohU32();
  m_seq        = i.ReadNtohU32();
  m_tx_time_us = i.ReadNtohU64();

  return GetSerializedSize();
}

void
FlockingHeader::SetPosition(Vector position)
{
  m_position = position;
}

void
FlockingHeader::SetVelocity(Vector velocity)
{
  m_velocity = velocity;
}

Vector
FlockingHeader::GetPosition() const
{
  return m_position;
}

Vector
FlockingHeader::GetVelocity() const
{
  return m_velocity;
}

void
FlockingHeader::SetSenderId(uint32_t sender_id)
{
  m_sender_id = sender_id;
}

void
FlockingHeader::SetFlowId(uint32_t flow_id)
{
  m_flow_id = flow_id;
}

void
FlockingHeader::SetSeq(uint32_t seq)
{
  m_seq = seq;
}

void
FlockingHeader::SetTxTimeUs(uint64_t tx_time_us)
{
  m_tx_time_us = tx_time_us;
}

uint32_t
FlockingHeader::GetSenderId() const
{
  return m_sender_id;
}

uint32_t
FlockingHeader::GetFlowId() const
{
  return m_flow_id;
}

uint32_t
FlockingHeader::GetSeq() const
{
  return m_seq;
}

uint64_t
FlockingHeader::GetTxTimeUs() const
{
  return m_tx_time_us;
}
