#pragma once

#include "ns3/application.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"

#include <cstdint>

using namespace ns3;

/**
 * WaypointBroadcaster application.
 */
class WaypointBroadcaster : public Application
{
public:
  static TypeId GetTypeId();
  WaypointBroadcaster();
  ~WaypointBroadcaster() override;

  uint64_t GetSent() const;

  void SetWaypoint(Vector waypoint);

protected:
  void DoDispose() override;

private:
  void StartApplication() override;
  void StopApplication() override;
  void SendPacket();

  uint32_t m_pktSize;                     //!< Additional payload bytes (excluding header)
  uint32_t m_destPort;                    //!< Destination port.
  Ptr<ConstantRandomVariable> m_interval; //!< Send interval.
  uint32_t m_flowId;                      //!< Flow Id tagged in the packets (optional).
  uint32_t m_senderId{0};                 //!< Logical sender id (agent_id)
  Vector m_waypoint;                      //!< Current waypoint.

  Ptr<Socket> m_socket; //!< Sending socket.
  EventId m_sendEvent;  //!< Send event.
  uint64_t m_sent;      //!< Packets sent.

  TracedCallback<Ptr<const Packet>> m_txTrace;

  uint32_t m_count; //!< Number of packets sent.
};

/**
 * WaypointReceiver application.
 */
class WaypointReceiver : public Application
{
public:
  static TypeId GetTypeId();
  WaypointReceiver();
  ~WaypointReceiver() override;

  uint64_t GetReceived() const;

protected:
  void DoDispose() override;

private:
  void StartApplication() override;
  void StopApplication() override;

  void Receive(Ptr<Socket> socket);

  uint32_t m_port;      //!< Listening port.
  Ptr<Socket> m_socket; //!< Receiving socket.
  uint64_t m_received;  //!< Packets received.

  TracedCallback<Ptr<const Packet>, int> m_rxTrace;
};

/**
 * WaypointHeader
 *
 * Layout (network byte order):
 *   float32 x, y, z          (12 bytes)
 *   uint32  sender_id        (4 bytes)
 *   uint32  flow_id          (4 bytes)
 *   uint32  seq              (4 bytes)
 *   uint64  tx_time_us       (8 bytes)
 * Total: 32 bytes
 */
class WaypointHeader : public Header
{
public:
  WaypointHeader();
  ~WaypointHeader() override;

  static TypeId GetTypeId();
  TypeId GetInstanceTypeId() const override;
  void Print(std::ostream& os) const override;
  uint32_t GetSerializedSize() const override;
  void Serialize(Buffer::Iterator start) const override;
  uint32_t Deserialize(Buffer::Iterator start) override;

  void SetPosition(Vector position);
  Vector GetPosition() const;

  void SetSenderId(uint32_t id);
  uint32_t GetSenderId() const;

  void SetFlowId(uint32_t id);
  uint32_t GetFlowId() const;

  void SetSeq(uint32_t s);
  uint32_t GetSeq() const;

  void SetTxTimeUs(uint64_t t_us);
  uint64_t GetTxTimeUs() const;

private:
  Vector   m_position;
  uint32_t m_sender_id{0};
  uint32_t m_flow_id{0};
  uint32_t m_seq{0};
  uint64_t m_tx_time_us{0};
};
