#ifndef FLOCKING_APPLICATION_H
#define FLOCKING_APPLICATION_H

#include "ns3/application.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/stats-module.h"
#include <ns3/internet-module.h>
#include <ns3/mobility-module.h>

#include <cstdint>

using namespace ns3;

/**
 * FlockingBroadcaster application.
 */
class FlockingBroadcaster : public Application
{
public:
  static TypeId GetTypeId();
  FlockingBroadcaster();
  ~FlockingBroadcaster() override;

  uint64_t GetSent() const;

protected:
  void DoDispose() override;

private:
  void StartApplication() override;
  void StopApplication() override;
  void SendPacket();

  uint32_t m_pktSize;
  uint32_t m_destPort;
  Ptr<ConstantRandomVariable> m_interval;
  uint32_t m_flowId;

  Ptr<Socket> m_socket;
  EventId m_sendEvent;
  uint64_t m_sent;

  TracedCallback<Ptr<const Packet>> m_txTrace;

  uint32_t m_count;
  uint32_t m_seq;
};

/**
 * FlockingReceiver application.
 */
class FlockingReceiver : public Application
{
public:
  static TypeId GetTypeId();
  FlockingReceiver();
  ~FlockingReceiver() override;

  void SetCounter(Ptr<CounterCalculator<>> calc);
  void SetDelayTracker(Ptr<TimeMinMaxAvgTotalCalculator> delay);

  uint64_t GetReceived() const;

protected:
  void DoDispose() override;

private:
  void StartApplication() override;
  void StopApplication() override;
  void Receive(Ptr<Socket> socket);

  uint32_t m_port;
  Ptr<Socket> m_socket;

  Ptr<CounterCalculator<>> m_calc;
  Ptr<TimeMinMaxAvgTotalCalculator> m_delay;

  uint64_t m_received;

  TracedCallback<Ptr<const Packet>, int> m_rxTrace;
};

/**
 * FlockingHeader
 *
 * scheme-1 扩展字段：
 *   sender_id(u32), flow_id(u32), seq(u32), tx_time_us(u64)
 */
class FlockingHeader : public Header
{
public:
  enum FlockingRole
  {
    Undefined = 0,
    Mission,
    Potential,
    Idle
  };

  FlockingHeader();
  ~FlockingHeader() override;

  static TypeId GetTypeId();
  TypeId GetInstanceTypeId() const override;
  void Print(std::ostream& os) const override;
  uint32_t GetSerializedSize() const override;
  void Serialize(Buffer::Iterator start) const override;
  uint32_t Deserialize(Buffer::Iterator start) override;

  void SetPosition(Vector position);
  void SetVelocity(Vector velocity);

  Vector GetPosition() const;
  Vector GetVelocity() const;

  // ---- scheme-1 fields ----
  void SetSenderId(uint32_t sender_id);
  void SetFlowId(uint32_t flow_id);
  void SetSeq(uint32_t seq);
  void SetTxTimeUs(uint64_t tx_time_us);

  uint32_t GetSenderId() const;
  uint32_t GetFlowId() const;
  uint32_t GetSeq() const;
  uint64_t GetTxTimeUs() const;

private:
  FlockingRole m_role;
  Vector m_position;
  Vector m_velocity;

  uint32_t m_sender_id;
  uint32_t m_flow_id;
  uint32_t m_seq;
  uint64_t m_tx_time_us;
};

#endif // FLOCKING_APPLICATION_H
