#pragma once

#include "ns3/application.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"

#include <cstdint>
#include <deque>
#include <string>

using namespace ns3;

class SwarmMessageBroadcaster : public Application
{
public:
  static TypeId GetTypeId();
  SwarmMessageBroadcaster();
  ~SwarmMessageBroadcaster() override;

  void EnqueuePayload(const std::string& payload);
  uint64_t GetSent() const;

protected:
  void DoDispose() override;

private:
  void StartApplication() override;
  void StopApplication() override;
  void FlushQueue();
  void SendPayload(const std::string& payload);

  uint32_t m_destPort{4100};
  uint32_t m_flowId{30};
  uint32_t m_senderId{0};

  Ptr<Socket> m_socket;
  EventId m_flushEvent;
  uint64_t m_sent{0};
  uint32_t m_seq{0};
  bool m_running{false};
  std::deque<std::string> m_pendingPayloads;

  TracedCallback<Ptr<const Packet>> m_txTrace;
};

class SwarmMessageReceiver : public Application
{
public:
  static TypeId GetTypeId();
  SwarmMessageReceiver();
  ~SwarmMessageReceiver() override;

  uint64_t GetReceived() const;

protected:
  void DoDispose() override;

private:
  void StartApplication() override;
  void StopApplication() override;
  void Receive(Ptr<Socket> socket);

  uint32_t m_port{4100};
  Ptr<Socket> m_socket;
  uint64_t m_received{0};

  TracedCallback<Ptr<const Packet>, int> m_rxTrace;
};

class SwarmMessageHeader : public Header
{
public:
  SwarmMessageHeader();
  ~SwarmMessageHeader() override;

  static TypeId GetTypeId();
  TypeId GetInstanceTypeId() const override;
  void Print(std::ostream& os) const override;
  uint32_t GetSerializedSize() const override;
  void Serialize(Buffer::Iterator start) const override;
  uint32_t Deserialize(Buffer::Iterator start) override;

  void SetSenderId(uint32_t sender_id);
  uint32_t GetSenderId() const;

  void SetFlowId(uint32_t flow_id);
  uint32_t GetFlowId() const;

  void SetSeq(uint32_t seq);
  uint32_t GetSeq() const;

  void SetTxTimeUs(uint64_t tx_time_us);
  uint64_t GetTxTimeUs() const;

private:
  uint32_t m_sender_id{0};
  uint32_t m_flow_id{0};
  uint32_t m_seq{0};
  uint64_t m_tx_time_us{0};
};
