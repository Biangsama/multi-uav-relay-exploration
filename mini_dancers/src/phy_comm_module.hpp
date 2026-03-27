#pragma once

#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>

#include <protobuf_msgs/net_rx_events.pb.h>

#include <queue>
#include <unordered_map>
#include <vector>
#include <cstdint>
#include <string>
#include <limits>

class PhyCommModule
{
public:
    struct Params
    {
        bool enable{true};

        // 事件从“网络接收时刻 rx_time_us”到“交付控制器”的额外处理延迟
        uint64_t processing_delay_us{0};

        // 事件过期阈值：now_us - rx_time_us > ttl_us 则丢弃（Phase3 可先设很大）
        uint64_t ttl_us{5'000'000}; // 5s

        // 发布的 topic（protobuf bytes，未 gzip）
        std::string out_topic{"/comm/rx_events_bytes"};
    };

    explicit PhyCommModule(ros::NodeHandle* nh);

    void SetParams(const Params& p);

    // 接收 NET->PHY 的一批事件（protobuf）
    void IngestBatch(const protobuf_msgs::NetRxEventsPayload& batch);

    // 在 PHY 时间推进到 now_us 时调用：投递所有 deliver_us <= now_us 的事件
    void Tick(uint64_t now_us);

    // 仅用于 debug/验证
    uint64_t pending_size() const { return (uint64_t)pq_.size(); }
    uint64_t delivered_total() const { return delivered_total_; }
    uint64_t dropped_ttl_total() const { return dropped_ttl_total_; }

private:
    struct PendingEvent
    {
        uint64_t deliver_us{0};  // 交付时刻（rx_time_us + processing_delay_us）
        protobuf_msgs::RxEvent ev;
    };

    struct Cmp
    {
        bool operator()(const PendingEvent& a, const PendingEvent& b) const
        {
            // priority_queue 默认最大堆，这里做最小堆：deliver_us 小的优先
            return a.deliver_us > b.deliver_us;
        }
    };

    ros::NodeHandle* nh_{nullptr};
    Params params_;

    ros::Publisher pub_;

    std::priority_queue<PendingEvent, std::vector<PendingEvent>, Cmp> pq_;

    uint64_t delivered_total_{0};
    uint64_t dropped_ttl_total_{0};

    void PublishBatch_(const protobuf_msgs::NetRxEventsPayload& batch);
};
