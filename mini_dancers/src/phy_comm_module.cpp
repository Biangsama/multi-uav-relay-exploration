#include "phy_comm_module.hpp"

PhyCommModule::PhyCommModule(ros::NodeHandle* nh) : nh_(nh)
{
    if (nh_ == nullptr) throw std::runtime_error("PhyCommModule: node handle is null");
}

void PhyCommModule::SetParams(const Params& p)
{
    params_ = p;

    if (params_.enable)
    {
        pub_ = nh_->advertise<std_msgs::UInt8MultiArray>(params_.out_topic, 10);
        ROS_INFO("[CommModule] enabled. out_topic=%s processing_delay_us=%llu ttl_us=%llu",
                 params_.out_topic.c_str(),
                 (unsigned long long)params_.processing_delay_us,
                 (unsigned long long)params_.ttl_us);
    }
    else
    {
        ROS_INFO("[CommModule] disabled.");
    }
}

void PhyCommModule::IngestBatch(const protobuf_msgs::NetRxEventsPayload& batch)
{
    if (!params_.enable) return;

    const int n = batch.events_size();
    for (int i = 0; i < n; ++i)
    {
        PendingEvent pe;
        pe.ev = batch.events(i);

        // deliver_us = rx_time_us + processing_delay_us
        const uint64_t rx_us = (uint64_t)pe.ev.rx_time_us();
        pe.deliver_us = rx_us + params_.processing_delay_us;

        pq_.push(std::move(pe));
    }
}

void PhyCommModule::Tick(uint64_t now_us)
{
    if (!params_.enable) return;
    if (!pub_) return;

    protobuf_msgs::NetRxEventsPayload out;
    out.set_t_window_start_us(now_us); // 这里用 now_us 做标记（Phase3 debug 用）
    out.set_t_window_end_us(now_us);

    uint64_t delivered_this_tick = 0;

    while (!pq_.empty())
    {
        const PendingEvent& top = pq_.top();
        if (top.deliver_us > now_us) break;

        // TTL：如果事件太老，丢弃
        const uint64_t rx_us = (uint64_t)top.ev.rx_time_us();
        if (params_.ttl_us > 0 && now_us > rx_us && (now_us - rx_us) > params_.ttl_us)
        {
            dropped_ttl_total_++;
            pq_.pop();
            continue;
        }

        // 投递
        *out.add_events() = top.ev;
        pq_.pop();

        delivered_this_tick++;
        delivered_total_++;
    }

    if (delivered_this_tick > 0)
    {
        PublishBatch_(out);

        ROS_DEBUG("[CommModule] tick now_us=%llu deliver=%llu pending=%llu dropped_ttl=%llu",
                  (unsigned long long)now_us,
                  (unsigned long long)delivered_this_tick,
                  (unsigned long long)pq_.size(),
                  (unsigned long long)dropped_ttl_total_);
    }
}

void PhyCommModule::PublishBatch_(const protobuf_msgs::NetRxEventsPayload& batch)
{
    std::string bytes;
    batch.SerializeToString(&bytes);

    std_msgs::UInt8MultiArray msg;
    msg.data.assign(bytes.begin(), bytes.end());

    pub_.publish(msg);
}
