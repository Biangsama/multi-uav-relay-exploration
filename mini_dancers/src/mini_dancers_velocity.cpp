#include <connector_core/connector.hpp>
#include <connector_core/metrics_logger.hpp>

// UAV System from CTU Prague
#include <uav_system.hpp>

// Common DANCERS libs
#include <agent.hpp>
#include <yaml_util.hpp>

// C++ libs
#include <shared_mutex>
#include <unordered_map>
#include <mutex>
#include <fstream>
#include <iomanip>
#include <limits>
#include <chrono>
#include <thread>
#include <vector>
#include <string>
#include <type_traits>
#include <cmath>
#include <algorithm>
#include <cstdlib>
#include <numeric>
#include <map>
#include <memory>
#include <deque>


#if __has_include(<rcpputils/filesystem_helper.hpp>)
  #include <rcpputils/filesystem_helper.hpp>
  namespace dancers_fs = rcpputils::fs;
#else
  #include <filesystem>
  namespace dancers_fs = std::filesystem;
#endif

// Custom ROS2 messages
#include <dancers_msgs/AgentStruct.h>
#include <dancers_msgs/GetAgentVelocities.h>

// Rviz ROS2 messages
#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

// Protobuf messages
#include <protobuf_msgs/pose_vector.pb.h>
#include <protobuf_msgs/state_payload.pb.h>   // CmdPayload/NodeCommMetrics
#include <protobuf_msgs/dancers_update.pb.h>  // DancersUpdate (net_rx_events_gz field)
#include <protobuf_msgs/net_rx_events.pb.h>   // NEW: NetRxEvents

// Protobuf reflection (for safe field read in first event)
#include <google/protobuf/descriptor.h>
#include <google/protobuf/reflection.h>

#include "phy_comm_module.hpp"   // Phase3: delayed delivery comm module

#define RCLCPP_INFO(logger, ...) ROS_INFO(__VA_ARGS__)
#define RCLCPP_WARN(logger, ...) ROS_WARN(__VA_ARGS__)
#define RCLCPP_ERROR(logger, ...) ROS_ERROR(__VA_ARGS__)
#define RCLCPP_FATAL(logger, ...) ROS_FATAL(__VA_ARGS__)
#define RCLCPP_DEBUG(logger, ...) ROS_DEBUG(__VA_ARGS__)

using namespace mrs_multirotor_simulator;

// -----------------------------
// Local utilities
// -----------------------------
static inline uint64_t now_wall_us()
{
    using namespace std::chrono;
    return (uint64_t)duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
}

class MiniDancers : public Connector
{
public:
    MiniDancers()
    : Connector("mini_dancers")
    {
        RCLCPP_INFO(this->get_logger(), "MiniDancers (PHY) started");

        this->ReadConfigFile();

        
        LoadDelayInjectConfig_();
        this->InitAgents();
        
        // ===== Phase4: VAT service cmd source params (from YAML) =====
        use_vat_service_cmd_ =
            (config_["phy_use_vat_service_cmd"] ? config_["phy_use_vat_service_cmd"].as<bool>() : false);

        vat_service_name_ =
            (config_["phy_vat_service_name"] ? config_["phy_vat_service_name"].as<std::string>()
                                            : std::string("/get_agents_velocities"));

        vat_control_period_us_ =
            (config_["control_period_us"] ? config_["control_period_us"].as<uint64_t>() : 100000ull);

        vat_timeout_ms_ =
            (config_["phy_vat_timeout_ms"] ? config_["phy_vat_timeout_ms"].as<int>() : 5);

        if (use_vat_service_cmd_) {
            vat_client_ = nh_.serviceClient<dancers_msgs::GetAgentVelocities>(vat_service_name_);
            RCLCPP_INFO(this->get_logger(),
                "[VAT_CMD] enabled=true srv=%s period_us=%llu timeout_ms=%d (payload cmd path will be bypassed)",
                vat_service_name_.c_str(),
                (unsigned long long)vat_control_period_us_,
                vat_timeout_ms_);
        } else {
            RCLCPP_INFO(this->get_logger(),
                "[VAT_CMD] enabled=false (use payload cmds / Stage2 inject path)");
        }


        // Existing debug CSV (step-level)
        this->init_phy_debug_csv_();
        
        // ---- Phase3: CommModule (delayed delivery) ----
        comm_ = std::make_unique<PhyCommModule>(&nh_);

        // YAML 可配置（没有就用默认）
        comm_params_.enable = (config_["comm_enable"] ? config_["comm_enable"].as<bool>() : true);
        comm_params_.processing_delay_us =
            (config_["comm_processing_delay_us"] ? config_["comm_processing_delay_us"].as<uint64_t>() : 0);
        comm_params_.ttl_us =
            (config_["comm_ttl_us"] ? config_["comm_ttl_us"].as<uint64_t>() : 5'000'000);
        comm_params_.out_topic =
            (config_["comm_out_topic"] ? config_["comm_out_topic"].as<std::string>()
                                    : std::string("/comm/rx_events_bytes"));

        comm_->SetParams(comm_params_);


        // ---- PHY agent debug CSV ----
        enable_agent_csv_   = (config_["enable_agent_debug_csv"] ? config_["enable_agent_debug_csv"].as<bool>() : true);
        agent_csv_every_n_  = (config_["agent_debug_every_n"] ? config_["agent_debug_every_n"].as<size_t>() : 1);
        if (enable_agent_csv_) InitAgentDebugCsv_();

        this->pose_array_pub_ = nh_.advertise<geometry_msgs::PoseArray>("agent_poses", 10);
        this->id_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("id_markers", 10);
        this->obstacles_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("obstacles", 10);
        this->targets_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("targets", 10);
        

        loop_thread_ = std::thread(&MiniDancers::Loop, this);
    }

private:
    // ===== NEW: received node comm metrics bookkeeping =====
    bool got_node_metrics_{false};
    uint64_t got_node_metrics_t_sim_us_{0};
    size_t got_node_metrics_n_{0};
    uint64_t last_metrics_print_it_{std::numeric_limits<uint64_t>::max()};

    // (optional cache) last metrics by node id
    std::unordered_map<uint32_t, dancers_update_proto::NodeCommMetrics> last_node_metrics_;

    // NEW: NET->PHY side-channel (already gzip bytes of NetRxEvents proto)
    // NEW: NET->PHY side-channel (already gzip bytes of NetRxEvents proto)
    std::string last_net_rx_events_gz_;
    uint64_t last_net_rx_events_bytes_{0};
    uint64_t last_net_rx_events_window_count_{0};
    mutable std::mutex latest_comm_rx_events_mu_;
    std::string latest_comm_rx_events_raw_;

    // NEW: parse bookkeeping
    uint64_t net_rx_events_parse_fail_{0};
    uint64_t last_net_rx_events_count_{0};

    // NEW: per-window debug CSV (optional but recommended)
    std::unique_ptr<dancers::metrics::CsvMetricsLogger> net_rx_ev_dbg_;



    // ===== Existing PHY debug CSV (step-level) =====
    std::unique_ptr<dancers::metrics::CsvMetricsLogger> phy_dbg_;
    size_t phy_dbg_flush_every_n_{50};
    size_t phy_dbg_every_n_{1};

    bool waypoint_relative_{true};
    std::unordered_map<uint32_t, Eigen::Vector3d> last_wp_raw_;
    std::unordered_map<uint32_t, Eigen::Vector3d> last_wp_world_;

    uint64_t dt_local_us_{0};
    uint64_t last_step_end_us_{0};

    // =============================
    // Phase4: VAT service cmd source (PHY pulls cmd, still execution-only)
    // =============================
    bool use_vat_service_cmd_{false};
    std::string vat_service_name_{"/get_agents_velocities"};
    uint64_t vat_control_period_us_{10000};  // 10ms
    int vat_timeout_ms_{5};

    uint64_t last_vat_call_us_{0};
    uint64_t last_vat_cmd_us_{0};
    uint64_t vat_ok_{0}, vat_timeout_{0}, vat_fail_{0};

    ros::ServiceClient vat_client_;

    // cache latest VAT cmds (ZOH happens naturally if no update)
    std::unordered_map<uint32_t, reference::VelocityHdg> last_vat_cmd_;


    // ---- Phase3: CommModule (delayed delivery) ----
    std::unique_ptr<PhyCommModule> comm_;
    PhyCommModule::Params comm_params_;

    // ---- Stage2: cmd delay injection queue ----


    // =============================
    // Stage2 observability (CSV + print)
    // =============================
    std::unique_ptr<dancers::metrics::CsvMetricsLogger> cmd_delay_dbg_;
    size_t cmd_delay_dbg_every_n_{1};          // 每步都记（可改 10）
    size_t cmd_delay_dbg_flush_every_n_{10};   // 快速落盘，便于 tail -f
    size_t cmd_delay_print_every_n_{50};       // 控制台每 N 步打印一次

    // 最近一次“为该 agent 入队时使用的注入延迟 / 原始 metric 延迟 / seq”
    std::unordered_map<uint32_t, uint64_t> last_injected_delay_us_by_agent_;
    std::unordered_map<uint32_t, uint64_t> last_metric_delay_us_by_agent_;
    std::unordered_map<uint32_t, uint64_t> last_cmd_seq_by_agent_;


    // =============================
    // Stage2: 命令延迟注入（PHY侧）
    // =============================
    struct PendingCmd
    {
        reference::VelocityHdg ctrl;   // 要 apply 的控制输入
        uint64_t t_recv_us{0};         // PHY 收到该命令的“仿真时刻”（用 step begin）
        uint64_t t_apply_us{0};        // 计划 apply 的“仿真时刻”
        uint64_t t_sample_us{0};       // NET侧采样时刻（如果cp里有就用；没有就=recv）
        uint64_t seq{0};               // 方便排查
    };

    // 每个agent一个队列：按 t_apply_us 递增
    std::unordered_map<uint32_t, std::deque<PendingCmd>> cmd_queues_;

    // 配置项（YAML）——给默认值，没配也能跑
    bool     inject_delay_enable_{true};      // 是否启用延迟注入
    double   inject_delay_scale_{1.0};        // 延迟缩放（例如1.0=原样，0.5=减半）
    uint64_t inject_delay_min_us_{0};         // clamp 下限
    uint64_t inject_delay_max_us_{2000000};   // clamp 上限（2s）
    uint64_t inject_delay_fallback_us_{0};    // 没有node_metrics时用的fallback
    size_t   inject_queue_max_len_{8};        // 防止队列无限增长

    // 记录最近一次实际 e2e（用于debug）
    std::unordered_map<uint32_t, uint64_t> last_cmd_e2e_us_by_agent_;


    bool QueryVatCmd_(uint64_t now_us)
    {
        if (!use_vat_service_cmd_ || !vat_client_.isValid()) return false;

        // periodic trigger
        if (last_vat_call_us_ != 0 && (now_us - last_vat_call_us_) < vat_control_period_us_) {
            return false;
        }
        last_vat_call_us_ = now_us;

        if (!vat_client_.exists() &&
            !vat_client_.waitForExistence(ros::Duration(std::max(0, vat_timeout_ms_) / 1000.0))) {
            vat_fail_++;
            if ((vat_fail_ % 50) == 1) {
                RCLCPP_WARN(this->get_logger(), "[VAT_CMD] service not ready: %s", vat_service_name_.c_str());
            }
            return false;
        }

        dancers_msgs::GetAgentVelocities srv;
        srv.request.agent_structs.clear();
        {
            std::lock_guard<std::mutex> lk(latest_comm_rx_events_mu_);
            srv.request.comm_rx_events_bytes.assign(
                latest_comm_rx_events_raw_.begin(), latest_comm_rx_events_raw_.end());
        }

        // Build request from current PHY states
        {
            std::shared_lock lk(this->agents_mutex_);
            srv.request.agent_structs.reserve(this->agents_.size());

            for (auto &kv : this->agents_) {
                const uint32_t id = kv.first;
                auto *agent_ptr = kv.second.get();
                const auto st = agent_ptr->uav_system->getState();

                dancers_msgs::AgentStruct a;
                a.agent_id = id;

                a.state.position.x = st.x.x();
                a.state.position.y = st.x.y();
                a.state.position.z = st.x.z();

                a.state.velocity.x = st.v.x();
                a.state.velocity.y = st.v.y();
                a.state.velocity.z = st.v.z();

                // yaw from R (Z-up)
                a.state.heading = std::atan2(st.R(1,0), st.R(0,0));

                // 注意：这里没有填 neighbor_array（因为消息字段不确定）。
                // 如果你希望 StepC1 的 gating 真正“删邻居”，最稳的方法是在 VAT controller 端
                // 用 request->agent_structs 自己构造全连接 neighbors，再做 gating（我后面可以给你一段可直接贴的 controller 改法）。

                srv.request.agent_structs.push_back(std::move(a));
            }
        }

        if (!vat_client_.call(srv)) {
            vat_fail_++;
            return false;
        }

        // Map response to agents by index order (request and agents_ iteration are both sorted by id)
        last_vat_cmd_.clear();
        const auto &arr = srv.response.velocity_headings.velocity_heading_array;

        {
            std::shared_lock lk(this->agents_mutex_);
            size_t i = 0;
            for (auto &kv : this->agents_) {
                const uint32_t id = kv.first;
                if (i >= arr.size()) break;

                const auto &vh = arr[i];

                reference::VelocityHdg ctrl;
                ctrl.velocity[0] = vh.velocity.x;
                ctrl.velocity[1] = vh.velocity.y;
                ctrl.velocity[2] = vh.velocity.z;
                ctrl.heading      = vh.heading;

                last_vat_cmd_[id] = ctrl;
                i++;
            }
        }

        last_vat_cmd_us_ = now_us;
        vat_ok_++;

        if ((vat_ok_ % 50) == 1) {
            RCLCPP_INFO(this->get_logger(),
                "[VAT_CMD] ok=%llu timeout=%llu fail=%llu n_cmd=%zu now_us=%llu",
                (unsigned long long)vat_ok_,
                (unsigned long long)vat_timeout_,
                (unsigned long long)vat_fail_,
                last_vat_cmd_.size(),
                (unsigned long long)now_us);
        }

        return true;
    }

    void ApplyVatCmds_(std::vector<uint32_t>& applied_ids)
    {
        applied_ids.clear();
        if (!use_vat_service_cmd_) return;
        if (last_vat_cmd_.empty()) return;

        std::unique_lock lk(this->agents_mutex_);
        applied_ids.reserve(last_vat_cmd_.size());

        for (auto &kv : last_vat_cmd_) {
            const uint32_t id = kv.first;
            const auto &ctrl  = kv.second;

            auto it = this->agents_.find(id);
            if (it == this->agents_.end()) continue;

            it->second->uav_system->setInput(ctrl);
            applied_ids.push_back(id);

            // keep your existing goal_cache_ observability
            auto &gc = goal_cache_[id];
            gc.has_goal = true;
            gc.gx = (double)ctrl.velocity[0];
            gc.gy = (double)ctrl.velocity[1];
            gc.gz = (double)ctrl.velocity[2];
        }
    }



    // 读取YAML参数（在构造函数里调用一次）
    void LoadDelayInjectConfig_()
    {
        inject_delay_enable_ = (config_["phy_inject_delay_enable"] ? config_["phy_inject_delay_enable"].as<bool>() : true);
        inject_delay_scale_  = (config_["phy_inject_delay_scale"]  ? config_["phy_inject_delay_scale"].as<double>() : 1.0);

        inject_delay_min_us_ = (config_["phy_inject_delay_min_us"] ? config_["phy_inject_delay_min_us"].as<uint64_t>() : 0);
        inject_delay_max_us_ = (config_["phy_inject_delay_max_us"] ? config_["phy_inject_delay_max_us"].as<uint64_t>() : 2000000);

        inject_delay_fallback_us_ = (config_["phy_inject_delay_fallback_us"] ? config_["phy_inject_delay_fallback_us"].as<uint64_t>() : 0);
        inject_queue_max_len_     = (config_["phy_inject_queue_max_len"] ? config_["phy_inject_queue_max_len"].as<size_t>() : 8);

        if (inject_delay_max_us_ < inject_delay_min_us_) inject_delay_max_us_ = inject_delay_min_us_;

        RCLCPP_INFO(this->get_logger(),
            "[PHY][DelayInject] enable=%s scale=%.3f clamp=[%lu,%lu]us fallback=%luus qmax=%zu",
            inject_delay_enable_ ? "true" : "false",
            inject_delay_scale_,
            (unsigned long)inject_delay_min_us_,
            (unsigned long)inject_delay_max_us_,
            (unsigned long)inject_delay_fallback_us_,
            inject_queue_max_len_);
    }

    // 取某个 agent 对应的 delay_mean_us（来自 node_metrics）
    // 若本 step 没收到 metrics 或该 node 没有条目，则用 fallback
    uint64_t GetDelayMeanUsForAgent_(uint32_t agent_id) const
    {
        if (!inject_delay_enable_) return 0;

        uint64_t d = inject_delay_fallback_us_;

        auto it = last_node_metrics_.find(agent_id);
        if (it != last_node_metrics_.end())
        {
            // node_metrics.proto 里字段名是 delay_mean_us
            d = (uint64_t)it->second.delay_mean_us();
        }

        // scale + clamp
        double ds = double(d) * inject_delay_scale_;
        if (ds < 0.0) ds = 0.0;
        uint64_t du = (uint64_t)std::llround(ds);

        if (du < inject_delay_min_us_) du = inject_delay_min_us_;
        if (du > inject_delay_max_us_) du = inject_delay_max_us_;
        return du;
    }


    // 入队：按 t_apply_us 递增插入，保证 ApplyDueCmds_ 只看 front() 也不会漏掉“已到期但在中间”的命令
    void EnqueueCmd_(uint32_t agent_id,
                    const reference::VelocityHdg& ctrl,
                    uint64_t t_recv_us,
                    uint64_t t_sample_us,
                    uint64_t seq,
                    uint64_t injected_delay_us)
    {
        PendingCmd pc;
        pc.ctrl        = ctrl;
        pc.t_recv_us   = t_recv_us;
        pc.t_apply_us  = t_recv_us + injected_delay_us;
        pc.t_sample_us = (t_sample_us != 0 ? t_sample_us : t_recv_us);
        pc.seq         = seq;

        auto &q = cmd_queues_[agent_id];

        // 关键：按 t_apply_us 插入排序（队列很短，O(n) 足够）
        auto it = std::upper_bound(
            q.begin(), q.end(), pc,
            [](const PendingCmd& a, const PendingCmd& b) {
                return a.t_apply_us < b.t_apply_us;
            });
        q.insert(it, pc);

        // ===== Stage2 observability: 记录可观测信息 =====
        last_injected_delay_us_by_agent_[agent_id] = injected_delay_us;
        last_cmd_seq_by_agent_[agent_id] = seq;
        //（metric_delay_us 不在这里记；在 StepSimulation 里算出来后记 last_metric_delay_us_by_agent_）

        // 防止队列爆炸：超长则丢弃“最早将要生效”的
        while (q.size() > inject_queue_max_len_) {
            q.pop_front();
        }
    }




    // 把到期命令 apply（每步开始时调用）
    // 策略：对每个 agent，把所有 “t_apply_us <= t_now_us” 的命令全部弹出，只应用最后一个（最新的）
    void ApplyDueCmds_(uint64_t t_now_us, std::vector<uint32_t>& applied_ids)
    {
        applied_ids.clear();
        applied_ids.reserve(16);

        std::unique_lock lock(this->agents_mutex_);

        for (auto &kv : cmd_queues_)
        {
            const uint32_t agent_id = kv.first;
            auto &q = kv.second;
            if (q.empty()) continue;

            bool has_due = false;
            PendingCmd last_due;

            while (!q.empty() && q.front().t_apply_us <= t_now_us)
            {
                last_due = q.front();
                q.pop_front();
                has_due = true;
            }

            if (!has_due) continue;

            auto it = this->agents_.find(agent_id);
            if (it == this->agents_.end())
            {
                // agent 不存在就丢弃
                continue;
            }

            it->second->uav_system->setInput(last_due.ctrl);
            applied_ids.push_back(agent_id);

            // 记录 e2e：用 (apply - sample)（sample若未知就等于recv）
            uint64_t t_sample = last_due.t_sample_us ? last_due.t_sample_us : last_due.t_recv_us;
            uint64_t e2e = (t_now_us >= t_sample) ? (t_now_us - t_sample) : 0;
            last_cmd_e2e_us_by_agent_[agent_id] = e2e;

            // debug里你之前用 goal_cache_ 存 vx/vy/vz，这里沿用
            auto &gc = goal_cache_[agent_id];
            gc.has_goal = true;
            gc.gx = (double)last_due.ctrl.velocity[0];
            gc.gy = (double)last_due.ctrl.velocity[1];
            gc.gz = (double)last_due.ctrl.velocity[2];
        }
    }


    void init_phy_debug_csv_()
    {
        const bool enable = (this->config_["enable_metrics"] ? this->config_["enable_metrics"].as<bool>() : true);
        if (!enable) return;

        const std::string metrics_dir = (this->config_["metrics_dir"] ? this->config_["metrics_dir"].as<std::string>() : "logs");
        const std::string exp_name    = (this->config_["experiment_name"] ? this->config_["experiment_name"].as<std::string>() : "exp");

        if (this->config_["flush_every_n"])     phy_dbg_flush_every_n_ = this->config_["flush_every_n"].as<size_t>();
        if (this->config_["phy_dbg_every_n"])   phy_dbg_every_n_       = this->config_["phy_dbg_every_n"].as<size_t>();

        waypoint_relative_ = (this->config_["phy_waypoint_relative"] ? this->config_["phy_waypoint_relative"].as<bool>() : true);

        dancers_fs::path out_dir = dancers_fs::path(this->ros_ws_path) / metrics_dir / exp_name;
        dancers_fs::create_directories(out_dir);
        dancers_fs::path out_csv = out_dir / "mini_dancers_phy_debug.csv";

        phy_dbg_ = std::make_unique<dancers::metrics::CsvMetricsLogger>(
            out_csv.string(),
            std::vector<std::string>{
                "wall_ts_us","it","msg_type",
                "t_target_us","watermark_us",
                "n_waypoints","payload_in_B",
                "a0_px","a0_py","a0_pz",
                "a0_vx","a0_vy","a0_vz","a0_speed",
                "a0_ref_raw_x","a0_ref_raw_y","a0_ref_raw_z",
                "a0_ref_world_x","a0_ref_world_y","a0_ref_world_z",
                "a0_err",
                "mean_speed","mean_err",
                // receipt indicator
                "got_node_metrics","n_node_metrics","node_metrics_t_sim_us",
                // NEW: prove Stage2 injection is truly bypassed when disabled
                "cmd_inject_enable","cmd_queue_total_len"
            },
            phy_dbg_flush_every_n_);

        // NEW: per-window NET rx events debug csv
        {
            dancers_fs::path out_csv2 = out_dir / "phy_net_rx_events_debug.csv";
            net_rx_ev_dbg_ = std::make_unique<dancers::metrics::CsvMetricsLogger>(
                out_csv2.string(),
                std::vector<std::string>{
                    "wall_ts_us","it",
                    "win_count",
                    "gz_bytes","raw_bytes",
                    "events",
                    "parse_fail",
                    "win_start_us","win_end_us",
                    "e0_rx_id","e0_tx_id","e0_flow_id","e0_seq",
                    "e0_tx_us","e0_rx_us","e0_delay_us",
                    "e0_payload_bytes","e0_rssi_dbm_x10","e0_is_duplicate"
                },
                phy_dbg_flush_every_n_);

            RCLCPP_INFO(this->get_logger(),
                "[PHYDBG] phy_net_rx_events_debug.csv enabled. out=%s",
                out_csv2.string().c_str());
        }

        // NEW: Stage2 cmd delay debug csv (独立，不影响旧CSV列结构)
        {
            size_t flush_n = phy_dbg_flush_every_n_;
            size_t flush_fast = std::min<size_t>(flush_n, cmd_delay_dbg_flush_every_n_);

            dancers_fs::path out_csv3 = out_dir / "mini_dancers_cmd_delay_debug.csv";
            cmd_delay_dbg_ = std::make_unique<dancers::metrics::CsvMetricsLogger>(
                out_csv3.string(),
                std::vector<std::string>{
                    "wall_ts_us","it",
                    "t_now_us","t_step_end_us",
                    "n_cmds_received","n_cmds_enqueued","n_cmds_applied",
                    "q_len_sum","q_len_max",
                    "a0_q_len",
                    "a0_metric_delay_us","a0_injected_delay_us",
                    "a0_last_seq",
                    "a0_last_e2e_us",
                    "got_node_metrics","n_node_metrics","node_metrics_t_sim_us"
                },
                flush_fast
            );

            RCLCPP_INFO(this->get_logger(),
                "[CMD_DELAY_DBG] mini_dancers_cmd_delay_debug.csv enabled. out=%s flush_every_n=%zu",
                out_csv3.string().c_str(), flush_fast);
        }

        RCLCPP_INFO(this->get_logger(),
            "[PHYDBG] mini_dancers_phy_debug.csv enabled. waypoint_relative=%s every_n=%zu out=%s",
            waypoint_relative_ ? "true" : "false",
            phy_dbg_every_n_,
            out_csv.string().c_str());
    }

    void log_phy_debug_row_(const dancers_update_proto::DancersUpdate& update_msg,
                            uint64_t t_target_us,
                            uint64_t watermark_us,
                            size_t n_waypoints,
                            size_t payload_in_B)
    {
        if (!phy_dbg_) return;
        if (phy_dbg_every_n_ > 1 && (this->it % phy_dbg_every_n_) != 0) return;

        double mean_speed = 0.0;
        double mean_err   = 0.0;
        size_t count = 0;

        double a0_px=0,a0_py=0,a0_pz=0,a0_vx=0,a0_vy=0,a0_vz=0,a0_speed=0;
        double a0_rrx=0,a0_rry=0,a0_rrz=0,a0_rwx=0,a0_rwy=0,a0_rwz=0,a0_err=0;

        // NEW: Stage2 bypass proof
        const int cmd_inj_enable = ((!use_vat_service_cmd_) && inject_delay_enable_) ? 1 : 0;

        size_t cmd_qsum = 0;
        for (auto &kv : cmd_queues_) cmd_qsum += kv.second.size();

        std::shared_lock lk(this->agents_mutex_);
        for (auto& [id, agent_ptr] : this->agents_)
        {
            const auto st = agent_ptr->uav_system->getState();
            const Eigen::Vector3d p = st.x;
            const Eigen::Vector3d v = st.v;
            const double speed = v.norm();

            mean_speed += speed;
            count++;

            auto itw = last_wp_world_.find(id);
            if (itw != last_wp_world_.end())
            {
                mean_err += (p - itw->second).norm();
            }

            if (id == 0)
            {
                a0_px=p.x(); a0_py=p.y(); a0_pz=p.z();
                a0_vx=v.x(); a0_vy=v.y(); a0_vz=v.z();
                a0_speed = speed;

                auto itr = last_wp_raw_.find(0);
                if (itr != last_wp_raw_.end()) {
                    a0_rrx = itr->second.x(); a0_rry = itr->second.y(); a0_rrz = itr->second.z();
                }
                if (itw != last_wp_world_.end()) {
                    a0_rwx = itw->second.x(); a0_rwy = itw->second.y(); a0_rwz = itw->second.z();
                    a0_err = (p - itw->second).norm();
                }
            }
        }

        if (count > 0) {
            mean_speed /= (double)count;
            mean_err   /= (double)count;
        }

        phy_dbg_->log_row(
            now_wall_us(),
            (unsigned long)this->it,
            (int)update_msg.msg_type(),
            (unsigned long)t_target_us,
            (unsigned long)watermark_us,
            (unsigned long)n_waypoints,
            (unsigned long)payload_in_B,
            a0_px,a0_py,a0_pz,
            a0_vx,a0_vy,a0_vz,a0_speed,
            a0_rrx,a0_rry,a0_rrz,
            a0_rwx,a0_rwy,a0_rwz,
            a0_err,
            mean_speed,
            mean_err,
            (got_node_metrics_ ? 1 : 0),
            (unsigned long)got_node_metrics_n_,
            (unsigned long long)got_node_metrics_t_sim_us_,
            // NEW
            (int)cmd_inj_enable,
            (unsigned long)cmd_qsum
        );
    }


    // ===== agent-level debug CSV (unchanged) =====
    struct GoalCache
    {
        bool has_goal = false;
        double gx = 0.0, gy = 0.0, gz = 0.0;
        uint64_t last_goal_us = 0;
    };

    std::unordered_map<uint32_t, GoalCache> goal_cache_;

    std::unique_ptr<dancers::metrics::CsvMetricsLogger> metrics_agent_;
    bool enable_agent_csv_{true};
    size_t agent_csv_every_n_{1};
    uint64_t agent_csv_seq_{0};

    void InitAgentDebugCsv_()
    {
        std::string metrics_dir = (config_["metrics_dir"] ? config_["metrics_dir"].as<std::string>() : "logs");
        std::string exp_name    = (config_["experiment_name"] ? config_["experiment_name"].as<std::string>() : "exp");
        size_t flush_n          = (config_["flush_every_n"] ? config_["flush_every_n"].as<size_t>() : 50);

        dancers_fs::path out_dir = dancers_fs::path(this->ros_ws_path) / metrics_dir / exp_name;
        dancers_fs::create_directories(out_dir);
        dancers_fs::path out_csv = out_dir / "mini_dancers_agent_debug.csv";

        metrics_agent_ = std::make_unique<dancers::metrics::CsvMetricsLogger>(
            out_csv.string(),
            std::vector<std::string>{
                "wall_us", "t_sim_us", "agent_id",
                "has_goal", "goal_x", "goal_y", "goal_z", "dist_goal",
                "self_x", "self_y", "self_z", "self_vx", "self_vy", "self_vz",
                "goal_last_us", "step_wall_us", "payload_empty", "t_target_us"
            },
            flush_n
        );

        RCLCPP_INFO(this->get_logger(), "[PHY] agent debug csv enabled: %s", out_csv.string().c_str());
    }

    void LogAgentDebugCsv_(uint64_t t_sim_us_step_end,
                           uint64_t step_wall_us,
                           bool payload_empty,
                           uint64_t t_target_us_from_msg)
    {
        if (!metrics_agent_) return;

        if (agent_csv_every_n_ > 1) {
            if ((agent_csv_seq_++ % agent_csv_every_n_) != 0) return;
        } else {
            agent_csv_seq_++;
        }

        const uint64_t wall_us = now_wall_us();

        std::shared_lock lock(this->agents_mutex_);
        for (auto &kv : this->agents_)
        {
            const uint32_t agent_id = kv.first;
            auto *agent_ptr = kv.second.get();
            const auto st = agent_ptr->uav_system->getState();

            const double self_x = st.x.x(), self_y = st.x.y(), self_z = st.x.z();
            const double self_vx = st.v.x(), self_vy = st.v.y(), self_vz = st.v.z();

            auto itg = goal_cache_.find(agent_id);
            const bool has_goal = (itg != goal_cache_.end() && itg->second.has_goal);

            double gx = std::numeric_limits<double>::quiet_NaN();
            double gy = std::numeric_limits<double>::quiet_NaN();
            double gz = std::numeric_limits<double>::quiet_NaN();
            double dist = std::numeric_limits<double>::quiet_NaN();
            uint64_t goal_last_us = 0;

            if (has_goal)
            {
                gx = itg->second.gx; gy = itg->second.gy; gz = itg->second.gz;
                goal_last_us = itg->second.last_goal_us;
                const double dx = self_x - gx, dy = self_y - gy, dz = self_z - gz;
                dist = std::sqrt(dx*dx + dy*dy + dz*dz);
            }

            metrics_agent_->log_row(
                (unsigned long long)wall_us,
                (unsigned long long)t_sim_us_step_end,
                (unsigned long)agent_id,
                (has_goal ? 1 : 0), gx, gy, gz, dist,
                self_x, self_y, self_z, self_vx, self_vy, self_vz,
                (unsigned long long)goal_last_us,
                (unsigned long long)step_wall_us,
                (payload_empty ? 1 : 0),
                (unsigned long long)t_target_us_from_msg
            );
        }
    }

    void ReadConfigFile() override
    {
        this->use_uds = getYamlValue<bool>(this->config_, "phy_use_uds");
        this->uds_server_address = getYamlValue<std::string>(this->config_, "phy_uds_server_address");
        this->ip_server_address = getYamlValue<std::string>(this->config_, "phy_ip_server_address");
        this->ip_server_port = getYamlValue<unsigned int>(this->config_, "phy_ip_server_port");

        unsigned int sync_window_int = config_["sync_window"].as<unsigned int>();
        unsigned int step_size_int = config_["phy_step_size"].as<unsigned int>();
        if (sync_window_int % step_size_int != 0)
        {
            RCLCPP_FATAL(this->get_logger(), "Sync window must be a multiple of the physics step size, aborting.");
            exit(EXIT_FAILURE);
        }

        const uint64_t phy_step_us = getYamlValue<uint64_t>(this->config_, "phy_step_size");
        this->step_size = (double)phy_step_us / 1e6;
        this->dt_local_us_ = phy_step_us;

        if (this->dt_local_us_ == 0) {
            RCLCPP_FATAL(this->get_logger(), "phy_step_size cannot be 0");
            throw std::runtime_error("phy_step_size cannot be 0");
        }

        this->it_end_sim = uint64_t(this->simulation_length / this->step_size);
    }

        dancers_update_proto::DancersUpdate StepSimulation(dancers_update_proto::DancersUpdate update_msg) override
        {
            const uint64_t wall_begin_us = now_wall_us();

            const bool payload_empty = update_msg.payload().empty();
            const size_t payload_in_B = (size_t)update_msg.payload().size();

            // 以“上一个step结束时刻”作为本step开始时刻（t_recv_us）
            const uint64_t t_recv_us = (this->last_step_end_us_ == 0) ? 0 : this->last_step_end_us_;

            // ------------------------------------------------------------
            // Phase3: receive + decompress + parse side-channel net rx events (already gzip bytes)
            //         + ingest into CommModule (delayed delivery)
            // ------------------------------------------------------------
            if (!update_msg.net_rx_events_gz().empty())
            {
                last_net_rx_events_gz_ = update_msg.net_rx_events_gz();
                last_net_rx_events_bytes_ = last_net_rx_events_gz_.size();
                last_net_rx_events_window_count_++;

                // 1) decompress (gzip)
                std::string ev_raw;
                try
                {
                    ev_raw = gzip_decompress(last_net_rx_events_gz_);
                }
                catch (...)
                {
                    net_rx_events_parse_fail_++;
                    RCLCPP_WARN(this->get_logger(),
                        "[PHY] net_rx_events_gz decompress FAIL gz_bytes=%lu fail=%lu",
                        (unsigned long)last_net_rx_events_bytes_,
                        (unsigned long)net_rx_events_parse_fail_);
                    // 不影响旧闭环：继续走原 payload/物理推进逻辑
                }

                // 2) parse proto (only if decompress succeeded)
                if (!ev_raw.empty())
                {
                    protobuf_msgs::NetRxEventsPayload ev;
                    if (!ev.ParseFromString(ev_raw))
                    {
                        net_rx_events_parse_fail_++;
                        RCLCPP_WARN(this->get_logger(),
                            "[PHY] NetRxEventsPayload ParseFromString FAIL raw_bytes=%lu fail=%lu",
                            (unsigned long)ev_raw.size(),
                            (unsigned long)net_rx_events_parse_fail_);
                    }
                    else
                    {
                        // 3) summarize
                        last_net_rx_events_count_ = (uint64_t)ev.events_size();

                        RCLCPP_INFO(this->get_logger(),
                            "[PHY] net_rx_events OK win_count=%lu gz_bytes=%lu raw_bytes=%lu events=%lu win=[%lu,%lu]us",
                            (unsigned long)last_net_rx_events_window_count_,
                            (unsigned long)last_net_rx_events_bytes_,
                            (unsigned long)ev_raw.size(),
                            (unsigned long)last_net_rx_events_count_,
                            (unsigned long)ev.t_window_start_us(),
                            (unsigned long)ev.t_window_end_us());

                        // 4) print first event
                        if (ev.events_size() > 0)
                        {
                            const auto& e0 = ev.events(0);
                            const uint32_t tx_id = (uint32_t)e0.src_id();
                            const uint32_t rx_id = (uint32_t)e0.dst_id();

                            RCLCPP_INFO(this->get_logger(),
                                "[PHY] e0 rx=%u tx=%u flow=%u seq=%u tx_us=%lu rx_us=%lu delay_us=%lu bytes=%u dup=%d rssi_x10=%d",
                                (unsigned)rx_id,
                                (unsigned)tx_id,
                                (unsigned)e0.flow_id(),
                                (unsigned)e0.seq(),
                                (unsigned long)e0.tx_time_us(),
                                (unsigned long)e0.rx_time_us(),
                                (unsigned long)e0.delay_us(),
                                (unsigned)e0.payload_bytes(),
                                (int)e0.is_duplicate(),
                                (int)e0.rssi_dbm_x10());
                        }

                        // 5) optional CSV row
                        if (net_rx_ev_dbg_)
                        {
                            uint64_t e0_rx=0, e0_tx=0, e0_tx_us=0, e0_rx_us=0, e0_delay_us=0, e0_seq=0, e0_flow=0, e0_bytes=0;
                            int64_t  e0_rssi_x10=0;
                            uint64_t e0_dup=0;

                            if (ev.events_size() > 0)
                            {
                                const auto& e0 = ev.events(0);
                                e0_tx       = (uint64_t)e0.src_id();
                                e0_rx       = (uint64_t)e0.dst_id();
                                e0_flow     = (uint64_t)e0.flow_id();
                                e0_seq      = (uint64_t)e0.seq();
                                e0_tx_us    = (uint64_t)e0.tx_time_us();
                                e0_rx_us    = (uint64_t)e0.rx_time_us();
                                e0_delay_us = (uint64_t)e0.delay_us();
                                e0_bytes    = (uint64_t)e0.payload_bytes();
                                e0_rssi_x10 = (int64_t)e0.rssi_dbm_x10();
                                e0_dup      = (uint64_t)(e0.is_duplicate() ? 1 : 0);
                            }

                            net_rx_ev_dbg_->log_row(
                                (unsigned long long)now_wall_us(),
                                (unsigned long)this->it,
                                (unsigned long)last_net_rx_events_window_count_,
                                (unsigned long)last_net_rx_events_bytes_,
                                (unsigned long)ev_raw.size(),
                                (unsigned long)last_net_rx_events_count_,
                                (unsigned long)net_rx_events_parse_fail_,
                                (unsigned long long)ev.t_window_start_us(),
                                (unsigned long long)ev.t_window_end_us(),
                                (unsigned long)e0_rx,
                                (unsigned long)e0_tx,
                                (unsigned long)e0_flow,
                                (unsigned long)e0_seq,
                                (unsigned long long)e0_tx_us,
                                (unsigned long long)e0_rx_us,
                                (unsigned long long)e0_delay_us,
                                (unsigned long)e0_bytes,
                                (long long)e0_rssi_x10,
                                (unsigned long)e0_dup
                            );
                        }

                        // ---- Phase3: ingest into CommModule (delayed delivery) ----
                        if (comm_) {
                            comm_->IngestBatch(ev);
                        }
                        {
                            std::lock_guard<std::mutex> lk(latest_comm_rx_events_mu_);
                            latest_comm_rx_events_raw_ = ev_raw;
                        }
                    }
                }
            }

            // ------------------------------------------------------------
            // Existing logic: parse cmds / node_metrics and step physics
            // ------------------------------------------------------------
            // ------------------------------------------------------------
            // Command source selection:
            //   - use_vat_service_cmd_=true  : PHY pulls VAT service cmd, bypass payload cmd & Stage2 inject
            //   - use_vat_service_cmd_=false : keep your existing payload cmd (+ optional Stage2 inject)
            // ------------------------------------------------------------
            size_t n_cmds = 0;
            std::vector<uint32_t> updated_ids;
            updated_ids.reserve(16);

            // reset receipt indicator each step
            got_node_metrics_ = false;
            got_node_metrics_n_ = 0;
            got_node_metrics_t_sim_us_ = 0;

            dancers_update_proto::PoseVector velocity_messages;

            // cmd seq for Stage2 tracking
            uint64_t cmd_seq_u64 = 0;

            // Stage2 observability
            size_t n_cmds_enqueued = 0;

            // effective inject enable (VAT mode bypasses inject)
            const bool inj_enable_effective = (!use_vat_service_cmd_ && inject_delay_enable_);

            if (use_vat_service_cmd_)
            {
                // Make sure legacy queues don't affect VAT mode
                if (!cmd_queues_.empty()) cmd_queues_.clear();

                // Pull VAT cmd periodically; if failed, we keep last input (ZOH)
                QueryVatCmd_(t_recv_us);

                // Apply latest VAT cmd (if any)
                ApplyVatCmds_(updated_ids);

                // payload cmds are bypassed in this mode
                n_cmds = 0;
                n_cmds_enqueued = 0;
            }
            else
            {
                // ----------------------------
                // Legacy payload cmd path
                // ----------------------------
                if (!payload_empty)
                {
                    const std::string raw = gzip_decompress(update_msg.payload());

                    // Try CmdPayload (new NET->PHY)
                    dancers_update_proto::CmdPayload cp;
                    if (cp.ParseFromString(raw) && !cp.cmd_pose_vector().empty())
                    {
                        cmd_seq_u64 = (uint64_t)cp.seq();

                        if (!velocity_messages.ParseFromString(cp.cmd_pose_vector()))
                        {
                            RCLCPP_WARN(this->get_logger(),
                                        "CmdPayload parsed but cmd_pose_vector PoseVector parse failed (bytes=%zu)",
                                        (size_t)cp.cmd_pose_vector().size());
                        }

                        got_node_metrics_ = (cp.node_metrics_size() > 0);
                        got_node_metrics_n_ = (size_t)cp.node_metrics_size();
                        got_node_metrics_t_sim_us_ = cp.t_sim_us();

                        if (got_node_metrics_)
                        {
                            last_node_metrics_.clear();
                            for (const auto& nm : cp.node_metrics())
                            {
                                last_node_metrics_[(uint32_t)nm.node_id()] = nm;
                            }

                            if (last_metrics_print_it_ != (uint64_t)this->it)
                            {
                                last_metrics_print_it_ = (uint64_t)this->it;
                                RCLCPP_INFO(this->get_logger(),
                                            "[PHY] Received node comm metrics: n=%zu (t_sim_us=%llu, seq=%llu, win=%llu)",
                                            got_node_metrics_n_,
                                            (unsigned long long)cp.t_sim_us(),
                                            (unsigned long long)cp.seq(),
                                            (unsigned long long)cp.window_idx());
                            }
                        }
                    }
                    else
                    {
                        if (!velocity_messages.ParseFromString(raw))
                        {
                            RCLCPP_WARN(this->get_logger(), "Failed to parse legacy PoseVector (raw=%zuB)", raw.size());
                        }
                    }

                    n_cmds = (size_t)velocity_messages.pose_size();

                    if (!inj_enable_effective)
                    {
                        if (!cmd_queues_.empty()) cmd_queues_.clear();

                        std::unique_lock lock(this->agents_mutex_);
                        for (const auto& vel : velocity_messages.pose())
                        {
                            const uint32_t agent_id = (uint32_t)vel.agent_id();

                            reference::VelocityHdg ctrl;
                            ctrl.velocity[0] = vel.vx();
                            ctrl.velocity[1] = vel.vy();
                            ctrl.velocity[2] = vel.vz();
                            ctrl.heading = 0.0;

                            auto itag = this->agents_.find(agent_id);
                            if (itag != this->agents_.end())
                            {
                                itag->second->uav_system->setInput(ctrl);

                                auto &gc = goal_cache_[agent_id];
                                gc.has_goal = true;
                                gc.gx = (double)ctrl.velocity[0];
                                gc.gy = (double)ctrl.velocity[1];
                                gc.gz = (double)ctrl.velocity[2];
                            }
                        }

                        updated_ids.clear();
                    }
                    else
                    {
                        for (const auto& vel : velocity_messages.pose())
                        {
                            const uint32_t agent_id = (uint32_t)vel.agent_id();

                            reference::VelocityHdg controller;
                            controller.velocity[0] = vel.vx();
                            controller.velocity[1] = vel.vy();
                            controller.velocity[2] = vel.vz();
                            controller.heading = 0.0;

                            uint64_t t_sample_us = 0;
                            if (got_node_metrics_t_sim_us_ > 0) t_sample_us = got_node_metrics_t_sim_us_;

                            uint64_t metric_delay_us = 0;
                            auto itnm = last_node_metrics_.find(agent_id);
                            if (itnm != last_node_metrics_.end()) {
                                metric_delay_us = (uint64_t)itnm->second.delay_mean_us();
                            }
                            last_metric_delay_us_by_agent_[agent_id] = metric_delay_us;

                            const uint64_t injected_delay_us = GetDelayMeanUsForAgent_(agent_id);

                            EnqueueCmd_(agent_id,
                                        controller,
                                        /*t_recv_us=*/t_recv_us,
                                        /*t_sample_us=*/t_sample_us,
                                        /*seq=*/(cmd_seq_u64 != 0 ? cmd_seq_u64 : (uint64_t)this->it),
                                        /*injected_delay_us=*/injected_delay_us);

                            n_cmds_enqueued++;
                        }

                        ApplyDueCmds_(t_recv_us, updated_ids);
                    }
                }
                else
                {
                    // payload empty -> keep previous input (ZOH)
                    // (If you want: if (inj_enable_effective) ApplyDueCmds_(t_recv_us, updated_ids); )
                }
            }


            // ============================================================
            // Stage2 observability: 先算统计（t_step_end_us 还没算出来，稍后再写 CSV）
            // ============================================================
            const size_t n_cmds_applied_agents = updated_ids.size();

            size_t q_sum = 0;
            size_t q_max = 0;
            size_t a0_q  = 0;
            for (auto &kv : cmd_queues_) {
                const size_t qs = kv.second.size();
                q_sum += qs;
                q_max = std::max(q_max, qs);
                if (kv.first == 0) a0_q = qs;
            }

            const uint64_t a0_metric = (last_metric_delay_us_by_agent_.count(0) ? last_metric_delay_us_by_agent_[0] : 0ull);
            const uint64_t a0_inj    = (last_injected_delay_us_by_agent_.count(0) ? last_injected_delay_us_by_agent_[0] : 0ull);
            const uint64_t a0_seq    = (last_cmd_seq_by_agent_.count(0) ? last_cmd_seq_by_agent_[0] : 0ull);
            const uint64_t a0_e2e    = (last_cmd_e2e_us_by_agent_.count(0) ? last_cmd_e2e_us_by_agent_[0] : 0ull);

            // Step physics (multi-thread)
            std::vector<Agent*> agent_list;
            agent_list.reserve(this->agents_.size());
            {
                std::shared_lock lk(this->agents_mutex_);
                for (auto& kv : this->agents_) agent_list.push_back(kv.second.get());
            }

            const size_t n_agents = agent_list.size();
            if (n_agents > 0)
            {
                unsigned hw = std::thread::hardware_concurrency();
                if (hw == 0) hw = 1;
                unsigned n_threads = hw;
                if (n_threads > (unsigned)n_agents) n_threads = (unsigned)n_agents;
                if (n_threads < 1) n_threads = 1;
                if (n_agents < 16) n_threads = 1;

                std::vector<std::thread> workers;
                workers.reserve(n_threads);

                for (unsigned t = 0; t < n_threads; ++t)
                {
                    const size_t begin = (n_agents * t) / n_threads;
                    const size_t end   = (n_agents * (t + 1)) / n_threads;

                    workers.emplace_back([this, &agent_list, begin, end]() {
                        for (size_t i = begin; i < end; ++i) {
                            agent_list[i]->uav_system->makeStep(this->step_size);
                        }
                    });
                }

                for (auto& w : workers) w.join();
            }

            this->DisplayRviz();

            const uint64_t step_wall_us = now_wall_us() - wall_begin_us;

            // Advance local sim-time (legacy)
            const uint64_t t_step_end_us = (this->last_step_end_us_ == 0) ? this->dt_local_us_
                                                                        : (this->last_step_end_us_ + this->dt_local_us_);
            this->last_step_end_us_ = t_step_end_us;

            // ---- Phase3: deliver events whose deliver_us <= now_us ----
            if (comm_) {
                comm_->Tick(t_step_end_us);
            }

            // back-fill goal time
            for (auto id : updated_ids) {
                auto it = goal_cache_.find(id);
                if (it != goal_cache_.end()) it->second.last_goal_us = t_step_end_us;
            }

            // ============================================================
            // Stage2 observability: 现在写 CSV + 控制台偶尔打印
            // ============================================================
            if (cmd_delay_dbg_ && (cmd_delay_dbg_every_n_ <= 1 || (this->it % cmd_delay_dbg_every_n_) == 0))
            {
                cmd_delay_dbg_->log_row(
                    (unsigned long long)now_wall_us(),
                    (unsigned long)this->it,
                    (unsigned long long)t_recv_us,
                    (unsigned long long)t_step_end_us,
                    (unsigned long)n_cmds,
                    (unsigned long)n_cmds_enqueued,
                    (unsigned long)n_cmds_applied_agents,
                    (unsigned long)q_sum,
                    (unsigned long)q_max,
                    (unsigned long)a0_q,
                    (unsigned long long)a0_metric,
                    (unsigned long long)a0_inj,
                    (unsigned long long)a0_seq,
                    (unsigned long long)a0_e2e,
                    (got_node_metrics_ ? 1 : 0),
                    (unsigned long)got_node_metrics_n_,
                    (unsigned long long)got_node_metrics_t_sim_us_
                );

                if (cmd_delay_print_every_n_ > 0 && (this->it % cmd_delay_print_every_n_) == 0) {
                    RCLCPP_INFO(this->get_logger(),
                        "[CMD_DELAY] it=%lu now=%lluus inj_enable=%d applied_agents=%zu q_sum=%zu q_max=%zu | a0(metric=%lluus inj=%lluus e2e=%lluus q=%zu seq=%llu) got_metrics=%d",
                        (unsigned long)this->it,
                        (unsigned long long)t_recv_us,
                        (inj_enable_effective ? 1 : 0),   // ✅ 修复：原来是 inj_enable
                        n_cmds_applied_agents,
                        q_sum, q_max,
                        (unsigned long long)a0_metric,
                        (unsigned long long)a0_inj,
                        (unsigned long long)a0_e2e,
                        a0_q,
                        (unsigned long long)a0_seq,
                        (got_node_metrics_ ? 1 : 0)
                    );
                }

            }

            const uint64_t t_target_us = t_step_end_us;
            const uint64_t watermark_us = t_step_end_us;

            this->log_phy_debug_row_(update_msg, t_target_us, watermark_us, n_cmds, payload_in_B);
            this->LogAgentDebugCsv_(t_step_end_us, step_wall_us, payload_empty, t_target_us);

            return this->GenerateResponseProtobuf();
        }



        /**
        * @brief Generate the response protobuf message to send to the coordinator
        * Keep legacy: PoseVector compressed (PHY->NET no protocol change)
        */
        dancers_update_proto::DancersUpdate GenerateResponseProtobuf()
        {
            dancers_update_proto::DancersUpdate response_msg;
            response_msg.set_msg_type(dancers_update_proto::DancersUpdate::END);

            std::shared_lock lock(this->agents_mutex_);
            dancers_update_proto::PoseVector robots_positions_msg;
            for (auto& [agent_id, agent_ptr] : this->agents_)
            {
                dancers_update_proto::Pose *robot_pose_msg = robots_positions_msg.add_pose();
                Eigen::Vector3d agent_position = agent_ptr->uav_system->getState().x;
                Eigen::Vector3d agent_velocity = agent_ptr->uav_system->getState().v;
                robot_pose_msg->set_agent_id(agent_ptr->id);
                robot_pose_msg->set_x(agent_position.x());
                robot_pose_msg->set_y(agent_position.y());
                robot_pose_msg->set_z(agent_position.z());
                robot_pose_msg->set_vx(agent_velocity.x());
                robot_pose_msg->set_vy(agent_velocity.y());
                robot_pose_msg->set_vz(agent_velocity.z());
            }

            std::string serialized_msg;
            robots_positions_msg.SerializeToString(&serialized_msg);
            response_msg.set_payload(gzip_compress(serialized_msg));
            return response_msg;
        }

        void InitAgents();
        void DisplayRviz();
        void SpawnUavClbk(const dancers_msgs::AgentStruct& msg);

        ros::Publisher pose_array_pub_;
        ros::Publisher id_markers_pub_;
        ros::Publisher obstacles_pub_;
        ros::Publisher targets_pub_;

        std::map<uint32_t, std::unique_ptr<Agent>> agents_;
        mutable std::shared_mutex agents_mutex_;

        std::thread loop_thread_;
    };



void MiniDancers::InitAgents()
{
    const unsigned int robots_number = getYamlValue<unsigned int>(this->config_, "robots_number");
    const YAML::Node initial_positions = this->config_["initial_positions"];

    if (initial_positions && initial_positions.IsSequence())
    {
        if (initial_positions.size() < robots_number)
        {
            throw std::runtime_error("initial_positions has fewer entries than robots_number.");
        }

        for (unsigned int i = 0; i < robots_number; ++i)
        {
            const YAML::Node pose = initial_positions[i];

            dancers_msgs::AgentStruct agent_ros_msg;
            agent_ros_msg.agent_id = i;
            agent_ros_msg.state.position.x = getYamlValue<double>(pose, "x");
            agent_ros_msg.state.position.y = getYamlValue<double>(pose, "y");
            agent_ros_msg.state.position.z = getYamlValue<double>(pose, "z");
            agent_ros_msg.state.heading = pose["heading"] ? pose["heading"].as<double>() : 0.0;
            agent_ros_msg.state.velocity.x = 0.0;
            agent_ros_msg.state.velocity.y = 0.0;
            agent_ros_msg.state.velocity.z = 0.0;

            this->SpawnUavClbk(agent_ros_msg);
        }
        return;
    }

    const int n_columns = std::max(1, (int)std::sqrt(getYamlValue<int>(this->config_, "robots_number")));
    const double spacing = getYamlValue<double>(this->config_, "initial_spacing");
    const Eigen::Vector3d grid_center = Eigen::Vector3d(getYamlValue<double>(this->config_, "initial_x"),
                                                        getYamlValue<double>(this->config_, "initial_y"),
                                                        getYamlValue<double>(this->config_, "initial_z"));
    const double initial_heading = getYamlValue<double>(this->config_, "initial_heading");

    for (unsigned int i = 0; i < robots_number; ++i)
    {
        dancers_msgs::AgentStruct agent_ros_msg;
        agent_ros_msg.agent_id = i;

        const int row = (int)i / n_columns;
        const int col = (int)i % n_columns;
        agent_ros_msg.state.position.x = grid_center.x() + (col - n_columns / 2) * spacing;
        agent_ros_msg.state.position.y = grid_center.y() + (row - n_columns / 2) * spacing;
        agent_ros_msg.state.position.z = grid_center.z();
        agent_ros_msg.state.heading = initial_heading;

        agent_ros_msg.state.velocity.x = 0.0;
        agent_ros_msg.state.velocity.y = 0.0;
        agent_ros_msg.state.velocity.z = 0.0;

        this->SpawnUavClbk(agent_ros_msg);
    }
}

void MiniDancers::SpawnUavClbk(const dancers_msgs::AgentStruct& msg)
{
    {
        std::shared_lock lock(this->agents_mutex_);
        if (this->agents_.find(msg.agent_id) != this->agents_.end())
        {
            RCLCPP_WARN(this->get_logger(), "Agent %d already exists, skipping spawn.", msg.agent_id);
            return;
        }
    }

    {
        std::unique_lock lock(this->agents_mutex_);
        auto [it, success] = this->agents_.emplace(msg.agent_id, std::make_unique<Agent>(AgentFromRosMsg(msg)));
        if (!success)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to insert new agent %d in the map, aborting.", msg.agent_id);
            exit(EXIT_FAILURE);
        }
        auto& [id, agent_ptr] = *it;

        MultirotorModel::ModelParams x500_params = MultirotorModel::ModelParams();
        x500_params.ground_enabled = true;

        agent_ptr->uav_system = std::make_shared<UavSystem>(x500_params, agent_ptr->position, agent_ptr->heading);
    }
}

void MiniDancers::DisplayRviz()
{
    geometry_msgs::PoseArray pose_array{};
    pose_array.header.frame_id = "map";
    pose_array.header.stamp = ros::Time::now();
    visualization_msgs::MarkerArray id_marker_array{};

    std::shared_lock lock(this->agents_mutex_);
    for (auto& [agent_id, agent_struct] : this->agents_)
    {
        MultirotorModel::State uav_state = agent_struct->uav_system->getState();
        geometry_msgs::Pose pose{};
        pose.position.x = uav_state.x.x();
        pose.position.y = uav_state.x.y();
        pose.position.z = uav_state.x.z();
        Eigen::Quaternion<double> quat = Eigen::Quaternion<double>(uav_state.R);
        pose.orientation.w = quat.w();
        pose.orientation.x = quat.x();
        pose.orientation.y = quat.y();
        pose.orientation.z = quat.z();
        pose_array.poses.push_back(pose);

        visualization_msgs::Marker id_marker{};
        id_marker.id = (int)agent_id;
        id_marker.header.frame_id = "map";
        id_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        id_marker.action = visualization_msgs::Marker::ADD;
        id_marker.color.r = 0.0;
        id_marker.color.g = 0.0;
        id_marker.color.b = 1.0;
        id_marker.color.a = 1.0;
        id_marker.scale.z = 2.0;
        id_marker.lifetime = ros::Duration(0.1);
        id_marker.text = std::to_string(agent_id);
        id_marker.pose.position.x = uav_state.x.x() - 1;
        id_marker.pose.position.y = uav_state.x.y();
        id_marker.pose.position.z = uav_state.x.z();
        id_marker_array.markers.push_back(id_marker);
    }

    visualization_msgs::MarkerArray obstacles_marker_array{};
    for (const auto& obstacle : this->config_["obstacles"])
    {
        visualization_msgs::Marker box;
        box.header.frame_id = "map";
        box.header.stamp = ros::Time::now();
        box.ns = "obstacles";
        box.id = obstacle["id"].as<int>();
        box.type = visualization_msgs::Marker::CUBE;
        box.action = visualization_msgs::Marker::ADD;

        box.pose.position.x = obstacle["x"].as<double>();
        box.pose.position.y = obstacle["y"].as<double>();
        box.pose.position.z = obstacle["z"].as<double>();
        box.pose.orientation.w = 1.0;

        box.scale.x = obstacle["size_x"].as<double>();
        box.scale.y = obstacle["size_y"].as<double>();
        box.scale.z = obstacle["size_z"].as<double>();

        box.color.r = 0.5;
        box.color.g = 0.5;
        box.color.b = 0.5;
        box.color.a = 0.8;

        box.lifetime = ros::Duration(0.0);
        obstacles_marker_array.markers.push_back(box);
    }

    visualization_msgs::MarkerArray targets_marker_array{};
    for (const auto& target : this->config_["targets"])
    {
        visualization_msgs::Marker sphere;
        sphere.header.frame_id = "map";
        sphere.header.stamp = ros::Time::now();
        sphere.ns = "targets";
        sphere.id = target["id"].as<int>();
        sphere.type = visualization_msgs::Marker::SPHERE;
        sphere.action = visualization_msgs::Marker::ADD;

        sphere.pose.position.x = target["x"].as<double>();
        sphere.pose.position.y = target["y"].as<double>();
        sphere.pose.position.z = target["z"].as<double>();
        sphere.pose.orientation.w = 1.0;

        double radius = target["radius"].as<double>();
        sphere.scale.x = 2 * radius;
        sphere.scale.y = 2 * radius;
        sphere.scale.z = 2 * radius;

        if (target["is_sink"])
        {
            sphere.color.r = 0.0;
            sphere.color.g = 1.0;
            sphere.color.b = 0.0;
        }
        else
        {
            sphere.color.r = 1.0;
            sphere.color.g = 0.0;
            sphere.color.b = 0.0;
        }
        sphere.color.a = 0.5;

        sphere.lifetime = ros::Duration(0.0);
        targets_marker_array.markers.push_back(sphere);
    }

    this->pose_array_pub_.publish(pose_array);
    this->id_markers_pub_.publish(id_marker_array);
    this->obstacles_pub_.publish(obstacles_marker_array);
    this->targets_pub_.publish(targets_marker_array);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mini_dancers_velocity");
    MiniDancers node;
    ros::spin();
    return 0;
}
