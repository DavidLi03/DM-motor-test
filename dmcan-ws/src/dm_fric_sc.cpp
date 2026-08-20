// dm_fric_sc.cpp —— 摩擦力系统辨识：多平台恒速扫描 + coast-down 自由停转
//
// 方法（与 dm_demo_sc 同一控制律：MIT 纯阻尼 kp=0, tau=0 ⇒ T = kd*(dq_des - dq)）：
//   1) 恒速平台法：稳态时 kd*(dq_des - ω) = T_f(ω)。每个平台给出一个 (ω, T_f) 点，
//      T_f 双法互证：反解 kd_q*(dq_cmd - ω_位移法) vs 巡航段 tau 遥测均值。
//   2) coast-down 法（H 批末尾）：最后一个平台结束后 kd 归零（保持使能、零力矩、
//      100Hz 继续发帧喂 ~120ms 看门狗），自由滑行到停。J*dω/dt = -T_f(ω)，
//      与平台点锚定可反推转动惯量 J，曲线形状独立验证平台法。
//
// 两条与 kd 无关的安全性质依旧成立：速度天花板只由 dq_des 决定；
// 最大力矩 = kd*(dq_des - dq)，堵转时 = kd*dq_des，联合钳位 kd*dq <= 4.0 Nm。
// coast 段 kd=0 ⇒ 全程零力矩，安全性与失能等同（但反馈与看门狗不断）。
//
// 批次（--set，双向：每个速度先 +ω 平台再 -ω 平台，行程自然对消）：
//   L: 0.08 0.12 0.18 0.28 0.42 rad/s  kd=1.5      测量窗 4s  （Stribeck 区）
//   M: 0.6  1.0  1.6  2.5   rad/s      kd=1.5      测量窗 3s  （库仑平台区）
//   H: 4.0  5.5  7.0  9.0   rad/s      kd=4.0/ω    测量窗 3s  （粘性区）+ coast-down
//   coast: 升到 9 rad/s 巡航 1s 后 coast-down（单独重跑惯量测试用）
//
// 会发送：0xFB 清错 / 0xFC 使能 / 0xFD 失能 / MIT 帧
// 绝不发送：0x55 写参数 / 0xAA 存参数 / 0xFE 存零点
//
// 停止途径（全部收敛到 0xFD 失能）：自然结束 / Ctrl-C×1 柔停 / Ctrl-C×2 立即失能
//   / 超速 / 力矩异常 / 过温(70℃) / ERR 故障 / 反馈丢失 / 计划时长+15s 兜底
//
// 用法: dm_fric_sc --set L|M|H|coast [--if can0] [--speeds 0.6,1.0,...]
//                  [--accel N] [--csv 路径] [--list]
//   --list 只打印段表与时长估计，不打开 CAN、不发任何帧（dry check 用）。

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

#include "dm_sc.hpp"

namespace {

constexpr int kPeriodMs = 10;    // 100Hz，远高于 ~120ms 看门狗
constexpr int kGraceMs  = 150;

// ---- 安全钳位 ----
constexpr float  kDqLimit    = 9.0f;   // 速度上限 rad/s（demo2 已验证的包络）
constexpr float  kKdLimit    = 1.5f;   // kd 上限（dm_spin 先例；仍受联合钳位约束）
constexpr float  kStallCap   = 4.0f;   // kd*dq 联合上限 Nm（堵转力矩天花板）
constexpr float  kAccelMin   = 0.5f;   // 斜坡限速器加速度边界 rad/s^2
constexpr float  kAccelMax   = 12.0f;  // demo2 已验证
constexpr float  kVelMargin  = 2.0f;   // 超速保护阈 = 全程最大目标速度 + 该裕量
constexpr float  kTauAbort   = 5.0f;
constexpr int    kTempAbort  = 70;
constexpr double kSettleS    = 0.7;    // 到速后的稳定期（等瞬态衰减，不计入测量窗）
constexpr double kBrakeS     = 0.5;
constexpr double kCoastMaxS  = 8.0;    // coast 段超时
constexpr float  kCoastStopV = 0.05f;  // |vel| 低于此值持续 0.3s 视为停转
constexpr int    kMaxSpeeds  = 12;     // 自定义速度点数上限（双向 = 平台数×2）

constexpr int kLogMax = 16000;         // 100Hz 下够 160 秒

using clk = std::chrono::steady_clock;
clk::time_point g_t0;
double now_s() { return std::chrono::duration<double>(clk::now() - g_t0).count(); }

std::atomic<int>  g_sig{0};
std::atomic<bool> g_rx_quit{false};
std::atomic<int>  g_rx{0}, g_tx{0}, g_txfail{0}, g_canerr{0};
std::atomic<int>  g_last_err{-1}, g_prev_err{-1};

std::atomic<float> g_pos{0}, g_vel{0}, g_tau{0};
std::atomic<float> g_travel{0};
std::atomic<int>   g_tmos{0}, g_trotor{0};
std::atomic<float> g_vel_max{0}, g_tau_max{0};
std::atomic<float> g_dq_cmd{0}, g_kd_cmd{0};
std::atomic<int>   g_seg{-1};          // 当前段号（-1=段表外：宽限/收尾）
std::atomic<int>   g_phase{0};         // 0=斜坡/稳定 1=测量窗 2=coast

struct Sample {
    float t, pos, trav, vel, tau, dq, kd;
    uint64_t kts;
    uint8_t tmos, trotor;
    int8_t seg, phase;
};
Sample g_log[kLogMax];
std::atomic<int> g_log_n{0};

// 解卷绕状态 —— 只在 RX 线程访问（±12.5 rad 回卷，同 dm_demo_sc）
float g_prev_pos = 0.0f;
bool  g_have_prev = false;

void handle_frame(const can_frame& fr, uint64_t kts) {
    if (fr.can_id & CAN_ERR_FLAG) {
        g_canerr++;
        std::printf("  [CAN-ERR] t=%6.3fs id=0x%08X\n", now_s(), fr.can_id);
        std::fflush(stdout);
        return;
    }
    if ((fr.can_id & CAN_SFF_MASK) != dmsc::kMstId || fr.can_dlc < 8) return;

    const uint8_t* d = fr.data;
    uint8_t id  = d[0] & 0x0F;
    uint8_t err = (d[0] >> 4) & 0x0F;
    if (id != (dmsc::kEscId & 0x0F)) return;

    uint16_t q_u   = ((uint16_t)d[1] << 8) | d[2];
    uint16_t dq_u  = ((uint16_t)d[3] << 4) | (d[4] >> 4);
    uint16_t tau_u = ((uint16_t)(d[4] & 0x0F) << 8) | d[5];

    float pos = dmsc::uint_to_float(q_u,   -dmsc::kPMax, dmsc::kPMax, 16);
    float vel = dmsc::uint_to_float(dq_u,  -dmsc::kVMax, dmsc::kVMax, 12);
    float tau = dmsc::uint_to_float(tau_u, -dmsc::kTMax, dmsc::kTMax, 12);

    if (g_have_prev) {
        float dlt = pos - g_prev_pos;
        if (dlt >  dmsc::kPMax) dlt -= 2.0f * dmsc::kPMax;
        if (dlt < -dmsc::kPMax) dlt += 2.0f * dmsc::kPMax;
        g_travel.store(g_travel.load() + dlt);
    } else {
        g_have_prev = true;
    }
    g_prev_pos = pos;

    g_pos.store(pos);
    g_vel.store(vel);
    g_tau.store(tau);
    g_tmos.store(d[6]);
    g_trotor.store(d[7]);
    if (std::fabs(vel) > g_vel_max.load()) g_vel_max.store(std::fabs(vel));
    if (std::fabs(tau) > g_tau_max.load()) g_tau_max.store(std::fabs(tau));
    g_rx++;

    int n = g_log_n.load();
    if (n < kLogMax) {
        g_log[n] = {(float)now_s(), pos, g_travel.load(), vel, tau,
                    g_dq_cmd.load(), g_kd_cmd.load(), kts, d[6], d[7],
                    (int8_t)g_seg.load(), (int8_t)g_phase.load()};
        g_log_n.store(n + 1);
    }

    int prev = g_prev_err.exchange(err);
    g_last_err.store(err);
    if (prev != (int)err) {
        std::printf("  [ERR跳变] t=%6.3fs  %s -> %d(%s)\n", now_s(),
                    prev < 0 ? "(初始)" : dmsc::err_name((uint8_t)prev),
                    err, dmsc::err_name(err));
        std::fflush(stdout);
    }
}

void rx_loop(int s) {
    can_frame fr{};
    uint64_t kts = 0;
    while (!g_rx_quit.load()) {
        int r = dmsc::recv_frame(s, &fr, &kts, 20);
        if (r <= 0) continue;
        handle_frame(fr, kts);
    }
}

void on_signal(int) { g_sig++; }

void send_cmd_counted(int s, uint8_t cmd) {
    if (dmsc::send_cmd(s, cmd)) g_tx++;
    else g_txfail++;
}
void send_mit_counted(int s, float dq, float kd) {
    // kp 与 tau 固定 0 —— 见文件头安全说明
    if (dmsc::send_mit(s, 0.0f, dq, 0.0f, kd, 0.0f)) g_tx++;
    else g_txfail++;
}
void burst(int s, uint8_t cmd, int n, int tail_ms) {
    for (int i = 0; i < n; ++i) {
        send_cmd_counted(s, cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(3));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(tail_ms));
}
void disable_motor(int s) {
    std::printf("\n>>> 失能 0xFD ×5 ...\n");
    burst(s, 0xFD, 5, 120);
    int e = g_last_err.load();
    std::printf(">>> 失能后 ERR = %d (%s)\n", e, e < 0 ? "无反馈" : dmsc::err_name((uint8_t)e));
}

// ---- 段表 ----
enum SegType { SEG_PLATEAU, SEG_COAST };
struct Seg {
    float   dq;        // 目标速度（带符号；COAST 段无意义）
    float   kd;        // 平台阻尼（COAST 段为 0）
    double  meas_s;    // 测量窗时长（COAST 段为超时上限）
    SegType type;
};

// 该速度允许的最大 kd：min(kKdLimit, kStallCap/|v|)
float kd_for(float v) {
    float kd = kKdLimit;
    if (kd * v > kStallCap) kd = kStallCap / v;
    return kd;
}

// 双向展开：每个速度 +v 平台后紧跟 -v 平台
void push_bidir(std::vector<Seg>* segs, float v, double meas_s) {
    float kd = kd_for(v);
    segs->push_back({+v, kd, meas_s, SEG_PLATEAU});
    segs->push_back({-v, kd, meas_s, SEG_PLATEAU});
}

// 计划时长估计（斜坡 + 稳定 + 测量，再加收尾）
double plan_seconds(const std::vector<Seg>& segs, float accel) {
    double t = kGraceMs / 1000.0;
    float dq_prev = 0.0f;
    for (const Seg& sg : segs) {
        if (sg.type == SEG_COAST) { t += sg.meas_s; continue; }
        t += std::fabs(sg.dq - dq_prev) / accel + kSettleS + sg.meas_s;
        dq_prev = sg.dq;
    }
    t += std::fabs(dq_prev) / accel + kBrakeS + 0.5;
    return t;
}

// 平台测量结果（即时打印 + 结尾汇总表）
struct PlatRes {
    float tgt, kd, v_meas, att_pct, fric_bs, tau_mean, tau_sd, stick_pct;
    int tmos, trotor;
};

}  // namespace

int main(int argc, char** argv) {
    std::string ifname = "can0";
    std::string set_name, csv_path;
    std::vector<float> custom_speeds;
    float accel = 0.0f;      // 0 = 用批次默认
    bool  list_only = false;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--if" && i + 1 < argc)          ifname = argv[++i];
        else if (a == "--set" && i + 1 < argc)    set_name = argv[++i];
        else if (a == "--accel" && i + 1 < argc)  accel = std::stof(argv[++i]);
        else if (a == "--csv" && i + 1 < argc)    csv_path = argv[++i];
        else if (a == "--list")                   list_only = true;
        else if (a == "--speeds" && i + 1 < argc) {
            std::string sp = argv[++i];
            size_t p = 0;
            while (p < sp.size()) {
                size_t q = sp.find(',', p);
                if (q == std::string::npos) q = sp.size();
                custom_speeds.push_back(std::stof(sp.substr(p, q - p)));
                p = q + 1;
            }
        } else if (a == "--help") {
            std::printf(
                "用法: dm_fric_sc --set L|M|H|coast [--if can0] [--speeds 0.6,1.0,...]\n"
                "                 [--accel N] [--csv 路径] [--list]\n"
                "摩擦辨识：恒速平台扫描（每速度 +/- 双向）+ coast-down 自由停转（H 批末尾）\n"
                "  L: 0.08~0.42 rad/s ×5 kd=1.5  (~%ds)   M: 0.6~2.5 ×4 kd=1.5  (~%ds)\n"
                "  H: 4.0~9.0 ×4 kd=4/ω + coast  (~%ds)   coast: 升 9 rad/s 后滑停\n"
                "钳位: |dq|<=%.1f rad/s  kd<=%.1f 且 kd*dq<=%.1f Nm  accel∈[%.1f,%.0f]\n"
                "保护: 超速/|tau|>%.1fNm/温度>%d℃/ERR/反馈丢失/计划时长+15s兜底 ⇒ 立即失能\n"
                "Ctrl-C ×1 = 柔停（斜坡减速）   Ctrl-C ×2 = 立即失能\n"
                "--list 只打印段表不发帧（dry check）\n",
                55, 40, 75, kDqLimit, kKdLimit, kStallCap, kAccelMin, kAccelMax,
                kTauAbort, kTempAbort);
            return 0;
        }
    }

    // ---- 组段表 ----
    std::vector<Seg> segs;
    bool with_coast = false;
    float accel_def = 3.0f;

    if (!custom_speeds.empty()) {
        if ((int)custom_speeds.size() > kMaxSpeeds) {
            std::printf("[钳位] 速度点 %zu 个 -> 前 %d 个\n", custom_speeds.size(), kMaxSpeeds);
            custom_speeds.resize(kMaxSpeeds);
        }
        for (float v : custom_speeds) {
            v = std::fabs(v);
            if (v > kDqLimit) { std::printf("[钳位] 速度 %.3f -> %.3f rad/s\n", v, kDqLimit); v = kDqLimit; }
            if (v < 0.05f)    { std::printf("[钳位] 速度 %.3f -> 0.05 rad/s\n", v); v = 0.05f; }
            push_bidir(&segs, v, v < 0.5f ? 4.0 : 3.0);
        }
        accel_def = 3.0f;
        set_name = "custom";
    } else if (set_name == "L") {
        for (float v : {0.08f, 0.12f, 0.18f, 0.28f, 0.42f}) push_bidir(&segs, v, 4.0);
        accel_def = 2.0f;
    } else if (set_name == "M") {
        for (float v : {0.6f, 1.0f, 1.6f, 2.5f}) push_bidir(&segs, v, 3.0);
        accel_def = 3.0f;
    } else if (set_name == "H") {
        for (float v : {4.0f, 5.5f, 7.0f, 9.0f}) push_bidir(&segs, v, 3.0);
        segs.push_back({0.0f, 0.0f, kCoastMaxS, SEG_COAST});   // 从末平台 -9 rad/s 直接滑停
        accel_def = 10.0f;
        with_coast = true;
    } else if (set_name == "coast") {
        segs.push_back({+9.0f, kd_for(9.0f), 1.0, SEG_PLATEAU});
        segs.push_back({0.0f, 0.0f, kCoastMaxS, SEG_COAST});
        accel_def = 10.0f;
        with_coast = true;
    } else {
        std::printf("须指定 --set L|M|H|coast 或 --speeds（--help 看说明）\n");
        return 1;
    }

    if (accel == 0.0f) accel = accel_def;
    if (accel > kAccelMax) { std::printf("[钳位] accel %.2f -> %.2f\n", accel, kAccelMax); accel = kAccelMax; }
    if (accel < kAccelMin) { std::printf("[钳位] accel %.2f -> %.2f\n", accel, kAccelMin); accel = kAccelMin; }

    // 全程最大速度 → 超速保护阈；最大堵转力矩 → 打印
    float v_top = 0.0f, stall_max = 0.0f;
    for (const Seg& sg : segs) {
        if (std::fabs(sg.dq) > v_top) v_top = std::fabs(sg.dq);
        if (sg.kd * std::fabs(sg.dq) > stall_max) stall_max = sg.kd * std::fabs(sg.dq);
    }
    const float  vel_abort = v_top + kVelMargin;
    const double plan_s    = plan_seconds(segs, accel);
    const double backstop  = plan_s + 15.0;

    g_t0 = clk::now();

    std::printf("=== 摩擦辨识（SocketCAN）：批次 %s，%zu 段 ===\n", set_name.c_str(), segs.size());
    std::printf("接口: %s   ESC_ID=0x%03X  MST_ID=0x%03X  模式: MIT (kp=0, tau=0)\n",
                ifname.c_str(), dmsc::kEscId, dmsc::kMstId);
    std::printf("段表（斜坡 accel=%.1f rad/s²，到速后稳定 %.1fs 再开测量窗）:\n", accel, kSettleS);
    for (size_t i = 0; i < segs.size(); ++i) {
        const Seg& sg = segs[i];
        if (sg.type == SEG_COAST)
            std::printf("  #%2zu coast-down: kd=0 零力矩滑行至 |vel|<%.2f rad/s（超时 %.0fs）\n",
                        i, kCoastStopV, sg.meas_s);
        else
            std::printf("  #%2zu 平台 dq=%+6.3f rad/s (%+6.1f rpm)  kd=%.3f  堵转上限 %.2f Nm  测 %.0fs\n",
                        i, sg.dq, sg.dq * 9.5493f, sg.kd, sg.kd * std::fabs(sg.dq), sg.meas_s);
    }
    std::printf("预计总时长 ~%.0fs（兜底 %.0fs）   最大堵转力矩 %.2f Nm   超速阈 %.1f rad/s\n",
                plan_s, backstop, stall_max, vel_abort);
    std::printf("停止: Ctrl-C ×1 = 柔停   Ctrl-C ×2 = 立即失能\n");
    std::printf("*** 手握外壳，远离输出轴。各平台间会换向，外壳有反拧感。***\n\n");

    if (list_only) { std::printf("(--list：只打印段表，不发帧，结束)\n"); return 0; }

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    std::string errmsg;
    int sock = dmsc::open_can(ifname.c_str(), true, &errmsg);
    if (sock < 0) { std::printf("%s\n", errmsg.c_str()); return 1; }
    std::thread rx(rx_loop, sock);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // ---- 清除锁存故障 ----
    std::printf(">>> 清除错误 0xFB ×3 ...\n");
    burst(sock, 0xFB, 3, 120);
    int e0 = g_last_err.load();
    std::printf(">>> 清错后 ERR = %d (%s)\n", e0, e0 < 0 ? "无反馈" : dmsc::err_name((uint8_t)e0));
    if (e0 < 0) {
        std::printf("\n!! 清错无应答 —— 先跑 dm_probe_sc 排查通路（slcand 是否 -s5？）\n");
        g_rx_quit.store(true); rx.join(); ::close(sock);
        return 2;
    }
    if (e0 >= 0x8) {
        std::printf("\n!! 清错后仍有故障 %d(%s)，不使能。\n", e0, dmsc::err_name((uint8_t)e0));
        g_rx_quit.store(true); rx.join(); ::close(sock);
        return 2;
    }

    // ---- 使能后立即进控制循环（不等 ack，避免触发通讯看门狗）----
    std::printf("\n>>> 使能 0xFC ×3，立即进入控制循环\n");
    burst(sock, 0xFC, 3, 5);

    auto tstart = clk::now();
    auto next_print = tstart;
    int ticks = 0;
    bool aborted = false, confirmed = false, softstop = false;

    // 段执行器状态
    enum Phase { P_GRACE, P_RAMP, P_SETTLE, P_MEASURE, P_COAST, P_RAMP_DN, P_BRAKE };
    Phase ph = P_GRACE;
    int    seg_i = 0;
    double ph_t0 = 0.0;
    float  dq_cmd = 0.0f, kd_cur = 0.0f;
    float  kd_last_nz = 0.5f;                        // 柔停/降速用的兜底阻尼
    const float step = accel * (kPeriodMs / 1000.0f);

    // 平台测量累积
    std::vector<PlatRes> results;
    float  m_trav0 = 0; double m_t0 = 0;
    double m_sum_tau = 0, m_sum_tau2 = 0;
    int    m_n = 0, m_stick = 0;
    // coast 记录
    float  coast_v0 = 0; double coast_t0 = 0, coast_dur = -1;
    int    coast_still = 0;

    auto begin_seg = [&](int i, double el) {
        seg_i = i;
        g_seg.store(i);
        if (i >= (int)segs.size()) {                 // 段表跑完 → 收尾
            ph = P_RAMP_DN; ph_t0 = el;
            kd_cur = kd_last_nz;
            g_phase.store(0);
            return;
        }
        const Seg& sg = segs[i];
        if (sg.type == SEG_COAST) {
            ph = P_COAST; ph_t0 = el;
            kd_cur = 0.0f; dq_cmd = 0.0f;            // 零力矩：kd=0 ⇒ T=0，dq 无作用
            coast_v0 = g_vel.load(); coast_t0 = el; coast_still = 0;
            g_phase.store(2);
            std::printf(">>> [t=%.3fs] coast-down：kd=0 零力矩滑行（起始 %+.3f rad/s）\n",
                        el, coast_v0);
        } else {
            ph = P_RAMP; ph_t0 = el;
            kd_cur = dmsc::quantized(sg.kd, 0.0f, 5.0f, 12);
            kd_last_nz = kd_cur;
            g_phase.store(0);
        }
    };

    while (true) {
        double el = std::chrono::duration<double>(clk::now() - tstart).count();

        int sig = g_sig.load();
        if (sig >= 2) { std::printf("\n(Ctrl-C ×2，立即失能)\n"); break; }
        if (sig >= 1 && !softstop) {
            softstop = true;
            if (ph != P_RAMP_DN && ph != P_BRAKE) {
                std::printf("\n(Ctrl-C，柔停：斜坡减速 → 制动 → 失能)\n");
                if (kd_cur <= 0.0f) kd_cur = kd_last_nz;   // coast 中柔停也要有阻尼可用
                ph = P_RAMP_DN; ph_t0 = el;
                g_seg.store(-1); g_phase.store(0);
            }
        }

        if (ph == P_GRACE) {
            dq_cmd = 0.0f;
            if (el * 1000 > kGraceMs) {
                int e = g_last_err.load();
                if (e >= 0x8) {
                    std::printf("\n!!! t=%.3fs ERR=%d (%s)，停止 !!!\n", el, e, dmsc::err_name((uint8_t)e));
                    aborted = true; break;
                }
                if (e == 0x0) {
                    std::printf("\n!!! t=%.3fs 电机未使能 (ERR=0)，停止 !!!\n", el);
                    aborted = true; break;
                }
                if (e < 0) {
                    std::printf("\n!!! t=%.3fs 宽限期内无反馈帧，停止 !!!\n", el);
                    aborted = true; break;
                }
                confirmed = true;
                std::printf(">>> [t=%.3fs] 已确认使能 (ERR=1)，开始段 #0\n", el);
                begin_seg(0, el);
            }
        }

        if (confirmed) {
            // ---- 运行时保护 ----
            int e = g_last_err.load();
            if (e >= 0x8) {
                std::printf("\n!!! t=%.3fs ERR=%d (%s)，立即停止 !!!\n", el, e, dmsc::err_name((uint8_t)e));
                aborted = true; break;
            }
            if (e == 0x0) {
                std::printf("\n!!! t=%.3fs 电机自行退出使能 (ERR=0)，停止 !!!\n", el);
                aborted = true; break;
            }
            float v = g_vel.load();
            if (std::fabs(v) > vel_abort) {
                std::printf("\n!!! t=%.3fs 超速 vel=%+.3f > %.1f rad/s，立即停止 !!!\n", el, v, vel_abort);
                aborted = true; break;
            }
            float tq = g_tau.load();
            if (std::fabs(tq) > kTauAbort) {
                std::printf("\n!!! t=%.3fs 力矩异常 tau=%+.3f > %.1f Nm，立即停止 !!!\n", el, tq, kTauAbort);
                aborted = true; break;
            }
            if (g_tmos.load() > kTempAbort || g_trotor.load() > kTempAbort) {
                std::printf("\n!!! t=%.3fs 过温 MOS=%d℃ 线圈=%d℃ > %d℃，立即停止 !!!\n",
                            el, g_tmos.load(), g_trotor.load(), kTempAbort);
                aborted = true; break;
            }

            // ---- 斜坡限速器（COAST 段不动 dq_cmd，本来就是 0）----
            float tgt = 0.0f;
            if (ph == P_RAMP || ph == P_SETTLE || ph == P_MEASURE)
                tgt = dmsc::quantized(segs[seg_i].dq, -dmsc::kVMax, dmsc::kVMax, 12);
            if (ph != P_COAST) {
                float diff = tgt - dq_cmd;
                if      (diff >  step) dq_cmd += step;
                else if (diff < -step) dq_cmd -= step;
                else                   dq_cmd  = tgt;
            }
            bool reached = (dq_cmd == tgt);

            // ---- 段执行器推进 ----
            switch (ph) {
                case P_RAMP:
                    if (reached) { ph = P_SETTLE; ph_t0 = el; }
                    break;
                case P_SETTLE:
                    if (el - ph_t0 >= kSettleS) {
                        ph = P_MEASURE; ph_t0 = el;
                        g_phase.store(1);
                        m_trav0 = g_travel.load(); m_t0 = el;
                        m_sum_tau = m_sum_tau2 = 0; m_n = 0; m_stick = 0;
                    }
                    break;
                case P_MEASURE: {
                    float dir = segs[seg_i].dq >= 0 ? 1.0f : -1.0f;
                    m_sum_tau  += tq;
                    m_sum_tau2 += (double)tq * tq;
                    if (v * dir <= 0.0f) m_stick++;
                    m_n++;
                    if (el - ph_t0 >= segs[seg_i].meas_s) {
                        const Seg& sg = segs[seg_i];
                        double dur = el - m_t0;
                        double vm  = (g_travel.load() - m_trav0) / dur;
                        double tm  = m_n ? m_sum_tau / m_n : 0;
                        double tsd = m_n ? std::sqrt(std::max(0.0, m_sum_tau2 / m_n - tm * tm)) : 0;
                        PlatRes r;
                        r.tgt = sg.dq; r.kd = kd_cur;
                        r.v_meas   = (float)vm;
                        r.att_pct  = (float)(100.0 * std::fabs(vm) / std::fabs(dq_cmd));
                        r.fric_bs  = (float)(kd_cur * (std::fabs(dq_cmd) - std::fabs(vm)));
                        r.tau_mean = (float)tm;
                        r.tau_sd   = (float)tsd;
                        r.stick_pct = m_n ? 100.0f * m_stick / m_n : 0;
                        r.tmos = g_tmos.load(); r.trotor = g_trotor.load();
                        results.push_back(r);
                        std::printf(">>> [t=%.3fs] 平台 #%d dq=%+.3f: 实测 %+.4f rad/s (%.0f%%) "
                                    "摩擦: 反解 %.4f / 遥测 %+.4f±%.4f Nm  粘滑 %.0f%%  "
                                    "MOS=%d℃ 线圈=%d℃\n",
                                    el, seg_i, sg.dq, vm, r.att_pct, r.fric_bs,
                                    tm, tsd, r.stick_pct, r.tmos, r.trotor);
                        std::fflush(stdout);
                        begin_seg(seg_i + 1, el);
                    }
                    break;
                }
                case P_COAST: {
                    if (std::fabs(v) < kCoastStopV) coast_still++;
                    else coast_still = 0;
                    bool stopped = coast_still >= (int)(300 / kPeriodMs);   // 0.3s
                    if (stopped || el - ph_t0 >= segs[seg_i].meas_s) {
                        coast_dur = el - coast_t0;
                        std::printf(">>> [t=%.3fs] coast %s：%+.3f -> %+.3f rad/s，历时 %.2fs\n",
                                    el, stopped ? "停转" : "超时",
                                    coast_v0, v, coast_dur);
                        begin_seg(seg_i + 1, el);
                    }
                    break;
                }
                case P_RAMP_DN:
                    if (reached) {   // dq_cmd 已到 0
                        std::printf(">>> [t=%.3fs] 降速完成，阻尼制动 %.1fs\n", el, kBrakeS);
                        ph = P_BRAKE; ph_t0 = el;
                    }
                    break;
                default: break;
            }
            // 用 el-ph_t0 重算，不复用循环顶部的时间（dm_spin 制动段跳过 bug 的教训）
            if (ph == P_BRAKE && (el - ph_t0) >= kBrakeS) break;
        }

        // ---- 兜底 ----
        if (el > backstop) {
            std::printf("\n!!! t=%.3fs 总时长兜底触发，停止 !!!\n", el);
            aborted = true; break;
        }
        if (g_txfail.load() > 10) {
            std::printf("\n!!! t=%.3fs 发送失败 %d 次（接口异常），停止 !!!\n", el, g_txfail.load());
            aborted = true; break;
        }

        g_dq_cmd.store(dq_cmd);
        g_kd_cmd.store(kd_cur);
        send_mit_counted(sock, dq_cmd, kd_cur);
        ticks++;

        auto now = clk::now();
        if (now >= next_print) {
            next_print = now + std::chrono::milliseconds(500);
            int e = g_last_err.load();
            const char* phn = ph == P_GRACE ? "宽限" : ph == P_RAMP ? "斜坡" :
                              ph == P_SETTLE ? "稳定" : ph == P_MEASURE ? "测量" :
                              ph == P_COAST ? "滑行" : ph == P_RAMP_DN ? "降速" : "制动";
            std::printf("  t=%5.2fs 段#%d[%s] dq=%+.3f kd=%.3f | ERR=%d(%-4s) vel=%+7.3f "
                        "tau=%+6.3f MOS=%d℃ 线圈=%d℃ rx=%d\n",
                        el, seg_i, phn, dq_cmd, kd_cur,
                        e, e < 0 ? "无" : dmsc::err_name((uint8_t)e),
                        g_vel.load(), g_tau.load(), g_tmos.load(), g_trotor.load(), g_rx.load());
            std::fflush(stdout);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(kPeriodMs));
    }

    disable_motor(sock);
    g_rx_quit.store(true);
    rx.join();

    // ---- 统计 ----
    std::printf("\n=== 统计 ===\n");
    std::printf("MIT 帧: %d   总发送: %d (失败 %d)   收到反馈: %d   CAN 错误: %d\n",
                ticks, g_tx.load(), g_txfail.load(), g_rx.load(), g_canerr.load());
    if (g_tx.load() > 0)
        std::printf("应答率: %.1f%%\n", 100.0 * g_rx.load() / g_tx.load());
    std::printf("峰值 |vel|: %.3f rad/s   峰值 |tau|: %.3f Nm   累计行程: %+.3f rad\n",
                g_vel_max.load(), g_tau_max.load(), g_travel.load());

    if (!results.empty()) {
        std::printf("\n=== 平台汇总（%zu/%zu 完成）===\n", results.size(),
                    with_coast ? segs.size() - 1 : segs.size());
        std::printf("  目标rad/s    kd    实测rad/s  达成率  反解Nm   遥测Nm(±σ)      粘滑  温度℃\n");
        for (const PlatRes& r : results)
            std::printf("  %+7.3f  %.3f  %+9.4f  %5.1f%%  %.4f  %+.4f(±%.4f)  %3.0f%%  %d/%d\n",
                        r.tgt, r.kd, r.v_meas, r.att_pct, r.fric_bs,
                        r.tau_mean, r.tau_sd, r.stick_pct, r.tmos, r.trotor);
    }
    if (coast_dur > 0)
        std::printf("coast-down: %+.3f rad/s -> 停，历时 %.2fs（逐帧曲线在 CSV，phase=2）\n",
                    coast_v0, coast_dur);

    // ---- 逐帧日志 ----
    if (!csv_path.empty()) {
        int n = g_log_n.load();
        FILE* fp = std::fopen(csv_path.c_str(), "w");
        if (!fp) std::printf("\n!! 无法写入 %s\n", csv_path.c_str());
        else {
            std::fprintf(fp, "t_s,kts_ns,pos_rad,travel_rad,vel_rad_s,tau_Nm,dq_cmd,kd_cmd,"
                             "t_mos,t_rotor,seg,phase\n");
            for (int i = 0; i < n; ++i) {
                const Sample& sm = g_log[i];
                std::fprintf(fp, "%.6f,%llu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d,%d,%d\n",
                             sm.t, (unsigned long long)sm.kts, sm.pos, sm.trav,
                             sm.vel, sm.tau, sm.dq, sm.kd, sm.tmos, sm.trotor,
                             sm.seg, sm.phase);
            }
            std::fclose(fp);
            std::printf("\n逐帧日志: %d 行 -> %s\n", n, csv_path.c_str());
            if (n >= kLogMax) std::printf("!! 日志缓冲已满(%d)\n", kLogMax);
        }
    }

    ::close(sock);
    return aborted ? 2 : 0;
}
