// dm_stir_sc.cpp —— 行星减速箱润滑脂搅拌（跑合）：多循环双向速度阶梯
//
// 目的：厂商标定工具做参数辨识前，先让减速箱润滑脂充分搅拌/升温均布，
//       降低「冷脂高粘」对辨识的污染。顺带逐段记录摩擦快照——搅拌是否
//       起效，直接看同速摩擦随循环数/温度的下降即可。
//
// 控制律与 dm_fric_sc 完全同源：MIT 纯阻尼（kp=0, tau=0 ⇒ T = kd*(dq_des - dq)）。
//   · 速度天花板只由 dq_des 决定，kd 不提速；
//   · 最大力矩 = 堵转时的 kd*dq_des，联合钳位 kd*dq <= 4.0 Nm。
//
// 段表：把速度阶梯（默认 1.0 2.0 3.0 4.0 rad/s，每速度 +v 再 -v）循环重复，
// 直到覆盖 --minutes 指定的总时长。所有速度切换（含过零换向）都走斜坡限速器。
//
// 本机（DM-J4340P-2EC，Gr=40）已实测：0.22 rad/s 时摩擦 ~0.41 Nm ⇒ kd=1.5 下
// 各阶梯速度的稳态速度误差约 0.3~0.6 rad/s，属预期，不影响搅拌目的。
//
// 会发送：0xFB 清错 / 0xFC 使能 / 0xFD 失能 / MIT 帧
// 绝不发送：0x55 写参数 / 0xAA 存参数 / 0xFE 存零点
//
// 停止途径（全部收敛到 0xFD 失能）：自然结束 / Ctrl-C×1 柔停 / Ctrl-C×2 立即失能
//   / 超速 / 力矩异常 / 过温(70℃) / ERR 故障 / 反馈丢失 / 计划时长+20s 兜底
//
// 用法: dm_stir_sc [--minutes 3] [--speeds 1.0,2.0,3.0,4.0] [--dwell 3]
//                  [--accel 3] [--if can0] [--csv 路径] [--list]
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
constexpr float  kDqLimit    = 6.0f;   // 搅拌速度上限 rad/s（≈57 rpm 输出轴，VMAX=20 的 30%）
constexpr float  kKdLimit    = 1.5f;
constexpr float  kStallCap   = 4.0f;   // kd*dq 联合上限 Nm（堵转力矩天花板）
constexpr float  kAccelMin   = 0.5f;
constexpr float  kAccelMax   = 8.0f;
constexpr float  kVelMargin  = 2.0f;
constexpr float  kTauAbort   = 5.0f;
constexpr int    kTempAbort  = 70;
constexpr double kSettleS    = 0.5;    // 到速后稳定期，其后算作该段的摩擦快照窗
constexpr double kBrakeS     = 0.5;
constexpr double kMinutesMax = 10.0;   // 单次运行时长上限
constexpr double kDwellMin   = 1.5, kDwellMax = 8.0;
constexpr int    kMaxSpeeds  = 6;
constexpr int    kMaxSegs    = 500;

constexpr int kLogMax = 64000;         // 100Hz 下够 640 秒（10 分钟上限 + 裕量）

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
std::atomic<int>   g_seg{-1};
std::atomic<int>   g_phase{0};         // 0=斜坡/稳定 1=快照窗

struct Sample {
    float t, pos, trav, vel, tau, dq, kd;
    uint64_t kts;
    uint8_t tmos, trotor;
    int16_t seg;                       // 循环搅拌段数可超 127，用 16 位
    int8_t  phase;
};
Sample g_log[kLogMax];
std::atomic<int> g_log_n{0};

// 解卷绕状态 —— 只在 RX 线程访问（±12.5 rad 回卷，同 dm_fric_sc）
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
                    (int16_t)g_seg.load(), (int8_t)g_phase.load()};
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
struct Seg {
    float  dq;         // 目标速度（带符号）
    float  kd;
    double dwell_s;    // 到速+稳定之后在该速度停留的时长（含快照窗）
    int    cycle;      // 属于第几个循环（0 起）
    int    ladder_i;   // 阶梯内序号（同速正反共用，用于首末循环对比）
};

// 该速度允许的最大 kd：min(kKdLimit, kStallCap/|v|)
float kd_for(float v) {
    float kd = kKdLimit;
    if (kd * v > kStallCap) kd = kStallCap / v;
    return kd;
}

// 计划时长估计（斜坡 + 稳定 + 停留，再加收尾）
double plan_seconds(const std::vector<Seg>& segs, float accel) {
    double t = kGraceMs / 1000.0;
    float dq_prev = 0.0f;
    for (const Seg& sg : segs) {
        t += std::fabs(sg.dq - dq_prev) / accel + kSettleS + sg.dwell_s;
        dq_prev = sg.dq;
    }
    t += std::fabs(dq_prev) / accel + kBrakeS + 0.5;
    return t;
}

// 每段的摩擦快照（即时打印 + 首末循环对比）
struct SnapRes {
    int   cycle, ladder_i;
    float tgt, v_meas, fric_bs, tau_mean;
    int   tmos, trotor;
};

}  // namespace

int main(int argc, char** argv) {
    std::string ifname = "can0";
    std::string csv_path;
    std::vector<float> ladder = {1.0f, 2.0f, 3.0f, 4.0f};
    double minutes = 3.0;
    double dwell   = 3.0;
    float  accel   = 3.0f;
    bool   list_only = false;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--if" && i + 1 < argc)           ifname = argv[++i];
        else if (a == "--minutes" && i + 1 < argc) minutes = std::stod(argv[++i]);
        else if (a == "--dwell" && i + 1 < argc)   dwell = std::stod(argv[++i]);
        else if (a == "--accel" && i + 1 < argc)   accel = std::stof(argv[++i]);
        else if (a == "--csv" && i + 1 < argc)     csv_path = argv[++i];
        else if (a == "--list")                    list_only = true;
        else if (a == "--speeds" && i + 1 < argc) {
            ladder.clear();
            std::string sp = argv[++i];
            size_t p = 0;
            while (p < sp.size()) {
                size_t q = sp.find(',', p);
                if (q == std::string::npos) q = sp.size();
                ladder.push_back(std::stof(sp.substr(p, q - p)));
                p = q + 1;
            }
        } else if (a == "--help") {
            std::printf(
                "用法: dm_stir_sc [--minutes 3] [--speeds 1.0,2.0,3.0,4.0] [--dwell 3]\n"
                "                 [--accel 3] [--if can0] [--csv 路径] [--list]\n"
                "润滑脂搅拌：速度阶梯（每速度 +/- 双向）循环重复，直到覆盖 --minutes。\n"
                "钳位: |dq|<=%.1f rad/s  kd<=%.1f 且 kd*dq<=%.1f Nm  minutes<=%.0f\n"
                "      dwell∈[%.1f,%.1f]s  accel∈[%.1f,%.0f] rad/s²\n"
                "保护: 超速/|tau|>%.1fNm/温度>%d℃/ERR/反馈丢失/计划时长+20s兜底 ⇒ 立即失能\n"
                "Ctrl-C ×1 = 柔停（斜坡减速）   Ctrl-C ×2 = 立即失能\n"
                "--list 只打印段表不发帧（dry check）\n",
                kDqLimit, kKdLimit, kStallCap, kMinutesMax,
                kDwellMin, kDwellMax, kAccelMin, kAccelMax, kTauAbort, kTempAbort);
            return 0;
        }
    }

    // ---- 参数钳位 ----
    if (minutes > kMinutesMax) { std::printf("[钳位] minutes %.1f -> %.1f\n", minutes, kMinutesMax); minutes = kMinutesMax; }
    if (minutes < 0.5)         { std::printf("[钳位] minutes %.1f -> 0.5\n", minutes); minutes = 0.5; }
    if (dwell > kDwellMax)     { std::printf("[钳位] dwell %.1f -> %.1f\n", dwell, kDwellMax); dwell = kDwellMax; }
    if (dwell < kDwellMin)     { std::printf("[钳位] dwell %.1f -> %.1f\n", dwell, kDwellMin); dwell = kDwellMin; }
    if (accel > kAccelMax)     { std::printf("[钳位] accel %.2f -> %.2f\n", accel, kAccelMax); accel = kAccelMax; }
    if (accel < kAccelMin)     { std::printf("[钳位] accel %.2f -> %.2f\n", accel, kAccelMin); accel = kAccelMin; }
    if ((int)ladder.size() > kMaxSpeeds) {
        std::printf("[钳位] 速度点 %zu 个 -> 前 %d 个\n", ladder.size(), kMaxSpeeds);
        ladder.resize(kMaxSpeeds);
    }
    if (ladder.empty()) { std::printf("--speeds 至少要 1 个速度点\n"); return 1; }
    for (float& v : ladder) {
        v = std::fabs(v);
        if (v > kDqLimit) { std::printf("[钳位] 速度 %.3f -> %.3f rad/s\n", v, kDqLimit); v = kDqLimit; }
        if (v < 0.2f)     { std::printf("[钳位] 速度 %.3f -> 0.2 rad/s\n", v); v = 0.2f; }
    }

    // ---- 组段表：阶梯循环直到覆盖总时长 ----
    std::vector<Seg> segs;
    {
        // 单循环时长估算，决定循环数
        std::vector<Seg> one;
        for (size_t li = 0; li < ladder.size(); ++li) {
            float v = ladder[li], kd = kd_for(v);
            one.push_back({+v, kd, dwell, 0, (int)li});
            one.push_back({-v, kd, dwell, 0, (int)li});
        }
        double cyc_s = plan_seconds(one, accel) - kGraceMs / 1000.0 - kBrakeS - 0.5;
        int n_cyc = (int)std::ceil(minutes * 60.0 / cyc_s);
        if (n_cyc < 1) n_cyc = 1;
        for (int c = 0; c < n_cyc && (int)segs.size() + (int)one.size() <= kMaxSegs; ++c)
            for (Seg sg : one) { sg.cycle = c; segs.push_back(sg); }
    }
    const int n_cycles = segs.back().cycle + 1;

    // 全程最大速度 → 超速保护阈；最大堵转力矩 → 打印
    float v_top = 0.0f, stall_max = 0.0f;
    for (const Seg& sg : segs) {
        if (std::fabs(sg.dq) > v_top) v_top = std::fabs(sg.dq);
        if (sg.kd * std::fabs(sg.dq) > stall_max) stall_max = sg.kd * std::fabs(sg.dq);
    }
    const float  vel_abort = v_top + kVelMargin;
    const double plan_s    = plan_seconds(segs, accel);
    const double backstop  = plan_s + 20.0;

    g_t0 = clk::now();

    std::printf("=== 润滑脂搅拌（SocketCAN）：%d 循环 × %zu 段/循环 = %zu 段 ===\n",
                n_cycles, segs.size() / n_cycles, segs.size());
    std::printf("接口: %s   ESC_ID=0x%03X  MST_ID=0x%03X  模式: MIT (kp=0, tau=0)\n",
                ifname.c_str(), dmsc::kEscId, dmsc::kMstId);
    std::printf("阶梯（每速度 +/- 双向，dwell %.1fs，accel %.1f rad/s²）:\n", dwell, accel);
    for (float v : ladder)
        std::printf("  %5.2f rad/s (%5.1f rpm)  kd=%.3f  堵转上限 %.2f Nm\n",
                    v, v * 9.5493f, kd_for(v), kd_for(v) * v);
    std::printf("预计总时长 ~%.0fs (~%.1f 分钟，兜底 %.0fs)   最大堵转力矩 %.2f Nm   超速阈 %.1f rad/s\n",
                plan_s, plan_s / 60.0, backstop, stall_max, vel_abort);
    std::printf("停止: Ctrl-C ×1 = 柔停   Ctrl-C ×2 = 立即失能\n");
    std::printf("*** 电机未固定：按住外壳或压紧，远离输出轴；换向时外壳有反拧感。***\n\n");

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
        std::printf("\n!! 清错无应答 —— 先跑 dm_probe_sc 排查通路\n");
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

    enum Phase { P_GRACE, P_RAMP, P_SETTLE, P_DWELL, P_RAMP_DN, P_BRAKE };
    Phase ph = P_GRACE;
    int    seg_i = 0;
    double ph_t0 = 0.0;
    float  dq_cmd = 0.0f, kd_cur = 0.0f;
    float  kd_last_nz = 0.5f;
    const float step = accel * (kPeriodMs / 1000.0f);

    std::vector<SnapRes> snaps;
    float  m_trav0 = 0; double m_t0 = 0;
    double m_sum_tau = 0; int m_n = 0;

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
        ph = P_RAMP; ph_t0 = el;
        kd_cur = dmsc::quantized(sg.kd, 0.0f, 5.0f, 12);
        kd_last_nz = kd_cur;
        g_phase.store(0);
    };

    while (true) {
        double el = std::chrono::duration<double>(clk::now() - tstart).count();

        int sig = g_sig.load();
        if (sig >= 2) { std::printf("\n(Ctrl-C ×2，立即失能)\n"); break; }
        if (sig >= 1 && !softstop) {
            softstop = true;
            if (ph != P_RAMP_DN && ph != P_BRAKE) {
                std::printf("\n(Ctrl-C，柔停：斜坡减速 → 制动 → 失能)\n");
                if (kd_cur <= 0.0f) kd_cur = kd_last_nz;
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
                std::printf(">>> [t=%.3fs] 已确认使能 (ERR=1)，开始循环 #0\n", el);
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

            // ---- 斜坡限速器 ----
            float tgt = 0.0f;
            if (ph == P_RAMP || ph == P_SETTLE || ph == P_DWELL)
                tgt = dmsc::quantized(segs[seg_i].dq, -dmsc::kVMax, dmsc::kVMax, 12);
            float diff = tgt - dq_cmd;
            if      (diff >  step) dq_cmd += step;
            else if (diff < -step) dq_cmd -= step;
            else                   dq_cmd  = tgt;
            bool reached = (dq_cmd == tgt);

            // ---- 段执行器推进 ----
            switch (ph) {
                case P_RAMP:
                    if (reached) { ph = P_SETTLE; ph_t0 = el; }
                    break;
                case P_SETTLE:
                    if (el - ph_t0 >= kSettleS) {
                        ph = P_DWELL; ph_t0 = el;
                        g_phase.store(1);
                        m_trav0 = g_travel.load(); m_t0 = el;
                        m_sum_tau = 0; m_n = 0;
                    }
                    break;
                case P_DWELL: {
                    m_sum_tau += tq;
                    m_n++;
                    if (el - ph_t0 >= segs[seg_i].dwell_s) {
                        const Seg& sg = segs[seg_i];
                        double dur = el - m_t0;
                        double vm  = (g_travel.load() - m_trav0) / dur;
                        double tm  = m_n ? m_sum_tau / m_n : 0;
                        SnapRes r;
                        r.cycle = sg.cycle; r.ladder_i = sg.ladder_i;
                        r.tgt = sg.dq;
                        r.v_meas   = (float)vm;
                        r.fric_bs  = (float)(kd_cur * (std::fabs(dq_cmd) - std::fabs(vm)));
                        r.tau_mean = (float)tm;
                        r.tmos = g_tmos.load(); r.trotor = g_trotor.load();
                        snaps.push_back(r);
                        std::printf(">>> 循环 %d/%d dq=%+5.2f: 实测 %+6.3f rad/s  摩擦 反解 %.3f / "
                                    "遥测 %+6.3f Nm  MOS=%d℃ 线圈=%d℃\n",
                                    sg.cycle + 1, n_cycles, sg.dq, vm, r.fric_bs, tm,
                                    r.tmos, r.trotor);
                        std::fflush(stdout);
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
            next_print = now + std::chrono::milliseconds(1000);
            int e = g_last_err.load();
            const char* phn = ph == P_GRACE ? "宽限" : ph == P_RAMP ? "斜坡" :
                              ph == P_SETTLE ? "稳定" : ph == P_DWELL ? "停留" :
                              ph == P_RAMP_DN ? "降速" : "制动";
            std::printf("  t=%6.1fs 段#%d[%s] dq=%+.2f kd=%.2f | ERR=%d(%-4s) vel=%+6.3f "
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

    // ---- 搅拌效果：首循环 vs 末循环 同速摩擦对比 ----
    if (!snaps.empty()) {
        int c_last = 0;
        for (const SnapRes& r : snaps) if (r.cycle > c_last) c_last = r.cycle;
        std::printf("\n=== 搅拌效果（首循环 vs 末循环，正反向均值）===\n");
        std::printf("  速度rad/s | 首循环摩擦Nm | 末循环摩擦Nm | 变化    | 末循环温度℃\n");
        for (int li = 0; li < (int)ladder.size(); ++li) {
            double f0 = 0, f1 = 0; int n0 = 0, n1 = 0, tm1 = 0, tr1 = 0;
            for (const SnapRes& r : snaps) {
                if (r.ladder_i != li) continue;
                if (r.cycle == 0)      { f0 += r.fric_bs; n0++; }
                if (r.cycle == c_last) { f1 += r.fric_bs; n1++; tm1 = r.tmos; tr1 = r.trotor; }
            }
            if (n0 && n1) {
                f0 /= n0; f1 /= n1;
                std::printf("  %8.2f  | %11.3f | %11.3f | %+6.1f%% | %d/%d\n",
                            ladder[li], f0, f1, f0 != 0 ? 100.0 * (f1 - f0) / f0 : 0.0, tm1, tr1);
            }
        }
        std::printf("（完成 %zu/%zu 段；逐段快照在 CSV phase=1 部分）\n", snaps.size(), segs.size());
    }

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
