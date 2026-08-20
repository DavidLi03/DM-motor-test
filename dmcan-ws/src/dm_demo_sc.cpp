// dm_demo_sc.cpp —— 力量/速度演示（手持）：MIT 纯阻尼中速旋转 + 正反换向
//
// 与 dm_spin_sc（方案 B'，0.5 rad/s 上限的测量程序）的关系：
//   控制律完全相同：kp=0, tau=0 ⇒ T = kd*(dq_des - dq)，纯速度阻尼。
//   两条安全性质依旧成立，且与 kd 取值无关：
//     · 速度自限：稳态转速天花板只由 dq_des 决定；
//     · 力矩自限：最大力矩 = kd*(dq_des - dq)，堵转时 = kd*dq_des。
//   差异：这是演示程序，速度上限放宽到 9 rad/s（输出轴 ≈86 rpm），
//   加入正→反换向段；所有指令变化都经过斜坡限速器（slew limiter），
//   任何时刻不存在速度阶跃。堵转力矩由 kd*dq<=4.0 Nm 联合钳位兜底，
//   速度越高允许的 kd 越低（高速下摩擦占比小，低 kd 也有高达成率）。
//
// 【手持演示的力学边界】—— 用户握持电机外壳时感到的反作用力矩：
//   巡航段          ≈ 摩擦力矩 ~0.10-0.15 Nm（几乎感觉不到）
//   加减速/换向段    ≈ kd * 跟踪误差，斜坡限速下 < ~0.5 Nm
//   堵转（输出轴被完全抓停）= kd*dq_des：默认 0.6*3.0 = 1.8 Nm
//   钳位联合上限 kd*dq <= 4.0 Nm —— 相当于用力拧罐头盖，成年人单手可控。
//   ⚠ 手只握【外壳】，绝不要碰旋转中的输出轴。
//
// 停止途径（全部收敛到 0xFD 失能）：
//   自然结束 / Ctrl-C×1 柔停（斜坡减速+阻尼制动）/ Ctrl-C×2 立即失能
//   / 超速 / 力矩异常 / 过温(70℃) / ERR 故障 / 反馈丢失 / 30s 总时长兜底
//
// 会发送：0xFB 清错 / 0xFC 使能 / 0xFD 失能 / MIT 帧
// 绝不发送：0x55 写参数 / 0xAA 存参数 / 0xFE 存零点
//
// 位置反馈在 ±12.5 rad 处回卷（多圈旋转必然跨越），本程序在接收侧做解卷绕，
// 行程统计用解卷绕后的累计值，不再有 dm_spin 的 ±1.5 rad 行程保护（多圈是本意）。
//
// 用法: dm_demo_sc [--if can0] [--dq 3.0] [--kd 0.6] [--seconds 3]
//                  [--accel 3.0] [--no-reverse] [--csv 路径]

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <string>
#include <thread>

#include "dm_sc.hpp"

namespace {

constexpr int kPeriodMs = 10;    // 100Hz，远高于 ~120ms 看门狗
constexpr int kGraceMs  = 150;

// ---- 安全钳位（上限，非默认值）----
constexpr float  kDqLimit   = 9.0f;   // 指令速度上限 rad/s（输出轴 ≈86 rpm）
constexpr float  kKdLimit   = 1.0f;   // 阻尼系数上限
constexpr float  kKdMin     = 0.2f;   // 下限：太小连摩擦都赢不了，演示会趴窝
constexpr double kSecLimit  = 5.0;    // 单方向巡航时长上限 s
constexpr float  kAccelMin  = 1.0f;   // 斜坡限速器加速度边界 rad/s^2
constexpr float  kAccelMax  = 12.0f;
constexpr float  kStallCap  = 4.0f;   // kd*dq 联合上限 Nm（堵转力矩天花板）
                                      // ⇒ 高速演示时 kd 被自动压低：dq=9 时 kd<=0.44
constexpr float  kVelMargin = 2.0f;   // 超速保护阈 = dq_des + 该裕量
constexpr float  kTauAbort  = 5.0f;   // 反馈力矩绝对值保护 Nm
constexpr int    kTempAbort = 70;     // MOS / 线圈温度保护 ℃
constexpr double kBrakeS    = 0.5;    // 降速到 0 后的主动阻尼制动时长 s
constexpr double kBackstopS = 30.0;   // 总时长兜底（正常全程 ~11s）

constexpr int kLogMax = 8000;         // 100Hz 下够 80 秒

using clk = std::chrono::steady_clock;
clk::time_point g_t0;
double now_s() { return std::chrono::duration<double>(clk::now() - g_t0).count(); }

std::atomic<int>  g_sig{0};
std::atomic<bool> g_rx_quit{false};
std::atomic<int>  g_rx{0}, g_tx{0}, g_txfail{0}, g_canerr{0};
std::atomic<int>  g_last_err{-1}, g_prev_err{-1};

std::atomic<float> g_pos{0}, g_vel{0}, g_tau{0};
std::atomic<float> g_travel{0};       // 解卷绕后的累计行程（相对首帧）
std::atomic<int>   g_tmos{0}, g_trotor{0};
std::atomic<float> g_vel_max{0}, g_tau_max{0};
std::atomic<float> g_dq_cmd{0}, g_kd_cmd{0};

struct Sample { float t, pos, trav, vel, tau, dq, kd; uint64_t kts; uint8_t tmos, trotor; };
Sample g_log[kLogMax];
std::atomic<int> g_log_n{0};

// 解卷绕状态 —— 只在 RX 线程访问
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

    // 位置解卷绕：±12.5 rad 处回卷。100Hz 下相邻帧真实位移 <0.05 rad，
    // 突变超过 PMAX 只可能是回卷，加/减 2*PMAX 修正。
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
                    g_dq_cmd.load(), g_kd_cmd.load(), kts, d[6], d[7]};
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

enum State { S_GRACE, S_RAMP_UP, S_CRUISE_F, S_REVERSE, S_CRUISE_R, S_RAMP_DN, S_BRAKE };
const char* state_name(State s) {
    switch (s) {
        case S_GRACE:    return "宽限";
        case S_RAMP_UP:  return "升速";
        case S_CRUISE_F: return "正转";
        case S_REVERSE:  return "换向";
        case S_CRUISE_R: return "反转";
        case S_RAMP_DN:  return "降速";
        default:         return "制动";
    }
}
// 各状态的 dq 目标；实际指令由斜坡限速器逼近，不会阶跃
float stage_target(State st, float dq) {
    switch (st) {
        case S_RAMP_UP: case S_CRUISE_F: return dq;
        case S_REVERSE: case S_CRUISE_R: return -dq;
        default: return 0.0f;
    }
}

}  // namespace

int main(int argc, char** argv) {
    std::string ifname = "can0";
    float  dq_tgt  = 3.0f;
    float  kd_v    = 0.6f;
    double seconds = 3.0;
    float  accel   = 3.0f;
    bool   do_rev  = true;
    std::string csv_path;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--if" && i + 1 < argc)           ifname  = argv[++i];
        else if (a == "--dq" && i + 1 < argc)      dq_tgt  = std::stof(argv[++i]);
        else if (a == "--kd" && i + 1 < argc)      kd_v    = std::stof(argv[++i]);
        else if (a == "--seconds" && i + 1 < argc) seconds = std::stod(argv[++i]);
        else if (a == "--accel" && i + 1 < argc)   accel   = std::stof(argv[++i]);
        else if (a == "--no-reverse")              do_rev  = false;
        else if (a == "--csv" && i + 1 < argc)     csv_path = argv[++i];
        else if (a == "--help") {
            std::printf(
                "用法: dm_demo_sc [--if can0] [--dq 3.0] [--kd 0.6] [--seconds 3]\n"
                "                 [--accel 3.0] [--no-reverse] [--csv 路径]\n"
                "手持力量/速度演示：升速 → 正转巡航 → 换向 → 反转巡航 → 降速 → 制动 → 失能\n"
                "钳位: |dq|<=%.1f rad/s  kd∈[%.1f,%.1f]  kd*dq<=%.1f Nm  seconds<=%.1f  accel∈[%.0f,%.0f]\n"
                "保护: 超速/|tau|>%.1fNm/温度>%d℃/ERR/反馈丢失/%.0fs兜底 ⇒ 立即失能\n"
                "Ctrl-C ×1 = 柔停（斜坡减速）   Ctrl-C ×2 = 立即失能\n",
                kDqLimit, kKdMin, kKdLimit, kStallCap, kSecLimit, kAccelMin, kAccelMax,
                kTauAbort, kTempAbort, kBackstopS);
            return 0;
        }
    }

    // ---- 钳位。超限时如实说明，不静默改数 ----
    dq_tgt = std::fabs(dq_tgt);
    if (dq_tgt > kDqLimit) { std::printf("[钳位] dq %.3f -> %.3f rad/s\n", dq_tgt, kDqLimit); dq_tgt = kDqLimit; }
    if (dq_tgt < 0.5f)     { dq_tgt = 0.5f; }
    if (kd_v > kKdLimit)   { std::printf("[钳位] kd %.3f -> %.3f\n", kd_v, kKdLimit); kd_v = kKdLimit; }
    if (kd_v < kKdMin)     { std::printf("[钳位] kd %.3f -> %.3f（太小挣不脱摩擦）\n", kd_v, kKdMin); kd_v = kKdMin; }
    if (kd_v * dq_tgt > kStallCap) {
        float kd_new = kStallCap / dq_tgt;
        std::printf("[钳位] kd*dq = %.2f Nm 超过堵转力矩上限 %.1f Nm，kd %.3f -> %.3f\n",
                    kd_v * dq_tgt, kStallCap, kd_v, kd_new);
        kd_v = kd_new;
    }
    if (seconds > kSecLimit) { std::printf("[钳位] seconds %.2f -> %.2f\n", seconds, kSecLimit); seconds = kSecLimit; }
    if (seconds < 0.5)       { seconds = 0.5; }
    if (accel > kAccelMax) { std::printf("[钳位] accel %.2f -> %.2f\n", accel, kAccelMax); accel = kAccelMax; }
    if (accel < kAccelMin) { std::printf("[钳位] accel %.2f -> %.2f\n", accel, kAccelMin); accel = kAccelMin; }
    if (dq_tgt / accel > 3.0f) {   // 斜坡不超过 3s，保证全程远小于总时长兜底
        float a_new = dq_tgt / 3.0f;
        std::printf("[钳位] accel %.2f -> %.2f（dq=%.1f 时斜坡须 <=3s）\n", accel, a_new, dq_tgt);
        accel = a_new;
    }

    const float dq_q = dmsc::quantized(dq_tgt, -dmsc::kVMax, dmsc::kVMax, 12);
    const float kd_q = dmsc::quantized(kd_v, 0.0f, 5.0f, 12);
    const float vel_abort = dq_tgt + kVelMargin;
    const double ramp_s = dq_tgt / accel;

    g_t0 = clk::now();

    std::printf("=== 手持演示（SocketCAN）：MIT 纯阻尼中速旋转%s ===\n",
                do_rev ? " + 正反换向" : "");
    std::printf("接口: %s   ESC_ID=0x%03X  MST_ID=0x%03X  模式: MIT (kp=0, tau=0)\n",
                ifname.c_str(), dmsc::kEscId, dmsc::kMstId);
    std::printf("目标: dq=%.3f rad/s (量化 %.4f) ≈ %.1f rpm @输出轴   kd=%.3f (量化 %.4f)\n",
                dq_tgt, dq_q, dq_tgt * 9.5493, kd_v, kd_q);
    std::printf("力矩边界: 堵转(抓停输出轴)时 = kd*dq = %.2f Nm（TMAX=%.0f 的 %.0f%%）；巡航段仅摩擦 ~0.1 Nm\n",
                kd_q * dq_q, dmsc::kTMax, 100.0 * kd_q * dq_q / dmsc::kTMax);
    std::printf("时序: 使能 → 宽限%dms → 升速%.1fs → 正转%.1fs%s → 降速%.1fs → 制动%.1fs → 失能（全程 ~%.0fs）\n",
                kGraceMs, ramp_s, seconds,
                do_rev ? " → 换向 → 反转" : "", ramp_s, kBrakeS,
                kGraceMs / 1000.0 + ramp_s * 2 + seconds * (do_rev ? 2 : 1) +
                (do_rev ? 2 * ramp_s : 0) + kBrakeS + 0.5);
    std::printf("保护: |vel|>%.1f rad/s | |tau|>%.1f Nm | 温度>%d℃ | ERR | 反馈丢失 | %.0fs兜底\n",
                vel_abort, kTauAbort, kTempAbort, kBackstopS);
    std::printf("停止: Ctrl-C ×1 = 柔停（~%.1fs 内减速停止）   Ctrl-C ×2 = 立即失能\n", ramp_s + kBrakeS);
    std::printf("*** 手握外壳，远离输出轴。电机会先正转再反转，换向时外壳会明显反拧。***\n\n");

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    std::string errmsg;
    int sock = dmsc::open_can(ifname.c_str(), true, &errmsg);
    if (sock < 0) { std::printf("%s\n", errmsg.c_str()); return 1; }
    std::thread rx(rx_loop, sock);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // ---- 阶段 1：清除锁存故障 ----
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

    // ---- 阶段 2：使能后立即进控制循环（不等 ack，避免触发通讯看门狗）----
    std::printf("\n>>> 使能 0xFC ×3，立即进入控制循环\n");
    burst(sock, 0xFC, 3, 5);

    auto tstart = clk::now();
    auto next_print = tstart;
    int ticks = 0;
    bool aborted = false, confirmed = false, softstop = false;

    State st = S_GRACE;
    double st_t0 = 0.0;
    float dq_cmd = 0.0f;
    const float step = accel * (kPeriodMs / 1000.0f);   // 斜坡限速器单步

    // 巡航段统计（解卷绕行程 + 时长 → 位移法转速）
    float  trav_f0 = 0, trav_f1 = 0, trav_r0 = 0, trav_r1 = 0;
    double dur_f = 0, dur_r = 0;

    while (true) {
        double el = std::chrono::duration<double>(clk::now() - tstart).count();
        double ts = el - st_t0;

        // Ctrl-C：×1 柔停，×2 立即失能
        int sig = g_sig.load();
        if (sig >= 2) { std::printf("\n(Ctrl-C ×2，立即失能)\n"); break; }
        if (sig >= 1 && !softstop) {
            softstop = true;
            if (st != S_RAMP_DN && st != S_BRAKE) {
                std::printf("\n(Ctrl-C，柔停：斜坡减速 → 制动 → 失能)\n");
                st = S_RAMP_DN; st_t0 = el; ts = 0.0;
            }
        }

        if (st == S_GRACE) {
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
                std::printf(">>> [t=%.3fs] 已确认使能 (ERR=1)，开始升速\n", el);
                st = S_RAMP_UP; st_t0 = el; ts = 0.0;
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

            // ---- 斜坡限速器：dq_cmd 以 accel 逼近本状态目标，永不阶跃 ----
            float tgt = stage_target(st, dq_q);
            float diff = tgt - dq_cmd;
            if      (diff >  step) dq_cmd += step;
            else if (diff < -step) dq_cmd -= step;
            else                   dq_cmd  = tgt;
            bool reached = (dq_cmd == tgt);

            // ---- 状态推进 ----
            switch (st) {
                case S_RAMP_UP:
                    if (reached) {
                        trav_f0 = g_travel.load();
                        std::printf(">>> [t=%.3fs] 达到 %+.3f rad/s，正转巡航 %.1fs\n", el, dq_cmd, seconds);
                        st = S_CRUISE_F; st_t0 = el;
                    }
                    break;
                case S_CRUISE_F:
                    if (ts >= seconds) {
                        trav_f1 = g_travel.load(); dur_f = ts;
                        if (do_rev) {
                            std::printf(">>> [t=%.3fs] 换向：%+.3f -> %+.3f rad/s（注意外壳反拧感）\n",
                                        el, dq_cmd, -dq_q);
                            st = S_REVERSE; st_t0 = el;
                        } else {
                            st = S_RAMP_DN; st_t0 = el;
                        }
                    }
                    break;
                case S_REVERSE:
                    if (reached) {
                        trav_r0 = g_travel.load();
                        std::printf(">>> [t=%.3fs] 达到 %+.3f rad/s，反转巡航 %.1fs\n", el, dq_cmd, seconds);
                        st = S_CRUISE_R; st_t0 = el;
                    }
                    break;
                case S_CRUISE_R:
                    if (ts >= seconds) {
                        trav_r1 = g_travel.load(); dur_r = ts;
                        st = S_RAMP_DN; st_t0 = el;
                    }
                    break;
                case S_RAMP_DN:
                    if (reached) {   // dq_cmd 已到 0
                        std::printf(">>> [t=%.3fs] 降速完成，阻尼制动 %.1fs\n", el, kBrakeS);
                        st = S_BRAKE; st_t0 = el;
                    }
                    break;
                case S_BRAKE:
                    break;
                default: break;
            }
            // 用 el-st_t0 重算，不复用循环顶部的 ts（dm_spin 的制动段跳过 bug，教训）
            if (st == S_BRAKE && (el - st_t0) >= kBrakeS) break;
        }

        // ---- 兜底 ----
        if (el > kBackstopS) {
            std::printf("\n!!! t=%.3fs 总时长兜底触发，停止 !!!\n", el);
            aborted = true; break;
        }
        if (g_txfail.load() > 10) {
            std::printf("\n!!! t=%.3fs 发送失败 %d 次（接口异常），停止 !!!\n", el, g_txfail.load());
            aborted = true; break;
        }

        g_dq_cmd.store(dq_cmd);
        g_kd_cmd.store(kd_q);
        send_mit_counted(sock, dq_cmd, kd_q);
        ticks++;

        auto now = clk::now();
        if (now >= next_print) {
            next_print = now + std::chrono::milliseconds(200);
            int e = g_last_err.load();
            std::printf("  t=%5.2fs [%s] dq=%+.3f | ERR=%d(%-8s) vel=%+6.3f rad/s (%+5.1f rpm) "
                        "tau=%+6.3f 行程=%+8.3f rad MOS=%d℃ 线圈=%d℃ rx=%d\n",
                        el, state_name(st), dq_cmd,
                        e, e < 0 ? "无反馈" : dmsc::err_name((uint8_t)e),
                        g_vel.load(), g_vel.load() * 9.5493f, g_tau.load(),
                        g_travel.load(), g_tmos.load(), g_trotor.load(), g_rx.load());
            std::fflush(stdout);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(kPeriodMs));
    }

    disable_motor(sock);
    g_rx_quit.store(true);
    rx.join();

    // ---- 统计 ----
    float trav = g_travel.load();
    std::printf("\n=== 统计 ===\n");
    std::printf("MIT 帧: %d   总发送: %d (失败 %d)   收到反馈: %d   CAN 错误: %d\n",
                ticks, g_tx.load(), g_txfail.load(), g_rx.load(), g_canerr.load());
    if (g_tx.load() > 0)
        std::printf("应答率: %.1f%%\n", 100.0 * g_rx.load() / g_tx.load());
    std::printf("累计行程: %+.3f rad = %+.1f° = %.2f 圈（解卷绕）\n",
                trav, trav * 57.2958f, std::fabs(trav) / 6.28319f);
    std::printf("峰值 |vel|: %.3f rad/s (%.1f rpm)   峰值 |tau|: %.3f Nm\n",
                g_vel_max.load(), g_vel_max.load() * 9.5493f, g_tau_max.load());

    auto report_cruise = [&](const char* name, float t0, float t1, double dur, float sign) {
        if (dur <= 0) return;
        double v = (t1 - t0) / dur;
        std::printf("%s巡航 %.2fs: 位移法转速 %+.4f rad/s（指令 %+.4f，达成率 %.0f%%）"
                    " ⇒ 反解摩擦 %.4f Nm\n",
                    name, dur, v, sign * dq_q, 100.0 * std::fabs(v) / dq_q,
                    kd_q * (dq_q - std::fabs(v)));
    };
    std::printf("\n=== 巡航段 ===\n");
    report_cruise("正转", trav_f0, trav_f1, dur_f, +1.0f);
    report_cruise("反转", trav_r0, trav_r1, dur_r, -1.0f);
    if (dur_f > 0)
        std::printf("(对照: 0.17 rad/s 低速工况反解摩擦 0.123-0.128 Nm；此处速度高一个量级，\n"
                    " 若摩擦更低即再添一个 Stribeck 曲线数据点)\n");

    // ---- 逐帧日志 ----
    if (!csv_path.empty()) {
        int n = g_log_n.load();
        FILE* fp = std::fopen(csv_path.c_str(), "w");
        if (!fp) std::printf("\n!! 无法写入 %s\n", csv_path.c_str());
        else {
            std::fprintf(fp, "t_s,kts_ns,pos_rad,travel_rad,vel_rad_s,tau_Nm,dq_cmd,kd_cmd,"
                             "t_mos,t_rotor\n");
            for (int i = 0; i < n; ++i) {
                const Sample& sm = g_log[i];
                std::fprintf(fp, "%.6f,%llu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d\n",
                             sm.t, (unsigned long long)sm.kts, sm.pos, sm.trav,
                             sm.vel, sm.tau, sm.dq, sm.kd, sm.tmos, sm.trotor);
            }
            std::fclose(fp);
            std::printf("\n逐帧日志: %d 行 -> %s\n", n, csv_path.c_str());
            if (n >= kLogMax) std::printf("!! 日志缓冲已满(%d)\n", kLogMax);
        }
    }

    ::close(sock);
    return aborted ? 2 : 0;
}
