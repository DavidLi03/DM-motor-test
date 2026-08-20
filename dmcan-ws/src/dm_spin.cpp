// dm_spin.cpp —— 方案 B'：清错 → 使能 → MIT 纯阻尼低速旋转 → 软停 → 失能
//
// 两种模式：
//   默认      固定 kd，恒速指令，跑一段固定时长。
//   --sweep   kd 从 kd_start 线性升到 kd_max，dq_des 锁死不动，
//             测出电机挣脱静摩擦那一刻的 kd ⇒ 反推减速箱静摩擦力矩。
//
// 控制律（MIT）：T = kp*(q_des-q) + kd*(dq_des-dq) + tau
//   本程序 kp = 0、tau = 0  ⇒  T = kd * (dq_des - dq)，退化为纯速度阻尼。
//   两个安全性质由此而来，且【与 kd 取值无关】：
//     · 速度自限：转速一旦超过 dq_des，力矩反号。速度天花板只由 dq_des 决定，
//       提高 kd 不会提速 —— 这就是 sweep 扫 kd 而不扫 dq 的原因。
//     · 力矩自限：最大力矩只出现在堵转瞬间，等于 kd * dq_des。一旦挣脱，
//       速度误差项塌缩，力矩自己就掉到「克服动摩擦所需」的水平。
//
// 安全设计（针对「电机未固定，摆在桌面上」）：
//   1. kp / tau 硬编码为 0，没有任何命令行参数可以改。
//   2. dq_des 上限 0.5 rad/s（速度天花板，安全关键），kd 上限 1.5（力矩权限，非安全关键）。
//      超出即钳位，实际生效值会打印出来。
//   3. 三重运行时保护，任一触发立刻失能退出：
//        · 超速   |vel| > 1.0 rad/s
//        · 超程   |pos - pos0| > 1.5 rad（远大于指令行程，防解码/符号错误）
//        · 故障   ERR >= 8，或使能后掉回 ERR == 0，或收不到反馈帧
//   4. 起停都走斜坡（各 0.3 秒），不做阶跃。
//   5. 正常结束 / 保护触发 / Ctrl-C 三条路径都会发 0xFD 失能。
//      kd 偏大时受 100Hz 周期 + 20ms 反馈延迟影响可能出现抖振啸叫，Ctrl-C 即停。
//
// 会发送的命令：0xFB 清错 / 0xFC 使能 / 0xFD 失能 / MIT 帧
// 绝不发送：0x55 写参数 / 0xAA 存参数 / 0xFE 存零点
//
// 时序约束见 ../PROGRESS-2026-08-02.md 第 4.5 节（通讯丢失看门狗）：
// 发完 0xFC 必须立即开始周期性发帧，且不能先等 ack —— 等 ack 期间的总线静默本身就会超时。

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <string>
#include <thread>

#include "dmcan.h"

namespace {

// ---- 本机实测参数（dm_probe 读回，见 PROGRESS 第 3.4 节）----
constexpr uint16_t kEscId  = 0x008;   // 0x08 ESC_ID  电机接收 ID
constexpr uint16_t kMstId  = 0x018;   // 0x07 MST_ID  电机反馈 ID
constexpr uint16_t kMitOff = 0x000;   // CTRL_MODE=1(MIT) ⇒ 控制帧发往 ESC_ID+0
constexpr float kPMax = 12.5f;        // 0x15 PMAX
constexpr float kVMax = 50.0f;        // 0x16 VMAX
constexpr float kTMax = 10.0f;        // 0x17 TMAX
constexpr float kGr   = 10.0f;        // 0x14 Gr 减速比（仅用于把输出端力矩折算到转子端）

constexpr uint32_t kCanBaud  = 1000000;  // 1Mbps ⇒ 经典 CAN 2.0B
constexpr int      kPeriodMs = 10;       // 100Hz
constexpr int      kGraceMs  = 150;      // 实测收帧延迟 16~22ms，取 150ms 留裕量

// ---- 安全上限（钳位，非默认值）----
constexpr float  kDqLimit  = 0.5f;   // 指令速度上限 rad/s —— 安全关键，决定转速天花板
constexpr float  kKdLimit  = 1.5f;   // 阻尼系数上限 —— 只决定力矩权限，不影响转速天花板
constexpr double kSecLimit = 3.0;    // 固定模式匀速段时长上限 s
constexpr float  kVelAbort = 1.0f;   // 超速保护阈值 rad/s
constexpr float  kPosAbort = 1.5f;   // 超程保护阈值 rad
constexpr double kRampS    = 0.3;    // 起停斜坡时长 s
constexpr double kBrakeS   = 0.3;    // 停止后主动阻尼制动时长 s

// ---- sweep 模式 ----
constexpr double kSweepS    = 6.0;    // kd 扫描时长 s
constexpr double kObserveS  = 1.0;    // 挣脱后冻结 kd 的观察时长 s
// 两个阈值分别对应两件不同的事，不能混为一谈：
//   kFirstMoveRad  首次位移 —— 挣脱【一个齿槽定位点】所需的力矩
//   kBreakRad      持续运动 —— 累积跨过若干定位点，接近「能连续转」所需的力矩
// pos 量化步长 25/65535 = 0.000381 rad，两者分别是它的 5 倍和 52 倍。
// 2026-08-02 首次 sweep 只报了后者（kd=0.76），把挣脱力矩高估了约 65%
//（实际首次位移在 kd≈0.46）。所以现在两个都记、都报。
constexpr float  kFirstMoveRad = 0.002f;
constexpr float  kBreakRad     = 0.02f;

constexpr int    kLogMax = 8000;      // 逐帧日志容量，100Hz 下够 80 秒

using clk = std::chrono::steady_clock;
clk::time_point g_t0;

double now_s() { return std::chrono::duration<double>(clk::now() - g_t0).count(); }

uint16_t float_to_uint(float x, float xmin, float xmax, uint8_t bits) {
    float span = xmax - xmin;
    float norm = (x - xmin) / span;
    return (uint16_t)(norm * ((1 << bits) - 1));
}

float uint_to_float(uint16_t x, float xmin, float xmax, uint8_t bits) {
    float span = xmax - xmin;
    float norm = (float)x / ((1 << bits) - 1);
    return norm * span + xmin;
}

// 指令值经量化后电机实际收到的值，用于如实打印
float quantized(float x, float xmin, float xmax, uint8_t bits) {
    return uint_to_float(float_to_uint(x, xmin, xmax, bits), xmin, xmax, bits);
}

const char* err_name(uint8_t e) {
    switch (e) {
        case 0x0: return "失能";
        case 0x1: return "使能";
        case 0x8: return "超压";
        case 0x9: return "欠压";
        case 0xA: return "过电流";
        case 0xB: return "MOS过温";
        case 0xC: return "线圈过温";
        case 0xD: return "通讯丢失";
        case 0xE: return "过载";
        default:  return "未知";
    }
}

std::atomic<bool> g_stop{false};
std::atomic<int>  g_rx{0};
std::atomic<int>  g_tx{0};
std::atomic<int>  g_canerr{0};
std::atomic<int>  g_last_err{-1};
std::atomic<int>  g_prev_err{-1};

std::atomic<float> g_pos{0}, g_vel{0}, g_tau{0};
std::atomic<int>   g_tmos{0}, g_trotor{0};
std::atomic<float> g_vel_max{0}, g_tau_max{0};

// 当前下发的指令，供 on_recv 一并记进逐帧日志
std::atomic<float> g_dq_cmd{0}, g_kd_cmd{0};

// 逐帧日志。200ms 打印一行看不见粘滑结构，且反馈的 vel 字段量化步长 0.0244 rad/s，
// 低速下几乎全趴在中点码上 —— 真实速度必须靠 pos 微分。
// 只在 on_recv 里写（SDK 单线程回调），主线程等回调停了再读，不做文件 IO。
// hw_ts 是模块的硬件时间戳，不受 USB 攒包影响 —— 批内 10 帧的主机时刻会塌成
// 同一瞬间，但它们的 hw_ts 仍相差一个控制周期。两点注意（2026-08-02 dm_batch 实测）：
//   ① 不是平坦 ns 计数：高32位=秒、低32位=秒内纳秒，平坦解读会出 3.295s 假跳变；
//   ② 其时钟比主机慢 3.71%（疑似内部 RC 振荡器，随温漂，修正系数不可硬编码）。
// 所以 hw_ts 只用于批内排序/相对间隔；算速度用「主机发送周期」做时基更准。
struct Sample { float t, pos, vel, tau, dq, kd; uint64_t hw_ts; };
Sample g_log[kLogMax];
std::atomic<int> g_log_n{0};

void on_recv(dmcan_device_handle*, usb_rx_frame_t* f) {
    int len = dmcan_utils_get_len_from_dlc(f->head.dlc);
    if (f->head.can_id != kMstId || len < 8) return;

    const uint8_t* d = f->payload;
    if (d[2] == 0x33 || d[2] == 0x55 || d[2] == 0xAA) return;   // 读参数应答，跳过

    uint8_t id  = d[0] & 0x0F;
    uint8_t err = (d[0] >> 4) & 0x0F;
    if (id != (kEscId & 0x0F)) return;

    uint16_t q_u   = ((uint16_t)d[1] << 8) | d[2];
    uint16_t dq_u  = ((uint16_t)d[3] << 4) | (d[4] >> 4);
    uint16_t tau_u = ((uint16_t)(d[4] & 0x0F) << 8) | d[5];

    float pos = uint_to_float(q_u,   -kPMax, kPMax, 16);
    float vel = uint_to_float(dq_u,  -kVMax, kVMax, 12);
    float tau = uint_to_float(tau_u, -kTMax, kTMax, 12);

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
        g_log[n] = {(float)now_s(), pos, vel, tau, g_dq_cmd.load(), g_kd_cmd.load(),
                    f->head.time_stamp};
        g_log_n.store(n + 1);
    }

    int prev = g_prev_err.exchange(err);
    g_last_err.store(err);
    if (prev != (int)err) {
        std::printf("  [ERR跳变] t=%6.3fs  %s -> %d(%s)   D0=0x%02X\n", now_s(),
                    prev < 0 ? "(初始)" : err_name((uint8_t)prev), err, err_name(err), d[0]);
        std::fflush(stdout);
    }
}

void on_err(dmcan_device_handle*, usb_rx_frame_t* f) {
    g_canerr++;
    std::printf("  [CAN-ERR] t=%6.3fs ch=%d id=0x%03X esi=%d\n", now_s(),
                f->head.channel, f->head.can_id, f->head.esi);
    std::fflush(stdout);
}

void on_signal(int) { g_stop.store(true); }

void send_cmd(dmcan_device_handle* dev, uint8_t cmd) {
    uint8_t d[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd};
    dmcan_device_send_can(dev, 0, kEscId + kMitOff, false, false, false, false, 8, d);
    g_tx++;
}

// MIT 帧。kp 与 tau 由调用方固定传 0 —— 见文件头安全说明。
void send_mit(dmcan_device_handle* dev, float q, float dq, float kp, float kd, float tau) {
    uint16_t q_u   = float_to_uint(q,   -kPMax, kPMax, 16);
    uint16_t dq_u  = float_to_uint(dq,  -kVMax, kVMax, 12);
    uint16_t kp_u  = float_to_uint(kp,  0.0f,   500.0f, 12);
    uint16_t kd_u  = float_to_uint(kd,  0.0f,   5.0f,   12);
    uint16_t tau_u = float_to_uint(tau, -kTMax, kTMax, 12);

    uint8_t d[8];
    d[0] = (q_u >> 8) & 0xFF;
    d[1] = q_u & 0xFF;
    d[2] = dq_u >> 4;
    d[3] = ((dq_u & 0x0F) << 4) | ((kp_u >> 8) & 0x0F);
    d[4] = kp_u & 0xFF;
    d[5] = kd_u >> 4;
    d[6] = ((kd_u & 0x0F) << 4) | ((tau_u >> 8) & 0x0F);
    d[7] = tau_u & 0xFF;

    dmcan_device_send_can(dev, 0, kEscId + kMitOff, false, false, false, false, 8, d);
    g_tx++;
}

// tail_ms: 发完后等应答的时间。使能之后这个值必须远小于通讯丢失超时
// （实测静默 120ms 必触发），否则电机会在这段总线静默里判超时并退出使能。
void burst(dmcan_device_handle* dev, uint8_t cmd, int n, int tail_ms) {
    for (int i = 0; i < n; ++i) {
        send_cmd(dev, cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(3));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(tail_ms));
}

void disable_motor(dmcan_device_handle* dev) {
    std::printf("\n>>> 失能 0xFD ×5 ...\n");
    burst(dev, 0xFD, 5, 120);
    int e = g_last_err.load();
    std::printf(">>> 失能后 ERR = %d (%s)\n", e, e < 0 ? "无反馈" : err_name((uint8_t)e));
}

enum State { S_GRACE, S_RAMP_UP, S_RUN, S_OBSERVE, S_RAMP_DN, S_BRAKE };
const char* state_name(State s) {
    switch (s) {
        case S_GRACE:   return "宽限";
        case S_RAMP_UP: return "升速";
        case S_RUN:     return "运行";
        case S_OBSERVE: return "观察";
        case S_RAMP_DN: return "降速";
        default:        return "制动";
    }
}

}  // namespace

int main(int argc, char** argv) {
    double seconds = 2.0;      // 固定模式的匀速段时长
    float  dq_tgt  = 0.3f;     // 指令速度 rad/s
    float  kd_base = 0.3f;     // 固定模式的 kd / sweep 模式的起点
    float  kd_max  = kKdLimit; // sweep 模式的终点
    bool   reverse = false;
    bool   sweep   = false;
    std::string csv_path;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--seconds" && i + 1 < argc)     seconds = std::stod(argv[++i]);
        else if (a == "--dq" && i + 1 < argc)     dq_tgt  = std::stof(argv[++i]);
        else if (a == "--kd" && i + 1 < argc)     kd_base = std::stof(argv[++i]);
        else if (a == "--kd-max" && i + 1 < argc) kd_max  = std::stof(argv[++i]);
        else if (a == "--csv" && i + 1 < argc)    csv_path = argv[++i];
        else if (a == "--reverse")                reverse = true;
        else if (a == "--sweep")                  sweep   = true;
        else if (a == "--help") {
            std::printf(
                "用法: dm_spin [--sweep] [--seconds 2] [--dq 0.3] [--kd 0.3] [--kd-max 1.5]\n"
                "              [--reverse] [--csv 路径]\n"
                "方案 B'：MIT 纯阻尼低速旋转（kp=0, tau=0 ⇒ T = kd*(dq_des-dq)）\n"
                "  默认    固定 kd 跑 --seconds 秒\n"
                "  --sweep kd 由 --kd 线性升到 --kd-max（%.1f 秒），dq_des 锁死不动，\n"
                "          分别测出「首次位移」和「持续运动」两个门限的 kd。\n"
                "          转速天花板全程只由 dq_des 决定，提高 kd 不提速。\n"
                "  --csv   逐帧记录 t,pos,vel,tau,dq_cmd,kd_cmd，用于事后微分求真实速度\n"
                "钳位: seconds<=%.1f  |dq|<=%.2f rad/s  kd<=%.2f\n"
                "保护: |vel|>%.1f rad/s 或 |行程|>%.1f rad 或 ERR 异常 ⇒ 立即失能\n",
                kSweepS, kSecLimit, kDqLimit, kKdLimit, kVelAbort, kPosAbort);
            return 0;
        }
    }

    // ---- 钳位。用户给的值超限时如实说明，不静默改数 ----
    if (seconds > kSecLimit) { std::printf("[钳位] seconds %.2f -> %.2f\n", seconds, kSecLimit); seconds = kSecLimit; }
    if (seconds < 0.2)       { seconds = 0.2; }
    dq_tgt = std::fabs(dq_tgt);
    if (dq_tgt > kDqLimit)   { std::printf("[钳位] dq %.3f -> %.3f rad/s\n", dq_tgt, kDqLimit); dq_tgt = kDqLimit; }
    if (kd_base > kKdLimit)  { std::printf("[钳位] kd %.3f -> %.3f\n", kd_base, kKdLimit); kd_base = kKdLimit; }
    if (kd_base < 0.0f)      { kd_base = 0.0f; }
    if (kd_max > kKdLimit)   { std::printf("[钳位] kd-max %.3f -> %.3f\n", kd_max, kKdLimit); kd_max = kKdLimit; }
    if (kd_max < kd_base)    { kd_max = kd_base; }
    if (reverse) dq_tgt = -dq_tgt;

    const float kp = 0.0f, tau_ff = 0.0f;   // 硬编码，无参数可改
    const float dq_q = quantized(dq_tgt, -kVMax, kVMax, 12);

    g_t0 = clk::now();

    std::printf("=== 方案 B'：MIT 纯阻尼低速旋转%s ===\n", sweep ? "（kd 扫描）" : "");
    std::printf("目标: ESC_ID=0x%03X  反馈: MST_ID=0x%03X  模式: MIT\n", kEscId, kMstId);
    std::printf("指令: kp=0  tau=0  dq_des=%+.3f(量化后 %+.4f) rad/s\n", dq_tgt, dq_q);
    if (sweep) {
        std::printf("      kd: %.2f -> %.2f，历时 %.1fs   ⇒ 堵转力矩 %.4f -> %.4f Nm\n",
                    kd_base, kd_max, kSweepS,
                    std::fabs(kd_base * dq_q), std::fabs(kd_max * dq_q));
        std::printf("      挣脱判据: 行程 > %.3f rad (%.1f°)，随后冻结 kd 观察 %.1fs\n",
                    kBreakRad, kBreakRad * 57.2958f, kObserveS);
        std::printf("*** dq_des 全程锁死，转速天花板始终是 %.3f rad/s；提高 kd 只加力矩不加速。***\n", std::fabs(dq_q));
    } else {
        const float kd_q = quantized(kd_base, 0.0f, 5.0f, 12);
        std::printf("      kd=%.3f(量化后 %.4f)  ⇒ 堵转最大力矩 %.4f Nm（TMAX=%.0f 的 %.2f%%）\n",
                    kd_base, kd_q, std::fabs(kd_q * dq_q), kTMax,
                    100.0 * std::fabs(kd_q * dq_q) / kTMax);
        std::printf("时序: 使能 → 宽限 %dms → 升速 %.1fs → 匀速 %.1fs → 降速 %.1fs → 制动 %.1fs → 失能\n",
                    kGraceMs, kRampS, seconds, kRampS, kBrakeS);
    }
    std::printf("保护: |vel|>%.1f rad/s | |行程|>%.1f rad | ERR异常  ⇒ 立即失能\n", kVelAbort, kPosAbort);
    std::printf("*** 电机会转动。请握紧外壳、确认线缆有余量、急停可随时按下。听到啸叫立即 Ctrl-C。***\n\n");

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    dmcan_context* ctx = nullptr;
    dmcan_context_create(&ctx);
    if (dmcan_find_devices(ctx) <= 0) {
        std::printf("未找到设备\n");
        dmcan_context_destroy(ctx);
        return 1;
    }
    dmcan_device_handle* dev = nullptr;
    if (!dmcan_device_get(ctx, &dev, 0) || !dmcan_device_open(dev)) {
        std::printf("打开设备失败\n");
        dmcan_context_destroy(ctx);
        return 1;
    }
    dmcan_device_print_version(dev);
    dmcan_device_enable_channel(dev, 0);

    dmcan_channel_can_info_t info{};
    dmcan_device_get_channel_baudrate(dev, 0, &info);
    info.channel = 0;
    info.canfd = false;              // 电机在 1Mbps ⇒ 经典 CAN 2.0B
    info.can_baudrate = kCanBaud;
    info.can_sp = 0.75f;
    if (!dmcan_device_set_channel_baudrate(dev, 0, info)) {
        std::printf("设置波特率失败\n");
        dmcan_context_destroy(ctx);
        return 1;
    }
    std::printf("通道: 经典 CAN 2.0 @ %u\n\n", kCanBaud);

    dmcan_device_hook_recv_callback(dev, on_recv);
    dmcan_device_hook_err_callback(dev, on_err);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // ---- 阶段 1：清除锁存的故障 ----
    std::printf(">>> 清除错误 0xFB ×3 ...\n");
    burst(dev, 0xFB, 3, 120);   // 尚未使能，无超时约束
    int e_after_clr = g_last_err.load();
    std::printf(">>> 清错后 ERR = %d (%s)\n", e_after_clr,
                e_after_clr < 0 ? "无反馈" : err_name((uint8_t)e_after_clr));
    if (e_after_clr >= 0x8) {
        std::printf("\n!! 清错后仍有故障 %d(%s)，是持续存在的真实故障，不使能。\n",
                    e_after_clr, err_name((uint8_t)e_after_clr));
        dmcan_context_destroy(ctx);
        return 2;
    }

    // ---- 阶段 2：使能，随后立即进入控制循环（不等 ack，避免总线静默）----
    std::printf("\n>>> 使能 0xFC ×3，立即进入控制循环\n");
    burst(dev, 0xFC, 3, 5);

    // ---- 阶段 3：状态机 ----
    // 宽限期内 dq_des 保持 0：此时 T = kd*(0-vel)，是阻尼到静止，不会驱动电机转。
    std::printf(">>> 运行中，%dms 宽限期后校验使能状态再起步\n", kGraceMs);
    auto tstart = clk::now();
    auto next_print = tstart;
    int ticks = 0;
    bool aborted = false;
    bool confirmed = false;
    float pos0 = 0.0f;              // 确认使能那一刻的位置，作为行程基准

    State st = S_GRACE;
    double st_t0 = 0.0;             // 当前状态的进入时刻（相对 tstart）
    float dq_cmd = 0.0f, kd_cmd = kd_base;
    float dq_at_rampdn = 0.0f;      // 进入降速时的 dq，作为降速斜坡起点

    // sweep 结果。两个门限分别记录 —— 见 kFirstMoveRad / kBreakRad 处的说明。
    bool  moved = false, broke = false;
    float kd_first = 0.0f, tau_first = 0.0f;
    double t_first = 0.0;
    float kd_break = 0.0f, tau_break = 0.0f, pos_break = 0.0f;
    double t_break = 0.0;
    // 位移法算速度：vel 字段量化步长 0.0244 rad/s，这个速度段下不可信，
    // 只能靠 pos 在一段时间上的差分。观察段（sweep）与匀速段（固定模式）各记一组。
    float  pos_obs_end = 0.0f;
    double obs_dur = 0.0;
    float  pos_run0 = 0.0f, pos_run1 = 0.0f;
    double run_dur = 0.0;

    while (!g_stop.load()) {
        double el = std::chrono::duration<double>(clk::now() - tstart).count();
        double ts = el - st_t0;     // 在当前状态里已经待了多久

        if (st == S_GRACE) {
            dq_cmd = 0.0f;
            kd_cmd = kd_base;
            if (el * 1000 > kGraceMs) {
                int e = g_last_err.load();
                if (e >= 0x8) {
                    std::printf("\n!!! t=%.3fs 电机报故障 ERR=%d (%s)，立即停止 !!!\n",
                                el, e, err_name((uint8_t)e));
                    aborted = true; break;
                }
                if (e == 0x0) {
                    std::printf("\n!!! t=%.3fs 电机未使能或已自行退出使能 (ERR=0)，停止 !!!\n", el);
                    aborted = true; break;
                }
                if (e < 0) {
                    std::printf("\n!!! t=%.3fs 宽限期内未收到任何反馈帧，停止 !!!\n", el);
                    aborted = true; break;
                }
                confirmed = true;
                pos0 = g_pos.load();
                std::printf(">>> [t=%.3fs] 已确认使能 (ERR=1)，起点 pos0=%+.4f rad，开始升速\n",
                            el, pos0);
                st = S_RAMP_UP; st_t0 = el; ts = 0.0;
            }
        }

        if (confirmed) {
            // ---- 运行时保护 ----
            int e = g_last_err.load();
            if (e >= 0x8) {
                std::printf("\n!!! t=%.3fs 电机报故障 ERR=%d (%s)，立即停止 !!!\n",
                            el, e, err_name((uint8_t)e));
                aborted = true; break;
            }
            if (e == 0x0) {
                std::printf("\n!!! t=%.3fs 电机自行退出使能 (ERR=0)，停止 !!!\n", el);
                aborted = true; break;
            }
            float v = g_vel.load();
            if (std::fabs(v) > kVelAbort) {
                std::printf("\n!!! t=%.3fs 超速保护 vel=%+.3f rad/s > %.1f，立即停止 !!!\n",
                            el, v, kVelAbort);
                aborted = true; break;
            }
            float travel = g_pos.load() - pos0;
            if (std::fabs(travel) > kPosAbort) {
                std::printf("\n!!! t=%.3fs 超程保护 行程=%+.3f rad > %.1f，立即停止 !!!\n",
                            el, travel, kPosAbort);
                aborted = true; break;
            }

            // 首次位移：挣脱一个齿槽定位点。比「持续运动」早得多，是更接近
            // 真实静摩擦的那个数 —— 单看持续运动门限会把力矩系统性高估。
            if (!moved && std::fabs(travel) > kFirstMoveRad) {
                moved = true;
                kd_first = kd_cmd;
                tau_first = g_tau.load();
                t_first = el;
                std::printf("\n>>> [t=%.3fs] 首次位移  kd=%.4f  堵转力矩=%.4f Nm  "
                            "行程=%+.4f rad (%.2f°)  此刻 tau=%.4f Nm\n\n",
                            el, kd_first, std::fabs(kd_first * dq_q),
                            travel, travel * 57.2958f, tau_first);
            }

            // ---- 状态推进 ----
            switch (st) {
                case S_RAMP_UP:
                    dq_cmd = dq_tgt * (float)(ts / kRampS);
                    kd_cmd = kd_base;
                    if (ts >= kRampS) { pos_run0 = g_pos.load(); st = S_RUN; st_t0 = el; }
                    break;

                case S_RUN:
                    dq_cmd = dq_tgt;
                    if (sweep) {
                        double frac = ts / kSweepS;
                        if (frac > 1.0) frac = 1.0;
                        kd_cmd = kd_base + (kd_max - kd_base) * (float)frac;
                        // 挣脱检测：位置判据比速度判据可靠得多
                        //（vel 量化步长 0.0244 rad/s，低速下几乎全落在中点码上）
                        if (std::fabs(travel) > kBreakRad) {
                            broke = true;
                            kd_break = kd_cmd;
                            tau_break = g_tau.load();
                            pos_break = g_pos.load();
                            t_break = ts;
                            std::printf("\n>>> [t=%.3fs] 挣脱！kd=%.4f  堵转力矩=%.4f Nm  "
                                        "此刻 tau=%.4f Nm  行程=%+.4f rad\n",
                                        el, kd_break, std::fabs(kd_break * dq_q),
                                        tau_break, travel);
                            std::printf(">>> 冻结 kd，观察 %.1fs 看能否稳定到 %.4f rad/s\n\n",
                                        kObserveS, std::fabs(dq_q));
                            st = S_OBSERVE; st_t0 = el;
                        } else if (ts >= kSweepS) {
                            std::printf("\n>>> 扫描结束，kd 已到 %.2f（堵转力矩 %.4f Nm）仍未挣脱\n",
                                        kd_max, std::fabs(kd_max * dq_q));
                            dq_at_rampdn = dq_cmd;
                            st = S_RAMP_DN; st_t0 = el;
                        }
                    } else {
                        kd_cmd = kd_base;
                        if (ts >= seconds) {
                            pos_run1 = g_pos.load();
                            run_dur = ts;
                            dq_at_rampdn = dq_cmd; st = S_RAMP_DN; st_t0 = el;
                        }
                    }
                    break;

                case S_OBSERVE:
                    dq_cmd = dq_tgt;
                    kd_cmd = kd_break;          // 冻结，不再继续加力矩
                    if (ts >= kObserveS) {
                        pos_obs_end = g_pos.load();
                        obs_dur = ts;
                        dq_at_rampdn = dq_cmd; st = S_RAMP_DN; st_t0 = el;
                    }
                    break;

                case S_RAMP_DN:
                    dq_cmd = dq_at_rampdn * (float)(1.0 - ts / kRampS);
                    if (ts >= kRampS) { st = S_BRAKE; st_t0 = el; }
                    break;

                case S_BRAKE:
                    dq_cmd = 0.0f;              // T = kd*(0-vel)，主动阻尼制动
                    break;

                default: break;
            }
            // 注意用 el-st_t0 重算，不能复用循环顶部的 ts ——
            // 刚从降速切进制动时 st_t0 已经更新，旧 ts 还是降速段的值（≈kRampS），
            // 直接拿它比 kBrakeS 会让制动段被整段跳过。
            if (st == S_BRAKE && (el - st_t0) >= kBrakeS) break;
        }

        // 总时长兜底：正常最长约 8.1s（sweep），超过 20s 说明状态机没推进
        if (el > 20.0) {
            std::printf("\n!!! t=%.3fs 总时长兜底触发，状态机未推进，停止 !!!\n", el);
            aborted = true; break;
        }

        g_dq_cmd.store(dq_cmd);
        g_kd_cmd.store(kd_cmd);
        send_mit(dev, 0.0f, dq_cmd, kp, kd_cmd, tau_ff);
        ticks++;

        auto now = clk::now();
        if (now >= next_print) {
            next_print = now + std::chrono::milliseconds(200);
            int e = g_last_err.load();
            std::printf("  t=%4.2fs [%s] dq=%+.3f kd=%.3f | ERR=%d(%-8s) pos=%+8.4f 行程=%+7.4f "
                        "vel=%+7.3f tau=%+6.3f MOS=%d℃ 线圈=%d℃ rx=%d\n",
                        el, state_name(st), dq_cmd, kd_cmd,
                        e, e < 0 ? "无反馈" : err_name((uint8_t)e),
                        g_pos.load(), confirmed ? g_pos.load() - pos0 : 0.0f,
                        g_vel.load(), g_tau.load(),
                        g_tmos.load(), g_trotor.load(), g_rx.load());
            std::fflush(stdout);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(kPeriodMs));
    }

    if (g_stop.load()) std::printf("\n(收到中断信号，直接失能)\n");
    float pos_end = g_pos.load();
    disable_motor(dev);

    // ---- 结果 ----
    float travel = confirmed ? (pos_end - pos0) : 0.0f;
    std::printf("\n=== 统计 ===\n");
    std::printf("MIT 帧:      %d\n", ticks);
    std::printf("总发送帧:    %d  (含清错/使能/失能)\n", g_tx.load());
    std::printf("收到反馈帧:  %d\n", g_rx.load());
    std::printf("CAN 错误:    %d\n", g_canerr.load());
    if (g_rx.load() > 0)
        std::printf("应答率:      %.1f%%\n", 100.0 * g_rx.load() / g_tx.load());

    std::printf("\n=== 运动结果 ===\n");
    std::printf("起点 pos0:   %+.4f rad\n", pos0);
    std::printf("终点 pos:    %+.4f rad\n", pos_end);
    std::printf("实际行程:    %+.4f rad = %+.1f°\n", travel, travel * 57.2958f);
    std::printf("峰值 |vel|:  %.3f rad/s   峰值 |tau|: %.4f Nm\n",
                g_vel_max.load(), g_tau_max.load());

    if (sweep) {
        std::printf("\n=== kd 扫描结果 ===\n");
        if (moved) {
            float t_first_stall = std::fabs(kd_first * dq_q);
            std::printf("[门限1 首次位移] kd=%.4f  堵转力矩=%.4f Nm @输出轴 = %.4f Nm @转子(Gr=%.0f)\n",
                        kd_first, t_first_stall, t_first_stall / kGr, kGr);
            std::printf("                 t=%.2fs  此刻 tau=%.4f Nm\n", t_first, tau_first);
            std::printf("  ⇒ 挣脱【单个齿槽定位点】所需力矩 ≈ %.4f Nm @输出轴\n", t_first_stall);
        } else {
            std::printf("[门限1 首次位移] 未触发（行程始终 < %.4f rad）\n", kFirstMoveRad);
        }
        if (broke) {
            float t_stall = std::fabs(kd_break * dq_q);
            std::printf("\n[门限2 持续运动] kd=%.4f  堵转力矩=%.4f Nm @输出轴\n", kd_break, t_stall);
            std::printf("                 t=%.2fs（扫描第 %.2fs）  此刻 tau=%.4f Nm  位置 %+.4f rad\n",
                        t_break + kGraceMs / 1000.0 + kRampS, t_break, tau_break, pos_break);
            std::printf("  ⇒ 累积跨过若干定位点所需力矩 ≈ %.4f Nm @输出轴\n", t_stall);
            if (moved)
                std::printf("  注: 门限2 比门限1 高 %.0f%% —— 差值就是齿槽定位力矩的起伏，\n"
                            "      不是「静摩擦」。用门限2 当静摩擦会系统性高估。\n",
                            100.0 * (kd_break / kd_first - 1.0));

            if (obs_dur > 0.0) {
                double v_disp = (pos_obs_end - pos_break) / obs_dur;
                std::printf("\n[观察段 %.2fs @ kd=%.4f 冻结]\n", obs_dur, kd_break);
                std::printf("  位移法转速:    %+.4f rad/s  (位移 %+.4f rad)\n",
                            v_disp, pos_obs_end - pos_break);
                std::printf("  指令转速:      %+.4f rad/s   达成率 %.0f%%\n",
                            dq_q, 100.0 * std::fabs(v_disp / dq_q));
                // 稳态时 T = kd*(dq_des-dq) = T_摩擦，反解动摩擦
                double t_fric = kd_break * (std::fabs(dq_q) - std::fabs(v_disp));
                std::printf("  ⇒ 反解动摩擦:  %.4f Nm @输出轴\n", t_fric);
                std::printf("  纯阻尼(kp=0,无前馈)的稳态速度误差恒为 T摩擦/kd，消不掉。\n");
                std::printf("  按此模型外推: kd=1.14 ⇒ %.3f rad/s;  kd=%.2f(上限) ⇒ %.3f rad/s\n",
                            std::fabs(dq_q) - t_fric / 1.14,
                            kKdLimit, std::fabs(dq_q) - t_fric / kKdLimit);
            }
        } else {
            float t_max = std::fabs(kd_max * dq_q);
            std::printf("未挣脱。kd 已扫到 %.2f，堵转力矩 %.4f Nm @ 输出轴仍不足以启动。\n",
                        kd_max, t_max);
            std::printf("⇒ 结论: 静摩擦力矩 > %.4f Nm @ 输出轴。\n", t_max);
            std::printf("  这已经是 J4310 输出端额定力矩的量级下限附近，偏高。\n");
            std::printf("  下一步应先排查而不是继续加力矩：输出轴是否被卡住/顶住？\n");
            std::printf("  失能状态下用手能否顺畅转动输出轴？（失能时应能背驱）\n");
        }
    } else {
        float kd_q = quantized(kd_base, 0.0f, 5.0f, 12);
        if (run_dur > 0.0) {
            double v_disp = (pos_run1 - pos_run0) / run_dur;
            std::printf("\n=== 匀速段 %.2fs @ kd=%.4f ===\n", run_dur, kd_q);
            std::printf("  位移法转速:    %+.4f rad/s  (位移 %+.4f rad = %+.1f°)\n",
                        v_disp, pos_run1 - pos_run0, (pos_run1 - pos_run0) * 57.2958f);
            std::printf("  指令转速:      %+.4f rad/s   达成率 %.0f%%\n",
                        dq_q, 100.0 * std::fabs(v_disp / dq_q));
            // 稳态 T = kd*(dq_des-dq) = T_摩擦
            double t_fric = kd_q * (std::fabs(dq_q) - std::fabs(v_disp));
            std::printf("  ⇒ 反解动摩擦:  %.4f Nm @输出轴\n", t_fric);
            std::printf("  (上一次 sweep 观察段反解为 0.164 Nm —— 两者是否一致，"
                        "决定摩擦模型能不能用)\n");
        }
        if (confirmed && std::fabs(travel) < 0.05f) {
            std::printf("\n>> 电机基本没转。堵转力矩 %.4f Nm 低于挣脱所需。\n",
                        std::fabs(kd_q * dq_q));
            std::printf(">> 建议用 --sweep 直接测出所需的 kd，不要靠猜。\n");
        }
    }

    // ---- 逐帧日志落盘 ----
    if (!csv_path.empty()) {
        int n = g_log_n.load();
        FILE* fp = std::fopen(csv_path.c_str(), "w");
        if (!fp) {
            std::printf("\n!! 无法写入 %s\n", csv_path.c_str());
        } else {
            std::fprintf(fp, "t_s,hw_ts_ns,pos_rad,vel_rad_s,tau_Nm,dq_cmd,kd_cmd\n");
            for (int i = 0; i < n; ++i) {
                const Sample& s = g_log[i];
                std::fprintf(fp, "%.6f,%llu,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                             s.t, (unsigned long long)s.hw_ts,
                             s.pos, s.vel, s.tau, s.dq, s.kd);
            }
            std::fclose(fp);
            std::printf("\n逐帧日志: %d 行 -> %s\n", n, csv_path.c_str());
            if (n >= kLogMax) std::printf("!! 日志缓冲已满(%d)，后续帧未记录\n", kLogMax);
        }
    }

    dmcan_context_destroy(ctx);
    return aborted ? 2 : 0;
}
