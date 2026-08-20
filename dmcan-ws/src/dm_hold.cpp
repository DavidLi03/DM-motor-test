// dm_hold.cpp —— 方案 A：清错 → 使能 → 零力矩保持
//
// 目的：验证使能流程与反馈帧回传，电机【不应自己转动】。
//
// 安全设计（针对「电机未固定，摆在桌面上」）：
//   1. kp / kd / tau 全部硬编码为 0，且【没有】命令行参数可以改。
//      MIT 力矩公式 T = kp*(q_des-q) + kd*(dq_des-dq) + tau ⇒ 恒等于 0。
//      电机上电但输出零力矩，轴是「软」的，可以用手拧动，不会自驱。
//   2. 边发边校验：宽限期后回读 ERR，非使能态（0 / >=8 / 无反馈）立即失能退出。
//      注意不能「先等 ack 再发帧」—— 等 ack 期间总线是静默的，会触发通讯丢失。
//   3. 运行时长默认 5 秒，硬上限 30 秒。
//   4. 正常退出 / 超时 / 校验失败 / Ctrl-C 四条路径都会发 0xFD 失能。
//   5. 反馈帧 ERR 码 >= 8（各类故障）立即失能并退出。
//
// 会发送的命令（0xFB 已获用户明确批准）：
//   0xFB 清除错误 —— 只清故障标志位，不写寄存器、不碰 flash、不动零点和 CTRL_MODE
//   0xFC 使能 / 0xFD 失能 / MIT 零力矩帧
// 绝不发送：0x55 写参数 / 0xAA 存参数 / 0xFE 存零点
//
// 协议依据见 ../PROGRESS-2026-08-02.md 第五节。

#include <atomic>
#include <chrono>
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

constexpr uint32_t kCanBaud   = 1000000;  // 1Mbps ⇒ 经典 CAN 2.0B
constexpr int      kPeriodMs  = 10;       // 100Hz，远快于 TIMEOUT=400
// 宽限期必须【长于】收帧延迟，否则读到的是使能前的旧 ERR 值。
// 实测收帧延迟 16~22ms（dm_probe --latency，20 帧样本），取 150ms 留足裕量。
constexpr int      kGraceMs   = 150;
// 使能状态下总线长时间静默会触发「通讯丢失」(ERR=0xD) 并自动退出使能。
// 实测边界：静默 120ms 必触发；静默 ~15ms 不触发。真实阈值在两者之间，未精确测定。
// 寄存器 0x09 TIMEOUT 读回 400，手册未标单位（若为 0.25ms 则恰好 100ms，仅属猜测）。

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
std::atomic<int>  g_prev_err{-1};   // 用于检测 ERR 跳变

std::atomic<float> g_pos{0}, g_vel{0}, g_tau{0};
std::atomic<int>   g_tmos{0}, g_trotor{0};

void on_recv(dmcan_device_handle*, usb_rx_frame_t* f) {
    int len = dmcan_utils_get_len_from_dlc(f->head.dlc);
    if (f->head.can_id != kMstId || len < 8) return;

    const uint8_t* d = f->payload;
    // 读参数应答与反馈帧同 ID，靠 D2==0x33/0x55/0xAA 区分，这里跳过应答帧
    if (d[2] == 0x33 || d[2] == 0x55 || d[2] == 0xAA) return;

    uint8_t id  = d[0] & 0x0F;
    uint8_t err = (d[0] >> 4) & 0x0F;
    if (id != (kEscId & 0x0F)) return;

    uint16_t q_u   = ((uint16_t)d[1] << 8) | d[2];
    uint16_t dq_u  = ((uint16_t)d[3] << 4) | (d[4] >> 4);
    uint16_t tau_u = ((uint16_t)(d[4] & 0x0F) << 8) | d[5];

    g_pos.store(uint_to_float(q_u,   -kPMax, kPMax, 16));
    g_vel.store(uint_to_float(dq_u,  -kVMax, kVMax, 12));
    g_tau.store(uint_to_float(tau_u, -kTMax, kTMax, 12));
    g_tmos.store(d[6]);
    g_trotor.store(d[7]);
    g_rx++;

    int prev = g_prev_err.exchange(err);
    g_last_err.store(err);
    if (prev != (int)err) {   // ERR 跳变，记录时刻与原始字节，便于定位
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

// 控制命令帧：FF FF FF FF FF FF FF <cmd>
void send_cmd(dmcan_device_handle* dev, uint8_t cmd) {
    uint8_t d[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd};
    dmcan_device_send_can(dev, 0, kEscId + kMitOff, false, false, false, false, 8, d);
    g_tx++;
}

// MIT 帧。本程序只以全零参数调用 —— 见文件头安全说明。
void send_mit_zero(dmcan_device_handle* dev) {
    const float q = 0.0f, dq = 0.0f, kp = 0.0f, kd = 0.0f, tau = 0.0f;

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

// tail_ms: 发完后等应答的时间。
// !! 使能之后这个值必须远小于通讯丢失超时（实测约 100ms），否则电机会在这段
//    总线静默里判超时并退出使能 —— 前三次调试失败的根因就是这里原本写死 120ms。
void burst(dmcan_device_handle* dev, uint8_t cmd, int n, int tail_ms) {
    for (int i = 0; i < n; ++i) {
        send_cmd(dev, cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(3));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(tail_ms));
}

void disable_motor(dmcan_device_handle* dev) {
    std::printf("\n>>> 失能 0xFD ×5 ...\n");
    burst(dev, 0xFD, 5, 120);   // 已失能，无超时约束
    int e = g_last_err.load();
    std::printf(">>> 失能后 ERR = %d (%s)\n", e, e < 0 ? "无反馈" : err_name((uint8_t)e));
}

}  // namespace

int main(int argc, char** argv) {
    double seconds = 5.0;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--seconds" && i + 1 < argc) seconds = std::stod(argv[++i]);
        else if (a == "--help") {
            std::printf("用法: dm_hold [--seconds 5]   (硬上限 30 秒)\n"
                        "方案 A：清错 → 使能 → 零力矩保持。kp/kd/tau 恒为 0，电机不会自转。\n");
            return 0;
        }
    }
    if (seconds > 30.0) seconds = 30.0;
    if (seconds < 0.5)  seconds = 0.5;

    g_t0 = clk::now();

    std::printf("=== 方案 A：清错 → 使能 → 零力矩保持 ===\n");
    std::printf("目标: ESC_ID=0x%03X  反馈: MST_ID=0x%03X  模式: MIT\n", kEscId, kMstId);
    std::printf("指令: q=0 dq=0 kp=0 kd=0 tau=0  ⇒  输出力矩恒为 0\n");
    std::printf("时长: %.1f 秒, 控制频率 %dHz (电机 TIMEOUT=400), 结束自动失能\n",
                seconds, 1000 / kPeriodMs);
    std::printf("电机会上电但输出零力矩（轴可用手拧动），不应自行转动。\n\n");

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
        std::printf("\n!! 清错后仍有故障 %d(%s)，说明是持续存在的真实故障，不继续使能。\n",
                    e_after_clr, err_name((uint8_t)e_after_clr));
        dmcan_context_destroy(ctx);
        return 2;
    }

    // ---- 阶段 2：使能，随后立即进入控制循环 ----
    //
    // 这里【不能】先等 ack 再进循环，两个约束互相冲突：
    //   · 收帧延迟 16~22ms ⇒ 想读到使能 ack 得先等一段
    //   · 通讯丢失看门狗 ⇒ 使能后静默过久（实测 120ms 必触发）就会退出使能
    // 解法是「边发边校验」：发完 0xFC 立刻开始发 MIT 帧维持总线活动，
    // 等宽限期过去（延迟已覆盖）再回读 ERR 确认是否真的使能了。
    std::printf("\n>>> 使能 0xFC ×3，立即进入控制循环（不等 ack，避免总线静默）\n");
    burst(dev, 0xFC, 3, 5);

    // ---- 阶段 3：零力矩保持 ----
    std::printf(">>> 零力矩保持中，%dms 宽限期后校验使能状态\n", kGraceMs);
    auto tstart = clk::now();
    auto next_print = tstart;
    int ticks = 0;
    bool aborted = false;
    bool confirmed = false;   // 是否已在宽限期后确认 ERR==1

    while (!g_stop.load()) {
        auto now = clk::now();
        double el = std::chrono::duration<double>(now - tstart).count();
        if (el >= seconds) break;

        if (el * 1000 > kGraceMs) {
            int e = g_last_err.load();
            if (e >= 0x8) {
                std::printf("\n!!! t=%.3fs 电机报故障 ERR=%d (%s)，立即停止 !!!\n",
                            el, e, err_name((uint8_t)e));
                aborted = true;
                break;
            }
            if (e == 0x0) {
                std::printf("\n!!! t=%.3fs 电机未使能或已自行退出使能 (ERR=0)，停止 !!!\n", el);
                aborted = true;
                break;
            }
            if (e < 0) {
                std::printf("\n!!! t=%.3fs 宽限期内未收到任何反馈帧，停止 !!!\n", el);
                aborted = true;
                break;
            }
            if (!confirmed) {   // 首次通过校验，明确告知已确认使能
                confirmed = true;
                std::printf(">>> [t=%.3fs] 已确认使能 (ERR=1)，继续保持\n", el);
            }
        }

        send_mit_zero(dev);
        ticks++;

        if (now >= next_print) {
            next_print = now + std::chrono::milliseconds(250);
            int e = g_last_err.load();
            std::printf("  t=%4.1fs  ERR=%d(%-8s) pos=%+8.4f rad  vel=%+7.3f rad/s  "
                        "tau=%+6.3f Nm  MOS=%d℃ 线圈=%d℃  rx=%d\n",
                        el, e, e < 0 ? "无反馈" : err_name((uint8_t)e),
                        g_pos.load(), g_vel.load(), g_tau.load(),
                        g_tmos.load(), g_trotor.load(), g_rx.load());
            std::fflush(stdout);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(kPeriodMs));
    }

    if (g_stop.load()) std::printf("\n(收到中断信号)\n");
    disable_motor(dev);

    std::printf("\n=== 统计 ===\n");
    std::printf("MIT 帧:      %d\n", ticks);
    std::printf("总发送帧:    %d  (含清错/使能/失能)\n", g_tx.load());
    std::printf("收到反馈帧:  %d\n", g_rx.load());
    std::printf("CAN 错误:    %d\n", g_canerr.load());
    if (g_rx.load() == 0)
        std::printf("!! 没有收到任何反馈帧 —— 请检查供电与接线\n");
    else
        std::printf("应答率:      %.1f%%\n", 100.0 * g_rx.load() / g_tx.load());

    dmcan_context_destroy(ctx);
    return aborted ? 2 : 0;
}
