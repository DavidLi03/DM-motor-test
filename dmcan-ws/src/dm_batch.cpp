// dm_batch.cpp —— 零运动实验：判定 USB 模块的反馈帧攒包机制
//
// 问题：2026-08-02 由 dm_spin --csv 发现，100Hz 控制下反馈帧不是逐帧到达，而是
//       每 100ms 一次性投递 10 帧（批内跨度 8 微秒，批间隔 100.08~100.38ms）。
//       需要判定这是【纯定时器】还是【填满即发】：
//         · 纯定时器  ⇒ 各发送速率下批间隔恒为 ~100ms，只是每批帧数变多。
//                       提高控制频率对反馈时效毫无帮助。
//         · 填满即发  ⇒ 高速率下批来得更快。可以靠提速换时效。
//
// 【本程序不会让电机转动，也不会让它上电。】
//   唯一的发送函数是 read_reg()，只发 0x7FF + 0x33【读】参数命令。
//   全文没有 0xFC(使能) / 0xFD(失能) / 0xFB(清错) / 0xFE(存零点) / MIT 帧，
//   也没有 0x55(写参数) / 0xAA(存参数)。电机全程保持失能，物理上不可能动。
//   之所以能用来测攒包：失能状态下电机同样会对每一条读命令回一帧应答。
//
// 判别依据是两个时间基准的对比：
//   host_t   主机侧回调触发时刻   —— 受攒包影响
//   hw_ts    模块硬件时间戳(ns)   —— 帧真正到达模块的时刻，不受攒包影响
// 若 hw_ts 间隔 ≈ 发送周期而 host_t 间隔呈双峰（批内 ~0 / 批间 100ms），
// 攒包就坐实在模块投递环节，而不是 CAN 总线或电机侧。

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>
#include <algorithm>

#include "dmcan.h"

namespace {

constexpr uint16_t kEscId   = 0x008;   // 电机接收 ID
constexpr uint16_t kMstId   = 0x018;   // 电机反馈 ID
constexpr uint16_t kQueryId = 0x7FF;   // 参数读写专用 ID
constexpr uint8_t  kRidSafe = 0x0A;    // CTRL_MODE —— 只读，不写
constexpr uint32_t kCanBaud = 1000000; // 1Mbps ⇒ 经典 CAN 2.0B
constexpr int      kLogMax  = 20000;

using clk = std::chrono::steady_clock;
clk::time_point g_t0;
long long now_us() {
    return std::chrono::duration_cast<std::chrono::microseconds>(clk::now() - g_t0).count();
}

std::atomic<bool>  g_stop{false};
std::atomic<int>   g_canerr{0};
std::atomic<float> g_rate_now{0};      // 当前阶段的发送速率，用于给样本打标

struct Sample {
    float    rate;
    long long t_us;      // 主机侧回调时刻
    uint64_t hw_ts;      // 模块硬件时间戳(ns)
    uint32_t can_id;
    uint8_t  dir;        // 0=rx 1=tx（模块可能回显发送帧）
};
Sample g_log[kLogMax];
std::atomic<int> g_log_n{0};

void on_recv(dmcan_device_handle*, usb_rx_frame_t* f) {
    int n = g_log_n.load();
    if (n < kLogMax) {
        g_log[n] = {g_rate_now.load(), now_us(), f->head.time_stamp,
                    (uint32_t)f->head.can_id, (uint8_t)f->head.dir};
        g_log_n.store(n + 1);
    }
}

void on_err(dmcan_device_handle*, usb_rx_frame_t*) { g_canerr++; }
void on_signal(int) { g_stop.store(true); }

double median(std::vector<double> v) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
}
double pct(std::vector<double> v, double p) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    size_t i = (size_t)(p * (v.size() - 1));
    return v[i];
}

}  // namespace

int main(int argc, char** argv) {
    std::vector<double> rates = {10, 20, 50, 100, 200, 500};
    double phase_s = 2.0;
    std::string csv_path;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--phase" && i + 1 < argc)    phase_s = std::stod(argv[++i]);
        else if (a == "--csv" && i + 1 < argc) csv_path = argv[++i];
        else if (a == "--rates" && i + 1 < argc) {
            rates.clear();
            std::string s = argv[++i], tok;
            for (char c : s) {
                if (c == ',') { if (!tok.empty()) rates.push_back(std::stod(tok)); tok.clear(); }
                else tok += c;
            }
            if (!tok.empty()) rates.push_back(std::stod(tok));
        }
        else if (a == "--help") {
            std::printf("用法: dm_batch [--rates 10,20,50,100,200,500] [--phase 2.0] [--csv 路径]\n"
                        "零运动实验：判定 USB 模块反馈帧的攒包机制是定时器还是填满即发。\n"
                        "只发 0x7FF+0x33 读参数命令，电机全程失能，不会转动。\n");
            return 0;
        }
    }
    if (phase_s < 0.5) phase_s = 0.5;
    if (phase_s > 10.0) phase_s = 10.0;

    g_t0 = clk::now();

    std::printf("=== 攒包机制判定（零运动） ===\n");
    std::printf("只发 0x7FF + 0x33 读 RID=0x%02X，电机保持【失能】，不会转动。\n", kRidSafe);
    std::printf("速率序列: ");
    for (size_t i = 0; i < rates.size(); ++i) std::printf("%.0f%s", rates[i], i + 1 < rates.size() ? ", " : "");
    std::printf(" Hz   每档 %.1f 秒\n\n", phase_s);

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    dmcan_context* ctx = nullptr;
    dmcan_context_create(&ctx);
    if (dmcan_find_devices(ctx) <= 0) {
        std::printf("未找到设备\n"); dmcan_context_destroy(ctx); return 1;
    }
    dmcan_device_handle* dev = nullptr;
    if (!dmcan_device_get(ctx, &dev, 0) || !dmcan_device_open(dev)) {
        std::printf("打开设备失败\n"); dmcan_context_destroy(ctx); return 1;
    }
    dmcan_device_print_version(dev);
    dmcan_device_enable_channel(dev, 0);

    dmcan_channel_can_info_t info{};
    dmcan_device_get_channel_baudrate(dev, 0, &info);
    info.channel = 0;
    info.canfd = false;
    info.can_baudrate = kCanBaud;
    info.can_sp = 0.75f;
    if (!dmcan_device_set_channel_baudrate(dev, 0, info)) {
        std::printf("设置波特率失败\n"); dmcan_context_destroy(ctx); return 1;
    }
    std::printf("通道: 经典 CAN 2.0 @ %u\n\n", kCanBaud);

    dmcan_device_hook_recv_callback(dev, on_recv);
    dmcan_device_hook_err_callback(dev, on_err);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // 本程序唯一的发送路径。0x33 = 读参数，对电机无任何副作用。
    auto read_reg = [&](void) {
        uint8_t d[4] = {(uint8_t)(kEscId & 0xFF), (uint8_t)((kEscId >> 8) & 0xFF), 0x33, kRidSafe};
        dmcan_device_send_can(dev, 0, kQueryId, false, false, false, false, 4, d);
    };

    struct PhaseInfo { double rate; int tx; int idx0, idx1; };
    std::vector<PhaseInfo> phases;

    for (double r : rates) {
        if (g_stop.load()) break;
        // 阶段之间静默 300ms：让上一阶段残留在模块缓冲里的帧全部落地，
        // 否则会被错算进下一阶段。
        g_rate_now.store(0.0f);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));

        int idx0 = g_log_n.load();
        g_rate_now.store((float)r);
        auto period = std::chrono::microseconds((long long)(1e6 / r));
        auto next = clk::now();
        auto endt = next + std::chrono::milliseconds((long long)(phase_s * 1000));
        int tx = 0;
        while (clk::now() < endt && !g_stop.load()) {
            read_reg(); tx++;
            next += period;
            std::this_thread::sleep_until(next);   // 绝对时基，避免误差累积
        }
        // 留 250ms 收尾，把最后一批收进来（>2 个攒包周期）
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        int idx1 = g_log_n.load();
        phases.push_back({r, tx, idx0, idx1});
        std::printf("  %5.0f Hz: 发 %4d 帧, 收 %4d 帧\n", r, tx, idx1 - idx0);
        std::fflush(stdout);
    }

    dmcan_device_hook_recv_callback(dev, nullptr);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // ---- 分析 ----
    std::printf("\n=== 逐档分析 ===\n");
    std::printf("%6s %6s %6s | %-26s | %-30s\n",
                "速率", "发", "收", "hw_ts 间隔(ms)", "主机到达间隔(ms)");
    std::printf("%6s %6s %6s | %8s %8s %8s | %7s %7s %7s %6s\n",
                "", "", "", "中位", "p90", "最大", "中位", "p90", "最大", "批数");
    std::printf("---------------------------------------------------------------------------------\n");

    for (auto& p : phases) {
        std::vector<double> hw_gap, host_gap;
        long long prev_t = -1; uint64_t prev_hw = 0;
        int nrx = 0;
        for (int i = p.idx0; i < p.idx1; ++i) {
            const Sample& s = g_log[i];
            if (s.can_id != kMstId) continue;       // 只算电机应答
            nrx++;
            if (prev_t >= 0) {
                host_gap.push_back((s.t_us - prev_t) / 1000.0);
                hw_gap.push_back(s.hw_ts > prev_hw ? (s.hw_ts - prev_hw) / 1e6 : 0.0);
            }
            prev_t = s.t_us; prev_hw = s.hw_ts;
        }
        // 批边界：主机到达间隔超过发送周期一半即认定跨批
        double period_ms = 1000.0 / p.rate;
        double thr = std::max(1.0, period_ms * 0.5);
        int nb = 1;
        for (double g : host_gap) if (g > thr) nb++;
        if (host_gap.empty()) nb = 0;

        std::printf("%5.0fH %6d %6d | %8.3f %8.3f %8.3f | %7.3f %7.3f %7.3f %6d\n",
                    p.rate, p.tx, nrx,
                    median(hw_gap), pct(hw_gap, 0.9), hw_gap.empty() ? 0 : *std::max_element(hw_gap.begin(), hw_gap.end()),
                    median(host_gap), pct(host_gap, 0.9), host_gap.empty() ? 0 : *std::max_element(host_gap.begin(), host_gap.end()),
                    nb);
    }

    std::printf("\n=== 批结构 ===\n");
    for (auto& p : phases) {
        std::vector<long long> ts;
        for (int i = p.idx0; i < p.idx1; ++i)
            if (g_log[i].can_id == kMstId) ts.push_back(g_log[i].t_us);
        if (ts.size() < 3) { std::printf("%5.0f Hz: 样本不足\n", p.rate); continue; }
        double period_ms = 1000.0 / p.rate;
        double thr = std::max(1.0, period_ms * 0.5) * 1000.0;   // us
        std::vector<double> bstart, bsize;
        double cnt = 1; bstart.push_back((double)ts[0]);
        for (size_t i = 1; i < ts.size(); ++i) {
            if (ts[i] - ts[i - 1] > thr) { bsize.push_back(cnt); cnt = 1; bstart.push_back((double)ts[i]); }
            else cnt++;
        }
        bsize.push_back(cnt);
        std::vector<double> biv;
        for (size_t i = 1; i < bstart.size(); ++i) biv.push_back((bstart[i] - bstart[i - 1]) / 1000.0);
        std::printf("%5.0f Hz: 批数 %2d  每批帧数 中位 %.0f (范围 %.0f-%.0f)  批间隔 中位 %.2fms (范围 %.2f-%.2f)\n",
                    p.rate, (int)bsize.size(), median(bsize),
                    *std::min_element(bsize.begin(), bsize.end()),
                    *std::max_element(bsize.begin(), bsize.end()),
                    median(biv),
                    biv.empty() ? 0 : *std::min_element(biv.begin(), biv.end()),
                    biv.empty() ? 0 : *std::max_element(biv.begin(), biv.end()));
    }

    std::printf("\n=== 判读 ===\n");
    std::printf("· 各档【批间隔】都 ≈100ms、而每批帧数随速率线性增长 ⇒ 纯定时器投递。\n");
    std::printf("  含义: 提高控制频率不改善反馈时效，闭环带宽被钉死在 ~10Hz。\n");
    std::printf("· 高速率档【批间隔】明显变小 ⇒ 填满即发，可用提速换时效。\n");
    std::printf("· 各档【hw_ts 间隔】都 ≈发送周期 ⇒ 电机逐帧应答正常，攒包在模块投递环节，\n");
    std::printf("  与 CAN 总线和电机无关。\n");
    std::printf("CAN 错误: %d\n", g_canerr.load());

    if (!csv_path.empty()) {
        int n = g_log_n.load();
        FILE* fp = std::fopen(csv_path.c_str(), "w");
        if (!fp) std::printf("\n!! 无法写入 %s\n", csv_path.c_str());
        else {
            std::fprintf(fp, "rate_hz,host_t_us,hw_ts_ns,can_id,dir\n");
            for (int i = 0; i < n; ++i)
                std::fprintf(fp, "%.0f,%lld,%llu,0x%03X,%u\n", g_log[i].rate,
                             (long long)g_log[i].t_us, (unsigned long long)g_log[i].hw_ts,
                             g_log[i].can_id, g_log[i].dir);
            std::fclose(fp);
            std::printf("\n逐帧日志: %d 行 -> %s\n", n, csv_path.c_str());
            if (n >= kLogMax) std::printf("!! 日志缓冲已满(%d)，后续帧未记录\n", kLogMax);
        }
    }

    dmcan_context_destroy(ctx);
    return 0;
}
