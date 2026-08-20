// dm_batch_sc.cpp —— SocketCAN 路线的投递时效验收（零运动）
//
// 背景：出厂固件按 100ms 纯定时器攒批投递反馈帧，应用层有效反馈率只有 10Hz
//      （2026-08-02 由 dm_batch 判定）。slcan 固件的卖点就是逐帧投递 ——
//       本程序用与 dm_batch 相同的实验设计验收这一点，并测出真实的
//       请求→应答延迟分布。
//
// 【本程序不会让电机转动，也不会让它上电。】
//   唯一的发送是 0x7FF + 0x33【读】参数命令。全文没有 0xFC/0xFD/0xFB/0xFE/
//   MIT 帧，也没有 0x55/0xAA。电机全程保持失能，物理上不可能动。
//
// 延迟测法（吸取 dm_probe --latency 的教训，见 PROGRESS 第 4.5 节）：
//   × 不拿「每帧到达时刻 − 最近一次发送」—— 攒包时结构上永远测不出真相。
//   ✓ 给每帧配对它自己的发送时刻：电机对每条读命令按序回一帧（SDK 路线
//     实测六档应答率均 100%），第 k 条应答配第 k 次发送。数量不齐则该档
//     只报到达间隔分布，不报配对延迟。
//
// 判读预期：
//   · 到达间隔中位 ≈ 发送周期、批数 ≈ 帧数（每批 1 帧）⇒ 逐帧投递，验收通过。
//   · 到达间隔呈双峰（~0 / 定值）⇒ 仍在攒包，slcan 固件没解决问题。
//
// 用法: dm_batch_sc [--if can0] [--rates 10,20,50,100,200,500,1000] [--phase 2.0] [--csv 路径]

#include <atomic>
#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

#include "dm_sc.hpp"

namespace {

constexpr uint8_t kRidSafe = 0x0A;   // CTRL_MODE —— 只读，不写
constexpr int     kLogMax  = 40000;

using clk = std::chrono::steady_clock;
clk::time_point g_t0;
long long now_us() {
    return std::chrono::duration_cast<std::chrono::microseconds>(clk::now() - g_t0).count();
}

std::atomic<bool> g_stop{false};
std::atomic<bool> g_rx_quit{false};
std::atomic<int>  g_canerr{0};
std::atomic<float> g_rate_now{0};

struct Sample {
    float     rate;
    long long t_us;      // 主机到达时刻（steady_clock）
    uint64_t  kts;       // 内核收包时间戳(ns, CLOCK_REALTIME 域)
    uint32_t  can_id;
};
Sample g_log[kLogMax];
std::atomic<int> g_log_n{0};

void rx_loop(int s) {
    can_frame fr{};
    uint64_t kts = 0;
    while (!g_rx_quit.load()) {
        int r = dmsc::recv_frame(s, &fr, &kts, 20);
        if (r <= 0) continue;
        if (fr.can_id & CAN_ERR_FLAG) { g_canerr++; continue; }
        int n = g_log_n.load();
        if (n < kLogMax) {
            g_log[n] = {g_rate_now.load(), now_us(), kts, fr.can_id & CAN_SFF_MASK};
            g_log_n.store(n + 1);
        }
    }
}

void on_signal(int) { g_stop.store(true); }

double median(std::vector<double> v) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
}
double pct(std::vector<double> v, double p) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    return v[(size_t)(p * (v.size() - 1))];
}

}  // namespace

int main(int argc, char** argv) {
    std::string ifname = "can0";
    std::vector<double> rates = {10, 20, 50, 100, 200, 500, 1000};
    double phase_s = 2.0;
    std::string csv_path;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--if" && i + 1 < argc)         ifname = argv[++i];
        else if (a == "--phase" && i + 1 < argc) phase_s = std::stod(argv[++i]);
        else if (a == "--csv" && i + 1 < argc)   csv_path = argv[++i];
        else if (a == "--rates" && i + 1 < argc) {
            rates.clear();
            std::string sarg = argv[++i], tok;
            for (char c : sarg) {
                if (c == ',') { if (!tok.empty()) rates.push_back(std::stod(tok)); tok.clear(); }
                else tok += c;
            }
            if (!tok.empty()) rates.push_back(std::stod(tok));
        }
        else if (a == "--help") {
            std::printf("用法: dm_batch_sc [--if can0] [--rates 10,...,1000] [--phase 2.0] [--csv 路径]\n"
                        "SocketCAN 投递时效验收：只发 0x33 读命令，电机全程失能，不会转动。\n");
            return 0;
        }
    }
    if (phase_s < 0.5) phase_s = 0.5;
    if (phase_s > 10.0) phase_s = 10.0;

    g_t0 = clk::now();

    std::printf("=== SocketCAN 投递时效验收（零运动）@ %s ===\n", ifname.c_str());
    std::printf("只发 0x7FF + 0x33 读 RID=0x%02X，电机保持【失能】，不会转动。\n", kRidSafe);
    std::printf("速率序列: ");
    for (size_t i = 0; i < rates.size(); ++i)
        std::printf("%.0f%s", rates[i], i + 1 < rates.size() ? ", " : "");
    std::printf(" Hz   每档 %.1f 秒\n\n", phase_s);

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    std::string err;
    int s = dmsc::open_can(ifname.c_str(), true, &err);
    if (s < 0) { std::printf("%s\n", err.c_str()); return 1; }

    std::thread rx(rx_loop, s);

    struct PhaseInfo { double rate; int tx; int txfail; int idx0, idx1; std::vector<long long> tx_us; };
    std::vector<PhaseInfo> phases;

    for (double r : rates) {
        if (g_stop.load()) break;
        // 阶段之间静默 300ms，把上一档残留的应答收干净
        g_rate_now.store(0.0f);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));

        PhaseInfo p;
        p.rate = r;
        p.tx = 0;
        p.txfail = 0;
        p.idx0 = g_log_n.load();
        g_rate_now.store((float)r);

        auto period = std::chrono::microseconds((long long)(1e6 / r));
        auto next = clk::now();
        auto endt = next + std::chrono::milliseconds((long long)(phase_s * 1000));
        while (clk::now() < endt && !g_stop.load()) {
            long long t = now_us();
            if (dmsc::send_read_reg(s, kRidSafe)) { p.tx_us.push_back(t); p.tx++; }
            else p.txfail++;
            next += period;
            std::this_thread::sleep_until(next);   // 绝对时基，避免误差累积
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(250));   // 收尾
        p.idx1 = g_log_n.load();
        phases.push_back(std::move(p));
        auto& q = phases.back();
        std::printf("  %5.0f Hz: 发 %4d 帧 (失败 %d), 收 %4d 帧\n",
                    r, q.tx, q.txfail, q.idx1 - q.idx0);
        std::fflush(stdout);
    }

    g_rx_quit.store(true);
    rx.join();

    // ---- 分析 ----
    std::printf("\n=== 逐档分析 ===\n");
    std::printf("%6s %6s %6s | %-26s | %-22s | %6s\n",
                "速率", "发", "收", "到达间隔(ms)", "配对延迟 请求→应答(ms)", "批数");
    std::printf("%6s %6s %6s | %8s %8s %8s | %7s %7s %6s |\n",
                "", "", "", "中位", "p90", "最大", "中位", "p90", "最大");
    std::printf("--------------------------------------------------------------------------------------\n");

    bool all_per_frame = true;
    for (auto& p : phases) {
        std::vector<double> gap, lat;
        long long prev_t = -1;
        int nrx = 0;
        for (int i = p.idx0; i < p.idx1; ++i) {
            const Sample& sm = g_log[i];
            if (sm.can_id != dmsc::kMstId) continue;
            if (prev_t >= 0) gap.push_back((sm.t_us - prev_t) / 1000.0);
            prev_t = sm.t_us;
            // 逐帧配对：第 nrx 条应答 ←→ 第 nrx 次发送
            if (nrx < (int)p.tx_us.size()) lat.push_back((sm.t_us - p.tx_us[nrx]) / 1000.0);
            nrx++;
        }
        bool paired = (nrx == p.tx);   // 数量不齐则配对失去意义
        double period_ms = 1000.0 / p.rate;
        // 批边界阈值 = 半个周期。不能再加 1ms 下限：1000Hz 档周期本身就是 1ms，
        // 下限会把阈值顶到与周期相等，约一半正常间隔被误判为批内 ⇒ 假 ✗。
        // 攒批时批内间隔是 µs 量级（内核时间戳同样能分辨），半周期阈值在各档都成立。
        double thr = period_ms * 0.5;
        int nb = gap.empty() ? (nrx > 0 ? 1 : 0) : 1;
        for (double g : gap) if (g > thr) nb++;
        // 逐帧投递的判据：批数接近帧数（每批约 1 帧）
        if (nrx >= 3 && nb < nrx * 3 / 4) all_per_frame = false;

        std::printf("%5.0fH %6d %6d | %8.3f %8.3f %8.3f | ", p.rate, p.tx, nrx,
                    median(gap), pct(gap, 0.9),
                    gap.empty() ? 0.0 : *std::max_element(gap.begin(), gap.end()));
        if (paired && !lat.empty())
            std::printf("%7.3f %7.3f %6.3f | %6d\n", median(lat), pct(lat, 0.9),
                        *std::max_element(lat.begin(), lat.end()), nb);
        else
            std::printf("%7s %7s %6s | %6d\n", "-", "-", "(数量不齐)", nb);
    }

    std::printf("\n=== 判读 ===\n");
    if (all_per_frame) {
        std::printf("✓ 各档批数 ≈ 帧数（每批 1 帧），到达间隔 ≈ 发送周期 ⇒ 【逐帧投递】。\n");
        std::printf("  出厂固件的 100ms 攒包已消除，闭环反馈率上限 = 电机应答率上限。\n");
        std::printf("  「配对延迟」列就是真实的请求→应答全链路延迟（含总线+固件+内核）。\n");
    } else {
        std::printf("✗ 存在批数远小于帧数的档位 ⇒ 仍有攒批行为，需检查 slcand 配置\n");
        std::printf("  或串口驱动缓冲（例如 setserial low_latency）。\n");
    }
    std::printf("CAN 错误帧: %d\n", g_canerr.load());

    if (!csv_path.empty()) {
        int n = g_log_n.load();
        FILE* fp = std::fopen(csv_path.c_str(), "w");
        if (!fp) std::printf("\n!! 无法写入 %s\n", csv_path.c_str());
        else {
            std::fprintf(fp, "rate_hz,host_t_us,kts_ns,can_id\n");
            for (int i = 0; i < n; ++i)
                std::fprintf(fp, "%.0f,%lld,%llu,0x%03X\n", g_log[i].rate,
                             g_log[i].t_us, (unsigned long long)g_log[i].kts, g_log[i].can_id);
            std::fclose(fp);
            std::printf("\n逐帧日志: %d 行 -> %s\n", n, csv_path.c_str());
            if (n >= kLogMax) std::printf("!! 日志缓冲已满(%d)，后续帧未记录\n", kLogMax);
        }
    }

    ::close(s);
    return 0;
}
