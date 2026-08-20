// dm_rated_sc.cpp —— 额定转速循环换向演示（DM-J10422P-2EC，电机已固定台架）
//
// 目的：输出轴以额定转速（100rpm = 10.47 rad/s）双向巡航，循环换向，
//       换向加速度逐循环升级（默认 5→10→20→30 rad/s²），感受性能上限。
//
// 方法：MIT 纯阻尼（kp=0, tau=0 ⇒ T = kd·(dq_des−ω)），kd=5.0（编码满量程）。
//   - 稳态差 T_f/kd ≈ 1~2 rad/s 由慢速积分修正（trim）补上，使实测 ω 达到目标；
//     trim 只在稳定段更新，测量段冻结，保证摩擦测量窗口指令恒定。
//   - dq_des 全程斜坡限速（当前循环的 accel），换向平滑过零，无指令阶跃。
//   - 48V 反电动势天花板 ≈ 13.1 rad/s ⇒ 指令钳位 13.5，物理上无法超速。
//
// 保护：ERR≥8 / ERR==0 意外失能 / |ω|>14.5 / |tau|>30 Nm / 温度>70℃ /
//       发送失败 / 计划时长+20s 兜底。Ctrl-C ×1 柔停，×2 立即失能。
// 只发 0xFB/0xFC/0xFD/MIT；绝无 0x55/0xAA/0xFE。
//
// 用法: dm_rated_sc [--target 10.47] [--cruise 3] [--cycles 4]
//                   [--accels 5,10,20,30] [--if can0] [--csv 路径] [--list]

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <string>
#include <vector>

#include "dm_sc.hpp"

namespace {

volatile std::sig_atomic_t g_stop = 0;
void on_sigint(int) { g_stop++; }

constexpr int    kPeriodMs   = 10;      // 100Hz
constexpr float  kKd         = 5.0f;    // MIT kd 编码满量程
constexpr float  kDqClamp    = 13.5f;   // 指令钳位（略高于 13.1 电压天花板）
constexpr float  kTrimMax    = 3.5f;    // 摩擦补偿积分上限
constexpr float  kKi         = 3.0f;    // trim 积分增益 (1/s)
constexpr float  kVelAbort   = 14.5f;
constexpr float  kTauAbort   = 30.0f;   // 换向峰值预期 ≤15 Nm
constexpr int    kTempAbort  = 70;
constexpr double kSettleS    = 1.0;     // 斜坡到位后稳定段（trim 在此收敛）
constexpr float  kWindAccel  = 8.0f;    // 收尾降速
constexpr int    kLogMax     = 20000;   // 100Hz × 200s

struct Sample {
    float t, pos, travel, vel, tau, dqc, kd;
    uint64_t kts;
    uint8_t tmos, trotor, phase;
    int16_t seg;
};

struct Fb { float pos, vel, tau; uint8_t err, tmos, trotor; };

bool decode_fb(const can_frame& fr, Fb* o) {
    if (fr.can_dlc < 8) return false;
    uint16_t q_u  = ((uint16_t)fr.data[1] << 8) | fr.data[2];
    uint16_t dq_u = ((uint16_t)fr.data[3] << 4) | (fr.data[4] >> 4);
    uint16_t t_u  = (((uint16_t)fr.data[4] & 0x0F) << 8) | fr.data[5];
    o->err    = fr.data[0] >> 4;
    o->pos    = dmsc::uint_to_float(q_u,  -dmsc::kPMax, dmsc::kPMax, 16);
    o->vel    = dmsc::uint_to_float(dq_u, -dmsc::kVMax, dmsc::kVMax, 12);
    o->tau    = dmsc::uint_to_float(t_u,  -dmsc::kTMax, dmsc::kTMax, 12);
    o->tmos   = fr.data[6];
    o->trotor = fr.data[7];
    return true;
}

double now_s() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

std::vector<float> parse_list(const char* s) {
    std::vector<float> v;
    std::string cur;
    for (const char* p = s;; ++p) {
        if (*p == ',' || *p == '\0') {
            if (!cur.empty()) v.push_back((float)std::atof(cur.c_str()));
            cur.clear();
            if (*p == '\0') break;
        } else cur += *p;
    }
    return v;
}

}  // namespace

int main(int argc, char** argv) {
    float  target  = 10.47f;   // 额定 100rpm
    double cruise  = 3.0;
    int    cycles  = 4;
    std::vector<float> accels = {5, 10, 20, 30};
    std::string ifname = "can0", csv_path;
    bool list_only = false;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if      (a == "--target" && i + 1 < argc) target = (float)std::atof(argv[++i]);
        else if (a == "--cruise" && i + 1 < argc) cruise = std::atof(argv[++i]);
        else if (a == "--cycles" && i + 1 < argc) cycles = std::atoi(argv[++i]);
        else if (a == "--accels" && i + 1 < argc) accels = parse_list(argv[++i]);
        else if (a == "--if"     && i + 1 < argc) ifname = argv[++i];
        else if (a == "--csv"    && i + 1 < argc) csv_path = argv[++i];
        else if (a == "--list") list_only = true;
        else {
            std::printf("用法: dm_rated_sc [--target 10.47] [--cruise 3] [--cycles 4]\n"
                        "                  [--accels 5,10,20,30] [--if can0] [--csv 路径] [--list]\n");
            return a == "--help" ? 0 : 1;
        }
    }
    target = clampf(target, 2.0f, 11.0f);
    cruise = cruise < 1.0 ? 1.0 : (cruise > 8.0 ? 8.0 : cruise);
    cycles = cycles < 1 ? 1 : (cycles > 8 ? 8 : cycles);
    if (accels.empty()) accels = {5, 10, 20, 30};
    for (float& a : accels) a = clampf(a, 2.0f, 30.0f);

    const int nseg = 2 * cycles;   // 每循环 +target / −target 两段
    double plan_s = 0.5;
    for (int i = 0; i < nseg; ++i) {
        float a = accels[(i / 2) < (int)accels.size() ? (i / 2) : accels.size() - 1];
        float span = (i == 0 ? 1.0f : 2.0f) * (target + 2.0f);   // trim 余量估 2
        plan_s += span / a + kSettleS + cruise;
    }
    plan_s += (target + 2.0f) / kWindAccel + 0.5;
    const double budget_s = plan_s + 20.0;

    std::printf("=== 额定转速循环换向（DM-J10422P）：%d 循环 × ±%.2f rad/s (%.0f rpm) ===\n",
                cycles, target, target * 60 / (2 * M_PI));
    std::printf("kd=%.1f  巡航 %.1fs/向  换向加速度: ", kKd, cruise);
    for (int c = 0; c < cycles; ++c)
        std::printf("%.0f%s", accels[c < (int)accels.size() ? c : accels.size() - 1],
                    c + 1 < cycles ? "→" : " rad/s²\n");
    std::printf("预计 %.0fs（兜底 %.0fs）  指令钳位 %.1f  超速阈 %.1f  力矩阈 %.0f Nm\n",
                plan_s, budget_s, kDqClamp, kVelAbort, kTauAbort);
    if (list_only) { std::printf("(--list：不发帧，结束)\n"); return 0; }

    std::signal(SIGINT, on_sigint);
    std::string err;
    int s = dmsc::open_can(ifname.c_str(), true, &err);
    if (s < 0) { std::printf("%s\n", err.c_str()); return 1; }

    std::vector<Sample> log;
    log.reserve(kLogMax);

    dmsc::send_cmd(s, 0xFB);
    usleep(20 * 1000);
    dmsc::send_cmd(s, 0xFC);

    enum Phase { P_GRACE, P_RAMP, P_SETTLE, P_MEASURE, P_WIND, P_BRAKE, P_DONE } ph = P_GRACE;
    double t0 = now_s(), ph_t0 = t0, last_print = 0;
    int seg = 0;
    float dq_base = 0, trim = 0;
    bool aborted = false;
    const char* why = nullptr;
    Fb fb{};
    bool have_fb = false;

    // 测量窗累计
    double ms_t0 = 0;
    float last_pos = 0, travel_seg = 0, travel_all = 0;
    bool have_pos = false, have_pos_all = false;
    float last_pos_all = 0;
    double tau_sum = 0;
    int tau_n = 0;
    float tau_peak = 0;

    struct Snap { int seg; float tgt, accel, w, fric, tau_mean, trim; uint8_t tmos, trotor; };
    std::vector<Snap> snaps;

    auto seg_tgt = [&](int i) { return (i % 2 == 0 ? +target : -target); };
    auto seg_acc = [&](int i) {
        int c = i / 2;
        return accels[c < (int)accels.size() ? c : accels.size() - 1];
    };

    while (ph != P_DONE) {
        double el = now_s() - t0;
        double pel = now_s() - ph_t0;
        if (el > budget_s) { why = "总时长兜底"; aborted = true; break; }
        if (g_stop == 1 && ph < P_WIND) { std::printf(">>> Ctrl-C，柔停\n"); ph = P_WIND; ph_t0 = now_s(); }
        if (g_stop >= 2) { why = "Ctrl-C x2"; aborted = true; break; }

        const float dt = kPeriodMs / 1000.0f;
        float tgt_eff = 0;
        if (ph == P_RAMP || ph == P_SETTLE || ph == P_MEASURE) {
            float tg = seg_tgt(seg);
            tgt_eff = clampf(tg + (tg > 0 ? trim : -trim), -kDqClamp, kDqClamp);
        }

        switch (ph) {
            case P_GRACE:
                if (pel >= 0.3) { ph = P_RAMP; ph_t0 = now_s(); }
                break;
            case P_RAMP: {
                float a = seg_acc(seg) * dt;
                if (dq_base < tgt_eff) dq_base = (dq_base + a < tgt_eff) ? dq_base + a : tgt_eff;
                else                   dq_base = (dq_base - a > tgt_eff) ? dq_base - a : tgt_eff;
                if (dq_base == tgt_eff) { ph = P_SETTLE; ph_t0 = now_s(); }
                break;
            }
            case P_SETTLE: {
                // trim 只在此更新（测量段冻结）；dq_base 以本段 accel 跟随 tgt_eff
                if (have_fb) {
                    float e = std::fabs(seg_tgt(seg)) - std::fabs(fb.vel);
                    trim = clampf(trim + kKi * e * dt, 0.0f, kTrimMax);
                }
                float a = seg_acc(seg) * dt;
                if (dq_base < tgt_eff) dq_base = (dq_base + a < tgt_eff) ? dq_base + a : tgt_eff;
                else                   dq_base = (dq_base - a > tgt_eff) ? dq_base - a : tgt_eff;
                if (pel >= kSettleS) {
                    ph = P_MEASURE; ph_t0 = now_s();
                    ms_t0 = now_s(); have_pos = false; travel_seg = 0;
                    tau_sum = 0; tau_n = 0;
                }
                break;
            }
            case P_MEASURE:
                if (pel >= cruise) {
                    double dur = now_s() - ms_t0;
                    float w = (dur > 0.5) ? travel_seg / (float)dur : 0.0f;
                    float dqq = dmsc::quantized(dq_base, -dmsc::kVMax, dmsc::kVMax, 12);
                    Snap sn{seg, seg_tgt(seg), seg_acc(seg), w, std::fabs(kKd * (dqq - w)),
                            tau_n ? (float)(tau_sum / tau_n) : 0.0f, trim, fb.tmos, fb.trotor};
                    snaps.push_back(sn);
                    std::printf(">>> 段#%d 循环%d dq=%+.2f a=%.0f: 实测 %+.3f rad/s (%.1f rpm) "
                                "摩擦反解 %.2f / 遥测 %+.2f Nm  trim=%.2f  MOS=%d℃ 线圈=%d℃\n",
                                seg, seg / 2 + 1, seg_tgt(seg), seg_acc(seg), w,
                                std::fabs(w) * 60 / (2 * M_PI), sn.fric, sn.tau_mean, trim,
                                fb.tmos, fb.trotor);
                    if (++seg >= nseg) { ph = P_WIND; ph_t0 = now_s(); }
                    else               { ph = P_RAMP; ph_t0 = now_s(); }
                }
                break;
            case P_WIND: {
                float a = kWindAccel * dt;
                if (dq_base > a)       dq_base -= a;
                else if (dq_base < -a) dq_base += a;
                else { dq_base = 0; ph = P_BRAKE; ph_t0 = now_s(); }
                break;
            }
            case P_BRAKE:
                if (pel >= 0.5) ph = P_DONE;
                break;
            default: break;
        }
        if (ph == P_DONE) break;

        if (!dmsc::send_mit(s, 0, dq_base, 0, kKd, 0)) { why = "发送失败"; aborted = true; break; }

        // 收本周期反馈
        double dl = now_s() + kPeriodMs / 1000.0;
        can_frame fr{};
        uint64_t kts = 0;
        while (now_s() < dl) {
            int left = (int)((dl - now_s()) * 1000);
            if (left < 1) left = 1;
            if (dmsc::recv_frame(s, &fr, &kts, left) != 1) continue;
            if (fr.can_id & CAN_ERR_FLAG) { std::printf("  [总线错误帧 0x%08X]\n", fr.can_id); continue; }
            if (!decode_fb(fr, &fb)) continue;
            have_fb = true;

            if (!have_pos_all) { last_pos_all = fb.pos; have_pos_all = true; }
            float dall = fb.pos - last_pos_all;
            if (dall >  dmsc::kPMax) dall -= 2 * dmsc::kPMax;
            if (dall < -dmsc::kPMax) dall += 2 * dmsc::kPMax;
            travel_all += dall;
            last_pos_all = fb.pos;

            if (ph == P_MEASURE) {
                if (!have_pos) { last_pos = fb.pos; have_pos = true; }
                float d = fb.pos - last_pos;
                if (d >  dmsc::kPMax) d -= 2 * dmsc::kPMax;
                if (d < -dmsc::kPMax) d += 2 * dmsc::kPMax;
                travel_seg += d;
                last_pos = fb.pos;
                tau_sum += fb.tau;
                tau_n++;
            }
            if (std::fabs(fb.tau) > std::fabs(tau_peak)) tau_peak = fb.tau;

            if ((int)log.size() < kLogMax) {
                Sample sm{};
                sm.t = (float)el; sm.kts = kts;
                sm.pos = fb.pos; sm.travel = travel_all; sm.vel = fb.vel; sm.tau = fb.tau;
                sm.dqc = dq_base; sm.kd = kKd;
                sm.tmos = fb.tmos; sm.trotor = fb.trotor;
                sm.seg = (int16_t)seg;
                sm.phase = (ph == P_MEASURE) ? 1 : 0;
                log.push_back(sm);
            }
        }

        if (have_fb && el > 0.3) {
            if (fb.err >= 8)  { why = dmsc::err_name(fb.err); aborted = true; break; }
            if (fb.err == 0)  { why = "意外失能"; aborted = true; break; }
            if (std::fabs(fb.vel) > kVelAbort) { why = "超速"; aborted = true; break; }
            if (std::fabs(fb.tau) > kTauAbort) { why = "力矩超限"; aborted = true; break; }
            if (fb.tmos > kTempAbort || fb.trotor > kTempAbort) { why = "过温"; aborted = true; break; }
        }

        if (el - last_print > 1.0) {
            last_print = el;
            const char* pn = ph == P_RAMP ? "斜坡" : ph == P_SETTLE ? "稳定" :
                             ph == P_MEASURE ? "巡航" : ph == P_WIND ? "降速" :
                             ph == P_BRAKE ? "制动" : "宽限";
            std::printf("  t=%5.1fs 段#%d[%s] dq=%+6.2f | ERR=%X vel=%+7.3f (%.0frpm) "
                        "tau=%+6.2f MOS=%d℃ 线圈=%d℃\n",
                        el, seg, pn, dq_base, fb.err, fb.vel,
                        std::fabs(fb.vel) * 60 / (2 * M_PI), fb.tau, fb.tmos, fb.trotor);
        }
    }

    if (aborted && why) std::printf("\n!!! 中止：%s（ERR=%X vel=%.3f tau=%.2f）\n", why, fb.err, fb.vel, fb.tau);

    for (int i = 0; i < 5; ++i) { dmsc::send_cmd(s, 0xFD); usleep(10 * 1000); }
    std::printf(">>> 已失能\n");

    std::printf("\n=== 汇总 ===\n峰值 |tau|: %.2f Nm   累计行程: %+.1f rad   记录 %zu 帧\n",
                std::fabs(tau_peak), travel_all, log.size());
    if (!snaps.empty()) {
        std::printf("段#  循环  目标rad/s  accel  实测rad/s   rpm   摩擦Nm  trim   温度℃\n");
        for (const Snap& sn : snaps)
            std::printf("%3d  %3d   %+7.2f   %4.0f   %+7.3f  %5.1f  %6.2f  %.2f  %d/%d\n",
                        sn.seg, sn.seg / 2 + 1, sn.tgt, sn.accel, sn.w,
                        std::fabs(sn.w) * 60 / (2 * M_PI), sn.fric, sn.trim, sn.tmos, sn.trotor);
        const Snap& lastp = snaps.back();
        if (lastp.trim >= kTrimMax - 0.05f && std::fabs(lastp.w) < 0.95f * target)
            std::printf("注：trim 已饱和仍未达目标 ⇒ 逼近 48V 反电动势天花板（≈13.1 rad/s 指令）。\n");
    }

    if (!csv_path.empty() && !log.empty()) {
        FILE* f = std::fopen(csv_path.c_str(), "w");
        if (f) {
            std::fprintf(f, "t_s,kts_ns,pos_rad,travel_rad,vel_rad_s,tau_Nm,dq_cmd,kd_cmd,t_mos,t_rotor,seg,phase\n");
            for (const Sample& sm : log)
                std::fprintf(f, "%.3f,%llu,%.5f,%.5f,%.4f,%.4f,%.4f,%.2f,%d,%d,%d,%d\n",
                             sm.t, (unsigned long long)sm.kts, sm.pos, sm.travel, sm.vel, sm.tau,
                             sm.dqc, sm.kd, sm.tmos, sm.trotor, sm.seg, sm.phase);
            std::fclose(f);
            std::printf("逐帧日志: %zu 行 -> %s\n", log.size(), csv_path.c_str());
        }
    }
    ::close(s);
    return aborted ? 2 : 0;
}
