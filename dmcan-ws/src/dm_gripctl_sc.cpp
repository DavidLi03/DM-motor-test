// dm_gripctl_sc.cpp —— 两指夹爪正式开合控制（DM-J4310-2EC，电机已固定台架）
//
// 坐标约定（2026-08-12 dm_grip_sc 标定 + 点动实证）：
//   +向 = 闭合 ⇒ 限位A=全闭、限位B=全开；
//   软件零点：B(全开)=0，正值=闭合量，量程 [0, span]（span 实测 ≈1.4916 rad = 85.4°，
//   读 grip_cal.json 的 span_rad，读不到则用内置默认值）。
// 安全红线禁发 0xFE ⇒ 不存在硬件零点，【每次运行都先 re-home】：
//   朝 B（全开）方向低速推到堵转即为零点 —— 该方向爪内有物也不会误夹。
//
// 动作（四选一）：
//   --open        全开（re-home 后即零点，原地即到位）
//   --close       全闭（伺服到 span，末端顶到限位A由堵转判稳收尾）
//   --deg X / --rad X   闭合到指定角度（自动钳到 [0, span]）
//   --grip F      限力夹持：+向闭合直到堵转（夹到物体或限位），
//                 以推力 F Nm（= kd·dq，钳到 [0.2, 1.0]）保持 --hold 秒后卸力
//
// 方法：MIT 纯阻尼（kp=0, tau=0 ⇒ T = kd·(dq_des−ω)），位置→速度伺服带
//   防粘滞速度下限与到位死区。最大推力 kd·dq 硬上限 1.0 Nm（超限自动压 kd）。
//   防粘滞下限的取法（二选一，自动）：
//     * grip_fric.json 在场（fit_gripfric.py 产出）⇒ 摩擦前馈
//       dq_ff = 1.10·T̂_f(方向,|ω|)/kd（静区取挣脱值 Ts_bk，动区取 Stribeck 模型）；
//     * 不在场 ⇒ 回落固定下限 kGotoMin=0.3 rad/s（与旧版行为完全一致）。
//
// 保护：ERR≥8 / 意外失能 / |ω|>3 / |tau|>2.5 Nm / 温度>70℃ / 发送失败 /
//       行程越界兜底 / 各阶段超时 / 总时长兜底。Ctrl-C ×1 柔停，×2 立即失能。
// 只发 0xFB/0xFC/0xFD/MIT；绝无 0x55/0xAA/0xFE。
//
// 用法: dm_gripctl_sc (--open|--close|--deg 40|--rad 0.7|--grip 0.6)
//                     [--hold 3] [--dq 0.4] [--kd 1.5] [--tol 0.02] [--if can0]
//                     [--csv 路径] [--cal grip_cal.json] [--fric grip_fric.json] [--list]

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <ctime>
#include <string>
#include <vector>

#include "dm_sc.hpp"

namespace {

volatile std::sig_atomic_t g_stop = 0;
void on_sigint(int) { g_stop++; }

constexpr int    kPeriodMs   = 10;      // 100Hz
constexpr float  kAccel      = 2.0f;    // dq 斜坡 (rad/s²)
constexpr float  kVelAbort   = 3.0f;
constexpr float  kTauAbort   = 2.5f;
constexpr int    kTempAbort  = 70;
constexpr float  kTravelGuard= 2.2f;    // 找零前距起点行程兜底（>126°）
constexpr float  kXMargin    = 0.30f;   // 找零后 x 允许越界量 (rad)
constexpr float  kStallVel   = 0.05f;   // 堵转判据：|ω| 阈值
constexpr float  kStallTravel= 0.01f;   // 堵转判据：窗口内行程增量阈值 (rad)
constexpr int    kStallN     = 30;      // 连续 30 周期 = 300ms
constexpr double kHoldoffS   = 0.35;    // 阶段起步判稳保持期（避开起动瞬态）
constexpr float  kGotoGain   = 2.0f;    // 位置→速度增益 (1/s)
constexpr float  kGotoMin    = 0.30f;   // 防粘滞速度下限
constexpr float  kGotoTol    = 0.02f;   // 到位死区 (rad)
constexpr float  kPushCap    = 1.0f;    // kd·dq 推力硬上限 (Nm)
constexpr double kHomeTmoS   = 15.0;    // 找零阶段超时
constexpr double kMoveTmoS   = 20.0;    // 伺服阶段超时
constexpr float  kSpanDflt   = 1.4916f; // 2026-08-12 实测行程（cal 文件缺失时用）
constexpr int    kLogMax     = 18000;   // 100Hz × 180s

struct Sample {
    float t, pos, x, vel, tau, dqc, kd;
    uint64_t kts;
    uint8_t tmos, trotor, phase;
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

// ---- grip_fric.json 摩擦模型（铺平键由 fit_gripfric.py 写出；同 dm_gripfric_sc）----
bool jnum(const char* buf, const char* key, float* out) {
    std::string pat = std::string("\"") + key + "\"";
    const char* p = std::strstr(buf, pat.c_str());
    if (!p) return false;
    p = std::strchr(p + pat.size(), ':');
    if (!p) return false;
    char* end = nullptr;
    double v = std::strtod(p + 1, &end);
    if (end == p + 1) return false;
    *out = (float)v;
    return true;
}

struct FricModel {
    bool ok = false;
    float Tc[2], Ts[2], ws[2], b[2], Tsbk[2];   // [0]=闭合(+) [1]=张开(−)
    float eval(int di, float w) const {          // w = |ω|
        float r = w / ws[di];
        return Tc[di] + (Ts[di] - Tc[di]) * std::exp(-r * r) + b[di] * w;
    }
};

FricModel load_fric(const std::string& path) {
    FricModel m;
    FILE* f = std::fopen(path.c_str(), "r");
    if (!f) return m;
    char buf[16384];
    size_t n = std::fread(buf, 1, sizeof(buf) - 1, f);
    std::fclose(f);
    buf[n] = 0;
    const char* sfx[2] = {"close", "open"};
    for (int i = 0; i < 2; ++i) {
        char k[32];
        std::snprintf(k, sizeof(k), "ff_Tc_%s", sfx[i]);
        if (!jnum(buf, k, &m.Tc[i])) return m;
        std::snprintf(k, sizeof(k), "ff_Ts_%s", sfx[i]);
        if (!jnum(buf, k, &m.Ts[i])) return m;
        std::snprintf(k, sizeof(k), "ff_ws_%s", sfx[i]);
        if (!jnum(buf, k, &m.ws[i])) return m;
        std::snprintf(k, sizeof(k), "ff_b_%s", sfx[i]);
        if (!jnum(buf, k, &m.b[i])) return m;
        std::snprintf(k, sizeof(k), "ff_Ts_bk_%s", sfx[i]);
        if (!jnum(buf, k, &m.Tsbk[i])) return m;
        // 合理域检查：出域 ⇒ 判未加载（回落旧行为）
        if (m.Tc[i] < 0.02f || m.Tc[i] > 1.0f) return m;
        if (m.Ts[i] < m.Tc[i] - 0.01f || m.Ts[i] > 1.0f) return m;
        if (m.ws[i] < 0.005f || m.ws[i] > 5.0f) return m;
        if (m.b[i] < 0.0f || m.b[i] > 0.3f) return m;
        if (m.Tsbk[i] < 0.05f || m.Tsbk[i] > 1.0f) return m;
    }
    m.ok = true;
    return m;
}

// 从 grip_cal.json 里抠 span_rad（无第三方库的极简解析，容错到默认值）
float load_span(const std::string& path) {
    FILE* f = std::fopen(path.c_str(), "r");
    if (!f) return 0;
    char buf[4096];
    size_t n = std::fread(buf, 1, sizeof(buf) - 1, f);
    std::fclose(f);
    buf[n] = 0;
    const char* p = std::strstr(buf, "\"span_rad\"");
    if (!p) return 0;
    p = std::strchr(p, ':');
    if (!p) return 0;
    return (float)std::strtod(p + 1, nullptr);
}

}  // namespace

int main(int argc, char** argv) {
    float dq_max = 0.4f, kd = 1.5f;
    float grip_push = 0, hold_s = 3.0f;
    float tgt = -1;                 // 闭合量目标 (rad)；<0 = 未指定
    float tol = kGotoTol;
    bool  grip_mode = false, list_only = false;
    std::string ifname = "can0", csv_path, cal_path = "grip_cal.json",
                fric_path = "grip_fric.json";

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if      (a == "--open")  tgt = 0;
        else if (a == "--close") tgt = 1e9f;   // 占位，加载 span 后钳到全闭
        else if (a == "--deg"  && i + 1 < argc) tgt = (float)std::atof(argv[++i]) * (float)M_PI / 180.0f;
        else if (a == "--rad"  && i + 1 < argc) tgt = (float)std::atof(argv[++i]);
        else if (a == "--grip" && i + 1 < argc) { grip_mode = true; grip_push = (float)std::atof(argv[++i]); }
        else if (a == "--hold" && i + 1 < argc) hold_s = (float)std::atof(argv[++i]);
        else if (a == "--dq"   && i + 1 < argc) dq_max = (float)std::atof(argv[++i]);
        else if (a == "--kd"   && i + 1 < argc) kd = (float)std::atof(argv[++i]);
        else if (a == "--tol"  && i + 1 < argc) tol = (float)std::atof(argv[++i]);
        else if (a == "--if"   && i + 1 < argc) ifname = argv[++i];
        else if (a == "--csv"  && i + 1 < argc) csv_path = argv[++i];
        else if (a == "--cal"  && i + 1 < argc) cal_path = argv[++i];
        else if (a == "--fric" && i + 1 < argc) fric_path = argv[++i];
        else if (a == "--list") list_only = true;
        else {
            std::printf("用法: dm_gripctl_sc (--open|--close|--deg 40|--rad 0.7|--grip 0.6)\n"
                        "                    [--hold 3] [--dq 0.4] [--kd 1.5] [--tol 0.02]\n"
                        "                    [--if can0] [--csv 路径] [--cal grip_cal.json]\n"
                        "                    [--fric grip_fric.json] [--list]\n");
            return a == "--help" ? 0 : 1;
        }
    }
    if (tgt < 0 && !grip_mode) {
        std::printf("请指定动作：--open / --close / --deg X / --rad X / --grip F\n");
        return 1;
    }
    if (grip_mode && tgt >= 0) {
        std::printf("--grip 与 --open/--close/--deg/--rad 互斥（夹持自带闭合动作）\n");
        return 1;
    }

    dq_max = clampf(dq_max, 0.1f, 0.8f);
    kd     = clampf(kd, 0.5f, 2.0f);
    if (kd * dq_max > kPushCap) kd = kPushCap / dq_max;   // 推力硬上限
    hold_s = clampf(hold_s, 0.5f, 30.0f);
    tol    = clampf(tol, 0.005f, 0.05f);

    float span = load_span(cal_path);
    bool span_from_cal = (span > 0.5f && span < 2.5f);
    if (!span_from_cal) span = kSpanDflt;

    FricModel fric = load_fric(fric_path);

    float dq_hold = 0;
    if (grip_mode) {
        grip_push = clampf(grip_push, 0.2f, kPushCap);
        dq_hold = grip_push / kd;                          // 堵转下持续推力 = kd·dq
        if (dq_hold > dq_max) { kd = grip_push / dq_max; dq_hold = dq_max; }
        tgt = span + 0.2f;      // 目标设在限位外，保证一路推到堵转（物体或限位A）
    } else {
        tgt = clampf(tgt, 0.0f, span);
    }

    const double budget_s = 75.0 + hold_s;
    std::printf("=== 夹爪开合控制（DM-J4310，零点=B全开，+=闭合）===\n");
    std::printf("行程 span=%.4f rad=%.1f°（%s）  dq≤%.2f  kd=%.2f  最大推力 %.2f Nm\n",
                span, span * 180 / M_PI, span_from_cal ? cal_path.c_str() : "内置默认",
                dq_max, kd, kd * dq_max);
    if (fric.ok)
        std::printf("摩擦前馈：%s（闭合 Tc=%.3f Ts=%.3f | 张开 Tc=%.3f Ts=%.3f | "
                    "挣脱 %.3f/%.3f Nm）\n", fric_path.c_str(),
                    fric.Tc[0], fric.Ts[0], fric.Tc[1], fric.Ts[1],
                    fric.Tsbk[0], fric.Tsbk[1]);
    else
        std::printf("摩擦前馈：未加载 %s，用固定防粘滞下限 %.2f rad/s（旧行为）\n",
                    fric_path.c_str(), kGotoMin);
    if (grip_mode)
        std::printf("动作：限力夹持 —— 闭合至堵转后以 %.2f Nm 保持 %.1fs\n", kd * dq_hold, hold_s);
    else
        std::printf("动作：闭合到 %.4f rad = %.1f°（0=全开，%.1f°=全闭）\n",
                    tgt, tgt * 180 / M_PI, span * 180 / M_PI);
    std::printf("流程：re-home 朝B(全开)找零 → 伺服到目标 → %s失能\n", grip_mode ? "保持 → " : "");
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

    enum Phase { P_GRACE, P_HOME, P_RELAX, P_MOVE, P_GRIP, P_BRAKE, P_DONE } ph = P_GRACE;
    double t0 = now_s(), ph_t0 = t0, last_print = 0;
    float dq_base = 0;
    bool aborted = false;
    const char* why = nullptr;
    Fb fb{};
    bool have_fb = false;

    // 解卷绕行程坐标（会话起点=0）；找零后 x = travel − home_travel 即闭合量
    float travel = 0, last_pos = 0;
    bool have_pos = false;
    float home_travel = 0;
    bool homed = false;
    // 堵转判稳
    int stall_n = 0;
    float stall_ref_travel = 0;
    // 结果
    bool reached = false, obstructed = false, grasped = false;
    float x_final = 0, grasp_x = 0, tau_peak = 0;

    while (ph != P_DONE) {
        double el = now_s() - t0;
        double pel = now_s() - ph_t0;
        if (el > budget_s) { why = "总时长兜底"; aborted = true; break; }
        if (g_stop == 1 && ph < P_BRAKE) { std::printf(">>> Ctrl-C，柔停\n"); ph = P_BRAKE; ph_t0 = now_s(); }
        if (g_stop >= 2) { why = "Ctrl-C x2"; aborted = true; break; }

        const float dt = kPeriodMs / 1000.0f;
        float x = travel - home_travel;   // 找零前无意义，仅找零后使用

        // 堵转判稳（P_HOME / P_MOVE 共用）：|ω| 小且行程不再增长，连续 kStallN 周期
        bool stalled = false;
        if ((ph == P_HOME || ph == P_MOVE) && pel > kHoldoffS && have_fb) {
            if (std::fabs(fb.vel) < kStallVel &&
                std::fabs(travel - stall_ref_travel) < kStallTravel) {
                stall_n++;
            } else {
                stall_n = 0;
                stall_ref_travel = travel;
            }
            stalled = (stall_n >= kStallN);
        }

        switch (ph) {
            case P_GRACE:
                if (pel >= 0.3) {
                    ph = P_HOME; ph_t0 = now_s();
                    stall_n = 0; stall_ref_travel = travel;
                }
                break;
            case P_HOME: {
                // 朝 B（全开，−向）低速推到堵转 ⇒ 零点。方向安全：爪内有物也不会误夹。
                float want = -dq_max;
                float a = kAccel * dt;
                if (dq_base < want) dq_base = (dq_base + a < want) ? dq_base + a : want;
                else                dq_base = (dq_base - a > want) ? dq_base - a : want;
                if (stalled) {
                    home_travel = travel; homed = true;
                    std::printf(">>> 找零完成：限位B(全开)=0（原始pos=%+.4f）\n", fb.pos);
                    ph = P_RELAX; ph_t0 = now_s();
                }
                if (pel > kHomeTmoS) { why = "找零超时"; aborted = true; }
                break;
            }
            case P_RELAX: {
                float a = kAccel * dt;
                if (dq_base > a)       dq_base -= a;
                else if (dq_base < -a) dq_base += a;
                else dq_base = 0;
                if (dq_base == 0 && pel >= 0.4) {
                    ph = P_MOVE; ph_t0 = now_s();
                    stall_n = 0; stall_ref_travel = travel;
                }
                break;
            }
            case P_MOVE: {
                // 位置→速度伺服（防粘滞下限 + 到位死区），堵转 = 顶到物体/限位。
                // 下限：有摩擦模型 ⇒ 前馈 dq_ff=1.10·T̂_f/kd（静区取挣脱值），否则固定 0.3
                float e = tgt - x;
                float dq_lo = kGotoMin;
                if (fric.ok) {
                    int di = e > 0 ? 0 : 1;
                    float w_now = have_fb ? std::fabs(fb.vel) : 0.0f;
                    float Th = (w_now < kStallVel) ? fric.Tsbk[di] : fric.eval(di, w_now);
                    dq_lo = clampf(1.10f * Th / kd, 0.08f, 0.45f);
                }
                float mag = clampf(kGotoGain * std::fabs(e), dq_lo, dq_max);
                float want = (std::fabs(e) < tol) ? 0.0f : (e > 0 ? mag : -mag);
                float a = kAccel * dt;
                if (dq_base < want) dq_base = (dq_base + a < want) ? dq_base + a : want;
                else                dq_base = (dq_base - a > want) ? dq_base - a : want;

                if (std::fabs(e) < tol && have_fb && std::fabs(fb.vel) < kStallVel) {
                    reached = true;
                    std::printf(">>> 到位：x=%.4f rad=%.1f°（目标 %.4f）\n", x, x * 180 / M_PI, tgt);
                    ph = P_BRAKE; ph_t0 = now_s();
                } else if (stalled) {
                    if (grip_mode) {
                        grasped = true; grasp_x = x;
                        std::printf(">>> 夹持接触：x=%.4f rad=%.1f°，转入 %.2f Nm 保持 %.1fs\n",
                                    x, x * 180 / M_PI, kd * dq_hold, hold_s);
                        ph = P_GRIP; ph_t0 = now_s();
                    } else {
                        obstructed = true;
                        std::printf(">>> 中途受阻（物体/限位）：x=%.4f rad=%.1f°（目标 %.4f），卸力\n",
                                    x, x * 180 / M_PI, tgt);
                        ph = P_BRAKE; ph_t0 = now_s();
                    }
                }
                if (pel > kMoveTmoS) {
                    std::printf(">>> 伺服超时，卸力（x=%.4f 目标 %.4f）\n", x, tgt);
                    ph = P_BRAKE; ph_t0 = now_s();
                }
                break;
            }
            case P_GRIP: {
                // 堵转下恒推：T = kd·(dq_hold−0) = 设定夹持力；持续监测温度/力矩
                float want = dq_hold;
                float a = kAccel * dt;
                if (dq_base < want) dq_base = (dq_base + a < want) ? dq_base + a : want;
                else                dq_base = (dq_base - a > want) ? dq_base - a : want;
                if (pel >= hold_s) { ph = P_BRAKE; ph_t0 = now_s(); }
                break;
            }
            case P_BRAKE: {
                float a = kAccel * dt;
                if (dq_base > a)       dq_base -= a;
                else if (dq_base < -a) dq_base += a;
                else dq_base = 0;
                if (dq_base == 0 && pel >= 0.5) { x_final = x; ph = P_DONE; }
                break;
            }
            default: break;
        }
        if (aborted || ph == P_DONE) break;

        if (!dmsc::send_mit(s, 0, dq_base, 0, kd, 0)) { why = "发送失败"; aborted = true; break; }

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

            if (!have_pos) { last_pos = fb.pos; have_pos = true; stall_ref_travel = 0; }
            float d = fb.pos - last_pos;
            if (d >  dmsc::kPMax) d -= 2 * dmsc::kPMax;
            if (d < -dmsc::kPMax) d += 2 * dmsc::kPMax;
            travel += d;
            last_pos = fb.pos;
            if (std::fabs(fb.tau) > std::fabs(tau_peak)) tau_peak = fb.tau;

            if ((int)log.size() < kLogMax) {
                Sample sm{};
                sm.t = (float)el; sm.kts = kts;
                sm.pos = fb.pos; sm.x = travel - home_travel;
                sm.vel = fb.vel; sm.tau = fb.tau;
                sm.dqc = dq_base; sm.kd = kd;
                sm.tmos = fb.tmos; sm.trotor = fb.trotor;
                sm.phase = (uint8_t)ph;
                log.push_back(sm);
            }
        }

        if (have_fb && el > 0.3) {
            if (fb.err >= 8)  { why = dmsc::err_name(fb.err); aborted = true; break; }
            if (fb.err == 0)  { why = "意外失能"; aborted = true; break; }
            if (std::fabs(fb.vel) > kVelAbort) { why = "超速"; aborted = true; break; }
            if (std::fabs(fb.tau) > kTauAbort) { why = "力矩超限"; aborted = true; break; }
            if (fb.tmos > kTempAbort || fb.trotor > kTempAbort) { why = "过温"; aborted = true; break; }
            float xg = travel - home_travel;
            if (!homed && std::fabs(travel) > kTravelGuard) { why = "找零行程超预期"; aborted = true; break; }
            if (homed && (xg < -kXMargin || xg > span + kXMargin)) { why = "行程越界"; aborted = true; break; }
        }

        if (el - last_print > 1.0) {
            last_print = el;
            const char* pn = ph == P_HOME ? "找零" : ph == P_RELAX ? "卸力" :
                             ph == P_MOVE ? "伺服" : ph == P_GRIP ? "夹持" :
                             ph == P_BRAKE ? "制动" : "宽限";
            std::printf("  t=%5.1fs [%s] dq=%+5.2f | ERR=%X x=%+8.4f vel=%+7.3f "
                        "tau=%+6.3f MOS=%d℃ 线圈=%d℃\n",
                        el, pn, dq_base, fb.err, travel - home_travel, fb.vel, fb.tau,
                        fb.tmos, fb.trotor);
        }
    }

    if (aborted && why) std::printf("\n!!! 中止：%s（ERR=%X x=%.4f vel=%.3f tau=%.3f）\n",
                                    why, fb.err, travel - home_travel, fb.vel, fb.tau);

    for (int i = 0; i < 5; ++i) { dmsc::send_cmd(s, 0xFD); usleep(10 * 1000); }
    std::printf(">>> 已失能\n");

    if (!aborted) {
        float xf = travel - home_travel;
        std::printf("\n=== 结果 ===\n");
        std::printf("终点闭合量 x=%.4f rad = %.1f°（%.0f%% 行程）  峰值|tau|=%.3f Nm\n",
                    xf, xf * 180 / M_PI, 100.0f * xf / span, std::fabs(tau_peak));
        if (grip_mode) {
            if (grasped) {
                // 空夹接触点实测 ≈span±0.002（重复性 ±0.001）⇒ 距全闭 >0.02 rad 即为真实物体
                float gap = span - grasp_x;
                std::printf("夹持：接触点 x=%.4f rad=%.1f°，距全闭 %.2f°（%s），"
                            "已按 %.2f Nm 保持 %.1fs 后卸力。\n"
                            "卸力后电机已失能，靠机构摩擦维持现位；需松开请再跑 --open。\n",
                            grasp_x, grasp_x * 180 / M_PI, gap * 180 / M_PI,
                            gap < 0.02f ? "≈限位A，疑似空夹" : "夹到物体",
                            kd * dq_hold, hold_s);
            }
            else
                std::printf("夹持：未检出接触（超时卸力），请检查。\n");
        } else {
            std::printf("状态：%s\n", reached ? "正常到位" :
                        obstructed ? "中途受阻（爪内有物？）已卸力" : "超时卸力，未到位");
        }
    }

    if (!csv_path.empty() && !log.empty()) {
        FILE* f = std::fopen(csv_path.c_str(), "w");
        if (f) {
            std::fprintf(f, "t_s,kts_ns,pos_rad,travel_rad,vel_rad_s,tau_Nm,dq_cmd,kd_cmd,t_mos,t_rotor,seg,phase\n");
            for (const Sample& sm : log)
                std::fprintf(f, "%.3f,%llu,%.5f,%.5f,%.4f,%.4f,%.4f,%.2f,%d,%d,%d,%d\n",
                             sm.t, (unsigned long long)sm.kts, sm.pos, sm.x, sm.vel, sm.tau,
                             sm.dqc, sm.kd, sm.tmos, sm.trotor, 0, sm.phase);
            std::fclose(f);
            std::printf("逐帧日志: %zu 行 -> %s\n", log.size(), csv_path.c_str());
        }
    }
    ::close(s);
    return aborted ? 2 : 0;
}
