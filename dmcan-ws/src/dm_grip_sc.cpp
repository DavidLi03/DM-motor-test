// dm_grip_sc.cpp —— 两指夹爪行程标定与软件零点（DM-J4310-2EC，电机已固定台架）
//
// 工况：输出轴经 1:1 齿轮驱动单自由度对称开合两指夹爪，标称终极行程 90°。
// 2026-08-12 实测：电机侧行程 1.490 rad = 85.4°（即标称 90° 指电机侧，含公差）；
// 点动实证 +向=闭合 ⇒ 限位A=全闭、限位B=全开。软件零点：B(全开)=0，正值=闭合量。
// re-home 一律朝 B（全开）方向，防止爪内有物时误夹。
// 夹爪两端为机械硬限位 —— 本例程【绝不连续旋转】，全程限位感知。
//
// 方法：MIT 纯阻尼（kp=0, tau=0 ⇒ T = kd·(dq_des−ω)），低速 dq 探测：
//   +方向缓推到堵转（|ω|≈0 且行程不再增长）⇒ 限位 A；
//   −方向同法 ⇒ 限位 B；行程 = A−B，与 45°/90° 两种假设比对；
//   然后回到行程中点，失能。堵转推力上限 = kd·dq（默认 0.4 Nm，钳位 ≤1.0）。
// 软件零点：安全红线禁发 0xFE（存零点），零点只在软件坐标定义 ——
//   标定结果（两限位的原始 pos 与相对行程）写 grip_cal.json，
//   后续例程每次上电以重找限位（re-home）为准，不依赖跨上电的绝对位置。
//
// 保护：ERR≥8 / 意外失能 / |ω|>3 / |tau|>2.5 Nm / 温度>70℃ / 发送失败 /
//       距起点行程>2.2 rad 兜底 / 总时长兜底。Ctrl-C ×1 柔停，×2 立即失能。
// 只发 0xFB/0xFC/0xFD/MIT；绝无 0x55/0xAA/0xFE。
//
// 用法: dm_grip_sc [--dq 0.4] [--kd 1.0] [--if can0] [--csv 路径]
//                  [--cal grip_cal.json] [--list]

#include <csignal>
#include <cstdio>
#include <cstdlib>
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
constexpr float  kTravelGuard= 2.2f;    // 距起点行程兜底（>126°，限位检测失效时护齿轮）
constexpr float  kStallVel   = 0.05f;   // 堵转判据：|ω| 阈值（vel 分辨率 0.024 的 2 LSB）
constexpr float  kStallTravel= 0.01f;   // 堵转判据：窗口内行程增量阈值 (rad)
constexpr int    kStallN     = 30;      // 连续 30 周期 = 300ms
constexpr double kHoldoffS   = 0.25;    // 斜坡到位后的判稳保持期（避开起动瞬态）
constexpr float  kGotoGain   = 2.0f;    // 回中点的位置→速度增益 (1/s)
constexpr float  kGotoMin    = 0.30f;   // 回中防粘滞速度下限：kd·dq 须≥夹爪机构挣脱摩擦
constexpr float  kGotoTol    = 0.02f;   // 回中到位死区 (rad)
constexpr int    kLogMax     = 12000;   // 100Hz × 120s

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

}  // namespace

int main(int argc, char** argv) {
    float dq_probe = 0.4f, kd = 1.0f;
    float jog = 0.0f;   // ≠0 ⇒ 点动模式：朝 sign(jog) 方向移动 |jog| rad 后停（碰限位提前停）
    std::string ifname = "can0", csv_path, cal_path = "grip_cal.json";
    bool list_only = false;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if      (a == "--dq"  && i + 1 < argc) dq_probe = (float)std::atof(argv[++i]);
        else if (a == "--kd"  && i + 1 < argc) kd = (float)std::atof(argv[++i]);
        else if (a == "--jog" && i + 1 < argc) jog = (float)std::atof(argv[++i]);
        else if (a == "--if"  && i + 1 < argc) ifname = argv[++i];
        else if (a == "--csv" && i + 1 < argc) csv_path = argv[++i];
        else if (a == "--cal" && i + 1 < argc) cal_path = argv[++i];
        else if (a == "--list") list_only = true;
        else {
            std::printf("用法: dm_grip_sc [--dq 0.4] [--kd 1.0] [--if can0] [--csv 路径]\n"
                        "                 [--cal grip_cal.json] [--jog ±0.25] [--list]\n");
            return a == "--help" ? 0 : 1;
        }
    }
    dq_probe = clampf(dq_probe, 0.1f, 0.8f);
    kd       = clampf(kd, 0.5f, 2.0f);
    if (kd * dq_probe > 1.0f) kd = 1.0f / dq_probe;   // 堵转推力硬上限 1.0 Nm
    if (jog != 0.0f) jog = clampf(jog, -0.6f, 0.6f);  // 点动幅度 ≤0.6 rad（<半行程）

    const double budget_s = 60.0;
    std::printf("=== 夹爪行程标定（DM-J4310 + 1:1 两指夹爪）===\n");
    std::printf("探测 dq=%.2f rad/s  kd=%.2f  堵转推力 %.2f Nm  行程兜底 %.1f rad  总兜底 %.0fs\n",
                dq_probe, kd, kd * dq_probe, kTravelGuard, budget_s);
    if (jog != 0.0f)
        std::printf("点动模式：朝 %c 方向移动 %.2f rad（≈%.0f°）后停，碰限位提前停。请观察夹爪开/合！\n",
                    jog > 0 ? '+' : '-', std::fabs(jog), std::fabs(jog) * 180 / M_PI);
    else
        std::printf("流程：+方向找限位A → −方向找限位B → 实测行程比对 45°/90° → 回中点失能\n");
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

    enum Phase { P_GRACE, P_SEEK, P_RELAX, P_GOTO, P_BRAKE, P_DONE } ph = P_GRACE;
    double t0 = now_s(), ph_t0 = t0, last_print = 0;
    int   seg = 0;              // 0=找A(+) 1=找B(−) 2=回中点
    float dir = +1.0f;
    float dq_base = 0;
    bool aborted = false;
    const char* why = nullptr;
    Fb fb{};
    bool have_fb = false;

    // 解卷绕行程坐标（起点=0）
    float travel = 0, last_pos = 0;
    bool have_pos = false;
    // 堵转判稳
    int stall_n = 0;
    float stall_ref_travel = 0;
    bool ramp_done = false;
    double ramp_done_t = 0;
    // 标定结果
    float travel_A = 0, travel_B = 0, posraw_A = 0, posraw_B = 0;
    bool  have_A = false, have_B = false;
    float goto_tgt = 0;
    float tau_peak = 0;

    while (ph != P_DONE) {
        double el = now_s() - t0;
        double pel = now_s() - ph_t0;
        if (el > budget_s) { why = "总时长兜底"; aborted = true; break; }
        if (g_stop == 1 && ph < P_BRAKE) { std::printf(">>> Ctrl-C，柔停\n"); ph = P_BRAKE; ph_t0 = now_s(); }
        if (g_stop >= 2) { why = "Ctrl-C x2"; aborted = true; break; }

        const float dt = kPeriodMs / 1000.0f;

        switch (ph) {
            case P_GRACE:
                if (pel >= 0.3) {
                    ph = P_SEEK; ph_t0 = now_s();
                    dir = (jog < 0) ? -1.0f : +1.0f;
                    ramp_done = false; stall_n = 0;
                }
                break;
            case P_SEEK: {
                // 点动模式：走满幅度即卸力收尾（碰限位则由下面的堵转判稳提前停）
                if (jog != 0.0f && std::fabs(travel) >= std::fabs(jog)) {
                    std::printf(">>> 点动到位：travel=%+.4f rad\n", travel);
                    ph = P_RELAX; ph_t0 = now_s();
                    break;
                }
                float tgt = dir * dq_probe;
                float a = kAccel * dt;
                if (dq_base < tgt) dq_base = (dq_base + a < tgt) ? dq_base + a : tgt;
                else               dq_base = (dq_base - a > tgt) ? dq_base - a : tgt;
                if (!ramp_done && dq_base == tgt) { ramp_done = true; ramp_done_t = now_s(); }

                // 堵转判稳：斜坡到位且过保持期后，|ω| 小且行程不再增长，连续 kStallN 周期
                if (ramp_done && now_s() - ramp_done_t > kHoldoffS && have_fb) {
                    if (std::fabs(fb.vel) < kStallVel &&
                        std::fabs(travel - stall_ref_travel) < kStallTravel) {
                        stall_n++;
                    } else {
                        stall_n = 0;
                        stall_ref_travel = travel;
                    }
                    if (stall_n >= kStallN) {
                        if (jog != 0.0f) {
                            std::printf(">>> 点动中碰到限位：travel=%+.4f rad，卸力\n", travel);
                            ph = P_RELAX; ph_t0 = now_s();
                            break;
                        }
                        if (seg == 0) {
                            travel_A = travel; posraw_A = fb.pos; have_A = true;
                            std::printf(">>> 限位A（+向）: travel=%+.4f rad  pos=%+.4f  tau=%+.3f Nm\n",
                                        travel_A, posraw_A, fb.tau);
                        } else {
                            travel_B = travel; posraw_B = fb.pos; have_B = true;
                            std::printf(">>> 限位B（−向）: travel=%+.4f rad  pos=%+.4f  tau=%+.3f Nm\n",
                                        travel_B, posraw_B, fb.tau);
                        }
                        ph = P_RELAX; ph_t0 = now_s();
                    }
                }
                break;
            }
            case P_RELAX: {
                float a = kAccel * dt;
                if (dq_base > a)       dq_base -= a;
                else if (dq_base < -a) dq_base += a;
                else dq_base = 0;
                if (dq_base == 0 && pel >= 0.4) {
                    if (jog != 0.0f) { ph = P_DONE; break; }
                    if (seg == 0) {
                        seg = 1; dir = -1.0f;
                        ph = P_SEEK; ph_t0 = now_s();
                        ramp_done = false; stall_n = 0; stall_ref_travel = travel;
                    } else {
                        seg = 2;
                        goto_tgt = 0.5f * (travel_A + travel_B);   // 行程中点
                        ph = P_GOTO; ph_t0 = now_s();
                    }
                }
                break;
            }
            case P_GOTO: {
                // 位置→速度伺服回中点（限探测速度），到位判稳后制动。
                // 死区外指令加下限 kGotoMin 防粘滞（比例项在近目标处推力低于挣脱摩擦会粘住）
                float e = goto_tgt - travel;
                float mag = clampf(kGotoGain * std::fabs(e), kGotoMin, dq_probe);
                float want = (std::fabs(e) < kGotoTol) ? 0.0f : (e > 0 ? mag : -mag);
                float a = kAccel * dt;
                if (dq_base < want) dq_base = (dq_base + a < want) ? dq_base + a : want;
                else                dq_base = (dq_base - a > want) ? dq_base - a : want;
                if (std::fabs(e) < kGotoTol && have_fb && std::fabs(fb.vel) < kStallVel) {
                    ph = P_BRAKE; ph_t0 = now_s();
                }
                if (pel > 15.0) { ph = P_BRAKE; ph_t0 = now_s(); }   // 回中兜底
                break;
            }
            case P_BRAKE: {
                float a = kAccel * dt;
                if (dq_base > a)       dq_base -= a;
                else if (dq_base < -a) dq_base += a;
                else dq_base = 0;
                if (dq_base == 0 && pel >= 0.5) ph = P_DONE;
                break;
            }
            default: break;
        }
        if (ph == P_DONE) break;

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
                sm.pos = fb.pos; sm.travel = travel; sm.vel = fb.vel; sm.tau = fb.tau;
                sm.dqc = dq_base; sm.kd = kd;
                sm.tmos = fb.tmos; sm.trotor = fb.trotor;
                sm.seg = (int16_t)seg;
                sm.phase = (ph == P_SEEK) ? 1 : (ph == P_GOTO ? 2 : 0);
                log.push_back(sm);
            }
        }

        if (have_fb && el > 0.3) {
            if (fb.err >= 8)  { why = dmsc::err_name(fb.err); aborted = true; break; }
            if (fb.err == 0)  { why = "意外失能"; aborted = true; break; }
            if (std::fabs(fb.vel) > kVelAbort) { why = "超速"; aborted = true; break; }
            if (std::fabs(fb.tau) > kTauAbort) { why = "力矩超限"; aborted = true; break; }
            if (fb.tmos > kTempAbort || fb.trotor > kTempAbort) { why = "过温"; aborted = true; break; }
            if (std::fabs(travel) > kTravelGuard) { why = "行程超预期（限位未检出？）"; aborted = true; break; }
        }

        if (el - last_print > 1.0) {
            last_print = el;
            const char* pn = ph == P_SEEK ? (seg == 0 ? "找A+" : "找B-") :
                             ph == P_RELAX ? "卸力" : ph == P_GOTO ? "回中" :
                             ph == P_BRAKE ? "制动" : "宽限";
            std::printf("  t=%5.1fs [%s] dq=%+5.2f | ERR=%X travel=%+8.4f vel=%+7.3f "
                        "tau=%+6.3f MOS=%d℃ 线圈=%d℃\n",
                        el, pn, dq_base, fb.err, travel, fb.vel, fb.tau, fb.tmos, fb.trotor);
        }
    }

    if (aborted && why) std::printf("\n!!! 中止：%s（ERR=%X travel=%.4f vel=%.3f tau=%.3f）\n",
                                    why, fb.err, travel, fb.vel, fb.tau);

    for (int i = 0; i < 5; ++i) { dmsc::send_cmd(s, 0xFD); usleep(10 * 1000); }
    std::printf(">>> 已失能\n");

    if (have_A && have_B) {
        float span = travel_A - travel_B;
        float deg = span * 180.0f / (float)M_PI;
        std::printf("\n=== 标定结果 ===\n");
        std::printf("限位A（+向）travel=%+.4f rad  原始pos=%+.4f\n", travel_A, posraw_A);
        std::printf("限位B（−向）travel=%+.4f rad  原始pos=%+.4f\n", travel_B, posraw_B);
        std::printf("电机侧实测行程 = %.4f rad = %.2f°\n", span, deg);
        std::printf("判定：|Δ45°|=%.1f°  |Δ90°|=%.1f°  ⇒ %s\n",
                    std::fabs(deg - 45.0f), std::fabs(deg - 90.0f),
                    std::fabs(deg - 45.0f) < std::fabs(deg - 90.0f)
                        ? "更接近 45° ⇒ 「行程90°」应指双指张角（电机转半程）"
                        : "更接近 90° ⇒ 电机侧行程即 90°");
        std::printf("软件零点约定：限位B=全开=0，正方向=闭合（朝限位A=全闭），量程 [0, %.4f] rad。\n"
                    "（安全红线禁发 0xFE，不写电机硬件零点；跨上电请朝 B 向 re-home。）\n", span);
        std::printf("峰值 |tau|=%.3f Nm  终点 travel=%+.4f（目标中点 %+.4f）\n",
                    std::fabs(tau_peak), travel, goto_tgt);

        FILE* f = std::fopen(cal_path.c_str(), "w");
        if (f) {
            time_t now = time(nullptr);
            char ts[32];
            strftime(ts, sizeof(ts), "%Y-%m-%dT%H:%M:%S", localtime(&now));
            std::fprintf(f,
                "{\n"
                "  \"motor\": \"DM-J4310-2EC\", \"date\": \"%s\",\n"
                "  \"dq_probe\": %.3f, \"kd\": %.3f, \"push_Nm\": %.3f,\n"
                "  \"travel_A_rad\": %.5f, \"pos_raw_A\": %.5f,\n"
                "  \"travel_B_rad\": %.5f, \"pos_raw_B\": %.5f,\n"
                "  \"span_rad\": %.5f, \"span_deg\": %.2f,\n"
                "  \"close_dir\": \"+\", \"limit_A\": \"全闭\", \"limit_B\": \"全开\",\n"
                "  \"note\": \"travel 系本次会话解卷绕坐标(起点=0)；跨上电以重找限位为准；"
                "开合方向 2026-08-12 点动实证：+向=闭合。re-home 请朝 B(全开)向，防误夹持物\"\n"
                "}\n",
                ts, dq_probe, kd, kd * dq_probe,
                travel_A, posraw_A, travel_B, posraw_B, span, deg);
            std::fclose(f);
            std::printf("标定文件 -> %s\n", cal_path.c_str());
        }
    } else if (jog != 0.0f) {
        std::printf("\n=== 点动结束 ===\n本次朝 %c 方向（限位%c 一侧）实际移动 %+.4f rad（≈%.1f°）。\n"
                    "请告知观察结果：夹爪是【张开】了还是【闭合】了？\n",
                    jog > 0 ? '+' : '-', jog > 0 ? 'A' : 'B', travel,
                    std::fabs(travel) * 180 / M_PI);
    } else {
        std::printf("\n未完成两端限位标定（A:%s B:%s）。\n",
                    have_A ? "√" : "×", have_B ? "√" : "×");
        if (!have_A)
            std::printf("若 [找A+] 阶段原地即判堵转，可能推力不足以挣脱夹爪机构摩擦，\n"
                        "试 --dq 0.5 --kd 1.5（堵转推力 0.75 Nm，仍在 1.0 上限内）。\n");
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
