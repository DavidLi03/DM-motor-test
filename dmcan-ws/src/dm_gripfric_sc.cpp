// dm_gripfric_sc.cpp —— 夹爪机构摩擦/阻力辨识（有限行程往返版，DM-J4310 + 1:1 齿轮两指夹爪）
//
// 背景：夹爪有硬限位（行程 span≈1.49 rad），连续旋转例程 dm_fric_sc 禁跑；
//   本程序把同一套「恒速平台 + coast-down」方法折叠进 [margin, span−margin] 行程内往返。
//   坐标约定同 dm_gripctl_sc：+向=闭合，限位B(全开)=0，每次运行先朝 B re-home 找零。
//
// 四个模式（四选一，均要求爪内【空载】）：
//   --sweep      8 档速度往返恒速平台（每档 +闭合段、−张开段）。纯阻尼稳态
//                kd·(dq_cmd − ω实测) = T_f(ω)，低速档配高 kd 压过挣脱摩擦。
//                测量窗以「动态守卫线」位置终止为常态，兼防撞限位。
//   --breakaway  静摩擦挣脱：3 位置 × 双向，定 dq=0.4、kd 从 0 缓慢上爬
//                （推力增速 0.05 Nm/s，分辨率 ~0.0005 Nm），运动 onset 即记 Ts。
//   --coast      转动惯量：中段加速到 ~1.1 rad/s 后 kd=0 自由滑停。滑停仅 30~50ms，
//                巡航/滑行/尾录相位内发帧周期切 2ms（500Hz），每向 8 次重复。
//   --track      补偿 A/B 验证：三角波+正弦轨迹跟踪（仍纯阻尼），--comp 时加
//                摩擦前馈 dq_ff = T̂_f/kd（读 grip_fric.json）。CSV 追加 x_ref 列。
//
// 保护（在 dm_gripctl_sc 六类之上收紧/新增）：ERR≥8 / 意外失能 / |ω|>2.0 /
//   |tau|>2.0 / 温度>70℃ / 发送失败 / 行程越界 / 找零行程兜底 / 总时长兜底 /
//   【瞬时推力 |kd·(dq_cmd−ω)|>1.25 Nm 持续 0.1s】/【贴限 0.08 rad 且朝限位动 ⇒ 强制卸力】/
//   平台中段受阻（动过又停 ⇒ 疑似爪内有物）立即中止。
// Ctrl-C ×1 柔停（跳过剩余段），×2 立即失能。
// 只发 0xFB/0xFC/0xFD/MIT；绝无 0x55/0xAA/0xFE。MIT 帧 kp=0、tau=0（纯阻尼）。
//
// 用法: dm_gripfric_sc (--sweep|--breakaway|--coast|--track [--comp])
//                      [--if can0] [--csv 路径] [--cal grip_cal.json]
//                      [--fric grip_fric.json] [--margin 0.12] [--reps 8]
//                      [--no-park] [--list]
// 分析: tools/fit_gripfric.py *.csv --json grip_fric.json

#include <algorithm>
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

// ---- 周期与保护 ----
constexpr int    kPeriodMs     = 10;     // 100Hz 主环
constexpr int    kPeriodFastMs = 2;      // 500Hz：S_COAST 巡航/滑行/尾录
constexpr float  kVelAbort     = 2.0f;   // 全计划最高指令 1.2 rad/s
constexpr float  kTauAbort     = 2.0f;
constexpr int    kTempAbort    = 70;
constexpr float  kTravelGuard  = 2.2f;   // 找零前行程兜底
constexpr float  kXMargin      = 0.30f;  // 找零后 x 越界 abort 兜底
constexpr float  kPushAbortNm  = 1.25f;  // 瞬时推力 abort 阈（计划内最大堵转推力 1.10）
constexpr double kPushAbortS   = 0.10;
constexpr float  kNearLimit    = 0.08f;  // 贴限强制卸力线（绝对兜底）
constexpr float  kGuardSlack   = 0.04f;  // 守卫线滑停余量
// ---- 堵转判稳（找零/复位伺服共用，同 dm_gripctl_sc）----
constexpr float  kStallVel     = 0.05f;
constexpr float  kStallTravel  = 0.01f;
constexpr int    kStallN       = 30;     // 300ms @100Hz
constexpr double kHoldoffS     = 0.35;
// ---- 找零 ----
constexpr float  kHomeDq = 0.4f, kHomeKd = 1.5f, kHomeAccel = 2.0f;
constexpr double kHomeTmoS = 15.0;
// ---- 复位伺服 S_MOVE / P_PARK ----
constexpr float  kMoveKd = 1.8f, kMoveGain = 2.0f, kMoveMin = 0.30f;
constexpr float  kMoveTol = 0.03f, kMoveNear = 0.05f, kMoveDqMax = 0.4f, kMoveAccel = 2.0f;
constexpr double kMoveTmoS = 8.0;
constexpr float  kParkX = 0.10f;
// ---- 平台 ----
constexpr double kMeasMinS  = 0.30;      // 有效窗下限（fit 门槛同步 0.3s/30 帧）
constexpr double kPlatChkS  = 1.0;       // 受阻/未挣脱滚动检查周期
constexpr float  kPlatChkD  = 0.02f;     // 检查窗内行程增量阈
// ---- 挣脱 ----
constexpr float  kPushDq     = 0.4f;
constexpr float  kPushKdRate = 0.125f;   // kd 上爬速率 ⇒ 推力增速 0.05 Nm/s
constexpr float  kPushKdCap  = 1.5f;     // 封顶推力 0.6 Nm
constexpr float  kPushOnset  = 0.01f;    // 运动 onset 阈 (rad)
constexpr double kPushRestS  = 1.0, kPushHoldS = 2.0, kPushTmoS = 18.0;
// ---- coast ----
constexpr float  kCoastV = 1.1f, kCoastKd = 1.0f, kCoastAccel = 6.0f;
constexpr float  kCoastStopD = 0.001f;   // 停转判据：行程增量 (rad)
constexpr int    kCoastStopN = 50;       // 连续 50 × 2ms = 0.1s
constexpr double kCoastTmoS = 1.5, kCoastTailS = 0.2, kCruiseMinS = 0.2, kCruiseTmoS = 4.0;
// ---- track ----
constexpr float  kTrkCenter = 0.75f, kTrkAmp = 0.30f, kTrkTriV = 0.3f;
constexpr double kTrkSineT = 4.0;
constexpr int    kTrkCycles = 6;
constexpr float  kTrkGain = 3.0f, kTrkKd = 1.5f, kTrkDqMax = 0.8f, kTrkAccel = 6.0f;
constexpr float  kTrkPushMax = 1.10f;    // 瞬时力矩钳：|kd·(dq−ω)| ≤ 此值
constexpr float  kTrkRevD = 0.005f;      // 换向跟随判据 (rad)
constexpr float  kFfMargin = 1.10f;      // 前馈裕量
// ----
constexpr float  kSpanDflt = 1.4916f;
constexpr int    kLogMax   = 45000;

struct Sample {
    float t, pos, x, vel, tau, dqc, kdc, xref;
    uint64_t kts;
    uint8_t tmos, trotor, phase;
    int8_t seg;
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

// 从 grip_cal.json 里抠 span_rad（同 dm_gripctl_sc）
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

// ---- grip_fric.json 摩擦模型（--track --comp 用；铺平键由 fit_gripfric.py 写出）----
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
    float tmos_ref = 0;
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
        // 合理域检查：出域 ⇒ 判未加载
        if (m.Tc[i] < 0.02f || m.Tc[i] > 1.0f) return m;
        if (m.Ts[i] < m.Tc[i] - 0.01f || m.Ts[i] > 1.0f) return m;
        if (m.ws[i] < 0.005f || m.ws[i] > 5.0f) return m;
        if (m.b[i] < 0.0f || m.b[i] > 0.3f) return m;
        if (m.Tsbk[i] < 0.05f || m.Tsbk[i] > 1.0f) return m;
    }
    jnum(buf, "ff_tmos_ref", &m.tmos_ref);
    m.ok = true;
    return m;
}

// ---- 段表 ----
enum GSegType { S_MOVE, S_PLATEAU, S_PUSH, S_COAST, S_TRACK };
struct GSeg {
    GSegType type;
    float  v;        // PLATEAU/COAST: 带符号目标速度  PUSH: ±1 推向  其余无意义
    float  kd;       // PLATEAU 阻尼 / COAST 巡航阻尼（MOVE/PUSH/TRACK 用各自常量）
    float  accel;
    double settle_s; // PLATEAU 稳定期
    double meas_s;   // 测量窗上限 / TRACK 总时长
    float  x_tgt;    // MOVE 目标 / COAST 触发线 / PUSH 位置（打印用）/ TRACK 中心
    int    wave;     // TRACK: 0=三角 1=正弦
};

// sweep 速度档表：{v, kd, accel, settle_s, meas_s}
// 低速档 kd 必须压过挣脱摩擦上界 0.4 Nm（kd·v ≥ 0.5）；kd 上限 = MIT 编码 5.0。
struct Band { float v, kd, accel; double settle, meas; };
constexpr Band kBands[] = {
    {0.10f, 5.0f, 1.0f, 0.8, 28.0},   // 档1：位置终止为主，全程铺 τ(x) 图
    {0.15f, 4.0f, 1.5f, 0.7,  8.0},
    {0.22f, 3.0f, 1.5f, 0.6,  6.0},
    {0.32f, 2.5f, 2.0f, 0.6,  3.5},
    {0.45f, 2.0f, 2.5f, 0.5,  2.5},
    {0.65f, 1.5f, 3.0f, 0.45, 1.5},
    {0.90f, 1.2f, 4.5f, 0.35, 0.9},
    {1.20f, 0.9f, 6.0f, 0.30, 0.55},
};
constexpr int kNBands = (int)(sizeof(kBands) / sizeof(kBands[0]));

// 段时长估计（--list 与总时长兜底用，宁高勿低）
double seg_est_s(const GSeg& sg) {
    switch (sg.type) {
        case S_MOVE:    return 5.5;                      // 最远 1.25 rad / ~0.3 rad/s
        case S_PLATEAU: return 2.0 * std::fabs(sg.v) / sg.accel + sg.settle_s + sg.meas_s + 0.5;
        case S_PUSH:    return kPushRestS + kPushKdCap / kPushKdRate + kPushHoldS + 1.0;
        case S_COAST:   return kCoastV / sg.accel + kCruiseTmoS + kCoastTmoS + kCoastTailS + 0.5;
        case S_TRACK:   return sg.meas_s + 1.0;
    }
    return 1.0;
}

// ---- 结果 ----
struct PlatRes {
    int   seg;
    float v_tgt, kd, vm, att, fric_bs, tau_m, tau_sd, stick_pct, x0, x1;
    int   tmos, trot;
    bool  stalled;
};
struct PushRes  { float xpos; int dir; float Ts; bool lower; };
struct CoastRes { float v0; double dur; int frames; };
struct TrackRes { int wave; bool comp; float rms, peak, lag_mean, lag_max; int nlag, n; };

}  // namespace

int main(int argc, char** argv) {
    enum Mode { M_NONE, M_SWEEP, M_BREAK, M_COAST, M_TRACK } mode = M_NONE;
    bool comp = false, list_only = false, no_park = false;
    int  reps = 8;
    float margin = 0.12f;
    std::string ifname = "can0", csv_path, cal_path = "grip_cal.json",
                fric_path = "grip_fric.json";

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if      (a == "--sweep")     mode = M_SWEEP;
        else if (a == "--breakaway") mode = M_BREAK;
        else if (a == "--coast")     mode = M_COAST;
        else if (a == "--track")     mode = M_TRACK;
        else if (a == "--comp")      comp = true;
        else if (a == "--list")      list_only = true;
        else if (a == "--no-park")   no_park = true;
        else if (a == "--if"     && i + 1 < argc) ifname = argv[++i];
        else if (a == "--csv"    && i + 1 < argc) csv_path = argv[++i];
        else if (a == "--cal"    && i + 1 < argc) cal_path = argv[++i];
        else if (a == "--fric"   && i + 1 < argc) fric_path = argv[++i];
        else if (a == "--margin" && i + 1 < argc) margin = (float)std::atof(argv[++i]);
        else if (a == "--reps"   && i + 1 < argc) reps = std::atoi(argv[++i]);
        else {
            std::printf(
                "用法: dm_gripfric_sc (--sweep|--breakaway|--coast|--track [--comp])\n"
                "                     [--if can0] [--csv 路径] [--cal grip_cal.json]\n"
                "                     [--fric grip_fric.json] [--margin 0.12] [--reps 8]\n"
                "                     [--no-park] [--list]\n"
                "夹爪空载摩擦辨识（有限行程往返，纯阻尼 kp=0 tau=0）：\n"
                "  --sweep      8 档往返恒速平台（0.10~1.20 rad/s，~2.5min）\n"
                "  --breakaway  3 位置×双向 静摩擦挣脱（kd 缓斜坡，~2min）\n"
                "  --coast      kd=0 自由滑停×8 双向，coast 相位 500Hz（~1.5min）\n"
                "  --track      三角+正弦轨迹跟踪 A/B（--comp 加摩擦前馈，~1min）\n"
                "--list 只打印段表不发帧（dry check）\n");
            return a == "--help" ? 0 : 1;
        }
    }
    if (mode == M_NONE) {
        std::printf("请指定模式：--sweep / --breakaway / --coast / --track（--help 看说明）\n");
        return 1;
    }
    margin = clampf(margin, 0.08f, 0.20f);
    if (reps < 2) reps = 2;
    if (reps > 12) reps = 12;

    float span = load_span(cal_path);
    bool span_from_cal = (span > 0.5f && span < 2.5f);
    if (!span_from_cal) span = kSpanDflt;
    const float x_lo = margin, x_hi = span - margin;   // 可用行程

    FricModel fric;
    if (mode == M_TRACK && comp) {
        fric = load_fric(fric_path);
        if (!fric.ok) {
            std::printf("--comp 需要有效的 %s（先跑 sweep/breakaway 并用 fit_gripfric.py "
                        "--json 生成），未加载，退出。\n", fric_path.c_str());
            return 1;
        }
    }

    // ---- 组段表 ----
    std::vector<GSeg> segs;
    auto push_move = [&](float xt) {
        segs.push_back({S_MOVE, 0, 0, kMoveAccel, 0, kMoveTmoS, clampf(xt, x_lo, x_hi), 0});
    };
    if (mode == M_SWEEP) {
        for (int i = 0; i < kNBands; ++i) {
            const Band& b = kBands[i];
            push_move(x_lo);
            segs.push_back({S_PLATEAU, +b.v, b.kd, b.accel, b.settle, b.meas, 0, 0});
            segs.push_back({S_PLATEAU, -b.v, b.kd, b.accel, b.settle, b.meas, 0, 0});
        }
    } else if (mode == M_BREAK) {
        for (float xp : {0.25f, 0.75f, 1.25f}) {
            push_move(xp);
            segs.push_back({S_PUSH, +1, 0, kTrkAccel, 0, kPushTmoS, xp, 0});
            push_move(xp);
            segs.push_back({S_PUSH, -1, 0, kTrkAccel, 0, kPushTmoS, xp, 0});
        }
    } else if (mode == M_COAST) {
        for (int r = 0; r < reps; ++r) {
            push_move(x_lo);
            segs.push_back({S_COAST, +kCoastV, kCoastKd, kCoastAccel, 0, kCoastTmoS, 0.60f, 0});
            push_move(x_hi);
            segs.push_back({S_COAST, -kCoastV, kCoastKd, kCoastAccel, 0, kCoastTmoS, 0.90f, 0});
        }
    } else {  // M_TRACK
        double tri_T = 4.0 * kTrkAmp / kTrkTriV;
        push_move(kTrkCenter);
        segs.push_back({S_TRACK, 0, 0, kTrkAccel, 0, kTrkCycles * tri_T, kTrkCenter, 0});
        push_move(kTrkCenter);
        segs.push_back({S_TRACK, 0, 0, kTrkAccel, 0, kTrkCycles * kTrkSineT, kTrkCenter, 1});
    }

    // 计划时长与最大堵转推力
    double plan_s = 0;
    float stall_max = kHomeKd * kHomeDq;               // 找零 0.6
    if (kMoveKd * kMoveDqMax > stall_max) stall_max = kMoveKd * kMoveDqMax;
    for (const GSeg& sg : segs) {
        plan_s += seg_est_s(sg);
        float sp = 0;
        if (sg.type == S_PLATEAU)      sp = sg.kd * std::fabs(sg.v);
        else if (sg.type == S_COAST)   sp = sg.kd * std::fabs(sg.v);
        else if (sg.type == S_PUSH)    sp = kPushKdCap * kPushDq;
        else if (sg.type == S_TRACK)   sp = kTrkPushMax;      // 瞬时力矩钳保证
        if (sp > stall_max) stall_max = sp;
    }
    const double budget_s = kHomeTmoS + plan_s + (no_park ? 0 : kMoveTmoS) + 25.0;

    const char* mode_name = mode == M_SWEEP ? "sweep 恒速往返" :
                            mode == M_BREAK ? "breakaway 挣脱" :
                            mode == M_COAST ? "coast 惯量" : "track 轨迹跟踪";
    std::printf("=== 夹爪摩擦辨识：%s%s（要求爪内空载！）===\n",
                mode_name, (mode == M_TRACK && comp) ? " + 摩擦前馈" : "");
    std::printf("行程 span=%.4f rad（%s）  可用 [%.2f, %.2f]  段数 %zu\n",
                span, span_from_cal ? cal_path.c_str() : "内置默认", x_lo, x_hi, segs.size());
    std::printf("预计 ~%.0fs（兜底 %.0fs）  最大堵转推力 %.2f Nm（瞬时推力保护 %.2f Nm/%.1fs）\n",
                plan_s, budget_s, stall_max, kPushAbortNm, kPushAbortS);
    if (mode == M_TRACK && comp)
        std::printf("前馈模型: 闭合 Tc=%.3f Ts=%.3f ωs=%.3f b=%.4f | 张开 Tc=%.3f Ts=%.3f "
                    "ωs=%.3f b=%.4f | 挣脱 %.3f/%.3f（参考温度 %.0f℃）\n",
                    fric.Tc[0], fric.Ts[0], fric.ws[0], fric.b[0],
                    fric.Tc[1], fric.Ts[1], fric.ws[1], fric.b[1],
                    fric.Tsbk[0], fric.Tsbk[1], fric.tmos_ref);
    std::printf("段表:\n");
    for (size_t i = 0; i < segs.size(); ++i) {
        const GSeg& sg = segs[i];
        switch (sg.type) {
            case S_MOVE:
                std::printf("  #%2zu 复位 → x=%.2f\n", i, sg.x_tgt);
                break;
            case S_PLATEAU:
                std::printf("  #%2zu 平台 dq=%+5.2f rad/s kd=%.2f 堵转 %.2f Nm  稳定 %.2fs 窗≤%.1fs\n",
                            i, sg.v, sg.kd, sg.kd * std::fabs(sg.v), sg.settle_s, sg.meas_s);
                break;
            case S_PUSH:
                std::printf("  #%2zu 挣脱 x=%.2f %s向  kd 0→%.1f @%.3f/s（推力≤%.1f Nm）\n",
                            i, sg.x_tgt, sg.v > 0 ? "+闭合" : "−张开",
                            kPushKdCap, kPushKdRate, kPushKdCap * kPushDq);
                break;
            case S_COAST:
                std::printf("  #%2zu coast dq=%+4.1f kd=%.1f 过 x=%.2f 后 kd=0 滑停（500Hz 采样）\n",
                            i, sg.v, sg.kd, sg.x_tgt);
                break;
            case S_TRACK:
                std::printf("  #%2zu 跟踪 %s 中心 %.2f 幅 %.2f  %.0fs%s\n",
                            i, sg.wave ? "正弦 T=4s" : "三角 v=0.3", sg.x_tgt, kTrkAmp,
                            sg.meas_s, comp ? "（带前馈）" : "");
                break;
        }
    }
    std::printf("流程：re-home 朝B找零 → 段表 → %s制动失能。Ctrl-C ×1 柔停 ×2 立即失能\n",
                no_park ? "" : "回 x=0.10 → ");
    if (list_only) { std::printf("(--list：不发帧，结束)\n"); return 0; }

    std::signal(SIGINT, on_sigint);
    std::string errstr;
    int s = dmsc::open_can(ifname.c_str(), true, &errstr);
    if (s < 0) { std::printf("%s\n", errstr.c_str()); return 1; }

    std::vector<Sample> log;
    log.reserve(kLogMax);
    std::vector<PlatRes>  plat_res;
    std::vector<PushRes>  push_res;
    std::vector<CoastRes> coast_res;
    std::vector<TrackRes> track_res;

    dmsc::send_cmd(s, 0xFB);
    usleep(20 * 1000);
    dmsc::send_cmd(s, 0xFC);

    enum Phase { P_GRACE, P_HOME, P_RELAX, P_RUN, P_PARK, P_BRAKE, P_DONE } ph = P_GRACE;
    enum RPh { R_MOVE, R_RAMP, R_SETTLE, R_MEASURE, R_RAMPDN, R_REST, R_PUSH, R_PUSHDN,
               R_CRUISE, R_COASTING, R_TAIL } rph = R_MOVE;

    double t0 = now_s(), ph_t0 = t0, seg_t0 = t0, last_print = 0;
    int    period_ms = kPeriodMs;
    float  dq_cmd = 0, kd_cur = kHomeKd;
    bool   aborted = false, softstop = false;
    const char* why = nullptr;
    Fb fb{};
    bool have_fb = false;

    float travel = 0, last_pos = 0;
    bool have_pos = false;
    float home_travel = 0;
    bool homed = false;
    int   stall_n = 0;
    float stall_ref_travel = 0;
    double push_over_t = 0;                 // 瞬时推力超限累计时间

    int    seg_i = -1;
    int    csv_phase = 0;
    // 平台测量
    float  m_trav0 = 0;  double m_t0 = 0;
    double m_sum_tau = 0, m_sum_tau2 = 0;
    int    m_n = 0, m_stick = 0;
    double chk_t0 = 0;   float chk_trav = 0;
    float  dq_tq = 0;                       // 平台量化后的目标速度（带符号）
    // 挣脱
    float  push_x0 = 0;  double push_capt0 = -1;
    // coast
    float  coast_v0 = 0, coast_ref = 0;
    double coast_t0 = 0;
    int    coast_n = 0, coast_frames = 0;
    // track
    double trk_t0 = 0;
    float  trk_xref = NAN, trk_xdot = 0;
    double trk_e2 = 0;  float trk_peak = 0;  int trk_n = 0;
    float  trk_lastdir = 0, trk_flip_x = 0;  double trk_flip_t = 0;
    bool   trk_wait_rev = false;
    double trk_lag_sum = 0;  float trk_lag_max = 0;  int trk_nlag = 0;

    auto begin_seg = [&](int i) {
        seg_i = i;
        seg_t0 = now_s();
        stall_n = 0;
        stall_ref_travel = travel;
        period_ms = kPeriodMs;
        csv_phase = 0;
        trk_xref = NAN;
        if (softstop || i >= (int)segs.size()) {
            if (kd_cur < 0.5f) kd_cur = 1.0f;           // coast 中柔停也要有阻尼可用
            if (softstop || no_park) { ph = P_BRAKE; ph_t0 = now_s(); }
            else                     { ph = P_PARK;  ph_t0 = now_s(); kd_cur = kMoveKd; }
            return;
        }
        const GSeg& sg = segs[i];
        switch (sg.type) {
            case S_MOVE:
                rph = R_MOVE;
                kd_cur = dmsc::quantized(kMoveKd, 0, 5, 12);
                break;
            case S_PLATEAU:
                rph = R_RAMP;
                kd_cur = dmsc::quantized(sg.kd, 0, 5, 12);
                dq_tq  = dmsc::quantized(sg.v, -dmsc::kVMax, dmsc::kVMax, 12);
                break;
            case S_PUSH:
                rph = R_REST;
                kd_cur = dmsc::quantized(kHomeKd, 0, 5, 12);
                push_capt0 = -1;
                break;
            case S_COAST:
                rph = R_RAMP;
                kd_cur = dmsc::quantized(sg.kd, 0, 5, 12);
                dq_tq  = dmsc::quantized(sg.v, -dmsc::kVMax, dmsc::kVMax, 12);
                break;
            case S_TRACK:
                rph = R_MOVE;   // 占位；下面立即转 TRACK 初始化
                kd_cur = dmsc::quantized(kTrkKd, 0, 5, 12);
                trk_t0 = now_s();
                trk_e2 = 0; trk_peak = 0; trk_n = 0;
                trk_lastdir = 0; trk_wait_rev = false;
                trk_lag_sum = 0; trk_lag_max = 0; trk_nlag = 0;
                rph = R_MEASURE;   // 复用枚举位无歧义处：TRACK 只有一个子相位
                break;
        }
    };

    while (ph != P_DONE) {
        double el = now_s() - t0;
        double pel = now_s() - ph_t0;
        double sel = now_s() - seg_t0;
        if (el > budget_s) { why = "总时长兜底"; aborted = true; break; }
        if (g_stop == 1 && !softstop) {
            softstop = true;
            std::printf(">>> Ctrl-C，柔停：结束当前段后制动\n");
            if (ph == P_RUN) {
                if (rph == R_COASTING || rph == R_TAIL) { period_ms = kPeriodMs; kd_cur = 1.0f; }
                rph = R_RAMPDN;
                csv_phase = 0;
            } else if (ph == P_HOME || ph == P_RELAX) {
                ph = P_BRAKE; ph_t0 = now_s();
            }
        }
        if (g_stop >= 2) { why = "Ctrl-C x2"; aborted = true; break; }

        const float dt = period_ms / 1000.0f;
        float x = travel - home_travel;

        // 堵转判稳（找零 / 复位伺服 / 停车共用）
        bool stalled = false;
        {
            bool ctx = (ph == P_HOME) || (ph == P_PARK) ||
                       (ph == P_RUN && segs[seg_i].type == S_MOVE && rph == R_MOVE);
            double ctx_el = (ph == P_RUN) ? sel : pel;
            if (ctx && ctx_el > kHoldoffS && have_fb) {
                if (std::fabs(fb.vel) < kStallVel &&
                    std::fabs(travel - stall_ref_travel) < kStallTravel) {
                    stall_n++;
                } else {
                    stall_n = 0;
                    stall_ref_travel = travel;
                }
                stalled = (stall_n >= kStallN);
            }
        }

        auto ramp_dq = [&](float tgt, float a) {
            float st = a * dt;
            float d = tgt - dq_cmd;
            if      (d >  st) dq_cmd += st;
            else if (d < -st) dq_cmd -= st;
            else              dq_cmd  = tgt;
        };

        switch (ph) {
            case P_GRACE:
                if (pel >= 0.3) { ph = P_HOME; ph_t0 = now_s(); stall_n = 0; stall_ref_travel = travel; }
                break;

            case P_HOME:
                ramp_dq(-kHomeDq, kHomeAccel);
                if (stalled) {
                    home_travel = travel; homed = true;
                    std::printf(">>> 找零完成：限位B(全开)=0（原始pos=%+.4f）\n", fb.pos);
                    ph = P_RELAX; ph_t0 = now_s();
                }
                if (pel > kHomeTmoS) { why = "找零超时"; aborted = true; }
                break;

            case P_RELAX:
                ramp_dq(0, kHomeAccel);
                if (dq_cmd == 0 && pel >= 0.4) { ph = P_RUN; begin_seg(0); }
                break;

            case P_RUN: {
                const GSeg& sg = segs[seg_i];
                float dir = (sg.type == S_PLATEAU || sg.type == S_COAST || sg.type == S_PUSH)
                            ? (sg.v > 0 ? 1.0f : -1.0f) : 0.0f;
                // 贴限强制卸力（绝对兜底；kd=0 的滑行段本就零力矩，不适用）
                if (rph != R_COASTING && rph != R_TAIL && rph != R_RAMPDN && rph != R_PUSHDN) {
                    if ((x < kNearLimit && dq_cmd < -0.01f) ||
                        (x > span - kNearLimit && dq_cmd > 0.01f)) {
                        std::printf(">>> [t=%.1fs] 段#%d 贴限 %.3f rad，强制卸力\n", el, seg_i, x);
                        if (rph == R_MEASURE && sg.type == S_PLATEAU) goto finalize_meas;
                        csv_phase = 0;
                        rph = R_RAMPDN;
                    }
                }
                // 动态守卫线（平台/巡航的主动早停）
                if (sg.type == S_PLATEAU &&
                    (rph == R_RAMP || rph == R_SETTLE || rph == R_MEASURE)) {
                    float dstop = dq_cmd * dq_cmd / (2 * sg.accel) + kGuardSlack;
                    bool hit = dir > 0 ? (x >= x_hi - dstop) : (x <= x_lo + dstop);
                    if (hit) {
                        if (rph == R_MEASURE) goto finalize_meas;
                        std::printf(">>> [t=%.1fs] 段#%d 守卫线早停（未开窗，本段作废）\n", el, seg_i);
                        csv_phase = 0;
                        rph = R_RAMPDN;
                    }
                }

                switch (rph) {
                    case R_MOVE: {   // 复位伺服（同 dm_gripctl_sc P_MOVE，tol 放宽）
                        float e = sg.x_tgt - x;
                        if (std::fabs(e) < kMoveNear && have_fb && std::fabs(fb.vel) < kStallVel) {
                            begin_seg(seg_i + 1);
                            break;
                        }
                        float mag = clampf(kMoveGain * std::fabs(e), kMoveMin, kMoveDqMax);
                        float want = (std::fabs(e) < kMoveTol) ? 0.0f : (e > 0 ? mag : -mag);
                        ramp_dq(want, kMoveAccel);
                        if (stalled && std::fabs(e) > 0.06f) {
                            why = "复位段中途受阻（爪内疑似有物，辨识须空载）";
                            aborted = true;
                        }
                        if (sel > kMoveTmoS) { why = "复位伺服超时"; aborted = true; }
                        break;
                    }

                    case R_RAMP:
                        ramp_dq(dq_tq, sg.accel);
                        if (dq_cmd == dq_tq) {
                            if (sg.type == S_COAST) {
                                rph = R_CRUISE;
                                period_ms = kPeriodFastMs;   // 500Hz：巡航起精测 ω₀
                                seg_t0 = now_s();
                            } else {
                                rph = R_SETTLE;
                                seg_t0 = now_s();
                            }
                        }
                        break;

                    case R_SETTLE:
                        if (sel >= sg.settle_s) {
                            rph = R_MEASURE;
                            csv_phase = 1;
                            m_trav0 = travel; m_t0 = now_s();
                            m_sum_tau = m_sum_tau2 = 0; m_n = 0; m_stick = 0;
                            chk_t0 = now_s(); chk_trav = travel;
                        }
                        break;

                    case R_MEASURE: {
                        if (sg.type == S_TRACK) {   // TRACK 唯一子相位
                            double tt = now_s() - trk_t0;
                            if (tt >= sg.meas_s) {
                                TrackRes r;
                                r.wave = sg.wave; r.comp = comp;
                                r.n = trk_n;
                                r.rms = trk_n ? (float)std::sqrt(trk_e2 / trk_n) : 0;
                                r.peak = trk_peak;
                                r.nlag = trk_nlag;
                                r.lag_mean = trk_nlag ? (float)(trk_lag_sum / trk_nlag) : 0;
                                r.lag_max = trk_lag_max;
                                track_res.push_back(r);
                                std::printf(">>> [t=%.1fs] %s跟踪%s：RMS=%.4f rad 峰值=%.4f "
                                            "换向滞后 均值 %.0fms 最大 %.0fms（%d 次）\n",
                                            el, sg.wave ? "正弦" : "三角", comp ? "(带前馈)" : "",
                                            r.rms, r.peak, r.lag_mean * 1000, r.lag_max * 1000, r.nlag);
                                trk_xref = NAN;
                                csv_phase = 0;
                                rph = R_RAMPDN;
                                break;
                            }
                            csv_phase = 4;
                            // 参考轨迹
                            float xr, xd;
                            if (sg.wave == 0) {   // 三角：从中心出发先 +向
                                double T = 4.0 * kTrkAmp / kTrkTriV;
                                double u = std::fmod(tt, T);
                                if (u < T / 4)      { xr = sg.x_tgt + kTrkTriV * (float)u;                    xd = +kTrkTriV; }
                                else if (u < 3 * T / 4) { xr = sg.x_tgt + kTrkAmp - kTrkTriV * (float)(u - T / 4); xd = -kTrkTriV; }
                                else                { xr = sg.x_tgt - kTrkAmp + kTrkTriV * (float)(u - 3 * T / 4); xd = +kTrkTriV; }
                            } else {              // 正弦
                                double w = 2.0 * M_PI / kTrkSineT;
                                xr = sg.x_tgt + kTrkAmp * (float)std::sin(w * tt);
                                xd = kTrkAmp * (float)w * (float)std::cos(w * tt);
                            }
                            trk_xref = xr; trk_xdot = xd;
                            float e = xr - x;
                            trk_e2 += (double)e * e; trk_n++;
                            if (std::fabs(e) > trk_peak) trk_peak = std::fabs(e);
                            // 换向滞后：参考速度翻向 → 实际行程跟着翻向的时间差
                            float d_ref = xd > 0 ? 1.0f : -1.0f;
                            if (trk_lastdir != 0 && d_ref != trk_lastdir) {
                                trk_wait_rev = true; trk_flip_t = now_s(); trk_flip_x = x;
                            }
                            trk_lastdir = d_ref;
                            if (trk_wait_rev && (x - trk_flip_x) * d_ref > kTrkRevD) {
                                double lag = now_s() - trk_flip_t;
                                trk_lag_sum += lag; trk_nlag++;
                                if (lag > trk_lag_max) trk_lag_max = (float)lag;
                                trk_wait_rev = false;
                            }
                            // 控制律：速度前馈 + P（纯阻尼通道），--comp 加摩擦前馈
                            float raw = xd + kTrkGain * e;
                            if (comp && fric.ok) {
                                int di = d_ref > 0 ? 0 : 1;
                                float w_now = std::fabs(fb.vel);
                                float Th = (w_now < kStallVel) ? fric.Tsbk[di]
                                                               : fric.eval(di, w_now);
                                raw += d_ref * clampf(kFfMargin * Th / kTrkKd, 0.0f, 0.45f);
                            }
                            raw = clampf(raw, -kTrkDqMax, kTrkDqMax);
                            // 瞬时力矩钳：|kd·(dq−ω)| ≤ kTrkPushMax
                            raw = clampf(raw, fb.vel - kTrkPushMax / kTrkKd,
                                              fb.vel + kTrkPushMax / kTrkKd);
                            ramp_dq(raw, kTrkAccel);
                            break;
                        }
                        // ---- 恒速平台测量窗 ----
                        m_sum_tau  += fb.tau;
                        m_sum_tau2 += (double)fb.tau * fb.tau;
                        if (fb.vel * dir <= 0.0f) m_stick++;
                        m_n++;
                        // 滚动受阻/未挣脱检查（1s 行程 <0.02 rad）
                        if (now_s() - chk_t0 >= kPlatChkS) {
                            if (std::fabs(travel - chk_trav) < kPlatChkD) {
                                if (std::fabs(travel - m_trav0) < kPlatChkD) {
                                    // 从未起动 ⇒ 未挣脱，Ts 下界 = 施加推力，继续后面的段
                                    PlatRes r{};
                                    r.seg = seg_i; r.v_tgt = sg.v; r.kd = kd_cur;
                                    r.vm = 0; r.att = 0;
                                    r.fric_bs = kd_cur * std::fabs(dq_tq);
                                    r.tau_m = m_n ? (float)(m_sum_tau / m_n) : 0;
                                    r.x0 = m_trav0 - home_travel; r.x1 = x;
                                    r.tmos = fb.tmos; r.trot = fb.trotor;
                                    r.stalled = true; r.stick_pct = 100;
                                    plat_res.push_back(r);
                                    std::printf(">>> [t=%.1fs] 平台#%d dq=%+.2f 未挣脱："
                                                "Ts ≥ %.3f Nm（记下界，跳下段）\n",
                                                el, seg_i, sg.v, r.fric_bs);
                                    csv_phase = 0;
                                    rph = R_RAMPDN;
                                    break;
                                }
                                why = "平台中段受阻（动过又停，爪内疑似有物）";
                                aborted = true;
                                break;
                            }
                            chk_t0 = now_s(); chk_trav = travel;
                        }
                        if (now_s() - m_t0 >= sg.meas_s) goto finalize_meas;
                        break;

                        finalize_meas: {
                            double dur = now_s() - m_t0;
                            csv_phase = 0;
                            if (dur >= kMeasMinS && m_n >= 25) {
                                double vm = (travel - m_trav0) / dur;
                                double tm = m_n ? m_sum_tau / m_n : 0;
                                double tsd = m_n ? std::sqrt(std::max(0.0, m_sum_tau2 / m_n - tm * tm)) : 0;
                                PlatRes r{};
                                r.seg = seg_i; r.v_tgt = sg.v; r.kd = kd_cur;
                                r.vm = (float)vm;
                                r.att = (float)(100.0 * std::fabs(vm) / std::fabs(dq_tq));
                                r.fric_bs = (float)(kd_cur * (std::fabs(dq_tq) - std::fabs(vm)));
                                r.tau_m = (float)tm; r.tau_sd = (float)tsd;
                                r.stick_pct = m_n ? 100.0f * m_stick / m_n : 0;
                                r.x0 = m_trav0 - home_travel; r.x1 = x;
                                r.tmos = fb.tmos; r.trot = fb.trotor;
                                r.stalled = false;
                                plat_res.push_back(r);
                                std::printf(">>> [t=%.1fs] 平台#%d dq=%+.2f: 实测 %+.4f rad/s "
                                            "(%.0f%%) 窗 %.1fs x[%.2f→%.2f] | 反解 %.4f / 遥测 "
                                            "%+.4f±%.4f Nm 粘滑 %.0f%% MOS=%d℃\n",
                                            el, seg_i, sg.v, r.vm, r.att, dur, r.x0, r.x1,
                                            r.fric_bs, r.tau_m, r.tau_sd, r.stick_pct, r.tmos);
                            } else {
                                std::printf(">>> [t=%.1fs] 平台#%d 窗过短（%.2fs/%d 帧），丢弃\n",
                                            el, seg_i, dur, m_n);
                            }
                            rph = R_RAMPDN;
                            break;
                        }
                    }

                    case R_RAMPDN:
                        ramp_dq(0, sg.accel > 0 ? sg.accel : kMoveAccel);
                        if (dq_cmd == 0) begin_seg(seg_i + 1);
                        break;

                    case R_REST:   // 挣脱：静置，记基准位
                        ramp_dq(0, kTrkAccel);
                        if (dq_cmd == 0 && sel >= kPushRestS) {
                            push_x0 = travel;
                            kd_cur = 0;
                            dq_cmd = dmsc::quantized(kPushDq, -dmsc::kVMax, dmsc::kVMax, 12) * dir;
                            rph = R_PUSH;
                            csv_phase = 3;
                            seg_t0 = now_s();
                            push_capt0 = -1;
                        }
                        break;

                    case R_PUSH: {  // 定 dq、kd 缓升 ⇒ 推力 kd·dq 以 0.05 Nm/s 上爬
                        if (std::fabs(travel - push_x0) > kPushOnset) {
                            // 真 onset 时刻 ω=0 ⇒ Ts = 当时施加推力 kd·|dq|；
                            // 检出滞后期 kd 仅多爬 ~0.01 Nm（不能扣 vel——那是检出时刻的瞬时值）
                            float kd_q = dmsc::quantized(kd_cur, 0, 5, 12);
                            float Ts = kd_q * std::fabs(dq_cmd);
                            push_res.push_back({sg.x_tgt, (int)dir, Ts, false});
                            std::printf(">>> [t=%.1fs] 挣脱 x=%.2f %s向: Ts=%.3f Nm（kd=%.3f）\n",
                                        el, sg.x_tgt, dir > 0 ? "+" : "−", Ts, kd_q);
                            csv_phase = 0;
                            rph = R_PUSHDN;
                            break;
                        }
                        if (kd_cur < kPushKdCap) {
                            kd_cur += kPushKdRate * dt;
                            if (kd_cur >= kPushKdCap) { kd_cur = kPushKdCap; push_capt0 = now_s(); }
                        } else if (push_capt0 > 0 && now_s() - push_capt0 >= kPushHoldS) {
                            float Ts = kPushKdCap * kPushDq;
                            push_res.push_back({sg.x_tgt, (int)dir, Ts, true});
                            std::printf(">>> [t=%.1fs] 挣脱 x=%.2f %s向: 未挣脱，Ts ≥ %.2f Nm\n",
                                        el, sg.x_tgt, dir > 0 ? "+" : "−", Ts);
                            csv_phase = 0;
                            rph = R_PUSHDN;
                        }
                        if (sel > kPushTmoS) { why = "挣脱段超时"; aborted = true; }
                        break;
                    }

                    case R_PUSHDN:
                        if (kd_cur < 1.0f) kd_cur = 1.0f;   // 恢复阻尼刹住 onset 后的残余运动
                        ramp_dq(0, kTrkAccel);
                        if (dq_cmd == 0 && std::fabs(fb.vel) < kStallVel) begin_seg(seg_i + 1);
                        break;

                    case R_CRUISE: {  // 500Hz 巡航，越过触发线 ⇒ 断力滑行
                        bool crossed = dir > 0 ? (x >= sg.x_tgt) : (x <= sg.x_tgt);
                        if (sel >= kCruiseMinS && crossed) {
                            kd_cur = 0; dq_cmd = 0;
                            coast_v0 = fb.vel;
                            coast_t0 = now_s();
                            coast_ref = travel; coast_n = 0; coast_frames = 0;
                            csv_phase = 2;
                            rph = R_COASTING;
                        } else if (sel > kCruiseTmoS) {
                            why = "coast 巡航未达触发线";
                            aborted = true;
                        }
                        break;
                    }

                    case R_COASTING:
                        coast_frames++;
                        if (std::fabs(travel - coast_ref) < kCoastStopD) coast_n++;
                        else { coast_n = 0; coast_ref = travel; }
                        if (coast_n >= kCoastStopN || now_s() - coast_t0 >= kCoastTmoS) {
                            coast_res.push_back({coast_v0, now_s() - coast_t0, coast_frames});
                            std::printf(">>> [t=%.1fs] coast#%zu: %+.3f rad/s → 停，"
                                        "%.0fms %d 帧（含静止确认 0.1s）\n",
                                        el, coast_res.size(), coast_v0,
                                        (now_s() - coast_t0) * 1000, coast_frames);
                            rph = R_TAIL;
                            seg_t0 = now_s();
                        }
                        break;

                    case R_TAIL:   // 停后尾录（仍 500Hz、kd=0）
                        if (sel >= kCoastTailS) {
                            csv_phase = 0;
                            begin_seg(seg_i + 1);
                        }
                        break;

                    default: break;
                }
                break;
            }

            case P_PARK: {   // 收尾回到近全开（伺服同 R_MOVE）
                float e = kParkX - x;
                if ((std::fabs(e) < kMoveNear && std::fabs(fb.vel) < kStallVel) ||
                    stalled || pel > kMoveTmoS) {
                    ph = P_BRAKE; ph_t0 = now_s();
                    break;
                }
                float mag = clampf(kMoveGain * std::fabs(e), kMoveMin, kMoveDqMax);
                float want = (std::fabs(e) < kMoveTol) ? 0.0f : (e > 0 ? mag : -mag);
                ramp_dq(want, kMoveAccel);
                break;
            }

            case P_BRAKE:
                if (kd_cur < 0.5f) kd_cur = 1.0f;
                ramp_dq(0, kMoveAccel);
                if (dq_cmd == 0 && pel >= 0.5) ph = P_DONE;
                break;

            default: break;
        }
        if (aborted || ph == P_DONE) break;

        if (!dmsc::send_mit(s, 0, dq_cmd, 0, kd_cur, 0)) { why = "发送失败"; aborted = true; break; }

        // 收本周期反馈
        double dl = now_s() + period_ms / 1000.0;
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

            if ((int)log.size() < kLogMax) {
                Sample sm{};
                sm.t = (float)el; sm.kts = kts;
                sm.pos = fb.pos; sm.x = travel - home_travel;
                sm.vel = fb.vel; sm.tau = fb.tau;
                sm.dqc = dq_cmd; sm.kdc = kd_cur;
                sm.xref = trk_xref;
                sm.tmos = fb.tmos; sm.trotor = fb.trotor;
                sm.seg = (int8_t)(ph == P_RUN ? seg_i : -1);
                sm.phase = (uint8_t)csv_phase;
                log.push_back(sm);
            }
        }

        // ---- 保护 ----
        if (have_fb && el > 0.3) {
            if (fb.err >= 8)  { why = dmsc::err_name(fb.err); aborted = true; break; }
            if (fb.err == 0)  { why = "意外失能"; aborted = true; break; }
            if (std::fabs(fb.vel) > kVelAbort) { why = "超速"; aborted = true; break; }
            if (std::fabs(fb.tau) > kTauAbort) { why = "力矩超限"; aborted = true; break; }
            if (fb.tmos > kTempAbort || fb.trotor > kTempAbort) { why = "过温"; aborted = true; break; }
            float push_now = kd_cur * (dq_cmd - fb.vel);
            if (std::fabs(push_now) > kPushAbortNm) {
                push_over_t += dt;
                if (push_over_t > kPushAbortS) { why = "瞬时推力超限"; aborted = true; break; }
            } else {
                push_over_t = 0;
            }
            float xg = travel - home_travel;
            if (!homed && std::fabs(travel) > kTravelGuard) { why = "找零行程超预期"; aborted = true; break; }
            if (homed && (xg < -kXMargin || xg > span + kXMargin)) { why = "行程越界"; aborted = true; break; }
        }

        if (el - last_print > 1.0) {
            last_print = el;
            const char* pn = ph == P_HOME ? "找零" : ph == P_RELAX ? "卸力" :
                             ph == P_RUN  ? (rph == R_MOVE ? "复位" : rph == R_RAMP ? "斜坡" :
                                             rph == R_SETTLE ? "稳定" : rph == R_MEASURE ? "测量" :
                                             rph == R_RAMPDN ? "降速" : rph == R_REST ? "静置" :
                                             rph == R_PUSH ? "推力" : rph == R_PUSHDN ? "撤力" :
                                             rph == R_CRUISE ? "巡航" : rph == R_COASTING ? "滑行" : "尾录") :
                             ph == P_PARK ? "停车" : ph == P_BRAKE ? "制动" : "宽限";
            std::printf("  t=%5.1fs 段#%d[%s] dq=%+5.2f kd=%.2f | ERR=%X x=%+7.4f "
                        "vel=%+6.3f tau=%+6.3f MOS=%d℃\n",
                        el, ph == P_RUN ? seg_i : -1, pn, dq_cmd, kd_cur, fb.err,
                        travel - home_travel, fb.vel, fb.tau, fb.tmos);
        }
    }

    if (aborted && why) std::printf("\n!!! 中止：%s（ERR=%X x=%.4f vel=%.3f tau=%.3f）\n",
                                    why, fb.err, travel - home_travel, fb.vel, fb.tau);

    for (int i = 0; i < 5; ++i) { dmsc::send_cmd(s, 0xFD); usleep(10 * 1000); }
    std::printf(">>> 已失能\n");

    // ---- 汇总 ----
    if (!plat_res.empty()) {
        std::printf("\n=== 平台汇总（%zu 段）===\n", plat_res.size());
        std::printf("  段  目标rad/s  kd    实测rad/s  达成率  反解Nm   遥测Nm(±σ)      粘滑  x窗        MOS℃\n");
        for (const PlatRes& r : plat_res)
            std::printf("  #%-2d %+7.3f  %.2f  %+9.4f  %5.1f%%  %.4f  %+.4f(±%.4f)  %3.0f%%  %.2f→%.2f  %d%s\n",
                        r.seg, r.v_tgt, r.kd, r.vm, r.att, r.fric_bs, r.tau_m, r.tau_sd,
                        r.stick_pct, r.x0, r.x1, r.tmos, r.stalled ? "  [未挣脱:下界]" : "");
    }
    if (!push_res.empty()) {
        std::printf("\n=== 挣脱汇总（%zu 试次）===\n", push_res.size());
        for (const PushRes& r : push_res)
            std::printf("  x=%.2f %s向  Ts %s %.3f Nm\n",
                        r.xpos, r.dir > 0 ? "+闭合" : "−张开", r.lower ? "≥" : "=", r.Ts);
    }
    if (!coast_res.empty()) {
        std::printf("\n=== coast 汇总（%zu 次滑停）===\n", coast_res.size());
        for (size_t i = 0; i < coast_res.size(); ++i)
            std::printf("  #%zu  ω0=%+.3f rad/s  %.0fms  %d 帧\n",
                        i + 1, coast_res[i].v0, coast_res[i].dur * 1000, coast_res[i].frames);
    }
    if (!track_res.empty()) {
        std::printf("\n=== 跟踪汇总%s ===\n", comp ? "（带前馈）" : "（无前馈）");
        for (const TrackRes& r : track_res)
            std::printf("  %s  RMS=%.4f rad  峰值=%.4f rad  换向滞后 均值 %.0fms 最大 %.0fms（%d 次）\n",
                        r.wave ? "正弦" : "三角", r.rms, r.peak,
                        r.lag_mean * 1000, r.lag_max * 1000, r.nlag);
    }

    if (!csv_path.empty() && !log.empty()) {
        FILE* f = std::fopen(csv_path.c_str(), "w");
        if (f) {
            std::fprintf(f, "t_s,kts_ns,pos_rad,travel_rad,vel_rad_s,tau_Nm,dq_cmd,kd_cmd,"
                            "t_mos,t_rotor,seg,phase,x_ref\n");
            for (const Sample& sm : log)
                std::fprintf(f, "%.3f,%llu,%.5f,%.5f,%.4f,%.4f,%.4f,%.3f,%d,%d,%d,%d,%.5f\n",
                             sm.t, (unsigned long long)sm.kts, sm.pos, sm.x, sm.vel, sm.tau,
                             sm.dqc, sm.kdc, sm.tmos, sm.trotor, sm.seg, sm.phase, sm.xref);
            std::fclose(f);
            std::printf("\n逐帧日志: %zu 行 -> %s\n", log.size(), csv_path.c_str());
            if ((int)log.size() >= kLogMax) std::printf("!! 日志缓冲已满(%d)\n", kLogMax);
        }
    }
    ::close(s);
    return aborted ? 2 : 0;
}
