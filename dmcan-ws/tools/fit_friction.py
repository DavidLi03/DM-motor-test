#!/usr/bin/env python3
# fit_friction.py —— DM-J4310 摩擦模型拟合（纯标准库，无 numpy 依赖）
#
# 输入：dm_fric_sc 产出的一个或多个 CSV（列含 seg/phase；多遍运行直接全部传入合并）。
# 处理：
#   1) 平台段（phase=1）→ 每平台一个 (ω, T_f) 点，双法互证：
#        反解法 T_bs = kd*(|dq_cmd| - |ω_位移法|)   （主值，用于拟合）
#        遥测法 |tau 均值|                          （互证，偏差>gate 的点剔除）
#      |ω| < stall 阈的平台视为「未挣脱」：只给静摩擦下界，不进曲线。
#   2) coast 段（phase=2）→ 滑动窗二次拟合 travel(t) 得 ω(t)、dω/dt(t)，
#      与平台点锚定反推 J = T_f(ω)/|dω/dt|（取中位数）。
#   3) 模型拟合（分方向 + 合并）：
#        T_f(ω) = Tc + (Ts-Tc)*exp(-(ω/ωs)^2) + b*ω     （ω>0）
#      ωs 网格扫描 + 3 参数线性最小二乘（正规方程），约束 Ts-Tc>=0、b>=0。
#
# 用法: python3 fit_friction.py fricL1.csv fricM1.csv fricH1.csv ...
#         [--gate 0.15] [--stall 0.02] [--win 9]

import argparse
import csv
import math
import sys

# ---------- 小工具 ----------

def median(xs):
    s = sorted(xs)
    n = len(s)
    if n == 0:
        return float('nan')
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


def mean_sd(xs):
    n = len(xs)
    if n == 0:
        return float('nan'), float('nan')
    m = sum(xs) / n
    var = sum((x - m) ** 2 for x in xs) / n
    return m, math.sqrt(var)


def solve_normal(cols, y):
    """最小二乘 y ~ cols（列向量列表）。返回系数列表；奇异时返回 None。"""
    k = len(cols)
    a = [[sum(cols[i][t] * cols[j][t] for t in range(len(y))) for j in range(k)]
         for i in range(k)]
    b = [sum(cols[i][t] * y[t] for t in range(len(y))) for i in range(k)]
    # 高斯消元（带主元）
    for col in range(k):
        piv = max(range(col, k), key=lambda r: abs(a[r][col]))
        if abs(a[piv][col]) < 1e-12:
            return None
        a[col], a[piv] = a[piv], a[col]
        b[col], b[piv] = b[piv], b[col]
        for r in range(k):
            if r == col:
                continue
            f = a[r][col] / a[col][col]
            for c in range(col, k):
                a[r][c] -= f * a[col][c]
            b[r] -= f * b[col]
    return [b[i] / a[i][i] for i in range(k)]


# ---------- 数据加载与切段 ----------

def load_csv(path):
    rows = []
    with open(path, newline='') as f:
        rd = csv.DictReader(f)
        need = {'t_s', 'travel_rad', 'vel_rad_s', 'tau_Nm', 'dq_cmd', 'kd_cmd',
                'seg', 'phase'}
        missing = need - set(rd.fieldnames or [])
        if missing:
            sys.exit(f'{path}: 缺列 {sorted(missing)}（须为 dm_fric_sc 产出的 CSV）')
        for r in rd:
            rows.append({
                't': float(r['t_s']), 'trav': float(r['travel_rad']),
                'vel': float(r['vel_rad_s']), 'tau': float(r['tau_Nm']),
                'dq': float(r['dq_cmd']), 'kd': float(r['kd_cmd']),
                'tmos': float(r.get('t_mos', 'nan') or 'nan'),
                'trot': float(r.get('t_rotor', 'nan') or 'nan'),
                'seg': int(r['seg']), 'phase': int(r['phase']),
            })
    return rows


def extract_plateaus(rows, fname, stall_thresh):
    """按 (seg, phase==1) 分组 → 平台点列表。"""
    plats = []
    groups = {}
    for r in rows:
        if r['phase'] == 1 and r['seg'] >= 0:
            groups.setdefault(r['seg'], []).append(r)
    for seg, g in sorted(groups.items()):
        dur = g[-1]['t'] - g[0]['t']
        if dur < 1.0 or len(g) < 50:
            continue
        omega = (g[-1]['trav'] - g[0]['trav']) / dur
        dq = median([abs(r['dq']) for r in g])
        kd = median([r['kd'] for r in g])
        sign = 1.0 if median([r['dq'] for r in g]) >= 0 else -1.0
        t_bs = kd * (dq - abs(omega))
        tau_m, tau_sd = mean_sd([r['tau'] for r in g])
        stick = sum(1 for r in g if r['vel'] * sign <= 0) / len(g)
        tmos = median([r['tmos'] for r in g])
        trot = median([r['trot'] for r in g])
        plats.append({
            'file': fname, 'seg': seg, 'dir': sign, 'dq': dq, 'kd': kd,
            'omega': abs(omega), 'att': abs(omega) / dq * 100 if dq else 0,
            't_bs': t_bs, 'tau_m': tau_m, 'tau_sd': tau_sd,
            'stick': stick * 100, 'tmos': tmos, 'trot': trot,
            'stalled': abs(omega) < stall_thresh,
        })
    return plats


def extract_coast(rows, win):
    """coast 段滑动窗二次拟合 → [(|ω|, |dω/dt|), ...]"""
    g = [r for r in rows if r['phase'] == 2]
    if len(g) < win + 2:
        return []
    half = win // 2
    out = []
    for i in range(half, len(g) - half):
        w = g[i - half:i + half + 1]
        t0 = w[half]['t']
        ts = [r['t'] - t0 for r in w]
        ys = [r['trav'] for r in w]
        c = solve_normal([[1.0] * len(ts), ts, [t * t for t in ts]], ys)
        if c is None:
            continue
        omega, acc = c[1], 2.0 * c[2]
        if abs(omega) > 0.3:                      # 低速端量化噪声大，截掉
            out.append((abs(omega), abs(acc)))
    return out


# ---------- 模型拟合 ----------

def fit_model(pts):
    """pts: [(ω, T)]。返回 dict(Tc, Ts, ws, b, r2, sse) 或 None。"""
    if len(pts) < 4:
        return None
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    n = len(xs)
    ym = sum(ys) / n
    sstot = sum((y - ym) ** 2 for y in ys) or 1e-12
    best = None
    ws_grid = [0.02 * (2.0 / 0.02) ** (i / 79) for i in range(80)]   # 0.02→2.0 对数网格
    for ws in ws_grid:
        e = [math.exp(-(x / ws) ** 2) for x in xs]
        ones = [1.0] * n
        # 全 3 参数 → 违反约束时降列重拟合
        for use_e, use_b in ((True, True), (True, False), (False, True), (False, False)):
            cols = [ones] + ([e] if use_e else []) + ([xs] if use_b else [])
            c = solve_normal(cols, ys)
            if c is None:
                continue
            tc = c[0]
            c1 = c[1] if use_e else 0.0
            b = (c[2] if use_e else c[1]) if use_b else 0.0
            if c1 < 0 or b < 0 or tc <= 0:
                continue
            sse = 0.0
            for x, y in zip(xs, ys):
                yy = tc + c1 * math.exp(-(x / ws) ** 2) + b * x
                sse += (y - yy) ** 2
            if best is None or sse < best['sse']:
                best = {'Tc': tc, 'Ts': tc + c1, 'ws': ws, 'b': b,
                        'sse': sse, 'r2': 1 - sse / sstot}
            break   # 本 ws 下取第一个满足约束的（列数最多的）方案
    return best


def model_eval(m, x):
    return m['Tc'] + (m['Ts'] - m['Tc']) * math.exp(-(x / m['ws']) ** 2) + m['b'] * x


# ---------- 主流程 ----------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csvs', nargs='+')
    ap.add_argument('--gate', type=float, default=0.15,
                    help='双法互证相对偏差门槛（默认 0.15）')
    ap.add_argument('--stall', type=float, default=0.02,
                    help='|ω| 低于此值视为未挣脱 rad/s（默认 0.02）')
    ap.add_argument('--win', type=int, default=9,
                    help='coast 滑动窗帧数（默认 9，须为奇数）')
    args = ap.parse_args()

    plats, coast = [], []
    for path in args.csvs:
        rows = load_csv(path)
        plats += extract_plateaus(rows, path, args.stall)
        coast += extract_coast(rows, args.win | 1)

    if not plats:
        sys.exit('没有可用平台段（phase=1）。')

    # ---- 平台点明细 ----
    print('## 平台点明细\n')
    print('| 文件 | 目标rad/s | kd | 实测ω | 达成率 | 反解T_f | |tau|均值 | 互证偏差 | 粘滑 | MOS/线圈℃ | 判定 |')
    print('|---|---|---|---|---|---|---|---|---|---|---|')
    fit_pts = {+1: [], -1: []}
    stall_bounds = []
    for p in plats:
        agree = abs(abs(p['tau_m']) - p['t_bs']) / max(p['t_bs'], 1e-9)
        if p['stalled']:
            verdict = '未挣脱(下界)'
            # 真堵转时 = kd*dq（施加力矩）；蠕动时 = 实际摩擦，两者都是 Ts 的有效下界
            stall_bounds.append(p['t_bs'])
        elif agree > args.gate:
            verdict = f'剔除(偏差{agree*100:.0f}%)'
        else:
            verdict = '入拟合'
            fit_pts[int(p['dir'])].append((p['omega'], p['t_bs']))
        print(f"| {p['file']} | {p['dir']*p['dq']:+.3f} | {p['kd']:.3f} "
              f"| {p['omega']:.4f} | {p['att']:.0f}% | {p['t_bs']:.4f} "
              f"| {abs(p['tau_m']):.4f}±{p['tau_sd']:.4f} | {agree*100:.0f}% "
              f"| {p['stick']:.0f}% | {p['tmos']:.0f}/{p['trot']:.0f} | {verdict} |")
    if stall_bounds:
        print(f'\n未挣脱平台给出的静摩擦下界: Ts >= {max(stall_bounds):.3f} Nm')

    # ---- 模型拟合 ----
    print('\n## 模型拟合  T_f(ω) = Tc + (Ts-Tc)·exp(-(ω/ωs)²) + b·ω\n')
    print('| 数据集 | 点数 | Tc (Nm) | Ts (Nm) | ωs (rad/s) | b (Nm·s/rad) | R² |')
    print('|---|---|---|---|---|---|---|')
    fits = {}
    for name, pts in (('正转', fit_pts[+1]), ('反转', fit_pts[-1]),
                      ('合并', fit_pts[+1] + fit_pts[-1])):
        m = fit_model(pts)
        fits[name] = m
        if m:
            print(f'| {name} | {len(pts)} | {m["Tc"]:.4f} | {m["Ts"]:.4f} '
                  f'| {m["ws"]:.3f} | {m["b"]:.4f} | {m["r2"]:.4f} |')
        else:
            print(f'| {name} | {len(pts)} | - | - | - | - | 点数不足 |')

    pooled = fits.get('合并')
    if pooled:
        print('\n残差（合并模型）:')
        for x, y in sorted(fit_pts[+1] + fit_pts[-1]):
            print(f'  ω={x:7.4f}  实测 {y:.4f}  模型 {model_eval(pooled, x):.4f} '
                  f' Δ={y - model_eval(pooled, x):+.4f} Nm')

        # 历史参照点（kd 扫描 / dm_spin / demo1 / demo2）
        hist = [(0.065, 0.164), (0.175, 0.121), (2.775, 0.130), (8.48, 0.209)]
        print('\n历史工况点 vs 本次模型:')
        for x, y in hist:
            print(f'  ω={x:7.3f}  历史 {y:.3f}  模型 {model_eval(pooled, x):.4f} '
                  f' Δ={y - model_eval(pooled, x):+.4f} Nm')

    # ---- coast-down → J ----
    if coast and pooled:
        coast.sort()
        js = []
        for om in [p[0] for p in fit_pts[+1] + fit_pts[-1] if p[0] > 0.5]:
            # 在 coast 采样中找最近邻的 |dω/dt|
            lo = min(coast, key=lambda c: abs(c[0] - om))
            if abs(lo[0] - om) < 0.5 and lo[1] > 1e-6:
                js.append(model_eval(pooled, lo[0]) / lo[1])
        print(f'\n## coast-down（{len(coast)} 个采样窗）')
        if js:
            jm = median(js)
            jl, jh = min(js), max(js)
            print(f'转动惯量估计 J = {jm:.5f} kg·m²（{len(js)} 个锚点，范围 '
                  f'{jl:.5f}~{jh:.5f}）')
            print('coast 曲线独立复算 T_f = J·|dω/dt|（与平台模型对照）:')
            step = max(1, len(coast) // 12)
            for om, ac in coast[::step]:
                print(f'  ω={om:6.3f}  T_f(coast)={jm * ac:.4f}  '
                      f'T_f(模型)={model_eval(pooled, om):.4f} Nm')
        else:
            print('coast 段与平台点速度无重叠，J 无法锚定。')
    elif not coast:
        print('\n（无 coast 段数据 —— H 批 CSV 未传入或 coast 太短）')


if __name__ == '__main__':
    main()
