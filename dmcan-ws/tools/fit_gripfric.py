#!/usr/bin/env python3
# fit_gripfric.py —— 夹爪机构摩擦/阻力辨识分析（纯标准库，无 numpy）
#
# 输入：dm_gripfric_sc 产出的一个或多个 CSV（--sweep/--breakaway/--coast/--track
#   任意组合直接全部传入，按 phase 自动分派分析）。
# 与 fit_friction.py 的差异（夹爪有限行程往返版）：
#   * 平台窗门槛放宽并参数化（高速档窗只有 0.35~0.9s）；
#   * 保留带符号方向（+=闭合 / −=张开），分「闭合/张开/合并」三组拟合；
#   * 慢速平台（|ω|≤vmax）按位置分箱：双向均值 = 位置/重力项 g(x)，
#     双向半差 = 摩擦 f(x)；max|g| 显著时平台点扣 g(x_mid) 再进拟合；
#   * coast 段按 seg 分组（多次滑停重复），500Hz 采样，滑窗 7 帧；
#     J 双估计量：锚点法 model(ω)/|dω/dt| 中位 + 冲量法 ∫T̂dt/ω₀ 逐次；
#   * phase=3 挣脱试次、phase=4 轨迹跟踪（x_ref 列）单独分析；
#   * --json 输出 grip_fric.json（结构化 + 铺平键，供 C++ strstr 解析）。
#
# 用法: python3 fit_gripfric.py gfsw1.csv gfsw2.csv gfbk1.csv gfco1.csv \
#         [--gate 0.20] [--stall 0.02] [--win 7] [--min-dur 0.3] [--min-len 25] \
#         [--coast-vmin 0.15] [--map-vmax 0.14] [--map-bin 0.05] [--json grip_fric.json]
#       python3 fit_gripfric.py --selftest   # 合成数据端到端自检，不需要 CSV

import argparse
import csv
import json
import math
import sys
import time

# ---------- 小工具（同 fit_friction.py）----------

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
            sys.exit(f'{path}: 缺列 {sorted(missing)}（须为 dm_gripfric_sc 产出的 CSV）')
        for r in rd:
            rows.append({
                't': float(r['t_s']), 'trav': float(r['travel_rad']),
                'vel': float(r['vel_rad_s']), 'tau': float(r['tau_Nm']),
                'dq': float(r['dq_cmd']), 'kd': float(r['kd_cmd']),
                'tmos': float(r.get('t_mos', 'nan') or 'nan'),
                'trot': float(r.get('t_rotor', 'nan') or 'nan'),
                'seg': int(r['seg']), 'phase': int(r['phase']),
                'xref': float(r.get('x_ref', 'nan') or 'nan'),
            })
    return rows


def group_phase(rows, phase):
    """按 (seg==常量, phase) 分组，返回 {seg: [rows...]}（seg<0 丢弃）。"""
    groups = {}
    for r in rows:
        if r['phase'] == phase and r['seg'] >= 0:
            groups.setdefault(r['seg'], []).append(r)
    return groups


def extract_plateaus(rows, fname, stall_thresh, min_dur, min_len):
    """恒速平台（phase=1）→ 平台点列表（带符号 ω 与 T_app，带位置窗）。"""
    plats = []
    for seg, g in sorted(group_phase(rows, 1).items()):
        dur = g[-1]['t'] - g[0]['t']
        if dur < min_dur or len(g) < min_len:
            continue
        omega = (g[-1]['trav'] - g[0]['trav']) / dur     # 带符号（travel 列即 x）
        dq = median([r['dq'] for r in g])                # 带符号
        kd = median([r['kd'] for r in g])
        sign = 1.0 if dq >= 0 else -1.0
        t_app = kd * (dq - omega)                        # 带符号施加力矩（稳态）
        tau_m, tau_sd = mean_sd([r['tau'] for r in g])
        stick = sum(1 for r in g if r['vel'] * sign <= 0) / len(g)
        plats.append({
            'file': fname, 'seg': seg, 'dir': sign, 'dq': abs(dq), 'kd': kd,
            'omega': abs(omega), 'att': abs(omega) / abs(dq) * 100 if dq else 0,
            't_app': t_app,
            't_bs': sign * t_app,                        # 摩擦幅值（扣 g 前）
            'tau_m': tau_m, 'tau_sd': tau_sd,
            'x0': g[0]['trav'], 'x1': g[-1]['trav'],
            'xmid': 0.5 * (g[0]['trav'] + g[-1]['trav']),
            'stick': stick * 100,
            'tmos': median([r['tmos'] for r in g]),
            'trot': median([r['trot'] for r in g]),
            'stalled': abs(omega) < stall_thresh,
        })
    return plats


def extract_tau_map(rows, seg_omegas, vmax, binw):
    """慢速平台逐帧按位置分箱 × 方向。返回 {bin_center: {+1:[T...], -1:[T...]}}。
    T_inst = kd·(dq − vel)（带符号瞬时施加力矩，遥测 vel，箱内平均消量化噪声）。"""
    bins = {}
    for seg, g in sorted(group_phase(rows, 1).items()):
        om = seg_omegas.get(seg)
        if om is None or abs(om) > vmax or abs(om) < 1e-4:
            continue
        d = 1 if om > 0 else -1
        for r in g:
            c = (math.floor(r['trav'] / binw) + 0.5) * binw
            bins.setdefault(round(c, 4), {1: [], -1: []})[d].append(
                r['kd'] * (r['dq'] - r['vel']))
    return bins


def gmap_from_bins(bins):
    """双向均值/半差分离：g(x) 位置项、f(x) 摩擦项。
    双向数据中「g 的常值部分」与「摩擦方向不对称」不可分 ⇒ 约定：
    g(x) 只保留零均值的位置变化项，常值 g_const 并入分方向摩擦模型（拟合自然吸收）。
    返回 ([(x, g_零均值, f, n+, n-)], g_const)。"""
    raw = []
    for c in sorted(bins):
        tp, tn = bins[c][1], bins[c][-1]
        if len(tp) < 20 or len(tn) < 20:
            continue
        mp, mn = sum(tp) / len(tp), sum(tn) / len(tn)
        raw.append((c, 0.5 * (mp + mn), 0.5 * (mp - mn), len(tp), len(tn)))
    if not raw:
        return [], 0.0
    gc = sum(e[1] for e in raw) / len(raw)
    return [(x, g - gc, f, np_, nn) for x, g, f, np_, nn in raw], gc


def g_at(gmap, x):
    """gmap 上取最近箱的 g(x)；空表返回 0。"""
    if not gmap:
        return 0.0
    return min(gmap, key=lambda e: abs(e[0] - x))[1]


def extract_coast(rows, fname, win):
    """coast（phase=2）按 seg 分组 → 每次滑停一个 rep：
    {dir, w0, t_stop, n, samples:[(|ω|,|dω/dt|)...]}。滑窗二次拟合 travel(t)。"""
    reps = []
    win |= 1
    half = win // 2
    for seg, g in sorted(group_phase(rows, 2).items()):
        if len(g) < win + 2:
            continue
        # ω₀：前 1/3（≥4 帧）线性拟合斜率
        n0 = max(4, min(10, len(g) // 3))
        ts0 = [r['t'] - g[0]['t'] for r in g[:n0]]
        ys0 = [r['trav'] for r in g[:n0]]
        c0 = solve_normal([[1.0] * n0, ts0], ys0)
        w0 = c0[1] if c0 else 0.0
        # t_stop：到行程不再变化（末值 0.002 rad 内）为止
        trav_end = g[-1]['trav']
        t_stop = 0.0
        for r in g:
            if abs(r['trav'] - trav_end) > 0.002:
                t_stop = r['t'] - g[0]['t']
        samples = []
        for i in range(half, len(g) - half):
            w = g[i - half:i + half + 1]
            t0 = w[half]['t']
            ts = [r['t'] - t0 for r in w]
            ys = [r['trav'] for r in w]
            c = solve_normal([[1.0] * len(ts), ts, [t * t for t in ts]], ys)
            if c is None:
                continue
            samples.append((abs(c[1]), abs(2.0 * c[2])))
        reps.append({'file': fname, 'seg': seg, 'dir': 1 if w0 >= 0 else -1,
                     'w0': abs(w0), 't_stop': t_stop, 'n': len(g),
                     'samples': samples})
    return reps


def extract_breakaway(rows, fname, onset=0.01):
    """挣脱（phase=3）按 seg 分组 → 每试次 {pos, dir, Ts, lower}。"""
    outs = []
    for seg, g in sorted(group_phase(rows, 3).items()):
        if len(g) < 10:
            continue
        x0 = g[0]['trav']
        d = 1 if median([r['dq'] for r in g]) >= 0 else -1
        hit = None
        for r in g:
            if abs(r['trav'] - x0) > onset:
                hit = r
                break
        if hit:
            # 真 onset 时刻 ω=0 ⇒ Ts = 当时施加推力 kd·|dq|（不能扣 vel）
            ts = abs(hit['kd'] * hit['dq'])
            outs.append({'file': fname, 'seg': seg, 'pos': x0, 'dir': d,
                         'Ts': ts, 'lower': False})
        else:
            ts = max(abs(r['kd'] * r['dq']) for r in g)
            outs.append({'file': fname, 'seg': seg, 'pos': x0, 'dir': d,
                         'Ts': ts, 'lower': True})
    return outs


def analyze_track(rows, fname, rev_d=0.005):
    """轨迹跟踪（phase=4）按 seg 分组 → {n, rms, peak, lags:[s...]}。"""
    outs = []
    for seg, g in sorted(group_phase(rows, 4).items()):
        g = [r for r in g if not math.isnan(r['xref'])]
        if len(g) < 100:
            continue
        errs = [r['xref'] - r['trav'] for r in g]
        rms = math.sqrt(sum(e * e for e in errs) / len(errs))
        peak = max(abs(e) for e in errs)
        # 参考方向翻转 → 实际行程跟着翻向的时差
        lags = []
        last_d, wait, flip_t, flip_x, new_d = 0, False, 0.0, 0.0, 0
        for i in range(1, len(g)):
            dref = g[i]['xref'] - g[i - 1]['xref']
            d = 1 if dref > 0 else (-1 if dref < 0 else last_d)
            if last_d != 0 and d != last_d:
                wait, flip_t, flip_x, new_d = True, g[i]['t'], g[i]['trav'], d
            last_d = d
            if wait and (g[i]['trav'] - flip_x) * new_d > rev_d:
                lags.append(g[i]['t'] - flip_t)
                wait = False
        outs.append({'file': fname, 'seg': seg, 'n': len(g), 'rms': rms,
                     'peak': peak, 'lags': lags})
    return outs


# ---------- 模型拟合（同 fit_friction.py）----------

def fit_model(pts):
    """pts: [(ω, T)]。T_f(ω)=Tc+(Ts−Tc)exp(−(ω/ωs)²)+bω。
    比 fit_friction.py 多两条物理约束（数据速度域窄，无约束时病态）：
      Ts ≤ 2·max(T)（防 ωs→数据下界以下时用尖峰穿单点）、b ≤ 0.06（裸电机 ~0.01）。"""
    if len(pts) < 4:
        return None
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    n = len(xs)
    ym = sum(ys) / n
    y_cap = 2.0 * max(ys)
    sstot = sum((y - ym) ** 2 for y in ys) or 1e-12
    best = None
    ws_grid = [0.02 * (2.0 / 0.02) ** (i / 79) for i in range(80)]
    for ws in ws_grid:
        e = [math.exp(-(x / ws) ** 2) for x in xs]
        ones = [1.0] * n
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
            if tc + c1 > y_cap or b > 0.06:
                continue
            sse = 0.0
            for x, y in zip(xs, ys):
                yy = tc + c1 * math.exp(-(x / ws) ** 2) + b * x
                sse += (y - yy) ** 2
            if best is None or sse < best['sse']:
                best = {'Tc': tc, 'Ts': tc + c1, 'ws': ws, 'b': b,
                        'sse': sse, 'r2': 1 - sse / sstot, 'n': n}
            break
    return best


def model_eval(m, x):
    return m['Tc'] + (m['Ts'] - m['Tc']) * math.exp(-(x / m['ws']) ** 2) + m['b'] * x


def estimate_J(coast_reps, model, vmin):
    """双估计量：锚点法（各 rep 滑窗样本合并）+ 冲量法（逐 rep）。"""
    anchors = []
    for rep in coast_reps:
        for w, acc in rep['samples']:
            if w > vmin and acc > 1e-6:
                anchors.append(model_eval(model, w) / acc)
    imps = []
    for rep in coast_reps:
        if rep['w0'] < 0.3 or rep['t_stop'] <= 0:
            continue
        n = 40
        tbar = sum(model_eval(model, rep['w0'] * (i + 0.5) / n) for i in range(n)) / n
        imps.append(tbar * rep['t_stop'] / rep['w0'])
    return {
        'anchor_med': median(anchors) if anchors else float('nan'),
        'anchor_lo': min(anchors) if anchors else float('nan'),
        'anchor_hi': max(anchors) if anchors else float('nan'),
        'n_anchor': len(anchors),
        'impulse_med': median(imps) if imps else float('nan'),
        'impulse_lo': min(imps) if imps else float('nan'),
        'impulse_hi': max(imps) if imps else float('nan'),
        'n_rep': len(imps),
    }


# ---------- JSON 输出 ----------

def write_grip_json(path, fits, bk_stats, gmap, g_const, jstats, tmos_ref, files):
    fc, fo = fits.get('闭合'), fits.get('张开')
    if not (fc and fo):
        print(f'\n!! 闭合/张开拟合不全，不写 {path}')
        return False

    def bk(di, fallback):
        s = bk_stats.get(di)
        return s['med'] if s and s['n'] else fallback

    data = {
        'generated_by': 'fit_gripfric.py',
        'date': time.strftime('%Y-%m-%d %H:%M'),
        'files': files,
        'tmos_ref_C': round(tmos_ref, 1),
        'temp_coeff_per_C': -0.023,     # 先验：裸电机 −14%/+6℃（fit_friction 2026-08-03）
        'close': {k: round(fc[k], 5) for k in ('Tc', 'Ts', 'ws', 'b', 'r2', 'n')},
        'open':  {k: round(fo[k], 5) for k in ('Tc', 'Ts', 'ws', 'b', 'r2', 'n')},
        'breakaway': {
            ('close' if d > 0 else 'open'): (
                {k: round(v, 4) if isinstance(v, float) else v
                 for k, v in bk_stats[d].items()} if d in bk_stats else None)
            for d in (1, -1)},
        'gmap_zeromean': [[round(x, 3), round(g, 4), round(f, 4)] for x, g, f, *_ in gmap],
        'g_const': round(g_const, 4),
        'J': ({k: (round(v, 6) if isinstance(v, float) and not math.isnan(v) else v)
               for k, v in jstats.items()} if jstats else None),
        # ---- 铺平键：dm_gripctl_sc / dm_gripfric_sc 的 strstr 极简解析用 ----
        'ff_Tc_close': round(fc['Tc'], 5), 'ff_Ts_close': round(fc['Ts'], 5),
        'ff_ws_close': round(fc['ws'], 5), 'ff_b_close': round(fc['b'], 5),
        'ff_Tc_open': round(fo['Tc'], 5), 'ff_Ts_open': round(fo['Ts'], 5),
        'ff_ws_open': round(fo['ws'], 5), 'ff_b_open': round(fo['b'], 5),
        'ff_Ts_bk_close': round(bk(1, fc['Ts']), 5),
        'ff_Ts_bk_open': round(bk(-1, fo['Ts']), 5),
        'ff_tmos_ref': round(tmos_ref, 1),
    }
    with open(path, 'w') as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
    print(f'\n参数已写入 {path}（含铺平键 ff_*，C++ 直接可读）')
    return True


# ---------- 主分析流程 ----------

def run_analysis(args):
    all_plats, all_coast, all_break, all_track = [], [], [], []
    tau_bins = {}
    for path in args.csvs:
        rows = load_csv(path)
        plats = extract_plateaus(rows, path, args.stall, args.min_dur, args.min_len)
        all_plats += plats
        seg_omegas = {p['seg']: p['dir'] * p['omega'] for p in plats}
        for c, dd in extract_tau_map(rows, seg_omegas, args.map_vmax,
                                     args.map_bin).items():
            tb = tau_bins.setdefault(c, {1: [], -1: []})
            tb[1] += dd[1]
            tb[-1] += dd[-1]
        all_coast += extract_coast(rows, path, args.win)
        all_break += extract_breakaway(rows, path)
        all_track += analyze_track(rows, path)

    # ---- g(x) 位置项 ----
    gmap, g_const = gmap_from_bins(tau_bins)
    g_max = max((abs(g) for _, g, *_ in gmap), default=0.0)
    use_g = g_max >= 0.02
    if gmap:
        print('## 位置分箱（慢速双向：均值=位置项 g(x)，半差=摩擦 f(x)）\n')
        print('| x (rad) | g(x) Nm | f(x) Nm | n+ | n− |')
        print('|---|---|---|---|---|')
        for x, g, f, np_, nn in gmap:
            print(f'| {x:.3f} | {g:+.4f} | {f:.4f} | {np_} | {nn} |')
        print(f'\n常值项 g_const = {g_const:+.4f} Nm（g 均值与方向不对称的混合，'
              f'已并入分方向摩擦模型，g(x) 表为零均值）')
        print(f'max|g| = {g_max:.4f} Nm → '
              f'{"显著，平台点扣 g(x_mid) 再拟合" if use_g else "不显著（<0.02），不修正"}')

    # ---- 平台点明细 ----
    fit_pts = {1: [], -1: []}
    stall_bounds = []
    if all_plats:
        print('\n## 平台点明细（+=闭合 −=张开）\n')
        print('| 文件 | 段 | 目标rad/s | kd | 实测ω | 达成率 | T_f | |tau|均值 '
              '| 偏差 | 粘滑 | x窗 | MOS℃ | 判定 |')
        print('|---|---|---|---|---|---|---|---|---|---|---|---|---|')
        for p in all_plats:
            tf = p['dir'] * (p['t_app'] - (g_at(gmap, p['xmid']) if use_g else 0.0))
            agree = abs(abs(p['tau_m']) - abs(tf)) / max(abs(tf), 1e-9)
            if p['stalled']:
                verdict = '未挣脱(下界)'
                stall_bounds.append(p['t_bs'])
            else:
                verdict = '入拟合' + (f'+偏差{agree*100:.0f}%⚑' if agree > args.gate else '')
                fit_pts[int(p['dir'])].append((p['omega'], tf))
            print(f"| {p['file']} | {p['seg']} | {p['dir']*p['dq']:+.3f} | {p['kd']:.2f} "
                  f"| {p['dir']*p['omega']:+.4f} | {p['att']:.0f}% | {tf:.4f} "
                  f"| {abs(p['tau_m']):.4f}±{p['tau_sd']:.4f} | {agree*100:.0f}% "
                  f"| {p['stick']:.0f}% | {p['x0']:.2f}→{p['x1']:.2f} "
                  f"| {p['tmos']:.0f} | {verdict} |")
        if stall_bounds:
            print(f'\n未挣脱平台的静摩擦下界: Ts >= {max(stall_bounds):.3f} Nm')

    # ---- 模型拟合 ----
    fits = {}
    if fit_pts[1] or fit_pts[-1]:
        print('\n## 模型拟合  T_f(ω) = Tc + (Ts−Tc)·exp(−(ω/ωs)²) + b·ω\n')
        print('| 数据集 | 点数 | Tc (Nm) | Ts (Nm) | ωs (rad/s) | b (Nm·s/rad) | R² |')
        print('|---|---|---|---|---|---|---|')
        for name, pts in (('闭合', fit_pts[1]), ('张开', fit_pts[-1]),
                          ('合并', fit_pts[1] + fit_pts[-1])):
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
            for x, y in sorted(fit_pts[1] + fit_pts[-1]):
                print(f'  ω={x:7.4f}  实测 {y:.4f}  模型 {model_eval(pooled, x):.4f} '
                      f' Δ={y - model_eval(pooled, x):+.4f} Nm')

    # ---- 挣脱 ----
    bk_stats = {}
    if all_break:
        print(f'\n## 挣脱试次（{len(all_break)} 次）\n')
        print('| 文件 | 位置 x | 方向 | Ts (Nm) | 备注 |')
        print('|---|---|---|---|---|')
        for r in all_break:
            print(f"| {r['file']} | {r['pos']:.2f} | {'+闭合' if r['dir'] > 0 else '−张开'} "
                  f"| {'≥' if r['lower'] else ''}{r['Ts']:.3f} "
                  f"| {'未挣脱(下界)' if r['lower'] else ''} |")
        for d in (1, -1):
            vals = [r['Ts'] for r in all_break if r['dir'] == d and not r['lower']]
            if vals:
                bk_stats[d] = {'med': median(vals), 'min': min(vals),
                               'max': max(vals), 'n': len(vals)}
                print(f"{'+闭合' if d > 0 else '−张开'}: Ts 中位 {bk_stats[d]['med']:.3f} "
                      f"（{bk_stats[d]['min']:.3f}~{bk_stats[d]['max']:.3f}，"
                      f"{bk_stats[d]['n']} 次）")

    # ---- coast → J ----
    jstats = None
    pooled = fits.get('合并')
    if all_coast:
        print(f'\n## coast-down（{len(all_coast)} 次滑停）\n')
        print('| 文件 | 段 | 方向 | ω₀ rad/s | t_stop ms | 帧数 | 滑窗样本 |')
        print('|---|---|---|---|---|---|---|')
        for rep in all_coast:
            print(f"| {rep['file']} | {rep['seg']} | {'+' if rep['dir'] > 0 else '−'} "
                  f"| {rep['w0']:.3f} | {rep['t_stop']*1000:.0f} | {rep['n']} "
                  f"| {len(rep['samples'])} |")
        if pooled:
            jstats = estimate_J(all_coast, pooled, args.coast_vmin)
            print(f"\n锚点法  J = {jstats['anchor_med']:.5f} kg·m²  "
                  f"（{jstats['n_anchor']} 锚点，{jstats['anchor_lo']:.5f}"
                  f"~{jstats['anchor_hi']:.5f}）")
            print(f"冲量法  J = {jstats['impulse_med']:.5f} kg·m²  "
                  f"（{jstats['n_rep']} 次，{jstats['impulse_lo']:.5f}"
                  f"~{jstats['impulse_hi']:.5f}）")
        else:
            print('\n（无平台模型可锚定 → 请把 sweep CSV 一并传入）')

    # ---- track ----
    if all_track:
        print(f'\n## 轨迹跟踪（{len(all_track)} 段；A/B 对比请分别传 无/有 --comp 的 CSV）\n')
        print('| 文件 | 段 | 帧数 | RMS误差 rad | 峰值 rad | 换向滞后均值 ms | 最大 ms | 次数 |')
        print('|---|---|---|---|---|---|---|---|')
        for r in all_track:
            lm = median(r['lags']) if r['lags'] else float('nan')
            lx = max(r['lags']) if r['lags'] else float('nan')
            print(f"| {r['file']} | {r['seg']} | {r['n']} | {r['rms']:.4f} "
                  f"| {r['peak']:.4f} | {lm*1000:.0f} | {lx*1000:.0f} | {len(r['lags'])} |")

    # ---- JSON ----
    if args.json:
        tmos_ref = median([p['tmos'] for p in all_plats
                           if not math.isnan(p['tmos'])]) if all_plats else float('nan')
        write_grip_json(args.json, fits, bk_stats, gmap, g_const, jstats,
                        tmos_ref if not math.isnan(tmos_ref) else 0.0, args.csvs)


# ---------- 合成数据自检 ----------

def selftest():
    """已知模型 → 合成 CSV 行 → 走完整管线 → 校验参数复原。"""
    TC, TS, WS, B = {1: 0.18, -1: 0.16}, {1: 0.30, -1: 0.27}, 0.15, {1: 0.012, -1: 0.010}
    JT = 0.0075
    SPAN = 1.49

    def tf(d, w):
        return TC[d] + (TS[d] - TC[d]) * math.exp(-(w / WS) ** 2) + B[d] * w

    def gx(x):
        return 0.03 * math.cos(math.pi * x / SPAN)     # 位置项 ±0.03 Nm

    def steady_w(d, dq, kd, x):
        w = dq * 0.7
        for _ in range(60):
            w = dq - d * (tf(d, abs(w)) + d * gx(x)) / kd
        return w

    rows = []
    seg = 0
    bands = [(0.10, 5.0), (0.15, 4.0), (0.22, 3.0), (0.32, 2.5),
             (0.45, 2.0), (0.65, 1.5), (0.90, 1.2), (1.20, 0.9)]
    t = 0.0
    for v, kd in bands:
        for d in (1, -1):
            # 慢档全程扫，快档中段窗（与 C++ 守卫行为一致）
            lo, hi = (0.14, 1.36) if v <= 0.16 else (0.30, 1.20)
            x = lo + 0.01 if d > 0 else hi - 0.01
            while lo < x < hi:
                w = steady_w(d, v * d, kd, x)
                rows.append({'t': t, 'trav': x, 'vel': w, 'tau': d * tf(d, abs(w)) + gx(x),
                             'dq': v * d, 'kd': kd, 'tmos': 33, 'trot': 30,
                             'seg': seg, 'phase': 1, 'xref': float('nan')})
                x += w * 0.01
                t += 0.01
            seg += 1
            t += 1.0
    # coast：500Hz ODE
    for rep in range(4):
        for d in (1, -1):
            w, x = 0.95 * d, 0.62 if d > 0 else 0.88
            tt = 0.0
            while abs(w) > 1e-3:
                rows.append({'t': t + tt, 'trav': x, 'vel': w, 'tau': 0.0,
                             'dq': 0.0, 'kd': 0.0, 'tmos': 33, 'trot': 30,
                             'seg': seg, 'phase': 2, 'xref': float('nan')})
                acc = -(tf(d, abs(w)) + d * gx(x)) * d / JT
                w += acc * 0.002
                if w * d < 0:
                    w = 0.0
                x += w * 0.002
                tt += 0.002
            for _ in range(100):                     # 停后尾录 0.2s
                rows.append({'t': t + tt, 'trav': x, 'vel': 0.0, 'tau': 0.0,
                             'dq': 0.0, 'kd': 0.0, 'tmos': 33, 'trot': 30,
                             'seg': seg, 'phase': 2, 'xref': float('nan')})
                tt += 0.002
            seg += 1
            t += tt + 1.0
    # breakaway：kd 缓升，推力过 Ts 后以小速度起动（检出滞后 ⇒ Ts 估计带 ~+0.01 偏置）
    for xp in (0.25, 0.75, 1.25):
        for d in (1, -1):
            ts_true = TS[d] + d * gx(xp)             # 挣脱须压过 摩擦+位置项
            kd, x, tt = 0.0, xp, 0.0
            while True:
                moving = kd * 0.4 > ts_true
                if moving:
                    x += d * 0.05 * 0.01
                rows.append({'t': t + tt, 'trav': x, 'vel': 0.05 * d if moving else 0.0,
                             'tau': min(kd * 0.4, ts_true) * d, 'dq': 0.4 * d, 'kd': kd,
                             'tmos': 33, 'trot': 30, 'seg': seg, 'phase': 3,
                             'xref': float('nan')})
                if abs(x - xp) > 0.011:
                    break
                kd += 0.125 * 0.01
                tt += 0.01
            seg += 1
            t += tt + 1.0

    class A:
        pass
    a = A()
    a.stall, a.min_dur, a.min_len = 0.02, 0.3, 25
    a.gate, a.win, a.coast_vmin = 0.20, 7, 0.15
    a.map_vmax, a.map_bin = 0.14, 0.05

    plats = extract_plateaus(rows, 'synth', a.stall, a.min_dur, a.min_len)
    seg_omegas = {p['seg']: p['dir'] * p['omega'] for p in plats}
    bins = extract_tau_map(rows, seg_omegas, a.map_vmax, a.map_bin)
    gmap, g_const = gmap_from_bins(bins)
    coast = extract_coast(rows, 'synth', a.win)
    brk = extract_breakaway(rows, 'synth')

    ok = True

    def check(name, got, want, tol):
        nonlocal ok
        rel = abs(got - want) / max(abs(want), 1e-9)
        good = rel <= tol
        ok = ok and good
        print(f'  {"✓" if good else "✗"} {name}: 复原 {got:.4f} / 真值 {want:.4f} '
              f'(偏差 {rel*100:.1f}%, 容限 {tol*100:.0f}%)')

    print('== selftest：合成数据端到端 ==')
    print(f'平台 {len(plats)} 段  g(x) 箱 {len(gmap)}  coast {len(coast)} 次  '
          f'挣脱 {len(brk)} 次')
    gxs = [gx(x) for x, *_ in gmap]
    gxm = sum(gxs) / len(gxs) if gxs else 0.0
    gdev = max((abs(g - (gx(x) - gxm)) for x, g, f, *_ in gmap), default=9.9)
    good_g = gdev < 0.006
    ok = ok and good_g
    print(f'  {"✓" if good_g else "✗"} g(x) 全箱最大偏差 {gdev:.4f} Nm（去均值后，容限 0.006）')
    fit_pts = {1: [], -1: []}
    for p in plats:
        if not p['stalled']:
            tfv = p['dir'] * (p['t_app'] - g_at(gmap, p['xmid']))
            fit_pts[int(p['dir'])].append((p['omega'], tfv))
    for d, nm in ((1, '闭合'), (-1, '张开')):
        m = fit_model(fit_pts[d])
        if not m:
            print(f'  ✗ {nm} 拟合失败')
            ok = False
            continue
        check(f'{nm} Tc', m['Tc'], TC[d], 0.15)
        check(f'{nm} Ts', m['Ts'], TS[d], 0.15)
        check(f'{nm} b',  m['b'],  B[d],  0.60)   # b 在 ≤1.2 rad/s 区间弱可辨，宽容限
    pooled = fit_model(fit_pts[1] + fit_pts[-1])
    js = estimate_J(coast, pooled, a.coast_vmin)
    check('J 锚点法', js['anchor_med'], JT, 0.25)
    check('J 冲量法', js['impulse_med'], JT, 0.30)
    for d, nm in ((1, '+闭合'), (-1, '−张开')):
        vals = [r['Ts'] for r in brk if r['dir'] == d and not r['lower']]
        if vals:
            want = sum(TS[d] + d * gx(x) for x in (0.25, 0.75, 1.25)) / 3
            check(f'挣脱 Ts {nm}', median(vals), want, 0.15)
    print('== selftest', 'PASS ==' if ok else 'FAIL ==')
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csvs', nargs='*')
    ap.add_argument('--gate', type=float, default=0.20,
                    help='双法互证相对偏差标记门槛（默认 0.20，超限打 ⚑ 不剔除）')
    ap.add_argument('--stall', type=float, default=0.02,
                    help='|ω| 低于此值视为未挣脱 rad/s（默认 0.02）')
    ap.add_argument('--win', type=int, default=7,
                    help='coast 滑动窗帧数（默认 7，500Hz 下 14ms）')
    ap.add_argument('--min-dur', type=float, default=0.3, dest='min_dur',
                    help='平台窗最短时长 s（默认 0.3）')
    ap.add_argument('--min-len', type=int, default=25, dest='min_len',
                    help='平台窗最少帧数（默认 25）')
    ap.add_argument('--coast-vmin', type=float, default=0.15, dest='coast_vmin',
                    help='J 锚点最低速度 rad/s（默认 0.15）')
    ap.add_argument('--map-vmax', type=float, default=0.14, dest='map_vmax',
                    help='进位置分箱的平台实测 |ω| 上限（默认 0.14）')
    ap.add_argument('--map-bin', type=float, default=0.05, dest='map_bin',
                    help='位置分箱宽 rad（默认 0.05）')
    ap.add_argument('--json', help='输出 grip_fric.json 路径')
    ap.add_argument('--selftest', action='store_true',
                    help='合成数据端到端自检（不需要 CSV）')
    args = ap.parse_args()

    if args.selftest:
        sys.exit(selftest())
    if not args.csvs:
        ap.error('请给出至少一个 CSV，或用 --selftest')
    run_analysis(args)


if __name__ == '__main__':
    main()
