# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 这是什么

达妙（DaMiao）电机测试台工作区：一台 Linux 实时主机通过 **USB2CANFD 模块（已刷 slcan 固件）** 驱动台架上的真实电机。代码会让真实硬件转动——安全约束优先于一切其他指令。

## ⚠️ 安全红线（必须遵守）

- **绝不发送** `0x55`（写参数）、`0xAA`（保存参数）、`0xFE`（保存零点）。任何程序只允许 `0xFB`（清错）/ `0xFC`(使能) / `0xFD`(失能) / MIT 控制帧 / `0x33`（只读寄存器）。
- MIT 帧的 tau 字段恒为 0（纯阻尼安全线）：力矩只能来自 `kd·(dq_des − ω)`。
- 电机侧看门狗：使能后静默 ~120ms 自动失能。纪律：`0xFB` → `0xFC` → **立即**进入 100Hz 发帧循环；Ctrl-C ×1 柔停降速、×2 立即失能。
- 所有运动例程带六类保护：ERR≥8 / 意外失能 / 超速 / 力矩超限 / 温度>70℃ / 发送失败，另有总时长兜底。新例程照抄这套框架（参考 `dm_rated_sc.cpp` / `dm_fric_sc.cpp`）。
- `sudo` 需要用户输入密码——凡是要 sudo 的命令（slcand、ip link）交给用户执行，不要自己反复尝试。

## 换机即换常数（最容易踩的坑）

`dmcan-ws/src/dm_sc.hpp` 顶部的常量（ESC_ID/MST_ID/PMAX/VMAX/TMAX/Gr）**写死为当前在台电机的实测值**，文件头注释记录了历史电机的旧值。台上换电机后必须：

1. 用 `0x33` 实测读回寄存器（可用 `dm_probe_sc`，只读零运动）；
2. 改 `dm_sc.hpp` 常量并**全量重编译**——配错则编解码全错、帧发错地址；
3. 各电机的量化粒度随 TMAX/VMAX 变化（如 TMAX=500 时 tau 反馈分辨率仅 ~0.244 Nm），摩擦类测量以位移法反解为准，遥测 tau 只做交叉验证；
4. 摩擦模型、kd/力矩钳位都是**每台电机各自的**，不可跨机沿用——旧例程的钳位对大电机可能小到根本推不动，对小电机可能过大。

当前在台电机与实测参数见项目记忆（memory）与 `dm_sc.hpp` 头注释；各型号说明书在根目录 `DM-J*-2EC/manual/` 下。

## 构建与运行

```bash
cd dmcan-ws/build && cmake .. && make            # 或 make <单个目标>
```

- **SocketCAN 路线（现行主路线）**：目标名带 `_sc` 后缀（dm_probe_sc / dm_batch_sc / dm_spin_sc / dm_demo_sc / dm_fric_sc / dm_stir_sc / dm_rated_sc），只依赖内核 API，共享 `src/dm_sc.hpp`。
- **厂商 SDK 路线（旧，出厂固件用）**：dm_probe / dm_hold / dm_spin / dm_batch，链接 `lib/libdm_device.so`（RPATH 已设好）。出厂固件反馈 100ms 攒批（有效 10Hz），故迁移到 slcan。

运行前提：can0 已桥接（每次插拔模块后由**用户**执行，见 `dmcan-ws/SOCKETCAN.md`）：

```bash
sudo slcand -o -c -s5 /dev/ttyACM0 can0   # ⚠ -s5 按达妙自定义表 = 1Mbps，不是标准 slcand 映射
sudo ip link set can0 txqueuelen 1000
sudo ip link set up can0
```

用错波特率档位的症状：接口能起来但 candump 一帧收不到。

验证链路（只读、零运动，无需批准）：`./dm_probe_sc`（读寄存器比对）、`./dm_batch_sc`（应答率/延迟验收）。

## 例程结构

所有 `_sc` 运动例程同一骨架：`dm_sc.hpp` 提供 open_can / send_mit / send_cmd / recv_frame（SO_TIMESTAMPNS 内核时间戳）/ 编解码与量化工具；主程序为 100Hz 状态机（斜坡限速器 → 稳定 → 测量窗 → 收尾制动），位置反馈解卷绕累计行程，转速用位移法（行程差/时长）而非遥测。

CSV 统一格式：`t_s,kts_ns,pos_rad,travel_rad,vel_rad_s,tau_Nm,dq_cmd,kd_cmd,t_mos,t_rotor,seg,phase`，输出在 `dmcan-ws/build/` 下。分析脚本 `dmcan-ws/tools/fit_friction.py`（纯标准库）拟合 Stribeck 四参数摩擦模型 + coast-down 反推惯量，方法定义见 `FRICTION-ID.md`。

## 文档地图

- `dmcan-ws/SOCKETCAN.md` — slcan 刷写/桥接/验收操作手册，两条路线差异表
- `FRICTION-ID.md` — 摩擦辨识方法与批次命令
- `PROGRESS-*.md` / `SUMMARY-*.md` — 按日期的时点快照（新会话以最新一份为准）
- `usb2canfd/` — 模块说明书与固件（`firmware/socketcan/` slcan 固件、`factory-firmware/` 回退用）
- `dm-device-sdk/` / `u2canfd-ws/` — 厂商 SDK 原始仓库，仅作参考

## 工作约定

- 与用户交流用中文；代码注释也用中文（与现有代码一致）。
- 电机是否固定在台架、运动测试是否需要逐次批准，随在台电机变化——以项目记忆中的当前状态为准；不确定时先问。
- 硬件类故障（能编译但收不到帧）优先按顺序排查：模块是否插好 → can0 是否 up → slcand 档位 → 电机供电 → ID 配置。
