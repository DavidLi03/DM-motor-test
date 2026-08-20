# SocketCAN 路线操作手册（slcan 固件）

刷写与桥接都要人工操作（刷写走上位机、桥接要 sudo），按顺序执行。
电机在 1Mbps 经典 CAN，ESC_ID=0x08 / MST_ID=0x18。

## 1. 刷 slcan 固件（上位机操作）

- 固件包：`../usb2canfd/firmware/socketcan/slcan-firmware/dm_usb2fdcan_slcan_1002.enc`
- 用模块配套上位机做固件升级（步骤见模块说明书；截图在固件目录 `img/` 下）。
- **回退**：重新刷入 `../usb2canfd/firmware/factory-firmware/dm_usb2canfd_app_1006.enc`
  （或与当前一致的 1005，在 `legacy-firmware/` 下）。厂商文档确认两者可互刷。
- 刷完 slcan 后模块不再被达妙上位机/SDK 识别，属预期现象。

## 2. 建立 CAN 接口（每次插入模块后执行一次）

```bash
# 确认虚拟串口出现（刷写成功的标志）
ls /dev/ttyACM*

# 桥接为 SocketCAN 接口。⚠ 波特率参数按【达妙自定义表】：-s5 = 1Mbps 经典 CAN。
# 这与标准 slcand 的映射（s8=1M）不同 —— slcand 只是把 "S5\r" 原样发给设备，
# 解释权在达妙固件侧。用错档位的症状：接口能起来但 candump 一帧收不到。
sudo slcand -o -c -s5 /dev/ttyACM0 can0

# 加大发送队列（默认 10，100Hz 控制流建议加大），然后启用
sudo ip link set can0 txqueuelen 1000
sudo ip link set up can0
```

达妙波特率表（节选）：`-s4`=500K、`-s5`=1000K（经典 CAN）；`-s6`=2M、`-s8`=5M（CANFD）。

## 3. 验证（零运动，安全）

```bash
cd build && cmake .. && make dm_probe_sc dm_batch_sc dm_spin_sc

# ① 通路验证：读回 10 个寄存器并与 SDK 路线实测值比对
./dm_probe_sc

# ② 投递时效验收：确认逐帧投递（出厂固件是 100ms 攒批 ⇒ 有效反馈率仅 10Hz）
./dm_batch_sc --csv batch_sc.csv
```

两步都只发 `0x33` 读命令，电机全程失能，不会转动。

## 4. 运动测试（需按安全约定先获批准）

```bash
# 与 SDK 版 dm_spin 参数完全一致，另加 --if 指定接口
./dm_spin_sc --kd 1.14 --seconds 3 --csv run_sc.csv
```

## 5. 收尾 / 拆除接口

```bash
sudo ip link set down can0
sudo pkill slcand
```

## 已知差异（SocketCAN vs 厂商 SDK 路线）

| | SDK（出厂固件） | SocketCAN（slcan 固件） |
|---|---|---|
| 反馈投递 | 100ms 定时攒批，有效 10Hz | 逐帧（厂商实测 9 电机@1kHz 可用） |
| 时间戳 | 模块 hw_ts（慢 3.71%，秒:纳秒结构） | 内核时间戳（主机时钟域，`SO_TIMESTAMPNS`） |
| 上位机调参 | 支持 | 不支持（需刷回出厂固件，或裸发 0x33/0x55 帧） |
| 电机侧看门狗 | 使能后静默 ~120ms 自动失能 —— **两条路线相同**，与模块固件无关 | 同左 |
