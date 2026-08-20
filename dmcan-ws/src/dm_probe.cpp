// dm_probe.cpp —— 达妙电机只读探测工具（基于新版 DM Device SDK）
//
// 为什么不用 usb2canfd 仓库里的旧版 SDK：
//   SDK/UPDATE.md 指明「旧版 SDK 不支持模块固件新版本，单路 USB2CANFD 1004 版本起
//   请谨慎核对兼容性」。实测旧版 SDK 的 CMD_SETUP_BUARD 被固件以 NACK_ARG_ERR(0x03)
//   拒绝，任何波特率组合都失败。
//
// 安全性说明：本程序【只】发送手册文档化的「读参数」命令(0x33)。绝不发送：
//   0xFC 使能 / 0xFD 失能 / 0xFE 存零点 / 0xFB 清错 / 0x55 写参数 / 0xAA 存参数
//   以及任何 MIT / 位置速度 / 速度 控制帧。
// 电机全程保持失能，不会转动。

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

#include "dmcan.h"

namespace {

constexpr uint32_t kQueryId = 0x7FF;  // 读/写参数命令的报文 ID

struct RegInfo {
    uint8_t rid;
    const char* name;
};

// 均取自 DM-J4310-2EC V1.2 手册<寄存器列表及范围>
const RegInfo kRegs[] = {
    {0x0E, "sw_ver    软件版本号"},
    {0x24, "sub_ver   子版本号"},
    {0x25, "Boot_ver  Boot版本号"},
    {0x07, "MST_ID    反馈ID"},
    {0x08, "ESC_ID    接收ID"},
    {0x0A, "CTRL_MODE 控制模式"},
    {0x09, "TIMEOUT   超时警报时间"},
    {0x23, "can_br    CAN波特率码"},
    {0x00, "UV_Value  低压保护值"},
    {0x1D, "OV_Value  过压保护值"},
    {0x02, "OT_Value  过温保护值"},
    {0x03, "OC_Value  过流保护值"},
    {0x15, "PMAX      位置映射范围"},
    {0x16, "VMAX      速度映射范围"},
    {0x17, "TMAX      扭矩映射范围"},
    {0x10, "NPP       电机极对数"},
    {0x14, "Gr        齿轮减速比"},
};

// 与旧版 damiao.cpp 的 is_in_ranges 一致：这些寄存器按 uint32 解释，其余按 float。
bool is_uint_reg(int rid) {
    return (rid >= 7 && rid <= 10) || (rid >= 13 && rid <= 16) ||
           (rid >= 35 && rid <= 36);
}

const char* mode_name(uint32_t m) {
    switch (m) {
        case 1: return "MIT";
        case 2: return "位置速度";
        case 3: return "速度";
        case 4: return "力位混控";
        default: return "未知";
    }
}

const char* baud_name(uint32_t c) {
    static const char* t[] = {"125K", "200K", "250K", "500K", "1M",
                              "2M",   "2.5M", "3.2M", "4M",   "5M"};
    return c < 10 ? t[c] : "未知";
}

std::atomic<int> g_rx{0};
std::atomic<int> g_err{0};

// 测量收帧延迟：记录最后一次发送时刻，收到应答时算差值。
// 目的是分清 ~100ms 的延迟出在电机、模块、还是主机 USB 接收路径。
using clk = std::chrono::steady_clock;
std::atomic<long long> g_tx_us{0};   // 最后一次 TX 的 steady_clock 微秒数
clk::time_point g_t0;

long long now_us() {
    return std::chrono::duration_cast<std::chrono::microseconds>(clk::now() - g_t0).count();
}

void on_recv(dmcan_device_handle*, usb_rx_frame_t* f) {
    g_rx++;
    long long dt = now_us() - g_tx_us.load();
    int len = dmcan_utils_get_len_from_dlc(f->head.dlc);
    // hw_ts 是模块给帧打的硬件时间戳；若 hw_ts 的间隔正常而主机 Δ 很大，
    // 说明延迟在 USB/主机侧而非 CAN 总线侧。
    std::printf("  [RX Δ=%6.1fms hw_ts=%llu] id=0x%03X %s dlc=%d len=%d data=",
                dt / 1000.0, (unsigned long long)f->head.time_stamp, f->head.can_id,
                f->head.canfd ? "FD " : "2.0", f->head.dlc, len);
    for (int i = 0; i < len && i < 64; ++i) std::printf("%02X ", f->payload[i]);

    // 读参数应答: D0=CANID_L D1=CANID_H D2=0x33 D3=RID D4..D7=数据(小端)
    if (len >= 8 && f->payload[2] == 0x33) {
        uint8_t rid = f->payload[3];
        uint32_t raw = (uint32_t)f->payload[4] | ((uint32_t)f->payload[5] << 8) |
                       ((uint32_t)f->payload[6] << 16) |
                       ((uint32_t)f->payload[7] << 24);
        std::printf(" | RID 0x%02X = ", rid);
        // 版本号寄存器既不是 uint 也不是 float，需按字节解释
        if (rid == 0x0E) {  // sw_ver: 4 个 ASCII 数字，如 "7118" → 前两位系列码/后两位版本号
            std::printf("\"%c%c%c%c\"  (系列 %c%c, 版本 V%c%c)\n", f->payload[4],
                        f->payload[5], f->payload[6], f->payload[7], f->payload[4],
                        f->payload[5], f->payload[6], f->payload[7]);
            std::fflush(stdout);
            return;
        }
        if (rid == 0x25) {  // Boot_ver: 逐字节版本号
            std::printf("%u.%u.%u.%u\n", f->payload[4], f->payload[5], f->payload[6],
                        f->payload[7]);
            std::fflush(stdout);
            return;
        }
        if (is_uint_reg(rid)) {
            std::printf("%u", raw);
            if (rid == 0x0A) std::printf("  (%s)", mode_name(raw));
            if (rid == 0x23) std::printf("  (%s)", baud_name(raw));
            if (rid == 0x07 || rid == 0x08) std::printf("  (0x%X)", raw);
        } else {
            float v;
            std::memcpy(&v, &raw, sizeof(v));
            std::printf("%g", v);
        }
    }
    std::printf("\n");
    std::fflush(stdout);
}

void on_err(dmcan_device_handle*, usb_rx_frame_t* f) {
    g_err++;
    std::printf("  [ERR] ch=%d id=0x%03X esi=%d\n", f->head.channel,
                f->head.can_id, f->head.esi);
    std::fflush(stdout);
}

}  // namespace

int main(int argc, char** argv) {
    uint32_t can_baud = 1000000;  // 电机侧 1Mbps ⇒ 手册规定其工作在经典 CAN 2.0B
    uint16_t can_id = 0x08;
    bool scan_ids = false;
    bool latency_test = false;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&]() -> std::string { return (i + 1 < argc) ? argv[++i] : ""; };
        if (a == "--baud")      can_baud = std::stoul(next());
        else if (a == "--id")   can_id = (uint16_t)std::stoul(next(), nullptr, 0);
        else if (a == "--scan") scan_ids = true;
        else if (a == "--latency") latency_test = true;
        else if (a == "--help") {
            std::printf("用法: dm_probe [--baud 1000000] [--id 0x08] [--scan] [--latency]\n"
                        "  --scan     遍历 CAN ID 0x01~0x10 各读一次 sw_ver\n"
                        "  --latency  25ms 间隔连发 20 次读，测量收帧延迟\n");
            return 0;
        }
    }

    g_t0 = clk::now();

    std::printf("=== 达妙电机只读探测 (DM Device SDK) ===\n");
    std::printf("CAN 波特率: %u (经典 CAN 2.0)\n", can_baud);
    std::printf("目标 CAN ID: 0x%02X\n", can_id);
    std::printf("本程序不使能电机，电机不应转动。\n\n");

    dmcan_context* ctx = nullptr;
    dmcan_context_create(&ctx);
    std::printf("--- SDK 版本 ---\n");
    dmcan_print_version(ctx);

    int n = dmcan_find_devices(ctx);
    std::printf("\n--- 找到 %d 个设备 ---\n", n);
    dmcan_show_all_devices(ctx);
    if (n <= 0) { dmcan_context_destroy(ctx); return 1; }

    dmcan_device_handle* dev = nullptr;
    if (!dmcan_device_get(ctx, &dev, 0)) {
        std::printf("dmcan_device_get 失败\n");
        dmcan_context_destroy(ctx); return 1;
    }
    if (!dmcan_device_open(dev)) {
        std::printf("dmcan_device_open 失败（检查 udev 权限）\n");
        dmcan_context_destroy(ctx); return 1;
    }
    std::printf("\n--- 模块固件版本 ---\n");
    dmcan_device_print_version(dev);

    if (!dmcan_device_enable_channel(dev, 0))
        std::printf("[警告] enable_channel(0) 返回 false\n");

    dmcan_channel_can_info_t info{};
    dmcan_device_get_channel_baudrate(dev, 0, &info);
    std::printf("\n当前通道配置: canfd=%d can=%u canfd_baud=%u sp=%.2f\n",
                info.canfd, info.can_baudrate, info.canfd_baudrate, info.can_sp);

    info.channel = 0;
    info.canfd = false;          // 关键：电机在 1Mbps ⇒ 经典 CAN 2.0B
    info.can_baudrate = can_baud;
    info.can_sp = 0.75f;
    if (!dmcan_device_set_channel_baudrate(dev, 0, info))
        std::printf("[警告] set_channel_baudrate 返回 false\n");
    else
        std::printf("已设为: 经典 CAN 2.0 @ %u, 采样点 75%%\n", can_baud);

    dmcan_device_hook_recv_callback(dev, on_recv);
    dmcan_device_hook_err_callback(dev, on_err);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    auto read_reg = [&](uint16_t id, uint8_t rid) {
        uint8_t d[4] = {(uint8_t)(id & 0xFF), (uint8_t)((id >> 8) & 0xFF), 0x33, rid};
        g_tx_us.store(now_us());
        // canfd=false, ext=false, rtr=false, brs=false ⇒ 纯经典 CAN 2.0 标准数据帧
        dmcan_device_send_can(dev, 0, kQueryId, false, false, false, false, 4, d);
    };

    if (latency_test) {
        // 以 25ms 间隔连发 20 次读 CTRL_MODE。若延迟是固定批处理周期，
        // 应答会成批到达（Δ 呈锯齿状）；若是恒定管道延迟，Δ 应稳定在同一值。
        std::printf("\n--- 收帧延迟测试：25ms 间隔连发 20 次读 0x0A ---\n");
        for (int k = 0; k < 20; ++k) {
            read_reg(can_id, 0x0A);
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }
    } else if (scan_ids) {
        std::printf("\n--- 扫描 CAN ID 0x01~0x10 (读 sw_ver) ---\n");
        for (uint16_t id = 0x01; id <= 0x10; ++id) {
            std::printf("[TX] id=0x%02X\n", id);
            read_reg(id, 0x0E);
            std::this_thread::sleep_for(std::chrono::milliseconds(120));
        }
    } else {
        std::printf("\n--- 读取寄存器 (0x33 读参数，只读不写) ---\n");
        for (const auto& r : kRegs) {
            std::printf("[TX] RID 0x%02X  %s\n", r.rid, r.name);
            read_reg(can_id, r.rid);
            std::this_thread::sleep_for(std::chrono::milliseconds(150));
        }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::printf("\n=== 收到 %d 帧, 错误 %d 次 ===\n", g_rx.load(), g_err.load());
    if (g_rx.load() == 0) {
        std::printf(
            "没有收到任何应答。排查顺序：\n"
            "  1. 电机是否上电（48V）\n"
            "  2. CAN_H/CAN_L 是否接反或接触不良\n"
            "  3. 末端 120Ω 终端电阻是否安装\n"
            "  4. CAN ID 是否真是 0x%02X —— 试 --scan 遍历\n"
            "  5. 波特率是否匹配 —— 试 --baud 500000 等\n",
            can_id);
    }

    dmcan_context_destroy(ctx);
    return 0;
}
