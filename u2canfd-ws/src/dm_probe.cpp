// dm_probe.cpp —— 达妙电机只读探测工具
//
// 安全性说明：本程序【只】发送手册文档化的「读参数」命令(0x33)，以及可选的
// 状态查询帧(0xCC)。绝不发送：
//   0xFC 使能 / 0xFD 失能 / 0xFE 存零点 / 0xFB 清错 / 0x55 写参数 / 0xAA 存参数
//   以及任何 MIT / 位置速度 / 速度 控制帧
// 因此电机在整个运行过程中保持失能状态，不会转动。
//
// 刻意【不】使用 damiao::Motor_Control —— 它的构造函数会调用 enable_all()，
// 从而改写控制模式寄存器并发送使能帧。这里直接使用底层 usb_class。

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "protocol/usb_class.h"

namespace {

constexpr uint32_t kQueryId = 0x7FF;  // 读/写参数命令的报文 ID

struct RegInfo {
    uint8_t rid;
    const char* name;
};

// 均为手册<寄存器列表>中的条目；标注 RO 的为只读，RW 的这里也仅做读取。
const RegInfo kRegs[] = {
    {0x0E, "sw_ver   软件版本号   (RO)"},
    {0x24, "sub_ver  子版本号     (RO)"},
    {0x25, "Boot_ver Boot版本号   (RO)"},
    {0x07, "MST_ID   反馈ID       (RW)"},
    {0x08, "ESC_ID   接收ID       (RW)"},
    {0x0A, "CTRL_MODE 控制模式    (RW)"},
    {0x23, "can_br   CAN波特率码  (RW)"},
    {0x15, "PMAX     位置映射范围 (RW)"},
    {0x16, "VMAX     速度映射范围 (RW)"},
    {0x17, "TMAX     扭矩映射范围 (RW)"},
    {0x10, "NPP      电机极对数   (RO)"},
    {0x14, "Gr       齿轮减速比   (RO)"},
};

// 与 damiao.cpp 中 Motor_Control::is_in_ranges 保持一致：
// 这些寄存器编号的返回值按 uint32 解释，其余按 float 解释。
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

std::atomic<int> g_frames{0};

}  // namespace

int main(int argc, char** argv) {
    std::string sn = "B79F842638FF84CC899D5DE57AE30005";
    uint32_t nom_baud = 1000000;
    uint32_t dat_baud = 1000000;  // <=1M ⇒ fillFDCANFrame 自动发经典 CAN 2.0B 帧
    uint16_t can_id = 0x08;
    bool do_status = false;
    bool dry_run = false;  // 只配置适配器波特率，不向 CAN 总线发送任何帧

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&]() -> std::string { return (i + 1 < argc) ? argv[++i] : ""; };
        if (a == "--sn")          sn = next();
        else if (a == "--nom")    nom_baud = std::stoul(next());
        else if (a == "--dat")    dat_baud = std::stoul(next());
        else if (a == "--id")     can_id = (uint16_t)std::stoul(next(), nullptr, 0);
        else if (a == "--status") do_status = true;
        else if (a == "--dry")    dry_run = true;
        else if (a == "--help") {
            std::printf(
                "用法: dm_probe [--sn SN] [--nom 1000000] [--dat 1000000]\n"
                "               [--id 0x08] [--status]\n"
                "  --status  额外发送 0xCC 状态查询帧(SDK 用法，手册未文档化)\n");
            return 0;
        }
    }

    std::printf("=== 达妙电机只读探测 ===\n");
    std::printf("适配器 SN : %s\n", sn.c_str());
    std::printf("波特率    : 仲裁 %u / 数据 %u  → 帧类型 %s\n", nom_baud, dat_baud,
                dat_baud > 1000000 ? "CAN FD (BRS)" : "经典 CAN 2.0B");
    std::printf("目标 CAN ID: 0x%02X\n", can_id);
    std::printf("本程序不会使能电机，电机不应转动。\n\n");

    usb_class usb(nom_baud, dat_baud, sn);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    usb.setFrameCallback([&](can_value_type& v) {
        g_frames++;
        int dlc = v.head.dlc;
        std::printf("  [RX] id=0x%03X %s dlc=%d data=", v.head.id,
                    v.head.can_type ? "FD " : "2.0", dlc);
        for (int i = 0; i < dlc && i < 64; ++i) std::printf("%02X ", v.data[i]);

        // 读参数应答: D0=CANID_L D1=CANID_H D2=0x33 D3=RID D4..D7=数据(小端)
        if (dlc >= 8 && v.data[2] == 0x33) {
            uint8_t rid = v.data[3];
            uint32_t raw = (uint32_t)v.data[4] | ((uint32_t)v.data[5] << 8) |
                           ((uint32_t)v.data[6] << 16) | ((uint32_t)v.data[7] << 24);
            std::printf(" | RID 0x%02X = ", rid);
            if (is_uint_reg(rid)) {
                std::printf("%u", raw);
                if (rid == 0x0A) std::printf(" (%s)", mode_name(raw));
                if (rid == 0x23) std::printf(" (%s)", baud_name(raw));
                if (rid == 0x07 || rid == 0x08) std::printf(" (0x%X)", raw);
            } else {
                float f;
                std::memcpy(&f, &raw, sizeof(f));
                std::printf("%g", f);
            }
        }
        std::printf("\n");
        std::fflush(stdout);
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    if (dry_run) {
        std::printf("\n[--dry] 仅测试适配器波特率配置，不发送任何 CAN 帧。\n");
        return 0;
    }

    std::printf("--- 逐个读取寄存器 (0x33 读参数，只读不写) ---\n");
    for (const auto& r : kRegs) {
        std::printf("[TX] 读 RID 0x%02X  %s\n", r.rid, r.name);
        std::vector<uint8_t> d = {(uint8_t)(can_id & 0xFF),
                                  (uint8_t)((can_id >> 8) & 0xFF), 0x33, r.rid};
        usb.fdcanFrameSend(d, kQueryId);
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
    }

    if (do_status) {
        std::printf("\n--- 状态查询 (0xCC) x5 ---\n");
        for (int i = 0; i < 5; ++i) {
            std::vector<uint8_t> d = {(uint8_t)(can_id & 0xFF),
                                      (uint8_t)((can_id >> 8) & 0xFF), 0xCC, 0x00};
            usb.fdcanFrameSend(d, kQueryId);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::printf("\n=== 共收到 %d 帧 ===\n", g_frames.load());
    if (g_frames.load() == 0) {
        std::printf(
            "没有收到任何帧。排查顺序：\n"
            "  1. 电机是否上电（48V）\n"
            "  2. CAN_H/CAN_L 是否接反、是否接触不良\n"
            "  3. 末端 120Ω 终端电阻是否安装\n"
            "  4. 电机 CAN ID 是否真的是 0x%02X（用 --id 试其他值）\n"
            "  5. 波特率是否匹配（--nom 试 500000 等）\n",
            can_id);
    }
    return 0;
}
