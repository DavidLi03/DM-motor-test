// dm_probe_sc.cpp —— SocketCAN 路线的通路验证：读回电机关键寄存器
//
// 用途：刷完 slcan 固件、slcand 建好接口后，第一个跑的程序。
//       能读回全部寄存器且值与 SDK 路线实测一致 ⇒ 固件/桥接/波特率全通。
//
// 【本程序不会让电机转动，也不会让它上电。】
//   唯一发送的是 0x7FF + 0x33【读】参数命令。全文没有 0xFC/0xFD/0xFB/0xFE/
//   MIT 帧，也没有 0x55/0xAA。电机全程保持失能。
//
// 用法: dm_probe_sc [--if can0]

#include <cstdio>
#include <string>

#include "dm_sc.hpp"

namespace {

struct RegDef { uint8_t rid; const char* name; bool is_float; };

// 关键寄存器与 2026-08-07 换机（DM10422）实测值（期望值），用于自动比对
const RegDef kRegs[] = {
    {0x07, "MST_ID",    false},   // 期望 0x11
    {0x08, "ESC_ID",    false},   // 期望 0x01
    {0x09, "TIMEOUT",   false},   // 期望 400
    {0x0A, "CTRL_MODE", false},   // 期望 1 (MIT)
    {0x10, "NPP",       false},   // 期望 16
    {0x14, "Gr",        true },   // 期望 22
    {0x15, "PMAX",      true },   // 期望 12.566
    {0x16, "VMAX",      true },   // 期望 20
    {0x17, "TMAX",      true },   // 期望 500
    {0x23, "can_br",    false},   // 期望 4 (=1Mbps)
    // 力矩换算链参数（Kt = 1.5*Npp*Flux*GR*GREF，手册 V1.2 第 7 页）与驱动器内置惯量
    {0x0C, "Inertia",   true },
    {0x13, "Flux",      true },
    {0x1E, "GREF",      true },
};

}  // namespace

int main(int argc, char** argv) {
    std::string ifname = "can0";
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--if" && i + 1 < argc) ifname = argv[++i];
        else if (a == "--help") {
            std::printf("用法: dm_probe_sc [--if can0]\n"
                        "SocketCAN 通路验证：只发 0x33 读参数命令，电机不会转动。\n");
            return 0;
        }
    }

    std::string err;
    int s = dmsc::open_can(ifname.c_str(), true, &err);
    if (s < 0) {
        std::printf("%s\n", err.c_str());
        return 1;
    }
    std::printf("=== SocketCAN 通路验证 @ %s ===\n", ifname.c_str());
    std::printf("只发 0x7FF + 0x33 读命令，电机保持【失能】，不会转动。\n\n");
    std::printf("%-10s %4s | %-10s %-12s | %s\n", "寄存器", "RID", "u32", "float", "首帧延迟");

    int got = 0;
    for (const RegDef& r : kRegs) {
        if (!dmsc::send_read_reg(s, r.rid)) {
            std::printf("%-10s 0x%02X | 发送失败: %s\n", r.name, r.rid, std::strerror(errno));
            continue;
        }
        timespec t0{};
        clock_gettime(CLOCK_MONOTONIC, &t0);

        // 等这一条的应答（应答也从 kMstId 来，payload[2]==0x33, [3]==rid）
        bool ok = false;
        for (int tries = 0; tries < 50 && !ok; ++tries) {
            can_frame fr{};
            uint64_t kts = 0;
            int rr = dmsc::recv_frame(s, &fr, &kts, 20);
            if (rr <= 0) continue;
            if (fr.can_id & CAN_ERR_FLAG) { std::printf("  [总线错误帧 0x%08X]\n", fr.can_id); continue; }
            if (fr.can_dlc < 8 || fr.data[2] != 0x33 || fr.data[3] != r.rid) continue;

            timespec t1{};
            clock_gettime(CLOCK_MONOTONIC, &t1);
            double lat_ms = (t1.tv_sec - t0.tv_sec) * 1e3 + (t1.tv_nsec - t0.tv_nsec) / 1e6;

            uint32_t u = (uint32_t)fr.data[4] | ((uint32_t)fr.data[5] << 8) |
                         ((uint32_t)fr.data[6] << 16) | ((uint32_t)fr.data[7] << 24);
            float f;
            std::memcpy(&f, &u, 4);
            if (r.is_float)
                std::printf("%-10s 0x%02X | %-10u %-12.4f | %.2f ms\n", r.name, r.rid, u, f, lat_ms);
            else
                std::printf("%-10s 0x%02X | %-10u %-12s | %.2f ms\n", r.name, r.rid, u, "-", lat_ms);
            ok = true;
            got++;
        }
        if (!ok) std::printf("%-10s 0x%02X | 1 秒内无应答\n", r.name, r.rid);
    }

    std::printf("\n%d/%d 个寄存器读回。\n", got, (int)(sizeof(kRegs) / sizeof(kRegs[0])));
    if (got == 0) {
        std::printf("一条都没收到时依次排查：\n"
                    "  1. candump %s 里能否看到发出的 0x7FF 帧（看不到 ⇒ 接口没起来）\n"
                    "  2. slcand 的波特率参数是否为 -s5（达妙自定义表：s5=1M 经典 CAN，\n"
                    "     与标准 slcand 的 s8=1M 映射【不同】，刷的是达妙固件就按达妙表）\n"
                    "  3. 电机是否上了 48V 电源\n", ifname.c_str());
    } else {
        std::printf("期望值（DM-J4310 实测）: MST_ID=0x18 ESC_ID=0x08 TIMEOUT=400 CTRL_MODE=1\n"
                    "                        NPP=14 Gr=10 PMAX=12.5 VMAX=50 TMAX=10 can_br=4\n"
                    "全部一致 ⇒ 通路完好，可以跑 dm_batch_sc 验证逐帧投递。\n");
    }

    ::close(s);
    return got > 0 ? 0 : 2;
}
