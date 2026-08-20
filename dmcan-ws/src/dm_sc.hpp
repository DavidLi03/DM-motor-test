// dm_sc.hpp —— DM 电机 SocketCAN 公共层（slcan 固件路线）
//
// 与厂商 SDK 路线 (dmcan.h + libdm_device.so) 的对应关系：
//   dmcan_device_send_can()      -> dmsc::send_frame()   （write 到 CAN_RAW socket）
//   dmcan_device_hook_recv_...   -> 自建 RX 线程 + dmsc::recv_frame()
//   模块硬件时间戳 hw_ts          -> 内核收包时间戳 kts（SO_TIMESTAMPNS，主机时钟域，
//                                    无 3.71% 尺度误差、无秒:纳秒拆分问题）
//
// 前提：slcan 固件已刷入，slcand 已把 /dev/ttyACM* 桥接成 CAN 接口（见 ../SOCKETCAN.md）。
// 电机在 1Mbps ⇒ 经典 CAN 2.0B，can_frame（非 canfd_frame）即可。

#pragma once

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <string>

namespace dmsc {

// ---- 本机实测参数（2026-08-12 换机，值来自 2026-08-02 对同一台电机的 0x33 实测读回）----
// 当前在台电机：DM-J4310-2EC V1.2 (48V)（Gr=10，NPP=14，CTRL_MODE=MIT）。
// 历史在台电机（换回时改这里并重编译）：
//   DM10422      : ESC=0x01 MST=0x11 PMAX=12.566 VMAX=20 TMAX=500 Gr=22（NPP=16, Kt≈3.18）
//   DM-J4340P-2EC: ESC=0x05 MST=0x15 PMAX=12.5  VMAX=20 TMAX=28 Gr=40
constexpr uint16_t kEscId   = 0x008;   // 0x08 ESC_ID  电机接收 ID
constexpr uint16_t kMstId   = 0x018;   // 0x07 MST_ID  电机反馈 ID（0x33 读参数应答也走这个 ID）
constexpr uint16_t kQueryId = 0x7FF;   // 参数读写专用 ID
constexpr uint16_t kMitOff  = 0x000;   // CTRL_MODE=1(MIT) ⇒ 控制帧发往 ESC_ID+0
constexpr float kPMax = 12.5f;
constexpr float kVMax = 50.0f;
constexpr float kTMax = 10.0f;
constexpr float kGr   = 10.0f;

inline uint16_t float_to_uint(float x, float xmin, float xmax, uint8_t bits) {
    float norm = (x - xmin) / (xmax - xmin);
    return (uint16_t)(norm * ((1 << bits) - 1));
}

inline float uint_to_float(uint16_t x, float xmin, float xmax, uint8_t bits) {
    float norm = (float)x / ((1 << bits) - 1);
    return norm * (xmax - xmin) + xmin;
}

// 指令值经量化后电机实际收到的值，用于如实打印
inline float quantized(float x, float xmin, float xmax, uint8_t bits) {
    return uint_to_float(float_to_uint(x, xmin, xmax, bits), xmin, xmax, bits);
}

inline const char* err_name(uint8_t e) {
    switch (e) {
        case 0x0: return "失能";
        case 0x1: return "使能";
        case 0x8: return "超压";
        case 0x9: return "欠压";
        case 0xA: return "过电流";
        case 0xB: return "MOS过温";
        case 0xC: return "线圈过温";
        case 0xD: return "通讯丢失";
        case 0xE: return "过载";
        default:  return "未知";
    }
}

// 打开并绑定 SocketCAN 接口。
//   only_mst=true  只收 MST_ID（控制类程序用，避免收到别的节点）
//   出错时把原因写进 *err 并返回 -1。错误帧不受 CAN_RAW_FILTER 影响，单独订阅。
inline int open_can(const char* ifname, bool only_mst, std::string* err) {
    int s = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        if (err) *err = std::string("socket(PF_CAN) 失败: ") + std::strerror(errno);
        return -1;
    }

    ifreq ifr{};
    std::strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    if (::ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        if (err) *err = std::string("接口 ") + ifname + " 不存在: " + std::strerror(errno) +
                        "\n  是否已按 SOCKETCAN.md 用 slcand 建好接口并 ip link set up？";
        ::close(s);
        return -1;
    }

    if (only_mst) {
        can_filter flt{};
        flt.can_id   = kMstId;
        flt.can_mask = CAN_SFF_MASK;
        ::setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &flt, sizeof(flt));
    }
    can_err_mask_t em = CAN_ERR_MASK;   // 订阅总线错误帧（CAN_ERR_FLAG 置位送达）
    ::setsockopt(s, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &em, sizeof(em));

    int on = 1;                          // 内核收包时间戳，recvmsg 的 cmsg 里取
    ::setsockopt(s, SOL_SOCKET, SO_TIMESTAMPNS, &on, sizeof(on));

    sockaddr_can addr{};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(s, (sockaddr*)&addr, sizeof(addr)) < 0) {
        if (err) *err = std::string("bind 失败: ") + std::strerror(errno);
        ::close(s);
        return -1;
    }
    return s;
}

// 收一帧，带内核时间戳(ns, CLOCK_REALTIME 域)。返回 1=有帧 0=超时 <0=出错。
// poll 超时使 RX 线程能周期性检查退出标志，不会卡死在 read 上。
inline int recv_frame(int s, can_frame* fr, uint64_t* kts_ns, int timeout_ms) {
    pollfd pfd{s, POLLIN, 0};
    int pr = ::poll(&pfd, 1, timeout_ms);
    if (pr <= 0) return pr;

    iovec iov{fr, sizeof(*fr)};
    alignas(cmsghdr) char cbuf[CMSG_SPACE(sizeof(timespec))];
    msghdr msg{};
    msg.msg_iov        = &iov;
    msg.msg_iovlen     = 1;
    msg.msg_control    = cbuf;
    msg.msg_controllen = sizeof(cbuf);

    ssize_t n = ::recvmsg(s, &msg, 0);
    if (n < (ssize_t)sizeof(can_frame)) return -1;

    *kts_ns = 0;
    for (cmsghdr* c = CMSG_FIRSTHDR(&msg); c; c = CMSG_NXTHDR(&msg, c)) {
        if (c->cmsg_level == SOL_SOCKET && c->cmsg_type == SCM_TIMESTAMPNS) {
            timespec ts;
            std::memcpy(&ts, CMSG_DATA(c), sizeof(ts));
            *kts_ns = (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
        }
    }
    return 1;
}

inline bool send_frame(int s, uint16_t id, const uint8_t* d, uint8_t len) {
    can_frame fr{};
    fr.can_id  = id;
    fr.can_dlc = len;
    std::memcpy(fr.data, d, len);
    return ::write(s, &fr, sizeof(fr)) == (ssize_t)sizeof(fr);
}

// 特殊命令帧：FF FF FF FF FF FF FF <cmd>，发往 ESC_ID+模式偏移
// 0xFB=清错  0xFC=使能  0xFD=失能（0xFE 存零点 —— 本工程绝不发送）
inline bool send_cmd(int s, uint8_t cmd) {
    uint8_t d[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd};
    return send_frame(s, kEscId + kMitOff, d, 8);
}

// 0x33 读参数（对电机无副作用）。应答从 kMstId 回来，payload[2]==0x33。
inline bool send_read_reg(int s, uint8_t rid) {
    uint8_t d[4] = {(uint8_t)(kEscId & 0xFF), (uint8_t)((kEscId >> 8) & 0xFF), 0x33, rid};
    return send_frame(s, kQueryId, d, 4);
}

// MIT 帧。kp 与 tau 由调用方固定传 0（纯阻尼方案），见各程序文件头的安全说明。
inline bool send_mit(int s, float q, float dq, float kp, float kd, float tau) {
    uint16_t q_u   = float_to_uint(q,   -kPMax, kPMax, 16);
    uint16_t dq_u  = float_to_uint(dq,  -kVMax, kVMax, 12);
    uint16_t kp_u  = float_to_uint(kp,  0.0f,   500.0f, 12);
    uint16_t kd_u  = float_to_uint(kd,  0.0f,   5.0f,   12);
    uint16_t tau_u = float_to_uint(tau, -kTMax, kTMax, 12);

    uint8_t d[8];
    d[0] = (q_u >> 8) & 0xFF;
    d[1] = q_u & 0xFF;
    d[2] = dq_u >> 4;
    d[3] = ((dq_u & 0x0F) << 4) | ((kp_u >> 8) & 0x0F);
    d[4] = kp_u & 0xFF;
    d[5] = kd_u >> 4;
    d[6] = ((kd_u & 0x0F) << 4) | ((tau_u >> 8) & 0x0F);
    d[7] = tau_u & 0xFF;
    return send_frame(s, kEscId + kMitOff, d, 8);
}

}  // namespace dmsc
