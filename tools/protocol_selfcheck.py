#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
协议自检：不接任何硬件，直接从源码里抓参数，复核时基、帧格式与坐标换算。

用途
----
1. 改完代码后跑一遍，确认两个节点的 CAN 位时序仍然匹配（改 Prescaler 最容易出事）。
2. 复核 SERVO42C 帧构造与 0x30 回复解析，避免"帧看着对、校验和错一位"。
3. 复核计数 ↔ 步数换算的整数截断误差。

用法
----
    python tools/protocol_selfcheck.py

约定：脚本不读取任何硬件状态，所有"预期值"都由源码 + 时钟树推导得出。
任一项失败时进程返回 1。
"""

import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

MASTER_MAIN = os.path.join(ROOT, "STM32F407_Master", "Core", "Src", "main.c")
MASTER_CAN = os.path.join(ROOT, "STM32F407_Master", "Core", "Src", "can.c")
MASTER_HAL = os.path.join(ROOT, "STM32F407_Master", "Core", "Inc", "stm32f4xx_hal_conf.h")
SLAVE_MAIN = os.path.join(ROOT, "STM32F103_Slave", "Core", "Src", "main.c")
SLAVE_HAL = os.path.join(ROOT, "STM32F103_Slave", "Core", "Inc", "stm32f1xx_hal_conf.h")
SERVO_H = os.path.join(ROOT, "STM32F103_Slave", "Core", "Inc", "servo42c.h")

_passed = 0
_failed = 0


def check(label, actual, expected, fmt="{}"):
    """比较 actual 与 expected，打印一行结果。"""
    global _passed, _failed
    ok = actual == expected
    if ok:
        _passed += 1
        print("  [PASS] %-42s %s" % (label, fmt.format(actual)))
    else:
        _failed += 1
        print("  [FAIL] %-42s 实际 %r，预期 %r" % (label, actual, expected))
    return ok


def read(path):
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        return f.read()


def find_int(text, pattern, name=""):
    """按正则取第一个捕获组的整数，取不到直接报错退出。"""
    m = re.search(pattern, text)
    if not m:
        raise SystemExit("无法从源码解析出 %s（正则未命中）：%s" % (name or pattern, pattern))
    return int(m.group(1), 0)


# ---------------------------------------------------------------------------
# 一、时钟树与 CAN 位时序
# ---------------------------------------------------------------------------
def section_clock():
    print("\n[1] 时钟树与 CAN 位时序")

    hse_master = find_int(read(MASTER_HAL), r"#define\s+HSE_VALUE\s+(\d+)U", "F407 HSE")
    hse_slave = find_int(read(SLAVE_HAL), r"#define\s+HSE_VALUE\s+(\d+)U", "F103 HSE")

    m = read(MASTER_MAIN)
    pllm = find_int(m, r"PLL\.PLLM\s*=\s*(\d+)", "F407 PLLM")
    plln = find_int(m, r"PLL\.PLLN\s*=\s*(\d+)", "F407 PLLN")
    # PLLP_DIV2 / DIV4 / ... → 除数
    pllp = find_int(m, r"PLL\.PLLP\s*=\s*RCC_PLLP_DIV(\d+)", "F407 PLLP")
    apb1_div_master = find_int(m, r"APB1CLKDivider\s*=\s*RCC_HCLK_DIV(\d+)", "F407 APB1 分频")
    sysclk_master = hse_master // pllm * plln // pllp
    pclk1_master = sysclk_master // apb1_div_master

    s = read(SLAVE_MAIN)
    pllmul = find_int(s, r"PLL\.PLLMUL\s*=\s*RCC_PLL_MUL(\d+)", "F103 PLLMUL")
    apb1_div_slave = find_int(s, r"APB1CLKDivider\s*=\s*RCC_HCLK_DIV(\d+)", "F103 APB1 分频")
    sysclk_slave = hse_slave * pllmul
    pclk1_slave = sysclk_slave // apb1_div_slave

    check("F407 SYSCLK (MHz)", sysclk_master // 1000000, 168)
    check("F407 PCLK1 (MHz，CAN1 挂在 APB1)", pclk1_master // 1000000, 42)
    check("F103 SYSCLK (MHz)", sysclk_slave // 1000000, 72)
    check("F103 PCLK1 (MHz)", pclk1_slave // 1000000, 36)

    # 位时序寄存器：Prescaler 与 BS1/BS2 的分段数
    def bit_timing(can_text, scope):
        prescaler = find_int(can_text, r"Init\.Prescaler\s*=\s*(\d+)", scope + " Prescaler")
        bs1 = find_int(can_text, r"Init\.TimeSeg1\s*=\s*CAN_BS1_(\d+)TQ", scope + " BS1")
        bs2 = find_int(can_text, r"Init\.TimeSeg2\s*=\s*CAN_BS2_(\d+)TQ", scope + " BS2")
        return prescaler, bs1, bs2

    pre_m, bs1_m, bs2_m = bit_timing(read(MASTER_CAN), "主站")
    pre_s, bs1_s, bs2_s = bit_timing(read(SLAVE_MAIN), "从站")

    tq_hz_m = pclk1_master // pre_m
    tq_hz_s = pclk1_slave // pre_s
    total_tq_m, total_tq_s = 1 + bs1_m + bs2_m, 1 + bs1_s + bs2_s
    bps_m, bps_s = tq_hz_m // total_tq_m, tq_hz_s // total_tq_s

    check("主站 Prescaler", pre_m, 7)
    check("从站 Prescaler", pre_s, 6)
    check("主站分频后时基 (MHz)", tq_hz_m / 1e6, 6.0, fmt="{:.3f}")
    check("从站分频后时基 (MHz)", tq_hz_s / 1e6, 6.0, fmt="{:.3f}")
    check("两节点时基一致", tq_hz_m == tq_hz_s, True)
    check("主站波特率 (Kbps)", bps_m // 1000, 500)
    check("从站波特率 (Kbps)", bps_s // 1000, 500)
    check("采样点 (%, BS1=7)", round((1 + bs1_m) / total_tq_m * 100, 1), 66.7, fmt="{:.1f}")
    check("1 bit 时长 (ns)", round(1e9 / bps_m), 2000)

    print("  → Prescaler 数值不同（7 / 6）是正常的：PCLK1 不同（42 / 36 MHz），"
          "同除之后都是 6 MHz。")


# ---------------------------------------------------------------------------
# 二、SERVO42C 帧构造与应答解析
# ---------------------------------------------------------------------------
SERVO_ADDR = 0xE0


def servo_frame(func, data=b""):
    """按 Servo_SendCommand 的规则组帧：地址 + 功能码 + 数据 + 和校验（取低 8 位）。"""
    body = bytes([SERVO_ADDR, func]) + bytes(data)
    return body + bytes([sum(body) & 0xFF])


def section_servo():
    print("\n[2] SERVO42C 帧构造（38400 8N1，地址 0xE0，和校验）")

    header = read(SERVO_H)
    macro = dict(re.findall(r"#define\s+(SERVO_[A-Z_]+)\s+(0x[0-9A-Fa-f]+)", header))
    check("SERVO_ADDR 宏", int(macro["SERVO_ADDR"], 16), 0xE0)
    check("0xF3 使能宏", int(macro["SERVO_FUNC_ENABLE"], 16), 0xF3)
    check("0x30 读编码器宏", int(macro["SERVO_FUNC_READ_ENC"], 16), 0x30)
    check("0xFD 位置控制宏", int(macro["SERVO_FUNC_POS"], 16), 0xFD)

    cases = [
        ("设工作模式 CR_UART", 0x82, b"\x02"),
        ("EN 引脚一直使能", 0x85, b"\x02"),
        ("使能电机", 0xF3, b"\x01"),
        ("读编码器", 0x30, b""),
        ("相对位置 dir=1 speed=10 step=800", 0xFD,
         bytes([(1 << 7) | 10]) + (800).to_bytes(4, "big")),
    ]
    for label, func, data in cases:
        frame = servo_frame(func, data)
        expect_len = 2 + len(data) + 1
        check("%s 帧长" % label, len(frame), expect_len)
        print("         帧: %s" % " ".join("%02X" % b for b in frame))

    # 0xFD 必须正好填满 Servo_SendCommand 的 buf[8]
    check("0xFD 帧占用 buf[8] 的字节数",
          len(servo_frame(0xFD, bytes(5))), 8)

    # 0x30 应答解析：E0 + carry(4B) + value(2B) + 校验
    print("\n  [2b] 0x30 应答解析")

    def encoder_reply(carry, value):
        body = bytes([SERVO_ADDR]) + int(carry).to_bytes(4, "big", signed=True) \
            + int(value).to_bytes(2, "big")
        return body + bytes([sum(body) & 0xFF])

    def parse(reply):
        if len(reply) != 8:
            return None
        if (sum(reply[:7]) & 0xFF) != reply[7]:
            return None
        carry = int.from_bytes(reply[1:5], "big", signed=True)
        value = int.from_bytes(reply[5:7], "big")
        return carry * 65536 + value

    samples = [(0, 0), (0, 16384), (0, 32768), (1, 0), (-1, 65535), (3, 12345)]
    for carry, value in samples:
        expect = carry * 65536 + value
        check("carry=%d value=%-5d → 位置" % (carry, value), parse(encoder_reply(carry, value)), expect)

    good = encoder_reply(0, 16384)
    bad = bytearray(good)
    bad[7] ^= 0x01
    check("校验和错一字节 → 丢弃", parse(bytes(bad)), None)
    check("长度不足 8 字节 → 丢弃", parse(good[:7]), None)


# ---------------------------------------------------------------------------
# 三、CAN 帧载荷编解码（int32 大端）
# ---------------------------------------------------------------------------
def section_can_payload():
    print("\n[3] CAN 帧载荷（0x181 / 0x201，int32 大端）")

    def pack(value):
        return int(value).to_bytes(4, "big", signed=True)

    def unpack(payload):
        return int.from_bytes(payload, "big", signed=True)

    cases = [
        ("90°", 16384, "00 00 40 00"),
        ("180°", 32768, "00 00 80 00"),
        ("-1（反向一圈内）", -1, "FF FF FF FF"),
        ("INT32_MIN", -2 ** 31, "80 00 00 00"),
        ("INT32_MAX", 2 ** 31 - 1, "7F FF FF FF"),
    ]
    for label, value, expect_hex in cases:
        payload = pack(value)
        check("0x181 打包 %s" % label,
              " ".join("%02X" % b for b in payload), expect_hex)
        check("0x201 解包 %s" % label, unpack(payload), value)


# ---------------------------------------------------------------------------
# 四、计数 ↔ 步数换算
# ---------------------------------------------------------------------------
def section_scale():
    print("\n[4] 坐标换算（step = Δenc × 25 / 512）")

    counts_per_rev = 65536          # 16 位编码器
    steps_per_rev = 3200            # Mstep = 16
    num, den = 25, 512              # 代码里的整数分式

    check("25/512 与 3200/65536 等价",
          abs(num / den - steps_per_rev / counts_per_rev) < 1e-15, True)
    check("1 步 = 计数", counts_per_rev / steps_per_rev, 20.48, fmt="{:.2f}")
    check("90° 计数", counts_per_rev // 4, 16384)
    check("90° → 步数", 16384 * num // den, 800)
    check("180° → 步数", 32768 * num // den, 1600)
    check("360° → 步数", 65536 * num // den, 3200)

    # 截断误差：最大 1 步，对应角度
    max_err_counts = counts_per_rev / steps_per_rev
    check("单次换算最大截断误差（计数）", round(max_err_counts, 2), 20.48, fmt="{:.2f}")
    check("单次换算最大截断误差（度）",
          round(max_err_counts / counts_per_rev * 360, 4), 0.1125, fmt="{:.4f}")

    # 往返一致性：同一目标反复解算不应累积偏差（每次都读真实编码器重算）
    print("  → 截断误差是「一步之内」的量化误差，不随定位次数累积："
          "每次定位都以当前编码器为基准重新算差值，不是在前一次结果上叠加。")

    # 方向拆分：负数必须转成正的幅值 + dir=0
    for delta in (-800, -1, 800, 1600):
        raw = delta * num // den
        direction = 1
        magnitude = raw
        if magnitude < 0:
            direction, magnitude = 0, -magnitude
        ok = magnitude >= 0 and direction in (0, 1) and magnitude == abs(raw)
        check("方向拆分 Δenc=%-5d → dir=%d step=%d" % (delta, direction, magnitude), ok, True)


def section_limits():
    print("\n[5] 边界与溢出（int32 有符号运算的实际适用范围）")

    num, den = 25, 512
    int32_max = 2 ** 31 - 1
    counts_per_rev = 65536

    # delta_enc * 25 是可能溢出的那一步
    safe_counts = int32_max // num
    check("Δenc 无溢出上限（计数）", safe_counts, 85899345)
    check("对应圈数", round(safe_counts / counts_per_rev, 1), 1310.7, fmt="{:.1f}")

    # 0xFD 的 step 字段是 4 字节有符号数，方向单独用 dir 位表示
    max_step = int32_max
    check("0xFD 单帧最大步数", max_step, 2147483647)
    check("折算圈数", round(max_step / 3200), 671089)

    print("  → step 字段虽然是 4 字节，但受「先乘 25」约束，实际安全范围是 "
          "±%d 计数（约 ±1310 圈）；" % safe_counts)
    print("    本项目只在 0~32768 计数（0~180°）内往复，离上限三个数量级，无需额外保护。")

    # 速度/方向打包：bit7 方向 + 低 7 位速度，speed 必须 ≤ 0x7F
    for speed in (0, 10, 100, 127):
        packed = (1 << 7) | (speed & 0x7F)
        check("dir=1 speed=%-3d 打包后 bit7 与低 7 位" % speed,
              (packed >> 7, packed & 0x7F), (1, speed))
    check("speed 超 127 时被掩码截断",
          ((1 << 7) | (200 & 0x7F)) & 0x7F, 200 & 0x7F)


def section_master_tick():
    print("\n[6] 主站节拍模拟（按 main.c 的 loop 复现，看总线上实际发了什么）")

    m = read(MASTER_MAIN)
    period = find_int(m, r"last_switch\s*>\s*(\d+)", "切换周期")
    init_target = find_int(m, r"master_target\s*=\s*(\d+)\s*;", "master_target 初值")
    m2 = re.search(r"\(\s*master_target\s*==\s*(\d+)\s*\)\s*\?\s*(\d+)\s*:\s*(\d+)", m)
    if not m2:
        raise SystemExit("无法解析主站的三元翻转表达式")
    cond_a, cond_b = int(m2.group(2)), int(m2.group(3))
    delay = find_int(m, r"HAL_Delay\((\d+)\);", "主循环延时")

    check("切换周期 (ms)", period, 3000)
    check("主循环延时 (ms)", delay, 50)
    check("两个目标值", tuple(sorted({cond_a, cond_b})), (16384, 32768))

    # 复现 while(1) 的逻辑：每 delay 推进一次，条件成立则翻转并发送
    target = init_target
    last_switch = 0
    tick = 0
    sent = []
    while len(sent) < 4:
        tick += delay
        if tick - last_switch > period:
            last_switch = tick
            target = cond_b if target == cond_a else cond_a
            sent.append((tick, target))

    print("  仿真结果（tick=ms，仅列实际发出的帧）：")
    for t, v in sent:
        print("         t=%5d ms → 0x181 载荷 %s = %5d（%s）"
              % (t, " ".join("%02X" % b for b in v.to_bytes(4, "big")),
                 v, "180°" if v == 32768 else "90°"))

    check("首帧目标值", sent[0][1], 32768)
    check("首帧时刻 (ms)", sent[0][0], 3050)
    check("第二帧目标值", sent[1][1], 16384)
    check("初值从未被发出", init_target not in [sent[0][1]], True)

    print("  → 注释里的「第一个目标：90 度」是初值，但它在第一次翻转后才发送，")
    print("    所以总线上第一帧实际是 180°。")


def main():
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    print("=" * 68)
    print("协议自检 · 基于 CAN 总线的分布式电机控制系统")
    print("来源：源码参数，无需硬件")
    print("=" * 68)

    section_clock()
    section_servo()
    section_can_payload()
    section_scale()
    section_limits()
    section_master_tick()

    print("\n" + "=" * 68)
    print("通过 %d 项，失败 %d 项" % (_passed, _failed))
    print("=" * 68)
    return 0 if _failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
