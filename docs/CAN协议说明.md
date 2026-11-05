# CAN 通信协议说明

> 裸 CAN（非 CANopen），标准帧 11 位 ID，数据帧，DLC = 4。
> 两个方向各一个 ID：`0x181` 下发目标位置，`0x201` 回传当前位置。

---

## 一、物理层与位时序

| 参数 | 值 | 说明 |
|---|---|---|
| 波特率 | **500 Kbps** | 工业现场常用档位 |
| 帧格式 | 标准帧（11 位 ID） | `IDE = CAN_ID_STD` |
| 帧类型 | 数据帧 | `RTR = CAN_RTR_DATA` |
| 数据长度 | 4 字节 | `DLC = 4` |
| 收发器 | TJA1050（从站侧外接） | 主站用板载 |
| 引脚 | PA11 = CAN_RX，PA12 = CAN_TX | 两节点一致 |

**位时序参数**（F103 从站，APB1 = 36 MHz）：

```
Prescaler = 6，BS1 = 7 TQ，BS2 = 4 TQ，SJW = 1 TQ
tq    = 6 / 36 MHz = 166.7 ns
1 bit = (1 + 7 + 4) × 166.7 ns = 2 μs  →  500 Kbps
采样点 = (1 + 7) / 12 ≈ 66.7%
```

> 采样点 66.7% 对桌面短总线足够；若拉长总线或提高波特率，可调整为 BS1 = 13 / BS2 = 2（采样点约 87.5%）以留出更多传播段余量。这属于可选优化，当前配置未做此项调整。

**总线接线要点**：CAN_H ↔ CAN_H、CAN_L ↔ CAN_L，两节点共地；总线两端（首尾两个节点）各接一只 120 Ω 终端电阻，中间节点不接。

---

## 二、帧定义

数据统一为 **int32 大端**，单位 = 编码器计数（65536 计数/圈）。

### 0x181 目标位置（主站 → 从站）

| 字节 | 内容 |
|---|---|
| Byte0 | 目标位置 bit31~24 |
| Byte1 | 目标位置 bit23~16 |
| Byte2 | 目标位置 bit15~8 |
| Byte3 | 目标位置 bit7~0 |

按协议推导的取值示例：

| 目标 | 十进制 | 十六进制 | 数据字节 |
|---|---|---|---|
| 90° | 16384 | `0x00004000` | `00 00 40 00` |
| 180° | 32768 | `0x00008000` | `00 00 80 00` |

### 0x201 当前位置（从站 → 主站）

| 字节 | 内容 |
|---|---|
| Byte0 | 当前编码器位置 bit31~24 |
| Byte1 | 当前编码器位置 bit23~16 |
| Byte2 | 当前编码器位置 bit15~8 |
| Byte3 | 当前编码器位置 bit7~0 |

从站打包代码：

```c
encoder_data[0] = (encoder_pos >> 24) & 0xFF;
encoder_data[1] = (encoder_pos >> 16) & 0xFF;
encoder_data[2] = (encoder_pos >> 8)  & 0xFF;
encoder_data[3] =  encoder_pos        & 0xFF;
```

> **为什么用大端**：字节顺序与"高位在前"的书写习惯一致，抓包时肉眼可直接读出数值；交换字节序的代价是每次收发都多两次移位，但没有跨平台移植需求，取可读性。

---

## 三、从站接收滤波器

从站只接收 `0x181`，用 32 位掩码模式配置 bank 0：

```c
sFilterConfig.FilterMode       = CAN_FILTERMODE_IDMASK;
sFilterConfig.FilterScale      = CAN_FILTERSCALE_32BIT;
sFilterConfig.FilterIdHigh     = 0x181 << 5;   // 11 位 ID 左移 5 位对齐
sFilterConfig.FilterIdLow      = 0x0000;
sFilterConfig.FilterMaskIdHigh = 0xFFFF;       // 高 16 位全比较
sFilterConfig.FilterMaskIdLow  = 0x0000;       // 低 16 位不比较
sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
```

**为什么是 `<< 5`**：32 位过滤器寄存器里，标准帧 ID（11 位）并不从 bit0 开始，而是放在 **bit31~bit21**（高 16 位内的 bit15~bit5 位置）。所以 `0x181` 必须左移 5 位才能对齐到 ID 域。

**为什么掩码是 `0xFFFF / 0x0000`**：只比较高 16 位（含 ID 域），低 16 位（扩展 ID / IDE / RTR / 数据）不参与匹配 → 等价于"ID 完全等于 0x181 才收"。

中断里只做解析和置标志，耗时动作全部留给主循环：

```c
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
    target_pos = (RxData[0] << 24) | (RxData[1] << 16) | (RxData[2] << 8) | RxData[3];
    new_target_flag = 1;      // 只置标志，不在这里读编码器/发串口
}
```

---

## 四、收发时序

```
时间 →                                              （单位：ms）

主站  0x181 ──┐                            ┌──── 0x181 ────（3 s 后再发下一目标）
              │                            │
从站          ▼                            ▼
        收帧→置标志                  收帧→置标志
        读编码器(≈10~20 ms)               读编码器
        算差值→发 0xFD                    算差值→发 0xFD
        等 500 ms 电机转完                等 500 ms
              │                            │
        回传 0x201（每 50 ms 一次）   回传 0x201 …
```

主站节拍：**每 3 s 切换一次目标**，切换时只发一帧（不连发）。
从站节拍：**收到目标立即定位**，定位完成后恢复 **50 ms** 周期的位置回传。

> 已知副作用：定位那 500 ms 里主循环被阻塞，位置回传暂停 —— 见 `系统设计说明.md` 第七节。

---

## 五、ID 规划与多节点扩展

当前使用 CANopen **预定义连接集**的默认 ID（NodeID = 1）：

| 方向 | CANopen 预定义 ID | 本项目实际使用 | 含义 |
|---|---|---|---|
| 主 → 从 | `0x180 + NodeID` = 0x181 | **0x181** | 目标位置（对应 TPDO1） |
| 从 → 主 | `0x200 + NodeID` = 0x201 | **0x201** | 当前位置（对应 RPDO1） |

**这不是巧合**：把裸 CAN 的 ID 直接选在预定义连接集上，后续接入 CANopenNode 时 ID 不用重排，节点行为对得上。多节点扩展只需按 `0x180+NodeID` / `0x200+NodeID` 分配：

| 从站 | 命令 ID | 反馈 ID |
|---|---|---|
| NodeID = 1 | 0x181 | 0x201 |
| NodeID = 2 | 0x182 | 0x202 |
| NodeID = n | 0x180+n | 0x200+n |

---

## 六、CANopen 侧已完成的配置（协议栈未启用）

F407 主站工程里已移植 CANopenNode v4.0，编译通过但主循环仍走裸 CAN。对象字典按**主节点视角**配置（这一点很容易搞反，见 `踩坑记录.md`）：

| 索引 | 对象 | 属性 | 映射 | COB-ID |
|---|---|---|---|---|
| `0x2000` | 目标转速 | `ODA_SDO_RW \| ODA_TPDO \| ODA_MB` | TPDO `0x1A00` ← `0x20000120` | `0x40000181` |
| `0x2001` | 实际转速 | `ODA_SDO_R \| ODA_RPDO \| ODA_MB` | RPDO `0x1600` ← `0x20010120` | `0x00000201` |

相关参数含义：

- **PDO 映射编码 `0x20010120`** = index `0x2001` + sub-index `0x01` + 位长 `0x20`（32 位），三段拼成一个 32 位数。
- **COB-ID 的 bit31 = 启用位**：`0x40000181` 表示"启用 + 不允许 RTR"，`0x00000201` 表示"启用"。若写成 `0x8…` / `0xC…` 开头则是停用状态。
- **`transmissionType = 0xFE`**：事件驱动（异步），不是周期发送。

---

## 七、待办

- [ ] 双节点 CANopen 联调：F103 从节点目前只有 LED 闪烁模板，需补电机控制 + 从节点对象字典
- [ ] 打开 `AutoRetransmission` / `AutoBusOff`，提升总线容错
- [ ] 增加第二从节点，验证多 ID 并行调度

---

*相关文档：[系统设计说明.md](系统设计说明.md) · [踩坑记录.md](踩坑记录.md) · [测试记录.md](测试记录.md)*
