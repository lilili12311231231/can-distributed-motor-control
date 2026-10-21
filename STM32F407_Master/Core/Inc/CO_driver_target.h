/*
 * Device and application specific definitions for CANopenNode.
 *
 * @file        CO_driver_target.h
 * @author      Janez Paternoster
 * @copyright   2021 Janez Paternoster
 *
 * This file is part of <https://github.com/CANopenNode/CANopenNode>, a CANopen Stack.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this
 * file except in compliance with the License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software distributed under the License is
 * distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and limitations under the License.
 */

#ifndef CO_DRIVER_TARGET_H
#define CO_DRIVER_TARGET_H

/* This file contains device and application specific definitions. It is included from CO_driver.h, which contains
 * documentation for common definitions below. */

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef CO_DRIVER_CUSTOM
#include "CO_driver_custom.h"
#endif

/* Stack configuration override default values. For more information see file CO_config.h. */

/* Basic definitions. If big endian, CO_SWAP_xx macros must swap bytes. */
#define CO_LITTLE_ENDIAN 1 /* 小端字节序 */
#define CO_SWAP_16(x) x
#define CO_SWAP_32(x) x
#define CO_SWAP_64(x) x
/* NULL is defined in stddef.h */
/* true and false are defined in stdbool.h */
/* int8_t to uint64_t are defined in stdint.h */
typedef uint_fast8_t bool_t; /* 布尔类型 */
typedef float float32_t; /* 32位浮点数 */
typedef double float64_t; /* 64位浮点数 */

/* 关闭需要额外文件的模块 */
#define CO_CONFIG_STORAGE 0 /* 关闭存储模块 */
#define CO_CONFIG_LEDS 0 /* 关闭LED模块 */
#define CO_CONFIG_LSS 0 /* 关闭LSS模块 */

/*定义CAN接收帧结构体CO_CANrxMsg_t*/
/* 11位标准ID  */

typedef struct {
    uint16_t ident; /* 11位标准ID  */
    uint8_t DLC; /* 帧的数据长度 */
    uint8_t data[8]; /* 帧的数据 */
} CO_CANrxMsg_t;

/* Access to received CAN frame */
#define CO_CANrxMsg_readIdent(msg) ((uint16_t)(((CO_CANrxMsg_t*)msg)->ident)) /*取出帧的 ID  */
#define CO_CANrxMsg_readDLC(msg)   ((uint8_t)(((CO_CANrxMsg_t*)msg)->DLC)) /*取出帧的数据长度  */
#define CO_CANrxMsg_readData(msg)  ((const uint8_t*)(((CO_CANrxMsg_t*)msg)->data)) /*取出帧数组的指针地址  */

/* Received frame object */
typedef struct {
    uint16_t ident; /* 11位标准ID  */
    uint16_t mask; /* 接收过滤器掩码 */
    void* object; /* 对象指针 */
    void (*CANrx_callback)(void* object, void* message); /* 接收回调函数指针 */
} CO_CANrx_t; /* 接收帧对象 */

/* Transmit frame object */
typedef struct {
    uint32_t ident; /* 29位扩展ID  */
    uint8_t DLC; /* 帧的数据长度 */
    uint8_t data[8]; /* 帧的数据 */
    volatile bool_t bufferFull; /* 发送缓冲区是否已满 */
    volatile bool_t syncFlag; /* 发送同步标志位 */
} CO_CANtx_t;

/* CAN module object */
typedef struct {
    void* CANptr; /* CAN句柄 */
    CO_CANrx_t* rxArray; /* 接收帧数组 */
    uint16_t rxSize; /* 接收帧数组的大小 */
    CO_CANtx_t* txArray; /* 发送帧数组 */
    uint16_t txSize; /* 发送帧数组的大小 */
    uint16_t CANerrorStatus; /* CAN错误状态 */
    volatile bool_t CANnormal; /* CAN是否正常 */
    volatile bool_t useCANrxFilters; /* 是否使用接收过滤器 */
    volatile bool_t bufferInhibitFlag; /* 发送缓冲区是否已禁用 */
    volatile bool_t firstCANtxMessage; /* 是否是第一个发送的CAN消息 */
    volatile uint16_t CANtxCount; /* 发送的CAN消息计数 */
    uint32_t errOld; /* 旧的错误码 */
} CO_CANmodule_t;

extern void CO_CANinterrupt(CO_CANmodule_t* CANmodule); /* CAN中断处理函数 */
/* Data storage object for one entry */
typedef struct {
    void* addr; /* 数据存储地址 */
    size_t len; /* 数据存储长度 */
    uint8_t subIndexOD; /* 子索引，用于访问Object Dictionary */
    uint8_t attr; /* 数据存储属性 */
    /* Additional variables (target specific) */
    void* addrNV; /* 非易失性存储地址 */
} CO_storage_entry_t;

/* (un)lock critical section in CO_CANsend() */
#define CO_LOCK_CAN_SEND(CAN_MODULE) /* 锁定CAN发送临界区 */
#define CO_UNLOCK_CAN_SEND(CAN_MODULE) /* 解锁CAN发送临界区 */

/* (un)lock critical section in CO_errorReport() or CO_errorReset() */
#define CO_LOCK_EMCY(CAN_MODULE) /* 锁定紧急错误处理临界区 */
#define CO_UNLOCK_EMCY(CAN_MODULE) /* 解锁紧急错误处理临界区 */

/* (un)lock critical section when accessing Object Dictionary */
#define CO_LOCK_OD(CAN_MODULE) /* 锁定Object Dictionary临界区 */
#define CO_UNLOCK_OD(CAN_MODULE) /* 解锁Object Dictionary临界区 */

/* Synchronization between CAN receive and data processing threads. */
#define CO_MemoryBarrier() /* 内存屏障 */
#define CO_FLAG_READ(rxNew) ((rxNew) != NULL) /* 读取接收新数据标志位 */
#define CO_FLAG_SET(rxNew)   /* 设置接收新数据标志位 */                                                                                           \
    {                                                                                                                  \
        CO_MemoryBarrier();                                                                                            \
        rxNew = (void*)1L; /* 设置接收新数据标志位为已接收 */                                                                                             \
    }
#define CO_FLAG_CLEAR(rxNew) /* 清除接收新数据标志位 */                                                                                           \
    {                                                                                                                  \
        CO_MemoryBarrier();                                                                                            \
        rxNew = NULL; /* 清除接收新数据标志位 */                                                                                                  \
    }

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* CO_DRIVER_TARGET_H */
