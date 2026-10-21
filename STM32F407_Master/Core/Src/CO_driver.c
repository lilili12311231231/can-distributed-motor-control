/*
 * CAN module object for generic microcontroller.
 *
 * This file is a template for other microcontrollers.
 *
 * @file        CO_driver.c
 * @ingroup     CO_driver
 * @author      Janez Paternoster
 * @copyright   2004 - 2020 Janez Paternoster
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

#include "CO_driver.h"
#include "stm32f4xx_hal.h"

extern CAN_HandleTypeDef hcan1; 

void
CO_CANsetConfigurationMode(void* CANptr) {
    HAL_CAN_Stop(&hcan1); /* 停止CAN模块 */
}
void
CO_CANsetNormalMode(CO_CANmodule_t* CANmodule) {
    HAL_CAN_Start(&hcan1); /* 启动CAN模块 */
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); /* 使能接收FIFO0消息待中断请求 */
    CANmodule->CANnormal = true; /* 设置CAN模块为正常模式 */
}

CO_ReturnError_t
CO_CANmodule_init(CO_CANmodule_t* CANmodule, void* CANptr, CO_CANrx_t rxArray[], uint16_t rxSize, CO_CANtx_t txArray[],
                  uint16_t txSize, uint16_t CANbitRate) {
    uint16_t i;

    /* verify arguments */
    if (CANmodule == NULL || rxArray == NULL || txArray == NULL) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    /* Configure object variables */
    CANmodule->CANptr = CANptr; /* CAN模块指针 */
    CANmodule->rxArray = rxArray; /* 接收帧对象数组 */
    CANmodule->rxSize = rxSize; /* 接收帧对象数组大小 */
    CANmodule->txArray = txArray; /* 发送帧对象数组 */
    CANmodule->txSize = txSize; /* 发送帧对象数组大小 */
    CANmodule->CANerrorStatus = 0; /* CAN错误状态 */
    CANmodule->CANnormal = false; /* 是否为正常模式 */
    CANmodule->useCANrxFilters = false; /* 是否使用接收过滤器 */
    CANmodule->bufferInhibitFlag = false; /* 是否禁用发送缓冲区 */
    CANmodule->firstCANtxMessage = true; /* 是否为第一个发送消息 */
    CANmodule->CANtxCount = 0U; /* 发送消息计数器 */
    CANmodule->errOld = 0U; /* 上一个错误状态 */

    for (i = 0U; i < rxSize; i++) {
        rxArray[i].ident = 0U; /* 接收帧标识符 */
        rxArray[i].mask = 0xFFFFU; /* 接收帧掩码 */
        rxArray[i].object = NULL; /* 接收帧对象 */
        rxArray[i].CANrx_callback = NULL; /* 接收帧回调函数 */
    }
    for (i = 0U; i < txSize; i++) {
        txArray[i].bufferFull = false; /* 发送帧缓冲区是否已满 */
    }

    /* Configure CAN module registers */

    /* Configure CAN timing */

    /* Configure CAN module hardware filters */
    if (CANmodule->useCANrxFilters) {
        /* CAN module filters are used, they will be configured with */
        /* CO_CANrxBufferInit() functions, called by separate CANopen */
        /* init functions. */
        /* Configure all masks so, that received CAN frame must match filter */
    } else {
        CAN_FilterTypeDef filterConfig;

        filterConfig.FilterBank = 0; /* 接收过滤器银行 */
        filterConfig.FilterMode = CAN_FILTERMODE_IDMASK; /* 接收过滤器模式 */
        filterConfig.FilterScale = CAN_FILTERSCALE_32BIT; /* 接收过滤器规模 */
        filterConfig.FilterIdHigh = 0; /* 接收过滤器高16位标识符 */
        filterConfig.FilterIdLow = 0; /* 接收过滤器低16位标识符 */
        filterConfig.FilterMaskIdHigh = 0; /* 接收过滤器掩码高16位标识符 */
        filterConfig.FilterMaskIdLow = 0; /* 接收过滤器掩码低16位标识符 */
        filterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0; /* 接收过滤器FIFO赋值 */
        filterConfig.FilterActivation = ENABLE; /* 接收过滤器使能 */
        filterConfig.SlaveStartFilterBank = 14; /* 从接收过滤器银行14开始 */

        HAL_CAN_ConfigFilter(&hcan1, &filterConfig); /* 配置接收过滤器 */

    }

    /* configure CAN interrupt registers */

    return CO_ERROR_NO; /* 返回成功 */
}

void
CO_CANmodule_disable(CO_CANmodule_t* CANmodule) {
    if (CANmodule != NULL) {
        HAL_CAN_Stop(&hcan1); /* 停止CAN模块 */
    }
}

CO_ReturnError_t
CO_CANrxBufferInit(CO_CANmodule_t* CANmodule, uint16_t index, uint16_t ident, uint16_t mask, bool_t rtr, void* object,
                   void (*CANrx_callback)(void* object, void* message)) {
    CO_ReturnError_t ret = CO_ERROR_NO; /* 返回成功 */

    if ((CANmodule != NULL) && (object != NULL) && (CANrx_callback != NULL) && (index < CANmodule->rxSize)) {
        /* buffer, which will be configured */
        CO_CANrx_t* buffer = &CANmodule->rxArray[index]; /* 接收帧对象指针 */

        /* Configure object variables */
        buffer->object = object; /* 接收帧对象 */
        buffer->CANrx_callback = CANrx_callback; /* 接收帧回调函数 */

        /* CAN identifier and CAN mask, bit aligned with CAN module. Different on different microcontrollers. */
        buffer->ident = ident & 0x07FFU; /* 接收帧标识符 */
        if (rtr) {
            buffer->ident |= 0x0800U; /* 设置RTR位 */
        }
        buffer->mask = (mask & 0x07FFU) | 0x0800U; /* 接收帧掩码 */

        /* Set CAN hardware module filter and mask. */
        if (CANmodule->useCANrxFilters) {}
    } else {
        ret = CO_ERROR_ILLEGAL_ARGUMENT; /* 返回错误：无效参数 */
    }

    return ret;
}

CO_CANtx_t*
CO_CANtxBufferInit(CO_CANmodule_t* CANmodule, uint16_t index, uint16_t ident, bool_t rtr, uint8_t noOfBytes,
                   bool_t syncFlag) {
    CO_CANtx_t* buffer = NULL;

    if ((CANmodule != NULL) && (index < CANmodule->txSize)) {
        /* get specific buffer */
        buffer = &CANmodule->txArray[index];

        /* CAN identifier, DLC and rtr, bit aligned with CAN module transmit buffer, microcontroller specific. */
        buffer->ident = ((uint32_t)ident & 0x07FFU) | ((uint32_t)(((uint32_t)noOfBytes & 0xFU) << 11U))
                        | ((uint32_t)(rtr ? 0x8000U : 0U));

        buffer->bufferFull = false;
        buffer->syncFlag = syncFlag;
    }

    return buffer;
}

CO_ReturnError_t
CO_CANsend(CO_CANmodule_t* CANmodule, CO_CANtx_t* buffer) {
    CO_ReturnError_t err = CO_ERROR_NO;

    /* Verify overflow */
    if (buffer->bufferFull) {
        if (!CANmodule->firstCANtxMessage) {
            /* don't set error, if bootup message is still on buffers */
            CANmodule->CANerrorStatus |= CO_CAN_ERRTX_OVERFLOW;
        }
        err = CO_ERROR_TX_OVERFLOW;
    }

    CO_LOCK_CAN_SEND(CANmodule);
    /* 是同步TPDO或非同步TPDO */
    if (CANmodule->CANtxCount == 0) {
        CANmodule->bufferInhibitFlag = buffer->syncFlag; /* 发送同步TPDO */
        CAN_TxHeaderTypeDef txHeader; /* 发送帧头 */
        uint32_t txMailbox; /* 发送邮箱 */

        txHeader.StdId = buffer->ident & 0x07FFU; /* 发送帧标识符 */
        txHeader.ExtId = 0; /* 发送帧扩展标识符 */
        txHeader.IDE = CAN_ID_STD; /* 发送帧标识符类型 */
        txHeader.RTR = CAN_RTR_DATA; /* 发送帧类型 */
        txHeader.DLC = (buffer->ident >> 11) & 0xFU; /* 发送帧数据长度 */
        txHeader.TransmitGlobalTime = DISABLE; /* 禁用全局时间 */

        HAL_CAN_AddTxMessage(&hcan1, &txHeader, buffer->data, &txMailbox); /* 添加送帧到发送邮箱 */
        
    }
    /* if no buffer is free, CAN frame will be sent by interrupt */
    else {
        /* Only increment count if buffer wasn't already full */
        if (!buffer->bufferFull) {
            buffer->bufferFull = true;
            CANmodule->CANtxCount++;
        }
    }
    CO_UNLOCK_CAN_SEND(CANmodule);

    return err;
}

void
CO_CANclearPendingSyncPDOs(CO_CANmodule_t* CANmodule) 
{
    uint32_t tpdoDeleted = 0U;

    CO_LOCK_CAN_SEND(CANmodule);
    /* Abort frame from CAN module, if there is synchronous TPDO.
     * Take special care with this functionality. */
    if (/* CAN frameIsOnCanBuffer && */ CANmodule->bufferInhibitFlag) {
        /* clear TXREQ */
        CANmodule->bufferInhibitFlag = false;
        tpdoDeleted = 1U;
    }
    /* delete also pending synchronous TPDOs in TX buffers */
    if (CANmodule->CANtxCount != 0U) {
        uint16_t i;
        CO_CANtx_t* buffer = &CANmodule->txArray[0];
        for (i = CANmodule->txSize; i > 0U; i--) {
            if (buffer->bufferFull) {
                if (buffer->syncFlag) {
                    buffer->bufferFull = false;
                    CANmodule->CANtxCount--;
                    tpdoDeleted = 2U;
                }
            }
            buffer++;
        }
    }
    CO_UNLOCK_CAN_SEND(CANmodule);

    if (tpdoDeleted != 0U) {
        CANmodule->CANerrorStatus |= CO_CAN_ERRTX_PDO_LATE;
    }
}

/* Get error counters from the module. If necessary, function may use different way to determine errors. */
static uint16_t rxErrors = 0, txErrors = 0, overflow = 0;

void
CO_CANmodule_process(CO_CANmodule_t* CANmodule) 
{
    uint32_t err;

    err = ((uint32_t)txErrors << 16) | ((uint32_t)rxErrors << 8) | overflow;

    if (CANmodule->errOld != err) {
        uint16_t status = CANmodule->CANerrorStatus;

        CANmodule->errOld = err;

        if (txErrors >= 256U) {
            /* bus off */
            status |= CO_CAN_ERRTX_BUS_OFF;
        } else {
            /* recalculate CANerrorStatus, first clear some flags */
            status &= 0xFFFF
                      ^ (CO_CAN_ERRTX_BUS_OFF | CO_CAN_ERRRX_WARNING | CO_CAN_ERRRX_PASSIVE | CO_CAN_ERRTX_WARNING
                         | CO_CAN_ERRTX_PASSIVE);

            /* rx bus warning or passive */
            if (rxErrors >= 128) {
                status |= CO_CAN_ERRRX_WARNING | CO_CAN_ERRRX_PASSIVE;
            } else if (rxErrors >= 96) {
                status |= CO_CAN_ERRRX_WARNING;
            }

            /* tx bus warning or passive */
            if (txErrors >= 128) {
                status |= CO_CAN_ERRTX_WARNING | CO_CAN_ERRTX_PASSIVE;
            } else if (txErrors >= 96) {
                status |= CO_CAN_ERRTX_WARNING;
            }

            /* if not tx passive clear also overflow */
            if ((status & CO_CAN_ERRTX_PASSIVE) == 0) {
                status &= 0xFFFF ^ CO_CAN_ERRTX_OVERFLOW;
            }
        }

        if (overflow != 0) {
            /* CAN RX bus overflow */
            status |= CO_CAN_ERRRX_OVERFLOW;
        }

        CANmodule->CANerrorStatus = status;
    }
}

static CO_CANrxMsg_t rxMsg; /*接收的CAN帧，用于存储接收的CAN帧信息*/
static CAN_RxHeaderTypeDef rxHeader; /* 接收帧头 */
void
CO_CANinterrupt(CO_CANmodule_t* CANmodule) 
{

    /* receive interrupt */
        CO_CANrxMsg_t* rcvMsg;     /* pointer to received CAN frame in CAN module */
        uint16_t index;            /* index of received CAN frame */
        uint32_t rcvMsgIdent;      /* identifier of the received CAN frame */
        CO_CANrx_t* buffer = NULL; /* receive CsAN frame buffer from CO_CANmodule_t object. */
        bool_t msgMatched = false; /*是否匹配的CAN帧*/

        if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxHeader, rxMsg.data) !=HAL_OK) {   /*接收CAN帧失败*/
            return; /*接收CAN帧失败*/
        }
        rxMsg.ident = rxHeader.StdId; /*接收的CAN帧标识符*/
        rxMsg.DLC = rxHeader.DLC; /*接收的CAN帧数据长度*/

        rcvMsg = &rxMsg;  /*接收的CAN帧指针*/
        rcvMsgIdent = rxMsg.ident; /*接收的CAN帧标识符*/

        
        buffer = &CANmodule->rxArray[0];  /*接收CAN帧缓冲区*/
               for (index = CANmodule->rxSize; index > 0U; index--) {  /*遍历CAN模块的接收缓冲区*/
            if (((rcvMsgIdent ^ buffer->ident) & buffer->mask) == 0U) {  /*判断是否匹配*/
                msgMatched = true;/*找到匹配的CAN帧*/
                break; /*找到匹配的CAN帧，跳出循环*/
            }
            buffer++; /*下一个CAN帧缓冲区*/
        }
/*如果匹配的CAN帧存在，且回调函数存在，调用回调函数*/
        if (msgMatched && (buffer != NULL) && (buffer->CANrx_callback != NULL)) {
            buffer->CANrx_callback(buffer->object, (void*)rcvMsg);
        }
}
