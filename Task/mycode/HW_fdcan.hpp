/*
 * @Author: Redbamboo 550362161@qq.com
 * @Date: 2025-10-26 22:31:34
 * @LastEditors: Redbamboo 550362161@qq.com
 * @LastEditTime: 2025-10-28 12:42:40
 * @FilePath: \Task\mycode\HW_fdcan.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
 *******************************************************************************
 * @file      :HW_fdcan.hpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      yyyy-mm-dd      <author>        1。<note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2025 Hello World Team,Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _HW_FDCAN_H_
#define _HW_FDCAN_H_
/* Includes ------------------------------------------------------------------*/
#include "fdcan.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

void FdcanFilterInit(FDCAN_HandleTypeDef *hfdcan, uint32_t fifo);
void FdcanSendMsg(FDCAN_HandleTypeDef *hfdcan, uint8_t *msg, uint32_t id, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif
