#ifndef __CAN_PUB_H__
#define __CAN_PUB_H__

#include <stdbool.h>
#include <stdint.h>
#include "main.h"

#ifdef __has_include
#define USER_CAN_HEADER_EXISTS(x) __has_include(x)
#else
#define USER_CAN_HEADER_EXISTS(x) 0
#endif

#if USER_CAN_HEADER_EXISTS("fdcan.h") || defined(HAL_FDCAN_MODULE_ENABLED)
#include "fdcan.h"
#define USER_CAN_HAS_FDCAN 1
#else
#define USER_CAN_HAS_FDCAN 0
#endif

#if USER_CAN_HEADER_EXISTS("can.h") || defined(HAL_CAN_MODULE_ENABLED)
#include "can.h"
#define USER_CAN_HAS_CAN 1
#else
#define USER_CAN_HAS_CAN 0
#endif

typedef enum
{
    CAN_STANDARD,//标准帧
    CAN_EXTENDED //扩展帧
} can_id_type_t; //CAN ID帧类型

typedef struct
{
    uint32_t id;                       //11 位标准 ID 或 29 位扩展 ID 
    can_id_type_t id_type;             //CAN ID 类型 
    uint8_t dlc;                       //有效数据长度，经典 CAN 范围为 0~8
    uint8_t data[8];                   //CAN 数据区域
} can_frame_t;

typedef bool (*can_send_t)(void *user_context,const can_frame_t *frame);

#if USER_CAN_HAS_CAN
void USER_CAN1_INIT(CAN_HandleTypeDef *hcan);
void USER_CAN2_INIT(CAN_HandleTypeDef *hcan);
bool CAN_Send(void *user_context,const can_frame_t *frame);
bool CAN_Read(CAN_HandleTypeDef *hcan, can_frame_t *frame);
#endif

#if USER_CAN_HAS_FDCAN
void USER_FDCAN1_INIT(FDCAN_HandleTypeDef *hfdcan);
bool FDCAN_Send(void *user_context,const can_frame_t *frame);
bool FDCAN_Read(FDCAN_HandleTypeDef *hfdcan, can_frame_t *frame);
#endif

#endif
