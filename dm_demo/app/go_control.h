#ifndef GO_CONTROL_H
#define GO_CONTROL_H

#include "main.h"
#include "Unitree_GO.h"

/* 宇树 GO-M8010-6 关节电机：两台挂在板载 RS485 第 1 路（USART2，4Mbps，DE=PD4）
 * ID 分别为 0 和 1；上电 500ms 后各自“当前位置 + 30°”，之后一直保持。 */
#define GO_JOINT_NUM        2

extern Unitree_GO_HandleTypeDef go_joint[GO_JOINT_NUM];

/** @brief 初始化两台 GO 电机（只建句柄；USART2 需先由 MX_USART2_UART_Init 配好） */
void GO_App_Init(void);

/** @brief GO 电机任务：500ms 后使能并转 +30°，之后 5ms 一轮轮询保持 */
void GO_Task(void *argument);

#endif /* GO_CONTROL_H */
