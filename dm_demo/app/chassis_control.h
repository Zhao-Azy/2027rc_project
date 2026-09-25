#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H


#include "handle_rev.h"
#include "uper_control.h"
#include "chassis.h"
#include "gyro_MINS500.h"
#include "radar.h"
#include "DT35.h"
#include "path.h"
#include "QT.h"
#include "STP_23L.h"


//DT35       
#define DT35_XF DT35.value[0]
#define DT35_YL DT35.value[1]
#define DT35_XB DT35.value[2]
#define DT35_YR DT35.value[3]


extern Chassis_Data chassis_data;
extern Gyro MINS500_data;
extern Radar_HandleTypeDef radar;
extern DT35_U DT35;
extern Path_HandleTypeDef path;
extern QT_HandleTypeDef qt;
extern uint8_t air_pum_state[4];
extern volatile uint8_t auto_flag; // 自动标志位
extern volatile uint8_t auto_finish_flag;
extern volatile uint8_t gyro_reset_flag;

void Chassis_Components_Init(void);


#endif
