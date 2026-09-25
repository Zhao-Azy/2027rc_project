#ifndef UPER_CONTROL_H
#define UPER_CONTROL_H

#include "DJI_Transform_master.h"
#include "DJI_C620.h"
#include "drv_can.h"
#include "motor_T_Planning.h"
#include "VESC.h"
#include "RS05.h"
#include "RS00.h"
#include "MG5010E.h"
#include "handle_rev.h"
#include "pid.h"
#include "fdcan_it.h"
#include "HT3505_8.h"
#include "WS2811.h"

/*****************  电机id  ******************/
//舵轮
#define WheelLF_ID 51
#define WheelRF_ID 52
#define WheelRB_ID 53
#define WheelLB_ID 54           //轮向电机
#define HelmLF_ID  0x01
#define HelmRF_ID  0x02
#define HelmRB_ID  0x03
#define HelmLB_ID  0x04         //舵向电机

//机械臂
#define L_HIGH_ID 23
#define L_DOWN_ID 0x01
#define L_SUCKER_ID 24
#define R_HIGH_ID 21
#define R_DOWN_ID 0x08
#define R_SUCKER_ID 22

//简式抬升
#define R_FLEX_ID 0x03
#define L_FLEX_ID 0x06
#define FLEX_TRANSLATION_ID 0x03

//抓取武器机构
#define SPIN_CLOW_ID 11


#define WEAPON_FLEX_ID 0x02
#define STORAGE_MOTOR_ID 0x01



/**         宏函数        **/
#define Open_Arm_Clow()          HAL_GPIO_WritePin(IO_1_GPIO_Port, IO_1_Pin, GPIO_PIN_SET);            //机械臂爪子气缸
#define Close_Arm_Clow()         HAL_GPIO_WritePin(IO_1_GPIO_Port, IO_1_Pin, GPIO_PIN_RESET);

#define Open_Left_Clow()          HAL_GPIO_WritePin(IO_3_GPIO_Port, IO_3_Pin, GPIO_PIN_SET); 			//右爪子气缸
#define Close_Left_Clow()         HAL_GPIO_WritePin(IO_3_GPIO_Port, IO_3_Pin, GPIO_PIN_RESET);

#define Open_Right_Clow();           HAL_GPIO_WritePin(IO_2_GPIO_Port, IO_2_Pin, GPIO_PIN_SET);           //左爪子气缸  
#define Close_Right_Clow();          HAL_GPIO_WritePin(IO_2_GPIO_Port, IO_2_Pin, GPIO_PIN_RESET);

#define Open_Center_Cylinder()          HAL_GPIO_WritePin(IO_0_GPIO_Port, IO_0_Pin, GPIO_PIN_SET);                 //归中气缸
#define CLose_Center_Cylinder()         HAL_GPIO_WritePin(IO_0_GPIO_Port, IO_0_Pin, GPIO_PIN_RESET);

//简式抬升伸缩
#define FlexLift(pos, speed_limit, current_limit)           DJI_PositionMode(&Left_flex, pos, speed_limit, current_limit);       \
                                                            osDelay(1); \
                                                            DJI_PositionMode(&Right_flex, -pos, speed_limit, current_limit);


//机械臂电机结构体
typedef struct 
{
    RS00_HandleTypeDef High_motor;  // 高位电机
    MG5010E_HandleTypeDef Down_motor;  // 低位电机
    RS05_HandleTypeDef Sucker_motor;// 吸盘电机

    T_Planning_HandleTypeDef High_T;
    T_Planning_HandleTypeDef Sucker_T;

	float high_pos;
	float down_pos;
	float sucker_pos;

    void (*Open_Sucker)(void);          //第一个void 返回值类型  (*Open_Sucker)指针变量名  (void)参数列表
    void (*Close_Sucker)(void);

    uint8_t sucker_state;           //机械臂吸盘状态    
}MechanicalArm_HandleTypeDef;


typedef struct
{
    uint32_t wait_homing_tick;              //等待电机回零时间
    uint32_t storage_time;                  //存块时间
    uint32_t transportation_tick;            //运输武器时间
    uint32_t translation_tick;
    uint32_t transportation_tick2;
    uint32_t translation_tick2;
    uint32_t start_capture_tick;            //开始抓杆电机转出时间

}Uper_Timer_HandleTypeDef;


typedef struct
{
    //归中状态机
    uint8_t center_state;
    uint8_t center_start_flag;

    //机械臂存块状态机
    uint8_t arm_storage_state;
    uint8_t arm_storage_flag;
    uint8_t arm_transmit_flag;
}Uper_FSM_HandleTypeDef;


typedef struct
{
    uint8_t poke_weapon_flag;		//戳块标志位
    uint8_t ready_suction_flag;			//准备吸块标志位
    uint8_t storage_block_flag;		//存块标志位
    uint8_t center_flag;			//归中标志位
    uint8_t recycle_arm_flag;		//回收机械臂标志位
    uint8_t suction_flag;			//吸块标志位
    uint8_t ready_connect_flag;			//准备对接标志位
    uint8_t ready_capture_flag;			//准备抓取标志位
    uint8_t start_capture_flag;         //开始抓取标志位
    uint8_t transimit_weapon_flag;		//转接武器标志位 
    uint8_t transmit_second_flag;   
    uint8_t start_transmit_flag;    
    uint8_t sucker_flag;                //吸盘    
    uint8_t left_clow_flag;
    uint8_t right_clow_flag;
    uint8_t flex_falg;
}Uper_Flag_HandleTypeDef;


typedef struct
{
    //电机位置变量
    float storage_pos;		//存储2006位置
    float spin_clow_pos;			//旋转武器RS00位置
    float translation_pos;		//平移HT位置
    float weapon_flex_pos;		//武器伸缩2006   
    float flex_pos;             //抬升2006 
}Uper_MotorPos_HandleTypeDef;

// 按键
extern Key key_left[8];
extern Key key_right[8];
extern Key key_middle[8];

extern uint8_t key_left_value[8];
extern uint8_t key_right_value[8];
extern uint8_t key_middle_value[8];

//灯带
extern WS2811_HandleTypeDef LEDStrip;

//舵电机（全向轮底盘已移除）
// extern DJI_HandleTypeDef LF_steer;
// extern DJI_HandleTypeDef LB_steer;
// extern DJI_HandleTypeDef RF_steer;
// extern DJI_HandleTypeDef RB_steer;

//轮电机（全向轮：4个大疆电机，定义在 chassis.c）
extern DJI_C620_HandleTypeDef wheelLF;
extern DJI_C620_HandleTypeDef wheelRF;
extern DJI_C620_HandleTypeDef wheelRB;
extern DJI_C620_HandleTypeDef wheelLB;


//回零相关（全向轮底盘已移除）
// extern volatile Steer_Homing_State steer_homing_state;
// extern volatile Steer_Homing_TypeDef steer_homing[4];



//右侧机械臂
extern MechanicalArm_HandleTypeDef Right_arm;

//左侧机械臂
extern MechanicalArm_HandleTypeDef Left_arm;

//抓取武器部分
extern RS00_HandleTypeDef Spin_clow;
extern DJI_HandleTypeDef Weapon_flex;
extern RS05_HandleTypeDef Translation_motor;
extern DJI_HandleTypeDef Storage_motor;



//简式抬升部分
extern DJI_HandleTypeDef Right_flex;
extern DJI_HandleTypeDef Left_flex;


extern T_Planning_HandleTypeDef Spin_T;	


extern Uper_Timer_HandleTypeDef uper_timer;
extern Uper_Flag_HandleTypeDef uper_debug_flag;
extern Uper_FSM_HandleTypeDef uper_FSM;
extern Uper_MotorPos_HandleTypeDef uper_motor_pos;

extern volatile uint8_t crawling_completed;	//抓杆是否完成标志位
extern volatile uint8_t block_num;
extern volatile uint8_t center_finish_flag;
extern volatile uint8_t transmit_block_flag;        //递块标志位
extern volatile uint8_t place_block_flag;
extern volatile uint8_t led_state;

void Upper_Init(void);


#endif

