#ifndef MOTOR_T_PLANNING_H
#define MOTOR_T_PLANNING_H

#include "main.h"
#include "math.h"

#define LOW_SPEED 0.1
#define HIGH_SPEED 1


typedef enum
{
    eMOTOR_ACCELERATE_PHASE,
    eMOTOR_CONSTANT_PHASE,
    eMOTOR_DECELERATE_PHASE,
    eMOTOR_END_PHASE,
}T_Planning_State;


typedef struct 
{
    float start_pos;                    //开始位置
    float end_pos;                      //结束位置
    float current_distance;             //当前位置（相对于开始位置）
    float total_distance;               //总距离

    float accelerate_ratio;             //加速路程比例
    float decelerate_ratio;             //减速路程比例
    float constant_distance;            //匀速路程

    float accelerate_distance;          //加速路程
    float decelerate_distance;          //减速路程

    float accelerate_A;              //加速阶段加速度
    float decelerate_A;              //减速阶段加速度

    float max_speed;                //最大速度
    float planning_speed;           //规划速度
	
    uint8_t end_flag;	
	uint8_t first_flag;
    uint8_t start_flag;

    float target_pos;
    float planning_pos; 
    float pos_temp;
    float planning_end_pos;

    T_Planning_State state;          //规划状态

}T_Planning_HandleTypeDef;


//根据位置规划速度，T型规划
void Motor_PlanningInit(T_Planning_HandleTypeDef *T_Planning, float start_pos, float end_pos, float accelerate_ratio, float decelerate_ratio, float max_speed);
float Motor_Planning(T_Planning_HandleTypeDef *T_Planning, float current_pos);


//逐步递增位置   用于电机的mit模式
void Motor_Pos_Planning_Init(T_Planning_HandleTypeDef *T_Planning, float start_pos, float target_pos, float temp);
float Pos_Planning(T_Planning_HandleTypeDef *T_Planning);

#endif
