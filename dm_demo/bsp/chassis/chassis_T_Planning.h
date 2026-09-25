#ifndef CHASSIS_T_PLANNING_H
#define CHASSIS_T_PLANNING_H


#include "main.h"


typedef enum
{
    eCHASSIS_ACCELERATE_PHASE,
    eCHASSIS_CONSTANT_PHASE,
    eCHASSIS_DECELERATE_PHASE,
    eCHASSIS_END_PHASE,
}Chassis_T_Planning_State;


typedef struct 
{
    Chassis_T_Planning_State state;

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

}Chassis_T_Planning_HandleTypeDef;


void Chassis_PlanningInit(Chassis_T_Planning_HandleTypeDef *T_Planning, float start_pos, float end_pos, float accelerate_ratio, float decelerate_ratio, float max_speed);
float Chassis_Planning(Chassis_T_Planning_HandleTypeDef *T_Planning, float current_pos);



#endif