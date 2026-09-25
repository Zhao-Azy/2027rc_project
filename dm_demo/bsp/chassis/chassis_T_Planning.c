#include "chassis_T_Planning.h"

#define ABS(x) ((x) > 0 ? (x) : (-(x)))




void Chassis_PlanningInit(Chassis_T_Planning_HandleTypeDef *T_Planning, float start_pos, float end_pos, float accelerate_ratio, float decelerate_ratio, float max_speed)
{
	
    //如果没有结束上一次规划直接返回
	if(T_Planning->start_flag == 1 && T_Planning->end_flag == 0)
    {
        return;
    }


    T_Planning->state     = eCHASSIS_ACCELERATE_PHASE;	
    T_Planning->start_pos = start_pos;                                              //起始点
    T_Planning->end_flag = 0;

	

    T_Planning->end_pos = end_pos;                                                  //终止点                                
    T_Planning->accelerate_ratio = accelerate_ratio;                                //加速比例
    T_Planning->decelerate_ratio = decelerate_ratio;                                 //减速比例
    T_Planning->max_speed = max_speed;                                              //最大速度

    if(accelerate_ratio + decelerate_ratio >= 1.0f || accelerate_ratio == 0 || decelerate_ratio == 0 || max_speed == 0.0f)
    {
        // 参数错误，无法进行规划
        return;
    }

    //计算总距离
    T_Planning->total_distance = ABS(T_Planning->start_pos - T_Planning->end_pos);

    // 计算加速距离
    T_Planning->accelerate_distance = T_Planning->total_distance * accelerate_ratio;

    // 计算减速距离
    T_Planning->decelerate_distance = T_Planning->total_distance * decelerate_ratio;

    // 计算匀速距离
    if(accelerate_ratio + decelerate_ratio >= 1.0f)
    {
        T_Planning->constant_distance = 0.0f; // 如果加速和减速比例之和大于等于1，则没有匀速阶段
    }
    else
    {
        T_Planning->constant_distance = T_Planning->total_distance - (T_Planning->accelerate_distance + T_Planning->decelerate_distance);             
    }

    //计算加速加速度  
    T_Planning->accelerate_A = (T_Planning->max_speed * T_Planning->max_speed - 0.0f) / (2.0f * T_Planning->accelerate_distance);

    //计算减速加速度  带符号
    T_Planning->decelerate_A = (0.0f - T_Planning->max_speed * T_Planning->max_speed) / (2.0f * T_Planning->decelerate_distance);
    

    T_Planning->start_flag = 1;
}



float Chassis_Planning(Chassis_T_Planning_HandleTypeDef *T_Planning, float current_pos)
{

    T_Planning->current_distance = ABS(current_pos - T_Planning->start_pos);             //当前距离（在这个起始点和终止点之间）

    
    if(T_Planning->current_distance <= T_Planning->accelerate_distance)
    {
        T_Planning->state = eCHASSIS_ACCELERATE_PHASE;
    }else if(T_Planning->current_distance >= T_Planning->total_distance)
    {
        T_Planning->planning_speed = 0.0f;
		T_Planning->state = eCHASSIS_END_PHASE;
    }


    switch (T_Planning->state)
    {
        case eCHASSIS_ACCELERATE_PHASE:
        {
            T_Planning->planning_speed = sqrtf(2.0f * T_Planning->accelerate_A * T_Planning->current_distance);
            if(T_Planning->current_distance >= T_Planning->accelerate_distance && T_Planning->current_distance <= T_Planning->accelerate_ratio + T_Planning->constant_distance)
            {
                T_Planning->state = eCHASSIS_CONSTANT_PHASE;
            }
            break;
        }
        case eCHASSIS_CONSTANT_PHASE:
        {
            T_Planning->planning_speed = T_Planning->max_speed;
            if(T_Planning->current_distance >= (T_Planning->accelerate_distance + T_Planning->constant_distance) && T_Planning->current_distance < T_Planning->total_distance)
            {
                T_Planning->state = eCHASSIS_DECELERATE_PHASE;
            }
            break;
        }
        case eCHASSIS_DECELERATE_PHASE:
        {
            T_Planning->planning_speed = sqrtf(T_Planning->max_speed * T_Planning->max_speed + 2.0f * T_Planning->decelerate_A * (T_Planning->current_distance - T_Planning->accelerate_distance - T_Planning->constant_distance));
            if(T_Planning->current_distance >= T_Planning->total_distance)
            {
                T_Planning->planning_speed = 0.0f;
                T_Planning->state = eCHASSIS_END_PHASE;
            }
            break;
        }
        case eCHASSIS_END_PHASE:
        {
            T_Planning->end_flag = 1;           //结束规划
            T_Planning->start_flag = 0;

            T_Planning->state = eCHASSIS_ACCELERATE_PHASE;    //为下一次规划做准备

            break;
        }
        default:
            break;
    }

    //如果反向运动，就速度取反
    if(T_Planning->start_pos > T_Planning->end_pos)
    {
        T_Planning->planning_speed = -T_Planning->planning_speed;
    }


    return T_Planning->planning_speed;
}
