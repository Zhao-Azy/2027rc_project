#include "motor_T_Planning.h"

//电机反馈频率


#define ABS(x) ((x) > 0 ? (x) : (-(x)))



void Motor_PlanningInit(T_Planning_HandleTypeDef *T_Planning, float start_pos, float end_pos, float accelerate_ratio, float decelerate_ratio, float max_speed)
{

    T_Planning->first_flag = 0;				//这里有问题，这个函数只能执行一次，执行多次的话会被覆盖
	T_Planning->end_flag = 0;
    T_Planning->state     = eMOTOR_ACCELERATE_PHASE;	
	
	
    //这里让电机反馈当前角度为当前位置
	if(T_Planning->first_flag == 0)
    {
        T_Planning->start_pos = start_pos;                                              //起始点
        T_Planning->first_flag = 1;
    }
	

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


//位置规划初始化
void Motor_Pos_Planning_Init(T_Planning_HandleTypeDef *T_Planning, float start_pos, float target_pos, float temp)
{
    T_Planning->target_pos = target_pos;
    T_Planning->pos_temp = temp / 100;
    
    //这里让电机反馈当前角度为当前位置
	if(T_Planning->first_flag == 0)
    {
        T_Planning->end_flag = 0;
        T_Planning->start_pos = start_pos;                                              //起始点
        T_Planning->planning_pos = start_pos;
        T_Planning->first_flag = 1;
    }

	T_Planning->start_flag = 1;
}


float Motor_Planning(T_Planning_HandleTypeDef *T_Planning, float current_pos)
{

    T_Planning->current_distance = ABS(current_pos - T_Planning->start_pos);             //当前距离（在这个起始点和终止点之间）

    
    if(T_Planning->current_distance <= T_Planning->accelerate_distance)
    {
        T_Planning->state = eMOTOR_ACCELERATE_PHASE;
    }

    switch (T_Planning->state)
    {
        case eMOTOR_ACCELERATE_PHASE:
        {
			//这里必须加入一个微小的初速度，不然电机转不起来（刚开始的时候电机传入的当前距离是0）
            T_Planning->planning_speed = sqrtf(LOW_SPEED + 2.0f * T_Planning->accelerate_A * T_Planning->current_distance);
            if(T_Planning->current_distance >= T_Planning->accelerate_distance)
            {
                T_Planning->state = eMOTOR_CONSTANT_PHASE;
            }
            break;
        }
        case eMOTOR_CONSTANT_PHASE:
        {
            T_Planning->planning_speed = T_Planning->max_speed;
            if(T_Planning->current_distance >= (T_Planning->accelerate_distance + T_Planning->constant_distance))
            {
                T_Planning->state = eMOTOR_DECELERATE_PHASE;
            }
            break;
        }
        case eMOTOR_DECELERATE_PHASE:
        {
            T_Planning->planning_speed = sqrtf(T_Planning->max_speed * T_Planning->max_speed + 2.0f * T_Planning->decelerate_A * (T_Planning->current_distance - T_Planning->accelerate_distance - T_Planning->constant_distance));
            if(T_Planning->current_distance >= T_Planning->total_distance)
            {
                T_Planning->planning_speed = 0.0f;
                T_Planning->state = eMOTOR_END_PHASE;
            }
            break;
        }
        case eMOTOR_END_PHASE:
        {
            T_Planning->end_flag = 1;           //结束规划
            T_Planning->first_flag = 0;

            T_Planning->state = eMOTOR_ACCELERATE_PHASE;    //为下一次规划做准备

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



//动态规划电机位置   调用周期：1ms
float Pos_Planning(T_Planning_HandleTypeDef *T_Planning)
{
    if(T_Planning->target_pos > T_Planning->start_pos)
    {
        T_Planning->planning_pos += T_Planning->pos_temp;
		if(T_Planning->planning_pos >= T_Planning->target_pos) T_Planning->planning_pos = T_Planning->target_pos;
        if(fabsf(T_Planning->planning_pos - T_Planning->target_pos) < 0.001f)
        {
            T_Planning->planning_pos = T_Planning->target_pos;
            T_Planning->end_flag = 1;           //结束规划
            T_Planning->first_flag = 0;
        }
    }else if(T_Planning->target_pos < T_Planning->start_pos)
    {
        T_Planning->planning_pos -= T_Planning->pos_temp;
		if(T_Planning->planning_pos <= T_Planning->target_pos) T_Planning->planning_pos = T_Planning->target_pos;
        if(fabsf(T_Planning->planning_pos - T_Planning->target_pos) < 0.001f)
        {
            T_Planning->planning_pos = T_Planning->target_pos;
            T_Planning->end_flag = 1;           //结束规划
            T_Planning->first_flag = 0;        }
    }
    else if(fabsf(T_Planning->target_pos - T_Planning->start_pos) <= 0.01f)
    {
        T_Planning->planning_pos = T_Planning->target_pos;
        T_Planning->end_flag = 1;           //结束规划
        T_Planning->first_flag = 0;    }

		
    return T_Planning->planning_pos;
}