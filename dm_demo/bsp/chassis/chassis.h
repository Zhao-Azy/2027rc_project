#ifndef CHASSIS_H
#define CHASSIS_H

#include "main.h"
#include "math.h"
#include "pid.h"

#define M_PI 3.14159265358979323846
// #define M_PI 3.12f
#define ANGLE_RANGE -0.0004f
#define ANGLE_PI 0.017453292f // 角度转弧度的系数 PI/180
#define THETA 44.2611f    //因为底盘不是正方形


#define DELTA 2.4f

#define LF_ANGLE_OFFSET (-21.492f + DELTA) * ANGLE_PI
#define RF_ANGLE_OFFSET (-60.279f + DELTA) * ANGLE_PI
#define RB_ANGLE_OFFSET (35.467f + DELTA) * ANGLE_PI
#define LB_ANGLE_OFFSET (59.942f + DELTA) * ANGLE_PI


// 速度结构体
typedef struct
{
    float Vx;
    float Vy;
    float Vw;
} Velocity_Data;

// 速度模式枚举
typedef enum
{
    stop,
    fine_adjustment,
    low_speed,
    mid_speed
} Chassis_Mode_e;

// 底盘结构体
typedef struct
{
    Chassis_Mode_e chassis_mode;

    Velocity_Data input_world_velocity;      // 输入世界坐标系速度
    Velocity_Data planning_velocity;         // 规划之后的速度  让加减速更平滑

    Velocity_Data target_world_velocity; // 目标世界坐标系速度
    Velocity_Data target_robot_velocity; // 目标机器人坐标系速度

    /**                 关于解算                  **/
    float wheel_cal_Speed[4];       // 轮向电机
    float steer_cal_angle[4];       // 舵向电机解算出的角度	
    float last_steer_cal_angle[4];     //上一次计算的舵向角度
    float angle_err[4];             // 角度误差数组
    float steer_back_angle[4];            // 舵向电机反馈角度
    /**                 关于解算                  **/


    float wheel_radius;   // 轮半径
    float chassis_radius; // 底盘半径

    float target_yaw;  // 目标yaw角
    float current_yaw; // 当前yaw角 赋值的时候必须为弧度制
    float err_yaw;

    float wheel_speed[4]; // 四轮速度       （最终发给can任务的数据）
    float steer_angle[4]; // 舵轮角度

    float increase_speed_temp;          //加速阶段的加速度
    float reduce_speed_temp;            //减速阶段的加速度

    uint8_t stop_flag;
    uint8_t park_flag;          //泊车标志位

    uint8_t ladar_err_flag;     //雷达错误标志位

} Chassis_Data;


// 底盘电机数据结构体
typedef struct
{
    float wheel_speed[4]; // 四轮速度
    float steer_angle[4]; // 舵轮角度
} Chassis_Motor;


extern PID_T pid_w;


void Chassis_Init(Chassis_Data *chassis, float wheel_radius, float chassis_radius);
void Coordinate_Transform_World_To_Robot(Velocity_Data *world_data, Velocity_Data *robot_data, float yaw);
void Chassis_Control(Chassis_Data *chassis_data, Chassis_Mode_e chassis_mode, uint8_t pid_flag);
void Chassis_Control_Auto(Chassis_Data *chassis_data);
void Velocity_Planning(Chassis_Data *chassis_data);
float Limit_Max_Min(float value);

#endif
