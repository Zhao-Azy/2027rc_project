#include "chassis.h"
#include "uper_control.h"
#include "chassis_control.h"

// 静态函数
static void Chassis_Contrary_Calculte(Chassis_Data *chassis_data);
static void Chassis_Pid_Init(void);
static float Angle_To_Rad(float angle);
static float Speed_Transform(float speed);
// static float Angle_Normalize(float angle); // 原舵轮代码使用，已停用
// static float Rad_To_Angle(float rad);       // 原舵轮代码使用，已停用

DJI_C620_HandleTypeDef wheelLF;
DJI_C620_HandleTypeDef wheelLB;
DJI_C620_HandleTypeDef wheelRF;
DJI_C620_HandleTypeDef wheelRB;
// pid结构体
PID_T pid_w = {0};


float a = 0.001f;

/**
 *  @brief 底盘初始化
 *  @param 轮半径
 *  @retval 无
 */
void Chassis_Init(Chassis_Data *chassis, float wheel_radius, float chassis_radius)
{
    chassis->wheel_radius = wheel_radius;     // 轮半径  单位：m
    chassis->chassis_radius = chassis_radius; // 底盘半径  单位：m

    /**  下面这两句是迫不得已  **/
    chassis->target_yaw = 180.0f; //
    chassis->current_yaw = 180.0f;

    Chassis_Pid_Init();
}

/**
 *  @brief 底盘控制
 *  @param 底盘结构体
 *  @param 速度枚举
 *  @param 全场坐标
 *  @param 目标点结构体
 *  @param 是否使用pid
 *  @param 是否自动
 *  @retval 无
 */
void Chassis_Control(Chassis_Data *chassis_data, Chassis_Mode_e chassis_mode,
                     uint8_t pid_flag)
{
    float speed_k = 0.0f;
    float w_k = 0.0f;

    chassis_data->chassis_mode = chassis_mode;

    switch (chassis_data->chassis_mode)
    {
    case stop:
        speed_k = 0.0f;
        w_k = 0.0f;
        break;
    case fine_adjustment:
        speed_k = 0.1f;
        w_k = 0.785f;
        break;
    case low_speed:
        speed_k = 0.3f;
        chassis_data->increase_speed_temp = 2.0f; // 加速度
        chassis_data->reduce_speed_temp = 2.0f;
        w_k = 1.5708f;
        break;
    case mid_speed:
        speed_k = 2.5f;
        chassis_data->increase_speed_temp = 0.3f; // 加速度
        chassis_data->reduce_speed_temp = 2.3f;
        w_k = 2.355f;
        break;
    default:
        break;
    }

    chassis_data->target_world_velocity.Vx = chassis_data->planning_velocity.Vx * speed_k * 0.01f;
    chassis_data->target_world_velocity.Vy = chassis_data->planning_velocity.Vy * speed_k * 0.01f;


    // 是否开启pid  这里一定要注意一下旋转方向的问题
    if (pid_flag)
    {
        // 计算yaw角误差(这里面的target_yaw 以及 current_yaw 都是角度制  注意！！！)
        chassis_data->err_yaw = (chassis_data->target_yaw - chassis_data->current_yaw) * ANGLE_PI; // 将误差转化为弧度制
        // yaw角误差归一化
        chassis_data->err_yaw = Limit_Max_Min(chassis_data->err_yaw);

        chassis_data->target_world_velocity.Vw = pid_calc_by_error(&pid_w, chassis_data->err_yaw);
    }
    else
    {
        chassis_data->target_world_velocity.Vw = chassis_data->input_world_velocity.Vw * w_k * 0.01f;
    }

    // 坐标系转换
    Coordinate_Transform_World_To_Robot(&chassis_data->target_world_velocity, &chassis_data->target_robot_velocity, chassis_data->current_yaw);
    // 逆解算轮速
    Chassis_Contrary_Calculte(chassis_data);
}

void Chassis_Control_Auto(Chassis_Data *chassis_data)
{

    // 坐标系转换
    Coordinate_Transform_World_To_Robot(&chassis_data->target_world_velocity, &chassis_data->target_robot_velocity, chassis_data->current_yaw);
    // 逆解算轮速
    Chassis_Contrary_Calculte(chassis_data);
}

/**
 *  @brief 底盘pid初始化
 *  @param 无
 *  @retval 无
 */
static void Chassis_Pid_Init(void)
{
    // pid参数初始化
    // 换用 HI14（CAN）陀螺仪后重调（依据 2026-09-25 实测阶跃日志）：
    //   - kd=4.0 会自激"快速左右摆动"（航向 100Hz 更新 vs 本环 500Hz，误差差分每 10ms 打尖峰）→ 保持 0；
    //   - kp=4.0：实测量得对象增益 ≈10.4 °/s 每单位 Vw，40° 阶跃约 1.5s 收到 ±6°、无过冲；
    //   - ki=0.002：纯 P 有 ±2° 静差，加很小的积分在到位后慢慢补掉
    //     （积分分离门限 0.2rad≈11.5°，打阶跃时积分被清零，不会卷绕）。
    pid_param_init(&pid_w,
                   PID_Position,
                   6.0f,          // 最大输出 角速度
                   1,             // 积分限制
                   0.2,           // 积分分离（rad≈11.5°）
                   0.003491f, // 死区（0.2度）
                   10.0f,         // 最大误差
                   4.0f,          // kp   （原 3.0；实测 4.0 收敛干净）
                   0.002f,        // ki   （原 0；用于消 ±2° 静差）
                   0.0f);         // kd   （原 4.0；会自激，保持 0）
}

// =====================================================================
//  原舵轮底盘逆解算（已停用，独立保留备用）
// =====================================================================
#if 0
/**
 *  @brief 舵轮底盘逆解算（原版）
 *  @param 底盘结构体
 *  @retval 无
 */
static void Chassis_Steer_Contrary_Calculte(Chassis_Data *chassis_data)
{
    // 舵向电机反馈角度  这里将反馈回来的角度转化为弧度   ********重要！*******
    chassis_data->steer_back_angle[0] = Angle_To_Rad(LF_steer.position + DELTA);
    chassis_data->steer_back_angle[1] = Angle_To_Rad(RF_steer.position + DELTA);
    chassis_data->steer_back_angle[2] = Angle_To_Rad(RB_steer.position + DELTA);
    chassis_data->steer_back_angle[3] = Angle_To_Rad(LB_steer.position + DELTA);


    // 底盘信息
    float rx = chassis_data->chassis_radius * sinf(Angle_To_Rad(THETA));
    float ry = chassis_data->chassis_radius * cosf(Angle_To_Rad(THETA));
    float Vx = chassis_data->target_robot_velocity.Vx;
    float Vy = chassis_data->target_robot_velocity.Vy;
    float Vw = chassis_data->target_robot_velocity.Vw;

    // 轮电机解算  前x左y
    chassis_data->wheel_cal_Speed[0] = sqrt(pow((Vx - Vw * ry), 2) + pow((Vy + Vw * rx), 2)); // LF
    chassis_data->wheel_cal_Speed[1] = sqrt(pow((Vx + Vw * ry), 2) + pow((Vy + Vw * rx), 2)); // RF
    chassis_data->wheel_cal_Speed[2] = sqrt(pow((Vx + Vw * ry), 2) + pow((Vy - Vw * rx), 2)); // RB
    chassis_data->wheel_cal_Speed[3] = sqrt(pow((Vx - Vw * ry), 2) + pow((Vy - Vw * rx), 2)); // LB

    //  舵向电机解算
    chassis_data->steer_cal_angle[0] = atan2f((Vy + Vw * rx), (Vx - Vw * ry)) + LF_ANGLE_OFFSET; // LF
    chassis_data->steer_cal_angle[1] = atan2f((Vy + Vw * rx), (Vx + Vw * ry)) + RF_ANGLE_OFFSET; // RF
    chassis_data->steer_cal_angle[2] = atan2f((Vy - Vw * rx), (Vx + Vw * ry)) + RB_ANGLE_OFFSET; // RB
    chassis_data->steer_cal_angle[3] = atan2f((Vy - Vw * rx), (Vx - Vw * ry)) + LB_ANGLE_OFFSET; // LB


    // 是否开启泊车  默认开启
    if (chassis_data->park_flag == 0)
    {
        if (fabsf(Vx) < 0.01f && fabsf(Vy) < 0.01f && fabsf(Vw) < 0.01f)
        {
            if (Vw > 0.0f)
            {
                chassis_data->steer_cal_angle[0] = Angle_To_Rad(180.0f - THETA) + LF_ANGLE_OFFSET; // LF
                chassis_data->steer_cal_angle[1] = Angle_To_Rad(THETA) + RF_ANGLE_OFFSET;           // RF
                chassis_data->steer_cal_angle[2] = -Angle_To_Rad(THETA) + RB_ANGLE_OFFSET;          // RB
                chassis_data->steer_cal_angle[3] = -Angle_To_Rad(180.0f - THETA) + LB_ANGLE_OFFSET; // LB
            }
            else
            {
                chassis_data->steer_cal_angle[0] = -Angle_To_Rad(THETA) + LF_ANGLE_OFFSET;          // LF
                chassis_data->steer_cal_angle[1] = -Angle_To_Rad(180.0f - THETA) + RF_ANGLE_OFFSET; // RF
                chassis_data->steer_cal_angle[2] = Angle_To_Rad(180.0f - THETA) + RB_ANGLE_OFFSET;  // RB
                chassis_data->steer_cal_angle[3] = Angle_To_Rad(THETA) + LB_ANGLE_OFFSET;           // LB
            }

        }
    }
    else // 不开启则保持原舵向
    {
        if (Vx == 0.0f && Vy == 0.0f && Vw == 0.0f)
        {
            chassis_data->steer_cal_angle[0] = chassis_data->last_steer_cal_angle[0];
            chassis_data->steer_cal_angle[1] = chassis_data->last_steer_cal_angle[1];
            chassis_data->steer_cal_angle[2] = chassis_data->last_steer_cal_angle[2];
            chassis_data->steer_cal_angle[3] = chassis_data->last_steer_cal_angle[3];
        }
    }

    // 刹车时泊车
    if (chassis_data->stop_flag == 1 || chassis_data->ladar_err_flag == 1)
    {
        chassis_data->steer_cal_angle[0] = Angle_To_Rad(90.0f - THETA) + LF_ANGLE_OFFSET; // LF
        chassis_data->steer_cal_angle[1] = Angle_To_Rad(THETA - 90.0f) + RF_ANGLE_OFFSET; // RF
        chassis_data->steer_cal_angle[2] = Angle_To_Rad(90.0f - THETA) + RB_ANGLE_OFFSET; // RB
        chassis_data->steer_cal_angle[3] = Angle_To_Rad(THETA - 90.0f) + LB_ANGLE_OFFSET; // LB

        chassis_data->wheel_cal_Speed[0] = 0.0f;
        chassis_data->wheel_cal_Speed[1] = 0.0f;
        chassis_data->wheel_cal_Speed[2] = 0.0f;
        chassis_data->wheel_cal_Speed[3] = 0.0f;
    }

    // 最小舵角处理
    for (int i = 0; i < 4; i++)
    {
        chassis_data->angle_err[i] = chassis_data->steer_cal_angle[i] - chassis_data->steer_back_angle[i]; // 求出两角误差
        chassis_data->angle_err[i] = Angle_Normalize(chassis_data->angle_err[i]);                          // 角度归一化


        if (chassis_data->angle_err[i] > M_PI / 2 && chassis_data->angle_err[i] <= M_PI)
        {
            chassis_data->steer_cal_angle[i] = chassis_data->steer_back_angle[i] + chassis_data->angle_err[i] - M_PI;
            chassis_data->wheel_cal_Speed[i] = -chassis_data->wheel_cal_Speed[i];
        }
        else if (chassis_data->angle_err[i] >= -M_PI && chassis_data->angle_err[i] < -(M_PI / 2))
        {
            chassis_data->steer_cal_angle[i] = chassis_data->steer_back_angle[i] + chassis_data->angle_err[i] + M_PI;
            chassis_data->wheel_cal_Speed[i] = -chassis_data->wheel_cal_Speed[i];
        }else
        {
            chassis_data->steer_cal_angle[i] = chassis_data->steer_back_angle[i] + chassis_data->angle_err[i];
        }
    }

    // 给上一次舵向角度赋值
    for (int i = 0; i < 4; i++)
    {
        chassis_data->last_steer_cal_angle[i] = chassis_data->steer_cal_angle[i];
    }

    // 给电机赋值
    for (int i = 0; i < 4; i++)
    {
        chassis_data->wheel_speed[i] = Speed_Transform(chassis_data->wheel_cal_Speed[i] / chassis_data->wheel_radius); // 将输出的线速度改为角速度 再改为rpm  m/s -> rad/s -> rpm
        chassis_data->steer_angle[i] = Rad_To_Angle(chassis_data->steer_cal_angle[i]);                                 // 将解算出来的弧度转化为角度
    }
}
#endif // 0

/**
 *  @brief 全向轮（麦克纳姆轮）底盘逆解算
 *  @param 底盘结构体
 *  @retval 无
 */
static void Chassis_Contrary_Calculte(Chassis_Data *chassis_data)
{
    // 底盘几何参数：麦轮在底盘坐标系中的半轮距
    // 沿用原舵轮底盘的底盘半径与THETA角换算轮距  前x左y
    float lx = chassis_data->chassis_radius * sinf(Angle_To_Rad(THETA)); // 纵向半轮距 x
    float ly = chassis_data->chassis_radius * cosf(Angle_To_Rad(THETA)); // 横向半轮距 y
    float Vx = chassis_data->target_robot_velocity.Vx;
    float Vy = chassis_data->target_robot_velocity.Vy;
    float Vw = chassis_data->target_robot_velocity.Vw;

    // 麦轮逆运动学解算  轮序: 0=左前(LF) 1=右前(RF) 2=右后(RB) 3=左后(LB)
    // 轮速 = 前后分量 ± 左右分量 ± 旋转分量*(lx+ly)
    // 注意：若实测平移或旋转方向相反，需相应取反 Vy / Vw
    chassis_data->wheel_cal_Speed[0] =  Vx - Vy +	2*Vw * (lx + ly); // LF
    chassis_data->wheel_cal_Speed[1] =  Vx + Vy + 2*Vw * (lx + ly); // RF
    chassis_data->wheel_cal_Speed[2] = -Vx + Vy + 2*Vw * (lx + ly); // RB
    chassis_data->wheel_cal_Speed[3] = -Vx - Vy + 2*Vw * (lx + ly); // LB

    // 刹车/雷达错误时急停
    if (chassis_data->stop_flag == 1 || chassis_data->ladar_err_flag == 1)
    {
        chassis_data->wheel_cal_Speed[0] = 0.0f;
        chassis_data->wheel_cal_Speed[1] = 0.0f;
        chassis_data->wheel_cal_Speed[2] = 0.0f;
        chassis_data->wheel_cal_Speed[3] = 0.0f;
    }

    // 麦轮底盘无舵向电机，舵向角固定为0
    for (int i = 0; i < 4; i++)
    {
        chassis_data->steer_cal_angle[i] = 0.0f;
        chassis_data->steer_angle[i] = 0.0f;
    }

    // 给电机赋值
    for (int i = 0; i < 4; i++)
    {
        chassis_data->wheel_speed[i] = Speed_Transform(chassis_data->wheel_cal_Speed[i] / chassis_data->wheel_radius); // m/s -> rad/s -> rpm
    }
}

void Velocity_Planning(Chassis_Data *chassis_data)
{
    float increase_speed_temp = chassis_data->increase_speed_temp;
    float reduce_speed_temp = chassis_data->reduce_speed_temp;

    // x向
    float cur_vx = chassis_data->input_world_velocity.Vx;
    float last_vx = chassis_data->planning_velocity.Vx;
    // y向
    float cur_vy = chassis_data->input_world_velocity.Vy;
    float last_vy = chassis_data->planning_velocity.Vy;

    // x向
    if (cur_vx * last_vx >= 0 && (fabsf(cur_vx) - fabsf(last_vx)) >= 0) // 加速
    {
        if (cur_vx >= 0) // 正向加速
        {
            chassis_data->planning_velocity.Vx += increase_speed_temp;
            if (chassis_data->planning_velocity.Vx > cur_vx)
                chassis_data->planning_velocity.Vx = cur_vx;
        }
        else // 反向加速
        {
            chassis_data->planning_velocity.Vx -= increase_speed_temp;
            if (chassis_data->planning_velocity.Vx < cur_vx)
                chassis_data->planning_velocity.Vx = cur_vx;
        }
    }
    else if ((cur_vx * last_vx >= 0 && (fabsf(cur_vx) - fabsf(last_vx)) < 0) || cur_vx * last_vx < 0) // 减速
    {
        if (cur_vx * last_vx >= 0)
        {
            if (last_vx >= 0) // 正向减速 100 → 0
            {
                chassis_data->planning_velocity.Vx -= reduce_speed_temp;
                if (chassis_data->planning_velocity.Vx < cur_vx)
                    chassis_data->planning_velocity.Vx = cur_vx;
            }
            else if (last_vx <= 0) // 反向减速 -100 → 0
            {
                chassis_data->planning_velocity.Vx += reduce_speed_temp;
                if (chassis_data->planning_velocity.Vx > cur_vx)
                    chassis_data->planning_velocity.Vx = cur_vx;
            }
        }
        else
        {
            if (last_vx >= 0) // 正向速度到负向速度的突变  100~-100
            {
                chassis_data->planning_velocity.Vx -= reduce_speed_temp;
                if (chassis_data->planning_velocity.Vx < cur_vx)
                    chassis_data->planning_velocity.Vx = cur_vx;
            }
            else if (last_vx < 0) // 负向速度到正向速度的突变  100~-100
            {
                chassis_data->planning_velocity.Vx += reduce_speed_temp;
                if (chassis_data->planning_velocity.Vx > cur_vx)
                    chassis_data->planning_velocity.Vx = cur_vx;
            }
        }
    }

    // y向
    if (cur_vy * last_vy >= 0 && (fabsf(cur_vy) - fabsf(last_vy)) >= 0) // 加速
    {
        if (cur_vy >= 0) // 正向加速
        {
            chassis_data->planning_velocity.Vy += increase_speed_temp;
            if (chassis_data->planning_velocity.Vy > cur_vy)
                chassis_data->planning_velocity.Vy = cur_vy;
        }
        else // 反向加速
        {
            chassis_data->planning_velocity.Vy -= increase_speed_temp;
            if (chassis_data->planning_velocity.Vy < cur_vy)
                chassis_data->planning_velocity.Vy = cur_vy;
        }
    }
    else if ((cur_vy * last_vy >= 0 && (fabsf(cur_vy) - fabsf(last_vy)) < 0) || cur_vy * last_vy < 0)
    {
        if (cur_vy * last_vy >= 0)
        {
            if (last_vy >= 0) // 正向减速 100 → 0
            {
                chassis_data->planning_velocity.Vy -= reduce_speed_temp;
                if (chassis_data->planning_velocity.Vy < cur_vy)
                    chassis_data->planning_velocity.Vy = cur_vy;
            }
            else if (last_vy <= 0) // 反向减速 -100 → 0
            {
                chassis_data->planning_velocity.Vy += reduce_speed_temp;
                if (chassis_data->planning_velocity.Vy > cur_vy)
                    chassis_data->planning_velocity.Vy = cur_vy;
            }
        }
        else
        {
            if (last_vy >= 0) // 100~-100
            {
                chassis_data->planning_velocity.Vy -= reduce_speed_temp;
                if (chassis_data->planning_velocity.Vy < cur_vy)
                    chassis_data->planning_velocity.Vy = cur_vy;
            }
            else if (last_vy < 0) // 100~-100
            {
                chassis_data->planning_velocity.Vy += reduce_speed_temp;
                if (chassis_data->planning_velocity.Vy > cur_vy)
                    chassis_data->planning_velocity.Vy = cur_vy;
            }
        }
    }
}

/**
 *  @brief 坐标系转化（顺时针旋转矩阵）
 *  @param 世界坐标系
 *  @param 机器人坐标系
 *  @param 底盘yaw角
 *  @retval 无
 */
void Coordinate_Transform_World_To_Robot(Velocity_Data *world_data, Velocity_Data *robot_data, float yaw)
{
    yaw = yaw * ANGLE_PI; // 度数与弧度的转化
    float cos_yaw = cosf(yaw);
    float sin_yaw = sinf(yaw);

    robot_data->Vx = cos_yaw * world_data->Vx + sin_yaw * world_data->Vy;
    robot_data->Vy = -sin_yaw * world_data->Vx + cos_yaw * world_data->Vy;
    robot_data->Vw = world_data->Vw;
}

// 速度转化 由rad/s转化到rpm  rpm = 60 * W / 2 * PI
static float Speed_Transform(float speed)
{
    return speed * (60.0f / (2 * M_PI));
}

// 限幅函数
float Limit_Max_Min(float value)
{
    while (value > 3.16159265357f)
        value -= 2 * 3.14159265357f;
    while (value < -3.176499)
        value += 2 * 3.14159265357f;

    return value;
}

/**
 *  @brief 角度转换
 *  @param 角度 （单位：度）
 *  @retval 弧度制的角度
 */
static float Angle_To_Rad(float angle)
{
    return (angle * 0.0174532925f);
}

// =====================================================================
//  原舵轮代码使用的辅助函数（已停用，保留备用）
// =====================================================================
#if 0
/**
 * @brief 弧度转角度
 * @param rad 弧度值
 * @retval 角度值（单位：度）
 */
static float Rad_To_Angle(float rad)
{
    return (rad * 57.2957795f); // 180 / π ≈ 57.2957795
}

// 角度归一化
static float Angle_Normalize(float angle)
{
    float m = fmod(angle, 2 * M_PI);

    if (m <= -M_PI)
    {
        m += 2 * M_PI;
    }
    else if (m >= M_PI)
    {
        m -= 2 * M_PI;
    }


    return m;
}
#endif // 0
