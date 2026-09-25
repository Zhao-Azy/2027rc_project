#ifndef PATH_LIB_H
#define PATH_LIB_H

#include "main.h"
#include "qt.h"
#include "pid.h"
#include "my_main.h"

typedef enum
{
    ePREPARE = 0,         //准备状态
    eWEAPON_RACK,           //武器架
    eCONNECT_WEAPON,           //对接武器
    eLEFT_AISLE,                //左过道起点
    eRIGHT_AISLE,
    eMIDDLE_AISLE,              //代表第一行中间的那个点位
    
    eL1,
    eL2,
    eL3,
    eL4,                        //左侧过道4个物块点位 
    eLEFT_EXIT,                 //左侧过道出口

    eR1,
    eR2,
    eR3,
    eR4,                        //右侧过道4个物块点位
    eRIGHT_EXIT,                //右侧过道出口

    eMIDDLE_EXIT,               //中间过道出口



    eZone3_ACTION_AREA,         //三区启动区
    eSUDOKO_1,                  //九宫格点位  场地外侧为1  场地内侧为3
    eSUDOKO_2,                  //九宫格中间
    eSUDOKO_3,                  
	
	eBLIFT_POINT,
    eFLIFT_POINT,
    eMLIFT_POINT,

    eFT_LIFT_Point,
    eBT_LIFT_Point,

    eTRANSMIT_BLOCK_POINT,
}Path_State;             //其实就是路径点


typedef enum
{
    CROSS_NONE = 0,
    CROSS_TO_LEFT_AISLE,
    CROSS_TO_LEFT_EXIT,
    CROSS_TO_RIGHT_AISLE,
    CROSS_TO_RIGHT_EXIT,
    CROSS_TO_TARGET    
}Cross_State;


typedef struct 
{
    float x;
    float y;
    float yaw;

    uint8_t area;
    uint8_t aisle;

    uint8_t is_outside;     //是否在过道， 出口入口点全部为1

}Point_HandleTypeDef;           //坐标点结构体


typedef struct 
{
	uint8_t DT_flag;                    //是否允许DT35微调

    uint8_t fine_tuning_weapon_flag;    //微调武器标志位

    float percentage;                 //比例

    //用于标志位只置1一次的标志位 
    uint8_t ready_capture_flag;
    uint8_t ready_connect_flag;
    uint8_t ready_transform_flag;
    uint8_t ready_lift_flag;

    //准备吸块标志位
    uint8_t ready_suction_flag;
    uint8_t take_block_flag;
    uint8_t recycle_arm_flag;
    uint8_t close_sucker_flag;
    uint8_t transmit_block_flag;            //递块标志位
    uint8_t confront_flag;              //对抗标志位
    uint8_t close_R_sucker_flag;          //递块关闭吸盘

    uint8_t is_chunk;       //当前所处梅林高度

    float x_speed_limit;            //速度限制
    float y_speed_limit;

    uint8_t sucker_time_flag; 

}Flag_HandleTypeDef;

typedef struct
{
    //输出的三个方向的速度
    float Vx;
    float Vy;
    float Vw;

    Path_State path_state;                      //路径状态
    Path_State last_path_state;                 //上一个路径状态
    Cross_State cross_state;
    uint8_t is_cross_flag;                  //是否需要横穿标志位
	uint8_t transform_weapon_count;

    Point_HandleTypeDef current_point;          //当前坐标点
    Point_HandleTypeDef target_point;           //目标坐标点
    Point_HandleTypeDef err_point;              //当前坐标点与目标坐标点的误差值
    Flag_HandleTypeDef R1;

    uint8_t fine_tuning_flag;           //是否开启微调
    uint8_t ladar_flag;              //是否开启雷达拉   

    float fine_tuning_x;               //世界坐标系下x方向微调距离
    float fine_tuning_y;              //世界坐标系下y方向微调距离

    uint8_t direction;          //左侧道路还是右侧道路
    uint8_t step;           //当前走到第几个点了
    uint8_t is_spin_flag;           //是否允许输出旋转方向速度

    Path_State *route;        //当前路径
    uint8_t point_num;

    uint8_t suction_arm;        //用于判断用哪个机械臂吸块
    uint8_t gain_arm;        //用于判断当前用哪个机械臂取归中里的块

    uint8_t line;           //放那一列
    uint8_t is_switch;          //是否切换斜着放置还是竖着放置

}Path_HandleTypeDef;



enum
{
    DT_XF_YL,
    DT_XF_YR,
    DT_XB_YL,
    DT_XB_YR
};


//当前点区域
enum
{
    eONE_AREA = 1,
    eTWO_AREA,
    eTHREE_AREA,
};


//当前点所处过道位置
enum
{
    eRIGHT = 1,
    eMIDDLE = 2,
    eLEFT = 3,
};


//使用哪边的机械臂
enum
{
    left_arm = 1,
    right_arm,
};


extern Path_State left_state[];
extern Path_State right_state[];
extern Path_State QT_State[3];
extern Point_HandleTypeDef target_pose[];


extern PID_T DT_x;
extern PID_T DT_y; 

void Path_Update_Status(Path_HandleTypeDef *path, Path_State state);
void Path_ParaInit(Path_HandleTypeDef *path, float fine_tuning_x, float fine_tuning_y, uint8_t is_yaw);
uint8_t Ladar_Determination(Point_HandleTypeDef curren_point, Point_HandleTypeDef target_pose, float dead_x, float dead_y, uint8_t count);
uint8_t DT35_Determination(Path_HandleTypeDef *path, float dt_x, float dt_y, float x_deadband, float y_deadband, uint8_t count);
void DT35_XF_Calc(Path_HandleTypeDef *path, float yaw);
void DT35_XB_Calc(Path_HandleTypeDef *path, float yaw);
void DT35_YL_Calc(Path_HandleTypeDef *path, float yaw);
void DT35_YR_Calc(Path_HandleTypeDef *path, float yaw);
void QT_target_process(QT_HandleTypeDef *qt, Path_State *state);
void Across_Process(Path_HandleTypeDef *path);
void Speed_limit(float *speed, float limit);
void Speed_para(Path_HandleTypeDef *path, float x_dis, float x_limit, float y_dis, float y_limit);
void Change_speed(float x_target_speed, float y_target_speed);
void GiveSemaphoreOnce(SemaphoreHandle_t sem, uint8_t *flag);
void Determine_Start_Sution(Path_HandleTypeDef *path);
void Update_Target_Point(Path_HandleTypeDef *path, Path_State s_path_state);
void DT35_Calc(Path_HandleTypeDef *path, uint8_t mode);
void Err_Update(Path_HandleTypeDef *path);


#endif

