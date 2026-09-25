#include "math.h"
#include <string.h>
#include "path.h"
#include "pid.h"

#include "my_main.h"
#include "chassis_control.h"
#include "uper_control.h"
#include "chassis.h"


/**                 pid参数初始化                 **/
PID_T x_pid = {0};
PID_T y_pid = {0};
PID_T w_pid = {0};


BaseType_t start_capture_debug = 0;         //开始抓取信号量
BaseType_t start_suction_debug = 0;         //开始吸块信号量
BaseType_t recycle_debug = 0;               //回收机械臂
BaseType_t transmit_block_debug = 0;               //递块


void Path_Init(Path_HandleTypeDef *path)
{
    //路径状态初始化
    path->path_state = ePREPARE;
	path->current_point.area = eONE_AREA; 
    
    /**  调试使用，后面记得删  **/
    //R1路径初始化
    qt.R1_route_state[0] = eL1;
    // qt.R1_route_state[1] = eL1;
    // qt.R1_route_state[2] = eL3;

    // pid参数初始化                    /**                 谨慎使用                  **/
    pid_param_init(&x_pid,
                   PID_Position,
                   3.0f,  // 最大输出 线速度                        2.3               
                   1,     // 积分限制
                   0.2,   // 积分分离
                   0.02f, // 死区		 
                   10.0f, // 最大误差
                   3.0f,  // kp
                   0.0f,  // ki
                   1.0f); // kd
    pid_param_init(&y_pid,
                   PID_Position,
                   3.0f,  // 最大输出 线速度                        2.3       
                   1,     // 积分限制
                   0.2,   // 积分分离
                   0.02f, // 死区		
                   10.0f, // 最大误差
                   3.0f,  // kp
                   0.0f,  // ki
                   1.0f); // kd
    pid_param_init(&w_pid,
                   PID_Position,
                   6.0f,  // 最大输出 线速度        
                   0.2,     // 积分限制
                   1,   // 积分分离
                   0.0017453292f, // 死区		
                   10.0f, // 最大误差
                   3.5f,  // kp
                   0.0f,  // ki
                   5.0f); // kd

 
    pid_param_init(&DT_x,
                   PID_Position,
                   1.7f,  // 最大输出 线速度                
                   0.2,     // 积分限制
                   0.2,   // 积分分离
                   0.004f, // 死区		
                   10.0f, // 最大误差
                   2.5f,  // kp
                   0.003f,  // ki
                   0.3f); // kd
    pid_param_init(&DT_y,
                   PID_Position,
                   1.7f,  // 最大输出 线速度                
                   0.2,     // 积分限制
                   0.2,   // 积分分离
                   0.004f, // 死区		
                   10.0f, // 最大误差
                   2.5f,  // kp
                   0.003f,  // ki
                   0.3f); // kd
}


void Path_Fsm(Path_HandleTypeDef *path, Point_HandleTypeDef *current_pose)
{
    static uint32_t current_time = 0;                  //当前时间
    static uint32_t wait_capture_time = 0;                     //抓取武器之后等待时间
    static uint32_t wait_sucker_time = 0;                         //等待吸盘时间
    static uint32_t wait_transmit_time = 0;                         //等待递块时间

    static uint8_t s_path_line = 0;                     //放置哪一列（途中切换）
    static uint8_t s_switch = 0;                    
    static uint8_t switch_init = 0;

    //开局同步开关状态
    if(switch_init == 0)
    {
        s_switch = path->is_switch;
        switch_init = 1;
    }

    current_time = xTaskGetTickCount();         //获取当前时间

    //更新当前坐标
    path->current_point.x = current_pose->x;
    path->current_point.y = current_pose->y;
    path->current_point.yaw = current_pose->yaw;                    //陀螺仪的角度制

    //计算目标点与当前点的差值
    path->err_point.x = path->target_point.x - path->current_point.x;
    path->err_point.y = path->target_point.y - path->current_point.y;
    path->err_point.yaw = (path->target_point.yaw - path->current_point.yaw) * ANGLE_PI;            //转化为弧度制
    path->err_point.yaw = Limit_Max_Min(path->err_point.yaw);          //角度限幅

    switch (path->path_state)
    {
        case ePREPARE:
        {
            Path_ParaInit(path, PREPARE_X, PREPARE_Y, 1);
            path->fine_tuning_flag = 1;
            path->current_point.area = eONE_AREA; 

            if(path->R1.DT_flag == 0)
            {
                DT35_XF_Calc(path, target_pose[ePREPARE].yaw);
                DT35_YR_Calc(path, target_pose[ePREPARE].yaw);

                if(DT35_Determination(path, DT35_XF, DT35_YR, 0.01f, 0.01f, 1) == 1)
                {
                    path->R1.DT_flag = 1;                      //允许停止DT35微调
                    path->Vy = 0;
                    path->Vx = 0;
                    
                    Path_Update_Status(path, eWEAPON_RACK);
                    auto_flag = 0;
                }
            }

            break;
        }
        case eWEAPON_RACK:
        {
            //防止提前进入DT35微调
            if(fabsf(DT35_XB) < 2.0f)
            {
                Path_ParaInit(path, WEAPON_PACK_X, WEAPON_PACK_Y, 1);
            }else
            {
                Path_ParaInit(path, 0.0f, WEAPON_PACK_Y, 1);
            }

            //这里在行进途中(判断误差值)给出一个标志位，用于上层准备抓取（信号量）
            GiveSemaphoreOnce(ready_captureHandle, &path->R1.ready_capture_flag);


            if(DT35_YR > 1.955f)             //1.945
            {
                path->current_point.area = eONE_AREA;
                path->fine_tuning_flag = 1;         //转入微调逻辑  
                path->Vx = 0.0f;
                path->Vy = 0.0f;
            }	
			
            //进入DT35微调
            if(path->fine_tuning_flag == 1)
            {
                //允许DT35微调
                if(path->R1.DT_flag == 0)
                {
                    DT35_XB_Calc(path, target_pose[eWEAPON_RACK].yaw);
                    DT35_YR_Calc(path, target_pose[eWEAPON_RACK].yaw);
                }else
                {
                    path->Vx = 0;
                    path->Vy = 0;
                }


                //限速
                if(fabsf(DT35_YR - path->fine_tuning_y) < 0.3f)
                {
                    if(path->Vy < -0.1f ) path->Vy = -0.1f;
                }


                if(path->R1.DT_flag == 0)               
                {
                    //判断是否到达微调目标点
                    if(DT35_Determination(path, DT35_XB, DT35_YR, 0.009f, 0.009f, 1) == 1)
                    {
                        start_capture_debug = xSemaphoreGive(start_captureHandle);        //开始抓取
                        path->R1.DT_flag = 1;                      //允许停止DT35微调
						path->Vx = 0;
						path->Vy = 0;
                        // auto_flag = 0;
                    }
                }
            }		


            //判断爪子是否抓取完成来判断是否进入对接武器姿态
            if(crawling_completed == 0)
            {
               wait_capture_time = current_time;
            }else if(crawling_completed == 1)
            {
				path->R1.DT_flag = 1;
               if(current_time - wait_capture_time > 50)
               {
                    Path_Update_Status(path, eCONNECT_WEAPON);
               }
            }
            break;
        }        
		case eCONNECT_WEAPON:
        {			
            led_state = 3;          //关闭灯带

            Path_ParaInit(path, CONNECT_WEAPON_X, CONNECT_WEAPON_Y, 1);

            GiveSemaphoreOnce(ready_connectHandle, &path->R1.ready_connect_flag);

            //转入DT35
            path->fine_tuning_flag = 1;
            
            if(path->R1.DT_flag == 0)
            {
                //这里如果pid走太慢的话可以直接赋值一个定速
                DT35_XB_Calc(path, target_pose[eCONNECT_WEAPON].yaw);
                DT35_YR_Calc(path, target_pose[eCONNECT_WEAPON].yaw);
            }
            
            if(DT35_Determination(path, DT35_XB, DT35_YR, 0.004f, 0.004f, 15) == 1)
            {
                path->current_point.area = eONE_AREA;
                path->R1.fine_tuning_weapon_flag = 1;         //底盘微调完成之后允许云台平移微调武器
                path->R1.DT_flag = 1;                      //允许停止DT35微调

                Change_speed(3.8f, 3.8f);

                auto_flag = 0;
            }
            break;
        }
        case eLEFT_AISLE:
        {
            //这个逻辑在横穿那里
			break;
        }
        case eRIGHT_AISLE:
        {
            //这个逻辑在横穿那里
            break;
        }
        case eMIDDLE_AISLE:
        {
            Path_ParaInit(path, MIDDLE_AISLE_X, MIDDLE_AISLE_Y, 1);
            path->suction_arm = left_arm;
            path->gain_arm = left_arm;
            path->R1.is_chunk = 2;

            //释放信号量，让上层机械臂到达准备系快姿态
            GiveSemaphoreOnce(ready_suctionHandle, &path->R1.ready_suction_flag);

            //允许上层存武器
            GiveSemaphoreOnce(transform_weaponHandle, &path->transform_weapon_count);
       

            //判断是否到达雷达目标点
            if(Ladar_Determination(path->current_point, path->target_point, 0.03f, 0.05f, 1)== 1)
            {
                path->current_point.area = eONE_AREA;
                path->current_point.aisle = eMIDDLE;
                path->R1.is_chunk = 2; 

                Determine_Start_Sution(path);            
            }

            //进入微调
            if(path->fine_tuning_flag == 1)
            {
                if(path->R1.DT_flag == 0)
                {
                    DT35_XB_Calc(path, target_pose[eMIDDLE_AISLE].yaw);
                    DT35_YL_Calc(path, target_pose[eMIDDLE_AISLE].yaw);

                    path->Vx = 2 * path->Vx;
                    path->Vy = 2 * path->Vy;

                    if(Left_arm.sucker_state == 1)
                    {
                        Update_Target_Point(path, eLEFT_EXIT);
                    }
                }
            }

            break;
        }
        //多个 case 可以共享同一段代码，通过连续写出 case 并省略中间的 break
        case eL1:
        case eL2:
        case eL3:
        case eL4:
        {
            uint8_t is_chunk_buff;
            if(center_finish_flag == 0)
            {
                path->suction_arm = left_arm;
                path->gain_arm = right_arm;
            }

            if(path->path_state == eL1) {Path_ParaInit(path, L1_X, L1_Y, 1); is_chunk_buff = 4;}
            else if(path->path_state == eL2) {Path_ParaInit(path, L2_X, L2_Y, 1); is_chunk_buff = 2;}
            else if(path->path_state == eL3) {Path_ParaInit(path, L3_X, L3_Y, 1); is_chunk_buff = 4;}
            else    {Path_ParaInit(path, L4_X, L4_Y, 1); is_chunk_buff = 2;}

            


            //到达一定距离之后使用速度限幅
            //第一个块不限幅
            if(path->path_state != eL1)
            {
                if(fabsf(path->err_point.x) < 1.0f)
                {
                    Speed_para(path, 0.5f, 1.0f, 1.0f, 1.0f);
                }
            }else if(path->path_state == eL1)
            {
                if(fabsf(path->err_point.x) < 1.0f)
                {
                    Speed_para(path, 0.5f, 1.0f, 1.0f, 1.0f);
                }
            }

            //如果距离第二个目标点还有一定距离的时候触发机械臂存块
            if(fabsf(path->err_point.x) < 0.5)
            {  
                //只有在取第二个块的时候才使用另一侧机械臂存块
                if(block_num == 2 && center_finish_flag == 1)
                {
                    //这个判断是为了防止在走到第三个目标点的时候重复触发
                    if(path->path_state == qt.R1_route_state[block_num - 1])
                    {
                        GiveSemaphoreOnce(Take_blockHandle, &path->R1.take_block_flag);
                    }
                }
            }

            //判断是否到达雷达目标点
            if(Ladar_Determination(path->current_point, path->target_point, 0.03f, 0.05f, 3) == 1 && path->is_cross_flag == 0)
            {
                path->current_point.area = eTWO_AREA;
                path->current_point.aisle = eLEFT;
                path->R1.is_chunk = is_chunk_buff;     
                

                Determine_Start_Sution(path);            
            }

            //这里微调向前吸块
            if(path->fine_tuning_flag == 1)
            {
                if(path->R1.DT_flag == 0)
                {
                    DT35_XF_Calc(path, target_pose[path->path_state].yaw);
                    DT35_YR_Calc(path, target_pose[path->path_state].yaw);


                    //调试使用，记得删
                    // if(block_num <= 1)
                    // {
                        path->Vx = 1.7 * path->Vx;
                        path->Vy = 1.7 * path->Vy;
                    // }
                    

                    if(Left_arm.sucker_state == 1)
                    {
                        Update_Target_Point(path, eLEFT_EXIT);
                    }
                }
            }
            break;
        }
        case eLEFT_EXIT:
        {
            Path_ParaInit(path, LEFT_EXIT_X, LEFT_EXIT_Y, 0);

            //如果是从右边出口
            if(path->last_path_state == eRIGHT_EXIT)
            {
                Speed_para(path, 1.0f, 1.0f, 1.0f, 1.0f);
            }

            //在行走了一段时间之后传入目标yaw角
            if(path->current_point.area == eTWO_AREA)
            {
                if(fabs(path->current_point.x - target_pose[eLEFT_EXIT].x) < 0.20f || fabs(path->current_point.y - target_pose[eLEFT_EXIT].y) < 0.05f)
                {
                    path->target_point.yaw = target_pose[eLEFT_EXIT].yaw;
                    chassis_data.target_yaw = path->target_point.yaw;        
                }
            }else if(path->current_point.area == eONE_AREA)
            {
                path->target_point.yaw = target_pose[eLEFT_EXIT].yaw;
                chassis_data.target_yaw = path->target_point.yaw;                       
            }

            //如果当前已经取完了所有目标块，则直接到达三区启动区
            if(path->current_point.aisle == eLEFT)
            {
                if(qt.R1_route_state[block_num] == eLEFT_EXIT)
                {
                    Path_Update_Status(path, eZone3_ACTION_AREA);
                    path->is_cross_flag = 0;          //重置横穿标志位(这个是吸完中间入口跑到这里的话有问题)
                }
            }

            if(Ladar_Determination(path->current_point, path->target_point, 0.15f, 0.10f, 1)== 1)
            {
                path->current_point.area = eTWO_AREA;
                path->current_point.aisle = eLEFT;

                // auto_flag = 0;
                if(qt.R1_route_state[block_num] == eLEFT_EXIT)
                {
                    Path_Update_Status(path, eZone3_ACTION_AREA);
                }           
            }

            break;
        }
        case eR1:
        case eR2:
        case eR3:
        case eR4:
        {
            uint8_t is_chunk_buff;
            if(center_finish_flag == 0)
            {
                path->suction_arm = right_arm;
                path->gain_arm = left_arm;
            }

            if(path->path_state == eR1) {Path_ParaInit(path, R1_X, R1_Y, 1); is_chunk_buff = 4;}
            else if(path->path_state == eR2) {Path_ParaInit(path, R2_X, R2_Y, 1); is_chunk_buff = 6;}
            else if(path->path_state == eR3) {Path_ParaInit(path, R3_X, R3_Y, 1); is_chunk_buff = 4;}
            else    {Path_ParaInit(path, R4_X, R4_Y, 1); is_chunk_buff = 2;}


            //到达一定距离之后使用速度限幅
            if(path->path_state != eR1)
            {
                if(fabsf(path->err_point.x) < 1.0f)
                {
                    Speed_para(path, 0.5f, 1.0f, 1.0f, 1.0f);
                }
            }else if(path->path_state == eR1)
            {
                Speed_para(path, 0.0f, 0.5f, 1.0f, 1.0f);
            }


            //如果距离第二个目标点还有一定距离的时候触发机械臂存块
            if(fabsf(path->err_point.x) < 0.5)
            {  
                //只有在取第二个块的时候才使用另一侧机械臂存块
                if(block_num == 2 && center_finish_flag == 1)
                {
                    //这个判断是为了防止在走到第三个目标点的时候重复触发
                    if(path->path_state == qt.R1_route_state[block_num - 1])
                    {
                        GiveSemaphoreOnce(Take_blockHandle, &path->R1.take_block_flag);
                    }
                }
            }


            //判断是否到达雷达目标点（在横穿逻辑判断完成之后在判断）
            if(Ladar_Determination(path->current_point, path->target_point, 0.03f, 0.05f, 3) == 1 && path->is_cross_flag == 0)
            {
                path->current_point.area = eTWO_AREA;
                path->current_point.aisle = eRIGHT;
                path->R1.is_chunk = is_chunk_buff;     
                
                Determine_Start_Sution(path);
            }

            //这里微调向前吸块
            if(path->fine_tuning_flag == 1)
            {
                if(path->R1.DT_flag == 0)
                {
                    //这里后面的话重新标了DT35再改这个判断
                    if(path->path_state == eR1)
                    {
                        DT35_XF_Calc(path, target_pose[path->path_state].yaw);
                        DT35_YR_Calc(path, target_pose[path->path_state].yaw);

  
                        path->Vx = 1.2 * path->Vx;
                        path->Vy = 1.2 * path->Vy;

                        if(Right_arm.sucker_state == 1)
                        {
                            Update_Target_Point(path, eRIGHT_EXIT);
                        }
                    }else
                    {
                        DT35_XB_Calc(path, target_pose[path->path_state].yaw);
                        DT35_YR_Calc(path, target_pose[path->path_state].yaw);


                        if(block_num <= 1)
                        {
                            path->Vx = 1.7 * path->Vx;
                            path->Vy = 1.7 * path->Vy;
                        }

                        if(Right_arm.sucker_state == 1)
                        {
                            Update_Target_Point(path, eRIGHT_EXIT);
                        }
                    }
                }
            }

            break;
        }
        case eRIGHT_EXIT:
        {
            Path_ParaInit(path, RIGHT_EXIT_X, RIGHT_EXIT_Y, 0);
            path->suction_arm = right_arm;
            path->gain_arm = left_arm;

            //在行走了一段时间之后传入目标yaw角
            if(fabs(path->current_point.x - path->target_point.x) < 0.05f || fabs(path->current_point.y - path->target_point.y) < 0.05f)
            {
                path->target_point.yaw = target_pose[eRIGHT_EXIT].yaw;
                chassis_data.target_yaw = path->target_point.yaw;
            }

            //判断是否到达目标点
            if(Ladar_Determination(path->current_point, path->target_point, 0.15f, 0.15f, 1)== 1)
            {
                path->current_point.area = eTWO_AREA;
                path->current_point.aisle = eRIGHT;
                // path->fine_tuning_flag = 1;         //转入微调逻辑      

                qt.R1_route_state[block_num] = eLEFT_EXIT;
                Path_Update_Status(path, qt.R1_route_state[block_num]);
            }

            break;
        }
        case eMIDDLE_EXIT:
        {
            Path_ParaInit(path, MIDDLE_EXIT_X, MIDDLE_EXIT_Y, 1);
            path->suction_arm = left_arm;
            path->gain_arm = left_arm;

            //速度限幅
            Speed_para(path, 0.5f, 0.5f, 1.0f, 1.0f);

            //判断是否到达雷达目标点
            if(Ladar_Determination(path->current_point, path->target_point, 0.03f, 0.05f, 3) == 1)
            {
                path->current_point.area = eTWO_AREA;
                path->current_point.aisle = eMIDDLE;
                path->R1.is_chunk = 4; 

                Determine_Start_Sution(path);
            }

            //进入微调
            if(path->fine_tuning_flag == 1)
            {
                DT35_XB_Calc(path, target_pose[eMIDDLE_EXIT].yaw);
                DT35_YR_Calc(path, target_pose[eMIDDLE_EXIT].yaw);

                path->Vx = 2.0f * path->Vx;
                path->Vy = 2.0f * path->Vy;

                if(Left_arm.sucker_state == 1)
                {
                    Update_Target_Point(path, eLEFT_EXIT);
                }
            }

            break;        
        }
        case eZone3_ACTION_AREA:
        {
            if(uper_FSM.arm_storage_flag == 0)
            {
                path->suction_arm = left_arm;
                path->gain_arm = left_arm;
            }
            

            //这个后续的话需要判断一下x方向的误差再进这个逻辑
            Change_speed(3.4f, 3.4f);
            // Change_speed(1.0f, 1.0f);

            Path_ParaInit(path, ZONE_3_AREA_X, ZONE_3_AREA_Y, 1);
            
            //在上坡的时候直接开始往出吸块
            if(fabsf(path->err_point.x) < 1.0f && fabsf(path->err_point.x) > 0.3f)          //0.5
            {
                
                Speed_para(path, 0.8f, 1.0f, 1.0f, 1.0f); 
                path->current_point.area = eTHREE_AREA;
                //让这个信号量只释放一次
                GiveSemaphoreOnce(Take_blockHandle, &path->R1.take_block_flag);
            }


            //判断是否到达雷达目标点
            if(Ladar_Determination(path->current_point, path->target_point, 0.10f, 0.10f, 1) == 1)
            {
                path->current_point.area = eTHREE_AREA;
                path->current_point.aisle = eLEFT;


                path->fine_tuning_flag = 1;         //转入微调逻辑      


                //判断是否需要递块
                if(radar.transmit_block_flag == 0)
                {
                    if(path->line == 1)
                    {
                        Path_Update_Status(path, eSUDOKO_1);
                    }else
                    {
                        Path_Update_Status(path, eSUDOKO_3);
                    }
                }else
                {
                    Path_Update_Status(path, eTRANSMIT_BLOCK_POINT);
                }

            }

            break;
        }
        case eSUDOKO_1:
        {
            DT_x.MaxOutput = 2.5f;
            DT_y.MaxOutput = 2.5f;


            //在过了斜坡之后才允许切换（防止撞到斜坡）
            if(path->current_point.y < -1.55f)
            {
                //途中切换放置左右列
                if(s_path_line != path->line)
                {
                    if(path->line == 1)
                    {
                        Path_Update_Status(path, eSUDOKO_1);
                        path->last_path_state = eZone3_ACTION_AREA;
                    }else
                    {
                        Path_Update_Status(path, eSUDOKO_3);
                        path->last_path_state = eSUDOKO_1;
                        s_path_line = path->line;
                        break;
                    }
                }
                s_path_line = path->line;
            }


            //放块完成之后将这个清空，防止递块逻辑冲突
            if(place_block_flag == 1)
            {
                path->gain_arm = 0;
            }

            //传入目标点
            if(path->last_path_state == eZone3_ACTION_AREA || path->last_path_state == eTRANSMIT_BLOCK_POINT)
            {
                if(path->current_point.y < -1.55f) 
                {
                    //y方向已经足够走过斜坡
                    Path_ParaInit(path, SUDOKO_1_X, SUDOKO_1_Y, 1);

                    //速度限幅
                    Speed_para(path, 0.1f, 0.0f, 1.0f, 0.8f);
                }else      //还未走到三区中间
                {
                    path->target_point.x = target_pose[eZone3_ACTION_AREA].x;
                    path->target_point.y = target_pose[eSUDOKO_1].y;
                    path->target_point.yaw = target_pose[eSUDOKO_1].yaw;

                }
            }else if(path->last_path_state == eSUDOKO_2 || path->last_path_state == eSUDOKO_3)
            {

                if(path->current_point.y >= MIN_Y_DISTANCE)
                {
                    path->target_point.x = target_pose[eSUDOKO_2].x;
                    path->target_point.y = MIN_Y_DISTANCE;
                    path->target_point.yaw = target_pose[eSUDOKO_1].yaw;                
                    
                    Err_Update(path);
                    if(fabsf(path->err_point.y) < 0.1f && fabsf(path->err_point.x) < 0.05f)
                    {
                        path->target_point.y = target_pose[eSUDOKO_1].y;
                    }
                }else
                {
                    if(fabsf(path->err_point.x) < 0.1f)
                    {
                        Path_ParaInit(path, SUDOKO_1_X, SUDOKO_1_Y, 1);
                        //y方向限速
                        Speed_para(path, 0.1f, 0.0f, 1.0f, 0.8f);
                    }else
                    {
                        path->target_point.x = target_pose[eSUDOKO_1].x;
                        path->target_point.y = MIN_Y_DISTANCE;
                        path->target_point.yaw = target_pose[eSUDOKO_1].yaw;
    
                        //x方向限速
                        Err_Update(path);
                        Speed_para(path, 1.0f, 1.0f, 0.5f, 0.0f);
                    }
                }
            }


            //判断是否到达雷达目标点
            if(Ladar_Determination(path->current_point, target_pose[eSUDOKO_1], 0.03f, 0.03f, 5)== 1)
            {
                path->current_point.area = eTHREE_AREA;
                path->current_point.aisle = eRIGHT;

                path->fine_tuning_flag = 1;         //转入微调逻辑      
				
				Change_speed(1.5f, 1.5f);
                
                if(path->R1.close_sucker_flag == 0)
                {
                    //释放信号量 关闭吸盘
                    transmit_block_debug = xSemaphoreGive(close_suckerHandle);      
                    wait_sucker_time = xTaskGetTickCount();
                    path->R1.close_sucker_flag = 1;
                }
            }

            if(current_time - wait_sucker_time > 500 && wait_sucker_time != 0)
            {
                Path_Update_Status(path, eFT_LIFT_Point);   
                wait_sucker_time = 0;
            }
            break;
        }
        case eSUDOKO_2:
        {
            DT_x.MaxOutput = 2.5f;
            DT_y.MaxOutput = 2.5f;



            //传入目标点
            if(path->last_path_state == eZone3_ACTION_AREA)
            {
                if(path->current_point.y < -1.5f) 
                {
                    //y方向已经足够走过斜坡
                    Path_ParaInit(path, SUDOKO_2_X, SUDOKO_2_Y, 1);

                    //速度限幅
                    Speed_para(path, 1.0f, 1.0f, 1.0f, 0.6f);
                }else      //还未走到三区中间
                {
                    path->target_point.x = target_pose[eZone3_ACTION_AREA].x;
                    path->target_point.y = target_pose[eSUDOKO_2].y;
                    path->target_point.yaw = target_pose[eSUDOKO_2].yaw;
                }
            }else if(path->last_path_state == eSUDOKO_1 || path->last_path_state == eSUDOKO_3 || path->last_path_state == eSUDOKO_2)
            {
                //如果遇到对抗，先走到目标x，到达x之后再向y方向移动
                Speed_para(path, 1.5f, 1.0f, 1.5f, 0.6f);

                if(fabsf(path->err_point.x) < 0.1f)
                {
                    Path_ParaInit(path, SUDOKO_2_X, SUDOKO_2_Y, 1);
                }else
                {
                    path->target_point.x = target_pose[eSUDOKO_2].x;
                    path->target_point.y = path->current_point.y;
                    path->target_point.yaw = target_pose[eSUDOKO_1].yaw;
                }
            }


            //判断是否到达雷达目标点
            if(Ladar_Determination(path->current_point, path->target_point, 0.03f, 0.05f, 1)== 1)
            {
                path->current_point.area = eTHREE_AREA;
                path->current_point.aisle = eRIGHT;

                path->fine_tuning_flag = 1;         //转入微调逻辑      
				
				Change_speed(1.5f, 1.5f);
                
                if(path->R1.close_sucker_flag == 0)
                {
                    //释放信号量 关闭吸盘
                    transmit_block_debug = xSemaphoreGive(close_suckerHandle);      
                    wait_sucker_time = xTaskGetTickCount();
                    path->R1.close_sucker_flag = 1;
                }

                if(current_time - wait_sucker_time > 500)
                {
                    Path_Update_Status(path, eMLIFT_POINT);   
                }      

                auto_flag = 0;
            }

            break;
        }

        case eSUDOKO_3:
        {
            DT_x.MaxOutput = 2.5f;
            DT_y.MaxOutput = 2.5f;

            Change_speed(3.2f, 3.2f);
            // Change_speed(1.0f, 1.0f);


            //在过了斜坡之后才允许切换（防止撞到斜坡）
            if(path->current_point.y < -1.55f)
            {
                //途中切换放置左右列
                if(s_path_line != path->line)
                {
                    if(path->line == 1)
                    {
                        Path_Update_Status(path, eSUDOKO_1);
                        path->last_path_state = eSUDOKO_3;
                        s_path_line = path->line;
                        break;
                    }else
                    {
                        Path_Update_Status(path, eSUDOKO_3);
                        path->last_path_state = eZone3_ACTION_AREA;
                    }
                }
                s_path_line = path->line;
            }

            //放块完成之后将这个清空，防止递块逻辑冲突
            if(place_block_flag == 1)
            {
                path->gain_arm = 0;
            }


            //传入目标点
            if(path->last_path_state == eZone3_ACTION_AREA || path->last_path_state == eTRANSMIT_BLOCK_POINT)
            {

                if(path->current_point.y < -1.38f) 
                {
                    //y方向已经足够走过斜坡
                    Path_ParaInit(path, SUDOKO_3_X, SUDOKO_3_Y, 1);

                    //速度限幅(这里x方向不限幅，让车体尽快到达目标x方向)
                    // if(path->current_point.x < 10.0f)
                    // {
                        Speed_para(path, 0.15f, 0.1f, 0.05f, 0.0f);          //0.4
                    // }
                }else      //还未走到三区中间  
                {
                    path->target_point.x = target_pose[eZone3_ACTION_AREA].x;
                    path->target_point.y = target_pose[eSUDOKO_3].y;
                    path->target_point.yaw = target_pose[eSUDOKO_3].yaw;
                }
            }else if(path->last_path_state == eSUDOKO_1 || path->last_path_state == eSUDOKO_2)
            {

                if(path->current_point.y >= MIN_Y_DISTANCE)
                {
                    path->target_point.x = target_pose[eSUDOKO_3].x;
                    path->target_point.y = MIN_Y_DISTANCE;
                    path->target_point.yaw = target_pose[eSUDOKO_3].yaw;      
                    
                    if(fabsf(path->err_point.y) < 0.1f)
                    {
                        path->target_point.y = target_pose[eSUDOKO_3].y;
                    }

                    Err_Update(path);
                    Speed_para(path, 0.48f, 0.1f, 0.1f, 0.0f); 
                }else
                {
                    if(fabsf(path->err_point.x) < 0.1f)
                    {
                        Path_ParaInit(path, SUDOKO_3_X, SUDOKO_3_Y, 1);
                        //y方向限速
                        // Speed_para(path, 0.1f, 0.0f, 1.0f, 0.0f);
                    }else
                    {
                        path->target_point.x = target_pose[eSUDOKO_3].x;
                        path->target_point.y = MIN_Y_DISTANCE;
                        path->target_point.yaw = target_pose[eSUDOKO_3].yaw;
    
                        Err_Update(path);
                        //x方向限速
                        // Speed_para(path, 1.0f, 1.0f, 0.5f, 0.0f);
                    }
                }
            }
            
            //判断是否到达雷达目标点(这里这个目标点需要写eSUDOKO3, 因为如果直接写path->target_point的话，可能会因为之前的目标点还没有完全到位导致误判)
            if(Ladar_Determination(path->current_point, target_pose[eSUDOKO_3], 0.03f, 0.03f, 5)== 1)
            {
                path->current_point.area = eTHREE_AREA;
                path->current_point.aisle = eRIGHT;

				
                path->fine_tuning_flag = 1;         //转入微调逻辑 
                path->Vx = 0.0f;
                path->Vy = 0.0f;   
				Change_speed(2.5f, 2.5f);
                // Change_speed(1.0f, 1.0f);
				
                if(path->R1.close_sucker_flag == 0)
                {
                    //释放信号量 关闭吸盘
                    xSemaphoreGive(close_suckerHandle);      
                    wait_sucker_time = xTaskGetTickCount();
                    path->R1.close_sucker_flag = 1;
                }
            }

            if(current_time - wait_sucker_time > 600 && wait_sucker_time != 0)
            {
                Path_Update_Status(path, eBT_LIFT_Point);   
                wait_sucker_time = 0;
            }

            break;
        }
        case eBLIFT_POINT:          //靠近启动区这一侧的抬升点位
        {
            //释放信号量
            xSemaphoreGive(Chassis_Low_SpeedHandle);            //让地盘改为低速
            

            Path_ParaInit(path, B_LIFT_X, B_LIFT_Y, 1);

            //让上层回收机械臂
            GiveSemaphoreOnce(arm_initHandle, &path->R1.confront_flag);

            if(s_switch != path->is_switch)
            {
                Path_Update_Status(path, eFT_LIFT_Point);
                s_switch = path->is_switch;
                break;
            }


            //判断是否到达雷达目标点
            if(Ladar_Determination(path->current_point, path->target_point, 0.03f, 0.03f, 10) == 1)
            {
                //紫灯
                led_state = 4;

                path->current_point.area = eTHREE_AREA;
                path->current_point.aisle = eRIGHT;


                path->fine_tuning_flag = 1;         //转入微调逻辑    
            }
            

           if(path->fine_tuning_flag == 1)
           {
               if(path->R1.DT_flag == 0)
               {
                   DT35_XF_Calc(path, target_pose[eBLIFT_POINT].yaw);
                   DT35_YL_Calc(path, target_pose[eBLIFT_POINT].yaw);

                   if(DT35_Determination(path, DT35_XF, DT35_YL, 0.010f, 0.010f, 5) == 1)
                   {
                       path->R1.DT_flag = 1;
                       auto_finish_flag = 1;            //自动阶段已经完成
                       auto_flag = 0;
                   }
               }
           }else
           {
                //灭灯（防止误识别）
                led_state = 3;
           }

            break;
        }

        case eFLIFT_POINT:
        {
            //释放信号量
            xSemaphoreGive(Chassis_Low_SpeedHandle);

            Path_ParaInit(path, F_LIFT_X, F_LIFT_Y, 1);

            //让上层回收机械臂
            GiveSemaphoreOnce(arm_initHandle, &path->R1.confront_flag);

            if(s_switch != path->is_switch)
            {
                Path_Update_Status(path, eBT_LIFT_Point);
                s_switch = path->is_switch;
                break;
            }

            //判断是否到达雷达目标点
            if(Ladar_Determination(path->current_point, path->target_point, 0.03f, 0.05f, 1)== 1)
            {
                //紫色
                led_state = 4;
                path->current_point.area = eTHREE_AREA;
                path->current_point.aisle = eRIGHT;


                path->fine_tuning_flag = 1;         //转入微调逻辑     
            }
            

            if(path->fine_tuning_flag == 1)
            {
                if(path->R1.DT_flag == 0)
                {
                    DT35_XF_Calc(path, target_pose[eFLIFT_POINT].yaw);
                    DT35_YR_Calc(path, target_pose[eFLIFT_POINT].yaw);
    
                    if(DT35_Determination(path, DT35_XF, DT35_YR, 0.01f, 0.01f, 5) == 1)
                    {
                        path->R1.DT_flag = 1;
                        auto_finish_flag = 1;            //自动阶段已经完成
                        auto_flag = 0;
                    }
                }
            }else
            {
                //灭灯（防止误识别）
                led_state = 3;
            }

            break;
        }
        case eFT_LIFT_Point:
        {
            Path_ParaInit(path, FT_LIFT_X, FT_LIFT_Y, 1);

            //绿灯
            led_state = 1;

            if(s_switch != path->is_switch)
            {
                Path_Update_Status(path, eBLIFT_POINT);
                s_switch = path->is_switch;
                break;
            }

            //让上层回收机械臂
            GiveSemaphoreOnce(arm_initHandle, &path->R1.confront_flag);

            //判断是否到达雷达目标点
            if(Ladar_Determination(path->current_point, path->target_point, 0.03f, 0.05f, 1)== 1)
            {
                path->current_point.area = eTHREE_AREA;
                path->current_point.aisle = eRIGHT;


                path->fine_tuning_flag = 1;         //转入微调逻辑     
                path->Vx = 0.0f;
            }    


            if(path->fine_tuning_flag == 1)
            {
                if(path->R1.DT_flag == 0)
                {
					DT35_XF_Calc(path, target_pose[eFT_LIFT_Point].yaw);
					DT35_YR_Calc(path, target_pose[eFT_LIFT_Point].yaw);
	
					if(DT35_Determination(path, DT35_XF, DT35_YR, 0.006f, 0.006f, 5) == 1)
					{
						path->R1.DT_flag = 1;
						auto_finish_flag = 1;            //自动阶段已经完成
						auto_flag = 0;
					}
                }
            }

            break;
        }
        case eBT_LIFT_Point:
        {
            Path_ParaInit(path, BT_LIFT_X, BT_LIFT_Y, 1);


            //绿灯
            led_state = 1;

            if(s_switch != path->is_switch)
            {
                Path_Update_Status(path, eFLIFT_POINT);
                s_switch = path->is_switch;
                break;
            }

            //让上层回收机械臂
            GiveSemaphoreOnce(arm_initHandle, &path->R1.confront_flag);

            //判断是否到达雷达目标点
            if(Ladar_Determination(path->current_point, path->target_point, 0.03f, 0.05f, 1)== 1)
            {
                path->current_point.area = eTHREE_AREA;
                path->current_point.aisle = eRIGHT;


                path->fine_tuning_flag = 1;         //转入微调逻辑     
            }    


            if(path->fine_tuning_flag == 1)
            {
                if(path->R1.DT_flag == 0)
                {
                    DT35_XF_Calc(path, target_pose[eBLIFT_POINT].yaw);
                    DT35_YL_Calc(path, target_pose[eBLIFT_POINT].yaw);
    
                    if(DT35_Determination(path, DT35_XF, DT35_YL, 0.005f, 0.005f, 10) == 1)
                    {
                        path->R1.DT_flag = 1;
                        auto_finish_flag = 1;            //自动阶段已经完成
                        path->Vx = 0;
                        path->Vy = 0;
                        auto_flag = 0;
                    }
                }
            }

            break;
        }
        case eTRANSMIT_BLOCK_POINT:
        {
            //清空这个，放置递块冲突
            path->gain_arm = 0;

            Path_ParaInit(path, TRANSMIT_BLOCK_X, TRANSMIT_BLOCK_Y, 1);

            Speed_para(path, 1.0f, 1.0f, 1.2f, 1.0f);

            GiveSemaphoreOnce(transmit_blockHandle, &path->R1.transmit_block_flag);

            if(Ladar_Determination(path->current_point, path->target_point, 0.05f, 0.05f, 1)== 1)
            {
                path->current_point.area = eTHREE_AREA;
                path->current_point.aisle = eRIGHT;


                path->fine_tuning_flag = 1;         //转入微调逻辑     
            }  


            if(path->fine_tuning_flag == 1)
            {
                if(path->R1.DT_flag == 0)
                {
                    DT35_XB_Calc(path, target_pose[eTRANSMIT_BLOCK_POINT].yaw);
                    DT35_YL_Calc(path, target_pose[eTRANSMIT_BLOCK_POINT].yaw);

                    if(DT35_Determination(path, DT35_XB, DT35_YL, 0.006f, 0.01f, 10) == 1)
                    {
                        path->R1.DT_flag = 1;
                        GiveSemaphoreOnce(Close_R_SuckerHandle, &path->R1.close_R_sucker_flag);
                        wait_transmit_time = xTaskGetTickCount();
                    }

                }
            }

            if(current_time - wait_transmit_time > 400 && wait_transmit_time != 0)
            {
                if(path->line == 1)
                {
                    Path_Update_Status(path, eSUDOKO_1);
                }else
                {
                    Path_Update_Status(path, eSUDOKO_3);
                }
                wait_transmit_time = 0;
            }
            break;
        }
        default:
        {

            break;
        }

    }

    //根据当前状态更新索引
    for(uint8_t i = 0; i < path->point_num; i++)
    {
        if(path->route[i] == path->path_state)
        {
            path->step = i;
            break;
        }
    }  

    //判断是否需要横穿梅林
    if((path->current_point.area == eONE_AREA && path->target_point.area == eTWO_AREA) || (path->current_point.area == eTWO_AREA && path->current_point.aisle != path->target_point.aisle && (path->current_point.is_outside != 1 && path->target_point.is_outside != 1)))              //判断目标点和当前点的距离，是不是会直接穿过梅林
    {
        //fabsf(err_point->x) > 0.4f
        path->is_cross_flag = 1;         //需要横穿
    }

    //不需要横穿
    if(path->is_cross_flag == 0)
    {
        //雷达拉
        if(path->fine_tuning_flag == 0)		
        { 
            //是否走斜线
            if(path->R1.percentage != 0)
            {
                path->Vx = pid_calc_by_error(&x_pid, path->err_point.x);
                path->Vy = pid_calc_by_error(&y_pid, path->err_point.y);
                
                if(path->Vy != 0)
                {
                    if(path->Vx * path->Vy > 0)
                    {
                        path->Vx = path->R1.percentage * path->Vy;
                    }else
                    {
                        path->Vx = -path->R1.percentage * path->Vy;
                    }
                }
                path->Vw = pid_calc_by_error(&w_pid, path->err_point.yaw);
            }else
            {
                path->Vx = pid_calc_by_error(&x_pid, path->err_point.x);
                path->Vy = pid_calc_by_error(&y_pid, path->err_point.y);
                path->Vw = pid_calc_by_error(&w_pid, path->err_point.yaw);
            }


            //速度限制
            if(path->R1.x_speed_limit != 0)
            {
                Speed_limit(&path->Vx, path->R1.x_speed_limit);
            }
            if(path->R1.y_speed_limit != 0)
            {
                Speed_limit(&path->Vy, path->R1.y_speed_limit);
            }
        }else
        { 
            path->Vw = pid_calc_by_error(&w_pid, path->err_point.yaw); 
        }
    }else
    {
        Across_Process(path);
    }
}
