#include "path_lib.h"
#include "chassis_control.h"
#include "string.h"
#include "my_main.h"


//DT35pid
PID_T DT_x = {0};
PID_T DT_y = {0}; 


/*  传入目标点，如果目标点右目标yaw角的时候，需要在对应的case里面更新chassis_data的target_yaw，不然自动且手动的话车体会自旋  */
Point_HandleTypeDef target_pose[] = 
{
    /**                 一区                  **/
    [ePREPARE] = {0, 0, 180, eONE_AREA, eLEFT},                                        //准备
    [eWEAPON_RACK] = {0.694f, -5.0f, 180, eONE_AREA, eMIDDLE},                                    //武器架  
    [eCONNECT_WEAPON] = {0.75f, -2.54f, 180, eONE_AREA, eMIDDLE},                                 //对接武器       这个点位的目标yaw角在后续状态的时候切换
    [eLEFT_AISLE] = {2.239f, -0.188f, 180, eONE_AREA, eLEFT, 1},                                     //左侧通道入口
    [eRIGHT_AISLE] = {2.204f, -5.007f, 180, eONE_AREA, eRIGHT, 1},                                    //右侧通道入口
    [eMIDDLE_AISLE] = {2.192f, -2.720f, 270, eONE_AREA, eMIDDLE, 1},                                   //第一排中间块点位

    /**                 二区                  **/
    [eL1] = {3.359f, -0.15f, 180, eTWO_AREA, eLEFT},                                             //左侧第一个梅林
    [eL2] = {4.558f, -0.138f, 180, eTWO_AREA, eLEFT},                                             //左侧第二个梅林
    [eL3] = {5.741f, -0.13f, 180, eTWO_AREA, eLEFT},                                             //左侧第三个梅林
    [eL4] = {6.936f, -0.09f, 180, eTWO_AREA, eLEFT},                                             //左侧第四个梅林
    [eLEFT_EXIT] = {8.420f, -0.20f, 180, eTWO_AREA, eLEFT, 1},                                    //左侧通道出口	

    [eR1] = {3.408f, -5.070f, 180, eTWO_AREA, eRIGHT},                                             //右侧第一个梅林
    [eR2] = {4.566f, -5.05f, 180, eTWO_AREA, eRIGHT},                                             //右侧第二个梅林
    [eR3] = {5.784f, -5.09f, 180, eTWO_AREA, eRIGHT},                                             //右侧第三个梅林
    [eR4] = {6.9764f, -5.13f, 180, eTWO_AREA, eRIGHT},                                             //右侧第四个梅林
    [eRIGHT_EXIT] = {8.420f, -4.977f, 180, eTWO_AREA, eRIGHT, 1},                                     //右侧通道出口

    [eMIDDLE_EXIT] = {8.329f, -2.423f, 90, eTWO_AREA, eMIDDLE, 1},                                    //中间过道出口

    /**                 三区                  **/
    [eZone3_ACTION_AREA] = {10.95f, -0.22f, 180, eTHREE_AREA, eLEFT},                              //三区启动区
    [eSUDOKO_1] = {10.82f, -4.56f, 180, eTHREE_AREA, eRIGHT},                                       //九宫格1
    [eSUDOKO_2] = {10.31f, -4.507f, 180, eTHREE_AREA, eRIGHT},                                       //九宫格2
    [eSUDOKO_3] = {9.77f, -4.561f, 180, eTHREE_AREA, eRIGHT},                                       //九宫格3
    [eBLIFT_POINT] = {9.87f, -4.75f, 180, eTHREE_AREA, eRIGHT},                                                      //抬升R2点位
    [eFLIFT_POINT] = {10.85f, -4.60f, 0, eTHREE_AREA, eRIGHT},
    [eBT_LIFT_Point] = {9.78f, -4.78f, 180, eTHREE_AREA, eRIGHT},
    [eFT_LIFT_Point] = {10.87f, -4.83f, 0, eTHREE_AREA, eRIGHT},
    [eTRANSMIT_BLOCK_POINT] = {10.87f, -1.87f, 180, eTHREE_AREA, eRIGHT},
};




//左侧路径
Path_State left_state[] = 
{
    ePREPARE,
    eWEAPON_RACK,
	eCONNECT_WEAPON,
    eLEFT_AISLE,
    eL1,
    eL2,
    eL3,
    eL4,
    eLEFT_EXIT,
    eZone3_ACTION_AREA,         //三区启动区
    eSUDOKO_1,                  //九宫格点位  场地外侧为1  场地内侧为3
    eBLIFT_POINT,                        //改动这里的话记得改那个路径长度
};

//右侧路径
Path_State right_state[] = 
{
    ePREPARE,
    eWEAPON_RACK,
	eCONNECT_WEAPON,
    eMIDDLE_AISLE,
    eRIGHT_AISLE,
    eR1,
    eR2,
    eR3,
    eR4,
    eRIGHT_EXIT,
    eLEFT_EXIT,
    eZone3_ACTION_AREA,         //三区启动区
    eSUDOKO_1,                  //九宫格点位  场地外侧为1  场地内侧为3
    eBLIFT_POINT,
};

//QT路径
Path_State QT_State[3] = {0};



//这个函数里面的啥时候给yaw角的判断有问题  要改动
void Across_Process(Path_HandleTypeDef *path)
{
    switch (path->cross_state)
    {
        case CROSS_NONE:        
        {
            //判断一下此时需要跑向哪个点
            if(path->current_point.area == eONE_AREA)       //如果当前在1区
            {
                if(path->target_point.aisle == eRIGHT)        //判断目标点位在左侧还是右侧
                {
                    if(fabs(path->current_point.x - target_pose[eRIGHT_EXIT].x) < fabs(path->current_point.x - target_pose[eRIGHT_AISLE].x))
                    {
                        path->cross_state = CROSS_TO_RIGHT_EXIT;
                    }else
                    {
                        path->cross_state = CROSS_TO_RIGHT_AISLE;
                    }

                }else if(path->target_point.aisle == eLEFT)
                {
                    if(fabs(path->current_point.x - target_pose[eLEFT_EXIT].x) < fabs(path->current_point.x - target_pose[eLEFT_AISLE].x))
                    {
                        path->cross_state = CROSS_TO_LEFT_EXIT;
                    }else
                    {
                        path->cross_state = CROSS_TO_LEFT_AISLE;
                    }
                }else if(path->target_point.aisle == eMIDDLE)          //如果此时处于中间的第一个快，则直接走到左侧入口
                {
                    path->cross_state = CROSS_TO_LEFT_AISLE;
                }
            }else if(path->current_point.area == eTWO_AREA)         //如果当前在二区
            {
                if(path->current_point.aisle == eRIGHT)        //判断一下当前点位在左侧还是右侧
                {
                    if(fabs(path->current_point.x - target_pose[eRIGHT_EXIT].x) < fabs(path->current_point.x - target_pose[eRIGHT_AISLE].x))
                    {
                        path->cross_state = CROSS_TO_RIGHT_EXIT;
                    }else
                    {
                        path->cross_state = CROSS_TO_RIGHT_AISLE;
                    }

                }else if(path->current_point.aisle == eLEFT)
                {
                    if(fabs(path->current_point.x - target_pose[eLEFT_EXIT].x) < fabs(path->current_point.x - target_pose[eLEFT_AISLE].x))
                    {
                        path->cross_state = CROSS_TO_LEFT_EXIT;
                    }else
                    {
                        path->cross_state = CROSS_TO_LEFT_AISLE;
                    }
                }
            }
            break;
        }
        case CROSS_TO_LEFT_AISLE:
        {
            //如果上一个点位是对接点位则走斜线
            if(path->last_path_state == eCONNECT_WEAPON)
            {
                if(fabsf(path->current_point.x - target_pose[eLEFT_AISLE].x) > 0.4f)
                {
                    path->R1.percentage = fabsf((target_pose[eLEFT_AISLE].x - target_pose[path->last_path_state].x) / (target_pose[eLEFT_AISLE].y - target_pose[path->last_path_state].y));
                }else
                {
                    path->R1.percentage = 0.0f;
                }
            }

            if(path->current_point.area == eTWO_AREA)       //如果当前在二区
            {
                path->Vx = pid_calc(&x_pid, target_pose[eLEFT_AISLE].x, path->current_point.x);
                path->Vy = pid_calc(&y_pid, target_pose[eLEFT_AISLE].y, path->current_point.y);

                if(fabs(path->current_point.x - target_pose[eLEFT_AISLE].x) < 0.10f && fabs(path->current_point.y - target_pose[eLEFT_AISLE].y) < 0.10f) 
                {
                    path->Vw = pid_calc_by_error(&w_pid, path->err_point.yaw); 
                    if(fabs(path->err_point.yaw) < 45.0f)
                    {
                        if(fabs(path->err_point.y) < 0.20f || fabs(path->err_point.x) < 0.20f)
                        {
                            path->cross_state = CROSS_TO_TARGET;
                        }else
                        {
                            path->cross_state = CROSS_TO_RIGHT_AISLE;
                        }
                    }
                }else
                {
                    if(fabs(path->err_point.yaw) < 1)       //如果已经转到目标角度，就保持目标角度
                    {
                        path->Vw = pid_calc_by_error(&w_pid, path->err_point.yaw);
                    }else
                    {
                        //这个有缺陷，后续应该修改成上一个状态的yaw角 (这里传入必须为弧度制，要不然单位不统一会导致转起来幅度很大)
                        path->Vw = pid_calc(&w_pid, target_pose[path->route[path->step]].yaw * ANGLE_PI, path->current_point.yaw * ANGLE_PI);
                    }
                }
            }else if(path->current_point.area == eONE_AREA)         //如果当前在一区
            {   
                if(block_num == 0)
                {
                    //释放信号量，让上层机械臂到达准备吸快姿态
                    if(path->R1.ready_suction_flag == 0)
                    {
                        xSemaphoreGive(ready_suctionHandle);
                        path->R1.ready_suction_flag = 1;
                    }
    
                    if(path->transform_weapon_count == 0)
                    {
                        xSemaphoreGive(transform_weaponHandle);             //允许上层存武器
                        path->transform_weapon_count ++;
                    }    
                } 

                
                path->Vx = pid_calc(&x_pid, target_pose[eLEFT_AISLE].x, path->current_point.x);
                path->Vy = pid_calc(&y_pid, target_pose[eLEFT_AISLE].y, path->current_point.y);
                path->Vw = pid_calc_by_error(&w_pid, path->err_point.yaw); 

                if(fabsf(path->current_point.x - target_pose[eLEFT_AISLE].x) < 0.25f && fabsf(path->current_point.y - target_pose[eLEFT_AISLE].y) < 0.20f && fabsf(path->err_point.yaw) < 1)
                {
                    //更新当前点位置
                    path->current_point.aisle = target_pose[eLEFT_AISLE].aisle;
                    path->current_point.area = target_pose[eLEFT_AISLE].area;
                    
                    //开启泊车
                    chassis_data.park_flag = 0;

                    //速度拉块
                    Change_speed(4.0f, 4.0f);
                    // Change_speed(1.0f, 1.0f);

                    path->R1.percentage = 0.0f;
					
                    if(path->target_point.aisle == eLEFT)
                    {
                        path->cross_state = CROSS_TO_TARGET;
                    }else if(path->target_point.aisle == eMIDDLE)
                    {
                        path->cross_state = CROSS_TO_LEFT_EXIT;
                    }
                }

            }

            //是否走斜线
            if(path->R1.percentage != 0)
            {
                path->Vx = pid_calc(&x_pid, target_pose[eLEFT_AISLE].x, path->current_point.x);
                path->Vy = pid_calc(&y_pid, target_pose[eLEFT_AISLE].y, path->current_point.y);
                
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
            }else
            {
                path->Vx = pid_calc(&x_pid, target_pose[eLEFT_AISLE].x, path->current_point.x);
                path->Vy = pid_calc(&y_pid, target_pose[eLEFT_AISLE].y, path->current_point.y);
            }

            //速度限幅
            if(fabsf(path->current_point.x - target_pose[eLEFT_AISLE].x) < 1.0f)
            {
                Speed_limit(&path->Vx, 1.0f);
            }

            if(fabsf(path->current_point.y - target_pose[eLEFT_AISLE].y) < 1.0f)
            {
                Speed_limit(&path->Vy, 1.0f);
            }

            break;
        }
        case CROSS_TO_LEFT_EXIT:
        {
            DT_x.MaxOutput = 2.0f;
            DT_x.MaxOutput = 2.0f;

            path->Vx = pid_calc(&x_pid, target_pose[eLEFT_EXIT].x, path->current_point.x);
            path->Vy = pid_calc(&y_pid, target_pose[eLEFT_EXIT].y, path->current_point.y);

            if(fabs(path->current_point.x - target_pose[eLEFT_EXIT].x) < 0.15f && fabs(path->current_point.y - target_pose[eLEFT_EXIT].y) < 0.10f)
            {
                path->current_point.aisle = target_pose[eLEFT_EXIT].aisle;
                path->current_point.area = target_pose[eLEFT_EXIT].area;

                path->Vw = pid_calc_by_error(&w_pid, path->err_point.yaw); 
                //可以将角速度改快一点
                if(fabs(path->err_point.yaw) < 45.0f)          //这里后面最好改成直接判断path.err_point.yaw 因为右侧路径因为转多圈的原因会导致角度不对
                {
                    if(fabs(path->err_point.y) < 0.20f || fabs(path->err_point.x) < 0.20f)
                    {
                        path->cross_state = CROSS_TO_TARGET;
                    }else
                    {
                        path->cross_state = CROSS_TO_RIGHT_EXIT;
                    }
                }
            }else       //未到达中转点的时候保持当前yaw角
            {
                if(fabs(path->err_point.yaw) < 1)       //如果已经转到目标角度，就保持目标角度
                {
                    path->Vw = pid_calc_by_error(&w_pid, path->err_point.yaw);
                }else
                {
                    //这个有缺陷，后续应该修改成上一个状态的yaw角
                    path->Vw = pid_calc(&w_pid, target_pose[path->route[path->step]].yaw * ANGLE_PI, path->current_point.yaw * ANGLE_PI);
                }                    
            }
            break;
        }
        case CROSS_TO_RIGHT_AISLE:
        {
            if(path->current_point.area == eTWO_AREA)
            {
                path->Vx = pid_calc(&x_pid, target_pose[eRIGHT_AISLE].x, path->current_point.x);
                path->Vy = pid_calc(&y_pid, target_pose[eRIGHT_AISLE].y, path->current_point.y);

                if(fabs(path->current_point.x - target_pose[eRIGHT_AISLE].x) < 0.10f && fabs(path->current_point.y - target_pose[eRIGHT_AISLE].y) < 0.10f)
                {
                    path->Vw = pid_calc_by_error(&w_pid, path->err_point.yaw); 
                    if(fabs(path->err_point.yaw) < 45.0f)
                    {
                        if(fabs(path->err_point.y) < 0.20f || fabs(path->err_point.x) < 0.20f)
                        {
                            path->cross_state = CROSS_TO_TARGET;
                        }else
                        {
                            path->cross_state = CROSS_TO_LEFT_AISLE;
                        }
                    }
                }else
                {
                    if(fabs(path->err_point.yaw) < 1)       //如果已经转到目标角度，就保持目标角度
                    {
                        path->Vw = pid_calc_by_error(&w_pid, path->err_point.yaw);
                    }else
                    {
                        //这个有缺陷，后续应该修改成上一个状态的yaw角
                        path->Vw = pid_calc(&w_pid, target_pose[path->route[path->step]].yaw * ANGLE_PI, path->current_point.yaw * ANGLE_PI);
                    }                    
                }
            }else if(path->current_point.area == eONE_AREA)             //如果当前在1区
            {   
                if(block_num == 0)
                {
                    //释放信号量，让上层机械臂到达准备吸快姿态
                    if(path->R1.ready_suction_flag == 0)
                    {
                        xSemaphoreGive(ready_suctionHandle);
                        path->R1.ready_suction_flag = 1;
                    }
    
                    if(path->transform_weapon_count == 0)
                    {
                        xSemaphoreGive(transform_weaponHandle);             //允许上层存武器
                        path->transform_weapon_count ++;
                    }    
                }

                //这里记得先向x方向走一段距离之后再走到右侧入口
                if(path->current_point.x < 0.81f)
                {
                    path->R1.percentage = 0.0f;
                    path->Vx = pid_calc(&x_pid, target_pose[eRIGHT_AISLE].x, path->current_point.x);
                    path->Vy = pid_calc(&y_pid, target_pose[eCONNECT_WEAPON].y, path->current_point.y);
                    path->Vw = pid_calc_by_error(&w_pid, path->err_point.yaw);                     
                }else
                {
//                    path->R1.percentage = fabsf((target_pose[eRIGHT_AISLE].x -  target_pose[path->last_path_state].x) / (target_pose[eRIGHT_AISLE].y - target_pose[path->last_path_state].y));
                    path->Vx = pid_calc(&x_pid, target_pose[eRIGHT_AISLE].x, path->current_point.x);
                    path->Vy = pid_calc(&y_pid, target_pose[eRIGHT_AISLE].y, path->current_point.y);
                    path->Vw = pid_calc_by_error(&w_pid, path->err_point.yaw); 
                }

                if(fabsf(path->current_point.x - target_pose[eRIGHT_AISLE].x) < 0.20f && fabsf(path->current_point.y - target_pose[eRIGHT_AISLE].y) < 0.10f && fabsf(path->err_point.yaw) < 1)
                {
                    //更新当前点位置
                    path->current_point.aisle = target_pose[eRIGHT_AISLE].aisle;
                    path->current_point.area = target_pose[eRIGHT_AISLE].area;
                    
                    //开启泊车
                    chassis_data.park_flag = 0;

                    //速度拉块
                    Change_speed(4.0f, 4.0f);
                    // Change_speed(1.0f, 1.0f);
                    
                    if(path->target_point.aisle == eRIGHT)
                    {
                        path->cross_state = CROSS_TO_TARGET;
                    }
                }
            
            }

            //是否走斜线
            if(path->R1.percentage != 0)
            {
                path->Vx = pid_calc(&x_pid, target_pose[eRIGHT_AISLE].x, path->current_point.x);
                path->Vy = pid_calc(&y_pid, target_pose[eRIGHT_AISLE].y, path->current_point.y);
                
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
            }
 

            break;
        }                
        case CROSS_TO_RIGHT_EXIT:
        {
            path->Vx = pid_calc(&x_pid, target_pose[eRIGHT_EXIT].x, path->current_point.x);
            path->Vy = pid_calc(&y_pid, target_pose[eRIGHT_EXIT].y, path->current_point.y);

            if(fabs(path->current_point.x - target_pose[eRIGHT_EXIT].x) < 0.10f && fabs(path->current_point.y - target_pose[eRIGHT_EXIT].y) < 0.10f)
            {
                path->Vw = pid_calc_by_error(&w_pid, path->err_point.yaw); 
                if(fabs(path->err_point.yaw) < 45.0f)
                {
                    if(fabs(path->err_point.y) < 0.10f || fabs(path->err_point.x) < 0.10f)
                    {
                        path->cross_state = CROSS_TO_TARGET;
                    }else
                    {
                        path->cross_state = CROSS_TO_LEFT_EXIT;
                    }
                }
            }else
            {
                if(fabs(path->err_point.yaw) < 1)       //如果已经转到目标角度，就保持目标角度
                {
                    path->Vw = pid_calc_by_error(&w_pid, path->err_point.yaw);
                }else
                {
                    //这个有缺陷，后续应该修改成上一个状态的yaw角
                    path->Vw = pid_calc(&w_pid, target_pose[path->route[path->step]].yaw * ANGLE_PI, path->current_point.yaw * ANGLE_PI);
                }                    
            }
            break;
        }
        case CROSS_TO_TARGET:
        {
            if(path->fine_tuning_flag == 0)     //使用雷达（这个是防止使用DT35的时候被覆盖）
            {
                path->Vx = pid_calc_by_error(&x_pid, path->err_point.x);
                path->Vy = pid_calc_by_error(&y_pid, path->err_point.y);
                path->Vw = pid_calc_by_error(&w_pid, path->err_point.yaw);


                //如果取第一个块就不减速
                if(path->path_state != eL1 && path->path_state != eR1)
                {
                    if(path->path_state == eL4 || path->path_state == eR4)
                    {
                        //速度限幅
                        if(fabsf(path->err_point.x) < 1.40f)
                        {
                            Speed_limit(&path->Vx, 1.0f);
                        }
        
                        if(fabsf(path->err_point.y) < 0.5f)
                        {
                            Speed_limit(&path->Vy, 1.0f);
                        }
                    }else
                    {
                        //速度限幅
                        if(fabsf(path->err_point.x) < 1.35f)
                        {
                            Speed_limit(&path->Vx, 1.0f);
                        }
        
                        if(fabsf(path->err_point.y) < 0.5f)
                        {
                            Speed_limit(&path->Vy, 1.0f);
                        }
                    }

                }else
                {
                    //速度限幅
                    if(fabsf(path->err_point.x) < 0.5f)
                    {
                        Speed_limit(&path->Vx, 1.0f);
                    }

                    if(fabsf(path->err_point.y) < 0.5f)
                    {
                        Speed_limit(&path->Vy, 1.0f);
                    }                   
                }

                if(Ladar_Determination(path->current_point, path->target_point, 0.03f, 0.05f, 1) == 1)
                {
                    //更新当前点位置，防止再次误判横穿
                    path->current_point.aisle = path->target_point.aisle;
                    path->current_point.area = path->target_point.area;
    
                    path->cross_state = CROSS_NONE;
                    path->is_cross_flag = 0;
                }
            }
            break;
        }
        default:
        {
            break;
        }     

    }

}


void QT_target_process(QT_HandleTypeDef *qt, Path_State *state)
{
    //将手机的编号转化为块的实际位置
    for(uint8_t i = 0; i < 3; i++)
    {
        switch (qt->R1_route[i])
        {
            case 1: state[i] = eR4; break;
            case 2: state[i] = eR3; break;
            case 3: state[i] = eR2; break;
            case 4: state[i] = eR1; break;
            case 5: state[i] = eMIDDLE_EXIT; break;
            case 8: state[i] = eMIDDLE_AISLE; break;
            case 9: state[i] = eL4; break;
            case 10: state[i] = eL3; break;
            case 11: state[i] = eL2; break;
            case 12: state[i] = eL1; break;
            default: break;
        }
    }

    //在R1的三个快全部存入之后再进行寻找要走的块
    if(state[0] != 0 && state[1] != 0 && state[2] != 0 && qt->calc_flag == 0)
    {
        if(radar.route_flag != 0)
        {
            //根据小电脑算出的路径走对应的块
            for(uint8_t i = 0; i < 3; i++)
            {
                if(target_pose[state[i]].aisle == radar.route_flag)
                {
                    qt->R1_route_state[qt->index] = state[i];
                    qt->index ++;
                }
            }
            qt->calc_flag = 1;
        }
    }
}




//状态更新函数
void Path_Update_Status(Path_HandleTypeDef *path, Path_State state)
{
    path->last_path_state = path->path_state;           //保存上一个状态
    path->path_state = state;
    path->target_point.x = target_pose[path->path_state].x;
    path->target_point.y = target_pose[path->path_state].y;
	path->target_point.area = target_pose[path->path_state].area;
	path->target_point.aisle = target_pose[path->path_state].aisle;
    path->target_point.is_outside = target_pose[path->path_state].is_outside;

    path->err_point.x = path->target_point.x - path->current_point.x;
    path->err_point.y = path->target_point.y - path->current_point.y;

    path->fine_tuning_flag = 0;             //开启雷达微调


    pid_reset(&DT_x);
    pid_reset(&DT_y);
    //清空标志位
    memset(&(path->R1), 0, sizeof(Flag_HandleTypeDef));
}


//路径结构体 目标点 微调x 微调y
void Path_ParaInit(Path_HandleTypeDef *path, float fine_tuning_x, float fine_tuning_y, uint8_t is_yaw)
{
    path->target_point.x = target_pose[path->path_state].x;
    path->target_point.y = target_pose[path->path_state].y;
	path->target_point.area = target_pose[path->path_state].area;
	path->target_point.aisle = target_pose[path->path_state].aisle;
    path->target_point.is_outside = target_pose[path->path_state].is_outside;

    path->err_point.x = path->target_point.x - path->current_point.x;
    path->err_point.y = path->target_point.y - path->current_point.y;
	
    path->fine_tuning_x =  fine_tuning_x;
    path->fine_tuning_y = fine_tuning_y;

    //是1则传入目标点yaw角，反之不传入
    if(is_yaw == 1)
    {
        path->target_point.yaw = target_pose[path->path_state].yaw;
    }    

    //手动底盘同步yaw角
    chassis_data.target_yaw = target_pose[path->path_state].yaw;
}


void Err_Update(Path_HandleTypeDef *path)
{
    path->err_point.x = path->target_point.x - path->current_point.x;
    path->err_point.y = path->target_point.y - path->current_point.y;
}


//判断是否到达目标点 当前点 目标点 x方向死区 y方向死区 纠正次数
uint8_t Ladar_Determination(Point_HandleTypeDef curren_point, Point_HandleTypeDef target_pose, float dead_x, float dead_y, uint8_t count)
{
    static uint8_t cnt = 0;
    if(fabsf(curren_point.x - target_pose.x) < dead_x && fabsf(curren_point.y - target_pose.y) < dead_y && fabsf(path.err_point.yaw) < 0.05f)
    {
        cnt ++;
        if(cnt > count)
        {
            cnt = 0;
            return 1;
        }else
        {
            return 0;
        }
    }else
    {
        cnt = 0;
        return 0;
    }
}

uint8_t DT35_Determination(Path_HandleTypeDef *path, float dt_x, float dt_y, float x_deadband, float y_deadband, uint8_t count)
{
    static uint8_t cnt = 0;
    if(fabsf(dt_x - path->fine_tuning_x) < x_deadband && fabsf(dt_y - path->fine_tuning_y) < y_deadband)
    {
        cnt ++;
        if(cnt > count)
        {
            cnt = 0;
            return 1;
        }else
        {
            return 0;
        }    
    }else
    {
        cnt = 0;
        return 0;
    }
}



//改变pid的最大速度
void Change_speed(float x_target_speed, float y_target_speed)
{
    x_pid.MaxOutput = x_target_speed;
    y_pid.MaxOutput = y_target_speed;
}



//速度限制
void Speed_para(Path_HandleTypeDef *path, float x_dis, float x_limit, float y_dis, float y_limit)
{
    if(fabsf(path->err_point.x) < x_dis)
    {
        path->R1.x_speed_limit = x_limit;
    }

    if(fabsf(path->err_point.y) < y_dis)
    {
        path->R1.y_speed_limit = y_limit;
    }
}


//速度限制
void Speed_limit(float *speed, float limit)
{
    if(limit != 0)
    {
        if(*speed >= limit)
        {
            *speed = limit;
        }else if(*speed < -limit)
        {
            *speed = -limit;
        }   
    }else
    {
        *speed = *speed;
    }
}


//释放一次信号量
void GiveSemaphoreOnce(SemaphoreHandle_t sem, uint8_t *flag)
{
    if (*flag == 0)
    {
        xSemaphoreGive(sem);
        *flag = 1;
    }
}

//判断是否开始吸块
void Determine_Start_Sution(Path_HandleTypeDef *path)
{
    //判断一下当前点位是不是所需要走的目标点位
    for(uint8_t i = 0; i < 3; i ++)
    {
        if(path->path_state == qt.R1_route_state[i])
        {
            //更新取块的数量
            block_num = i + 1;

            if(block_num > 1)
            {
                if(center_finish_flag == 1)
                {
                    //释放开始吸块信号量
                    xSemaphoreGive(start_suctionHandle);
                }
            }else
            {
                xSemaphoreGive(start_suctionHandle);
            }
            break;
        }
    }

    if(block_num > 1)
    {
        //在吸比一个块多的情况的时候需要等待归中完成再进入微调逻辑
        if(center_finish_flag == 1)
        {
            path->fine_tuning_flag = 1;         //转入微调逻辑  
        }
    }else
    {
        path->fine_tuning_flag = 1;         //转入微调逻辑  
    }
}


//更新目标点（吸块完成之后调用）  后面如果要修改吸块数量的话就改这里
void Update_Target_Point(Path_HandleTypeDef *path, Path_State s_path_state)
{
   
    if(block_num < 2)     
    {
        //判断是否需要递块来决定要不要吸第二个块
        if(radar.transmit_block_flag == 1)
        {
            //直接更新下一个目标点
            if(qt.R1_route_state[block_num] == 0)
            {
                qt.R1_route_state[block_num] = s_path_state;
            }
            Path_Update_Status(path, qt.R1_route_state[block_num]);
        }else
        {
            //如果不需要递块直接走到出口
            qt.R1_route_state[block_num] = s_path_state;
            Path_Update_Status(path, qt.R1_route_state[block_num]);
        }
    }else
    {
        //如果不需要递块直接走到出口
        qt.R1_route_state[block_num] = s_path_state;
        Path_Update_Status(path, qt.R1_route_state[block_num]);        
    }

    //允许归中（在更新目标点之后触发）
    xSemaphoreGive(start_centerHandle);
}


void DT35_Calc(Path_HandleTypeDef *path, uint8_t mode)
{
    switch(mode)
    {
        case DT_XF_YL:
        {
            DT35_XF_Calc(path, path->current_point.yaw);
            DT35_YL_Calc(path, path->current_point.yaw);
            break;
        }
        case DT_XF_YR:
        {
            DT35_XF_Calc(path, path->current_point.yaw);
            DT35_YR_Calc(path, path->current_point.yaw);
            break;
        }
        case DT_XB_YL:
        {
            DT35_XB_Calc(path, path->current_point.yaw);
            DT35_YL_Calc(path, path->current_point.yaw);
            break;
        }
        case DT_XB_YR:
        {
            DT35_XB_Calc(path, path->current_point.yaw);
            DT35_YR_Calc(path, path->current_point.yaw);
            break;
        }
        default:
        {
            break;
        }

    }

}


void DT35_XF_Calc(Path_HandleTypeDef *path, float yaw)
{
    while(yaw < 0)
    {
        yaw += 360;
    }
    while (yaw > 360)
    {
        yaw -= 360;
    } 

    if(fabsf(yaw - 180) < 0.5f)
    {
        if(path->fine_tuning_x != 0)
        {
            path->Vx = pid_calc(&DT_x, path->fine_tuning_x, DT35_XF);
        }else
        {
            path->Vx = 0;
        }
    }else if(fabsf(yaw - 270) < 0.5f)
    {
        if(path->fine_tuning_y != 0)
        {
            path->Vy = pid_calc(&DT_y, path->fine_tuning_y, DT35_XF);
        }else
        {
            path->Vy = 0;
        }
    }else if(fabsf(yaw - 0) < 0.5f)
    {
        if(path->fine_tuning_x != 0)
        {
            path->Vx = -pid_calc(&DT_x, path->fine_tuning_x, DT35_XF);
        }else
        {
            path->Vx = 0;
        }
    }else if(fabsf(yaw - 90) < 0.5f)
    {
        if(path->fine_tuning_y != 0)
        {
            path->Vy = -pid_calc(&DT_y, path->fine_tuning_y, DT35_XF);
        }else
        {
            path->Vy = 0;
        }
    }
}

void DT35_XB_Calc(Path_HandleTypeDef *path, float yaw)
{
    while(yaw < 0)
    {
        yaw += 360;
    }
    while (yaw > 360)
    {
        yaw -= 360;
    } 

    if(fabsf(yaw - 180) < 0.5f)
    {
        if(path->fine_tuning_x != 0)
        {
            path->Vx = -pid_calc(&DT_x, path->fine_tuning_x, DT35_XB);
        }else
        {
            path->Vx = 0;
        }
    }else if(fabsf(yaw - 270) < 0.5f)
    {
        if(path->fine_tuning_y != 0)
        {
            path->Vy = -pid_calc(&DT_y, path->fine_tuning_y, DT35_XB);
        }else
        {
            path->Vy = 0;
        }
    }else if(fabsf(yaw - 0) < 0.5f)
    {
        if(path->fine_tuning_x != 0)
        {
            path->Vx = pid_calc(&DT_x, path->fine_tuning_x, DT35_XB);
        }else
        {
            path->Vx = 0;
        }
    }else if(fabsf(yaw - 90) < 0.5f)
    {
        if(path->fine_tuning_y != 0)
        {
            path->Vy = pid_calc(&DT_y, path->fine_tuning_y, DT35_XB);
        }else
        {
            path->Vy = 0;
        }
    }
}

void DT35_YR_Calc(Path_HandleTypeDef *path, float yaw)
{
    while(yaw < 0)
    {
        yaw += 360;
    }
    while (yaw > 360)
    {
        yaw -= 360;
    } 

    if(fabsf(yaw - 180) < 0.5f)
    {
        if(path->fine_tuning_y != 0)
        {
            path->Vy = -pid_calc(&DT_y, path->fine_tuning_y, DT35_YR);
        }else
        {
            path->Vy = 0;
        }
    }else if(fabsf(yaw - 270) < 0.5f)
    {
        if(path->fine_tuning_x != 0)
        {
            path->Vx = pid_calc(&DT_x, path->fine_tuning_x, DT35_YR);
        }else
        {
            path->Vy = 0;
        }
    }else if(fabsf(yaw - 0) < 0.5f)
    {
        if(path->fine_tuning_y != 0)
        {
            path->Vy = pid_calc(&DT_y, path->fine_tuning_y, DT35_YR);
        }else
        {
            path->Vy = 0;
        }
    }else if(fabsf(yaw - 90) < 0.5f)
    {
        if(path->fine_tuning_x != 0)
        {
            path->Vx = -pid_calc(&DT_x, path->fine_tuning_x, DT35_YR);
        }else
        {
            path->Vy = 0;
        }
    }
}

void DT35_YL_Calc(Path_HandleTypeDef *path, float yaw)
{
    while(yaw < 0)
    {
        yaw += 360;
    }
    while (yaw > 360)
    {
        yaw -= 360;
    } 

    if(fabsf(yaw - 180) < 0.5f)
    {
        if(path->fine_tuning_y != 0)
        {
            path->Vy = pid_calc(&DT_y, path->fine_tuning_y, DT35_YL);
        }else
        {                                                                                   
            path->Vy = 0;
        }
    }else if(fabsf(yaw - 270) < 0.5f)
    {
        if(path->fine_tuning_x != 0)
        {
            path->Vx = -pid_calc(&DT_x, path->fine_tuning_x, DT35_YL);
        }else
        {
            path->Vy = 0;
        }
    }else if(fabsf(yaw - 0) < 0.5f)
    {
        if(path->fine_tuning_y != 0)
        {
            path->Vy = -pid_calc(&DT_y, path->fine_tuning_y, DT35_YL);
        }else
        {
            path->Vy = 0;
        }
    }else if(fabsf(yaw - 90) < 0.5f)
    {
        if(path->fine_tuning_x != 0)
        {
            path->Vx = pid_calc(&DT_x, path->fine_tuning_x, DT35_YL);
        }else
        {
            path->Vy = 0;
        }
    }
}
