#include "uper_control.h"
#include "chassis_control.h"
#include "my_main.h"

float d_v = 5.0f;
float h_v = 0.2f;
float s_v = 0.2f;


static void Arm_action(MechanicalArm_HandleTypeDef *arm, float target_down_pos, float target_down_speed, float target_high_pos, float target_high_speed, float target_sucker_pos, float target_sucker_speed);
static uint8_t Arm_Determination(MechanicalArm_HandleTypeDef *arm, float target_down_pos, float down_dead, float target_high_pos, float high_dead, float target_sucker_pos, float sucker_dead);

//归中状态机
enum
{
    STORAGE_IDLE = 0,      // 空闲
    STORAGE_WAIT,          // 等待1200ms
    STORAGE_CLOSE,         // 关闭吸盘 + 抬升
    STORAGE_FINISH         // 收尾（气缸复位）
}Storage_State_e;


//机械臂存块状态机
enum
{
	ARM_STORAGE_IDLE,
	ARM_STORAGE_WAITING,
	ARM_STORAGE_FINISH
};

//机械臂放块状态机（放回归中）
enum
{
	ARM_PULL_IDLE,
	ARM_PULL_WAITING,
	ARM_PULL_FINISH
};

//吸块函数
void Suction_block(MechanicalArm_HandleTypeDef *arm, uint8_t chunk_buff)
{
	//开启吸盘
	arm->Open_Sucker();		
	//平移电机到达不影响归中位置
	RS05_PositionMode(&Translation_motor, 2.33f, 20.0f);

	if(arm == &Right_arm)
	{
		switch (chunk_buff)
		{
			case 2:
			{
				MG5010E_SpeedPosMode(&arm->Down_motor, 17.0f, 121.0f);
				Motor_Pos_Planning_Init(&arm->High_T, arm->High_motor.real_pos, 3.55f, 0.6f);
				Motor_Pos_Planning_Init(&arm->Sucker_T, arm->Sucker_motor.real_pos, 2.80f, 0.5f);
				break;
			}
			case 4:
			{
				MG5010E_SpeedPosMode(&arm->Down_motor, 17.0f, 96.0f);
				Motor_Pos_Planning_Init(&arm->High_T, arm->High_motor.real_pos, 3.65f, 0.5f);
				Motor_Pos_Planning_Init(&arm->Sucker_T, arm->Sucker_motor.real_pos, 2.47f, 0.5f);
				break;
			}
			case 6:
			{
				MG5010E_SpeedPosMode(&arm->Down_motor, 17.0f, 84.0f);
				Motor_Pos_Planning_Init(&arm->High_T, arm->High_motor.real_pos, 3.35f, 0.5f);
				Motor_Pos_Planning_Init(&arm->Sucker_T, arm->Sucker_motor.real_pos, 1.98f, 0.5f);
				break;
			}
			default:
			{
				break;
			}
		}
	}else if(arm == &Left_arm)
	{
		switch (chunk_buff)
		{
			case 2:
			{
				MG5010E_SpeedPosMode(&arm->Down_motor, 15.0f, -121.8f);
				Motor_Pos_Planning_Init(&arm->High_T, arm->High_motor.real_pos, 1.03f, 0.5f);
				Motor_Pos_Planning_Init(&arm->Sucker_T, arm->Sucker_motor.real_pos, 0.87f, 0.5f);
				break;
			}
			case 4:
			{
				MG5010E_SpeedPosMode(&arm->Down_motor, 15.0f, -98.8f);
				Motor_Pos_Planning_Init(&arm->High_T, arm->High_motor.real_pos, 1.008f, 0.5f);
				Motor_Pos_Planning_Init(&arm->Sucker_T, arm->Sucker_motor.real_pos, 1.17f, 0.5f);
				break;
			}
			default:
			{
				break;
			}
		}
	}
}


//存块（吸块之后放回归中）
void Storage_block(MechanicalArm_HandleTypeDef *arm)
{
	if(arm == &Right_arm)
	{
		MG5010E_SpeedPosMode(&arm->Down_motor, 17.0f, 84.7f);
		Motor_Pos_Planning_Init(&arm->High_T, arm->High_motor.real_pos, 0.58f, 0.35f);
		Motor_Pos_Planning_Init(&arm->Sucker_T, arm->Sucker_motor.real_pos, 2.80f, 0.5f);				//2，81
	}else if(arm == &Left_arm)
	{
		MG5010E_SpeedPosMode(&arm->Down_motor, 17.0f, -83.7f);
		Motor_Pos_Planning_Init(&arm->High_T, arm->High_motor.real_pos, 4.09f, 0.35f);
		Motor_Pos_Planning_Init(&arm->Sucker_T, arm->Sucker_motor.real_pos, 0.95f, 0.5f);
	}
}

//从归中吸块（机械臂存块）		//这里看一下在走右侧吸两个快的时候不用从归中里面吸块 后面记得该逻辑
void Gain_center_block(MechanicalArm_HandleTypeDef *arm)
{
	if(arm == &Right_arm)
	{
		//如果只取了一个或者两个块
		if(block_num < 3)
		{
			arm->Open_Sucker();
			switch(uper_FSM.arm_storage_state)
			{
				case ARM_STORAGE_IDLE:
				{
					//如果此时在三区触发且是从左侧上的三区，机械臂直接到达递块姿态
					if(path.current_point.area == eTHREE_AREA && target_pose[qt.R1_route_state[block_num - 1]].aisle == eLEFT && radar.transmit_block_flag == 1)
					{
						MG5010E_SpeedPosMode(&arm->Down_motor, 10.0f, 96.0f);
						Motor_Pos_Planning_Init(&arm->High_T, arm->High_motor.real_pos, 4.55f, 0.42f);
						Motor_Pos_Planning_Init(&arm->Sucker_T, arm->Sucker_motor.real_pos, 1.735f, 0.5f);
						uper_FSM.arm_transmit_flag = 0;				//清空递块标志位
					}else		//其他情况则走正常逻辑
					{ 
						MG5010E_SpeedPosMode(&arm->Down_motor, 20.0f, 77.1f);
						Motor_Pos_Planning_Init(&arm->High_T, arm->High_motor.real_pos, 0.414f, 0.5f);
						Motor_Pos_Planning_Init(&arm->Sucker_T, arm->Sucker_motor.real_pos, 2.719f, 0.5f);			//2.769
						uper_FSM.arm_storage_state = ARM_STORAGE_WAITING;
					}
					break;
				}
				case ARM_STORAGE_WAITING:
				{
					//电机到位之后开启归中
					if(Arm_Determination(arm, 77.1f, 1.0f, 0.414f, 0.2f, 2.719f, 0.2f) == 1)
					{
						osDelay(300);
						Open_Center_Cylinder();		//开启归中
						uper_FSM.arm_storage_state = ARM_STORAGE_FINISH;						
					}
					break;
				}
				case ARM_STORAGE_FINISH:
				{
					if(path.current_point.area == 3 && radar.transmit_block_flag == 1)			//递块姿态
					{
						MG5010E_SpeedPosMode(&arm->Down_motor, 10.0f, 96.0f);
						Motor_Pos_Planning_Init(&arm->High_T, arm->High_motor.real_pos, 4.55f, 0.42f);
						Motor_Pos_Planning_Init(&arm->Sucker_T, arm->Sucker_motor.real_pos, 1.735f, 0.5f);
						
						uper_FSM.arm_storage_state = ARM_STORAGE_IDLE;
						uper_FSM.arm_transmit_flag = 0;				//清空递块标志位

					}else			//存块姿态
					{
						MG5010E_SpeedPosMode(&arm->Down_motor, 20.0f, 123.0f);
						Motor_Pos_Planning_Init(&arm->High_T, arm->High_motor.real_pos, 1.16f, 0.5f);
						Motor_Pos_Planning_Init(&arm->Sucker_T, arm->Sucker_motor.real_pos, 3.15f, 0.5f);

						if(Arm_Determination(arm, 123.0f, 1.0f, 1.16f, 0.1f, 3.15f, 0.1f) == 1)
						{
							uper_FSM.arm_storage_state = ARM_STORAGE_IDLE;
							uper_FSM.arm_storage_flag = 0;							
						}
					}
					break;
				}
				default:
				{
					break;
				}
			}
		}else if(block_num == 3)		//如果取了三个块
		{
			//机械臂到达指定位置
			// Motor_Pos_Planning_Init(&arm->High_T, arm->High_motor.real_pos, 0.30f, 0.32f);
			// Motor_Pos_Planning_Init(&arm->Down_T, arm->Down_motor.real_pos, 0.66f, 0.32f);
			// Motor_Pos_Planning_Init(&arm->Sucker_T, arm->Sucker_motor.real_pos, 0.69f, 0.4f);		
			// arm_storage_flag = 0;	
		}
	}else if(arm == &Left_arm)
	{
		//如果走的右侧路径且吸了两个及以上个块，这时候可以直接摆到放块姿态
		if(path.current_point.area == eTHREE_AREA && target_pose[qt.R1_route_state[block_num - 1]].aisle == eRIGHT && block_num >= 2)
		{
			RS05_PositionMode(&Translation_motor, 0.5f, 20.0f);

			MG5010E_SpeedPosMode(&arm->Down_motor, 10.0f, -91.2f);
			Motor_Pos_Planning_Init(&arm->High_T, arm->High_motor.real_pos, 0.89f, 0.4f);
			Motor_Pos_Planning_Init(&arm->Sucker_T, arm->Sucker_motor.real_pos, 0.99f, 0.1f);	
			
			uper_FSM.arm_storage_flag = 0;
			place_block_flag = 1;		//放块动作完成
		}else		//其他情况则正常
		{
			//如果只取了一个或者两个块
			if(block_num < 3)
			{
				arm->Open_Sucker();
				switch(uper_FSM.arm_storage_state)
				{
					case ARM_STORAGE_IDLE:
					{
						MG5010E_SpeedPosMode(&arm->Down_motor, 15.0f, -76.0f);
						Motor_Pos_Planning_Init(&arm->High_T, arm->High_motor.real_pos, 4.21f, 0.5f);
						Motor_Pos_Planning_Init(&arm->Sucker_T, arm->Sucker_motor.real_pos, 1.10f, 0.7f);

						uper_FSM.arm_storage_state = ARM_STORAGE_WAITING;
						break;
					}
					case ARM_STORAGE_WAITING:
					{
						//电机到位之后开启归中
						if(Arm_Determination(arm, -76.0f, 1.0f, 4.21f, 0.3f, 1.10f, 0.3f) == 1)
						{
							osDelay(400);
							Open_Center_Cylinder();		//开启归中
							uper_FSM.arm_storage_state = ARM_STORAGE_FINISH;								
						} 
						break;
					}
					case ARM_STORAGE_FINISH:
					{
						if(path.current_point.area == 3)
						{
							RS05_PositionMode(&Translation_motor, 0.5f, 20.0f);

							//直接到达放块位置
							MG5010E_SpeedPosMode(&arm->Down_motor, 10.0f, -92.2f);
							Motor_Pos_Planning_Init(&arm->High_T, arm->High_motor.real_pos, 1.1f, 0.4f);
							Motor_Pos_Planning_Init(&arm->Sucker_T, arm->Sucker_motor.real_pos, 0.99f, 0.1f);	

							//快到目标点时减速一手
							if(fabsf(Left_arm.High_motor.real_pos - 1.0f) < 0.3f)
							{
								Motor_Pos_Planning_Init(&arm->High_T, arm->High_motor.real_pos, 0.89f, 0.08f);
							}

							if(Arm_Determination(arm, -92.2f, 1.1f, 0.89f, 0.3f, 0.99f, 0.3) == 1)
							{
								uper_FSM.arm_storage_state = ARM_STORAGE_IDLE;
								uper_FSM.arm_storage_flag = 0;
								place_block_flag = 1;			//放块动作完成
							}
						}else
						{
							//机械臂到达存块位置 
							MG5010E_SpeedPosMode(&arm->Down_motor, 15.0f, -127.8f);
							Motor_Pos_Planning_Init(&arm->High_T, arm->High_motor.real_pos, 3.355f, 0.5f);
							Motor_Pos_Planning_Init(&arm->Sucker_T, arm->Sucker_motor.real_pos, 0.39f, 0.5f);

							uper_FSM.arm_storage_state = ARM_STORAGE_IDLE;
							uper_FSM.arm_storage_flag = 0;
						}					
						break;
					}
					default:
					{
						break;
					}
				}
			}else if(block_num == 3)		//如果取了三个块就直接举着
			{
				//机械臂到达放块位置


			}
		}

	}
}


//到达预备吸块姿态	
void Ready_suction(uint8_t aisle)
{			
	if(aisle == 1)		//走左侧，使用左臂  
	{
		Left_arm.Open_Sucker();
		//准备吸块
		MG5010E_SpeedPosMode(&Left_arm.Down_motor, 15.0f, -98.8f);
		osDelay(1);
		Motor_Pos_Planning_Init(&Left_arm.High_T, Left_arm.High_motor.real_pos, 1.008f, 0.7f);
		osDelay(1);
		Motor_Pos_Planning_Init(&Left_arm.Sucker_T, Left_arm.Sucker_motor.real_pos, 1.26f, 0.5f);		
		osDelay(1);

		//转到不影响归中位置
		MG5010E_SpeedPosMode(&Right_arm.Down_motor, 15.0f, 121.80f);
		osDelay(1);
		Motor_Pos_Planning_Init(&Right_arm.High_T, Right_arm.High_motor.real_pos, 0.53f, 0.7f);
		osDelay(1);
		Motor_Pos_Planning_Init(&Right_arm.Sucker_T, Right_arm.Sucker_motor.real_pos, 3.43f, 0.5f);		
	}else if(aisle == 2)
	{
		Right_arm.Open_Sucker();
		//准备吸块
		MG5010E_SpeedPosMode(&Right_arm.Down_motor, 15.0f, 84.0f);
		osDelay(1);
		Motor_Pos_Planning_Init(&Right_arm.High_T, Right_arm.High_motor.real_pos, 3.36f, 0.7f);
		osDelay(1);
		Motor_Pos_Planning_Init(&Right_arm.Sucker_T, Right_arm.Sucker_motor.real_pos, 1.98f, 0.5f);			

		//转到不影响归中位置
		osDelay(1);
		MG5010E_SpeedPosMode(&Left_arm.Down_motor, 15.0f, -120.3f);
		osDelay(1);
		Motor_Pos_Planning_Init(&Left_arm.High_T, Left_arm.High_motor.real_pos, 4.00f, 0.7f);
		osDelay(1);
		Motor_Pos_Planning_Init(&Left_arm.Sucker_T, Left_arm.Sucker_motor.real_pos, 0.52f, 0.5f);		
	}	
}

//取块之后归中状态机
void Center_FSM(MechanicalArm_HandleTypeDef *arm)
{
	static uint8_t s_block_num = 0;

	//如果是右臂
	if(arm == &Right_arm)
	{
		//只取了一个或者两个块
		if(block_num < 3)
		{
			switch(uper_FSM.center_state)
			{
				case STORAGE_IDLE:
				{
					//清空归中标志位
					center_finish_flag = 0;
					Open_Center_Cylinder();
					s_block_num = block_num;
					Storage_block(arm);
					uper_FSM.center_state = STORAGE_WAIT;
					break;
				}
				case STORAGE_WAIT:
				{
					if(Arm_Determination(arm, 84.7f, 1.0f, 0.58f, 0.2f, 2.80f, 0.3f) == 1)
					{
						arm->Close_Sucker();
	
						uper_timer.storage_time = xTaskGetTickCount();
						uper_FSM.center_state = STORAGE_CLOSE;
					}
					break;
				}
				case STORAGE_CLOSE:
				{
					if(xTaskGetTickCount() - uper_timer.storage_time > 300)
					{
						CLose_Center_Cylinder();
						//如果下一个目标点有块，这个标志位需要晚置一点（等待电机到位）
						if(qt.R1_route_state[s_block_num] != 0 && qt.R1_route_state[s_block_num] != eRIGHT_EXIT && qt.R1_route_state[s_block_num] != eLEFT_EXIT)
						{
							MG5010E_SpeedPosMode(&arm->Down_motor, 13.0f, 84.0f);
							Motor_Pos_Planning_Init(&arm->High_T, arm->High_motor.real_pos, 3.36f, 0.8f);
							Motor_Pos_Planning_Init(&arm->Sucker_T, arm->Sucker_motor.real_pos, 1.98f, 0.5f);	

							//等待电机到位之后置标志位
							if(Arm_Determination(arm, 84.0f, 1.0f, 3.36f, 0.3f, 1.98f, 0.3f) == 1)
							{
								center_finish_flag = 1;				//归中完成标志位
								uper_FSM.center_start_flag = 0;
								uper_FSM.center_state = STORAGE_IDLE;
							}
						}else
						{
							MG5010E_SpeedPosMode(&arm->Down_motor, 13.0f, 94.0f);
							center_finish_flag = 1;			//归中完成标志位
							uper_FSM.center_start_flag = 0;
							uper_FSM.center_state = STORAGE_IDLE;
						}
					}
					break;
				}
				default:
				{
					break;
				}
			}
		}else if(block_num == 3)		//如果此时是第三个块，直接让他一直举着
		{


			uper_FSM.center_start_flag = 0;
			center_finish_flag = 1;
		}
	}else if(arm == &Left_arm)
	{
		//如果只取了1个或者2个块
		if(block_num < 3)
		{
			switch(uper_FSM.center_state)
			{
				case STORAGE_IDLE:
				{
					//清空归中标志位
					center_finish_flag = 0;
					Open_Center_Cylinder();
					s_block_num = block_num;
					Storage_block(arm);
					uper_FSM.center_state = STORAGE_WAIT;
					break;
				}
				case STORAGE_WAIT:
				{
					//判断电机提前到位松吸盘

					if(Arm_Determination(arm, -83.0f, 1.0f, 4.09f, 0.4f, 0.95f, 0.3f) == 1)
					{
						arm->Close_Sucker();

						uper_timer.storage_time = xTaskGetTickCount();
						uper_FSM.center_state = STORAGE_CLOSE;
					}
					break;
				}
				case STORAGE_CLOSE:
				{
					if(xTaskGetTickCount() - uper_timer.storage_time > 400)
					{
						CLose_Center_Cylinder();
						//如果下一个目标点有块，这个标志位需要晚置一点（等待电机到位）
						if(qt.R1_route_state[s_block_num] != 0 && qt.R1_route_state[s_block_num] != eLEFT_EXIT)
						{
							MG5010E_SpeedPosMode(&arm->Down_motor, 5.0f, -98.8f);
							Motor_Pos_Planning_Init(&arm->High_T, arm->High_motor.real_pos, 0.99f, 0.8f);
							Motor_Pos_Planning_Init(&arm->Sucker_T, arm->Sucker_motor.real_pos, 1.26f, 0.5f);	

							if(Arm_Determination(arm, -98.8f, 1.0f, 0.99f, 0.2f, 1.26f, 0.2f) == 1)
							{
								center_finish_flag = 1;
								uper_FSM.center_start_flag = 0;
								uper_FSM.center_state = STORAGE_IDLE;
							}
						}else
						{
							center_finish_flag = 1;			//归中完成标志位
							uper_FSM.center_start_flag = 0;
							uper_FSM.center_state = STORAGE_IDLE;
						}
					}
					break;
				}
				default:
				{
					break;
				}
			}
		}else if(block_num == 3)
		{	
			//如果取了三个就直接举着

			MG5010E_SpeedPosMode(&arm->Down_motor, 5.0f, -81.3f);
			Motor_Pos_Planning_Init(&arm->High_T, arm->High_motor.real_pos, 0.65f, 0.5f);
			Motor_Pos_Planning_Init(&arm->Sucker_T, arm->Sucker_motor.real_pos, 0.92f, 0.5f);		

			uper_FSM.center_start_flag = 0;
			center_finish_flag = 1;
		}

	}
}

void Recycle_arm(void)
{
	//平移电机回收
	RS05_PositionMode(&Translation_motor, 0.0f, 20.0f);


    //机械臂回收逻辑
	MG5010E_SpeedPosMode(&Left_arm.Down_motor, 15.0f, -36.5f);
	Motor_Pos_Planning_Init(&Left_arm.High_T, Left_arm.High_motor.real_pos, 1.80f, 0.5f);
	Motor_Pos_Planning_Init(&Left_arm.Sucker_T, Left_arm.Sucker_motor.real_pos, 3.11f, 0.5f);	


	MG5010E_SpeedPosMode(&Right_arm.Down_motor, 15.0f, 40.4f);
	Motor_Pos_Planning_Init(&Right_arm.High_T, Right_arm.High_motor.real_pos, 2.49f, 0.5f);
	Motor_Pos_Planning_Init(&Right_arm.Sucker_T, Right_arm.Sucker_motor.real_pos, 0.42f, 0.5f);	
}


//机械臂动作
void Arm_action(MechanicalArm_HandleTypeDef *arm, float target_down_pos, float target_down_speed, float target_high_pos, float target_high_speed, float target_sucker_pos, float target_sucker_speed)
{
    MG5010E_SpeedPosMode(&arm->Down_motor, target_down_pos, target_down_speed);
    Motor_Pos_Planning_Init(&arm->High_T, arm->High_motor.real_pos, target_high_pos, target_high_speed);
    Motor_Pos_Planning_Init(&arm->Sucker_T, arm->Sucker_motor.real_pos, target_sucker_pos, target_sucker_speed);
}


//判断机械臂是否到位
uint8_t Arm_Determination(MechanicalArm_HandleTypeDef *arm, float target_down_pos, float down_dead, float target_high_pos, float high_dead, float target_sucker_pos, float sucker_dead)
{
    if(fabsf(arm->Down_motor.pos - target_down_pos) <= down_dead && fabsf(arm->High_motor.real_pos - target_high_pos) <= high_dead && fabsf(arm->Sucker_motor.real_pos - target_sucker_pos) <= sucker_dead)
    {
        return 1;       
    }else
    {
        return 0;
    }
}

void Arm_fine_tuning(Handle_Data *handle_data, MechanicalArm_HandleTypeDef *arm)
{
	static int last_left_thumb_state = 0;
	static int last_right_thumb_state = 0;
	static float last_right_rock = 0;


	if(last_left_thumb_state == 0 && handle_data->rock_data.left_thumb != 0)
	{
		arm->down_pos = arm->Down_motor.pos;
	}
	last_left_thumb_state = handle_data->rock_data.left_thumb;

	if(last_right_thumb_state == 0 && handle_data->rock_data.right_thumb != 0)
	{
		arm->high_pos = arm->High_motor.real_pos;
	}
	last_right_thumb_state = handle_data->rock_data.right_thumb;

	if(fabsf(last_right_rock) < 40 && fabsf(handle_data->rock_data.rock_right_y) > 40)
	{	
		arm->sucker_pos = arm->Sucker_motor.real_pos;
	}
	last_right_rock = handle_data->rock_data.rock_right_y;

	//大臂MG5010E
	if (handle_data->rock_data.left_thumb == 1)
	{
		arm->down_pos += 0.3f;
		if(arm == &Left_arm)
		{
			if(arm->down_pos > 0) arm->down_pos = 0.0f;
		}else
		{
			if(arm->down_pos > 172) arm->down_pos = 172.0f;
		}
		MG5010E_SpeedPosMode(&arm->Down_motor, 5.0f, arm->down_pos);
	}
	else if (handle_data->rock_data.left_thumb == 3)
	{
		arm->down_pos -= 0.3f;
		if(arm == &Left_arm)
		{
			if(arm->down_pos < -173) arm->down_pos = -173.0f;
		}else
		{
			if(arm->down_pos < 0) arm->down_pos = 0.0f;
		}		
		MG5010E_SpeedPosMode(&arm->Down_motor, 5.0f, arm->down_pos);
	}

	//小臂RS00
	if (handle_data->rock_data.right_thumb == 1)
	{		
		arm->high_pos += 0.005f;
		if(arm->high_pos > 4.56f) arm->high_pos = 4.56f;
		RS00_MITMode(&arm->High_motor, arm->high_pos, 0.0f, 50.0f, 5.0f, 0.0f);
	}
	else if (handle_data->rock_data.right_thumb == 3)
	{
		arm->high_pos -= 0.005f;
		if(arm->high_pos < 0.0f) arm->high_pos = 0.0f;
		RS00_MITMode(&arm->High_motor, arm->high_pos, 0.0f, 50.0f, 5.0f, 0.0f);
	}


	//吸盘RS05
	if (handle_data->rock_data.rock_right_y >= 40)
	{
		arm->sucker_pos += 0.01f;
		if(arm->sucker_pos > 3.7f) arm->sucker_pos = 3.7f;
		RS05_MITMode(&arm->Sucker_motor, arm->sucker_pos, 0.0f, 10.0f, 5.0f, 0.0f);
	}
	else if (handle_data->rock_data.rock_right_y <= -40)
	{
		arm->sucker_pos -= 0.01f;
		if(arm->sucker_pos < 0.0f) arm->sucker_pos = 0.0f;
		RS05_MITMode(&arm->Sucker_motor, arm->sucker_pos, 0.0f, 10.0f, 5.0f, 0.0f);
	}

}
