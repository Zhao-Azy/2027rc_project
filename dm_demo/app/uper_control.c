#include "my_main.h"

#include "uper_control.h"
#include "chassis_control.h"
#include "usart_it.h"
#include "fdcan_it.h"
#include "uper_lib.h"	


//初始化
static void Device_Init(void);
static void Key_DataProcess(Handle_Data *handle_data);
static void One_AreaAction(Handle_Data *handle_data );
static void Two_AreaAction(Handle_Data *handle_data );
static void Three_AreaAction(void);

static void Open_Left_Sucker(void);
static void Close_Left_Sucker(void);
static void Open_Right_Sucker(void);
static void Close_Right_Sucker(void);

/*****************  全局变量  ******************/
// 按键
Key key_left[8];
Key key_right[8];
Key key_middle[8];

uint8_t key_left_value[8];
uint8_t key_right_value[8];
uint8_t key_middle_value[8];

//灯带
WS2811_HandleTypeDef LEDStrip;


// 舵电机（全向轮底盘已移除）
// DJI_HandleTypeDef LF_steer;
// DJI_HandleTypeDef LB_steer;
// DJI_HandleTypeDef RF_steer;
// DJI_HandleTypeDef RB_steer;

// 轮电机（全向轮：4个大疆电机，定义在 chassis.c）
// VESC_HandleTypeDef wheelLF;
// VESC_HandleTypeDef wheelRF;
// VESC_HandleTypeDef wheelRB;
// VESC_HandleTypeDef wheelLB;


//舵向电机回零结构体（全向轮底盘已移除）
// volatile Steer_Homing_State steer_homing_state = eHOMING_IDLE;
// volatile Steer_Homing_TypeDef steer_homing[4] = {
//     {&LF_steer, IO_20_GPIO_Port, IO_20_Pin, 0},
//     {&RF_steer, IO_21_GPIO_Port, IO_21_Pin, 0},
//     {&RB_steer, IO_18_GPIO_Port, IO_18_Pin, 0},
//     {&LB_steer, IO_19_GPIO_Port, IO_19_Pin, 0}
// };

//右侧机械臂
MechanicalArm_HandleTypeDef Right_arm = {0};

//左侧机械臂
MechanicalArm_HandleTypeDef Left_arm = {0};


//抓取武器部分
RS00_HandleTypeDef Spin_clow;
RS05_HandleTypeDef Translation_motor;
DJI_HandleTypeDef Weapon_flex;
DJI_HandleTypeDef Storage_motor;

//简式抬升部分
DJI_HandleTypeDef Right_flex;
DJI_HandleTypeDef Left_flex;


// 电机T度规划结构体(爪子旋转RS00)
T_Planning_HandleTypeDef Spin_T;	


//结构体
Uper_Timer_HandleTypeDef uper_timer = {0};
Uper_Flag_HandleTypeDef uper_debug_flag = {0};
Uper_FSM_HandleTypeDef uper_FSM = {0};
Uper_MotorPos_HandleTypeDef uper_motor_pos = {0};



volatile uint8_t place_block_flag = 0;		//三区放块动作是否完成标志位
volatile uint8_t crawling_completed = 0;		//是否抓取完成标志位		
volatile uint8_t block_num = 0;				//已经取的块个数
volatile uint8_t center_finish_flag = 0;		//归中完成标志位
volatile uint8_t led_state = 0;					//

float speed_limit = 150.0f;
uint8_t debug_io[4] = {0};
uint8_t cnt = 0;
float pos[4] = {0};
float temp = 0.6f;
float speed = 80.0f;
float s = 33.0f;
float k = 2.1f;

void Upper_Init(void)
{
	WS2811_Init(LEDStrip, htim1, TIM_CHANNEL_2, 20);
	LEDStrip.pf->setColor(&LEDStrip, 1, 20, LED_COLOR_BLACK);

	// Can初始化
	FDCAN_All_Init();

	// 定时器中断开启（电机规划 / can定时检测）
	HAL_TIM_Base_Start_IT(&htim4);


	/**  电机初始化  **/
	// 底盘			can1
	DJI_C620_Init(&wheelLF, &hfdcan1, 1);
	DJI_C620_Init(&wheelRF, &hfdcan1, 2);
	DJI_C620_Init(&wheelRB, &hfdcan1, 3);
	DJI_C620_Init(&wheelLB, &hfdcan1, 4);


	//记得更改3508电机id
	// DJI_Init(&LF_steer, &hfdcan1, M3508, HelmLF_ID, 1);
	// DJI_Init(&RF_steer, &hfdcan1, M3508, HelmRF_ID, 1);
	// DJI_Init(&RB_steer, &hfdcan1, M3508, HelmRB_ID, 1);
	// DJI_Init(&LB_steer, &hfdcan1, M3508, HelmLB_ID, 1);

	//机械臂电机		can2
	//右臂
	MG5010E_Init(&Right_arm.Down_motor, &hfdcan2, R_DOWN_ID, 36.0f);
	RS00_Init(&Right_arm.High_motor, &hfdcan2, eRS00_MIT_MODE, R_HIGH_ID);
	RS05_Init(&Right_arm.Sucker_motor, &hfdcan2, eRS05_MIT_MODE, R_SUCKER_ID);
	Right_arm.Open_Sucker = Open_Right_Sucker;
	Right_arm.Close_Sucker = Close_Right_Sucker;
	Right_arm.high_pos = 0.0f;
	Right_arm.sucker_pos = 3.12f;

	//左臂
	MG5010E_Init(&Left_arm.Down_motor, &hfdcan2, L_DOWN_ID, 36.0f);
	RS00_Init(&Left_arm.High_motor, &hfdcan2, eRS00_MIT_MODE, L_HIGH_ID);
	RS05_Init(&Left_arm.Sucker_motor, &hfdcan2, eRS05_MIT_MODE, L_SUCKER_ID);
	Left_arm.Open_Sucker = Open_Left_Sucker;
	Left_arm.Close_Sucker = Close_Left_Sucker;
	Left_arm.high_pos = 4.57f;
	Left_arm.sucker_pos = 0.42f;


	//抓取武器部分
	RS00_Init(&Spin_clow, &hfdcan3, eRS00_MIT_MODE, SPIN_CLOW_ID);
	RS05_Init(&Translation_motor, &hfdcan3, eRS05_POSITION_MODE, 25);
	DJI_Init(&Weapon_flex, &hfdcan3, M2006, 1, WEAPON_FLEX_ID);
	DJI_Init(&Storage_motor, &hfdcan3, M2006, 1, STORAGE_MOTOR_ID);

	//简式抬升部分
	DJI_Init(&Right_flex, &hfdcan3, M2006, 1, R_FLEX_ID);
	DJI_Init(&Left_flex, &hfdcan3, M2006, 1, L_FLEX_ID);



	// 各项设备初始化
	Device_Init();
}



//车体1分钟准备的时候的姿态
void Device_Init(void)
{
	//舵轮回零
	// steer_homing_state = eHOMING_READY; // 全向轮底盘无舵向回零


	//DJI回零指令
	DJI_HomingMode(&Storage_motor, -100.0f);
	DJI_HomingMode(&Weapon_flex, -100.0f);
	DJI_HomingMode(&Left_flex, -100.0f);
	DJI_HomingMode(&Right_flex, 100.0f);


	uper_timer.wait_homing_tick = HAL_GetTick();			//获取回零时间

	//机械臂锁零点
	//left_arm
	MG5010E_SpeedPosMode(&Left_arm.Down_motor, 5.0f, 0.0f);
	Motor_Pos_Planning_Init(&Left_arm.High_T, Left_arm.High_motor.real_pos, 0.0f, 0.5f);
	Motor_Pos_Planning_Init(&Left_arm.Sucker_T, Left_arm.Sucker_motor.real_pos, 3.12f, 0.1f);

	//right_arm
	MG5010E_SpeedPosMode(&Right_arm.Down_motor, 5.0f, 0.0f);
	Motor_Pos_Planning_Init(&Right_arm.High_T, Right_arm.High_motor.real_pos, 4.57f, 0.5f);
	Motor_Pos_Planning_Init(&Right_arm.Sucker_T, Right_arm.Sucker_motor.real_pos, 0.42f, 0.1f);

	//抓取部分锁零点
	Motor_Pos_Planning_Init(&Spin_T, Spin_clow.real_pos, 1.57f, 0.5f);
	RS05_PositionMode(&Translation_motor, 0.2f, 5.0f);

	//等待电机回零
	while (HAL_GetTick() - uper_timer.wait_homing_tick < 800)
	{
		HAL_Delay(1);
	}


	//DJI锁零点
	DJI_PositionMode(&Storage_motor, 1.0f, 50.0f, 50);
	HAL_Delay(1);
	DJI_PositionMode(&Weapon_flex, 0.0f, 50.0f, 50);
	HAL_Delay(1);
	DJI_PositionMode(&Left_flex, 1.0f, 50.0f, 50);
	HAL_Delay(1);
	DJI_PositionMode(&Right_flex, -1.0f, 50.0f, 50);

	//气缸开启	
	Open_Center_Cylinder();         
	Open_Arm_Clow();   
}

void App_Uper(void *argument)
{
	static Handle_Data s_handle_data = {0};


    for(;;)
    {
		xQueueReceive(Handle_Data_ToUpperHandle, &s_handle_data, 1); // 手柄数据

		//按键数据处理
		Key_DataProcess(&s_handle_data);

		One_AreaAction(&s_handle_data);
		Two_AreaAction(&s_handle_data);
		Three_AreaAction();


		osDelay(1);
	}
}


void Key_DataProcess(Handle_Data *handle_data)
{
	// 存储按键状态·
	// 右边按键  （！！！记得更改按键按下有效次数）
	key_right_value[0] = Key_Detect(&key_right[0], 2);
	key_right_value[1] = Key_Detect(&key_right[1], 2);
	key_right_value[2] = Key_Detect(&key_right[2], 2);
	key_right_value[3] = Key_Detect(&key_right[3], 1);
	key_right_value[4] = Key_Detect(&key_right[4], 1);
	key_right_value[5] = Key_Detect(&key_right[5], 2);
	key_right_value[6] = Key_Detect(&key_right[6], 1);
	key_right_value[7] = Key_Detect(&key_right[7], 2);

	key_left_value[5] = Key_Detect(&key_left[5], 2);
	key_left_value[0] = Key_Detect(&key_left[0], 2);



	//判断是左臂还是右臂
	if(key_middle[5].key == 0)
	{
		Arm_fine_tuning(handle_data, &Left_arm);
	}else
	{
		Arm_fine_tuning(handle_data, &Right_arm);
	}


	if(key_right_value[0] == 1)
	{
		Motor_Pos_Planning_Init(&Spin_T, Spin_clow.real_pos, 1.57f, 0.5f);
	}else if(key_right_value[0] == 2)
	{
		Motor_Pos_Planning_Init(&Spin_T, Spin_clow.real_pos, 0.0f, 0.5f);
	}

	if(key_right_value[1] == 1)
	{
		FlexLift(2044, 0, 80);
	}else if(key_right_value[1] == 2)
	{
		FlexLift(0, 0, 80);
	}

	if(key_left_value[5] == 1)
	{
		Open_Right_Sucker();
		Open_Left_Sucker();
	}else if(key_left_value[5] == 2)
	{
		Close_Left_Sucker();
		Close_Right_Sucker();
	}
	

	if(key_right_value[7] == 1)
	{
		Open_Right_Clow();
	}else if(key_right_value[7] == 2)
	{
		Close_Right_Clow();
	}


}


void One_AreaAction(Handle_Data *handle_data )
{
	/* ====================================== 准备抓取 ====================================== */
	if(xSemaphoreTake(ready_captureHandle, 1) == pdTRUE || uper_debug_flag.ready_capture_flag == 1)
	{
		Open_Arm_Clow();
		//DM伸出
		RS05_PositionMode(&Translation_motor, 14.0f, 20.0f);
		DJI_PositionMode(&Weapon_flex, 135.0f, 50.0f, 50);
		osDelay(1);
		Motor_Pos_Planning_Init(&Spin_T, Spin_clow.real_pos, 0.005f, 0.5f);
		uper_debug_flag.ready_capture_flag = 0;
	}

	/* ====================================== 开始抓取 ====================================== */
	if(xSemaphoreTake(start_captureHandle, 1) == pdTRUE || uper_debug_flag.start_capture_flag == 1)
	{
		Motor_Pos_Planning_Init(&Spin_T, Spin_clow.real_pos, 0.005f, 0.5f);
		RS05_PositionMode(&Translation_motor, 16.93f, 33.0f);
		uper_debug_flag.start_capture_flag = 0;
		uper_motor_pos.translation_pos = 16.93f;
		uper_timer.start_capture_tick = xTaskGetTickCount(); 
	}

	/* ====================================== 转出自动抓杆 ====================================== */
	if(xTaskGetTickCount() - uper_timer.start_capture_tick > 180 && uper_timer.start_capture_tick != 0)
	{
		Close_Arm_Clow();
		osDelay(80);
		//2006往上拔		
		DJI_PositionMode(&Weapon_flex, 200.0f, 50.0f, 50);
		uper_timer.start_capture_tick = 0;		

		crawling_completed = 1;
		//记得解注释
		// auto_flag = 1;
	}


	/* ====================================== 爪子闭合抓取 ====================================== */
	if(key_right_value[2] == 1)
	{
		Close_Arm_Clow();
		osDelay(120);
		//2006往上拔
		DJI_PositionMode(&Weapon_flex, 200.0f, 60.0f, 50);
	}else if(key_right_value[2] == 2)
	{
		Open_Arm_Clow();
		//2006往下放
		DJI_PositionMode(&Weapon_flex, 135.0f, 60.0f, 50);
	}


	/* ====================================== 准备对接 ====================================== */
	if(xSemaphoreTake(ready_connectHandle, 1) == pdTRUE || uper_debug_flag.ready_connect_flag == 1)
	{
		uper_motor_pos.translation_pos = 1.0f;
		//平移DM到位 以及伸缩2006往外出
		
		RS05_PositionMode(&Translation_motor, uper_motor_pos.translation_pos, 33.0f);
		osDelay(6);
		DJI_PositionMode(&Weapon_flex, 390.0f, 90, 50);
		osDelay(180);
		DJI_PositionMode(&Weapon_flex, 390.0f, 90, 50);					//发两次
		Motor_Pos_Planning_Init(&Spin_T, Spin_clow.real_pos, 1.565f, 0.7f);
		uper_motor_pos.spin_clow_pos = 1.565f;
		uper_motor_pos.weapon_flex_pos = 390.0f;
		uper_debug_flag.ready_connect_flag = 0;
	}




	/* ====================================== 对接武器微调 ====================================== */
	if(key_left[0].key == 1)
	{
		uper_motor_pos.weapon_flex_pos += 7.0f;
		// if(weapon_flex_pos < -530.0f) weapon_flex_pos = -530.0f;
		DJI_PositionMode(&Weapon_flex, uper_motor_pos.weapon_flex_pos, 50, 50);
	}

	if(key_right_value[3] == 1)
	{
		uper_motor_pos.spin_clow_pos -= 0.005;
		Motor_Pos_Planning_Init(&Spin_T, Spin_clow.real_pos, uper_motor_pos.spin_clow_pos, 0.4f);
	}

	if(key_right_value[4] == 1)
	{
		uper_motor_pos.spin_clow_pos += 0.005;
		Motor_Pos_Planning_Init(&Spin_T, Spin_clow.real_pos, uper_motor_pos.spin_clow_pos, 0.4f);
	}

	if(path.R1.fine_tuning_weapon_flag == 1)
	{
		if(fabsf(radar.translation_distance) > 0.005 && fabsf(radar.translation_distance) < 0.03)
		{
			uper_motor_pos.translation_pos += radar.translation_distance * k;
		}else if(fabsf(radar.translation_distance) < 0.005 && fabsf(radar.translation_distance) > 0.0005)
		{
			if(radar.translation_distance > 0)
			{
				uper_motor_pos.translation_pos += 0.004;
			}else if(radar.translation_distance < 0)
			{
				uper_motor_pos.translation_pos -= 0.004;
			}
		}
		if(uper_motor_pos.translation_pos <= 0.2f) uper_motor_pos.translation_pos = 0.2f;
		if(uper_motor_pos.translation_pos >= 18.2f) uper_motor_pos.translation_pos = 18.2f;
		RS05_PositionMode(&Translation_motor, uper_motor_pos.translation_pos, s);

	}


	/* ====================================== 是否关闭微调 ====================================== */
	if(key_right_value[5] == 1)
	{
		path.R1.fine_tuning_weapon_flag = 0;
	}else if(key_right_value[5] == 2)
	{
		path.R1.fine_tuning_weapon_flag = 1;
	}

	//手动微调对接平移
	if(path.R1.fine_tuning_weapon_flag == 0)
	{
		if (handle_data->rock_data.rock_right_x >= 40)
		{
			uper_motor_pos.translation_pos -= 0.05f;
			if(uper_motor_pos.translation_pos <= 0.0f) uper_motor_pos.translation_pos = 0.2f;
			//电机转动
			RS05_PositionMode(&Translation_motor, uper_motor_pos.translation_pos, 20.0f);

		}
		else if (handle_data->rock_data.rock_right_x <= -40)
		{
			uper_motor_pos.translation_pos += 0.05f;
			if(uper_motor_pos.translation_pos >= 18.2f) uper_motor_pos.translation_pos = 18.2f;
			//电机转动
			RS05_PositionMode(&Translation_motor, uper_motor_pos.translation_pos, 20.0f);
		}
	}

	/* ====================================== 对接完成 ====================================== */
	//将武器捅出之后延时显示二维码
	if(key_right_value[6] == 1)			
	{
		uper_motor_pos.weapon_flex_pos = Weapon_flex.position;
		uper_motor_pos.weapon_flex_pos += 130;


		//这里将武器捅出
		DJI_PositionMode(&Weapon_flex, uper_motor_pos.weapon_flex_pos, 70, 60);

		osDelay(500);
		HAL_UART_Transmit_DMA(&RADAR_SERIAL, nuc_cmd1, 4);
		
		//关闭武器对接微调
		path.R1.fine_tuning_weapon_flag = 0;
	}


	/* ====================================== 转接武器 ====================================== */
	if(xSemaphoreTake(transform_weaponHandle, 1) == pdTRUE || uper_debug_flag.transimit_weapon_flag == 1)
	{
		uper_debug_flag.transimit_weapon_flag = 0;
		RS05_PositionMode(&Translation_motor, 1.0f, 33.0f);
		DJI_PositionMode(&Storage_motor, 550.0f, 80.0f, 50);
		// RS05_PositionMode(&Translation_motor, 8.5f, 33.0f);
		// DJI_PositionMode(&Storage_motor, 550.0f, 80.0f, 50);
		// Open_Right_Clow();
		// //DM转出
		// Motor_Pos_Planning_Init(&Spin_T, Spin_clow.real_pos, -0.01f, 0.9f);
		// DJI_PositionMode(&Weapon_flex, 485, 70.0f, 50);

		// uper_timer.translation_tick =  xTaskGetTickCount();
	}

	if(xTaskGetTickCount() - uper_timer.translation_tick > 400 && uper_timer.translation_tick != 0)
	{
		//DM回收至爪子位置
		RS05_PositionMode(&Translation_motor, 2.84f, 25.0f);
		uper_timer.transportation_tick = xTaskGetTickCount();		
		uper_timer.translation_tick = 0;
	}

	//判断时间是否到达
	if(xTaskGetTickCount() - uper_timer.transportation_tick > 300 && uper_timer.transportation_tick != 0)
	{
		Close_Right_Clow();
		osDelay(100);
		Open_Arm_Clow();
		// //回收至限位
		RS05_PositionMode(&Translation_motor, 0.5f, 20.0f);

		// osDelay(100);
		DJI_PositionMode(&Storage_motor, 260.0f, 80.0f, 50);
		uper_timer.transportation_tick = 0;
		uper_timer.translation_tick = 0;
	}

	/* ====================================== 转接武器 ====================================== */
	// if(xSemaphoreTake(transform_weaponHandle, 1) == pdTRUE || uper_debug_flag.transmit_second_flag == 1)
	// {
	// 	uper_debug_flag.transmit_second_flag = 0;
	// 	RS05_PositionMode(&Translation_motor, 8.5f, 33.0f);
	// 	DJI_PositionMode(&Storage_motor, 260.0f, 80.0f, 50);
	// 	Open_Left_Clow();
	// 	//DM转出
	// 	Motor_Pos_Planning_Init(&Spin_T, Spin_clow.real_pos, -0.01f, 0.9f);
	// 	DJI_PositionMode(&Weapon_flex, 485, 70.0f, 50);

	// 	uper_timer.translation_tick2 =  xTaskGetTickCount();
	// }

	// if(xTaskGetTickCount() - uper_timer.translation_tick2 > 400 && uper_timer.translation_tick2 != 0)
	// {
	// 	//DM回收至爪子位置
	// 	RS05_PositionMode(&Translation_motor, 2.84f, 25.0f);
	// 	uper_timer.transportation_tick2 = xTaskGetTickCount();		
	// 	uper_timer.translation_tick2 = 0;
	// }

	// //判断时间是否到达
	// if(xTaskGetTickCount() - uper_timer.transportation_tick2 > 300 && uper_timer.transportation_tick2 != 0)
	// {
	// 	Close_Left_Clow();
	// 	osDelay(100);
	// 	Open_Arm_Clow();
	// 	// //回收至限位
	// 	RS05_PositionMode(&Translation_motor, 0.5f, 20.0f);

	// 	// osDelay(100);
	// 	DJI_PositionMode(&Storage_motor, 10.0f, 50.0f, 50);
	// 	uper_timer.transportation_tick2 = 0;
	// 	uper_timer.translation_tick2 = 0;
	// }
}



void Two_AreaAction(Handle_Data *handle_data)
{
	/* ====================================== 准备吸块 ====================================== */
	if(xSemaphoreTake(ready_suctionHandle, 1) == pdTRUE || uper_debug_flag.ready_suction_flag == 1)
	{
		if(path.suction_arm == 1)
		{
			Ready_suction(1);
		}else if(path.suction_arm == 2)
		{
			Ready_suction(2);
		}
		uper_debug_flag.ready_suction_flag = 0;
	}



	/* ====================================== 吸块 ====================================== */		//这里要根据左中右侧判断使用哪个臂 左侧和中侧全部使用左臂
	if(xSemaphoreTake(start_suctionHandle, 1) == pdTRUE || uper_debug_flag.suction_flag == 1)
	{
		uper_debug_flag.suction_flag = 0;
		//判断使用哪边机械臂
		if(path.suction_arm == 1)
		{
			Suction_block(&Left_arm, path.R1.is_chunk);
		}else if(path.suction_arm == 2)
		{
			Suction_block(&Right_arm, path.R1.is_chunk);
		}
	}

	/* ====================================== 吸块之后归中 ====================================== */ 		
	if(xSemaphoreTake(start_centerHandle, 1) == pdTRUE || handle_data->rock_data.right_thumb == 2)
	{
		uper_FSM.center_start_flag = 1;
	}
	
	if(uper_FSM.center_start_flag == 1)
	{
		if(path.suction_arm == 1)
		{
			Center_FSM(&Left_arm);
		}else if(path.suction_arm == 2)
		{
			Center_FSM(&Right_arm);
		}
	}


	/* ====================================== 另一机械臂存块*（机械臂取归中里的块） ====================================== */
	if(xSemaphoreTake(Take_blockHandle, 1) == pdTRUE)
	{
		uper_FSM.arm_storage_flag = 1;
	}


	/**  这里递块的同时不需要跑这段代码，后面需要改一下  **/
	if(uper_FSM.arm_storage_flag == 1)
	{
		if(path.gain_arm == 1)
		{
			Gain_center_block(&Left_arm);
		}else if(path.gain_arm == 2)
		{
			Gain_center_block(&Right_arm);
		}
	}
}


void Three_AreaAction(void)
{
	/* 回收机械臂放置在抬升的过程中达到三区 -------------*/
	if(xSemaphoreTake(recycle_armHandle, 1) || uper_debug_flag.recycle_arm_flag == 1)
	{
		uper_debug_flag.recycle_arm_flag = 0;
		CLose_Center_Cylinder();
		Recycle_arm();


		//如果需要递块，应该清空标志位，防止会受不了机械臂
		if(radar.transmit_block_flag == 1)
		{
			uper_FSM.arm_storage_flag = 0;
		}
	}



	/* ====================== 在三区放块的同时递块 ====================== */
	if(xSemaphoreTake(transmit_blockHandle, 1) == pdTRUE)
	{
		// uper_FSM.arm_storage_flag = 1;
		// uper_debug_flag.start_transmit_flag = 1;
		uper_FSM.arm_transmit_flag = 1;
	}

	if(uper_FSM.arm_transmit_flag == 1)
	{
		Gain_center_block(&Right_arm);
	}


	//关闭吸盘
	if(xSemaphoreTake(close_suckerHandle, 1) == pdTRUE)
	{
		//这里清空递块残留的标志位，后面可以看一下怎么改一下
		uper_FSM.arm_storage_flag = 0;

		Close_Left_Sucker();
		Close_Right_Sucker();
	}

	if(xSemaphoreTake(Close_R_SuckerHandle, 1) == pdTRUE)
	{
		Close_Right_Sucker();
	}

	//防守
	if(xSemaphoreTake(arm_initHandle, 1) == pdTRUE)
	{
		CLose_Center_Cylinder();

		MG5010E_SpeedPosMode(&Left_arm.Down_motor, 20.0f, 0.0f);
		Motor_Pos_Planning_Init(&Left_arm.High_T, Left_arm.High_motor.real_pos, 0.0f, 0.5f);
		Motor_Pos_Planning_Init(&Left_arm.Sucker_T, Left_arm.Sucker_motor.real_pos, 3.12f, 0.5f);

		//right_arm
		MG5010E_SpeedPosMode(&Right_arm.Down_motor, 20.0f, 0.0f);
		Motor_Pos_Planning_Init(&Right_arm.High_T, Right_arm.High_motor.real_pos, 4.57f, 0.6f);
		Motor_Pos_Planning_Init(&Right_arm.Sucker_T, Right_arm.Sucker_motor.real_pos, 0.42f, 0.5f);		

	}
}


void Open_Left_Sucker(void)
{
	HAL_GPIO_WritePin(IO_6_GPIO_Port, IO_6_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IO_7_GPIO_Port, IO_7_Pin, GPIO_PIN_SET);
}

void Close_Left_Sucker(void)
{
	HAL_GPIO_WritePin(IO_6_GPIO_Port, IO_6_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IO_7_GPIO_Port, IO_7_Pin, GPIO_PIN_RESET);
}

void Open_Right_Sucker(void)
{
	HAL_GPIO_WritePin(IO_5_GPIO_Port, IO_5_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IO_4_GPIO_Port, IO_4_Pin, GPIO_PIN_SET);
}

void Close_Right_Sucker(void)
{
	HAL_GPIO_WritePin(IO_5_GPIO_Port, IO_5_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IO_4_GPIO_Port, IO_4_Pin, GPIO_PIN_RESET);
}









