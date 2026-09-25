#ifndef PATH_H
#define PATH_H


#include "main.h"
#include "path_lib.h"


#define ANGLE_PI 0.017453292f
#define MAX_X 3
#define MAX_Y 4

#define DT35_DEADBAND_X 0.01
#define DT35_DEADBAND_Y 0.004
//各个状态的微调距离
#define PREPARE_X 0.33f
#define PREPARE_Y 0.269f
#define WEAPON_PACK_X 1.903f
#define WEAPON_PACK_Y 2.6075f                  
#define CONNECT_WEAPON_X  1.900f	        //1.900			  
#define CONNECT_WEAPON_Y 2.6075f            //2.6055
#define LEFT_AISLE_X 0
#define LEFT_AISLE_Y 0
#define RIGHT_AISLE_X 0
#define RIGHT_AISLE_Y 0
#define MIDDLE_AISLE_X 0.1772f
#define MIDDLE_AISLE_Y 2.8655f
#define L1_X 3.425f
#define L1_Y 0.41f
#define L2_X 4.638f
#define L2_Y 0.41f
#define L3_X 0
#define L3_Y 0.41f
#define L4_X 0
#define L4_Y 0.41f
#define LEFT_EXIT_X 0 
#define LEFT_EXIT_Y 0
#define R1_X 3.511f
#define R1_Y 0.10f
#define R2_X 4.335f
#define R2_Y 0.10f
#define R3_X 3.1126f
#define R3_Y 0.10f
#define R4_X 1.8935f
#define R4_Y 0.10f
#define RIGHT_EXIT_X 0
#define RIGHT_EXIT_Y 0
#define MIDDLE_EXIT_X 0.6407f
#define MIDDLE_EXIT_Y 2.8682f
#define ZONE_3_AREA_X 0
#define ZONE_3_AREA_Y 0
#define SUDOKO_1_X 1.3754f
#define SUDOKO_1_Y 4.473f
#define SUDOKO_2_X 0
#define SUDOKO_2_Y 0
#define SUDOKO_3_X 0
#define SUDOKO_3_Y 0
#define B_LIFT_X 0.78f
#define B_LIFT_Y 0.17f

#define F_LIFT_X 0.78f
#define F_LIFT_Y 0.21f
#define FT_LIFT_X 0.429f
#define FT_LIFT_Y 0.204f
#define BT_LIFT_X 0.465f
#define BT_LIFT_Y 0.178f

#define TRANSMIT_BLOCK_X 0.519f
#define TRANSMIT_BLOCK_Y 3.332f

#define ACROSS_LEFT_X 8.3083f
#define ACROSS_LEFT_Y -0.6f

#define LEFT_POINT_NUM 12
#define RIGHT_POINT_NUM 14

#define MIN_Y_DISTANCE -3.88f


extern PID_T x_pid;
extern PID_T y_pid;
extern PID_T w_pid;




void Path_Init(Path_HandleTypeDef *path);
void Path_Update_Status(Path_HandleTypeDef *path, Path_State state);
void Path_Fsm(Path_HandleTypeDef *path, Point_HandleTypeDef *current_pose);



#endif
