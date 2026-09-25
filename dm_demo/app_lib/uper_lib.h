#ifndef UPER_LIB_H
#define UPER_LIB_H

#include "uper_control.h"


void Suction_block(MechanicalArm_HandleTypeDef *arm, uint8_t chunk_buff);
void Storage_block(MechanicalArm_HandleTypeDef *arm);
void Gain_center_block(MechanicalArm_HandleTypeDef *arm);
void Ready_suction(uint8_t aisle);
void Center_FSM(MechanicalArm_HandleTypeDef *arm);
void Recycle_arm(void);
void Arm_fine_tuning(Handle_Data *handle_data, MechanicalArm_HandleTypeDef *arm);


#endif
