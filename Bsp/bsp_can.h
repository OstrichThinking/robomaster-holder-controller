/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#ifndef __BSP_CAN
#define __BSP_CAN

#include "can.h"

#define FEEDBACK_ID_BASE      0x204
#define CAN_CONTROL_ID_BASE   0x1ff
#define CAN_CONTROL_ID_EXTEND 0x2ff
#define MOTOR_MAX_NUM         7

typedef struct
{
	uint16_t can_id;         //设备ID
    int16_t  set_current;    //设置的电流
    uint16_t rotor_angle;    //机械角度
    int16_t  torque_current; //实际转矩电流
    uint8_t  torque_rated;   //给定转矩电流
}moto_info_6623;

typedef struct
{
    uint16_t can_id;         
    int16_t  set_voltage;
    uint16_t rotor_angle;
    int16_t  rotor_speed;
    int16_t  torque_current;
    uint8_t  temp;
}moto_info_6020;

extern moto_info_6020 motor_info;

void can_user_init(CAN_HandleTypeDef* hcan);
void set_motor_voltage(int16_t v1, int16_t v2, int16_t v3, int16_t v4);
#endif
