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
 
#include "bsp_can.h"
#include "stdio.h"


moto_info_6020 motor_info_6020_1 = {0};
moto_info_6020 motor_info_6020_2 = {0};
moto_info_6020 motor_info_6020_3 = {0};
moto_info_6020 motor_info_6020_4 = {0};

moto_info_6623 motor_info_6623_1 = {0};
moto_info_6623 motor_info_6623_2 = {0};
moto_info_6623 motor_info_6623_3 = {0};
moto_info_6623 motor_info_6623_4 = {0};

uint16_t can_cnt;

/**
  * @brief  init can filter, start can, enable can rx interrupt
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void can_user_init(CAN_HandleTypeDef* hcan )
{
  CAN_FilterTypeDef  can_filter;

  can_filter.FilterBank = 0;                       // filter 0
  can_filter.FilterMode =  CAN_FILTERMODE_IDMASK;  // mask mode
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh = 0x0000;
  can_filter.FilterIdLow  = 0x0000;
  can_filter.FilterMaskIdHigh = 0x0000;
  can_filter.FilterMaskIdLow  = 0x0000;           // set mask 0 to receive all can id
  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0; // assign to fifo0
  can_filter.FilterActivation = ENABLE;           // enable can filter
  can_filter.SlaveStartFilterBank  = 14;          // only meaningful in dual can mode
   
  if(HAL_CAN_ConfigFilter(hcan,&can_filter) != HAL_OK){
		Error_Handler();
	    //printf("过滤器初始化失败");
	}
	if(HAL_CAN_ActivateNotification(hcan,CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK){
		Error_Handler();
	}
	if(HAL_CAN_Start(hcan) != HAL_OK){
		Error_Handler();
	}
}

/**
  * @brief  can rx callback, get motor feedback info
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
	
	if(hcan->Instance == CAN1){
		HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rx_header,rx_data);
	}
	uint8_t index = rx_header.StdId - FEEDBACK_ID_BASE;  // get motor index by can_id
	switch(index){
		case 1:
			motor_info_6623_1.rotor_angle    = ((rx_data[0] << 8) | rx_data[1]) / 22.75f; //换为0-360内的角度
			motor_info_6623_1.torque_current = ((rx_data[2] << 8) | rx_data[3]);
			motor_info_6623_1.torque_rated   = ((rx_data[4] << 8) | rx_data[5]);
//			printf("  6623-1机械角度  %2d ", motor_info_6623_1.rotor_angle);
//			printf("  实际转矩电流 %2d ",    motor_info_6623_1.torque_current);
//			printf("  给定转矩电流 %2d \r\n",    motor_info_6623_1.torque_rated);
//		
		
//			can_cnt++;
//			motor_info_6020_1.rotor_angle    = ((rx_data[0] << 8) | rx_data[1]) / 22.75f; //换为0-360内的角度
//			motor_info_6020_1.rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
//			motor_info_6020_1.torque_current = ((rx_data[4] << 8) | rx_data[5]);
//			motor_info_6020_1.temp           = ((rx_data[6] << 8) | rx_data[7]);
//			printf("  2060-1机械角度  %2d ",    motor_info.rotor_angle);
//			printf("  转速 %2d ",motor_info.rotor_speed);
//			printf("  实际转矩电流 %2d ",    motor_info.torque_current);
//			printf("  温度 %2d  \r\n",motor_info.temp);
			break;
		case 2:
			motor_info_6623_2.rotor_angle    = ((rx_data[0] << 8) | rx_data[1]) / 22.75f; //换为0-360内的角度
			motor_info_6623_2.torque_current = ((rx_data[2] << 8) | rx_data[3]);
			motor_info_6623_2.torque_rated   = ((rx_data[4] << 8) | rx_data[5]);
//			printf("  6623-2机械角度  %2d ", motor_info_6623_2.rotor_angle);
//			printf("  实际转矩电流 %2d ",    motor_info_6623_2.torque_current);
//			printf("  给定转矩电流 %2d \r\n",    motor_info_6623_2.torque_rated);
		
//			can_cnt++;
//			motor_info_6020_2.rotor_angle    = ((rx_data[0] << 8) | rx_data[1]) / 22.75f; //换为0-360内的角度
//			motor_info_6020_2.rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
//			motor_info_6020_2.torque_current = ((rx_data[4] << 8) | rx_data[5]);
//			motor_info_6020_2.temp           = ((rx_data[6] << 8) | rx_data[7]);
//			printf("  2060-2机械角度  %2d ",    motor_info.rotor_angle);
//			printf("  转速 %2d ",motor_info.rotor_speed);
//			printf("  实际转矩电流 %2d ",    motor_info.torque_current);
//			printf("  温度 %2d  \r\n",motor_info.temp);
			break;
		case 3:
			can_cnt++;
			motor_info_6020_3.rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);
			motor_info_6020_3.rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
			motor_info_6020_3.torque_current = ((rx_data[4] << 8) | rx_data[5]);
			motor_info_6020_3.temp           = ((rx_data[6] << 8) | rx_data[7]);
//			printf("  6060-3机械角度  %2d ",    motor_info.rotor_angle);
//			printf("  转速 %2d ",motor_info.rotor_speed);
//			printf("  实际转矩电流 %2d ",    motor_info.torque_current);
//			printf("  温度 %2d  \r\n",motor_info.temp);
			break;
		case 4:
			can_cnt++;
			motor_info_6020_4.rotor_angle    = ((rx_data[0] << 8) | rx_data[1]) / 22.75f; //换为0-360内的角度
			motor_info_6020_4.rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
			motor_info_6020_4.torque_current = ((rx_data[4] << 8) | rx_data[5]);
			motor_info_6020_4.temp           = ((rx_data[6] << 8) | rx_data[7]);
			break;
	}
}

/**
  * @brief  send motor control message through can bus
  * @param  id_range to select can control id 0x1ff or 0x2ff
  * @param  motor voltage 1,2,3,4 or 5,6,7
  * @retval None
  */
void set_motor_voltage(int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
  CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
    
  tx_header.StdId = 0x1ff;
  tx_header.IDE   = CAN_ID_STD;
  tx_header.RTR   = CAN_RTR_DATA;
  tx_header.DLC   = 8;

  tx_data[0] = (v1>>8)&0xff; //电机1
  tx_data[1] =    (v1)&0xff;
  tx_data[2] = (v2>>8)&0xff; //电机2
  tx_data[3] =    (v2)&0xff;

  tx_data[4] = (v3>>8)&0xff; //电机3
  tx_data[5] =    (v3)&0xff;
	
  tx_data[6] = (v4>>8)&0xff; //电机4
  tx_data[7] =    (v4)&0xff;

	HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0); 
}
