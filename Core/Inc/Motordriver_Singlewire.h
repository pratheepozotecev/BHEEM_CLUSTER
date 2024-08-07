/*
 * Motordriver_Singlewire.h
 *
 *  Created on: Mar 20, 2024
 *      Author: LENOVO
 */

#ifndef INC_MOTORDRIVER_SINGLEWIRE_H_
#define INC_MOTORDRIVER_SINGLEWIRE_H_
#include"stdint.h"
#define STORED_DATA_SIZE 12 //Declare an final data stored in array using STORED_DATA_SIZE constant
void processData(void); // It will store the data
extern struct motor_controller // structure for storing the final data in bit field.
{
	uint8_t Device_Code:8;
	uint8_t Sequence_code:8;
	uint8_t Alternate_Bit1:4;
	uint8_t Parking_Indication:1;
	uint8_t Speed_Limit:1;
	uint8_t Alternate_Bit2:1;
	uint8_t Side_Brace_Indication:1;
	uint8_t Pushcart_prohibitedsign:1;
	uint8_t Hall_fault:1;
	uint8_t Throttle_fault:1;
	uint8_t Controller_fault:1;
	uint8_t Under_Voltageprotection:1;
	uint8_t Cruise:1;
	uint8_t Assistance_power:1;
	uint8_t Motor_phase_loss:1;
	uint8_t Four_gear_indication:1;
	uint8_t Motor_Running:1;
	uint8_t brake:1;
	uint8_t Controller_protect:1;
	uint8_t Slide_charging:1;
	uint8_t Antiflying_vehicle_protection:1;
	uint8_t Three_speed:2;
	uint8_t Cloud_powermode:1;
	uint8_t Push_to_talk:1;
	uint8_t Standby_powersupply:1;
	uint8_t Overcurrent_protection:1;
	uint8_t Locked_rotor_protection:1;
	uint8_t Reverse:1;
	uint8_t Electronic_break:1;
	uint8_t Speed_limit:1;
	uint8_t Operating_current:8;
	uint8_t Battery_Level:8;
	uint8_t Current_Level:8;
	uint16_t Speed;
	uint8_t Checksum;
}Motor_Data;


#endif /* INC_MOTORDRIVER_SINGLEWIRE_H_ */
