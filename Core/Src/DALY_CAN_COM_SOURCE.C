/*
 * DALY_CAN_COM_SOURCE.C
 *
 *  Created on: Dec 18, 2023
 *      Author: LENOVO
 */
#include"main.h"
#include"DALY_CAN_COM.H"

/*Merging and Splitting function
 * In this function 8 different bytes of data received from BMS according to the identifier request
 * Some data contains 1 of byte and some data contain 2 byte of data
 * Data will be splitting and merging  based on the requirement and stored in a corresponding variable
 * It likely disassembles larger data units into smaller parts or combines smaller parts to form larger data entities.s
 */

void merge(uint32_t Id)
{
switch(Id)
{
case ID_90:
				 BMS.Cumulative_Total_Voltage = (Received_Data[0]<<8)|Received_Data[1];//Cumulative total voltage of BMS
				 BMS.Gather_Total_Voltage = (Received_Data[2]<<8)|Received_Data[3];//Gather total voltage of BMS
				 BMS.Current = ((Received_Data[4]<<8)|Received_Data[5])-30000;//Total Current of BMS
				 BMS.SOC = (Received_Data[6]<<8)|Received_Data[7];//state of charge of BMS
                 break;
case ID_91:
                 BMS.Max_Cell_Voltage = (Received_Data[0]<<8)|Received_Data[1];//Maximum Voltage value among all the cells
			     BMS.Max_Cell_Voltage_No = Received_Data[2];//Maximum voltage cell number among all the cells
			     BMS.Min_Cell_Voltage = (Received_Data[3]<<8)|Received_Data[4];//Minimum voltage value among all the cells
			     BMS.Min_Cell_Voltage_No =  Received_Data[5];//Minimum voltage cell number among all the cells
			     break;
case ID_92:
                 BMS.Max_Temp = (Received_Data[0]);//Maximum temperature sensor value among all the sensor
			     BMS.Max_Temp_Sensor_No = Received_Data[1];//Number of the sensor that has the highest temperature value among the sensors
		         BMS.Min_Temp = (Received_Data[2]);//Minimum temperature value among all the sensors
		         BMS.Min_Temp_Sensor_No = Received_Data[3];//Number of the sensor that has the lowest temperature value among the sensors
                 break;
case ID_93:
                 BMS.Charger_State = Received_Data[0];//0:stationary 1:charge 2:discharge
 		  	     BMS.Charge_MOS_State= Received_Data[1];//0:stationary 1:charge 2:discharge
 		  	     BMS.Discharge_MOS_State= Received_Data[2];//0:stationary 1:charge 2:discharge
 		  		 BMS.BMS_Life = Received_Data[3];//Count of charge-discharge cycles
 		  	     BMS.Capacity = Received_Data[4]|(Received_Data[5]<<8)|(Received_Data[6]<<16)|(Received_Data[7]<<24);//Remaining capacity of BMS
                 break;
case ID_94:
	             BMS.No_Of_Battery = Received_Data[0];//Total number cells connected in BMS
 			     BMS.No_Of_Temp_Sensor = Received_Data[1];//Total Number of temperature sensor connected in BMS
 			     BMS.Charger_Status = Received_Data[2];//0:disconnect,1:connect
 			     BMS.Load_Status = Received_Data[3];//0:disconnect,1:connect
 			     BMS.DI1state = (Received_Data[4] & 0X01)>>0;
 			     BMS.DI2state = (Received_Data[4] & 0X02)>>1;
 		         BMS.DI3state = (Received_Data[4] & 0X04)>>2;
 			     BMS.DI4state = (Received_Data[4] & 0X08)>>3;
 			     BMS.DO1state = (Received_Data[4] & 0X10)>>4;
 			     BMS.DO2state = (Received_Data[4] & 0X20)>>5;
 			     BMS.DO3state = (Received_Data[4] & 0X40)>>6;
 			     BMS.DO4state = (Received_Data[4] & 0X80)>>7;
                 break;
case ID_95:
	             break;
case ID_96:
	             BMS.Temp_Sensor1 = (Received_Data[1])-40;//1st temperature sensor value
	 			 BMS.Temp_Sensor2 = (Received_Data[2])-40;//2nd temperature sensor value
	 			 BMS.Temp_Sensor3 = (Received_Data[3])-40;//3rd temperature sensor value
	 			 BMS.Temp_Sensor4 = (Received_Data[4])-40;//4th temperature sensor value
	 			 break;
case ID_97:
	             BMS.Cell1_Balance_State = (Received_Data[0] & 0x01)>>0;//0： Closed,1： Open
	 		     BMS.Cell2_Balance_State = (Received_Data[0] & 0x02)>>1;
	 			 BMS.Cell3_Balance_State = (Received_Data[0] & 0x04)>>2;
	 			 BMS.Cell4_Balance_State = (Received_Data[0] & 0x08)>>3;
	 			 BMS.Cell5_Balance_State = (Received_Data[0] & 0x10)>>4;
	 			 BMS.Cell6_Balance_State = (Received_Data[0] & 0x20)>>5;
	 			 BMS.Cell7_Balance_State = (Received_Data[0] & 0x40)>>6;
	 			 BMS.Cell8_Balance_State = (Received_Data[0] & 0x80)>>7;
	 			 BMS.Cell9_Balance_State = (Received_Data[1] & 0x01)>>0;
	 			 BMS.Cell10_Balance_State = (Received_Data[1] & 0x02)>>1;
	 			 BMS.Cell11_Balance_State = (Received_Data[1] & 0x04)>>2;
	 			 BMS.Cell12_Balance_State = (Received_Data[1] & 0x08)>>3;
	 			 BMS.Cell13_Balance_State = (Received_Data[1] & 0x10)>>4;
	 			 BMS.Cell14_Balance_State = (Received_Data[1] & 0x20)>>5;
	 			 BMS.Cell15_Balance_State = (Received_Data[1] & 0x40)>>6;
	 			 BMS.Cell16_Balance_State = (Received_Data[1] & 0x80)>>7;
	 			 BMS.Cell17_Balance_State = (Received_Data[2] & 0x01)>>0;
	 			 BMS.Cell18_Balance_State = (Received_Data[2] & 0x02)>>1;
	 			 BMS.Cell19_Balance_State = (Received_Data[2] & 0x04)>>2;
	 			 BMS.Cell20_Balance_State = (Received_Data[2] & 0x08)>>3;
	 			 BMS.Cell21_Balance_State = (Received_Data[2] & 0x10)>>4;
	 			 BMS.Cell22_Balance_State = (Received_Data[2] & 0x20)>>5;
	 			 BMS.Cell23_Balance_State = (Received_Data[2] & 0x40)>>6;
	 			 BMS.Cell24_Balance_State = (Received_Data[2] & 0x80)>>7;
	 			 BMS.Cell25_Balance_State = (Received_Data[3] & 0x01)>>0;
	 			 BMS.Cell26_Balance_State = (Received_Data[3] & 0x02)>>1;
	 			 BMS.Cell27_Balance_State = (Received_Data[3] & 0x04)>>2;
	 			 BMS.Cell28_Balance_State = (Received_Data[3] & 0x08)>>3;
	 			 BMS.Cell29_Balance_State = (Received_Data[3] & 0x10)>>4;
	 			 BMS.Cell30_Balance_State = (Received_Data[3] & 0x20)>>5;
	 			 BMS.Cell31_Balance_State = (Received_Data[3] & 0x40)>>6;
	 			 BMS.Cell32_Balance_State = (Received_Data[3] & 0x80)>>7;
	 			 BMS.Cell33_Balance_State = (Received_Data[4] & 0x01)>>0;
	 			 BMS.Cell34_Balance_State = (Received_Data[4] & 0x02)>>1;
	 			 BMS.Cell35_Balance_State = (Received_Data[4] & 0x04)>>2;
	 			 BMS.Cell36_Balance_State = (Received_Data[4] & 0x08)>>3;
	 			 BMS.Cell37_Balance_State = (Received_Data[4] & 0x10)>>4;
	 			 BMS.Cell38_Balance_State = (Received_Data[4] & 0x20)>>5;
	 			 BMS.Cell39_Balance_State = (Received_Data[4] & 0x40)>>6;
	 			 BMS.Cell40_Balance_State = (Received_Data[4] & 0x80)>>7;
	 			 BMS.Cell41_Balance_State = (Received_Data[5] & 0x01)>>0;
	 			 BMS.Cell42_Balance_State = (Received_Data[5] & 0x02)>>1;
	 			 BMS.Cell43_Balance_State = (Received_Data[5] & 0x04)>>2;
	 			 BMS.Cell44_Balance_State = (Received_Data[5] & 0x08)>>3;
	 			 BMS.Cell45_Balance_State = (Received_Data[5] & 0x10)>>4;
	 			 BMS.Cell46_Balance_State = (Received_Data[5] & 0x20)>>5;
	 			 BMS.Cell47_Balance_State = (Received_Data[5] & 0x40)>>6;
	 			 BMS.Cell48_Balance_State = (Received_Data[5] & 0x80)>>7;
	 			 break;

case ID_98:
	             BMS.Cell_Volt_High_Level_1 = (Received_Data[0] & 0x01)>>0;//0:No error,1:Error
	 		     BMS.Cell_Volt_High_Level_2 = (Received_Data[0] & 0x02)>>1;
	 	         BMS.Cell_Volt_Low_Level_1 = (Received_Data[0] & 0x04)>>2;
	 		     BMS.Cell_Volt_Low_Level_2 = (Received_Data[0] & 0x08)>>3;
	 		     BMS.Sum_Volt_High_Level_1 = (Received_Data[0] & 0x10)>>4;
	 			 BMS.Sum_Volt_High_Level_2 = (Received_Data[0] & 0x20)>>5;
	 			 BMS.Sum_Volt_Low_Level_1 = (Received_Data[0] & 0x40)>>6;
	 			 BMS.Sum_Volt_Low_Level_2 = (Received_Data[0] & 0x80)>>7;
	 			 BMS.Chg_Temp_High_Level_1 = (Received_Data[1] & 0x01)>>0;
	 			 BMS.Chg_Temp_High_Level_2 = (Received_Data[1] & 0x02)>>1;
	 		     BMS.Chg_Temp_Low_Level_1 = (Received_Data[1] & 0x04)>>2;
	 		     BMS.Chg_Temp_Low_Level_2 = (Received_Data[1] & 0x08)>>3;
	 		     BMS.Dischg_Temp_High_Level_1 = (Received_Data[1] & 0x10)>>4;
	 			 BMS.Dischg_Temp_High_Level_2 = (Received_Data[1] & 0x20)>>5;
	 			 BMS.Dischg_Temp_Low_Level_1 = (Received_Data[1] & 0x40)>>6;
	 			 BMS.Dischg_Temp_Low_Level_2 = (Received_Data[1] & 0x80)>>7;
	 			 BMS.Chg_Overcurrent_Level_1 = (Received_Data[2] & 0x01)>>0;
	 		     BMS.Chg_Overcurrent_Level_2 = (Received_Data[2] & 0x02)>>1;
	 			 BMS.Dischg_Overcurrent_Level_1 = (Received_Data[2] & 0x04)>>2;
	 			 BMS.Dischg_Overcurrent_Level_2 = (Received_Data[2] & 0x08)>>3;
	 		     BMS.SOC_High_Level_1 = (Received_Data[2] & 0x10)>>4;
	 		     BMS.SOC_High_Level_2 = (Received_Data[2] & 0x20)>>5;
	 		     BMS.SOC_Low_Level_1 = (Received_Data[2] & 0x40)>>6;
	 			 BMS.SOC_Low_Level_2 = (Received_Data[2] & 0x80)>>7;
	 			 BMS.Diff_Volt_Level_1 = (Received_Data[3] & 0x01)>>0;
	 			 BMS.Diff_Volt_Level_2 = (Received_Data[3] & 0x02)>>1;
	 			 BMS.Diff_Temp_Level_1 = (Received_Data[3] & 0x04)>>2;
	 			 BMS.Diff_Temp_Level_2 = (Received_Data[3] & 0x08)>>3;
	 			 BMS.Chg_MOS_Temp_High_Alarm = (Received_Data[4] & 0x01)>>0;//Alarm condition related to a high temperature in the charging MOSFET
	 			 BMS.Dischg_MOS_Temp_High_Alarm = (Received_Data[4] & 0x02)>>1;//Alarm condition related to a high temperature in the discharging MOSFET
	 		     BMS.Chg_MOS_Temp_Sensor_Err = (Received_Data[4] & 0x04)>>2;//Error related to the temperature sensor(s) monitoring the charging MOSFET
	 			 BMS.Dischg_MOS_Temp_Sensor_Err = (Received_Data[4] & 0x08)>>3;//Error related to the temperature sensor(s) monitoring the discharging MOSFET
	 			 BMS.Chg_MOS_Adhesion_Err = (Received_Data[4] & 0x10)>>4;//Fault condition associated with the adhesion or attachment of components related to the Charging MOSFET
	 			 BMS.Dischg_MOS_Adhesion_Err = (Received_Data[4] & 0x20)>>5;//Fault condition associated with the adhesion or attachment of components related to the discharging MOSFET
	 		     BMS.Chg_MOS_Open_Circuit_Err = (Received_Data[4] & 0x40)>>6;//Error condition associated with an open circuit or an open-circuit fault detected in the Charging MOSFET
	 			 BMS.Discrg_MOS_Open_Circuit_Err = (Received_Data[4] & 0x80)>>7;//Error condition associated with an open circuit or an open-circuit fault detected in the discharging MOSFET
	 			 BMS.AFE_Collect_Chip_Err = (Received_Data[5] & 0x01)>>0;//Error condition associated with the Analog Front-End (AFE) chip or circuitry responsible for collecting and processing analog signals from various sensors or measurement points within the battery system
	 			 BMS.Voltage_Collect_Dropped = (Received_Data[5] & 0x02)>>1;//Alert, condition, or indication within the BMS that detects a significant and sudden drop in the collected voltage readings from sensors
	 			 BMS.Cell_Temp_Sensor_Err = (Received_Data[5] & 0x04)>>2;//Error or fault condition associated with temperature sensors
	 			 BMS.EEPROM_Err = (Received_Data[5] & 0x08)>>3;//Error condition or fault related to the Electrically Erasable Programmable Read-Only Memory (EEPROM) used within the BMS circuitry or microcontroller.
	 			 BMS.RTC_Err = (Received_Data[5] & 0x10)>>4;//Error condition related to the Real-Time Clock (RTC) component or module within the BMS.
	 			 BMS.Precharge_Failure = (Received_Data[5] & 0x20)>>5;//fault where the precharging process within the battery system has encountered
	 			 BMS.Communication_Failure = (Received_Data[5] & 0x40)>>6;//disruptions in communication between various components, modules, or external devices within the battery system.
	 			 BMS.Internal_Communication_Failure = (Received_Data[5] & 0x80)>>7;//disruptions in the internal communication between various components, modules, or subsystems within the BMS itself.
	 			 BMS.Current_Module_Fault = (Received_Data[6] & 0x01)>>0;//Fault condition related to the current measurement module within the BMS.
	 			 BMS.Sum_Voltage_Detect_Fault = (Received_Data[6] & 0x02)>>1;//Fault condition related to the detection or measurement of the total sum voltage across the battery cells or modules within the system.
	 			 BMS.Short_Circuit_Protect_Fault = (Received_Data[6] & 0x04)>>2;//fault condition related to the protective mechanisms within the BMS that are designed to detect and respond to short-circuit events occurring within the battery system.
	 			 BMS.Low_Volt_Forbidden_Chg_Fault = (Received_Data[6] & 0x08)>>3;// fault condition where charging is forbidden due to low voltage detected in the battery cells or pack, preventing the charging process from initiating or continuing.
	 			 BMS.Faultcode = Received_Data[7];//Fault code that represents an error, fault, or abnormal condition detected by the BMS
	 			 break;

case ID_52:
	             BMS.Cumulative_Charge = ((Received_Data[1]<<16)|(Received_Data[2]<<8)|(Received_Data[3]));//The total amount of charge that has been delivered or supplied to a battery
	             break;

case ID_50:
	             BMS.Battery_capacity = (((Received_Data[2]<<8)|Received_Data[3])/1000);//The battery rated capacity in Ah eg(58 Ah)
	             break;

default:

	break;

 }
 }








