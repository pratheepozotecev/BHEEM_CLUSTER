
/*
 * DALY_CAN_COM.H
 *
 *  Created on: Dec 18, 2023
 *      Author: LENOVO
 */

#ifndef INC_DALY_CAN_COM_H_
#define INC_DALY_CAN_COM_H_
#define ID_90 0x18904001
#define ID_91 0x18914001
#define ID_92 0x18924001
#define ID_93 0x18934001
#define ID_94 0x18944001
#define ID_95 0x18954001
#define ID_96 0x18964001
#define ID_97 0x18974001
#define ID_98 0x18984001
#define ID_52 0x18524001
#define ID_50 0x18504001
#define ID_01 0x09012024
#define ID_02 0x09021024

void merge(uint32_t);//Declaration
uint8_t Received_Data[8];// Receiving data are stored in this variable
struct ReceivedData// Structure for declaring needed variables
{
  uint16_t Cumulative_Total_Voltage;
  uint16_t Gather_Total_Voltage;
  uint16_t Current;
  uint32_t Battery_capacity;
  int16_t SOC;
  uint16_t Max_Cell_Voltage;
  uint8_t Max_Cell_Voltage_No;
  uint16_t Min_Cell_Voltage;
  uint8_t Min_Cell_Voltage_No;
  uint8_t Max_Temp;
  uint8_t Max_Temp_Sensor_No;
  uint8_t Min_Temp;
  uint8_t Min_Temp_Sensor_No;
  uint8_t Charger_State;
  uint8_t Charge_MOS_State;
  uint8_t Discharge_MOS_State;
  uint8_t BMS_Life;
  uint32_t Capacity;
  uint8_t No_Of_Battery;
  uint8_t No_Of_Temp_Sensor;
  uint8_t Charger_Status;
  uint8_t Load_Status;
  uint8_t DI1state:1;
  uint8_t DI2state:1;
  uint8_t DI3state:1;
  uint8_t DI4state:1;
  uint8_t DO1state:1;
  uint8_t DO2state:1;
  uint8_t DO3state:1;
  uint8_t DO4state:1;
//  uint8_t Voltage_Frame_No;
 uint16_t Frame1_Cell1_Voltage;
//  uint16_t Frame1_Cell2_Voltage;
//  uint16_t Frame1_Cell3_Voltage;
  uint16_t Frame2_Cell1_Voltage;
//  uint16_t Frame2_Cell2_Voltage;
//  uint16_t Frame2_Cell3_Voltage;
//  uint16_t Frame3_Cell1_Voltage;
//  uint16_t Frame3_Cell2_Voltage;
//  uint16_t Frame3_Cell3_Voltage;
//  uint16_t Frame4_Cell1_Voltage;
//  uint16_t Frame4_Cell2_Voltage;
//  uint16_t Frame4_Cell3_Voltage;
//  uint16_t Frame5_Cell1_Voltage;
//  uint16_t Frame5_Cell2_Voltage;
//  uint16_t Frame5_Cell3_Voltage;
   uint8_t Temp_Sensor1;
   uint8_t Temp_Sensor2;
   uint8_t Temp_Sensor3;
   uint8_t Temp_Sensor4;
   uint8_t Cell1_Balance_State:1;
   uint8_t Cell2_Balance_State:1;
   uint8_t Cell3_Balance_State:1;
   uint8_t Cell4_Balance_State:1;
   uint8_t Cell5_Balance_State:1;
   uint8_t Cell6_Balance_State:1;
   uint8_t Cell7_Balance_State:1;
   uint8_t Cell8_Balance_State:1;
   uint8_t Cell9_Balance_State:1;
   uint8_t Cell10_Balance_State:1;
   uint8_t Cell11_Balance_State:1;
   uint8_t Cell12_Balance_State:1;
   uint8_t Cell13_Balance_State:1;
   uint8_t Cell14_Balance_State:1;
   uint8_t Cell15_Balance_State:1;
   uint8_t Cell16_Balance_State:1;
   uint8_t Cell17_Balance_State:1;
   uint8_t Cell18_Balance_State:1;
   uint8_t Cell19_Balance_State:1;
   uint8_t Cell20_Balance_State:1;
   uint8_t Cell21_Balance_State:1;
   uint8_t Cell22_Balance_State:1;
   uint8_t Cell23_Balance_State:1;
   uint8_t Cell24_Balance_State:1;
   uint8_t Cell25_Balance_State:1;
   uint8_t Cell26_Balance_State:1;
   uint8_t Cell27_Balance_State:1;
   uint8_t Cell28_Balance_State:1;
   uint8_t Cell29_Balance_State:1;
   uint8_t Cell30_Balance_State:1;
   uint8_t Cell31_Balance_State:1;
   uint8_t Cell32_Balance_State:1;
   uint8_t Cell33_Balance_State:1;
   uint8_t Cell34_Balance_State:1;
   uint8_t Cell35_Balance_State:1;
   uint8_t Cell36_Balance_State:1;
   uint8_t Cell37_Balance_State:1;
   uint8_t Cell38_Balance_State:1;
   uint8_t Cell39_Balance_State:1;
   uint8_t Cell40_Balance_State:1;
   uint8_t Cell41_Balance_State:1;
   uint8_t Cell42_Balance_State:1;
   uint8_t Cell43_Balance_State:1;
   uint8_t Cell44_Balance_State:1;
   uint8_t Cell45_Balance_State:1;
   uint8_t Cell46_Balance_State:1;
   uint8_t Cell47_Balance_State:1;
   uint8_t Cell48_Balance_State:1;

   uint8_t Cell_Volt_High_Level_1:1;
   uint8_t Cell_Volt_High_Level_2:1;
   uint8_t Cell_Volt_Low_Level_1:1;
   uint8_t Cell_Volt_Low_Level_2:1;
   uint8_t Sum_Volt_High_Level_1:1;
   uint8_t Sum_Volt_High_Level_2:1;
   uint8_t Sum_Volt_Low_Level_1:1;
   uint8_t Sum_Volt_Low_Level_2:1;
   uint8_t Chg_Temp_High_Level_1:1;
   uint8_t Chg_Temp_High_Level_2:1;
   uint8_t Chg_Temp_Low_Level_1:1;
   uint8_t Chg_Temp_Low_Level_2:1;
   uint8_t Dischg_Temp_High_Level_1:1;
   uint8_t Dischg_Temp_High_Level_2:1;
   uint8_t Dischg_Temp_Low_Level_1:1;
   uint8_t Dischg_Temp_Low_Level_2:1;
   uint8_t Chg_Overcurrent_Level_1:1;
   uint8_t Chg_Overcurrent_Level_2:1;
   uint8_t Dischg_Overcurrent_Level_1:1;
   uint8_t Dischg_Overcurrent_Level_2:1;
   uint8_t SOC_High_Level_1:1;
   uint8_t SOC_High_Level_2:1;
   uint8_t SOC_Low_Level_1:1;
   uint8_t SOC_Low_Level_2:1;
   uint8_t Diff_Volt_Level_1:1;
   uint8_t Diff_Volt_Level_2:1;
   uint8_t Diff_Temp_Level_1:1;
   uint8_t Diff_Temp_Level_2:1;
   uint8_t Reserved_4:1;
   uint8_t Reserved_5:1;
   uint8_t Reserved_6:1;
   uint8_t Reserved_7:1;
   uint8_t Chg_MOS_Temp_High_Alarm:1;
   uint8_t Dischg_MOS_Temp_High_Alarm:1;
   uint8_t Chg_MOS_Temp_Sensor_Err:1;
   uint8_t Dischg_MOS_Temp_Sensor_Err:1;
   uint8_t Chg_MOS_Adhesion_Err:1;
   uint8_t Dischg_MOS_Adhesion_Err:1;
   uint8_t Chg_MOS_Open_Circuit_Err:1;
   uint8_t Discrg_MOS_Open_Circuit_Err:1;
   uint8_t AFE_Collect_Chip_Err:1;
   uint8_t Voltage_Collect_Dropped:1;
   uint8_t Cell_Temp_Sensor_Err:1;
   uint8_t EEPROM_Err:1;
   uint8_t RTC_Err:1;
   uint8_t Precharge_Failure:1;
   uint8_t Communication_Failure:1;
   uint8_t Internal_Communication_Failure:1;
   uint8_t Current_Module_Fault:1;
   uint8_t Sum_Voltage_Detect_Fault:1;
   uint8_t Short_Circuit_Protect_Fault:1;
   uint8_t Low_Volt_Forbidden_Chg_Fault:1;
   uint8_t Reserved4:1;
   uint8_t Reserved5:1;
   uint8_t Reserved6:1;
   uint8_t Reserved7:1;
   uint8_t Faultcode;
   uint32_t Cumulative_Charge;

}BMS;

struct conzo
{
	float Gear_ratio;
	uint8_t speed_sensor_type;
}OBD;


#endif /* INC_DALY_CAN_COM_H_ */




