/* USER CODE BEGIN Header */
/** to ceck pull in git
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "Motordriver_Singlewire.h"
#include "Daly_can_com.h"
#include "Daly_can_com_source.c"
#include "FLASH_PAGE_F1.h"
#define EEPROM_DEV_ADD 0xA0 // EEPROM 7-bit address
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define  calib_reg 440//1656-165;
#define poles 24  //6*4
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
volatile uint8_t syncFlag = 0; // Whenever this flag is set the External Interrupt Will Disable, Reset External Interrupt Will Enable
volatile uint16_t signalCounter; //It increments the External Interrupt detection stored in dataArray.
volatile uint16_t timerCounter = 0; //It will incremented every timer interrupt occur with time.
volatile uint16_t dataArray[400]; //It stores whenever External Interrupt detected that corresponding time store in the timerCounter Variable.
volatile uint8_t validDataFlag; //If the checksum is equal to the received data then it will be stored 1 else it will 0.
uint16_t m_sec=0,lop=0;
uint8_t Tx_count=0;//This variable is for loop iteration
uint8_t Transmit_Data[8]; //Declaration of an array capable of holding one byte of transmit data
uint32_t Rx_Id; //Temporary variable for storing received identifier
uint32_t TxMailBox; // This variable is used to  transmitting data and acts like a message buffer
uint32_t speed_count=0,next_update=500000;
uint32_t speed_count_temp=0;
uint8_t after_sec=0;
uint8_t Odo_Write[3];//Array to store odo-meter values for the purpose of writing data
uint8_t data_Read[3];//Array to store the read odo-meter data retrieved from EEPROM
uint32_t Odo_Value[10],Odo_Value_1[10];
uint8_t Km_Flag=1;
uint8_t first_time=1;
uint8_t ODO_SEC=1;
uint8_t sensor_change=0;
uint8_t trip_reset_state=0;
int16_t Reserved_SOC=0;
uint16_t can_delay=500;
uint8_t can_state=0;
uint8_t print_pass=0;
uint8_t Battery_high_Temp=0;
uint16_t print_delay=2000;
uint8_t print_state=1;
uint8_t bat_icon_toogle=0;
uint8_t dataArray1[12];
uint8_t checksum_error;
uint16_t SYC_DATA=0;
uint8_t Pin_State;
uint16_t LBD_value;
uint8_t LBD_State=0;
uint32_t CGC_value;
uint32_t last_flash_update,read_flash_update;
uint8_t EEPROM_ADDRESS[10]={0x10,0x20,0x30,0x40,0x50,0x60,0x70,0x80,0x90,0xA0};
uint8_t TEMP_SP=0;

uint16_t Speed_cal=0;
uint8_t sensor_type=0;
//uint8_t syncFlag = 0; // Whenever this flag is set the External Interrupt Will Disable, Reset External Interrupt Will Enable
//uint8_t signalCounter=0; //It increments bit position for data bit by bit storing.
uint8_t timerCounter1 = 0; //It will incremented every timer interrupt occur with time.
uint8_t timerCounter2 = 0;
uint16_t timerCounter3 = 0;
struct motor_controller Motor_Data;
struct Speed //structure declaration
{
	uint32_t Odometer_Value;//structure members
	uint32_t Odometer_Value_temp;
	uint32_t Ref;
}Range;
void Split(uint32_t);//Function declaration
void Merge();//Function declaration
//Array declaration
uint8_t data_Write[3];
uint8_t data_Read[3];
uint16_t calib_write[2];
uint16_t calib_read[2];
uint8_t ADR_LOC=0;
uint8_t test_bit = 0;
CAN_TxHeaderTypeDef TxHeader;// Adding structure variable for accessing Transmitter frame format element Eg: StdId,RTR..
CAN_RxHeaderTypeDef RxHeader;// Adding Structure variable for accessing Receiver frame format element

uint16_t Bike_speed;
unsigned int BMS_ID[]={0X18900140,0X18910140, 0X18920140,0X18930140,0X18940140,0X18950140,0X18960140,0X18970140,0X18980140,0X18520140,0X18500140,0x09011024};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void EEPROM_Write(uint16_t address, uint8_t* data, uint16_t size);//Write Function Declaration
void EEPROM_Read(uint16_t address, uint8_t* data, uint16_t size);//Read Function Declaration
void CAN_BMS(void);//Declaration

////10 different type of Extended Identifier stored in a array

///*Transmitter Function
// * In this function 10 different types of identifier throw on the CAN bus in a particular time delay
// * for Receiving response from BMS according to the Identifier request
// * Transmit data should be given to the mailbox
// */

void parllel_transmit(uint8_t data)
{
	GPIOA->ODR=0X00|data;
}

void Split(uint32_t Odo_Value)
{
	data_Write[0]= Odo_Value & 0xFF;
	data_Write[1]= (Odo_Value>>8) & 0xFF;
	data_Write[2]= (Odo_Value>>16) & 0xFF;
}
/*
 * This function is used to merging data which is read from EEPROM
 * Merging individual byte of data and stored in a Structure
 */


void I2C_Write_EEPROM(uint32_t data_eeprom,uint8_t address)
{
   Split(data_eeprom);
   EEPROM_Write(address, data_Write, sizeof(data_Write));//Write odo-meter data
}

int I2C_Read_EEPROM(uint8_t address)
{
	EEPROM_Read(address, data_Read, sizeof(data_Read));
	return((data_Read[2] << 16) | (data_Read[1] << 8) | (data_Read[0]));
}

/*
 * This function is used to write the data in EEPROM
 * This function contains Target device address,Internal Memory address
 * Size of internal memory address,Data buffer for storing write data ,
 * Size Amount of data to be sent and Timeout Timeout duration
 */
void EEPROM_Write(uint16_t address, uint8_t* data, uint16_t size)
{
  HAL_I2C_Mem_Write(&hi2c1, EEPROM_DEV_ADD, address, I2C_MEMADD_SIZE_8BIT, data, size, HAL_MAX_DELAY);
  HAL_Delay(5); // Delay for EEPROM write operation
}
/*
 * This function is used to read the data from EEPROM
 * This function contains Target device address,Internal Memory address
 * Size of internal memory address,Data Buffer for storing read data,
 * Size Amount of data to be read and Timeout Timeout duration
 */
void EEPROM_Read(uint16_t address, uint8_t* data, uint16_t size)
{
  HAL_I2C_Mem_Read(&hi2c1, EEPROM_DEV_ADD, address, I2C_MEMADD_SIZE_8BIT, data, size, HAL_MAX_DELAY);
  HAL_Delay(5); // Delay for EEPROM read operation
}

void battery_voltage()
{
	lcd_clear(7, 0, 12);
	lcd_print_digit_wos(7, 0, (BMS.Cumulative_Total_Voltage/100));
	lcd_print_digit_wos(7, 6, (BMS.Cumulative_Total_Voltage/10)%10);
	lcd_print_char(7, 12, "V");
}

void battery_current()
{
	lcd_clear(7, 110, 12);
	lcd_print_digit_wos(7, 110, (BMS.Current/100));
	lcd_print_digit_wos(7, 116, (BMS.Current/10)%10);
	lcd_print_char(7, 122, "A");
}
void battery_cycle()
{
	charge_cycle_print();
	CGC_value=BMS.Cumulative_Charge/BMS.Battery_capacity;
 	lcd_print_digit_wos(3, 103, (CGC_value)/1000);
	lcd_print_digit_wos(3, 109, ((CGC_value)%1000)/100);
	lcd_print_digit_wos(3, 115, ((((CGC_value)%1000)%100)/10));
	lcd_print_digit_wos(3, 121, ((((CGC_value)%1000)%100)%10));
}



uint8_t tog_temp=0;
void battery_temp()
{

	if((BMS.Max_Temp==0)&&(BMS.Min_Temp==0))
	{
		lcd_print_digit(0, 0, BMS.Max_Temp);
		lcd_print_convert(0, 12, 0x03);
		lcd_print_convert(0, 13, 0x03);
		lcd_print_char(0, 15, "C");
	}
	else
	{
		if(BMS.Max_Temp>50)
		{
			lcd_print_digit(0, 0, ((BMS.Max_Temp)-40));
			lcd_print_convert(0, 12, 0x03);
			lcd_print_convert(0, 13, 0x03);
			lcd_print_char(0, 15, "C");
		}
		else
		{
			if((BMS.Min_Temp<=49)&&(BMS.Min_Temp>=40))// less tham 10 degree min temp will be printed
			{
				lcd_print_digit(0, 0, ((BMS.Min_Temp)-40));//
				lcd_print_convert(0, 12, 0x03);
				lcd_print_convert(0, 13, 0x03);
				lcd_print_char(0, 15, "C");
			}

			if((BMS.Min_Temp<=39)&&(BMS.Min_Temp>=31))// less tham 10 degree min temp will be printed
			{
				lcd_print_convert(0, 0, 0x10);
				lcd_print_convert(0, 1, 0x10);
				lcd_print_convert(0, 2, 0x10);
				lcd_print_convert(0, 3, 0x10);
				lcd_print_digit(0, 5, 40-((BMS.Min_Temp)));//
				lcd_print_convert(0, 11, 0x03);
				lcd_print_convert(0, 12, 0x03);
				lcd_print_char(0, 14, "C");
			}
			if((BMS.Min_Temp<=30)&&(BMS.Min_Temp>=0))// less tham 10 degree min temp will be printed
			{
				lcd_print_convert(0, 0, 0x10);
				lcd_print_convert(0, 1, 0x10);
				lcd_print_convert(0, 2, 0x10);
				lcd_print_convert(0, 3, 0x10);
				lcd_print_digit(0, 5, 40-((BMS.Min_Temp)));//
				lcd_print_convert(0, 17, 0x03);
				lcd_print_convert(0, 18, 0x03);
				lcd_print_char(0, 20, "C");
			}
		}
	}

	// negative value should be printer draw (-)

	if(BMS.Max_Temp>95)
	{
		tog_temp=(!tog_temp);

		if(tog_temp)
		{
			Battery_high_Temp=1;
			over_temperature_print();
		}
		else
		{
			lcd_clear(1, 25, 13);
			lcd_clear(2, 25, 13);
		}
	}

	if(BMS.Max_Temp<90)
	{
		lcd_clear(1, 25, 12);
		lcd_clear(2, 25, 13);
		Battery_high_Temp=0;
	}

	if((BMS.Max_Temp==0)&&(BMS.Min_Temp==0))
	{
		lcd_print_digit_wos(0, 0, 0);
		lcd_print_digit_wos(0, 6, 0);
	}
}
uint8_t neg_soc=0;
void battery_soc()
{
	if(BMS.SOC>=150)
	{
		neg_soc=0;
		Reserved_SOC=(int)1000.0-((1000.0-BMS.SOC)*(100.0/85.0));
	}
	else
	{
		neg_soc=1;
		Reserved_SOC=150-BMS.SOC;
	}

	if(Reserved_SOC==1000){
		lcd_clear(7, 98, 29);
		lcd_print_digit_wos(7, 104, 1);
		lcd_print_digit_wos(7, 110, 0);
		lcd_print_digit_wos(7, 116, 0);
		lcd_print_char(7, 122, "%");
	}
	else{
		lcd_clear(7, 98, 29);
		if(neg_soc)
		{
			//lcd_print_convert(7, 104, 0x10);
			lcd_print_convert(7, 105, 0x10);
			lcd_print_convert(7, 106, 0x10);
			lcd_print_convert(7, 107, 0x10);
			lcd_print_convert(7, 108, 0x10);
		}
		lcd_print_digit_wos(7, 110, (Reserved_SOC/100));
		lcd_print_digit_wos(7, 116, (Reserved_SOC/10)%10);
		lcd_print_char(7, 122, "%");
	}
	battery_bar_soc();
}
uint8_t start_inc=0;uint8_t Reverse_status=0;uint16_t DTE=0;
void Gear_Status()
{

	if(HAL_GPIO_ReadPin(GPIOB, Reverse_state_Pin)==1)
		{
			gear_status_print(0); // REVERSE
			Reverse_status=1;
		}
	else
		{
			Reverse_status=0;
			if(Motor_Data.Device_Code==8)
			{
				gear_status_print(Motor_Data.Three_speed);

				if(Motor_Data.Three_speed==1)
				{
					DTE=(Reserved_SOC*1.4)/10.0;
				}
				if(Motor_Data.Three_speed==2)
				{
					DTE=(Reserved_SOC*1.2)/10.0;
				}
				if(Motor_Data.Three_speed==3)
				{
					DTE=(Reserved_SOC*1)/10.0;
				}
			}

			if(Motor_Data.Device_Code==13)
			{
				if(Motor_Data.Three_speed==3)
				{
					gear_status_print(1);
					DTE=(Reserved_SOC*1.4)/10.0;
				}
				if(Motor_Data.Three_speed==0)
				{
					gear_status_print(2);
					DTE=(Reserved_SOC*1.2)/10.0;
				}
				if(Motor_Data.Three_speed==1)
				{
					gear_status_print(3);
					DTE=(Reserved_SOC*1)/10.0;
				}
			}

//				if((HAL_GPIO_ReadPin(GPIOB, Gear_1_Pin)==1)&&((HAL_GPIO_ReadPin(GPIOB, Gear_3_Pin)==0)))
//					{
//						Gear_State=3;
//						DTE=(Reserved_SOC*1)/10.0;
//					}
//				if((HAL_GPIO_ReadPin(GPIOB, Gear_1_Pin)==0)&&((HAL_GPIO_ReadPin(GPIOB, Gear_3_Pin)==0)))
//					{
//						Gear_State=2;
//						DTE=(Reserved_SOC*1.2)/10.0;
//					}
//				if((HAL_GPIO_ReadPin(GPIOB, Gear_1_Pin)==0)&&((HAL_GPIO_ReadPin(GPIOB, Gear_3_Pin)==1)))
//					{
//						Gear_State=1;
//						DTE=(Reserved_SOC*1.4)/10.0;
//					}

		}
	if(BMS.SOC<150)
	{
		DTE=0;
	}
}

void EEPROM_init()
{
	I2C_Write_EEPROM(0, 0x00);
  Range.Ref=I2C_Read_EEPROM(ODO_ADDRESS_Ref);
	 if(Range.Ref!=1234)
	 {
		 I2C_Write_EEPROM(7000, gear_ratio_EEPROM);
		 I2C_Write_EEPROM(1, speed_sensor_type_EEPROM);
		 I2C_Write_EEPROM(1234, ODO_ADDRESS_Ref);
		 I2C_Write_EEPROM(0, last_flash_update_EEPROM);
		 for(uint8_t inf=0;inf<=9;inf++)  //Read data from EEPROM
		 {
			I2C_Write_EEPROM(0,EEPROM_ADDRESS[inf]);
		 }
		 HAL_Delay(50);
	 }
	 else
	 {
		 last_flash_update=I2C_Read_EEPROM(last_flash_update_EEPROM); // read last flash write data
		 OBD.speed_sensor_type=I2C_Read_EEPROM(speed_sensor_type_EEPROM);
		 OBD.Gear_ratio=(float)((I2C_Read_EEPROM(gear_ratio_EEPROM))/1000.0);

		 for(uint8_t inf=0;inf<=9;inf++)  //Read data from EEPROM
		 {
			Odo_Value_1[inf]=Odo_Value[inf]=I2C_Read_EEPROM(EEPROM_ADDRESS[inf]);
		 }

		 for (uint8_t i = 1; i < 10; i++)
		 {
			if (Odo_Value[i] < Odo_Value[ADR_LOC])
			{
				ADR_LOC= i;
			}
		 }

		uint32_t temp=0;
		 for (uint8_t i = 0; i < 10; i++)
		 {
			 for (uint8_t j = 0; j < 9-i; j++)
			 {
				 if (Odo_Value[j] > Odo_Value[j+1])
				 {
					 // Swap arr[j] and arr[j+1]
					 temp = Odo_Value[j];
					 Odo_Value[j] = Odo_Value[j+1];
					 Odo_Value[j+1] = temp;
				 }
			 }
		 }
		 uint8_t check_temp=0;
		for(uint8_t i=0;i<=9;i++)
		{
			if((last_flash_update<=Odo_Value[i])&&((last_flash_update+1000)>=Odo_Value[i]))
			{
				Range.Odometer_Value=Odo_Value[i];
				check_temp++;
			}
		}

		if(check_temp==0)
		{
			Range.Odometer_Value=last_flash_update;
		}
	 }
	 first_time=3;
}



uint8_t can_error=0,can_error_state=0,error_count=0;
void BMS_CAN()//Transmitter function
{
	for(Tx_count=0; Tx_count<12; Tx_count++)
	{
		TxHeader.ExtId = BMS_ID[Tx_count]; // Extended Identifier
		TxHeader.IDE = CAN_ID_EXT; // Identifier Extension
		TxHeader.RTR = CAN_RTR_DATA;// Remote Transmission Request bit, here send data frame
		TxHeader.DLC = 8;//Data length code
		Transmit_Data[Tx_count]=0x00;//Data

		if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, &Transmit_Data[Tx_count], &TxMailBox) != HAL_OK)//Adding data to the mailbox for transmitting
		{
			//Error_Handler();
		}
		HAL_Delay(50);
	}
		if(can_error_state==0) // Can Error finder
		{
			if(can_error)
			{
				error_count++;
			}
			if(error_count>=10)
			{
				can_error_state=1;
			}
		}
		else
		{
			lcd_print_char(3, 85, "Batt");
			lcd_print_char(3, 110, "Err");
			BMS.SOC=BMS.Max_Temp=BMS.Min_Temp=BMS.Cumulative_Total_Voltage=0;
		}
		can_error=1;
	Tx_count=0;
}

uint8_t temp_bat=0;
void battery_bar_soc()
{

	if((Reserved_SOC>=950)&&(Reserved_SOC<=1000))
	{
		battery_bar_print(5);
	}
	else if((Reserved_SOC>=800)&&(Reserved_SOC<=949))
	{
		battery_bar_print(4);
	}
	else if((Reserved_SOC>=600)&&(Reserved_SOC<=799))
	{
		battery_bar_print(3);
	}
	else if((Reserved_SOC>=400)&&(Reserved_SOC<=599))
	{
		battery_bar_print(2);
	}
	else if((Reserved_SOC>=200)&&(Reserved_SOC<=399))
	{
		battery_bar_print(1);
	}
	else if((Reserved_SOC>=0)&&(Reserved_SOC<=199))
	{
		bat_icon_toogle=!bat_icon_toogle;

		if(bat_icon_toogle){
			battery_bar_print(0);
		}
		else{

			line_print();
		}
	}
	else
	{
		battery_bar_print(0);
	}
}
uint8_t rx=0;
uint8_t can_gear_req=0;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)//Receiver Interrupt Function
{
 	if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, Received_Data) == HAL_OK)//Receiving data through FIFO
 	{
 		Rx_Id = RxHeader.ExtId;
 		merge(Rx_Id);// Implementation of merging and splitting received BMS data
 		can_error_state=0;
 		can_error=0;
 		error_count=0;
 		if((Rx_Id==0x09021024)||(Rx_Id==0x09031024))
 		{
 			rx=1;
 		}
 	}
}

void Tx_fun(uint32_t TX_data)
{

		switch (TX_data)
		{
			case IDT_02:
			{
				TxHeader.ExtId =0x09022024; // Extended Identifier
				TxHeader.IDE = CAN_ID_EXT; // Identifier Extension
				TxHeader.RTR = CAN_RTR_DATA;// Remote Transmission Request bit, here send data frame
				TxHeader.DLC = 8;//Data length code
				Transmit_Data[0]=0x02;//Flio->1 Bheem->2
				Transmit_Data[1]=((Range.Odometer_Value&0xff000000)>>24);//Data
				Transmit_Data[2]=((Range.Odometer_Value&0x00ff0000)>>16);//Data
				Transmit_Data[3]=((Range.Odometer_Value&0x0000ff00)>>8);//Data
				Transmit_Data[4]=((Range.Odometer_Value&0x000000ff));//Data
				Transmit_Data[5]=((CGC_value&0xff00)>>8);//Data
				Transmit_Data[6]=((CGC_value&0x00ff)>>0);;//Data
				Transmit_Data[7]=(Reserved_SOC/10);//Data

				if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, &Transmit_Data[Tx_count], &TxMailBox) != HAL_OK)//Adding data to the mailbox for transmitting
				{
				//Error_Handler();
				}
				break;
			}
			case IDT_03:
			{

				TxHeader.ExtId =0x09032024; // Extended Identifier
				TxHeader.IDE = CAN_ID_EXT; // Identifier Extension
				TxHeader.RTR = CAN_RTR_DATA;// Remote Transmission Request bit, here send data frame
				TxHeader.DLC = 8;//Data length code
				Transmit_Data[0]=OBD.speed_sensor_type;//Front->1 back->2
				Transmit_Data[1]=(((uint32_t)OBD.Gear_ratio&0xff000000)>>24);//Data
				Transmit_Data[2]=(((uint32_t)OBD.Gear_ratio&0x00ff0000)>>16);//Data
				Transmit_Data[3]=(((uint32_t)OBD.Gear_ratio&0x0000ff00)>>8);//Data
				Transmit_Data[4]=(((uint32_t)OBD.Gear_ratio&0x000000ff));//Data
				Transmit_Data[5]=0;
				Transmit_Data[6]=0;
				Transmit_Data[7]=0;
				if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, &Transmit_Data[Tx_count], &TxMailBox) != HAL_OK)//Adding data to the mailbox for transmitting
				{
					//Error_Handler();
				}
				break;
			}
			default:
					break;
		}
		rx=0;
	}

uint16_t pluse_count=0;uint8_t bike_speed;
float speed_result_float=0;
void ODO_PRINT()
{
	if(OBD.speed_sensor_type==1)
	{
		lcd_speed(bike_speed,OBD.speed_sensor_type);
	}
	else
	{
		lcd_speed((int)speed_result_float,OBD.speed_sensor_type);
	}

	lcd_clear(0, 96, 51);
	uint8_t first_1=DTE/100;
	uint8_t second_1=((DTE%100)/10);
	uint8_t third_1=((DTE%100)%10);
	dte_icon_print();
	lcd_print_digit_wos(0, 98,first_1);
	lcd_print_digit_wos(0, 104,second_1);
	lcd_print_digit_wos(0, 110,third_1);
	lcd_print_char(0, 116, "km");

	uint8_t first = (Range.Odometer_Value / 1000000);
	uint8_t second = ((Range.Odometer_Value % 1000000) / 100000);
	uint8_t third = ((Range.Odometer_Value % 100000) / 10000);
	uint8_t fourth = ((Range.Odometer_Value % 10000) / 1000);
	uint8_t fifth = ((Range.Odometer_Value % 1000) / 100);
	uint8_t sixth = ((Range.Odometer_Value % 100) / 10);
	uint8_t seventh = (Range.Odometer_Value % 10);

	lcd_clear(5, 32, 64);
	odo_icon_print();
	lcd_print_digit_wos(5, 50,first);
	lcd_print_digit_wos(5, 56,second);
	lcd_print_digit_wos(5, 62,third);
	lcd_print_digit_wos(5, 68,fourth);
	lcd_print_digit_wos(5, 74,fifth);
	lcd_print_digit_wos(5, 80,sixth);
	lcd_print_char(5,87, "km");

}
uint16_t speed_temp1=0;
uint32_t controller_speed_data=0;
uint16_t meter_count=0,meter_sec=0;

void ODO_calculation()
{
	if(after_sec)
	{
		if(OBD.speed_sensor_type==1)
		{
			bike_speed=((speed_count*360)/calib_reg);
			pluse_count+=speed_count;
			after_sec=0;
		}
		else
		{
			speed_result_float =(((((float)controller_speed_data * 12305)/poles)/1000)/OBD.Gear_ratio); // ((((N*7200*D*Pi)/P)/1000)/gear_ratio)
			meter_sec=(int)((speed_result_float)*0.2777);    //m/sec  1000/3600
			meter_count=meter_count+meter_sec;
		}
	}

	if((pluse_count>=calib_reg)||(meter_count>=100))
	{
		pluse_count=meter_count=0;
		Range.Odometer_Value++;
		I2C_Write_EEPROM(Range.Odometer_Value,EEPROM_ADDRESS[ADR_LOC]);
		Odo_Value[ADR_LOC]=Range.Odometer_Value_temp=I2C_Read_EEPROM(EEPROM_ADDRESS[ADR_LOC]);
		if(Range.Odometer_Value==Range.Odometer_Value_temp)
		{
			ADR_LOC++;
		}
		else
		{
			ADR_LOC++;
			if(ADR_LOC>=10)
			{
				ADR_LOC=0;
			}
			I2C_Write_EEPROM(Range.Odometer_Value,EEPROM_ADDRESS[ADR_LOC]);
			ADR_LOC++;
		}
		if(ADR_LOC>=10)
		{
			ADR_LOC=0;
			if((last_flash_update<=Range.Odometer_Value)&&((last_flash_update+1)>=Range.Odometer_Value))
			{
				Range.Odometer_Value=last_flash_update;
			}
		}
	}
}
uint8_t cmd_rx;
uint16_t speed_temp=0,Controller_Speed=0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  PVD_Init();
  HAL_I2C_Init(&hi2c1);
  HAL_CAN_Start(&hcan);// CAN protocol enable function
  HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING); // Interrupt activation for Receiving data ,whenever data is received in FIFO, this function will get triggered and goes to receiver interrupt function
  HAL_TIM_Base_Start_IT(&htim2); // Timer2 Interrupt Start
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  lcd_init();
     if(RCC->CSR&RCC_CSR_IWDGRSTF)
     {
    	 RCC->CSR|=1<<24;
     }
     else
     {
		lcd_into();
		HAL_Delay(500);
		lcd_clear(0, 0, 127);
		lcd_clear(1, 0, 127);
		lcd_clear(2, 0, 127);
		lcd_clear(3, 0, 127);
		lcd_clear(4, 0, 127);
		lcd_clear(5, 0, 127);
		lcd_clear(6, 0, 127);
		lcd_clear(7, 0, 127);
     }
     EEPROM_init();
	 MX_IWDG_Init();
 while(1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if(print_state)
	  {
		print_state=0;
		lcd_clear(0, 0, 127);
		lcd_clear(1, 0, 127);
		lcd_clear(2, 0, 127);
		lcd_clear(3, 0, 127);
		lcd_clear(4, 0, 127);
		lcd_clear(5, 0, 127);
		lcd_clear(6, 0, 127);
		lcd_clear(7, 0, 127);
		BMS_CAN();// read data from the BMS through the can protocol
		if(first_time)
		{
		   battery_cycle();
		   first_time--;
		}
		Gear_Status();
		battery_temp();
		line_print();
		battery_soc();
		ODO_calculation();
		ODO_PRINT();
		battery_voltage();
		Lcd_cmd(0xA2); // ADC select
		Lcd_cmd(0xA0);// SHL select
		Lcd_cmd(0xC0);// Initial display line
		Lcd_cmd(0x40);
		lcd_invert_process();
		lcd_print_ram_1();
	  }
	  if(sensor_change)
	  {
		  sensor_change=0;
		  I2C_Write_EEPROM(OBD.Gear_ratio,gear_ratio_EEPROM);
		  I2C_Write_EEPROM(OBD.speed_sensor_type,speed_sensor_type_EEPROM);
	  }
		if((Range.Odometer_Value>=last_flash_update+1000)&&(Range.Odometer_Value<=last_flash_update+1010))
		{
			last_flash_update=Range.Odometer_Value;
			I2C_Write_EEPROM(last_flash_update,last_flash_update_EEPROM);
		}
		if(rx)
		{
			Tx_fun(Rx_Id);
		}
		HAL_IWDG_Refresh(&hiwdg);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 8;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  /*Filter Configuration
    * Filter ID and Filter Mask are used to compare and verify the incoming Identifier for receiving data
    * FIFO 0 is used for store the received data
    *In FiterID mask mode , two 32 bit registers (Filter ID and Filter Mask)
    */
	CAN_FilterTypeDef filtercon;// Need to add filter configuration for receiving data
	filtercon.FilterActivation = CAN_FILTER_ENABLE;//Enable Filter for receiving data
	filtercon.FilterBank = 1;//Here using filter bank 1 for receiving data(Controller- single CAN -14 filter bank)
	filtercon.FilterFIFOAssignment = CAN_FILTER_FIFO0;//using FIFO 0 for receiving data
	filtercon.FilterIdHigh = 0x0000;//controller receives all the identifier without any restriction if gives 0x0000
	filtercon.FilterIdLow = 0x0000;//controller receives all the identifier without any restriction if gives 0x0000
	filtercon.FilterMaskIdHigh = 0X0000;//controller receives all the identifier without any restriction if gives 0x0000
	filtercon.FilterMaskIdLow = 0X0000;//controller receives all the identifier without any restriction if gives 0x0000
	filtercon.FilterMode = CAN_FILTERMODE_IDMASK;//using ID mask mode receiving identifier
	filtercon.FilterScale = CAN_FILTERSCALE_32BIT;//32 bit register ID and Mask register
	filtercon.SlaveStartFilterBank = 0;// don't care if the controller has single CAN
	HAL_CAN_ConfigFilter(&hcan,&filtercon);//Filter configuration declaration
  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_128;
  hiwdg.Init.Reload = 1000;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 100-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 72-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_D0_Pin|LCD_D1_Pin|LCD_D2_Pin|LCD_D3_Pin
                          |LCD_D4_Pin|LCD_D5_Pin|LCD_D6_Pin|LCD_D7_Pin
                          |Buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, lcd_adr_Pin|LCD_RD_Pin|lcd_reset_Pin|lcd_chip_sel_Pin
                          |check_led_Pin|Test_pin_Pin|LCD_RW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : speed_sensor_Pin */
  GPIO_InitStruct.Pin = speed_sensor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(speed_sensor_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_D0_Pin LCD_D1_Pin LCD_D2_Pin LCD_D3_Pin
                           LCD_D4_Pin LCD_D5_Pin LCD_D6_Pin LCD_D7_Pin
                           Buzzer_Pin */
  GPIO_InitStruct.Pin = LCD_D0_Pin|LCD_D1_Pin|LCD_D2_Pin|LCD_D3_Pin
                          |LCD_D4_Pin|LCD_D5_Pin|LCD_D6_Pin|LCD_D7_Pin
                          |Buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : lcd_adr_Pin LCD_RD_Pin lcd_reset_Pin lcd_chip_sel_Pin
                           check_led_Pin Test_pin_Pin LCD_RW_Pin */
  GPIO_InitStruct.Pin = lcd_adr_Pin|LCD_RD_Pin|lcd_reset_Pin|lcd_chip_sel_Pin
                          |check_led_Pin|Test_pin_Pin|LCD_RW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ONE_WIRE_PRT_Pin */
  GPIO_InitStruct.Pin = ONE_WIRE_PRT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ONE_WIRE_PRT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Reverse_state_Pin */
  GPIO_InitStruct.Pin = Reverse_state_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Reverse_state_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint8_t inc_buz=0;
uint16_t speed_time=0,can_buzzer_delay=1000,sec=0,rev_buzzr_delay=0,temp_buzzr_delay=0,TRIP_TIMER=0;
uint16_t SYC_PASS = 0;//data_delay = 60 for bheem
int8_t bit_count=7;
uint8_t avg=0,last_count=0; uint16_t doc=0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) // To create a every 100us timer Interrupt. prescalar value is 100 and ARR(Auto Reload Register) = 72.
{
  if (htim->Instance == TIM2)
  {
	 timerCounter++; //It will incremented every timer interrupt occur with time.
	 timerCounter2++;
	 timerCounter3++;
	 if(timerCounter==10)
	 {
		 m_sec++;
		 TRIP_TIMER++;
		 timerCounter=0;
	 }
	 if(m_sec==print_delay)
	 {
		 print_delay=m_sec+1000;
	     print_state=1;
	 }

	 if(m_sec==rev_buzzr_delay)
	 {
		 rev_buzzr_delay=m_sec+500;
		 if(Reverse_status==1)
		 {
			 HAL_GPIO_TogglePin(GPIOA, Buzzer_Pin);
		 }
	 }
	 if(m_sec==temp_buzzr_delay)
	 {
		 temp_buzzr_delay=m_sec+100;
		 if(Battery_high_Temp==1)
		 {
			 HAL_GPIO_TogglePin(GPIOA, Buzzer_Pin);
		 }
	 }

	 if(m_sec==speed_time)
	 {
		 if(doc>5)
		 {
			doc=0;
			speed_time=m_sec+1000;
			speed_count =last_count;
			after_sec=1;
			speed_count_temp=0;
		 }
		 else
		 {
			doc=0;
			speed_time=m_sec+1000;
			speed_count =speed_count_temp;;
			last_count=speed_count;
			after_sec=1;
			speed_count_temp=0;
		 }
	 }

	 if(m_sec == can_buzzer_delay)
	 {
		 if(inc_buz<=4)
		 {
			 can_buzzer_delay = m_sec+150;
			 if(can_error_state == 1)
			 {
				 inc_buz++;
			 }
		 }

		 if(can_error_state == 1)
		 {
			 HAL_GPIO_TogglePin(GPIOA, Buzzer_Pin);
			 if(inc_buz>4)
			 {
				 can_buzzer_delay = m_sec+60000;
				 inc_buz=0;
			 }
		 }
	 }

	 if(Pin_State == 1)
	 {
	 if(timerCounter2>=30)		// after every 750us only check the signal data from external interrupt pin.
	{
		 HAL_GPIO_TogglePin(GPIOB, Test_pin_Pin);
		 test_bit++;
		if(HAL_GPIO_ReadPin(GPIOB, ONE_WIRE_PRT_Pin)) 		//after 750us pin in high state , data 0
		{
			bitClear(dataArray[signalCounter],bit_count);
		}
		else
		{
			bitSet(dataArray[signalCounter],bit_count);		//after 750us pin in low state , data 1
		}
		Pin_State = 0;
		bit_count--;

		if(bit_count<0) 		// bit decrement for 7 to 0
		{
			signalCounter++;
			bit_count=7;
		}

		if(signalCounter >= 12)		//byte increment 0 to 12
		{
			signalCounter = 0;
			syncFlag = 1;
			SYC_DATA=0;
			timerCounter3=0;
			for(uint8_t i = 0; i < 12;i++)
			{
				dataArray1[i]=dataArray[i];		//The received data are stored in another array and it will wait for next data reception.
			}
			 processData();    	//after storing the data should be processed for structure for easy access to the user.
		}
		timerCounter2 = 0;
	}
	 }
  }
}

	volatile uint8_t buttonState = 0;  // Current button state (1 if pressed, 0 if released)
	volatile uint8_t lastButtonState = 1;  // Previous button state (1 if pressed, 0 if released)
	volatile uint32_t lastDebounceTime = 0,startDebounceTime=0;  // Time of the last button state change
	volatile uint32_t debounceDelay = 90;  // Debounce time in milliseconds
	volatile uint32_t temp_var;
	uint16_t reading,top=0;

void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin)
{
	if(GPIO_Pin==speed_sensor_Pin)
	{
		uint8_t reading = HAL_GPIO_ReadPin(GPIOC, speed_sensor_Pin);

		if (reading == 1)
		{
			startDebounceTime =HAL_GetTick();
		}
		if (reading == 0)
		{
			lastDebounceTime = HAL_GetTick();
		}

		if(lastDebounceTime<startDebounceTime)
		{
			top=startDebounceTime-lastDebounceTime;
		}
		else
		{
			top=lastDebounceTime-startDebounceTime;
		}

		if(top>=8)
		{
			speed_count_temp++;
		}
		else
		{
			doc++;
		}
	}
 if (GPIO_Pin ==ONE_WIRE_PRT_Pin )  	//Every rising edge detected the interrupt will be triggered.
  {
	 if(SYC_DATA ==0)
	 {
		 if(timerCounter3 > 0)
		 {
			SYC_PASS = timerCounter3;		//whenever the interrupt triggered that corresponding timer interrupt count should be stored in variable.
			timerCounter3 = 0;
			 if(SYC_PASS >= 500)		//(SYN_PASS >=1000 For bheem)	//whenever the one interrupt to another interrupt time should be greater than 20ms , it is synchronization period . After the synchronization period data are storing process should be start.(20x50 = 1ms so 200x50 = 10ms)
			 {
				SYC_DATA = 1;
				SYC_PASS = 0;
				timerCounter3 = 0;
			 }
		 }
	 }

	 if(SYC_DATA == 1)
	 {
		Pin_State=1;   	//after synchronization this bit set for data storing.
		timerCounter2=0;
	 }
	 timerCounter3 = 0;
  }
}

void processData(void) { // find out the synchronization period and converting data into 's and 1's

   uint8_t calculatedChecksum = 0;
	for (int checksum_count = 0; checksum_count < STORED_DATA_SIZE - 1; checksum_count++) { //To calculate the checksum for DATA0 to DATA10 and check the received checksum in DATA11
		calculatedChecksum ^= dataArray1[checksum_count];
	}
	if (calculatedChecksum == dataArray1[STORED_DATA_SIZE - 1]) { // If the calculated checksum is equal to the received checksum then set the validDataFlag
		validDataFlag = 1;
	}
	else
	{
		checksum_error++;
		validDataFlag = 0;// If the calculated checksum is Not equal to the received checksum then Reset the validDataFlag
	}

	if(validDataFlag == 1)
	{
		Motor_Data.Device_Code = dataArray1[0]; // Default Device code stored in this variable. 8bit
		Motor_Data.Sequence_code = dataArray1[1]; // Default Sequence code stored in this variable. 8bit
		Motor_Data.Alternate_Bit1 = ((dataArray1[2] & 0xF0) >> 4); // This variable used for Alternate Function/operation . 4bit
		Motor_Data.Parking_Indication = ((dataArray1[2] & 0x08) >> 3); //This variable is stored the parking Indication state. 1bit
		Motor_Data.Speed_Limit = ((dataArray1[2] & 0x04) >> 2); // This variable is stored the Speed limit state. 1bit
		Motor_Data.Alternate_Bit2 = ((dataArray1[2] & 0x02) << 1); // This variable used for Alternate Function/operation 1bit
		Motor_Data.Side_Brace_Indication = ((dataArray1[2] & 0x01) >> 0);
		Motor_Data.Pushcart_prohibitedsign = ((dataArray1[3] & 0x80) >> 7); // If any Prohibited sign to indicate the display using this variable. 1bit
		Motor_Data.Hall_fault = ((dataArray1[3] & 0x40) >> 6); // If hall sensor status stored in this variable. 1bit(0-o hall fault, 1-Hall Fault).
		Motor_Data.Throttle_fault = ((dataArray1[3] & 0x20) >> 5); //If throttle status stored in this variable. 1bit(0-No Throttle Fault, 1-Throttle Fault).
		Motor_Data.Controller_fault = ((dataArray1[3] & 0x10) >> 4); //If the controller status stored in this variable. 1bit(0-No Controller Fault, 1-Controller Fault).
		Motor_Data.Under_Voltageprotection = ((dataArray1[3] & 0x08) >> 3); //Under voltage protection status stored in this variable. 1bit(0-No under voltage protection, 1- under voltage protection).
		Motor_Data.Cruise = ((dataArray1[3] & 0x04) >> 2); // Cruise mode on/off status stored in this variable. 1bit(0-No cruise mode on,1-cruise mode on).
		Motor_Data.Assistance_power = ((dataArray1[3] & 0x02) >> 1); //Power assistance status stored in this variable. 1bit(0-No power assistance, 1-Power assistance On).
		Motor_Data.Motor_phase_loss = ((dataArray1[3] & 0x01) >> 0); //Motor phase loss status stored in this variable. 1bit(0-No motor phase loss, 1-Motor phase loss).
		Motor_Data.Four_gear_indication = ((dataArray1[4] & 0x80) >> 7); //Four gear indication status stored in this variable. 1bit
		Motor_Data.Motor_Running = ((dataArray1[4] & 0x40) >> 6);//Motor status stored in this variable. 1bit(0- Motor is stopping ,1- Motor is running)
		Motor_Data.brake = ((dataArray1[4] & 0x20) >> 5); // Brake status stored in this variable. 1bit(0-No brake applied, 1- Brake applied)
		Motor_Data.Controller_protect = ((dataArray1[4] & 0x10) >> 4); //Controller protection  status stored in this variable. 1bit(0-No controller protection,1-Controller protection)
		Motor_Data.Slide_charging = ((dataArray1[4] & 0x08) >> 3); // Regeneration status stored in this variable. 1bit(0- No regeneration detected, 1- regeneration detected.	   	Motor_Data.Antiflying_vehicle_protection = ((dataArray[4] & 0x04) >> 2);
		Motor_Data.Three_speed = ((dataArray1[4] & 0x03) >> 0); // Three speed mode status stored in this variable. 2bit (00 - Controller without three speed, 01 - Low Speed, 10- Medium Speed, 11- High speed)
		Motor_Data.Cloud_powermode = ((dataArray1[5] & 0x80) >> 7); // Speed increase status stored in this variable. 1bit
		Motor_Data.Push_to_talk = ((dataArray1[5] & 0x40) >> 6); //Most EVs have bluetooth connectivity that allows you to connect your phone and use its features hands-free while driving. This includes making calls, sending messages, and using navigation app.1bit
		Motor_Data.Standby_powersupply = ((dataArray1[5] & 0x20) >> 5); //The standby power supply are stored in this variable.1bit(0-No standby power supply,1-Standby power supply mode on).
		Motor_Data.Overcurrent_protection = ((dataArray1[5] & 0x10) >> 4); //Over current protection status stored in this variable.1bit(0-No over current protection, 1-over current protection)
		Motor_Data.Locked_rotor_protection = ((dataArray1[5] & 0x08) >> 3); //This variable is stored the status of Locked Rotor protection. (0-No locked Rotor protection,1-Locked Rotor Protection)
		Motor_Data.Reverse = ((dataArray1[5] & 0x04) >> 2); //This variable is stored the status of reverse indication. 1bit(0- Forward, 1- Reverse).
		Motor_Data.Electronic_break = ((dataArray1[5] & 0x02) >> 1); //It is a Low Brake. The Low brake indication status are stored in this bit . 1bit
		Motor_Data.Speed_limit = ((dataArray1[5] & 0x01) >> 0); //To set the speed limit using this variable. 1bit
		Motor_Data.Operating_current = dataArray1[6];//Operating current rating are stored in this variable. 8bit
		Motor_Data.Speed = ((dataArray1[7] << 8 )| dataArray[8]); //Speed calculation are stored in this variable. Speed byte high(8bit) and speed byte low(8bit) are combined and stored in this single variable. 16bits
		Motor_Data.Battery_Level = dataArray1[9]; //Battery level are stored in this variable. 8bit
		Motor_Data.Current_Level = dataArray1[10]; //Current level are stored in this variable. 8bit
		Motor_Data.Checksum = dataArray1[11]; // Received checksum data's are stored in this variable. 8bit
		validDataFlag = 0;


		if(Motor_Data.Device_Code==13)
			{
				if(dataArray1[7]==106)
				{
					if((dataArray[8]>=106)&&(dataArray1[8]<=255))
					{
						speed_temp1=dataArray1[8]-106;
					}
					if((dataArray[8]>=0)&&(dataArray1[8]<=105))
					{
						speed_temp1=dataArray1[8]+149;
					}
				}

				if(dataArray1[7]==107)
				{
					if((dataArray[8]>=106)&&(dataArray1[8]<=255))
					{
						speed_temp1=dataArray1[8]+149;
					}
					if((dataArray[8]>=0)&&(dataArray1[8]<=105))
					{
						speed_temp1=dataArray1[8]+404;
					}
				}

				if(dataArray1[7]==108)
				{
					if((dataArray[8]>=106)&&(dataArray1[8]<=255))
					{
						speed_temp1=dataArray1[8]+404;
					}
					if((dataArray[8]>=0)&&(dataArray1[8]<=105))
					{
						speed_temp1=dataArray1[8]+659;
					}
				}
				controller_speed_data=speed_temp1*1.1;
	}
		if(Motor_Data.Device_Code==8)
		{
			controller_speed_data=Motor_Data.Speed*1.1;
		}

}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
	    HAL_NVIC_SystemReset();
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
