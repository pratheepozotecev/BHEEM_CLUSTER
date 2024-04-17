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

//#include "lcd_print_process.h"
#include "Daly_can_com.h"
#include "Daly_can_com_source.c"
#define EEPROM_DEV_ADD 0xA0 // EEPROM 7-bit address

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

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
volatile uint8_t syncFlag = 0; // Whenever this flag is set the External Interrupt Will Disable, Reset External Interrupt Will Enable
volatile uint16_t signalCounter; //It increments the External Interrupt detection stored in dataArray.
volatile uint16_t timerCounter = 0; //It will incremented every timer interrupt occur with time.
volatile uint16_t dataArray[400]; //It stores whenever External Interrupt detected that corresponding time store in the timerCounter Variable.
volatile uint8_t validDataFlag; //If the checksum is equal to the received data then it will be stored 1 else it will 0.
uint16_t m_sec=0,lop=0;
uint8_t Tx_count=0;//This variable is for loop iteration
uint8_t Transmit_Data[1]; //Declaration of an array capable of holding one byte of transmit data
uint32_t Rx_Id; //Temporary variable for storing received identifier
uint32_t TxMailBox; // This variable is used to transmitting data and acts like a message buffer
uint32_t speed_count=0;
uint32_t speed_count_temp=0;
uint8_t after_sec=0;
uint8_t Odo_Write[3];//Array to store odo-meter values for the purpose of writing data
uint8_t data_Read[3];//Array to store the read odo-meter data retrieved from EEPROM
uint32_t Odo_Value;
uint8_t Km_Flag=1;
uint16_t inc_delay=0;
uint8_t first_time=1;
uint16_t Reserved_SOC=0;
uint16_t can_delay=500;
uint8_t can_state=0;
uint8_t Battery_high_Temp=0;
#define  calib_reg 440//1656-165;
uint16_t print_delay=2000;
uint8_t print_state=0;
uint8_t bat_icon_toogle=0;
struct Speed //structure declaration
{
	uint32_t Odometer_Value;//structure members
	uint16_t Trip_value;//structure members
}Range;
void Split(uint32_t);//Function declaration
void Merge();//Function declaration
//Array declaration
uint8_t data_Write[3];
uint8_t data_Read[3];
uint16_t calib_write[2];
uint16_t calib_read[2];

CAN_TxHeaderTypeDef TxHeader;// Adding structure variable for accessing Transmitter frame format element Eg: StdId,RTR..
CAN_RxHeaderTypeDef RxHeader;// Adding Structure variable for accessing Receiver frame format element

uint16_t Bike_speed;
unsigned int BMS_ID[]={0X18900140,0X18910140, 0X18920140,0X18930140,0X18940140,0X18950140,0X18960140,0X18970140,0X18980140,0X18520140,0X18500140};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
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
	lcd_print_digit_wos(1, 0, (BMS.Cumulative_Charge/BMS.Battery_capacity)/1000);
	lcd_print_digit_wos(1, 6, ((BMS.Cumulative_Charge/BMS.Battery_capacity)%1000)/100);
	lcd_print_digit_wos(1, 12, ((((BMS.Cumulative_Charge/BMS.Battery_capacity)%1000)%100)/10));
	lcd_print_digit_wos(1, 18, ((((BMS.Cumulative_Charge/BMS.Battery_capacity)%1000)%100)%10));
}

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
			if(BMS.Min_Temp<=50)// less tham 10 degree min temp will be printed
			{
				lcd_print_digit(0, 0, ((BMS.Min_Temp)-40));//
				lcd_print_convert(0, 12, 0x03);
				lcd_print_convert(0, 13, 0x03);
				lcd_print_char(0, 15, "C");
			}
	}

	// negative value should be printer draw (-)
//	if(BMS.Min_Temp<=50)// less tham 10 degree min temp will be printed
//	{
//		lcd_print_digit(7, 0, ((BMS.Min_Temp)-40));//
//		lcd_print_convert(7, 12, 0x03);
//		lcd_print_convert(7, 13, 0x03);
//		lcd_print_char(7, 15, "C");
//	}

	if(BMS.Max_Temp>95)
	{
		Battery_high_Temp=1;
	}

	if(BMS.Max_Temp<90)
	{
		Battery_high_Temp=0;
	}
}

void battery_soc()
{
	Reserved_SOC=(int)1000.0-((1000.0-BMS.SOC)*(100.0/85.0));

	if(Reserved_SOC==1000){
		lcd_clear(7, 98, 29);
		lcd_print_digit_wos(7, 104, 1);
		lcd_print_digit_wos(7, 110, 0);
		lcd_print_digit_wos(7, 116, 0);
		lcd_print_char(7, 122, "%");
	}
	else{
		lcd_clear(7, 98, 29);
		lcd_print_digit_wos(7, 110, (Reserved_SOC/100));
		lcd_print_digit_wos(7, 116, (Reserved_SOC/10)%10);
		lcd_print_char(7, 122, "%");
	}
}
uint8_t start_inc=0;uint8_t Reverse_status=0;uint16_t DTE=0;
void Gear_Status()
{
	uint8_t Gear_State=0;
	GPIOA->CRL=0X88888888;
	GPIOA->ODR|=0XFF;
	HAL_GPIO_WritePin(GPIOB, check_led_Pin, RESET);
	HAL_Delay(50);
//	if(HAL_GPIO_ReadPin(GPIOA, LCD_D4_Pin)==0) //PARKING
//		{danger_icon();}
//	else
//		{danger_clear();}

	if(HAL_GPIO_ReadPin(GPIOA, LCD_D1_Pin)==1)
		{
			Gear_State=0;// REVERSE
			//Reverse_status=1;
		}
	else
		{
			Reverse_status=0;
			if((HAL_GPIO_ReadPin(GPIOA, LCD_D2_Pin)==1)&&((HAL_GPIO_ReadPin(GPIOA, LCD_D3_Pin)==0)))
				{
					Gear_State=1;
					DTE=(Reserved_SOC*130)/100;
				}
			if((HAL_GPIO_ReadPin(GPIOA, LCD_D2_Pin)==0)&&((HAL_GPIO_ReadPin(GPIOA, LCD_D3_Pin)==0)))
				{
					Gear_State=2;
					DTE=(Reserved_SOC*100)/100;
				}
			if((HAL_GPIO_ReadPin(GPIOA, LCD_D2_Pin)==0)&&((HAL_GPIO_ReadPin(GPIOA, LCD_D3_Pin)==1)))
				{
					Gear_State=3;
					DTE=(Reserved_SOC*70)/100;
				}
		}

	 if((HAL_GPIO_ReadPin(GPIOA, LCD_D5_Pin)==RESET))
		{
			 start_inc++;
			 if(start_inc>=3)
			 {
				start_inc=1;
			 }
			 I2C_Write_EEPROM(start_inc, LAST_STATE_ADDRESS_EEPROM);
		}
	 GPIOA->CRL=0X22222222;
	 HAL_Delay(50);
	 gear_status_print(Gear_State);
	 HAL_GPIO_WritePin(GPIOB, check_led_Pin, SET);
}

void BMS_CAN()//Transmitter function
{
	for(Tx_count=0; Tx_count<11; Tx_count++)
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
	Tx_count=0;
}
//Receiver Interrupt Function,
// * This function is triggered when data is received by the controller through a filter
// * from BMS according to the Identifier request
// * Received Identifier stored in a temporary variable for passing arguments to a function to process the data
//

//uint32_t last_range; uint8_t last_soc;
//uint16_t DTE=0,last_SOC,last_data=1,last_ODO=0;
//void distance_to_emt()
//{
//	if(last_data)
//	{
//		last_ODO=Range.Odometer_Value+10;
//		last_SOC=Reserved_SOC;
//		last_data=0;
//	}
//	if(Range.Odometer_Value>=last_ODO)
//	{
//		last_data=1;
//		DTE=(Reserved_SOC)/(last_SOC-Reserved_SOC);
//	}
//}

//uint16_t timer_sec_1=0;
//uint8_t last_soc_status=1,TTF_hr,TTF_min;
//uint16_t last_soc_ttf=0;
//uint8_t waittime_ttf=0;
//void TTf_staus()
//{
//	if(last_soc_status)
//	{
//		last_soc_ttf=Reserved_SOC+10;
//		last_soc_status=0;
//	}
//	else
//	{
//		if(last_soc_ttf<=Reserved_SOC)
//		{
//			uint32_t TTF_sec=timer_sec_1*((1000-Reserved_SOC)/10);
//			timer_sec_1=0;
//			TTF_min=(TTF_sec%3600)/60;
//			TTF_hr=TTF_sec/3600;
//			last_soc_status=1;
//		}
//		lcd_print_digit_wos(5, 46,TTF_hr/10);
//		lcd_print_digit_wos(5, 52,TTF_hr%10);
//		lcd_print_char(5, 58, "h");
//		lcd_print_digit_wos(5, 65,TTF_min/10);
//		lcd_print_digit_wos(5, 71,TTF_min%10);
//		lcd_print_char(5, 77, "m");
//		ttf_print();
//	}
//}

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

			if(bat_icon_toogle)
			{
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

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)//Receiver Interrupt Function
{
 	if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, Received_Data) == HAL_OK)//Receiving data through FIFO
 	{
 		Rx_Id = RxHeader.ExtId;
 		merge(Rx_Id);// Implementation of merging and splitting received BMS data
 	}
}
//uint8_t tog=1;
//void time_print()
//{
//	tog=(!tog);
//	if(BMS.Charger_State==1)
//	{
//		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
//		lcd_print_digit_wos(0, 102,(sTime.Hours/10));
//		lcd_print_digit_wos(0, 108,(sTime.Hours%10));
//		(tog)?lcd_print_convert(0, 114, 0x44):lcd_print_convert(0, 114, 0x00);
//		lcd_print_convert(0, 114, 0x44);
//		lcd_print_digit_wos(0,116,sTime.Minutes/10);
//		lcd_print_digit_wos(0, 122,sTime.Minutes%10);
//	}
//	else
//	{
//		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
//		lcd_print_digit_wos(0, 0,(sTime.Hours/10));
//		lcd_print_digit_wos(0, 6,(sTime.Hours%10));
//		(tog)?lcd_print_convert(0, 12, 0x44):lcd_print_convert(0, 114, 0x00);
//		lcd_print_digit_wos(0,14,sTime.Minutes/10);
//		lcd_print_digit_wos(0, 20,sTime.Minutes%10);
//		print_am();
//	}
//}

uint16_t pluse_count=0;

void ODO_calculation()
{
	uint8_t bike_speed=((speed_count*360)/calib_reg);
    lcd_speed(bike_speed);

	if(after_sec)
	{
		pluse_count+=speed_count;
		after_sec=0;
	}

	if(pluse_count>=calib_reg)
	{
		pluse_count=0;
		Range.Odometer_Value++;
		Range.Trip_value++;
		I2C_Write_EEPROM(Range.Odometer_Value,ODO_ADDRESS_EEPROM);
		I2C_Write_EEPROM(Range.Trip_value,TRIP_ADDRESS_EEPROM);
	}

	if(Range.Trip_value>=10000)
	{
		Range.Trip_value=0;
		I2C_Write_EEPROM(Range.Trip_value,TRIP_ADDRESS_EEPROM);
	}
		lcd_clear(0, 96, 51);
		uint8_t first=DTE/100;
		uint8_t second=((DTE%100)/10);
		uint8_t third=((DTE%100)%10);
		dte_icon_print();
		lcd_print_digit_wos(0, 98,first);
		lcd_print_digit_wos(0, 104,second);
		lcd_print_digit_wos(0, 110,third);
		lcd_print_char(0, 116, "km");
switch(start_inc)
	{
	case 1:
		{
			uint8_t first = (Range.Odometer_Value / 1000000);
			uint8_t second = ((Range.Odometer_Value % 1000000) / 100000);
			uint8_t third = (((Range.Odometer_Value % 1000000) % 100000) / 10000);
			uint8_t fourth = ((((Range.Odometer_Value % 1000000) % 100000) % 10000) / 1000);
			uint8_t fifth = (((((Range.Odometer_Value % 1000000) % 100000) % 10000) % 1000) / 100);
			uint8_t sixth = ((((((Range.Odometer_Value % 1000000) % 100000) % 10000) % 1000) % 100) / 10);
			uint8_t seventh = (((((((Range.Odometer_Value % 1000000) % 100000) % 10000) % 1000) % 100) % 10));
			lcd_clear(5, 32, 64);
			odo_icon_print();
			lcd_print_digit_wos(5, 50,first);
			lcd_print_digit_wos(5, 56,second);
			lcd_print_digit_wos(5, 62,third);
			lcd_print_digit_wos(5, 68,fourth);
			lcd_print_digit_wos(5, 74,fifth);
			lcd_print_digit_wos(5, 80,sixth);
			lcd_print_char(5,89, "km");
			break;
		}
	case 2:
		{
			uint8_t second=((Range.Trip_value%100000)/10000);
			uint8_t third=(((Range.Trip_value%100000)%10000)/1000);
			uint8_t fourth=((((Range.Trip_value%100000)%10000)%1000)/100);
			uint8_t fifth=(((((Range.Trip_value%100000)%10000)%1000)%100)/10);
			uint8_t sixth=(((((Range.Trip_value%100000)%10000)%1000)%100)%10);
			lcd_clear(5, 40, 51);
			trp_icon_print();
			lcd_print_digit_wos(5, 52,second);
			lcd_print_digit_wos(5, 58,third);
			lcd_print_digit_wos(5, 64,fourth);
			lcd_print_digit_wos(5, 70,fifth);
			lcd_print_convert(5, 76, 0x40);
			lcd_print_digit_wos(5, 78,sixth);
			lcd_print_char(5,84, "km");
			break;
		}

	default:
		{
			start_inc=1;
			break;
		}
	}
}

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
  /* USER CODE BEGIN 2 */

  HAL_I2C_Init(&hi2c1);
  HAL_CAN_Start(&hcan);// CAN protocol enable function
  HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING); // Interrupt activation for Receiving data ,whenever data is received in FIFO, this function will get triggered and goes to receiver interrupt function
  HAL_TIM_Base_Start_IT(&htim2); // Timer2 Interrupt Start
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
     lcd_init();
	 lcd_into();
	 HAL_Delay(1000);
	 lcd_clear(0, 0, 127);
	 lcd_clear(1, 0, 127);
	 lcd_clear(2, 0, 127);
	 lcd_clear(3, 0, 127);
	 lcd_clear(4, 0, 127);
	 lcd_clear(5, 0, 127);
	 lcd_clear(6, 0, 127);
	 lcd_clear(7, 0, 127);

//	 I2C_Write_EEPROM(0,ODO_ADDRESS_EEPROM);
//	 I2C_Write_EEPROM(0,TRIP_ADDRESS_EEPROM);
//   I2C_Write_EEPROM(1,LAST_STATE_ADDRESS_EEPROM);
	 Range.Odometer_Value=I2C_Read_EEPROM(ODO_ADDRESS_EEPROM);
	 Range.Trip_value=I2C_Read_EEPROM(TRIP_ADDRESS_EEPROM);
	 start_inc=I2C_Read_EEPROM(LAST_STATE_ADDRESS_EEPROM);

	 first_time=1;
	 uint8_t tog=0;


 while(1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  if(print_state)
  {
	  	print_state=0;
		BMS_CAN();// read data from the BMS through the can protocol
		lcd_clear(0, 0, 127);
		lcd_clear(1, 0, 127);
		lcd_clear(2, 0, 127);
		lcd_clear(3, 0, 127);
		lcd_clear(4, 0, 127);
		lcd_clear(5, 0, 127);
		lcd_clear(6, 0, 127);
		lcd_clear(7, 0, 127);
		battery_temp();
		battery_soc();
		line_print();
		battery_bar_soc();
		ODO_calculation();
		Gear_Status();
		battery_voltage();
		Lcd_cmd(0xA2); // ADC select
		Lcd_cmd(0xA0);// SHL select
		Lcd_cmd(0xC0);// Initial display line
		Lcd_cmd(0x40);
		lcd_invert_process();
		lcd_print_ram_1();
	  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
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
                          |check_led_Pin, GPIO_PIN_RESET);

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
                           check_led_Pin */
  GPIO_InitStruct.Pin = lcd_adr_Pin|LCD_RD_Pin|lcd_reset_Pin|lcd_chip_sel_Pin
                          |check_led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint16_t speed_time=0,speed_count_avg=0,ttf_delay=0,sec=0,rev_buzzr_delay=0,temp_buzzr_delay=0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) // To create a every 100us timer Interrupt. prescalar value is 100 and ARR(Auto Reload Register) = 72.
{
  if (htim->Instance == TIM2)
  {
	 timerCounter++; //It will incremented every timer interrupt occur with time.
	 if(timerCounter==10)
	 {
		 m_sec++;
		 timerCounter=0;
	 }
	 if(m_sec==print_delay)
	 {
		 print_delay=m_sec+1000;
		 print_state=1;
		 speed_count=speed_count_temp;
		 after_sec=1;
		 speed_count_temp=0;
	 }

	 if(m_sec==rev_buzzr_delay){
		 rev_buzzr_delay=m_sec+500;
		 if(Reverse_status==1){
			 HAL_GPIO_TogglePin(GPIOA, Buzzer_Pin);
		 }
	 }
	 if(m_sec==temp_buzzr_delay){
		 temp_buzzr_delay=m_sec+100;
		 if(Battery_high_Temp==1){
			 HAL_GPIO_TogglePin(GPIOA, Buzzer_Pin);
		 }
	}
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==speed_sensor_Pin)
	{
		speed_count_temp++;
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
