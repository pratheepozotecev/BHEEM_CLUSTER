/*
 * ST7565P.c
 *
 *  Created on: Dec 12, 2023
 *      Author: LENOVO
 */

#include "main.h"
#include "ST7565p.H"
//extern enum alp{A=1,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P,Q,R,S,T,W,X,Y,Z};
uint8_t lcd_temp_ram[8][128];
uint8_t lcd_temp_ram_1[8][128];

void Lcd_cmd(uint8_t cmd)
{
	GPIOB->BRR|=lcd_adr_Pin;  //Address(RESET);
	GPIOB->BRR|=lcd_chip_sel_Pin;  //CS_1(RESET);
	parllel_transmit(cmd&0xFF);
	GPIOB->ODR|=LCD_RD_Pin;
	GPIOB->BRR|=LCD_RD_Pin;
	GPIOB->ODR|=lcd_chip_sel_Pin;   //CS_1(SET);
}

void lcd_print_ram_1()
{
	for(int y_axsis=0;y_axsis<=7;y_axsis++)
	{
		Lcd_cmd(y_axsis+0xB0);
		for(int x_axsis=0;x_axsis<=127;x_axsis++)
		{
			lcd_x_axis(x_axsis);
			GPIOB->ODR|=lcd_adr_Pin;  //Address(RESET);
			GPIOB->ODR|=LCD_RD_Pin;
			GPIOB->BRR|=lcd_chip_sel_Pin;  //CS_1(RESET);
			parllel_transmit( (lcd_temp_ram_1[y_axsis][x_axsis])& 0XFF);
			GPIOB->ODR|=lcd_chip_sel_Pin;
			GPIOB->BRR|=LCD_RD_Pin;//CS_1(SET);
		}
	}
}

void lcd_init(){
	GPIOB->BRR|=lcd_chip_sel_Pin;	 				//HAL_GPIO_WritePin(GPIOA,  CS_1_Pin,RESET);// low the cs pin to listen the controller
	HAL_GPIO_WritePin(GPIOB, lcd_reset_Pin,RESET); // low reset button
	HAL_Delay(500); // wait for 500ms
	HAL_GPIO_WritePin(GPIOB, lcd_reset_Pin,SET); // high the reset button for initial reset

	  // LCD bias select
	  Lcd_cmd(CMD_SET_BIAS_9);
	  // ADC select
	  Lcd_cmd(CMD_SET_ADC_NORMAL);
	  // SHL select
	  Lcd_cmd(CMD_SET_COM_NORMAL);
	  // Initial display line
	  Lcd_cmd(CMD_SET_DISP_START_LINE);

	  // turn on voltage converter (VC=1, VR=0, VF=0)
	  Lcd_cmd(CMD_SET_POWER_CONTROL | 0x4);
	  // wait for 50% rising
	  HAL_Delay(50);

	  // turn on voltage regulator (VC=1, VR=1, VF=0)
	  Lcd_cmd(CMD_SET_POWER_CONTROL | 0x6);
	  // wait >=50ms
	  HAL_Delay(50);

	  // turn on voltage follower (VC=1, VR=1, VF=1)
	  Lcd_cmd(CMD_SET_POWER_CONTROL | 0x7);
	  // wait
	  HAL_Delay(50);

	  // set lcd operating voltage (regulator resistor, ref voltage resistor)
	  Lcd_cmd(CMD_SET_RESISTOR_RATIO | 0x4);

	  // initial display line
	  // set page address
	  // set column address
	  // write display data

	  // set up a bounding box for screen updates

	  Lcd_cmd(0xAF);    //Display on
	  Lcd_cmd(0XA4);
	}

void lcd_x_axis(uint8_t value){

    Lcd_cmd(0x10|((value&0xF0)>>4));//Setting  y-address  Msb
    Lcd_cmd(value&0x0F); //Setting  y-address  Lsb
}

void lcd_invert_process()
{
	for(int y_axsis=0;y_axsis<=7;y_axsis++){
			uint8_t temp_x_axsis=0;
			for(int x_axsis=127;x_axsis>=0;x_axsis--)
			{
				lcd_temp_ram_1[y_axsis][temp_x_axsis++]=(lcd_temp_ram[y_axsis][x_axsis]);
			}
		}
}

void lcd_speed(uint8_t num){
	uint8_t x_axis,y_axis,value_num=0,first_num,second_num,third_num;
    first_num=num/100;
    second_num=((num%100)/10);
    third_num=((num%100)%10);
    value_num=0;

    for(x_axis=0;x_axis<20;x_axis++)
    {
        for(y_axis=0;y_axis<5;y_axis++)
        {
            lcd_print_convert((2+(y_axis)),(x_axis+41),number[second_num][value_num]);
            value_num++;
        }
    }
        value_num=0;
    for(x_axis=0;x_axis<20;x_axis++)
    {
        for(y_axis=0;y_axis<5;y_axis++)
        {
            lcd_print_convert((2+(y_axis)),(x_axis+63),number[third_num][value_num]);
            value_num++;
        }
    }

    lcd_print_char(6, 85, "km");
    lcd_print_char(6,97, "{");

    lcd_print_char(6,103, "h");
}

void battery_charge_soc(uint16_t num){

	uint8_t x_axis,y_axis,value_num=0,first_num,second_num,third_num;
    first_num=num/1000;
    second_num=((num%1000)/100);
    third_num=((num%1000)%100)/10;
    value_num=0;
    if(num==1000)
    {
		for(x_axis=0;x_axis<20;x_axis++)
		   {
			   for(y_axis=0;y_axis<5;y_axis++)
			   {
				   lcd_print_convert((2+(y_axis)),(x_axis+16),number[1][value_num]);
				   value_num++;
			   }
		   }
		value_num=0;
    }
    for(x_axis=0;x_axis<20;x_axis++)
    {
        for(y_axis=0;y_axis<5;y_axis++)
        {
            lcd_print_convert((2+(y_axis)),(x_axis+41),number[second_num][value_num]);
            value_num++;
        }
    }
    value_num=0;
    for(x_axis=0;x_axis<20;x_axis++)
    {
        for(y_axis=0;y_axis<5;y_axis++)
        {
            lcd_print_convert((2+(y_axis)),(x_axis+63),number[third_num][value_num]);
            value_num++;
        }
    }
    value_num=0;
	 for(x_axis=0;x_axis<9;x_axis++)
	 {
		 for(y_axis=0;y_axis<2;y_axis++)
		 {
			 lcd_print_convert((5 +(y_axis)),(x_axis+86),persentage_big[value_num]);
			 value_num++;
		 }
	 }
}

void gear_status_print(uint8_t data_gear)
{
	uint8_t temp_gear=0;
	 for(uint8_t y_axsis=3;y_axsis<=6;y_axsis++)
		{
			for(uint8_t x_axsis=2;x_axsis<=19;x_axsis++)
			{
				lcd_print_convert(y_axsis,x_axsis,gear_print[data_gear][temp_gear++]);
			}
		}
	 temp_gear=0;
}

void battery_bar_print(uint8_t battery_temp)
{
	uint16_t temp_bat=0;
	 for(uint8_t y_axsis=1;y_axsis<=6;y_axsis++)
		{
			for(uint8_t x_axsis=112;x_axsis<=124;x_axsis++)
			{
				lcd_print_convert(y_axsis,x_axsis,Battery_bar[battery_temp][temp_bat++]);
			}
		}
	 temp_bat=0;
}

void service_icon()
{
	uint8_t temp_gear=0;
	 for(uint8_t y_axsis=2;y_axsis<=3;y_axsis++)
		{
			for(uint8_t x_axsis=0;x_axsis<=11;x_axsis++)
			{
			 lcd_print_convert(y_axsis,x_axsis,service_reminder_icon[temp_gear++]);
			}
		}
	 temp_gear=0;
}

void danger_icon()
{
	uint8_t temp_gear=0;
	 for(uint8_t y_axsis=2;y_axsis<=3;y_axsis++)
		{
			for(uint8_t x_axsis=22;x_axsis<=35;x_axsis++)
			{
			 lcd_print_convert(y_axsis,x_axsis,danger[temp_gear++]);
			}
		}
	 temp_gear=0;
}

void danger_clear()
{
	uint8_t temp_gear=0;
	 for(uint8_t y_axsis=2;y_axsis<=3;y_axsis++)
		{
			for(uint8_t x_axsis=22;x_axsis<=35;x_axsis++)
			{
			 lcd_print_convert(y_axsis,x_axsis,0x00);
			}
		}
	 temp_gear=0;
}

uint8_t print_value_int=0;

void lcd_print_char(uint8_t y_axis_start, uint8_t x_axis_start, char* print_value)//(x_axis_start, y_axis_start, icon_width, icon_height, *print_value
{
	uint8_t space=1;
	Lcd_cmd((0xB0|(y_axis_start)));
	lcd_x_axis(x_axis_start);
	while(*print_value)
	{
	 for(int temp1=0;temp1<5;temp1++)
			{
			 if((*print_value>=65)&&(*print_value<=90))
				{
				 print_value_int=*print_value;
				// lcd_print(alphabet[print_value_int-65][temp1]);
				 lcd_print_convert(y_axis_start, x_axis_start++, (alphabet[print_value_int-65][temp1]));
				 space=1;
				}

			 if((*print_value>=97)&&(*print_value<=124))
				{
				 print_value_int=*print_value;
				 //lcd_print(alphabet[print_value_int-71][temp1]);

				 lcd_print_convert(y_axis_start,x_axis_start++, (alphabet[print_value_int-71][temp1]));
				 space=0;
				}

			 if((*print_value>=48)&&(*print_value<=57))
				{
				 print_value_int=*print_value;
				 //lcd_print(digit[print_value_int-48][temp1]);

				 lcd_print_convert(y_axis_start, x_axis_start++, (digit[print_value_int-48][temp1]));
				 space=1;
				}
			 if(*print_value==46)
				 {
				 // lcd_print(0x00);
				  lcd_print_convert(y_axis_start, x_axis_start++, 0x40 );
				  break;
				 }
			 if(*print_value==32)
				 {
				 // lcd_print(0x00);
				  lcd_print_convert(y_axis_start, x_axis_start++, 0x00 );
				  break;
				 }

			 if(*print_value==37)
			 {
				 lcd_print_convert(y_axis_start, x_axis_start++,persentage[temp1]);
			 }
		 }
	 print_value++;
	 //lcd_print(0x00);
	 if(space)
	 {
		 lcd_print_convert(y_axis_start, x_axis_start++, 0x00 );
		 space=0;
	 }
	}
}
void lcd_clear(uint8_t y_axsis,uint8_t x_axsis, uint8_t count)
{
	for(uint8_t x_axis_start=0;x_axis_start<=count;x_axis_start++)
	{
		lcd_temp_ram[y_axsis][x_axsis++]=0x00;
	}
}
void lcd_print_digit_wos(uint8_t y_axis_start, uint8_t x_axis_start,uint8_t print_value)//(x_axis_start, y_axis_start, icon_width, icon_height, *print_value
{
	Lcd_cmd((0xB0|(y_axis_start)));
	lcd_x_axis(x_axis_start);
	for(int temp1=0;temp1<5;temp1++)
	{
		//lcd_print(digit[currentDigit][temp1]);
		lcd_print_convert(y_axis_start, x_axis_start++, digit[print_value][temp1]);
	}
	lcd_print_convert(y_axis_start, x_axis_start++, 0X00);
}
void lcd_print_digit(uint8_t y_axis_start, uint8_t x_axis_start,uint16_t print_value)//(x_axis_start, y_axis_start, icon_width, icon_height, *print_value
{
  uint8_t digitCount=0;
  int originalNumber = print_value;
	Lcd_cmd((0xB0|(y_axis_start)));
	lcd_x_axis(x_axis_start);
	int num_digit=0;
	while (originalNumber > 0)
		{
			digitCount++;
			originalNumber /= 10;
		}

	for (int8_t temp_flag_1 = digitCount - 1; temp_flag_1 >= 0; temp_flag_1--)
	{
		  // Extract the digit at position i
		  int divisor = 1;
		  for (uint8_t temp_flag_2 = 0; temp_flag_2 < temp_flag_1; temp_flag_2++) {
			  divisor *= 10;
	}
	  uint8_t currentDigit = (print_value / divisor) % 10;
		for(int temp1=0;temp1<5;temp1++)
		{
			//lcd_print(digit[currentDigit][temp1]);
			lcd_print_convert(y_axis_start, x_axis_start++, digit[currentDigit][temp1]);
		}
		lcd_print_convert(y_axis_start, x_axis_start++, 0X00);
}
uint8_t print_value_int=0;
}

void lcd_into()
{
	uint16_t temp=0;
	for(uint8_t y_axsis=0;y_axsis<=7;y_axsis++)
		{
			for(uint8_t x_axsis=0;x_axsis<=127;x_axsis++)
			{
				lcd_print_convert(y_axsis,x_axsis,intro1[temp++]);
			}
		}
	 Lcd_cmd(0xA2);// ADC select
	 Lcd_cmd(0xA0);// SHL select
	 Lcd_cmd(0xC0);// Initial display line
	 Lcd_cmd(0x40);
	 lcd_invert_process();
	 lcd_print_ram_1();
}


void lcd_print_convert(uint8_t y_axsis,uint8_t x_axsis,uint8_t data)
{
	lcd_temp_ram[y_axsis][x_axsis]=data;
}
