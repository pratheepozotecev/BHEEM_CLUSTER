/*
 * ST7565P.c
 *
 *  Created on: Dec 12, 2023
 *      Author: LENOVO
 */

#include "main.h"
#include "ST7565p.H"
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
	HAL_Delay(50); // wait for 500ms
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
	  Lcd_cmd(CMD_SET_RESISTOR_RATIO | 0x4);// contrast CHANGE OPTION USING EEPROM

	  // initial display line
	  // set page address
	  // set column address
	  // write display data

	  // set up a bounding box for screen updates

	  Lcd_cmd(0xAF);    //Display on
	  Lcd_cmd(0XA4);  // Display clear all
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


   for(y_axis=0;y_axis<4;y_axis++)
	{
		for(x_axis=0;x_axis<16;x_axis++)
		{
			lcd_print_convert((0+(y_axis)),(x_axis+46),(font16x32_digits[second_num][value_num]));
			value_num++;
		}
	}
   value_num=0;
    for(y_axis=0;y_axis<4;y_axis++)
     {
         for(x_axis=0;x_axis<16;x_axis++)
         {

             lcd_print_convert((0+(y_axis)),(x_axis+64),(font16x32_digits[third_num][value_num]));
             value_num++;
         }
     }
    value_num=0;
    for(y_axis=0;y_axis<1;y_axis++)
        {
            for(x_axis=0;x_axis<21;x_axis++)
            {

                lcd_print_convert((4+(y_axis)),(x_axis+52),kmph[value_num]);
                value_num++;
            }
        }
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

    for(y_axis=0;y_axis<4;y_axis++)
 	{
 		for(x_axis=0;x_axis<16;x_axis++)
 		{
 			lcd_print_convert((0+(y_axis)),(x_axis+46),(font16x32_digits[second_num][value_num]));
 			value_num++;
 		}
 	}
    value_num=0;
     for(y_axis=0;y_axis<4;y_axis++)
      {
          for(x_axis=0;x_axis<16;x_axis++)
          {

              lcd_print_convert((0+(y_axis)),(x_axis+64),(font16x32_digits[third_num][value_num]));
              value_num++;
          }
      }
     value_num=0;
	 for(x_axis=0;x_axis<9;x_axis++)
	 {
		 for(y_axis=0;y_axis<2;y_axis++)
		 {
			 lcd_print_convert((2 +(y_axis)),(x_axis+80),persentage_big[value_num]);
			 value_num++;
		 }
	 }
}

void gear_status_print(uint8_t data_gear)
{
	uint8_t temp_gear=0;
	if(data_gear==0)
	{
		for(uint8_t y_axsis=3;y_axsis<4;y_axsis++)
		{
			for(uint8_t x_axsis=0;x_axsis<29;x_axsis++)
			{
				lcd_print_convert(y_axsis,x_axsis,reverse_icon[temp_gear++]);
			}
		}
	}
	else{

	 for(uint8_t y_axsis=3;y_axsis<4;y_axsis++)
		{
			for(uint8_t x_axsis=0;x_axsis<21;x_axsis++)
			{
				lcd_print_convert(y_axsis,x_axsis,gear_print[data_gear-1][temp_gear++]);
			}
		}
	}
}

void battery_bar_print(uint8_t battery_temp)
{
	uint8_t temp_bat=0;
	 for(uint8_t y_axsis=6;y_axsis<=7;y_axsis++)
		{
			for(uint8_t x_axsis=26;x_axsis<=102;x_axsis++)
			{
				lcd_print_convert(y_axsis,x_axsis,Battery_bar[battery_temp][temp_bat++]);
			}
		}
	 temp_bat=0;
}

void service_icon()
{
	uint8_t temp_gear=0;
	 for(uint8_t y_axsis=1;y_axsis<=2;y_axsis++)
		{
			for(uint8_t x_axsis=0;x_axsis<=11;x_axsis++)
			{
				lcd_print_convert(y_axsis,110+ x_axsis,service_reminder_icon[temp_gear++]);
			}
		}
	 temp_gear=0;
}

void danger_icon()
{
	uint8_t temp_gear=0;
	 for(uint8_t y_axsis=3;y_axsis<=4;y_axsis++)
		{
			for(uint8_t x_axsis=100;x_axsis<=113;x_axsis++)
			{
				lcd_print_convert(y_axsis,x_axsis,danger[temp_gear++]);
			}
		}
	 temp_gear=0;
}

void danger_clear()
{
	uint8_t temp_gear=0;
	 for(uint8_t y_axsis=3;y_axsis<=4;y_axsis++)
		{
			for(uint8_t x_axsis=100;x_axsis<=113;x_axsis++)
			{
			 lcd_print_convert(y_axsis,x_axsis,0x00);
			}
		}
	 temp_gear=0;
}

void odo_icon_print()
{
	uint8_t temp_gear=0;
	for(uint8_t x_axsis=32;x_axsis<=47;x_axsis++)
		{
		 	 lcd_print_convert(5,x_axsis,ODO_ICON[temp_gear++]);
		}
}

void dte_icon_print()
{
	uint8_t temp_gear=0;
	for(uint8_t x_axsis=84;x_axsis<=96;x_axsis++)
	{
	 lcd_print_convert(0,x_axsis,DTE_ICON[temp_gear++]);
	}
}

void trp_icon_print()
{
	uint8_t temp_gear=0;
	for(uint8_t x_axsis=36;x_axsis<=50;x_axsis++)
	{
	 lcd_print_convert(5,x_axsis,TRP_ICON[temp_gear++]);
	}
}
void charge_cycle_print()
{
	uint8_t temp_gear=0;
	for(uint8_t x_axsis=107;x_axsis<=123;x_axsis++)
	{
	 lcd_print_convert(2,x_axsis,cgc_icon[temp_gear++]);
	}
}
void LBD_PRINT()
{
	uint8_t temp_gear=0;
	for(uint8_t x_axsis=8;x_axsis<=22;x_axsis++)
	{
	 lcd_print_convert(2,x_axsis,LBD_icon[temp_gear++]);
	}
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
				 lcd_print_convert(y_axis_start, x_axis_start++, (alphabet[print_value_int-65][temp1])<<1);
				 space=1;
				}

			 if((*print_value>=97)&&(*print_value<=124))
				{
				 print_value_int=*print_value;
				 //lcd_print(alphabet[print_value_int-71][temp1]);

				 lcd_print_convert(y_axis_start,x_axis_start++, (alphabet[print_value_int-71][temp1])<<1);
				 space=0;
				}

			 if((*print_value>=48)&&(*print_value<=57))
				{
				 print_value_int=*print_value;
				 //lcd_print(digit[print_value_int-48][temp1]);

				 lcd_print_convert(y_axis_start, x_axis_start++, (digit[print_value_int-48][temp1])<<1);
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
				 lcd_print_convert(y_axis_start, x_axis_start++,persentage[temp1]<<1);
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
		lcd_print_convert(y_axis_start, x_axis_start++, (digit[print_value][temp1])<<1);
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
			lcd_print_convert(y_axis_start, x_axis_start++, (digit[currentDigit][temp1])<<1);
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

void print_am()
{
	for(uint8_t x_axis=0;x_axis<12;x_axis++)
	   {
		   for(uint8_t y_axis=0;y_axis<1;y_axis++)
		   {
			   lcd_print_convert((1+(y_axis)),(x_axis+7),time_am[x_axis]);
		   }
	   }
}

void over_temperature_print()
{
	uint8_t temp_gear = 0;
	for(uint8_t y_axsis=1;y_axsis<=2;y_axsis++)
	{
		for(uint8_t x_axsis=25; x_axsis<=38;x_axsis++)
		{
			lcd_print_convert(y_axsis,x_axsis,High_temperature[temp_gear++]);
		}
	}
}

void line_print()
{
	 for(uint8_t y_axis=0;y_axis<1;y_axis++)
			   {
		for(uint8_t x_axis=0;x_axis<128;x_axis++)
		   {
			//if(x_axis==26){x_axis=102;}
			   lcd_print_convert((6+(y_axis)),(x_axis),0x04);
		   }
	   }
}
void ttf_print()
{
	 for(uint8_t y_axis=0;y_axis<1;y_axis++)
			   {
		for(uint8_t x_axis=0;x_axis<13;x_axis++)
		   {
			   lcd_print_convert((5+(y_axis)),(x_axis+31),TTF_icon[x_axis]);
		   }
	   }
}
void start_icon_fun()
{
	uint16_t value_temp=0;
	for(uint8_t y_axis=0;y_axis<7;y_axis++)
	   {
		for(uint8_t x_axis=57;x_axis<128;x_axis++)
		   {
			   lcd_print_convert(((y_axis)),(x_axis),start_icon[value_temp++]);
		   }
	   }
}
