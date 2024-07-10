//时间部分参数需要修改

#include "fu.h"
#include "main.h"
#include "tim.h"
#include "2812B.h"
#include "gpio.h"
#include "dma.h"

#include <stdlib.h>
#include <time.h>

//击打标志位，击打成功置1
int flag_1 = 0;
int flag_2 = 0;
int flag_3 = 0;
int flag_4 = 0;
int flag_5 = 0;
 
int skip_flag1=0;
int skip_flag2=0;
int skip_flag3=0;
int skip_flag4=0;
int skip_flag5=0;
//用于判断是否击打错误
uint8_t LastState1;
uint8_t LastState2;
uint8_t LastState3;
uint8_t LastState4;
uint8_t LastState5;

uint8_t CurrentState1;
uint8_t CurrentState2;
uint8_t CurrentState3;
uint8_t CurrentState4;
uint8_t CurrentState5;

int seed=0;//负责判断是否要重新打乱数组
uint8_t Mode=0;//判断亮灯要用怎么亮
int random_time=0;
void Text_Fu(void)//12345   //注释看Text_Fu理解整体流程，其他注释有问题，懒得改了
{
		HAL_GPIO_TogglePin(GPIOF, LED_GREEN_Pin);//判断是否正常运行
		if(flag_1 == 0)
		 {	
			 /*TIM2*/
			 //fu1亮部分灯
			RGB_SendArray_Part_1(FU_1_TIM,FU_1_CHA);
			//其他区域不亮灯
			RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
			RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
			RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
			RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);

			 for( int time = 0; time < 9000000 ; time++)//9000000近似为现实的2.5s
		  {
			   if( HAL_GPIO_ReadPin(KEY_1_GPIO,KEY_1_GPIO_Port) != 1 )//判断是否成功击打
		    {
			    flag_1 = 1;//击打成功
				break;
		    }

		  }
					if(flag_1!=1)//2.5内未击打成功
				  {
					flag_1 = 0;
					flag_2 = 0;
					flag_3 = 0;
					flag_4 = 0;
					flag_5 = 0;
					  
					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					  
					HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
					HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
					HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
					HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
					HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);

					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					
					 skip_flag1=0;
					 skip_flag2=0;
					 skip_flag3=0;
					 skip_flag4=0;
					 skip_flag5=0;
					 Mode++;
				  }		  
		 }
    else if(flag_1 == 1)
		{
			/*TIM2*/
			//fu1设置为全亮
			if(skip_flag1==0)//只发一次数据，发多了灯条会乱闪
			{
			RGB_SendArray_All_1(FU_1_TIM,FU_1_CHA);
			skip_flag1=1;
			}		
			if(flag_2 == 0)
		  	{
				//fu2设置部分亮，fu3、4、5不亮
				RGB_SendArray_Part_2(FU_2_TIM,FU_2_CHA);
				RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
				RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
				RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);

				for( int time = 0; time < 9000000 ; time++)//留出时间
				  {
					  if( HAL_GPIO_ReadPin(KEY_2_GPIO,KEY_2_GPIO_Port) != 1 )//判断是否成功击打
					{
						//击打成功
						flag_2 = 1;
						break;
					}
					
				  }
					if(flag_2!=1)
				  {
					flag_1 = 0;
					flag_2 = 0;
					flag_3 = 0;
					flag_4 = 0;
					flag_5 = 0;
					  
					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					  
					HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
					HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
					HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
					HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
					HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);

					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					
					 skip_flag1=0;
					 skip_flag2=0;
					 skip_flag3=0;
					 skip_flag4=0;
					 skip_flag5=0;
					 Mode++;
				  }		  
		 }
			
			else if(flag_2 == 1)
			{
			  //fu2设置为全亮
			if(skip_flag2==0)
			{
			RGB_SendArray_All_2(FU_2_TIM,FU_2_CHA);
			skip_flag2=1;
			}	
			  if(flag_3 == 0)
				{
					//fu3设置
				  RGB_SendArray_Part_3(FU_3_TIM,FU_3_CHA);
				  RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
				  RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);

				  for( int time = 0; time < 9000000 ; time++)//留出时间
				  {
					if( HAL_GPIO_ReadPin(KEY_3_GPIO,KEY_3_GPIO_Port) != 1 )//判断是否成功击打
					{
						//击打成功
					  flag_3 = 1;
						break;
					}
				  }
					if(flag_3!=1)
				  {
					flag_1 = 0;
					flag_2 = 0;
					flag_3 = 0;
					flag_4 = 0;
					flag_5 = 0;
					  
					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					  
					HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
					HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
					HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
					HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
					HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);

					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					
					 skip_flag1=0;
					 skip_flag2=0;
					 skip_flag3=0;
					 skip_flag4=0;
					 skip_flag5=0;
					 Mode++;
				  }		  
		 }
				else if(flag_3 == 1)
				{
				  //fu3设置为全亮
				if(skip_flag3==0)
				{
				RGB_SendArray_All_3(FU_3_TIM,FU_3_CHA);
				skip_flag3=1;
				}
				  if(flag_4 == 0)
					{
						//fu4设置
					  RGB_SendArray_Part_4(FU_4_TIM,FU_4_CHA);
					  RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					  for( int time = 0; time < 9000000 ; time++)//留出时间
					  {
						if( HAL_GPIO_ReadPin(KEY_4_GPIO,KEY_4_GPIO_Port) != 1 )//判断是否成功击打
						{
							//击打成功
							flag_4 = 1;
							break;

							}
					   }
					if(flag_4!=1)
				  {
					flag_1 = 0;
					flag_2 = 0;
					flag_3 = 0;
					flag_4 = 0;
					flag_5 = 0;
					  
					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					  
					HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
					HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
					HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
					HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
					HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);

					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					
					 skip_flag1=0;
					 skip_flag2=0;
					 skip_flag3=0;
					 skip_flag4=0;
					 skip_flag5=0;
					 Mode++;
				  }		  
		 }				
					else if(flag_4 == 1)
					{
					  //fu4设置为全亮
					if(skip_flag4==0)
					{
					RGB_SendArray_All_4(FU_4_TIM,FU_4_CHA);
					skip_flag4=1;
					}		
					  if(flag_5 == 0)
						{
						  //fu5设置
						  RGB_SendArray_Part_5(FU_5_TIM,FU_5_CHA);
						  for( int time = 0; time < 9000000 ; time++)//留出时间
						  {
							if( HAL_GPIO_ReadPin(KEY_5_GPIO,KEY_5_GPIO_Port) != 1 )//判断是否成功击打
							{
								flag_5 = 1;
								break;
							}
						  }
					if(flag_5!=1)
				  {
					flag_1 = 0;
					flag_2 = 0;
					flag_3 = 0;
					flag_4 = 0;
					flag_5 = 0;
					  
					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					  
					HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
					HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
					HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
					HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
					HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);

					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);		  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					
					 skip_flag1=0;
					 skip_flag2=0;
					 skip_flag3=0;
					 skip_flag4=0;
					 skip_flag5=0;
					 Mode++;
				  }		  
		 }						  
						
						else if (flag_5 == 1)
						{
							RGB_SendArray_All_5(FU_5_TIM,FU_5_CHA);//fu5全亮
							HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);//测试完成
							HAL_Delay(3000);//庆祝一下
							//重新初始化
							flag_1 = 0;
							flag_2 = 0;
							flag_3 = 0;
							flag_4 = 0;
							flag_5 = 0;
							RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
							RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
							RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
							RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
							RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
							
							 skip_flag1=0;
							 skip_flag2=0;
							 skip_flag3=0;
							 skip_flag4=0;
							 skip_flag5=0;
							 Mode++;
						}
					}
				}

			}	
		}
		HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
		HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
		HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
		HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
		HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);
			
}

void Text_Fu2(void)//21345   //注释看Text_Fu理解整体流程，其他注释有问题，懒得改了
{
		HAL_GPIO_TogglePin(GPIOF, LED_GREEN_Pin);//判断是否正常运行
		if(flag_2 == 0)
		 {	
			 /*TIM2*/
			 //fu1亮部分灯
			RGB_SendArray_Part_2(FU_2_TIM,FU_2_CHA);
			//其他区域不亮灯
			RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);
			RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
			RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
			RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);

			 for( int time = 0; time < 9000000 ; time++)//给出击打时间，避免烧坏LED，删去可能导致反复快速翻转LED。
		  {
			   if( HAL_GPIO_ReadPin(KEY_2_GPIO,KEY_2_GPIO_Port) != 1 )//判断是否成功击打
		    {
			    flag_2 = 1;//击打成功
				break;
		    }

		  }
					if(flag_2!=1)
				  {
					flag_1 = 0;
					flag_2 = 0;
					flag_3 = 0;
					flag_4 = 0;
					flag_5 = 0;
					  
					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					  
					HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
					HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
					HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
					HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
					HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);

					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					
					
					
					 skip_flag1=0;
					 skip_flag2=0;
					 skip_flag3=0;
					 skip_flag4=0;
					 skip_flag5=0;
					 Mode++;
				  }		  
		 }
    else if(flag_2 == 1)
		{
			/*TIM2*/
			//fu1设置为全亮
			if(skip_flag2==0)
			{
			RGB_SendArray_All_2(FU_2_TIM,FU_2_CHA);
			skip_flag2=1;
			}		
			if(flag_1 == 0)
		  	{
				//fu2设置部分亮，fu3、4、5不亮
				RGB_SendArray_Part_1(FU_1_TIM,FU_1_CHA);
				RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
				RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
				RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);

				for( int time = 0; time < 9000000 ; time++)//留出时间
				  {
					  if( HAL_GPIO_ReadPin(KEY_1_GPIO,KEY_1_GPIO_Port) != 1 )//判断是否成功击打
					{
						//击打成功
						flag_1 = 1;
						break;
					}
					
				  }
					if(flag_1!=1)
				  {
					flag_1 = 0;
					flag_2 = 0;
					flag_3 = 0;
					flag_4 = 0;
					flag_5 = 0;
					  
					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					  
					HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
					HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
					HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
					HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
					HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);

					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					
					 skip_flag1=0;
					 skip_flag2=0;
					 skip_flag3=0;
					 skip_flag4=0;
					 skip_flag5=0;
					 Mode++;
				  }		  
		 }
			
			else if(flag_1 == 1)
			{
			  //fu2设置为全亮
			if(skip_flag1==0)
			{
			RGB_SendArray_All_1(FU_1_TIM,FU_1_CHA);
			skip_flag2=1;
			}	
			  if(flag_3 == 0)
				{
					//fu3设置
				  RGB_SendArray_Part_3(FU_3_TIM,FU_3_CHA);
				  RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
				  RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);

				  for( int time = 0; time < 9000000 ; time++)//留出时间
				  {
					if( HAL_GPIO_ReadPin(KEY_3_GPIO,KEY_3_GPIO_Port) != 1 )//判断是否成功击打
					{
						//击打成功
					  flag_3 = 1;
						break;
					}
				  }
					if(flag_3!=1)
				  {
					flag_1 = 0;
					flag_2 = 0;
					flag_3 = 0;
					flag_4 = 0;
					flag_5 = 0;
					  
					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					  
					HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
					HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
					HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
					HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
					HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);

					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					
					 skip_flag1=0;
					 skip_flag2=0;
					 skip_flag3=0;
					 skip_flag4=0;
					 skip_flag5=0;
					 Mode++;
				  }		  
		 }
				else if(flag_3 == 1)
				{
				  //fu3设置为全亮
				if(skip_flag3==0)
				{
				RGB_SendArray_All_3(FU_3_TIM,FU_3_CHA);
				skip_flag3=1;
				}
				  if(flag_4 == 0)
					{
						//fu4设置
					  RGB_SendArray_Part_4(FU_4_TIM,FU_4_CHA);
					  RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					  for( int time = 0; time < 9000000 ; time++)//留出时间
					  {
						if( HAL_GPIO_ReadPin(KEY_4_GPIO,KEY_4_GPIO_Port) != 1 )//判断是否成功击打
						{
							//击打成功
							flag_4 = 1;
							break;

							}
					   }
					if(flag_4!=1)
				  {
					flag_1 = 0;
					flag_2 = 0;
					flag_3 = 0;
					flag_4 = 0;
					flag_5 = 0;
					  
					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					  
					HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
					HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
					HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
					HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
					HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);

					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					
					 skip_flag1=0;
					 skip_flag2=0;
					 skip_flag3=0;
					 skip_flag4=0;
					 skip_flag5=0;
					 Mode++;
				  }		  
		 }				
					else if(flag_4 == 1)
					{
					  //fu4设置为全亮
					if(skip_flag4==0)
					{
					RGB_SendArray_All_4(FU_4_TIM,FU_4_CHA);
					skip_flag4=1;
					}		
					  if(flag_5 == 0)
						{
						  //fu5设置
						  RGB_SendArray_Part_5(FU_5_TIM,FU_5_CHA);
						  for( int time = 0; time < 9000000 ; time++)//留出时间
						  {
							if( HAL_GPIO_ReadPin(KEY_5_GPIO,KEY_5_GPIO_Port) != 1 )//判断是否成功击打
							{
								flag_5 = 1;
								break;
							}
						  }
					if(flag_5!=1)
				  {
					flag_1 = 0;
					flag_2 = 0;
					flag_3 = 0;
					flag_4 = 0;
					flag_5 = 0;
					  
					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					  
					HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
					HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
					HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
					HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
					HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);

					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);		  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					
					 skip_flag1=0;
					 skip_flag2=0;
					 skip_flag3=0;
					 skip_flag4=0;
					 skip_flag5=0;
					 Mode++;
				  }		  
		 }						  
						
						else if (flag_5 == 1)
						{
							RGB_SendArray_All_5(FU_5_TIM,FU_5_CHA);//fu5全亮
							HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);//测试完成
							HAL_Delay(3000);//庆祝一下
							//重新初始化
							flag_1 = 0;
							flag_2 = 0;
							flag_3 = 0;
							flag_4 = 0;
							flag_5 = 0;
							RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
							RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
							RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
							RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
							RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
							
//							HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
//							HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
//							HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
//							HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
//							HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);
//							
//							RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);		  
//							RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
//							RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
//							RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
//							RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
//							
//							HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
//							HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
//							HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
//							HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
//							HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);
							
							 skip_flag1=0;
							 skip_flag2=0;
							 skip_flag3=0;
							 skip_flag4=0;
							 skip_flag5=0;
							 Mode++;
						}
					}
				}

			}	
		}
		HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
		HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
		HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
		HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
		HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);
			
}

void Text_Fu3(void)//32145   //注释看Text_Fu理解整体流程，其他注释有问题，懒得改了
{
		HAL_GPIO_TogglePin(GPIOF, LED_GREEN_Pin);//判断是否正常运行
		if(flag_3 == 0)
		 {	
			 /*TIM2*/
			 //fu1亮部分灯
			RGB_SendArray_Part_3(FU_3_TIM,FU_3_CHA);
			//其他区域不亮灯
			RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
			RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);		
			RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
			RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);

			 for( int time = 0; time < 9000000 ; time++)//给出击打时间，避免烧坏LED，删去可能导致反复快速翻转LED。
		  {
			   if( HAL_GPIO_ReadPin(KEY_3_GPIO,KEY_3_GPIO_Port) != 1 )//判断是否成功击打
		    {
			    flag_3 = 1;//击打成功
				break;
		    }

		  }
					if(flag_3!=1)
				  {
					flag_1 = 0;
					flag_2 = 0;
					flag_3 = 0;
					flag_4 = 0;
					flag_5 = 0;
					  
					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					  
					HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
					HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
					HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
					HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
					HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);

					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					
					 skip_flag1=0;
					 skip_flag2=0;
					 skip_flag3=0;
					 skip_flag4=0;
					 skip_flag5=0;
					 Mode++;
				  }		  
		 }
    else if(flag_3 == 1)
		{
			/*TIM2*/
			//fu1设置为全亮
			if(skip_flag3==0)
			{
			RGB_SendArray_All_3(FU_3_TIM,FU_3_CHA);
			skip_flag3=1;
			}		
			if(flag_2 == 0)
		  	{
				//fu2设置部分亮，fu3、4、5不亮
				RGB_SendArray_Part_2(FU_2_TIM,FU_2_CHA);
				RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);		
				RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
				RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);

				for( int time = 0; time < 9000000 ; time++)//留出时间
				  {
					  if( HAL_GPIO_ReadPin(KEY_2_GPIO,KEY_2_GPIO_Port) != 1 )//判断是否成功击打
					{
						//击打成功
						flag_2 = 1;
						break;
					}
					
				  }
					if(flag_2!=1)
				  {
					flag_1 = 0;
					flag_2 = 0;
					flag_3 = 0;
					flag_4 = 0;
					flag_5 = 0;
					  
					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					  
					HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
					HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
					HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
					HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
					HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);

					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					
					 skip_flag1=0;
					 skip_flag2=0;
					 skip_flag3=0;
					 skip_flag4=0;
					 skip_flag5=0;
					 Mode++;
				  }		  
		 }
			
			else if(flag_2 == 1)
			{
			  //fu2设置为全亮
			if(skip_flag2==0)
			{
			RGB_SendArray_All_2(FU_2_TIM,FU_2_CHA);
			skip_flag2=1;
			}	
			  if(flag_1 == 0)
				{
					//fu3设置
				  RGB_SendArray_Part_1(FU_1_TIM,FU_1_CHA);
				  RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
				  RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);

				  for( int time = 0; time < 9000000 ; time++)//留出时间
				  {
					if( HAL_GPIO_ReadPin(KEY_1_GPIO,KEY_1_GPIO_Port) != 1 )//判断是否成功击打
					{
						//击打成功
					  flag_1 = 1;
						break;
					}
				  }
					if(flag_1!=1)
				  {
					flag_1 = 0;
					flag_2 = 0;
					flag_3 = 0;
					flag_4 = 0;
					flag_5 = 0;
					  
					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					  
					HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
					HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
					HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
					HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
					HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);

					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					
					 skip_flag1=0;
					 skip_flag2=0;
					 skip_flag3=0;
					 skip_flag4=0;
					 skip_flag5=0;
					 Mode++;
				  }		  
		 }
				else if(flag_1 == 1)
				{
				  //fu3设置为全亮
				if(skip_flag1==0)
				{
				RGB_SendArray_All_1(FU_1_TIM,FU_1_CHA);
				skip_flag1=1;
				}
				  if(flag_4 == 0)
					{
						//fu4设置
					  RGB_SendArray_Part_4(FU_4_TIM,FU_4_CHA);
					  RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					  for( int time = 0; time < 9000000 ; time++)//留出时间
					  {
						if( HAL_GPIO_ReadPin(KEY_4_GPIO,KEY_4_GPIO_Port) != 1 )//判断是否成功击打
						{
							//击打成功
							flag_4 = 1;
							break;

							}
					   }
					if(flag_4!=1)
				  {
					flag_1 = 0;
					flag_2 = 0;
					flag_3 = 0;
					flag_4 = 0;
					flag_5 = 0;
					  
					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					  
					HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
					HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
					HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
					HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
					HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);

					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					
					 skip_flag1=0;
					 skip_flag2=0;
					 skip_flag3=0;
					 skip_flag4=0;
					 skip_flag5=0;
					 Mode++;
				  }		  
		 }				
					else if(flag_4 == 1)
					{
					  //fu4设置为全亮
					if(skip_flag4==0)
					{
					RGB_SendArray_All_4(FU_4_TIM,FU_4_CHA);
					skip_flag4=1;
					}		
					  if(flag_5 == 0)
						{
						  //fu5设置
						  RGB_SendArray_Part_5(FU_5_TIM,FU_5_CHA);
						  for( int time = 0; time < 9000000 ; time++)//留出时间
						  {
							if( HAL_GPIO_ReadPin(KEY_5_GPIO,KEY_5_GPIO_Port) != 1 )//判断是否成功击打
							{
								flag_5 = 1;
								break;
							}
						  }
					if(flag_5!=1)
				  {
					flag_1 = 0;
					flag_2 = 0;
					flag_3 = 0;
					flag_4 = 0;
					flag_5 = 0;
					  
					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					  
					HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
					HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
					HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
					HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
					HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);

					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);		  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					
					 skip_flag1=0;
					 skip_flag2=0;
					 skip_flag3=0;
					 skip_flag4=0;
					 skip_flag5=0;
					 Mode++;
				  }		  
		 }						  
						
						else if (flag_5 == 1)
						{
							RGB_SendArray_All_5(FU_5_TIM,FU_5_CHA);//fu5全亮
							HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);//测试完成
							HAL_Delay(3000);//庆祝一下
							//重新初始化
							flag_1 = 0;
							flag_2 = 0;
							flag_3 = 0;
							flag_4 = 0;
							flag_5 = 0;
							RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
							RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
							RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
							RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
							RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
							
//							HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
//							HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
//							HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
//							HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
//							HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);
//							
//							RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);		  
//							RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
//							RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
//							RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
//							RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
//							
//							HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
//							HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
//							HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
//							HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
//							HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);
							
							 skip_flag1=0;
							 skip_flag2=0;
							 skip_flag3=0;
							 skip_flag4=0;
							 skip_flag5=0;
							 Mode++;
						}
					}
				}

			}	
		}
		HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
		HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
		HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
		HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
		HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);
			
}

void Text_Fu4(void)//42315   //注释看Text_Fu理解整体流程，其他注释有问题，懒得改了
{
		HAL_GPIO_TogglePin(GPIOF, LED_GREEN_Pin);//判断是否正常运行
		if(flag_4 == 0)
		 {	
			 /*TIM2*/
			 //fu1亮部分灯
			RGB_SendArray_Part_4(FU_4_TIM,FU_4_CHA);
			//其他区域不亮灯
			RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
			RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
			RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);
			RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);

			 for( int time = 0; time < 9000000 ; time++)//给出击打时间，避免烧坏LED，删去可能导致反复快速翻转LED。
		  {
			   if( HAL_GPIO_ReadPin(KEY_4_GPIO,KEY_4_GPIO_Port) != 1 )//判断是否成功击打
		    {
			    flag_4 = 1;//击打成功
				break;
		    }

		  }
					if(flag_4!=1)
				  {
					flag_1 = 0;
					flag_2 = 0;
					flag_3 = 0;
					flag_4 = 0;
					flag_5 = 0;
					  
					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					  
					HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
					HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
					HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
					HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
					HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);

					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					
					 skip_flag1=0;
					 skip_flag2=0;
					 skip_flag3=0;
					 skip_flag4=0;
					 skip_flag5=0;
					 Mode++;
				  }		  
		 }
    else if(flag_4 == 1)
		{
			/*TIM2*/
			//fu1设置为全亮
			if(skip_flag4==0)
			{
			RGB_SendArray_All_4(FU_4_TIM,FU_4_CHA);
			skip_flag1=1;
			}		
			if(flag_2 == 0)
		  	{
				//fu2设置部分亮，fu3、4、5不亮
				RGB_SendArray_Part_2(FU_2_TIM,FU_2_CHA);
				RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
				RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);
				RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);

				for( int time = 0; time < 9000000 ; time++)//留出时间
				  {
					  if( HAL_GPIO_ReadPin(KEY_2_GPIO,KEY_2_GPIO_Port) != 1 )//判断是否成功击打
					{
						//击打成功
						flag_2 = 1;
						break;
					}
					
				  }
					if(flag_2!=1)
				  {
					flag_1 = 0;
					flag_2 = 0;
					flag_3 = 0;
					flag_4 = 0;
					flag_5 = 0;
					  
					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					  
					HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
					HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
					HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
					HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
					HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);

					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					
					 skip_flag1=0;
					 skip_flag2=0;
					 skip_flag3=0;
					 skip_flag4=0;
					 skip_flag5=0;
					 Mode++;
				  }		  
		 }
			
			else if(flag_2 == 1)
			{
			  //fu2设置为全亮
			if(skip_flag2==0)
			{
			RGB_SendArray_All_2(FU_2_TIM,FU_2_CHA);
			skip_flag2=1;
			}	
			  if(flag_3 == 0)
				{
					//fu3设置
				  RGB_SendArray_Part_3(FU_3_TIM,FU_3_CHA);
				  RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);
				  RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);

				  for( int time = 0; time < 9000000 ; time++)//留出时间
				  {
					if( HAL_GPIO_ReadPin(KEY_3_GPIO,KEY_3_GPIO_Port) != 1 )//判断是否成功击打
					{
						//击打成功
					  flag_3 = 1;
						break;
					}
				  }
					if(flag_3!=1)
				  {
					flag_1 = 0;
					flag_2 = 0;
					flag_3 = 0;
					flag_4 = 0;
					flag_5 = 0;
					  
					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					  
					HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
					HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
					HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
					HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
					HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);

					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					
					 skip_flag1=0;
					 skip_flag2=0;
					 skip_flag3=0;
					 skip_flag4=0;
					 skip_flag5=0;
					 Mode++;
				  }		  
		 }
				else if(flag_3 == 1)
				{
				  //fu3设置为全亮
				if(skip_flag3==0)
				{
				RGB_SendArray_All_3(FU_3_TIM,FU_3_CHA);
				skip_flag3=1;
				}
				  if(flag_1 == 0)
					{
						//fu4设置
					  RGB_SendArray_Part_1(FU_1_TIM,FU_1_CHA);
					  RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					  for( int time = 0; time < 9000000 ; time++)//留出时间
					  {
						if( HAL_GPIO_ReadPin(KEY_1_GPIO,KEY_1_GPIO_Port) != 1 )//判断是否成功击打
						{
							//击打成功
							flag_1 = 1;
							break;

							}
					   }
					if(flag_1!=1)
				  {
					flag_1 = 0;
					flag_2 = 0;
					flag_3 = 0;
					flag_4 = 0;
					flag_5 = 0;
					  
					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					  
					HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
					HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
					HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
					HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
					HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);

					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					
					 skip_flag1=0;
					 skip_flag2=0;
					 skip_flag3=0;
					 skip_flag4=0;
					 skip_flag5=0;
					 Mode++;
				  }		  
		 }				
					else if(flag_1 == 1)
					{
					  //fu4设置为全亮
					if(skip_flag1==0)
					{
					RGB_SendArray_All_1(FU_1_TIM,FU_1_CHA);
					skip_flag1=1;
					}		
					  if(flag_5 == 0)
						{
						  //fu5设置
						  RGB_SendArray_Part_5(FU_5_TIM,FU_5_CHA);
						  for( int time = 0; time < 9000000 ; time++)//留出时间
						  {
							if( HAL_GPIO_ReadPin(KEY_5_GPIO,KEY_5_GPIO_Port) != 1 )//判断是否成功击打
							{
								flag_5 = 1;
								break;
							}
						  }
					if(flag_5!=1)
				  {
					flag_1 = 0;
					flag_2 = 0;
					flag_3 = 0;
					flag_4 = 0;
					flag_5 = 0;
					  
					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					  
					HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
					HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
					HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
					HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
					HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);

					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);		  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					
					 skip_flag1=0;
					 skip_flag2=0;
					 skip_flag3=0;
					 skip_flag4=0;
					 skip_flag5=0;
					 Mode++;
				  }		  
		 }						  
						
						else if (flag_5 == 1)
						{
							RGB_SendArray_All_5(FU_5_TIM,FU_5_CHA);//fu5全亮
							HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);//测试完成
							HAL_Delay(3000);//庆祝一下
							//重新初始化
							flag_1 = 0;
							flag_2 = 0;
							flag_3 = 0;
							flag_4 = 0;
							flag_5 = 0;
							RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
							RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
							RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
							RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
							RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
							
//							HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
//							HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
//							HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
//							HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
//							HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);
//							
//							RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);		  
//							RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
//							RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
//							RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
//							RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
//							
//							HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
//							HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
//							HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
//							HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
//							HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);
							
							 skip_flag1=0;
							 skip_flag2=0;
							 skip_flag3=0;
							 skip_flag4=0;
							 skip_flag5=0;
							 Mode++;
						}
					}
				}

			}	
		}
		HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
		HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
		HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
		HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
		HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);
			
}

void Text_Fu5(void)//52341   //注释看Text_Fu理解整体流程，其他注释有问题，懒得改了
{
		HAL_GPIO_TogglePin(GPIOF, LED_GREEN_Pin);//判断是否正常运行
		if(flag_5 == 0)
		 {	
			 /*TIM2*/
			 //fu1亮部分灯
			RGB_SendArray_Part_5(FU_5_TIM,FU_5_CHA);
			//其他区域不亮灯
			RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
			RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
			RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
			RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);

			 for( int time = 0; time < 9000000 ; time++)//给出击打时间，避免烧坏LED，删去可能导致反复快速翻转LED。
		  {
			   if( HAL_GPIO_ReadPin(KEY_5_GPIO,KEY_5_GPIO_Port) != 1 )//判断是否成功击打
		    {
			    flag_5 = 1;//击打成功
				break;
		    }

		  }
					if(flag_5!=1)
				  {
					flag_1 = 0;
					flag_2 = 0;
					flag_3 = 0;
					flag_4 = 0;
					flag_5 = 0;
					  
					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					  
					HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
					HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
					HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
					HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
					HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);

					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					
					 skip_flag1=0;
					 skip_flag2=0;
					 skip_flag3=0;
					 skip_flag4=0;
					 skip_flag5=0;
					 Mode++;
				  }		  
		 }
    else if(flag_5 == 1)
		{
			/*TIM2*/
			//fu1设置为全亮
			if(skip_flag5==0)
			{
			RGB_SendArray_All_5(FU_5_TIM,FU_5_CHA);
			skip_flag5=1;
			}		
			if(flag_2 == 0)
		  	{
				//fu2设置部分亮，fu3、4、5不亮
				RGB_SendArray_Part_2(FU_2_TIM,FU_2_CHA);
				RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
				RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
				RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);

				for( int time = 0; time < 9000000 ; time++)//留出时间
				  {
					  if( HAL_GPIO_ReadPin(KEY_2_GPIO,KEY_2_GPIO_Port) != 1 )//判断是否成功击打
					{
						//击打成功
						flag_2 = 1;
						break;
					}
					
				  }
					if(flag_2!=1)
				  {
					flag_1 = 0;
					flag_2 = 0;
					flag_3 = 0;
					flag_4 = 0;
					flag_5 = 0;
					  
					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					  
					HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
					HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
					HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
					HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
					HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);

					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					
					 skip_flag1=0;
					 skip_flag2=0;
					 skip_flag3=0;
					 skip_flag4=0;
					 skip_flag5=0;
					 Mode++;
				  }		  
		 }
			
			else if(flag_2 == 1)
			{
			  //fu2设置为全亮
			if(skip_flag2==0)
			{
			RGB_SendArray_All_2(FU_2_TIM,FU_2_CHA);
			skip_flag2=1;
			}	
			  if(flag_3 == 0)
				{
					//fu3设置
				  RGB_SendArray_Part_3(FU_3_TIM,FU_3_CHA);
				  RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
				  RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);

				  for( int time = 0; time < 9000000 ; time++)//留出时间
				  {
					if( HAL_GPIO_ReadPin(KEY_3_GPIO,KEY_3_GPIO_Port) != 1 )//判断是否成功击打
					{
						//击打成功
					  flag_3 = 1;
						break;
					}
				  }
					if(flag_3!=1)
				  {
					flag_1 = 0;
					flag_2 = 0;
					flag_3 = 0;
					flag_4 = 0;
					flag_5 = 0;
					  
					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					  
					HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
					HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
					HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
					HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
					HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);

					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					
					 skip_flag1=0;
					 skip_flag2=0;
					 skip_flag3=0;
					 skip_flag4=0;
					 skip_flag5=0;
					 Mode++;
				  }		  
		 }
				else if(flag_3 == 1)
				{
				  //fu3设置为全亮
				if(skip_flag3==0)
				{
				RGB_SendArray_All_3(FU_3_TIM,FU_3_CHA);
				skip_flag3=1;
				}
				  if(flag_4 == 0)
					{
						//fu4设置
					  RGB_SendArray_Part_4(FU_4_TIM,FU_4_CHA);
					  RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);
					  for( int time = 0; time < 9000000 ; time++)//留出时间
					  {
						if( HAL_GPIO_ReadPin(KEY_4_GPIO,KEY_4_GPIO_Port) != 1 )//判断是否成功击打
						{
							//击打成功
							flag_4 = 1;
							break;

							}
					   }
					if(flag_4!=1)
				  {
					flag_1 = 0;
					flag_2 = 0;
					flag_3 = 0;
					flag_4 = 0;
					flag_5 = 0;
					  
					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					  
					HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
					HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
					HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
					HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
					HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);

					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					
					 skip_flag1=0;
					 skip_flag2=0;
					 skip_flag3=0;
					 skip_flag4=0;
					 skip_flag5=0;
					 Mode++;
				  }		  
		 }				
					else if(flag_4 == 1)
					{
					  //fu4设置为全亮
					if(skip_flag4==0)
					{
					RGB_SendArray_All_4(FU_4_TIM,FU_4_CHA);
					skip_flag4=1;
					}		
					  if(flag_1 == 0)
						{
						  //fu5设置
						  RGB_SendArray_Part_1(FU_1_TIM,FU_1_CHA);
						  for( int time = 0; time < 9000000 ; time++)//留出时间
						  {
							if( HAL_GPIO_ReadPin(KEY_1_GPIO,KEY_1_GPIO_Port) != 1 )//判断是否成功击打
							{
								flag_1 = 1;
								break;
							}
						  }
					if(flag_1!=1)
				  {
					flag_1 = 0;
					flag_2 = 0;
					flag_3 = 0;
					flag_4 = 0;
					flag_5 = 0;
					  
					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					  
					HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
					HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
					HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
					HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
					HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);

					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);		  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					
					 skip_flag1=0;
					 skip_flag2=0;
					 skip_flag3=0;
					 skip_flag4=0;
					 skip_flag5=0;
					 Mode++;
				  }		  
		 }						  
						
						else if (flag_1 == 1)
						{
							RGB_SendArray_All_1(FU_1_TIM,FU_1_CHA);//fu5全亮
							HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);//测试完成
							HAL_Delay(3000);//庆祝一下
							//重新初始化
							flag_1 = 0;
							flag_2 = 0;
							flag_3 = 0;
							flag_4 = 0;
							flag_5 = 0;
							RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
							RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
							RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
							RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
							RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
							
//							HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
//							HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
//							HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
//							HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
//							HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);
//							
//							RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);		  
//							RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
//							RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
//							RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
//							RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
//							
//							HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
//							HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
//							HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
//							HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
//							HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);
							
							 skip_flag1=0;
							 skip_flag2=0;
							 skip_flag3=0;
							 skip_flag4=0;
							 skip_flag5=0;
							 Mode++;
						}
					}
				}

			}	
		}
		HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
		HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
		HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
		HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
		HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);
			
}

void Text_Fu6(void)//12435   //注释看Text_Fu理解整体流程，其他注释有问题，懒得改了
{
		HAL_GPIO_TogglePin(GPIOF, LED_GREEN_Pin);//判断是否正常运行
		if(flag_1 == 0)
		 {	
			 /*TIM2*/
			 //fu1亮部分灯
			RGB_SendArray_Part_1(FU_1_TIM,FU_1_CHA);
			//其他区域不亮灯
			RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
			RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
			RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
			RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);

			 for( int time = 0; time < 9000000 ; time++)//给出击打时间，避免烧坏LED，删去可能导致反复快速翻转LED。
		  {
			   if( HAL_GPIO_ReadPin(KEY_1_GPIO,KEY_1_GPIO_Port) != 1 )//判断是否成功击打
		    {
			    flag_1 = 1;//击打成功
				break;
		    }

		  }
					if(flag_1!=1)
				  {
					flag_1 = 0;
					flag_2 = 0;
					flag_3 = 0;
					flag_4 = 0;
					flag_5 = 0;
					  
					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					  
					HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
					HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
					HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
					HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
					HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);

					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					
					 skip_flag1=0;
					 skip_flag2=0;
					 skip_flag3=0;
					 skip_flag4=0;
					 skip_flag5=0;
					 Mode++;
				  }		  
		 }
    else if(flag_1 == 1)
		{
			/*TIM2*/
			//fu1设置为全亮
			if(skip_flag1==0)
			{
			RGB_SendArray_All_1(FU_1_TIM,FU_1_CHA);
			skip_flag1=1;
			}		
			if(flag_2 == 0)
		  	{
				//fu2设置部分亮，fu3、4、5不亮
				RGB_SendArray_Part_2(FU_2_TIM,FU_2_CHA);
				RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
				RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
				RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);

				for( int time = 0; time < 9000000 ; time++)//留出时间
				  {
					  if( HAL_GPIO_ReadPin(KEY_2_GPIO,KEY_2_GPIO_Port) != 1 )//判断是否成功击打
					{
						//击打成功
						flag_2 = 1;
						break;
					}
					
				  }
					if(flag_2!=1)
				  {
					flag_1 = 0;
					flag_2 = 0;
					flag_3 = 0;
					flag_4 = 0;
					flag_5 = 0;
					  
					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					  
					HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
					HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
					HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
					HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
					HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);

					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					
					 skip_flag1=0;
					 skip_flag2=0;
					 skip_flag3=0;
					 skip_flag4=0;
					 skip_flag5=0;
					 Mode++;
				  }		  
		 }
			
			else if(flag_2 == 1)
			{
			  //fu2设置为全亮
			if(skip_flag2==0)
			{
			RGB_SendArray_All_2(FU_2_TIM,FU_2_CHA);
			skip_flag2=1;
			}	
			  if(flag_4 == 0)
				{
					//fu3设置
				  RGB_SendArray_Part_4(FU_4_TIM,FU_4_CHA);
				  RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);
				  RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);

				  for( int time = 0; time < 9000000 ; time++)//留出时间
				  {
					if( HAL_GPIO_ReadPin(KEY_4_GPIO,KEY_4_GPIO_Port) != 1 )//判断是否成功击打
					{
						//击打成功
					  flag_4 = 1;
						break;
					}
				  }
					if(flag_4!=1)
				  {
					flag_1 = 0;
					flag_2 = 0;
					flag_3 = 0;
					flag_4 = 0;
					flag_5 = 0;
					  
					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					  
					HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
					HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
					HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
					HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
					HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);

					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					
					 skip_flag1=0;
					 skip_flag2=0;
					 skip_flag3=0;
					 skip_flag4=0;
					 skip_flag5=0;
					 Mode++;
				  }		  
		 }
				else if(flag_4 == 1)
				{
				  //fu3设置为全亮
				if(skip_flag4==0)
				{
				RGB_SendArray_All_4(FU_4_TIM,FU_4_CHA);
				skip_flag4=1;
				}
				  if(flag_3 == 0)
					{
						//fu4设置
					  RGB_SendArray_Part_3(FU_3_TIM,FU_3_CHA);
					  RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					  for( int time = 0; time < 9000000 ; time++)//留出时间
					  {
						if( HAL_GPIO_ReadPin(KEY_3_GPIO,KEY_3_GPIO_Port) != 1 )//判断是否成功击打
						{
							//击打成功
							flag_3 = 1;
							break;

							}
					   }
					if(flag_3!=1)
				  {
					flag_1 = 0;
					flag_2 = 0;
					flag_3 = 0;
					flag_4 = 0;
					flag_5 = 0;
					  
					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					  
					HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
					HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
					HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
					HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
					HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);

					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					
					 skip_flag1=0;
					 skip_flag2=0;
					 skip_flag3=0;
					 skip_flag4=0;
					 skip_flag5=0;
					 Mode++;
				  }		  
		 }				
					else if(flag_3 == 1)
					{
					  //fu4设置为全亮
					if(skip_flag3==0)
					{
					RGB_SendArray_All_3(FU_3_TIM,FU_3_CHA);
					skip_flag3=1;
					}		
					  if(flag_5 == 0)
						{
						  //fu5设置
						  RGB_SendArray_Part_5(FU_5_TIM,FU_5_CHA);
						  for( int time = 0; time < 9000000 ; time++)//留出时间
						  {
							if( HAL_GPIO_ReadPin(KEY_5_GPIO,KEY_5_GPIO_Port) != 1 )//判断是否成功击打
							{
								flag_5 = 1;
								break;
							}
						  }
					if(flag_5!=1)
				  {
					flag_1 = 0;
					flag_2 = 0;
					flag_3 = 0;
					flag_4 = 0;
					flag_5 = 0;
					  
					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					  
					HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
					HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
					HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
					HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
					HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);

					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);		  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					
					 skip_flag1=0;
					 skip_flag2=0;
					 skip_flag3=0;
					 skip_flag4=0;
					 skip_flag5=0;
					 Mode++;
				  }		  
		 }						  
						
						else if (flag_5 == 1)
						{
							RGB_SendArray_All_5(FU_5_TIM,FU_5_CHA);//fu5全亮
							HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);//测试完成
							HAL_Delay(3000);//庆祝一下
							//重新初始化
							flag_1 = 0;
							flag_2 = 0;
							flag_3 = 0;
							flag_4 = 0;
							flag_5 = 0;
							RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
							RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
							RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
							RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
							RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
							
//							HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
//							HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
//							HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
//							HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
//							HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);
//							
//							RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);		  
//							RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
//							RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
//							RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
//							RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
//							
//							HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
//							HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
//							HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
//							HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
//							HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);
							
							 skip_flag1=0;
							 skip_flag2=0;
							 skip_flag3=0;
							 skip_flag4=0;
							 skip_flag5=0;
							 Mode++;
						}
					}
				}

			}	
		}
		HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
		HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
		HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
		HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
		HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);
			
}

void Text_Fu7(void)//23145   //注释看Text_Fu理解整体流程，其他注释有问题，懒得改了
{
		HAL_GPIO_TogglePin(GPIOF, LED_GREEN_Pin);//判断是否正常运行
		if(flag_2 == 0)
		 {	
			 /*TIM2*/
			 //fu1亮部分灯
			RGB_SendArray_Part_2(FU_2_TIM,FU_2_CHA);
			//其他区域不亮灯
			RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);
			RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
			RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
			RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);

			 for( int time = 0; time < 9000000 ; time++)//给出击打时间，避免烧坏LED，删去可能导致反复快速翻转LED。
		  {
			   if( HAL_GPIO_ReadPin(KEY_2_GPIO,KEY_2_GPIO_Port) != 1 )//判断是否成功击打
		    {
			    flag_2 = 1;//击打成功
				break;
		    }

		  }
					if(flag_2!=1)
				  {
					flag_1 = 0;
					flag_2 = 0;
					flag_3 = 0;
					flag_4 = 0;
					flag_5 = 0;
					  
					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					  
					HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
					HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
					HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
					HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
					HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);

					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					
					 skip_flag1=0;
					 skip_flag2=0;
					 skip_flag3=0;
					 skip_flag4=0;
					 skip_flag5=0;
					 Mode++;
				  }		  
		 }
    else if(flag_2 == 1)
		{
			/*TIM2*/
			//fu1设置为全亮
			if(skip_flag2==0)
			{
			RGB_SendArray_All_2(FU_2_TIM,FU_2_CHA);
			skip_flag2=1;
			}		
			if(flag_3 == 0)
		  	{
				//fu2设置部分亮，fu3、4、5不亮
				RGB_SendArray_Part_3(FU_3_TIM,FU_3_CHA);
				RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);		
				RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
				RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);

				for( int time = 0; time < 9000000 ; time++)//留出时间
				  {
					  if( HAL_GPIO_ReadPin(KEY_3_GPIO,KEY_3_GPIO_Port) != 1 )//判断是否成功击打
					{
						//击打成功
						flag_3 = 1;
						break;
					}
					
				  }
					if(flag_3!=1)
				  {
					flag_1 = 0;
					flag_2 = 0;
					flag_3 = 0;
					flag_4 = 0;
					flag_5 = 0;
					  
					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					  
					HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
					HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
					HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
					HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
					HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);

					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					
					 skip_flag1=0;
					 skip_flag2=0;
					 skip_flag3=0;
					 skip_flag4=0;
					 skip_flag5=0;
					 Mode++;
				  }		  
		 }
			
			else if(flag_3 == 1)
			{
			  //fu2设置为全亮
			if(skip_flag3==0)
			{
			RGB_SendArray_All_3(FU_3_TIM,FU_3_CHA);
			skip_flag3=1;
			}	
			  if(flag_1 == 0)
				{
					//fu3设置
				  RGB_SendArray_Part_1(FU_1_TIM,FU_1_CHA);
				  RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
				  RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);

				  for( int time = 0; time < 9000000 ; time++)//留出时间
				  {
					if( HAL_GPIO_ReadPin(KEY_1_GPIO,KEY_1_GPIO_Port) != 1 )//判断是否成功击打
					{
						//击打成功
					  flag_1 = 1;
						break;
					}
				  }
					if(flag_1!=1)
				  {
					flag_1 = 0;
					flag_2 = 0;
					flag_3 = 0;
					flag_4 = 0;
					flag_5 = 0;
					  
					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					  
					HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
					HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
					HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
					HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
					HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);

					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					
					 skip_flag1=0;
					 skip_flag2=0;
					 skip_flag3=0;
					 skip_flag4=0;
					 skip_flag5=0;
					 Mode++;
				  }		  
		 }
				else if(flag_1 == 1)
				{
				  //fu3设置为全亮
				if(skip_flag1==0)
				{
				RGB_SendArray_All_1(FU_1_TIM,FU_1_CHA);
				skip_flag1=1;
				}
				  if(flag_4 == 0)
					{
						//fu4设置
					  RGB_SendArray_Part_4(FU_4_TIM,FU_4_CHA);
					  RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					  for( int time = 0; time < 9000000 ; time++)//留出时间
					  {
						if( HAL_GPIO_ReadPin(KEY_4_GPIO,KEY_4_GPIO_Port) != 1 )//判断是否成功击打
						{
							//击打成功
							flag_4 = 1;
							break;

							}
					   }
					if(flag_4!=1)
				  {
					flag_1 = 0;
					flag_2 = 0;
					flag_3 = 0;
					flag_4 = 0;
					flag_5 = 0;
					  
					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					  
					HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
					HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
					HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
					HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
					HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);

					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					
					 skip_flag1=0;
					 skip_flag2=0;
					 skip_flag3=0;
					 skip_flag4=0;
					 skip_flag5=0;
					 Mode++;
				  }		  
		 }				
					else if(flag_4 == 1)
					{
					  //fu4设置为全亮
					if(skip_flag4==0)
					{
					RGB_SendArray_All_4(FU_4_TIM,FU_4_CHA);
					skip_flag4=1;
					}		
					  if(flag_5 == 0)
						{
						  //fu5设置
						  RGB_SendArray_Part_5(FU_5_TIM,FU_5_CHA);
						  for( int time = 0; time < 9000000 ; time++)//留出时间
						  {
							if( HAL_GPIO_ReadPin(KEY_5_GPIO,KEY_5_GPIO_Port) != 1 )//判断是否成功击打
							{
								flag_5 = 1;
								break;
							}
						  }
					if(flag_5!=1)
				  {
					flag_1 = 0;
					flag_2 = 0;
					flag_3 = 0;
					flag_4 = 0;
					flag_5 = 0;
					  
					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					  
					HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
					HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
					HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
					HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
					HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);

					RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);		  
					RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
					RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
					RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
					RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					
					 skip_flag1=0;
					 skip_flag2=0;
					 skip_flag3=0;
					 skip_flag4=0;
					 skip_flag5=0;
					 Mode++;
				  }		  
		 }						  
						
						else if (flag_5 == 1)
						{
							RGB_SendArray_All_5(FU_5_TIM,FU_5_CHA);//fu5全亮
							HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);//测试完成
							HAL_Delay(3000);//庆祝一下
							//重新初始化
							flag_1 = 0;
							flag_2 = 0;
							flag_3 = 0;
							flag_4 = 0;
							flag_5 = 0;
							RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);			  
							RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
							RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
							RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
							RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
							
//							HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
//							HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
//							HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
//							HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
//							HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);
//							
//							RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);		  
//							RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
//							RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
//							RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
//							RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
//							
//							HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
//							HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
//							HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
//							HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
//							HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);
							
							 skip_flag1=0;
							 skip_flag2=0;
							 skip_flag3=0;
							 skip_flag4=0;
							 skip_flag5=0;
							 Mode++;
						}
					}
				}

			}	
		}
		HAL_TIM_PWM_Stop(FU_1_TIM,FU_1_CHA);
		HAL_TIM_PWM_Stop(FU_2_TIM,FU_2_CHA);
		HAL_TIM_PWM_Stop(FU_3_TIM,FU_3_CHA);
		HAL_TIM_PWM_Stop(FU_4_TIM,FU_4_CHA);
		HAL_TIM_PWM_Stop(FU_5_TIM,FU_5_CHA);
			
}



void Fake_random(void)
{

	if(seed==0)
	{
	Mode=random_time/5;
	}
	
	random_time++;
	seed=1;
	Mode=Mode%5;
	
	if(Mode==1)
	{
		Text_Fu();
	}
	else if (Mode==2)
	{
		Text_Fu2();
	}
	else if (Mode==3)
	{
		Text_Fu3();
	}
	else if (Mode==4)
	{
		Text_Fu4();
	}
	else
	{
		Text_Fu5();
	}		
}
