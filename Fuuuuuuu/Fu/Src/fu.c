//ʱ�䲿�ֲ�����Ҫ�޸�

#include "fu.h"
#include "main.h"
#include "tim.h"
#include "2812B.h"
#include "gpio.h"
#include "dma.h"

#include <stdlib.h>
#include <time.h>

//�����־λ������ɹ���1
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
//�����ж��Ƿ�������
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

int seed=0;//�����ж��Ƿ�Ҫ���´�������
uint8_t Mode=0;//�ж�����Ҫ����ô��
int random_time=0;
void Text_Fu(void)//12345   //ע�Ϳ�Text_Fu����������̣�����ע�������⣬���ø���
{
		HAL_GPIO_TogglePin(GPIOF, LED_GREEN_Pin);//�ж��Ƿ���������
		if(flag_1 == 0)
		 {	
			 /*TIM2*/
			 //fu1�����ֵ�
			RGB_SendArray_Part_1(FU_1_TIM,FU_1_CHA);
			//������������
			RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
			RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
			RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
			RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);

			 for( int time = 0; time < 9000000 ; time++)//9000000����Ϊ��ʵ��2.5s
		  {
			   if( HAL_GPIO_ReadPin(KEY_1_GPIO,KEY_1_GPIO_Port) != 1 )//�ж��Ƿ�ɹ�����
		    {
			    flag_1 = 1;//����ɹ�
				break;
		    }

		  }
					if(flag_1!=1)//2.5��δ����ɹ�
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
			//fu1����Ϊȫ��
			if(skip_flag1==0)//ֻ��һ�����ݣ������˵���������
			{
			RGB_SendArray_All_1(FU_1_TIM,FU_1_CHA);
			skip_flag1=1;
			}		
			if(flag_2 == 0)
		  	{
				//fu2���ò�������fu3��4��5����
				RGB_SendArray_Part_2(FU_2_TIM,FU_2_CHA);
				RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
				RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
				RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);

				for( int time = 0; time < 9000000 ; time++)//����ʱ��
				  {
					  if( HAL_GPIO_ReadPin(KEY_2_GPIO,KEY_2_GPIO_Port) != 1 )//�ж��Ƿ�ɹ�����
					{
						//����ɹ�
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
			  //fu2����Ϊȫ��
			if(skip_flag2==0)
			{
			RGB_SendArray_All_2(FU_2_TIM,FU_2_CHA);
			skip_flag2=1;
			}	
			  if(flag_3 == 0)
				{
					//fu3����
				  RGB_SendArray_Part_3(FU_3_TIM,FU_3_CHA);
				  RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
				  RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);

				  for( int time = 0; time < 9000000 ; time++)//����ʱ��
				  {
					if( HAL_GPIO_ReadPin(KEY_3_GPIO,KEY_3_GPIO_Port) != 1 )//�ж��Ƿ�ɹ�����
					{
						//����ɹ�
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
				  //fu3����Ϊȫ��
				if(skip_flag3==0)
				{
				RGB_SendArray_All_3(FU_3_TIM,FU_3_CHA);
				skip_flag3=1;
				}
				  if(flag_4 == 0)
					{
						//fu4����
					  RGB_SendArray_Part_4(FU_4_TIM,FU_4_CHA);
					  RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					  for( int time = 0; time < 9000000 ; time++)//����ʱ��
					  {
						if( HAL_GPIO_ReadPin(KEY_4_GPIO,KEY_4_GPIO_Port) != 1 )//�ж��Ƿ�ɹ�����
						{
							//����ɹ�
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
					  //fu4����Ϊȫ��
					if(skip_flag4==0)
					{
					RGB_SendArray_All_4(FU_4_TIM,FU_4_CHA);
					skip_flag4=1;
					}		
					  if(flag_5 == 0)
						{
						  //fu5����
						  RGB_SendArray_Part_5(FU_5_TIM,FU_5_CHA);
						  for( int time = 0; time < 9000000 ; time++)//����ʱ��
						  {
							if( HAL_GPIO_ReadPin(KEY_5_GPIO,KEY_5_GPIO_Port) != 1 )//�ж��Ƿ�ɹ�����
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
							RGB_SendArray_All_5(FU_5_TIM,FU_5_CHA);//fu5ȫ��
							HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);//�������
							HAL_Delay(3000);//��ףһ��
							//���³�ʼ��
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

void Text_Fu2(void)//21345   //ע�Ϳ�Text_Fu����������̣�����ע�������⣬���ø���
{
		HAL_GPIO_TogglePin(GPIOF, LED_GREEN_Pin);//�ж��Ƿ���������
		if(flag_2 == 0)
		 {	
			 /*TIM2*/
			 //fu1�����ֵ�
			RGB_SendArray_Part_2(FU_2_TIM,FU_2_CHA);
			//������������
			RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);
			RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
			RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
			RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);

			 for( int time = 0; time < 9000000 ; time++)//��������ʱ�䣬�����ջ�LED��ɾȥ���ܵ��·������ٷ�תLED��
		  {
			   if( HAL_GPIO_ReadPin(KEY_2_GPIO,KEY_2_GPIO_Port) != 1 )//�ж��Ƿ�ɹ�����
		    {
			    flag_2 = 1;//����ɹ�
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
			//fu1����Ϊȫ��
			if(skip_flag2==0)
			{
			RGB_SendArray_All_2(FU_2_TIM,FU_2_CHA);
			skip_flag2=1;
			}		
			if(flag_1 == 0)
		  	{
				//fu2���ò�������fu3��4��5����
				RGB_SendArray_Part_1(FU_1_TIM,FU_1_CHA);
				RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
				RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
				RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);

				for( int time = 0; time < 9000000 ; time++)//����ʱ��
				  {
					  if( HAL_GPIO_ReadPin(KEY_1_GPIO,KEY_1_GPIO_Port) != 1 )//�ж��Ƿ�ɹ�����
					{
						//����ɹ�
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
			  //fu2����Ϊȫ��
			if(skip_flag1==0)
			{
			RGB_SendArray_All_1(FU_1_TIM,FU_1_CHA);
			skip_flag2=1;
			}	
			  if(flag_3 == 0)
				{
					//fu3����
				  RGB_SendArray_Part_3(FU_3_TIM,FU_3_CHA);
				  RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
				  RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);

				  for( int time = 0; time < 9000000 ; time++)//����ʱ��
				  {
					if( HAL_GPIO_ReadPin(KEY_3_GPIO,KEY_3_GPIO_Port) != 1 )//�ж��Ƿ�ɹ�����
					{
						//����ɹ�
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
				  //fu3����Ϊȫ��
				if(skip_flag3==0)
				{
				RGB_SendArray_All_3(FU_3_TIM,FU_3_CHA);
				skip_flag3=1;
				}
				  if(flag_4 == 0)
					{
						//fu4����
					  RGB_SendArray_Part_4(FU_4_TIM,FU_4_CHA);
					  RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					  for( int time = 0; time < 9000000 ; time++)//����ʱ��
					  {
						if( HAL_GPIO_ReadPin(KEY_4_GPIO,KEY_4_GPIO_Port) != 1 )//�ж��Ƿ�ɹ�����
						{
							//����ɹ�
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
					  //fu4����Ϊȫ��
					if(skip_flag4==0)
					{
					RGB_SendArray_All_4(FU_4_TIM,FU_4_CHA);
					skip_flag4=1;
					}		
					  if(flag_5 == 0)
						{
						  //fu5����
						  RGB_SendArray_Part_5(FU_5_TIM,FU_5_CHA);
						  for( int time = 0; time < 9000000 ; time++)//����ʱ��
						  {
							if( HAL_GPIO_ReadPin(KEY_5_GPIO,KEY_5_GPIO_Port) != 1 )//�ж��Ƿ�ɹ�����
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
							RGB_SendArray_All_5(FU_5_TIM,FU_5_CHA);//fu5ȫ��
							HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);//�������
							HAL_Delay(3000);//��ףһ��
							//���³�ʼ��
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

void Text_Fu3(void)//32145   //ע�Ϳ�Text_Fu����������̣�����ע�������⣬���ø���
{
		HAL_GPIO_TogglePin(GPIOF, LED_GREEN_Pin);//�ж��Ƿ���������
		if(flag_3 == 0)
		 {	
			 /*TIM2*/
			 //fu1�����ֵ�
			RGB_SendArray_Part_3(FU_3_TIM,FU_3_CHA);
			//������������
			RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
			RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);		
			RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
			RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);

			 for( int time = 0; time < 9000000 ; time++)//��������ʱ�䣬�����ջ�LED��ɾȥ���ܵ��·������ٷ�תLED��
		  {
			   if( HAL_GPIO_ReadPin(KEY_3_GPIO,KEY_3_GPIO_Port) != 1 )//�ж��Ƿ�ɹ�����
		    {
			    flag_3 = 1;//����ɹ�
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
			//fu1����Ϊȫ��
			if(skip_flag3==0)
			{
			RGB_SendArray_All_3(FU_3_TIM,FU_3_CHA);
			skip_flag3=1;
			}		
			if(flag_2 == 0)
		  	{
				//fu2���ò�������fu3��4��5����
				RGB_SendArray_Part_2(FU_2_TIM,FU_2_CHA);
				RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);		
				RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
				RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);

				for( int time = 0; time < 9000000 ; time++)//����ʱ��
				  {
					  if( HAL_GPIO_ReadPin(KEY_2_GPIO,KEY_2_GPIO_Port) != 1 )//�ж��Ƿ�ɹ�����
					{
						//����ɹ�
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
			  //fu2����Ϊȫ��
			if(skip_flag2==0)
			{
			RGB_SendArray_All_2(FU_2_TIM,FU_2_CHA);
			skip_flag2=1;
			}	
			  if(flag_1 == 0)
				{
					//fu3����
				  RGB_SendArray_Part_1(FU_1_TIM,FU_1_CHA);
				  RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
				  RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);

				  for( int time = 0; time < 9000000 ; time++)//����ʱ��
				  {
					if( HAL_GPIO_ReadPin(KEY_1_GPIO,KEY_1_GPIO_Port) != 1 )//�ж��Ƿ�ɹ�����
					{
						//����ɹ�
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
				  //fu3����Ϊȫ��
				if(skip_flag1==0)
				{
				RGB_SendArray_All_1(FU_1_TIM,FU_1_CHA);
				skip_flag1=1;
				}
				  if(flag_4 == 0)
					{
						//fu4����
					  RGB_SendArray_Part_4(FU_4_TIM,FU_4_CHA);
					  RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					  for( int time = 0; time < 9000000 ; time++)//����ʱ��
					  {
						if( HAL_GPIO_ReadPin(KEY_4_GPIO,KEY_4_GPIO_Port) != 1 )//�ж��Ƿ�ɹ�����
						{
							//����ɹ�
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
					  //fu4����Ϊȫ��
					if(skip_flag4==0)
					{
					RGB_SendArray_All_4(FU_4_TIM,FU_4_CHA);
					skip_flag4=1;
					}		
					  if(flag_5 == 0)
						{
						  //fu5����
						  RGB_SendArray_Part_5(FU_5_TIM,FU_5_CHA);
						  for( int time = 0; time < 9000000 ; time++)//����ʱ��
						  {
							if( HAL_GPIO_ReadPin(KEY_5_GPIO,KEY_5_GPIO_Port) != 1 )//�ж��Ƿ�ɹ�����
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
							RGB_SendArray_All_5(FU_5_TIM,FU_5_CHA);//fu5ȫ��
							HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);//�������
							HAL_Delay(3000);//��ףһ��
							//���³�ʼ��
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

void Text_Fu4(void)//42315   //ע�Ϳ�Text_Fu����������̣�����ע�������⣬���ø���
{
		HAL_GPIO_TogglePin(GPIOF, LED_GREEN_Pin);//�ж��Ƿ���������
		if(flag_4 == 0)
		 {	
			 /*TIM2*/
			 //fu1�����ֵ�
			RGB_SendArray_Part_4(FU_4_TIM,FU_4_CHA);
			//������������
			RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
			RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
			RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);
			RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);

			 for( int time = 0; time < 9000000 ; time++)//��������ʱ�䣬�����ջ�LED��ɾȥ���ܵ��·������ٷ�תLED��
		  {
			   if( HAL_GPIO_ReadPin(KEY_4_GPIO,KEY_4_GPIO_Port) != 1 )//�ж��Ƿ�ɹ�����
		    {
			    flag_4 = 1;//����ɹ�
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
			//fu1����Ϊȫ��
			if(skip_flag4==0)
			{
			RGB_SendArray_All_4(FU_4_TIM,FU_4_CHA);
			skip_flag1=1;
			}		
			if(flag_2 == 0)
		  	{
				//fu2���ò�������fu3��4��5����
				RGB_SendArray_Part_2(FU_2_TIM,FU_2_CHA);
				RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
				RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);
				RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);

				for( int time = 0; time < 9000000 ; time++)//����ʱ��
				  {
					  if( HAL_GPIO_ReadPin(KEY_2_GPIO,KEY_2_GPIO_Port) != 1 )//�ж��Ƿ�ɹ�����
					{
						//����ɹ�
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
			  //fu2����Ϊȫ��
			if(skip_flag2==0)
			{
			RGB_SendArray_All_2(FU_2_TIM,FU_2_CHA);
			skip_flag2=1;
			}	
			  if(flag_3 == 0)
				{
					//fu3����
				  RGB_SendArray_Part_3(FU_3_TIM,FU_3_CHA);
				  RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);
				  RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);

				  for( int time = 0; time < 9000000 ; time++)//����ʱ��
				  {
					if( HAL_GPIO_ReadPin(KEY_3_GPIO,KEY_3_GPIO_Port) != 1 )//�ж��Ƿ�ɹ�����
					{
						//����ɹ�
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
				  //fu3����Ϊȫ��
				if(skip_flag3==0)
				{
				RGB_SendArray_All_3(FU_3_TIM,FU_3_CHA);
				skip_flag3=1;
				}
				  if(flag_1 == 0)
					{
						//fu4����
					  RGB_SendArray_Part_1(FU_1_TIM,FU_1_CHA);
					  RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					  for( int time = 0; time < 9000000 ; time++)//����ʱ��
					  {
						if( HAL_GPIO_ReadPin(KEY_1_GPIO,KEY_1_GPIO_Port) != 1 )//�ж��Ƿ�ɹ�����
						{
							//����ɹ�
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
					  //fu4����Ϊȫ��
					if(skip_flag1==0)
					{
					RGB_SendArray_All_1(FU_1_TIM,FU_1_CHA);
					skip_flag1=1;
					}		
					  if(flag_5 == 0)
						{
						  //fu5����
						  RGB_SendArray_Part_5(FU_5_TIM,FU_5_CHA);
						  for( int time = 0; time < 9000000 ; time++)//����ʱ��
						  {
							if( HAL_GPIO_ReadPin(KEY_5_GPIO,KEY_5_GPIO_Port) != 1 )//�ж��Ƿ�ɹ�����
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
							RGB_SendArray_All_5(FU_5_TIM,FU_5_CHA);//fu5ȫ��
							HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);//�������
							HAL_Delay(3000);//��ףһ��
							//���³�ʼ��
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

void Text_Fu5(void)//52341   //ע�Ϳ�Text_Fu����������̣�����ע�������⣬���ø���
{
		HAL_GPIO_TogglePin(GPIOF, LED_GREEN_Pin);//�ж��Ƿ���������
		if(flag_5 == 0)
		 {	
			 /*TIM2*/
			 //fu1�����ֵ�
			RGB_SendArray_Part_5(FU_5_TIM,FU_5_CHA);
			//������������
			RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
			RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
			RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
			RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);

			 for( int time = 0; time < 9000000 ; time++)//��������ʱ�䣬�����ջ�LED��ɾȥ���ܵ��·������ٷ�תLED��
		  {
			   if( HAL_GPIO_ReadPin(KEY_5_GPIO,KEY_5_GPIO_Port) != 1 )//�ж��Ƿ�ɹ�����
		    {
			    flag_5 = 1;//����ɹ�
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
			//fu1����Ϊȫ��
			if(skip_flag5==0)
			{
			RGB_SendArray_All_5(FU_5_TIM,FU_5_CHA);
			skip_flag5=1;
			}		
			if(flag_2 == 0)
		  	{
				//fu2���ò�������fu3��4��5����
				RGB_SendArray_Part_2(FU_2_TIM,FU_2_CHA);
				RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
				RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
				RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);

				for( int time = 0; time < 9000000 ; time++)//����ʱ��
				  {
					  if( HAL_GPIO_ReadPin(KEY_2_GPIO,KEY_2_GPIO_Port) != 1 )//�ж��Ƿ�ɹ�����
					{
						//����ɹ�
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
			  //fu2����Ϊȫ��
			if(skip_flag2==0)
			{
			RGB_SendArray_All_2(FU_2_TIM,FU_2_CHA);
			skip_flag2=1;
			}	
			  if(flag_3 == 0)
				{
					//fu3����
				  RGB_SendArray_Part_3(FU_3_TIM,FU_3_CHA);
				  RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
				  RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);

				  for( int time = 0; time < 9000000 ; time++)//����ʱ��
				  {
					if( HAL_GPIO_ReadPin(KEY_3_GPIO,KEY_3_GPIO_Port) != 1 )//�ж��Ƿ�ɹ�����
					{
						//����ɹ�
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
				  //fu3����Ϊȫ��
				if(skip_flag3==0)
				{
				RGB_SendArray_All_3(FU_3_TIM,FU_3_CHA);
				skip_flag3=1;
				}
				  if(flag_4 == 0)
					{
						//fu4����
					  RGB_SendArray_Part_4(FU_4_TIM,FU_4_CHA);
					  RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);
					  for( int time = 0; time < 9000000 ; time++)//����ʱ��
					  {
						if( HAL_GPIO_ReadPin(KEY_4_GPIO,KEY_4_GPIO_Port) != 1 )//�ж��Ƿ�ɹ�����
						{
							//����ɹ�
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
					  //fu4����Ϊȫ��
					if(skip_flag4==0)
					{
					RGB_SendArray_All_4(FU_4_TIM,FU_4_CHA);
					skip_flag4=1;
					}		
					  if(flag_1 == 0)
						{
						  //fu5����
						  RGB_SendArray_Part_1(FU_1_TIM,FU_1_CHA);
						  for( int time = 0; time < 9000000 ; time++)//����ʱ��
						  {
							if( HAL_GPIO_ReadPin(KEY_1_GPIO,KEY_1_GPIO_Port) != 1 )//�ж��Ƿ�ɹ�����
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
							RGB_SendArray_All_1(FU_1_TIM,FU_1_CHA);//fu5ȫ��
							HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);//�������
							HAL_Delay(3000);//��ףһ��
							//���³�ʼ��
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

void Text_Fu6(void)//12435   //ע�Ϳ�Text_Fu����������̣�����ע�������⣬���ø���
{
		HAL_GPIO_TogglePin(GPIOF, LED_GREEN_Pin);//�ж��Ƿ���������
		if(flag_1 == 0)
		 {	
			 /*TIM2*/
			 //fu1�����ֵ�
			RGB_SendArray_Part_1(FU_1_TIM,FU_1_CHA);
			//������������
			RGB_SendArray_Black_2(FU_2_TIM,FU_2_CHA);
			RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
			RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
			RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);

			 for( int time = 0; time < 9000000 ; time++)//��������ʱ�䣬�����ջ�LED��ɾȥ���ܵ��·������ٷ�תLED��
		  {
			   if( HAL_GPIO_ReadPin(KEY_1_GPIO,KEY_1_GPIO_Port) != 1 )//�ж��Ƿ�ɹ�����
		    {
			    flag_1 = 1;//����ɹ�
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
			//fu1����Ϊȫ��
			if(skip_flag1==0)
			{
			RGB_SendArray_All_1(FU_1_TIM,FU_1_CHA);
			skip_flag1=1;
			}		
			if(flag_2 == 0)
		  	{
				//fu2���ò�������fu3��4��5����
				RGB_SendArray_Part_2(FU_2_TIM,FU_2_CHA);
				RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
				RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
				RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);

				for( int time = 0; time < 9000000 ; time++)//����ʱ��
				  {
					  if( HAL_GPIO_ReadPin(KEY_2_GPIO,KEY_2_GPIO_Port) != 1 )//�ж��Ƿ�ɹ�����
					{
						//����ɹ�
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
			  //fu2����Ϊȫ��
			if(skip_flag2==0)
			{
			RGB_SendArray_All_2(FU_2_TIM,FU_2_CHA);
			skip_flag2=1;
			}	
			  if(flag_4 == 0)
				{
					//fu3����
				  RGB_SendArray_Part_4(FU_4_TIM,FU_4_CHA);
				  RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);
				  RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);

				  for( int time = 0; time < 9000000 ; time++)//����ʱ��
				  {
					if( HAL_GPIO_ReadPin(KEY_4_GPIO,KEY_4_GPIO_Port) != 1 )//�ж��Ƿ�ɹ�����
					{
						//����ɹ�
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
				  //fu3����Ϊȫ��
				if(skip_flag4==0)
				{
				RGB_SendArray_All_4(FU_4_TIM,FU_4_CHA);
				skip_flag4=1;
				}
				  if(flag_3 == 0)
					{
						//fu4����
					  RGB_SendArray_Part_3(FU_3_TIM,FU_3_CHA);
					  RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					  for( int time = 0; time < 9000000 ; time++)//����ʱ��
					  {
						if( HAL_GPIO_ReadPin(KEY_3_GPIO,KEY_3_GPIO_Port) != 1 )//�ж��Ƿ�ɹ�����
						{
							//����ɹ�
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
					  //fu4����Ϊȫ��
					if(skip_flag3==0)
					{
					RGB_SendArray_All_3(FU_3_TIM,FU_3_CHA);
					skip_flag3=1;
					}		
					  if(flag_5 == 0)
						{
						  //fu5����
						  RGB_SendArray_Part_5(FU_5_TIM,FU_5_CHA);
						  for( int time = 0; time < 9000000 ; time++)//����ʱ��
						  {
							if( HAL_GPIO_ReadPin(KEY_5_GPIO,KEY_5_GPIO_Port) != 1 )//�ж��Ƿ�ɹ�����
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
							RGB_SendArray_All_5(FU_5_TIM,FU_5_CHA);//fu5ȫ��
							HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);//�������
							HAL_Delay(3000);//��ףһ��
							//���³�ʼ��
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

void Text_Fu7(void)//23145   //ע�Ϳ�Text_Fu����������̣�����ע�������⣬���ø���
{
		HAL_GPIO_TogglePin(GPIOF, LED_GREEN_Pin);//�ж��Ƿ���������
		if(flag_2 == 0)
		 {	
			 /*TIM2*/
			 //fu1�����ֵ�
			RGB_SendArray_Part_2(FU_2_TIM,FU_2_CHA);
			//������������
			RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);
			RGB_SendArray_Black_3(FU_3_TIM,FU_3_CHA);		
			RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
			RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);

			 for( int time = 0; time < 9000000 ; time++)//��������ʱ�䣬�����ջ�LED��ɾȥ���ܵ��·������ٷ�תLED��
		  {
			   if( HAL_GPIO_ReadPin(KEY_2_GPIO,KEY_2_GPIO_Port) != 1 )//�ж��Ƿ�ɹ�����
		    {
			    flag_2 = 1;//����ɹ�
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
			//fu1����Ϊȫ��
			if(skip_flag2==0)
			{
			RGB_SendArray_All_2(FU_2_TIM,FU_2_CHA);
			skip_flag2=1;
			}		
			if(flag_3 == 0)
		  	{
				//fu2���ò�������fu3��4��5����
				RGB_SendArray_Part_3(FU_3_TIM,FU_3_CHA);
				RGB_SendArray_Black_1(FU_1_TIM,FU_1_CHA);		
				RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
				RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);

				for( int time = 0; time < 9000000 ; time++)//����ʱ��
				  {
					  if( HAL_GPIO_ReadPin(KEY_3_GPIO,KEY_3_GPIO_Port) != 1 )//�ж��Ƿ�ɹ�����
					{
						//����ɹ�
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
			  //fu2����Ϊȫ��
			if(skip_flag3==0)
			{
			RGB_SendArray_All_3(FU_3_TIM,FU_3_CHA);
			skip_flag3=1;
			}	
			  if(flag_1 == 0)
				{
					//fu3����
				  RGB_SendArray_Part_1(FU_1_TIM,FU_1_CHA);
				  RGB_SendArray_Black_4(FU_4_TIM,FU_4_CHA);
				  RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);

				  for( int time = 0; time < 9000000 ; time++)//����ʱ��
				  {
					if( HAL_GPIO_ReadPin(KEY_1_GPIO,KEY_1_GPIO_Port) != 1 )//�ж��Ƿ�ɹ�����
					{
						//����ɹ�
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
				  //fu3����Ϊȫ��
				if(skip_flag1==0)
				{
				RGB_SendArray_All_1(FU_1_TIM,FU_1_CHA);
				skip_flag1=1;
				}
				  if(flag_4 == 0)
					{
						//fu4����
					  RGB_SendArray_Part_4(FU_4_TIM,FU_4_CHA);
					  RGB_SendArray_Black_5(FU_5_TIM,FU_5_CHA);
					  for( int time = 0; time < 9000000 ; time++)//����ʱ��
					  {
						if( HAL_GPIO_ReadPin(KEY_4_GPIO,KEY_4_GPIO_Port) != 1 )//�ж��Ƿ�ɹ�����
						{
							//����ɹ�
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
					  //fu4����Ϊȫ��
					if(skip_flag4==0)
					{
					RGB_SendArray_All_4(FU_4_TIM,FU_4_CHA);
					skip_flag4=1;
					}		
					  if(flag_5 == 0)
						{
						  //fu5����
						  RGB_SendArray_Part_5(FU_5_TIM,FU_5_CHA);
						  for( int time = 0; time < 9000000 ; time++)//����ʱ��
						  {
							if( HAL_GPIO_ReadPin(KEY_5_GPIO,KEY_5_GPIO_Port) != 1 )//�ж��Ƿ�ɹ�����
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
							RGB_SendArray_All_5(FU_5_TIM,FU_5_CHA);//fu5ȫ��
							HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);//�������
							HAL_Delay(3000);//��ףһ��
							//���³�ʼ��
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
