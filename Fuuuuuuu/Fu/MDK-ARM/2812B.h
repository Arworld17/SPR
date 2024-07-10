
#ifndef __RGB_H__
#define __RGB_H__
 
#include "main.h"
 
/*���������ļ�������CCR�ĺ궨��*/
#define CODE_1       (58)       //1�붨ʱ����������
#define CODE_0       (25)       //0�붨ʱ����������
 
/*����һ�����嵥��LED��ԭɫֵ��С�Ľṹ��*/
typedef struct
{
	uint8_t R;
	uint8_t G;
	uint8_t B;
}RGB_Color_TypeDef;
 
#define Pixel_NUM 500  //LED�����궨�壬������ʹ��һ��LED��������pixelΪ���ص���˼��
 
void RGB_SetColor_Part(uint16_t LedId,RGB_Color_TypeDef Color);
void Reset_Load_Part(void);
void Part(void);
void RGB_SetColor_All(uint16_t LedId,RGB_Color_TypeDef Color);
void Reset_Load_All(void);
void All(void);
void RGB_SetColor_Black(uint16_t LedId,RGB_Color_TypeDef Color);
void Reset_Load_Black(void);
void All_Black(void);

void RGB_SendArray_Part_1(TIM_HandleTypeDef *htim, uint32_t Channel);
void RGB_SendArray_Part_2(TIM_HandleTypeDef *htim, uint32_t Channel);
void RGB_SendArray_Part_3(TIM_HandleTypeDef *htim, uint32_t Channel);
void RGB_SendArray_Part_4(TIM_HandleTypeDef *htim, uint32_t Channel);
void RGB_SendArray_Part_5(TIM_HandleTypeDef *htim, uint32_t Channel);

void RGB_SendArray_All_1(TIM_HandleTypeDef *htim, uint32_t Channel);
void RGB_SendArray_All_2(TIM_HandleTypeDef *htim, uint32_t Channel);
void RGB_SendArray_All_3(TIM_HandleTypeDef *htim, uint32_t Channel);
void RGB_SendArray_All_4(TIM_HandleTypeDef *htim, uint32_t Channel);
void RGB_SendArray_All_5(TIM_HandleTypeDef *htim, uint32_t Channel);

void RGB_SendArray_Black_1(TIM_HandleTypeDef *htim, uint32_t Channel);
void RGB_SendArray_Black_2(TIM_HandleTypeDef *htim, uint32_t Channel);
void RGB_SendArray_Black_3(TIM_HandleTypeDef *htim, uint32_t Channel);
void RGB_SendArray_Black_4(TIM_HandleTypeDef *htim, uint32_t Channel);
void RGB_SendArray_Black_5(TIM_HandleTypeDef *htim, uint32_t Channel);
	
void RGB_BLACK(TIM_HandleTypeDef *htim, uint32_t Channel, uint16_t Pixel_Len);
void RGB_GREEN(uint16_t Pixel_Len);//��ʾ�̵�
void RGB_BLUE(uint16_t Pixel_Len); //��ʾ����
void RGB_WHITE(uint16_t Pixel_Len);//��ʾ�׵�


#endif
