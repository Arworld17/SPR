
#ifndef __RGB_H__
#define __RGB_H__
 
#include "main.h"
 
/*这里是上文计算所得CCR的宏定义*/
#define CODE_1       (58)       //1码定时器计数次数
#define CODE_0       (25)       //0码定时器计数次数
 
/*建立一个定义单个LED三原色值大小的结构体*/
typedef struct
{
	uint8_t R;
	uint8_t G;
	uint8_t B;
}RGB_Color_TypeDef;
 
#define Pixel_NUM 500  //LED数量宏定义，这里我使用一个LED，（单词pixel为像素的意思）
 
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
void RGB_GREEN(uint16_t Pixel_Len);//显示绿灯
void RGB_BLUE(uint16_t Pixel_Len); //显示蓝灯
void RGB_WHITE(uint16_t Pixel_Len);//显示白灯


#endif
