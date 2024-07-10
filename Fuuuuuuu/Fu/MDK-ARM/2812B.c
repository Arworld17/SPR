#include "2812B.h"
#include "fu.h"
#include "tim.h"
 
/*Some Static Colors------------------------------*/
const RGB_Color_TypeDef RED      = {255,0,0};   //显示红色RGB数据
const RGB_Color_TypeDef GREEN    = {0,255,0};
const RGB_Color_TypeDef BLUE     = {0,0,255};
const RGB_Color_TypeDef SKY      = {0,255,255};
const RGB_Color_TypeDef MAGENTA  = {255,0,220};
const RGB_Color_TypeDef YELLOW   = {127,216,0};
const RGB_Color_TypeDef OEANGE   = {127,106,0};
const RGB_Color_TypeDef BLACK    = {0,0,0};
const RGB_Color_TypeDef WHITE    = {255,255,255};

extern TIM_HandleTypeDef * WS2812B_htim;
extern uint32_t WS2812B_Channel;

/*二维数组存放最终PWM输出数组，每一行24个
数据代表一个LED，最后一行24个0代表RESET码*/
uint32_t Pixel_Buf_Part[Pixel_NUM+1][24];
uint32_t Pixel_Buf_All[Pixel_NUM+1][24];
uint32_t Pixel_Buf_Black[Pixel_NUM+1][24];

void RGB_SetColor_Black(uint16_t LedId,RGB_Color_TypeDef Color)
{
	uint8_t i; 
	if(LedId > Pixel_NUM) return; //avoid overflow 防止写入ID大于LED总数
	
	for(i=0;i<8;i++) Pixel_Buf_Black[LedId][i]   = ( (Color.G & (1 << (7 -i)))? (CODE_1):CODE_0 );//数组某一行0~7转化存放G
	for(i=8;i<16;i++) Pixel_Buf_Black[LedId][i]  = ( (Color.R & (1 << (15-i)))? (CODE_1):CODE_0 );//数组某一行8~15转化存放R
	for(i=16;i<24;i++) Pixel_Buf_Black[LedId][i] = ( (Color.B & (1 << (23-i)))? (CODE_1):CODE_0 );//数组某一行16~23转化存放B
}

void Reset_Load_Black(void)
{
	uint8_t j;
	for(j=0;j<24;j++)
	{
		Pixel_Buf_Black[Pixel_NUM][j] = 0;
	}
}


void All_Black(void)
{
	uint16_t i;
	for(i=0;i<Fu_NUM;i++)//给对应个数LED写入红色
	{
//		if(i < 0)
//			RGB_SetColor_Black(i,RED);
//		else
			RGB_SetColor_Black(i, BLACK);
	}
	Reset_Load_Black();
}


void RGB_SetColor_Part(uint16_t LedId,RGB_Color_TypeDef Color)
{
	uint8_t i; 
	if(LedId > Pixel_NUM) return; //avoid overflow 防止写入ID大于LED总数
	
	for(i=0;i<8;i++) Pixel_Buf_Part[LedId][i]   = ( (Color.G & (1 << (7 -i)))? (CODE_1):CODE_0 );//数组某一行0~7转化存放G
	for(i=8;i<16;i++) Pixel_Buf_Part[LedId][i]  = ( (Color.R & (1 << (15-i)))? (CODE_1):CODE_0 );//数组某一行8~15转化存放R
	for(i=16;i<24;i++) Pixel_Buf_Part[LedId][i] = ( (Color.B & (1 << (23-i)))? (CODE_1):CODE_0 );//数组某一行16~23转化存放B
}

void Reset_Load_Part(void)
{
	uint8_t j;
	for(j=0;j<24;j++)
	{
		Pixel_Buf_Part[Pixel_NUM][j] = 0;
	}
}


void Part(void)
{
	uint16_t i;
	for(i=0;i<Fu_NUM;i++)//给对应个数LED写入红色
	{
		if(i < Fu_NUM_Part)
			RGB_SetColor_Part(i,RED);
		else
			RGB_SetColor_Part(i, BLACK);
	}
	RGB_SetColor_Part(106,BLACK);
	RGB_SetColor_Part(107,BLACK);
	RGB_SetColor_Part(108,BLACK);
	Reset_Load_Part();
}

void RGB_SetColor_All(uint16_t LedId,RGB_Color_TypeDef Color)
{
	uint8_t i; 
	if(LedId > Pixel_NUM) return; //avoid overflow 防止写入ID大于LED总数
	
	for(i=0;i<8;i++) Pixel_Buf_All[LedId][i]   = ( (Color.G & (1 << (7 -i)))? (CODE_1):CODE_0 );//数组某一行0~7转化存放G
	for(i=8;i<16;i++) Pixel_Buf_All[LedId][i]  = ( (Color.R & (1 << (15-i)))? (CODE_1):CODE_0 );//数组某一行8~15转化存放R
	for(i=16;i<24;i++) Pixel_Buf_All[LedId][i] = ( (Color.B & (1 << (23-i)))? (CODE_1):CODE_0 );//数组某一行16~23转化存放B
}

void Reset_Load_All(void)
{
	uint8_t j;
	for(j=0;j<24;j++)
	{
		Pixel_Buf_All[Pixel_NUM][j] = 0;
	}
}


void All(void)
{
	uint16_t i;
	for(i=0;i<Fu_NUM;i++)//给对应个数LED写入红色
	{
		if(i < Fu_NUM)
			RGB_SetColor_All(i,RED);
		else
			RGB_SetColor_All(i, BLACK);
	}
	Reset_Load_Part();
}

void RGB_SendArray_Part_1(TIM_HandleTypeDef *htim, uint32_t Channel)
{
	RGB_SetColor_Part(144,BLACK);
	RGB_SetColor_Part(143,BLACK);
	HAL_Delay(30);
	HAL_TIM_PWM_Start_DMA(htim, Channel, (uint32_t *)Pixel_Buf_Part,(Pixel_NUM+1)*24);
}

void RGB_SendArray_Part_2(TIM_HandleTypeDef *htim, uint32_t Channel)
{
	RGB_SetColor_Part(145,BLACK);
	HAL_Delay(30);
	HAL_TIM_PWM_Start_DMA(htim, Channel, (uint32_t *)Pixel_Buf_Part,(Pixel_NUM+1)*24);
}

void RGB_SendArray_Part_3(TIM_HandleTypeDef *htim, uint32_t Channel)
{
	HAL_Delay(30);
	HAL_TIM_PWM_Start_DMA(htim, Channel, (uint32_t *)Pixel_Buf_Part,(Pixel_NUM+1)*24);
}

void RGB_SendArray_Part_4(TIM_HandleTypeDef *htim, uint32_t Channel)
{
	HAL_Delay(30);
	HAL_TIM_PWM_Start_DMA(htim, Channel, (uint32_t *)Pixel_Buf_Part,(Pixel_NUM+1)*24);
}

void RGB_SendArray_Part_5(TIM_HandleTypeDef *htim, uint32_t Channel)
{
	HAL_Delay(30);
	HAL_TIM_PWM_Start_DMA(htim, Channel, (uint32_t *)Pixel_Buf_Part,(Pixel_NUM+1)*24);
}

void RGB_SendArray_All_1(TIM_HandleTypeDef *htim, uint32_t Channel)
{
	HAL_Delay(30);
	HAL_TIM_PWM_Start_DMA(htim, Channel, (uint32_t *)Pixel_Buf_All,(Pixel_NUM+1)*24);
}

void RGB_SendArray_All_2(TIM_HandleTypeDef *htim, uint32_t Channel)
{
	HAL_Delay(30);
	HAL_TIM_PWM_Start_DMA(htim, Channel, (uint32_t *)Pixel_Buf_All,(Pixel_NUM+1)*24);
}

void RGB_SendArray_All_3(TIM_HandleTypeDef *htim, uint32_t Channel)
{
	HAL_Delay(30);
	HAL_TIM_PWM_Start_DMA(htim, Channel, (uint32_t *)Pixel_Buf_All,(Pixel_NUM+1)*24);
}

void RGB_SendArray_All_4(TIM_HandleTypeDef *htim, uint32_t Channel)
{
	HAL_Delay(30);
	HAL_TIM_PWM_Start_DMA(htim, Channel, (uint32_t *)Pixel_Buf_All,(Pixel_NUM+1)*24);
}

void RGB_SendArray_All_5(TIM_HandleTypeDef *htim, uint32_t Channel)
{
	static int i;
	for(i=465;i<493;i++)
	{
		RGB_SetColor_All(i, BLACK);
	}
	HAL_Delay(30);
	HAL_TIM_PWM_Start_DMA(htim, Channel, (uint32_t *)Pixel_Buf_All,(Pixel_NUM+1)*24);
}
void RGB_SendArray_Black_1(TIM_HandleTypeDef *htim, uint32_t Channel)
{
	HAL_Delay(30);
	HAL_TIM_PWM_Start_DMA(htim, Channel, (uint32_t *)Pixel_Buf_Black,(Pixel_NUM+1)*24);
}

void RGB_SendArray_Black_2(TIM_HandleTypeDef *htim, uint32_t Channel)
{
	HAL_Delay(30);
	HAL_TIM_PWM_Start_DMA(htim, Channel, (uint32_t *)Pixel_Buf_Black,(Pixel_NUM+1)*24);
}

void RGB_SendArray_Black_3(TIM_HandleTypeDef *htim, uint32_t Channel)
{
	HAL_Delay(30);
	HAL_TIM_PWM_Start_DMA(htim, Channel, (uint32_t *)Pixel_Buf_Black,(Pixel_NUM+1)*24);
}

void RGB_SendArray_Black_4(TIM_HandleTypeDef *htim, uint32_t Channel)
{
	HAL_Delay(30);
	HAL_TIM_PWM_Start_DMA(htim, Channel, (uint32_t *)Pixel_Buf_Black,(Pixel_NUM+1)*24);
}

void RGB_SendArray_Black_5(TIM_HandleTypeDef *htim, uint32_t Channel)
{
	HAL_Delay(30);
	HAL_TIM_PWM_Start_DMA(htim, Channel, (uint32_t *)Pixel_Buf_Black,(Pixel_NUM+1)*24);
}
