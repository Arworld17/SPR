#ifndef __FU_H__
#define __FU_H__


#define FU_1_TIM &htim2
#define FU_2_TIM &htim2
#define FU_3_TIM &htim2
#define FU_4_TIM &htim4
#define FU_5_TIM &htim4

#define FU_1_CHA TIM_CHANNEL_1
#define FU_2_CHA TIM_CHANNEL_2
#define FU_3_CHA TIM_CHANNEL_3
#define FU_4_CHA TIM_CHANNEL_1
#define FU_5_CHA TIM_CHANNEL_2

#define Fu_NUM_Part 145
#define Fu_NUM 493

extern int flag_1 ;
extern int flag_2 ;
extern int flag_3 ;
extern int flag_4 ;
extern int flag_5 ;

void Fake_random(void);
void Text_Fu(void);
void Text_Fu2(void);
void Text_Fu3(void);
void Text_Fu4(void);
void Text_Fu5(void);
void Text_Fu6(void);
void Text_Fu7(void);
#endif
