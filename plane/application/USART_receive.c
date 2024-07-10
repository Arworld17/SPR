/**
 * @file USART_receive.c
 * @author 何清华
 * @brief 串口中断接收函数，用来处理单片机与其他设备的串口通信数据
 * @version 0.1
 * @date 2022-03-29
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "USART_receive.h"
#include "bsp_usart.h"
#include "cmsis_os.h"
#include "main.h"
#include "detect_task.h"
#include "referee.h"

extern UART_HandleTypeDef huart1;
extern ext_game_robot_state_t robot_state;  //步兵等级


auto_shoot_t auto_shoot = {0.0f, 0.0f, 0}; //自瞄数据
user_send_data_t user_send_data;
int16_t c_switch = 7;
//接收原始数据，为10个字节，给了18个字节长度，防止DMA传输越界
uint8_t usart1_rx_buf[2][USART1_RX_BUF_NUM];

/**
 * @brief 用户数据解包
 *
 * @param buf 串口接收数据指针
 * @param auto_shoot 自瞄数据结构指针
 */
void user_data_solve(volatile const uint8_t *buf, auto_shoot_t *auto_shoot);

void user_usart_init(void)
{
    usart1_init(usart1_rx_buf[0], usart1_rx_buf[1], USART1_RX_BUF_NUM);
}

void USART1_IRQHandler(void)
{
    if (huart1.Instance->SR & UART_FLAG_RXNE) //接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);
    }
    else if (USART1->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart1);

        if ((huart1.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */
            //失效DMA
            __HAL_DMA_DISABLE(huart1.hdmarx);
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = USART1_RX_BUF_NUM - huart1.hdmarx->Instance->NDTR;
            //重新设定数据长度
            huart1.hdmarx->Instance->NDTR = USART1_RX_BUF_NUM;
            //设定缓冲区1
            huart1.hdmarx->Instance->CR |= DMA_SxCR_CT;
            //使能DMA
            __HAL_DMA_ENABLE(huart1.hdmarx);

            if (this_time_rx_len == USER_FRAME_LENGTH)
            {
                //读取数据
                user_data_solve(usart1_rx_buf[0], &auto_shoot);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //失效DMA
            __HAL_DMA_DISABLE(huart1.hdmarx);
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = USART1_RX_BUF_NUM - huart1.hdmarx->Instance->NDTR;
            //重新设定数据长度
            huart1.hdmarx->Instance->NDTR = USART1_RX_BUF_NUM;
            //设定缓冲区0
            huart1.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
            //使能DMA
            __HAL_DMA_ENABLE(huart1.hdmarx);
            if (this_time_rx_len == USER_FRAME_LENGTH)
            {
                //读取数据
                user_data_solve(usart1_rx_buf[1], &auto_shoot);
            }
        }
    }
}

/**
 * @brief 用户数据解包
 *
 * @param buf 串口接收数据指针
 * @param auto_shoot 自瞄数据结构指针
 */
void user_data_solve(volatile const uint8_t *buf, auto_shoot_t *auto_shoot)
{
    //校验
    if (buf[0] == 0xFF && buf[5] == 0xFE)
    {
        auto_shoot->pitch_add = (0.0001f * ((float)(buf[2] << 8 | buf[1]))) - USART_PI;
        auto_shoot->yaw_add = (0.0001f * ((float)(buf[4] << 8 | buf[3]))) - USART_PI;
        detect_hook(USER_USART_DATA_TOE);
        auto_shoot->new_usart_data_flag = 1;
    }
}

void user_data_pack_handle()
{
    static uint8_t tx_buf[8];
    if (get_robot_id() <= 7)
    {
        user_send_data.enemy_color = 0; //红色
    }
    else
    {
        user_send_data.enemy_color = 1; //蓝色
    }
//判断步兵等级
    if( robot_state.robot_level==1 )
    {
        user_send_data.level=1;
    }
    else if( robot_state.robot_level==2 )
    {
        user_send_data.level=2;
    }
    else if( robot_state.robot_level==3 )
    {
        user_send_data.level=3;
    }
    else
    {
        user_send_data.level=0;
    }

    tx_buf[0] = 0xFF;

    // tx_buf[1] = ((uint16_t)(10000 * (*user_send_data.gimbal_pitch_angle + USART_PI)) >> 8);
    // tx_buf[2] = (uint16_t)(10000 * (*user_send_data.gimbal_pitch_angle + USART_PI));
    // tx_buf[3] = ((uint16_t)(10000 * (*user_send_data.gimbal_yaw_gyro + USART_PI)) >> 8);
    // tx_buf[4] = (uint16_t)(10000 * (*user_send_data.gimbal_yaw_gyro + USART_PI));

    tx_buf[1] = user_send_data.enemy_color;
    //tx_buf[1] = user_send_data.level;
    tx_buf[2] = 0xFE;

    usart1_tx_dma_enable(tx_buf, 3);
}
