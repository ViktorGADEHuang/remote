/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       remote_control.c/h
  * @brief      遥控器处理，遥控器是通过类似SBUS的协议传输，利用DMA传输方式节约CPU
  *             资源，利用串口空闲中断来拉起处理函数，同时提供一些掉线重启DMA，串口
  *             的方式保证热插拔的稳定性。
  * @note       1.该任务是通过串口中断启动，不是freeRTOS任务
	            2.遥控器数据结构体中有关键盘的部分采用了位域
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-01-2019     RM              1. 完成
  *  V2.0.0     Dec-09-2020     HL              2. 位域部分未测试
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "remote_control.h"

#include "main.h"

#include "detect_task.h"

#include "bsp_rc.h"

#include "string.h"

#include "stdio.h"

#define RC_CHANNAL_ERROR_VALUE                           700



#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }
		
		
#if (DBUS_USART == 3)
	extern UART_HandleTypeDef huart3;
	extern DMA_HandleTypeDef hdma_usart3_rx;
#elif (DBUS_USART == 1)
	extern UART_HandleTypeDef huart1;
	extern DMA_HandleTypeDef hdma_usart1_rx;
#endif

static int16_t RC_abs(int16_t value);

/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

//remote control data 
//遥控器控制变量
RC_ctrl_t rc_ctrl;



//receive data, 18 bytes one frame, but set 36 bytes 
//接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

void slove_RC_lost(void);

void slove_data_error(void);

/**
  * @brief          remote control init
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          遥控器初始化
  * @param[in]      none
  * @retval         none
  */
void remote_control_init(void)
{
    RC_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}
/**
  * @brief          get remote control data point
  * @param[in]      none
  * @retval         remote control data point
  */
/**
  * @brief          获取遥控器数据指针
  * @param[in]      none
  * @retval         遥控器数据指针
  */
const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}


//bool_t RC_data_is_error(void)
//{
//   
//    if (RC_abs(rc_ctrl.rc.ch0) > RC_CHANNAL_ERROR_VALUE)
//    {
//        goto error;
//    }
//    if (RC_abs(rc_ctrl.rc.ch1) > RC_CHANNAL_ERROR_VALUE)
//    {
//        goto error;
//    }
//    if (RC_abs(rc_ctrl.rc.ch2) > RC_CHANNAL_ERROR_VALUE)
//    {
//        goto error;
//    }
//    if (RC_abs(rc_ctrl.rc.ch3) > RC_CHANNAL_ERROR_VALUE)
//    {
//        goto error;
//    }
//    if (0 == rc_ctrl.rc.s_l)
//    {
//        goto error;
//    }
//    if (0 == rc_ctrl.rc.s_r)
//    {
//        goto error;
//    }
//		if (1 == error_list[DBUS_TOE].is_lost)
//		{
//			  goto error;
//		}
//    return 0;

//error:
//    rc_ctrl.rc.ch0 = 0;
//    rc_ctrl.rc.ch1 = 0;
//    rc_ctrl.rc.ch2 = 0;
//    rc_ctrl.rc.ch3 = 0;
//    rc_ctrl.rc.s_l = 0;
//    rc_ctrl.rc.s_r = 0;
//    rc_ctrl.mouse.vx = 0;
//    rc_ctrl.mouse.vy = 0;
//    rc_ctrl.mouse.vz = 0;
//    rc_ctrl.mouse.press_l = 0;
//    rc_ctrl.mouse.press_r = 0;
//    return 1;
//}




/**
  * @brief          遥控器数据错误处理,将遥控器所有数据清零
  * @param[in]      none
  * @retval         none
  */
bool_t RC_data_is_error_handle(void)
{
    if (RC_abs(rc_ctrl.rc.ch0) > RC_CHANNAL_ERROR_VALUE)
    {
        goto rc_error_handle;
    }
    if (RC_abs(rc_ctrl.rc.ch1) > RC_CHANNAL_ERROR_VALUE)
    {
        goto rc_error_handle;
    }
    if (RC_abs(rc_ctrl.rc.ch2) > RC_CHANNAL_ERROR_VALUE)
    {
        goto rc_error_handle;
    }
    if (RC_abs(rc_ctrl.rc.ch3) > RC_CHANNAL_ERROR_VALUE)
    {
        goto rc_error_handle;
    }
    if (0 == rc_ctrl.rc.s_l)
    {
        goto rc_error_handle;
    }
    if (0 == rc_ctrl.rc.s_r)
    {
        goto rc_error_handle;
    }
    return 0;

rc_error_handle:
    memset(&rc_ctrl, 0, sizeof(RC_ctrl_t));
    return 1;
}



/**
  * @brief          遥控器掉线处理,将遥控器所有数据清零
  * @param[in]      none
  * @retval         none
  */
bool_t RC_offline_data_handle(void)
{
	memset(&rc_ctrl, 0, sizeof(RC_ctrl_t));
	return 0;
}



/**
  * @brief          遥控器掉线处理,软件重连
  * @param[in]      none
  * @retval         none
  */
void RC_connect_soft_restart(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}



#if (DBUS_USART == 3)
	//串口3中断
	void USART3_IRQHandler(void)
	{
			if (huart3.Instance->SR & UART_FLAG_RXNE)
			{
					__HAL_UART_CLEAR_PEFLAG(&huart3);
			}
			else if (USART3->SR & UART_FLAG_IDLE)
			{
					static uint16_t this_time_rx_len = 0;

					__HAL_UART_CLEAR_PEFLAG(&huart3);

					if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
					{
							/* Current memory buffer used is Memory 0 */

							// disable DMA

							__HAL_DMA_DISABLE(&hdma_usart3_rx);

							// get receive data length, length = set_data_length - remain_length

							this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

							// reset set_data_lenght

							hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

							// set memory buffer 1

							hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;

							// enable DMA

							__HAL_DMA_ENABLE(&hdma_usart3_rx);

							if (this_time_rx_len == RC_FRAME_LENGTH)
							{
									sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);

									#ifdef USE_DETECT
										communication_frame_rate_update(DBUS_TOE);
									#endif
							}
					}
					else
					{
							/* Current memory buffer used is Memory 1 */
							// disable DMA

							__HAL_DMA_DISABLE(&hdma_usart3_rx);

							// get receive data length, length = set_data_length - remain_length

							this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

							// reset set_data_lenght

							hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

							// set memory buffer 0

							DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

							// enable DMA

							__HAL_DMA_ENABLE(&hdma_usart3_rx);

							if (this_time_rx_len == RC_FRAME_LENGTH)
							{

									sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);

									#ifdef USE_DETECT
										communication_frame_rate_update(DBUS_TOE);
									#endif
							}
					}
			}
	}

#elif (DBUS_USART == 1)
	//????
	void USART1_IRQHandler(void)
	{
			if(huart1.Instance->SR & UART_FLAG_RXNE)//?????
			{
					__HAL_UART_CLEAR_PEFLAG(&huart1);
			}
			else if(USART1->SR & UART_FLAG_IDLE)
			{
					static uint16_t this_time_rx_len = 0;

					__HAL_UART_CLEAR_PEFLAG(&huart1);

					if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET)
					{
							/* Current memory buffer used is Memory 0 */
			
							//disable DMA
							__HAL_DMA_DISABLE(&hdma_usart1_rx);

							//get receive data length, length = set_data_length - remain_length
							this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

							//reset set_data_lenght
							hdma_usart1_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

							//set memory buffer 1
							hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;
							
							//enable DMA
							__HAL_DMA_ENABLE(&hdma_usart1_rx);

							if(this_time_rx_len == RC_FRAME_LENGTH)
							{
									sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
									#ifdef USE_DETECT
										communication_frame_rate_update(DBUS_TOE);
									#endif
							}
					}
					else
					{
							/* Current memory buffer used is Memory 1 */
							//disable DMA
							__HAL_DMA_DISABLE(&hdma_usart1_rx);

							//get receive data length, length = set_data_length - remain_length
							this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

							//reset set_data_lenght
							hdma_usart1_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

							//set memory buffer 0
							DMA2_Stream2->CR &= ~(DMA_SxCR_CT);
							
							//enable DMA
							__HAL_DMA_ENABLE(&hdma_usart1_rx);

							if(this_time_rx_len == RC_FRAME_LENGTH)
							{
									sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
									#ifdef USE_DETECT
										communication_frame_rate_update(DBUS_TOE);
									#endif
							}
					}
			}
	}
#endif




static int16_t RC_abs(int16_t value)
{
    if (value > 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}


/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->rc.ch0 = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch1 = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch2 = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                       (sbus_buf[4] << 10)) &
                      0x07ff;
    rc_ctrl->rc.ch3 = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s_r = ((sbus_buf[5] >> 4) & 0x0003);                      //!< Switch left
    rc_ctrl->rc.s_l = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                 //!< Switch right
    rc_ctrl->rc.wheel = sbus_buf[16] | (sbus_buf[17] << 8);               // NULL

    rc_ctrl->mouse.vx = sbus_buf[6] | (sbus_buf[7] << 8);   //!< Mouse X axis
    rc_ctrl->mouse.vy = sbus_buf[8] | (sbus_buf[9] << 8);   //!< Mouse Y axis
    rc_ctrl->mouse.vz = sbus_buf[10] | (sbus_buf[11] << 8); //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                  //!< Mouse Right Is Press ?

    rc_ctrl->keyboard.v = sbus_buf[14] | (sbus_buf[15] << 8); //!< KeyBoard value
    rc_ctrl->keyboard.key_W = (sbus_buf[14] >> 0) & 0x01;
    rc_ctrl->keyboard.key_S = (sbus_buf[14] >> 1) & 0x01;
    rc_ctrl->keyboard.key_A = (sbus_buf[14] >> 2) & 0x01;
    rc_ctrl->keyboard.key_D = (sbus_buf[14] >> 3) & 0x01;
    rc_ctrl->keyboard.key_Shift = (sbus_buf[14] >> 4) & 0x01;
    rc_ctrl->keyboard.key_CTRL = (sbus_buf[14] >> 5) & 0x01;
    rc_ctrl->keyboard.key_Q = (sbus_buf[14] >> 6) & 0x01;
    rc_ctrl->keyboard.key_E = (sbus_buf[14] >> 7) & 0x01;

    rc_ctrl->keyboard.key_R = (sbus_buf[15] >> 0) & 0x01;
    rc_ctrl->keyboard.key_F = (sbus_buf[15] >> 1) & 0x01;
    rc_ctrl->keyboard.key_G = (sbus_buf[15] >> 2) & 0x01;
    rc_ctrl->keyboard.key_Z = (sbus_buf[15] >> 3) & 0x01;
    rc_ctrl->keyboard.key_X = (sbus_buf[15] >> 4) & 0x01;
    rc_ctrl->keyboard.key_C = (sbus_buf[15] >> 5) & 0x01;
    rc_ctrl->keyboard.key_V = (sbus_buf[15] >> 6) & 0x01;
    rc_ctrl->keyboard.key_B = (sbus_buf[15] >> 7) & 0x01;

    rc_ctrl->rc.ch0 -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch1 -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch2 -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch3 -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.wheel -= RC_CH_VALUE_OFFSET;

    //		if((rc_ctrl->keyboard.key_Z == 1) && (rc_ctrl->keyboard.last_Z == 0))                            //flag  切换模式的按键
    //		{
    //			rc_ctrl->keyboard.flag_Z =! rc_ctrl->keyboard.flag_Z;
    //		}
    //		rc_ctrl->keyboard.last_Z =rc_ctrl->keyboard.key_Z;
    //
    		if((rc_ctrl->keyboard.key_X == 1) && (rc_ctrl->keyboard.last_X == 0))                            //flag  切换模式的按键
    		{
    			rc_ctrl->keyboard.flag_X =! rc_ctrl->keyboard.flag_X;
    		}
    		rc_ctrl->keyboard.last_X =rc_ctrl->keyboard.key_X;
    //
    //		if((rc_ctrl->keyboard.key_C == 1) && (rc_ctrl->keyboard.last_C == 0))                            //flag  切换模式的按键
    //		{
    //			rc_ctrl->keyboard.flag_C =! rc_ctrl->keyboard.flag_C;
    //		}
    //		rc_ctrl->keyboard.last_C =rc_ctrl->keyboard.key_C;
    //
    //		if((rc_ctrl->keyboard.key_V == 1) && (rc_ctrl->keyboard.last_V == 0))                            //flag  切换模式的按键
    //		{
    //			rc_ctrl->keyboard.flag_V =! rc_ctrl->keyboard.flag_V;
    //		}
    //		rc_ctrl->keyboard.last_V =rc_ctrl->keyboard.key_V;

    rc_ctrl->keyboard.last_Q = rc_ctrl->keyboard.key_Q;
    rc_ctrl->keyboard.last_F = rc_ctrl->keyboard.key_F;
    rc_ctrl->keyboard.last_G = rc_ctrl->keyboard.key_G;
}


