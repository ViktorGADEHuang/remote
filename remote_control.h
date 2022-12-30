/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       remote_control.c/h
  * @brief      ң��������ң������ͨ������SBUS��Э�鴫�䣬����DMA���䷽ʽ��ԼCPU
  *             ��Դ�����ô��ڿ����ж�������������ͬʱ�ṩһЩ��������DMA������
  *             �ķ�ʽ��֤�Ȳ�ε��ȶ��ԡ�
  * @note       1.��������ͨ�������ж�����������freeRTOS����
	            2.ң�������ݽṹ�����йؼ��̵Ĳ��ֲ�����λ��
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-01-2019     RM              1. ���
  *  V2.0.0     Dec-09-2020     HL              2. λ�򲿷�δ����
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include "struct_typedef.h"

#include "bsp_rc.h"



/** ��ң������Ϣͼ��
 * ����������������������������������������������������������������������������������������
 * �� ����������1  660                  660  ����������1 ��
 * �� ��S l��3   ��                    ��   ��S r��3 ��
 * �� ����������2   ��                    ��   ����������2 ��
 * ��          ��                    ��          ��
 * �� CH2<�����������੤��������>660  CH0<�����������੤��������>660 ��
 * ��          ��                    ��          ��
 * ��          ��                    ��          ��
 * ��          V                    V          ��
 * ��         CH3                  CH1         ��
 * ����������������������������������������������������������������������������������������
 */
 
 

#define SBUS_RX_BUF_NUM 36u

#define RC_FRAME_LENGTH 18u

#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define SWITCH_UP                ((uint16_t)1)
#define SWITCH_MIDDLE               ((uint16_t)3)
#define SWITCH_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)

#define WHEEL_MAX_RIGHT 660
#define WHEEL_MIN_LEFT -660

#define RC_CHANNEL_MAX 660
#define RC_CHANNEL_MIN -660

#define KEY_DOWN 0 
#define KEY_UP 1

#define FLAG_ON 0 
#define FLAG_OFF 1

#define REMOTE_MODE 0
#define KEYBORD_MODE 1

/* ----------------------- Data Struct ------------------------------------- */
typedef struct
{
	struct
	{
		short ch0;
		short ch1;
		short ch2;
		short ch3;
		short wheel;
		char s_l;
		char s_r;
	}rc;
	
	struct 
	{
		short vx;
		short vy;
		short vz;
		unsigned char press_l;
		unsigned char press_r;
	}mouse;
	
	struct
	{
		unsigned short v;
		uint8_t key_W : 1;
		uint8_t key_S : 1;
		uint8_t key_A : 1;
		uint8_t key_D : 1;
		uint8_t key_Shift : 1;
		uint8_t key_CTRL : 1;
		uint8_t key_Q : 1;
		uint8_t key_E : 1;
		
		uint8_t key_R : 1;
		uint8_t key_F : 1;
		uint8_t key_G : 1;
		uint8_t key_Z : 1;
		uint8_t key_X : 1;
		uint8_t key_C : 1;
		uint8_t key_V : 1;
		uint8_t key_B : 1;
		
		uint8_t last_W : 1;
		uint8_t last_S : 1;
		uint8_t last_A : 1;
		uint8_t last_D : 1;
		uint8_t last_Shift : 1;
		uint8_t last_CTRL : 1;
		uint8_t last_Q : 1;
		uint8_t last_E : 1;
		
		uint8_t last_R : 1;
		uint8_t last_F : 1;
		uint8_t last_G : 1;
		uint8_t last_Z : 1;
		uint8_t last_X : 1;
		uint8_t last_C : 1;
		uint8_t last_V : 1;
		uint8_t last_B : 1;
		
		uint8_t flag_X;

	}keyboard;

} RC_ctrl_t;

/* ----------------------- Internal Data ----------------------------------- */

/**
  * @brief          remote control init
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ң������ʼ��
  * @param[in]      none
  * @retval         none
  */
extern void remote_control_init(void);
/**
  * @brief          get remote control data point
  * @param[in]      none
  * @retval         remote control data point
  */
/**
  * @brief          ��ȡң��������ָ��
  * @param[in]      none
  * @retval         ң��������ָ��
  */
extern const RC_ctrl_t *get_remote_control_point(void);



/**
  * @brief          ң�������ݴ�����,��ң����������������
  * @param[in]      none
  * @retval         none
  */
bool_t RC_data_is_error_handle(void);



/**
  * @brief          ң�������ߴ���,��ң����������������
  * @param[in]      none
  * @retval         none
  */
bool_t RC_offline_data_handle(void);


/**
  * @brief          ң�������ߴ���,�������
  * @param[in]      none
  * @retval         none
  */
void RC_connect_soft_restart(void);


#endif
