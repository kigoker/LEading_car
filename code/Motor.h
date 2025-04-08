/*
 * Motor.h
 *
 *  Created on: 2025��1��11��
 *      Author: zhuji
 */

#ifndef CODE_MOTOR_H_
#define CODE_MOTOR_H_

#include "zf_common_headfile.h"

#define MOTOR01_SPEED_PIN     ATOM0_CH4_P02_4 //01����
#define MOTOR01_DIR_PIN       P02_5                   //01������1��ת��0��ת
#define MOTOR02_SPEED_PIN     ATOM0_CH6_P02_6 //02�ҵ��
#define MOTOR02_DIR_PIN       P02_7                   //02�ҵ����1��ת��0��ת

//�����ʼ��
void Motor_Init(void);

//����ٶ�����
void Motor_Left(int pwm_L);
void Motor_Right(int pwm_R);

void wu_shua(void);

#endif /* CODE_MOTOR_H_ */
