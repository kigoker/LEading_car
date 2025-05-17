/*
 * imu.h
 *
 *  Created on: 2025��3��6��
 *      Author: �Ժؼ�
 */

#ifndef CODE_IMU_H_
#define CODE_IMU_H_
#include "zf_common_headfile.h"


/* ������ */
#define   GYRO_SENS             1/16.4
#define   ACCE_SENS             90.0/4096
#define   DT                    0.01

/**********��������*************/
extern volatile float FJ_Pitch;//ƫ����
extern volatile float FJ_Angle;//������

extern int16 gyro_x_act, gyro_y_act,gyro_z_act;
extern int16 acc_x_act,acc_y_act,acc_z_act;

void Gyroscope_Init(void);
void Gyroscope_GetData(void);
void Zero_Point_Detect(void);
void Get_Gyroscope_Angle(void);
void Clear_Gyroscope_Angle(void);
void Get_Gyroscope_Pitch(void);
void Clear_Gyroscope_Pitch(void);





#endif /* CODE_IMU_H_ */
