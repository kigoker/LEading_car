/*
 * Wireless_transmission.c
 *
 *  Created on: 2025年1月24日
 *      Author: zhuji
 */
#include "Wireless_transmission.h"
uint8 data_buffer_01[200];
uint8 data_len_01;
void Send_information_VOFA(void)
{
//    printf("%d,%d\n",speed1,speed2);
    printf("%d,%d,%d,%d,%f,%f,%d,%f,%d,%d\n",speed1*6,speed2*6,Increase1,Increase2,erspeed,divertion,imu660ra_gyro_x,steer1,zhidaoflag,test_Speed);
//    printf("%d,%d,%d,%d,%d,%d\n",imu660ra_gyro_x,imu660ra_gyro_y,imu660ra_gyro_z,imu660ra_acc_x,imu660ra_acc_y,imu660ra_acc_z);
//    printf("%d,%d,%d,%d,%d,%d\n",gyro_x_act,gyro_y_act,gyro_z_act,imu660rb_acc_x,imu660rb_acc_y,imu660rb_acc_z);
//    printf("%d,%d,%d\n",imu660ra_gyro_x,imu660ra_gyro_y,imu660ra_gyro_z);
//    printf("Type: %d, Curvature: %f,%d,%d,%d,%d,%f,%f\n", current_road_type,curve.curvature,speed1,speed2,Increase1,Increase2,steer,divertion);
//    printf("Round State: %d\n", round_state);
//        printf("Flags: Normal=%d, L_Start=%d, L_Trans=%d, L_In=%d, L_Exit=%d, L_Out=%d, L_End=%d\n",
//               round_flags.normal, round_flags.left_start, round_flags.left_trans,
//               round_flags.left_in, round_flags.left_exit, round_flags.left_out, round_flags.left_end);
//        printf("       R_Start=%d, R_Trans=%d, R_In=%d, R_Exit=%d, R_Out=%d, R_End=%d\n",
//               round_flags.right_start, round_flags.right_trans, round_flags.right_in,
//               round_flags.right_exit, round_flags.right_out, round_flags.right_end);
}


/*
 * 解析出data_buffer中的数据
 * 返回解析得到的数据
 */
float BLE_get_data(void)
{
    uint8_t data_Start_Num = 0; // 记录数据位开始的地方
    uint8_t data_End_Num = 0; // 记录数据位结束的地方
    uint8_t data_Num = 0; // 记录数据位数
    uint8_t minus_Flag = 0; // 判断是不是负数
    float data_return = 0; // 解析得到的数据

    data_len_01 = (uint8)wireless_uart_read_buffer(data_buffer_01, 200);


    for(uint8_t i=0;i<200;i++) // 查找等号和感叹号的位置
    {
        if(data_buffer_01[i] == '=') data_Start_Num = i + 1; // +1是直接定位到数据起始位
        if(data_buffer_01[i] == '!')
        {
            data_End_Num = i - 1;
            break;
        }
    }
    if(data_buffer_01[data_Start_Num] == '-') // 如果是负数
    {
        data_Start_Num += 1; // 后移一位到数据位
        minus_Flag = 1; // 负数flag
    }
    data_Num = data_End_Num - data_Start_Num + 1;
    if(data_Num == 4) // 数据共4位
    {
        data_return = (data_buffer_01[data_Start_Num]-48)  + (data_buffer_01[data_Start_Num+2]-48)*0.1f +
                (data_buffer_01[data_Start_Num+3]-48)*0.01f;
    }
    else if(data_Num == 5) // 数据共5位
    {
        data_return = (data_buffer_01[data_Start_Num]-48)*10 + (data_buffer_01[data_Start_Num+1]-48) + (data_buffer_01[data_Start_Num+3]-48)*0.1f +
                (data_buffer_01[data_Start_Num+4]-48)*0.01f;
    }
    else if(data_Num == 6) // 数据共6位
    {
        data_return = (data_buffer_01[data_Start_Num]-48)*100 + (data_buffer_01[data_Start_Num+1]-48)*10 + (data_buffer_01[data_Start_Num+2]-48) +
                (data_buffer_01[data_Start_Num+4]-48)*0.1f + (data_buffer_01[data_Start_Num+5]-48)*0.01f;
    }
    if(minus_Flag == 1)  data_return = -data_return;

//    printf("data=%.2f\r\n",data_return);
    return data_return;
}

/*
 * 根据串口信息进行PID调参
 */
void BLE_PID_Adjust(void)
{
    float data_Get = BLE_get_data(); // 存放接收到的数据
    if(data_buffer_01[0]=='S' && data_buffer_01[1]=='1')
        test_Speed = data_Get;
//    printf("data=%.2f\r\n",data_Get);
//    if(Motor_n == 1)//左边电机
//    {
//        if(data_buffer[0]=='P' && data_buffer[1]=='1')
//            P_L = data_Get;
//        else if(data_buffer[0]=='I' && data_buffer[1]=='1')
//            I_L = data_Get;
//    }
//    else if(Motor_n == 2) // 右边电机
//    {
//        if(data_buffer[0]=='P' && data_buffer[1]=='2')
//            P_R = data_Get;
//        else if(data_buffer[0]=='I' && data_buffer[1]=='2')
//            I_R = data_Get;

//        else if(data_buffer[0]=='C' && data_buffer[1]=='1')
//            car_go = data_Get;
//    }
}
