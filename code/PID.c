/*
 * PID.c
 *
 *  Created on: 2025年1月25日
 *      Author: zhuji
 */
#include "PID.h"

int basespeed=1000;
int setspeed1=1000;
int setspeed2=1000;
int speed1;//左电机
int speed2;
int Increase1=0;
int Increase2=0;
float divertion;
float erspeed=0;

float gyro_filtered = 0; // 滤波后的陀螺仪值
float ratio=0;
double steer =0 ;
double steer1 =0 ;

int test_Speed = 1400;

PID_Datatypedef sptr1,sptr2;
PID_imu_Datatypedef imu;
PID_Angeltypedef angel;
//float P_L=0.1;
//float I_L=0;
//float P_R=0.1;
//float I_R=0;

void PID_Init(PID_Datatypedef*sptr)
{
    sptr->P=0;
    sptr->I=0;
    sptr->D=0;
    sptr->LastError=0;
    sptr->PrevError=0;
}

//**************************************************************************************************************
float A_KP_L=6.5;
float A_KI_L=0;
float A_KD_L=0.01;
float A_KF_L=0;

float A_KP_R=7;
float A_KI_R=0;
float A_KD_R=0.01;
float A_KF_R=0;

int ADRC_L(int set_speed ,int speed)
{
    int e; // 误差
    int e_prev = 0; // 上一次的误差
    int e_int=0; // 误差的积分
    int de; // 误差的变化率
    int u_c; // 控制律计算的控制量
    int u_est=0; // 扰动估计量
    int u; // 最终的控制量
    float dt = 0.01; // 控制周期 10ms

    // 计算误差和误差的变化率
    e = set_speed - speed;
    de = (e - e_prev) / dt;

    // 更新误差的积分
    e_int += e * dt;

//    //停车处理
//        if(Zebra_Stripes_Flag!=0)
//        {
//            A_KP_L=A_KP_L+5;
//        }

    // 计算控制律的各部分
    u_c = A_KP_L * e + A_KI_L * e_int + A_KD_L * de;

    // 更新扰动估计量（使用低通滤波器）
    u_est += A_KF_L * (e - u_est) * dt;

    // 计算最终的控制量
    u = u_c - u_est;; // 实际应用中可能需要一个饱和函数来限制控制量的范围

    // 更新变量为下一次控制周期
    e_prev = e;

    if(u>=SPEED_MAX)//限幅处理
        u=SPEED_MAX;
    else if(u<=SPEED_MIN)
        u=SPEED_MIN;
    return (int) u;
}

int ADRC_R(int set_speed ,int speed)
{
    int e; // 误差
    int e_prev = 0; // 上一次的误差
    int e_int=0; // 误差的积分
    int de; // 误差的变化率
    int u_c; // 控制律计算的控制量
    int u_est=0; // 扰动估计量
    int u; // 最终的控制量
    float dt = 0.01; // 控制周期 10ms

    // 计算误差和误差的变化率
    e = set_speed - speed;
    de = (e - e_prev) / dt;

    // 更新误差的积分
    e_int += e * dt;

//    //停车处理
//        if(Zebra_Stripes_Flag!=0)
//        {
//            A_KP_R=A_KP_R+5;
//        }

    // 计算控制律的各部分
    u_c = A_KP_R * e + A_KI_R * e_int + A_KD_R * de;

    // 更新扰动估计量（使用低通滤波器）
    u_est += A_KF_R * (e - u_est) * dt;

    // 计算最终的控制量
    u = u_c - u_est;; // 实际应用中可能需要一个饱和函数来限制控制量的范围

    // 更新变量为下一次控制周期
    e_prev = e;

    if(u>=SPEED_MAX)//限幅处理
        u=SPEED_MAX;
    else if(u<=SPEED_MIN)
        u=SPEED_MIN;
    return (int) u;
}
void imu_PID_Init(PID_imu_Datatypedef*imu)
{
    imu->KP_1=0;
    imu->KD_1=0;
    imu->KP_2=0;
    imu->GKD=0;
    imu->lasterror=0;

    imu->integrator = 0;   // 积分项初始化为0
}
int MotorPID_Output(PID_Datatypedef*sptr,float NowSpeed,int ExpectSpeed)
{
    int Increase;
    int iError;
    iError=ExpectSpeed-NowSpeed;
    Increase=(int)(sptr->P*(iError-sptr->LastError)+sptr->I*iError+sptr->D*(iError-2*sptr->LastError+sptr->PrevError));
    sptr->PrevError=sptr->LastError;
    sptr->LastError=iError;
    return Increase;
}
float imuPID_Output(float erspeed,PID_imu_Datatypedef*imu)
{
    float imu_out;
//    imu_out=erspeed*imu->KP_1+(erspeed-imu->lasterror)*imu->KD_1-imu660ra_gyro_x*imu->GKD;
//    imu->lasterror=erspeed;
    imu_out = (erspeed * imu->KP_1) + (erspeed * fabsf(erspeed) * imu->KP_2) + (erspeed - imu->lasterror) * imu->KD_1 + imu660ra_gyro_x * imu->GKD;
    imu->lasterror = erspeed; // 更新上一次误差
    if (imu_out > 2300) imu_out =2300;
    else if (imu_out < -2300) imu_out = -2300;
    return imu_out;

}
//int ang_pid(float b,int c)
//{
//    int t;
//    int temp_speed;
//
//    temp_speed=(speed1+speed2)/2;
//
//    t=angel.kP*(temp_speed-b)+angel.kD*(temp_speed-angel.LastError1);
//    angel.LastError1=temp_speed-b;
//
//    return t;
//
//}

int pid_Moter(int set_speed,int real_speed)
{
    float kp,ki,kd;     //增量式PID参数  
    float out_increment;//增量式PID输出增量  
    static int out;          //输出量  
    static int16 ek,ek1,ek2;//前后三次误差  
    ek2 = ek1;//保存上上次误差            
    ek1 = ek; //保存上次误差            
    ek = set_speed-real_speed;//计算当前误差                       
    //设置PID系数            
    kp = 6;
    ki = 0.01;
    kd = 0;//进行增量式PID运算            
    out_increment = (int16)(kp*(ek-ek1) + ki*ek + kd*(ek-2*ek2+ek2)); //计算增量          
    out += (int)out_increment;  //输出增量          
    return out;
}

void Velocity_Control(int set_speed,int speed_left_real,int speed_right_real)//赛道类型判别，来选定速度
{
    int Pid_Speed = set_speed;
    imu.KP_1 = 8.0;   // 降低比例增益，平稳转向
    imu.KD_1 = 0.8;   // 禁用微分项（可根据需要调整）
    imu.GKD = -0.65;   // 陀螺仪增益，适度校正


    divertion=imuPID_Output(erspeed,&imu);
//    divertion = center_line_error*100;
//    Pid_Speed= pid_Moter(set_speed ,speed_left_real+speed_right_real);//pid控制电机转速
    Increase1=Pid_Speed+divertion;
    Increase2=Pid_Speed-divertion;
//    Increase1= ADRC_L(Increase1 ,speed_left_real );//pid控制电机转速
//    Increase2= ADRC_R(Increase2,speed_right_real);//pid控制电机转速



    if(Car_GO_Flag==1)
    {
        Motor_Left (Increase1);
        Motor_Right(Increase2);
//        Motor_Left (Pid_Speed);
//        Motor_Right(Pid_Speed);
    }
    else
    {
        Motor_Left(0);
        Motor_Right(0);
    }
}
void Control_car(void)
{
    // 获取视觉系统的中心线偏差（正或负表示偏离方向）
    float steer = center_line_error;
    steer1 = center_line_error;
    // 计算曲率（使用偏差的绝对值，类似原始代码中的 steer1）
    float curvature = fabsf(steer);
    // 定义速度范围（根据小车性能调整）

    // 根据曲率调整比例（ratio），曲率越大，ratio 越大，速度越低
    float ratio;
    if (zhidaoflag) {  // 直道模式
        if (curvature < 0.1) {
            ratio = 15.0 * curvature / MAX_DEVIATION;  // 小偏差，低曲率
        } else if (curvature < 0.5) {
            ratio = 3.0 * curvature / MAX_DEVIATION;   // 中等偏差
        } else if (curvature < 1.0) {
            ratio = 4.0 * curvature / MAX_DEVIATION;   // 较大偏差
        } else {
            ratio = 1.5 * curvature / MAX_DEVIATION;   // 大偏差
        }
    } else {  // 非直道模式（弯道）
        if (curvature < 0.1) {
            ratio = 35.0 * curvature / MAX_DEVIATION;  // 小偏差，高敏感
        } else if (curvature < 0.5) {
            ratio = 5.0 * curvature / MAX_DEVIATION;   // 中等偏差
        } else if (curvature < 1.0) {
            ratio = 8.0 * curvature / MAX_DEVIATION;   // 较大偏差
        } else {
            ratio = 2.5 * curvature / MAX_DEVIATION;   // 大偏差
        }
    }

    // 限制 ratio 范围（0.0 到 0.8），避免速度过低或过高
    if (ratio > 0.8) {
        ratio = 0.8;
    } else if (ratio < 0.0) {
        ratio = 0.0;
    }

    // 计算目标速度：曲率越大，速度越接近 MIN_SPEED
    int target_speed = MAX_SPEED - (int)((MAX_SPEED - MIN_SPEED) * ratio);
    //target_speed = 1300;
    // 设置左右电机的目标速度（速度环不引入差速）
    setspeed1 = target_speed;  // 左电机
    setspeed2 = target_speed;  // 右电机

    // 确保目标速度在 MIN_SPEED 和 MAX_SPEED 之间
    if (setspeed1 < MIN_SPEED) setspeed1 = MIN_SPEED;
    if (setspeed1 > MAX_SPEED) setspeed1 = MAX_SPEED;
    if (setspeed2 < MIN_SPEED) setspeed2 = MIN_SPEED;
    if (setspeed2 > MAX_SPEED) setspeed2 = MAX_SPEED;

    // 配置 IMU PID 参数（根据速度调整，防止过激校正）
    if (target_speed >= (MAX_SPEED + MIN_SPEED) / 2) {  // 高速模式
        imu.KP_1 = 5.0;   // 降低比例增益，平稳转向
        imu.KD_1 = 0.0;   // 禁用微分项（可根据需要调整）
        imu.GKD = -1.0;   // 陀螺仪增益，适度校正
    } else {  // 低速模式
        imu.KP_1 = 5.0;   // 更低比例增益，适合弯道
        imu.KD_1 = 0.0;   // 禁用微分项
        imu.GKD = -1.0;   // 减小陀螺仪影响
    }

    // 计算 IMU PID 的误差速度（降低增益，防止过大校正）
    erspeed = steer * 40.0;

    // 限制误差速度，防止 IMU 校正过于剧烈
    const float MAX_ERSPEED = 2000.0;
    if (erspeed > MAX_ERSPEED) {
        erspeed = MAX_ERSPEED;
    } else if (erspeed < -MAX_ERSPEED) {
        erspeed = -MAX_ERSPEED;
    }
}

//void Control_car(void)
//{
//    steer = center_line_error;
//    steer1 = fabsf(center_line_error);
//    if(zhidaoflag)
//    {  if (0.1>steer1)
//          steer1=15*steer1;
//       else if (0.5>steer1)
//       steer1=3*steer1;
//       else if (1>steer1)
//       steer1=4*steer1;
//       else
//           steer1=1.5*steer1;}
//    else if(!zhidaoflag)
//    { if (0.1>steer1)
//           steer1=35*steer1;
//        else if (0.5>steer1)
//        steer1=5*steer1;
//        else if (1>steer1)
//        steer1=8*steer1;
//        else
//            steer1=2.5*steer1;}
//    if(8<steer1)
//        steer1=8;
//       ratio = steer1 / MAX_DEVIATION;  // 计算比例（0.0~1.0）
//          setspeed1=MAX_SPEED - (MAX_SPEED - MIN_SPEED) * ratio;
//          setspeed2=MAX_SPEED - (MAX_SPEED - MIN_SPEED) * ratio;
//
//     if(setspeed1>=(MAX_SPEED+MIN_SPEED)*0.5)
//     {  imu.KP_1 = 18;
//         imu.KD_1 = 0;
//     //     imu.KP_2 = 0.1;
//         imu.GKD = -2.7;}
//     else if(setspeed1<(MAX_SPEED+MIN_SPEED)*0.5)
//     { imu.KP_1 = 15;
//             imu.KD_1 = 0;
//         //     imu.KP_2 = 0.1;
//             imu.GKD = -2.5;}
//             if((steer>=-10&steer<=-3)||(steer<=10&&steer>=3))
//                       erspeed=70*steer;//7.3
//             else if ((steer > 1 && steer < 3) || (steer > -3 && steer < -1))
//                        erspeed=60*steer;
//          //              else
//             else if(steer<1||steer>-1)
//                        erspeed=40*steer;
//
//                        else
//                            erspeed=65*steer;
//}
void PID_output(void)
{


//            增量式PID控制小车直行
//    Increase1=MotorPID_Output(&sptr1,speed1,setspeed1);
//    Increase2=MotorPID_Output(&sptr2,speed2,setspeed2);
//        Increase1=MotorPID_Output(&sptr1,speed1,1300);
//        Increase2=MotorPID_Output(&sptr2,speed2,1300);
//      Increase1 = setspeed1;
//              Increase2=setspeed2;
            //方向环直接扭转小车运行方向
//    Increase1=Increase1-divertion;
//    Increase2=Increase2+divertion;
//    Increase1=-divertion;
//    Increase2=+divertion;
    divertion=imuPID_Output(erspeed,&imu);
//    divertion = center_line_error*100;
    Increase1=Increase1+divertion;
    Increase2=Increase2-divertion;
    // 限制输出，防止反转或过大
//    if (Increase1 < 0) Increase1 = 0.02*Increase2;
//    if (Increase2 < 0) Increase2 = 0.02*Increase1;
//    if (Increase1 > 2500) Increase1 = 3000;
//    if (Increase2 > 2500) Increase2 = 3000;
    if(Car_GO_Flag == 1)
    {
        Motor_Left(Increase1);
        Motor_Right(Increase2);
        Increase1 = 0;
        Increase2 = 0;
//        Motor_Left(MotorPID_Output(&sptr1,speed1,0));
//        Motor_Right(MotorPID_Output(&sptr2,speed2,0));
    }
    else
    {
        Motor_Left(0);
        Motor_Right(0);
    }

//    Motor_Left(1000);
//    Motor_Right(1000);

}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     PID控制
  @param     int set_speed ,int speed,期望值，实际值
  @return    电机占空比
  Sample     pwm_R= PID_R(set_speed_right,right_wheel);//pid控制电机转速
             pwm_L= PID_L(set_speed_left,left_wheel );//pid控制电机转速
  @note      调参呗
-------------------------------------------------------------------------------------------------------------------*/
//int PID_L(int set_speed ,float speed)//pid控制电机转速
//{
//    volatile static int out;
//    volatile static int out_increment;
//    volatile static int ek,ek1;
//    volatile static int speed_bb;
//
//    ek1 = ek;
//    ek = set_speed - speed;
//
//    if(ek>80) speed_bb=SPEED_MAX;
//    else if(ek<-80) speed_bb=SPEED_MIN;
//    else speed_bb=0;
//
//    out_increment= (int)(P_L*(ek-ek1) + I_L*ek + speed_bb);
//    out+= out_increment;
//
//    if(out>=SPEED_MAX)//限幅处理
//        out=SPEED_MAX;
//    else if(out<=SPEED_MIN)
//        out=SPEED_MIN;
//    return (int) out;
//}
//
///*-------------------------------------------------------------------------------------------------------------------
//  @brief     PID控制
//  @param     int set_speed ,int speed,期望值，实际值
//  @return    电机占空比
//  Sample     pwm_R= PID_R(set_speed_right,right_wheel);//pid控制电机转速
//             pwm_L= PID_L(set_speed_left,left_wheel );//pid控制电机转速
//  @note      调参呗
//-------------------------------------------------------------------------------------------------------------------*/
//int PID_R(int set_speed ,float speed)//pid控制电机转速
//{
//    volatile static int  out;
//    volatile static int  out_increment;
//    volatile static int  ek,ek1;
//    volatile static int speed_bb;
//
//    ek1 = ek;
//    ek = set_speed - speed;
//
//    if(ek>80) speed_bb=SPEED_MAX;
//    else if(ek<-80) speed_bb=SPEED_MIN;
//    else speed_bb=0;
//
//    out_increment= (int)(P_R*(ek-ek1) + I_R*ek + speed_bb);
//    out+= out_increment;
//
//    if(out>=SPEED_MAX)//限幅处理
//        out=SPEED_MAX;
//    else if(out<=SPEED_MIN)
//        out=SPEED_MIN;
//    return (int) out;
//}



