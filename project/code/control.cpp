#include "zf_common_headfile.hpp"
#include "zf_driver_gpio.hpp"
//#define LEFT_DEADZONE 7 //PWM左轮死区补偿，融入前馈
//#define RIGHT_DEADZONE 8 //PWM右轮死区补偿，融入前馈
#define ZERO_THRESHOLD 2 //死区判断阈值
//速度前馈需要上赛道重调
#define kf_l 21.46 //左PWM与速度的前馈系数1   只做了正向！反向会有一定误差
#define bf_l 409.89 //左PWM与速度的前馈系数2  PWM_f = target_speed * kf + bf
#define kf_r 19.733 //右PWM与速度的前馈系数1   只做了正向！反向会有一定误差
#define bf_r 497.42 //右PWM与速度的前馈系数2  PWM_f = target_speed * kf + bf
//角度前馈
#define kf_turn 7.25//轮距/2  kf_turn * omega = delt_target_speed

zf_device_imu imu_dev;
float target_lspeed = 0;
float target_rspeed = 0;
float current_lspeed = 0;
float current_rspeed = 0;
float left_PWM_PI = 0;
float right_PWM_PI = 0;
float left_PWM = 0;
float right_PWM = 0;
float target_omega = 0;
float current_omega = 0;
float steer = 0;
uint8 t_n = 0;//中断计数

const float encoder_filter = 0.3;//编码器滤波系数(越小越强) ps.实测建议在0.25~0.35之间
const float filter_off = 4;//速度需要大幅变动时，停止滤波的速度差值

// 限幅函数
int constrain(int val, int min_val, int max_val)
{
    if(val > max_val) return max_val;
    if(val < min_val) return min_val;
    return val;
}

float Feed_Forward_l(float target_speed)
{
    if (target_speed > ZERO_THRESHOLD)//死区判定
        return target_speed * kf_l + bf_l;//输出前馈
    else if (target_speed < -ZERO_THRESHOLD)
        return  -(-target_speed * kf_l + bf_l);
    else
        return 0;
}

float Feed_Forward_r(float target_speed)
{
    if (target_speed > ZERO_THRESHOLD)//死区判定
        return target_speed * kf_r + bf_r;//输出前馈
    else if (target_speed < -ZERO_THRESHOLD)
        return  -(-target_speed * kf_r + bf_r);
    else
        return 0;
}

void line_follow_pid_control(void)
{
    if (t_n == 10)
        t_n = 0;//重置计数
    t_n++;
    // ===================== 1. 外环：图像偏差 → 目标角速度 =====================
    //center_error在图像处理中给出
    if (t_n == 10)//100ms跑一次
        target_omega = PID_Positional_Calculate(&TracePID, center_error, 0.0f);//根据偏差得出角速度
    // ===================== 2. 内环：陀螺仪角速度 → 转向差速 =====================
    current_omega = imu_dev.get_gyro_z();  // 替换为你的陀螺仪航向角函数
    steer = PID_Positional_Calculate(&AnglePID, current_omega, target_omega);
    // ===================== 3. 速度环：编码器 → 稳定前进速度 =====================
    //给出目标速度，基础 + 角度环输出 + 角度前馈
    target_lspeed = BASE_SPEED + steer + kf_turn * target_omega;
    target_rspeed = BASE_SPEED - steer - kf_turn * target_omega;
    //编码器读取，并滤波
    float raw_lspeed = get_left_speed_mps();//左
    if (abs(raw_lspeed - current_lspeed) > filter_off)//大于阈值不滤波
        current_lspeed = raw_lspeed;
    else//小于阈值滤波
       current_lspeed = (encoder_filter * raw_lspeed) + ((1.0 - encoder_filter) * current_lspeed);
    float raw_rspeed = get_right_speed_mps();//右
    if (abs(raw_rspeed - current_rspeed) > filter_off)
        current_rspeed = raw_rspeed;
    else
        current_rspeed = (encoder_filter * raw_rspeed) + ((1.0 - encoder_filter) * current_rspeed);
    //PID调用，输出PWM = 前馈 + PI输出
    left_PWM_PI = PID_Incremental_Calculate(&Speed_lPID, current_lspeed, target_lspeed);//左
    left_PWM = Feed_Forward_l(target_lspeed) + left_PWM_PI;
    right_PWM_PI = PID_Incremental_Calculate(&Speed_rPID, current_rspeed, target_rspeed);//右
    right_PWM = Feed_Forward_r(target_rspeed) + right_PWM_PI;

    // ===================== 5. 限幅（防止超范围） =====================
     left_PWM  = constrain(left_PWM,  -2000, 2000);
     right_PWM = constrain(right_PWM, -2000, 2000);

    // ===================== 6. 输出到电机 =====================
    set_left_speed((int)left_PWM);
    set_right_speed((int)right_PWM);
}

