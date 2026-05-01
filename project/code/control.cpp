#include "zf_common_headfile.hpp"

zf_device_imu imu_dev;
float target_speed = 0;
float current_lspeed = 0;
float current_rspeed = 0;
float left_speed = 0;
float right_speed = 0;

// 限幅函数
int constrain(int val, int min_val, int max_val)
{
    if(val > max_val) return max_val;
    if(val < min_val) return min_val;
    return val;
}

// 获取有效中线（底部多行加权平均，抗干扰）
float get_center_error(void)
{
    int sum = 0;
    int cnt = 0;
    // 取图像底部30行（y从90到119，共30行），越靠近小车越重要
    for(int y = IMAGE_H - 30; y < IMAGE_H; y++){
        // 过滤无效值（0和超宽值）
        if(center_line[y] > 5 && center_line[y] < IMAGE_W - 5){
            sum += center_line[y];
            cnt++;
        }
    }
    // 无有效线时返回中心（不跑偏）
    if(cnt == 0) return 0.0f;
    
    int avg_mid = sum / cnt;
    // 误差 = 中心 - 当前中线（正=偏右，负=偏左）
    return (float)(CENTER_X - avg_mid);
}
float steer1,target_angle1;

void line_follow_pid_control(void)
{
    // // ===================== 1. 外环：图像偏差 → 目标角度 =====================
    // float line_error = get_center_error();
    // float target_angle = PID_Positional_Calculate(&TracePID, line_error, 0.0f);
    // target_angle1=target_angle;
    // // ===================== 2. 内环：陀螺仪角速度 → 转向差速 =====================
    // float current_angle = imu_dev.get_gyro_z()*0.001f;  // 替换为你的陀螺仪航向角函数
    // float steer = PID_Positional_Calculate(&AnglePID, current_angle, target_angle);
    // steer1=steer;
    // ===================== 3. 速度环：编码器 → 稳定前进速度 =====================
    //target_speed = 0.0f;  // 目标前进速度 m/s
    current_lspeed = get_left_speed_mps();
    current_rspeed = get_right_speed_mps();
    left_speed = PID_Incremental_Calculate(&Speed_lPID, current_lspeed, target_speed);
    right_speed = PID_Incremental_Calculate(&Speed_rPID, current_rspeed, target_speed);

    // // ===================== 4. 电机速度合成 =====================
    // int base_speed = BASE_SPEED ;  // 基础速度 + 速度环修正
    // int left_speed  = base_speed + (int)steer;
    // int right_speed = base_speed - (int)steer;

    // // ===================== 5. 限幅（防止超范围） =====================
    // left_speed  = constrain(left_speed,  -20, 20);
    // right_speed = constrain(right_speed, -20, 20);

    // ===================== 6. 输出到电机 =====================
    set_left_speed((int)left_speed);
    set_right_speed((int)right_speed);
}

float get_steer(void)
{
    return steer1;
}

float get_target_angle(void)
{
    return target_angle1;
}
