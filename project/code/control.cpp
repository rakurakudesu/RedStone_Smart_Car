#include "zf_common_headfile.hpp"

// 串级PID + 速度环
PD_TypeDef  Outer_PD;    // 外环：图像中线误差 → 目标角度
PD_TypeDef  Inner_PD;    // 内环：陀螺仪角度 → 转向差速
PD_TypeDef  Speed_PD;    // 速度环：编码器 → 稳定速度

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

void line_follow_pid_control(void)
{
    // ===================== 1. 外环：图像偏差 → 目标角度 =====================
    float line_error = get_center_error();
    float target_angle = PD_Outer_Calculate(&Outer_PD, line_error, 0.0f);

    // ===================== 2. 内环：陀螺仪角度 → 转向差速 =====================
    float current_angle = eulerAngle.yaw;  // 替换为你的陀螺仪航向角函数
    float steer = PD_Inner_Calculate(&Inner_PD, current_angle, target_angle);

    // ===================== 3. 速度环：编码器 → 稳定前进速度 =====================
    float target_speed = 0.35f;  // 目标前进速度 m/s
    float current_speed = (get_left_speed_mps() + get_right_speed_mps()) / 2.0f;
 // float speed_out = PD_Outer_Calculate(&Speed_PD, current_speed, target_speed);
    float speed_out =0.0;

    // ===================== 4. 电机速度合成 =====================
    int base_speed = BASE_SPEED + (int)speed_out;  // 基础速度 + 速度环修正
    int left_speed  = base_speed + (int)target_angle;
    int right_speed = base_speed - (int)target_angle;

    // ===================== 5. 限幅（防止超范围） =====================
    left_speed  = constrain(left_speed,  -10, 10);
    right_speed = constrain(right_speed, -10, 10);

    // ===================== 6. 输出到电机 =====================
    set_left_speed(left_speed);
    set_right_speed(right_speed);
}
