#include "zf_common_headfile.hpp"

zf_device_imu imu_dev;

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

// 假设外部初始化时，PIT 定时器配置为 5ms 触发一次

void line_follow_pid_control(void)
{
    // 定义静态计数器（每次进入函数不会清零）
    static uint8_t speed_cnt  = 0;
    static uint8_t vision_cnt = 0;
    // 定义静态跨环传递变量（保持上一次的值，供内环使用）
    static float target_angle = 0.0f; 
    static float speed_out    = 0.0f;

    vision_cnt++;
    speed_cnt++;

    // ===================== 1. 外环：寻迹环 (20ms 执行一次) =====================
    // 5ms * 4 = 20ms
    if (vision_cnt >= 4) 
    {
        vision_cnt = 0; // 计数器清零
        image_process();
        float line_error = get_center_error();
        // 计算目标角度，存入 static 变量
        target_angle = PID_Positional_Calculate(&TracePID, line_error, 0.0f);
        target_angle1 = target_angle; // 给外部调试用
    }

    // ===================== 2. 速度环：编码器 (50ms 执行一次) =====================
    // 5ms * 10 = 50ms
    if (speed_cnt >= 10)
    {
        speed_cnt = 0; // 计数器清零
        
        float target_speed = 0.35f;  // 目标前进速度 m/s
        float current_speed = (get_left_speed_mps() + get_right_speed_mps()) / 2.0f;
        
        // 计算速度环输出，存入 static 变量
        speed_out = PID_Incremental_Calculate(&SpeedPID, current_speed, target_speed);
    }

    // ===================== 3. 内环：陀螺仪 (5ms 执行一次，每次必进) =====================
    float current_angle = imu_dev.get_gyro_z();
    float steer = PID_Positional_Calculate(&AnglePID, current_angle, target_angle);
    steer1 = steer;

    // ===================== 4. 电机速度合成 (5ms 执行一次) =====================
    int base_speed = BASE_SPEED + (int)speed_out;  
    int left_speed  = base_speed + (int)steer;
    int right_speed = base_speed - (int)steer;

    // ===================== 5. 限幅与输出 =====================
    left_speed  = constrain(left_speed,  -20, 20);
    right_speed = constrain(right_speed, -20, 20);

    set_left_speed(left_speed);
    set_right_speed(right_speed);
}

float get_steer(void)
{
    return steer1;
}

float get_target_angle(void)
{
    return target_angle1;
}
