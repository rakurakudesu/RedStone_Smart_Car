#include "zf_common_headfile.hpp"

PID_TypeDef TracePID;
PID_TypeDef AnglePID;
PID_TypeDef Speed_lPID;
PID_TypeDef Speed_rPID;
PID_TypeDef Delta_SpPID;

// PID初始化
void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd, float output_limit, float integral_limit, float sep_threshold)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->output_limit = output_limit;
    pid->integral_limit = integral_limit;
    pid->sep_threshold = sep_threshold;

    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

//==================== 增量式 PID（内环：电机/角度控制）====================
float PID_Incremental_Calculate(PID_TypeDef *pid, float feedback, float setpoint)
{
    if(pid == NULL) return 0.0f;

    // 计算当前偏差
    pid->error = setpoint - feedback;

    // 增量式PID公式
    float delta_u = pid->Kp * (pid->error - pid->last_error)
                   + pid->Ki * pid->error
                   + pid->Kd * (pid->error - 2 * pid->last_error + pid->prev_error);

    // 输出累加 + 限幅
    pid->output += delta_u;
    if(pid->output >  pid->output_limit)  pid->output =  pid->output_limit;
    if(pid->output < -pid->output_limit)  pid->output = -pid->output_limit;

    // 更新偏差历史
    pid->prev_error = pid->last_error;
    pid->last_error = pid->error;

    return pid->output;
}

//==================== 位置式 PID ====================
float PID_Positional_Calculate(PID_TypeDef *pid, float feedback, float setpoint)
{
    if(pid == NULL) return 0.0f;

    // 计算当前偏差
    pid->error = setpoint - feedback;

    // ===== 改进的积分分离 + 抗饱和 =====
    // 1. 误差幅度分离：|error| 小 → 全积分；中等 → 渐变；大 → 冻结
    // 2. 输出饱和保护：饱和同向 → 停积分；反向 → 允许退饱和
    if (pid->Ki > 0.001f)
    {
        float error_abs = fabs(pid->error);
        float sep_coef;

        if (error_abs < pid->sep_threshold) {
            sep_coef = 1.0f;  // 误差小，全效积分消静差
        } else if (error_abs < pid->sep_threshold * 2.0f) {
            sep_coef = 2.0f - error_abs / pid->sep_threshold;  // 过渡区线性衰减
        } else {
            sep_coef = 0.0f;  // 误差大，冻结积分，P/D 主导暂态
        }

        // 输出饱和保护：饱和 + 同向误差 → 停止积分（防 windup）
        bool sat_positive = (pid->output >=  pid->output_limit && pid->error > 0);
        bool sat_negative = (pid->output <= -pid->output_limit && pid->error < 0);

        if (!sat_positive && !sat_negative && sep_coef > 0.0f) {
            pid->integral += pid->Ki * pid->error * sep_coef;
            if (pid->integral >  pid->integral_limit) pid->integral =  pid->integral_limit;
            if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
        }
    }
    else
    {
        // Ki=0，跳过积分
        pid->integral = 0.0f;
    }

    // 位置式 PID 公式
    float output =  pid->Kp * pid->error
                  + pid->integral
                  + pid->Kd * (pid->error - pid->last_error);

    // 输出限幅
    pid->output = output;
    if(pid->output >  pid->output_limit)  pid->output =  pid->output_limit;
    if(pid->output < -pid->output_limit)  pid->output = -pid->output_limit;

    // 更新上一次偏差
    pid->last_error = pid->error;

    return pid->output;
}

// PID重置
void PID_Reset(PID_TypeDef *pid)
{
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}
