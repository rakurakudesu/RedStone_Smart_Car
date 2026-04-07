#ifndef __PID_H
#define __PID_H

#define BASE_SPEED     28   // 基础速度(0~100)
#define PWM_MAX       100   // PWM最大值
#define LINE_THRESHOLD 120       // 折线判定阈值（传感器偏差超15=进入折线）
#define BIAS_DEADZONE 0         // 直道死区（偏差<5=直道）

// PD结构体（通用）
typedef struct 
{
    float Kp;          // 比例系数
    float Kd;          // 微分系数
    float error;       // 当前误差
    float last_error;  // 上一次误差
    float prev_error;  // 上上次误差
    float output_limit;// 输出限幅
    float output;      // PD输出
} PD_TypeDef;


// 串级PD实例：外环（寻迹偏差→目标角度）、内环（角度偏差→电机差速）
extern PD_TypeDef OuterPD;  // 外环PD（寻迹偏差）
extern PD_TypeDef InnerPD;  // 内环PD（角度偏差）
static float Last_error; 	//外环Last_error

void PD_Init(PD_TypeDef *pd, float Kp, float Kd, float output_limit);
float PD_Outer_Calculate(PD_TypeDef *pd, float feedback, float setpoint);//外环PD计算，位置式
float PD_Inner_Calculate(PD_TypeDef *pd, float feedback, float setpoint);//内环PD计算，增量式
void PD_Reset(PD_TypeDef *pd);

#endif