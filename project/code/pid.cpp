#include "zf_common_headfile.hpp"

// PD初始化
void PD_Init(PD_TypeDef *pd, float Kp, float Kd, float output_limit) 
	{
    pd->Kp = Kp;
    pd->Kd = Kd;
    pd->output_limit = output_limit;
    pd->error = pd->last_error = pd->prev_error = pd->output = 0.0f;
}

// 增量式PD计算
float PD_Inner_Calculate(PD_TypeDef *pd, float feedback, float setpoint)
{
    if(pd == NULL) return 0.0f;
    
    // 1. 计算当前偏差
    pd->error = setpoint - feedback;
	
    // 2. 增量式PD核心公式
    float delta_u = pd->Kp * (pd->error - pd->last_error) 
                  + pd->Kd * (pd->error - 2*pd->last_error + pd->prev_error);
	
    // 4. 累计输出+总限幅
    pd->output += delta_u;
    pd->output = pd->output > pd->output_limit ? pd->output_limit : pd->output;
    pd->output = pd->output < -pd->output_limit ? -pd->output_limit : pd->output;
	
    // 5. 更新偏差历史
    pd->prev_error = pd->last_error;
    pd->last_error = pd->error;
    
    return pd->output;
}

//位置式计算
float PD_Outer_Calculate(PD_TypeDef *pd, float feedback, float setpoint)
{
    if(pd == NULL) return 0.0f;
    
    // 1. 计算当前偏差
    pd->error = setpoint - feedback;
	
    // 2. 增量式PD核心公式
    float delta_u = pd->Kp * pd->error 
                  + pd->Kd * (pd->error - pd->last_error);
	
    // 4. 累计输出+总限幅
    pd->output = delta_u;
    pd->output = pd->output > pd->output_limit ? pd->output_limit : pd->output;
    pd->output = pd->output < -pd->output_limit ? -pd->output_limit : pd->output;
	
    // 5. 更新偏差历史
    pd->last_error = pd->error;
    
    return pd->output;
}

    
// PD重置（折线转弯后清空累积）
void PD_Reset(PD_TypeDef *pd) 
{
    pd->error = pd->last_error = pd->prev_error = 0.0f;
	pd->output = 0.0f;
}
