#ifndef CONTROL_H
#define CONTROL_H

extern PD_TypeDef  Outer_PD;    // 外环：图像中线误差 → 目标角度
extern PD_TypeDef  Inner_PD;    // 内环：陀螺仪角度 → 转向差速
extern PD_TypeDef  Speed_PD;    // 速度环：编码器 → 稳定速度

void line_follow_pid_control(void);
#define CENTER_X (IMAGE_W / 2)
#define BASE_SPEED 20

float get_target_angle(void);
float get_steer(void);

#endif 