#ifndef CONTROL_HPP
#define CONTROL_HPP

void line_follow_pid_control(void);
#define CENTER_X (IMAGE_W / 2)
#define BASE_SPEED 15
extern zf_device_imu imu_dev;
float get_target_angle(void);
float get_steer(void);
extern float target_speed;
extern float left_speed;
extern float right_speed;
extern float current_lspeed;
extern float current_rspeed;

#endif 