#ifndef _IMAGE_ELEMENTS_HPP
#define _IMAGE_ELEMENTS_HPP

// 圆环识别状态（0=未识别, 1-7=各阶段）
extern uint8 left_ring;
extern uint8 right_ring;
void find_flagpoint(void);
void cross_fill(void);
void ring_recognize(void);
void brick_recognize(void);     // 红色砖块识别+边线修正
void element_state_machine(void);  // 元素优先级状态机（砖块>圆环>十字）

#endif