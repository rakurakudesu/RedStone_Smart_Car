#include "zf_common_headfile.hpp"

void draw_3x3(int x, int y, int color)
{
    ips200.draw_point(x-1, y-1, color);
    ips200.draw_point(x  , y-1, color);
    ips200.draw_point(x+1, y-1, color);
    ips200.draw_point(x-1, y  , color);
    ips200.draw_point(x  , y  , color);
    ips200.draw_point(x+1, y  , color);
    ips200.draw_point(x-1, y+1, color);
    ips200.draw_point(x  , y+1, color);
    ips200.draw_point(x+1, y+1, color);
} 

/** 
* @brief 最小二乘法
* @param uint8 begin				输入起点
* @param uint8 end					输入终点
* @param uint8 *border				输入需要计算斜率的边界首地址
*  @see CTest		Slope_Calculate(start, end, border);//斜率
* @return 返回说明
*     -<em>false</em> fail
*     -<em>true</em> succeed
*/
float Slope_Calculate(uint8 begin, uint8 end, uint8 *border)
{
	float xsum = 0, ysum = 0, xysum = 0, x2sum = 0;
	int16 i = 0;
	float result = 0;
	static float resultlast;

	for (i = begin; i < end; i++)
	{
		xsum += i;
		ysum += border[i];
		xysum += i * (border[i]);
		x2sum += i * i;

	}
	if ((end - begin)*x2sum - xsum * xsum) //判断除数是否为零
	{
		result = ((end - begin)*xysum - xsum * ysum) / ((end - begin)*x2sum - xsum * xsum);
		resultlast = result;
	}
	else
	{
		result = resultlast;
	}
	return result;
}

/** 
* @brief 计算斜率截距
* @param uint8 start				输入起点
* @param uint8 end					输入终点
* @param uint8 *border				输入需要计算斜率的边界
* @param float *slope_rate			输入斜率地址
* @param float *intercept			输入截距地址
*  @see CTest		calculate_s_i(start, end, r_border, &slope_l_rate, &intercept_l);
* @return 返回说明
*     -<em>false</em> fail
*     -<em>true</em> succeed
*/
void calculate_s_i(uint8 start, uint8 end, uint8 *border, float *slope_rate, float *intercept)
{
	uint16 i, num = 0;
	uint16 xsum = 0, ysum = 0;
	float y_average, x_average;

	num = 0;
	xsum = 0;
	ysum = 0;
	y_average = 0;
	x_average = 0;
	for (i = start; i < end; i++)
	{
		xsum += i;
		ysum += border[i];
		num++;
	}

	//计算各个平均数
	if (num)
	{
		x_average = (float)(xsum / num);
		y_average = (float)(ysum / num);

	}

	/*计算斜率*/
	*slope_rate = Slope_Calculate(start, end, border);//斜率
	*intercept = y_average - (*slope_rate)*x_average;//截距
}

/**
 * @brief 两点连线补线函数（基于最小二乘法拟合直线）
 * @param start   输入：起点坐标 [X,Y]
 * @param end     输入：终点坐标 [X,Y]
 * @param border  输入：需要补线的边界数组
 * @return 无
 * @note  复用 calculate_s_i 计算斜率截距，自动填充两点间所有边界点
 * @see   calculate_s_i
 */
void connect_points(uint16 start[2], uint16 end[2], uint8 *border)
{
    // 无效点
    if(start[1] == 0 || end[1] == 0) return;
    if(start[1] == end[1]) return;

    // 取出你的两个断裂点（已知有效点）
    float x1 = start[0], y1 = start[1];
    float x2 = end[0],   y2 = end[1];

    // 范围：从 上断点 到 下断点
    uint16 y_start = end[1];
    uint16 y_end   = start[1];

    // 两点式直线插值
    for(uint16 y = y_start; y <= y_end; y++)
    {
        // 直线公式：用两个断点直接算 x
        float x = x1 + (x2 - x1) * (y - y1) / (y2 - y1);

        // 限幅
        if(x < 0) x = 0;
        if(x >= IMAGE_W) x = IMAGE_W - 1;

        // 赋值边界，补线成功
        border[y] = (uint8)x;
    }
    //printf("补线\n");
}

//变量
uint16 r_v_down[2] ={0, 0};  //右圆环下v点
uint16 l_v_down[2] ={0, 0};  //左圆环下v点
uint16 r_v_up[2] ={0, 0};    //右圆环上v点
uint16 l_v_up[2] ={0, 0};    //左圆环上v点
uint16 r_p[2] ={0, 0};       //右圆环切点P
uint16 l_p[2] ={0, 0};       //左圆环切点P

//屏幕四角
uint16 screen_l_up[2]     = {1, 50};        // 屏幕左上角 
uint16 screen_r_up[2]     = {160, 50};      // 屏幕右上角 
uint16 screen_l_down[2]   = {1, 120};      // 屏幕左下角 
uint16 screen_r_down[2]   = {160, 120};    // 屏幕右下角 

uint16 r_break_down[2] = {0, 0};  // 右下断裂点
uint16 l_break_down[2] = {0, 0};  // 左下断裂点
uint16 r_break_up[2]   = {0, 0};  // 右上断裂点
uint16 l_break_up[2]   = {0, 0};  // 左上断裂点

bool flag_r_v_down = 0;     //找到右圆环下v点标志
bool flag_l_v_down = 0;     //找到左圆环下v点标志
bool flag_r_v_up = 0;       //找到右圆环上v点标志
bool flag_l_v_up = 0;       //找到左圆环上v点标志
bool flag_r_p = 0;          //找到右圆环切点P标志
bool flag_l_p = 0;          //找到左圆环切点P标志

bool flag_r_conti = 1;      // 右边界连续标志
bool flag_l_conti = 1;      // 左边界连续标志


uint8 start_check = IMAGE_H - 10;  // 从底部往上10行开始
uint8 end_check = 20; 
void find_flagpoint(void)
{
    uint16 i;
    // 初始化标志位
    flag_r_v_down = 0;     
    flag_l_v_down = 0;     
    flag_r_v_up = 0;       
    flag_l_v_up = 0;      
    flag_r_p = 0;          
    flag_l_p = 0;         

    flag_r_conti = 1;   // 右边界默认连续
    flag_l_conti = 1;   // 左边界默认连续

    r_v_down[0] = 0;
    r_v_down[1] = 0;
    l_v_down[0] = 0;
    l_v_down[1] = 0;
    r_v_up[0] = 0;
    r_v_up[1] = 0;
    l_v_up[0] = 0;
    l_v_up[1] = 0;
    r_p[0] = 0;
    r_p[1] = 0;
    l_p[0] = 0;
    l_p[1] = 0;
    r_break_down[0] = 0;
    r_break_down[1] = 0;
    l_break_down[0] = 0;
    l_break_down[1] = 0;
    r_break_up[0] = 0;
    r_break_up[1] = 0;
    l_break_up[0] = 0;
    l_break_up[1] = 0;

     // 边界连续性检查
    // 从下往上遍历（底部 → 顶部）
    for(i = start_check; i > end_check; i--) 
    {
         if(my_abs((int16)r_border[i] - (int16)r_border[i+1]) > 3 || r_border[end_check] == border_max)
        {
            flag_r_conti = 0;
            //printf("rbreak\n");
        }
         if(my_abs((int16)l_border[i] - (int16)l_border[i+1]) > 3 || l_border[end_check] == border_min)
        {
            flag_l_conti = 0;
            //printf("lbreak\n");
        }
        // ===================== 右边界 上下断裂检测 =====================
        if( i+1 < IMAGE_H && (((int16)r_border[i] - (int16)r_border[i+1]) < -3 
        &&my_abs((int16)r_border[i] - (int16)r_border[i-1]) < 3) )
        {
            // 上断裂点：只赋值第一次
            if(r_break_up[1] == 0)
            {
                r_break_up[0] = r_border[i]; 
                r_break_up[1] = i;
                //printf("右上：X=%d, Y=%d\r\n", r_break_down[0], r_break_down[1]);
                //draw_3x3(r_break_up[0], r_break_up[1],uesr_RED);
            }
        }

        if( i+1 < IMAGE_H && (((int16)r_border[i] - (int16)r_border[i-1]) < -3)
        &&(my_abs((int16)r_border[i] - (int16)r_border[i+1]) < 3))
        {
            // 下断裂点
            if(r_break_down[1] == 0)
            {
                r_break_down[0] = r_border[i]; 
                r_break_down[1] = i;
                //printf("右下：X=%d, Y=%d\r\n", r_break_up[0], r_break_up[1]);
                //draw_3x3(r_break_down[0], r_break_down[1],uesr_RED);
            }
        }

        // ===================== 左边界 上下断裂检测 =====================
        if( i+1 < IMAGE_H && (((int16)l_border[i] - (int16)l_border[i+1]) > 3)
        &&(my_abs((int16)l_border[i] - (int16)l_border[i-1]) < 3))
        {
            // 上断裂点：只赋值第一次
            if(l_break_up[1] == 0)
            {
                l_break_up[0] = l_border[i]; 
                l_break_up[1] = i;
                //printf("左下：X=%d, Y=%d\r\n", l_break_down[0], l_break_down[1]);
                //draw_3x3(l_break_up[0], l_break_up[1],uesr_RED);
            }
        }
        if( i+1 < IMAGE_H && (((int16)l_border[i] - (int16)l_border[i-1]) > 3)
        &&(my_abs((int16)l_border[i] - (int16)l_border[i+1]) < 3))
        {
            if(l_break_down[1] == 0)
            {
                l_break_down[0] = l_border[i]; 
                l_break_down[1] = i;
                //printf("左上：X=%d, Y=%d\r\n", l_break_up[0], l_break_up[1]);
                //draw_3x3(l_break_down[0], l_break_down[1],uesr_RED);
            }
        }

        if(r_border[i]<=r_border[i+4] && r_border[i]<=r_border[i-4]
         &&r_border[i]<r_border[i+5] && r_border[i]<r_border[i-5]
         &&r_border[i]<r_border[i+3] && r_border[i]<r_border[i-3]
         &&r_border[i]!=border_max 
         &&r_border[i+4]!=border_max && r_border[i+5]!=border_max
         &&r_border[i-4]!=border_max && r_border[i-5]!=border_max
         &&r_border[i-3]!=border_max && r_border[i+3]!=border_max
         &&bin_image[i-2][r_border[i]+10] == 0)
        {
            flag_r_p = 1;
            r_p[0]=r_border[i];
            r_p[1]=i;
            //printf("rp：X=%d, Y=%d\r\n", r_p[0], r_p[1]);
            //draw_3x3(r_p[0],r_p[1],uesr_BLUE);
        }

        if(l_border[i]>=l_border[i+4] && l_border[i]>=l_border[i-4]
         &&l_border[i]>l_border[i+5] && l_border[i]>l_border[i-5]
         &&l_border[i]>l_border[i+3] && l_border[i]>l_border[i-3]
         &&l_border[i]!=border_min 
         &&l_border[i+4]!=border_min && l_border[i+5]!=border_min
         &&l_border[i-4]!=border_min && l_border[i-5]!=border_min
         &&l_border[i-3]!=border_min && l_border[i+3]!=border_min
         &&bin_image[i-2][l_border[i]-10] == 0)
        {
            flag_l_p = 1;
            l_p[0]=l_border[i];
            l_p[1]=i;
            //printf("lp\n");
        }
    }

    if ((!flag_r_v_down) && flag_l_conti)  //右下v点未找到且左边线连续
    {
        for (i = data_stastics_r; i > end_check; i--) 
        {
            uint16 index = i;   
            // 防越界保护
            if(index == 0xFFFF || index < 3 || index + 3 >= data_stastics_r)
            continue;

            if (points_r[index-8][0]>points_r[index][0]
            &&points_r[index-8][1]>points_r[index][1]
            &&points_r[index+8][0]>points_r[index][0]
            &&points_r[index+8][1]>points_r[index][1])
            {
                r_v_down[0] = points_r[index][0];
                r_v_down[1] = points_r[index][1];
                flag_r_v_down = 1;
                //printf("rvdown\n");
                //ips200.draw_point(r_v_down[0], r_v_down[1], uesr_GREEN);
                break;
            }
        }
              
    }

    if ((!flag_r_v_up) && r_break_up[1] != 0)  //右上v点未找到且左边线连续
    {
       for (i = data_stastics_r; i > end_check; i--) 
        {
                uint16 index = i;  

                // 防越界保护
                if(index == 0xFFFF || index < 3 || index + 3 >= data_stastics_r)
                    continue;

                if (points_r[index-8][0]>points_r[index][0]
                &&points_r[index-8][1]<=points_r[index][1]
                &&points_r[index+8][0]<points_r[index][0]
                &&points_r[index+8][1]<points_r[index][1]
                &&index != 0xFFFF)
                {
                    r_v_up[0] = points_r[index][0];
                    r_v_up[1] = points_r[index][1];
                    flag_r_v_up = 1;
                    //printf("rvup\n");
                    //ips200.draw_point(r_v_up[0], r_v_up[1], uesr_GREEN);
                    break;
                }
            
        }  
    }

    if ((!flag_l_v_down) && flag_r_conti)  //左下v点未找到且右边线连续
    {

        for (i = data_stastics_l; i > end_check; i--) 
        {
                uint16 index = i;

                // 防越界保护
                if(index == 0xFFFF || index < 3 || index + 3 >= data_stastics_l)
                    continue;

                if (points_l[index-8][0]<points_l[index][0]
                &&points_l[index-8][1]>points_l[index][1]
                &&points_l[index+8][0]<points_l[index][0]
                &&points_l[index+8][1]>points_l[index][1])
                {
                    l_v_down[0] = points_l[index][0];
                    l_v_down[1] = points_l[index][1];
                    flag_l_v_down = 1;
                    //printf("lvdown\n");
                    ips200.draw_point(l_v_down[0], l_v_down[1], uesr_BLUE);
                    break;
                }
            
        }  
    }

    if ((!flag_l_v_up) && l_break_up[1] != 0)  //左上v点未找到且右边线连续
    {

        for (i = data_stastics_l; i > end_check; i--) 
        {
                uint16 index = i;

                // 防越界保护
                if(index == 0xFFFF || index < 3 || index + 3 >= data_stastics_l)
                    continue;

                if (points_l[index-8][0]<points_l[index][0]
                &&points_l[index-8][1]<=points_l[index][1]
                &&points_l[index+8][0]>points_l[index][0]
                &&points_l[index+8][1]<points_l[index][1])
                {
                    l_v_up[0] = points_l[index][0];
                    l_v_up[1] = points_l[index][1];
                    flag_l_v_up = 1;
                    //printf("lvup\n");
                    //ips200.draw_point(l_v_up[0], l_v_up[1], uesr_GREEN);
                    break;
                }
            
        }  
    }  
}

void cross_fill(void)
{
    if(flag_r_conti == 0 && flag_l_conti == 0)
    {
        if(r_break_down[1] != 0 && r_break_up[1] != 0
         &&l_break_down[1] != 0 && l_break_up[1] != 0)
        {
            connect_points(r_break_down, r_break_up, r_border);
            connect_points(l_break_down, l_break_up, l_border);
            //printf("右上：X=%d, Y=%d\r\n", r_break_up[0], r_break_up[1]);
            //printf("右下：X=%d, Y=%d\r\n", r_break_down[0], r_break_down[1]);
            //printf("左上：X=%d, Y=%d\r\n", l_break_up[0], l_break_up[1]);
            //printf("左下：X=%d, Y=%d\r\n", l_break_down[0], l_break_down[1]);
            //printf("补线\n");
        }
        else if(r_break_up[1] != 0 && l_break_up[1] != 0)
        {
            connect_points(screen_r_down, r_break_up, r_border);
            connect_points(screen_l_down, l_break_up, l_border);
        }
    }
}


/*
 * @brief 圆环识别状态机
 *        state 0: 待机，检测圆环入口特征
 *        state 1: 检测到特征，等待车到达入口
 *        state 2: 入圆确认（通过丢线深度+拐点消失判断）
 *        state 3: 圆内行驶，等待对侧出现出口特征
 *        state 4: 接近出口，等待两侧恢复直线
 *        state 5: 完全出圆，恢复直行
 * 调用：image_process() 中 get_left()/get_right() 之后，求 center_line 之前
 */
// ============ 元素优先级状态机 ============
// IDLE → BRICK(砖块打断圆环/十字) → IDLE
// IDLE → RING(圆环打断十字)       → IDLE
#define ELEM_IDLE  0
#define ELEM_BRICK 1
#define ELEM_RING  2
uint8 elem_state = ELEM_IDLE;

uint8 left_ring;
uint8 right_ring;

void ring_recognize(void)
{
    //  左圆环状态机
    switch (left_ring)
    {
        case 0:
            // 【初见环岛】：右边界连续 + 左边界断裂 + 找到左下V拐点 + 左P点
            if(flag_r_conti && !flag_l_conti
                && flag_l_v_down && flag_l_p
                && right_ring == 0)
            {
                // 满足环岛入口唯一特征，切换状态
                left_ring = 1;
                connect_points(l_v_down, l_p, l_border);
                printf("左状态1\n");
            }
            break;

        case 1:
            connect_points(l_v_down, l_p, l_border);
            // 【初入环岛】：左拐点消失，断裂稳定，准备入环
            if(!flag_l_v_down && !flag_l_conti && l_p[1] > 30)
            {
                left_ring = 2;
                connect_points(screen_l_down, l_p, l_border);
                printf("左状态2\n");
            }
            // 特征消失，退回待机
            else if(flag_l_conti && flag_r_conti)
            {
                left_ring = 0;
            }
            break;

        case 2:
            // 【第一次到环岛出口】：检测到左上出口V拐点
            connect_points(screen_l_down, l_p, l_border);
            if(!flag_l_conti && flag_r_conti && l_p[1]>50)
            {
                left_ring = 3;
                connect_points(l_p, l_v_up, l_border);
                printf("左状态3\n");
            }
             // 特征消失，退回待机
            else if(flag_l_conti && flag_r_conti)
            {
                left_ring = 0;
            }
            break;

        case 3:
            // 【即将入环】：出口拐点稳定，断裂持续
            connect_points(l_p, l_v_up, l_border);
            if(flag_l_v_up && !flag_l_conti && !flag_l_p)
            {
                left_ring = 4;
                Mahony_ResetZero();
                connect_points(screen_r_down, l_v_up, r_border);
                printf("左状态4\n");
            }
            break;

        case 4:
            connect_points(screen_r_down, l_v_up, r_border);
            // 【完全入环】：双侧边界恢复有效连续
            if(!flag_l_v_up && flag_l_p && eulerAngle.yaw_cont < -50)
            {
                left_ring = 5;
                printf("左状态5\n");
            }
            else if(!flag_l_v_up)
            {
                connect_points(screen_r_down, screen_l_up, r_border);
            }
            break;
 
        case 5:
            // 【即将出环】：对侧（右）出现断裂，出环特征
            if(r_break_down[1]!=0)
            {
                left_ring = 6;
                connect_points(r_break_down, screen_l_up, r_border);
                printf("左状态6\n");
            }
            else if(eulerAngle.yaw_cont < -300.0)
            {
                left_ring = 0;
                printf("左状态0\n");
            }
            break;

        case 6:
            // 【完全出环】：双侧全恢复，清除状态
            connect_points(r_break_down, screen_l_up, r_border);
            if(eulerAngle.yaw_cont <- 300.0)
            {
                left_ring = 7;
                printf("左状态7\n");
            }
            else if(r_break_down[1] == 0)
            {
                connect_points(screen_r_down, screen_l_up, r_border);
            }
            break;
        
        case 7:
           if(flag_l_v_up)
           {
              connect_points(screen_l_down, l_v_up, l_border);
           }
           else if(flag_l_conti)
           {
                left_ring = 0;
                printf("左状态0\n");
           }
           break;
 
        default:
            left_ring = 0;
            break;
    }

    
    //  右圆环状态机
    switch (right_ring)
    {
        case 0:
            // 【初见环岛】：左边界连续 + 右边界断裂 + 找到右下V拐点 + 右P点
            if(flag_l_conti && !flag_r_conti
                && flag_r_v_down && flag_r_p
                && left_ring == 0)
            {
                right_ring = 1;
                connect_points(r_v_down, r_p, r_border);
                printf("右状态1\n");
            }
            break;

        case 1:
            connect_points(r_v_down, r_p, r_border);
            // 【初入环岛】：右拐点消失，断裂稳定
            if(!flag_r_v_down && !flag_r_conti && r_p[1]>30)
            {
                right_ring = 2;
                connect_points(screen_r_down, r_p, r_border);
                printf("右状态2\n");
            }
            // 特征消失，退回待机
             else if(flag_l_conti && flag_r_conti)
            {
                right_ring = 0;
                printf("右状态0\n");
            } 
            break;

        case 2:
            connect_points(screen_r_down, r_p, r_border);
            // 【第一次到环岛出口】：检测到右上出口V拐点
            if(!flag_r_conti && flag_l_conti && r_p[1]>50)
            {
                right_ring = 3;
                connect_points(r_p, r_v_up, r_border);
                printf("右状态3\n");
            }
            // 特征消失，退回待机
            else if(flag_l_conti && flag_r_conti)
            {
                right_ring = 0;
            }
            break;

        case 3:
            connect_points(r_p, r_v_up, r_border);
            // 【即将入环】：出口拐点稳定，断裂持续
            if(flag_r_v_up && !flag_r_conti && !flag_r_p) 
            {
                right_ring = 4;
                Mahony_ResetZero();
                connect_points(screen_l_down, r_v_up, l_border);
                printf("右状态4\n");
            }
            break;

        case 4:
            connect_points(screen_l_down, r_v_up, l_border);
            // 【完全入环】：双侧边界恢复有效连续
            if(eulerAngle.yaw_cont > 50)
            {
                right_ring = 5;
                printf("右状态5\n");
            }
            else if(!flag_r_v_up)
            {
                connect_points(screen_l_down, screen_r_up, l_border);
            }
            break;

         case 5:
            // 【即将出环】：对侧（左）出现断裂，出环特征
            if(l_break_down[1]!=0)
            {
                right_ring = 6;
                connect_points(l_break_down, screen_r_up, l_border);
                printf("右状态6\n");
            }
            else if(eulerAngle.yaw_cont > 300.0)
            {
                right_ring = 0;
                printf("右状态0\n");
            }
            break;

        case 6:
            // 【完全出环】
            connect_points(l_break_down, screen_r_up, l_border);
            if(eulerAngle.yaw_cont > 300.0)
            {
                right_ring = 7;
                printf("右状态7\n");
            }
            else if(l_break_down[1] == 0)
            {
                connect_points(screen_l_down, screen_r_up, l_border);
            }
            break; 

        case 7:
            // 【保持状态】：双侧全恢复，清除状态
            if(flag_r_v_up)
            {
                connect_points(screen_r_down, r_v_up, r_border);
            }
            else if(flag_r_conti)
            {
                right_ring = 0;
                printf("右状态0\n");
            }
            break;

        default:
            right_ring = 0;
            break;
    }

    // 状态机：圆环激活/释放
    if (left_ring != 0 || right_ring != 0) {
        if (elem_state == ELEM_IDLE) elem_state = ELEM_RING;
    } else {
        if (elem_state == ELEM_RING) elem_state = ELEM_IDLE;
    }
}

/*
 * @brief 红色砖块识别+边线修正（RGB565彩色检测）
 *
 * 思路：砖块只出现在赛道左半区或右半区，不会横跨中线。
 *       检测到砖块后，修改对应侧的边线，使中线绕开砖块。
 *       - 左砖块 → l_border 推到砖块右侧 → 车右转避让
 *       - 右砖块 → r_border 推到砖块左侧 → 车左转避让
 *
 * 调用位置：image_process() 中 ring_recognize() 之后、算中线之前
 */

// ============ RGB565 红色砖块检测参数 ============
#define BRICK_R_MIN       140      // R通道最低值(8-bit, 0-255)
#define BRICK_RG_RATIO    1.35f   // R必须 > G * 此值
#define BRICK_RB_RATIO    1.35f   // R必须 > B * 此值
#define BRICK_MIN_ROW     4       // 单行最少红色像素才认为有效
#define BRICK_MIN_PIXELS  30      // 整帧最少红色像素才触发避障
#define BRICK_SAFE_MARGIN 18      // 绕行边距(像素)，越大绕得越远
#define BRICK_FADE_ROWS   5       // 过渡区行数(顶部渐变)
#define BRICK_BOTTOM_FLAT 10       // 前部平坦延伸行数
#define BRICK_BOTTOM_FADE 20       // 前部渐变区行数

void brick_recognize(void)
{
    // ==================== 1. RGB565 彩色检测，构建每行砖块范围 ====================
    uint16_t *rgb_image = uvc_dev.get_rgb_image_ptr();
    if (rgb_image == nullptr)
        return;

    // 每行砖块统计
    uint8 brick_l[IMAGE_H];     // 该行砖块最左X
    uint8 brick_r[IMAGE_H];     // 该行砖块最右X
    bool has_brick[IMAGE_H];   // 该行是否有砖块
    memset(has_brick, 0, sizeof(has_brick));

    uint16 left_cnt = 0, right_cnt = 0;     // 左右半区砖块像素计数
    uint8 brick_top = IMAGE_H, brick_bottom = 0;  // 砖块垂直范围

    // 只扫描图像下半部分（靠近小车的前方赛道）
    for (uint8 y = IMAGE_H - 70; y < IMAGE_H - 5; y++)
    {
        uint8 row_min = IMAGE_W, row_max = 0;
        uint16 row_cnt = 0;

        for (uint8 x = border_min + 3; x < border_max - 3; x++)
        {
            uint16_t pixel = rgb_image[y * IMAGE_W + x];

            // RGB565 → 8-bit 各通道
            uint8 r = ((pixel >> 11) & 0x1F) * 255 / 31;   // R: 5bit → 8bit
            uint8 g = ((pixel >> 5)  & 0x3F) * 255 / 63;   // G: 6bit → 8bit
            uint8 b = ((pixel)       & 0x1F) * 255 / 31;   // B: 5bit → 8bit

            // 红色判定：R足够高 且 R显著大于G、B
            if (r > BRICK_R_MIN
                && (float)r > (float)g * BRICK_RG_RATIO
                && (float)r > (float)b * BRICK_RB_RATIO)
            {
                //printf("r:%d\n",r);
                //printf("g:%d\n",g);
                //printf("b:%d\n",b);
                if (x < row_min) row_min = x;
                if (x > row_max) row_max = x;
                row_cnt++;

                if (x < IMAGE_W / 2) left_cnt++;
                else                 right_cnt++;
            }
        }

        // 该行有效红色区域（至少连续几个像素）
        if (row_cnt >= BRICK_MIN_ROW && row_max - row_min <= 80)
        {
            brick_l[y] = row_min;
            brick_r[y] = row_max;
            has_brick[y] = true;

            if (y < brick_top)    brick_top = y;
            if (y > brick_bottom) brick_bottom = y;
        }
    }

    // ==================== 2. 分类：左砖块 or 右砖块 ====================
    if (brick_top >= brick_bottom) {
        if (elem_state == ELEM_BRICK) elem_state = ELEM_IDLE;
        return;  // 没检测到砖块区域
    }

    uint16 total = left_cnt + right_cnt;
    if (total < BRICK_MIN_PIXELS) {
        if (elem_state == ELEM_BRICK) elem_state = ELEM_IDLE;
        return;
    }

    // 砖块不跨中线，直接比较左右半区像素数
    bool brick_on_left = (left_cnt > right_cnt);

    // ==================== 3. 备份原始边线（顶部渐变区 + 砖块区 + 前部过渡区）====================
    uint8 orig_l[IMAGE_H] = {0};
    uint8 orig_r[IMAGE_H] = {0};
    uint8 y_backup_start = (brick_top > BRICK_FADE_ROWS) ? (brick_top - BRICK_FADE_ROWS) : 0;
    uint8 y_backup_end   = brick_bottom + BRICK_BOTTOM_FLAT + BRICK_BOTTOM_FADE;
    if (y_backup_end >= IMAGE_H) y_backup_end = IMAGE_H - 1;
    for (uint8 y = y_backup_start; y <= y_backup_end; y++)
    {
        orig_l[y] = l_border[y];
        orig_r[y] = r_border[y];
    }

    // ==================== 4. 边线修正 ====================
    for (uint8 y = brick_top; y <= brick_bottom; y++)
    {
        if (!has_brick[y])
            continue;

        if (brick_on_left)
        {
            // 左砖块 → 左边界推到砖块右侧 → 车右转绕行
            uint8 new_l = brick_r[y] + BRICK_SAFE_MARGIN;
            new_l = (uint8)limit_a_b((int16)new_l, border_min, border_max);

            // 确保左右边界不交错
            if (new_l < r_border[y] - 10)
                l_border[y] = new_l;
        }
        else
        {
            // 右砖块 → 右边界推到砖块左侧 → 车左转绕行
            int16 new_r = (int16)brick_l[y] - BRICK_SAFE_MARGIN;
            new_r = limit_a_b(new_r, border_min, border_max);

            // 确保左右边界不交错
            if (new_r > l_border[y] + 10)
                r_border[y] = (uint8)new_r;
        }
    }

    // ==================== 5. 顶部过渡区平滑（渐变融合）====================
    // 从 brick_top 往上 BRICK_FADE_ROWS 行做线性插值
    if (brick_top > BRICK_FADE_ROWS)
    {
        for (uint8 y = brick_top - BRICK_FADE_ROWS; y < brick_top; y++)
        {
            if (has_brick[brick_top])
            {
                // 插值权重：越靠近砖块，修正量越大
                float t = (float)(y - (brick_top - BRICK_FADE_ROWS)) / (float)BRICK_FADE_ROWS;
                if (brick_on_left)
                {
                    float val = (float)orig_l[brick_top] + t * (float)(l_border[brick_top] - orig_l[brick_top]);
                    l_border[y] = (uint8)limit_a_b((int16)val, border_min, border_max);
                }
                else
                {
                    float val = (float)orig_r[brick_top] + t * (float)(r_border[brick_top] - orig_r[brick_top]);
                    r_border[y] = (uint8)limit_a_b((int16)val, border_min, border_max);
                }
            }
        }
    }
    //printf("brick\n");

    // ==================== 6. 前部过渡区（渐变融合）====================
    // 砖块前部 BRICK_BOTTOM_FLAT 行：与 brick_bottom 保持相同 X（平坦延伸）
    // 再往前 BRICK_BOTTOM_FADE 行：从平坦 X 线性渐变回原始边界
    if (brick_bottom + BRICK_BOTTOM_FLAT + BRICK_BOTTOM_FADE < IMAGE_H)
    {
        // 计算砖块底部修正后的 X（复刻 section 4 的逻辑）
        uint8 bottom_x;
        if (brick_on_left)
        {
            bottom_x = brick_r[brick_bottom] + BRICK_SAFE_MARGIN;
            bottom_x = (uint8)limit_a_b((int16)bottom_x, border_min, border_max);
        }
        else
        {
            int16 tmp = (int16)brick_l[brick_bottom] - BRICK_SAFE_MARGIN;
            bottom_x = (uint8)limit_a_b(tmp, border_min, border_max);
        }

        // --- 6a. 前部平坦延伸：brick_bottom+1 ~ brick_bottom+BRICK_BOTTOM_FLAT ---
        for (uint8 y = brick_bottom + 1; y <= brick_bottom + BRICK_BOTTOM_FLAT; y++)
        {
            if (brick_on_left)
            {
                if (bottom_x < r_border[y] - 10)
                    l_border[y] = bottom_x;
            }
            else
            {
                if (bottom_x > l_border[y] + 10)
                    r_border[y] = bottom_x;
            }
        }

        // --- 6b. 前部渐变区：平坦 → 原始 ---
        uint8 fade_start = brick_bottom + BRICK_BOTTOM_FLAT + 1;
        uint8 fade_end   = brick_bottom + BRICK_BOTTOM_FLAT + BRICK_BOTTOM_FADE;
        for (uint8 y = fade_start; y <= fade_end; y++)
        {
            // t: 1.0 (全修正) → 0.0 (全原始)
            float t = (float)(fade_end - y) / (float)BRICK_BOTTOM_FADE;
            if (brick_on_left)
            {
                float val = (float)bottom_x * t + (float)orig_l[y] * (1.0f - t);
                uint8 new_val = (uint8)limit_a_b((int16)val, border_min, border_max);
                if (new_val < r_border[y] - 10)
                    l_border[y] = new_val;
            }
            else
            {
                float val = (float)bottom_x * t + (float)orig_r[y] * (1.0f - t);
                uint8 new_val = (uint8)limit_a_b((int16)val, border_min, border_max);
                if (new_val > l_border[y] + 10)
                    r_border[y] = (uint8)new_val;
            }
        }
    }

    elem_state = ELEM_BRICK;  // 砖块检测成功，占据优先级
}

/*
 * @brief 元素优先级状态机
 *        优先级: 砖块 > 圆环 > 十字
 *        进入高优先级元素时打断低优先级元素
 *        调用位置: image_process() 中 get_left/get_right 之后
 */
void element_state_machine(void)
{
    // ===== 1. 砖块检测（最高优先级，仅依赖 l_border/r_border）=====
    brick_recognize();

    if (elem_state == ELEM_BRICK) {
        // 砖块激活 → 复位圆环状态机，跳过圆环和十字
        left_ring = 0;
        right_ring = 0;
        return;
    }

    // ===== 2. 特征点 + 圆环识别 =====
    find_flagpoint();
    ring_recognize();

    if (elem_state == ELEM_RING) {
        // 圆环激活 → 跳过十字补线
        return;
    }

    // ===== 3. 十字补线（无砖块、无圆环时）=====
    cross_fill();
}