#include "zf_common_headfile.hpp"

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

// ================================================================
//  圆环识别 ring_recognize
//  功能：对圆环/匝道进行特征提取并用状态机判断入圆/出圆
//  调用：image_process() 中 get_left/get_right 之后、求中线之前
// ================================================================

uint8 left_ring  = 0;  // 左圆环状态  0=无, 1-5=各阶段
uint8 right_ring = 0;  // 右圆环状态

// ---- 内部特征变量 ----
static uint16 s_lost_l       = 0;  // 左侧丢线行数
static uint16 s_lost_r       = 0;  // 右侧丢线行数
static uint8  s_ldown        = 0;  // 左向下拐点标志
static uint8  s_rdown        = 0;  // 右向下拐点标志
static uint8  s_lup          = 0;  // 左向上拐点标志（出圆）
static uint8  s_rup          = 0;  // 右向上拐点标志
static uint8  s_ldown_row    = 0;  // 左向下拐点行
static uint8  s_rdown_row    = 0;  // 右向下拐点行
static uint8  s_valid_l      = 0;  // 左线底部连续有效行数
static uint8  s_valid_r      = 0;  // 右线底部连续有效行数

/*
 * @brief 判断一段边线是否近似直线（最大偏差 < 5px 视为直线）
 * @param end_row   向上取样的最低行（=有效段顶部行）
 * @param num_rows  取样行数（建议10）
 * @param side      0=左线(l_border), 1=右线(r_border)
 * @return 1=直线, 0=非直线
 */
static uint8 ring_is_straight(uint8 end_row, uint8 num_rows, uint8 side)
{
    if (num_rows < 3 || end_row < num_rows) return 0;
    uint8 *border = (side == 0) ? l_border : r_border;
    uint8  begin  = end_row - num_rows;
    uint8  end    = end_row;
    if (end > IMAGE_H) end = IMAGE_H;

    float slope = Slope_Calculate(begin, end, border);
    uint8 mid   = (begin + end) / 2;
    float b     = (float)border[mid] - slope * (float)mid;

    float max_dev = 0.0f;
    uint8 i;
    for (i = begin; i < end; i++)
    {
        float dev = (float)border[i] - (slope * (float)i + b);
        if (dev < 0.0f) dev = -dev;
        if (dev > max_dev) max_dev = dev;
    }
    return (max_dev < 5.0f) ? 1 : 0;
}

/*
 * @brief 从当前帧边线数据提取圆环识别所需特征
 */
static void ring_update_features(void)
{
    uint16 i;

    // 丢线行数（边线贴着边缘视为丢线）
    s_lost_l = 0;
    s_lost_r = 0;
    for (i = 0; i < IMAGE_H; i++)
    {
        if (l_border[i] <= (uint8)(border_min + 1)) s_lost_l++;
        if (r_border[i] >= (uint8)(border_max - 1)) s_lost_r++;
    }

    // 从图像底部向上连续有效行数
    s_valid_l = 0;
    s_valid_r = 0;
    for (i = IMAGE_H - 1; (int16)i >= 0; i--)
    {
        if (l_border[i] <= (uint8)(border_min + 1)) break;
        s_valid_l++;
    }
    for (i = IMAGE_H - 1; (int16)i >= 0; i--)
    {
        if (r_border[i] >= (uint8)(border_max - 1)) break;
        s_valid_r++;
    }

    // 拐点检测：方向编码 4=向上 6=向右/向左 0=向下
    s_ldown = 0; s_lup = 0; s_ldown_row = 0;
    if (data_stastics_l > 8)
    {
        for (i = 1; i + 7 < data_stastics_l; i++)
        {
            // 向下拐点（行走方向向上，后折向右）
            if (dir_l[i-1]==4 && dir_l[i]==4 &&
                dir_l[i+3]==6 && dir_l[i+5]==6 && dir_l[i+7]==6)
            {
                s_ldown     = 1;
                s_ldown_row = (uint8)points_l[i][1];
                break;
            }
        }
        for (i = 1; i + 7 < data_stastics_l; i++)
        {
            // 向上拐点（行走方向向下，后折回）
            if (dir_l[i-1]==0 && dir_l[i]==0 &&
                dir_l[i+3]==2 && dir_l[i+5]==2 && dir_l[i+7]==2)
            {
                s_lup = 1;
                break;
            }
        }
    }

    s_rdown = 0; s_rup = 0; s_rdown_row = 0;
    if (data_stastics_r > 8)
    {
        for (i = 1; i + 7 < data_stastics_r; i++)
        {
            if (dir_r[i-1]==4 && dir_r[i]==4 &&
                dir_r[i+3]==6 && dir_r[i+5]==6 && dir_r[i+7]==6)
            {
                s_rdown     = 1;
                s_rdown_row = (uint8)points_r[i][1];
                break;
            }
        }
        for (i = 1; i + 7 < data_stastics_r; i++)
        {
            if (dir_r[i-1]==0 && dir_r[i]==0 &&
                dir_r[i+3]==2 && dir_r[i+5]==2 && dir_r[i+7]==2)
            {
                s_rup = 1;
                break;
            }
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
void ring_recognize(void)
{
    uint8 vl_row, vr_row;  // 有效段顶部行

    ring_update_features();

    // 有效段顶部行（=底部往上 s_valid_x 行的起始行）
    vl_row = (s_valid_l < IMAGE_H) ? (uint8)(IMAGE_H - s_valid_l) : 0;
    vr_row = (s_valid_r < IMAGE_H) ? (uint8)(IMAGE_H - s_valid_r) : 0;

    // ===========================================================
    //  左圆环状态机
    // ===========================================================
    switch (left_ring)
    {
        case 0:
            // 进入条件：只有左线出现向下拐点，右线没有；左侧丢线20~80行
            if (!right_ring
                && s_ldown && !s_rdown
                && s_lost_l > 20 && s_lost_l < 80)
            {
                // 拐点上方连续几行确认确实丢线，且右线仍是直线
                if (s_ldown_row >= 5
                    && l_border[s_ldown_row - 2] <= (uint8)(border_min + 1)
                    && l_border[s_ldown_row - 5] <= (uint8)(border_min + 1)
                    && ring_is_straight(vr_row, 10, 1)
                    && !ring_is_straight(vl_row, 10, 0))
                {
                    left_ring = 1;
                }
            }
            break;

        case 1:
            // 等待车到达入口：左线底部有效行数增长且拐点消失
            if (s_valid_l > 50 && s_valid_l < 70 && !s_ldown)
            {
                left_ring = 2;
            }
            break;

        case 2:
            // 入圆确认：左线丢线加深，且左线无有效直线段
            if (s_lost_l > 60 && !ring_is_straight(vl_row, 10, 0))
            {
                left_ring = 3;
            }
            else if (s_lost_l == 0 && s_valid_l > 80)
            {
                // 特征消失，退回
                left_ring = 0;
            }
            break;

        case 3:
            // 圆内：等待右侧出现出口特征（向下拐点）或左线开始恢复
            if (s_rdown || (s_valid_l > 40 && ring_is_straight(vl_row, 10, 0)))
            {
                left_ring = 4;
            }
            break;

        case 4:
            // 接近出口：等待两侧都能找到直线段
            if (ring_is_straight(vl_row, 10, 0) && ring_is_straight(vr_row, 10, 1)
                && !s_ldown && !s_rdown)
            {
                left_ring = 5;
            }
            break;

        case 5:
            // 完全驶出：两侧直线且无任何拐点
            if (!s_lup && !s_rup
                && ring_is_straight(vl_row, 10, 0)
                && ring_is_straight(vr_row, 10, 1))
            {
                left_ring = 0;
            }
            break;

        default:
            left_ring = 0;
            break;
    }

    // ===========================================================
    //  右圆环状态机（对称）
    // ===========================================================
    switch (right_ring)
    {
        case 0:
            if (!left_ring
                && s_rdown && !s_ldown
                && s_lost_r > 20 && s_lost_r < 80)
            {
                if (s_rdown_row >= 5
                    && r_border[s_rdown_row - 2] >= (uint8)(border_max - 1)
                    && r_border[s_rdown_row - 5] >= (uint8)(border_max - 1)
                    && ring_is_straight(vl_row, 10, 0)
                    && !ring_is_straight(vr_row, 10, 1))
                {
                    right_ring = 1;
                }
            }
            break;

        case 1:
            if (s_valid_r > 50 && s_valid_r < 70 && !s_rdown)
            {
                right_ring = 2;
            }
            break;

        case 2:
            if (s_lost_r > 60 && !ring_is_straight(vr_row, 10, 1))
            {
                right_ring = 3;
            }
            else if (s_lost_r == 0 && s_valid_r > 80)
            {
                right_ring = 0;
            }
            break;

        case 3:
            if (s_ldown || (s_valid_r > 40 && ring_is_straight(vr_row, 10, 1)))
            {
                right_ring = 4;
            }
            break;

        case 4:
            if (ring_is_straight(vl_row, 10, 0) && ring_is_straight(vr_row, 10, 1)
                && !s_ldown && !s_rdown)
            {
                right_ring = 5;
            }
            break;

        case 5:
            if (!s_lup && !s_rup
                && ring_is_straight(vl_row, 10, 0)
                && ring_is_straight(vr_row, 10, 1))
            {
                right_ring = 0;
            }
            break;

        default:
            right_ring = 0;
            break;
    }
}

/** 
* @brief 十字补线函数
* @return 返回说明
*     -<em>false</em> fail
*     -<em>true</em> succeed
 */
void cross_fill(void)
{
    uint16 i;
    uint8 break_num_l = 0;
    uint8 break_num_r = 0;
    uint8 start, end;
    float slope_l_rate = 0, intercept_l = 0;

    for (i = 1; i + 7 < data_stastics_l; i++)
    {
        if (dir_l[i - 1] == 4 && dir_l[i] == 4 && dir_l[i + 3] == 2 && dir_l[i + 5] == 2 && dir_l[i + 7] == 2)
        {
            break_num_l = points_l[i][1];
            printf("brea_knum-L:%d\n", break_num_l);
			printf("I:%d\n", i);
            break;
        }
    }
    for (i = 1; i + 7 < data_stastics_r; i++)
    {
        if (dir_r[i - 1] == 4 && dir_r[i] == 4 && dir_r[i + 3] == 6 && dir_r[i + 5] == 6 && dir_r[i + 7] == 6)
        {
            break_num_r = points_r[i][1];
            printf("brea_knum-R:%d\n", break_num_r);
			printf("I:%d\n", i);
            break;
        }
    }

    if (break_num_l && break_num_r
        && bin_image[IMAGE_H-2][4]
        && bin_image[IMAGE_H-2][IMAGE_W-4])
    {
        start = break_num_l - 15;
        start = limit_a_b(start, 0, IMAGE_H);
        end   = break_num_l - 5;
        calculate_s_i(start, end, l_border, &slope_l_rate, &intercept_l);
        for (i = break_num_l - 5; i < IMAGE_H - 1; i++)
        {
            l_border[i] = slope_l_rate * i + intercept_l;
            l_border[i] = limit_a_b(l_border[i], border_min, border_max);
        }

        start = break_num_r - 15;
        start = limit_a_b(start, 0, IMAGE_H);
        end   = break_num_r - 5;
        calculate_s_i(start, end, r_border, &slope_l_rate, &intercept_l);
        for (i = break_num_r - 5; i < IMAGE_H - 1; i++)
        {
            r_border[i] = slope_l_rate * i + intercept_l;
            r_border[i] = limit_a_b(r_border[i], border_min, border_max);
        }
        printf("补线");
    }
}