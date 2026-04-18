//-------------------------------------------------------------------------------------------------------------------
//  简介:八邻域图像处理

//------------------------------------------------------------------------------------------------------------------
#include "zf_common_headfile.hpp"

zf_device_ips200 ips200;
zf_device_uvc    uvc_dev;

uint8* gray_image = uvc_dev.get_gray_image_ptr(); //摄像头灰度图像指针
/*
函数名称：int my_abs(int value)
功能说明：求绝对值
参数说明：
函数返回：绝对值
修改时间：2022年9月8日
备    注：
example：  my_abs( x)；
 */
int my_abs(int value)
{
if(value>=0) return value;
else return -value;
}

int16 limit_a_b(int16 x, int a, int b)
{
    if(x<a) x = a;
    if(x>b) x = b;
    return x;
}

/*
函数名称：int16 limit(int16 x, int16 y)
功能说明：求x,y中的最小值
参数说明：
函数返回：返回两值中的最小值
修改时间：2022年9月8日
备    注：
example：  limit( x,  y)
 */
int16 limit1(int16 x, int16 y)
{
	if (x > y)             return y;
	else if (x < -y)       return -y;
	else                return x;
}


/*变量声明*/
uint8 original_image[IMAGE_H][IMAGE_W];
uint8 image_thereshold;//图像分割阈值
//------------------------------------------------------------------------------------------------------------------
//  @brief      获得一副灰度图像
//  @since      v1.0 
//------------------------------------------------------------------------------------------------------------------
void Get_image(uint8 gray_image[IMAGE_W])
{
#define use_num    1    //1就是不压缩，2就是压缩一倍
    uint8 i = 0, j = 0, row = 0, line = 0;
    for (i = 0; i < IMAGE_H; i += use_num)
    {
        line = 0;
        for (j = 0; j < IMAGE_W; j += use_num)
        {
            // 一维图像数据的索引：i行j列 = i * 宽度 + j
            original_image[row][line] = gray_image[i * IMAGE_W + j];
            line++;
        }
        row++;
    }
}
//------------------------------------------------------------------------------------------------------------------
//  @brief     动态阈值
//  @since      v1.0 
//------------------------------------------------------------------------------------------------------------------
uint8 otsuThreshold(uint8 *image, uint16 col, uint16 row)
{
#define GrayScale 256
    uint16 Image_Width  = col;
    uint16 Image_Height = row;
    int X; uint16 Y;
    uint8* data = image;
    int HistGram[GrayScale] = {0};
	
	uint32 Amount = 0;
    uint32 PixelBack = 0;
    uint32 PixelIntegralBack = 0;
    uint32 PixelIntegral = 0;
    int32 PixelIntegralFore = 0;
    int32 PixelFore = 0;
    double OmegaBack=0, OmegaFore=0, MicroBack=0, MicroFore=0, SigmaB=0, Sigma=0; // 类间方差;
    uint8 MinValue=0, MaxValue=0;
    uint8 Threshold = 0;
	
	
    for (Y = 0; Y <Image_Height; Y++) //Y<Image_Height改为Y =Image_Height；以便进行 行二值化
    {
        //Y=Image_Height;
        for (X = 0; X < Image_Width; X++)
        {
        HistGram[(int)data[Y*Image_Width + X]]++; //统计每个灰度值的个数信息
        }
    }




    for (MinValue = 0; MinValue < 256 && HistGram[MinValue] == 0; MinValue++) ;        //获取最小灰度的值
    for (MaxValue = 255; MaxValue > MinValue && HistGram[MinValue] == 0; MaxValue--) ; //获取最大灰度的值

    if (MaxValue == MinValue)
    {
        return MaxValue;          // 图像中只有一个颜色
    }
    if (MinValue + 1 == MaxValue)
    {
        return MinValue;      // 图像中只有二个颜色
    }

    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        Amount += HistGram[Y];        //  像素总数
    }

    PixelIntegral = 0;
    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        PixelIntegral += HistGram[Y] * Y;//灰度值总数
    }
    SigmaB = -1;
    for (Y = MinValue; Y < MaxValue; Y++)
    {
          PixelBack = PixelBack + HistGram[Y];    //前景像素点数
          PixelFore = Amount - PixelBack;         //背景像素点数
          OmegaBack = (double)PixelBack / Amount;//前景像素百分比
          OmegaFore = (double)PixelFore / Amount;//背景像素百分比
          PixelIntegralBack += HistGram[Y] * Y;  //前景灰度值
          PixelIntegralFore = PixelIntegral - PixelIntegralBack;//背景灰度值
          MicroBack = (double)PixelIntegralBack / PixelBack;//前景灰度百分比
          MicroFore = (double)PixelIntegralFore / PixelFore;//背景灰度百分比
          Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);//g
          if (Sigma > SigmaB)//遍历最大的类间方差g
          {
              SigmaB = Sigma;
              Threshold = (uint8)Y;
          }
    }
   return Threshold;
}
//------------------------------------------------------------------------------------------------------------------
//  @brief      图像二值化，这里用的是大津法二值化。
//  @since      v1.0 
//------------------------------------------------------------------------------------------------------------------
uint8 bin_image[IMAGE_H][IMAGE_W];//图像数组
void turn_to_bin(void)
{
  uint8 i,j;
 image_thereshold = otsuThreshold(original_image[0], IMAGE_W, IMAGE_H);
  for(i = 0;i<IMAGE_H;i++)
  {
      for(j = 0;j<IMAGE_W;j++)
      {
          if(original_image[i][j]>image_thereshold)bin_image[i][j] = white_pixel;
          else bin_image[i][j] = black_pixel;
      }
  }
}


/*
函数名称：void get_start_point(uint8 start_row)
功能说明：寻找两个边界的边界点作为八邻域循环的起始点
参数说明：输入任意行数
函数返回：无
修改时间：2022年9月8日
备    注：
example：  get_start_point(IMAGE_H-2)
 */
uint8 start_point_l[2] = { 0 };//左边起点的x，y值
uint8 start_point_r[2] = { 0 };//右边起点的x，y值
uint8 get_start_point(uint8 start_row)
{
	uint8 i = 0,l_found = 0,r_found = 0;
	//清零
	start_point_l[0] = 0;//x
	start_point_l[1] = 0;//y

	start_point_r[0] = 0;//x
	start_point_r[1] = 0;//y

		//从中间往左边，先找起点
	for (i = IMAGE_W / 2; i > border_min; i--)
	{
		start_point_l[0] = i;//x
		start_point_l[1] = start_row;//y
		if (bin_image[start_row][i] == 255 && bin_image[start_row][i - 1] == 0)
		{
			printf("找到左边起点image[%d][%d]\n", start_row,i);
			l_found = 1;
			break;
		}
	}

	for (i = IMAGE_W / 2; i < border_max; i++)
	{
		start_point_r[0] = i;//x
		start_point_r[1] = start_row;//y
		if (bin_image[start_row][i] == 255 && bin_image[start_row][i + 1] == 0)
		{
			printf("找到右边起点image[%d][%d]\n",start_row, i);
			r_found = 1;
			break;
		}
	}

	if(l_found&&r_found)return 1;
	else {
		printf("未找到起点\n");
		return 0;
	} 
}

/*
函数名称：void search_l_r(uint16 break_flag, uint8(*image)[IMAGE_W],uint16 *l_stastic, uint16 *r_stastic,
							uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y,uint8*hightest)

功能说明：八邻域正式开始找右边点的函数，输入参数有点多，调用的时候不要漏了，这个是左右线一次性找完。
参数说明：
break_flag_r			：最多需要循环的次数
(*image)[IMAGE_W]		：需要进行找点的图像数组，必须是二值图,填入数组名称即可
					   特别注意，不要拿宏定义名字作为输入参数，否则数据可能无法传递过来
*l_stastic				：统计左边数据，用来输入初始数组成员的序号和取出循环次数
*r_stastic				：统计右边数据，用来输入初始数组成员的序号和取出循环次数
l_start_x				：左边起点横坐标
l_start_y				：左边起点纵坐标
r_start_x				：右边起点横坐标
r_start_y				：右边起点纵坐标
hightest				：循环结束所得到的最高高度
函数返回：无
修改时间：2022年9月25日
备    注：
example：
	search_l_r((uint16)USE_num,image,&data_stastics_l, &data_stastics_r,start_point_l[0],
				start_point_l[1], start_point_r[0], start_point_r[1],&hightest);
 */
#define USE_num	IMAGE_H*300	//定义找点的数组成员个数按理说300个点能放下，但是有些特殊情况确实难顶，多定义了一点

 //存放点的x，y坐标
uint16 points_l[(uint16)USE_num][2] = { {  0 } };//左线
uint16 points_r[(uint16)USE_num][2] = { {  0 } };//右线
uint16 dir_r[(uint16)USE_num] = { 0 };//用来存储右边生长方向
uint16 dir_l[(uint16)USE_num] = { 0 };//用来存储左边生长方向
uint16 data_stastics_l = 0;//统计左边找到点的个数
uint16 data_stastics_r = 0;//统计右边找到点的个数
uint8 hightest = 0;//最高点
void search_l_r(uint16 break_flag, uint8(*image)[IMAGE_W], uint16 *l_stastic, uint16 *r_stastic, uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, uint8*hightest)
{

	uint8 i = 0, j = 0;

	//左边变量
	uint8 search_filds_l[8][2] = { {  0 } };
	uint8 index_l = 0;
	uint8 temp_l[8][2] = { {  0 } };
	uint8 center_point_l[2] = {  0 };
	uint16 l_data_statics;//统计左边
	//定义八个邻域
	static int8 seeds_l[8][2] = { {0,  1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,  0},{1, 1}, };
	//{-1,-1},{0,-1},{+1,-1},
	//{-1, 0},	     {+1, 0},
	//{-1,+1},{0,+1},{+1,+1},
	//这个是顺时针

	//右边变量
	uint8 search_filds_r[8][2] = { {  0 } };
	uint8 center_point_r[2] = { 0 };//中心坐标点
	uint8 index_r = 0;//索引下标
	uint8 temp_r[8][2] = { {  0 } };
	uint16 r_data_statics;//统计右边
	//定义八个邻域
	static int8 seeds_r[8][2] = { {0,  1},{1,1},{1,0}, {1,-1},{0,-1},{-1,-1}, {-1,  0},{-1, 1}, };
	//{-1,-1},{0,-1},{+1,-1},
	//{-1, 0},	     {+1, 0},
	//{-1,+1},{0,+1},{+1,+1},
	//这个是逆时针

	l_data_statics = *l_stastic;//统计找到了多少个点，方便后续把点全部画出来
	r_data_statics = *r_stastic;//统计找到了多少个点，方便后续把点全部画出来

	//第一次更新坐标点  将找到的起点值传进来
	center_point_l[0] = l_start_x;//x
	center_point_l[1] = l_start_y;//y
	center_point_r[0] = r_start_x;//x
	center_point_r[1] = r_start_y;//y

		//开启邻域循环
	while (break_flag--)
	{

		// 数组边界防护，放在while循环内部最前面
		if (r_data_statics >= USE_num - 1 || l_data_statics >= USE_num - 1)
		{
			printf("数组即将溢出！r_data_statics=%d, l_data_statics=%d\n", r_data_statics, l_data_statics);
		    r_data_statics = 0;  
            l_data_statics = 0;
			break; // 强制退出循环
		}
		if(break_flag == 0) break;
		//左边
		for (i = 0; i < 8; i++)//传递8F坐标
		{
			search_filds_l[i][0] = center_point_l[0] + seeds_l[i][0];//x
			search_filds_l[i][1] = center_point_l[1] + seeds_l[i][1];//y
		}
		//中心坐标点填充到已经找到的点内
		points_l[l_data_statics][0] = center_point_l[0];//x
		points_l[l_data_statics][1] = center_point_l[1];//y
		l_data_statics++;//索引加一

		//右边
		for (i = 0; i < 8; i++)//传递8F坐标
		{
			search_filds_r[i][0] = center_point_r[0] + seeds_r[i][0];//x
			search_filds_r[i][1] = center_point_r[1] + seeds_r[i][1];//y
		}
		//中心坐标点填充到已经找到的点内
		points_r[r_data_statics][0] = center_point_r[0];//x
		points_r[r_data_statics][1] = center_point_r[1];//y

		index_l = 0;//先清零，后使用
		for (i = 0; i < 8; i++)
		{
			temp_l[i][0] = 0;//先清零，后使用
			temp_l[i][1] = 0;//先清零，后使用
		}

		//左边判断
		for (i = 0; i < 8; i++)
		{
			if (image[search_filds_l[i][1]][search_filds_l[i][0]] == 0
				&& image[search_filds_l[(i + 1) & 7][1]][search_filds_l[(i + 1) & 7][0]] == 255)
			{
				temp_l[index_l][0] = search_filds_l[(i)][0];
				temp_l[index_l][1] = search_filds_l[(i)][1];
				index_l++;
				dir_l[l_data_statics - 1] = (i);//记录生长方向
			}

			if (index_l)
			{
				//更新坐标点
				center_point_l[0] = temp_l[0][0];//x
				center_point_l[1] = temp_l[0][1];//y
				for (j = 0; j < index_l; j++)
				{
					if (center_point_l[1] > temp_l[j][1])
					{
						center_point_l[0] = temp_l[j][0];//x
						center_point_l[1] = temp_l[j][1];//y
					}
				}
			}

		}
		if ((points_r[r_data_statics][0]== points_r[r_data_statics-1][0]&& points_r[r_data_statics][0] == points_r[r_data_statics - 2][0]
			&& points_r[r_data_statics][1] == points_r[r_data_statics - 1][1] && points_r[r_data_statics][1] == points_r[r_data_statics - 2][1])
			||(points_l[l_data_statics-1][0] == points_l[l_data_statics - 2][0] && points_l[l_data_statics-1][0] == points_l[l_data_statics - 3][0]
				&& points_l[l_data_statics-1][1] == points_l[l_data_statics - 2][1] && points_l[l_data_statics-1][1] == points_l[l_data_statics - 3][1]))
		{
			printf("三次进入同一个点，退出\n");
			break;
		}
		if (my_abs(points_r[r_data_statics][0] - points_l[l_data_statics - 1][0]) < 2
			&& my_abs(points_r[r_data_statics][1] - points_l[l_data_statics - 1][1] < 2)
			)
		{
			printf("\n左右相遇退出\n");	
			*hightest = (points_r[r_data_statics][1] + points_l[l_data_statics - 1][1]) >> 1;//取出最高点
			printf("\n在y=%d处退出\n",*hightest);
			break;
		}
		if ((points_r[r_data_statics][1] < points_l[l_data_statics - 1][1]))
		{
			 // 1. 新增：定义连续等待计数器（函数内顶部声明）
			static uint8 wait_count = 0;
			// 2. 最大等待次数（可根据实际调整，50次足够）
			#define MAX_WAIT_TIMES 1

			wait_count++;
			printf("\n左边比右边高，等待次数：%d\n", wait_count);

			// ============== 核心修复：超过最大等待次数 → 强制退出循环 ==============
			if(wait_count > MAX_WAIT_TIMES)
			{
				printf("等待超时，强制退出八邻域！\n");
				wait_count = 0;  // 计数器清零
				break;          // 退出while死循环
			}

			// ============== 防护：数组索引即将溢出 → 强制退出 ==============
			if(r_data_statics >= USE_num-1 || l_data_statics >= USE_num-1)
			{
				printf("数组索引溢出，强制退出！\n");
				wait_count = 0;
				break;
			}

			continue; // 正常等待：跳过本次右边处理，重新循环 

		}
			if (dir_l[l_data_statics - 1] == 7
			&& (points_r[r_data_statics][1] > points_l[l_data_statics - 1][1])
			&& l_data_statics > 1)  // 🔥 新增：禁止回退到0以下，杜绝无限死循环
		{
			center_point_l[0] = points_l[l_data_statics - 1][0];
			center_point_l[1] = points_l[l_data_statics - 1][1];
			l_data_statics--;
		}
		r_data_statics++;//索引加一

		index_r = 0;//先清零，后使用
		for (i = 0; i < 8; i++)
		{
			temp_r[i][0] = 0;//先清零，后使用
			temp_r[i][1] = 0;//先清零，后使用
		}

		//右边判断
		for (i = 0; i < 8; i++)
		{
			if (image[search_filds_r[i][1]][search_filds_r[i][0]] == 0
				&& image[search_filds_r[(i + 1) & 7][1]][search_filds_r[(i + 1) & 7][0]] == 255)
			{
				temp_r[index_r][0] = search_filds_r[(i)][0];
				temp_r[index_r][1] = search_filds_r[(i)][1];
				index_r++;//索引加一
				dir_r[r_data_statics - 1] = (i);//记录生长方向
				//printf("dir[%d]:%d\n", r_data_statics - 1, dir_r[r_data_statics - 1]);
			}
			if (index_r)
			{

				//更新坐标点
				center_point_r[0] = temp_r[0][0];//x
				center_point_r[1] = temp_r[0][1];//y
				for (j = 0; j < index_r; j++)
				{
					if (center_point_r[1] > temp_r[j][1])
					{
						center_point_r[0] = temp_r[j][0];//x
						center_point_r[1] = temp_r[j][1];//y
					}
				}

			}
		}


	}


	//取出循环次数
	*l_stastic = l_data_statics;
	*r_stastic = r_data_statics;

}
/*
函数名称：void get_left(uint16 total_L)
功能说明：从八邻域边界里提取需要的边线
参数说明：
total_L	：找到的点的总数
函数返回：无
修改时间：2022年9月25日
备    注：
example： get_left(data_stastics_l );
 */
uint8 l_border[IMAGE_H];//左线数组
uint8 r_border[IMAGE_H];//右线数组
uint8 center_line[IMAGE_H];//中线数组
void get_left(uint16 total_L)
{
	uint8 i = 0;
	uint16 j = 0;
	uint8 h = 0;
	//初始化
	for (i = 0;i<IMAGE_H;i++)
	{
		l_border[i] = border_min;
	}
	h = IMAGE_H - 2;
	//左边
	for (j = 0; j < total_L; j++)
	{
		//printf("%d\n", j);
		if (points_l[j][1] == h)
		{
			l_border[h] = points_l[j][0]+1;
		}
		else continue; //每行只取一个点，没到下一行就不记录
		h--;
		if (h == 0) 
		{
			break;//到最后一行退出
		}
	}
}
/*
函数名称：void get_right(uint16 total_R)
功能说明：从八邻域边界里提取需要的边线
参数说明：
total_R  ：找到的点的总数
函数返回：无
修改时间：2022年9月25日
备    注：
example：get_right(data_stastics_r);
 */
void get_right(uint16 total_R)
{
	uint8 i = 0;
	uint16 j = 0;
	uint8 h = 0;
	for (i = 0; i < IMAGE_H; i++)
	{
		r_border[i] = border_max;//右边线初始化放到最右边，左边线放到最左边，这样八邻域闭合区域外的中线就会在中间，不会干扰得到的数据
	}
	h = IMAGE_H - 2;
	//右边
	for (j = 0; j < total_R; j++)
	{
		if (points_r[j][1] == h)
		{
			r_border[h] = points_r[j][0] - 1;
		}
		else continue;//每行只取一个点，没到下一行就不记录
		h--;
		if (h == 0)break;//到最后一行退出
	}
}

//定义膨胀和腐蚀的阈值区间
#define threshold_max	255*5//此参数可根据自己的需求调节
#define threshold_min	255*2//此参数可根据自己的需求调节
void image_filter(uint8(*bin_image)[IMAGE_W])//形态学滤波，简单来说就是膨胀和腐蚀的思想
{
	uint16 i, j;
	uint32 num = 0;


	for (i = 1; i < IMAGE_H - 1; i++)
	{
		for (j = 1; j < (IMAGE_W - 1); j++)
		{
			//统计八个方向的像素值
			num =
				bin_image[i - 1][j - 1] + bin_image[i - 1][j] + bin_image[i - 1][j + 1]
				+ bin_image[i][j - 1] + bin_image[i][j + 1]
				+ bin_image[i + 1][j - 1] + bin_image[i + 1][j] + bin_image[i + 1][j + 1];


			if (num >= threshold_max && bin_image[i][j] == 0)
			{

				bin_image[i][j] = 255;//白  可以搞成宏定义，方便更改

			}
			if (num <= threshold_min && bin_image[i][j] == 255)
			{

				bin_image[i][j] = 0;//黑

			}

		}
	}

}

/*
函数名称：void image_draw_rectan(uint8(*image)[IMAGE_W])
功能说明：给图像画一个黑框
参数说明：uint8(*image)[IMAGE_W]	图像首地址
函数返回：无
修改时间：2022年9月8日
备    注：
example： image_draw_rectan(bin_image);
 */
void image_draw_rectan(uint8(*image)[IMAGE_W])
{

	uint8 i = 0;
	for (i = 0; i < IMAGE_H; i++)
	{
		image[i][0] = 0;
		image[i][1] = 0;
		image[i][IMAGE_W - 1] = 0;
		image[i][IMAGE_W - 2] = 0;

	}
	for (i = 0; i < IMAGE_W; i++)
	{
		image[0][i] = 0;
		image[1][i] = 0;
		//image[IMAGE_H-1][i] = 0;

	}
}


/*
函数名称：void image_process(void)
功能说明：最终处理函数
参数说明：无
函数返回：无
修改时间：2022年9月8日
备    注：
example： image_process();
 */
void image_process(void)
{
    // 1. 图像指针获取与空指针防护
	printf("绘制黑框，避免边缘干扰\n");
    gray_image = uvc_dev.get_gray_image_ptr();
    if (gray_image == nullptr)
    {
        printf("图像为空...\n");
        return;
    }

    // 2. 局部变量初始化
    uint16 i = 0;
    uint8 hightest = 0; // 边界最高行（y值最小）
    uint8 find_ok = 0;  // 起点查找成功标志

    printf("核心数据强制重置\n");
    // 3. 核心数据强制重置（解决旧数据复用导致的卡死）
    memset(points_l, 0, sizeof(points_l));         // 左边界点数组清零
    memset(points_r, 0, sizeof(points_r));         // 右边界点数组清零
    memset(dir_l, 0, sizeof(dir_l));               // 左生长方向数组清零
    memset(dir_r, 0, sizeof(dir_r));               // 右生长方向数组清零
    memset(l_border, border_min, sizeof(l_border));// 左边界数组初始化到最小值
    memset(r_border, border_max, sizeof(r_border));// 右边界数组初始化到最大值
    data_stastics_l = 0;                           // 左点计数清零
    data_stastics_r = 0;                           // 右点计数清零


    // 4. 图像预处理流程
    Get_image(gray_image);       // 灰度图像转二维数组
	printf("灰度图像转二维数组\n");
    turn_to_bin();               // 大津法二值化
	printf("大津法二值化\n");
    image_filter(bin_image);     // 形态学滤波去噪
	printf("形态学滤波去噪\n");
    image_draw_rectan(bin_image);// 绘制黑框，避免边缘干扰
	printf("绘制黑框，避免边缘干扰\n");

    // 5. 多行查找起点（提升起点查找成功率，避免单行无数据）
    for(uint8 row = IMAGE_H - 2; row >= IMAGE_H - 10; row--)// 从倒数第2行向上找10行
    {
        if (get_start_point(row))
        {
            find_ok = 1;
            printf("在第%d行找到起点\n", row);
            break;
        }
    }

    // 6. 八邻域边界查找（仅找到起点时执行）
    if (find_ok)
    {
        printf("正在开始八领域\n");
        // 执行八邻域查找
        search_l_r((uint16)USE_num, bin_image, &data_stastics_l, &data_stastics_r, 
                   start_point_l[0], start_point_l[1], start_point_r[0], start_point_r[1], &hightest);
        printf("八邻域已结束，左点数量：%d，右点数量：%d\n", data_stastics_l, data_stastics_r);
        
        // 仅当找到有效边界点时，提取边线（避免无效数据处理）
        if (data_stastics_l > 3 && data_stastics_r > 3) // 至少3个点才视为有效
        {
            get_left(data_stastics_l);
            get_right(data_stastics_r);
/*             // 补线函数调用（防护空数据）
            cross_fill(
                bin_image,          
                l_border,           
                r_border,           
                data_stastics_l,    
                data_stastics_r,    
                dir_l,              
                dir_r,              
                points_l,           
                points_r            
            ); */
        }
        else
        {
            printf("八邻域有效点不足，重置边界\n");
            memset(l_border, border_min, sizeof(l_border));
            memset(r_border, border_max, sizeof(r_border));
        }
    }
    else
    {
        // 未找到起点：重置边界+仅显示灰度图，直接返回
        printf("未找到起点，重置边界数据\n");
        memset(l_border, border_min, sizeof(l_border));
        memset(r_border, border_max, sizeof(r_border));
        ips200.displayimage_gray((uint8_t *)bin_image, UVC_WIDTH, UVC_HEIGHT);
        return;
    }

     // 7. 图像显示（仅有效数据时绘制边界，减少资源占用）
    ips200.displayimage_gray((uint8_t *)bin_image, UVC_WIDTH, UVC_HEIGHT);

    // 绘制左边界点（仅有效点时绘制）
    if (data_stastics_l > 0)
    {
        for (i = 0; i < data_stastics_l; i++)
        {
            ips200.draw_point(points_l[i][0]+2, points_l[i][1], uesr_BLUE);
        }
    }

    // 绘制右边界点（仅有效点时绘制）
    if (data_stastics_r > 0)
    {
        for (i = 0; i < data_stastics_r; i++)
        {
            ips200.draw_point(points_r[i][0]-2, points_r[i][1], uesr_RED);
        }
    }

    // 绘制中线/左右边线（仅有效高度范围内绘制）
    if (hightest < IMAGE_H - 1)
    {
        for (i = hightest; i < IMAGE_H-1; i++)
        {
            center_line[i] = (l_border[i] + r_border[i]) >> 1; // 计算中线
            ips200.draw_point(center_line[i], i, uesr_GREEN);  // 绘制中线
            ips200.draw_point(l_border[i], i, uesr_GREEN);     // 绘制左边界
            ips200.draw_point(r_border[i], i, uesr_GREEN);     // 绘制右边界
        }
    } 
}




/*

这里是起点（0.0）***************——>*************x值最大
************************************************************
************************************************************
************************************************************
************************************************************
******************假如这是一副图像*************************
***********************************************************
***********************************************************
***********************************************************
***********************************************************
***********************************************************
***********************************************************
y值最大*******************************************(188.120)

*/


