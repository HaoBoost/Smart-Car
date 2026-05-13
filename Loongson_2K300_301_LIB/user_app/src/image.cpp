#include "image.hpp"

#include "lq_common.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

#ifdef LQ_HAVE_OPENCV
#include <opencv2/imgproc.hpp>

//*****************************************全局变量声明

cv::Mat First_image;
cv::Mat Cut_image;
cv::Mat Resized_image;
cv::Mat Gray_image;
cv::Mat Bin_Frame;
#endif

int ImageScanInterval;                         // 扫描范围   当前行的边界+-ImageScanInterval
int ImageScanInterval_Cross;                   // 270度转向十字的扫描范围
int border_point = 0;
int top_point = 0;
uint8_t Image_Use[LCDH][LCDW];          // 灰度图像
uint8_t Pixle[LCDH][LCDW];              // 用于处理的二值图像
uint8_t printArray[LCDH][LCDW];
static int Ysite = 0, Xsite = 0;              // Ysite为行坐标，Xsite为列坐标
static uint8_t* PicTemp;                      // 存储单行图像
static int IntervalLow = 0, IntervalHigh = 0; // 扫描区间上下界
static int ytemp = 0;                         // 临时变量
static int TFSite = 0, FTSite = 0;            // 起始点
static float DetR = 0, DetL = 0;              // 左右斜率
static int BottomBorderRight = 79,            // 59行右边界
BottomBorderLeft = 0,                         // 59行左边界
BottomCenter = 0;                             // 59行中心点
ImageDealDatatypedef ImageDeal[60];           // 记录每行的信息
ImageStatustypedef ImageStatus;               // 图像状态（结构体）
ImageStatustypedef ImageData;                 // 图像数据（结构体）
SystemDatatypdef SystemData;                  // 系统数据（结构体）
ImageFlagtypedef ImageFlag;                   // 图像标志（结构体）
float Weighting[10] = {0.96, 0.92, 0.88, 0.83, 0.77,0.71, 0.65, 0.59, 0.53, 0.47};// 10个权重系数，按正态分布设置
uint8_t ExtenLFlag = 0;  // 是否左延长标志
uint8_t ExtenRFlag = 0;  // 是否右延长标志
uint8 Ring_Help_Flag = 0;                      // 环岛辅助标志
int Left_RingsFlag_Point1_Ysite, Left_RingsFlag_Point2_Ysite;   // 左环岛判断点的行坐标
int Right_RingsFlag_Point1_Ysite, Right_RingsFlag_Point2_Ysite; // 右环岛判断点的行坐标
int Point_Xsite,Point_Ysite;                   // 顶点坐标
int Repair_Point_Xsite,Repair_Point_Ysite;     // 修复点坐标
uint8_t *binar;    // 灰度图像数组指针

uint8 Half_Road_Wide[60] =                      // 直道半宽度
{  4, 5, 5, 6, 6, 6, 7, 7, 8, 8,
        9, 9,10,10,10,11,12,12,13,13,
       13,14,14,15,15,16,16,17,17,17,
       18,18,19,19,20,20,20,21,21,22,
       23,23,23,24,24,25,25,25,26,26,
       27,28,28,28,29,30,31,31,31,32,
};

uint8 Half_Bend_Wide[60] =                      // 弯道半宽度
{   33,33,33,33,33,33,33,33,33,33,
    33,33,32,32,30,30,29,29,28,27,
    28,27,27,26,26,25,25,24,24,23,
    22,21,21,22,22,22,23,24,24,24,
    25,25,25,26,26,26,27,27,28,28,
    28,29,29,30,30,31,31,32,32,33,
};

// 绝对值函数
float my_abs(float x)
{
    if (x < 0)
    {
        x = -x;
    }
    return x;
}

static void Image_Process()
{
#ifdef LQ_HAVE_OPENCV
    if (First_image.empty())
    {
        Gray_image.release();
        Cut_image.release();
        Resized_image.release();
        Bin_Frame.release();
        return;
    }

    const int cut_width = 160;
    const int cut_height = 60;
    if (First_image.cols < cut_width || First_image.rows < cut_height)
    {
        Gray_image.release();
        Cut_image.release();
        Resized_image.release();
        Bin_Frame.release();
        return;
    }

    cv::rotate(First_image, First_image, cv::ROTATE_180);

    int x = (First_image.cols - cut_width) / 2;
    int y = (First_image.rows - cut_height) / 2;
    cv::Rect roi(x, y, cut_width, cut_height);
    Cut_image = First_image(roi).clone();
    cv::resize(Cut_image, Resized_image, cv::Size(LCDW, LCDH));
    cv::cvtColor(Resized_image, Gray_image, cv::COLOR_BGR2GRAY);
#else
    return;
#endif
}

void Data_Settings(void)
{
    ImageStatus.MiddleLine = ImageSensorMid;
    ImageStatus.TowPoint_Gain = 0.2;
    ImageStatus.TowPoint_Offset_Max = 5;
    ImageStatus.TowPoint_Offset_Min = -2;
    ImageStatus.TowPointAdjust_v = 160;
    ImageStatus.Det_all_k = 0.7f;
    ImageStatus.CirquePass = 'F';
    ImageStatus.IsCinqueOutIn = 'F';
    ImageStatus.CirqueOut = 'F';
    ImageStatus.CirqueOff = 'F';
    ImageStatus.Barn_Flag = 0;
    ImageStatus.straight_acc = 0;
    ImageStatus.TowPoint = 21;
    ImageStatus.Threshold_static = 70;
    ImageStatus.Threshold_detach = 180;
    ImageStatus.variance_acc = 25;
    ImageStatus.newblue_flag = 0;
    ImageStatus.Road_type = Normol;
    ImageStatus.Det_True = ImageStatus.MiddleLine;
    ImageStatus.OFFLine = 2;
    ImageStatus.WhiteLine = 0;
    ImageStatus.Left_Line = 0;
    ImageStatus.Right_Line = 0;

    ImageScanInterval = 2;
    ImageScanInterval_Cross = 2;

    SystemData.clrcle_num = 0;
    SystemData.Stop = 1;
    SystemData.straighet_towpoint = 30;

    border_point = 0;
    top_point = 0;
}

void cleanup(void)
{
#ifdef LQ_HAVE_OPENCV
    First_image.release();
    Cut_image.release();
    Resized_image.release();
    Gray_image.release();
    Bin_Frame.release();
#endif
}

// 图像压缩：将OpenCV的Mat图像复制到自定义数组中
void compressimage()
{
#ifdef LQ_HAVE_OPENCV
  for (int i = 0; i < Gray_image.rows; i++)
  {
    for (int j = 0; j < Gray_image.cols; j++)
    {
      Image_Use[i][j] = Gray_image.at<uint8_t>(i, j);
    }
  }
#endif
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      优化的OTSU阈值算法
//  @param      image  图像指针
//  @param      col    列数
//  @param      row    行数
//  @param      pixel_threshold 像素阈值
//  @return     uint8 阈值
//  @since      2021.6.23
//  示例用法:   Threshold_deal(image, col, row, pixel_threshold);
//-------------------------------------------------------------------------------------------------------------------
uint8_t Threshold_deal(uint8_t* image,uint16_t col,uint16_t row,uint32_t pixel_threshold) {
#define GrayScale 256
  uint16_t width = col;
  uint16_t height = row;
  int pixelCount[GrayScale];
  float pixelPro[GrayScale];
  int i, j, pixelSum = width * height;
  uint8_t threshold = 0;
  uint8_t* data = image;  // 指向图像数据的指针
  for (i = 0; i < GrayScale; i++) {
    pixelCount[i] = 0;
    pixelPro[i] = 0;
  }

  uint32_t gray_sum = 0;
  // 统计灰度级中每个像素在整幅图像中的个数
  for (i = 0; i < height; i += 1) {
    for (j = 0; j < width; j += 1) {
      // if((sun_mode&&data[i*width+j]<pixel_threshold)||(!sun_mode))
      //{
      pixelCount[(
          int)data[i * width + j]]++;  // 将当前的像素值作为索引，累加次数
      gray_sum += (int)data[i * width + j];  // 灰度值总和
      //}
    }
  }

  // 计算每个灰度级占图像中的比例
  for (i = 0; i < GrayScale; i++) {
    pixelPro[i] = (float)pixelCount[i] / pixelSum;
  }

  // 遍历灰度级[0,255]
  float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
  w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
  for (j = 0; j < pixel_threshold; j++) {
    w0 +=
        pixelPro[j];  // 背景部分每个灰度值的像素占整个图像的比例 背景比例
    u0tmp += j * pixelPro[j];  // 背景部分 每个灰度值的比例*灰度值

    w1 = 1 - w0;
    u1tmp = gray_sum / pixelSum - u0tmp;

    u0 = u0tmp / w0;    // 背景平均灰度
    u1 = u1tmp / w1;    // 前景平均灰度
    u = u0tmp + u1tmp;  // 全局平均灰度
    deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);
    if (deltaTmp > deltaMax) {
      deltaMax = deltaTmp;
      threshold = j;
    }
    if (deltaTmp < deltaMax) break;
  }
  return threshold;
}

// 图像二值化处理
void Get01change_dajin() {
  ImageStatus.Threshold = Threshold_deal(Image_Use[0], LCDW, LCDH, ImageStatus.Threshold_detach);
  if (ImageStatus.Threshold < ImageStatus.Threshold_static)
    ImageStatus.Threshold = ImageStatus.Threshold_static;
  uint8_t i, j = 0;
  uint8_t thre;
  for (i = 0; i < LCDH; i++) {
    for (j = 0; j < LCDW; j++) {
      if (j <= 15)
        thre = ImageStatus.Threshold - 10;
      else if ((j > 70 && j <= 75))
        thre = ImageStatus.Threshold - 10;
      else if (j >= 65)
        thre = ImageStatus.Threshold - 10;
      else
        thre = ImageStatus.Threshold;

      if (Image_Use[i][j] >(thre))         // 阈值越高表示黑色越多，即浅色图像也表示黑色
        Pixle[i][j] = 1;  // 白
      else
        Pixle[i][j] = 0;  // 黑
    }
  }
}

// 中值滤波
void Pixle_Filter() 
{
  int nr;  // 行
  int nc;  // 列

  for (nr = 10; nr < 40; nr++) {
    for (nc = 10; nc < 70; nc = nc + 1) {
      if ((Pixle[nr][nc] == 0) && (Pixle[nr - 1][nc] + Pixle[nr + 1][nc] +
                                       Pixle[nr][nc + 1] + Pixle[nr][nc - 1] >=
                                   3)) {
        Pixle[nr][nc] = 1;
      }
      //      else
      //      if((Pixle[nr][nc]==1)&&(Pixle[nr-1][nc]+Pixle[nr+1][nc]+Pixle[nr][nc+1]+Pixle[nr][nc-1]<2))
      //      {
      //        Pixle[nr][nc]=0;
      //      }
    }
  }
}

// 获取跳变点：在一行数据中查找边界点
void GetJumpPointFromDet(uint8_t* p,uint8_t type,int L,int H,JumpPointtypedef* Q)  
{                                                                              // 参数1：要查找的数组（80个点）
  int i = 0;                                                                   // 参数2：扫描左右边
  if (type == 'L')                              // 扫描左边
  {
    for (i = H; i >= L; i--) {
      if (*(p + i) == 1 && *(p + i - 1) != 1)   // 从黑变白
      {
        Q->point = i;                           // 记录跳变点
        Q->type = 'T';                          // 正常边界
        break;
      } 
      else if (i == (L + 1))                  // 如果扫描完也没找到
      {
        if (*(p + (L + H) / 2) != 0)            // 如果中间是白点
        {
          Q->point = (L + H) / 2;               // 则把中间点作为边界
          Q->type = 'W';                        // 白色区域（无边）
          break;
        } 
        else                                  // 如果中间是黑点
        {
          Q->point = H;                         // 如果中间是黑点
          Q->type = 'H';                        // 黑色区域（错误）
          break;
        }
      }
    }
  } else if (type == 'R')                       // 扫描右边
  {
    for (i = L; i <= H; i++)                    // 从左向右扫描
    {
      if (*(p + i) == 1 && *(p + i + 1) != 1)   // 从黑到白的跳变
      {
        Q->point = i;                           // 记录
        Q->type = 'T';
        break;
      } else if (i == (H - 1))                  // 如果扫描完也没找到
      {
        if (*(p + (L + H) / 2) != 0)            // 如果中间是白点
        {
          Q->point = (L + H) / 2;               // 右边区域中点
          Q->type = 'W';
          break;
        } else                                  // 如果中间是黑点
        {
          Q->point = L;                         // 直接赋值错误
          Q->type = 'H';
          break;
        }
      }
    }
  }
}

// 初始化底部边界
static uint8_t DrawLinesFirst(void) 
{
  PicTemp = Pixle[59];                                     // 原59 改为57
  if (*(PicTemp + ImageSensorMid) == 0)                 // 如果图像中点黑（异常情况）
  {
    for (Xsite = 0; Xsite < ImageSensorMid; Xsite++)    // 向左右扩展
    {
      if (*(PicTemp + ImageSensorMid - Xsite) != 0)     // 一旦找到白色就停止并记录距离
        break;                                          
      if (*(PicTemp + ImageSensorMid + Xsite) != 0)
        break;
    }

    if (*(PicTemp + ImageSensorMid - Xsite) != 0)       // 如果左边先找到
    {
      BottomBorderRight = ImageSensorMid - Xsite + 1;   // 59行右边界
      for (Xsite = BottomBorderRight; Xsite > 0; Xsite--)  // 开始找59行左边界
      {
        if (*(PicTemp + Xsite) == 0 &&*(PicTemp + Xsite - 1) == 0)                // 找到两个黑点，滤波
        {
          BottomBorderLeft = Xsite;                     // 左边界找到
          break;
        } 
        else if (Xsite == 1) 
        {
          BottomBorderLeft = 0;                         // 如果找到最左边，说明左边无边，设为0
          break;
        }
      }
    } else if (*(PicTemp + ImageSensorMid + Xsite) != 0)  // 如果右边先找到
    {
      BottomBorderLeft = ImageSensorMid + Xsite - 1;    // 59行左边界
      for (Xsite = BottomBorderLeft; Xsite < 79; Xsite++)  // 开始找59行右边界
      {
        if (  *(PicTemp + Xsite) == 0
            &&*(PicTemp + Xsite + 1) == 0)              // 找到两个黑点，滤波
        {
          BottomBorderRight = Xsite;                    // 右边界找到
          break;
        } else if (Xsite == 78) {
          BottomBorderRight = 79;                       // 如果找到最右边，说明右边无边，设为79
          break;
        }
      }
    }
  }
  else                                                // 如果中点是白的，正常情况
  {
    for (Xsite = 79; Xsite >ImageSensorMid; Xsite--)   // 从右向左找右边界
    {
      if (  *(PicTemp + Xsite) == 1
          &&*(PicTemp + Xsite - 1) == 1)                // 找到两个白点，滤波
      {
        BottomBorderRight = Xsite;                      // 找到并记录
        break;
      } else if (Xsite == 40) {
        BottomBorderRight = 39;                         // 找不到设为79
        break;
      }
    }
    for (Xsite = 0; Xsite < ImageSensorMid; Xsite++)    // 从左向右找左边界
    {
      if (  *(PicTemp + Xsite) == 1
          &&*(PicTemp + Xsite + 1) == 1)                // 找到两个白点，滤波
      {
        BottomBorderLeft = Xsite;                       // 找到并记录
        break;
      } else if (Xsite == 38) {
        BottomBorderLeft = 39;                           // 找不到设为0
        break;
      }
    }
  }
  BottomCenter =(BottomBorderLeft + BottomBorderRight) / 2;   // 59行中心直接取平均
  ImageDeal[59].LeftBorder = BottomBorderLeft;                // 将左右边界记录到结构体中
  ImageDeal[59].RightBorder = BottomBorderRight;
  ImageDeal[59].Center = BottomCenter;                        // 确定底边
  ImageDeal[59].Wide = BottomBorderRight - BottomBorderLeft;  // 存储宽度信息
  ImageDeal[59].IsLeftFind = 'T';
  ImageDeal[59].IsRightFind = 'T';
  for (Ysite = 58; Ysite > 54; Ysite--)                       // 向上几行确定底部区域
  {
    PicTemp = Pixle[Ysite];
    for (Xsite = 79; Xsite > ImageDeal[Ysite + 1].Center;Xsite--)                                             // 从前一行的中点向右
    {
      if (*(PicTemp + Xsite) == 1 && *(PicTemp + Xsite - 1) == 1) {
        ImageDeal[Ysite].RightBorder = Xsite;
        break;
      } else if (Xsite == (ImageDeal[Ysite + 1].Center+1)) {
        ImageDeal[Ysite].RightBorder = ImageDeal[Ysite + 1].Center;
        break;
      }
    }
    for (Xsite = 0; Xsite < ImageDeal[Ysite + 1].Center;Xsite++)                                             // 从前一行的中点向左
    {
      if (*(PicTemp + Xsite) == 1 && *(PicTemp + Xsite + 1) == 1) {
        ImageDeal[Ysite].LeftBorder = Xsite;
        break;
      } else if (Xsite == (ImageDeal[Ysite + 1].Center-1)) {
        ImageDeal[Ysite].LeftBorder = ImageDeal[Ysite + 1].Center;
        break;
      }
    }
    ImageDeal[Ysite].IsLeftFind = 'T';                        // 这些信息存储到结构体
    ImageDeal[Ysite].IsRightFind = 'T';
    ImageDeal[Ysite].Center =
        (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) /2; // 存储中点
    ImageDeal[Ysite].Wide =
        ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder;      // 存储宽度
  }
  return 'T';
}                                                                 // 初始化完成，后续行处理不受安装时摄像头倾斜影响

/* 向上逐行处理得到边界 */
static void DrawLinesProcess(void)  
{
  uint8_t L_Found_T = 'F';  // 确定无边斜率的基准线是否找到的标志
  uint8_t Get_L_line = 'F';  // 找到第一帧图像的基准斜率

  uint8_t R_Found_T = 'F';  // 确定无边斜率的基准线是否找到的标志
  uint8_t Get_R_line = 'F';  // 找到第一帧图像的基准斜率

  float D_L = 0;           // 延长线左斜率
  float D_R = 0;           // 延长线右斜率

  int ytemp_W_L;           // 记录首次丢边界行
  int ytemp_W_R;           // 记录首次丢右边界
  ExtenRFlag = 0;          // 标志位清0
  ExtenLFlag = 0;
   ImageStatus.Left_Line = 0;
   ImageStatus.WhiteLine = 0;
   ImageStatus.Right_Line = 0;

  for (Ysite = 54 ; Ysite > ImageStatus.OFFLine; Ysite--) // 前5行已处理，从55行到设定的断行OFFLine
  {                        
    PicTemp = Pixle[Ysite];
    JumpPointtypedef JumpPoint[2];  // 0左1右

   /******************************扫描本行右边界******************************/                                           
    if (ImageStatus.Road_type != Cross_ture /* &&SystemData.SpeedData.Length*OX>500*/) 
    {
      IntervalLow =ImageDeal[Ysite + 1].RightBorder -ImageScanInterval;             // 从前一行右边界-Interval的点开始（确定扫描开始点）
      IntervalHigh =ImageDeal[Ysite + 1].RightBorder + ImageScanInterval;           // 从前一行右边界+Interval的点结束（确定扫描结束点）
    } 
    else 
    {
      IntervalLow =ImageDeal[Ysite + 1].RightBorder -ImageScanInterval_Cross;       // 从前一行右边界-Interval_Cross的点开始（确定扫描开始点）
      IntervalHigh = ImageDeal[Ysite + 1].RightBorder + ImageScanInterval_Cross;    // 从前一行右边界+Interval_Cross的点开始（确定扫描开始点）
    }

    LimitL(IntervalLow);   // 限制扫描区间并防溢出
    LimitH(IntervalHigh);  // 限制扫描区间并防溢出
    GetJumpPointFromDet(PicTemp, 'R', IntervalLow, IntervalHigh,&JumpPoint[1]);     // 扫描右边界
  /******************************扫描本行右边界******************************/  




 /******************************扫描本行左边界******************************/
    IntervalLow =ImageDeal[Ysite + 1].LeftBorder -ImageScanInterval;                // 从前一行左边界-5的点开始（确定扫描开始点）
    IntervalHigh =ImageDeal[Ysite + 1].LeftBorder +ImageScanInterval;               // 从前一行左边界+5的点结束（确定扫描结束点）

    LimitL(IntervalLow);   // 限制扫描区间并防溢出
    LimitH(IntervalHigh);  // 限制扫描区间并防溢出
    GetJumpPointFromDet(PicTemp, 'L', IntervalLow, IntervalHigh,&JumpPoint[0]);
  /******************************扫描本行左边界******************************/





    if (JumpPoint[0].type =='W')                                                    // 如果左边界扫描区间内，连续10个点都是白的
    {
      ImageDeal[Ysite].LeftBorder =ImageDeal[Ysite + 1].LeftBorder;                 // 左边界赋值为前一行的左边界
    } 
    else                                                                          // 左边界正常
    {
      ImageDeal[Ysite].LeftBorder = JumpPoint[0].point;                             // 记录边界点
    }

    if (JumpPoint[1].type == 'W')                                                   // 如果右边界扫描区间内
    {
      ImageDeal[Ysite].RightBorder =ImageDeal[Ysite + 1].RightBorder;               // 右边界赋值为前一行的右边界值
    } 
    else                                                                          // 右边界正常
    {
      ImageDeal[Ysite].RightBorder = JumpPoint[1].point;                            // 记录边界点
    }

    ImageDeal[Ysite].IsLeftFind =JumpPoint[0].type;                                 // 记录左边界是否找到（或类型）
    ImageDeal[Ysite].IsRightFind = JumpPoint[1].type;



    /************************************处理异常边界（H类）*************************************/
    if ((ImageDeal[Ysite].IsLeftFind == 'H'||ImageDeal[Ysite].IsRightFind == 'H')) 
    {

      /**************************处理左边界的异常***************************/
      if (ImageDeal[Ysite].IsLeftFind == 'H')                                   // 左边界异常
      { 
        for (Xsite = (ImageDeal[Ysite].LeftBorder + 1);Xsite <= (ImageDeal[Ysite].RightBorder - 1);Xsite++)                                                           // 在左右边界之间扫描
          {
            if ((*(PicTemp + Xsite) == 0) && (*(PicTemp + Xsite + 1) != 0)) 
            {
              ImageDeal[Ysite].LeftBorder =Xsite;                                 // 找到左边第一个黑到白的跳变作为左边界直接取左
              ImageDeal[Ysite].IsLeftFind = 'T';
              break;
            } 
            else if (*(PicTemp + Xsite) != 0) break;                                  // 一旦遇到白点就直接退出
              
            else if (Xsite ==(ImageDeal[Ysite].RightBorder - 1))
            {
              ImageDeal[Ysite].IsLeftFind = 'T';
              break;
            }
          }
      }
      
      if ((ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder) <=7)                              // 图像宽度过窄
      {
        ImageStatus.OFFLine = Ysite + 1;  // 如果宽度小于7就直接不要了
        break;
      }

      /**************************处理右边界的异常***************************/
      if (ImageDeal[Ysite].IsRightFind == 'H')
      { 
        for (Xsite = (ImageDeal[Ysite].RightBorder - 1);Xsite >= (ImageDeal[Ysite].LeftBorder + 1); Xsite--) 
        {
          if ((*(PicTemp + Xsite) == 0) && (*(PicTemp + Xsite - 1) != 0))
          {
            ImageDeal[Ysite].RightBorder =Xsite;                    // 找到右边界左边的第一个黑到白跳变作为右边界直接取右
            ImageDeal[Ysite].IsRightFind = 'T';
            break;
          } 
          else if (*(PicTemp + Xsite) != 0)
              break;
          else if (Xsite == (ImageDeal[Ysite].LeftBorder + 1))
          {
              ImageDeal[Ysite].RightBorder = Xsite;
              ImageDeal[Ysite].IsRightFind = 'T';
              break;
          }
        }
      }
    }

    /************************************处理无边行（W类）的边界************************************/
    int ysite = 0;
    uint8_t L_found_point = 0;
    uint8_t R_found_point = 0;


    if(ImageStatus.Road_type != Ramp)
    {
      if (ImageDeal[Ysite].IsRightFind == 'W'&&Ysite > 10&&Ysite < 50&&ImageStatus.Road_type!=Barn_in) // 如果当前行右边界无边，先看右边界
      {
        if (Get_R_line == 'F')    // 第一帧图像没有能够找到一条基准线的情况
        {
          Get_R_line = 'T';       // 只记录一次 一帧图像只一次 为T
          ytemp_W_R = Ysite + 2;
          for (ysite = Ysite + 1; ysite < Ysite + 15; ysite++) 
          {
            if (ImageDeal[ysite].IsRightFind =='T')  R_found_point++;     // 在无边行下方找有边界行 一般都是有边的    
          }

          if (R_found_point >8)                      // 找到基准斜率后 延长线确定无边 有边的点数大于8
          {
            D_R = ((float)(ImageDeal[Ysite + R_found_point].RightBorder - ImageDeal[Ysite + 3].RightBorder)) /((float)(R_found_point - 3));
                                                    // 用这些有边界点计算斜率
                                                    // 用无边行的下方有边行来延长
            if (D_R > 0) 
            {
              R_Found_T ='T';                       // 如果斜率大于0 那么找到了正确的基准线 因为丢的右边
                                                    // 一般来说右边界斜率大于0 小于0说明右边界向左延长 没必要
            } 
            else 
            {
              R_Found_T = 'F';                      // 没找到基准线
              if(D_R < 0)  ExtenRFlag = 'F';                   // 延长标志位复位 死角点补线 防止图像误导

            }
          }
        }


        if (R_Found_T == 'T') ImageDeal[Ysite].RightBorder =ImageDeal[ytemp_W_R].RightBorder -D_R * (ytemp_W_R - Ysite);  // 如果找到了 那么以基准线来延长
         

        LimitL(ImageDeal[Ysite].RightBorder);  // 防溢出
        LimitH(ImageDeal[Ysite].RightBorder);  // 防溢出
      }

      if(ImageDeal[Ysite].IsLeftFind == 'W' && Ysite > 10 && Ysite < 50 &&ImageStatus.Road_type != Barn_in)    // 同理 左边界处理
      {
        if (Get_L_line == 'F')
        {
          Get_L_line = 'T';
          ytemp_W_L = Ysite + 2;
          for (ysite = Ysite + 1; ysite < Ysite + 15; ysite++) 
          {
            if (ImageDeal[ysite].IsLeftFind == 'T')
              L_found_point++;
          }
          if (L_found_point > 8)              // 找到基准斜率后 延长线确定无边
          {
            D_L = ((float)(ImageDeal[Ysite + 3].LeftBorder -ImageDeal[Ysite + L_found_point].LeftBorder)) /((float)(L_found_point - 3));
            if (D_L > 0) 
            {
              L_Found_T = 'T';

            } 
            else 
            {
              L_Found_T = 'F';
              if (D_L < 0)
                ExtenLFlag = 'F';
            }
          }
        }

        if (L_Found_T == 'T')
          ImageDeal[Ysite].LeftBorder =ImageDeal[ytemp_W_L].LeftBorder + D_L * (ytemp_W_L - Ysite);

        LimitL(ImageDeal[Ysite].LeftBorder);  // 防溢出
        LimitH(ImageDeal[Ysite].LeftBorder);  // 防溢出
      }
  }



    if (ImageDeal[Ysite].IsLeftFind == 'W'&&ImageDeal[Ysite].IsRightFind == 'W')
    {
        ImageStatus.WhiteLine++;  // 要统计左右都无边，白线数+1
    }
    if (ImageDeal[Ysite].IsLeftFind == 'W'&&Ysite<55)
    {
      ImageStatus.Left_Line++;
    }
    if(ImageDeal[Ysite].IsRightFind == 'W'&&Ysite<55)
    {
      ImageStatus.Right_Line++;
    }


      LimitL(ImageDeal[Ysite].LeftBorder);   // 防溢出
      LimitH(ImageDeal[Ysite].LeftBorder);   // 防溢出
      LimitL(ImageDeal[Ysite].RightBorder);  // 防溢出
      LimitH(ImageDeal[Ysite].RightBorder);  // 防溢出

      ImageDeal[Ysite].Wide =ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder;
      ImageDeal[Ysite].Center =(ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;

    if (ImageDeal[Ysite].Wide <= 7)         // 确定视觉距离
    {
      ImageStatus.OFFLine = Ysite + 1;
      break;
    }

    else if (  ImageDeal[Ysite].RightBorder <= 10||ImageDeal[Ysite].LeftBorder >= 70) 
    {
      ImageStatus.OFFLine = Ysite + 1;// 当图像左边小于10或者右边达到一定值时停止巡线
      break;
    }                       
  }


  return;
}

// 延长线绘制，实际上是基准线确定
static void DrawExtensionLine(void)        // 处理延长线部分，确保边界连续 巡线部分斜率
{
/***************************************************延长右边界 ***********************************************/ 
  if ((ImageStatus.Road_type != Barn_in&&ImageStatus.Road_type != Ramp)/*&&ImageStatus.pansancha_Lenth* OX==0*/&&ImageStatus.Road_type !=LeftCirque&&ImageStatus.Road_type !=RightCirque)// g5.22  6.22注意注释  ǵĻ���//过三叉元素时
  {

/**************************************** 延长左边界****************************************/   
    if(ImageStatus.WhiteLine >= ImageStatus.TowPoint_True - 15) TFSite = 55;    
//    if (ImageStatus.CirqueOff == 'T' && ImageStatus.Road_type == LeftCirque) TFSite = 55;  

    if(ExtenLFlag != 'F')
    {
      for (Ysite = 54; Ysite >= (ImageStatus.OFFLine + 4);Ysite--)// 从当前行开始向下扫扫到断行处  结尾处//延一次                                  
      {
        PicTemp = Pixle[Ysite];           // 存当前行
        if (ImageDeal[Ysite].IsLeftFind =='W')                          // 如果当前行左边界没扫到，扫描是白色，说明这里没有左边界
        {
          //**************************************************//**************************************************
          if (ImageDeal[Ysite + 1].LeftBorder >= 70)                    // 左边界实在太右边
          {
            ImageStatus.OFFLine = Ysite + 1;
            break;                        // 直接结束向上搜索
          }
          //************************************************//*************************************************

          while (Ysite >= (ImageStatus.OFFLine + 4))                    // 只要当前行没扫到左边界就一直循环
          {
            Ysite--;                      // 继续向上扫
            if (ImageDeal[Ysite].IsLeftFind == 'T'&&ImageDeal[Ysite - 1].IsLeftFind == 'T'&&ImageDeal[Ysite - 2].IsLeftFind == 'T'&&ImageDeal[Ysite - 2].LeftBorder > 0&&ImageDeal[Ysite - 2].LeftBorder <70)                                                       // 连续三行出现了并且左边界点（左边界在空白上方）
            {
              FTSite = Ysite - 2;          // 把连续三行的第二行存入FTsite
              break;
            }
          }

          DetL =((float)(ImageDeal[FTSite].LeftBorder -ImageDeal[TFSite].LeftBorder)) /((float)(FTSite - TFSite));  // 左边界斜率，一般为正

          if (FTSite > ImageStatus.OFFLine)
          {          
            for (ytemp = TFSite; ytemp >= FTSite; ytemp--)         // 从第一次扫描左边界到第二行的坐标开始向下扫直到空白上方左边界填充数值
            {
              ImageDeal[ytemp].LeftBorder =(int)(DetL * ((float)(ytemp - TFSite))) +ImageDeal[TFSite].LeftBorder;   // 将期间的空白补线（斜线），目的是拟合图像曲线
            }
          }

        } 
        else
          TFSite = Ysite + 2;                                           // 如果当前行左边界找到了，把当前行的下一行存入，便于计算斜率
      }
    }
/**************************************** 延长左边界****************************************/   

/**************************************** 延长右边界****************************************/   
    if (ImageStatus.WhiteLine >= ImageStatus.TowPoint_True - 15)
      TFSite = 55;
    // g5.22
    if (ImageStatus.CirqueOff == 'T' && ImageStatus.Road_type == RightCirque)
      TFSite = 55;

    if (ExtenRFlag != 'F')
    
    {
      for (Ysite = 54; Ysite >= (ImageStatus.OFFLine + 4); Ysite--)            // 从当前行开始向下扫扫到断行处
      {
        PicTemp = Pixle[Ysite];  // 存当前行

        if (ImageDeal[Ysite].IsRightFind =='W')                       // 如果当前行右边界没扫到，扫描是白色，说明这里没有右边界，但是存在边界点
        {

          if (ImageDeal[Ysite + 1].RightBorder <= 10)                 // 右边界实在太左边
          {
            ImageStatus.OFFLine =Ysite + 1;                           // 直接结束说明向上搜索结束
            break;
          }

          while (Ysite >= (ImageStatus.OFFLine + 4))                  // 当前行没扫到右边界就一直循环
          {
            Ysite--;
            if (  ImageDeal[Ysite].IsRightFind == 'T'&&ImageDeal[Ysite - 1].IsRightFind == 'T'&&ImageDeal[Ysite - 2].IsRightFind == 'T'&&ImageDeal[Ysite - 2].RightBorder < 70&&ImageDeal[Ysite - 2].RightBorder > 10)  // 连续三行出现了并且右边界点（右边界在空白上方）
            {
              FTSite = Ysite - 2;                                      // 把连续三行的第二行存入FTsite
              break;
            }
          }

          DetR =((float)(ImageDeal[FTSite].RightBorder -ImageDeal[TFSite].RightBorder)) /((float)(FTSite - TFSite));         // 右边界斜率，一般为负

          if (FTSite > ImageStatus.OFFLine)
          {          
            for (ytemp = TFSite; ytemp >= FTSite;ytemp--)              // 从第一次扫描右边界到第二行的坐标开始向下扫直到空白上方右边界填充数值
            {
              ImageDeal[ytemp].RightBorder =(int)(DetR * ((float)(ytemp - TFSite))) +ImageDeal[TFSite].RightBorder;          // 将期间的空白补线（斜线），目的是拟合图像曲线
            }
          }

        } 
        else
          TFSite =Ysite +2;                                           // 如果当前行右边界找到了，把当前行的下一行存入TFsite
      }
    }
/**************************************** 延长右边界****************************************/    
  }
/***************************************************延长右边界 ***********************************************/ 

/********************************************延长后重新计算中线和宽度 ***********************************************/
  for (Ysite = 59; Ysite >= ImageStatus.OFFLine; Ysite--) 
  {
    ImageDeal[Ysite].Center =(ImageDeal[Ysite].LeftBorder + ImageDeal[Ysite].RightBorder) /2;                                // 扫描结束后最后一行优化之后的中值重新计算
    ImageDeal[Ysite].Wide =-ImageDeal[Ysite].LeftBorder +ImageDeal[Ysite].RightBorder;                                       // 存储优化之后的宽度
  }
/********************************************延长后重新计算中线和宽度 ***********************************************/  
}


/* 上边界S型巡线法，因为用于环岛判断元素的第二次判断 */
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Search_Bottom_Line_OTSU
//  @brief          获取底部左右边界
//  @param          imageInput[IMAGE_ROW][IMAGE_COL]        输入的二值图像数组
//  @param          Row                                     图像行数
//  @param          Col                                     图像列数
//  @param          Bottonline                              底部行选择
//  @return         Bottonline                              底边线
//  @time           2022年10月9日
//  @Author
//  示例用法:       Search_Bottom_Line_OTSU(imageInput, Row, Col, Bottonline);
//--------------------------------------------------------------------------------------------------------------------------------------------

void Search_Bottom_Line_OTSU(uint8 imageInput[LCDH][LCDW], uint8 Row, uint8 Col, uint8 Bottonline)
{

    // 寻找左边界
    for (int Xsite = Col / 2-2; Xsite > 1; Xsite--)
    {
        if (imageInput[Bottonline][Xsite] == 1 && imageInput[Bottonline][Xsite - 1] == 0)
        {
            ImageDeal[Bottonline].LeftBoundary = Xsite;// 获取底部左边界
            break;
        }
    }
    for (int Xsite = Col / 2+2; Xsite < LCDW-1; Xsite++)
    {
        if (imageInput[Bottonline][Xsite] == 1 && imageInput[Bottonline][Xsite + 1] == 0)
        {
            ImageDeal[Bottonline].RightBoundary = Xsite;// 获取底部右边界
            break;
        }
    }

}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Search_Left_and_Right_Lines
//  @brief          通过sobel获取左右边界
//  @param          imageInput[IMAGE_ROW][IMAGE_COL]        输入的二值图像数组
//  @param          Row                                     图像行数
//  @param          Col                                     图像列数
//  @param          Bottonline                              底部行选择
//  @return         无
//  @time           2022年10月7日
//  @Author
//  示例用法:       Search_Left_and_Right_Lines(imageInput, Row, Col, Bottonline);
//--------------------------------------------------------------------------------------------------------------------------------------------

void Search_Left_and_Right_Lines(uint8 imageInput[LCDH][LCDW], uint8 Row, uint8 Col, uint8 Bottonline)
{
    // 定义小车的当前方向状态为 上 右 下 左 一定要是 上（白色） 左（黑色） 下（白色） 右（黑色）
/*  前四个方向：
                *   0
                * 3   1
                *   2
*/
/* 寻找左边界 */
    int Left_Rule[2][8] = {
                                  {0,-1,1,0,0,1,-1,0 },//{0,-1},{1,0},{0,1},{-1,0},  (x,y )
                                  {-1,-1,1,-1,1,1,-1,1} //{-1,-1},{1,-1},{1,1},{-1,1}
    };
    /* 寻找右边界 */
    int Right_Rule[2][8] = {
                              {0,-1,1,0,0,1,-1,0 },//{0,-1},{1,0},{0,1},{-1,0},
                              {1,-1,1,1,-1,1,-1,-1} //{1,-1},{1,1},{-1,1},{-1,-1}
    };
      int num=0;
    uint8 Left_Ysite = Bottonline;
    uint8 Left_Xsite = ImageDeal[Bottonline].LeftBoundary;
    uint8 Left_Rirection = 0;// 左方向
    uint8 Pixel_Left_Ysite = Bottonline;
    uint8 Pixel_Left_Xsite = 0;

    uint8 Right_Ysite = Bottonline;
    uint8 Right_Xsite = ImageDeal[Bottonline].RightBoundary;
    uint8 Right_Rirection = 0;// 右方向
    uint8 Pixel_Right_Ysite = Bottonline;
    uint8 Pixel_Right_Xsite = 0;
    uint8 Ysite = Bottonline;
    ImageStatus.OFFLineBoundary = 5;
    while (1)
    {
            num++;
            if(num>480)
            {
                 ImageStatus.OFFLineBoundary = Ysite;
                 border_point = Ysite;
                 top_point=Ysite;
                break;
            }
        if (Ysite >= Pixel_Left_Ysite && Ysite >= Pixel_Right_Ysite)
        {
            if (Ysite < ImageStatus.OFFLineBoundary)
            {
                ImageStatus.OFFLineBoundary = Ysite;
                border_point = Ysite;
                top_point=Ysite;
                break;
            }
            else
            {
                Ysite--;
            }
        }
        /********* 左边巡线 *******/
        if ((Pixel_Left_Ysite > Ysite) || Ysite == ImageStatus.OFFLineBoundary)// 右边扫描
        {
            /* 检查前方 */
            Pixel_Left_Ysite = Left_Ysite + Left_Rule[0][2 * Left_Rirection + 1];
            Pixel_Left_Xsite = Left_Xsite + Left_Rule[0][2 * Left_Rirection];

            if (imageInput[Pixel_Left_Ysite][Pixel_Left_Xsite] == 0)// 前方是黑色
            {
                // 顺时针旋转90
                if (Left_Rirection == 3)
                    Left_Rirection = 0;
                else
                    Left_Rirection++;
            }
            else// 前方是白色
            {
                /* 检查左前方 */
                Pixel_Left_Ysite = Left_Ysite + Left_Rule[1][2 * Left_Rirection + 1];
                Pixel_Left_Xsite = Left_Xsite + Left_Rule[1][2 * Left_Rirection];

                if (imageInput[Pixel_Left_Ysite][Pixel_Left_Xsite] == 0)// 左前方为黑色
                {
                    // 方向不变 Left_Rirection
                    Left_Ysite = Left_Ysite + Left_Rule[0][2 * Left_Rirection + 1];
                    Left_Xsite = Left_Xsite + Left_Rule[0][2 * Left_Rirection];
                    if (ImageDeal[Left_Ysite].LeftBoundary_First == 0){
                        ImageDeal[Left_Ysite].LeftBoundary_First = Left_Xsite;
                        ImageDeal[Left_Ysite].LeftBoundary = Left_Xsite;
                    }
                }
                else// 左前方为白色
                {
                    // 方向改变 Left_Rirection 逆时针90度
                    Left_Ysite = Left_Ysite + Left_Rule[1][2 * Left_Rirection + 1];
                    Left_Xsite = Left_Xsite + Left_Rule[1][2 * Left_Rirection];
                    if (ImageDeal[Left_Ysite].LeftBoundary_First == 0 )
                        ImageDeal[Left_Ysite].LeftBoundary_First = Left_Xsite;
                    ImageDeal[Left_Ysite].LeftBoundary = Left_Xsite;
                    if (Left_Rirection == 0)
                        Left_Rirection = 3;
                    else
                        Left_Rirection--;
                }

            }
        }
        /********* 右边巡线 *******/
        if ((Pixel_Right_Ysite > Ysite) || Ysite == ImageStatus.OFFLineBoundary)// 右边扫描
        {
            /* 检查前方 */
            Pixel_Right_Ysite = Right_Ysite + Right_Rule[0][2 * Right_Rirection + 1];
            Pixel_Right_Xsite = Right_Xsite + Right_Rule[0][2 * Right_Rirection];

            if (imageInput[Pixel_Right_Ysite][Pixel_Right_Xsite] == 0)// 前方是黑色
            {
                // 逆时针旋转90
                if (Right_Rirection == 0)
                    Right_Rirection = 3;
                else
                    Right_Rirection--;
            }
            else// 前方是白色
            {
                /* 检查右前方 */
                Pixel_Right_Ysite = Right_Ysite + Right_Rule[1][2 * Right_Rirection + 1];
                Pixel_Right_Xsite = Right_Xsite + Right_Rule[1][2 * Right_Rirection];

                if (imageInput[Pixel_Right_Ysite][Pixel_Right_Xsite] == 0)// 右前方为黑色
                {
                    // 方向不变 Right_Rirection
                    Right_Ysite = Right_Ysite + Right_Rule[0][2 * Right_Rirection + 1];
                    Right_Xsite = Right_Xsite + Right_Rule[0][2 * Right_Rirection];
                    if (ImageDeal[Right_Ysite].RightBoundary_First == 79 )
                        ImageDeal[Right_Ysite].RightBoundary_First = Right_Xsite;
                    ImageDeal[Right_Ysite].RightBoundary = Right_Xsite;
                }
                else// 右前方为白色
                {
                    // 方向改变 Right_Rirection 顺时针90度
                    Right_Ysite = Right_Ysite + Right_Rule[1][2 * Right_Rirection + 1];
                    Right_Xsite = Right_Xsite + Right_Rule[1][2 * Right_Rirection];
                    if (ImageDeal[Right_Ysite].RightBoundary_First == 79)
                        ImageDeal[Right_Ysite].RightBoundary_First = Right_Xsite;
                    ImageDeal[Right_Ysite].RightBoundary = Right_Xsite;
                    if (Right_Rirection == 3)
                        Right_Rirection = 0;
                    else
                        Right_Rirection++;
                }

            }
        }

        if (abs(Pixel_Right_Xsite - Pixel_Left_Xsite) < 3)// Ysite<80为了防止在底部前瞻线扫描到顶部 3 && Ysite < 30
        {

            ImageStatus.OFFLineBoundary = Ysite;
            border_point = Ysite;
            top_point=Ysite;
            break;
        }

    }
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Search_Border_OTSU
//  @brief          通过OTSU获取边界信息
//  @param          imageInput[IMAGE_ROW][IMAGE_COL]        输入的二值图像数组
//  @param          Row                                     图像行数
//  @param          Col                                     图像列数
//  @param          Bottonline                              底部行选择
//  @return         无
//  @time           2022年10月7日
//  @Author
//  示例用法:       Search_Border_OTSU(mt9v03x_image, IMAGE_ROW, IMAGE_COL, IMAGE_ROW-8);
//--------------------------------------------------------------------------------------------------------------------------------------------

void Search_Border_OTSU(uint8 imageInput[LCDH][LCDW], uint8 Row, uint8 Col, uint8 Bottonline)
{
    ImageStatus.WhiteLine_L = 0;
    ImageStatus.WhiteLine_R = 0;
    //ImageStatus.OFFLine = 1;
    /* 处理下边界处 */
    for (int Xsite = 0; Xsite < LCDW; Xsite++)
    {
        imageInput[0][Xsite] = 0;
        imageInput[Bottonline + 1][Xsite] = 0;
    }
    /* 处理右边界处 */
    for (int Ysite = 0; Ysite < LCDH; Ysite++)
    {
            ImageDeal[Ysite].LeftBoundary_First = 0;
            ImageDeal[Ysite].RightBoundary_First = 79;

            imageInput[Ysite][0] = 0;
            imageInput[Ysite][LCDW - 1] = 0;
    }

    // top_point=Find_top_point();

    // change_e = (1+(float)(top_point/60)*change_c);

    // printf("ctrl:%f\r\n", ((float)(top_point/60)*change_c));

    /******** 获取底部边界 *********/
    Search_Bottom_Line_OTSU(imageInput, Row, Col, Bottonline);
    /******** 获取左右边界 *********/
    Search_Left_and_Right_Lines(imageInput, Row, Col, Bottonline);



    for (int Ysite = Bottonline; Ysite > ImageStatus.OFFLineBoundary + 1; Ysite--)
    {
        if (ImageDeal[Ysite].LeftBoundary < 3)
        {
            ImageStatus.WhiteLine_L++;
        }
        if (ImageDeal[Ysite].RightBoundary > LCDW - 3)
        {
            ImageStatus.WhiteLine_R++;
        }
    }

    
}

// 左环岛判断
void Element_Judgment_Left_Rings()
{
//    Disf = 0;
    if (
              ImageStatus.Right_Line > 2
            ||ImageStatus.Left_Line < 20 // 13
            ||ImageStatus.OFFLine > 2
          //  ||variance_acc>20
           // || Straight_Judge(2, 25, 45) > 1
            || ImageStatus.WhiteLine>5
//            || (ImageDeal[48].RightBorder - ImageDeal[48].LeftBorder)<51
            // || ImageDeal[52].IsLeftFind == 'W'
            // || ImageDeal[53].IsLeftFind == 'W'
            // || ImageDeal[54].IsLeftFind == 'W'
            || ImageDeal[55].IsLeftFind == 'W'
            || ImageDeal[56].IsLeftFind == 'W'
            || ImageDeal[57].IsLeftFind == 'W'
            || ImageDeal[58].IsLeftFind == 'W'
        )
            return;
    int ring_ysite = 25;
  //  uint8 Left_Less_Num = 0;
    Left_RingsFlag_Point1_Ysite = 0;
    Left_RingsFlag_Point2_Ysite = 0;
 //   ceshi_flag = 1;
    for (int Ysite = 58; Ysite > ring_ysite; Ysite--)
    {
        if (ImageDeal[Ysite].LeftBoundary_First - ImageDeal[Ysite - 1].LeftBoundary_First > 4)
        {
            Left_RingsFlag_Point1_Ysite = Ysite;
            break;
        }
    }
    for (int Ysite = 58; Ysite > ring_ysite; Ysite--)
    {
        if (ImageDeal[Ysite + 1].LeftBoundary - ImageDeal[Ysite].LeftBoundary > 4)
        {
            Left_RingsFlag_Point2_Ysite = Ysite;
            break;
        }
    }

    for (int Ysite = Left_RingsFlag_Point1_Ysite; Ysite > ImageStatus.OFFLine; Ysite--)
    {
//        if (ImageDeal[Ysite + 3].LeftBoundary_First < ImageDeal[Ysite].LeftBoundary_First
//            && ImageDeal[Ysite + 2].LeftBoundary_First < ImageDeal[Ysite].LeftBoundary_First
//            && ImageDeal[Ysite].LeftBoundary_First > ImageDeal[Ysite - 1].LeftBoundary_First
//            && ImageDeal[Ysite].LeftBoundary_First > ImageDeal[Ysite - 1].LeftBoundary_First
//            )
        if (   ImageDeal[Ysite + 6].LeftBorder < ImageDeal[Ysite+3].LeftBorder
            && ImageDeal[Ysite + 5].LeftBorder < ImageDeal[Ysite+3].LeftBorder
            && ImageDeal[Ysite + 3].LeftBorder > ImageDeal[Ysite + 2].LeftBorder
            && ImageDeal[Ysite + 3].LeftBorder > ImageDeal[Ysite + 1].LeftBorder
            )
        {
            Ring_Help_Flag = 1;
            break;
        }
    }
    if(Left_RingsFlag_Point2_Ysite > Left_RingsFlag_Point1_Ysite+1 && Ring_Help_Flag == 0 )
    {
        if(ImageStatus.Left_Line > 7)//13
            Ring_Help_Flag = 1;
    }
    if (Left_RingsFlag_Point2_Ysite > Left_RingsFlag_Point1_Ysite+1 && Ring_Help_Flag == 1 && ImageFlag.image_element_rings_flag ==0)
    {

        ImageFlag.image_element_rings = 1;
        ImageFlag.image_element_rings_flag = 1;
        ImageFlag.ring_big_small=1;

        ImageStatus.Road_type = LeftCirque;
        //gpio_set_level(P20_8, 0);
        //wireless_uart_send_byte(9);
    }
    Ring_Help_Flag = 0;
}

//--------------------------------------------------------------
//  @name           Element_Judgment_Right_Rings()
//  @brief          通过图像判断丢右边情况，进一步判断右环岛元素.
//  @parameter      void
//  @time
//  @Author         MRCHEN
//  示例用法:       Element_Judgment_Right_Rings();
//--------------------------------------------------------------
void Element_Judgment_Right_Rings()
{
    if (   ImageStatus.Left_Line > 2
        || ImageStatus.Right_Line < 20 //13
        || ImageStatus.OFFLine >2
       // || Straight_Judge(1, 20, 45) > 1
      //  ||variance_acc>18
        || ImageStatus.WhiteLine>5
//        || (ImageDeal[48].RightBorder - ImageDeal[48].LeftBorder)<51
//        || (ImageDeal[18].RightBoundary_First - ImageDeal[18].LeftBoundary_First)<70
        // || ImageDeal[52].IsRightFind == 'W'
         || ImageDeal[53].IsRightFind == 'W'
         || ImageDeal[54].IsRightFind == 'W'
        || ImageDeal[55].IsRightFind == 'W'
        || ImageDeal[56].IsRightFind == 'W'
        || ImageDeal[57].IsRightFind == 'W'
        || ImageDeal[58].IsRightFind == 'W'
        )
        {return;}
    int ring_ysite = 25;
    Right_RingsFlag_Point1_Ysite = 0;
    Right_RingsFlag_Point2_Ysite = 0;
    for (int Ysite = 58; Ysite > ring_ysite; Ysite--)
    {
        if (ImageDeal[Ysite - 1].RightBoundary_First - ImageDeal[Ysite].RightBoundary_First > 4)
        {
            Right_RingsFlag_Point1_Ysite = Ysite;
            break;
        }
    }
    for (int Ysite = 58; Ysite > ring_ysite; Ysite--)
    {
        if (ImageDeal[Ysite].RightBoundary - ImageDeal[Ysite + 1].RightBoundary > 4)
        {
            Right_RingsFlag_Point2_Ysite = Ysite;
            break;
        }
    }
    for (int Ysite = Right_RingsFlag_Point1_Ysite; Ysite > 10; Ysite--)
    {
//        if (ImageDeal[Ysite + 3].RightBoundary_First > ImageDeal[Ysite].RightBoundary_First
//            && ImageDeal[Ysite + 2].RightBoundary_First > ImageDeal[Ysite].RightBoundary_First
//            && ImageDeal[Ysite].RightBoundary_First < ImageDeal[Ysite - 1].RightBoundary_First
//            && ImageDeal[Ysite].RightBoundary_First < ImageDeal[Ysite - 2].RightBoundary_First
//           )
        if (   ImageDeal[Ysite + 6].RightBorder > ImageDeal[Ysite + 3].RightBorder
            && ImageDeal[Ysite + 5].RightBorder > ImageDeal[Ysite + 3].RightBorder
            && ImageDeal[Ysite + 3].RightBorder < ImageDeal[Ysite + 2].RightBorder
            && ImageDeal[Ysite + 3].RightBorder < ImageDeal[Ysite + 1].RightBorder
           )
        {
            Ring_Help_Flag = 1;
            break;
        }
    }
    if(Right_RingsFlag_Point2_Ysite > Right_RingsFlag_Point1_Ysite+1 && Ring_Help_Flag == 0)
    {
        if(ImageStatus.Right_Line>7)
            Ring_Help_Flag = 1;
    }
    if (Right_RingsFlag_Point2_Ysite > Right_RingsFlag_Point1_Ysite+1 && Ring_Help_Flag == 1 && ImageFlag.image_element_rings_flag == 0)
    {

        ImageFlag.image_element_rings = 2;
        ImageFlag.image_element_rings_flag = 1;
        ImageFlag.ring_big_small=1;     // 小环
        SystemData.Stop =1;
        ImageStatus.Road_type = RightCirque;
//        flag_ceshi++;
//        gpio_set_level(Bee1p, 1);
    }
    Ring_Help_Flag = 0;
}

// 环岛处理
void Element_Handle_Left_Rings()
{
    /***************************************判断**************************************/
    int num = 0;
    for (int Ysite = 55; Ysite > 30; Ysite--)
    {
        if(ImageDeal[Ysite].IsLeftFind == 'W')
            num++;
        if(    ImageDeal[Ysite+3].IsLeftFind == 'W' && ImageDeal[Ysite+2].IsLeftFind == 'W'
            && ImageDeal[Ysite+1].IsLeftFind == 'W' && ImageDeal[Ysite].IsLeftFind == 'T')
            break;
    }
//    tft180_show_int(60,125,num,3);
//    int ring_ysite = 30;
//    for (int Ysite = 5; Ysite < ring_ysite; Ysite++)
//    {
//        if (ImageDeal[Ysite - 1].RightBoundary_First - ImageDeal[Ysite].RightBoundary_First > 4)
//        {
//            Right_RingsFlag_Point1_Ysite = Ysite;
//            break;
//        }
//    }
//    for (int Ysite = 58; Ysite > ring_ysite; Ysite--)
//    {
//        if (ImageDeal[Ysite].RightBoundary - ImageDeal[Ysite + 1].RightBoundary > 4)
//        {
//            Right_RingsFlag_Point2_Ysite = Ysite;
//            break;
//        }
//    }
        // 准备进环
    if (ImageFlag.image_element_rings_flag == 1 && num>10)
    {
        ImageFlag.image_element_rings_flag = 2;
        //wireless_uart_send_byte(2);
    }
    if (ImageFlag.image_element_rings_flag == 2 && num<8)
    {

        ImageFlag.image_element_rings_flag = 5;

        //wireless_uart_send_byte(5);
    }
        // 进环
    if(ImageFlag.image_element_rings_flag == 5 && /*num>15)*/ImageStatus.Right_Line>15)
    {
        ImageFlag.image_element_rings_flag = 6;
        
     //   ImageStatus.Road_type = LeftCirque;
        //wireless_uart_send_byte(6);
    }
        // 小环岛出环
    if(ImageFlag.image_element_rings_flag == 6 && ImageStatus.Right_Line<4)
    {
        //Stop = 1;
        ImageFlag.image_element_rings_flag = 7;
        //wireless_uart_send_byte(8);
    }
        // 出环 环岛顶点判断
    if (ImageFlag.ring_big_small == 1 && ImageFlag.image_element_rings_flag == 7)
    {
        Point_Ysite = 0;
        Point_Xsite = 0;
        for (int Ysite = 50; Ysite > ImageStatus.OFFLine + 3; Ysite--)
        {
            if (       ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite + 2].RightBorder
                    && ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite - 2].RightBorder
                    && ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite + 1].RightBorder
                    && ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite - 1].RightBorder
                    && ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite + 3].RightBorder
                    && ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite - 3].RightBorder
               )
            {
                Point_Xsite = ImageDeal[Ysite].RightBorder;
                Point_Ysite = Ysite;
                break;
            }
        }
        if (Point_Ysite > 20)
        {
            ImageFlag.image_element_rings_flag = 8;
           // wireless_uart_send_byte(8);
            //Stop = 1;
        }
    }
//        // 小环岛顶点判断
//    if (ImageFlag.image_element_rings_flag == 7 && ImageFlag.ring_big_small == 2)
//    {
//        Point_Ysite = 0;
//        Point_Xsite = 0;
//        for (int Ysite = 50; Ysite > ImageStatus.OFFLineBoundary + 3; Ysite--)
//        {
//            if (    ImageDeal[Ysite].RightBoundary < ImageDeal[Ysite + 2].RightBoundary
//                 && ImageDeal[Ysite].RightBoundary < ImageDeal[Ysite - 2].RightBoundary
//               )
//            {
//                Point_Xsite = ImageDeal[Ysite].RightBoundary;
//                Point_Ysite = Ysite;
//                break;
//            }
//        }
//        if (Point_Ysite > 20)
//          ImageFlag.image_element_rings_flag = 8;
//    }
    // 出环
        if (ImageFlag.image_element_rings_flag == 8)
        {
             if (
                     //Straight_Judge(2, ImageStatus.OFFLine+15, 50) < 1
                 ImageStatus.Right_Line < 7     // 出于对国赛工字环岛考虑，进环后补直线过渡，增强适应性
                 && ImageStatus.OFFLine < 6)    // 右边为直线且截止行（前瞻值）较小
               {ImageFlag.image_element_rings_flag = 9;
                //wireless_uart_send_byte(9);
               }
//             else if(gyro_yaw>300)
//             {
//                 ImageFlag.image_element_rings_flag = 9;
//             }
        }

        // 环岛结束
        if (ImageFlag.image_element_rings_flag == 9)
        {
            int num=0;
            for (int Ysite = 45; Ysite > 8; Ysite--)
            {
                if(ImageDeal[Ysite].IsLeftFind == 'W' )
                    num++;
            }
            if(num < 5)
            {
              ImageStatus.Road_type = Normol;   // 返回正常的道路类型 0
              ImageFlag.image_element_rings_flag = 0;
              ImageFlag.image_element_rings = 0;
              ImageFlag.ring_big_small = 0;
                //ImageStatus.Road_type = Normol;
                //wireless_uart_send_byte(0);
//                gpio_set_level(Beep, 0);
            }
    }



    /***************************************控制**************************************/
        // 准备进环 切内线
    if (   ImageFlag.image_element_rings_flag == 1
        || ImageFlag.image_element_rings_flag == 2
        || ImageFlag.image_element_rings_flag == 3
        || ImageFlag.image_element_rings_flag == 4)
    {
        for (int Ysite = 57; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Road_Wide[Ysite]-5;
        }
    }
        // 进环 切外
    if  ( ImageFlag.image_element_rings_flag == 5
        ||ImageFlag.image_element_rings_flag == 6
        )
    {
        int  flag_Xsite_1=0;
        int flag_Ysite_1=0;
        float Slope_Rings=0;
        for(Ysite=55;Ysite>ImageStatus.OFFLine;Ysite--)// 寻找A点
        {
            for(Xsite=ImageDeal[Ysite].LeftBorder + 1;Xsite<ImageDeal[Ysite].RightBorder - 1;Xsite++)
            {
                if(  Pixle[Ysite][Xsite] == 1 && Pixle[Ysite][Xsite + 1] == 0)
                 {
                   flag_Ysite_1 = Ysite;
                   flag_Xsite_1 = Xsite;
                   Slope_Rings=(float)(79-flag_Xsite_1)/(float)(59-flag_Ysite_1);
                   break;
                 }
            }
            if(flag_Ysite_1 != 0)
            {
                break;
            }
        }
        if(flag_Ysite_1 == 0)
        {
            for(Ysite=ImageStatus.OFFLine+1;Ysite<30;Ysite++)
            {
                if(ImageDeal[Ysite].IsLeftFind=='T'&&ImageDeal[Ysite+1].IsLeftFind=='T'&&ImageDeal[Ysite+2].IsLeftFind=='W'
                    &&abs(ImageDeal[Ysite].LeftBorder-ImageDeal[Ysite+2].LeftBorder)>10
                  )
                {
                    flag_Ysite_1=Ysite;
                    flag_Xsite_1=ImageDeal[flag_Ysite_1].LeftBorder;
                    ImageStatus.OFFLine=Ysite;
                    Slope_Rings=(float)(79-flag_Xsite_1)/(float)(59-flag_Ysite_1);
                    break;
                }

            }
        }
        // 补线
        if(flag_Ysite_1 != 0)
        {
            for(Ysite=flag_Ysite_1;Ysite<60;Ysite++)
            {
                ImageDeal[Ysite].RightBorder=flag_Xsite_1+Slope_Rings*(Ysite-flag_Ysite_1);
                //if(ImageFlag.ring_big_small==1)// 大环岛补中线
                    ImageDeal[Ysite].Center = ((ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder)/2);
                //else// 小环岛补中线
                //    ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Bend_Wide[Ysite];
                if(ImageDeal[Ysite].Center<4)
                    ImageDeal[Ysite].Center = 4;
            }
            ImageDeal[flag_Ysite_1].RightBorder=flag_Xsite_1;
            for(Ysite=flag_Ysite_1-1;Ysite>10;Ysite--) // A点上方继续扫线
            {
                for(Xsite=ImageDeal[Ysite+1].RightBorder-10;Xsite<ImageDeal[Ysite+1].RightBorder+2;Xsite++)
                {
                    if(Pixle[Ysite][Xsite]==1 && Pixle[Ysite][Xsite+1]==0)
                    {
                        ImageDeal[Ysite].RightBorder=Xsite;
                        //if(ImageFlag.ring_big_small==1)// 大环岛补中线
                            ImageDeal[Ysite].Center = ((ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder)/2);
                        //else// 小环岛补中线
                        //    ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Bend_Wide[Ysite];
                        if(ImageDeal[Ysite].Center<4)
                            ImageDeal[Ysite].Center = 4;
                        ImageDeal[Ysite].Wide=ImageDeal[Ysite].RightBorder-ImageDeal[Ysite].LeftBorder;
                        break;
                    }
                }

                if(ImageDeal[Ysite].Wide>8 &&ImageDeal[Ysite].RightBorder< ImageDeal[Ysite+2].RightBorder)
                {
                    continue;
                }
                else
                {
                    ImageStatus.OFFLine=Ysite+2;
                    break;
                }
            }
        }
    }
        // 小环岛出环 环内不处理
    if (ImageFlag.image_element_rings_flag == 7)
    {

    }
        // 大环岛出环 补线
    if (ImageFlag.image_element_rings_flag == 8 && ImageFlag.ring_big_small == 1)    // 大环
    {
//        Repair_Point_Xsite = 40;
        Repair_Point_Ysite = 7;
//        for (int Ysite = 40; Ysite > 5; Ysite--)
//        {
//            if (Pixle[Ysite][28] == 1 && Pixle[Ysite-1][28] == 0)//28
//            {
//                Repair_Point_Xsite = 40;
//                Repair_Point_Ysite= Ysite-1;
//                ImageStatus.OFFLine = Ysite + 1;  // 防止继续向上规划
//                break;
//            }
//        }
        for (int Ysite = 57; Ysite > Repair_Point_Ysite-3; Ysite--)         // 补线
        {
//            ImageDeal[Ysite].RightBorder = (ImageDeal[58].RightBorder - Repair_Point_Xsite) * (Ysite - 58) / (58 - Repair_Point_Ysite)  + ImageDeal[58].RightBorder;
            ImageDeal[Ysite].RightBorder = ImageDeal[Ysite].LeftBorder + Half_Bend_Wide[Ysite];
            if(ImageDeal[Ysite].RightBorder>77){ImageDeal[Ysite].RightBorder = 77;}
            ImageDeal[Ysite].Center = ((ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2);
        }
    }
//        // 小环岛出环 补线
//    if (ImageFlag.image_element_rings_flag == 8 && ImageFlag.ring_big_small == 2)    // 小环
//    {
//        Repair_Point_Xsite = 0;
//        Repair_Point_Ysite = 0;
//        for (int Ysite = 55; Ysite > 5; Ysite--)
//        {
//            if (Pixle[Ysite][15] == 1 && Pixle[Ysite-1][15] == 0)
//            {
//                Repair_Point_Xsite = 15;
//                Repair_Point_Ysite = Ysite-1;
//                ImageStatus.OFFLine = Ysite + 1;  // 防止继续向上规划
//                break;
//            }
//        }
//        for (int Ysite = 57; Ysite > Repair_Point_Ysite-3; Ysite--)         // 补线
//        {
//            ImageDeal[Ysite].RightBorder = (ImageDeal[58].RightBorder - Repair_Point_Xsite) * (Ysite - 58) / (58 - Repair_Point_Ysite)  + ImageDeal[58].RightBorder;
//            ImageDeal[Ysite].Center = (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;
//        }
//    }
        // 已出环 切内线
    if (ImageFlag.image_element_rings_flag == 9 || ImageFlag.image_element_rings_flag == 10)
    {
        for (int Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Road_Wide[Ysite];
        }
    }
}
//--------------------------------------------------------------
//  @name           Element_Handle_Right_Rings()
//  @brief          通过图像处理丢右边情况，进一步处理右环岛元素.
//  @parameter      void
//  @time
//  @Author         MRCHEN
//  示例用法:       Element_Handle_Right_Rings();
//-------------------------------------------------------------
void Element_Handle_Right_Rings()
{
    /****************判断*****************/
    int num =0 ;
    for (int Ysite = 55; Ysite > 30; Ysite--)
    {
        if(ImageDeal[Ysite].IsRightFind == 'W')
        {
            num++;
        }
        if(    ImageDeal[Ysite+3].IsRightFind == 'W' && ImageDeal[Ysite+2].IsRightFind == 'W'
            && ImageDeal[Ysite+1].IsRightFind == 'W' && ImageDeal[Ysite].IsRightFind == 'T' )
            break;
    }
        // 准备进环
    if (ImageFlag.image_element_rings_flag == 1 && num>10)
    {
        ImageFlag.image_element_rings_flag = 2;
    }
    if (ImageFlag.image_element_rings_flag == 2 && num<8)
    {
        ImageFlag.image_element_rings_flag = 5;
    }
        // 进环
    if(ImageFlag.image_element_rings_flag == 5 && ImageStatus.Left_Line>15)
    {
        ImageFlag.image_element_rings_flag = 6;
       // ImageStatus.Road_type = RightCirque;
    }
        // 小环岛出环
    if(ImageFlag.image_element_rings_flag == 6 && ImageStatus.Left_Line<4)
    {
        ImageFlag.image_element_rings_flag = 7;
        //Stop=1;
    }
    if (ImageFlag.image_element_rings_flag == 7)
    {
        Point_Xsite = 0;
        Point_Ysite = 0;
        for (int Ysite = 55; Ysite > ImageStatus.OFFLine+3; Ysite--)
        {
            if (    ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite + 2].LeftBorder
                 && ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite - 2].LeftBorder
                 && ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite + 1].LeftBorder
                 && ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite - 1].LeftBorder
                 && ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite + 4].LeftBorder
                 && ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite - 4].LeftBorder
                )

            {
                        Point_Xsite = ImageDeal[Ysite].LeftBorder;
                        Point_Ysite = Ysite;
                        break;
            }
        }
        if (Point_Ysite > 18)
        {
            ImageFlag.image_element_rings_flag = 8;
//            if(flag_ceshi == 2)
//            {
//                SystemData.Stop = 1;
//            }
        }
        else if(ImageDeal[18].RightBoundary_First - ImageDeal[18].LeftBoundary_First>70)
        {
            ImageFlag.image_element_rings_flag = 8;
        }
    }
    if (ImageFlag.image_element_rings_flag == 8)
    {
         if (   
          // Straight_Judge(1, ImageStatus.OFFLine+10, 45) < 1
             /*&&*/ ImageStatus.Left_Line < 5 // 出于对国赛工字环岛考虑，进环后补直线过渡，增强适应性
             && ImageStatus.OFFLine < 8)    // 右边为直线且截止行（前瞻值）较小
            {ImageFlag.image_element_rings_flag = 9;}

    }
    if(ImageFlag.image_element_rings_flag == 9 )
    {
        int num=0;
        for (int Ysite = 45; Ysite > 10; Ysite--)
        {
            if(ImageDeal[Ysite].IsRightFind == 'W' )
            {
                num++;
            }
        }
        if(num < 5)
        {
            ImageStatus.Road_type = Normol;   // 返回正常的道路类型 0
            ImageFlag.image_element_rings_flag = 0;
            ImageFlag.image_element_rings = 0;
            ImageFlag.ring_big_small = 0;
           // ImageStatus.Road_type = Normol;
//            Front_Ring_Continue_Count++;
//            gpio_set_level(Beep, 0);
        }
    }
    /***************************************控制**************************************/
         // 准备进环 切内线
    if (   ImageFlag.image_element_rings_flag == 1
        || ImageFlag.image_element_rings_flag == 2
        || ImageFlag.image_element_rings_flag == 3
        || ImageFlag.image_element_rings_flag == 4)
    {
        for (int Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + Half_Road_Wide[Ysite];
        }
    }

        // 进环 切外
    if (   ImageFlag.image_element_rings_flag == 5
        || ImageFlag.image_element_rings_flag == 6
       )
    {
        int flag_Xsite_1=0;
        int  flag_Ysite_1=0;
        float Slope_Right_Rings = 0;
        for(Ysite=55;Ysite>ImageStatus.OFFLine;Ysite--)
        {
            for(Xsite=ImageDeal[Ysite].LeftBorder + 1;Xsite<ImageDeal[Ysite].RightBorder - 1;Xsite++)
            {
                if(Pixle[Ysite][Xsite]==1 && Pixle[Ysite][Xsite+1]==0)
                {
                    flag_Ysite_1=Ysite;
                    flag_Xsite_1=Xsite;
                    Slope_Right_Rings=(float)(0-flag_Xsite_1)/(float)(59-flag_Ysite_1);
                    break;
                }
            }
            if(flag_Ysite_1!=0)
            {
              break;
            }
        }
        if(flag_Ysite_1==0)
        {
        for(Ysite=ImageStatus.OFFLine+5;Ysite<30;Ysite++)
        {
         if(ImageDeal[Ysite].IsRightFind=='T'&&ImageDeal[Ysite+1].IsRightFind=='T'&&ImageDeal[Ysite+2].IsRightFind=='W'
               &&abs(ImageDeal[Ysite].RightBorder-ImageDeal[Ysite+2].RightBorder)>10
         )
         {
             flag_Ysite_1=Ysite;
             flag_Xsite_1=ImageDeal[flag_Ysite_1].RightBorder;
             ImageStatus.OFFLine=Ysite;
             Slope_Right_Rings=(float)(0-flag_Xsite_1)/(float)(59-flag_Ysite_1);
             break;
         }

        }

        }
        // 补线
        if(flag_Ysite_1!=0)
        {
            for(Ysite=flag_Ysite_1;Ysite<58;Ysite++)
            {
                ImageDeal[Ysite].LeftBorder=flag_Xsite_1+Slope_Right_Rings*(Ysite-flag_Ysite_1);
//                if(ImageFlag.ring_big_small==2)// 小环岛补中线
//                    ImageDeal[Ysite].Center=ImageDeal[Ysite].LeftBorder+Half_Bend_Wide[Ysite];// 中线
//                else// 大环岛补中线
                    ImageDeal[Ysite].Center=(ImageDeal[Ysite].LeftBorder+ImageDeal[Ysite].RightBorder)/2;// 中线
                if(ImageDeal[Ysite].Center>79)
                    ImageDeal[Ysite].Center=79;
            }
            ImageDeal[flag_Ysite_1].LeftBorder=flag_Xsite_1;
            for(Ysite=flag_Ysite_1-1;Ysite>10;Ysite--) // A点上方继续扫线
            {
                for(Xsite=ImageDeal[Ysite+1].LeftBorder+8;Xsite>ImageDeal[Ysite+1].LeftBorder-4;Xsite--)
                {
                    if(Pixle[Ysite][Xsite]==1 && Pixle[Ysite][Xsite-1]==0)
                    {
                     ImageDeal[Ysite].LeftBorder=Xsite;
                     ImageDeal[Ysite].Wide=ImageDeal[Ysite].RightBorder-ImageDeal[Ysite].LeftBorder;
//                     if(ImageFlag.ring_big_small==2)// 小环岛补中线
//                         ImageDeal[Ysite].Center=ImageDeal[Ysite].LeftBorder+Half_Bend_Wide[Ysite];// 中线
//                     else// 大环岛补中线
                         ImageDeal[Ysite].Center=(ImageDeal[Ysite].LeftBorder+ImageDeal[Ysite].RightBorder)/2;// 中线
                     if(ImageDeal[Ysite].Center>79)
                         ImageDeal[Ysite].Center=79;
                     if(ImageDeal[Ysite].Center<5)
                         ImageDeal[Ysite].Center=5;
                     break;
                    }
                }
                if(ImageDeal[Ysite].Wide>8 && ImageDeal[Ysite].LeftBorder>  ImageDeal[Ysite+2].LeftBorder)
                {
                    continue;
                }
                else
                {
                    ImageStatus.OFFLine=Ysite+2;
                    break;
                }
            }
        }
    }
        // 环内不处理
    if (ImageFlag.image_element_rings_flag == 7)
    {

    }
        // 大环岛出环 补线
    if (ImageFlag.image_element_rings_flag == 8)  // 大环
    {
//        Repair_Point_Xsite = 42;
        Repair_Point_Ysite = 7;
//        for (int Ysite = 40; Ysite > 8; Ysite--)
//        {
//            if (Pixle[Ysite][28] == 1 && Pixle[Ysite-1][28] == 0)
//            {
//                Repair_Point_Xsite = 42;
//                Repair_Point_Ysite = Ysite-1;
//                ImageStatus.OFFLine = Ysite + 1;  // 防止继续向上规划
//                break;
//            }
//        }
        for (int Ysite = 57; Ysite > Repair_Point_Ysite-3; Ysite--)         // 补线
        {
//            ImageDeal[Ysite].LeftBorder = (ImageDeal[58].LeftBorder - Repair_Point_Xsite) * (Ysite - 58) / (58 - Repair_Point_Ysite)  + ImageDeal[58].LeftBorder;
            //if(ImageDeal[Ysite].LeftBorder<3){ImageDeal[Ysite].LeftBorder = 3;}
            ImageDeal[Ysite].LeftBorder = ImageDeal[Ysite].RightBorder -  Half_Bend_Wide[Ysite];
            if(ImageDeal[Ysite].LeftBorder<3){ImageDeal[Ysite].LeftBorder = 3;}
            ImageDeal[Ysite].Center = (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;
        }
    }
        // 已出环 切内线
    if (ImageFlag.image_element_rings_flag == 9)
    {
        for (int Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + Half_Road_Wide[Ysite];
        }
    }
}

// 元素测试函数
void Element_Test(void) {
 

  if (    ImageStatus.Road_type != RightCirque
      && ImageStatus.Road_type != LeftCirque
    //  &&SystemData.Stop == 0
      )
  { 
          Element_Judgment_Left_Rings();           // 左环岛判断
          Element_Judgment_Right_Rings();          // 右环岛判断

  }
}

// 元素处理函数
void Element_Handle() {
  if (ImageFlag.image_element_rings == 1)
      Element_Handle_Left_Rings();
  else if(ImageFlag.image_element_rings == 2)
      Element_Handle_Right_Rings();

}


// 丢双线的时候 处理无边行的补线
static void RouteFilter(void) 
{
  for (Ysite = 58; Ysite >= (ImageStatus.OFFLine + 5);Ysite--)  // 从开始位置到停止位置 原58
  {
    if (ImageDeal[Ysite].IsLeftFind == 'W'&&ImageDeal[Ysite].IsRightFind == 'W'&&Ysite <= 45&&ImageDeal[Ysite - 1].IsLeftFind == 'W'&&ImageDeal[Ysite - 1].IsRightFind =='W')  // 当前行左右都无边，并且当前行45以内  滤波
    {
      ytemp = Ysite;
      while (ytemp >= (ImageStatus.OFFLine +5))     // 四个特性，-6效果差一些   原+5
      {
        ytemp--;
        if (  ImageDeal[ytemp].IsLeftFind == 'T'&&ImageDeal[ytemp].IsRightFind == 'T')  // 寻找双线都有的，找到与当前行的距离就不补了
        {
          DetR = (float)(ImageDeal[ytemp - 1].Center - ImageDeal[Ysite + 2].Center) /(float)(ytemp - 1 - Ysite - 2);          // 计算斜率
          int CenterTemp = ImageDeal[Ysite + 2].Center;
          int LineTemp = Ysite + 2;
          while (Ysite >= ytemp) 
          {
            ImageDeal[Ysite].Center =(int)(CenterTemp +DetR * (float)(Ysite - LineTemp));                                     // 按斜率补
            Ysite--;
          }
          break;
        }
      }
    }
    ImageDeal[Ysite].Center =(ImageDeal[Ysite - 1].Center + 2 * ImageDeal[Ysite].Center) /3;                                  // 做平滑应该比较缓和  周围三行取平均
  }
}

// 绘制边界线 用于调试
void DrawLine()
{
    uint8_t i, j;

    for (i = 0; i < 60; ++i)
    {
        for (j = 0; j < 80; ++j)
        {
          printArray[i][j] = 32;
        }
    }

    for (i = 59; i > ImageStatus.OFFLine; i--)
    {
      Pixle[i][ImageDeal[i].LeftBorder + 2] = 0;  // 移动两位便于观察
      Pixle[i][ImageDeal[i].RightBorder - 2] = 0;
      Pixle[i][ImageDeal[i].Center] = 0;

        // 将信息叠加到打印数组
      printArray[i][ImageDeal[i].LeftBorder + 2] = '0';
      printArray[i][ImageDeal[i].RightBorder - 2] = '0';
      printArray[i][ImageDeal[i].Center] = '0';
    }

    // 打印叠加后的二维数组
    for (i = 0; i < 60; ++i)
    {
      for (j = 0; j < 80; ++j)
      {
          std::cout << static_cast<char>(printArray[i][j]);
      }
      std::cout << std::endl;
  }
}

/*****************偏差按权重系数加权平均**********************/
void GetDet() 
{
  float DetTemp = 0;
  int TowPoint = 0;
  float SpeedGain = 0;
  float UnitAll = 0;

    if ((ImageStatus.Road_type == RightCirque ||ImageStatus.Road_type == LeftCirque)&&ImageStatus.CirqueOff == 'F')
    TowPoint = 30;                                                                      // 环岛前瞻

    else if (ImageStatus.Road_type == Straight)
    TowPoint = SystemData.straighet_towpoint;

  else if(ImageStatus.Road_type ==Cross_ture)
  {
      TowPoint=22;
  }
  else if(ImageFlag.image_element_rings_flag == 1
          || ImageFlag.image_element_rings_flag == 2)
  {
      TowPoint=30;
  }
else
    TowPoint = ImageStatus.TowPoint-SpeedGain;                                          // 速度越快前瞻越大

  if (TowPoint < ImageStatus.OFFLine)
    TowPoint = ImageStatus.OFFLine + 1;

  if (TowPoint >= 49)
    TowPoint = 49;

  if ((TowPoint - 5) >= ImageStatus.OFFLine) {                                          // 前瞻取设定前瞻和视觉距离 取较小值
    for (int Ysite = (TowPoint - 5); Ysite < TowPoint; Ysite++) {
      DetTemp = DetTemp + Weighting[TowPoint - Ysite - 1] * (ImageDeal[Ysite].Center);
      UnitAll = UnitAll + Weighting[TowPoint - Ysite - 1];
    }
    for (Ysite = (TowPoint + 5); Ysite > TowPoint; Ysite--) {
      DetTemp += Weighting[-TowPoint + Ysite - 1] * (ImageDeal[Ysite].Center);
      UnitAll += Weighting[-TowPoint + Ysite - 1];
    }
    DetTemp = (ImageDeal[TowPoint].Center + DetTemp) / (UnitAll + 1);

  } else if (TowPoint > ImageStatus.OFFLine) {
    for (Ysite = ImageStatus.OFFLine; Ysite < TowPoint; Ysite++) {
      DetTemp += Weighting[TowPoint - Ysite - 1] * (ImageDeal[Ysite].Center);
      UnitAll += Weighting[TowPoint - Ysite - 1];
    }
    for (Ysite = (TowPoint + TowPoint - ImageStatus.OFFLine); Ysite > TowPoint;
         Ysite--) {
      DetTemp += Weighting[-TowPoint + Ysite - 1] * (ImageDeal[Ysite].Center);
      UnitAll += Weighting[-TowPoint + Ysite - 1];
    }
    DetTemp = (ImageDeal[Ysite].Center + DetTemp) / (UnitAll + 1);
  } else if (ImageStatus.OFFLine < 49) 
  {
    for (Ysite = (ImageStatus.OFFLine + 3); Ysite > ImageStatus.OFFLine;
         Ysite--) {
      DetTemp += Weighting[-TowPoint + Ysite - 1] * (ImageDeal[Ysite].Center);
      UnitAll += Weighting[-TowPoint + Ysite - 1];
    }
    DetTemp = (ImageDeal[ImageStatus.OFFLine].Center + DetTemp) / (UnitAll + 1);

  } else
    DetTemp =ImageStatus.Det_True;                                                     // 如果出现OFFLine>50，那么保持上一次的偏差值

  ImageStatus.Det_True = DetTemp;                                                      // 这时的计算结果作为平均图像偏差

  ImageStatus.TowPoint_True = TowPoint;                                                // 这时的前瞻
}

float Det = 0;
// 图像处理主函数
void ImageProcess(void)
{
    static uint32_t image_debug_log_divider = 0;
#ifdef LQ_HAVE_OPENCV
    Image_Process();
    if (Gray_image.empty())
    {
        return;
    }
#else
    return;
#endif

    compressimage();          // 图像压缩 0.6ms
    ImageStatus.OFFLine = 2;  // 初值设定，实际会根据图像处理结果调整
    ImageStatus.WhiteLine = 0;
  for (Ysite = 59; Ysite >= ImageStatus.OFFLine; Ysite--) {
    ImageDeal[Ysite].IsLeftFind = 'F';
    ImageDeal[Ysite].IsRightFind = 'F';
    ImageDeal[Ysite].LeftBorder = 0;
    ImageDeal[Ysite].RightBorder = 79;
    ImageDeal[Ysite].LeftTemp = 0;
    ImageDeal[Ysite].RightTemp = 79;
//    ImageDeal[Ysite].Black_Wide_L = 39;
//    ImageDeal[Ysite].Black_Wide_R = 39;
//    ImageDeal[Ysite].BlackWide = 0;

    // g  5.12
    ImageDeal[Ysite].close_LeftBorder = 0;
    ImageDeal[Ysite].close_RightBorder = 79;
//    ImageDeal[Ysite].opp_LeftBorder = 0;
//    ImageDeal[Ysite].opp_RightBorder = 0;

  }                     // 边界和标志位初始化

 // K = Fit1_k(59-ImageStatus.OFFLine+10,10);

  Get01change_dajin();  // 图像二值化 2.7ms
  //tft180_show_string(0,90,"che");
  //if(SystemData.clrcle_num!=2)
 // { Pixle_Filter();}       // 腐蚀 1.7ms

  DrawLinesFirst();     // 绘制底边 30us
  DrawLinesProcess();   // 得到向上边界 8us
        
  Search_Border_OTSU(Pixle, LCDH, LCDW, LCDH - 2);// 58行位置

  /***元素识别*****/
  Element_Test();                   // 5us
  /***元素识别*****/
  DrawExtensionLine();
  RouteFilter();        // 路径滤波平滑 2us
  /***元素处理*****/
  //  Element_Handle();  // 3us
  /***元素处理*****/
  // Stop_Test();           // 过桥保护   出环后  确保已经过环
  GetDet();               // 获取动态前瞻 并计算图像偏差 3us
     // Menu_key_set();

  //Stop_Test3();//

//  ImageStatus.Foresight = ((((ImageDeal[ImageStatus.OFFLine + 1].Center) +
//                             (ImageDeal[ImageStatus.OFFLine + 2].Center) +
//                             (ImageDeal[ImageStatus.OFFLine + 3].Center)) /3) -40);
//
//  ImageStatus.Det_all = (ImageStatus.Foresight + 40) - ImageDeal[54].Center;
//  ImageStatus.Det_all_k =(float)(ImageStatus.Det_all) / (ImageStatus.OFFLine + 2 - 54) * 30;
//  ImageStatus.Foresight = abs(ImageStatus.Foresight);
// std::this_maad::sleep_for(std::chrono::milliseconds(10));
}