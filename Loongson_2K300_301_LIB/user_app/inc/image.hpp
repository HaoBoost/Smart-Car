#ifndef IMAGE_HPP
#define IMAGE_HPP

#include <cstdint>

#ifdef LQ_HAVE_OPENCV
#include <opencv2/opencv.hpp>
#endif

using uint8 = uint8_t;

#define LCDH 60
#define LCDW 80
#define LimitL(L) (L = ((L < 1) ? 1 : L))
#define LimitH(H) (H = ((H > 78) ? 78 : H))
#define ImageSensorMid 39

extern int ImageScanInterval;
extern int ImageScanInterval_Cross;
extern int border_point;
extern int top_point;
extern uint8_t Image_Use[LCDH][LCDW];
extern uint8_t Pixle[LCDH][LCDW];
extern uint8_t printArray[LCDH][LCDW];

#ifdef LQ_HAVE_OPENCV
extern cv::Mat First_image;
extern cv::Mat Cut_image;
extern cv::Mat Resized_image;
extern cv::Mat Gray_image;
extern cv::Mat Bin_Frame;
#endif

typedef struct {
  int point;
  uint8_t type;
} JumpPointtypedef;

typedef struct {
  float nowspeed;
  int expectspeed;
  int motor_duty;
  float Length;
  int Circle_OUT_th;
  int MinSpeed;
  int MaxSpeed;
  float expect_True_speed;
  int straight_speed;
} SpeedDatatypedef;

typedef struct {
  uint8_t IsRightFind;
  uint8_t IsLeftFind;
  uint8_t isBlackFind;
  int Wide;
  int LeftBorder;
  int RightBorder;
  int close_LeftBorder;
  int close_RightBorder;
  int opp_LeftBorder;
  int opp_RightBorder;
  int Center;
  int RightTemp;
  int LeftTemp;
  int CenterTemp;
  int Black_Point;
  int LeftBoundary_First;
  int RightBoundary_First;
  int LeftBoundary;
  int RightBoundary;
} ImageDealDatatypedef;

typedef enum {
  Normol,
  Straight,
  Cross,
  Ramp,
  LeftCirque,
  RightCirque,
  Forkin,
  Forkout,
  Barn_out,
  Barn_in,
  Cross_ture,
} RoadType_e;

typedef struct {
  int TowPoint;
  int TowPointAdjust_v;
  int TowPoint_True;
  int TowPoint_Gain;
  int TowPoint_Offset_Max;
  int TowPoint_Offset_Min;
  int Det_True;
  int Det_all;
  float Det_all_k;
  uint8_t Threshold;
  uint32_t Threshold_static;
  uint8_t Threshold_detach;
  uint8_t MiddleLine;
  int Foresight;
  uint8_t Left_Line;
  uint8_t Right_Line;
  uint8_t OFFLine;
  uint8_t WhiteLine;
  float ExpectCur;
  float White_Ritio;
  int Black_Pro_ALL;
  float Piriod_P;
  float MU_P;
  RoadType_e Road_type;
  uint8_t IsCinqueOutIn;
  uint8_t CirquePass;
  uint8_t CirqueOut;
  uint8_t CirqueOff;
  int16_t WhiteLine_L;
  int16_t WhiteLine_R;
  int16_t OFFLineBoundary;
  int Pass_Lenth;
  int Cirque1lenth;
  int Cirque2lenth;
  int Out_Lenth;
  int Fork_Out_Len;
  int Dowm_lenth;
  int Cross_Lenth;
  int Cross_ture_lenth;
  int Sita;
  int pansancha_Lenth;
  int Barn_Flag;
  int Barn_Lenth;
  int sanchaju;
  int Stop_lenth;
  int Ramp_lenth;
  int variance;
  int straight_acc;
  int variance_acc;
  int ramptestlenth;
  int rukuwait_lenth;
  int rukuwait_flag;
  int newblue_flag;
} ImageStatustypedef;

typedef struct {
  uint8_t SteerOK;
  uint8_t CameraOK;
  uint8_t OldCameraOK;
  uint8_t MotorOK;
  uint8_t Point;
  uint8_t UpdataOK;
  uint8_t Stop;
  uint8_t GO_OK;
  int Model;
  int OutCicle_line;
  int L_T_R_W;
  int Circleoff_offline;
  int CircleP;
  int fork_start_lenth;
  int fork_off_lenth;
  int circle_off_lenth;
  int circles_pl;
  int circles_pr;
  int circles_off_lenth;
  int circlem_pl;
  int circlem_pr;
  int circlem_off_lenth;
  int circlel_pl;
  int circlel_pr;
  int circlel_off_lenth;
  int clrcle_priority[3];
  int clrcle_num;
  int circle_kin;
  float circle_kout;
  int circle_max_ang;
  float straight_p;
  float straight_d;
  int straighet_towpoint;
  int debug_lenth;
  int exp_time;
  int mtv_lroffset;
  int mtv_gain;
  int ramp_lenth_start;
  int fork_lenth_start;
  int barn_lenth;
  int outbent_acc;
  int rounds;
  int speed_per_round;
  SpeedDatatypedef SpeedData;
} SystemDatatypdef;

typedef struct {
  int16_t image_element_rings;
  int16_t ring_big_small;
  int16_t image_element_rings_flag;
  int16_t straight_long;
} ImageFlagtypedef;

extern ImageStatustypedef ImageStatus;
extern SystemDatatypdef SystemData;
extern ImageDealDatatypedef ImageDeal[LCDH];
extern ImageFlagtypedef ImageFlag;
extern ImageStatustypedef ImageData;

float my_abs(float x);
void Data_Settings(void);
void ImageProcess(void);
void DrawLine(void);
void Element_Test(void);
void Element_Handle(void);
void cleanup(void);

#endif
