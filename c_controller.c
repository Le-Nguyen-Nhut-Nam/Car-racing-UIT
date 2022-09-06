#include <stdio.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/camera.h>
#include <webots/supervisor.h>
#include <stdio.h>
#include <stdlib.h>

#define TIME_STEP 8

// Macro để định dạng byte sang nhị phân sử dụng trong hàm printf()
#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0') 

// 8 IR ground color sensors
#define NB_GROUND_SENS 8
#define NB_LEDS 5

// Định nghĩa các tín hiệu của xe
#define BLANK_SIGNAL  1
#define MID   2
#define LEFT  3
#define RIGHT 4
#define FULL_SIGNAL 5
#define OUT_SIGNAL 6
#define OTHER 7
#define MAX_SPEED_1 80
#define MAX_SPEED_2 40


// IR Ground Sensors
WbDeviceTag gs[NB_GROUND_SENS];

// LEDs 
WbDeviceTag led[NB_LEDS];

// Motors
WbDeviceTag left_motor, right_motor;

// Khai báo biến
unsigned short gs_value[NB_GROUND_SENS] = {350, 350, 350, 350, 350, 350, 350, 350};
short pos = MID;
float LeftSum,MidSum,RightSum=0;
float LeftSpeed=0;
float RightSpeed=0;

float PreviousLeftSpeed=0;
float PreviousRightSpeed=0;

float CurrentLeftEdge,CurrentRightEdge,PreviousLeftEdge,PreviousRightEdge;


// Danh sách các flag cần thiết cho sử lý
short FlagPrepareChallenge=-1; // chuẩn bị vào thử thách
short FlagTrackIntoChallenge=0; // con đường ngắn đi vào thử thách
short FlagSlowDown=0; // đi chậm lại
short FlagInChallenge=0; // Đi vào thử thách
short FlagOutChallenge=0; // đi ra thử thách
short FlagBlurSignal=0; // Flag tín hiệu mờ

short FlagIntoCircle=0; // flag này chỉ dành riêng cho vòng tròn
short ConditionJump=-1; // Điều kiện nhảy để xử lí

// Danh sách các hàm dùng để xác định tín hiệu

  bool IsFullSignal()
  {
    switch(FlagBlurSignal)
    {
    case 0:
    {
    if(LeftSum>850 && MidSum>600 && RightSum>850)
      return true;
    else
      return false;
    }
    break;
    printf("hello world");
    case 1:
    {
     if(LeftSum>500 && MidSum>300 && RightSum>500)
      return true;
    else
      return false;
      
    } 
    break;
    }
  }
  
  bool IsLeftHalfFullSignal()
  {
  switch(FlagBlurSignal)
  {
  case 0:
  {
    if(LeftSum>1100 && MidSum> 500)
      return true;
    else
      return false;
  }
  break;
  
  case 1:
  {
  if(LeftSum>500 && MidSum> 200)
      return true;
    else
      return false;
  }
  break;
  }
  }
  
  bool IsRightHalfFullSignal()
  {
  switch(FlagBlurSignal)
    {
    case 0:
      {
    if(MidSum > 500 && RightSum>1100)
      return true;
    else
      return false;
      }
  break;
    
    case 1:
    {
    if(MidSum > 300 && RightSum>600)
      return true;
    else
      return false;
    }
    break;
  
    }
  }
  bool IsOutSignal()
  {
    if((LeftSum>400 && MidSum > 800 && RightSum>400) ||
        (LeftSum>300 && MidSum > 600 && RightSum>300)
        || (LeftSum>600 && MidSum<100 && RightSum>600))
      return true;
    else
      return false;
  }
  bool IsBlankSignal()
  {
  if(LeftSum<100 && MidSum<100 && RightSum<100)
    return true;
  else
    return false;
  }
  
  bool IsMidSignal()
  {
  if(LeftSum<300 && MidSum>400 && RightSum<300)
    return true;
  else
    return false;
  }
  bool IsOutLine()
  {
  if(((PreviousLeftEdge - CurrentLeftEdge) > 300 ||
      (PreviousRightEdge - CurrentRightEdge)>300) &&
       IsBlankSignal())
    return true;
  else
    return false;
  }
  void IsBlurSignal()
  {
  if(gs_value[0] <100 || gs_value[6] < 100)
    FlagBlurSignal=1;
  else
    FlagBlurSignal=0;
  }

// Hàm xác định tín hiệu
short DetectChallenge()
{
IsBlurSignal();

  if(IsOutLine())
    return 0;
 else if(IsFullSignal()) // tín hiệu full signal
    return 1;
  else if(IsLeftHalfFullSignal()) // tín hiệu rẽ nữa trái
    return 2;
  else if(IsRightHalfFullSignal()) // tín hiệu rẽ nữa phải
    return 3;
  else return -1; // không có tín hiệu thử thách, đường cong của vòng tròn.
}
//Xác dịnh tín hiệu và flag dành cho ngã tư, cua vuông
short DetectOutLine()
{
  if(IsBlankSignal())
    return BLANK_SIGNAL;
  else if(IsMidSignal())
    return MID;
}
void SwitchConditionOutLine()
{
  pos=DetectOutLine();
  switch(ConditionJump)
  {
  case -1:
   ConditionJump = 0;
   FlagSlowDown = 1;
  break;
  
  case 0:
    if((pos==BLANK_SIGNAL && ConditionJump==0) || pos==MID)
      ConditionJump=1;
  break;
  
  case 1:
    if(pos==MID && ConditionJump==1)
    {
      ConditionJump=-1;
      FlagPrepareChallenge=-1;
      FlagSlowDown=1;
    }
  break;
  
  }
}
short DetectIntersectionSignal()
{
  
  if(IsBlankSignal())
    return BLANK_SIGNAL; // Tín hiệu trống.
  else if(IsMidSignal())
    return MID; // tín hiệu đã trở về đường chính
}

void SwitchConditionIntersection()
{
pos = DetectIntersectionSignal();
switch (ConditionJump)
{
 case -1:
    ConditionJump = 0;
    FlagSlowDown = 1;
    break;
 case 0:
    if(pos==BLANK_SIGNAL)
      ConditionJump=1;
    break;

 case 1:
    if (pos == MID && ConditionJump == 1)
    {
     ConditionJump = FlagPrepareChallenge= -1;
     FlagSlowDown = FlagTrackIntoChallenge = FlagInChallenge = FlagOutChallenge = 0;
    }
    break;
    }
}
// Xác  định tín hiệu và flag dành cho vòng tròn
short DetectCircleSignal()
{
   if(IsFullSignal())
     return FULL_SIGNAL; // tín hiệu vào vòng tròn
     
   else if(IsOutSignal())
     return OUT_SIGNAL; // tín hiệu ra vòng tròn
     
   else if(IsMidSignal())
     return MID; // tín hiệu đã trở về đường chính  
}
void SwitchConditionCircle()
{
  pos=DetectCircleSignal();
  
  switch(ConditionJump)
  {
    case -1:
    ConditionJump=0;
    FlagSlowDown=1;
    break;
    
    case 0:
    if(pos==MID && FlagSlowDown==1 && ConditionJump==0)
    {
      FlagTrackIntoChallenge=1;
      ConditionJump=1;
    }
    break;
    
    case 1:
    if(pos==FULL_SIGNAL && FlagTrackIntoChallenge==1 && ConditionJump==1)
      {
      ConditionJump=2;
      FlagInChallenge=1;
      }
    break;
    
    case 2:
    if(pos==MID && ConditionJump==2)
    {
    ConditionJump=3;
    FlagIntoCircle=1;
    }
    break;
    
    case 3:
    if(pos==OUT_SIGNAL && ConditionJump==3)
      ConditionJump=4;
    break;
    
    case 4:
    if(pos==MID && ConditionJump==4)
    {
    FlagPrepareChallenge=-1;
    FlagTrackIntoChallenge=FlagSlowDown=FlagInChallenge=FlagIntoCircle=FlagOutChallenge=0;
    ConditionJump=-1;
    }
    break;
    
  }
  
}

// Hàm xác định tín hiệu chính
void ClassifySignal()
{
  if(FlagPrepareChallenge == -1) // Kiểm tra xem có còn đang đi trên đường hay vào thư thách
  {
     FlagPrepareChallenge=DetectChallenge();   
  }
  
  else // Đã vào thử thách
  {
  
    switch(FlagPrepareChallenge)
    {
      case 0:
      SwitchConditionOutLine();
      break;
      
      case 1:
      SwitchConditionCircle();
      break;
      
      case 2:
      SwitchConditionIntersection();
      break;
      
      case 3:
      SwitchConditionIntersection();
      break;
    }
  }
}
// Giới hạn tốc độ xe
void Constrains()
{
  if(LeftSpeed<0)
    LeftSpeed=0;
  else if(LeftSpeed > 100)
    LeftSpeed=100;
  if(RightSpeed<0)
    RightSpeed=0;
  else if(RightSpeed>100)
    RightSpeed=100;
}

void ControlOutLineZone()
{
/*
  if(PreviousLeftSpeed>PreviousRightSpeed)
  {
    LeftSpeed=50;
    RightSpeed=0;
  }
  else
  {
    LeftSpeed=0;
    RightSpeed=50;
  }
  */
}

void ControlNormalZone() // không cần sử dụng đạo hàm vì xử lí hiện tại đã ổn định
{                        
  LeftSpeed=MAX_SPEED_1 - 0.15*LeftSum ;
  RightSpeed=MAX_SPEED_1 - 0.15*RightSum ;  
}
void ControlCircleZone()
{	
switch (ConditionJump)
  {
case 0:
  LeftSpeed = MAX_SPEED_2 - 0.09 * LeftSum;
  RightSpeed = MAX_SPEED_2 - 0.09 * RightSum;
  break;
case 1:
  LeftSpeed = MAX_SPEED_2 - 0.09 * LeftSum;
  RightSpeed = MAX_SPEED_2 - 0.09 * RightSum;
  break;
case 2:
  LeftSpeed = MAX_SPEED_2;
  RightSpeed = 0;
  break;
case 3:
  LeftSpeed = MAX_SPEED_2 - 0.09 * LeftSum;
  RightSpeed = MAX_SPEED_2 - 0.09 * RightSum;
  break;

case 4:
  LeftSpeed = MAX_SPEED_2;
  RightSpeed = 0;
  break;
  }
}
void ControlIntersectionZone()
{
  switch(FlagPrepareChallenge)
  {
    case 2:
    switch(ConditionJump)
    {   
      case 0:
      LeftSpeed = MAX_SPEED_2;
      RightSpeed = MAX_SPEED_2 ;
      break;

      case 1:
      LeftSpeed=0;
      RightSpeed=MAX_SPEED_2;
      break;      
    }
    break;
    case 3:
    switch(ConditionJump)
    {   
      case 0:
      LeftSpeed = MAX_SPEED_2;
      RightSpeed = MAX_SPEED_2 ;
      break;

      case 1:
      LeftSpeed=MAX_SPEED_2;
      RightSpeed=0;
      break;      
    }
    break;
  }
}
// Xác định các vị trị lệch và các tín hiệu bắt được của xe
void Control()
{
  switch(FlagPrepareChallenge)
  {
    case -1:
    ControlNormalZone();
    break;
    
    case 0:
    ControlOutLineZone();
    break;
    
    case 1:
    ControlCircleZone();
    break;
    
    case 2:
    ControlIntersectionZone();
    break;
    
    case 3:
    ControlIntersectionZone();
    break;
  }
}


// In cac du lieu can thiet
void PrintData()
{
    printf("\n");
     printf("Left Speed: %f", LeftSpeed);
     printf("                           ");
     printf("Right Speed: %f", RightSpeed);
     printf("\n");
     printf("Left Sum: %f",LeftSum);
     printf("                           ");
     printf("Mid Sum: %f",MidSum);
     printf("                           ");
     printf("Right Sum: %f",RightSum);
    
     printf("\n");
    // printf("Flag Prepare Challenge: %d",FlagPrepareChallenge);
    // printf("\n");
    // printf("Flag Slow Down: %d", FlagSlowDown);
    // printf("\n");
    // printf("Flag In Challenge: %d", FlagInChallenge);
    // printf("\n");
    // printf("Flag Out Challenge: %d", FlagOutChallenge);
    // printf("\n");
    // printf("Condition Jump: %d",ConditionJump);
    // printf("\n \n  ");
    return ;
};
// Tính tổng từng bên.
void CalculateSum(int i, short Value)
{
  switch(i)
  {
  case 0:
  LeftSum =  LeftSum +  Value - 130;
  CurrentLeftEdge=Value-130;
  break;
  case 1:
  LeftSum =  LeftSum + Value - 130;
  break;
  case 2:
  LeftSum =  LeftSum + Value - 130;
  break;
  
  case 3:
  MidSum =  MidSum + Value - 130;
  break;
  case 4:
  MidSum = MidSum +  Value - 130;
  break;
  
  case 5:
  RightSum = RightSum + Value - 130;
  break;
  case 6:
  RightSum =  RightSum + Value - 130;
  break;
  case 7:
  RightSum = RightSum + Value - 130;
  CurrentRightEdge=Value;
  break;
  }
}

//Hàm đọc giá trị sensors
void ReadSensors(void)
{
  
  for(int i=0; i<NB_GROUND_SENS; i++)
  {
    gs_value[i] = wb_distance_sensor_get_value(gs[i]);
    printf("%d ",gs_value[i]);
    
    CalculateSum(i,gs_value[i]);
  }
}

// Giới hạn tốc độ của bánh xe


/*
 * This is the main program.
 */
int main() 
{

  /* necessary to initialize webots stuff */
  wb_robot_init();

  /* get and enable the camera and accelerometer */
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, 64);

  /* initialization */
  char name[20];
  for (int i = 0; i < NB_GROUND_SENS; i++) {
    sprintf(name, "gs%d", i);
    gs[i] = wb_robot_get_device(name); /* ground sensors */
    wb_distance_sensor_enable(gs[i], TIME_STEP);
  }

  for (int i = 0; i < NB_LEDS; i++) {
    sprintf(name, "led%d", i);
    led[i] = wb_robot_get_device(name);
    wb_led_set(led[i], 1);
  }
  
  // motors
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  // Chương trình sẽ được lặp lại vô tận trong hàm for(;;)
  for (;;) {
    // Run one simulation step
    wb_robot_step(TIME_STEP);
    
    // Đọc giá trị của sensors, điểu khiển và in dữ liệu
    ReadSensors();
    ClassifySignal();
    Control();  
    Constrains();
    PrintData();
    // Lấy các giá trị LeftSum,RightSum.LeftSpeed,RightSpeed
    PreviousLeftEdge=CurrentLeftEdge;
    PreviousRightEdge=CurrentRightEdge;
      
    PreviousLeftSpeed=LeftSpeed;
    PreviousRightSpeed=RightSpeed;
    
    // Trả lại MIdSum, LeftSum, RightSum = 0
    MidSum=LeftSum=RightSum=0;
     wb_motor_set_velocity(left_motor, LeftSpeed);
    wb_motor_set_velocity(right_motor,RightSpeed);
    
    
  };

  wb_robot_cleanup();

  return 0;
}