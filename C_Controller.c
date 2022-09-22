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
#define MAX_SPEED_1 50
#define MAX_SPEED_2 20


// IR Ground Sensors
WbDeviceTag gs[NB_GROUND_SENS];

// LEDs 
WbDeviceTag led[NB_LEDS];

// Motors
WbDeviceTag left_motor, right_motor;

// Khai báo biến
unsigned short gs_value[NB_GROUND_SENS] = {350, 350, 350, 350, 350, 350, 350, 350};
short pos = MID;
short SmallestValue,SecondSmallestValue=0;
short threshold=0;
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

short FlagIntoCircle=0; // flag này chỉ dành riêng cho vòng tròn
short ConditionJump=-1; // Điều kiện nhảy để xử lí

// Danh sách các hàm dùng để xác định tín hiệu

  bool IsFullSignal()
  {
    if(LeftSum>=5 && MidSum>=2 && RightSum>=5)
      return true;
    else
      return false;
  }
  
  bool IsLeftHalfFullSignal()
  {  
    if(LeftSum>=5 && MidSum>=1)
      return true;
    else
      return false;
  }
  
  bool IsRightHalfFullSignal()
  { 
    if(MidSum >=1 && RightSum>=5)
      return true;
    else
      return false;
  }
  bool IsOutSignal()
  {
    if((LeftSum>=1 && MidSum >=2 && RightSum>=2) ||
       (LeftSum==0 && MidSum>1 && RightSum>=3))
      return true;
    else
      return false;
  }
  bool IsBlankSignal()
  {
  if(LeftSum==0 && MidSum==0 && RightSum==0)
    return true;
  else
    return false;
  }
  
  bool IsMidSignal()
  {
  if(LeftSum==0 && MidSum>=2 && RightSum==0)
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
  

// Hàm xác định tín hiệu
short DetectChallenge()
{

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
  else if(LeftSpeed > 70)
    LeftSpeed=70;
  if(RightSpeed<0)
    RightSpeed=0;
  else if(RightSpeed>70)
    RightSpeed=70;
}

void ControlOutLineZone()
{

  if(PreviousLeftSpeed>PreviousRightSpeed)
  {
    LeftSpeed=30;
    RightSpeed=0;
  }
  else
  {
    LeftSpeed=0;
    RightSpeed=30;
  }
  
}

void ControlNormalZone() // không cần sử dụng đạo hàm vì xử lí hiện tại đã ổn định
{                        
  LeftSpeed=MAX_SPEED_1 - 12*LeftSum ;
  RightSpeed=MAX_SPEED_1 - 12*RightSum ;  
}
void ControlCircleZone()
{	
switch (ConditionJump)
  {
case 0:
  LeftSpeed = MAX_SPEED_2 - 8 * LeftSum;
  RightSpeed = MAX_SPEED_2 - 8 * RightSum;
  break;
case 1:
  LeftSpeed = MAX_SPEED_2 - 8 * LeftSum;
  RightSpeed = MAX_SPEED_2 - 8 * RightSum;
  break;
case 2:
  LeftSpeed = MAX_SPEED_2;
  RightSpeed = 0;
  break;
case 3:
  LeftSpeed = MAX_SPEED_2 - 8 * LeftSum;
  RightSpeed = MAX_SPEED_2 - 8 * RightSum;
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
    printf("Left Speed: %f",LeftSpeed);
    printf("              ");
    printf("Right Speed: %f", RightSpeed);
      printf("\n");
    printf("Flag Prepare Challenge: %d",FlagPrepareChallenge);
      printf("\n");
    printf("Flag Track Into Challenge: %d",FlagTrackIntoChallenge);
      printf("\n");
    printf("Flag Slow Down: %d", FlagSlowDown);
      printf("\n");
    printf("Flag In Challenge: %d", FlagInChallenge);
      printf("\n");
    printf("Flag Out Challenge: %d", FlagOutChallenge);
      printf("\n");
    printf("Flag Into Circle: %d", FlagIntoCircle);
      printf("\n");
    printf("Condition Jump: %d", ConditionJump);
      printf("\n");
};
// Tính tổng từng bên.
void CalculateSum()
{
  for(int i=0;i<NB_GROUND_SENS;i++)
  {
  switch(i)
  {
  case 0:
  LeftSum =  LeftSum +gs_value[i];
  break;
  case 1:
  LeftSum =  LeftSum +gs_value[i];
  break;
  case 2:
  LeftSum =  LeftSum +gs_value[i];
  break;
  
  case 3:
  MidSum =  MidSum + gs_value[i];
  break;
  case 4:
  MidSum = MidSum +  gs_value[i];;
  break;
  
  case 5:
  RightSum = RightSum +gs_value[i];
  break;
  case 6:
  RightSum =  RightSum +gs_value[i];
  break;
  case 7:
  RightSum = RightSum +gs_value[i];
 
  break;
  }
  }
}
void FindSmallestValue(int CurrentValue)
{
    if(SmallestValue>=CurrentValue)
    {
      SmallestValue=CurrentValue;
    }
}
void GetThreshold()
{
  threshold=SmallestValue * 1.25;
}
void SetValue()
{
printf("Hold value: %d ",threshold);
printf("\n");
printf("Smallest: %d",SmallestValue);
printf("\n");

  for(int i=0;i<NB_GROUND_SENS;i++)
  {
  if(gs_value[i] < threshold)
    {
  switch(i)
      {
  case 0:
  gs_value[i]=3;
  break;
  case 1:
  gs_value[i]=2;
  break;
  case 2:
  gs_value[i]=1;
  break;
  
  case 3:
  gs_value[i]=1;
  break;
  case 4:
  gs_value[i]=1;
  break;
  
  case 5:
  gs_value[i]=1;
  break;
  case 6:
  gs_value[i]=2;
  break;
  case 7:
  gs_value[i]=3;
  break;
      }
    }
  else
  gs_value[i]=0;
 }
}

//Hàm đọc giá trị sensors
void ReadSensors(void)
{
  SmallestValue=wb_distance_sensor_get_value(gs[0]);
  
  for(int i=0; i<NB_GROUND_SENS; i++)
  {
    gs_value[i] = wb_distance_sensor_get_value(gs[i]);
    FindSmallestValue(gs_value[i]);
  }
}
void PrintSensorInput()
{
  printf("Input sensor: \n");
  
  for(int i=0;i<NB_GROUND_SENS;i++)
  {
   printf("%d  ",gs_value[i]);
  }
  
  printf("\n");
}
void PrintSensorOutput()
{
  printf("Output sensor: \n");
  
  for(int i=0;i<NB_GROUND_SENS;i++)
  {
   printf("%d  ",gs_value[i]);
  }
  
  printf("\n");
}


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
   while(wb_robot_step(TIME_STEP) != -1)
    {
      if(wb_robot_get_time()>0.15) break;
    }
  for (;;) {
 
    // Run one simulation step
    wb_robot_step(TIME_STEP);
    
    // Đọc giá trị của sensors, điểu khiển và in dữ liệu
    ReadSensors();
    PrintSensorInput();
   
    GetThreshold();
    SetValue();
    
    PrintSensorOutput();
    CalculateSum();
  
    ClassifySignal();
    Control();  
    Constrains();
    PrintData();
    // Lấy các giá trị LeftSum,RightSum.LeftSpeed,RightSpeed
    SmallestValue=SecondSmallestValue=0;
    PreviousLeftEdge=CurrentLeftEdge;
    PreviousRightEdge=CurrentRightEdge;
      
    PreviousLeftSpeed=LeftSpeed;
    PreviousRightSpeed=RightSpeed;
    
    // Trả lại MIdSum, LeftSum, RightSum = 0
    MidSum=LeftSum=RightSum=0;
     wb_motor_set_velocity(left_motor, LeftSpeed);
    wb_motor_set_velocity(right_motor, RightSpeed);
    
    
  };

  wb_robot_cleanup();

  return 0;
}