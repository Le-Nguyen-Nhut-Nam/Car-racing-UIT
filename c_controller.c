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
#define MAX_SPEED_2 30


// IR Ground Sensors
WbDeviceTag gs[NB_GROUND_SENS];

// LEDs 
WbDeviceTag led[NB_LEDS];

// Motors
WbDeviceTag left_motor, right_motor;

// Khai báo biến
unsigned short gs_value[NB_GROUND_SENS] = {350, 350, 350, 350, 350, 350, 350, 350};
unsigned char filted = 0b00000000;
unsigned char preFilted = 0b00000000;
short pos = MID;
float LeftSum,MidSum,RightSum=0;
float LeftSpeed=0;
float RightSpeed=0;

float PreviousLeftSum,PreviousRightSum;


// Danh sách các flag cần thiết cho sử lý
short FlagPrepareChallenge=-1;
short FlagTrackIntoChallenge=0;
short FlagSlowDown=0;
short FlagInChallenge=0;
short FlagOutChallenge=0;

short FlagIntoCircle=0; // flag này chỉ dành riêng cho vòng tròn
short ConditionJump=-1;

// Hàm xác định tín hiệu
short DetectChallenge()
{
  if(LeftSum>850 && MidSum>600 && RightSum>850) // tín hiệu full signal
    return 0;
  else if(LeftSum>1000 && MidSum> 400) // tín hiệu rẽ nữa trái
    return 1;
  else if(MidSum > 400 && RightSum>1000) // tín hiệu rẽ nữa phải
    return 2;
  else return -1; // không có tín hiệu thử thách, đường cong của vòng tròn.
}
//Xác dịnh tín hiệu và flag dành cho ngã tư, cua vuông
short DetectIntersectionSignal()
{
  if(LeftSum>800 && MidSum>600 && RightSum>800) // tín hiệu full signal
    return FULL_SIGNAL;
  else if(LeftSum<100 && MidSum<100 && RightSum<100)
    return BLANK_SIGNAL; // Tín hiệu trống.
  else if(LeftSum<300 && MidSum>400 && RightSum<300)
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
      ConditionJump=2;
    else if (pos==MID && ConditionJump==0)
    {
     ConditionJump = 1;
     FlagTrackIntoChallenge = 1;
    }
    break;

 case 1:
    if (pos == FULL_SIGNAL && ConditionJump==1)  // Thoát khỏi thử thách
    {
     ConditionJump = 2;
     FlagInChallenge = FlagOutChallenge = 1;
    }
    break;
 case 2:
    if (pos == MID && ConditionJump == 2)
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
   if(LeftSum>800 && MidSum>600 && RightSum>800)
     return FULL_SIGNAL; // tín hiệu vào vòng tròn
     
   else if(LeftSum>400 && MidSum>400 && RightSum>800)
     return OUT_SIGNAL; // tín hiệu ra vòng tròn
     
   else if(LeftSum<300 && MidSum>400 && RightSum<300)
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
      SwitchConditionCircle();
      break;
      case 1:
      SwitchConditionIntersection();
      break;
      case 2:
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
  else if(LeftSpeed > 80)
    LeftSpeed=80;
  if(RightSpeed<0)
    RightSpeed=0;
  else if(RightSpeed>80)
    RightSpeed=80;
}

void ControlNormalZone() // không cần sử dụng đạo hàm vì xử lí hiện tại đã ổn định
{                        // việc thêm đạo hàm và tính phân gây phức tạp cho thuật toán
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
    case 1:
    switch(ConditionJump)
    {   
      case 0:
      LeftSpeed = MAX_SPEED_2 ;
      RightSpeed = MAX_SPEED_2;
      break;
      
      case 1:
      LeftSpeed = MAX_SPEED_2 - 0.09 * LeftSum;
      RightSpeed = MAX_SPEED_2 - 0.09 * RightSum;
      break;
      
      case 2:
      LeftSpeed=0;
      RightSpeed=MAX_SPEED_2;
      break;      
    }
    break;
    case 2:
    switch(ConditionJump)
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
    ControlCircleZone();
    break;
    
    case 1:
    ControlIntersectionZone();
    break;
    
    case 2:
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
    printf("Flag Prepare Challenge: %d",FlagPrepareChallenge);
    printf("\n");
    printf("Flag Slow Down: %d", FlagSlowDown);
    printf("\n");
    printf("Flag In Challenge: %d", FlagInChallenge);
    printf("\n");
    printf("Flag Out Challenge: %d", FlagOutChallenge);
    printf("\n");
    printf("Condition Jump: %d",ConditionJump);
    printf("\n \n  ");
     
};
// Tính tổng từng bên.
void CalculateSum(int i, short Value)
{
  switch(i)
  {
  case 0:
  LeftSum =  LeftSum +  Value - 130;
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
int main() {

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
    // Lấy các giá trị LeftSum,RightSum phía trước để tính đạo hàm
    PreviousLeftSum=LeftSum;
    PreviousRightSum=RightSum;
      
    // Giới hạn tốc độ
    
    // Trả lại MIdSum, LeftSum, RightSum = 0
    MidSum=LeftSum=RightSum=0;
    wb_motor_set_velocity(left_motor, LeftSpeed);
    wb_motor_set_velocity(right_motor,RightSpeed);
    
    
  };

  wb_robot_cleanup();

  return 0;
}