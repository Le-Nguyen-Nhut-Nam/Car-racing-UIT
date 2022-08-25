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
#define BLANK_SIGNAL  0
#define MID   1
#define LEFT  2
#define RIGHT 3
#define FULL_SIGNAL 4
#define CURVE 5
#define OUT_SPECIAL_SIGNAL 6

#define MAX_SPEED 80



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
float left_ratio = 0.0;
float right_ratio = 0.0;

float PreviousLeftRatio=0.0;
float PreviousRightRatio=0.0;

//Rẽ ngã tư trái trả về 1, Rẽ ngã tư phải trả về 2, Tín hiệu bắt đầu vòng tròn trả về 3


// Xác định kiểu thử thách: 1 là thử thách đơn giản, bao gồm chạy thẳng, chạy cong, chạy không có tín hiệu
// 2,3 là Rẽ ngã tư trái, ngã tư phải 
// 4 là Vòng tròn
short Determination_Type=1;
// Danh sách các flag cần thiết cho sử lý
short FlagInCircle=0;
short FlagOutCircle=0;
short FlagIntersection=0;
short FlagBlankSignal=0;

// Hàm điều kiện

// Hàm xác định tín hiệu
short Determine_Position_Normal()
{
  if(LeftSum < 100 && MidSum < 100 && RightSum < 100)
  {
  Determination_Type=1;
  return BLANK_SIGNAL; // trả về 0
  }
  
  else if (LeftSum<200 && MidSum > 350 && RightSum<200)
  {
  Determination_Type=1;
  return MID; // TRẢ VỀ 1
  }
  
  else if(LeftSum>1000 && MidSum >300 && RightSum < 200)
  {
 
  Determination_Type=2;
  return LEFT; // TRẢ VỀ 2  
  }
  
  else if(LeftSum<200 && MidSum>300 && RightSum>1000)
  {
  Determination_Type=3;
  return RIGHT; //TRẢ VỀ 3 
  }
  
  else if(LeftSum>400 && MidSum>800 && RightSum>400)
  {
 
  Determination_Type=4;
  return FULL_SIGNAL; // TRẢ VỀ 4
  }
  
  else
  {
  Determination_Type=1;
  return CURVE;
  }
}
short Determination_Position_Circle()
{
  if (LeftSum<200 && MidSum > 350 && RightSum<200)
  return MID; // TRẢ VỀ 1
  
  else if(LeftSum>600 && MidSum<100 && RightSum>600)
  return OUT_SPECIAL_SIGNAL; // Trả về 6
  
  else if(LeftSum>400 && MidSum>800 && RightSum>400)
  return FULL_SIGNAL;
  
  else
  return CURVE;
}

short Determine_Position_Left_Intersection()
{
  if(LeftSum < 100 && MidSum < 100 && RightSum < 100)
  {
  return BLANK_SIGNAL; // trả về 0
  }
  
  else if (LeftSum<200 && MidSum > 350 && RightSum<200)
  {
  return MID; // TRẢ VỀ 1
  }
  
  else if(LeftSum>1000 && MidSum >300 && RightSum < 200)
  {
  return LEFT; // TRẢ VỀ 2  
  }
  
  else if(LeftSum<200 && MidSum>300 && RightSum>1000)
  {
  return RIGHT; //TRẢ VỀ 3 
  }
  
  else if(LeftSum>400 && MidSum>800 && RightSum>400)
  {
  return FULL_SIGNAL; // TRẢ VỀ 4
  }  
  else
  {
  return CURVE;
  }
}
short Determine_Position_Right_Intersection()
{
  if (LeftSum<200 && MidSum > 350 && RightSum<200)
  return MID;
  else if(LeftSum>400 && MidSum>800 && RightSum>400)
  return FULL_SIGNAL;
  else
  return OUT_SPECIAL_SIGNAL;
}
// Hàm điều khiển

void Normal_Control()
{
  switch(pos)
  {
  case MID: // Điều khiển chạy thẳng
  left_ratio=1 - (LeftSum)/945.0;
  right_ratio=1 - (RightSum)/945.0;
  break;
  
  case BLANK_SIGNAL: // điều khiển chạy không tín hiệu
  left_ratio=1.0;
  right_ratio=1.0;
  break;
  
  case CURVE: // điều khiển chạy đường cong
  left_ratio=1 - (LeftSum)/945.0;
  right_ratio=1 - (RightSum)/945.0;
  break;
  
  case FULL_SIGNAL:
    left_ratio=1.0;
    right_ratio=1.0;
  break;
  }
}

void CircleControl()
{
  switch(pos)
  {
    case MID:
    left_ratio=0.5;
    right_ratio=0.5; 
    
    if(FlagOutCircle==1)
          {
            FlagOutCircle=0;
            Determination_Type=1;
          }
    break;
    
    case FULL_SIGNAL:       
          left_ratio=1.0;
          right_ratio=0.0;  
          
          if(FlagInCircle==0)
            FlagInCircle=1;                         
    break;
    
    case CURVE:
      left_ratio=1 - (LeftSum)/945.0;
      right_ratio=1 - (RightSum)/945.0;
    break;
    
    case OUT_SPECIAL_SIGNAL:   
      left_ratio=1.0;
      right_ratio=0.0;
      
      if(FlagInCircle==1)  
      {
        FlagOutCircle=1;
        FlagInCircle=0;
      }
     break;
  }
}
void LeftIntersectionControl()
{
  switch(pos)
  {
  case MID:
  left_ratio=(1 - (LeftSum)/945.0)*0.5;
  right_ratio=(1 - (RightSum)/945.0)*0.5;
  if(FlagIntersection==1)
  Determination_Type=1;
  if(FlagBlankSignal==1)
  Determination_Type=1;
  break;
  
  case LEFT:
  left_ratio=right_ratio=0.5;
  break;
  
  case RIGHT:
  left_ratio=right_ratio=0.5;
  break;
  
  case BLANK_SIGNAL:
  left_ratio=PreviousLeftRatio;
  right_ratio=PreviousRightRatio;
  FlagBlankSignal=1;
  break;
  
  case FULL_SIGNAL:
  left_ratio=0.0;
  right_ratio=0.9;
  FlagIntersection=1;
  break;
  
  case CURVE:
  if(FlagIntersection==1)
  {
  left_ratio=0.0;
  right_ratio=0.9;
  }
  else
  {
  left_ratio=1 - (LeftSum)/945.0;
  right_ratio=1 - (RightSum)/945.0;
  }
  break;
  
  case OUT_SPECIAL_SIGNAL:
  left_ratio=0.0;
  right_ratio=0.9;
  break;
  }
}
void RightIntersectionControl()
{
   switch(pos)
  {
  case MID:
  left_ratio=(1 - (LeftSum)/945.0)*0.5;
  right_ratio=(1 - (RightSum)/945.0)*0.5;
  if(FlagIntersection==1)
  Determination_Type=1;
  if(FlagBlankSignal==1)
  Determination_Type=1;
  break;
  
  
  case RIGHT:
  left_ratio=right_ratio=0.5;
  break;
  
  case BLANK_SIGNAL:
  left_ratio=PreviousLeftRatio;
  right_ratio=PreviousRightRatio;
  FlagBlankSignal=1;
  break;
  
  case FULL_SIGNAL:
  left_ratio=1.0;
  right_ratio=0.0;
  FlagIntersection=1;
  break;
  
  case CURVE:
  if(FlagIntersection==1)
  {
  left_ratio=1.0;
  right_ratio=0.0;
  }
  else
  {
  left_ratio=1 - (LeftSum)/945.0;
  right_ratio=1 - (RightSum)/945.0;
  }
  break;
  
  case OUT_SPECIAL_SIGNAL:
  left_ratio=1.0;
  right_ratio=0.0;
  break;
  }
}
// Hàm điều khiển
void Control()
{
  switch(Determination_Type)
  {
  case 1:
  pos=Determine_Position_Normal();
  Normal_Control();
  break;
  
  case 2:
  pos=Determine_Position_Left_Intersection();
  LeftIntersectionControl();
  break;
  
  case 3:
  pos=Determine_Position_Right_Intersection();
  RightIntersectionControl();
  break;
  
  case 4:
  pos=Determination_Position_Circle();
  CircleControl();
  break;
  }
}


// In cac du lieu can thiet
void PrintData()
{
    printf("\n");
    printf("Left ratio: %f", left_ratio);
    printf("                           ");
    printf("Right ratio: %f", right_ratio);
    printf("\n");
    printf("Left Sum: %f",LeftSum);
    printf("                           ");
    printf("Mid Sum: %f",MidSum);
    printf("                           ");
    printf("Right Sum: %f",RightSum);
    printf("\n");
    printf("Position: %d", pos);
    printf("            ");
    printf("Mode: %d", Determination_Type);
    printf("\n");
};
// Tính tổng từng bên.
void CalculateSum(int i, short Value)
{
  switch(i)
  {
  case 0:
  LeftSum = LeftSum + Value - 120;
  break;
  case 1:
  LeftSum = LeftSum + Value - 120;
  break;
  case 2:
  LeftSum = LeftSum + Value - 120;
  break;
  
  case 3:
  MidSum = MidSum + Value - 120;
  break;
  case 4:
  MidSum = MidSum + Value - 120;
  break;
  
  case 5:
  RightSum = RightSum + Value - 120;
  break;
  case 6:
  RightSum = RightSum + Value - 120;
  break;
  case 7:
  RightSum = RightSum + Value - 120;
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

// Xác định các vị trị lệch và các tín hiệu bắt được của xe

void constrain(float *value, float min, float max) {
  if (*value > max) *value = max;
  if (*value < min) *value = min;
}

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
    
    // Đọc giá trị của sensors
    ReadSensors();
    PrintData();
    Control();  
    printf("\n");
 
    
    // Giới hạn tỉ lệ tốc độ của động cơ
    PreviousLeftRatio=left_ratio;
    PreviousRightRatio=right_ratio;
    
    constrain(&left_ratio, 0, 1);
    constrain(&right_ratio, 0, 1);   
    // Điều chỉnh tốc độ động cơ
    wb_motor_set_velocity(left_motor, left_ratio * MAX_SPEED);
    wb_motor_set_velocity(right_motor, right_ratio * MAX_SPEED);
    
    LeftSum=0;
    RightSum=0;
    MidSum=0;
  };

  wb_robot_cleanup();

  return 0;
}

