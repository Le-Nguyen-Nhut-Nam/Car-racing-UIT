#include <stdio.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/camera.h>
#include <webots/supervisor.h>
#define TIME_STEP 32

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

// IR Ground Sensors
WbDeviceTag gs[NB_GROUND_SENS];

// LEDs 
WbDeviceTag led[NB_LEDS];

// Motors
WbDeviceTag left_motor, right_motor;

/* Phần code được phép chỉnh sửa cho phù hợp */
// Định nghĩa các tín hiệu của xe
#define NOP  -1
#define MID   0
#define LEFT  1
#define RIGHT 2
#define FULL_SIGNAL 3
#define BLANK_SIGNAL 4

// Điểu chỉnh tốc độ phù hợp
#define MAX_SPEED 20

// Khai báo biến
unsigned short threshold[NB_GROUND_SENS] = {350, 350, 350, 350, 350, 350, 350, 350};
unsigned char filted = 0b00000000;
unsigned char preFilted = 0b00000000;
int pos = MID;
unsigned char intersectionDirect = LEFT;

// Biến lưu giá trị tỉ lệ tốc độ của động cơ
float left_ratio = 0.0;
float right_ratio = 0.0;

//Hàm đọc giá trị sensors
unsigned char ReadSensors(void){
  filted = 0b00000000;
  unsigned short gs_value[NB_GROUND_SENS] = {0, 0, 0, 0, 0, 0, 0, 0};
  
  for(int i=0; i<NB_GROUND_SENS; i++){
    gs_value[i] = wb_distance_sensor_get_value(gs[i]);
    // So sánh giá trị gs_value với threshold -> chuyển đổi sang nhị phân
    if (gs_value[i] > threshold[i])
      filted |= (0x01 << (NB_GROUND_SENS - i - 1));
  }
  // Tắt comment để gỡ lỗi
  /*for(int i=0; i<NB_GROUND_SENS; i++){printf("%d\t", gs_value[i]);}*/
  return filted;
}

// Phần code điều khiển xe
int DeterminePosition(unsigned char filted) {
  switch (filted) {
    case 0b00010000:
    case 0b00001000:
    case 0b00011000:
    case 0b00111000:
    case 0b00011100:
        return MID;
    case 0b00110000:
    case 0b01100000:
    case 0b11000000:
    case 0b10000000:
      return RIGHT;
    case 0b00001100:
    case 0b00000110:
    case 0b00000011:
    case 0b00000001:
      return LEFT;
    case 0b00000000:
      return BLANK_SIGNAL;
    case 0b11111111:
    case 0b01111111:
    case 0b11111110:
    case 0b01111110:
      return FULL_SIGNAL;
    default: return NOP;
  }
}

void GoStraight(unsigned char filted) {
  switch (filted) {
    case 0b00001000:
      left_ratio = 1.0;
      right_ratio = 0.9;
      break;
    case 0b00010000:
      left_ratio = 0.9;
      right_ratio = 1.0;
      break;
    case 0b00011000:
      left_ratio = right_ratio = 1;
    default:
     left_ratio=right_ratio=1;
      break;
  }
}

void TurnLeft(unsigned char filted) {
  switch (filted) {
    case 0b00110000:
      left_ratio = 0.7;
      right_ratio = 0.9;
      break;
    case 0b01100000:
      left_ratio = 0.55;
      right_ratio = 0.8;
      break;
    case 0b11000000:
      left_ratio = 0.3;
      right_ratio = 0.7;
      break;
    case 0b10000000:
      left_ratio = 0.2;
      right_ratio = 0.7;
      break;
  }
}

void TurnRight(unsigned char filted) {
  switch (filted) {
 
    case 0b00001100:
      left_ratio = 0.9;
      right_ratio = 0.7;
      break;
    case 0b00000110:
      left_ratio = 0.8;
      right_ratio = 0.55;
      break;
    case 0b00000011:
      left_ratio = 0.7;
      right_ratio = 0.3;
      break;
    case 0b00000001:
      left_ratio = 0.7;
      right_ratio = 0.2;
      break;
  }
  }
void Blank(unsigned char filted){
  switch(filted){
  case 0b00000000:
    left_ratio=1.0;
    right_ratio=1.0;
  default:
    left_ratio=1.0;
    right_ratio=1.0;
  }
}


void TurnLeftCorner(unsigned char filted) {
 switch(filted){
   case 0b11111000:
     left_ratio=0.0;
     right_ratio=1.0;
   case 0b11110000:
     left_ratio=0.0;
     right_ratio=1.0;
   case 0b11100000:
     left_ratio=0.0;
     right_ratio=1.0;
   case 0b11000000:
     left_ratio=0.0;
     right_ratio=1.0;
   case 0b10000000:
     left_ratio=0.0;
     right_ratio=1.0;
   default:
     left_ratio=0;
     right_ratio=1;  
}
}
void TurnRightCorner(unsigned char filted) {
  switch(filted){
    case 0b00011111:
      left_ratio=1.0;
      right_ratio=0.0;
    case 0b00001111:
      left_ratio=1.0;
      right_ratio=0.0;
    case 0b00000111:
      left_ratio=1.0;
      right_ratio=0.0;
    case 0b00000011:
      left_ratio=1.0;
      right_ratio=0.0;
    default:
      left_ratio=1.0;
     right_ratio=0.0;
    break;
}
}
void TurnLeftIntersection(unsigned char filted)
{
  switch(filted)
  {
    case 0b11111000:
      left_ratio=0.0;
      right_ratio=1.0;
    case 0b11110000:
      left_ratio=0.0;
      right_ratio=1.0;
    case 0b11100000:
      left_ratio=0.0;
      right_ratio=1.0;
    case 0b11000000:
      left_ratio=0.0;
      right_ratio=1.0;
    case 0b10000000:
      left_ratio=0.0;
      right_ratio=1.0;
    default:
      left_ratio=0.0;
      right_ratio=1.0;
    
  }
}
void TurnRightIntersection(unsigned char filted)
{
  switch(filted)
  {
    case 0b00011111:
      left_ratio=1.0;
      right_ratio=0.0;
    case 0b00001111:
      left_ratio=1.0;
      right_ratio=0.0;
    case 0b00000111:
      left_ratio=1.0;
      right_ratio=0.0;
    case 0b00000011:
      left_ratio=1.0;
      right_ratio=0.0;
    case 0b00000001:
      left_ratio=1.0;
      right_ratio=0.0;
    default:
      left_ratio=1.0;
      right_ratio=0.0;
    
  }
}

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
    filted = ReadSensors();
    // Xác định tín hiệu của line đường
    pos = DeterminePosition(filted);
    // Xác định hướng rẽ của ngã tư
    if (filted == 0b11110000 || filted == 0b11111000)
      intersectionDirect = LEFT;
    else if (filted == 0b00001111 || filted == 0b00011111)
      intersectionDirect = RIGHT;
    // In ra màn hình giá trị của filted ở dạng nhị phân
    printf("Position: "BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(filted));
    printf("\n");
    
    // Điều khiển xe
    if (pos == MID) {
      GoStraight(filted);
    }
    else if (pos == RIGHT) {
      TurnLeft(filted);
    }
    else if (pos == LEFT) {
      TurnRight(filted);
    }
    else if (pos == BLANK_SIGNAL) {
      if (preFilted == 0b11000000 || preFilted == 0b11100000 || preFilted == 0b11110000 || preFilted == 0b11111000)
      {
        TurnLeftCorner(filted);
        
        printf("Turn left corner\n");
      }
      else if (preFilted == 0b00000011 || preFilted == 0b00000111 || preFilted == 0b00001111 || preFilted == 0b00011111)
      {
        TurnRightCorner(filted);
        printf("Turn right corner\n");
      }
      else if (preFilted == 0b00011000 || preFilted == 0b00010000 || preFilted == 0b00001000)
      {
        Blank(filted);
        // Điều khiển xe mất line
        printf("Lost of road markings\n");
      }
    }
    else if (pos == FULL_SIGNAL)
    {
      if (intersectionDirect == RIGHT) {
        TurnRightIntersection(filted);
        // Điều khiển xe rẽ ngã tư trái
        printf("RIGHT Intersection\n");
      }
      else if (intersectionDirect == LEFT) {
        TurnLeftIntersection(filted);
        // Điều khiển xe rẽ ngã tư trái
        printf("LEFT Intersection\n");
      }
    }
    
    preFilted = filted;
     
    // Giới hạn tỉ lệ tốc độ của động cơ
    constrain(&left_ratio, 0, 1);
    constrain(&right_ratio, 0, 1);   
    // Điều chỉnh tốc độ động cơ
    wb_motor_set_velocity(left_motor, left_ratio * MAX_SPEED);
    wb_motor_set_velocity(right_motor, right_ratio * MAX_SPEED);
  };

  wb_robot_cleanup();

  return 0;
}

