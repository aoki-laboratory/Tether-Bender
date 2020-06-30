//------------------------------------------------------------------//
//Supported MCU:   ESP32 (M5Stack)
//File Contents:   Tether Bender
//Version number:  Ver.1.0
//Date:            2020/02/29
//------------------------------------------------------------------//
 
//This program supports the following boards:
//* M5Stack(Grey version)
 
//Include
//------------------------------------------------------------------//
#include <stdint.h>
#include <M5Stack.h>
#include <mcp_can.h>
#include <WiFi.h>
#include <CircularBuffer.h>
#include <EEPROM.h>
#include "driver/pcnt.h"

//Define
//------------------------------------------------------------------//
#define TIMER_INTERRUPT 1 

#define CAN0_INT 15                             // Set INT to pin 15

#define HX711_DOUT_X0  34
#define HX711_SCLK_X0  0
#define HX711_DOUT_X1  35
#define HX711_SCLK_X1  16
#define HX711_DOUT_Y0  36
#define HX711_SCLK_Y0  17
#define HX711_DOUT_Y1  2
#define HX711_SCLK_Y1  5

#define AVERATING_BUFFER_SIZE 100
#define INITIALIZEING_SAMPLE 100
#define CALIBRATION_PERIOD 15

#define STABLE_TIME 1500                        // ms
#define LOG_TIME 500                            // ms

#define TORQUE_TOLERANCE_X 0.5
#define I_TORQUE_TOLERANCE_X 50
#define TORQUE_TOLERANCE_Y 3
#define I_TORQUE_TOLERANCE_Y 100
#define ANGLE_TOLERANCE 1.0

#define YAXIS_MAX_ANGLE 45
#define YAXIS_MIN_ANGLE -43

#define ANGLE_AVERATING 3
#define ANGLE_IGNORE_THRESHOLD 10

#define BufferRecords 64                    // 1Cycle Buffer Records 

//Global
//------------------------------------------------------------------//
TaskHandle_t task_handl;

CircularBuffer<int, AVERATING_BUFFER_SIZE> buffer_x0;
CircularBuffer<int, AVERATING_BUFFER_SIZE> buffer_x1;
CircularBuffer<int, AVERATING_BUFFER_SIZE> buffer_y0;
CircularBuffer<int, AVERATING_BUFFER_SIZE> buffer_y1;

CircularBuffer<float, ANGLE_AVERATING> angle_averating_buffer_x0;
CircularBuffer<float, ANGLE_AVERATING> angle_averating_buffer_x1;
CircularBuffer<float, ANGLE_AVERATING> angle_averating_buffer_y0;
CircularBuffer<float, ANGLE_AVERATING> angle_averating_buffer_y1;

const unsigned int LCD_STATUS = M5.Lcd.color565(0,0,0);
const unsigned int LCD_MAIN = M5.Lcd.color565(8,8,8);
const unsigned int LCD_BUTTON = M5.Lcd.color565(37,37,37);
const unsigned int LCD_LINE = M5.Lcd.color565(20,20,20);

const unsigned int LCD_GREEN = M5.Lcd.color565(160,200,170);
const unsigned int LCD_RED = M5.Lcd.color565(195,95,45);


// CAN
MCP_CAN CAN0(12);                               // Set CS to pin 12
byte data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];                        // Array to store serial string

// Main
volatile int cnt1 = 0;
bool sd_insert = false;
unsigned char lcd_pattern = 0;
unsigned char lcd_cnt = 0;
volatile bool  lcd_flag = false;
volatile unsigned int millis_buffer = 0;
bool init_flag = false;
char progress_value = 0;
volatile bool tel_flag = false;

float target_angle_x0;
float target_angle_x1;
float target_angle_y0;
float target_angle_y1;

float initial_angle_x0;
float initial_angle_x1;
float initial_angle_y0;
float initial_angle_y1;

float angle_averating_x0;
float angle_averating_x1;
float angle_averating_y0;
float angle_averating_y1;

float angle_x;
float angle_y;

float step_angle_x = 0;
float step_angle_y = 0;

bool interrupt_flag_x0 = false;
bool interrupt_flag2_x0 = false;
bool interrupt_flag_x1 = false;
bool interrupt_flag2_x1 = false;
bool interrupt_flag_y0 = false;
bool interrupt_flag2_y0 = false;
bool interrupt_flag_y1 = false;
bool interrupt_flag2_y1 = false;

int power_x0 = 0;
int power_x1 = 0;
int power_y0 = 0;
int power_y1 = 0;

int servo_x0_output = 0;
int servo_x1_output = 0;
int servo_y0_output = 0;
int servo_y1_output = 0;

volatile int pattern = 0;
int iP_x0, iD_x0;
float iI_x0;
float iBefore_x0;
int iP_x1, iD_x1;
float iI_x1;
float iBefore_x1;
int iP_y0, iD_y0;
float iI_y0;
float iBefore_y0;
int iP_y1, iD_y1;
float iI_y1;
float iBefore_y1;
int agg_kp = 1000;
int agg_ki = 10;
int agg_kd = 0;
int cons_kp = 500;
int cons_ki = 1;
int cons_kd = 0;

float angle_offset_x0 = 0;
float angle_offset_x1 = 0;
float angle_offset_y0 = -1;
float angle_offset_y1 = 0.8;



// WiFi credentials.
// Set password to "" for open networks.
//char ssid[] = "Buffalo-G-0CBA";
//char pass[] = "hh4aexcxesasx";
char ssid[] = "X1Extreme-Hotspot";
char pass[] = "5]6C458w";
//char ssid[] = "Macaw";
//char pass[] = "1234567890";

bool wifi_flag = true;
unsigned char wifi_cnt = 0;

// Time
char ntpServer[] = "ntp.nict.jp";
const long gmtOffset_sec = 9 * 3600;
const int  daylightOffset_sec = 0;
struct tm timeinfo;
String dateStr;
String timeStr;


bool hx711_flag = false;
char hx711_pattern = 0;
float load_step_x[] = {0, 2025, 4050, 6075, 8100, 10125, 12150, 14175, 16200, 18225, 20250};
float load_step_y[] = {0, 45000, 90000, 135000, 180000, 225000};
// HX711 X0
long    native_data_x0;
long    native_data_x0_buffer;
long    averating_data_x0;
long    averating_data_x0_buffer;
long    calibration_factor_x0[11];
float   coefficient_x0;
float   intercept_x0;
long    coefficient_x0_eeprom;
long    intercept_x0_eeprom;
float   torque_x0;
// HX711 X1
long    native_data_x1;
long    native_data_x1_buffer;
long    averating_data_x1;
long    averating_data_x1_buffer;
long    calibration_factor_x1[11];
float   coefficient_x1;
float   intercept_x1;
long    coefficient_x1_eeprom;
long    intercept_x1_eeprom;
float   torque_x1;
// HX711 Y0
long    native_data_y0;
long    native_data_y0_buffer;
long    averating_data_y0;
long    averating_data_y0_buffer;
long    calibration_factor_y0[11];
float   coefficient_y0;
float   intercept_y0;
long    coefficient_y0_eeprom;
long    intercept_y0_eeprom;
float   torque_y0;
// HX711 Y1
long     native_data_y1;
long     native_data_y1_buffer;
long     averating_data_y1;
long     averating_data_y1_buffer;
long     calibration_factor_y1[11];
float    coefficient_y1;
float    intercept_y1;
long    coefficient_y1_eeprom;
long    intercept_y1_eeprom;
float   torque_y1;

int level_x0;
int level_x1;
int level_y0;
int level_y1;

// GM6020
float angle_can_x0;
short velocity_can_x0;
float torque_can_x0;
int8_t temp_can_x0;
float angle_can_x1;
short velocity_can_x1;
float torque_can_x1;
int8_t temp_can_x1;
float angle_can_y0;
short velocity_can_y0;
float torque_can_y0;
int8_t temp_can_y0;
float angle_can_y1;
short velocity_can_y1;
float torque_can_y1;
int8_t temp_can_y1;

// LED 
static const int ledPin = 13;
bool led_flag = false;

// Battery
unsigned char battery_status;
char battery_persent;

// Serial
char     tx_pattern = 1;
char     rx_pattern = 0;
int      rx_val = 0;
char     xbee_rx_buffer[16];
int      xbee_index = 0;

//SD
File file;
String fname_buff;
const char* fname;
volatile bool log_flag = false;

// Timer
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile int interruptCounter;
int iTimer15;
int iTimer50;

// Log
typedef struct {
    float log_angle_x0;
    float log_angle_x1;
    float log_angle_x;
    float log_torque_x0;
    float log_torque_x1;
    float log_torque_x;
    float log_angle_y0;
    float log_angle_y1;
    float log_angle_y;    
    float log_torque_y0;
    float log_torque_y1;
    float log_torque_y;
} RecordType;

static RecordType buffer[2][BufferRecords];
static volatile int writeBank = 0;
static volatile int bufferIndex[2] = {0, 0};

//Prototype
//------------------------------------------------------------------//
void init_can(void);
void test_can(void);
void button_action(void);
void initLCD(void);
void lcdDisplay(void);
void taskDisplay(void *pvParameters);
void IRAM_ATTR onTimer(void);
void TimerInterrupt(void);
void servoControlX0(float ang, bool range);
void servoControlX1(float ang, bool range);
void servoControlY0(float ang, bool range);
void servoControlY1(float ang, bool range);
void targetAngleX0(float tang, float step);
void targetAngleX1(float tang, float step);
void targetAngleY0(float tang, float step);
void targetAngleY1(float tang, float step);
void stepAngleX(float sang);
void stepAngleY(float sang);
void AE_HX711_Init(void);
void AE_HX711_Reset(void);
void AE_HX711_Read(void);
void getTimeFromNTP(void);
void getTime(void);
void SerialRX(void);
void SerialTX(void);
void eeprom_write(void);
void eeprom_read(void);
void getOffsetValue(void);
void getCalibrationDataX0(void);
void getCalibrationDataX1(void);
void getCalibrationDataY0(void);
void getCalibrationDataY1(void);


//Setup #1
//------------------------------------------------------------------//
void setup() {
  M5.begin(1, 1, 0, 1);

  Serial.begin(230400);

  // Initialize Timer Interrupt
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, TIMER_INTERRUPT * 1000, true);
  timerAlarmEnable(timer); 

  pinMode(ledPin, OUTPUT);

  AE_HX711_Init();
  AE_HX711_Reset();

  init_can();

  sd_insert = SD.begin(TFCARD_CS_PIN, SPI, 40000000);

  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.printf("Startup");  

  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    wifi_cnt++;
    M5.Lcd.printf(".");  
    if( wifi_cnt > 14 ) {
      wifi_flag = false;
      break;
    }
    delay(500);  
  }

  if( wifi_flag ) {
    // timeSet
    getTimeFromNTP();
    getTime();
    while(WiFi.status() == WL_CONNECTED ){
      WiFi.disconnect();
      delay(2000);
    }
  }

  initLCD();   

  delay(500);  

  xTaskCreatePinnedToCore(&taskDisplay, "taskDisplay", 4096, NULL, 1, &task_handl, 0);

  delay(500);  

  EEPROM.begin(128);
  delay(10);
  eeprom_read();
   
}

//Main #1
//------------------------------------------------------------------//
void loop() {

  test_can();  

  SerialRX();
  SerialTX();

  
  int readBank = !writeBank;
  if (bufferIndex[readBank] >= BufferRecords) {
    static RecordType temp[BufferRecords];

    memcpy(temp, buffer[readBank], sizeof(temp));
    bufferIndex[readBank] = 0;
    file = SD.open(fname, FILE_APPEND);
    for (int i = 0; i < BufferRecords; i++) {
        file.print(temp[i].log_angle_x0);
        file.print(",");
        file.print(temp[i].log_angle_x1);
        file.print(",");
        file.print(temp[i].log_angle_x);
        file.print(",");
        file.print(temp[i].log_torque_x);
        file.print(",");
        file.print(temp[i].log_torque_x0);
        file.print(",");
        file.print(temp[i].log_torque_x1);        
        file.print(",");
        file.print(temp[i].log_angle_y0);
        file.print(",");
        file.print(temp[i].log_angle_y1);
        file.print(",");
        file.print(temp[i].log_angle_y);
        file.print(",");
        file.print(temp[i].log_torque_y);
        file.print(",");          
        file.print(temp[i].log_torque_y0);
        file.print(",");
        file.println(temp[i].log_torque_y1);        
    }
    file.close();
  }

  switch (pattern) {
  case 0: 
    power_x0 = 0;   
    power_x1 = 0;     
    power_y0 = 0;   
    power_y1 = 0;   
    break;

  // Start Sequence
  case 11:
    step_angle_y = 0;
    targetAngleX0(0, 0.01); 
    targetAngleX1(0, 0.01); 
    servoControlX0(target_angle_x0, 0);  
    servoControlX1(target_angle_x1, 0);  
    power_x0 = servo_x0_output;
    power_x1 = servo_x1_output;
    if(angle_can_x0 > -ANGLE_TOLERANCE && angle_can_x0 < ANGLE_TOLERANCE && angle_can_x1 > -ANGLE_TOLERANCE && angle_can_x1 < ANGLE_TOLERANCE) {
      pattern = 12;
    }
    break;
    
  case 12:
    servoControlX0(0, 1); 
    servoControlX1(0, 1);   
    power_x0 = servo_x0_output;
    power_x1 = servo_x1_output;
    servoControlY0(0, 1);  
    servoControlY1(0, 1);  
    power_y0 = servo_y0_output;
    power_y1 = servo_y1_output;
    if(angle_can_y0 > -ANGLE_TOLERANCE && angle_can_y0 < ANGLE_TOLERANCE && angle_can_y1 > -ANGLE_TOLERANCE && angle_can_y1 < ANGLE_TOLERANCE) {
      getOffsetValue();
      lcd_pattern = 0;
      millis_buffer = millis();
      pattern = 13;
    }
    break;

  case 13:
    tel_flag = true;
    log_flag = true;
    servoControlX0(0, 1); 
    servoControlX1(0, 1);   
    power_x0 = servo_x0_output;
    power_x1 = servo_x1_output;
    servoControlY0(0, 1);  
    servoControlY1(0, 1);  
    power_y0 = servo_y0_output;
    power_y1 = servo_y1_output;    
    if(millis() - millis_buffer > STABLE_TIME) {     
      pattern = 14;
    }
    break;

  case 14:
    servoControlX0(0, 1); 
    servoControlX1(0, 1);   
    power_x0 = servo_x0_output;
    power_x1 = servo_x1_output;
    stepAngleY(YAXIS_MAX_ANGLE);
    servoControlY0(step_angle_y, 1);  
    servoControlY1(step_angle_y, 1);  
    power_y0 = servo_y0_output;
    power_y1 = servo_y1_output;
    if(step_angle_y >= YAXIS_MAX_ANGLE) {
      pattern = 15;
    }
    break;

  case 15:
    servoControlX0(0, 1); 
    servoControlX1(0, 1);   
    power_x0 = servo_x0_output;
    power_x1 = servo_x1_output;
    stepAngleY(0);
    servoControlY0(step_angle_y, 1);  
    servoControlY1(step_angle_y, 1);  
    power_y0 = servo_y0_output;
    power_y1 = servo_y1_output;
    if(step_angle_y < 0) {
      //tx_pattern = 1;
      log_flag = false;
      pattern = 21;
    }
    break;

  // Start Sequence
  case 21:
    step_angle_y = 0;
    targetAngleX0(0, 0.01); 
    targetAngleX1(0, 0.01); 
    servoControlX0(target_angle_x0, 0);  
    servoControlX1(target_angle_x1, 0);  
    power_x0 = servo_x0_output;
    power_x1 = servo_x1_output;
    if(angle_can_x0 > -ANGLE_TOLERANCE && angle_can_x0 < ANGLE_TOLERANCE && angle_can_x1 > -ANGLE_TOLERANCE && angle_can_x1 < ANGLE_TOLERANCE) {
      pattern = 22;
    }
    break;
    
  case 22:
    servoControlX0(0, 1); 
    servoControlX1(0, 1);   
    power_x0 = servo_x0_output;
    power_x1 = servo_x1_output;
    servoControlY0(0, 1);  
    servoControlY1(0, 1);  
    power_y0 = servo_y0_output;
    power_y1 = servo_y1_output;
    if(angle_can_y0 > -ANGLE_TOLERANCE && angle_can_y0 < ANGLE_TOLERANCE && angle_can_y1 > -ANGLE_TOLERANCE && angle_can_y1 < ANGLE_TOLERANCE) {
      getOffsetValue();
      millis_buffer = millis();
      pattern = 23;
    }
    break;

  case 23:
    tel_flag = true;
    log_flag = true;
    servoControlX0(0, 1); 
    servoControlX1(0, 1);   
    power_x0 = servo_x0_output;
    power_x1 = servo_x1_output;
    servoControlY0(0, 1);  
    servoControlY1(0, 1);  
    power_y0 = servo_y0_output;
    power_y1 = servo_y1_output;
    if(millis() - millis_buffer > STABLE_TIME) {
      pattern = 24;
    }
    break;

  case 24:
    servoControlX0(0, 1); 
    servoControlX1(0, 1);   
    power_x0 = servo_x0_output;
    power_x1 = servo_x1_output;
    stepAngleY(YAXIS_MIN_ANGLE);
    servoControlY0(step_angle_y, 0);  
    servoControlY1(step_angle_y, 0);  
    power_y0 = servo_y0_output;
    power_y1 = servo_y1_output;
    if(step_angle_y <= YAXIS_MIN_ANGLE) {
      pattern = 25;
    }
    break;

  case 25:
    servoControlX0(0, 1); 
    servoControlX1(0, 1);   
    power_x0 = servo_x0_output;
    power_x1 = servo_x1_output;
    stepAngleY(0);
    servoControlY0(step_angle_y, 0);  
    servoControlY1(step_angle_y, 0);  
    power_y0 = servo_y0_output;
    power_y1 = servo_y1_output;
    if(step_angle_y > 0) {
      tx_pattern = 1;
      log_flag = false;
      pattern = 0;
    }
    break;

  // Start Sequence
  case 51:
    targetAngleX0(0, 0.001); 
    targetAngleX1(0, 0.001); 
    servoControlX0(target_angle_x0, 0);  
    servoControlX1(target_angle_x1, 0);  
    power_x0 = servo_x0_output;
    power_x1 = servo_x1_output;
    if(angle_can_x0 > -ANGLE_TOLERANCE && angle_can_x0 < ANGLE_TOLERANCE && angle_can_x1 > -ANGLE_TOLERANCE && angle_can_x1 < ANGLE_TOLERANCE) {
      pattern = 52;
    }
    break;
    
  case 52:
    servoControlX0(0, 1); 
    servoControlX1(0, 1);   
    power_x0 = servo_x0_output;
    power_x1 = servo_x1_output;
    servoControlY0(0, 1);  
    servoControlY1(0, 1);  
    power_y0 = servo_y0_output;
    power_y1 = servo_y1_output;
    if(angle_can_y0 > -ANGLE_TOLERANCE && angle_can_y0 < ANGLE_TOLERANCE && angle_can_y1 > -ANGLE_TOLERANCE && angle_can_y1 < ANGLE_TOLERANCE) {
      getOffsetValue();
      millis_buffer = millis();
      pattern = 53;
    }
    break;

  case 53:
    tel_flag = true;
    servoControlX0(0, 1); 
    servoControlX1(0, 1);   
    power_x0 = servo_x0_output;
    power_x1 = servo_x1_output;
    servoControlY0(0, 1);  
    servoControlY1(0, 1);  
    power_y0 = servo_y0_output;
    power_y1 = servo_y1_output;
    if(millis() - millis_buffer > STABLE_TIME) {
      initial_angle_y0 = angle_can_y0;
      initial_angle_y1 = angle_can_y1;
      pattern = 54;
    }
    break;

  case 54:
    servoControlX0(0, 1); 
    servoControlX1(0, 1);   
    power_x0 = servo_x0_output;
    power_x1 = servo_x1_output;
    targetAngleY0(YAXIS_MAX_ANGLE, 0.0005); 
    targetAngleY1(YAXIS_MAX_ANGLE, 0.0005); 
    servoControlY0(target_angle_y0, 1);  
    servoControlY1(target_angle_y1, 1);  
    power_y0 = servo_y0_output;
    power_y1 = servo_y1_output;
    if(angle_can_y0 > YAXIS_MAX_ANGLE-ANGLE_TOLERANCE && angle_can_y0 < YAXIS_MAX_ANGLE+ANGLE_TOLERANCE && angle_can_y1 > YAXIS_MAX_ANGLE-ANGLE_TOLERANCE && angle_can_y1 < YAXIS_MAX_ANGLE+ANGLE_TOLERANCE) {
      millis_buffer = millis();
      pattern = 55;
    }
    break;

  case 55:
    servoControlX0(0, 1); 
    servoControlX1(0, 1);   
    power_x0 = servo_x0_output;
    power_x1 = servo_x1_output;
    servoControlY0(YAXIS_MAX_ANGLE, 1);  
    servoControlY1(YAXIS_MAX_ANGLE, 1);  
    power_y0 = servo_y0_output;
    power_y1 = servo_y1_output;
    if(millis() - millis_buffer > STABLE_TIME) {
      pattern = 56;
    }
    break;

  case 56:
    servoControlX0(0, 1); 
    servoControlX1(0, 1);   
    power_x0 = servo_x0_output;
    power_x1 = servo_x1_output;
    targetAngleY0(0, 0.0005); 
    targetAngleY1(0, 0.0005); 
    servoControlY0(target_angle_y0, 1);  
    servoControlY1(target_angle_y1, 1);  
    power_y0 = servo_y0_output;
    power_y1 = servo_y1_output;
    if(angle_can_y0 > -ANGLE_TOLERANCE && angle_can_y0 < ANGLE_TOLERANCE && angle_can_y1 > -ANGLE_TOLERANCE && angle_can_y1 < ANGLE_TOLERANCE) {
      millis_buffer = millis();
      pattern = 57;
    }
    break;

  case 57:
    servoControlX0(0, 1); 
    servoControlX1(0, 1);   
    power_x0 = servo_x0_output;
    power_x1 = servo_x1_output;
    servoControlY0(0, 1);  
    servoControlY1(0, 1);  
    power_y0 = servo_y0_output;
    power_y1 = servo_y1_output;
    if(millis() - millis_buffer > STABLE_TIME) {
      tx_pattern = 1;
      pattern = 0;
    }
    break;

    // Start Sequence
  case 61:
    targetAngleX0(0, 0.001); 
    targetAngleX1(0, 0.001); 
    servoControlX0(target_angle_x0, 0);  
    servoControlX1(target_angle_x1, 0);  
    power_x0 = servo_x0_output;
    power_x1 = servo_x1_output;
    if(angle_can_x0 > -ANGLE_TOLERANCE && angle_can_x0 < ANGLE_TOLERANCE && angle_can_x1 > -ANGLE_TOLERANCE && angle_can_x1 < ANGLE_TOLERANCE) {
      pattern = 62;
    }
    break;
    
  case 62:
    servoControlX0(0, 1); 
    servoControlX1(0, 1);   
    power_x0 = servo_x0_output;
    power_x1 = servo_x1_output;
    servoControlY0(0, 1);  
    servoControlY1(0, 1);  
    power_y0 = servo_y0_output;
    power_y1 = servo_y1_output;
    if(angle_can_y0 > -ANGLE_TOLERANCE && angle_can_y0 < ANGLE_TOLERANCE && angle_can_y1 > -ANGLE_TOLERANCE && angle_can_y1 < ANGLE_TOLERANCE) {
      getOffsetValue();
      millis_buffer = millis();
      pattern = 63;
    }
    break;

  case 63:
    tel_flag = true;
    servoControlX0(0, 1); 
    servoControlX1(0, 1);   
    power_x0 = servo_x0_output;
    power_x1 = servo_x1_output;
    servoControlY0(0, 1);  
    servoControlY1(0, 1);  
    power_y0 = servo_y0_output;
    power_y1 = servo_y1_output;
    if(millis() - millis_buffer > STABLE_TIME) {
      initial_angle_y0 = angle_can_y0;
      initial_angle_y1 = angle_can_y1;
      pattern = 64;
    }
    break;

  case 64:
    servoControlX0(0, 1); 
    servoControlX1(0, 1);   
    power_x0 = servo_x0_output;
    power_x1 = servo_x1_output;
    targetAngleY0(YAXIS_MIN_ANGLE, 0.0005); 
    targetAngleY1(YAXIS_MIN_ANGLE, 0.0005); 
    servoControlY0(target_angle_y0, 1);  
    servoControlY1(target_angle_y1, 1);  
    power_y0 = servo_y0_output;
    power_y1 = servo_y1_output;
    if(angle_can_y0 > YAXIS_MIN_ANGLE-ANGLE_TOLERANCE && angle_can_y0 < YAXIS_MIN_ANGLE+ANGLE_TOLERANCE && angle_can_y1 > YAXIS_MIN_ANGLE-ANGLE_TOLERANCE && angle_can_y1 < YAXIS_MIN_ANGLE+ANGLE_TOLERANCE) {
      millis_buffer = millis();
      pattern = 65;
    }
    break;

  case 65:
    servoControlX0(0, 1); 
    servoControlX1(0, 1);   
    power_x0 = servo_x0_output;
    power_x1 = servo_x1_output;
    servoControlY0(YAXIS_MIN_ANGLE, 1);  
    servoControlY1(YAXIS_MIN_ANGLE, 1);  
    power_y0 = servo_y0_output;
    power_y1 = servo_y1_output;
    if(millis() - millis_buffer > STABLE_TIME) {
      pattern = 66;
    }
    break;

  case 66:
    servoControlX0(0, 1); 
    servoControlX1(0, 1);   
    power_x0 = servo_x0_output;
    power_x1 = servo_x1_output;
    targetAngleY0(0, 0.0005); 
    targetAngleY1(0, 0.0005); 
    servoControlY0(target_angle_y0, 1);  
    servoControlY1(target_angle_y1, 1);  
    power_y0 = servo_y0_output;
    power_y1 = servo_y1_output;
    if(angle_can_y0 > -ANGLE_TOLERANCE && angle_can_y0 < ANGLE_TOLERANCE && angle_can_y1 > -ANGLE_TOLERANCE && angle_can_y1 < ANGLE_TOLERANCE) {
      millis_buffer = millis();
      pattern = 67;
    }
    break;

  case 67:
    servoControlX0(0, 1); 
    servoControlX1(0, 1);   
    power_x0 = servo_x0_output;
    power_x1 = servo_x1_output;
    servoControlY0(0, 1);  
    servoControlY1(0, 1);  
    power_y0 = servo_y0_output;
    power_y1 = servo_y1_output;
    if(millis() - millis_buffer > STABLE_TIME) {
      tx_pattern = 1;
      pattern = 0;
    }
    break;
  
  // Set Home Position
  case 101:
    if(angle_can_x0 > angle_can_x1+ANGLE_TOLERANCE) {
      if( angle_can_x0 > 0 && angle_can_x1 > 0) {
        pattern = 111;
        break;
      } else if( angle_can_x0 < 0 && angle_can_x1 < 0) {
        pattern = 121;
        break;
      } else {
        pattern = 131;
        break;
      }     
    } else if(angle_can_x0 < angle_can_x1-ANGLE_TOLERANCE) {
      if( angle_can_x0 > 0 && angle_can_x1 > 0) {
        pattern = 121;
        break;
      } else if( angle_can_x0 < 0 && angle_can_x1 < 0) {
        pattern = 111;
        break;
      } else {
        pattern = 131;
        break;
      }
    } else {
      pattern = 131;
      break;
    }
    break;

  case 111:
    targetAngleX0(angle_can_x1, 0.01); 
    servoControlX0(target_angle_x0, 0);  
    power_x0 = servo_x0_output;
    if(angle_can_x0 > angle_can_x1-ANGLE_TOLERANCE && angle_can_x0 < angle_can_x1+ANGLE_TOLERANCE) {
      pattern = 131;
    }
    break;
  
  case 121:
    targetAngleX1(angle_can_x0, 0.01); 
    servoControlX1(target_angle_x1, 0);  
    power_x1 = servo_x1_output;
    if(angle_can_x1 > angle_can_x0-ANGLE_TOLERANCE && angle_can_x1 < angle_can_x0+ANGLE_TOLERANCE) {
      pattern = 131;
    }
    break;

  case 131:
    targetAngleX0(0, 0.01); 
    targetAngleX1(0, 0.01); 
    servoControlX0(target_angle_x0, 0);  
    servoControlX1(target_angle_x1, 0);  
    power_x0 = servo_x0_output;
    power_x1 = servo_x1_output;
    if(angle_can_x0 > -ANGLE_TOLERANCE && angle_can_x0 < ANGLE_TOLERANCE && angle_can_x1 > -ANGLE_TOLERANCE && angle_can_x1 < ANGLE_TOLERANCE) {
      pattern = 151;
    }
    break;

  case 151:
    if(angle_can_y0 > angle_can_y1+ANGLE_TOLERANCE) {
      if( angle_can_y0 > 0 && angle_can_y1 > 0) {
        pattern = 161;
        break;
      } else if( angle_can_y0 < 0 && angle_can_y1 < 0) {
        pattern = 171;
        break;
      } else {
        pattern = 181;
        break;
      }     
    } else if(angle_can_y0 < angle_can_y1-ANGLE_TOLERANCE) {
      if( angle_can_y0 > 0 && angle_can_y1 > 0) {
        pattern = 171;
        break;
      } else if( angle_can_y0 < 0 && angle_can_y1 < 0) {
        pattern = 161;
        break;
      } else {
        pattern = 181;
        break;
      }
    } else {
      pattern = 181;
      break;
    }
    break;

  case 161:
    targetAngleY0(angle_can_y1, 0.01); 
    servoControlY0(target_angle_y0, 0);  
    power_y0 = servo_y0_output;
    if(angle_can_y0 > angle_can_y1-ANGLE_TOLERANCE && angle_can_y0 < angle_can_y1+ANGLE_TOLERANCE) {
      pattern = 181;
    }
    break;
  
  case 171:
    targetAngleY1(angle_can_y0, 0.01); 
    servoControlY1(target_angle_y1, 0);  
    power_y1 = servo_y1_output;
    if(angle_can_y1 > angle_can_y0-ANGLE_TOLERANCE && angle_can_y1 < angle_can_y0+ANGLE_TOLERANCE) {
      pattern = 181;
    }
    break;

  case 181:
    targetAngleY0(0, 0.01); 
    targetAngleY1(0, 0.01); 
    servoControlY0(target_angle_y0, 0);  
    servoControlY1(target_angle_y1, 0);  
    power_y0 = servo_y0_output;
    power_y1 = servo_y1_output;
    if(angle_can_y0 > -ANGLE_TOLERANCE && angle_can_y0 < ANGLE_TOLERANCE && angle_can_y1 > -ANGLE_TOLERANCE && angle_can_y1 < ANGLE_TOLERANCE) {
      pattern = 0;
      Serial.printf(" Complete!");
      lcd_pattern = 0;
      tx_pattern = 1;
    }
    break;

  // Initialize
  case 201:
    getOffsetValue();
    lcd_pattern = 0;
    pattern = 0;
    break;

  // Calibration
  case 301:
    getCalibrationDataX0();
    pattern = 0;
    break;

  case 311:
    getCalibrationDataX1();
    pattern = 0;
    break;

  case 321:
    getCalibrationDataY0();
    pattern = 0;
    break;

  case 331:
    getCalibrationDataY1();
    pattern = 0;
    break;


  }
}

//Main #0
//------------------------------------------------------------------//
void taskDisplay(void *pvParameters){  

  disableCore0WDT(); 
  
  while(1){    

    TimerInterrupt();  

    lcdDisplay();   

    button_action(); 
    AE_HX711_Read();
  }
}

// Timer Interrupt
//------------------------------------------------------------------//
void TimerInterrupt( void ){
  if (interruptCounter > 0) {
    portENTER_CRITICAL(&timerMux);
    interruptCounter--;
    portEXIT_CRITICAL(&timerMux);   

    interrupt_flag_x0 =  true;
    interrupt_flag2_x0 = true;
    interrupt_flag_x1 =  true;
    interrupt_flag2_x1 = true;
    interrupt_flag_y0 =  true;
    interrupt_flag2_y0 = true;
    interrupt_flag_y1 =  true;
    interrupt_flag2_y1 = true;

    angle_x = angle_can_x0 + angle_can_x1;
    angle_y = angle_can_y0 + angle_can_y1;

    iTimer15++;
    switch (iTimer15) {
    case 1:  
      hx711_flag = true;
      break;
    case 2:      
      break;
    case 3:  
      hx711_flag = true;    
      break;
    case 4:      
      break;
    case 5:   
      hx711_flag = true;   
      break;
    case 6:      
      break;
    case 7:  
      hx711_flag = true;    
      break;
    case 15:      
      iTimer15 = 0;
      break;
    }

    iTimer50++;
    // 50ms timerinterrupt
    switch (iTimer50) {
    case 10:
      if( tx_pattern == 101 ) {
        if(pattern < 30 && pattern != 0) {
          if(tel_flag) {
            Serial.printf("%5.2f, ", float(millis()) / 1000); 
            Serial.printf("%5d, ", pattern); 
            Serial.printf("%5.2f, ", step_angle_y*2); 
            Serial.printf("%5.2f, ", angle_y); 
            Serial.printf("%5.2f, ", torque_y0); 
            Serial.printf("%5.2f\n", torque_y1); 
          }
        } else if(pattern > 50 && pattern < 100) {
          Serial.printf("%5.2f, ", float(millis()) / 1000); 
          Serial.printf("%5d, ", pattern); 
          Serial.printf("%5.2f, ", angle_y); 
          Serial.printf("%5.2f, ", torque_y0); 
          Serial.printf("%5.2f\n", torque_y1); 
        } else {
          Serial.printf("%5.2f, ", float(millis()) / 1000); 
          Serial.printf("%5d, ", pattern); 
          Serial.printf("%3.2f, ", angle_can_x0); 
          Serial.printf("%3.2f, ", angle_can_x1); 
          Serial.printf("%3.2f, ", angle_can_y0); 
          Serial.printf("%3.2f, ", angle_can_y1); 
          Serial.printf("%10ld, ", native_data_x0); 
          Serial.printf("%5d, ", level_x0); 
          Serial.printf("%10ld, ", native_data_x1); 
          Serial.printf("%5d, ", level_x1); 
          Serial.printf("%10ld, ", native_data_y0); 
          Serial.printf("%5d, ", level_y0); 
          Serial.printf("%10ld, ", native_data_y1); 
          Serial.printf("%5d, ", level_y1); 
          Serial.printf("%5.2f, ", torque_x0); 
          Serial.printf("%5.2f, ", torque_x1); 
          Serial.printf("%5.2f, ", torque_y0); 
          Serial.printf("%5.2f\n", torque_y1); 
        } 
      }
      break;
    case 20:    
      if (log_flag && bufferIndex[writeBank] < BufferRecords) {
        RecordType* rp = &buffer[writeBank][bufferIndex[writeBank]];        
        rp->log_angle_x0 = angle_can_x0;
        rp->log_angle_x1 = angle_can_x1;
        rp->log_angle_x = angle_x;
        rp->log_torque_x = torque_x0 + torque_x1;
        rp->log_torque_x0 = torque_x0;
        rp->log_torque_x1 = torque_x1;
        rp->log_angle_y0 = angle_can_y0;
        rp->log_angle_y1 = angle_can_y1;
        rp->log_angle_y = angle_y;        
        rp->log_torque_y = torque_y0 + torque_y1;
        rp->log_torque_y0 = torque_y0;
        rp->log_torque_y1 = torque_y1;
        if (++bufferIndex[writeBank] >= BufferRecords) {
            writeBank = !writeBank;
        }
      }      
      break;
    case 30:
      break;
    case 40:
      break;
    case 50:
      lcd_flag = true;
      iTimer50 = 0;
      break;
    }
  }
}

// Serial RX
//------------------------------------------------------------------//
void SerialRX(void) {

  while (Serial.available()) {
    xbee_rx_buffer[xbee_index] = Serial.read();
    Serial.write(xbee_rx_buffer[xbee_index]);

    if( xbee_rx_buffer[xbee_index] == 0x08 ) {
      xbee_rx_buffer[xbee_index-1] = NULL;
      xbee_index--;
      Serial.print(" ");      
      Serial.write(0x08);
    } else if( xbee_rx_buffer[xbee_index] == 0x0D ) {
      Serial.read();
      Serial.print("\n\n"); 
      if( tx_pattern == 0 ) {
        rx_pattern = atoi(xbee_rx_buffer);
      } else if( tx_pattern == 2 ) {
        rx_val = atof(xbee_rx_buffer);
      } else if( tx_pattern == 3 ) {
        fname_buff = "/log/"
                +(String)atoi(xbee_rx_buffer)
                +".csv";
        Serial.println(fname_buff);
        fname = fname_buff.c_str();
        rx_pattern = 51;
      }
      xbee_index = 0;
      
      switch ( rx_pattern ) {          
      case 0:
        tx_pattern = 1;
        break;
        
      case 11:      
        tx_pattern = 51;
        break;

      case 51:      
        file = SD.open(fname, FILE_APPEND); 
        file.print("Angle-X0");
        file.print(",");
        file.print("Angle-X1");
        file.print(",");
        file.print("Angle-X");
        file.print(",");
        file.print("Moment-X");
        file.print(",");      
        file.print("Moment-X0");
        file.print(",");        
        file.print("Moment-X1");        
        file.print(",");  
        file.print("Angle-Y0");
        file.print(",");
        file.print("Angle-Y1");
        file.print(",");
        file.print("Angle-Y");
        file.print(",");
        file.println("Moment-Y");
        file.print(",");        
        file.print("Moment-Y0");
        file.print(",");        
        file.print("Moment-Y1");        
        file.close();
        tx_pattern = 11;
        rx_pattern = 101;
        break;


      case 101:
        if( rx_val == 1 ) {   
          initial_angle_x0 = angle_can_x0;
          initial_angle_x1 = angle_can_x1;
          initial_angle_y0 = angle_can_y0;
          initial_angle_y1 = angle_can_y1;
          iI_x0 = 0;       
          iBefore_x0 = 0;
          iI_x1 = 0;       
          iBefore_x1 = 0;
          iI_y0 = 0;       
          iBefore_y0 = 0;
          iI_y1 = 0;       
          iBefore_y1 = 0;         
          pattern = 11;
        } else {
          tx_pattern = 1;
        }
        rx_pattern = 0;
        break;

      case 12:      
        tx_pattern = 12;
        rx_pattern = 102;
        break;

      case 102:
        if( rx_val == 1 ) {   
          initial_angle_x0 = angle_can_x0;
          initial_angle_x1 = angle_can_x1;
          initial_angle_y0 = angle_can_y0;
          initial_angle_y1 = angle_can_y1;
          iI_x0 = 0;       
          iBefore_x0 = 0;
          iI_x1 = 0;       
          iBefore_x1 = 0;
          iI_y0 = 0;       
          iBefore_y0 = 0;
          iI_y1 = 0;       
          iBefore_y1 = 0;         
          pattern = 21;
        } else {
          tx_pattern = 1;
        }
        rx_pattern = 0;
        break;

      case 13:      
        tx_pattern = 13;
        rx_pattern = 103;
        break;

      case 103:
        if( rx_val == 1 ) {   
          initial_angle_x0 = angle_can_x0;
          initial_angle_x1 = angle_can_x1;
          initial_angle_y0 = angle_can_y0;
          initial_angle_y1 = angle_can_y1;
          iI_x0 = 0;       
          iBefore_x0 = 0;
          iI_x1 = 0;       
          iBefore_x1 = 0;
          iI_y0 = 0;       
          iBefore_y0 = 0;
          iI_y1 = 0;       
          iBefore_y1 = 0;         
          pattern = 51;
        } else {
          tx_pattern = 1;
        }
        rx_pattern = 0;
        break;

      case 14:      
        tx_pattern = 14;
        rx_pattern = 104;
        break;

      case 104:
        if( rx_val == 1 ) {   
          initial_angle_x0 = angle_can_x0;
          initial_angle_x1 = angle_can_x1;
          initial_angle_y0 = angle_can_y0;
          initial_angle_y1 = angle_can_y1;
          iI_x0 = 0;       
          iBefore_x0 = 0;
          iI_x1 = 0;       
          iBefore_x1 = 0;
          iI_y0 = 0;       
          iBefore_y0 = 0;
          iI_y1 = 0;       
          iBefore_y1 = 0;         
          pattern = 61;
        } else {
          tx_pattern = 1;
        }
        rx_pattern = 0;
        break;

      case 21:         
        pattern = 301;
        rx_pattern = 0;
        break;

      case 22:             
        pattern = 311;
        rx_pattern = 0;
        break;

      case 23:             
        pattern = 321;
        rx_pattern = 0;
        break;

      case 24:             
        pattern = 331;
        rx_pattern = 0;
        break;

      case 31:
        tx_pattern = 31;
        rx_pattern = 41;
        break;
      case 41:
        tx_pattern = 1;
        rx_pattern = 0;
        break;

      case 201:
        if( rx_val == 1 ) {  
          pattern = 201;
        } else {
          tx_pattern = 1;
        }
        rx_pattern = 0;
        break;
      }      
      
    } else if( xbee_rx_buffer[xbee_index] == ' ') {
      pattern = 0;
      rx_pattern = 0;      
      tx_pattern = 1;
    } else if( xbee_rx_buffer[xbee_index] == 'T' || xbee_rx_buffer[xbee_index] == 't' ) {
      rx_pattern = 0;
      Serial.printf("\n\n");
      tx_pattern = 101;
    } else if( xbee_rx_buffer[xbee_index] == 'I' || xbee_rx_buffer[xbee_index] == 'i' ) {
      if(pattern == 0) {
        Serial.printf("\n\n");
        tx_pattern = 201;
        rx_pattern = 201;
      }
    } else if( xbee_rx_buffer[xbee_index] == 'H' || xbee_rx_buffer[xbee_index] == 'h' ) {
      if(pattern == 0) {
        Serial.printf("\n\n");
        initial_angle_x0 = angle_can_x0;
        initial_angle_x1 = angle_can_x1;
        initial_angle_y0 = angle_can_y0;
        initial_angle_y1 = angle_can_y1;
        iI_x0 = 0;       
        iBefore_x0 = 0;
        iI_x1 = 0;       
        iBefore_x1 = 0;
        iI_y0 = 0;       
        iBefore_y0 = 0;
        iI_y1 = 0;       
        iBefore_y1 = 0;
        lcd_pattern = 20;
        pattern = 101;
        tx_pattern = 202;
        rx_pattern = 0;
      }
    } else {
        xbee_index++;
    }
  }

}

// Serial TX
//------------------------------------------------------------------//
void SerialTX(void) {

  switch ( tx_pattern ) {   
    // Waiting Command
    case 0:
      break;

    case 1:
      Serial.printf("\n\n\n\n\n\n");
      Serial.printf(" Climber Controller (M5Stack version) "
                     "Test Program Ver1.20\n");
      Serial.printf("\n");
      Serial.printf(" 11 : Start Seqence Step 2D+\n");
      Serial.printf(" 12 : Start Seqence Step 2D-\n");
      Serial.printf(" 13 : Start Seqence Continuous 2D+\n");
      Serial.printf(" 14 : Start Seqence Continuous 2D-\n");
      Serial.printf("\n");
      Serial.printf(" 21 : Calibration X0\n");
      Serial.printf(" 22 : Calibration X1\n");
      Serial.printf(" 23 : Calibration Y0\n");
      Serial.printf(" 24 : Calibration Y1\n");
      Serial.printf("\n");
      Serial.printf(" T : Telemetry\n");
      Serial.printf(" I : Initialize\n");
      Serial.printf(" H : Homing\n");
      
      Serial.printf("\n");
      Serial.printf(" Please enter 11 to 35 -> ");
      
      tx_pattern = 0;
      break;

    // Waiting Value
    case 2:
      break;

    // Waiting Name
    case 3:
      break;

    case 11:
      Serial.printf(" Confirm to Start Sequence Step 2D+? -> ");
      tx_pattern = 2;
      break;

    case 12:
      Serial.printf(" Confirm to Start Sequence Step 2D-? -> ");
      tx_pattern = 2;
      break;

    case 13:
      Serial.printf(" Confirm to Start Sequence Continuous 2D+? -> ");
      tx_pattern = 2;
      break;

    case 14:
      Serial.printf(" Confirm to Start Sequence Continuous 2D-? -> ");
      tx_pattern = 2;
      break;

    case 51:
      Serial.printf(" Enter the experimental parameters -> ");
      tx_pattern = 3;
      break;

    // Telemetry Mode
    case 101:
      break;

    // Initialize 
    case 201:
      Serial.printf(" Confirm to Initialize ? -> ");
      tx_pattern = 2;
      break;

    // Initialize 
    case 202:
      Serial.printf(" Homing All Axis...\n\n");
      tx_pattern = 0;
      break;

  }
}

// EEPROM Write
//------------------------------------------------------------------// 
void eeprom_write(void) {
  EEPROM.write(0, (coefficient_x0_eeprom & 0xFF));
  EEPROM.write(1, (coefficient_x0_eeprom>>8 & 0xFF));
  EEPROM.write(2, (coefficient_x0_eeprom>>16 & 0xFF));
  EEPROM.write(3, (coefficient_x0_eeprom>>24 & 0xFF));
  EEPROM.write(4, (coefficient_x1_eeprom & 0xFF));
  EEPROM.write(5, (coefficient_x1_eeprom>>8 & 0xFF));
  EEPROM.write(6, (coefficient_x1_eeprom>>16 & 0xFF));
  EEPROM.write(7, (coefficient_x1_eeprom>>24 & 0xFF));
  EEPROM.write(8, (coefficient_y0_eeprom & 0xFF));
  EEPROM.write(9, (coefficient_y0_eeprom>>8 & 0xFF));
  EEPROM.write(10, (coefficient_y0_eeprom>>16 & 0xFF));
  EEPROM.write(11, (coefficient_y0_eeprom>>24 & 0xFF));
  EEPROM.write(12, (coefficient_y1_eeprom & 0xFF));
  EEPROM.write(13, (coefficient_y1_eeprom>>8 & 0xFF));
  EEPROM.write(14, (coefficient_y1_eeprom>>16 & 0xFF));
  EEPROM.write(15, (coefficient_y1_eeprom>>24 & 0xFF));
  EEPROM.write(16, (intercept_x0_eeprom & 0xFF));
  EEPROM.write(17, (intercept_x0_eeprom>>8 & 0xFF));
  EEPROM.write(18, (intercept_x0_eeprom>>16 & 0xFF));
  EEPROM.write(19, (intercept_x0_eeprom>>24 & 0xFF));
  EEPROM.write(20, (intercept_x1_eeprom & 0xFF));
  EEPROM.write(21, (intercept_x1_eeprom>>8 & 0xFF));
  EEPROM.write(22, (intercept_x1_eeprom>>16 & 0xFF));
  EEPROM.write(23, (intercept_x1_eeprom>>24 & 0xFF));
  EEPROM.write(24, (intercept_y0_eeprom & 0xFF));
  EEPROM.write(25, (intercept_y0_eeprom>>8 & 0xFF));
  EEPROM.write(26, (intercept_y0_eeprom>>16 & 0xFF));
  EEPROM.write(27, (intercept_y0_eeprom>>24 & 0xFF));
  EEPROM.write(28, (intercept_y1_eeprom & 0xFF));
  EEPROM.write(29, (intercept_y1_eeprom>>8 & 0xFF));
  EEPROM.write(30, (intercept_y1_eeprom>>16 & 0xFF));
  EEPROM.write(31, (intercept_y1_eeprom>>24 & 0xFF));
  delay(10);
  EEPROM.commit();
  delay(10);
}

// EEPROM Read
//------------------------------------------------------------------// 
void eeprom_read(void) {
    coefficient_x0_eeprom = EEPROM.read(0) + (EEPROM.read(1)<<8) + (EEPROM.read(2)<<16) + (EEPROM.read(3)<<24);
    coefficient_x1_eeprom = EEPROM.read(4) + (EEPROM.read(5)<<8) + (EEPROM.read(6)<<16) + (EEPROM.read(7)<<24);
    coefficient_y0_eeprom = EEPROM.read(8) + (EEPROM.read(9)<<8) + (EEPROM.read(10)<<16) + (EEPROM.read(11)<<24);
    coefficient_y1_eeprom = EEPROM.read(12) + (EEPROM.read(13)<<8) + (EEPROM.read(14)<<16) + (EEPROM.read(15)<<24);
    coefficient_x0 = (float)coefficient_x0_eeprom / 1000000;
    coefficient_x1 = (float)coefficient_x1_eeprom / 1000000;
    coefficient_y0 = (float)coefficient_y0_eeprom / 1000000;
    coefficient_y1 = (float)coefficient_y1_eeprom / 1000000;
    intercept_x0_eeprom = EEPROM.read(16) + (EEPROM.read(17)<<8) + (EEPROM.read(18)<<16) + (EEPROM.read(19)<<24);
    intercept_x1_eeprom = EEPROM.read(20) + (EEPROM.read(21)<<8) + (EEPROM.read(22)<<16) + (EEPROM.read(23)<<24);
    intercept_y0_eeprom = EEPROM.read(24) + (EEPROM.read(25)<<8) + (EEPROM.read(26)<<16) + (EEPROM.read(27)<<24);
    intercept_y1_eeprom = EEPROM.read(28) + (EEPROM.read(29)<<8) + (EEPROM.read(30)<<16) + (EEPROM.read(31)<<24);
    delay(10);
}

// Initialize LCD
//------------------------------------------------------------------//
void initLCD(void) {

  M5.Lcd.clear();     
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.setCursor(20, 10);
  M5.Lcd.setTextSize(2);
  M5.Lcd.printf("T+:");  
  M5.Lcd.setCursor(20, 60);
  if( sd_insert ) {
    M5.Lcd.drawJpgFile(SD, "/icon/icons8-sd.jpg", 248, 0);
  }
  if( wifi_flag ) {
    M5.Lcd.drawJpgFile(SD, "/icon/icons8-wi-fi-1.jpg", 286, 0);      
  } else {
    M5.Lcd.drawJpgFile(SD, "/icon/icons8-wi-fi-0.jpg", 286, 0);     
  }

  M5.Lcd.fillRect(0, 32, 320, 180, LCD_MAIN);
  
  M5.Lcd.fillRect(0, 210, 320, 30, LCD_BUTTON);
  M5.Lcd.drawFastVLine(110, 210, 30, LCD_LINE);
  M5.Lcd.drawFastVLine(210, 210, 30, LCD_LINE);
  M5.Lcd.setTextColor(WHITE, LCD_BUTTON);
  M5.Lcd.setCursor(37, 217);
  M5.Lcd.printf("Init");
  M5.Lcd.setCursor(137, 217);
  M5.Lcd.printf("Home");
  M5.Lcd.setCursor(235, 217);
  M5.Lcd.printf("Seq");

}

// LCD Display
//------------------------------------------------------------------//
void lcdDisplay(void) {
  unsigned int time_calc, time_h, time_m, time_s;

  if( lcd_flag ) {    
    time_calc = millis() / 1000;
    time_h = time_calc / 3600;
    time_calc %= 3600;
    time_m = time_calc / 60;
    time_calc %= 60;
    time_s = time_calc;  

    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.setCursor(60, 10);
    M5.Lcd.printf("%02d:%02d:%02d", time_h, time_m, time_s);

    // Main
    switch (lcd_pattern) {
    case 0:      
      M5.Lcd.fillRect(0, 32, 320, 180, LCD_MAIN);
      M5.Lcd.drawFastHLine(0, 77, 320, LCD_LINE);
      M5.Lcd.drawFastHLine(0, 122, 320, LCD_LINE);
      M5.Lcd.drawFastHLine(0, 167, 320, LCD_LINE);
      lcd_pattern = 1;
      break;
    case 1:    
      M5.Lcd.setCursor(10, 47);
      M5.Lcd.printf("X0"); 
      if( angle_can_x0 > ANGLE_TOLERANCE || angle_can_x0 < ANGLE_TOLERANCE*-1 ) M5.Lcd.setTextColor(LCD_RED, LCD_MAIN);   
      else M5.Lcd.setTextColor(LCD_GREEN, LCD_MAIN); 
      M5.Lcd.setCursor(50, 47);
      M5.Lcd.printf("%+05.2f", angle_can_x0); 
      if( torque_x0 > TORQUE_TOLERANCE_X || torque_x0 < TORQUE_TOLERANCE_X*-1 ) M5.Lcd.setTextColor(LCD_RED, LCD_MAIN);        
      else M5.Lcd.setTextColor(LCD_GREEN, LCD_MAIN);      
      M5.Lcd.setCursor(130, 47);
      M5.Lcd.printf("%+03.2f", torque_x0);   
      if( level_x0 > I_TORQUE_TOLERANCE_X || level_x0 < I_TORQUE_TOLERANCE_X*-1 ) M5.Lcd.setTextColor(LCD_RED, LCD_MAIN);        
      else M5.Lcd.setTextColor(LCD_GREEN, LCD_MAIN);      
      M5.Lcd.setCursor(230, 47);
      M5.Lcd.printf("%+04d", level_x0);   
      lcd_pattern = 2;
      //lcd_flag = false; 
      break;
    case 2:    
      M5.Lcd.setCursor(10, 92);
      M5.Lcd.printf("X1"); 
      if( angle_can_x1 > ANGLE_TOLERANCE || angle_can_x1 < ANGLE_TOLERANCE*-1 ) M5.Lcd.setTextColor(LCD_RED, LCD_MAIN);   
      else M5.Lcd.setTextColor(LCD_GREEN, LCD_MAIN); 
      M5.Lcd.setCursor(50, 92);
      M5.Lcd.printf("%+05.2f", angle_can_x1); 
      if( torque_x1 > TORQUE_TOLERANCE_X || torque_x1 < TORQUE_TOLERANCE_X*-1 ) M5.Lcd.setTextColor(LCD_RED, LCD_MAIN);        
      else M5.Lcd.setTextColor(LCD_GREEN, LCD_MAIN);      
      M5.Lcd.setCursor(130, 92);
      M5.Lcd.printf("%+03.2f", torque_x1);   
      if( level_x1 > I_TORQUE_TOLERANCE_X || level_x1 < I_TORQUE_TOLERANCE_X*-1 ) M5.Lcd.setTextColor(LCD_RED, LCD_MAIN);        
      else M5.Lcd.setTextColor(LCD_GREEN, LCD_MAIN);      
      M5.Lcd.setCursor(230, 92);
      M5.Lcd.printf("%+04d", level_x1);   
      lcd_pattern = 3;
      //lcd_flag = false; 
      break;
    case 3:    
      M5.Lcd.setCursor(10, 137);
      M5.Lcd.printf("Y0"); 
      if( angle_can_y0 > ANGLE_TOLERANCE || angle_can_y0 < ANGLE_TOLERANCE*-1 ) M5.Lcd.setTextColor(LCD_RED, LCD_MAIN);   
      else M5.Lcd.setTextColor(LCD_GREEN, LCD_MAIN); 
      M5.Lcd.setCursor(50, 137);
      M5.Lcd.printf("%+05.2f", angle_can_y0); 
      if( torque_y0 > TORQUE_TOLERANCE_Y || torque_y0 < TORQUE_TOLERANCE_Y*-1 ) M5.Lcd.setTextColor(LCD_RED, LCD_MAIN);        
      else M5.Lcd.setTextColor(LCD_GREEN, LCD_MAIN);      
      M5.Lcd.setCursor(130, 137);
      M5.Lcd.printf("%+03.2f", torque_y0);   
      if( level_y0 > I_TORQUE_TOLERANCE_Y || level_y0 < I_TORQUE_TOLERANCE_Y*-1 ) M5.Lcd.setTextColor(LCD_RED, LCD_MAIN);        
      else M5.Lcd.setTextColor(LCD_GREEN, LCD_MAIN);      
      M5.Lcd.setCursor(230, 137);
      M5.Lcd.printf("%+04d", level_y0);   
      lcd_pattern = 4;
      //lcd_flag = false; 
      break;
    case 4:    
      M5.Lcd.setCursor(10, 182);
      M5.Lcd.printf("Y1"); 
      if( angle_can_y1 > ANGLE_TOLERANCE || angle_can_y1 < ANGLE_TOLERANCE*-1 ) M5.Lcd.setTextColor(LCD_RED, LCD_MAIN);   
      else M5.Lcd.setTextColor(LCD_GREEN, LCD_MAIN); 
      M5.Lcd.setCursor(50, 182);
      M5.Lcd.printf("%+05.2f", angle_can_y1); 
      if( torque_y1 > TORQUE_TOLERANCE_Y || torque_y1 < TORQUE_TOLERANCE_Y*-1 ) M5.Lcd.setTextColor(LCD_RED, LCD_MAIN);        
      else M5.Lcd.setTextColor(LCD_GREEN, LCD_MAIN);      
      M5.Lcd.setCursor(130, 182);
      M5.Lcd.printf("%+03.2f", torque_y1);   
      if( level_y1 > I_TORQUE_TOLERANCE_Y || level_y1 < I_TORQUE_TOLERANCE_Y*-1 ) M5.Lcd.setTextColor(LCD_RED, LCD_MAIN);        
      else M5.Lcd.setTextColor(LCD_GREEN, LCD_MAIN);      
      M5.Lcd.setCursor(230, 182);
      M5.Lcd.printf("%+04d", level_y1);   
      lcd_pattern = 1;
      lcd_flag = false; 
      break;
    // Initialize
    case 10:      
      M5.Lcd.fillRect(0, 32, 320, 180, LCD_MAIN);
      lcd_pattern = 11;
      lcd_flag = false; 
    case 11:      
      M5.Lcd.setTextColor(WHITE, LCD_MAIN);
      M5.Lcd.setCursor(30, 100);
      M5.Lcd.printf("Initializing X0...");      
      M5.Lcd.progressBar(20,150,280,10, progress_value);
      lcd_flag = false; 
      break;
    case 12:      
      M5.Lcd.setTextColor(WHITE, LCD_MAIN);
      M5.Lcd.setCursor(30, 100);
      M5.Lcd.printf("Initializing X1...");      
      M5.Lcd.progressBar(20,150,280,10, progress_value);
      lcd_flag = false; 
      break;
    case 13:      
      M5.Lcd.setTextColor(WHITE, LCD_MAIN);
      M5.Lcd.setCursor(30, 100);
      M5.Lcd.printf("Initializing Y0...");      
      M5.Lcd.progressBar(20,150,280,10, progress_value);
      lcd_flag = false; 
      break;
    case 14:      
      M5.Lcd.setTextColor(WHITE, LCD_MAIN);
      M5.Lcd.setCursor(30, 100);
      M5.Lcd.printf("Initializing Y1...");      
      M5.Lcd.progressBar(20,150,280,10, progress_value);
      lcd_flag = false; 
      break;
    case 15:      
      M5.Lcd.setTextColor(WHITE, LCD_MAIN);
      M5.Lcd.setCursor(30, 100);
      M5.Lcd.printf("Complete!");      
      lcd_flag = false; 
      break;
    // Homing
    case 20:      
      M5.Lcd.fillRect(0, 32, 320, 180, LCD_MAIN);
      lcd_pattern = 21;
      lcd_flag = false; 
    case 21:      
      M5.Lcd.setTextColor(WHITE, LCD_MAIN);
      M5.Lcd.setCursor(30, 100);
      M5.Lcd.printf("Homing...");      
      lcd_flag = false; 
      break;
    }
  }
}

// Button Action
//------------------------------------------------------------------//
void button_action(void) {
  M5.update();
  if (M5.BtnA.wasPressed()) {
    if(pattern == 0) {
      Serial.printf("\n\n Initializing...\n\n");
      pattern = 201;
    }
  } else if (M5.BtnB.wasPressed()) {   
    if(pattern == 0) {
      Serial.printf("\n\n");
      initial_angle_x0 = angle_can_x0;
      initial_angle_x1 = angle_can_x1;
      initial_angle_y0 = angle_can_y0;
      initial_angle_y1 = angle_can_y1;
      iI_x0 = 0;       
      iBefore_x0 = 0;
      iI_x1 = 0;       
      iBefore_x1 = 0;
      iI_y0 = 0;       
      iBefore_y0 = 0;
      iI_y1 = 0;       
      iBefore_y1 = 0;
      lcd_pattern = 20;
      pattern = 101;
      tx_pattern = 202;
      rx_pattern = 0;
    }
   
  } else if (M5.BtnC.wasPressed()) {  

  }
} 

// ServoControl
//------------------------------------------------------------------//
void servoControlX0(float ang, bool range){
  float i;
  int iRet;
  
  if(interrupt_flag_x0) {
    if(range) {
      i = ang - angle_can_x0;
      iP_x0 = agg_kp * i;
      iI_x0 += (float)agg_ki / 1000 * i;
      iD_x0 = agg_kd * (angle_can_x0 - iBefore_x0) * 1000;
      iRet = iP_x0 + iI_x0 + iD_x0;
      iBefore_x0 = angle_can_x0;
      if( iRet > 20000 ) iRet = 20000;
      if( iRet < -20000 ) iRet = -20000;
      servo_x0_output = iRet;  
      interrupt_flag_x0 = false;
    } else {
      i = ang - angle_can_x0;
      iP_x0 = cons_kp * i;
      iI_x0 += (float)cons_ki / 1000 * i;
      iD_x0 = cons_kd * (angle_can_x0 - iBefore_x0) * 1000;
      iRet = iP_x0 + iI_x0 + iD_x0;
      iBefore_x0 = angle_can_x0;
      if( iRet > 10000 ) iRet = 10000;
      if( iRet < -10000 ) iRet = -10000;
      servo_x0_output = iRet;  
      interrupt_flag_x0 = false;
    }
  }

}

// ServoControl
//------------------------------------------------------------------//
void servoControlX1(float ang, bool range){
  float i;
  int iRet;
  
  if(interrupt_flag_x1) {
    if(range) {
      i = ang - angle_can_x1;
      iP_x1 = agg_kp * i;
      iI_x1 += (float)agg_ki / 1000 * i;
      iD_x1 = agg_kd * (angle_can_x1 - iBefore_x1) * 1000;
      iRet = iP_x1 + iI_x1 + iD_x1;
      iBefore_x1 = angle_can_x1;
      if( iRet > 20000 ) iRet = 20000;
      if( iRet < -20000 ) iRet = -20000;
      servo_x1_output = iRet;  
      interrupt_flag_x1 = false;
    } else {
      i = ang - angle_can_x1;
      iP_x1 = cons_kp * i;
      iI_x1 += (float)cons_ki / 1000 * i;
      iD_x1 = cons_kd * (angle_can_x1 - iBefore_x1) * 1000;
      iRet = iP_x1 + iI_x1 + iD_x1;
      iBefore_x1 = angle_can_x1;
      if( iRet > 10000 ) iRet = 10000;
      if( iRet < -10000 ) iRet = -10000;
      servo_x1_output = iRet;  
      interrupt_flag_x1 = false;
    }
  }
}

// ServoControl
//------------------------------------------------------------------//
void servoControlY0(float ang, bool range){
  float i;
  int iRet;
  
  if(interrupt_flag_y0) {
    if(range) {
      i = ang - angle_can_y0;
      iP_y0 = agg_kp * i;
      iI_y0 += (float)agg_ki / 1000 * i;
      iD_y0 = agg_kd * (angle_can_y0 - iBefore_y0) * 1000;
      iRet = iP_y0 + iI_y0 + iD_y0;
      iBefore_y0 = angle_can_y0;
      if( iRet > 10000 ) iRet = 10000;
      if( iRet < -10000 ) iRet = -10000;
      servo_y0_output = iRet;  
      interrupt_flag_y0 = false;
    } else {
      i = ang - angle_can_y0;
      iP_y0 = cons_kp * i;
      iI_y0 += (float)cons_ki / 1000 * i;
      iD_y0 = cons_kd * (angle_can_y0 - iBefore_y0) * 1000;
      iRet = iP_y0 + iI_y0 + iD_y0;
      iBefore_y0 = angle_can_y0;
      if( iRet > 10000 ) iRet = 10000;
      if( iRet < -10000 ) iRet = -10000;
      servo_y0_output = iRet;  
      interrupt_flag_y0 = false;
    }    
  }
}

// ServoControl
//------------------------------------------------------------------//
void servoControlY1(float ang, bool range){
  float i;
  int iRet;
  
  if(interrupt_flag_y1) {
    if(range) {
      i = angle_can_y1 - ang;
      iP_y1 = agg_kp * i;
      iI_y1 += (float)agg_ki / 1000 * i;
      iD_y1 = agg_kd * (angle_can_y1 - iBefore_y1) * 1000;
      iRet = iP_y1 + iI_y1 + iD_y1;
      iBefore_y1 = angle_can_y1;
      if( iRet > 10000 ) iRet = 10000;
      if( iRet < -10000 ) iRet = -10000;
      servo_y1_output = iRet;  
      interrupt_flag_y1 = false;
    } else {
      i = angle_can_y1 - ang;
      iP_y1 = cons_kp * i;
      iI_y1 += (float)cons_ki / 1000 * i;
      iD_y1 = cons_kd * (angle_can_y1 - iBefore_y1) * 1000;
      iRet = iP_y1 + iI_y1 + iD_y1;
      iBefore_y1 = angle_can_y1;
      if( iRet > 10000 ) iRet = 10000;
      if( iRet < -10000 ) iRet = -10000;
      servo_y1_output = iRet;  
      interrupt_flag_y1 = false;
    }
  }
}

// TargetAngle Calculate
//------------------------------------------------------------------//
void targetAngleX0(float tang, float step) {

  if (interrupt_flag2_x0) {
    if(angle_can_x0 > (tang + ANGLE_TOLERANCE)) {
      initial_angle_x0 -= step;
      target_angle_x0 = initial_angle_x0;
    } else if(angle_can_x0 < (tang - ANGLE_TOLERANCE)
    ) {
      initial_angle_x0 += step;
      target_angle_x0 = initial_angle_x0;
    } else {
      target_angle_x0 = tang;
    }
    interrupt_flag2_x0 = false;
  }
}

// TargetAngle Calculate
//------------------------------------------------------------------//
void targetAngleX1(float tang, float step) {

  if (interrupt_flag2_x1) {
    if(angle_can_x1 > (tang + ANGLE_TOLERANCE)) {
      initial_angle_x1 -= step;
      target_angle_x1 = initial_angle_x1;
    } else if(angle_can_x1 < (tang - ANGLE_TOLERANCE)
    ) {
      initial_angle_x1 += step;
      target_angle_x1 = initial_angle_x1;
    } else {
      target_angle_x1 = tang;
    }
    interrupt_flag2_x1 = false;
  }
}

// TargetAngle Calculate
//------------------------------------------------------------------//
void targetAngleY0(float tang, float step) {

  if (interrupt_flag2_y0) {
    if(angle_can_y0 > (tang + ANGLE_TOLERANCE)) {
      initial_angle_y0 -= step;
      target_angle_y0 = initial_angle_y0;
    } else if(angle_can_y0 < (tang - ANGLE_TOLERANCE)
    ) {
      initial_angle_y0 += step;
      target_angle_y0 = initial_angle_y0;
    } else {
      target_angle_y0 = tang;
    }
    interrupt_flag2_y0 = false;
  }
}

// TargetAngle Calculate
//------------------------------------------------------------------//
void targetAngleY1(float tang, float step) {

  if (interrupt_flag2_y1) {
    if(angle_can_y1 > (tang + ANGLE_TOLERANCE)) {
      initial_angle_y1 -= step;
      target_angle_y1 = initial_angle_y1;
    } else if(angle_can_y1 < (tang - ANGLE_TOLERANCE)
    ) {
      initial_angle_y1 += step;
      target_angle_y1 = initial_angle_y1;
    } else {
      target_angle_y1 = tang;
    }
    interrupt_flag2_y1 = false;
  }
}

// StepAngle Calculate
//------------------------------------------------------------------//
void stepAngleY(float sang) {

  if(millis() - millis_buffer > STABLE_TIME - LOG_TIME) {
    tel_flag = true;
  }
  if(millis() - millis_buffer > STABLE_TIME) {
    tel_flag = false;
    millis_buffer = millis();    
    if( step_angle_y <= sang && (pattern == 14 || pattern == 25)) step_angle_y+=0.5;
    if( step_angle_y >= sang && (pattern == 15 || pattern == 24)) step_angle_y-=0.5;
  }

}

// Initialize CAN
//------------------------------------------------------------------//
void init_can(void){

  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ);

  CAN0.setMode(MCP_NORMAL); // Set operation mode to normal so the MCP2515 sends acks to received data.

  pinMode(CAN0_INT, INPUT); // Configuring pin for /INT input

}

// Test CAN
//------------------------------------------------------------------//
void test_can(void){

  byte angle_H;
  byte angle_L;
  byte velocity_H;
  byte velocity_L;
  byte torque_H;
  byte torque_L;
  byte temp_L;

  int angle_buff;
  int torque_buff;

  float sum = 0;  

  data[0] = power_x1 >> 8 & 0xFF;
  data[1] = power_x1 & 0xFF;
  data[2] = power_x0 >> 8 & 0xFF;
  data[3] = power_x0 & 0xFF;
  data[4] = power_y0 >> 8 & 0xFF;
  data[5] = power_y0 & 0xFF;
  data[6] = power_y1 >> 8 & 0xFF;
  data[7] = power_y1 & 0xFF;
  CAN0.sendMsgBuf(0x1FF, 0, 8, data);

  if(!digitalRead(CAN0_INT)) // If CAN0_INT pin is low, read receive buffer
  {
    CAN0.readMsgBuf(&rxId, &len, rxBuf); // Read data: len = data length, buf = data byte(s)

    for(byte i = 0; i<len; i++){
      switch (i) {
      case 0:
        angle_H = rxBuf[i];
        break;
      case 1:
        angle_L = rxBuf[i];
        break;
      case 2:
        velocity_H = rxBuf[i];
        break;
      case 3:
        velocity_L = rxBuf[i];
        break;
      case 4:
        torque_H = rxBuf[i];
        break;
      case 5:
        torque_L = rxBuf[i];
        break;
      case 6:
        temp_L = rxBuf[i];
        break;
      case 7:
        break;
      }
    }
    angle_buff = ((angle_H << 8) | angle_L & 0xFF);
    torque_buff = ((torque_H << 8) | torque_L & 0xFF)*741/1000;
    if( rxId == 0x206 ) {  
      angle_can_x0 = (float)angle_buff * 360/8192-180;
      if(angle_can_x0 > (angle_averating_x0+ANGLE_IGNORE_THRESHOLD) && angle_can_x0 < (angle_averating_x0-ANGLE_IGNORE_THRESHOLD)) {
        angle_averating_buffer_x0.push(angle_can_x0);
      } else {
        angle_averating_buffer_x0.push(angle_averating_x0);
      }
      for(int i=0; i<ANGLE_AVERATING; i++) {
        sum += angle_can_x0;
      }
      angle_averating_x0 = sum / ANGLE_AVERATING;     
      velocity_can_x0 = ((velocity_H << 8) | velocity_L & 0xFF);     
      torque_can_x0 = float(torque_buff) / 10;
      temp_can_x0 = temp_L; 
    }
    if( rxId == 0x205 ) {  
      angle_can_x1 = (float)angle_buff * 360/8192-180;
      if(angle_can_x1 > (angle_averating_x1+ANGLE_IGNORE_THRESHOLD) && angle_can_x1 < (angle_averating_x1-ANGLE_IGNORE_THRESHOLD)) {
        angle_averating_buffer_x1.push(angle_can_x1);
      } else {
        angle_averating_buffer_x1.push(angle_averating_x1);
      }
      for(int i=0; i<ANGLE_AVERATING; i++) {
        sum += angle_can_x1;
      }
      angle_averating_x1 = sum / ANGLE_AVERATING;    
      velocity_can_x1 = ((velocity_H << 8) | velocity_L & 0xFF);     
      torque_can_x1 = float(torque_buff) / 10;
      temp_can_x1 = temp_L; 
    }
    if( rxId == 0x207 ) {  
      angle_can_y0 = (float)angle_buff * 360/8192-180-1;
      if(angle_can_y0 > (angle_averating_y0+ANGLE_IGNORE_THRESHOLD) && angle_can_y0 < (angle_averating_y0-ANGLE_IGNORE_THRESHOLD)) {
        angle_averating_buffer_y0.push(angle_can_y0);
      } else {
        angle_averating_buffer_y0.push(angle_averating_y0);
      }
      for(int i=0; i<ANGLE_AVERATING; i++) {
        sum += angle_can_y0;
      }
      angle_averating_y0 = sum / ANGLE_AVERATING;    
      velocity_can_y0 = ((velocity_H << 8) | velocity_L & 0xFF);     
      torque_can_y0 = float(torque_buff) / 10;
      temp_can_y0 = temp_L; 
    }
    if( rxId == 0x208 ) {  
      angle_can_y1 = (float)angle_buff*-1 * 360/8192+180.8;
      if(angle_can_y1 > (angle_averating_y1+ANGLE_IGNORE_THRESHOLD) && angle_can_y1 < (angle_averating_y1-ANGLE_IGNORE_THRESHOLD)) {
        angle_averating_buffer_y1.push(angle_can_y1);
      } else {
        angle_averating_buffer_y1.push(angle_averating_y1);
      }
      for(int i=0; i<ANGLE_AVERATING; i++) {
        sum += angle_can_y1;
      }
      angle_averating_y1 = sum / ANGLE_AVERATING;    
      velocity_can_y1 = ((velocity_H << 8) | velocity_L & 0xFF);     
      torque_can_y1 = float(torque_buff) / 10;
      temp_can_y1 = temp_L; 
    }
  }
}

//AE HX711 Init
//------------------------------------------------------------------//
void AE_HX711_Init(void)
{
  pinMode(HX711_SCLK_X0, OUTPUT);
  pinMode(HX711_SCLK_X1, OUTPUT);
  pinMode(HX711_SCLK_Y0, OUTPUT);
  pinMode(HX711_SCLK_Y1, OUTPUT);
  pinMode(HX711_DOUT_X0, INPUT);
  pinMode(HX711_DOUT_X1, INPUT);
  pinMode(HX711_DOUT_Y0, INPUT);
  pinMode(HX711_DOUT_Y1, INPUT);
}

//AE HX711 Reset
//------------------------------------------------------------------//
void AE_HX711_Reset(void)
{
  digitalWrite(HX711_SCLK_X0,1);
  digitalWrite(HX711_SCLK_X1,1);
  digitalWrite(HX711_SCLK_Y0,1);
  digitalWrite(HX711_SCLK_Y1,1);
  delayMicroseconds(100);
  digitalWrite(HX711_SCLK_X0,0);
  digitalWrite(HX711_SCLK_X1,0);
  digitalWrite(HX711_SCLK_Y0,0);
  digitalWrite(HX711_SCLK_Y1,0);
  delayMicroseconds(100); 
}

//AE HX711 Read
//------------------------------------------------------------------//
void AE_HX711_Read(void)
{
  long data=0;
  int sum = 0;  
  if( hx711_flag ) {
    switch (hx711_pattern) {
    // X0
    case 0:
      digitalWrite(ledPin, led_flag);  
      led_flag = !led_flag;  
      while(digitalRead(HX711_DOUT_X0)!=0);
      delayMicroseconds(1);
      for(int i=0;i<24;i++)
      {
        digitalWrite(HX711_SCLK_X0,1);
        delayMicroseconds(1);
        digitalWrite(HX711_SCLK_X0,0);
        delayMicroseconds(1);
        data = (data<<1)|(digitalRead(HX711_DOUT_X0));
      }  
      digitalWrite(HX711_SCLK_X0,1);
      delayMicroseconds(1);
      digitalWrite(HX711_SCLK_X0,0);
      delayMicroseconds(1);

      native_data_x0 = (data^0x800000) - averating_data_x0;      
      buffer_x0.push(native_data_x0 - native_data_x0_buffer);
      native_data_x0_buffer = native_data_x0;
      for(int i=0; i<AVERATING_BUFFER_SIZE; i++) {
        sum += buffer_x0[i];
      }
      level_x0 = sum / AVERATING_BUFFER_SIZE;

      torque_x0 = native_data_x0 * coefficient_x0 / 1000;
      hx711_pattern = 1;
      hx711_flag = false;
      digitalWrite(ledPin, led_flag);  
      led_flag = !led_flag;  
      break;

    // X1
    case 1:
      digitalWrite(ledPin, led_flag);  
      led_flag = !led_flag;  
      while(digitalRead(HX711_DOUT_X1)!=0);
      delayMicroseconds(1);
      for(int i=0;i<24;i++)
      {
        digitalWrite(HX711_SCLK_X1,1);
        delayMicroseconds(1);
        digitalWrite(HX711_SCLK_X1,0);
        delayMicroseconds(1);
        data = (data<<1)|(digitalRead(HX711_DOUT_X1));
      }  
      digitalWrite(HX711_SCLK_X1,1);
      delayMicroseconds(1);
      digitalWrite(HX711_SCLK_X1,0);
      delayMicroseconds(1);

      native_data_x1 = (data^0x800000) - averating_data_x1;      
      buffer_x1.push(native_data_x1 - native_data_x1_buffer);
      native_data_x1_buffer = native_data_x1;
      for(int i=0; i<AVERATING_BUFFER_SIZE; i++) {
        sum += buffer_x1[i];
      }
      level_x1 = sum / AVERATING_BUFFER_SIZE;

      torque_x1 = native_data_x1 * coefficient_x1 / 1000;
      hx711_pattern = 2;
      hx711_flag = false;
      digitalWrite(ledPin, led_flag);  
      led_flag = !led_flag;  
      break;

    // Y0
    case 2:
      digitalWrite(ledPin, led_flag);  
      led_flag = !led_flag;  
      while(digitalRead(HX711_DOUT_Y0)!=0);
      delayMicroseconds(1);
      for(int i=0;i<24;i++)
      {
        digitalWrite(HX711_SCLK_Y0,1);
        delayMicroseconds(1);
        digitalWrite(HX711_SCLK_Y0,0);
        delayMicroseconds(1);
        data = (data<<1)|(digitalRead(HX711_DOUT_Y0));
      }  
      digitalWrite(HX711_SCLK_Y0,1);
      delayMicroseconds(1);
      digitalWrite(HX711_SCLK_Y0,0);
      delayMicroseconds(1);

      native_data_y0 = (data^0x800000) - averating_data_y0;      
      buffer_y0.push(native_data_y0 - native_data_y0_buffer);
      native_data_y0_buffer = native_data_y0;
      for(int i=0; i<AVERATING_BUFFER_SIZE; i++) {
        sum += buffer_y0[i];
      }
      level_y0 = sum / AVERATING_BUFFER_SIZE;

      torque_y0 = native_data_y0 * coefficient_y0 / 1000;
      hx711_pattern = 3;
      hx711_flag = false;
      digitalWrite(ledPin, led_flag);  
      led_flag = !led_flag;  
      break;

    // Y1
    case 3:
      digitalWrite(ledPin, led_flag);  
      led_flag = !led_flag;  
      while(digitalRead(HX711_DOUT_Y1)!=0);
      delayMicroseconds(1);
      for(int i=0;i<24;i++)
      {
        digitalWrite(HX711_SCLK_Y1,1);
        delayMicroseconds(1);
        digitalWrite(HX711_SCLK_Y1,0);
        delayMicroseconds(1);
        data = (data<<1)|(digitalRead(HX711_DOUT_Y1));
      }  
      digitalWrite(HX711_SCLK_Y1,1);
      delayMicroseconds(1);
      digitalWrite(HX711_SCLK_Y1,0);
      delayMicroseconds(1);

      native_data_y1 = (data^0x800000) - averating_data_y1;      
      buffer_y1.push(native_data_y1 - native_data_y1_buffer);
      native_data_y1_buffer = native_data_y1;
      for(int i=0; i<AVERATING_BUFFER_SIZE; i++) {
        sum += buffer_y1[i];
      }
      level_y1 = sum / AVERATING_BUFFER_SIZE;

      torque_y1 = native_data_y1 * coefficient_y1 / 1000;
      hx711_pattern = 0;
      hx711_flag = false;
      digitalWrite(ledPin, led_flag);  
      led_flag = !led_flag;  
      break;
    }
  }
}

// Get Offset Value from HX711
//------------------------------------------------------------------//
void getOffsetValue(void) {

  averating_data_x0_buffer = 0;
  averating_data_x0 = 0;
  averating_data_x1_buffer = 0;
  averating_data_x1 = 0;
  averating_data_y0_buffer = 0;
  averating_data_y0 = 0;
  averating_data_y1_buffer = 0;
  averating_data_y1 = 0;
  millis_buffer = millis();

  lcd_pattern = 10;
  
  for(int i=0; i<INITIALIZEING_SAMPLE; i++) {
    progress_value = i;
    millis_buffer = millis();
    averating_data_x0_buffer += native_data_x0;       
    if(i==INITIALIZEING_SAMPLE-1) {
      Serial.printf(" Getting Offset Value X0 [##########]\n");      
    } else if(i>=0.9*INITIALIZEING_SAMPLE) {
      Serial.printf(" Getting Offset Value X0 [######### ]\r");  
    } else if(i>=0.8*INITIALIZEING_SAMPLE) {
      Serial.printf(" Getting Offset Value X0 [########  ]\r");  
    } else if(i>=0.7*INITIALIZEING_SAMPLE) {
      Serial.printf(" Getting Offset Value X0 [#######   ]\r");  
    } else if(i>=0.6*INITIALIZEING_SAMPLE) {
      Serial.printf(" Getting Offset Value X0 [######    ]\r");  
    } else if(i>=0.5*INITIALIZEING_SAMPLE) {
      Serial.printf(" Getting Offset Value X0 [#####     ]\r");  
    } else if(i>=0.4*INITIALIZEING_SAMPLE) {
      Serial.printf(" Getting Offset Value X0 [####      ]\r");  
    } else if(i>=0.3*INITIALIZEING_SAMPLE) {
      Serial.printf(" Getting Offset Value X0 [###       ]\r");  
    } else if(i>=0.2*INITIALIZEING_SAMPLE) {
      Serial.printf(" Getting Offset Value X0 [##        ]\r");  
    } else if(i>=0.1*INITIALIZEING_SAMPLE) {
      Serial.printf(" Getting Offset Value X0 [#         ]\r");  
    } else {
      Serial.printf(" Getting Offset Value X0 [          ]\r");  
    }  
    while( millis() - millis_buffer < 15);
  }  
  averating_data_x0 = averating_data_x0_buffer / INITIALIZEING_SAMPLE;
  Serial.printf(" X0 offset value = %d\n\n", averating_data_x0);  

  while(lcd_flag);
  M5.Lcd.fillRect(20,150,280,10,LCD_MAIN);
  lcd_pattern = 12;

  for(int i=0; i<INITIALIZEING_SAMPLE; i++) {
    progress_value = i;
    millis_buffer = millis();
    averating_data_x1_buffer += native_data_x1;   
    if(i==INITIALIZEING_SAMPLE-1) {
      Serial.printf(" Getting Offset Value X1 [##########]\n");      
    } else if(i>=0.9*INITIALIZEING_SAMPLE) {
      Serial.printf(" Getting Offset Value X1 [######### ]\r");  
    } else if(i>=0.8*INITIALIZEING_SAMPLE) {
      Serial.printf(" Getting Offset Value X1 [########  ]\r");  
    } else if(i>=0.7*INITIALIZEING_SAMPLE) {
      Serial.printf(" Getting Offset Value X1 [#######   ]\r");  
    } else if(i>=0.6*INITIALIZEING_SAMPLE) {
      Serial.printf(" Getting Offset Value X1 [######    ]\r");  
    } else if(i>=0.5*INITIALIZEING_SAMPLE) {
      Serial.printf(" Getting Offset Value X1 [#####     ]\r");  
    } else if(i>=0.4*INITIALIZEING_SAMPLE) {
      Serial.printf(" Getting Offset Value X1 [####      ]\r");  
    } else if(i>=0.3*INITIALIZEING_SAMPLE) {
      Serial.printf(" Getting Offset Value X1 [###       ]\r");  
    } else if(i>=0.2*INITIALIZEING_SAMPLE) {
      Serial.printf(" Getting Offset Value X1 [##        ]\r");  
    } else if(i>=0.1*INITIALIZEING_SAMPLE) {
      Serial.printf(" Getting Offset Value X1 [#         ]\r");  
    } else {
      Serial.printf(" Getting Offset Value X1 [          ]\r");  
    }  
    while( millis() - millis_buffer < 15);
  }  
  averating_data_x1 = averating_data_x1_buffer / INITIALIZEING_SAMPLE;
  Serial.printf(" X1 offset value = %d\n\n", averating_data_x1); 

  while(lcd_flag);
  M5.Lcd.fillRect(20,150,280,10,LCD_MAIN);
  lcd_pattern = 13; 

  for(int i=0; i<INITIALIZEING_SAMPLE; i++) {
    progress_value = i;
    millis_buffer = millis();
    averating_data_y0_buffer += native_data_y0;   
    if(i==INITIALIZEING_SAMPLE-1) {
      Serial.printf(" Getting Offset Value Y0 [##########]\n");      
    } else if(i>=0.9*INITIALIZEING_SAMPLE) {
      Serial.printf(" Getting Offset Value Y0 [######### ]\r");  
    } else if(i>=0.8*INITIALIZEING_SAMPLE) {
      Serial.printf(" Getting Offset Value Y0 [########  ]\r");  
    } else if(i>=0.7*INITIALIZEING_SAMPLE) {
      Serial.printf(" Getting Offset Value Y0 [#######   ]\r");  
    } else if(i>=0.6*INITIALIZEING_SAMPLE) {
      Serial.printf(" Getting Offset Value Y0 [######    ]\r");  
    } else if(i>=0.5*INITIALIZEING_SAMPLE) {
      Serial.printf(" Getting Offset Value Y0 [#####     ]\r");  
    } else if(i>=0.4*INITIALIZEING_SAMPLE) {
      Serial.printf(" Getting Offset Value Y0 [####      ]\r");  
    } else if(i>=0.3*INITIALIZEING_SAMPLE) {
      Serial.printf(" Getting Offset Value Y0 [###       ]\r");  
    } else if(i>=0.2*INITIALIZEING_SAMPLE) {
      Serial.printf(" Getting Offset Value Y0 [##        ]\r");  
    } else if(i>=0.1*INITIALIZEING_SAMPLE) {
      Serial.printf(" Getting Offset Value Y0 [#         ]\r");  
    } else {
      Serial.printf(" Getting Offset Value Y0 [          ]\r");  
    }  
    while( millis() - millis_buffer < 15);
  }  
  averating_data_y0 = averating_data_y0_buffer / INITIALIZEING_SAMPLE;
  Serial.printf(" Y0 offset value = %d\n\n", averating_data_y0);  

  while(lcd_flag);
  M5.Lcd.fillRect(20,150,280,10,LCD_MAIN);
  lcd_pattern = 14;

  for(int i=0; i<INITIALIZEING_SAMPLE; i++) {
    progress_value = i;
    millis_buffer = millis();
    averating_data_y1_buffer += native_data_y1;  
    
    if(i==INITIALIZEING_SAMPLE-1) {
      Serial.printf(" Getting Offset Value Y1 [##########]\n");      
    } else if(i>=0.9*INITIALIZEING_SAMPLE) {
      Serial.printf(" Getting Offset Value Y1 [######### ]\r");  
    } else if(i>=0.8*INITIALIZEING_SAMPLE) {
      Serial.printf(" Getting Offset Value Y1 [########  ]\r");  
    } else if(i>=0.7*INITIALIZEING_SAMPLE) {
      Serial.printf(" Getting Offset Value Y1 [#######   ]\r");  
    } else if(i>=0.6*INITIALIZEING_SAMPLE) {
      Serial.printf(" Getting Offset Value Y1 [######    ]\r");  
    } else if(i>=0.5*INITIALIZEING_SAMPLE) {
      Serial.printf(" Getting Offset Value Y1 [#####     ]\r");  
    } else if(i>=0.4*INITIALIZEING_SAMPLE) {
      Serial.printf(" Getting Offset Value Y1 [####      ]\r");  
    } else if(i>=0.3*INITIALIZEING_SAMPLE) {
      Serial.printf(" Getting Offset Value Y1 [###       ]\r");  
    } else if(i>=0.2*INITIALIZEING_SAMPLE) {
      Serial.printf(" Getting Offset Value Y1 [##        ]\r");  
    } else if(i>=0.1*INITIALIZEING_SAMPLE) {
      Serial.printf(" Getting Offset Value Y1 [#         ]\r");  
    } else {
      Serial.printf(" Getting Offset Value Y1 [          ]\r");  
    }  
    while( millis() - millis_buffer < 15);
  }  
  averating_data_y1 = averating_data_y1_buffer / INITIALIZEING_SAMPLE;
  Serial.printf(" Y1 offset value = %d\n\n", averating_data_y1);  

  while(lcd_flag);
  M5.Lcd.fillRect(20,150,280,10,LCD_MAIN);
  lcd_pattern = 15;

  Serial.printf(" Initialization complete\n");  

  if(pattern == 201) {
    tx_pattern = 1;
  } else {
    tx_pattern = 101;
  }
  

}

// Calibration X0
//------------------------------------------------------------------//
void getCalibrationDataX0(void) {

  for(int j=0; j<11; j++) {
    if(j==0) Serial.printf(" Prepare for the X0 axis calibration and press the ButtonA.\n"); 
    if(j==1) Serial.printf(" Load 1 coin and press button A.\n"); 
    if(j==2) Serial.printf(" Load 2 coins and press button A.\n"); 
    if(j==3) Serial.printf(" Load 3 coins and press button A.\n"); 
    if(j==4) Serial.printf(" Load 4 coins and press button A.\n"); 
    if(j==5) Serial.printf(" Load 5 coins and press button A.\n"); 
    if(j==6) Serial.printf(" Load 6 coins and press button A.\n"); 
    if(j==7) Serial.printf(" Load 7 coins and press button A.\n"); 
    if(j==8) Serial.printf(" Load 8 coins and press button A.\n"); 
    if(j==9) Serial.printf(" Load 9 coins and press button A.\n"); 
    if(j==10) Serial.printf(" Load 10 coins and press button A.\n"); 
    while(!M5.BtnA.isPressed());
    averating_data_x0_buffer = 0;
    for(int i=0; i<INITIALIZEING_SAMPLE; i++) {
      progress_value = i;
      millis_buffer = millis();
      averating_data_x0_buffer += native_data_x0; 
      if(i==INITIALIZEING_SAMPLE-1) {
        Serial.printf(" Getting Calibration Facotr X0 [##########]\n\n");      
      } else if(i>=0.9*INITIALIZEING_SAMPLE) {
        Serial.printf(" Getting Calibration Facotr X0 [######### ]\r");  
      } else if(i>=0.8*INITIALIZEING_SAMPLE) {
        Serial.printf(" Getting Calibration Facotr X0 [########  ]\r");  
      } else if(i>=0.7*INITIALIZEING_SAMPLE) {
        Serial.printf(" Getting Calibration Facotr X0 [#######   ]\r");  
      } else if(i>=0.6*INITIALIZEING_SAMPLE) {
        Serial.printf(" Getting Calibration Facotr X0 [######    ]\r");  
      } else if(i>=0.5*INITIALIZEING_SAMPLE) {
        Serial.printf(" Getting Calibration Facotr X0 [#####     ]\r");  
      } else if(i>=0.4*INITIALIZEING_SAMPLE) {
        Serial.printf(" Getting Calibration Facotr X0 [####      ]\r");  
      } else if(i>=0.3*INITIALIZEING_SAMPLE) {
        Serial.printf(" Getting Calibration Facotr X0 [###       ]\r");  
      } else if(i>=0.2*INITIALIZEING_SAMPLE) {
        Serial.printf(" Getting Calibration Facotr X0 [##        ]\r");  
      } else if(i>=0.1*INITIALIZEING_SAMPLE) {
        Serial.printf(" Getting Calibration Facotr X0 [#         ]\r");  
      } else {
        Serial.printf(" Getting Calibration Facotr X0 [          ]\r");  
      }  
      while( millis() - millis_buffer < CALIBRATION_PERIOD); 
    }
    calibration_factor_x0[j] = averating_data_x0_buffer / INITIALIZEING_SAMPLE;  
  }

  for (int i=0; i<11; i++) {
    Serial.printf(" %d, %f\n", calibration_factor_x0[i], load_step_x[i]); 
  }

  float sum_xy = 0, sum_x = 0, sum_y = 0, sum_x2 = 0;

  for (int i=0; i<11; i++) {
    sum_xy += calibration_factor_x0[i] * load_step_x[i];
    sum_x += calibration_factor_x0[i];
    sum_y += load_step_x[i];
    sum_x2 += pow(calibration_factor_x0[i], 2);
  }
  
  coefficient_x0 = (11 * sum_xy - sum_x * sum_y) / (11 * sum_x2 - pow(sum_x, 2));
  intercept_x0 = (sum_x2 * sum_y - sum_xy * sum_x) / (11 * sum_x2 - pow(sum_x, 2));

  coefficient_x0_eeprom = coefficient_x0 * 1000000;
  intercept_x0_eeprom = intercept_x0 * 1000;

  eeprom_write();

  Serial.printf(" Coefficient = %f, Intercept = %f\n\n", coefficient_x0, intercept_x0);    
  tx_pattern = 1;  
  
}

// Calibration X1
//------------------------------------------------------------------//
void getCalibrationDataX1(void) {

  for(int j=0; j<11; j++) {
    if(j==0) Serial.printf(" Prepare for the X1 axis calibration and press the ButtonA.\n"); 
    if(j==1) Serial.printf(" Load 1 coin and press button A.\n"); 
    if(j==2) Serial.printf(" Load 2 coins and press button A.\n"); 
    if(j==3) Serial.printf(" Load 3 coins and press button A.\n"); 
    if(j==4) Serial.printf(" Load 4 coins and press button A.\n"); 
    if(j==5) Serial.printf(" Load 5 coins and press button A.\n"); 
    if(j==6) Serial.printf(" Load 6 coins and press button A.\n"); 
    if(j==7) Serial.printf(" Load 7 coins and press button A.\n"); 
    if(j==8) Serial.printf(" Load 8 coins and press button A.\n"); 
    if(j==9) Serial.printf(" Load 9 coins and press button A.\n"); 
    if(j==10) Serial.printf(" Load 10 coins and press button A.\n"); 
    while(!M5.BtnA.isPressed());
    averating_data_x1_buffer = 0;
    for(int i=0; i<INITIALIZEING_SAMPLE; i++) {
      progress_value = i;
      millis_buffer = millis();
      averating_data_x1_buffer += native_data_x1; 
      if(i==INITIALIZEING_SAMPLE-1) {
        Serial.printf(" Getting Calibration Facotr X1 [##########]\n\n");      
      } else if(i>=0.9*INITIALIZEING_SAMPLE) {
        Serial.printf(" Getting Calibration Facotr X1 [######### ]\r");  
      } else if(i>=0.8*INITIALIZEING_SAMPLE) {
        Serial.printf(" Getting Calibration Facotr X1 [########  ]\r");  
      } else if(i>=0.7*INITIALIZEING_SAMPLE) {
        Serial.printf(" Getting Calibration Facotr X1 [#######   ]\r");  
      } else if(i>=0.6*INITIALIZEING_SAMPLE) {
        Serial.printf(" Getting Calibration Facotr X1 [######    ]\r");  
      } else if(i>=0.5*INITIALIZEING_SAMPLE) {
        Serial.printf(" Getting Calibration Facotr X1 [#####     ]\r");  
      } else if(i>=0.4*INITIALIZEING_SAMPLE) {
        Serial.printf(" Getting Calibration Facotr X1 [####      ]\r");  
      } else if(i>=0.3*INITIALIZEING_SAMPLE) {
        Serial.printf(" Getting Calibration Facotr X1 [###       ]\r");  
      } else if(i>=0.2*INITIALIZEING_SAMPLE) {
        Serial.printf(" Getting Calibration Facotr X1 [##        ]\r");  
      } else if(i>=0.1*INITIALIZEING_SAMPLE) {
        Serial.printf(" Getting Calibration Facotr X1 [#         ]\r");  
      } else {
        Serial.printf(" Getting Calibration Facotr X1 [          ]\r");  
      }  
      while( millis() - millis_buffer < CALIBRATION_PERIOD); 
    }
    calibration_factor_x1[j] = averating_data_x1_buffer / INITIALIZEING_SAMPLE;  
  }

  for (int i=0; i<11; i++) {
    Serial.printf(" %d, %f\n", calibration_factor_x1[i], load_step_x[i]); 
  }

  float sum_xy = 0, sum_x = 0, sum_y = 0, sum_x2 = 0;

  for (int i=0; i<11; i++) {
    sum_xy += calibration_factor_x1[i] * load_step_x[i];
    sum_x += calibration_factor_x1[i];
    sum_y += load_step_x[i];
    sum_x2 += pow(calibration_factor_x1[i], 2);
  }
  
  coefficient_x1 = (11 * sum_xy - sum_x * sum_y) / (11 * sum_x2 - pow(sum_x, 2));
  intercept_x1 = (sum_x2 * sum_y - sum_xy * sum_x) / (11 * sum_x2 - pow(sum_x, 2));

  coefficient_x1_eeprom = coefficient_x1 * 1000000;
  intercept_x1_eeprom = intercept_x1 * 1000;

  eeprom_write();

  Serial.printf(" Coefficient = %f, Intercept = %f\n\n", coefficient_x1, intercept_x1);    
  tx_pattern = 1;  
  
}

// Calibration Y0
//------------------------------------------------------------------//
void getCalibrationDataY0(void) {

  for(int j=0; j<6; j++) {
    if(j==0) Serial.printf(" Prepare for the Y0 axis calibration and press the ButtonA.\n"); 
    if(j==1) Serial.printf(" Load 1 weight and press button A.\n"); 
    if(j==2) Serial.printf(" Load 2 weight and press button A.\n"); 
    if(j==3) Serial.printf(" Load 3 weight and press button A.\n"); 
    if(j==4) Serial.printf(" Load 4 weight and press button A.\n"); 
    if(j==5) Serial.printf(" Load 5 weight and press button A.\n"); 
    while(!M5.BtnA.isPressed());
    averating_data_y0_buffer = 0;
    for(int i=0; i<INITIALIZEING_SAMPLE; i++) {
      progress_value = i;
      millis_buffer = millis();
      averating_data_y0_buffer += native_data_y0; 
      if(i==INITIALIZEING_SAMPLE-1) {
        Serial.printf(" Getting Calibration Facotr Y0 [##########]\n\n");      
      } else if(i>=0.9*INITIALIZEING_SAMPLE) {
        Serial.printf(" Getting Calibration Facotr Y0 [######### ]\r");  
      } else if(i>=0.8*INITIALIZEING_SAMPLE) {
        Serial.printf(" Getting Calibration Facotr Y0 [########  ]\r");  
      } else if(i>=0.7*INITIALIZEING_SAMPLE) {
        Serial.printf(" Getting Calibration Facotr Y0 [#######   ]\r");  
      } else if(i>=0.6*INITIALIZEING_SAMPLE) {
        Serial.printf(" Getting Calibration Facotr Y0 [######    ]\r");  
      } else if(i>=0.5*INITIALIZEING_SAMPLE) {
        Serial.printf(" Getting Calibration Facotr Y0 [#####     ]\r");  
      } else if(i>=0.4*INITIALIZEING_SAMPLE) {
        Serial.printf(" Getting Calibration Facotr Y0 [####      ]\r");  
      } else if(i>=0.3*INITIALIZEING_SAMPLE) {
        Serial.printf(" Getting Calibration Facotr Y0 [###       ]\r");  
      } else if(i>=0.2*INITIALIZEING_SAMPLE) {
        Serial.printf(" Getting Calibration Facotr Y0 [##        ]\r");  
      } else if(i>=0.1*INITIALIZEING_SAMPLE) {
        Serial.printf(" Getting Calibration Facotr Y0 [#         ]\r");  
      } else {
        Serial.printf(" Getting Calibration Facotr Y0 [          ]\r");  
      }  
      while( millis() - millis_buffer < CALIBRATION_PERIOD); 
    }
    calibration_factor_y0[j] = averating_data_y0_buffer / INITIALIZEING_SAMPLE;  
  }

  for (int i=0; i<6; i++) {
    Serial.printf(" %d, %f\n", calibration_factor_y0[i], load_step_y[i]); 
  }

  float sum_xy = 0, sum_x = 0, sum_y = 0, sum_x2 = 0;

  for (int i=0; i<6; i++) {
    sum_xy += calibration_factor_y0[i] * load_step_y[i];
    sum_x += calibration_factor_y0[i];
    sum_y += load_step_y[i];
    sum_x2 += pow(calibration_factor_y0[i], 2);
  }
  
  coefficient_y0 = (6 * sum_xy - sum_x * sum_y) / (6 * sum_x2 - pow(sum_x, 2));
  intercept_y0 = (sum_x2 * sum_y - sum_xy * sum_x) / (6 * sum_x2 - pow(sum_x, 2));

  coefficient_y0_eeprom = coefficient_y0 * 1000000;
  intercept_y0_eeprom = intercept_y0 * 1000;

  eeprom_write();

  Serial.printf(" Coefficient = %f, Intercept = %f\n\n", coefficient_y0, intercept_y0);    
  tx_pattern = 1;  

  
}

// Calibration Y1
//------------------------------------------------------------------//
void getCalibrationDataY1(void) {

  for(int j=0; j<6; j++) {
    if(j==0) Serial.printf(" Prepare for the Y1 axis calibration and press the ButtonA.\n"); 
    if(j==1) Serial.printf(" Load 1 weight and press button A.\n"); 
    if(j==2) Serial.printf(" Load 2 weight and press button A.\n"); 
    if(j==3) Serial.printf(" Load 3 weight and press button A.\n"); 
    if(j==4) Serial.printf(" Load 4 weight and press button A.\n"); 
    if(j==5) Serial.printf(" Load 5 weight and press button A.\n"); 
    while(!M5.BtnA.isPressed());
    averating_data_y1_buffer = 0;
    for(int i=0; i<INITIALIZEING_SAMPLE; i++) {
      progress_value = i;
      millis_buffer = millis();
      averating_data_y1_buffer += native_data_y1; 
      if(i==INITIALIZEING_SAMPLE-1) {
        Serial.printf(" Getting Calibration Facotr Y1 [##########]\n\n");      
      } else if(i>=0.9*INITIALIZEING_SAMPLE) {
        Serial.printf(" Getting Calibration Facotr Y1 [######### ]\r");  
      } else if(i>=0.8*INITIALIZEING_SAMPLE) {
        Serial.printf(" Getting Calibration Facotr Y1 [########  ]\r");  
      } else if(i>=0.7*INITIALIZEING_SAMPLE) {
        Serial.printf(" Getting Calibration Facotr Y1 [#######   ]\r");  
      } else if(i>=0.6*INITIALIZEING_SAMPLE) {
        Serial.printf(" Getting Calibration Facotr Y1 [######    ]\r");  
      } else if(i>=0.5*INITIALIZEING_SAMPLE) {
        Serial.printf(" Getting Calibration Facotr Y1 [#####     ]\r");  
      } else if(i>=0.4*INITIALIZEING_SAMPLE) {
        Serial.printf(" Getting Calibration Facotr Y1 [####      ]\r");  
      } else if(i>=0.3*INITIALIZEING_SAMPLE) {
        Serial.printf(" Getting Calibration Facotr Y1 [###       ]\r");  
      } else if(i>=0.2*INITIALIZEING_SAMPLE) {
        Serial.printf(" Getting Calibration Facotr Y1 [##        ]\r");  
      } else if(i>=0.1*INITIALIZEING_SAMPLE) {
        Serial.printf(" Getting Calibration Facotr Y1 [#         ]\r");  
      } else {
        Serial.printf(" Getting Calibration Facotr Y1 [          ]\r");  
      }  
      while( millis() - millis_buffer < CALIBRATION_PERIOD); 
    }
    calibration_factor_y1[j] = averating_data_y1_buffer / INITIALIZEING_SAMPLE;  
  }

  for (int i=0; i<6; i++) {
    Serial.printf(" %d, %f\n", calibration_factor_y1[i], load_step_y[i]); 
  }

  float sum_xy = 0, sum_x = 0, sum_y = 0, sum_x2 = 0;

  for (int i=0; i<6; i++) {
    sum_xy += calibration_factor_y1[i] * load_step_y[i];
    sum_x += calibration_factor_y1[i];
    sum_y += load_step_y[i];
    sum_x2 += pow(calibration_factor_y1[i], 2);
  }
  
  coefficient_y1 = (6 * sum_xy - sum_x * sum_y) / (6 * sum_x2 - pow(sum_x, 2));
  intercept_y1 = (sum_x2 * sum_y - sum_xy * sum_x) / (6 * sum_x2 - pow(sum_x, 2));

  coefficient_y1_eeprom = coefficient_y1 * 1000000;
  intercept_y1_eeprom = intercept_y1 * 1000;

  eeprom_write();

  Serial.printf(" Coefficient = %f, Intercept = %f\n\n", coefficient_y1, intercept_y1);    
  tx_pattern = 1;  
  
}

//Get Time From NTP
//------------------------------------------------------------------//
void getTimeFromNTP(void){
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  while (!getLocalTime(&timeinfo)) {
    delay(1000);
  }
}

//Get Convert Time
//------------------------------------------------------------------//
void getTime(void){
  getLocalTime(&timeinfo);
  dateStr = (String)(timeinfo.tm_year + 1900)
          + "/" + (String)(timeinfo.tm_mon + 1)
          + "/" + (String)timeinfo.tm_mday;
  timeStr = (String)timeinfo.tm_hour
          + ":" + (String)timeinfo.tm_min
          + ":" + (String)timeinfo.tm_sec;
}

// IRAM
//------------------------------------------------------------------//
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter=1;
  portEXIT_CRITICAL_ISR(&timerMux);
}

