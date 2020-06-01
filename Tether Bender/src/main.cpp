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
#include "driver/pcnt.h"

//Define
//------------------------------------------------------------------//
#define CAN0_INT 15                             // Set INT to pin 15
#define TIMER_INTERRUPT 1

#define HX711_DOUT  5
#define HX711_SCLK  2
#define OUT_VOL     0.0007f
#define LOAD        500.0f

#define AVERATING_BUFFER_SIZE 100
#define INITIALIZEING_SAMPLE 100

//Global
//------------------------------------------------------------------//
TaskHandle_t task_handl;

CircularBuffer<int, AVERATING_BUFFER_SIZE> buffer_x0;

// CAN
MCP_CAN CAN0(12);                               // Set CS to pin 12
byte data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];                        // Array to store serial string

byte angle_H;
byte angle_L;
byte velocity_H;
byte velocity_L;
byte torque_H;
byte torque_L;
byte temp_L;

// Main
volatile int cnt1 = 0;
unsigned short angle_buff;
float angle;
short velocity;
int torque_buff;
float torque;
int8_t temp;
bool sd_insert = false;
unsigned char lcd_pattern = 0;
unsigned char lcd_cnt = 0;
volatile bool  lcd_flag = false;
volatile unsigned int millis_buffer = 0;
bool init_flag = false;

bool x0_flag = false;
bool x1_flag = false;
bool y0_flag = false;
bool y1_flag = false;

int level_x0;
int level_x1;
int level_y0;
int level_y1;

int power1 = 0;
int power2 = 0;
int power3 = 0;
int power4 = 0;

int pattern = 0;
int iP, iD, iI;
int kp = 25;
int ki = 10;
int kd = 100;
float iBefore;

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

// HX711
float hx711_offset;
float hx711_data;
long  native_data_x0;
long  native_data_x0_buffer;
long  averating_data;
long  averating_data_buffer;

// LED 
static const int ledPin = 3;
bool led_flag = false;


// Battery
unsigned char battery_status;
char battery_persent;


// Timer
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile int interruptCounter;
int iTimer10;
int iTimer80;

//Prototype
//------------------------------------------------------------------//
void init_can();
void test_can();
void button_action();
void initLCD(void);
void lcdDisplay(void);
void taskDisplay(void *pvParameters);
void IRAM_ATTR onTimer(void);
void Timer_Interrupt(void);
void servoControl(float ang);
void AE_HX711_Init(void);
void AE_HX711_Reset(void);
long AE_HX711_Read(void);
long AE_HX711_Averaging(long adc,char num);
float AE_HX711_getGram(char num);
void getTimeFromNTP(void);
void getTime(void);
void createLogfile(void);


//Setup #1
//------------------------------------------------------------------//
void setup() {
  M5.begin(1, 1, 0, 1);

  Serial.begin(115200);

  // Initialize Timer Interrupt
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, TIMER_INTERRUPT * 1000, true);
  timerAlarmEnable(timer); 

  M5.Lcd.setTextColor(BLACK);

  pinMode(ledPin, OUTPUT);

  AE_HX711_Init();
  AE_HX711_Reset();
  //hx711_offset = AE_HX711_getGram(30); 

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
  lcd_pattern = 100;

  xTaskCreatePinnedToCore(&taskDisplay, "taskDisplay", 4096, NULL, 1, &task_handl, 0);

   
}

//Main #1
//------------------------------------------------------------------//
void loop() {
  Timer_Interrupt(); 
  //test_can();  

  switch (pattern) {
  case 0:    
    break;
  
  case 11:
    break;

  case 12:
    delay(3000);
    power1 = 1000;
    pattern = 13;
    break;

  case 13:
    if(angle >= 120) {
      power1 = 0;
      pattern = 0;
    }
    break;

  case 101:
    servoControl(0);
    if(velocity > -1 &&velocity <= 1) {
      if(angle > -0.1 && angle < 0.1) {
        pattern = 102;
        iI = 0;
        break;
      }
    }    
    averating_data_buffer = 0;
    averating_data = 0;
    pattern = 102;
    break;
  
  case 102:    
    if( millis() - millis_buffer > 100 ) {
      averating_data_buffer += AE_HX711_Read();   
      cnt1++;
      millis_buffer = millis();
      if( cnt1 >= INITIALIZEING_SAMPLE ) {
        averating_data = averating_data_buffer / INITIALIZEING_SAMPLE;  
        millis_buffer = millis();
        lcd_pattern = 102;
        init_flag = true;
        pattern = 0;  
      }
    }
    break;
  }
}

//Main #0
//------------------------------------------------------------------//
void taskDisplay(void *pvParameters){  

  disableCore0WDT(); 
  
  while(1){    
    lcdDisplay();
    button_action(); 

    if( x0_flag && pattern < 100) {
      int sum = 0;
      digitalWrite(ledPin, led_flag);  
      led_flag = !led_flag;  
      native_data_x0 = AE_HX711_Read() - averating_data;      
      buffer_x0.push(native_data_x0 - native_data_x0_buffer);
      native_data_x0_buffer = native_data_x0;
      for(int i=0; i<AVERATING_BUFFER_SIZE; i++) {
        sum += buffer_x0[i];
      }
      level_x0 = sum / AVERATING_BUFFER_SIZE;
      x0_flag = false;
      digitalWrite(ledPin, led_flag);  
      led_flag = !led_flag;  
    }

  }
}

// Timer Interrupt
//------------------------------------------------------------------//
void Timer_Interrupt( void ){
  if (interruptCounter > 0) {
    portENTER_CRITICAL(&timerMux);
    interruptCounter--;
    portEXIT_CRITICAL(&timerMux);            

    iTimer10++;
    switch (iTimer10) {
    case 1:  
      break;
    case 5:      
      break;
    case 10:      
      iTimer10 = 0;
      break;
    }

    iTimer80++;
    // 80ms timerinterrupt
    switch (iTimer80) {
    case 10:
      Serial.printf("%5.2f, ", float(millis()) / 1000); 
      Serial.printf("%3.2f, ", angle); 
      Serial.printf("%5d, ", velocity); 
      Serial.printf("%4.2f, ", torque); 
      Serial.printf("%8d, ", native_data_x0); 
      Serial.printf("%8d, ", level_x0); 
      Serial.printf("%2.2f\n", temp); 
      break;
    case 20:
      x0_flag = true;
      break;
    case 30:
      break;
    case 40:
      break;
    case 50:
      break;
    case 60:
      break;
    case 70:
      break;
    case 80:
      lcd_flag = true;
      iTimer80 = 0;
      break;
    }
  }
}

// Initialize LCD
//------------------------------------------------------------------//
void initLCD(void) {

  M5.Lcd.clear();   
  switch (lcd_pattern) {
  case 0:         
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.setCursor(20, 10);
    M5.Lcd.setTextSize(2);
    M5.Lcd.printf("T+:");  
    M5.Lcd.setCursor(20, 60);
    if( sd_insert ) {
      M5.Lcd.drawJpgFile(SD, "/icon/icons8-sd.jpg", 214, 0);
    }
    if( wifi_flag ) {
      M5.Lcd.drawJpgFile(SD, "/icon/icons8-wi-fi-1.jpg", 248, 0);      
    } else {
      M5.Lcd.drawJpgFile(SD, "/icon/icons8-wi-fi-0.jpg", 248, 0);     
    }
    break;
  case 10:
    break;
  case 20:   
    break;
  }
}


// LCD Display
//------------------------------------------------------------------//
void lcdDisplay(void) {
  unsigned int time_calc, time_h, time_m, time_s;

  if( lcd_flag ) {    
    // Refresh Display
    switch (lcd_pattern) {
    case 0:
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
      if( battery_persent == 100) {
        M5.Lcd.drawJpgFile(SD, "/icon/icons8-battery-level-100.jpg", 290, 0);
      } else if( battery_persent == 75) {
        M5.Lcd.drawJpgFile(SD, "/icon/icons8-battery-level-75.jpg", 290, 0);
      } else if( battery_persent == 50) {
        M5.Lcd.drawJpgFile(SD, "/icon/icons8-battery-level-50.jpg", 290, 0);
      } else if( battery_persent == 25) {
        M5.Lcd.drawJpgFile(SD, "/icon/icons8-battery-level-25.jpg", 290, 0);
      } else {
        M5.Lcd.drawJpgFile(SD, "/icon/icons8-battery-level-0.jpg", 290, 0);
      }  
      lcd_flag = false; 
      break;
    case 10:
      M5.Lcd.setTextColor(WHITE, BLACK);
      M5.Lcd.setTextSize(2);
      M5.Lcd.setCursor(80, 160);
      M5.Lcd.printf("HX711 %7d", native_data_x0);
      M5.Lcd.setCursor(80, 190);
      M5.Lcd.printf("level %7d", level_x0);
      lcd_flag = false;
      break;
    case 20:
      lcd_flag = false;
      break;
    case 100:    
      M5.Lcd.setCursor(25, 110);
      M5.Lcd.printf("Waiting for initialize");
      M5.Lcd.setCursor(82, 140);
      M5.Lcd.printf("Press button A");
      M5.Lcd.drawLine(73, 165, 253, 165, WHITE);
      M5.Lcd.drawLine(73, 165, 62, 240, WHITE);   
      lcd_pattern = 0;   
      lcd_flag = false;
      break;
    case 101:
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
      if( battery_persent == 100) {
        M5.Lcd.drawJpgFile(SD, "/icon/icons8-battery-level-100.jpg", 290, 0);
      } else if( battery_persent == 75) {
        M5.Lcd.drawJpgFile(SD, "/icon/icons8-battery-level-75.jpg", 290, 0);
      } else if( battery_persent == 50) {
        M5.Lcd.drawJpgFile(SD, "/icon/icons8-battery-level-50.jpg", 290, 0);
      } else if( battery_persent == 25) {
        M5.Lcd.drawJpgFile(SD, "/icon/icons8-battery-level-25.jpg", 290, 0);
      } else {
        M5.Lcd.drawJpgFile(SD, "/icon/icons8-battery-level-0.jpg", 290, 0);
      }  
      M5.Lcd.setCursor(70, 110);
      M5.Lcd.printf("Initializing...");
      M5.Lcd.progressBar(40,160,240,20, cnt1);
      lcd_flag = false; 
      break;
    case 102:
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
      if( battery_persent == 100) {
        M5.Lcd.drawJpgFile(SD, "/icon/icons8-battery-level-100.jpg", 290, 0);
      } else if( battery_persent == 75) {
        M5.Lcd.drawJpgFile(SD, "/icon/icons8-battery-level-75.jpg", 290, 0);
      } else if( battery_persent == 50) {
        M5.Lcd.drawJpgFile(SD, "/icon/icons8-battery-level-50.jpg", 290, 0);
      } else if( battery_persent == 25) {
        M5.Lcd.drawJpgFile(SD, "/icon/icons8-battery-level-25.jpg", 290, 0);
      } else {
        M5.Lcd.drawJpgFile(SD, "/icon/icons8-battery-level-0.jpg", 290, 0);
      }  
      M5.Lcd.setCursor(40, 110);
      M5.Lcd.printf("Initialize complete");
      if( millis() - millis_buffer > 2000 ){ 
        lcd_pattern = 0;
        initLCD();
      }
      lcd_flag = false; 
      break;
    }
  }
}

// ServoControl
//------------------------------------------------------------------//
void servoControl(float ang){
  float i;
  int iRet, dt, preTime;

  dt = (micros() - preTime) / 1000000;
  preTime = micros();

  i = ang - angle;
  iP = kp * i;
  iI += ki * i;
  iD = kd * (i - iBefore);
  iRet = iP + iI + iD;
  if( iRet > 30000 ) iRet = 30000;
  if( iRet < -30000 ) iRet = -30000;
  power1 = iRet;  
}

// IRAM
//------------------------------------------------------------------//
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter=1;
  portEXIT_CRITICAL_ISR(&timerMux);
}

// Initialize CAN
//------------------------------------------------------------------//
void init_can(){
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(0, 10);
  delay(500);

  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ);

  CAN0.setMode(MCP_NORMAL); // Set operation mode to normal so the MCP2515 sends acks to received data.

  pinMode(CAN0_INT, INPUT); // Configuring pin for /INT input

}

// Test CAN
//------------------------------------------------------------------//
void test_can(){

  data[0] = power1 >> 8 & 0xFF;
  data[1] = power1 & 0xFF;
  data[2] = power2 >> 8 & 0xFF;
  data[3] = power2 & 0xFF;
  data[4] = power3 >> 8 & 0xFF;
  data[5] = power3 & 0xFF;
  data[6] = power4 >> 8 & 0xFF;
  data[7] = power4 & 0xFF;
  CAN0.sendMsgBuf(0x1FF, 0, 8, data);

  if(!digitalRead(CAN0_INT)) // If CAN0_INT pin is low, read receive buffer
  {
    CAN0.readMsgBuf(&rxId, &len, rxBuf); // Read data: len = data length, buf = data byte(s)
  
      for(byte i = 0; i<len; i++){
        //sprintf(msgString, " 0x%.2X", rxBuf[i]);
        //M5.Lcd.printf(msgString);
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
      angle = (float)angle_buff * 360/8192-180;
      velocity = ((velocity_H << 8) | velocity_L & 0xFF);
      torque_buff = ((torque_H << 8) | torque_L & 0xFF)*741/1000;
      if( torque_buff > -20000 && torque_buff < 20000 ) {
        torque = float(torque_buff) / 10;
      }
      temp = temp_L;
      //M5.Lcd.setTextColor(WHITE, BLACK);
      //M5.Lcd.setTextSize(2);
      //M5.Lcd.setCursor(80, 100);
      //M5.Lcd.printf("Rotor Angle %3.2f", angle);
      //M5.Lcd.setCursor(80, 130);
      //M5.Lcd.printf("Velocity %4d", velocity);
      //M5.Lcd.setCursor(80, 130);
      //M5.Lcd.printf("Torque %5.3f", torque);
      //M5.Lcd.setCursor(80, 190);
      //M5.Lcd.printf("temp %3d", temp);     
  }
}

// Button Action
//------------------------------------------------------------------//
void button_action() {
  M5.update();
  if (M5.BtnA.wasPressed()) {
    if( pattern == 0 ) {
      if( init_flag ) {       
        lcd_pattern = 0; 
        initLCD();   
      } else {
        M5.Lcd.fillRect(0,30,320,210,0);
        lcd_pattern = 101;
        pattern = 101;
      }      
    }  
  } else if (M5.BtnB.wasPressed()) {    
    if( pattern == 0 ) {
      if(lcd_pattern >= 10 && lcd_pattern < 20) {
        lcd_pattern++;
        if(lcd_pattern >= 12) lcd_pattern = 10;
      } else {
        lcd_pattern = 10;
      }     
      initLCD();   
    } 
   
  } else if (M5.BtnC.wasPressed()) {    
    if( pattern == 0 ) {
      if(lcd_pattern >= 20 && lcd_pattern < 30) {
        lcd_pattern++;
        if(lcd_pattern >= 22) lcd_pattern = 20;
      } else {
        lcd_pattern = 20;
      } 
      initLCD();  
    } 
  }
} 

//AE HX711 Init
//------------------------------------------------------------------//
void AE_HX711_Init(void)
{
  pinMode(HX711_SCLK, OUTPUT);
  pinMode(HX711_DOUT, INPUT);
}

//AE HX711 Reset
//------------------------------------------------------------------//
void AE_HX711_Reset(void)
{
  digitalWrite(HX711_SCLK,1);
  delayMicroseconds(100);
  digitalWrite(HX711_SCLK,0);
  delayMicroseconds(100); 
}

//AE HX711 Read
//------------------------------------------------------------------//
long AE_HX711_Read(void)
{
  long data=0;
  while(digitalRead(HX711_DOUT)!=0);
  delayMicroseconds(1);
  for(int i=0;i<24;i++)
  {
    digitalWrite(HX711_SCLK,1);
    delayMicroseconds(1);
    digitalWrite(HX711_SCLK,0);
    delayMicroseconds(1);
    data = (data<<1)|(digitalRead(HX711_DOUT));
  }  
  digitalWrite(HX711_SCLK,1);
  delayMicroseconds(1);
  digitalWrite(HX711_SCLK,0);
  delayMicroseconds(1);
  return data^0x800000; 
}


long AE_HX711_Averaging(long adc,char num)
{
  long sum = 0;
  for (int i = 0; i < num; i++) sum += AE_HX711_Read();
  return sum / num;
}

float AE_HX711_getGram(char num)
{
  #define HX711_R1  20000.0f
  #define HX711_R2  8200.0f
  #define HX711_VBG 1.25f
  #define HX711_AVDD      4.2987f//(HX711_VBG*((HX711_R1+HX711_R2)/HX711_R2))
  #define HX711_ADC1bit   HX711_AVDD/16777216 //16777216=(2^24)
  #define HX711_PGA 128
  #define HX711_SCALE     (OUT_VOL * HX711_AVDD / LOAD *HX711_PGA)
  
  float data;

  data = AE_HX711_Averaging(AE_HX711_Read(),num)*HX711_ADC1bit; 
  data =  data / HX711_SCALE;
  return data;
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