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
#define CAN0_INT 15                             // Set INT to pin 15
#define TIMER_INTERRUPT 1
#define HX711_DOUT_X0  2
#define HX711_SCLK_X0  5
#define HX711_DOUT_X1  35
#define HX711_SCLK_X1  16
#define HX711_DOUT_Y0  36
#define HX711_SCLK_Y0  17
#define HX711_DOUT_Y1  34
#define HX711_SCLK_Y1  0

#define AVERATING_BUFFER_SIZE 100
#define INITIALIZEING_SAMPLE 100
#define CALIBRATION_PERIOD 15

//Global
//------------------------------------------------------------------//
TaskHandle_t task_handl;

CircularBuffer<int, AVERATING_BUFFER_SIZE> buffer_x0;
CircularBuffer<int, AVERATING_BUFFER_SIZE> buffer_x1;
CircularBuffer<int, AVERATING_BUFFER_SIZE> buffer_y0;
CircularBuffer<int, AVERATING_BUFFER_SIZE> buffer_y1;

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

int level_x0;
int level_x1;
int level_y0;
int level_y1;

int power1 = 0;
int power2 = 0;
int power3 = 0;
int power4 = 0;

volatile int pattern = 0;
int iP, iD, iI;
int kp = 25;
int ki = 10;
int kd = 100;
float iBefore;

// WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "Buffalo-G-0CBA";
char pass[] = "hh4aexcxesasx";
//char ssid[] = "X1Extreme-Hotspot";
//char pass[] = "5]6C458w";
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
float load_step_y[] = {0, 2025, 4050, 6075, 8100, 10125, 12150, 14175, 16200, 18225, 20250};
// HX711 X0
long  native_data_x0;
long  native_data_x0_buffer;
long  averating_data_x0;
long  averating_data_x0_buffer;
long  calibration_factor_x0[11];
float coefficient_x0;
float intercept_x0;
// HX711 X1
long  native_data_x1;
long  native_data_x1_buffer;
long  averating_data_x1;
long  averating_data_x1_buffer;
long  calibration_factor_x1[11];
float coefficient_x1;
float intercept_x1;
// HX711 Y0
long  native_data_y0;
long  native_data_y0_buffer;
long  averating_data_y0;
long  averating_data_y0_buffer;
long  calibration_factor_y0[11];
float coefficient_y0;
float intercept_y0;
// HX711 Y1
long  native_data_y1;
long  native_data_y1_buffer;
long  averating_data_y1;
long  averating_data_y1_buffer;
long  calibration_factor_y1[11];
float coefficient_y1;
float intercept_y1;

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

// Timer
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile int interruptCounter;
int iTimer15;
int iTimer50;

//Prototype
//------------------------------------------------------------------//
void init_can();
void test_can();
void button_action();
void initLCD(void);
void lcdDisplay(void);
void taskDisplay(void *pvParameters);
void IRAM_ATTR onTimer(void);
void TimerInterrupt(void);
void servoControl(float ang);
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

   
}

//Main #1
//------------------------------------------------------------------//
void loop() {

  //test_can();  

  SerialRX();
  SerialTX();

  switch (pattern) {
  case 0:    
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
        Serial.printf("%5.2f, ", float(millis()) / 1000); 
        Serial.printf("%3.2f, ", angle); 
        Serial.printf("%5d, ", velocity); 
        Serial.printf("%4.2f, ", torque); 
        Serial.printf("%10ld, ", native_data_x0); 
        Serial.printf("%10d, ", level_x0); 
        Serial.printf("%10ld, ", native_data_x1); 
        Serial.printf("%10d, ", level_x1); 
        Serial.printf("%10ld, ", native_data_y0); 
        Serial.printf("%10d, ", level_y0); 
        Serial.printf("%10ld, ", native_data_y1); 
        Serial.printf("%10d, ", level_y1); 
        Serial.printf("%2.2f\n", temp); 
      }
      break;
    case 20:      
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
      }
      xbee_index = 0;
      
      switch ( rx_pattern ) {          
      case 0:
        tx_pattern = 1;
        break;
        
      case 11:      
        tx_pattern = 11;
        rx_pattern = 21;
        break;

      case 21:             
        getCalibrationDataX0();
        rx_pattern = 0;
        break;

      case 22:             
        getCalibrationDataX1();
        rx_pattern = 0;
        break;

      case 23:             
        getCalibrationDataY0();
        rx_pattern = 0;
        break;

      case 24:             
        getCalibrationDataY1();
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
          getOffsetValue();
        } else {
          tx_pattern = 1;
        }
        rx_pattern = 0;
        break;
      }      
      
    } else if( xbee_rx_buffer[xbee_index] == 'T' || xbee_rx_buffer[xbee_index] == 't' ) {
      rx_pattern = 0;
      Serial.printf("\n\n");
      tx_pattern = 101;
    } else if( xbee_rx_buffer[xbee_index] == 'I' || xbee_rx_buffer[xbee_index] == 'i' ) {
      Serial.printf("\n\n");
      tx_pattern = 201;
      rx_pattern = 201;
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
      Serial.printf(" 11 : Start Seqence\n");
      Serial.printf("\n");
      Serial.printf(" 21 : Calibration X0\n");
      Serial.printf(" 22 : Calibration X1\n");
      Serial.printf(" 23 : Calibration Y0\n");
      Serial.printf(" 24 : Calibration Y1\n");
      Serial.printf("\n");
      Serial.printf("\n");
      Serial.printf(" T : Telemetry\n");
      Serial.printf(" I : Initialize\n");
      
      Serial.printf("\n");
      Serial.printf(" Please enter 11 to 35 -> ");
      
      tx_pattern = 0;
      break;

    // Waiting Value
    case 2:
      break;

    case 11:
      Serial.printf("\n Check Current Parameters\n");
      Serial.printf("\n");
      Serial.printf("\n");
      Serial.printf(" Confirm to Climb? -> ");
      tx_pattern = 2;
      break;

    // Telemetry Mode
    case 101:
      break;

    // Initialize 
    case 201:
      Serial.printf(" Confirm to Initialize ? -> ");
      tx_pattern = 2;
      break;

  }
}

// EEPROM Write
//------------------------------------------------------------------// 
void eeprom_write(void) {
  delay(10);
  EEPROM.commit();
  delay(10);
}

// EEPROM Read
//------------------------------------------------------------------// 
void eeprom_read(void) {
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

}


// LCD Display
//------------------------------------------------------------------//
void lcdDisplay(void) {
  unsigned int time_calc, time_h, time_m, time_s;

  if( lcd_flag ) {    
    // Main
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
      lcd_flag = false; 
      break;

    }
  }
}


// Button Action
//------------------------------------------------------------------//
void button_action() {
  M5.update();
  if (M5.BtnA.wasPressed()) {

  } else if (M5.BtnB.wasPressed()) {    
   
  } else if (M5.BtnC.wasPressed()) {  

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
void AE_HX711_Read( void )
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
  
  for(int i=0; i<INITIALIZEING_SAMPLE; i++) {
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

  for(int i=0; i<INITIALIZEING_SAMPLE; i++) {
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

  for(int i=0; i<INITIALIZEING_SAMPLE; i++) {
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

  for(int i=0; i<INITIALIZEING_SAMPLE; i++) {
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

  Serial.printf(" Initialization complete\n");  

  tx_pattern = 1;

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

  Serial.printf(" Calibration Result\n\n");  
  for(int i=0; i<11; i++) {
    Serial.printf("%2d, %10d\n", i, calibration_factor_x0[i]);  
  }
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

  Serial.printf(" Calibration Result\n\n");  
  for(int i=0; i<11; i++) {
    Serial.printf("%2d, %10d\n", i, calibration_factor_x1[i]);  
  }
  tx_pattern = 1;  
  
}

// Calibration Y0
//------------------------------------------------------------------//
void getCalibrationDataY0(void) {

  for(int j=0; j<11; j++) {
    if(j==0) Serial.printf(" Prepare for the Y0 axis calibration and press the ButtonA.\n"); 
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
    averating_data_y0_buffer = 0;
    for(int i=0; i<INITIALIZEING_SAMPLE; i++) {
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

  float sum_xy = 0, sum_x = 0, sum_y = 0, sum_x2 = 0;

  for (int i=0; i<11; i++) {
    sum_xy += calibration_factor_y0[i] * load_step_y[i];
    sum_x += calibration_factor_y0[i];
    sum_y += load_step_y[i];
    sum_x2 += pow(calibration_factor_y0[i], 2);
  }
  
  coefficient_y0 = (11 * sum_xy - sum_x * sum_y) / (11 * sum_x2 - pow(sum_x, 2));
  intercept_y0 = (sum_x2 * sum_y - sum_xy * sum_x) / (11 * sum_x2 - pow(sum_x, 2));


  Serial.printf(" Calibration Result\n\n");  
  for(int i=0; i<11; i++) {
    Serial.printf(" %10d, %3.3f\n", calibration_factor_y0[i], load_step_y[i]);  
  }

  Serial.printf("\n Coefficient = %f, Intercept = %f\n\n", coefficient_y0, intercept_y0);  
  tx_pattern = 1;  

  
}

// Calibration Y1
//------------------------------------------------------------------//
void getCalibrationDataY1(void) {

  for(int j=0; j<11; j++) {
    if(j==0) Serial.printf(" Prepare for the Y1 axis calibration and press the ButtonA.\n"); 
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
    averating_data_y1_buffer = 0;
    for(int i=0; i<INITIALIZEING_SAMPLE; i++) {
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

  Serial.printf(" Calibration Result\n\n");  
  for(int i=0; i<11; i++) {
    Serial.printf("%2d, %10d\n", i, calibration_factor_y1[i]);  
  }
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