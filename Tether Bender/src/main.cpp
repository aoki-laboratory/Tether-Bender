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
#include <M5Stack.h>
#include <mcp_can.h>

//Define
//------------------------------------------------------------------//
#define CAN0_INT 15                             // Set INT to pin 15
#define TIMER_INTERRUPT 10

//Global
//------------------------------------------------------------------//
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

unsigned short angle_buff;
float angle;
short velocity;
short torque;
int8_t temp;

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

// Timer
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile int interruptCounter;
int iTimer10;

//Prototype
//------------------------------------------------------------------//
void init_can();
void test_can();
void taskInit();
void button_action();
void IRAM_ATTR onTimer(void);
void Timer_Interrupt(void);
void servoControl(float ang);

//Setup #1
//------------------------------------------------------------------//
void setup() {
  M5.begin();

  Serial.begin(115200);

  delay(500);

  // Initialize Timer Interrupt
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, TIMER_INTERRUPT * 1000, true);
  timerAlarmEnable(timer); 

  M5.Lcd.setTextColor(BLACK);

  taskInit();
  init_can();
}

//Main #1
//------------------------------------------------------------------//
void loop() {
  Timer_Interrupt(); 
  test_can(); 
  M5.update();
  button_action(); 

  switch (pattern) {
  case 0:
    break;
  
  case 11:
    servoControl(0);
    if(velocity > -1 &&velocity <= 1) {
      if(angle > -0.1 && angle < 0.1) {
        pattern = 12;
        iI = 0;
        break;
      }
    }    
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

  }
  delay(1);
}

// Timer Interrupt
//------------------------------------------------------------------//
void Timer_Interrupt( void ){
  if (interruptCounter > 0) {
    portENTER_CRITICAL(&timerMux);
    interruptCounter--;
    portEXIT_CRITICAL(&timerMux);

    if(pattern == 13 && power1 < 30000) {
      M5.Lcd.setTextSize(3);
      M5.Lcd.setCursor(220, 40);
      M5.Lcd.setTextColor(TFT_DARKGREY);
      M5.Lcd.printf("%4d",power1);
      power1+=5;
      M5.Lcd.setCursor(220, 40);
      M5.Lcd.setTextColor(WHITE);
      M5.Lcd.printf("%4d",power1);
    }

    iTimer10++;
    switch (iTimer10) {
    case 1:
      break;
    case 10:
      iTimer10 = 0;
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
  M5.Lcd.setTextSize(3);
  M5.Lcd.setCursor(180, 40);
  M5.Lcd.setTextColor(TFT_DARKGREY);
  M5.Lcd.printf("%4d",iI);
  iP = kp * i;
  iI += ki * i;
  iD = kd * (i - iBefore);
  iRet = iP + iI + iD;
  M5.Lcd.setTextSize(3);
  M5.Lcd.setCursor(180, 40);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.printf("%4d",iI);
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
      M5.Lcd.setTextColor(BLACK);
      M5.Lcd.setTextSize(2);
      M5.Lcd.setCursor(80, 100);
      M5.Lcd.printf("Roter Angle %3.2f", angle);
      M5.Lcd.setTextSize(2);
      M5.Lcd.setCursor(80, 130);
      M5.Lcd.printf("Velocity %4d", velocity);
      M5.Lcd.setTextSize(2);
      M5.Lcd.setCursor(80, 160);
      M5.Lcd.printf("Torque %5d", torque);
      M5.Lcd.setTextSize(2);
      M5.Lcd.setCursor(80, 190);
      M5.Lcd.printf("temp %3d", temp);
      angle_buff = ((angle_H << 8) | angle_L & 0xFF);
      angle = (float)angle_buff * 360/8192-180;
      velocity = ((velocity_H << 8) | velocity_L & 0xFF);
      torque = ((torque_H << 8) | torque_L & 0xFF)*741/1000;
      temp = temp_L;
      M5.Lcd.setTextColor(WHITE);
      M5.Lcd.setTextSize(2);
      M5.Lcd.setCursor(80, 100);
      M5.Lcd.printf("Roter Angle %3.2f", angle);
      M5.Lcd.setTextSize(2);
      M5.Lcd.setCursor(80, 130);
      M5.Lcd.printf("Velocity %4d", velocity);
      M5.Lcd.setTextSize(2);
      M5.Lcd.setCursor(80, 160);
      M5.Lcd.printf("Torque %5d", torque);
      M5.Lcd.setTextSize(2);
      M5.Lcd.setCursor(80, 190);
      M5.Lcd.printf("temp %3d", temp);  
      Serial.print(angle); 
      Serial.print(", ");   
      Serial.print(velocity); 
      Serial.print(", ");   
      Serial.print(torque); 
      Serial.print(", ");   
      Serial.println(temp); 
    
  }
}

// Initialize Task
//------------------------------------------------------------------//
void taskInit() {
  M5.Lcd.fillRect(0, 0, 320, 20, TFT_WHITE);
  M5.Lcd.fillRect(60, 20, 260, 60, TFT_DARKGREY);
  M5.Lcd.fillRect(0, 80, 60, 160, TFT_DARKGREY);
  M5.Lcd.fillRect(0, 20, 60, 60, TFT_LIGHTGREY);
  M5.Lcd.fillRect(0, 220, 320, 20, TFT_WHITE);

  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(8, 2);
  M5.Lcd.setTextColor(BLACK);
  M5.Lcd.print("Tether Bender V1");
  M5.Lcd.setCursor(40, 222);
  M5.Lcd.print("ROTA");
  M5.Lcd.setCursor(140, 222);
  M5.Lcd.print("MODE");
  M5.Lcd.setCursor(228, 222);
  M5.Lcd.print("SEAQ");
  M5.Lcd.setTextSize(4);
  M5.Lcd.setCursor(8, 36);
  M5.Lcd.setTextColor(BLACK);
  M5.Lcd.print("TB");
  M5.Lcd.setTextSize(3);
  M5.Lcd.setCursor(80, 40);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.printf("Power");
  M5.Lcd.setTextSize(3);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setCursor(8, 110);
  M5.Lcd.print("No.");

  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setTextSize(5);
  M5.Lcd.setCursor(0, 150);
  M5.Lcd.printf("%2d", 1);
}

// Button Action
//------------------------------------------------------------------//
void button_action() {
  if (M5.BtnA.wasPressed() && pattern == 0) {
    M5.Lcd.setTextSize(3);
    M5.Lcd.setCursor(220, 40);
    M5.Lcd.setTextColor(TFT_DARKGREY);
    M5.Lcd.printf("%4d",power1);
    power1+=1000;
    if( power1 > 30000 ) {
      power1 = 0;
    }
    M5.Lcd.setTextSize(3);
    M5.Lcd.setCursor(220, 40);
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.printf("%4d",power1);
  } else if (M5.BtnB.wasPressed()) {    
  } else if (M5.BtnC.wasPressed() && pattern == 0) {    
    pattern = 11;
  }
} 