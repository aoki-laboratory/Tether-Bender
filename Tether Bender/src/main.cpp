#include <M5Stack.h>
#include <mcp_can.h>

/**
 * variable for loop
 */

byte data[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};


/**
 * variable for CAN
 */
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];                        // Array to store serial string

#define CAN0_INT 15                              // Set INT to pin 2
MCP_CAN CAN0(12);                               // Set CS to pin 10

void init_can();
void test_can();

void setup() {
  M5.begin();

  M5.Lcd.fillScreen(WHITE);
  delay(500);
  M5.Lcd.setTextColor(BLACK);
  // M5.Lcd.setTextSize(1);

  init_can();
}

void loop() {
  if(M5.BtnA.wasPressed())
  {
    M5.Lcd.clear();
    M5.Lcd.printf("CAN Test A!\n");
    M5.Lcd.fillScreen(WHITE);
    init_can();
  }
  test_can();
  M5.update();
}

void init_can(){
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(0, 10);
  M5.Lcd.fillScreen(WHITE);
  delay(500);

  M5.Lcd.printf("CAN Test A!\n");
  M5.Lcd.printf("Receive first, then testing for sending function!\n");

  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ);

  CAN0.setMode(MCP_NORMAL); // Set operation mode to normal so the MCP2515 sends acks to received data.

  pinMode(CAN0_INT, INPUT); // Configuring pin for /INT input

}

void test_can(){
  if(!digitalRead(CAN0_INT)) // If CAN0_INT pin is low, read receive buffer
  {
    CAN0.readMsgBuf(&rxId, &len, rxBuf); // Read data: len = data length, buf = data byte(s)
  
    if((rxId & 0x40000000) == 0x40000000){    // Determine if message is a remote request frame.
      sprintf(msgString, " REMOTE REQUEST FRAME");
      Serial.print(msgString);
    } else {
      for(byte i = 0; i<len; i++){
        sprintf(msgString, " 0x%.2X", rxBuf[i]);
        M5.Lcd.printf(msgString);
      }
    }
    M5.Lcd.printf("\n");
  }
}