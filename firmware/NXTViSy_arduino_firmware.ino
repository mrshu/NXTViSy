#include <SPI.h>
#include <Pixy.h>
#include <PixyViSy.h>
#include <Wire.h>
#include <stdint.h>


//Uncomment line below for some debug information on Serial Monitor
//  "OK" means that some configures were set to pixy_vi_sy
//  "value1 value2" means I2C master wants read data
//    value1 = distance from pixy_vi_sy
//    value2 = action = pixy_vi_sy
//#define DBG
#define I2C_ADDRESS 0x0A
#define GET_DATA 0x01
#define SET_SIGNATURE 0x02
#define SET_HEIGHT 0x04
#define SET_CALIB_CONST 0x08

uint8_t data;
PixyViSy pixy_vi_sy(1, 1, 1);
uint16_t last_distance = 0x00;
uint8_t last_action = 0x00;

void setup()
{
  Wire.begin(I2C_ADDRESS); // Start I2C on Address 0x0A
  Wire.onReceive(receiveI2C);
  Wire.onRequest(requestI2C);
  
  #ifdef DBG
  Serial.begin(9600);
  #endif
}

void loop()
{
  pixy_vi_sy.update();
  last_distance = pixy_vi_sy.getDistance();
  last_action = pixy_vi_sy.getAction();
  delay(25); // without delay it doesnt work
}

void receiveI2C(int bytesIn) 
{
  data = Wire.read();
  byte signature = ~0;
  uint8_t goal_height = ~0;
  uint16_t calib_const = ~0;
  
  if ((data & GET_DATA) == false) { // set data
    // number of variables to set to pixy_vi_sy
    byte configures = ((data & SET_SIGNATURE) >> 1) + ((data & SET_HEIGHT) >> 2) + ((data & SET_CALIB_CONST) >> 3);
    
    if (bytesIn - 1 >= configures) { // enough bytes were send
      #ifdef DBG
      Serial.println("OK");
      #endif
      
      if (data & SET_SIGNATURE) {
        signature = Wire.read();
        pixy_vi_sy.setSignature(signature);
      }
      if (data & SET_HEIGHT) {
        goal_height = Wire.read();
        pixy_vi_sy.setGoalHeight(goal_height);
      }
      if (data & SET_CALIB_CONST) {
        calib_const = Wire.read(); // low bits
        calib_const |= (Wire.read() << 8); // high bits
        pixy_vi_sy.setCalibConst(calib_const);
      }
    }
  }  
}

void requestI2C()
{
  if (data & GET_DATA) { // get data
    byte low_bits = last_distance & 0xFF;
    byte high_bits = last_distance >> 8;
    byte array[] = { low_bits, high_bits, last_action };
    
    Wire.write(array, 3);
    
    #ifdef DBG
    Serial.print(last_distance);
    Serial.print(' ');
    Serial.println(last_action);
    #endif
  }
}
