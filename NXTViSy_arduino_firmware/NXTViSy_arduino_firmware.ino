#include <SPI.h>
#include <Pixy.h>
#include <XLCPixy.h>
#include <PixyViSy.h>
#include <Wire.h>
#include <stdint.h>


//Uncomment line below for some debug information on Serial Monitor
//WARNING: (slow) serial printing has problems with I2C
//  "SET" means that some configures were set to pixy_vi_sy
//  "value1 value2" means I2C master wants read data
//    value1 = distance from pixy_vi_sy
//    value2 = action = pixy_vi_sy
//#define DBG
#define I2C_ADDRESS 0x0A
#define GET_DATA            0x0001
#define SET_PIXEL_FX        0x0002   // 2-byte
#define SET_PIXEL_FY        0x0004   // 2-byte
#define SET_GOAL_SIG        0x0008   // 1-byte
#define SET_GOAL_HEIGHT     0x0010   // 1-byte
#define SET_MIN_GOAL_SIZE   0x0020   // 2-byte
#define SET_BALL_SIZE       0x0040   // 1-byte
#define SET_BALL_SIG        0x0080   // 1-byte
#define SET_MIN_BALL_SIZE   0x0100   // 2-byte
#define SET_FLAG            0x0200   // 1-byte
#define SETTINGS_COUNT 9

#define FOC_LEN_X 1
#define FOC_LEN_Y 1
#define GOAL_SIG 1
#define BALL_SIG 1
#define GOAL_HEIGHT 1
#define BALL_DIAMETER 1
#define MIN_GOAL_SIZE 1
#define MIN_BALL_SIZE 1

PixyViSy pixy_vi_sy(FOC_LEN_X, FOC_LEN_Y,
               GOAL_SIG, GOAL_HEIGHT, MIN_GOAL_SIZE,
               BALL_DIAMETER, BALL_SIG, MIN_BALL_SIZE,
               PIXYVISY_GOAL);

typedef void (PixyViSy::*PixyViSy_set_func_1_t)(uint8_t);
typedef void (PixyViSy::*PixyViSy_set_func_2_t)(uint16_t);
union PixyViSy_set_func_t {
    PixyViSy_set_func_1_t pixyViSy_set_func_1;
    PixyViSy_set_func_2_t pixyViSy_set_func_2;
};
uint16_t settings_flag[SETTINGS_COUNT] = { SET_PIXEL_FX, SET_PIXEL_FY,
    SET_GOAL_SIG, SET_GOAL_HEIGHT, SET_MIN_GOAL_SIZE, SET_BALL_SIZE,
    SET_BALL_SIG, SET_MIN_BALL_SIZE, SET_FLAG };
uint16_t settings_bytes[SETTINGS_COUNT] = { 2, 2, 1, 1, 2, 1, 1, 2, 1 };
PixyViSy_set_func_t settings[SETTINGS_COUNT] = { NULL };

uint16_t data;
uint16_t last_distance = 0x00;
uint8_t last_action = 0x00;

void setup()
{
    settings[0].pixyViSy_set_func_2 = &PixyViSy::setPixelFx;
    settings[1].pixyViSy_set_func_2 = &PixyViSy::setPixelFy;
    settings[2].pixyViSy_set_func_1 = &PixyViSy::setGoalSig;
    settings[3].pixyViSy_set_func_1 = &PixyViSy::setGoalHeight;
    settings[4].pixyViSy_set_func_2 = &PixyViSy::setMinGoalSize;
    settings[5].pixyViSy_set_func_1 = &PixyViSy::setBallSize;
    settings[6].pixyViSy_set_func_1 = &PixyViSy::setBallSig;
    settings[7].pixyViSy_set_func_2 = &PixyViSy::setMinBallSize;
    settings[8].pixyViSy_set_func_1 = &PixyViSy::setFlag;

    Wire.begin(I2C_ADDRESS); // Start I2C on Address 0x0A
    Wire.onReceive(receiveI2C);
    Wire.onRequest(requestI2C);

    #ifdef DBG
    Serial.begin(115200);
    Serial.println("setup");
    #endif
}

void loop()
{
    pixy_vi_sy.update();
    last_distance = pixy_vi_sy.getGoalDist();
    last_action = pixy_vi_sy.getGoalAction();
    delay(25); // without delay it doesnt work
}

void receiveI2C(int bytesIn) 
{
    #ifdef DBG
    Serial.println("receive");
    #endif
    if (Wire.available() < 2) {
        #ifdef DBG
        Serial.println("Error: Not enough bytes sent for setting");
        #endif
        return;
    }
    data = Wire.read(); // low bits
    data |= (Wire.read() << 8); // high bits

    #ifdef DBG
    Serial.println(data >> 8, BIN);
    Serial.println(data & 0xFF, BIN);
    #endif

    if ((data & GET_DATA) == true) { // get data
        #ifdef DBG
        Serial.println("GET DATA");
        #endif
        return;
    }
    #ifdef DBG
    Serial.print("SET DATA: bytes sent: ");
    Serial.println(Wire.available());
    #endif
    for (uint8_t i = 0; i < SETTINGS_COUNT; i++) {
        if ((data & settings_flag[i]) == false) {
            continue;
        }
        #ifdef DBG
        Serial.print("SET DATA: setting ");
        Serial.println(i);
        #endif
        if (Wire.available() < settings_bytes[i]) {
            #ifdef DBG
            Serial.println("Error: Not enough bytes sent for setting");
            #endif
            return;
        }
        switch (settings_bytes[i]) {
            case 1 : {
                uint8_t set_data = Wire.read();
                (pixy_vi_sy.*(settings[i].pixyViSy_set_func_1))(set_data);
                break;
            }
            case 2 : {
                uint16_t set_data = Wire.read(); // low bits
                set_data |= (Wire.read() << 8); // high bits
                (pixy_vi_sy.*(settings[i].pixyViSy_set_func_2))(set_data);
                break;
            }
            default : {
                #ifdef DBG
                Serial.println("Error: Error in code");
                #endif
                break;
            }
        }
    }
    #ifdef DBG
    pixy_vi_sy.printParams();
    Serial.print("Wire.available = ");
    Serial.println(Wire.available());
    #endif
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

/* vim: set tabstop=4 softtabstop=4 shiftwidth=4 expandtab : */
