#include <SPI.h>
#include <Pixy.h>
#include <XLCPixy.h>
#include <PixyViSy.h>
#include <Wire.h>
#include <stdint.h>


//Uncomment line below for some debug information on Serial Monitor
//WARNING: (slow) serial printing has problems with I2C (because it's called
//from and ISR
//#define DBG
#define I2C_ADDRESS 0x0A
#define GET_DATA            0x0001
#define GET_DATA_GOAL       0x0002
#define GET_DATA_BALL       0x0004
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

PixyViSy pixy_vi_sy(1, 1, 1, 1, 1, 1, 1, 1, 1);

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
uint16_t goal_dist = 0;
uint8_t goal_action = 0;
uint16_t ball_dist = 0;
int8_t ball_angle = 0;

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
    goal_dist = pixy_vi_sy.getGoalDist();
    goal_action = pixy_vi_sy.getGoalAction();
    ball_dist = pixy_vi_sy.getBallDist();
    ball_angle = pixy_vi_sy.getBallAngle();
    delay(25); // without delay it doesnt work
}

void receiveI2C(int bytesIn) 
{
    #ifdef DBG
    Serial.println("receive");
    #endif
    if (Wire.available() < 2) {
        #ifdef DBG
        Serial.println("Error: Few bytes");
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
        //Serial.print("SET DATA: setting ");
        //Serial.println(i);
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
    //pixy_vi_sy.printParams();
    Serial.print("Wire.available = ");
    Serial.println(Wire.available());
    #endif
}

void requestI2C()
{
    if (data & GET_DATA) { // get data
        if ((data & GET_DATA_GOAL) && (data & GET_DATA_BALL)) {
            byte goal_low_bits = goal_dist & 0xFF;
            byte goal_high_bits = goal_dist >> 8;
            byte ball_low_bits = ball_dist & 0xFF;
            byte ball_high_bits = ball_dist >> 8;
            byte array[] = { goal_low_bits, goal_high_bits, goal_action,
                ball_low_bits, ball_high_bits, (byte)ball_angle };

            Wire.write(array, 6);

            #ifdef DBG
            Serial.print(array[0]);
            Serial.print(' ');
            Serial.print(array[1]);
            Serial.print(' ');
            Serial.print(array[2]);
            Serial.print(' ');
            Serial.print(array[3]);
            Serial.print(' ');
            Serial.print(array[4]);
            Serial.print(' ');
            Serial.println(array[5]);
            #endif
        } else if (data & GET_DATA_GOAL) {
            byte low_bits = goal_dist & 0xFF;
            byte high_bits = goal_dist >> 8;
            byte array[] = { low_bits, high_bits, goal_action };

            Wire.write(array, 3);

            #ifdef DBG
            Serial.print(array[0]);
            Serial.print(' ');
            Serial.print(array[1]);
            Serial.print(' ');
            Serial.println(array[2]);
            #endif
        } else if (data & GET_DATA_BALL) {
            byte low_bits = ball_dist & 0xFF;
            byte high_bits = ball_dist >> 8;
            byte array[] = { low_bits, high_bits, (byte)ball_angle };

            Wire.write(array, 3);

            #ifdef DBG
            Serial.print(array[0]);
            Serial.print(' ');
            Serial.print(array[1]);
            Serial.print(' ');
            Serial.println(array[2]);
            #endif
        }
    }
}

/* vim: set tabstop=4 softtabstop=4 shiftwidth=4 expandtab : */
