#ifndef __SYSTEM_H__
#define __SYSTEM_H__


#include "def.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <i2c.h>
#include <adc.h>
#include "stm32g4xx_hal.h"
#include <bno055.h>
#include "stdint.h"
#include "LoRa.h"
#include "gps.h"
#include "BME280_STM32.h"



extern uint8_t flightState;
extern uint8_t controlDebugState;

extern Time time;
extern Accel accel;
extern Gyro gyro;
extern Altitude altitude;
extern Velocity velocity;
extern Angle angle;
extern Gps gps;
extern Jei jei;
extern Lenna lenna;





extern uint8_t groundTransmitPacket[73];
extern uint8_t loraReceiveBuffer[8];
extern uint8_t lennaReceiveBuffer[4];
extern char telemetryStrPacket[200];



extern uint32_t voltageAdcRaw;
extern float voltage;
extern int spin;



void readTime();
void serialPrint();
void buzzerShoutOut(uint32_t time);
void openingThemeSong(uint32_t time);
void wink(uint32_t time);
void initBarometer();
void readAltitude();
void create_packet();
void initIMU();
void initLoRa();
void readIMU();
float kalmanFilter(float vari, float mea_e, float est_e, float q, float last);
void readIMU();

float getVoltage();
void loraRecevice();

void loraTransmit(uint8_t* data, uint16_t size);
void readGPS();

void offsetIMU();


void initSD();
void writeSD(char *data);

void buzzer(uint8_t state);
void led(uint8_t state);

#endif /* __SYSTEM_H__ */
