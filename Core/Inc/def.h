#ifndef __DEF_H__
#define __DEF_H__

#include "stdint.h"



//-----------------

#define SEPARATION_ALTITUDE 400
#define WAIT_AFTER_SEPARATION_TIME_SECONDS 5

#define STEADY_WAITING_ALTITUDE 200
#define STEADY_WAITING_TIME_SECONDS 1



enum flight_state_enum{
  DEBUG_PROFILE_1,
  START,
  AFTER_LIFTOFF,
  AFTER_APOGEE,
  AFTER_SEPARATION, // 400 -> 200 
  STEADY, // 200
  AFTER_200M, // 200 -> 2
  AFTER_2M,
  LANDED,
};

enum control_state_enum {
    STOP,
    START_TEST,
    PID_TEST,
    QR_TEST,
    INPUT_TEST,
};

enum ground_receive_commdands{
    BUZZER_ON = 0xA1, 
    BUZZER_OFF = 0xA2,
    SEPARATE = 0xB1,
    REVERT = 0xB2,
    MOTOR_STOP = 0xC1,
    MOTOR_START_TEST = 0xC2,
    MOTOR_PID_TEST = 0xC3,
    MOTOR_QR_TEST = 0xC4,
    MOTOR_INPUT_TEST = 0xC5,
};


typedef struct Accel{
    float x;
    float y;
    float z;
    float prev_x;
    float prev_y;
    float prev_z;
    float offset_x;
    float offset_y;
    float offset_z;
}Accel;

typedef struct Gyro{
    float x;
    float y;
    float z;
    float prev_x;
    float prev_y;
    float prev_z;
    float offset_x;
    float offset_y;
    float offset_z;
}Gyro;

typedef struct Angle{
    float roll;
    float pitch;
    float yaw;
    float prev_roll;
    float prev_pitch; 
    float prev_yaw;
    float offset_roll;
    float offset_pitch;
    float offset_yaw;
    

} Angle;

typedef struct Altitude{
    float pressure;
    float basePressure;
    float temperature;
   float altitude;
   float prevAltitude;
   float prevAltitudeForVelocity;
   float maxAltitude;
   float diffToMax;
} Altitude;

typedef struct Time{
   float current;
   float prevTime;
   float liftoffTime;
   float apogeeTime;
    float timeDifference;
    float flightTime;
    float landingTime;
} Time;

typedef struct Velocity{
    
    float verticalVelocity;
    float timeDiffVertical;
    float prevTimeVertical;
    float trueVelocity;
    float timeDiffTrue;
    float prevTimeTrue;
    
} Velocity;

typedef struct Gps{
    float latitude;
    float longtitude;
    float altitude;
    int sat;
    float utc_time;


    float velocity;
} Gps;

typedef struct Jei {
    float altitude;
    float pressure;
    float latitude;
    float longtitude;
    float gpsAltitude;
} Jei;

typedef struct Lenna {
    uint8_t tranmissionPercentage;
} Lenna;

typedef union {
    double u64;
    uint8_t u8[8];
} double_to_u8;

typedef union {
    float u32;
    uint8_t u8[4];
} float_to_u8;

typedef union {
    uint16_t u16;
    // degeri alirken cast etmek gerekiyor
    int16_t i16;
    uint8_t u8[2];
} u16_to_u8;





#endif // __DEF_H__
