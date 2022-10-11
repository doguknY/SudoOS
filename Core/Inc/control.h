#ifndef   __CONTROL_H__
#define   __CONTROL_H__

#include "main.h"
#include "tim.h"
#include "stm32g474xx.h"
#include "system.h"

extern float mot1;
extern float mot2;
extern float mot3;
extern float mot4;
extern float old_out_thrust;
extern float out_roll;
extern float out_pitch;
extern float out_thrust;
extern float set_point_pitch;
extern float set_point_roll;
extern uint32_t referenceTime;
extern uint8_t inputDuty;
extern uint8_t receivedDuty;
extern uint8_t motorDutyTask;

extern uint8_t kpBuffer[7];
extern uint8_t kiBuffer[7];
extern uint8_t kdBuffer[7];
extern float kp;
extern float ki;
extern float kd;

typedef struct {

	/* Controller gains */
	float Kp;
	float Ki;
	float Kd;

	/* Derivative low-pass filter time constant */
	float tau;

	/* Output limits */
	float limMin;
	float limMax;
	
	/* Integrator limits */
	float limMinInt;
	float limMaxInt;

	/* Sample time (in seconds) */
	float T;

	/* Controller "memory" */
	float integrator;
	float prevError;			/* Required for integrator */
	float differentiator;
	float prevMeasurement;		/* Required for differentiator */

	/* Controller output */
	float out;

} PID;

extern PID pid_roll, pid_pitch, pid_thrust;

float PID_Measure( PID *pid, float setpoint, float measurement);
float duty_to_thrust(float duty);
float servo_duty_to_thrust(uint16_t duty);
void init_control();
void calibrate_esc();
void init_esc();
void motor_control(uint8_t motor_start_stop, float vertical_velocity, int16_t targetAltitude);
void motor_control_debug();
void init_servo();
void servo_separate();
void servo_revert();




#endif /* __CONTROL_H__ */
