#include "control.h"

#define MOTOR_MAX_DUTY 10000
#define MOTOR_INIT_DUTY 6000
#define MOTOR_MIN_DUTY 5000

float old_out_thrust = 0;
float out_thrust = 0;
float out_roll = 0;
float out_pitch = 0;
float set_point_pitch = 0;
float set_point_roll = 90;

uint8_t kpBuffer[7];
uint8_t kiBuffer[7];
uint8_t kdBuffer[7];

float kp;
float ki;
float kd;

uint8_t receivedDuty;
uint8_t inputDuty;
uint8_t motorDutyTask;

uint32_t referenceTime;

PID pid_roll, pid_pitch, pid_thrust;


void init_control()
{
    pid_roll.Kp = 0;
    pid_roll.Ki = 0;
    pid_roll.Kd = 0;
    pid_roll.tau = 0.0;
    pid_roll.limMin = 0;
    pid_roll.limMax = 0;
    pid_roll.limMinInt = 0;
    pid_roll.limMaxInt = 0;
    pid_roll.T = 1000;

    pid_pitch.Kp = 0;
    pid_pitch.Ki = 0;
    pid_pitch.Kd = 0;
    pid_pitch.tau = 0;
    pid_pitch.limMin = 0;
    pid_pitch.limMax = 0;
    pid_pitch.limMinInt = 0;
    pid_pitch.limMaxInt = 0;
    pid_pitch.T = 1000;

    pid_thrust.Kp = 0;
    pid_thrust.Ki = 0;
    pid_thrust.Kd = 0;
    pid_thrust.tau = 0;
    pid_thrust.limMin = 0;
    pid_thrust.limMax = 0;
    pid_thrust.limMinInt = 0;
    pid_thrust.limMaxInt = 0;
    pid_thrust.T = 1000;
}

float duty_to_thrust(float duty) {

    float thrust = 5000.0 + duty * 50.0;

    if (thrust > 10000.0) thrust = 10000.0;
    if (thrust < 5000.0) thrust = 5000.0;

    return thrust;
}

float servo_duty_to_thrust(uint16_t duty) {

    float thrust = 500.0 + duty * 5;

    if (thrust > 1000.0) thrust = 1000.0;
    if (thrust < 500.0) thrust = 500.0;

    return thrust;
}

static void motor1_power(uint16_t power)
{

    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, power);
}

static void motor2_power(uint16_t power)
{

    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, power);
}

static void motor3_power(uint16_t power)
{

    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, power);
}

static void motor4_power(uint16_t power)
{

    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, power);
}

/*
 * @brief  ESC calib function
 * @param  None
 * @retval None
 */

void calibrate_esc() {

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, MOTOR_MAX_DUTY);
    HAL_Delay(2000);
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, MOTOR_MIN_DUTY);

    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, MOTOR_MAX_DUTY);
    HAL_Delay(2000);
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, MOTOR_MIN_DUTY);

    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, MOTOR_MAX_DUTY);
    HAL_Delay(2000);
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, MOTOR_MIN_DUTY);

    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, MOTOR_MAX_DUTY);
    HAL_Delay(2000);
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, MOTOR_MIN_DUTY);
}

/*
 * @brief  ESC init function
 * @param  None
 * @retval None
 */

void init_esc() {

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, MOTOR_INIT_DUTY);
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, MOTOR_INIT_DUTY);
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, MOTOR_INIT_DUTY);
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, MOTOR_INIT_DUTY);

    HAL_Delay(500);
    
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, MOTOR_MIN_DUTY);
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, MOTOR_MIN_DUTY);
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, MOTOR_MIN_DUTY);
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, MOTOR_MIN_DUTY);
}

/*
 * @brief  Motor control function
 * @param  motor_start_stop: -1 for stop, 1 for start
 * @param  vertical_velocity: vertical velocity
 * @param  targertAltitude: if it bigger than 0, motor will control according to altitude value
 * @retval None
 * @todo   target lat and long value
 */
void motor_control(uint8_t motor_start_stop, float vertical_velocity, int16_t targetAltitude)
{

    if (motor_start_stop < 0)
    {

        out_thrust = duty_to_thrust(0);

        motor1_power(out_thrust);
        motor2_power(out_thrust);
        motor3_power(out_thrust);
        motor4_power(out_thrust);
        return;
    }

    else {

        out_thrust = duty_to_thrust(42);
        out_thrust += PID_Measure(&pid_thrust, vertical_velocity, velocity.verticalVelocity);

        out_roll = PID_Measure(&pid_roll, set_point_roll, (angle.roll));
        out_pitch = PID_Measure(&pid_pitch, set_point_pitch, (angle.pitch));

        motor1_power((out_thrust + out_roll + out_pitch)); // front right
        motor2_power((out_thrust + out_roll - out_pitch)); // front left
        motor3_power((out_thrust - out_roll + out_pitch)); // rear right
        motor4_power((out_thrust - out_roll - out_pitch)); // rear left
    }
}

void motor_control_debug(){

    switch (controlDebugState) {

        case STOP:

            out_thrust = duty_to_thrust(0);

            motor1_power(out_thrust);
            motor2_power(out_thrust);
            motor3_power(out_thrust);
            motor4_power(out_thrust);

            referenceTime = time.current;
            old_out_thrust = out_thrust;

            motorDutyTask = 0;

            break;

        case START_TEST:


            out_thrust = duty_to_thrust(20);

            motor1_power(out_thrust);
            motor2_power(out_thrust);
            motor3_power(out_thrust);
            motor4_power(out_thrust);

            old_out_thrust = out_thrust;

            break;

        case PID_TEST:

            out_thrust = duty_to_thrust(20);

            out_roll = PID_Measure(&pid_roll, set_point_roll, (angle.roll));
            out_pitch = PID_Measure(&pid_pitch, set_point_pitch, (angle.pitch));

            /* X Model */

            motor1_power((out_thrust + out_roll + out_pitch)); // front right
            motor2_power((out_thrust + out_roll - out_pitch)); // front left
            motor3_power((out_thrust - out_roll + out_pitch)); // rear right
            motor4_power((out_thrust - out_roll - out_pitch)); // rear left

            /* T Model */

            /*
            motor1_power((out_thrust + out_pitch)); // rear
            motor2_power((out_thrust + out_roll)); // left
            motor3_power((out_thrust - out_roll)); // right
            motor4_power((out_thrust - out_pitch)); // front
            */

            break;

        case QR_TEST:

            out_thrust = duty_to_thrust(20);

            motor1_power(out_thrust);
            motor2_power(out_thrust);
            motor3_power(out_thrust);
            motor4_power(out_thrust);

            if (time.current - referenceTime > 10000) {

                out_thrust = duty_to_thrust(0);

                motor1_power(out_thrust);
                motor2_power(out_thrust);
                motor3_power(out_thrust);
                motor4_power(out_thrust);
            }

            break;

        case INPUT_TEST:

            out_thrust = duty_to_thrust(inputDuty);

            motor1_power(out_thrust);
            motor2_power(out_thrust);
            motor3_power(out_thrust);
            motor4_power(out_thrust);

            /*

            if (duty_to_thrust(inputDuty) > old_out_thrust) {

                while (duty_to_thrust(inputDuty) != old_out_thrust) {

                    out_thrust = duty_to_thrust(inputDuty);

                    motor1_power(out_thrust);
                    motor2_power(out_thrust);
                    motor3_power(out_thrust);
                    motor4_power(out_thrust);

                }

                old_out_thrust = out_thrust;
            }

            else {

                while (duty_to_thrust(inputDuty) != old_out_thrust) {

                    out_thrust = duty_to_thrust(inputDuty);

                    motor1_power(out_thrust);
                    motor2_power(out_thrust);
                    motor3_power(out_thrust);
                    motor4_power(out_thrust);

                    inputDuty++;
                }

                old_out_thrust = out_thrust;
            }

            */

            break;

        default:
            
            break;

    }
}



float PID_Measure(PID *pid, float setpoint, float measurement)
{

    /*
     * Error signal
     */
    float error = setpoint - measurement;

    /*
     * Proportional
     */
    float proportional = pid->Kp * error;

    /*
     * Integral
     */
    pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError);

    /* Anti-wind-up via integrator clamping */
    if (pid->integrator > pid->limMaxInt)
    {

        pid->integrator = pid->limMaxInt;
    }
    else if (pid->integrator < pid->limMinInt)
    {

        pid->integrator = pid->limMinInt;
    }

    /*
     * Derivative (band-limited differentiator)
     */

    pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement) /* Note: derivative on measurement, therefore minus sign in front of equation! */
                            + (2.0f * pid->tau - pid->T) * pid->differentiator) /
                          (2.0f * pid->tau + pid->T);

    /*
     * Compute output and apply limits
     */
    pid->out = proportional + pid->integrator + pid->differentiator;

    if (pid->out > pid->limMax)
    {

        pid->out = pid->limMax;
    }
    else if (pid->out < pid->limMin)
    {

        pid->out = pid->limMin;
    }

    /* Store error and measurement for later use */
    pid->prevError = error;
    pid->prevMeasurement = measurement;

    /* Return controller output */
    return pid->out;
}

void init_servo()
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 500);
}

void servo_separate()
{
    out_thrust = servo_duty_to_thrust(100);
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, out_thrust);
}

void servo_revert()
{
    out_thrust = servo_duty_to_thrust(0);
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, out_thrust);
}
