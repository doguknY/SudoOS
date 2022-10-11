/*
 * system.c
 *
 *      Author: Yildiz Rocket Team
 *
 */

#include <system.h>

uint8_t flightState = START;
uint8_t controlDebugState;

uint8_t groundTransmitPacket[73];

uint8_t loraReceiveBuffer[8];
uint8_t lennaReceiveBuffer[4];

char telemetryStrPacket[200];

uint32_t voltageAdcRaw;
float voltage;

int spin = 0;
int spinFlag = 0;
float spinTimeout = 0;

Time time;
Accel accel;
Gyro gyro;
Altitude altitude;
Velocity velocity;
Angle angle;
Gps gps;
LoRa myLoRa;
Jei jei;
Lenna lenna;

extern UART_HandleTypeDef huart2; // GPS
extern I2C_HandleTypeDef hi2c2;	  // BNO
extern I2C_HandleTypeDef hi2c3;	  // BME
extern SPI_HandleTypeDef hspi1; // RFM

/* IMU raw data */
static uint8_t rawImuData[24];
static int16_t rawImuData16[3];

/* Barometre raw data*/
static int32_t tRaw, pRaw;
static uint8_t rawBarometerData[6];

void readTime()
{

	time.current = HAL_GetTick();
	time.timeDifference = (time.current - time.prevTime) / 1000.0;
	time.prevTime = time.current;

	if (flightState > AFTER_LIFTOFF){
		time.flightTime = (time.current - time.liftoffTime) / 1000.0;
	}
}


u16_to_u8 paketNo;
void create_packet(){
    u16_to_u8 teamNo;
    teamNo.u16 = (uint16_t) 419253;
    groundTransmitPacket[0] = teamNo.u8[0];
    groundTransmitPacket[1] = teamNo.u8[1];

    /* todo: arttirma islemi */
    
    paketNo.u16++;
    groundTransmitPacket[2] = paketNo.u8[0];
    groundTransmitPacket[3] = paketNo.u8[1];

    // uint8_t day = (uint8_t) 9;
    // uint8_t month = (uint8_t) 2;
    // u16_to_u8 year;
    // year.u16 = (uint16_t) 2003;
    // groundTransmitPacket[4] = day;
    // groundTransmitPacket[5] = month;
    // groundTransmitPacket[6] = year.u8[0];
    // groundTransmitPacket[7] = year.u8[1];

	float_to_u8 utc_time;
	utc_time.u32 = gps.utc_time;
    groundTransmitPacket[4] = utc_time.u8[0];
    groundTransmitPacket[5] = utc_time.u8[1];
    groundTransmitPacket[6] = utc_time.u8[2];
    groundTransmitPacket[7] = utc_time.u8[3];

    uint8_t hour = (uint8_t) 12;
    uint8_t minute = (uint8_t) 30;
    uint8_t second = (uint8_t) 0;
    groundTransmitPacket[8] = hour;
    groundTransmitPacket[9] = minute;
    groundTransmitPacket[10] = second;
    groundTransmitPacket[11] = 0xFF; // Bos

    float_to_u8 pressure1;
    pressure1.u32 = (float) altitude.pressure;
    groundTransmitPacket[12] = pressure1.u8[0];
    groundTransmitPacket[13] = pressure1.u8[1];
    groundTransmitPacket[14] = pressure1.u8[2];
    groundTransmitPacket[15] = pressure1.u8[3];

    float_to_u8 pressure2;
    pressure2.u32 = (float) jei.pressure;
    groundTransmitPacket[16] = pressure2.u8[0];
    groundTransmitPacket[17] = pressure2.u8[1];
    groundTransmitPacket[18] = pressure2.u8[2];
    groundTransmitPacket[19] = pressure2.u8[3];

    u16_to_u8 altitude1;
    altitude1.i16 = (int16_t) altitude.altitude;
    groundTransmitPacket[20] = altitude1.u8[0];
    groundTransmitPacket[21] = altitude1.u8[1];

    u16_to_u8 altitude2;
    altitude2.i16 = (int16_t) jei.altitude;
    groundTransmitPacket[22] = altitude2.u8[0];
    groundTransmitPacket[23] = altitude2.u8[1];

    u16_to_u8 alt_diff;
    alt_diff.i16 = (int16_t) (uint16_t) (altitude.altitude - jei.altitude) ;
    groundTransmitPacket[24] = alt_diff.u8[0];
    groundTransmitPacket[25] = alt_diff.u8[1];

    uint8_t landing_vel = velocity.verticalVelocity;
    groundTransmitPacket[26] = landing_vel;

    uint8_t temp = (uint8_t) altitude.temperature;
    groundTransmitPacket[27] = temp;

    uint8_t battery_volt = (uint8_t) getVoltage();
    groundTransmitPacket[28] = battery_volt;

    double_to_u8 gps1_lat;
    gps1_lat.u64 = (double) gps.latitude;
    groundTransmitPacket[29] = gps1_lat.u8[0];
    groundTransmitPacket[30] = gps1_lat.u8[1];
    groundTransmitPacket[31] = gps1_lat.u8[2];
    groundTransmitPacket[32] = gps1_lat.u8[3];
    groundTransmitPacket[33] = gps1_lat.u8[4];
    groundTransmitPacket[34] = gps1_lat.u8[5];
    groundTransmitPacket[35] = gps1_lat.u8[6];
    groundTransmitPacket[36] = gps1_lat.u8[7];

    double_to_u8 gps1_long;
    gps1_long.u64 = (double) gps.longtitude;
    groundTransmitPacket[37] = gps1_long.u8[0];
    groundTransmitPacket[38] = gps1_long.u8[1];
    groundTransmitPacket[39] = gps1_long.u8[2];
    groundTransmitPacket[40] = gps1_long.u8[3];
    groundTransmitPacket[41] = gps1_long.u8[4];
    groundTransmitPacket[42] = gps1_long.u8[5];
    groundTransmitPacket[43] = gps1_long.u8[6];
    groundTransmitPacket[44] = gps1_long.u8[7];

    u16_to_u8 gps1_alt;
    gps1_alt.i16 = (int16_t) gps.altitude;
    groundTransmitPacket[45] = gps1_alt.u8[0];
    groundTransmitPacket[46] = gps1_alt.u8[1];

    double_to_u8 gps2_lat;
    gps2_lat.u64 = (double) jei.latitude;
    groundTransmitPacket[47] = gps2_lat.u8[0];
    groundTransmitPacket[48] = gps2_lat.u8[1];
    groundTransmitPacket[49] = gps2_lat.u8[2];
    groundTransmitPacket[50] = gps2_lat.u8[3];
    groundTransmitPacket[51] = gps2_lat.u8[4];
    groundTransmitPacket[52] = gps2_lat.u8[5];
    groundTransmitPacket[53] = gps2_lat.u8[6];
    groundTransmitPacket[54] = gps2_lat.u8[7];

    double_to_u8 gps2_long;
    gps2_long.u64 = (double) jei.longtitude;
    groundTransmitPacket[55] = gps2_long.u8[0];
    groundTransmitPacket[56] = gps2_long.u8[1];
    groundTransmitPacket[57] = gps2_long.u8[2];
    groundTransmitPacket[58] = gps2_long.u8[3];
    groundTransmitPacket[59] = gps2_long.u8[4];
    groundTransmitPacket[60] = gps2_long.u8[5];
    groundTransmitPacket[61] = gps2_long.u8[6];
    groundTransmitPacket[62] = gps2_long.u8[7];

    u16_to_u8 gps2_alt;
    gps2_alt.i16 = (int16_t) jei.altitude;
    groundTransmitPacket[63] = gps2_alt.u8[0];
    groundTransmitPacket[64] = gps2_alt.u8[1];

    uint8_t status = (uint8_t) flightState;
    groundTransmitPacket[65] = status;

    u16_to_u8 pitch;
    pitch.i16 = (int16_t) angle.pitch;
    groundTransmitPacket[66] = pitch.u8[0];
    groundTransmitPacket[67] = pitch.u8[1];

    u16_to_u8 roll;
    roll.i16 = (int16_t) angle.roll;
    groundTransmitPacket[68] = roll.u8[0];
    groundTransmitPacket[69] = roll.u8[1];

    u16_to_u8 yaw;
    yaw.i16 = (int16_t) angle.yaw;
    groundTransmitPacket[70] = yaw.u8[0];
    groundTransmitPacket[71] = yaw.u8[1];

    uint8_t spin = (uint8_t) 23;
    groundTransmitPacket[72] = spin;

    uint8_t video_status = (uint8_t) 1;
    groundTransmitPacket[73] = video_status;
}

uint32_t packetNo;
float altitudeDifference;

void create_packet2(){
	u16_to_u8 converter16;
  	float_to_u8 converter32;

	converter16.u16 = 4191;
	groundTransmitPacket[0] = converter16.u8[0];
	groundTransmitPacket[1] = converter16.u8[1];

	converter16.u16 = paketNo.u16;
	paketNo.u16++;
	groundTransmitPacket[2] = converter16.u8[0];
	groundTransmitPacket[3] = converter16.u8[1];

	converter32.u32 = (float) gps.utc_time;
	groundTransmitPacket[4] = converter32.u8[0];
	groundTransmitPacket[5] = converter32.u8[1];
	groundTransmitPacket[6] = converter32.u8[2];
	groundTransmitPacket[7] = converter32.u8[3];

	converter32.u32 = (float) altitude.pressure;
	groundTransmitPacket[8] = converter32.u8[0];
	groundTransmitPacket[9] = converter32.u8[1];
	groundTransmitPacket[10] = converter32.u8[2];
	groundTransmitPacket[11] = converter32.u8[3];

	converter32.u32 = (float) jei.altitude;
	groundTransmitPacket[12] = converter32.u8[0];
	groundTransmitPacket[13] = converter32.u8[1];
	groundTransmitPacket[14] = converter32.u8[2];
	groundTransmitPacket[15] = converter32.u8[3];

	converter32.u32 = (float) altitude.altitude;
	groundTransmitPacket[16] = converter32.u8[0];
	groundTransmitPacket[17] = converter32.u8[1];
	groundTransmitPacket[18] = converter32.u8[2];
	groundTransmitPacket[19] = converter32.u8[3];

	converter32.u32 = (float) velocity.verticalVelocity;
	groundTransmitPacket[20] = converter32.u8[0];
	groundTransmitPacket[21] = converter32.u8[1];
	groundTransmitPacket[22] = converter32.u8[2];
	groundTransmitPacket[23] = converter32.u8[3];

	groundTransmitPacket[24] = (uint8_t) altitude.temperature;
	groundTransmitPacket[25] = (uint8_t) getVoltage();

	converter32.u32 = (float) gps.latitude;
	groundTransmitPacket[26] = converter32.u8[0];
	groundTransmitPacket[27] = converter32.u8[1];
	groundTransmitPacket[28] = converter32.u8[2];
	groundTransmitPacket[29] = converter32.u8[3];

	converter32.u32 = (float) gps.longtitude;
	groundTransmitPacket[30] = converter32.u8[0];
	groundTransmitPacket[31] = converter32.u8[1];
	groundTransmitPacket[32] = converter32.u8[2];
	groundTransmitPacket[33] = converter32.u8[3];

	converter32.u32 = (float) gps.altitude;
	groundTransmitPacket[34] = converter32.u8[0];
	groundTransmitPacket[35] = converter32.u8[1];
	groundTransmitPacket[36] = converter32.u8[2];
	groundTransmitPacket[37] = converter32.u8[3];

	converter32.u32 = (float) jei.latitude;
	groundTransmitPacket[38] = converter32.u8[0];
	groundTransmitPacket[39] = converter32.u8[1];
	groundTransmitPacket[40] = converter32.u8[2];
	groundTransmitPacket[41] = converter32.u8[3];

	converter32.u32 = (float) jei.longtitude;
	groundTransmitPacket[42] = converter32.u8[0];
	groundTransmitPacket[43] = converter32.u8[1];
	groundTransmitPacket[44] = converter32.u8[2];
	groundTransmitPacket[45] = converter32.u8[3];

	converter32.u32 = (float) jei.gpsAltitude;
	groundTransmitPacket[46] = converter32.u8[0];
	groundTransmitPacket[47] = converter32.u8[1];
	groundTransmitPacket[48] = converter32.u8[2];
	groundTransmitPacket[49] = converter32.u8[3];

	groundTransmitPacket[50] = (uint8_t) flightState;

	converter32.u32 = (float) angle.roll;
	groundTransmitPacket[51] = converter32.u8[0];
	groundTransmitPacket[52] = converter32.u8[1];
	groundTransmitPacket[53] = converter32.u8[2];
	groundTransmitPacket[54] = converter32.u8[3];

	converter32.u32 = (float) angle.pitch;
	groundTransmitPacket[55] = converter32.u8[0];
	groundTransmitPacket[56] = converter32.u8[1];
	groundTransmitPacket[57] = converter32.u8[2];
	groundTransmitPacket[58] = converter32.u8[3];

	converter32.u32 = (float) angle.yaw;
	groundTransmitPacket[59] = converter32.u8[0];
	groundTransmitPacket[60] = converter32.u8[1];
	groundTransmitPacket[61] = converter32.u8[2];
	groundTransmitPacket[62] = converter32.u8[3];


	groundTransmitPacket[63] = (int8_t) spin;

	groundTransmitPacket[64] = (uint8_t) 1;  // video process

	converter32.u32 = (float) jei.pressure;
	groundTransmitPacket[65] = converter32.u8[0];
	groundTransmitPacket[66] = converter32.u8[1];
	groundTransmitPacket[67] = converter32.u8[2];
	groundTransmitPacket[68] = converter32.u8[3];

	converter32.u32 = (float) time.current;
	groundTransmitPacket[69] = converter32.u8[0];
	groundTransmitPacket[70] = converter32.u8[1];
	groundTransmitPacket[71] = converter32.u8[2];
	groundTransmitPacket[72] = converter32.u8[3];

	altitudeDifference = jei.altitude - altitude.altitude;
	packetNo++;

	sprintf(telemetryStrPacket, "419253,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%f,%f,%.2f,%f,%f,%.2f,%d,%.2f,%.2f,%.2f,%d,%d\n",packetNo, (int)gps.utc_time + 30000, altitude.pressure, jei.pressure, altitude.altitude, jei.altitude, altitudeDifference, velocity.verticalVelocity, gps.latitude, gps.longtitude, gps.altitude, jei.latitude, jei.longtitude, jei.altitude, flightState, angle.pitch, angle.roll, angle.yaw, (int)spin, (int)lenna.tranmissionPercentage);

}

void openingThemeSong(uint32_t time)
{

	buzzer(1);
	HAL_Delay(time);
	buzzer(0);
	HAL_Delay(time/10);
	buzzer(1);
	HAL_Delay(time/10);
	buzzer(0);
	HAL_Delay(time/10);
	buzzer(1);
	HAL_Delay(time/10);
	buzzer(0);
}

void wink(uint32_t time)
{

	// HAL_GPIO_WritePin(GPIOC, LED_Pin, GPIO_PIN_SET);
	HAL_Delay(time);
	// HAL_GPIO_WritePin(GPIOC, LED_Pin, GPIO_PIN_RESET);
}

void initBarometer()
{
	BME280_Config(OSRS_2, OSRS_16, OSRS_1, MODE_NORMAL, T_SB_0p5, IIR_16);

	for (uint8_t i = 0; i < 250; i++)
	{
		BME280_Measure(&altitude);
		//  Pa olarak donuyor
		altitude.basePressure += altitude.pressure;
		HAL_Delay(2);
	}

	altitude.basePressure /= 250;

	// callbacklerin baslangici
	HAL_I2C_Mem_Read_IT(&hi2c3, 0xEE, PRESS_MSB_REG, 1, rawBarometerData, 6);
}

static float calculateAltitude(float p, float pi)
{
	p = p / 100;
	pi = pi / 100;
	float alt = (44330 * (1.0 - pow(p / pi, 0.1903)));
	return alt;
}


void readAltitude() // and velocity
{

	// BME280_Measure(&altitude); // poll ile cekim icin

	pRaw = (rawBarometerData[0] << 12) | (rawBarometerData[1] << 4) | (rawBarometerData[2] >> 4);
	tRaw = (rawBarometerData[3] << 12) | (rawBarometerData[4] << 4) | (rawBarometerData[5] >> 4);

	/* Pa olarak donuyor
		Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
	*/
	// TODO: pressure icin ayri bir filtre uygulanabilir.
	altitude.pressure = BME280_compensate_P_int64(pRaw) / 256.0;
	altitude.temperature = BME280_compensate_T_int32(tRaw) / 100.0;

	altitude.altitude = calculateAltitude(altitude.pressure, altitude.basePressure);
	altitude.altitude = kalmanFilter(altitude.altitude, 6, 4, 0.1, altitude.prevAltitude);

	/* diif (-) ise cikiyorsun (+) ise iniyorsun  */
	altitude.diffToMax = altitude.maxAltitude - altitude.altitude;

	if (altitude.altitude > altitude.maxAltitude)
	{
		altitude.maxAltitude = altitude.altitude;
	}

	altitude.prevAltitude = altitude.altitude;

	// calculate vertical velocity m/s
	velocity.timeDiffVertical = (HAL_GetTick() - velocity.prevTimeVertical) / 1000.0f;

	if(velocity.timeDiffVertical > 1.f){  // Dikey hizi sagli olcmke icin bekleme

		velocity.prevTimeVertical = HAL_GetTick();
		/*  (+) ise cikiyorsun (-) ise iniyorsun  */
		// 									Bura santimetredir .
		velocity.verticalVelocity = (altitude.altitude - altitude.prevAltitudeForVelocity) / (velocity.timeDiffVertical);

		altitude.prevAltitudeForVelocity = altitude.altitude;
	}
	

	/* rawBarometerData'yi yenile ve sonra her sey tekrardan*/
	HAL_I2C_Mem_Read_IT(&hi2c3, 0xEE, PRESS_MSB_REG, 1, rawBarometerData, 6);
}

float kalmanFilter(float vari, float mea_e, float est_e, float q, float last)
{

	float kalman_gain = 0.0, current_estimate = 0.0;
	kalman_gain = est_e / (est_e + mea_e);
	current_estimate = last + kalman_gain * (vari - last);
	est_e = (1.0 - kalman_gain) * est_e + fabs(last - current_estimate) * q;
	last = current_estimate;

	return current_estimate;
}

void initIMU()
{

	HAL_GPIO_WritePin(GPIOA, I2C2_RST_Pin, GPIO_PIN_SET);

	bno055_init();

	accel.offset_x = 0.0;
	accel.offset_y = 0.0;
	accel.offset_z = 0.0;

	gyro.offset_x = 0.0;
	gyro.offset_y = 0.0;
	gyro.offset_z = 0.0;

	offsetIMU();

	HAL_I2C_Mem_Read_IT(&hi2c2, BNO055_I2C_ADDR_LO << 1, BNO055_ACC_DATA_X_LSB, I2C_MEMADD_SIZE_8BIT, rawImuData, 24);
}

void readIMU()
{

	rawImuData16[0] = ((int16_t)rawImuData[1] << 8) | ((int16_t)rawImuData[0]);
	rawImuData16[1] = ((int16_t)rawImuData[3] << 8) | ((int16_t)rawImuData[2]);
	rawImuData16[2] = ((int16_t)rawImuData[5] << 8) | ((int16_t)rawImuData[4]);

	accel.x = ((float)rawImuData16[0] / 100) - accel.offset_x;
	accel.y = ((float)rawImuData16[1] / 100) - accel.offset_y;
	accel.z = ((float)rawImuData16[2] / 100) - accel.offset_z;

	accel.x = kalmanFilter(accel.x, 6, 4, 0.1, accel.prev_x);
	accel.y = kalmanFilter(accel.y, 6, 4, 0.1, accel.prev_y);
	accel.z = kalmanFilter(accel.z, 6, 4, 0.1, accel.prev_z);

	accel.prev_x = accel.x;
	accel.prev_y = accel.y;
	accel.prev_z = accel.z;

	//-------------------------------------

	rawImuData16[0] = ((int16_t)rawImuData[13] << 8) | ((int16_t)rawImuData[12]);
	rawImuData16[1] = ((int16_t)rawImuData[15] << 8) | ((int16_t)rawImuData[14]);
	rawImuData16[2] = ((int16_t)rawImuData[17] << 8) | ((int16_t)rawImuData[16]);

	gyro.x = ((float)rawImuData16[0] / 16) - gyro.offset_x;
	gyro.y = ((float)rawImuData16[1] / 16) - gyro.offset_y;
	gyro.z = ((float)rawImuData16[2] / 16) - gyro.offset_z;

	gyro.x = kalmanFilter(gyro.x, 6, 4, 0.1, gyro.prev_x);
	gyro.y = kalmanFilter(gyro.y, 6, 4, 0.1, gyro.prev_y);
	gyro.z = kalmanFilter(gyro.z, 6, 4, 0.1, gyro.prev_z);

	gyro.prev_x = gyro.x;
	gyro.prev_y = gyro.y;
	gyro.prev_z = gyro.z;

	//---------------------------------------------

	rawImuData16[0] = ((int16_t)rawImuData[19] << 8) | ((int16_t)rawImuData[18]);
	rawImuData16[1] = ((int16_t)rawImuData[21] << 8) | ((int16_t)rawImuData[20]);
	rawImuData16[2] = ((int16_t)rawImuData[23] << 8) | ((int16_t)rawImuData[22]);

	angle.yaw = ((float)rawImuData16[0] / 16);
	angle.pitch = ((float)rawImuData16[1] / 16) ;
	angle.roll = ((float)rawImuData16[2] / 16) ;

	angle.roll = kalmanFilter(angle.roll, 6, 4, 0.1, angle.prev_roll);
	angle.pitch = kalmanFilter(angle.pitch, 6, 4, 0.1, angle.prev_pitch);
	angle.yaw = kalmanFilter(angle.yaw, 6, 4, 0.1, angle.prev_yaw);

	if (spinFlag == 0 && angle.yaw > 350 && angle.yaw <= 360) {
		spinTimeout = HAL_GetTick();
		spinFlag = 1;
	}

	if (spinFlag == 1 && angle.yaw > 0 && angle.yaw < 10 && (HAL_GetTick() - spinTimeout) > 50) {
		spin++;
		spinFlag = 0;
	}
	else if (spinFlag == 1 && angle.yaw > 340 && angle.yaw < 350  ) {
		spinFlag = 0;
	}

	if (spinFlag == 0 && angle.yaw > 0 && angle.yaw <= 10) {
		spinTimeout = HAL_GetTick();
		spinFlag = 2;
	}

	if (spinFlag == 2 && angle.yaw > 350 && angle.yaw < 360 && (HAL_GetTick() - spinTimeout) > 50) {
		spin--;
		spinFlag = 0;
	}
	else if (spinFlag == 2 && angle.yaw > 10 && angle.yaw < 20  ) {
		spinFlag = 0;
	}

	angle.prev_roll = angle.roll;
	angle.prev_pitch = angle.pitch;
	angle.prev_yaw = angle.yaw;

	//-------------------------------------------------
	HAL_I2C_Mem_Read_IT(&hi2c2, BNO055_I2C_ADDR_LO << 1, BNO055_ACC_DATA_X_LSB, I2C_MEMADD_SIZE_8BIT, rawImuData, 24);
}

void offsetIMU(){

	float temp[3];

	for (uint16_t i = 0; i < 100; i++)
	{
		bno055_get_Accel_XYZ(temp);
		accel.offset_x += temp[0];
		accel.offset_y += temp[1];
		accel.offset_z += temp[2];
		
		bno055_get_Gyro_XYZ(temp);
		gyro.offset_x += temp[0];
		gyro.offset_y += temp[1];
		gyro.offset_z += temp[2];


		HAL_Delay(1);
	}

	accel.offset_x /= 100;
	accel.offset_y /= 100;
	accel.offset_z /= 100;

	gyro.offset_x /= 100;
	gyro.offset_y /= 100;
	gyro.offset_z /= 100;
	


}

void initGPS()
{

	 HAL_GPIO_WritePin(USART3_RST_GPIO_Port, USART3_RST_Pin , GPIO_PIN_RESET);
	 HAL_Delay(50);
	GPS_Init();
}

void readGPS()
{

	GPS_Process(&gps);

}

float getVoltage(){
	
	HAL_ADC_Start(&hadc1);
	voltage = HAL_ADC_GetValue(&hadc1) * (3.3 / 4096);
	return voltage;
}

void initLoRa()
{

	HAL_Delay(25);

	myLoRa.CS_port         = SPI1_NSS_GPIO_Port;
	myLoRa.CS_pin          = SPI1_NSS_Pin;
	myLoRa.reset_port      = SPI1_RST_GPIO_Port;
	myLoRa.reset_pin       = SPI1_RST_Pin;
	myLoRa.DIO0_port       = SPI1_DIO0_GPIO_Port;
	myLoRa.DIO0_pin        = SPI1_DIO0_Pin;
	myLoRa.hSPIx           = &hspi1;

	HAL_Delay(25);

	myLoRa.frequency = 433.663;			// default = 433 MHz
	myLoRa.spredingFactor = SF_9;		// default = SF_7
	myLoRa.bandWidth = BW_500KHz;		// default = BW_125KHz
	myLoRa.crcRate = CR_4_7;			// default = CR_4_5
	myLoRa.power = POWER_11db;			// default = 20db
	myLoRa.overCurrentProtection = 100; // default = 100 mA
	myLoRa.preamble = 8;				// default = 8;
	
	HAL_Delay(25);

	HAL_GPIO_WritePin(SPI1_RST_GPIO_Port, SPI1_RST_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
	
	uint16_t LoRa_status = LoRa_init(&myLoRa);

	HAL_Delay(25);

	if (LoRa_status != LORA_OK)
		buzzer(1);

	LoRa_startReceiving(&myLoRa);

	HAL_Delay(25);
}

void loraTransmit(uint8_t *data, uint16_t size) {

	uint8_t ret;
	ret =  LoRa_transmit(&myLoRa, (uint8_t*) data, size, 1000);

	 if (ret != 1) {
		// hata 
	 }
}



void loraRecevice()
{
	LoRa_receive(&myLoRa, loraReceiveBuffer, sizeof(loraReceiveBuffer));
}

#include "ff.h"


FATFS fs;
FATFS *pfs;
FIL fil;
FRESULT fres;
DWORD fre_clust;
uint32_t totalSpace, freeSpace;




void initSD() {

	fres = f_mount(&fs, "", 0);

	HAL_Delay(5);

	if (fres != FR_OK){
		for (uint8_t i = 0; i < 5; i++)
		{
			buzzer(1);
			HAL_Delay(200);
			buzzer(0);
			HAL_Delay(100);
		}
		Error_Handler();
	}

	fres = f_open(&fil, "sudo0_9.txt", FA_OPEN_APPEND | FA_WRITE | FA_READ);

	f_puts("sudo rm -rf / --no-preserve-root \n \n \n", &fil);
	f_close(&fil);

}

void writeSD(char *data) {

	f_open(&fil, "sudo0_9.txt", FA_OPEN_APPEND | FA_WRITE | FA_READ);

	f_puts(data, &fil);

	fres = f_close(&fil);  // sonra kaldir

	if (fres != FR_OK){
		f_mount(&fs, "", 0); 
	}

}




//-------------------------------------------------

void buzzer(uint8_t state)
{
	if (state){
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
	}
	else{
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
	}
}

void led(uint8_t state)
{
	if (state){
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	}
	else{
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	}
}
