//       author: dogu

#include "bno055.h"

// BNO055'in proje generate ederken olusan handleri
extern I2C_HandleTypeDef hi2c2;
#define BNO055_I2C &hi2c2

// I2C adresi pg 90  maybe 0x40
#define BNO055_ADDRESS_1 0x28
#define BNO055_ADDRESS_2 0x29

// pg 29
const uint8_t GyroPowerMode = NormalG;
const uint8_t GyroRange = GFS_1000DPS;
const uint8_t GyroBandwith = GBW_523Hz;

// pg 28
const uint8_t AccelRange = AFS_16G;
const uint8_t AccelMode = NormalA;
const uint8_t AccelBandwith = ABW_1000Hz;

const uint8_t MagRate = MODR_30Hz;
const uint8_t MagOperMode = EnhancedRegular;
const uint8_t MagPwrMode = Normal;

// pg21
const uint8_t PWRMode = Normalpwr;
const uint8_t OPRMode = AMG;

static uint8_t data[6];
static int16_t rawData[3];

void bno055_iic_write_byte(uint8_t reg, uint8_t data) {
    HAL_I2C_Mem_Write(BNO055_I2C, BNO055_ADDRESS_1 << 1, reg,
                      I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    HAL_Delay(5);
}

void bno055_iic_read_byte(uint8_t reg, uint8_t *data) {
    HAL_I2C_Mem_Read(BNO055_I2C, BNO055_ADDRESS_1 << 1, reg,
                     I2C_MEMADD_SIZE_8BIT, data, 1, 100);
}

void _setPage(uint8_t page) {
    bno055_iic_write_byte(BNO055_PAGE_ID, page);
    HAL_Delay(30);
}

void _setMode(uint8_t mode) {
    bno055_iic_write_byte(BNO055_OPR_MODE, mode);
    HAL_Delay(70);
}

void _config(void) {
    _setPage(1);

    uint8_t accelConf =
        (AccelRange << 0) | (AccelMode << 5) | (AccelBandwith << 2);
    bno055_iic_write_byte(BNO055_ACC_CONFIG, accelConf);
    HAL_Delay(30);

    uint8_t gyroConf = (GyroRange << 0) | (GyroBandwith << 3);
    bno055_iic_write_byte(BNO055_GYRO_CONFIG_0, gyroConf);
    HAL_Delay(30);

    uint8_t gyroConf2 = GyroPowerMode << 0;
    bno055_iic_write_byte(BNO055_GYRO_CONFIG_1, gyroConf2);
    HAL_Delay(30);

    uint8_t magConf = (MagRate << 0) | (MagOperMode << 3) | (MagPwrMode << 5);
    bno055_iic_write_byte(BNO055_MAG_CONFIG, magConf);
    HAL_Delay(30);

    _setPage(0);
}

void _set_mode(uint8_t mode) {
    bno055_iic_write_byte(BNO055_OPR_MODE, mode);
    HAL_Delay(30);
}

uint16_t bno055_init(void) {
    bno055_iic_write_byte(BNO055_SYS_TRIGGER, 0x20);
    HAL_Delay(100);

    uint8_t id;
    bno055_iic_read_byte(BNO055_CHIP_ID, &id);
    HAL_Delay(30);

    _setMode(CONFIGMODE);

    // trigger reset

    // power mode normal
    bno055_iic_write_byte(BNO055_PWR_MODE, PWRMode);
    HAL_Delay(15);

    _setPage(0);

    // todo nereye koycagin
    _config();

    // trigger
    bno055_iic_write_byte(BNO055_SYS_TRIGGER, 0x0);
    HAL_Delay(20);

    // vertical mode
    bno055_iic_write_byte(BNO055_AXIS_MAP_CONFIG, 0x21);

    //    bno055_iic_write_byte(BNO055_AXIS_MAP_SIGN, 0x0);

    HAL_Delay(20);
    bno055_iic_read_byte(BNO055_AXIS_MAP_CONFIG, &id);

    _setMode(OPRMode);
    HAL_Delay(50);

    //    _config();

    return id;
}

void bno055_get_Accel_XYZ(float xyz[3]) {
    /** Sensorde sayfa 33 den itibaren gosterildigi gibi her deger 2 bytedir(16
     * bit). Her deger 8 bitlik iki parcaya ayrilip bu parcalar registerlara
     * konur. Mesala ivme sensorunun degeri binary olarak ikiye ayrilir: MSB ve
     * LSB (binarynin buyuk kismi(sol) MSBdir) Iki registerdan veri cekip
     * bunlari 16 bitlik hale ceviririz. Sonra MSB tarafini 8 bit sola kaydirip
     * LSB ile 'veya' yardimiyla birlestirirz birlestiririz. Bu saf degerimizi
     * verir safi isleyip istenilen formata getirip kullaniriz.
     * **/

    for (uint8_t i = 0; i < 6; i++) {
        bno055_iic_read_byte(BNO055_ACC_DATA_X_LSB + i, data + i);
    }

    rawData[0] = ((int16_t)data[1] << 8) | ((int16_t)data[0]);
    rawData[1] = ((int16_t)data[3] << 8) | ((int16_t)data[2]);
    rawData[2] = ((int16_t)data[5] << 8) | ((int16_t)data[4]);

    xyz[0] = (float)rawData[0] / 100;
    xyz[1] = (float)rawData[1] / 100;
    xyz[2] = (float)rawData[2] / 100;
}

void bno055_get_Gyro_XYZ(float xyz[3]) {
    // Accelometre ile mantigi ayni

    for (uint8_t i = 0; i < 6; i++) {
        bno055_iic_read_byte(BNO055_GYR_DATA_X_LSB + i, data + i);
    }

    rawData[0] = ((int16_t)data[1] << 8) | ((int16_t)data[0]);
    rawData[1] = ((int16_t)data[3] << 8) | ((int16_t)data[2]);
    rawData[2] = ((int16_t)data[5] << 8) | ((int16_t)data[4]);

    xyz[0] = (float)rawData[0] / 16;
    xyz[1] = (float)rawData[1] / 16;
    xyz[2] = (float)rawData[2] / 16;
}

void bno055_get_Mag_XYZ(float xyz[3]) {
    for (uint8_t i = 0; i < 6; i++) {
        bno055_iic_read_byte(BNO055_MAG_DATA_X_LSB + i, data + i);
    }

    rawData[0] = ((int16_t)data[1] << 8) | ((int16_t)data[0]);
    rawData[1] = ((int16_t)data[3] << 8) | ((int16_t)data[2]);
    rawData[2] = ((int16_t)data[5] << 8) | ((int16_t)data[4]);

    xyz[0] = (float)rawData[0] / 16;
    xyz[1] = (float)rawData[1] / 16;
    xyz[2] = (float)rawData[2] / 16;
}

void bno055_get_Euler_XYZ(float xyz[3]) {
    for (uint8_t i = 0; i < 6; i++) {
        bno055_iic_read_byte(BNO055_EUL_HEADING_LSB + i, data + i);
    }

    rawData[0] = ((int16_t)data[1] << 8) | ((int16_t)data[0]);
    rawData[1] = ((int16_t)data[3] << 8) | ((int16_t)data[2]);
    rawData[2] = ((int16_t)data[5] << 8) | ((int16_t)data[4]);

    xyz[0] = (float)rawData[0] / 16;
    xyz[1] = (float)rawData[1] / 16;
    xyz[2] = (float)rawData[2] / 16;
}

void bno055_get_Qua_XYZ(float xyz[3]) {
    for (uint8_t i = 0; i < 6; i++) {
        bno055_iic_read_byte(BNO055_QUA_DATA_W_LSB + i, data + i);
    }

    rawData[0] = ((int16_t)data[1] << 8) | ((int16_t)data[0]);
    rawData[1] = ((int16_t)data[3] << 8) | ((int16_t)data[2]);
    rawData[2] = ((int16_t)data[5] << 8) | ((int16_t)data[4]);

    xyz[0] = (float)rawData[0] / (1 << 14);
    xyz[1] = (float)rawData[1] / (1 << 14);
    xyz[2] = (float)rawData[2] / (1 << 14);
}
