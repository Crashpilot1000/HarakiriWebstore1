#include "board.h"
#include "mw.h"

// MPU3050, Standard address 0x68
#define MPU3050_ADDRESS         0x68

// Registers
#define MPU3050_SMPLRT_DIV      0x15
#define MPU3050_DLPF_FS_SYNC    0x16
#define MPU3050_INT_CFG         0x17
#define MPU3050_TEMP_OUT        0x1B
#define MPU3050_GYRO_OUT        0x1D
#define MPU3050_USER_CTRL       0x3D
#define MPU3050_PWR_MGM         0x3E

// Bits
#define MPU3050_FS_SEL_2000DPS  0x18
#define MPU3050_DLPF_10HZ       0x05
#define MPU3050_DLPF_20HZ       0x04
#define MPU3050_DLPF_42HZ       0x03
#define MPU3050_DLPF_98HZ       0x02
#define MPU3050_DLPF_188HZ      0x01
#define MPU3050_DLPF_256HZ      0x00

#define MPU3050_USER_RESET      0x01
#define MPU3050_CLK_SEL_PLL_GX  0x01

static uint8_t mpuLowPassFilter = MPU3050_DLPF_42HZ;

static void mpu3050Init(void);
static void mpu3050Read(int16_t *gyroData);
static void mpu3050Align(int16_t *gyroData);
static void mpu3050ReadTempC100(int16_t *tempData);

bool mpu3050Detect(sensor_t *gyro)
{
    bool ack;
    delay(35);                                                // datasheet page 13 says 20ms. other stuff could have been running meanwhile. but we'll be safe
    ack = i2cWrite(MPU3050_ADDRESS, MPU3050_SMPLRT_DIV, 0);
    if (!ack) return false;
    gyro->init         = mpu3050Init;
    gyro->read         = mpu3050Read;
    gyro->align        = mpu3050Align;
    gyro->senstempC100 = mpu3050ReadTempC100;
    return true;
}

void mpu3050Config(void)
{
    if      (cfg.gy_lpf >= 256) mpuLowPassFilter = MPU3050_DLPF_256HZ;
    else if (cfg.gy_lpf >= 188) mpuLowPassFilter = MPU3050_DLPF_188HZ;
    else if (cfg.gy_lpf >= 98)  mpuLowPassFilter = MPU3050_DLPF_98HZ;
    else if (cfg.gy_lpf >= 42)  mpuLowPassFilter = MPU3050_DLPF_42HZ;
    else if (cfg.gy_lpf >= 20)  mpuLowPassFilter = MPU3050_DLPF_20HZ;
    else if (cfg.gy_lpf >= 10)  mpuLowPassFilter = MPU3050_DLPF_10HZ;
    else mpuLowPassFilter = MPU3050_DLPF_42HZ;
    i2cWrite(MPU3050_ADDRESS, MPU3050_DLPF_FS_SYNC, MPU3050_FS_SEL_2000DPS | mpuLowPassFilter);
}

static void mpu3050Init(void)
{
    bool ack;
    delay(35);                                                // datasheet page 13 says 20ms. other stuff could have been running meanwhile. but we'll be safe
    ack = i2cWrite(MPU3050_ADDRESS, MPU3050_SMPLRT_DIV, 0);
    if (!ack) failureMode(3);
    i2cWrite(MPU3050_ADDRESS, MPU3050_DLPF_FS_SYNC, MPU3050_FS_SEL_2000DPS | mpuLowPassFilter);
    i2cWrite(MPU3050_ADDRESS, MPU3050_INT_CFG, 0);
    i2cWrite(MPU3050_ADDRESS, MPU3050_USER_CTRL, MPU3050_USER_RESET);
    i2cWrite(MPU3050_ADDRESS, MPU3050_PWR_MGM, MPU3050_CLK_SEL_PLL_GX);
}

static void mpu3050Align(int16_t *gyroData)
{
    gyroData[0] =  gyroData[0];                               // official direction is RPY
    gyroData[1] =  gyroData[1];
    gyroData[2] = -gyroData[2];
}

// Read 3 gyro values into user-provided buffer. No overrun checking is done.
static void mpu3050Read(int16_t *gyroData)
{
    uint8_t buf[6];
    i2cRead(MPU3050_ADDRESS, MPU3050_GYRO_OUT, 6, buf);
    gyroData[0] = combine16(buf[0], buf[1]);                  // Changed to full resolution here
    gyroData[1] = combine16(buf[2], buf[3]);
    gyroData[2] = combine16(buf[4], buf[5]);
}

static void mpu3050ReadTempC100(int16_t *tempData)            // Output is in Degree * 100
{
    uint8_t buf[2];
    int32_t temp;
    i2cRead(MPU3050_ADDRESS, MPU3050_TEMP_OUT, 2, buf);
    temp = (int16_t)combine16(buf[0], buf[1]);
    *tempData = 3500 + ((100 * (temp + 13200)) / 280);        // *tempData = 35.0f + (((float)temp + 13200.0f) / 280.0f);
}
