#include "board.h"
#include "mw.h"

// L3G4200D, Standard address 0x68
#define L3G4200D_ADDRESS         0x68
#define L3G4200D_ID              0xD3

// Registers
#define L3G4200D_WHO_AM_I        0x0F
#define L3G4200D_CTRL_REG1       0x20
#define L3G4200D_CTRL_REG2       0x21
#define L3G4200D_CTRL_REG3       0x22
#define L3G4200D_CTRL_REG4       0x23
#define L3G4200D_CTRL_REG5       0x24
#define L3G4200D_REFERENCE       0x25
#define L3G4200D_STATUS_REG      0x27
#define L3G4200D_GYRO_OUT        0x28

// Bits
#define L3G4200D_POWER_ON        0x0F
#define L3G4200D_FS_SEL_2000DPS  0xF0
#define L3G4200D_DLPF_32HZ       0x00
#define L3G4200D_DLPF_54HZ       0x40
#define L3G4200D_DLPF_78HZ       0x80
#define L3G4200D_DLPF_93HZ       0xC0

static uint8_t mpuLowPassFilter = L3G4200D_DLPF_32HZ;

static void l3g4200dInit(void);
static void l3g4200dRead(int16_t *gyroData);
static void l3g4200dAlign(int16_t *gyroData);
static void l3g4200dReadTempC100(int16_t *tempData);

bool l3g4200dDetect(sensor_t *gyro)
{
    uint8_t deviceid;
    delay(25);
    i2cRead(L3G4200D_ADDRESS, L3G4200D_WHO_AM_I, 1, &deviceid);
    if (deviceid != L3G4200D_ID) return false;
    gyro->init         = l3g4200dInit;
    gyro->read         = l3g4200dRead;
    gyro->align        = l3g4200dAlign;
    gyro->senstempC100 = l3g4200dReadTempC100;
    return true;
}

void l3g4200dConfig(void)
{
    if      (cfg.gy_lpf >= 93) mpuLowPassFilter = L3G4200D_DLPF_93HZ;
    else if (cfg.gy_lpf >= 78) mpuLowPassFilter = L3G4200D_DLPF_78HZ;
    else if (cfg.gy_lpf >= 54) mpuLowPassFilter = L3G4200D_DLPF_54HZ;
    else if (cfg.gy_lpf >= 32) mpuLowPassFilter = L3G4200D_DLPF_32HZ;
    else  mpuLowPassFilter = L3G4200D_DLPF_54HZ;
    i2cWrite(L3G4200D_ADDRESS, L3G4200D_CTRL_REG1, L3G4200D_POWER_ON | mpuLowPassFilter);
}

static void l3g4200dInit(void)
{
    bool ack;
    delay(100);
    ack = i2cWrite(L3G4200D_ADDRESS, L3G4200D_CTRL_REG4, L3G4200D_FS_SEL_2000DPS);
    if (!ack) failureMode(3);
    delay(5);
    i2cWrite(L3G4200D_ADDRESS, L3G4200D_CTRL_REG1, L3G4200D_POWER_ON | mpuLowPassFilter);
}

static void l3g4200dAlign(int16_t *gyroData)
{
    gyroData[0] = -gyroData[0];
    gyroData[1] =  gyroData[1];
    gyroData[2] = -gyroData[2];
}

static void l3g4200dRead(int16_t *gyroData)                   // Read 3 gyro values into user-provided buffer. No overrun checking is done.
{
    uint8_t buf[6];
    i2cRead(L3G4200D_ADDRESS, L3G4200D_GYRO_OUT, 6, buf);
    gyroData[1] = combine16(buf[0], buf[1]);
    gyroData[0] = combine16(buf[2], buf[3]);
    gyroData[2] = combine16(buf[4], buf[5]);
}

static void l3g4200dReadTempC100(int16_t *tempData)
{
    *tempData = 0;                                            // Fixme when we have temp compensation for gyro
}
