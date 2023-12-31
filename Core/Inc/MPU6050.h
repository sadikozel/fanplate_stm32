#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include "main.h"

#define MPU6050_ADDRESS 0x68 << 1
#define MPU6050_CONFIG 0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_TEMP_OUT_H 0x41
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_GYRO_YOUT_H 0x45
#define MPU6050_GYRO_ZOUT_H 0x47
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_WHO_AM_I 0x75

#define MPU6050_STRUCT_BUFFER_LEN 6


struct MPU6050{
    uint8_t address;
    uint8_t DLPF_CFG;
    uint8_t FS_SEL;		// GYRO_CONFIG 0:+-250/s 1:+-500/s 2:+-1000/s 3:2000/s
    uint8_t AFS_SEL;	// ACCEL_CONFIG	0:+-2g	1:+-4g	2:+-8g	3:+-16g
    uint8_t CLK_SEL;
    uint8_t check;
    int16_t accelX;
    int16_t accelY;
    int16_t accelZ;
    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;
    float temp;
    uint8_t buff[MPU6050_STRUCT_BUFFER_LEN]; // this buffer is for i2c, you must be sure reset all elements at initialize
};

uint32_t MPU6050Init(I2C_HandleTypeDef *i2c, struct MPU6050 *mpu, uint8_t dlpf_cfg, uint8_t fs_sel, uint8_t afs_sel, uint8_t clk_sel);
void MPU6050Start(void);
void MPU6050UpdateAccel(void);
void MPU6050UpdateTemp(void);
void MPU6050UpdateGyro(void);
void MPU6050WriteRegister(uint8_t address, uint8_t byte);
void MPU6050ReadRegister(uint8_t address, uint16_t size);
void MPU6050BufferReset(void);
void MPU6050Check(void);


#endif
