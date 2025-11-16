#ifndef MPU6050_H
#define	MPU6050_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "I2C.h"

#define MPU6050_I2C_ADDRESS 0x68
    
int16_t gyro_z_offset = 0;

void MPU6050_Init(void);
void CalibrateGyro(void);
void updateYaw(void);

int16_t MPU6050_ReadAccelX(void);
int16_t MPU6050_ReadAccelY(void);
int16_t MPU6050_ReadAccelZ(void);
int16_t MPU6050_ReadGyroX(void);
int16_t MPU6050_ReadGyroY(void);
int16_t MPU6050_ReadGyroZ(void);

void MPU6050_Init(void) {
    I2C_Start();
    I2C_Write(MPU6050_I2C_ADDRESS << 1);
    I2C_Write(0x6B);   // Power Management 1
    I2C_Write(0x00);   // Wake up
    I2C_Stop();
    
    CalibrateGyro();
}

void CalibrateGyro() {
    long sum = 0;
    
    __delay_ms(200);
    
    for (int i=0; i<2000; i++) {
        sum += MPU6050_ReadGyroZ();
        __delay_ms(2);
    }
    gyro_z_offset = sum / 2000;
    
    __delay_ms(200);
}

volatile float yaw_angle = 0.0f;

void updateYaw() {
    int16_t gyro_z_raw = 0;
    float gyro_z_dps = 0.0f;
    
    // 1. Read raw gyro Z
    gyro_z_raw = MPU6050_ReadGyroZ();

    // 2. Remove calibrated offset
    int16_t corrected = gyro_z_raw - gyro_z_offset;

    // 3. Convert to degrees per second
    gyro_z_dps = (float)corrected / 131.0f;

    // 5. Integrate angle
    yaw_angle += gyro_z_dps * 0.25f;
    //0.25 is dt. Here defined as 0.25 according to observation. Should use proper timer during implementation

    // OPTIONAL: keep within 0?360
    if (yaw_angle > 360) yaw_angle -= 360;
    if (yaw_angle < 0)   yaw_angle += 360;
}

int16_t MPU6050_ReadAccelX(void) {
    uint8_t high, low;
    I2C_Start();
    I2C_Write(MPU6050_I2C_ADDRESS << 1);
    I2C_Write(0x3B); // ACCEL_XOUT_H
    I2C_RepeatedStart();
    I2C_Write((MPU6050_I2C_ADDRESS << 1) | 1);
    high = I2C_Read(1);
    low  = I2C_Read(0);
    I2C_Stop();
    return (int16_t)((high << 8) | low);
}

int16_t MPU6050_ReadAccelY(void) {
    uint8_t high, low;
    I2C_Start();
    I2C_Write(MPU6050_I2C_ADDRESS << 1);
    I2C_Write(0x3D); // ACCEL_YOUT_H
    I2C_RepeatedStart();
    I2C_Write((MPU6050_I2C_ADDRESS << 1) | 1);
    high = I2C_Read(1);
    low  = I2C_Read(0);
    I2C_Stop();
    return (int16_t)((high << 8) | low);
}

int16_t MPU6050_ReadAccelZ(void) {
    uint8_t high, low;
    I2C_Start();
    I2C_Write(MPU6050_I2C_ADDRESS << 1);
    I2C_Write(0x3F); // ACCEL_ZOUT_H
    I2C_RepeatedStart();
    I2C_Write((MPU6050_I2C_ADDRESS << 1) | 1);
    high = I2C_Read(1);
    low  = I2C_Read(0);
    I2C_Stop();
    return (int16_t)((high << 8) | low);
}

int16_t MPU6050_ReadGyroX(void) {
    uint8_t high, low;
    I2C_Start();
    I2C_Write(MPU6050_I2C_ADDRESS << 1);
    I2C_Write(0x43); // GYRO_XOUT_H
    I2C_RepeatedStart();
    I2C_Write((MPU6050_I2C_ADDRESS << 1) | 1);
    high = I2C_Read(1);
    low  = I2C_Read(0);
    I2C_Stop();
    return (int16_t)((high << 8) | low);
}

int16_t MPU6050_ReadGyroY(void) {
    uint8_t high, low;
    I2C_Start();
    I2C_Write(MPU6050_I2C_ADDRESS << 1);
    I2C_Write(0x45); // GYRO_YOUT_H
    I2C_RepeatedStart();
    I2C_Write((MPU6050_I2C_ADDRESS << 1) | 1);
    high = I2C_Read(1);
    low  = I2C_Read(0);
    I2C_Stop();
    return (int16_t)((high << 8) | low);
}

int16_t MPU6050_ReadGyroZ(void) {
    uint8_t high, low;
    I2C_Start();
    I2C_Write(MPU6050_I2C_ADDRESS << 1);
    I2C_Write(0x47); // GYRO_ZOUT_H
    I2C_RepeatedStart();
    I2C_Write((MPU6050_I2C_ADDRESS << 1) | 1);
    high = I2C_Read(1);
    low  = I2C_Read(0);
    I2C_Stop();
    return (int16_t)((high << 8) | low);
}

#ifdef	__cplusplus
}
#endif

#endif	/* MPU6050_H */
