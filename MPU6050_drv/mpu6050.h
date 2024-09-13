
#ifndef __MPU6050_H_
#define __MPU6050_H_


#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define MPU6050_ADDRESS         0x68
#define MPU6050_i2c             i2c0
#define MPU6050_SDA_PIN         4
#define MPU6050_SCL_PIN         5

typedef struct  {
    uint8_t gyro_config;
    uint8_t accel_config;
    struct gyro_cali_t {
        float xg_calibration;
        float yg_calibration;
        float zg_calibration;
    } gyro_cali;
}mpu6050_t;

enum {
    GYRO_CONFIG_250 =           0,
    GYRO_CONFIG_500 =           1,
    GYRO_CONFIG_1000=           2,
    GYRO_CONFIG_2000=           3,
};

enum {
    ACCEL_CONFIG_2G =           0,
    ACCEL_CONFIG_4G =           1,
    ACCEL_CONFIG_8G =           2,
    ACCEL_CONFIG_16G=           3,
};

//registers
#define MPU6050_SMPRT_DIV       0x19
#define MPU6050_CONFIG          0x1A
#define MPU6050_GYRO_CONFIG     0x1B
#define MPU6050_ACCEL_CONFIG    0x1C
#define MPU6050_ACCEL_OUT       0x3B
#define MPU6050_ACCEL_XOUT      0x3B
#define MPU6050_ACCEL_YOUT      0x3D
#define MPU6050_ACCEL_ZOUT      0x3F
#define MPU6050_TEMP_OUT        0x41
#define MPU6050_GYRO_OUT        0x43
#define MPU6050_GYRO_XOUT       0x43
#define MPU6050_GYRO_YOUT       0x45
#define MPU6050_GYRO_ZOUT       0x47

enum {
    INT_LEVEL=                  1<<7,
    INT_OPEN=                   1<<6,
    LATCH_INT_EN=               1<<5,
    INT_RD_CLEAR=               1<<4,
    FSYNC_INT_LEVEL=            1<<3,
    FSYNC_INT_EN=               1<<2,
    I2C_BYPASS_EN=              1<<1,
    CLKOUT_EN=                  1<<0,
};
#define MPU6050_INT_PIN_CONFIG  0x37

enum {
    FF_EN=                      1<<7,
    MOT_EN=                     1<<6,
    ZMOT_EN=                    1<<5,
    FIFO_OFLOW_EN=              1<<4,
    I2C_MST_INT_EN=             1<<3,
    PLL_RDY_INT_EN=             1<<2,
    DMP_INT_EN=                 1<<1,
    DATA_RDY_EN=                1<<0,
};
#define MPU6050_INT_ENABLE      0x38

enum {
    FF_INT=                     1<<7,
    MOT_INT=                    1<<6,
    ZMOT_INT=                   1<<5,
    FIFO_OFLOW_INT=             1<<4,
    I2C_MST_INT=                1<<3,
    PLL_RDY_INT=                1<<2,
    DMP_INT=                    1<<1,
    DATA_RDY_INT=                1<<0,
};
#define MPU6050_INT_STATUS      0x3A

#define MPU6050_MOT_THR         0x1F
#define MPU6050_MOT_DUR         0x20
#define MPU6050_ZRMOT_THR       0x21
#define MPU6050_ZRMOT_DUR       0x22
#define MPU6050_FF_THR          0x1D
#define MPU6050_FF_DUR          0x1E


enum {
    MOT_XNEG=                   1<<7,
    MOT_XPOS=                   1<<6,
    MOT_YNEG=                   1<<5,
    MOT_YPOS=                   1<<4,
    MOT_ZNEG=                   1<<3,
    MOT_ZPOS=                   1<<2,
    MOT_ZRMOT=                  1<<0,
};
#define MPU6050_MOT_DETECT_STATUS 0x61 

#ifdef __cplusplus
extern "C" {
#endif

void mpu6050_init();
void mpu6050_init_with_calibration(uint8_t gyro_config, uint8_t accel_config, uint8_t time);
void mpu6050_get_gyro(float *gx, float *gy, float *gz);
void mpu6050_get_accel(float *ax, float *ay, float *az);
void mpu6050_get_temp(float *temp);
void mpu6050_config_gyro(uint8_t gyro_config);
void mpu6050_config_accel(uint8_t accel_config);
void mpu6050_int_pin_config(uint8_t config);
void mpu6050_int_enable(uint8_t config);
void mpu6050_read_int_status(uint8_t *status);
void mpu6050_read_mot_detect_status(uint8_t *status);

void mpu6050_set_motion_detection_threshold(uint8_t threshold);
void mpu6050_set_motion_detection_duration(uint8_t duration);
void mpu6050_set_zero_motion_detection_threshold(uint8_t threshold);
void mpu6050_set_zero_motion_detection_duration(uint8_t duration);
void mpu6050_set_free_fall_detection_threshold(uint8_t threshold);
void mpu6050_set_free_fall_detection_duration(uint8_t duration);
void mpu6050_read_register(uint8_t reg, uint8_t *value, uint8_t len);
void mpu6050_config_DLPF(uint8_t dlpf);
void mpu6050_config_DHPF(uint8_t dhpf);

void mpu6050_get_accel_raw(int16_t *ax, int16_t *ay, int16_t *az);
void mpu6050_get_gyro_raw(int16_t *gx, int16_t *gy, int16_t *gz);

#ifdef __cplusplus
}
#endif

#endif
