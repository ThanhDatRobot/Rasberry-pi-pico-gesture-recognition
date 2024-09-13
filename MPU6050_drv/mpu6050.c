#include "mpu6050.h"

static mpu6050_t mpu6050;

static float mpu6050_dps_per_digit[] = {
    .007633f,  .015267f, .030487f, .060975f
}; // 1/131 LSB/°/s, 1/65.5 LSB/°/s, 1/32.8 LSB/°/s, 1/16.4 LSB/°/s


static float mpu6050_range_per_digit[] = {
    .000061f, .000122f, .000244f, .0004882f
}; //1/16384 LSB/g(2G), 1/8192 LSB/g(4G), 1/4096 LSB/g(8G), 1/2048 LSB/g(16G)


void mpu6050_read_register(uint8_t reg, uint8_t *value, uint8_t len) {
    uint8_t buf;
    buf = reg;
    i2c_write_blocking(MPU6050_i2c, MPU6050_ADDRESS, &buf, 1, true);
    i2c_read_blocking(MPU6050_i2c, MPU6050_ADDRESS, value, len, false);
}

void mpu6050_reset() {
    uint8_t buf[] = {0x6B, 0x80};
    i2c_write_blocking(MPU6050_i2c, MPU6050_ADDRESS, buf, 2, false);
    sleep_ms(10);
    buf[1] = 0x00;
    i2c_write_blocking(MPU6050_i2c, MPU6050_ADDRESS, buf, 2, false);
}

void mpu6050_config_gyro(uint8_t gyro_config) {
    uint8_t buf[2];
    buf[0] = MPU6050_GYRO_CONFIG;
    buf[1] = gyro_config << 3;
    i2c_write_blocking(MPU6050_i2c, MPU6050_ADDRESS, buf, 2, false);
    mpu6050.gyro_config = gyro_config; 
}

void mpu6050_config_accel(uint8_t accel_config) {
    uint8_t buf[2];
    uint8_t reg;
    mpu6050_read_register(MPU6050_ACCEL_CONFIG, &reg, 1);
    reg &= 0b11100111;
    buf[0] = MPU6050_ACCEL_CONFIG;
    buf[1] = accel_config << 3 | reg;
    i2c_write_blocking(MPU6050_i2c, MPU6050_ADDRESS, buf, 2, false);
    mpu6050.accel_config = accel_config;
}

/*
0 = 260 Hz, 0 ms / 256 Hz, 0.98 ms, 8 kHz
1 = 184 Hz, 2.0 ms / 188 Hz, 1.9 ms, 1 kHz
2 = 94 Hz, 3.0 ms / 98 Hz, 2.8 1 ms, kHz
3 = 44 Hz, 4.9 ms / 42 Hz, 4.8 1 ms, kHz
4 = 21 Hz, 8.5 ms / 20 Hz, 8.3 1 ms, kHz
5 = 10 Hz, 13.8 ms / 10 Hz, 13.4 ms, 1 kHz
6 = 5 Hz, 19.0 ms / 5 Hz, 18.6 1 ms, kHz
7 = RESERVED / RESERVED, 8 kHz
3-bit unsigned value. Configures the DLPF setting
*/
void mpu6050_config_DLPF(uint8_t dlpf) {
    uint8_t buf[2];
    uint8_t reg;
    mpu6050_read_register(MPU6050_CONFIG, &reg, 1);
    reg &= 0b11111000;
    buf[0] = MPU6050_CONFIG;
    buf[1] = (dlpf&0b00000111) | reg;
    i2c_write_blocking(MPU6050_i2c, MPU6050_ADDRESS, buf, 2, false);
}

/*
0 = Reset
1 = On @ 5 Hz
2 = On @ 2.5 Hz
3 = On @ 1.25 Hz
4 = On @ 0.63 Hz
7 = Hold
*/
void mpu6050_config_DHPF(uint8_t dhpf) {
    uint8_t buf[2];
    uint8_t reg;
    mpu6050_read_register(MPU6050_ACCEL_CONFIG, &reg, 1);
    reg &= 0b11111000;
    buf[0] = MPU6050_ACCEL_CONFIG;
    buf[1] = (dhpf&0b00000111) | reg;
    i2c_write_blocking(MPU6050_i2c, MPU6050_ADDRESS, buf, 2, false);        
}

void mpu6050_init() {
    // init I2C
    i2c_init(MPU6050_i2c, 400 * 1000);
    gpio_set_function(MPU6050_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(MPU6050_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(MPU6050_SDA_PIN);
    gpio_pull_up(MPU6050_SCL_PIN);

    mpu6050_reset();
    mpu6050_config_gyro(GYRO_CONFIG_250);
    mpu6050_config_accel(ACCEL_CONFIG_2G);

    mpu6050.gyro_cali.xg_calibration=0;
    mpu6050.gyro_cali.yg_calibration=0;
    mpu6050.gyro_cali.zg_calibration=0;
}

void mpu6050_get_accel_raw(int16_t *ax, int16_t *ay, int16_t *az) {
    uint8_t buf[6];
    buf[0]=MPU6050_ACCEL_OUT;

    i2c_write_blocking(MPU6050_i2c, MPU6050_ADDRESS, buf, 1, true);
    i2c_read_blocking(MPU6050_i2c, MPU6050_ADDRESS, buf, 6, false);

    *ax = (int16_t)(buf[0] << 8 | buf[1]);
    *ay = (int16_t)(buf[2] << 8 | buf[3]);
    *az = (int16_t)(buf[4] << 8 | buf[5]);
}

void mpu6050_get_gyro_raw(int16_t *gx, int16_t *gy, int16_t *gz) {
    uint8_t buf[6];
    buf[0]=MPU6050_GYRO_OUT;

    i2c_write_blocking(MPU6050_i2c, MPU6050_ADDRESS, buf, 1, true);
    i2c_read_blocking(MPU6050_i2c, MPU6050_ADDRESS, buf, 6, false);

    *gx = (int16_t)(buf[0] << 8 | buf[1]);
    *gy = (int16_t)(buf[2] << 8 | buf[3]);
    *gz = (int16_t)(buf[4] << 8 | buf[5]);
    
}

void mpu6050_calibration(uint8_t time) {
    int32_t x_sum=0, y_sum=0, z_sum=0;
    uint32_t cnt=0;
    int16_t gx, gy, gz;

    absolute_time_t timeout = make_timeout_time_ms(time*1000);
    while (absolute_time_diff_us(get_absolute_time(), timeout) > 0) { 
        cnt++;
        mpu6050_get_gyro_raw(&gx,&gy,&gz);
        x_sum += gx; y_sum += gy; z_sum += gz;
        sleep_ms(1);
    }
    mpu6050.gyro_cali.xg_calibration=(float)(x_sum)/cnt*mpu6050_dps_per_digit[mpu6050.gyro_config];
    mpu6050.gyro_cali.yg_calibration=(float)(y_sum)/cnt*mpu6050_dps_per_digit[mpu6050.gyro_config];
    mpu6050.gyro_cali.zg_calibration=(float)(z_sum)/cnt*mpu6050_dps_per_digit[mpu6050.gyro_config];
}

void mpu6050_init_with_calibration(uint8_t gyro_config, uint8_t accel_config, uint8_t time) {
    // init I2C
    i2c_init(MPU6050_i2c, 400 * 1000);
    gpio_set_function(MPU6050_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(MPU6050_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(MPU6050_SDA_PIN);
    gpio_pull_up(MPU6050_SCL_PIN);

    mpu6050_reset();
    mpu6050_config_gyro(gyro_config);
    mpu6050_config_accel(accel_config);
    mpu6050_calibration(time);
}

void mpu6050_get_temp(float *temp) {
    uint8_t buf[6];
    buf[0]=MPU6050_TEMP_OUT;

    i2c_write_blocking(MPU6050_i2c, MPU6050_ADDRESS, buf, 1, true);
    i2c_read_blocking(MPU6050_i2c, MPU6050_ADDRESS, buf, 2, false);
    *temp = (int16_t)(buf[0] << 8 | buf[1])/340.0f + 36.53f;
}

void mpu6050_get_accel(float *ax, float *ay, float *az) {
    uint8_t buf[6];
    
    buf[0]=MPU6050_ACCEL_OUT;

    i2c_write_blocking(MPU6050_i2c, MPU6050_ADDRESS, buf, 1, true);
    i2c_read_blocking(MPU6050_i2c, MPU6050_ADDRESS, buf, 6, false);

    *ax = (int16_t)(buf[0] << 8 | buf[1])*mpu6050_range_per_digit[mpu6050.accel_config];
    *ay = (int16_t)(buf[2] << 8 | buf[3])*mpu6050_range_per_digit[mpu6050.accel_config];
    *az = (int16_t)(buf[4] << 8 | buf[5])*mpu6050_range_per_digit[mpu6050.accel_config];

}
void mpu6050_get_gyro(float *gx, float *gy, float *gz) {
    uint8_t buf[6];
    
    buf[0]=MPU6050_GYRO_OUT;

    i2c_write_blocking(MPU6050_i2c, MPU6050_ADDRESS, buf, 1, true);
    i2c_read_blocking(MPU6050_i2c, MPU6050_ADDRESS, buf, 6, false);

    *gx = (int16_t)(buf[0] << 8 | buf[1])*mpu6050_dps_per_digit[mpu6050.gyro_config]-mpu6050.gyro_cali.xg_calibration;
    *gy = (int16_t)(buf[2] << 8 | buf[3])*mpu6050_dps_per_digit[mpu6050.gyro_config]-mpu6050.gyro_cali.yg_calibration;
    *gz = (int16_t)(buf[4] << 8 | buf[5])*mpu6050_dps_per_digit[mpu6050.gyro_config]-mpu6050.gyro_cali.zg_calibration;
}

/*
[7] INT_LEVEL
[6] INT_OPEN
[5] LATCH_INT_EN
[4] INT_RD_CLEAR
[3] FSYNC_INT_LEVEL
[2] FSYNC_INT_EN
[1] I2C_BYPASS_EN
[0] CLKOUT_EN
INT_LEVEL:  0, the logic level for the INT pin is active high.
            1, the logic level for the INT pin is active low.
INT_OPEN:   0, the INT pin is configured as push-pull.
            1, the INT pin is configured as open drain.
LATCH_INT_EN:0, the INT pin emits a 50us long pulse.
             1, the INT pin is held high until the interrupt is cleared.
INT_RD_CLEAR:0, interrupt status bits are cleared only by reading INT_STATUS
             1, interrupt status bits are cleared on any read operation.
FSYNC_INT_LEVEL: 0, the logic level for the FSYNC pin is active high.
                 1, the logic level for the FSYNC pin is active low.
FSYNC_INT_EN: 0, this bit disables the FSYNC pin from causing an interrupt to the host processor.
              1, this bit enables the FSYNC pin to be used as an interrupt to the host processor.

I2C_BYPASS_EN: 1 and I2C_MST_EN 0, the host application processor will be able to directly access the auxiliary I2C bus of the MPU-60X0.
               0, the host application processor will not be able to directly access the auxiliary I2C bus of the MPU-60X0 regardless of the state of I2C_MST_EN (Register 106 bit[5]).
*/
void mpu6050_int_pin_config(uint8_t config) {
    uint8_t buf[2] = {MPU6050_INT_PIN_CONFIG, config};
    i2c_write_blocking(MPU6050_i2c, MPU6050_ADDRESS, buf, 2, false);
}

/*
[7] FF_EN
[6] MOT_EN
[5] ZMOT_EN
[4] FIFO_OFLOW_EN
[3] I2C_MST_INT_EN
[2] PLL_RDY_INT_EN
[1] DMP_INT_EN
[0] RATA_RDY_EN
*/
void mpu6050_int_enable(uint8_t config) {
    uint8_t buf[2] = {MPU6050_INT_ENABLE, config};
    i2c_write_blocking(MPU6050_i2c, MPU6050_ADDRESS, buf, 2, false);
}

/*
[7] FF_INT
[6] MOT_INT
[5] ZMOT_INT
[4] FIFO_OFLOW_INT
[3] I2C_MST_INT
[2] PLL_RDY_INT
[1] DMP_INT
[0] RAW_RDY_INT
Each bit will clear after the register is read.
*/
void mpu6050_read_int_status(uint8_t *status) {
    uint8_t buf;
    buf = MPU6050_INT_STATUS;
    i2c_write_blocking(MPU6050_i2c, MPU6050_ADDRESS, &buf, 1, true);
    i2c_read_blocking(MPU6050_i2c, MPU6050_ADDRESS, &buf, 1, false);
    *status = buf;
}

/*
[7] MOT_XNEG
[6] MOT_XPOS
[5] MOT_YNEG
[4] MOT_YPOS
[3] MOT_ZNEG
[2] MOT_ZPOS
[0] MOT_ZRMOT
Reading this register clears the Motion detection bits. However, the MOT_ZRMOT bit does not clear until Zero Motion is no longer detected.
*/
void mpu6050_read_mot_detect_status(uint8_t *status) {
    uint8_t buf;
    buf = MPU6050_MOT_DETECT_STATUS;
    i2c_write_blocking(MPU6050_i2c, MPU6050_ADDRESS, &buf, 1, true);
    i2c_read_blocking(MPU6050_i2c, MPU6050_ADDRESS, &buf, 1, false);
    *status = buf;
}

void mpu6050_set_motion_detection_threshold(uint8_t threshold)
{
    uint8_t buf[2] = {MPU6050_MOT_THR, threshold};
    i2c_write_blocking(MPU6050_i2c, MPU6050_ADDRESS, buf, 2, false);
}

void mpu6050_set_motion_detection_duration(uint8_t duration)
{
    uint8_t buf[2] = {MPU6050_MOT_DUR, duration};
    i2c_write_blocking(MPU6050_i2c, MPU6050_ADDRESS, buf, 2, false);
}

void mpu6050_set_zero_motion_detection_threshold(uint8_t threshold)
{
    uint8_t buf[2] = {MPU6050_ZRMOT_THR, threshold};
    i2c_write_blocking(MPU6050_i2c, MPU6050_ADDRESS, buf, 2, false);
}

void mpu6050_set_zero_motion_detection_duration(uint8_t duration)
{
    uint8_t buf[2] = {MPU6050_ZRMOT_DUR, duration};
    i2c_write_blocking(MPU6050_i2c, MPU6050_ADDRESS, buf, 2, false);
}

void mpu6050_set_free_fall_detection_threshold(uint8_t threshold)
{
    uint8_t buf[2] = {MPU6050_FF_THR, threshold};
    i2c_write_blocking(MPU6050_i2c, MPU6050_ADDRESS, buf, 2, false);
}

void mpu6050_set_free_fall_detection_duration(uint8_t duration)
{
    uint8_t buf[2] = {MPU6050_FF_DUR, duration};
    i2c_write_blocking(MPU6050_i2c, MPU6050_ADDRESS, buf, 2, false);
}
