#include <stdio.h>
#include <stdbool.h> // Include this header for true/false definitions
#include "pico/stdlib.h"
#include "string.h"
#include "mpu6050.h"

void training_gesture() {
    int16_t rax, ray, raz;
    int16_t rgx, rgy, rgz;
    
    while (1) {
        mpu6050_get_accel_raw(&rax, &ray, &raz);
        mpu6050_get_gyro_raw(&rgx, &rgy, &rgz);
        //printf("%d,%d,%d, %d,%d,%d\n", rax, ray, raz, rgx, rgy, rgz);
        printf("%d,%d,%d\n",  rax, ray, raz);
        sleep_ms(10);
   }
}

int main()
{
    stdio_init_all();
    uint32_t count=0;
    uint8_t status;
    int16_t rax, ray, raz;
    int16_t rgx, rgy, rgz;

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, true);

    mpu6050_init();
    mpu6050_config_accel(ACCEL_CONFIG_2G);
    mpu6050_config_gyro(GYRO_CONFIG_250);

    training_gesture();  // uncomment when capture training dataset

    while(1) {
    }

    return 0;

}

