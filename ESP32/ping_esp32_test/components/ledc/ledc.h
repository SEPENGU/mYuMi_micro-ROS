#ifndef __LEDC_H__
#define __LEDC_H__

#ifdef __cplusplus
extern "C" {
#endif

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
//#define LEDC_OUTPUT_IO          (5) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (4095) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define LEDC_FREQUENCY          (5000) // Frequency in Hertz. Set frequency at x kHz

#define DUTY_MAX 8190

// Functions here
void ledc_init(int pin);
void ledc_duty(int duty);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __LEDC_H__ */
