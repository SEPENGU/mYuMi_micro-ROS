#ifndef SERVO_H_
#define SERVO_H_


#define SERVO_MIN 800
#define SERVO_MAX 2200
#define ANGLE_MIN 0
#define ANGLE_MAX 180

typedef struct {
    TIM_HandleTypeDef* htim;
    uint32_t channel;
} servo_t;

void Servo_Init(servo_t *servo, TIM_HandleTypeDef *_htim, uint32_t _channel);
void Servo_SetAngle(servo_t *servo, uint16_t angle);
void Servo_SetAngleFine(servo_t *servo, float angle);

#endif /* SERVO_H_ */
