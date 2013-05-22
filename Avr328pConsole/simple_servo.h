/*
 * simple_servo.h
 *
 *  Created on: Apr 22, 2010
 *      Author: Andrzej
 */
#include <avr/io.h>
#include <util/delay.h>
#include "timer.h"

#ifndef SIMPLE_SERVO_H_
#define SIMPLE_SERVO_H_


#define SERVO_MIN_WIDTH 500
#define SERVO_MAX_WIDTH 10000
#define SERVO_NEUTRAL_WIDTH 3000

//Solid State realy
#define SSR_MIN_POWER 0
#define SSR_MAX_POWER 15625

void ServoInit(void);
void SetServoPosition(int16_t pulse_width, uint16_t delay);

void SSRInit(void);
void SetSSRPwm(uint16_t pulse_width);

#endif /* SIMPLE_SERVO_H_ */
