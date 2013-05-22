/*
 * simple_servo.c
 *
 *  Created on: Apr 22, 2010
 *      Author: Andrzej
 */

#include <util/delay.h>
#include "timer.h"
#include "simple_servo.h"


void ServoInit(void)
{
	DDRB  = _BV(1) | _BV(2);
	PORTB = 0x07;

	timer1PWMInitICR(50000);
	timer1SetPrescaler(2);
	timer1PWMAOn();
	timer1PWMBOn();
	timer1PWMASet(SERVO_NEUTRAL_WIDTH);
	timer1PWMBSet(SERVO_NEUTRAL_WIDTH);
}

void SetServoPosition(int16_t pulse_width, uint16_t delay)
{
		pulse_width = (uint16_t)(SERVO_NEUTRAL_WIDTH + pulse_width);
		if (pulse_width < SERVO_MIN_WIDTH)
			pulse_width = SERVO_MIN_WIDTH;
		if (pulse_width > SERVO_MAX_WIDTH)
			pulse_width = SERVO_MAX_WIDTH;

		if (delay > 0)
		{
			while(OCR1B != pulse_width)
			{
				if(OCR1B > pulse_width)
					OCR1B--;
				else
					OCR1B++;

				if (OCR1B == pulse_width)
					break;
//				_delay_us(1000*delay);
			}
		}
		else
			OCR1B = pulse_width;

}

void SSRInit(void)
{
	DDRB  = _BV(1) | _BV(2);
	PORTB = 0x07;

	timer1PWMInitICR(SSR_MAX_POWER);
	timer1SetPrescaler(TIMER_CLK_DIV1024);
	timer1PWMAOn();
	timer1PWMASet(SSR_MIN_POWER);

}
void SetSSRPwm(uint16_t pulse_width)
{
	if((uint16_t)pulse_width <= SSR_MIN_POWER)
		pulse_width = SSR_MIN_POWER;
	if((uint16_t)pulse_width >= SSR_MAX_POWER)
			pulse_width = SSR_MAX_POWER;
	timer1PWMASet(pulse_width);
}
