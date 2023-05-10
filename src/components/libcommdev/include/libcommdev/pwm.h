#include <stdint.h>
#include <stdio.h>

typedef struct servo_control {
	uint32_t pwm_period;
	uint32_t min_pulse_width;
	uint32_t max_pulse_width;

	int16_t angle_range;
	
} servo_control_t;


//uint32_t angle_to_pulse_width(servo_control_t *servo, float angle);		// v2
uint32_t angle_pulsewidth_ii(servo_control_t *servo, int16_t angle);

uint32_t angle_to_duty_cycle(int16_t angle, int16_t min_angle, int16_t max_angle, uint32_t min_duty, uint32_t max_duty);


