#include "pwm.h"

/**
 * Maps a value from one range to another.
 *
 * @param value     The value to map.
 * @param from_low  The lower bound of the original range.
 * @param from_high The upper bound of the original range.
 * @param to_low    The lower bound of the target range.
 * @param to_high   The upper bound of the target range.
 *
 * @return The mapped value.
 */
float map_range(float value, float from_low, float from_high, float to_low, float to_high) {
	return (value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low;
}

int map_range_ii(int val, int from_low, int from_high, int to_low, int to_high) {
	return (val - from_low) * (to_high - to_low) / (from_high - from_low) + to_low;
}

uint32_t angle_pulsewidth_ii(servo_control_t *servo, int16_t angle) {
	
	/*printf("servo control; period: %u us, min_duty: %u us, max_duty: %u us, ang rng: %d deg\n",
			servo->pwm_period, servo->min_pulse_width, servo->max_pulse_width,
			servo->angle_range);

	printf("initial angle: %d deg\n", angle);*/
	
	/* Ensure the angle is within the angle range */
	const int16_t min_angle = -servo->angle_range / 2;
	const int16_t max_angle = servo->angle_range / 2;
	angle = (angle < min_angle) ? min_angle : 
		(angle > max_angle) ? max_angle : angle;

	uint32_t pw = (uint32_t)map_range_ii(angle, 
			min_angle, max_angle, 
			servo->min_pulse_width, servo->max_pulse_width);

	return pw;
}


uint32_t angle_to_duty_cycle(
                int16_t angle, 
                int16_t min_angle, 
                int16_t max_angle, 
                uint32_t min_duty, 
                uint32_t max_duty) 
{
        int16_t old_range = (max_angle - min_angle);    /* calculate angle range */
        uint32_t new_range = (max_duty - min_duty);             /* calculate duty range*/

        /**
         * scale angle by subtracting from min_angle and multiplying the result
         * by the ratio of the new_range to old_range, then adding the min
         * duty
         */
        uint32_t duty = (((angle - min_angle) * new_range) / old_range) + min_duty;     

        return duty;
}


