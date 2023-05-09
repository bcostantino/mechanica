/**
 * MECHANICA - ESP8266
 * 
 * Control a robotic arm
 *
 * Last Updated: 05.09.2023
 * Author: Brian Costantino
 * License: MIT
 */

#include <stdio.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_heap_caps.h"
#include "esp_heap_trace.h"

#include "libcommdev/pwm.h"

#include "driver/gpio.h"
#include "driver/pwm.h"

#define PWM_PERIOD (20000)
#define NUM_SERVOS 3


const uint32_t servo_pins[NUM_SERVOS] = {
	GPIO_NUM_14,
	GPIO_NUM_4,
	GPIO_NUM_15
};

void initialize_gpio() {
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_POSEDGE;
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pin_bit_mask = GPIO_Pin_13;

	//io_conf.pull_down_en = 1;

	gpio_config(&io_conf);
}

void initialize_servos(servo_control_t *servos) {

	for(int i = 0; i < NUM_SERVOS; ++i) {
		servos[i].pwm_period = PWM_PERIOD;
		servos[i].min_pulse_width = 500;
		servos[i].max_pulse_width = 2500;
		servos[i].angle_range = 180;
	}

}

void reset_angle(servo_control_t *servos, uint8_t channel_num, int16_t angle) {	
	uint32_t duty = angle_pulsewidth_ii(&(servos[channel_num]), angle);
	printf("Setting servo on channel %u to angle %d deg, mapped to PWM duty %u us\n", channel_num, angle, duty);
	pwm_set_duty(channel_num, duty);
}

void map_angles(uint32_t *duties, servo_control_t *servos, int num_servos, int16_t *angles) {
	for (int i = 0; i < num_servos; ++i) {
		servo_control_t servo = servos[i];
		int16_t angle = angles[i];
		uint32_t duty = angle_pulsewidth_ii(&servo, angle);
		printf("[SRVO]: Channel %d to angle %d deg, mapped to PWM duty %u us\n", i, angle, duty);
		duties[i] = duty;
	}
}

const int16_t NEUTRAL_POSITION[3] = { 0, 0, 0 };

/**
 *  move servos to angles in control vector
 */
void move_servos(servo_control_t *servos, int16_t *control_vector, int num_servos) {	
	uint32_t duties[num_servos];
	map_angles(duties, servos, NUM_SERVOS, control_vector);
	pwm_set_duties(duties);
	pwm_start();
}

void move_to_neutral(servo_control_t *servos, int num_servos) {
	move_servos(servos, NEUTRAL_POSITION, num_servos);
}


const int routine_a_len = 7;
const int16_t _routine_a[7][3] = {
	{ 0, 0, 0 },
	{ 0, -90, 90 },
	{ 0, 90, -90 },
	{ -90, 0, 0 },
	{ -90, 90, -90 },
	{ -90, -90, 90 },
	{ 0, 0, 0 }
};

void routine_a(servo_control_t *servos) {
	for(int i = 0; i < routine_a_len; ++i) {

		int16_t *angles = _routine_a[i];
		move_servos(servos, angles, NUM_SERVOS);

		vTaskDelay(1000 / portTICK_RATE_MS);
	}
}

/* TODO: make gpio_num optional (use pointer and NULL to denote no value) */
typedef struct {
	uint32_t gpio_num;			/* gpio pin number that triggered control task */
	uint8_t servo_count;		/* number of servos */
	int16_t *position_vector;	/* array of len servo_count, contains angles */
} servo_control_args_t;


static xQueueHandle servo_control_queue = NULL;

static void reset_servos_isr_handler(void *arg) {
	servo_control_args_t *args = (servo_control_args_t*)arg;
	xQueueSendFromISR(servo_control_queue, args, NULL);
}

static void servo_control_task(void *arg) {

	printf("Initializing servos and PWM channels\n");
	servo_control_t *servos;
	servos = (servo_control_t*)heap_caps_calloc(NUM_SERVOS, sizeof(servo_control_t), MALLOC_CAP_32BIT);
	initialize_servos(servos);

	uint32_t init_duty[NUM_SERVOS];
	float phases[NUM_SERVOS];

	for(int i = 0; i < NUM_SERVOS; ++i) {
		init_duty[i] = angle_pulsewidth_ii(&(servos[i]), 0);
		phases[i] = 0;
		printf("[SRVO]: servo channel #%d:\n", i);
		printf("\tconfig: period = %u us, min_pw = %u us, max_px = %u us, range = %d deg\n", 
				servos[i].pwm_period, servos[i].min_pulse_width,
				servos[i].max_pulse_width, servos[i].angle_range);
		printf("\tinitial: pw = %u us, phase = %f\n", init_duty[i], phases[i]);
	}

	pwm_init(PWM_PERIOD, init_duty, NUM_SERVOS, servo_pins);
	pwm_set_phases(phases);

	printf("Moving to neutral position...\n");
	move_to_neutral(servos, NUM_SERVOS);

	int16_t last_positon[NUM_SERVOS];
	for (;;) {
		servo_control_args_t *args = NULL;

		/* execute block if queue item exists
		 * see https://www.freertos.org/a00118.html */
		if (xQueueReceive(servo_control_queue, &args, portMAX_DELAY)) {
			if (args == NULL) {
				printf("[ERR]: servo control args is NULL, skipping\n");
				continue;
			}

			printf("Triggered by GPIO:%d, setting %d servo positions to ", args->gpio_num, args->servo_count);
			printf("[ %d, %d, %d ]\n",
					args->position_vector[0],
					args->position_vector[1],
					args->position_vector[2]);

			/* TODO: set last_position here, and guard */

			/* maneuver servos here */

			//move_to_neutral(servos, NUM_SERVOS);
			//routine_a(servos);
			
			/* free args mem */
			//free(args->position_vector);
			//heap_caps_free(args);
		}
	}

	heap_caps_free(servos);

}

volatile servo_control_args_t *next_isr_args = NULL;


void app_main() {

	/* Print chip information */
	esp_chip_info_t chip_info;
	esp_chip_info(&chip_info);
	printf("This is ESP8266 chip with %d CPU cores, WiFi, ", chip_info.cores);
	printf("silicon revision %d, ", chip_info.revision);
	printf("%uMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
			(chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

	printf("Initializing GPIO\n");
	initialize_gpio();

	heap_trace_start(HEAP_TRACE_LEAKS);		// for debugging

	/* call xQueueCreate(uxQueueLength, uxItemSize) to create queue (kinda event buffer) 
	 * see http://web.ist.utl.pt/~ist11993/FRTOS-API/group___queue_management.html
	 *	   https://www.freertos.org/Embedded-RTOS-Queues.html
	 *	   https://www.freertos.org/a00116.html */
	servo_control_queue = xQueueCreate(10, sizeof(servo_control_args_t*));

	/* create task 
	 * see https://www.freertos.org/a00125.html and https://www.freertos.org/taskandcr.html */
	xTaskCreate(servo_control_task, "servo_control_task", 2048, NULL, 10, NULL);

	/* allocate args structure + control vector length
	 * initialize control vector to 0 */
	/* TODO: free this memory somewhere */
	servo_control_args_t *args;
	args = (servo_control_args_t*)heap_caps_zalloc(sizeof(servo_control_args_t), MALLOC_CAP_32BIT);
	args->gpio_num = GPIO_NUM_13;
	args->servo_count = NUM_SERVOS;
	args->position_vector = (int16_t*)calloc(args->servo_count, sizeof(int16_t));

	next_isr_args = args;

	/* add isr handler for gpio
	 * pass pointer to args struct reference */
	gpio_install_isr_service(0);
	gpio_isr_handler_add(GPIO_NUM_13, reset_servos_isr_handler, (void*)&next_isr_args);

	for(;;);

	printf("Restarting in 5 seconds...\n");
	vTaskDelay(5000 / portTICK_RATE_MS);

	pwm_stop(0);

	fflush(stdout);
	esp_restart();
	
}

/* TO READ:
 *	- https://forums.freertos.org/t/tasks-v-s-threads/16757
 *	- https://www.freertos.org/implementation/main.html
 *
 *
 */

/** MIT License
 *
 * Copyright (C) 2023 Brian Costantino
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of 
 * this software and associated documentation files (the “Software”), to deal in 
 * the Software without restriction, including without limitation the rights to use, 
 * copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the 
 * Software, and to permit persons to whom the Software is furnished to do so, 
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS 
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR 
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN 
 * AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION 
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
