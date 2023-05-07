/* Mechanica - ESP8266 */

#include <stdio.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_heap_caps.h"

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
	//pwm_stop(0x0);
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

const int routine_a_len = 6;
const int16_t _routine_a[6][3] = {
	{ 0, 0, 0 },
	{ 0, -90, -90 },
	{ 0, 90, 90 },
	{ -90, 0, 0 },
	{ -90, 90, 90 },
	{ -90, -90, -90 }
};

void routine_a(servo_control_t *servos) {
	for(int i = 0; i < routine_a_len; ++i) {
		
		int16_t *angles = _routine_a[i];
		uint32_t duties[NUM_SERVOS];

		map_angles(duties, servos, NUM_SERVOS, angles);
		pwm_set_duties(duties);
		pwm_start();

		vTaskDelay(1000 / portTICK_RATE_MS);
	}
} 

void move_to_neutral(servo_control_t *servos, int num_servos) {
	uint32_t duties[num_servos];
	map_angles(duties, servos, NUM_SERVOS, NEUTRAL_POSITION);
	pwm_set_duties(duties);
	pwm_start();
}

/*struct gpio_isr_args {
	servo_control_t *servos;
	int num_servos;
	int *counter;
};

int counter = 0;
void gpio_isr_handler(void *arg) {
	struct gpio_isr_args *_args = (struct gpio_isr_args *)arg;
	//move_to_neutral(_args->servos, _args->num_servos);
}*/

static xQueueHandle gpio_evt_queue = NULL;

static void gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_example(void *arg)
{
    uint32_t io_num;

    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            //ESP_LOGI(TAG, "GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
            printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
        }
    }
}


void app_main() {
    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP8266 chip with %d CPU cores, WiFi, ",
            chip_info.cores);

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

	printf("Initializing GPIO\n");
	initialize_gpio();


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

    // create a queue to handle gpio event from isr and start task
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);
	
	
	gpio_install_isr_service(0);
	gpio_isr_handler_add(GPIO_NUM_13, gpio_isr_handler, (void*)GPIO_NUM_13);


	while(1) {
	}

	printf("Restarting in 5 seconds...\n");
	vTaskDelay(5000 / portTICK_RATE_MS);

	pwm_stop(0);
	heap_caps_free(servos);
	fflush(stdout);
    esp_restart();
}

