#include "button.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "config.h"

void buttonTask(void *pvParameters) {
	bool buttonState = false;
	while (true) {
		if (gpio_get_level(BUTTON_PIN) == LOW) {
			if (!buttonState) {
				buttonState = true;
				printf("Button pressed\n");
			}
		} else {
			if (buttonState) {
				buttonState = false;
				printf("Button released\n");
			}
		}
		vTaskDelay(10);
	}
}

void buttonInit() {
	gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = 1 << BUTTON_PIN;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

	xTaskCreatePinnedToCore(buttonTask, "button_task", 2048, NULL, 5, NULL, 1);
}