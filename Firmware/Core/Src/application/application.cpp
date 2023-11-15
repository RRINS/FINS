#include "../application/application.hpp"

#include "stm32f4xx_ll_gpio.h"
#include "main.h"
#include "stdio.h"

extern "C" {

int count = 0;

void setup() {
    // For future episodes ;)
}

void loop() {
	printf("Count: %i\n", count++);
    LL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

    HAL_Delay(500U);
}

}
