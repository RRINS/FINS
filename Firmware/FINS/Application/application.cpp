#include "application.hpp"

#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_gpio.h"
#include "main.h"
#include "stdio.h"

extern "C" {

int count = 0;

void setup() {

}

void loop() {
	printf("Count: %i\n", count++);
}

}
