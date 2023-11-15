#include "../application/application.hpp"

#include "stm32f4xx_ll_gpio.h"
#include "main.h"

extern "C" {

void setup() {
    // For future episodes ;)
}

void loop() {
    LL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

    HAL_Delay(200U);
}

}
