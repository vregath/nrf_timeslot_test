
#include "power.h"

#include "nrf_gpio.h"

#define POWER_BUTTON_PIN  NRF_GPIO_PIN_MAP(1, 15)
#define POWER_HOLD_PIN  NRF_GPIO_PIN_MAP(0, 3)

void initPower(void) {
    nrf_gpio_cfg_output(POWER_HOLD_PIN);
    
}

void holdPower(void) {
    nrf_gpio_pin_write(POWER_HOLD_PIN, 1);
}

void releasePower(void) {
    nrf_gpio_pin_write(POWER_HOLD_PIN, 0);
}
