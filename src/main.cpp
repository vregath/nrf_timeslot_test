
#include "app_timer.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include <SEGGER_RTT.h>

#include "ble/ble.hpp"

#include "time_synchron/time_synchron.h"
#include "time_synchron/time_sync.h"

#include "log/log.h"
#include "power/power.h"

#include <nrf_sdh.h>

#define BLE_ENABLED             1

#define DEAD_BEEF                       0xDEADBEEF

#ifdef DEBUG
volatile __attribute__((section(".uicr_appprotect"))) const uint32_t dissable_app_protect_uicr  = 0x5A ;
#else
volatile __attribute__((section(".uicr_appprotect"))) const uint32_t dissable_app_protect_uicr  = 0x00 ;
#endif

void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name) {
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

#include <nrfx_gpiote.h>

#include "nrfx_ppi.h"

#define LED_R_PIN       NRF_GPIO_PIN_MAP(0, 28)

int main() {
                                                  
    NVIC_SetPriority(DebugMonitor_IRQn, 6ul);

    initPower();
    holdPower();

    nrf_gpio_cfg_output(LED_R_PIN);
    nrf_gpio_pin_write(LED_R_PIN, 0);

    uint32_t err_code;
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
   
    DBGLOGI("MAIN", "Init");

    test::ble.initialize();

    test::timeSynchron.initialize();

    test::ble.advertisingStart(120, false, true);
    
    while (1) {

        test::timeSynchron.doStartSDRadioSession();
    }
}
