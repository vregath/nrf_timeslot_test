
#include "time_synchron.h"
#include "time_sync.h"

#include "nrf_gpiote.h"
#include "nrf_gpio.h"

#include "log/log.h"

namespace test {

    TimeSynchron timeSynchron;

    #define TEST_PIN    NRF_GPIO_PIN_MAP(1, 9)

    static void ts_event_handler(const ts_evt_t *evt) {
        switch(evt->type) {
            case TS_EVT_SYNCHRONIZED: {
                    DBGLOGI("TS", "Synchronized");
                }
                break;
            case TS_EVT_TRIGGERED: {
                    
                }
                break;
            case TS_EVT_TIMESTAMP: {
                    break;
                }
            case TS_EVT_DESYNCHRONIZED:
                DBGLOGI("TS", "Desynchronized");
                break;
        }
    }

    void TimeSynchron::initialize() {
        ts_init_t init_ts;
        init_ts.high_freq_timer[0] = NRF_TIMER3;
        init_ts.high_freq_timer[1] = NRF_TIMER4;
        init_ts.egu = NRF_EGU3;
        init_ts.egu_irq_type = SWI3_EGU3_IRQn;
        init_ts.evt_handler = ts_event_handler;
    
        ts_init(&init_ts);

        ts_rf_config_t rt_config = {
            .rf_chn = 80, 
            .rf_addr = {0xDE, 0xAD, 0xBE, 0xEF, 0x19}
        };

        ts_enable(&rt_config);

        ts_test_pin_configure(TEST_PIN);
    }

    void TimeSynchron::start(bool isMaster) {
        if (mIsStarted) {
            return;
        }
        mIsStarted = true;

        mIsMaster = isMaster;

        if (mIsMaster) {
            ts_tx_start(TIME_SYNC_FREQ_AUTO);
        }
    }

    void TimeSynchron::stop() {
        if (!mIsStarted) {
            return;
        }
        if (mIsMaster) {
            ts_tx_stop();
        }
        mIsMaster = false;
        mIsStarted = false;
        //ts_disable();
    }

    void TimeSynchron::stopSDRadioSession() {
        ts_sd_radio_session_stop();
        mIsMaster = false;
        mIsStarted = false;
    }
    void TimeSynchron::startSDRadioSession() {
        mIsRadioSessionStart = true;
    }

    void TimeSynchron::doStartSDRadioSession() {
        if (!mIsRadioSessionStart) {
            return;
        }
        
        if (!ts_get_pending_close()) {
            ts_sd_radio_session_start();
            start(mIsMaster);
            mIsRadioSessionStart = false;
        }
    }
}
