
#ifndef _NOTCH_BLE_HPP__
#define _NOTCH_BLE_HPP__

#include "nrf_sdh_ble.h"
#include "ble_conn_params.h"
#include "app_timer.h"
#include "ble_advertising.h"
#include "peer_manager.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_scan.h"

#include <string>

namespace  test {

#define SEC_PARAM_BOND                      1                                          /**< Perform bonding. */
#define SEC_PARAM_MITM                      0                                          /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                      0                                          /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS                  0                                          /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES           BLE_GAP_IO_CAPS_NONE                       /**< No I/O capabilities. */
#define SEC_PARAM_OOB                       0                                          /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE              7                                          /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE              16                   

#define BLE_CUSTOM_BASE                                                                                \
	{                                                                                                  \
		0x44, 0x98, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00 \
	}

#define BLE_NOTCH_FACTORY {\
                0x43, 0x98, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00 \
        }

  class BLE {

    //current ios max mtu
    static const int mMAX_MTU_SIZE = 244;
    uint8_t mReceiveBuffer[mMAX_MTU_SIZE];
    uint16_t mReceiveBufferLen;

    const int mAPP_BLE_CONN_CFG_TAG = 1;
    const char* mDEVICE_NAME = "Test";
    const uint16_t mCOMPANY_IDENTIFIER = 456;
    const int mMIN_CONN_INTERVAL = MSEC_TO_UNITS(7.5, UNIT_1_25_MS);
    const int mMAX_CONN_INTERVAL = MSEC_TO_UNITS(12, UNIT_1_25_MS);
    const int mSLAVE_LATENCY = 0;
    const int mCONN_SUP_TIMEOUT = MSEC_TO_UNITS(4000, UNIT_10_MS);

    const int mFIRST_CONN_PARAMS_UPDATE_DELAY = APP_TIMER_TICKS(5000);
    const int mNEXT_CONN_PARAMS_UPDATE_DELAY = APP_TIMER_TICKS(30000);
    const int mMAX_CONN_PARAMS_UPDATE_COUNT = 3;

    const int mAPP_ADV_FAST_INTERVAL = 0x0028;   /**< Fast advertising interval (in units of 0.625 ms. This value corresponds to 25 ms.). */
    const int mAPP_ADV_SLOW_INTERVAL = 0x0C80;   /**< Slow advertising interval (in units of 0.625 ms. This value corrsponds to 2 seconds). */

    const int mAPP_ADV_FAST_DURATION = 12000;     /**< The advertising duration of fast advertising in units of 10 milliseconds. */
    const int mAPP_ADV_SLOW_DURATION = 18000;    /**< The advertising duration of slow advertising in units of 10 milliseconds. */

    const ble_uuid128_t mBaseUUID = {BLE_CUSTOM_BASE};
    const ble_uuid_t mFactoryUUID = {
        .uuid = 0x1111, 
        .type = BLE_UUID_TYPE_BLE};
    const uint16_t mBaseServiceUUID = 0x1000;
    const uint16_t mCommandCharacteristicUUID = 0x2005;
    const uint16_t mOldResponseCharacteristicUUID = 0x2002;
    const uint16_t mResponseCharacteristicUUID = 0x2003;
    const uint16_t mStartBondingCharacteristicUUID = 0x2004;

    uint16_t mBaseServiceHandle;
    ble_gatts_char_handles_t mCommandCharacteristicHandle;
    ble_gatts_char_handles_t mResponseCharacteristicHandle;
    ble_gatts_char_handles_t mStartBondingCharacteristicHandle;
    ble_gatts_char_handles_t mOldResponseCharacteristicHandle;
    uint16_t mConnectionHandle = BLE_CONN_HANDLE_INVALID;
    uint8_t mAdvHandle;
    int64_t mRxTs = 0, mProcTs=0;

    uint8_t mTXPowerLevelChanged = 0;
    int8_t mTXPowerLevel = 0;

    uint8_t mRSSIChanged = 0;
    int8_t mRSSI = 0;
    uint8_t mChannel = 0;
    
    uint8_t mMTUChanged = 0;
    uint32_t mMTU;

    uint8_t mDoUpdatePhy = 0;
    uint8_t mPhyChanged = 0;
    uint32_t mTxPhy;
    uint32_t mRxPhy;

    uint8_t mAdvRestartWithoutWhiteList = 0;

    static pm_peer_id_t mPeerID;      /**< Device reference handle to the current bonded central. */

    static bool mIsScanShouldRun;
    static bool mIsScanRun;

    int mLastBatteryData = 0;

    ble_advdata_manuf_data_t mBLEManuData;
    static const int mManuDataSize = 7;
    uint8_t mManuData[mManuDataSize];
    ble_uuid_t mAdvUUIds[1] = {
      {mBaseServiceUUID, BLE_UUID_TYPE_VENDOR_BEGIN} //,
                                                     //{BLE_UUID_SENSOR_DATA_SERVICE, BLE_UUID_TYPE_BLE}//,
                                                     //{BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
    };

    uint8_t mIsStoreConnectedPeer = 0;

    public:
        void initialize();
        void advertisingStart(int durationInSeconds, bool deleteBondsInfo, bool isConnectable);
        void refreshAdvertisingBatteryData();

        int64_t RxTs() const { return mRxTs; }
        int64_t ProcTs() const { return mProcTs; }

        static void bleEventHandlerEntry(ble_evt_t const * p_ble_evt, void * p_context);

        bool OldSend(const void* data, uint16_t len);

        void startRSSIReport();
        void stopRSSIReport();

        void setTXPowerLevel();
        void setTXPowerLevel(int8_t txPowerLevel);

        void setSendMTUChangeRequest();
        void setSendPhyChangeRequest();

        void triggerPhyUpdate() {
            mDoUpdatePhy = 1;
        }

        void doUpdatePhy();

        void requestAdvertiseRestartWithoutWhiteList(void);

        void requestDeletBonds(uint8_t isStoreConnectedPeer);

        bool isConnected() const {
            return mConnectionHandle != BLE_CONN_HANDLE_INVALID;
        }

    private:

        bool DoSend(const void* data, uint16_t len);
        bool SendOnCh(const void* data, uint16_t len, const ble_gatts_char_handles_t& ch);

        static void gattEventHandler(nrf_ble_gatt_t *p_gatt, nrf_ble_gatt_evt_t const *p_evt);

        static void onConnParamsEvent(ble_conn_params_evt_t *p_evt);
        static void connParamsErrorHandler(uint32_t nrf_error);

        static void advEventHandler(ble_adv_evt_t ble_adv_evt);
        static void advErrorHandler(uint32_t nrf_error);

        static void scanEvtHandler(scan_evt_t const *scanEvt);

        static void pmEventHandler(pm_evt_t const * p_evt);
      
        void bleEventHandler(ble_evt_t const * p_ble_evt);

        static void identitiesSet(pm_peer_id_list_skip_t skip);
        static void whitelistSet(pm_peer_id_list_skip_t skip);
        void deleteBonds();

        static void scanStart();
        static void scanStop();

        void stackInit();
        void GATTInit();
        void GAPParamsInit();
        void connParamsInit();
        void servicesInit();
        void addDeviceInformationService();
        void setAdvertisingData(
                ble_advdata_t &advData, 
                ble_advdata_t &scanRespData);
        void advertisingInit(int durationInSeconds, bool isConnectable);
        void peerManagerInit();
        void scanInit();

        void addCommandCharacteristic();
        void addResponseCharacteristic();
        void addStartBondingCharacteristic();

        void addOldResponseCharacteristic();

        bool config_is_valid(ble_adv_modes_config_t const * const p_config);
        uint16_t adv_set_data_size_max_get(ble_advertising_t const * const p_advertising);
        uint32_t custom_ble_advertising_init(ble_advertising_t * const p_advertising,
                              ble_advertising_init_t const * const p_init, 
                              uint8_t type);
  };

//now this is for backward compatible communication
extern BLE ble;

}

#endif
