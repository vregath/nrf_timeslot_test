
#include "notch_ble.hpp"

#include "app_error.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "ble_dis.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_scan.h"
#include "nrf_delay.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_nvic.h"
#include "peer_manager_handler.h"

#include "log/log.h"
#include "power/power.h"

#include "time_synchron/time_synchron.h"

#include <SEGGER_RTT.h>

namespace notch {

// defined in sdk_config.h
//#define APP_BLE_OBSERVER_PRIO   3

//this must be put here because NRF_SDH_BLE_OBSERVER works only in global scope
NotchBLE notchBLE;
NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, notch::NotchBLE::bleEventHandlerEntry, &notchBLE);

NRF_BLE_GATT_DEF(m_gatt);

BLE_ADVERTISING_DEF(m_advertising);                                 /**< Advertising module instance. */

NRF_BLE_SCAN_DEF(m_scan);

pm_peer_id_t NotchBLE::mPeerID;
bool NotchBLE::mIsScanRun = false;
bool NotchBLE::mIsScanShouldRun = true;

void NotchBLE::initialize() {
    mIsStoreConnectedPeer = false;
    mConnectionHandle = BLE_CONN_HANDLE_INVALID;
    stackInit();
    scanInit();
    GATTInit();
    GAPParamsInit();
    connParamsInit();
    servicesInit();
    addDeviceInformationService();
    //advertisingInit();
    peerManagerInit();

    mIsScanShouldRun = true;
    mIsScanRun = false;

    scanStart();
}

void NotchBLE::doUpdatePhy() {
    if (!mDoUpdatePhy) {
        return;
    }
    if (mConnectionHandle == BLE_CONN_HANDLE_INVALID) {
        mDoUpdatePhy = 0;
        return;
    }
    mDoUpdatePhy = 0;
    ble_gap_phys_t const phys = {
            //.tx_phys = BLE_GAP_PHY_AUTO,
            //.rx_phys = BLE_GAP_PHY_AUTO, 
            .tx_phys = BLE_GAP_PHY_2MBPS,
            .rx_phys = BLE_GAP_PHY_2MBPS, 
        };

    uint32_t errorCode = sd_ble_gap_phy_update(mConnectionHandle, &phys);
    APP_ERROR_CHECK(errorCode);
}

void NotchBLE::requestAdvertiseRestartWithoutWhiteList() {
   
    // if we are connected advertise restart without whitelist happen after central disconnect
    if (mConnectionHandle != BLE_CONN_HANDLE_INVALID) {
        mAdvRestartWithoutWhiteList = 1;
    } 
    // if we are not connecting we restart advertise
    else {
        ret_code_t ret = ble_advertising_restart_without_whitelist(&m_advertising);
        APP_ERROR_CHECK(ret);
    }
}

/*
void NotchBLE::checkRequestAdvertiseRestartWithoutWhiteList()  {
    if (mAdvRestartWithoutWhiteList) {
        ret_code_t ret;
        ret = sd_ble_gap_disconnect(mConnectionHandle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(ret);
    }
}
*/

void NotchBLE::requestDeletBonds(uint8_t isStoreConnectedPeer) {
    mIsStoreConnectedPeer = isStoreConnectedPeer;
    advertisingStart(120, true, true);
}

void NotchBLE::advertisingStart(int durationInSeconds, bool deleteBondsInfo, bool isConnectable) {
    if (deleteBondsInfo) {
        deleteBonds();
        //if central disconnect adv will restart, we don't have to disconnect here
        /*
        if (mConnectionHandle != BLE_CONN_HANDLE_INVALID) {
            ret_code_t ret = sd_ble_gap_disconnect(mConnectionHandle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(ret);
        }
        */
        return;
    }
    whitelistSet(PM_PEER_ID_LIST_SKIP_NO_ID_ADDR);

    advertisingInit(durationInSeconds, isConnectable);

    if (mConnectionHandle == BLE_CONN_HANDLE_INVALID) {
        ret_code_t ret = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(ret);
    }
}

void NotchBLE::refreshAdvertisingBatteryData() {
    
    int batteryData = 4;
    if (batteryData == mLastBatteryData) {
        return;
    }
    mLastBatteryData = batteryData;

    ble_advdata_t advData;
    ble_advdata_t scanRespData;

    setAdvertisingData(advData, scanRespData);

    ret_code_t ret = ble_advertising_advdata_update(
        &m_advertising, 
        &advData, 
        &scanRespData
    );
    APP_ERROR_CHECK(ret);
    
}

void NotchBLE::scanStart() {
    if (!mIsScanShouldRun) {
        return;
    }
    if (mIsScanRun) {
        return;
    }
    ret_code_t err_code;
    err_code = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(err_code);
    mIsScanRun = true;
}

void NotchBLE::scanStop() {
    if (!mIsScanRun) {
        return;
    }
    nrf_ble_scan_stop();
    mIsScanRun = false;
}

void NotchBLE::identitiesSet(pm_peer_id_list_skip_t skip) {
    pm_peer_id_t peer_ids[BLE_GAP_DEVICE_IDENTITIES_MAX_COUNT];
    uint32_t     peer_id_count = BLE_GAP_DEVICE_IDENTITIES_MAX_COUNT;

    ret_code_t err_code = pm_peer_id_list(peer_ids, &peer_id_count, PM_PEER_ID_INVALID, skip);
    APP_ERROR_CHECK(err_code);

    scanStop();
    err_code = pm_device_identities_list_set(peer_ids, peer_id_count);
    APP_ERROR_CHECK(err_code);
    scanStart();
}

void NotchBLE::whitelistSet(pm_peer_id_list_skip_t skip) {
    pm_peer_id_t peer_ids[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
    uint32_t     peer_id_count = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

    ret_code_t err_code = pm_peer_id_list(peer_ids, &peer_id_count, PM_PEER_ID_INVALID, skip);
    APP_ERROR_CHECK(err_code);

    SEGGER_RTT_printf(0, "\tm_whitelist_peer_cnt %d, MAX_PEERS_WLIST %d\n", 
                peer_id_count + 1, BLE_GAP_WHITELIST_ADDR_MAX_COUNT);

    scanStop();
    err_code = pm_whitelist_set(peer_ids, peer_id_count);
    APP_ERROR_CHECK(err_code);
    scanStart();
}

bool NotchBLE::DoSend(const void *data, uint16_t len) {
    return SendOnCh(data, len, mResponseCharacteristicHandle);
}

bool NotchBLE::OldSend(const void *data, uint16_t len) {
    return SendOnCh(data, len, mOldResponseCharacteristicHandle);
}

void NotchBLE::startRSSIReport() {
    ret_code_t errorCode = sd_ble_gap_rssi_start(mConnectionHandle, 10, 0);   
    APP_ERROR_CHECK(errorCode); 
}

void NotchBLE::stopRSSIReport() {
    ret_code_t errorCode = sd_ble_gap_rssi_stop(mConnectionHandle);   
}

void NotchBLE::setTXPowerLevel() {
    setTXPowerLevel(mTXPowerLevel);
}

void NotchBLE::setTXPowerLevel(int8_t txPowerLevel) {
    if (mConnectionHandle == BLE_CONN_HANDLE_INVALID) {
        return;
    }
    /// if we set -40 connection lost sendMessage throw error code 13
    /// if we want -40 this should be a separate ticket to fix this
    if (txPowerLevel == -40) {
        txPowerLevel = -20;
    }
    ret_code_t ret = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN, mConnectionHandle, txPowerLevel);
    //APP_ERROR_CHECK(ret);
    if (ret == NRF_SUCCESS) {
        mTXPowerLevel = txPowerLevel;
        mTXPowerLevelChanged = 1;
        //protocol->SendTXPowerLevelChangeReport(txPowerLevel);
    } else {
        SEGGER_RTT_printf(0, "txPowerLevelSet(%d) error: %x\n", txPowerLevel, ret);
    }
}

void NotchBLE::setSendMTUChangeRequest() {
    mMTUChanged = 1;
}

void NotchBLE::setSendPhyChangeRequest() {
    mPhyChanged = 1;
}

bool NotchBLE::SendOnCh(const void* data, uint16_t len, const ble_gatts_char_handles_t& ch) {

    // we are not connected
    if (mConnectionHandle == BLE_CONN_HANDLE_INVALID) {
        return false;
    }

    //uint8_t nested;
    //sd_nvic_critical_region_enter(&nested);

    ble_gatts_hvx_params_t hvx_params;
    memset(&hvx_params, 0, sizeof(hvx_params));
    hvx_params.handle = ch.value_handle;
    hvx_params.type = BLE_GATT_HVX_NOTIFICATION;
    hvx_params.offset = 0;
    // TODO: here somehow a len check look necessary using current mtu and max message length
    hvx_params.p_len = &len;
    hvx_params.p_data = (uint8_t *)data;

    uint32_t result = sd_ble_gatts_hvx(mConnectionHandle, &hvx_params);

    //sd_nvic_critical_region_exit(nested);

    if (result != NRF_SUCCESS && ch.value_handle == mResponseCharacteristicHandle.value_handle) {
    SEGGER_RTT_printf(0, "message send error: %x\n", result);
    /// 3401: BLE_GATTS_EVT_SYS_ATTR_MISSING
    /// 13: 
    /// getting BLE_ERROR_GATTS_SYS_ATTR_MISSING on one of the characteristics when sending on both (depends on the app used which characteristic)
    }

    return result == NRF_SUCCESS;
}

void NotchBLE::bleEventHandlerEntry(ble_evt_t const *p_ble_evt, void *p_context) {
  NotchBLE *me = (NotchBLE *)p_context;
  me->bleEventHandler(p_ble_evt);
}

void NotchBLE::bleEventHandler(ble_evt_t const *p_ble_evt) {
    ret_code_t errorCode;

    //SEGGER_RTT_printf(0, "BLE event %d\n", p_ble_evt->header.evt_id);

    switch (p_ble_evt->header.evt_id) {
    case BLE_GAP_EVT_CONNECTED: {
        mIsScanShouldRun = false;
        scanStop();

        SEGGER_RTT_printf(0, "BLE connected\n");
        mConnectionHandle = p_ble_evt->evt.gap_evt.conn_handle;
        }
        break;
    case BLE_GAP_EVT_DISCONNECTED:
        SEGGER_RTT_printf(0, "BLE disconnected\n");
        mIsScanShouldRun = true;

        scanStart();
        
        mConnectionHandle = BLE_CONN_HANDLE_INVALID;
        break;
    case BLE_GAP_EVT_TIMEOUT:
        DBGLOGI("BLE", "BLE_GAP_EVT_TIMEOUT");
        break;
    case BLE_GATTS_EVT_WRITE: {
            
            const ble_gatts_evt_write_t *writeEvent = &p_ble_evt->evt.gatts_evt.params.write;

            if (writeEvent->len > mMAX_MTU_SIZE) {
                //TODO: error field save
                SEGGER_RTT_printf(0, "mtu size skip");
                break;
            }
            
            DBGLOGI("BLE", "Get message: %x", writeEvent->data[0]);
            if (writeEvent->len != 1) {
                break;
            }
            if (writeEvent->data[0] == 0xFF) {
                notch::timeSynchron.setIsMaster(true);
                notch::timeSynchron.startSDRadioSession();
                break;
            }
            if (writeEvent->data[0] == 0xAA) {
                notch::timeSynchron.setIsMaster(false);
                notch::timeSynchron.startSDRadioSession();
                break;
            }
            if (writeEvent->data[0] == 0x11) {
                notch::timeSynchron.stopSDRadioSession();
                break;
            }

            
        }
        break;
    case BLE_GAP_EVT_ADV_SET_TERMINATED:
        if (mAdvRestartWithoutWhiteList) {
            SEGGER_RTT_printf(0, "RESTART advertise without whitelist\n");
            mAdvRestartWithoutWhiteList = 0;
            m_advertising.whitelist_temporarily_disabled = true;
            m_advertising.whitelist_in_use               = false;
            m_advertising.adv_params.filter_policy       = BLE_GAP_ADV_FP_ANY;
            break;
        } else {
            m_advertising.whitelist_temporarily_disabled = false;
        }
        break;
    case BLE_GAP_EVT_PHY_UPDATE_REQUEST: {
            SEGGER_RTT_printf(0, "BLE_GAP_EVT_PHY_UPDATE_REQUEST\n");
            ble_gap_phys_t const phys = {
                //.tx_phys = BLE_GAP_PHY_AUTO,
                //.rx_phys = BLE_GAP_PHY_AUTO, 
                .tx_phys = BLE_GAP_PHY_2MBPS,
                .rx_phys = BLE_GAP_PHY_2MBPS, 
            };
            errorCode = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(errorCode);
            SEGGER_RTT_printf(0, "tx phy: %x, rx phy: %x\n", phys.tx_phys, phys.rx_phys);
        }
        break;
    case BLE_GAP_EVT_PHY_UPDATE: {
            ble_gap_evt_phy_update_t phy_update = p_ble_evt->evt.gap_evt.params.phy_update;
            if(phy_update.status == BLE_HCI_STATUS_CODE_SUCCESS) {
                SEGGER_RTT_printf(0, "PHY updated: %d, %d\n", phy_update.tx_phy, phy_update.rx_phy);
                mTxPhy = phy_update.tx_phy;
                mRxPhy = phy_update.rx_phy;
                mPhyChanged = 1;
            } else {
                DBGLOGI("BLE", "PHY failre: %d, %d %d", phy_update.status, phy_update.tx_phy, phy_update.rx_phy);
                ble_gap_phys_t const phys = {
                    .tx_phys = BLE_GAP_PHY_AUTO,
                    .rx_phys = BLE_GAP_PHY_AUTO, 
                };
                errorCode = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
                APP_ERROR_CHECK(errorCode);
            }
            
        }
        break;
    case BLE_GATTS_EVT_SYS_ATTR_MISSING:
        SEGGER_RTT_printf(0, "sys attr missing\n");
        errorCode = sd_ble_gatts_sys_attr_set(mConnectionHandle, NULL, 0, 0);
        APP_ERROR_CHECK(errorCode);
        break;
    
    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
        SEGGER_RTT_printf(0, "sec params request\n");
        break;
    case BLE_GAP_EVT_RSSI_CHANGED: {
            int8_t rssi; 
            uint8_t channelIdx; 
            errorCode = sd_ble_gap_rssi_get(mConnectionHandle, &rssi, &channelIdx);  
            DBGLOGI("BLE", "rssi: %d, ch: %d", rssi, channelIdx);
            mRSSI = rssi;
            mChannel = channelIdx;
            mRSSIChanged = 1;
            APP_ERROR_CHECK(errorCode);
        }
        break;
    case BLE_GAP_EVT_ADV_REPORT:
        //SEGGER_RTT_printf(0, "ADV report\n");
        if (!p_ble_evt->evt.gap_evt.params.adv_report.type.scan_response) {
            break;
        }
        //parseAdvData(
        //    p_ble_evt->evt.gap_evt.params.adv_report.data.p_data, 
        //    p_ble_evt->evt.gap_evt.params.adv_report.data.len);
        break;
    default: // error handler?
        break;
    }
}

void NotchBLE::advEventHandler(ble_adv_evt_t ble_adv_evt) {
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_DIRECTED_HIGH_DUTY:
            SEGGER_RTT_printf(0, "High Duty Directed advertising.\n");
            //NRF_LOG_INFO("High Duty Directed advertising.");
            //err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_DIRECTED);
            //APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_DIRECTED:
            SEGGER_RTT_printf(0, "Directed advertising.\n");
            //NRF_LOG_INFO("Directed advertising.");
            //err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_DIRECTED);
            //APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_FAST:
            SEGGER_RTT_printf(0, "Fast advertising.\n");
            //NRF_LOG_INFO("Fast advertising.");
            //err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            //APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_SLOW:
            SEGGER_RTT_printf(0, "Slow advertising.\n");
            //NRF_LOG_INFO("Slow advertising.");
            //err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_SLOW);
            //APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_FAST_WHITELIST:
            SEGGER_RTT_printf(0, "Fast advertising with whitelist.\n");
            //NRF_LOG_INFO("Fast advertising with whitelist.");
            //err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_WHITELIST);
            //APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_SLOW_WHITELIST:
            SEGGER_RTT_printf(0, "Slow advertising with whitelist.\n");
            //NRF_LOG_INFO("Slow advertising with whitelist.");
            //err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_WHITELIST);
            //APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            SEGGER_RTT_printf(0, "ADV IDLE\n");

            releasePower();

            SEGGER_RTT_printf(0, "RESTART advertise after IDLE\n");
            notchBLE.advertisingStart(120, false, true);

            //sleep_mode_enter();
            break;

        case BLE_ADV_EVT_WHITELIST_REQUEST:
        {
            ble_gap_addr_t whitelist_addrs[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
            ble_gap_irk_t  whitelist_irks[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
            uint32_t       addr_cnt = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;
            uint32_t       irk_cnt  = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

            SEGGER_RTT_printf(0, "BLE ADV whitelist request\n");

            err_code = pm_whitelist_get(whitelist_addrs, &addr_cnt,
                                        whitelist_irks,  &irk_cnt);
            APP_ERROR_CHECK(err_code);
            //NRF_LOG_DEBUG("pm_whitelist_get returns %d addr in whitelist and %d irk whitelist",
            //              addr_cnt, irk_cnt);
            SEGGER_RTT_printf(0, "pm_whitelist_get returns %d addr in whitelist and %d irk whitelist\n", 
                            addr_cnt, irk_cnt);

            // without we get an error that whitelist list is in use
            sd_ble_gap_adv_stop(m_advertising.adv_handle);
            // Set the correct identities list (no excluding peers with no Central Address Resolution).
            identitiesSet(PM_PEER_ID_LIST_SKIP_NO_IRK);

            //ble_advertising_start(&m_advertising, m_advertising.adv_mode_current);

            // Apply the whitelist.
            err_code = ble_advertising_whitelist_reply(&m_advertising,
                                                       whitelist_addrs,
                                                       addr_cnt,
                                                       whitelist_irks,
                                                       irk_cnt);
            APP_ERROR_CHECK(err_code);

        } break; //BLE_ADV_EVT_WHITELIST_REQUEST

        case BLE_ADV_EVT_PEER_ADDR_REQUEST:
        {
            SEGGER_RTT_printf(0, "PEER addr request\n");
            pm_peer_data_bonding_t peer_bonding_data;

            // Only Give peer address if we have a handle to the bonded peer.
            if (mPeerID != PM_PEER_ID_INVALID)
            {
                err_code = pm_peer_data_bonding_load(mPeerID, &peer_bonding_data);
                if (err_code != NRF_ERROR_NOT_FOUND)
                {
                    APP_ERROR_CHECK(err_code);

                    // without we get an error that whitelist list is in use
                    sd_ble_gap_adv_stop(m_advertising.adv_handle);
                    // Manipulate identities to exclude peers with no Central Address Resolution.
                    identitiesSet(PM_PEER_ID_LIST_SKIP_ALL);

                    ble_gap_addr_t * p_peer_addr = &(peer_bonding_data.peer_ble_id.id_addr_info);
                    err_code = ble_advertising_peer_addr_reply(&m_advertising, p_peer_addr);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; //BLE_ADV_EVT_PEER_ADDR_REQUEST

        default:
            break;
    }
}

void NotchBLE::advErrorHandler(uint32_t nrf_error) {
    APP_ERROR_HANDLER(nrf_error);
}

void NotchBLE::scanEvtHandler(scan_evt_t const *scanEvt) {
    ret_code_t err_code;

    //SEGGER_RTT_printf(0, "Scan event\n");

    switch(scanEvt->scan_evt_id) {
        case NRF_BLE_SCAN_EVT_FILTER_MATCH:
            SEGGER_RTT_printf(0, "NRF_BLE_SCAN_EVT_FILTER_MATCH\n");
            // TODO: process data
            break;
        case NRF_BLE_SCAN_EVT_WHITELIST_REQUEST:
            SEGGER_RTT_printf(0, "NRF_BLE_SCAN_EVT_WHITELIST_REQUEST\n");
            break;
        case NRF_BLE_SCAN_EVT_WHITELIST_ADV_REPORT:
            SEGGER_RTT_printf(0, "NRF_BLE_SCAN_EVT_WHITELIST_ADV_REPORT\n");
            break;
        case NRF_BLE_SCAN_EVT_NOT_FOUND:
            //SEGGER_RTT_printf(0, "NRF_BLE_SCAN_EVT_NOT_FOUND\n");
            break;
        case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:
            SEGGER_RTT_printf(0, "NRF_BLE_SCAN_EVT_SCAN_TIMEOUT\n");
            break;
        case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
            SEGGER_RTT_printf(0, "NRF_BLE_SCAN_EVT_CONNECTING_ERROR\n");
            break;
        case NRF_BLE_SCAN_EVT_CONNECTED:
            SEGGER_RTT_printf(0, "NRF_BLE_SCAN_EVT_CONNECTED\n");
            break;
    }
    
}

void NotchBLE::pmEventHandler(pm_evt_t const * p_evt) {
    SEGGER_RTT_printf(0, "pmEventHandler: %d\n", p_evt->evt_id);
    pm_handler_on_pm_evt(p_evt);
    //pm_handler_disconnect_on_sec_failure(p_evt);
    pm_handler_flash_clean(p_evt);

    ret_code_t err_code;

    switch (p_evt->evt_id)
    {
        case PM_EVT_CONN_SEC_SUCCEEDED:
            mPeerID = p_evt->peer_id;
            break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            SEGGER_RTT_printf(0, "Peers delete, advertising start\n");
            notchBLE.advertisingStart(120, false, true);
            break;

        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
            if (     p_evt->params.peer_data_update_succeeded.flash_changed
                 && (p_evt->params.peer_data_update_succeeded.data_id == PM_PEER_DATA_ID_BONDING))
            {
                SEGGER_RTT_printf(0, "New Bond, add the peer to the whitelist if possible\n");
                //NRF_LOG_INFO("New Bond, add the peer to the whitelist if possible");
                // Note: You should check on what kind of white list policy your application should use.

                whitelistSet(PM_PEER_ID_LIST_SKIP_NO_ID_ADDR);
            }
            break;
        case PM_EVT_CONN_SEC_CONFIG_REQ: {
                SEGGER_RTT_printf(0, "pmEventHandler: PM_EVT_CONN_SEC_CONFIG_REQ\n");
                pm_conn_sec_config_t config = {.allow_repairing = true};
                pm_conn_sec_config_reply(p_evt->conn_handle, &config);
            }
            break;
        case PM_EVT_CONN_CONFIG_REQ: 
            SEGGER_RTT_printf(0, "pmEventHandler: PM_EVT_CONN_CONFIG_REQ\n");
            break;

        case PM_EVT_CONN_SEC_PARAMS_REQ:
            SEGGER_RTT_printf(0, "pmEventHandler: PM_EVT_CONN_SEC_PARAMS_REQ\n");
            /*
            err_code = sd_ble_gap_sec_params_reply(
                notchBLE.mConnectionHandle, BLE_GAP_SEC_STATUS_SUCCESS, NULL, 
                p_evt->params.conn_sec_params_req.p_context);
        
            APP_ERROR_CHECK(err_code);
            */
            break;
        case PM_EVT_CONN_SEC_FAILED: {
            SEGGER_RTT_printf(0, "pmEventHandler: PM_EVT_CONN_SEC_FAILED\n");

            }
            break;
        default:
            break;
    }
}

void NotchBLE::gattEventHandler(nrf_ble_gatt_t *p_gatt, nrf_ble_gatt_evt_t const *p_evt) {
    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED) {
        SEGGER_RTT_printf(0, "GATT ATT MTU on connection 0x%x changed to %d\n",
                     p_evt->conn_handle,
                     p_evt->params.att_mtu_effective);
        notchBLE.mMTU = p_evt->params.att_mtu_effective;
        notchBLE.mMTUChanged = 1;
    }
}

void NotchBLE::onConnParamsEvent(ble_conn_params_evt_t *p_evt) {
    uint32_t err_code;

    switch (p_evt->evt_type) {
    case BLE_CONN_PARAMS_EVT_FAILED:
    // err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);					//ble_hci.h
    // APP_ERROR_CHECK(err_code);
    // NRF_LOG_WARNING("BLE_CONN_PARAMS_EVT_FAILED\n");
    break;

    default:
    // No implementation needed.
    break;
    }
}

void NotchBLE::connParamsErrorHandler(uint32_t nrf_error) {
    APP_ERROR_HANDLER(nrf_error);
}

void NotchBLE::deleteBonds() {
    ret_code_t err_code;

    if (!mIsStoreConnectedPeer) {
        err_code = pm_peers_delete();
        APP_ERROR_CHECK(err_code);
    } else {
        pm_peer_id_t connectedPeerID;
        err_code = pm_peer_id_get(mConnectionHandle, &connectedPeerID);
        if (err_code != NRF_SUCCESS) {
            return;
        }
        pm_peer_id_t peer_id_list[PM_PEER_ID_N_AVAILABLE_IDS];
        uint32_t peer_id_count = PM_PEER_ID_N_AVAILABLE_IDS;
        pm_peer_id_list(peer_id_list, &peer_id_count, PM_PEER_ID_INVALID, PM_PEER_ID_LIST_ALL_ID);

        for(int i=0;i<peer_id_count;i++) {
            if (peer_id_list[i] == connectedPeerID) {
                continue;
            }
            pm_peer_delete(peer_id_list[i]);
        }
    }
}

void NotchBLE::stackInit() {
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    ASSERT(nrf_sdh_is_enabled());

    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(mAPP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);
}

void NotchBLE::GATTInit() {
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NotchBLE::gattEventHandler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}

void NotchBLE::GAPParamsInit() {
    uint32_t err_code;
    ble_gap_conn_params_t gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    uint8_t tempAddr[6];
    memcpy(tempAddr, (void *)(&NRF_FICR->DEVICEADDR[0]), 6);
    char deviceName[9] = {0};
    //snprintf(deviceName, 9, "%s#%x", mDEVICE_NAME, tempAddr[5]);
    snprintf(deviceName, 9, "%s", mDEVICE_NAME);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
      (const uint8_t *)deviceName,
      strlen(deviceName));
    APP_ERROR_CHECK(err_code);

    //err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HEART_RATE_SENSOR_HEART_RATE_BELT);
    //APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = mMIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = mMAX_CONN_INTERVAL;
    gap_conn_params.slave_latency = mSLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout = mCONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

void NotchBLE::connParamsInit() {
    uint32_t err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params = NULL;
    cp_init.first_conn_params_update_delay = mFIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay = mNEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count = mMAX_CONN_PARAMS_UPDATE_COUNT;
    // cp_init.start_on_notify_cccd_handle = mResponseCharacteristicHandle.cccd_handle;
    cp_init.start_on_notify_cccd_handle = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail = false;
    cp_init.evt_handler = onConnParamsEvent;
    cp_init.error_handler = connParamsErrorHandler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

void NotchBLE::servicesInit() {
    uint32_t err_code;
    ble_uuid_t service_uuid;

    service_uuid.uuid = mBaseServiceUUID;

    err_code = sd_ble_uuid_vs_add(&mBaseUUID, &service_uuid.type);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
      &service_uuid,
      &mBaseServiceHandle);
    APP_ERROR_CHECK(err_code);

    addCommandCharacteristic();
    addResponseCharacteristic();
    addStartBondingCharacteristic();
    addOldResponseCharacteristic();
}

void NotchBLE::addDeviceInformationService() {
    uint32_t err_code;
    ble_dis_init_t dis_init;
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)"timesloat");
    ble_srv_ascii_to_utf8(&dis_init.model_num_str, (char *)"23");
    ble_srv_ascii_to_utf8(&dis_init.hw_rev_str, (char *)"0");
    ble_srv_ascii_to_utf8(&dis_init.fw_rev_str, (char *)"0");

    ble_gap_addr_t MAC;
    char pr[20];
    memcpy(&MAC.addr, (void *)(&NRF_FICR->DEVICEADDR[0]), 6);
    sprintf(pr, "%02X%02X%02X%02X%02X%02X\0", MAC.addr[5], MAC.addr[4], MAC.addr[3], MAC.addr[2], MAC.addr[1], MAC.addr[0]);
    dis_init.serial_num_str.p_str = (uint8_t *)pr;
    dis_init.serial_num_str.length = 13;

    dis_init.dis_char_rd_sec = SEC_OPEN;

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
}

void NotchBLE::setAdvertisingData(ble_advdata_t &advData, ble_advdata_t &scanRespData) {
    memset(&advData, 0, sizeof(ble_advdata_t));
    memset(&scanRespData, 0, sizeof(ble_advdata_t));

    uint8_t                adv_flags;
    adv_flags                       = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    advData.name_type               = BLE_ADVDATA_FULL_NAME;
    advData.include_appearance      = true;
    advData.flags                   = adv_flags;
    advData.uuids_complete.uuid_cnt = sizeof(mAdvUUIds) / sizeof(mAdvUUIds[0]);
    advData.uuids_complete.p_uuids  = mAdvUUIds;

    uint8_t tempAddr[6];
    memcpy(tempAddr, (void *)(&NRF_FICR->DEVICEADDR[0]), 6);
    mManuData[0] = tempAddr[5];
    mManuData[1] = tempAddr[4];
    mManuData[2] = tempAddr[3];
    mManuData[3] = tempAddr[2];
    mManuData[4] = tempAddr[1];
    mManuData[5] = tempAddr[0];
    mManuData[6] = (uint8_t)(4);

    mBLEManuData.company_identifier = mCOMPANY_IDENTIFIER;
    mBLEManuData.data.p_data = mManuData;
    mBLEManuData.data.size = mManuDataSize;

    scanRespData.name_type = BLE_ADVDATA_FULL_NAME;
    scanRespData.p_manuf_specific_data = &mBLEManuData;
}

void NotchBLE::advertisingInit(int durationInSeconds, bool isConnectable) {

    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    setAdvertisingData(init.advdata, init.srdata);
    
    init.config.ble_adv_whitelist_enabled          = true;
    init.config.ble_adv_directed_high_duty_enabled = isConnectable;
    init.config.ble_adv_directed_enabled           = false;
    init.config.ble_adv_directed_interval          = 0;
    init.config.ble_adv_directed_timeout           = 0;
    init.config.ble_adv_fast_enabled               = true;
    init.config.ble_adv_fast_interval              = mAPP_ADV_FAST_INTERVAL;
    init.config.ble_adv_fast_timeout               = durationInSeconds * 100;
    init.config.ble_adv_slow_enabled               = false;
    init.config.ble_adv_slow_interval              = mAPP_ADV_SLOW_INTERVAL;
    init.config.ble_adv_slow_timeout               = mAPP_ADV_SLOW_DURATION;

    init.evt_handler   = advEventHandler;
    init.error_handler = advErrorHandler;

    uint8_t type = isConnectable ? BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED : 
                                   BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;

    DBGLOGI(__FUNCTION__, "ADVERTISING_TYPE: %d, %x", isConnectable, type);

    //err_code = custom_ble_advertising_init(&m_advertising, &init, type);
    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, mAPP_BLE_CONN_CFG_TAG);
}

// copy from ble_advertising.c because we need custom init function
bool NotchBLE::config_is_valid(ble_adv_modes_config_t const * const p_config) {
    if ((p_config->ble_adv_directed_high_duty_enabled == true) &&
        (p_config->ble_adv_extended_enabled == true))
    {
        return false;
    }
#if !defined (S140)
    else if ( p_config->ble_adv_primary_phy == BLE_GAP_PHY_CODED ||
              p_config->ble_adv_secondary_phy == BLE_GAP_PHY_CODED)
    {
        return false;
    }
#endif // !defined (S140)
    else
    {
        return true;
    }
}

// copy from ble_advertising.c because we need custom init function
uint16_t NotchBLE::adv_set_data_size_max_get(ble_advertising_t const * const p_advertising) {
    uint16_t adv_set_data_size_max;

    if (p_advertising->adv_modes_config.ble_adv_extended_enabled == true)
    {
#ifdef BLE_GAP_ADV_SET_DATA_SIZE_EXTENDED_CONNECTABLE_MAX_SUPPORTED
        adv_set_data_size_max = BLE_GAP_ADV_SET_DATA_SIZE_EXTENDED_CONNECTABLE_MAX_SUPPORTED;
#else
        adv_set_data_size_max = BLE_GAP_ADV_SET_DATA_SIZE_MAX;
#endif // BLE_GAP_ADV_SET_DATA_SIZE_EXTENDED_CONNECTABLE_MAX_SUPPORTED
    }
    else
    {
        adv_set_data_size_max = BLE_GAP_ADV_SET_DATA_SIZE_MAX;
    }

    return adv_set_data_size_max;
}


uint32_t NotchBLE::custom_ble_advertising_init(ble_advertising_t * const p_advertising,
                              ble_advertising_init_t const * const p_init, 
                              uint8_t type) {
                              
    uint32_t ret;
    if ((p_init == NULL) || (p_advertising == NULL))
    {
        return NRF_ERROR_NULL;
    }
    if (!config_is_valid(&p_init->config))
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    p_advertising->adv_mode_current               = BLE_ADV_MODE_IDLE;
    p_advertising->adv_modes_config               = p_init->config;
    p_advertising->conn_cfg_tag                   = BLE_CONN_CFG_TAG_DEFAULT;
    p_advertising->evt_handler                    = p_init->evt_handler;
    p_advertising->error_handler                  = p_init->error_handler;
    p_advertising->current_slave_link_conn_handle = BLE_CONN_HANDLE_INVALID;
    p_advertising->p_adv_data                     = &p_advertising->adv_data;

    memset(&p_advertising->peer_address, 0, sizeof(p_advertising->peer_address));

    // Copy advertising data.
    if (!p_advertising->initialized)
    {
        p_advertising->adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;
    }
    p_advertising->adv_data.adv_data.p_data = p_advertising->enc_advdata[0];
    p_advertising->adv_data.adv_data.len = adv_set_data_size_max_get(p_advertising);

    ret = ble_advdata_encode(&p_init->advdata, p_advertising->enc_advdata[0], &p_advertising->adv_data.adv_data.len);
    VERIFY_SUCCESS(ret);

    p_advertising->adv_data.scan_rsp_data.p_data = p_advertising->enc_scan_rsp_data[0];
    p_advertising->adv_data.scan_rsp_data.len = adv_set_data_size_max_get(p_advertising);

    ret = ble_advdata_encode(&p_init->srdata,
                              p_advertising->adv_data.scan_rsp_data.p_data,
                             &p_advertising->adv_data.scan_rsp_data.len);
    VERIFY_SUCCESS(ret);

    // Configure a initial advertising configuration. The advertising data and and advertising
    // parameters will be changed later when we call @ref ble_advertising_start, but must be set
    // to legal values here to define an advertising handle.
    p_advertising->adv_params.primary_phy     = BLE_GAP_PHY_1MBPS;
    p_advertising->adv_params.duration        = p_advertising->adv_modes_config.ble_adv_fast_timeout;
    p_advertising->adv_params.properties.type = type;
    p_advertising->adv_params.p_peer_addr     = NULL;
    p_advertising->adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    p_advertising->adv_params.interval        = p_advertising->adv_modes_config.ble_adv_fast_interval;

    ret = sd_ble_gap_adv_set_configure(&p_advertising->adv_handle, NULL, &p_advertising->adv_params);
    VERIFY_SUCCESS(ret);

    p_advertising->initialized = true;
    return ret;
}

void NotchBLE::peerManagerInit() {
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pmEventHandler);
    APP_ERROR_CHECK(err_code);
}

void NotchBLE::scanInit() {
    ret_code_t err_code;
    nrf_ble_scan_init_t scanInitData;
    memset(&scanInitData, 0, sizeof(scanInitData));

    scanInitData.connect_if_match = false;
    scanInitData.conn_cfg_tag = mAPP_BLE_CONN_CFG_TAG;

    err_code = nrf_ble_scan_init(&m_scan, &scanInitData, scanEvtHandler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_UUID_FILTER, false);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_UUID_FILTER, &mNotchFactoryUUID);
    APP_ERROR_CHECK(err_code);
}

void NotchBLE::addCommandCharacteristic() {
  uint32_t err_code;

  ble_uuid_t char_uuid;
  char_uuid.uuid = mCommandCharacteristicUUID;
  err_code = sd_ble_uuid_vs_add(&mBaseUUID, &char_uuid.type);
  APP_ERROR_CHECK(err_code);

  // metdata, attributes will store softdevice managed memory
  ble_gatts_attr_md_t attr_md;
  memset(&attr_md, 0, sizeof(attr_md));
  attr_md.vloc = BLE_GATTS_VLOC_STACK;
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
  attr_md.vlen = 1;

  ble_gatts_char_md_t char_md;
  memset(&char_md, 0, sizeof(char_md));
  char_md.char_props.write_wo_resp = 1;

  ble_gatts_attr_t attr_char_value;
  memset(&attr_char_value, 0, sizeof(attr_md));
  attr_char_value.p_uuid = &char_uuid;
  attr_char_value.p_attr_md = &attr_md;
  attr_char_value.init_len = 0;
  attr_char_value.init_offs = 0;
  attr_char_value.max_len = 80;

  err_code = sd_ble_gatts_characteristic_add(mBaseServiceHandle,
      &char_md,
      &attr_char_value,
      &mCommandCharacteristicHandle);
  APP_ERROR_CHECK(err_code);
}

void NotchBLE::addResponseCharacteristic() {
  uint32_t err_code;

  ble_uuid_t char_uuid;
  char_uuid.uuid = mResponseCharacteristicUUID;
  err_code = sd_ble_uuid_vs_add(&mBaseUUID, &char_uuid.type);
  APP_ERROR_CHECK(err_code);

  // metdata, attributes will store softdevice managed memory
  ble_gatts_attr_md_t attr_md;
  memset(&attr_md, 0, sizeof(attr_md));
  attr_md.vloc = BLE_GATTS_VLOC_STACK;
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
  attr_md.vlen = 1;

  ble_gatts_attr_md_t cccd_md;
  memset(&cccd_md, 0, sizeof(cccd_md));
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
  //BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&cccd_md.write_perm);
  cccd_md.vloc = BLE_GATTS_VLOC_STACK;

  ble_gatts_char_md_t char_md;
  memset(&char_md, 0, sizeof(char_md));
  char_md.char_props.read = 1;
  char_md.char_props.notify = 1;
  char_md.p_cccd_md = &cccd_md;

  ble_gatts_attr_t attr_char_value;
  memset(&attr_char_value, 0, sizeof(attr_char_value));
  attr_char_value.p_uuid = &char_uuid;
  attr_char_value.p_attr_md = &attr_md;
  attr_char_value.init_len = 4;
  attr_char_value.init_offs = 0;
  attr_char_value.max_len = mMAX_MTU_SIZE;
  uint8_t value[4] = {0x12, 0x34, 0x56, 0x78};
  attr_char_value.p_value = value;

  err_code = sd_ble_gatts_characteristic_add(mBaseServiceHandle,
      &char_md,
      &attr_char_value,
      &mResponseCharacteristicHandle);
  APP_ERROR_CHECK(err_code);
}

void NotchBLE::addStartBondingCharacteristic() {
  uint32_t err_code;

  ble_uuid_t char_uuid;
  char_uuid.uuid = mStartBondingCharacteristicUUID;
  err_code = sd_ble_uuid_vs_add(&mBaseUUID, &char_uuid.type);
  APP_ERROR_CHECK(err_code);

  // metdata, attributes will store softdevice managed memory
  ble_gatts_attr_md_t attr_md;
  memset(&attr_md, 0, sizeof(attr_md));
  attr_md.vloc = BLE_GATTS_VLOC_STACK;
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
  attr_md.vlen = 1;

  ble_gatts_attr_md_t cccd_md;
  memset(&cccd_md, 0, sizeof(cccd_md));
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
  //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
  BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&cccd_md.write_perm);
  cccd_md.vloc = BLE_GATTS_VLOC_STACK;

  ble_gatts_char_md_t char_md;
  memset(&char_md, 0, sizeof(char_md));
  char_md.char_props.read = 1;
  char_md.char_props.notify = 1;
  char_md.p_cccd_md = &cccd_md;

  ble_gatts_attr_t attr_char_value;
  memset(&attr_char_value, 0, sizeof(attr_char_value));
  attr_char_value.p_uuid = &char_uuid;
  attr_char_value.p_attr_md = &attr_md;
  attr_char_value.init_len = 4;
  attr_char_value.init_offs = 0;
  attr_char_value.max_len = mMAX_MTU_SIZE;
  uint8_t value[4] = {0x12, 0x34, 0x56, 0x78};
  attr_char_value.p_value = value;

  err_code = sd_ble_gatts_characteristic_add(mBaseServiceHandle,
      &char_md,
      &attr_char_value,
      &mStartBondingCharacteristicHandle);
  APP_ERROR_CHECK(err_code);
}

void NotchBLE::addOldResponseCharacteristic() {
  uint32_t err_code;

  ble_uuid_t char_uuid;
  char_uuid.uuid = mOldResponseCharacteristicUUID;
  err_code = sd_ble_uuid_vs_add(&mBaseUUID, &char_uuid.type);
  APP_ERROR_CHECK(err_code);

  // metdata, attributes will store softdevice managed memory
  ble_gatts_attr_md_t attr_md;
  memset(&attr_md, 0, sizeof(attr_md));
  attr_md.vloc = BLE_GATTS_VLOC_STACK;
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
  attr_md.vlen = 1;

  ble_gatts_attr_md_t cccd_md;
  memset(&cccd_md, 0, sizeof(cccd_md));
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
  cccd_md.vloc = BLE_GATTS_VLOC_STACK;

  ble_gatts_char_md_t char_md;
  memset(&char_md, 0, sizeof(char_md));
  char_md.char_props.read = 1;
  char_md.char_props.notify = 1;
  char_md.p_cccd_md = &cccd_md;

  ble_gatts_attr_t attr_char_value;
  memset(&attr_char_value, 0, sizeof(attr_char_value));
  attr_char_value.p_uuid = &char_uuid;
  attr_char_value.p_attr_md = &attr_md;
  attr_char_value.init_len = 4;
  attr_char_value.init_offs = 0;
  attr_char_value.max_len = mMAX_MTU_SIZE;
  uint8_t value[4] = {0x12, 0x34, 0x56, 0x78};
  attr_char_value.p_value = value;

  err_code = sd_ble_gatts_characteristic_add(mBaseServiceHandle,
      &char_md,
      &attr_char_value,
      &mOldResponseCharacteristicHandle);
  APP_ERROR_CHECK(err_code);
}

}
