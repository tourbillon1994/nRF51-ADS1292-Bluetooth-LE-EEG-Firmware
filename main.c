/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_app_template_main main.c
 * @{
 * @ingroup ble_sdk_app_template
 * @brief Template project main file.
 *
 * This file contains a template for creating a new application. It has the code necessary to wakeup
 * from button, advertise, get a connection restart advertising on disconnect and if no new
 * connection created go back to system-off mode.
 * It can easily be used as a starting point for creating a new application, the comments identified
 * with 'YOUR_JOB' indicates where and how you can customize.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "boards.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "device_manager.h"
#include "pstorage.h"
#include "app_trace.h"
#include "sensorsim.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_dis.h"
#include "ble_bms.h"
#include "app_util_platform.h"
#include "nrf_log.h"
#include "nrf_drv_clock.h"
#include "nrf_delay.h"
/**@ADS1291: **/
#include "ads1291-2.h" /*< For the ADS1291 ECG Chip */
#include "nrf_drv_gpiote.h"
#include "nrf_gpio.h"
/**@MPU: **/
#include "ble_mpu.h"
#if (defined(MPU9150) || defined(MPU9255) || defined(MPU60x0)) 
#include "mpu.h"
#endif

//#include "bsp.h"
//#include "bsp_btn_ble.h"

/**@TODO: DFU Support: */
#ifdef BLE_DFU_APP_SUPPORT
#include "ble_dfu.h"
#include "dfu_app_handler.h"
#include "nrf_delay.h"
#endif // BLE_DFU_APP_SUPPORT

#define IS_SRVC_CHANGED_CHARACT_PRESENT  1                                          /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/
#define CENTRAL_LINK_COUNT               0                                          /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT            1                                          /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/
			/**@DEVICE INFO*/
#define DEVICE_NAME                      "EEG SSVEP"                      	    /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                "VCU-YEO-LAB"                      				/**< Manufacturer. Will be passed to Device Information Service. */
#define DEVICE_MODEL_NUMBERSTR					 "Version 1.0"
#define DEVICE_FIRMWARE_STRING					 "Version 1.0"
			/**@ADVERTISING INITIALIZATION: */
#define APP_ADV_INTERVAL                 300                                        /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS       180                                        /**< The advertising timeout in units of seconds. */
			/**@TIMER DETAILS:*/
#define APP_TIMER_PRESCALER              0                                          /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE          4                                          /**< Size of timer operation queues. */
			/**@GAP INITIALIZATION:*/
#define MIN_CONN_INTERVAL                MSEC_TO_UNITS(33, UNIT_1_25_MS)//41.25        /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL                MSEC_TO_UNITS(34, UNIT_1_25_MS)//42.5        /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                    0                                          /**< Slave latency. */
#define CONN_SUP_TIMEOUT                 MSEC_TO_UNITS(4000, UNIT_10_MS)            /**< Connection supervisory timeout (4 seconds). */
			/**@CONNPARAMS MODULE:*/
#define FIRST_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY    APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)/**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT     3                                          /**< Number of attempts before giving up the connection parameter negotiation. */
			/**@DEVICEMANAGER_INIT Definitions: */
#define SEC_PARAM_BOND                   1                                          /**< Perform bonding. */
#define SEC_PARAM_MITM                   0                                          /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                   0                                          /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS               0                                          /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES        BLE_GAP_IO_CAPS_NONE                       /**< No I/O capabilities. */
#define SEC_PARAM_OOB                    0                                          /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE           7                                          /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE           16                                         /**< Maximum encryption key size. */
			/**@ERROR MACRO:*/
#define DEAD_BEEF                        0xDEADBEEF                                 /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
			/**@BLE HANDLES*/
static dm_application_instance_t         m_app_handle;                              /**< Application identifier allocated by device manager */
static uint16_t                          m_conn_handle = BLE_CONN_HANDLE_INVALID;   /**< Handle of the current connection. */
/**@BMS STUFF */
ble_bms_t 															 m_bms;
/**@MPU STUFF */
ble_mpu_t 															 m_mpu;

#if (defined(MPU60x0) || defined(MPU9150) || defined(MPU9255))
		static const nrf_drv_twi_t m_twi_instance = NRF_DRV_TWI_INSTANCE(1);
		static bool 														 mpu_read_send_flag = false;
#endif /**@(defined(MPU60x0) || defined(MPU9150) || defined(MPU9255))*/

#if defined(ADS1291)
//#define TX_RX_MSG_LENGTH 								7
//static uint8_t 													m_tx_data_spi[TX_RX_MSG_LENGTH]; /**< SPI master TX buffer. */
//static uint8_t 													m_rx_data_spi[TX_RX_MSG_LENGTH]; /**< SPI master RX buffer. */
#endif

/**@GPIOTE */
#if (defined(ADS1291) || defined(ADS1292) || defined(ADS1292R))
static bool															m_drdy = false;
static bool															m_send_eeg = false;
#define DRDY_GPIO_PIN_IN 11
#endif //(defined(ADS1291) || defined(ADS1292) || defined(ADS1292R))
/**@TIMER: -Timer Stuff- */
APP_TIMER_DEF(m_bms_send_timer_id);
APP_TIMER_DEF(m_mpu_send_timer_id);
#define TIMER_INTERVAL_UPDATE    		 		APP_TIMER_TICKS(12, APP_TIMER_PRESCALER)//25Hz*10dataPoints //40
#define MPU_TIMER_INTERVAL							APP_TIMER_TICKS(70, APP_TIMER_PRESCALER)//70msInterval
/**@DFU Support: */

/**@DFU Support(2): */

/**@Services declared under ble_###*/
/*static ble_uuid_t m_adv_uuids[] = 
{
		{BLE_UUID_BIOPOTENTIAL_MEASUREMENT_SERVICE, BLE_UUID_TYPE_BLE},
		{BLE_UUID_MPU_SERVICE_UUID, 								BLE_UUID_TYPE_BLE},
		{BLE_UUID_DEVICE_INFORMATION_SERVICE, 			BLE_UUID_TYPE_BLE}
};*/ /**< Universally unique service identifiers. */
static ble_uuid_t m_adv_uuids[] = {
		{BLE_UUID_BIOPOTENTIAL_MEASUREMENT_SERVICE, BLE_UUID_TYPE_BLE},
		{BLE_UUID_DEVICE_INFORMATION_SERVICE, 			BLE_UUID_TYPE_BLE}
};
                                   
/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

static void timer_send_timeout_handler(void *p_context){
		UNUSED_PARAMETER(p_context);
		#if (defined(ADS1291) || defined(ADS1292) || defined(ADS1292R))
		//ble_bms_send(&m_bms);
		//ble_eeg_fp1_send(&m_bms);
		m_send_eeg = true;
		#endif //(defined(ADS1291) || defined(ADS1292) || defined(ADS1292R))
}

static void mpu_send_timeout_handler(void *p_context) {
	#if (defined(MPU60x0) || defined(MPU9150) || defined(MPU9255))
			//DEPENDSS ON SAMPLING RATE
			mpu_read_send_flag = true;	
	#endif /**@(defined(MPU60x0) || defined(MPU9150) || defined(MPU9255))*/
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
    // Create timers.
    uint32_t err_code;
    err_code = app_timer_create(&m_bms_send_timer_id, APP_TIMER_MODE_REPEATED, timer_send_timeout_handler);
    APP_ERROR_CHECK(err_code);
		//#if (defined(MPU60x0) || defined(MPU9150) || defined(MPU9255))
		err_code = app_timer_create(&m_mpu_send_timer_id, APP_TIMER_MODE_REPEATED, mpu_send_timeout_handler);
		APP_ERROR_CHECK(err_code);
		//#endif /**@(defined(MPU60x0) || defined(MPU9150) || defined(MPU9255))*/
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the YYY Service events. 
 * YOUR_JOB implement a service handler function depending on the event the service you are using can generate
 *
 * @details This function will be called for all YY Service events which are passed to
 *          the application.
 *
 * @param[in]   p_yy_service   YY Service structure.
 * @param[in]   p_evt          Event received from the YY Service.
 *
 *
static void on_yys_evt(ble_yy_service_t     * p_yy_service, 
                       ble_yy_service_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_YY_NAME_EVT_WRITE:
            APPL_LOG("[APPL]: charact written with value %s. \r\n", p_evt->params.char_xx.value.p_str);
            break;
        
        default:
            // No implementation needed.
            break;
    }
}*/

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    ble_ecg_service_init(&m_bms);
		//ble_mpu_service_init(&m_mpu);
		/**@Device Information Service:*/
		uint32_t err_code;
		ble_dis_init_t dis_init;
		
		memset(&dis_init, 0, sizeof(dis_init));
		ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)MANUFACTURER_NAME);
		ble_srv_ascii_to_utf8(&dis_init.model_num_str, (char *)DEVICE_MODEL_NUMBERSTR);
		ble_srv_ascii_to_utf8(&dis_init.fw_rev_str, (char *)DEVICE_FIRMWARE_STRING);
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);
		err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting timers.
*/
static void application_timers_start(void)
{
    /* YOUR_JOB: Start your timers. below is an example of how to start a timer.*/
    //uint32_t err_code;
    //err_code = app_timer_start(m_bms_record_timer_id, TIMER_INTERVAL_UPDATE, NULL);
    //APP_ERROR_CHECK(err_code); 
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
		uint32_t err_code;
    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    //uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id) {
        case BLE_GAP_EVT_CONNECTED:
						ads1291_2_wake();
            
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
						err_code = app_timer_start(m_bms_send_timer_id, TIMER_INTERVAL_UPDATE, NULL);
						APP_ERROR_CHECK(err_code);
						err_code = app_timer_start(m_mpu_send_timer_id, MPU_TIMER_INTERVAL, NULL);
						APP_ERROR_CHECK(err_code);
						NRF_LOG_PRINTF("App timer start: m_bms_record_timer_id..\r\n");
            break;

        case BLE_GAP_EVT_DISCONNECTED:
						ads1291_2_standby();
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
						app_timer_stop(m_bms_send_timer_id);
						app_timer_stop(m_mpu_send_timer_id);
						NRF_LOG_PRINTF("App timer stop: m_bms_record_timer_id..\r\n");
            break;
        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    dm_ble_evt_handler(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
		ble_bms_on_ble_evt(&m_bms, p_ble_evt);
		//ble_mpu_on_ble_evt(&m_mpu, p_ble_evt);

}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in] sys_evt  System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
    ble_advertising_on_sys_evt(sys_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    /** MAY NEED TO CHANGE THIS */
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
	
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);
    
    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);
    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);
    
    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Device Manager events.
 *
 * @param[in] p_evt  Data associated to the device manager event.
 */
static uint32_t device_manager_evt_handler(dm_handle_t const * p_handle,
                                           dm_event_t const  * p_event,
                                           ret_code_t        event_result)
{
    APP_ERROR_CHECK(event_result);
    return NRF_SUCCESS;
}


/**@brief Function for the Device Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Device Manager.
 */
static void device_manager_init(bool erase_bonds)
{
    uint32_t               err_code;
    dm_init_param_t        init_param = {.clear_persistent_data = erase_bonds};
    dm_application_param_t register_param;

    // Initialize persistent storage module.
    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

    err_code = dm_init(&init_param);
    APP_ERROR_CHECK(err_code);

    memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));

    register_param.sec_param.bond         = SEC_PARAM_BOND;
    register_param.sec_param.mitm         = SEC_PARAM_MITM;
    register_param.sec_param.lesc         = SEC_PARAM_LESC;
    register_param.sec_param.keypress     = SEC_PARAM_KEYPRESS;
    register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    register_param.sec_param.oob          = SEC_PARAM_OOB;
    register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    register_param.evt_handler            = device_manager_evt_handler;
    register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

    err_code = dm_register(&m_app_handle, &register_param);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = m_adv_uuids;

    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}
#if (defined(MPU60x0) || defined(MPU9150) || defined(MPU9255))
/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{   
	switch(p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            // If EVT_DONE (event done) is received a device is found and responding on that particular address
            //NRF_LOG_PRINTF("\r\n!****************************!\r\nDevice found at 7-bit address: %#x!\r\n!****************************!\r\n\r\n");
						//device_found = true;
            break;
        case NRF_DRV_TWI_EVT_ADDRESS_NACK:
            //NRF_LOG_PRINTF("No address ACK on address: %#x!\r\n");
            break;
        case NRF_DRV_TWI_EVT_DATA_NACK:
            //NRF_LOG_PRINTF("No data ACK on address: %#x!\r\n");
            break;
        default:
            break;        
    }   
    // Pass TWI events down to the MPU driver.
    mpu_twi_event_handler(p_event);
		UNUSED_PARAMETER(p_context);
}

/**
 * @brief TWI initialization.
 * Nothing special here
 */
void twi_setup(void)
{
    ret_code_t err_code;
    
    /**/const nrf_drv_twi_config_t twi_mpu_config = {
       .scl                = MPU_TWI_SCL_PIN,//MPU_TWI_SCL_PIN,
       .sda                = MPU_TWI_SDA_PIN,//MPU_TWI_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH
    };
		//nrf_drv_twi_config_t twi_mpu_config = NRF_DRV_TWI_DEFAULT_CONFIG(1);
    
    err_code = nrf_drv_twi_init(&m_twi_instance, &twi_mpu_config, twi_handler, NULL);
		NRF_LOG_PRINTF("ERRCODE: nrf_drv_twi_init: %d \r\n", err_code);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_PRINTF(" TWI (I2C) Enabled! \r\n");
    nrf_drv_twi_enable(&m_twi_instance);
}

void mpu_setup(void)
{
    ret_code_t ret_code;
    // Initiate MPU driver with TWI instance handler
    ret_code = mpu_init(&m_twi_instance);
		NRF_LOG_PRINTF("ERRCODE: mpu_init: %d \r\n", ret_code);
    //APP_ERROR_CHECK(ret_code); // Check for errors in return value
    
    // Setup and configure the MPU with intial values
    mpu_config_t p_mpu_config = MPU_DEFAULT_CONFIG(); // Load default values
    p_mpu_config.smplrt_div = 19;   // Change sampelrate. Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV). 19 gives a sample rate of 50Hz
    p_mpu_config.accel_config.afs_sel = AFS_2G; // Set accelerometer full scale range to 2G
    ret_code = mpu_config(&p_mpu_config); // Configure the MPU with above values
	  NRF_LOG_PRINTF("ERRCODE: mpu_config: %d \r\n", ret_code);
    //APP_ERROR_CHECK(ret_code); // Check for errors in return value 
}

#endif /**@(defined(MPU60x0) || defined(MPU9150) || defined(MPU9255))*/

/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}
#if (defined(ADS1291) || defined(ADS1292) || defined(ADS1292R))
void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
		UNUSED_PARAMETER(pin);
		UNUSED_PARAMETER(action);
    m_drdy = true;
}
#endif //(defined(ADS1291) || defined(ADS1292) || defined(ADS1292R))

/**@OLD GPIO INIT (ALSO WORKS FINE!)*/
/*static void gpio_init(void) {
		nrf_gpio_pin_dir_set(ADS1291_2_DRDY_PIN, NRF_GPIO_PIN_DIR_INPUT);
		nrf_gpio_pin_dir_set(ADS1291_2_PWDN_PIN, NRF_GPIO_PIN_DIR_OUTPUT);
		ret_code_t err_code;
		err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);
		//Send data when drdy is low? (see datasheet).
		bool is_high_accuracy = true;
		nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(is_high_accuracy);
		//nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(is_high_accuracy);
		in_config.is_watcher = false;
		in_config.pull = NRF_GPIO_PIN_NOPULL;
		err_code = nrf_drv_gpiote_in_init(DRDY_GPIO_PIN_IN, &in_config, in_pin_handler);
    APP_ERROR_CHECK(err_code);
		nrf_drv_gpiote_in_event_enable(DRDY_GPIO_PIN_IN, true);
}*/
#if (defined(ADS1291) || defined(ADS1292) || defined(ADS1292R))
static void gpio_init(void) {
		nrf_gpio_pin_dir_set(ADS1291_2_DRDY_PIN, NRF_GPIO_PIN_DIR_INPUT); //sets 'direction' = input/output
		nrf_gpio_pin_dir_set(ADS1291_2_PWDN_PIN, NRF_GPIO_PIN_DIR_OUTPUT);
		uint32_t err_code;
		if(!nrf_drv_gpiote_is_init())
		{
				err_code = nrf_drv_gpiote_init();
		}
		NRF_LOG_PRINTF("nrf_drv_gpiote_init: %d\r\n",err_code);
    APP_ERROR_CHECK(err_code);/**/
		bool is_high_accuracy = true;
		nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(is_high_accuracy);
		in_config.is_watcher = true;
		in_config.pull = NRF_GPIO_PIN_NOPULL;
		err_code = nrf_drv_gpiote_in_init(DRDY_GPIO_PIN_IN, &in_config, in_pin_handler);
		NRF_LOG_PRINTF(" nrf_drv_gpiote_in_init: %d: \r\n",err_code);
		APP_ERROR_CHECK(err_code);
		nrf_drv_gpiote_in_event_enable(DRDY_GPIO_PIN_IN, true);
		ads1291_2_powerdn();
}
#endif //(defined(ADS1291) || defined(ADS1292) || defined(ADS1292R))

/**@brief Function for application main entry.
 */
int main(void)
{
		NRF_LOG_PRINTF(" BLE ECG WITH MPU - START..\r\n");
    uint32_t err_code;
    bool erase_bonds;
    // Initialize.
    timers_init();
    ble_stack_init();
		err_code = nrf_drv_clock_init();
		NRF_LOG_PRINTF("ERRCODE: DRV CLOCK: %d \r\n", err_code);
		APP_ERROR_CHECK(err_code);
		#if (defined(ADS1291) || defined(ADS1292) || defined(ADS1292R))
		gpio_init();
		#endif //(defined(ADS1291) || defined(ADS1292) || defined(ADS1292R))
    device_manager_init(erase_bonds);
    gap_params_init();
    advertising_init();
    services_init();
    conn_params_init();

		//SPI STUFF FOR ADS:.
		#if (defined(ADS1291) || defined(ADS1292) || defined(ADS1292R))
		ads1291_2_powerup();
		ads_spi_init();		
		
		//init_buf(m_tx_data_spi, m_rx_data_spi, TX_RX_MSG_LENGTH);
		// Stop continuous data conversion and initialize registers to default values
		ads1291_2_stop_rdatac();
		ads1291_2_init_regs();
					
		ads1291_2_soft_start_conversion();
			ads1291_2_check_id();
		ads1291_2_start_rdatac();
			
		// Put AFE to sleep while we're not connected
		ads1291_2_standby();
		
		//body_voltage_t fp1;
		//body_voltage_t fp2;
		#endif //(defined(ADS1291) || defined(ADS1292) || defined(ADS1292R))
					
		#if (defined(MPU60x0) || defined(MPU9150) || defined(MPU9255))
		/**@MPU STUFF:*/
			twi_setup();
			//device_address = 0x68;
			uint8_t dummy_data = 0x55;
		nrf_drv_twi_tx(&m_twi_instance, 0x68, &dummy_data, 1, false);
		nrf_delay_ms(12);
			mpu_setup();
			accel_values_t accel_values;
			gyro_values_t   gyro_values;
			magn_values_t   magn_values;
			temp_value_t    temperature;
		#endif /**@(defined(MPU60x0) || defined(MPU9150) || defined(MPU9255))*/
		
    // Start execution.
    application_timers_start();
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
		NRF_LOG_PRINTF(" BLE Advertising Start! \r\n");
		/**@TEMPORARY:**/
			//ads1291_2_wake();
		// Enter main loop.
		uint8_t cnt = 0;
		eeg_values_t fp1_struct;
		eeg_values_t fp2_struct;
		combined_eeg fp1_combined;
		combined_eeg fp2_combined;
    for (;;)
    {
				/** TODO @SEND MULTIPLE PACKETS PER INTERVAL
					*	@POTATO @SANDWICH
					*
					*
					**/
				#if (defined(ADS1291) || defined(ADS1292) || defined(ADS1292R))
				/**/
				if(m_drdy) {
						m_drdy = false;
						get_eeg_sample(&fp1_struct, &fp2_struct);
						switch(cnt) {
							case 0:
								fp1_combined.a = (eeg_values_t1){fp1_struct.a, fp1_struct.b, fp1_struct.c};
								fp2_combined.a = (eeg_values_t1){fp2_struct.a, fp2_struct.b, fp2_struct.c};
							//NRF_LOG_PRINTF("++fp1[%d]: 0x%x, %d \r\n",cnt,fp1_combined.a,fp1_combined.a);
								cnt++;
								break;
							case 1:
								fp1_combined.b = (eeg_values_t1){fp1_struct.a, fp1_struct.b, fp1_struct.c};
								fp2_combined.b = (eeg_values_t1){fp2_struct.a, fp2_struct.b, fp2_struct.c};
							//NRF_LOG_PRINTF("++fp1[%d]: 0x%x, %d \r\n",cnt,fp1_combined.b,fp1_combined.b);
								cnt++;
								break;
							case 2:
								fp1_combined.c = (eeg_values_t1){fp1_struct.a, fp1_struct.b, fp1_struct.c};
								fp2_combined.c = (eeg_values_t1){fp2_struct.a, fp2_struct.b, fp2_struct.c};
							//NRF_LOG_PRINTF("++fp1[%d]: 0x%x, %d \r\n",cnt,fp1_combined.c,fp1_combined.c);
								cnt++;
								break;
							case 3:
								fp1_combined.d = (eeg_values_t1){fp1_struct.a, fp1_struct.b, fp1_struct.c};
								fp2_combined.d = (eeg_values_t1){fp2_struct.a, fp2_struct.b, fp2_struct.c};
								cnt++;
								break;
							case 4:
								fp1_combined.e = (eeg_values_t1){fp1_struct.a, fp1_struct.b, fp1_struct.c};
								fp2_combined.e = (eeg_values_t1){fp2_struct.a, fp2_struct.b, fp2_struct.c};
								cnt++;
								break;
							case 5:
								fp1_combined.f = (eeg_values_t1){fp1_struct.a, fp1_struct.b, fp1_struct.c};
								fp2_combined.f = (eeg_values_t1){fp2_struct.a, fp2_struct.b, fp2_struct.c};
								cnt++;
								cnt=0;
								break;
							default:
								NRF_LOG_PRINTF("CATASTROPHIC ERROR! (SWITCH) \r\n");
								break;
						}
				}
				if(m_send_eeg) {
					m_send_eeg = false;
					ble_eeg_fp1_update(&m_bms, &fp1_combined);
					ble_eeg_fp2_update(&m_bms, &fp2_combined);
					NRF_LOG_PRINTF("++SEND_FP1: 0x%x%x%x\r\n\r\n",fp1_combined.a, fp1_combined.b, fp1_combined.c);
					NRF_LOG_PRINTF("++SEND_FP2: 0x%x%x%x\r\n\r\n",fp2_combined.a, fp2_combined.b, fp2_combined.c);
				}
				
				/*if(m_drdy) {
					m_drdy=false;
					get_eeg_sample(&fp1_struct, &fp2_struct);
					eeg_values_t1 eeg_fp1 = {fp1_struct.a, fp1_struct.b, fp1_struct.c};
					NRF_LOG_PRINTF("++fp1[]: 0x%x, %d \r\n",eeg_fp1,eeg_fp1);
					ble_eeg_fp1_update(&m_bms, &eeg_fp1);
				}*/
				#endif //(defined(ADS1291) || defined(ADS1292) || defined(ADS1292R))
				//Send all values (50ms), handled by timer
				#if (defined(MPU60x0) || defined(MPU9150) || defined(MPU9255))
				if(mpu_read_send_flag) {
						//read & send values via thing.
						mpu_read_accel(&accel_values);
						mpu_read_gyro(&gyro_values);
						mpu_read_magn(&magn_values);
						mpu_read_temp(&temperature);
						combined_values_t combined_values = {
							accel_values.x, accel_values.y, accel_values.z,
							gyro_values.x, gyro_values.y, gyro_values.z
						};
            ble_mpu_accel_update(&m_mpu, &accel_values);
						ble_mpu_gyro_update(&m_mpu, &gyro_values);
						ble_mpu_magnt_update(&m_mpu, &magn_values);
						ble_mpu_temp_update(&m_mpu, &temperature);
						ble_mpu_combined_update(&m_mpu, &combined_values);
						mpu_read_send_flag = false;
				}
				#endif /**@(defined(MPU60x0) || defined(MPU9150) || defined(MPU9255))*/
				power_manage();
    }
}

/**
 * @}
 */
