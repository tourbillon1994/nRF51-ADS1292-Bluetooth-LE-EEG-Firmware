/* Copyright (c) 2016 Musa Mahmood
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "ble_bms.h"
#include <string.h>
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "app_error.h"
#include "ads1291-2.h"

#define MAX_BVM_LENGTH   		20//60																								 /**< Maximum size in bytes of a transmitted Body Voltage Measurement. */
#define MAX_BVM_LENGTH_T			18
void ble_bms_on_ble_evt(ble_bms_t * p_bms, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
						p_bms->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
						p_bms->conn_handle = BLE_CONN_HANDLE_INVALID;
            break;
        default:
            break;
    }
}

/**@brief Function for encoding int16 Body Voltage Measurement buffer to a byte array.
 *
 * @param[in]   p_bms              Biopotential Measurement Service structure.
 * @param[in]   body_voltage       Measurement to be encoded.
 * @param[out]  p_encoded_buffer   Buffer where the encoded data will be written.
 *
 * @return      Size of encoded data.
 */ 
static uint8_t bvm_encode(ble_bms_t * p_bms, uint8_t * p_encoded_buffer)
{
    uint8_t len   = 0;
    int     i;

    // Encode body voltage measurement
    for (i = 0; i < p_bms->bvm_count; i++)
    {			
        if (len + sizeof(uint16_t) > MAX_BVM_LENGTH)
        {
            // Not all stored voltage values can fit into the packet, so
            // move the remaining values to the start of the buffer.
            memmove(&p_bms->bvm_buffer[0],
                    &p_bms->bvm_buffer[i],
                    (p_bms->bvm_count - i) * sizeof(uint16_t));
            break;
        }
        len += uint16_encode(p_bms->bvm_buffer[i], &p_encoded_buffer[len]);
    }
    p_bms->bvm_count -= i;
    return len;
}

/**@brief Function for adding the Body Voltage Measurement characteristic.
 *
 * @param[in]   p_bms        Biopotential Measurement Service structure.
 * @param[in]   p_bms_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
 /**/
static uint32_t fp1_voltage_measurement_char_add(ble_bms_t * p_bms)
{
		uint32_t err_code = 0;
		ble_uuid_t	 						char_uuid;
		uint8_t             encoded_initial_bvm[MAX_BVM_LENGTH];
		BLE_UUID_BLE_ASSIGN(char_uuid, BLE_UUID_EEG_FP1_CHAR);
	
		ble_gatts_char_md_t char_md;
	
		memset(&char_md, 0, sizeof(char_md));
		char_md.char_props.read = 1;
		char_md.char_props.write = 0;
		
		ble_gatts_attr_md_t cccd_md;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc                = BLE_GATTS_VLOC_STACK;    
    char_md.p_cccd_md           = &cccd_md;
    char_md.char_props.notify   = 1;
		ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md));
    attr_md.vloc = BLE_GATTS_VLOC_STACK;    
    attr_md.vlen = 1;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
		
		ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid      = &char_uuid;
    attr_char_value.p_attr_md   = &attr_md;
		attr_char_value.init_len		= bvm_encode(p_bms, encoded_initial_bvm);
		//attr_char_value.init_len		= bvm_encode2(p_bms, encoded_initial_bvm);
		attr_char_value.init_offs		= 0;
		attr_char_value.max_len			= MAX_BVM_LENGTH;
		attr_char_value.p_value   	= encoded_initial_bvm;
		err_code = sd_ble_gatts_characteristic_add(p_bms->service_handle,
																							&char_md,
																							&attr_char_value,
																							&p_bms->fp1_handles);
    APP_ERROR_CHECK(err_code);   

    return NRF_SUCCESS;
		//SET UP LIKE IN MPU EXAMPLE
}

static uint32_t fp2_voltage_measurement_char_add(ble_bms_t *p_bms) {
	uint32_t err_code = 0;
		ble_uuid_t	 						char_uuid;
		uint8_t             encoded_initial_bvm[MAX_BVM_LENGTH];
		BLE_UUID_BLE_ASSIGN(char_uuid, BLE_UUID_EEG_FP2_CHAR);
	
		ble_gatts_char_md_t char_md;
	
		memset(&char_md, 0, sizeof(char_md));
		char_md.char_props.read = 1;
		char_md.char_props.write = 0;
		
		ble_gatts_attr_md_t cccd_md;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc                = BLE_GATTS_VLOC_STACK;    
    char_md.p_cccd_md           = &cccd_md;
    char_md.char_props.notify   = 1;
		ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md));
    attr_md.vloc = BLE_GATTS_VLOC_STACK;    
    attr_md.vlen = 1;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
		
		ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid      = &char_uuid;
    attr_char_value.p_attr_md   = &attr_md;
		attr_char_value.init_len		= bvm_encode(p_bms, encoded_initial_bvm);
		attr_char_value.init_offs		= 0;
		attr_char_value.max_len			= MAX_BVM_LENGTH;
		attr_char_value.p_value   	= encoded_initial_bvm;
		err_code = sd_ble_gatts_characteristic_add(p_bms->service_handle,
																							&char_md,
																							&attr_char_value,
																							&p_bms->fp2_handles);
    APP_ERROR_CHECK(err_code);   

    return NRF_SUCCESS;
}

/**@brief Function for initiating our new service.
 *
 * @param[in]   p_mpu        Our Service structure.
 *
 */
void ble_ecg_service_init(ble_bms_t *p_bms) {
		uint32_t   err_code; // Variable to hold return codes from library and softdevice functions

    ble_uuid_t        service_uuid;
    ble_uuid128_t     base_uuid = BMS_UUID_BASE;
    service_uuid.uuid = BLE_UUID_BIOPOTENTIAL_MEASUREMENT_SERVICE;
    err_code = sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type);
    APP_ERROR_CHECK(err_code);    

    p_bms->conn_handle = BLE_CONN_HANDLE_INVALID;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &service_uuid,
                                        &p_bms->service_handle);
    
    APP_ERROR_CHECK(err_code);
		/*ADD CHARACTERISTIC(S)*/
		fp1_voltage_measurement_char_add(p_bms);
		fp2_voltage_measurement_char_add(p_bms);
}
#if (defined(ADS1291) || defined(ADS1292) || defined(ADS1292R))
/**@Update adds single int16_t voltage value: */
/**@FP1 */
void ble_eeg_fp1_send (ble_bms_t *p_bms) {
	if(p_bms->conn_handle!=BLE_CONN_HANDLE_INVALID) {
			uint8_t	encoded_bvm[MAX_BVM_LENGTH_T];
			uint16_t len = sizeof(combined_eeg);//sizeof(combined_fp1);
			uint16_t hvx_len = len;
			ble_gatts_hvx_params_t hvxp;
			memset(&hvxp,0,sizeof(hvxp));
			hvxp.handle = p_bms->fp1_handles.value_handle;
			hvxp.type		= BLE_GATT_HVX_NOTIFICATION;
			hvxp.offset = 0;
			hvxp.p_len 	= &hvx_len;
			hvxp.p_data = encoded_bvm;
			sd_ble_gatts_hvx(p_bms->conn_handle, &hvxp);
		}
}

void ble_eeg_fp1_update(ble_bms_t *p_bms, combined_eeg *fp1) {
		ble_gatts_value_t gatts_value;
		// Initialize value struct.
		memset(&gatts_value, 0, sizeof(gatts_value));
		gatts_value.len = (sizeof(combined_eeg));
		gatts_value.offset = 0; //0???
		gatts_value.p_value = (uint8_t*)(fp1);
		sd_ble_gatts_value_set(p_bms->conn_handle, p_bms->fp1_handles.value_handle, &gatts_value);
		if(p_bms->conn_handle!=BLE_CONN_HANDLE_INVALID) {
			uint16_t len = sizeof(combined_eeg);
			uint16_t hvx_len = len;
			ble_gatts_hvx_params_t hvxp;
			memset(&hvxp,0,sizeof(hvxp));
			hvxp.handle = p_bms->fp1_handles.value_handle;
			hvxp.type		= BLE_GATT_HVX_NOTIFICATION;
			hvxp.offset = 0;
			hvxp.p_len 	= &hvx_len;
			hvxp.p_data = (uint8_t*)fp1;
			sd_ble_gatts_hvx(p_bms->conn_handle, &hvxp);
		}
}
/**@FP2 */
void ble_eeg_fp2_send (ble_bms_t *p_bms) {
	if(p_bms->conn_handle!=BLE_CONN_HANDLE_INVALID) {
			uint8_t	encoded_bvm[MAX_BVM_LENGTH_T];
			uint16_t len = sizeof(combined_eeg);//bvm_encode2(p_bms, encoded_bvm);
			uint16_t hvx_len = len;
			ble_gatts_hvx_params_t hvxp;
			memset(&hvxp,0,sizeof(hvxp));
			hvxp.handle = p_bms->fp2_handles.value_handle;
			hvxp.type		= BLE_GATT_HVX_NOTIFICATION;
			hvxp.offset = 0;
			hvxp.p_len 	= &hvx_len;
			hvxp.p_data = encoded_bvm;
			sd_ble_gatts_hvx(p_bms->conn_handle, &hvxp);
		}
}

void ble_eeg_fp2_update(ble_bms_t *p_bms, combined_eeg *fp2) {
		ble_gatts_value_t gatts_value;
		// Initialize value struct.
		memset(&gatts_value, 0, sizeof(gatts_value));
		gatts_value.len = (sizeof(combined_eeg));
		gatts_value.offset = 0; //0???
		gatts_value.p_value = (uint8_t*)(fp2);
		sd_ble_gatts_value_set(p_bms->conn_handle, p_bms->fp2_handles.value_handle, &gatts_value);
		if(p_bms->conn_handle!=BLE_CONN_HANDLE_INVALID) {
			uint16_t len = sizeof(combined_eeg);
			uint16_t hvx_len = len;
			ble_gatts_hvx_params_t hvxp;
			memset(&hvxp,0,sizeof(hvxp));
			hvxp.handle = p_bms->fp2_handles.value_handle;
			hvxp.type		= BLE_GATT_HVX_NOTIFICATION;
			hvxp.offset = 0;
			hvxp.p_len 	= &hvx_len;
			hvxp.p_data = (uint8_t*)fp2;
			sd_ble_gatts_hvx(p_bms->conn_handle, &hvxp);
		}
}
#endif// (defined(ADS1291) || defined(ADS1292) || defined(ADS1292R))
