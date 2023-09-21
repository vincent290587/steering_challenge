#include "ble_cus.h"
#include "ble_srv_common.h"
#include "boards.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "sdk_common.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <limits.h>

/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_cus       Custom Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_cus_t *p_cus, ble_evt_t const *p_ble_evt) {

	ble_cus_evt_t evt;

	if (p_cus && p_cus->evt_handler) {

		p_cus->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

		evt.evt_type = BLE_CUS_EVT_CONNECTED;
		p_cus->evt_handler(p_cus, &evt);
	} else {
		NRF_LOG_ERROR("CUS is not init !!");
	}

}

/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_cus       Custom Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_cus_t *p_cus, ble_evt_t const *p_ble_evt) {

	UNUSED_PARAMETER(p_ble_evt);

	ble_cus_evt_t evt;

	evt.evt_type = BLE_CUS_EVT_DISCONNECTED;

	if (p_cus && p_cus->evt_handler) {
		p_cus->conn_handle = BLE_CONN_HANDLE_INVALID;
		p_cus->evt_handler(p_cus, &evt);
	}
}

static uint8_t challengeRequest[] = {0x03, 0x10, 0x12, 0x34};
static uint8_t someOtherThing[] = {0x03, 0x11, 0xff, 0xff};

static uint32_t rotate_left32 (uint32_t value, uint32_t count) {
    const uint32_t mask = (CHAR_BIT * sizeof (value)) - 1;
    count &= mask;
    return (value << count) | (value >> (-count & mask));
}

static uint32_t hashed(uint64_t seed) {
    uint32_t ret = (seed + 0x16fa5717);
    uint64_t rax = seed * 0xba2e8ba3;
    uint64_t eax = (rax >> 35) * 0xb;
    uint64_t ecx = seed - eax;
    uint32_t edx = rotate_left32(seed, ecx & 0x0F);
    ret ^= edx;
    return ret;
}

/**@brief Function for handling the Write event.
 *
 * @param[in]   p_cus       Custom Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_cus_t *p_cus, ble_evt_t const *p_ble_evt) {

	ble_gatts_evt_write_t const *p_evt_write =
			&p_ble_evt->evt.gatts_evt.params.write;

	NRF_LOG_WARNING("CUS onWrite: len=%u handle=%u", p_evt_write->len, p_evt_write->handle);

	// Custom Value TX Characteristic Written to.
	if (p_evt_write->handle == p_cus->rx_handles.value_handle) {

		NRF_LOG_WARNING("CUS RX handle: len=%u", p_evt_write->len);
		NRF_LOG_HEXDUMP_WARNING(p_evt_write->data, p_evt_write->len);

		// if we got 0x0310
		if (p_evt_write->data[0] == 0x03 && p_evt_write->data[1] == 0x10) {
			// issue the challenge of 0x0310yyyy on 0x0032
			NRF_LOG_WARNING("Got request for challenge");
			ret_code_t ret = ble_cus_tx_value_update(p_cus, challengeRequest, sizeof(challengeRequest));
			APP_ERROR_CHECK(ret);
		}

		// if we got 0x0311
		if (p_evt_write->data[0] == 0x03 && p_evt_write->data[1] == 0x11) {
			// emit 0x0311ffff on 0x0032
			NRF_LOG_WARNING("Got thing2");
			ret_code_t ret = ble_cus_tx_value_update(p_cus, someOtherThing, sizeof(someOtherThing));
			APP_ERROR_CHECK(ret);
			// tell app to start firing off steering data
			ble_cus_evt_t evt;
			evt.evt_type = BLE_CUS_START_SENDING_STEERING_DATA;

			// Call the application event handler.
			if (p_cus->evt_handler) {
				p_cus->evt_handler(p_cus, &evt);
			}
		}

		if (p_evt_write->data[0] == 0x03 && p_evt_write->data[1] == 0x12) {

			uint32_t seed = p_evt_write->data[5] << 24;
			seed |= p_evt_write->data[4] << 16;
			seed |= p_evt_write->data[3] << 8;
			seed |= p_evt_write->data[2];
			uint32_t password = hashed(seed);

			NRF_LOG_WARNING("Got challenge %X, response: %lX -> %lX", p_evt_write->data[1], seed, password);

			uint8_t response[6];
			response[0] = 0x03;
			response[1] = p_evt_write->data[1];
			response[5] = (password & 0xFF000000 ) >> 24;
			response[4] = (password & 0x00FF0000 ) >> 16;
			response[3] = (password & 0x0000FF00 ) >> 8;
			response[2] = (password & 0x000000FF );
			ret_code_t ret = ble_cus_tx_value_update(p_cus, response, sizeof(response));
			APP_ERROR_CHECK(ret);

		} else if (p_evt_write->data[0] == 0x03 && p_evt_write->data[1] == 0x13) {

			NRF_LOG_WARNING("Got response %X", p_evt_write->data[1]);

			uint8_t response[3];
			response[0] = 0x03;
			response[1] = p_evt_write->data[1];
			response[2] = 0xFF;
			ret_code_t ret = ble_cus_tx_value_update(p_cus, response, sizeof(response));
			APP_ERROR_CHECK(ret);

			// tell app to start firing off steering data
			ble_cus_evt_t evt;
			evt.evt_type = BLE_CUS_START_SENDING_STEERING_DATA;

			// Call the application event handler.
			if (p_cus->evt_handler) {
				p_cus->evt_handler(p_cus, &evt);
			}
		}

	} else
	if (p_evt_write->handle == p_cus->r2_handles.value_handle) {

		NRF_LOG_WARNING("CUS R2 VAL: len=%u", p_evt_write->len);
	} else
	if (p_evt_write->handle == p_cus->r2_handles.cccd_handle) {

		NRF_LOG_WARNING("CUS R2 CCCD: len=%u", p_evt_write->len);
	} else
	if (p_evt_write->handle == p_cus->r1_handles.value_handle) {

		NRF_LOG_WARNING("CUS R1 VAL: len=%u", p_evt_write->len);
	} else
	if (p_evt_write->handle == p_cus->r1_handles.cccd_handle) {

		NRF_LOG_WARNING("CUS R1 CCCD: len=%u", p_evt_write->len);
	} else
	if (p_evt_write->handle == p_cus->tx_handles.value_handle) {

		NRF_LOG_WARNING("CUS TX VAL: len=%u", p_evt_write->len);
	} else
	if (p_evt_write->handle == p_cus->tx_handles.cccd_handle) {

		NRF_LOG_WARNING("CUS TX CCCD: len=%u", p_evt_write->len);
	} else
	if (p_evt_write->handle == p_cus->w1_handles.value_handle) {

		NRF_LOG_WARNING("CUS W1 VAL: len=%u", p_evt_write->len);
	} else if (p_evt_write->handle == p_cus->n1_handles.cccd_handle) {

		NRF_LOG_WARNING("CUS N1 CCCD: len=%u", p_evt_write->len);
	} else {

//		NRF_LOG_WARNING("CUS unknown handle: %u", p_evt_write->handle);
//		NRF_LOG_HEXDUMP_WARNING(p_evt_write->data, p_evt_write->len);
	}
}

void ble_cus_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context) {
	ble_cus_t *p_cus = (ble_cus_t *)p_context;

	NRF_LOG_DEBUG("BLE event received. Event type = %d",
			p_ble_evt->header.evt_id);
	if (p_cus == NULL || p_ble_evt == NULL) {
		return;
	}

	switch (p_ble_evt->header.evt_id) {
	case BLE_GAP_EVT_CONNECTED:
		on_connect(p_cus, p_ble_evt);
		break;

	case BLE_GAP_EVT_DISCONNECTED:
		on_disconnect(p_cus, p_ble_evt);
		break;

	case BLE_GATTS_EVT_WRITE:
		on_write(p_cus, p_ble_evt);
		break;

	case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
		NRF_LOG_INFO("BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST");
		break;

	default:
		// No implementation needed.
		break;
	}
}

/**@brief Function for adding the Custom Value characteristic.
 *
 * @param[in]   p_cus        Battery Service structure.
 * @param[in]   p_cus_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t custom_value_char_add(ble_cus_t *p_cus, uint16_t uuid,
		const ble_cus_init_t *p_cus_init, ble_gatts_char_handles_t *p_handles) {

	uint32_t err_code;
	ble_gatts_char_md_t char_md;
	ble_gatts_attr_md_t cccd_md;
	ble_gatts_attr_t attr_char_value;
	ble_uuid_t ble_uuid;
	ble_gatts_attr_md_t attr_md;

	// Add Custom Value characteristic
	memset(&cccd_md, 0, sizeof(cccd_md));

	//  Read  operation on cccd should be possible without authentication.
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

	cccd_md.write_perm = p_cus_init->custom_value_char_attr_md.cccd_write_perm;
	cccd_md.vloc = BLE_GATTS_VLOC_STACK;

	memset(&char_md, 0, sizeof(char_md));

	char_md.char_props.notify = 1;
	//char_md.char_props.read = 1;
	char_md.p_char_user_desc = NULL;
	char_md.p_char_pf = NULL;
	char_md.p_user_desc_md = NULL;
	char_md.p_cccd_md = &cccd_md;
	char_md.p_sccd_md = NULL;

	ble_uuid.type = p_cus->uuid_type;
	ble_uuid.uuid = uuid;

	memset(&attr_md, 0, sizeof(attr_md));

	attr_md.read_perm = p_cus_init->custom_value_char_attr_md.read_perm;
	attr_md.write_perm = p_cus_init->custom_value_char_attr_md.write_perm;
	attr_md.vloc = BLE_GATTS_VLOC_USER;
	attr_md.rd_auth = 0;
	attr_md.wr_auth = 0;
	attr_md.vlen = 0;

	memset(&attr_char_value, 0, sizeof(attr_char_value));

	attr_char_value.p_uuid = &ble_uuid;
	attr_char_value.p_attr_md = &attr_md;
	attr_char_value.init_offs = 0;

	if (uuid == N1_CHAR_UUID) {

		static uint8_t pValue[1] = { 0xFE };
		attr_char_value.init_len = sizeof(pValue);
		attr_char_value.max_len = sizeof(pValue);
		attr_char_value.p_value = pValue;
	} else if (uuid == STEERER_CHAR_UUID) {

		static float pValue[1] = { 20.f };
		attr_char_value.init_len = sizeof(float);
		attr_char_value.max_len = sizeof(float);
		attr_char_value.p_value = (uint8_t*)pValue;
	} else {
		NRF_LOG_ERROR("Unknown UUID init. ?");

		static float pValue[1] = { 20.f };
		attr_char_value.init_len = sizeof(float);
		attr_char_value.max_len = sizeof(float);
		attr_char_value.p_value = (uint8_t*)pValue;
	}

	err_code = sd_ble_gatts_characteristic_add(p_cus->service_handle, &char_md,
			&attr_char_value,
			p_handles);
	if (err_code != NRF_SUCCESS) {
		return err_code;
	}

	return NRF_SUCCESS;
}

/**@brief Function for adding the Custom Value characteristic.
 *
 * @param[in]   p_cus        Battery Service structure.
 * @param[in]   p_cus_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t tx_char_add(ble_cus_t *p_cus, uint16_t uuid,
		const ble_cus_init_t *p_cus_init, ble_gatts_char_handles_t *p_handles) {
	uint32_t err_code;
	ble_gatts_char_md_t char_md;
	ble_gatts_attr_md_t cccd_md;
	ble_gatts_attr_t attr_char_value;
	ble_uuid_t ble_uuid;
	ble_gatts_attr_md_t attr_md;

	// Add Custom Value characteristic
	memset(&cccd_md, 0, sizeof(cccd_md));

	//  Read  operation on cccd should be possible without authentication.
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

	cccd_md.write_perm = p_cus_init->custom_value_char_attr_md.cccd_write_perm;
	cccd_md.vloc = BLE_GATTS_VLOC_STACK;

	memset(&char_md, 0, sizeof(char_md));

	if (uuid == TX_CHAR_UUID) {

		char_md.char_props.indicate = 1;
	} else {

		char_md.char_props.write = 1;
	}
	char_md.p_char_user_desc = NULL;
	char_md.p_char_pf = NULL;
	char_md.p_user_desc_md = NULL;
	char_md.p_cccd_md = &cccd_md;
	char_md.p_sccd_md = NULL;

	ble_uuid.type = p_cus->uuid_type;
	ble_uuid.uuid = uuid;

	memset(&attr_md, 0, sizeof(attr_md));

	attr_md.read_perm = p_cus_init->custom_value_char_attr_md.read_perm;
	attr_md.write_perm = p_cus_init->custom_value_char_attr_md.write_perm;
	attr_md.vloc = BLE_GATTS_VLOC_USER;
	attr_md.rd_auth = 0;
	attr_md.wr_auth = 0;
	attr_md.vlen = 0;

	memset(&attr_char_value, 0, sizeof(attr_char_value));

	static uint8_t _index = 0;
	static uint8_t _array[3][135] = {0};
	attr_char_value.p_uuid = &ble_uuid;
	attr_char_value.p_attr_md = &attr_md;
	attr_char_value.init_len = 0;
	attr_char_value.p_value = _array[_index++];
	attr_char_value.init_offs = 0;
	attr_char_value.max_len = 135;

	err_code = sd_ble_gatts_characteristic_add(
			p_cus->service_handle, &char_md, &attr_char_value, p_handles);
	if (err_code != NRF_SUCCESS) {
		return err_code;
	}

	return NRF_SUCCESS;
}

#if 0
/**@brief Function for adding the Custom Value characteristic.
 *
 * @param[in]   p_cus        Battery Service structure.
 * @param[in]   p_cus_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t rx_char_add(ble_cus_t *p_cus, uint16_t uuid,
		const ble_cus_init_t *p_cus_init, ble_gatts_char_handles_t *p_handles) {

	uint32_t err_code;
	ble_gatts_char_md_t char_md;
	ble_gatts_attr_md_t cccd_md;
	ble_gatts_attr_t attr_char_value;
	ble_uuid_t ble_uuid;
	ble_gatts_attr_md_t attr_md;

	// Add Custom Value characteristic
	memset(&cccd_md, 0, sizeof(cccd_md));

	//  Read  operation on cccd should be possible without authentication.
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

	cccd_md.write_perm = p_cus_init->custom_value_char_attr_md.cccd_write_perm;
	cccd_md.vloc = BLE_GATTS_VLOC_STACK;

	memset(&char_md, 0, sizeof(char_md));

	char_md.char_props.read = 1;
	char_md.p_char_user_desc = NULL;
	char_md.p_char_pf = NULL;
	char_md.p_user_desc_md = NULL;
	char_md.p_cccd_md = &cccd_md;
	char_md.p_sccd_md = NULL;

	ble_uuid.type = p_cus->uuid_type;
	ble_uuid.uuid = uuid;

	memset(&attr_md, 0, sizeof(attr_md));

	attr_md.read_perm = p_cus_init->custom_value_char_attr_md.read_perm;
	attr_md.write_perm = p_cus_init->custom_value_char_attr_md.write_perm;
	attr_md.vloc = BLE_GATTS_VLOC_USER;
	attr_md.rd_auth = 0;
	attr_md.wr_auth = 0;
	attr_md.vlen = 0;

	memset(&attr_char_value, 0, sizeof(attr_char_value));

	attr_char_value.p_uuid = &ble_uuid;
	attr_char_value.p_attr_md = &attr_md;
	attr_char_value.init_offs = 0;

	if (uuid == R1_CHAR_UUID) {

		static uint8_t initVal[135] = {0x00};
		attr_char_value.init_len = sizeof(initVal);
		attr_char_value.p_value = initVal;
		attr_char_value.max_len = sizeof(initVal);
	} else if (uuid == R2_CHAR_UUID) {

		static uint8_t initVal[] = {0x00, 0xE4, 0x01, 0x00, 0x00};
		attr_char_value.init_len = sizeof(initVal);
		attr_char_value.p_value = initVal;
		attr_char_value.max_len = 135;

	} else {

		static uint8_t initVal[] = {0x00};
		attr_char_value.init_len = sizeof(initVal);
		attr_char_value.p_value = initVal;
		attr_char_value.max_len = 135;
	}

	err_code = sd_ble_gatts_characteristic_add(
			p_cus->service_handle, &char_md, &attr_char_value, p_handles);
	if (err_code != NRF_SUCCESS) {
		return err_code;
	}

	return NRF_SUCCESS;
}
#endif

uint32_t ble_cus_init(ble_cus_t *p_cus, const ble_cus_init_t *p_cus_init) {

    VERIFY_PARAM_NOT_NULL(p_cus);
    VERIFY_PARAM_NOT_NULL(p_cus_init);

	uint32_t err_code;
	ble_uuid_t ble_uuid;
	ble_uuid128_t base_uuid = {STEERER_SERVICE_UUID_BASE};

	// Initialize service structure
	p_cus->evt_handler = p_cus_init->evt_handler;
	p_cus->conn_handle = BLE_CONN_HANDLE_INVALID;

	// Add Custom Service UUID
	err_code = sd_ble_uuid_vs_add(&base_uuid, &p_cus->uuid_type);
	VERIFY_SUCCESS(err_code);

	ble_uuid.type = p_cus->uuid_type;
	ble_uuid.uuid = STEERER_SERVICE_UUID;

    /**@snippet [Adding proprietary Service to the SoftDevice] */
	err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
			&ble_uuid,
			&p_cus->service_handle);
    VERIFY_SUCCESS(err_code);

	// Add Custom Value characteristic
    err_code = custom_value_char_add(p_cus, STEERER_CHAR_UUID, p_cus_init, &p_cus->steerer_handles);
    VERIFY_SUCCESS(err_code);

    // Those do not seem needed
#if 0
    err_code = custom_value_char_add(p_cus, N1_CHAR_UUID, p_cus_init, &p_cus->n1_handles);
    VERIFY_SUCCESS(err_code);

    err_code = rx_char_add(p_cus, R2_CHAR_UUID, p_cus_init, &p_cus->r2_handles);
    VERIFY_SUCCESS(err_code);

    err_code = rx_char_add(p_cus, R1_CHAR_UUID, p_cus_init, &p_cus->r1_handles);
    VERIFY_SUCCESS(err_code);

    err_code = tx_char_add(p_cus, W1_CHAR_UUID, p_cus_init, &p_cus->w1_handles);
    VERIFY_SUCCESS(err_code);
#endif

    err_code = tx_char_add(p_cus, RX_CHAR_UUID, p_cus_init, &p_cus->rx_handles);
    VERIFY_SUCCESS(err_code);

    err_code = tx_char_add(p_cus, TX_CHAR_UUID, p_cus_init, &p_cus->tx_handles);
    VERIFY_SUCCESS(err_code);

	return (NRF_SUCCESS);
}

uint32_t ble_cus_tx_value_update(ble_cus_t *p_cus, uint8_t *custom_value, uint8_t data_len) {

	NRF_LOG_INFO("In ble_cus_tx_value_update. ");
	NRF_LOG_HEXDUMP_INFO(custom_value, data_len);

	if (p_cus == NULL) {
		return NRF_ERROR_NULL;
	}

	uint32_t err_code = NRF_SUCCESS;
	ble_gatts_value_t gatts_value;

	// Initialize value struct.
	memset(&gatts_value, 0, sizeof(gatts_value));

	gatts_value.len = data_len;
	gatts_value.offset = 0;
	gatts_value.p_value = custom_value;

	// Send value if connected and notifying.
	if ((p_cus->conn_handle != BLE_CONN_HANDLE_INVALID)) {
		ble_gatts_hvx_params_t hvx_params;

		memset(&hvx_params, 0, sizeof(hvx_params));

		hvx_params.handle = p_cus->tx_handles.value_handle;
		hvx_params.type = BLE_GATT_HVX_INDICATION;
		hvx_params.offset = gatts_value.offset;
		hvx_params.p_len = &gatts_value.len;
		hvx_params.p_data = gatts_value.p_value;

		err_code = sd_ble_gatts_hvx(p_cus->conn_handle, &hvx_params);
		NRF_LOG_INFO("ble_cus_tx_value_update result: %x. ", err_code);
		NRF_LOG_HEXDUMP_INFO(gatts_value.p_value, gatts_value.len);
	} else {
		err_code = NRF_ERROR_INVALID_STATE;
		NRF_LOG_INFO("ble_cus_tx_value_update result: NRF_ERROR_INVALID_STATE. ");
	}

	// Update database.
	err_code = sd_ble_gatts_value_set(
			p_cus->conn_handle, p_cus->tx_handles.value_handle, &gatts_value);
	if (err_code != NRF_SUCCESS) {
		return err_code;
	}

	return err_code;
}

uint32_t ble_cus_steering_value_update(ble_cus_t *p_cus, float angle) {

	NRF_LOG_DEBUG("In ble_cus_steerer_value_update. ");

	if (p_cus == NULL) {
		return NRF_ERROR_NULL;
	}

	uint32_t err_code = NRF_SUCCESS;
	ble_gatts_value_t gatts_value;

	// Initialize value struct.
	memset(&gatts_value, 0, sizeof(gatts_value));

	gatts_value.len = 4;
	gatts_value.offset = 0;
	gatts_value.p_value = (uint8_t *)&angle;

	// Update database.
	err_code = sd_ble_gatts_value_set(
			p_cus->conn_handle, p_cus->steerer_handles.value_handle, &gatts_value);
	if (err_code != NRF_SUCCESS) {
		return err_code;
	}

	// Send value if connected and notifying.
	if ((p_cus->conn_handle != BLE_CONN_HANDLE_INVALID)) {
		ble_gatts_hvx_params_t hvx_params;

		memset(&hvx_params, 0, sizeof(hvx_params));

		hvx_params.handle = p_cus->steerer_handles.value_handle;
		hvx_params.type = BLE_GATT_HVX_NOTIFICATION;
		hvx_params.offset = gatts_value.offset;
		hvx_params.p_len = &gatts_value.len;
		hvx_params.p_data = gatts_value.p_value;

		err_code = sd_ble_gatts_hvx(p_cus->conn_handle, &hvx_params);
		NRF_LOG_INFO("ble_cus_steering_value_update result: %x. ", err_code);

		NRF_LOG_HEXDUMP_WARNING(gatts_value.p_value, gatts_value.len);
	} else {
		err_code = NRF_ERROR_INVALID_STATE;
		NRF_LOG_ERROR("ble_cus_steering_value_update result: NRF_ERROR_INVALID_STATE. ");
	}

	return err_code;
}
