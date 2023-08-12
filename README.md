# steering_challenge

Zwift <=> Elite Sterzo challenge request / exchange:

```c
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

	}
}

```
