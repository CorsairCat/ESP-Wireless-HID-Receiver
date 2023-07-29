#include "wireless.h"

#define TUSB_DESC_TOTAL_LEN      (TUD_CONFIG_DESC_LEN + CFG_TUD_HID * TUD_HID_DESC_LEN)

/**
 * @brief HID report descriptor
 *
 * In this example we implement Keyboard + Mouse HID device,
 * so we must define both report descriptors
 */
const uint8_t hid_report_descriptor[] = {
    TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(HID_ITF_PROTOCOL_KEYBOARD)),
    TUD_HID_REPORT_DESC_MOUSE_SELF(HID_REPORT_ID(HID_ITF_PROTOCOL_MOUSE))
};

/**
 * @brief String descriptor
 */
const char* hid_string_descriptor[5] = {
    // array of pointer to string descriptors
    (char[]){0x09, 0x04},  // 0: is supported language is English (0x0409)
    "TinyUSB",             // 1: Manufacturer
    "TinyUSB Device",      // 2: Product
    "123456",              // 3: Serials, should use chip ID
    "Example HID interface",  // 4: HID
};

static const uint8_t hid_configuration_descriptor[] = {
    // Configuration number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    // Interface number, string index, boot protocol, report descriptor len, EP In address, size & polling interval
    TUD_HID_DESCRIPTOR(0, 4, false, sizeof(hid_report_descriptor), 0x81, 16, 10),
};

/********* TinyUSB HID callbacks ***************/

// Invoked when received GET HID REPORT DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    // We use only one interface and one HID report descriptor, so we can ignore parameter 'instance'
    return hid_report_descriptor;
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
  (void) instance;
  (void) report_id;
  (void) report_type;
  (void) buffer;
  (void) reqlen;

  return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
}

/********* Application ***************/

static void app_send_hid_mouse(void)
{
    uint8_t buffer[HID_MOUSE_IN_RPT_LEN];
    hid_mouse_input_report_queue_t evt;
    while (xQueueReceive(g_usbhid_queue, &evt, portMAX_DELAY) == pdTRUE)
    {// wait for the packing of data from usb
        if (tud_mounted())
		{
			buffer[0] = evt.buttons.val;                    // Buttons
			buffer[1] = evt.z_displacement;                 // Wheel
			buffer[2] = evt.displacement[0];                // X
			buffer[3] = evt.displacement[0] >> 8;           // X lower
			buffer[4] = evt.displacement[1];                // Y
			buffer[5] = evt.displacement[1] >> 8;           // Y lower
			// ESP_LOGI(HID_LE_PRF_TAG, "Leftclick: %d\r", buffer[0]);
			tud_hid_report(HID_ITF_PROTOCOL_MOUSE, buffer, HID_MOUSE_IN_RPT_LEN);
		}
        // send calling queue back to usb func and ask for next commands for send
        // ESP_LOGW(TAG, "Current BLE Include %d reports, overall %lld us", evt.steps, evt._time_stamp);
        // _last_time_stamp = evt._time_stamp;
        // wait here for rough 0.5ms for balance
        // 3ms delay to make the usb reach 1000hz while the ble reach 180hz
        //vTaskDelay(1 / portTICK_PERIOD_MS);
	}
}

void wifi_sniffer_init(void)
{
	nvs_flash_init();
	// ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
	ESP_ERROR_CHECK( esp_wifi_set_country(&wifi_country) ); /* set country for channel range [1, 13] */
	ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
	ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_NULL) );
	ESP_ERROR_CHECK( esp_wifi_start() );
	esp_wifi_set_promiscuous(true);
	esp_wifi_set_promiscuous_rx_cb(&wifi_sniffer_packet_handler);
}

void wifi_sniffer_set_channel(uint8_t channel)
{
	esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
}

const char * wifi_sniffer_packet_type2str(wifi_promiscuous_pkt_type_t type)
{
	switch(type) {
	case WIFI_PKT_MGMT: return "MGMT";
	case WIFI_PKT_DATA: return "DATA";
	default:  
	case WIFI_PKT_MISC: return "MISC";
	}
}

void wifi_sniffer_packet_handler(void* buff, wifi_promiscuous_pkt_type_t type)
{
	if (type != WIFI_PKT_DATA)
		return;

	const wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t *)buff;
	const wifi_ieee80211_packet_t *ipkt = (wifi_ieee80211_packet_t *)ppkt->payload;
	const wifi_ieee80211_mac_hdr_t *hdr = &ipkt->hdr;

	hid_mouse_input_report_queue_t *mouse_report = (hid_mouse_input_report_queue_t *)ipkt;

	if (wifi_check_mac_address(paired_mac, hdr->addr2))
	{
		if (ppkt->rx_ctrl.sig_len < sizeof(hid_mouse_input_report_queue_t))
			return;
		xQueueSend(g_usbhid_queue, &mouse_report, 0);
	}
	else return;
	/*
	printf("PACKET TYPE=%s, CHAN=%02d, RSSI=%02d,"
		" RECV ADDR=%02x:%02x:%02x:%02x:%02x:%02x,"
		" SEND ADDR=%02x:%02x:%02x:%02x:%02x:%02x,"
		" FILT ADDR=%02x:%02x:%02x:%02x:%02x:%02x,"
		" PKG SIZE= %02d\n",
		wifi_sniffer_packet_type2str(type),
		ppkt->rx_ctrl.channel,
		ppkt->rx_ctrl.rssi,
		hdr->addr1[0],hdr->addr1[1],hdr->addr1[2],
		hdr->addr1[3],hdr->addr1[4],hdr->addr1[5],
		hdr->addr2[0],hdr->addr2[1],hdr->addr2[2],
		hdr->addr2[3],hdr->addr2[4],hdr->addr2[5],
		hdr->addr3[0],hdr->addr3[1],hdr->addr3[2],
		hdr->addr3[3],hdr->addr3[4],hdr->addr3[5],
		ppkt->rx_ctrl.sig_len
	);
	*/
}

uint8_t wifi_check_mac_address(uint8_t *a, uint8_t *b)
{
	for (uint8_t i = 0; i < 6; i++)
	{
		if (a[i] != b[i]) {return 0;}
	}
	return 1;
}

void app_main(void)
{
	TaskHandle_t usb_events_task_handle;
    BaseType_t task_created;

	g_usbhid_queue = xQueueCreate(USBHID_QUEUE_SIZE, sizeof(hid_mouse_input_report_queue_t));
    if (g_usbhid_queue == NULL) {
        ESP_LOGE(TAG, "Create mutex fail");
        return;
    }

	ESP_LOGI(TAG, "USB initialization");
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = hid_string_descriptor,
        .string_descriptor_count = sizeof(hid_string_descriptor) / sizeof(hid_string_descriptor[0]),
        .external_phy = false,
        .configuration_descriptor = hid_configuration_descriptor,
    };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGI(TAG, "USB initialization DONE");

	task_created = xTaskCreate(app_send_hid_mouse, "usb_events", 4096, NULL, 3, &usb_events_task_handle);
    assert(task_created);

	wifi_sniffer_init();
	wifi_sniffer_set_channel(channel);
}
