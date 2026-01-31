#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_log.h"
#include "string.h"

static const char *TAG = "DEBUG"; // Define a tag for log messages

twai_node_handle_t node_hdl = NULL;
twai_onchip_node_config_t node_config = {
    .io_cfg = {
        .tx = 4,       // TWAI TX GPIO pin
        .rx = 5,       // TWAI RX GPIO pin
    },
    .bit_timing = {.bitrate = 200000,},  // 200 kbps bitrate
    .tx_queue_depth = 5,        // Transmit queue depth set to 5
};

static bool twai_rx_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
    uint8_t recv_buff[8];
    twai_frame_t rx_frame = {
        .buffer = recv_buff,
        .buffer_len = sizeof(recv_buff),
    };
    if (ESP_OK == twai_node_receive_from_isr(handle, &rx_frame)) {
        // receive ok, do something here
        ESP_LOGI(TAG, "RECEIVED");
        // Print buffer as hex
        ESP_LOG_BUFFER_HEX(TAG, rx_frame.buffer, rx_frame.buffer_len);
        // Or if it's a null-terminated string:
        // ESP_LOGI(TAG, "Data: %s", (char*)rx_frame.buffer);
    }
    return false;
}

void setup_twai() {
    // Create a new TWAI controller driver instance
    ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &node_hdl));
    twai_event_callbacks_t user_cbs = {
    .on_rx_done = twai_rx_cb,
    };
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(node_hdl, &user_cbs, NULL));
    // Start the TWAI controller
    ESP_ERROR_CHECK(twai_node_enable(node_hdl));
}
void send_msg(uint8_t send_buff[8] = {0}) {
    twai_frame_t tx_msg = {};
    tx_msg.header.id = 0x1;           // Message ID
    tx_msg.header.ide = true;         // Use 29-bit extended ID format
    tx_msg.buffer = send_buff;        // Pointer to data to transmit
    tx_msg.buffer_len = sizeof(send_buff);  // Length of data to transmit
    ESP_ERROR_CHECK(twai_node_transmit(node_hdl, &tx_msg, 0));  // Timeout = 0: returns immediately if queue is full
    // ESP_ERROR_CHECK(twai_node_transmit_wait_all_done(node_hdl, -1));  // Wait for transmission to finish
    ESP_ERROR_CHECK(twai_event_callbacks_t::on_tx_done(node_hdl, &user_cbs, NULL));
}