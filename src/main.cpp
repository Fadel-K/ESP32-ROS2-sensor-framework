#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "TWAI";

static twai_node_handle_t node_hdl = NULL;

static twai_onchip_node_config_t node_config = {
    .io_cfg = {
        .tx = 4,
        .rx = 5,
    },
    .bit_timing = {
        .bitrate = 200000,
    },
    .tx_queue_depth = 5,
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

void can_setup(void)
{
    ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &node_hdl));

    twai_event_callbacks_t user_cbs = {
    .on_rx_done = twai_rx_cb,
    };
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(node_hdl, &user_cbs, NULL));
    
    ESP_ERROR_CHECK(twai_node_enable(node_hdl));
    ESP_LOGI(TAG, "TWAI enabled");
}

// Send 8 bytes, classic CAN (DLC=8)
esp_err_t can_send_u8_8(uint32_t id, bool extended, const uint8_t data[8])
{
    // IMPORTANT: some TWAI implementations are zero-copy for TX buffers,
    // so make the buffer stable until TX completes.
    static uint8_t tx_buf[8];
    memcpy(tx_buf, data, 8);

    twai_frame_t tx = {
        .header = {
            .id  = id,
            .ide = extended, // true = 29-bit, false = 11-bit
            .rtr = false,
            .dlc = 8,
        },
        .buffer = tx_buf,
        .buffer_len = 8,
    };

    ESP_LOGI(TAG, "%d", sizeof(tx_buf));

    esp_err_t err = twai_node_transmit(node_hdl, &tx, -1); // wait for space in TX queue
    if (err != ESP_OK) return err;

    // wait until actually sent (simplest, safest while learning)
    return twai_node_transmit_wait_all_done(node_hdl, -1);
}

// Blocking receive: waits until *one* frame arrives
// esp_err_t can_recv_once(void)
// {
//     uint8_t rx_buf[8];
//     twai_frame_t rx = {
//         .buffer = rx_buf,
//         .buffer_len = sizeof(rx_buf),
//     };

//     // This call blocks until a frame is available.
//     // (Function name may differ depending on IDF version; if yours lacks it,
//     // I’ll map it to the correct one once you paste the compile error.)
//     esp_err_t err = twai_node_receive(node_hdl, &rx, -1);
//     if (err != ESP_OK) return err;

//     // DLC is 0..8 for classic CAN :contentReference[oaicite:3]{index=3}
//     uint8_t len = (rx.header.dlc <= 8) ? rx.header.dlc : 8;

//     ESP_LOGI(TAG, "RX id=0x%lx ide=%d dlc=%d", (unsigned long)rx.header.id, rx.header.ide, rx.header.dlc);
//     ESP_LOG_BUFFER_HEX(TAG, rx_buf, len);

//     return ESP_OK;
// }

void app_main(void)
{
    can_setup();

    // test send (needs another node on bus to ACK, otherwise you can get TX errors)
    const uint8_t data[8] = {1,2,3,4,5,6,7,8};
    ESP_ERROR_CHECK(can_send_u8_8(0x123, false, data));

    while (1) {
        ESP_ERROR_CHECK(can_recv_once()); // blocks until something arrives
    }
}
