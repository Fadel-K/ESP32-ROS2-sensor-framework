#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_log.h"
#include <string.h>
#include <esp_attr.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "TWAI";

static twai_node_handle_t node_hdl = NULL;

// ---- RX mailbox (ISR writes, main reads) ----
static volatile bool rx_pending = false;
static twai_frame_header_t rx_hdr;
static uint8_t rx_data[8];
static uint8_t rx_len;

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


static bool IRAM_ATTR twai_rx_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx)
{   
    (void)edata; //avoid unused variables
    (void)user_ctx; //avoid unused variables

    uint8_t tmp[8];
    twai_frame_t rx_frame = {
        .buffer = tmp,
        .buffer_len = sizeof(tmp),
    };
    if (ESP_OK == twai_node_receive_from_isr(handle, &rx_frame)) {
        rx_hdr = rx_frame.header;

        uint8_t len = (rx_frame.header.dlc <= 8) ? rx_frame.header.dlc : 8;
        memcpy((void*)rx_data, tmp, len);
        rx_len = len;

        rx_pending = true;
    }
    return false;
}
static const twai_event_callbacks_t user_cbs = {
.on_rx_done = twai_rx_cb,
};

void can_setup(void)
{
    ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &node_hdl));


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

void app_main(void)
{
    can_setup();

    // test send (needs another node on bus to ACK, otherwise you can get TX errors)
    const uint8_t data[8] = {1,2,3,4,5,6,7,8};
    ESP_ERROR_CHECK(can_send_u8_8(0x123, false, data));
   
    while (1) {
        if (rx_pending) {
            rx_pending = false;
            ESP_LOGI(TAG, "RX id=0x%lx ide=%d dlc=%d",
                     (unsigned long)rx_hdr.id, rx_hdr.ide, rx_hdr.dlc);
            ESP_LOG_BUFFER_HEX(TAG, rx_data, rx_len);
        }
        // optional: tiny delay to avoid a hot spin loop
        vTaskDelay(10);
    }
}
