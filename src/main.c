#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_log.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"

#define I2C_MASTER_SCL_IO 9
#define I2C_MASTER_SDA_IO 8

#define ADXL345_ADDRS 0x53
#define ITG3205_ADDRS 0x68
#define HMC5883L_ADDRS 0x1E

static const char *TAG = "TWAI";

static twai_node_handle_t node_hdl = NULL;

// ---- I2c BUS config ------

i2c_master_bus_config_t i2c_mst_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_NUM_0,
    .scl_io_num = I2C_MASTER_SCL_IO,
    .sda_io_num = I2C_MASTER_SDA_IO,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
};


i2c_master_bus_handle_t master_bus_handle;

i2c_device_config_t adxl345_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = ADXL345_ADDRS,
    .scl_speed_hz = 100000,
};

i2c_device_config_t itg3205_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = ITG3205_ADDRS,
    .scl_speed_hz = 100000,
};

i2c_device_config_t hmc5883l_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = HMC5883L_ADDRS,
    .scl_speed_hz = 100000,
};

i2c_master_dev_handle_t adxl345_handle;
i2c_master_dev_handle_t itg3205_handle;
i2c_master_dev_handle_t hmc5883l_handle;

void i2c_setup(){
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &master_bus_handle));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(master_bus_handle, &adxl345_cfg, &adxl345_handle));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(master_bus_handle, &itg3205_cfg, &itg3205_handle));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(master_bus_handle, &hmc5883l_cfg, &hmc5883l_handle));
}
static esp_err_t i2c_probe_addr(uint8_t addr)
{
    // timeout in ms
    return i2c_master_probe(master_bus_handle, addr, 50);
}

static void i2c_check_gy85_addrs(void)
{
    const struct { const char *name; uint8_t addr; } devs[] = {
        {"ADXL345",  ADXL345_ADDRS},
        {"ITG3205",  ITG3205_ADDRS},
        {"HMC5883L", HMC5883L_ADDRS},
    };

    for (int i = 0; i < (int)(sizeof(devs)/sizeof(devs[0])); i++) {
        esp_err_t err = i2c_probe_addr(devs[i].addr);
        if (err == ESP_OK) {
            ESP_LOGI("I2C", "%s @ 0x%02X : OK (ACK)", devs[i].name, devs[i].addr);
        } else {
            ESP_LOGE("I2C", "%s @ 0x%02X : FAIL (%s)", devs[i].name, devs[i].addr, esp_err_to_name(err));
        }
    }
}

esp_err_t receive_i2c(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *buffer, size_t len)
{
    uint8_t regbuf[1] = { reg };
    return i2c_master_transmit_receive(dev, regbuf, sizeof(regbuf), buffer, len, 50);
}

esp_err_t receive_i2c(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *buffer, size_t len)
{
    uint8_t regbuf[1] = { reg };
    return i2c_master_transmit(dev, buffer, len, 50);
}

// ---- RX mailbox (ISR writes, main reads) ----
static volatile bool rx_pending = false;
static twai_frame_header_t rx_hdr;
static uint8_t rx_data[8];
static uint8_t rx_len;

// static TaskHandle_t s_tx_waiter = NULL;
// static volatile bool s_tx_ok = false;
static volatile bool tx_busy = false;
static uint8_t tx_buf[8];  // persistent

//TODO: Allow upto 5 tx_buf using tx_pool 

static twai_onchip_node_config_t node_config = {
    .io_cfg = {
        .tx = GPIO_NUM_4,
        .rx = GPIO_NUM_5,
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
        memcpy(rx_data, tmp, len);
        rx_len = len;

        rx_pending = true;
    }
    return false;
}

static bool IRAM_ATTR twai_tx_done_cb(twai_node_handle_t handle, const twai_tx_done_event_data_t *edata, void *user_ctx) 
{
    (void)handle; (void)user_ctx; (void)edata; // stopping not used warning
    
    tx_busy=false;
    return false;
}

static const twai_event_callbacks_t user_cbs = {
    .on_tx_done = twai_tx_done_cb,
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
    if (tx_busy) return ESP_ERR_INVALID_STATE; //Remove for Pool?

    // IMPORTANT: some TWAI implementations are zero-copy for TX buffers,
    // so make the buffer stable until TX completes.
    memcpy(tx_buf, data, 8);

    twai_frame_t tx;
    tx.header.id = id;
    tx.header.dlc = 8;
    tx.header.ide = extended;
    tx.header.rtr = false;
    tx.buffer = tx_buf;
    tx.buffer_len = 8;

    tx_busy = true;

    ESP_LOGI(TAG, "%d", sizeof(tx_buf));

    esp_err_t err = twai_node_transmit(node_hdl, &tx, 0); // doesn't wait for queue
        if (err != ESP_OK) {
        tx_busy = false; // didn’t queue, buffer can be reused
    }
    return err;
}
void app_main(void)
{
    ESP_LOGI(TAG, "APP_MAIN STARTED");

    can_setup();
    i2c_setup();

    i2c_check_gy85_addrs();

    uint8_t rx_data[1];
    receive_i2c(adxl345_handle, 0x00, rx_data, 1);
    uint8_t tx_data[1];
    write_i2c(adxl345_handle, 0x00, tx_data, 1);

    ESP_LOGI(TAG, "Data: 0x{%02X}", data[0]);
    
    // test send (needs another node on bus to ACK, otherwise you can get TX errors)
    // const uint8_t data[8] = {1,2,3,4,5,6,7,8};
    // ESP_ERROR_CHECK(can_send_u8_8(0x123, false, data));
   
    // while (1) {
    //     if (rx_pending) {
    //         rx_pending = false;
    //         ESP_LOGI(TAG, "RX id=0x%lx ide=%d dlc=%d",
    //                  (unsigned long)rx_hdr.id, rx_hdr.ide, rx_hdr.dlc);
    //         ESP_LOG_BUFFER_HEX(TAG, rx_data, rx_len);
    //     }
    //     // optional: tiny delay to avoid a hot spin loop
    //     vTaskDelay(10);
    // }
}
