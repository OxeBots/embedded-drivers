#ifndef _NRF24L01_H_
#define _NRF24L01_H_

#include <driver/gpio.h>
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <atomic>
#include <cstdint>
#include <cstring>

#include "mirf.h"

typedef void (*data_received_callback_t)(void * pvParameters);

// Raw payload passed to the data-received callback.
struct DataReceivedEvent
{
    void * context;
    const uint8_t * payload;
    uint8_t length;
    uint8_t pipe;
};

class NRF24L01
{
   public:
    enum class TxStatus : uint8_t
    {
        OK = 0,
        MAX_RETRIES = 1,
        NO_ACK_SENT = 2,
        RADIO_ERR = 3,
    };

    struct TxResult
    {
        TxStatus status = TxStatus::RADIO_ERR;
        uint8_t retries = 0;
        uint16_t tx_time_us = 0;
    };

    // CE is a mirf CE pin: a GPIO, a virtual pin (HC595::Pin::write with a
    // shift-register bit as context) or none.
    NRF24L01(gpio_num_t irq, NRF24_CePin ce);
    ~NRF24L01();

    // Brings up the nRF24 device (Nrf24_init). SPIdev::init() must have
    // created the shared bus first; the other SPI devices register on it.
    void init_hw();

    // Legacy string-address overload kept for hardware tests.
    esp_err_t init(uint8_t channel,
                   uint8_t payload_size,
                   const char * tx_addr,
                   const char * rx_addr);

    // Binary 5-byte address initialization (LSByte-first hardware order).
    esp_err_t init_bytes(uint8_t channel,
                         uint8_t payload_size,
                         const uint8_t tx_addr[5],
                         const uint8_t rx_addr[5]);

    bool start(data_received_callback_t callback, void * context);

    // Asynchronous queued transmit to the configured default TX address (used by robots).
    esp_err_t send_raw(const uint8_t * data, uint8_t len);

    // Synchronous addressed transmit (used by the ESP32 Base Station).
    // Switches TX_ADDR / RX_ADDR_P0 on demand, waits for hardware TX_DS / MAX_RT
    // with a bounded timeout, reads OBSERVE_TX retry count, and returns to PRX.
    esp_err_t send_to_sync(const uint8_t tx_addr[5],
                           const uint8_t * data,
                           uint8_t len,
                           bool no_ack,
                           TxResult * out_result,
                           uint32_t timeout_ms = 10);

    // Enable Pipe 2 for shared broadcast reception using W_TX_PAYLOAD_NO_ACK.
    esp_err_t enable_broadcast_pipe(uint8_t broadcast_lsb);

    // Runtime reconfiguration primitives (thread-safe via internal radio mutex).
    esp_err_t set_channel(uint8_t channel);
    esp_err_t set_rx_address(const uint8_t rx_addr[5]);
    esp_err_t set_rf_params(rf24_datarate_e rate,
                            rf24_pa_dbm_e pa,
                            uint8_t retransmit_delay,
                            uint8_t retransmit_count);

    // Sweep RPD (-64 dBm carrier detect) across [start_ch..end_ch] and write
    // busy percentage (0..100) for each channel to out_busy_pct.
    esp_err_t scan_spectrum(uint8_t start_ch,
                            uint8_t end_ch,
                            uint8_t samples_per_ch,
                            uint8_t * out_busy_pct);

    // Radio Health Watchdog: verifies core registers and soft-resets the chip
    // if brownout or electromagnetic interference reset the radio registers.
    bool is_config_healthy();
    esp_err_t reconfigure_if_needed();

    bool is_ready() const;
    uint8_t get_channel() const;
    rf24_datarate_e get_data_rate() const;
    rf24_pa_dbm_e get_pa_level() const;
    uint8_t get_retransmit_delay() const;
    uint8_t get_retransmit_count() const;
    uint16_t get_reinit_count() const;

   private:
    static void IRAM_ATTR isr_handler(void * dev);
    static void task_wrapper(void * dev);
    void receiver_task();

    void drain_rx_locked();
    void apply_idle_rx_pipes_locked();
    bool is_config_healthy_locked();
    esp_err_t apply_full_config_locked();
    esp_err_t reconfigure_locked();
    void flush_tx_locked();

    NRF24_t m_dev;
    gpio_num_t m_irq_pin;
    NRF24_CePin m_ce;
    TaskHandle_t m_task_handle;
    QueueHandle_t m_interrupt_queue;
    QueueHandle_t m_tx_queue;
    SemaphoreHandle_t m_radio_mutex;
    data_received_callback_t m_on_data_received;
    void * m_callback_context = nullptr;

    uint8_t m_payload_size = 32;
    uint8_t m_channel = 76;
    rf24_datarate_e m_data_rate = RF24_2MBPS;
    rf24_pa_dbm_e m_pa_level = RF24_PA_MAX;
    uint8_t m_retransmit_delay = 2;
    uint8_t m_retransmit_count = 15;
    uint8_t m_tx_addr[5] = {};
    uint8_t m_rx_addr[5] = {};
    uint8_t m_last_tx_addr[5] = {};
    bool m_last_tx_addr_valid = false;
    bool m_broadcast_enabled = false;
    uint8_t m_broadcast_lsb = 0;

    bool m_hw_ready = false;
    bool m_initialized = false;
    uint64_t m_tx_started_us = 0;
    uint64_t m_last_health_check_us = 0;
    std::atomic<uint16_t> m_reinit_count{0};
};

#endif  // _NRF24L01_H_
