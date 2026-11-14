#ifndef _NRF24L01_H_
#define _NRF24L01_H_

#include <driver/gpio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <cstring>

#include "mirf.h"

typedef void (*data_received_callback_t)(void * pvParameters);

// Raw payload passed to the data-received callback.
struct DataReceivedEvent
{
    void * context;
    const uint8_t * payload;
    uint8_t length;
};

class NRF24L01
{
   public:
    // CE is a mirf CE pin: a GPIO, a virtual pin (HC595::Pin::write with a
    // shift-register bit as context) or none.
    NRF24L01(gpio_num_t irq, NRF24_CePin ce);
    ~NRF24L01();

    // Brings up the nRF24 device (Nrf24_init). SPIdev::init() must have
    // created the shared bus first; the other SPI devices register on it.
    void init_hw();

    esp_err_t init(uint8_t channel,
                   uint8_t payload_size,
                   const char * tx_addr,
                   const char * rx_addr);
    bool start(data_received_callback_t callback, void * context);
    esp_err_t send_raw(const uint8_t * data, uint8_t len);

   private:
    static void IRAM_ATTR isr_handler(void * dev);
    static void task_wrapper(void * dev);
    void receiver_task();

    NRF24_t m_dev;
    gpio_num_t m_irq_pin;
    NRF24_CePin m_ce;
    TaskHandle_t m_task_handle;
    QueueHandle_t m_interrupt_queue;
    QueueHandle_t m_tx_queue;
    data_received_callback_t m_on_data_received;
    void * m_callback_context = nullptr;
    uint8_t m_payload_size;
    bool m_hw_ready;
};

#endif  // _NRF24L01_H_
