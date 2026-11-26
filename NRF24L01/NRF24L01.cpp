#include "NRF24L01.h"

#include <esp_err.h>
#include <esp_timer.h>
#include <sdkconfig.h>

static const char * TAG = "NRF24L01";

/**
 * @brief Construct a radio with the given CE line.
 * @param irq The GPIO pin connected to the NRF24L01's IRQ pin.
 * @param ce  CE line: a GPIO or a virtual pin (e.g. a shift-register output).
 */
NRF24L01::NRF24L01(gpio_num_t irq, NRF24_CePin ce)
: m_irq_pin(irq),
  m_ce(ce),
  m_task_handle(NULL),
  m_interrupt_queue(NULL),
  m_tx_queue(NULL),
  m_on_data_received(nullptr),
  m_payload_size(32),
  m_hw_ready(false)
{
    memset(&m_dev, 0, sizeof(NRF24_t));
}

/**
 * @brief Destroy the NRF24L01 object and free resources.
 */
NRF24L01::~NRF24L01()
{
    if (m_task_handle)
        vTaskDelete(m_task_handle);
    if (m_interrupt_queue)
        vQueueDelete(m_interrupt_queue);
    if (m_tx_queue)
        vQueueDelete(m_tx_queue);

    gpio_isr_handler_remove(m_irq_pin);
    Nrf24_deinit(&m_dev);
}

/**
 * @brief Registers the nRF24 on the shared SPI bus and brings the device up.
 *
 * SPIdev::init() must run first. The mirf driver drives the chip enable line
 * configured by the constructor.
 */
void NRF24L01::init_hw()
{
    Nrf24_init(&m_dev, m_ce);
    vTaskDelay(pdMS_TO_TICKS(100));
    m_hw_ready = true;
}

/**
 * @brief Initializes the NRF24L01 module.
 * @param channel      RF Channel (0-125).
 * @param payload_size The fixed payload size (1-32 bytes).
 * @param tx_addr      The 5-byte transmit address.
 * @param rx_addr      The 5-byte receive address.
 * @return ESP_OK on success, or an error from the mirf library.
 */
esp_err_t NRF24L01::init(uint8_t channel,
                         uint8_t payload_size,
                         const char * tx_addr,
                         const char * rx_addr)
{
    if (!m_hw_ready)
    {
        ESP_LOGE(TAG, "init() called before init_hw()");
        return ESP_ERR_INVALID_STATE;
    }

    m_payload_size = payload_size;

    Nrf24_config(&m_dev, channel, m_payload_size);

#ifdef CONFIG_RF_RATIO_250K
    Nrf24_SetSpeedDataRates(&m_dev, RF24_250KBPS);
#elif defined(CONFIG_RF_RATIO_1M)
    Nrf24_SetSpeedDataRates(&m_dev, RF24_1MBPS);
#elif defined(CONFIG_RF_RATIO_2M)
    Nrf24_SetSpeedDataRates(&m_dev, RF24_2MBPS);
#else
#error "Invalid RF Data Rate. Check menuconfig settings."
#endif

    esp_err_t ret = Nrf24_setRADDR(&m_dev, (uint8_t *)rx_addr);
    if (ret != ESP_OK)
    {
        ESP_LOGE(
          TAG, "nrf24l01 not installed / failed to set RADDR (Check Wiring & add 10uF Capacitor!)");
        return ret;
    }

    // TX_ADDR must match RX_ADDR_P0 for Auto-ACK to work.
    ret = Nrf24_setTADDR(&m_dev, (uint8_t *)tx_addr);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set TADDR/RADDR_P0. NRF24 not found?");
        return ret;
    }

    ESP_LOGI(TAG, "Using Pipe 0. RX_ADDR_P0 and TX_ADDR set to: %s", tx_addr);

    Nrf24_setRetransmitCount(&m_dev, CONFIG_RETRANSMIT_COUNT);
    Nrf24_setRetransmitDelay(&m_dev, CONFIG_RETRANSMIT_DELAY);

    // Keep only Pipe 1 enabled while idle in receive mode so robots do not
    // listen to Pipe 0 (the base station receive address) and answer other
    // robots' telemetry with false or colliding acknowledgements. Pipe 0 is
    // re-enabled only while transmitting so the radio still receives the base
    // station's acknowledgement. Keep automatic acknowledgement enabled on both
    // pipes because the dongle always transmits Enhanced ShockBurst frames
    // (clearing Pipe 1 here drops the 9-bit packet control field and corrupts
    // the payload alignment).
    Nrf24_configRegister(&m_dev, EN_RXADDR, (1 << ERX_P1));
    Nrf24_configRegister(&m_dev, EN_AA, (1 << ENAA_P0) | (1 << ENAA_P1));

    ESP_LOGI(TAG, "NRF24L01 Receiver Initialized:");
    Nrf24_printDetails(&m_dev);

    m_interrupt_queue = xQueueCreate(10, sizeof(uint32_t));
    m_tx_queue = xQueueCreate(10, m_payload_size * sizeof(uint8_t));
    if (m_interrupt_queue == NULL || m_tx_queue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create the receiver and TX queues");
        return ESP_FAIL;
    }

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = (1ULL << m_irq_pin);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;

    ret = gpio_config(&io_conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure the IRQ GPIO: %s", esp_err_to_name(ret));
        return ret;
    }

    // The IRQ handler is negative-edge triggered. If RX_DR (or any other
    // interrupt flag) was already latched during init — e.g. a packet arrived
    // while the radio was powering up — the IRQ line is already low when the
    // handler is registered, the falling edge is missed, and the receiver
    // task would never wake up (and never flush queued TX either). Clear the
    // flags and flush the RX FIFO so the line returns to idle first.
    Nrf24_configRegister(&m_dev, REG_STATUS, (1 << RX_DR) | (1 << TX_DS) | (1 << MAX_RT));
    Nrf24_flushRx(&m_dev);

    // gpio_install_isr_service() must be called in main.cpp before this.
    ret = gpio_isr_handler_add(m_irq_pin, NRF24L01::isr_handler, (void *)this);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to register the IRQ handler: %s", esp_err_to_name(ret));
        return ret;
    }

    // A packet arriving in the window between clearing STATUS and registering
    // the handler would again be missed. If the line is already low, push one
    // synthetic interrupt so the receiver task drains whatever is pending.
    if (gpio_get_level(m_irq_pin) == 0)
    {
        uint32_t irq_pin = static_cast<uint32_t>(m_irq_pin);
        xQueueSend(m_interrupt_queue, &irq_pin, 0);
    }

    return ESP_OK;
}

/**
 * @brief Starts the background receiver task.
 * @param callback Called with a DataReceivedEvent whenever a packet arrives.
 * @param context Stored in the event so the callback can recover its owner.
 * @return true if the task started successfully.
 */
bool NRF24L01::start(data_received_callback_t callback, void * context)
{
    if (!callback)
    {
        ESP_LOGE(TAG, "No data-received callback configured");
        return false;
    }
    m_on_data_received = callback;
    m_callback_context = context;

    BaseType_t task_created =
      xTaskCreate(&NRF24L01::task_wrapper, "nrf24_receiver_task", 4096, this, 5, &m_task_handle);
    return task_created == pdPASS;
}

/**
 * @brief Static ISR handler for the IRQ pin — triggers receiver task via queue.
 */
void IRAM_ATTR NRF24L01::isr_handler(void * dev)
{
    auto * instance = static_cast<NRF24L01 *>(dev);
    uint32_t gpio_num = static_cast<uint32_t>(instance->m_irq_pin);
    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(instance->m_interrupt_queue, &gpio_num, &woken);
    // Wake the receiver task now instead of at the next 10 ms tick.
    portYIELD_FROM_ISR(woken);
}

/**
 * @brief Static trampoline to launch the FreeRTOS receiver task.
 */
void NRF24L01::task_wrapper(void * dev)
{
    static_cast<NRF24L01 *>(dev)->receiver_task();
}

/**
 * @brief Background task: drains the RX FIFO and flushes the TX queue.
 *
 * On RX: strips the dongle's prepended length byte, then passes the raw
 * payload bytes directly to the application callback — no protocol parsing.
 *
 * On TX: sends whatever raw frames the application enqueued via send_raw().
 */
void NRF24L01::receiver_task()
{
    ESP_LOGI(TAG, "Receiver task started. Listening for data...");
    uint32_t io_num;
    uint8_t buffer[32];
    uint8_t tx_buffer[32];

    while (true)
    {
        // Normal path wakes on the radio IRQ. The timeout is a safety net for
        // a TX completion whose falling edge was masked by an already-latched
        // RX_DR, and for frames queued by a task other than this one.
        const bool signalled =
          xQueueReceive(m_interrupt_queue, &io_num, pdMS_TO_TICKS(10)) == pdTRUE;
        if (signalled && io_num != static_cast<uint32_t>(m_irq_pin))
            continue;

        auto drain_rx = [&]() {
            while (Nrf24_dataReady(&m_dev))
            {
                Nrf24_getData(&m_dev, buffer);

                // byte 0 is the dongle's payload-length prefix
                uint8_t payload_len = buffer[0];

                if (payload_len > 0 && payload_len <= 31 && m_on_data_received)
                {
                    // Deliver raw bytes starting after the length byte
                    DataReceivedEvent event = {m_callback_context, &buffer[1], payload_len};
                    m_on_data_received(&event);
                }
                else
                {
                    ESP_LOGW(TAG, "Dropped packet — invalid length byte: %d", payload_len);
                }
            }
        };

        // Transmit complete and retry-limit interrupts also pull the interrupt
        // line low. Return to receive mode with Pipe 0 disabled so this robot
        // never acknowledges another robot's uplink frames.
        if (m_dev.PTX)
        {
            const uint8_t tx_status = Nrf24_getStatus(&m_dev);
            if (tx_status & ((1 << TX_DS) | (1 << MAX_RT)))
            {
                Nrf24_configRegister(&m_dev, EN_RXADDR, (1 << ERX_P1));
                Nrf24_powerUpRx(&m_dev);
            }
        }

        // Drain pending receive packets on every wakeup while not transmitting
        // so a latched receive interrupt can never stall the edge-triggered line.
        if (!m_dev.PTX)
            drain_rx();

        // Send at most one queued transmit frame while the radio is in receive
        // mode. Completion is handled by the interrupt branch above.
        if (!m_dev.PTX && xQueueReceive(m_tx_queue, &tx_buffer, 0) == pdTRUE)
        {
            // Wait at least 5 ms for the dongle to finish its transmit cycle
            // and return to receive mode before replying. At 100 Hz tick rate,
            // pdMS_TO_TICKS(5) truncates to zero, so sleep full ticks until
            // 5 ms have actually elapsed.
            const int64_t wait_start_us = esp_timer_get_time();
            do
            {
                vTaskDelay(1);
            } while ((esp_timer_get_time() - wait_start_us) < 5000);

            // Clear any duplicate downlink retransmissions that arrived during
            // the wait so the interrupt line returns high before transmitting.
            drain_rx();

            // Enable Pipe 0 only while in transmit mode (with chip enable low)
            // so the radio can receive the base station's acknowledgement.
            Nrf24_ceLow(&m_dev);
            Nrf24_configRegister(&m_dev, EN_RXADDR, (1 << ERX_P0) | (1 << ERX_P1));
            Nrf24_send(&m_dev, tx_buffer);
        }
    }
}

/**
 * @brief Enqueue raw bytes for over-the-air transmission.
 *
 * Builds a 32-byte RF frame by placing the payload length in byte 0 (dongle
 * convention) and copying the caller's data into bytes 1..len.  The rest of
 * the frame is zero-padded.  The caller must NOT include the length byte or
 * any padding — just the meaningful protocol payload.
 *
 * @param data  Pointer to payload bytes.
 * @param len   Payload length (must be ≤ 31).
 * @return ESP_OK on success.
 */
esp_err_t NRF24L01::send_raw(const uint8_t * data, uint8_t len)
{
    if (len > 31)
    {
        ESP_LOGE(TAG, "send_raw: payload too large (%d bytes, max 31)", len);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t frame[32] = {0};
    frame[0] = len;                // dongle length prefix
    memcpy(&frame[1], data, len);  // protocol payload

    if (xQueueSend(m_tx_queue, frame, pdMS_TO_TICKS(100)) != pdPASS)
    {
        ESP_LOGE(TAG, "send_raw: TX queue full");
        return ESP_FAIL;
    }

    return ESP_OK;
}
