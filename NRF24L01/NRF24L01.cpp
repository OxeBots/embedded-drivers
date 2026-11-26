#include "NRF24L01.h"

#include <esp_err.h>
#include <esp_timer.h>
#include <rom/ets_sys.h>
#include <sdkconfig.h>

#include "SPIdev.h"

static const char * TAG = "NRF24L01";

static constexpr uint8_t MAX_RF_CHANNEL = 125;
static constexpr uint64_t HEALTH_CHECK_INTERVAL_US = 250000ULL;  // 250 ms idle check
static constexpr uint64_t ASYNC_TX_TIMEOUT_US = 15000ULL;        // 15 ms guard against stuck PTX

#ifdef CONFIG_RADIO_REPLY_DELAY_US
static constexpr uint32_t ROBOT_REPLY_DELAY_US = CONFIG_RADIO_REPLY_DELAY_US;
#else
static constexpr uint32_t ROBOT_REPLY_DELAY_US = 500U;
#endif

static rf24_datarate_e default_data_rate()
{
#ifdef CONFIG_RF_RATIO_250K
    return RF24_250KBPS;
#elif defined(CONFIG_RF_RATIO_1M)
    return RF24_1MBPS;
#else
    return RF24_2MBPS;
#endif
}

static uint8_t expected_rf_setup_mask(rf24_datarate_e rate, rf24_pa_dbm_e pa)
{
    uint8_t reg = static_cast<uint8_t>((static_cast<uint8_t>(pa) & 0x03U) << RF_PWR);
    if (rate == RF24_250KBPS)
        reg |= static_cast<uint8_t>(1U << RF_DR_LOW);
    else if (rate == RF24_2MBPS)
        reg |= static_cast<uint8_t>(1U << RF_DR_HIGH);
    return reg;
}

NRF24L01::NRF24L01(gpio_num_t irq, NRF24_CePin ce)
: m_irq_pin(irq),
  m_ce(ce),
  m_task_handle(nullptr),
  m_interrupt_queue(nullptr),
  m_tx_queue(nullptr),
  m_radio_mutex(xSemaphoreCreateRecursiveMutex()),
  m_on_data_received(nullptr),
  m_payload_size(32),
  m_data_rate(default_data_rate()),
  m_pa_level(RF24_PA_MAX),
  m_retransmit_delay(CONFIG_RETRANSMIT_DELAY),
  m_retransmit_count(CONFIG_RETRANSMIT_COUNT)
{
    memset(&m_dev, 0, sizeof(NRF24_t));
}

NRF24L01::~NRF24L01()
{
    if (m_task_handle)
        vTaskDelete(m_task_handle);
    if (m_interrupt_queue)
        vQueueDelete(m_interrupt_queue);
    if (m_tx_queue)
        vQueueDelete(m_tx_queue);
    if (m_radio_mutex)
        vSemaphoreDelete(m_radio_mutex);

    gpio_isr_handler_remove(m_irq_pin);
    Nrf24_deinit(&m_dev);
}

void NRF24L01::init_hw()
{
    Nrf24_init(&m_dev, m_ce);
    vTaskDelay(pdMS_TO_TICKS(100));
    m_hw_ready = true;
}

esp_err_t NRF24L01::init(uint8_t channel,
                         uint8_t payload_size,
                         const char * tx_addr,
                         const char * rx_addr)
{
    if (tx_addr == nullptr || rx_addr == nullptr)
        return ESP_ERR_INVALID_ARG;

    uint8_t tx_bytes[5] = {};
    uint8_t rx_bytes[5] = {};
    memcpy(tx_bytes, tx_addr, 5);
    memcpy(rx_bytes, rx_addr, 5);
    return init_bytes(channel, payload_size, tx_bytes, rx_bytes);
}

esp_err_t NRF24L01::init_bytes(uint8_t channel,
                               uint8_t payload_size,
                               const uint8_t tx_addr[5],
                               const uint8_t rx_addr[5])
{
    if (!m_hw_ready)
    {
        ESP_LOGE(TAG, "init_bytes() called before init_hw()");
        return ESP_ERR_INVALID_STATE;
    }
    if (tx_addr == nullptr || rx_addr == nullptr || channel > MAX_RF_CHANNEL || payload_size == 0 ||
        payload_size > 32)
        return ESP_ERR_INVALID_ARG;
    if (m_radio_mutex == nullptr)
        return ESP_ERR_NO_MEM;

    xSemaphoreTakeRecursive(m_radio_mutex, portMAX_DELAY);

    m_channel = channel;
    m_payload_size = payload_size;
    memcpy(m_tx_addr, tx_addr, 5);
    memcpy(m_rx_addr, rx_addr, 5);
    m_last_tx_addr_valid = false;

    esp_err_t ret = apply_full_config_locked();
    if (ret != ESP_OK)
    {
        xSemaphoreGiveRecursive(m_radio_mutex);
        return ret;
    }

    ESP_LOGI(TAG, "NRF24L01 Initialized (ch=%u, rate=%d, pa=%d, ard=%u, arc=%u):",
             m_channel,
             static_cast<int>(m_data_rate),
             static_cast<int>(m_pa_level),
             m_retransmit_delay,
             m_retransmit_count);
    Nrf24_printDetails(&m_dev);

    if (m_interrupt_queue == nullptr)
        m_interrupt_queue = xQueueCreate(16, sizeof(uint32_t));
    if (m_tx_queue == nullptr)
        m_tx_queue = xQueueCreate(10, m_payload_size * sizeof(uint8_t));
    if (m_interrupt_queue == nullptr || m_tx_queue == nullptr)
    {
        ESP_LOGE(TAG, "Failed to create the receiver and transmit queues");
        xSemaphoreGiveRecursive(m_radio_mutex);
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
        ESP_LOGE(TAG, "Failed to configure the interrupt pin: %s", esp_err_to_name(ret));
        xSemaphoreGiveRecursive(m_radio_mutex);
        return ret;
    }

    Nrf24_configRegister(&m_dev, REG_STATUS, (1 << RX_DR) | (1 << TX_DS) | (1 << MAX_RT));
    Nrf24_flushRx(&m_dev);
    flush_tx_locked();

    ret = gpio_isr_handler_add(m_irq_pin, NRF24L01::isr_handler, (void *)this);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to register the interrupt handler: %s", esp_err_to_name(ret));
        xSemaphoreGiveRecursive(m_radio_mutex);
        return ret;
    }

    m_initialized = true;
    m_last_health_check_us = static_cast<uint64_t>(esp_timer_get_time());

    if (gpio_get_level(m_irq_pin) == 0)
    {
        uint32_t irq_pin = static_cast<uint32_t>(m_irq_pin);
        xQueueSend(m_interrupt_queue, &irq_pin, 0);
    }

    xSemaphoreGiveRecursive(m_radio_mutex);
    return ESP_OK;
}

esp_err_t NRF24L01::apply_full_config_locked()
{
    Nrf24_ceLow(&m_dev);
    Nrf24_config(&m_dev, m_channel, m_payload_size);
    Nrf24_SetSpeedDataRates(&m_dev, static_cast<uint8_t>(m_data_rate));
    Nrf24_SetOutputRF_PWR(&m_dev, static_cast<uint8_t>(m_pa_level));
    Nrf24_setRetransmitDelay(&m_dev, m_retransmit_delay);
    Nrf24_setRetransmitCount(&m_dev, m_retransmit_count);
    Nrf24_enableNoAckFeature(&m_dev);

    esp_err_t ret = Nrf24_setRADDR(&m_dev, m_rx_addr);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG,
                 "nRF24L01 failed to set receive address (check wiring and 10uF + 100nF decoupling)");
        return ret;
    }

    ret = Nrf24_setTADDR(&m_dev, m_tx_addr);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set transmit address");
        return ret;
    }
    memcpy(m_last_tx_addr, m_tx_addr, 5);
    m_last_tx_addr_valid = true;

    if (m_broadcast_enabled)
        Nrf24_addRADDR(&m_dev, 2, m_broadcast_lsb);

    apply_idle_rx_pipes_locked();

    uint8_t en_aa = static_cast<uint8_t>((1U << ENAA_P0) | (1U << ENAA_P1));
    if (m_broadcast_enabled)
        en_aa |= static_cast<uint8_t>(1U << ENAA_P2);
    Nrf24_configRegister(&m_dev, EN_AA, en_aa);

    flush_tx_locked();
    Nrf24_flushRx(&m_dev);
    Nrf24_configRegister(&m_dev, REG_STATUS, (1 << RX_DR) | (1 << TX_DS) | (1 << MAX_RT));
    Nrf24_powerUpRx(&m_dev);
    return ESP_OK;
}

void NRF24L01::apply_idle_rx_pipes_locked()
{
    uint8_t en_rx = static_cast<uint8_t>(1U << ERX_P1);
    if (m_broadcast_enabled)
        en_rx |= static_cast<uint8_t>(1U << ERX_P2);
    Nrf24_configRegister(&m_dev, EN_RXADDR, en_rx);
}

void NRF24L01::flush_tx_locked()
{
    const uint8_t cmd = FLUSH_TX;
    SPIdev::write(m_dev.device_key, &cmd, 1);
}

bool NRF24L01::is_config_healthy_locked()
{
    if (!m_initialized)
        return false;

    uint8_t cfg = 0;
    uint8_t ch = 0;
    uint8_t rf_setup = 0;
    uint8_t en_rx = 0;
    uint8_t rx_p1[5] = {};

    Nrf24_readRegister(&m_dev, CONFIG, &cfg, 1);
    Nrf24_readRegister(&m_dev, RF_CH, &ch, 1);
    Nrf24_readRegister(&m_dev, RF_SETUP, &rf_setup, 1);
    Nrf24_readRegister(&m_dev, EN_RXADDR, &en_rx, 1);
    Nrf24_readRegister(&m_dev, RX_ADDR_P1, rx_p1, 5);

    const uint8_t required_cfg = static_cast<uint8_t>((1U << PWR_UP) | (1U << EN_CRC) | (1U << CRCO));
    if ((cfg & required_cfg) != required_cfg)
        return false;
    if (ch != m_channel)
        return false;

    const uint8_t rf_mask = static_cast<uint8_t>((1U << RF_DR_LOW) | (1U << RF_DR_HIGH) | (0x03U << RF_PWR));
    if ((rf_setup & rf_mask) != expected_rf_setup_mask(m_data_rate, m_pa_level))
        return false;

    const uint8_t required_rx = static_cast<uint8_t>((1U << ERX_P1) | (m_broadcast_enabled ? (1U << ERX_P2) : 0U));
    if ((en_rx & required_rx) != required_rx)
        return false;

    if (memcmp(rx_p1, m_rx_addr, 5) != 0)
        return false;

    return true;
}

esp_err_t NRF24L01::reconfigure_locked()
{
    ESP_LOGW(TAG, "Radio register drift or timeout detected; soft-resetting nRF24L01+");
    Nrf24_powerDown(&m_dev);
    vTaskDelay(pdMS_TO_TICKS(2));
    const esp_err_t err = apply_full_config_locked();
    if (err == ESP_OK)
        m_reinit_count.fetch_add(1, std::memory_order_relaxed);
    return err;
}

bool NRF24L01::is_config_healthy()
{
    if (!m_initialized || m_radio_mutex == nullptr)
        return false;
    xSemaphoreTakeRecursive(m_radio_mutex, portMAX_DELAY);
    const bool ok = is_config_healthy_locked();
    xSemaphoreGiveRecursive(m_radio_mutex);
    return ok;
}

esp_err_t NRF24L01::reconfigure_if_needed()
{
    if (!m_initialized || m_radio_mutex == nullptr)
        return ESP_ERR_INVALID_STATE;
    xSemaphoreTakeRecursive(m_radio_mutex, portMAX_DELAY);
    esp_err_t err = ESP_OK;
    if (!is_config_healthy_locked())
        err = reconfigure_locked();
    xSemaphoreGiveRecursive(m_radio_mutex);
    return err;
}

esp_err_t NRF24L01::enable_broadcast_pipe(uint8_t broadcast_lsb)
{
    if (!m_initialized || m_radio_mutex == nullptr)
        return ESP_ERR_INVALID_STATE;

    xSemaphoreTakeRecursive(m_radio_mutex, portMAX_DELAY);
    m_broadcast_enabled = true;
    m_broadcast_lsb = broadcast_lsb;

    Nrf24_ceLow(&m_dev);
    Nrf24_addRADDR(&m_dev, 2, m_broadcast_lsb);
    Nrf24_enableNoAckFeature(&m_dev);
    apply_idle_rx_pipes_locked();
    Nrf24_configRegister(&m_dev, EN_AA, (1 << ENAA_P0) | (1 << ENAA_P1) | (1 << ENAA_P2));
    Nrf24_powerUpRx(&m_dev);

    xSemaphoreGiveRecursive(m_radio_mutex);
    return ESP_OK;
}

esp_err_t NRF24L01::set_channel(uint8_t channel)
{
    if (channel > MAX_RF_CHANNEL)
        return ESP_ERR_INVALID_ARG;
    if (!m_initialized || m_radio_mutex == nullptr)
        return ESP_ERR_INVALID_STATE;

    xSemaphoreTakeRecursive(m_radio_mutex, portMAX_DELAY);
    m_channel = channel;
    Nrf24_ceLow(&m_dev);
    m_dev.channel = channel;
    Nrf24_configRegister(&m_dev, RF_CH, channel);
    Nrf24_flushRx(&m_dev);
    flush_tx_locked();
    Nrf24_configRegister(&m_dev, REG_STATUS, (1 << RX_DR) | (1 << TX_DS) | (1 << MAX_RT));
    Nrf24_powerUpRx(&m_dev);
    xSemaphoreGiveRecursive(m_radio_mutex);
    return ESP_OK;
}

esp_err_t NRF24L01::set_rx_address(const uint8_t rx_addr[5])
{
    if (rx_addr == nullptr)
        return ESP_ERR_INVALID_ARG;
    if (!m_initialized || m_radio_mutex == nullptr)
        return ESP_ERR_INVALID_STATE;

    xSemaphoreTakeRecursive(m_radio_mutex, portMAX_DELAY);
    memcpy(m_rx_addr, rx_addr, 5);
    Nrf24_ceLow(&m_dev);
    const esp_err_t err = Nrf24_setRADDR(&m_dev, m_rx_addr);
    if (m_broadcast_enabled)
        Nrf24_addRADDR(&m_dev, 2, m_broadcast_lsb);
    apply_idle_rx_pipes_locked();
    Nrf24_flushRx(&m_dev);
    Nrf24_powerUpRx(&m_dev);
    xSemaphoreGiveRecursive(m_radio_mutex);
    return err;
}

esp_err_t NRF24L01::set_rf_params(rf24_datarate_e rate,
                                  rf24_pa_dbm_e pa,
                                  uint8_t retransmit_delay,
                                  uint8_t retransmit_count)
{
    if (static_cast<uint8_t>(rate) > static_cast<uint8_t>(RF24_250KBPS) ||
        static_cast<uint8_t>(pa) > static_cast<uint8_t>(RF24_PA_MAX) ||
        retransmit_delay > 15 || retransmit_count > 15)
        return ESP_ERR_INVALID_ARG;
    if (!m_initialized || m_radio_mutex == nullptr)
        return ESP_ERR_INVALID_STATE;

    xSemaphoreTakeRecursive(m_radio_mutex, portMAX_DELAY);
    m_data_rate = rate;
    m_pa_level = pa;
    m_retransmit_delay = retransmit_delay;
    m_retransmit_count = retransmit_count;
    const esp_err_t err = apply_full_config_locked();
    xSemaphoreGiveRecursive(m_radio_mutex);
    return err;
}

bool NRF24L01::start(data_received_callback_t callback, void * context)
{
    if (!callback || !m_initialized)
    {
        ESP_LOGE(TAG, "Cannot start receiver task before init and callback registration");
        return false;
    }
    m_on_data_received = callback;
    m_callback_context = context;

    BaseType_t task_created =
      xTaskCreate(&NRF24L01::task_wrapper, "nrf24_receiver_task", 4096, this, 5, &m_task_handle);
    return task_created == pdPASS;
}

void IRAM_ATTR NRF24L01::isr_handler(void * dev)
{
    auto * instance = static_cast<NRF24L01 *>(dev);
    uint32_t gpio_num = static_cast<uint32_t>(instance->m_irq_pin);
    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(instance->m_interrupt_queue, &gpio_num, &woken);
    portYIELD_FROM_ISR(woken);
}

void NRF24L01::task_wrapper(void * dev)
{
    static_cast<NRF24L01 *>(dev)->receiver_task();
}

void NRF24L01::drain_rx_locked()
{
    uint8_t buffer[32];
    while (Nrf24_dataReady(&m_dev))
    {
        const uint8_t pipe = Nrf24_getDataPipe(&m_dev);
        Nrf24_getData(&m_dev, buffer);

        const uint8_t payload_len = buffer[0];
        if (payload_len > 0 && payload_len <= 31 && m_on_data_received)
        {
            DataReceivedEvent event = {m_callback_context, &buffer[1], payload_len, pipe};
            m_on_data_received(&event);
        }
        else
        {
            ESP_LOGW(TAG, "Dropped packet — invalid length byte: %d", payload_len);
        }
    }
}

void NRF24L01::receiver_task()
{
    ESP_LOGI(TAG, "Receiver task started. Listening for data...");
    uint32_t io_num;
    uint8_t tx_buffer[32];

    while (true)
    {
        const bool signalled =
          xQueueReceive(m_interrupt_queue, &io_num, pdMS_TO_TICKS(10)) == pdTRUE;
        if (signalled && io_num != static_cast<uint32_t>(m_irq_pin))
            continue;

        if (xSemaphoreTakeRecursive(m_radio_mutex, pdMS_TO_TICKS(5)) != pdTRUE)
            continue;

        const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());

        if (m_dev.PTX)
        {
            const uint8_t tx_status = Nrf24_getStatus(&m_dev);
            if (tx_status & ((1 << TX_DS) | (1 << MAX_RT)))
            {
                if (tx_status & (1 << MAX_RT))
                    flush_tx_locked();
                apply_idle_rx_pipes_locked();
                Nrf24_powerUpRx(&m_dev);
            }
            else if (now_us >= m_tx_started_us && (now_us - m_tx_started_us) > ASYNC_TX_TIMEOUT_US)
            {
                reconfigure_locked();
            }
        }

        if (!m_dev.PTX)
        {
            drain_rx_locked();

            if (!signalled && (now_us - m_last_health_check_us) >= HEALTH_CHECK_INTERVAL_US)
            {
                m_last_health_check_us = now_us;
                if (!is_config_healthy_locked())
                    reconfigure_locked();
            }
        }

        if (!m_dev.PTX && xQueueReceive(m_tx_queue, &tx_buffer, 0) == pdTRUE)
        {
            // Short turnaround pause so the base station returns to receive mode.
            if (ROBOT_REPLY_DELAY_US > 0U)
                ets_delay_us(ROBOT_REPLY_DELAY_US);

            drain_rx_locked();

            Nrf24_ceLow(&m_dev);
            if (!m_last_tx_addr_valid || memcmp(m_last_tx_addr, m_tx_addr, 5) != 0)
            {
                Nrf24_setTADDR(&m_dev, m_tx_addr);
                memcpy(m_last_tx_addr, m_tx_addr, 5);
                m_last_tx_addr_valid = true;
            }
            uint8_t tx_pipes = static_cast<uint8_t>((1U << ERX_P0) | (1U << ERX_P1));
            if (m_broadcast_enabled)
                tx_pipes |= static_cast<uint8_t>(1U << ERX_P2);
            Nrf24_configRegister(&m_dev, EN_RXADDR, tx_pipes);
            m_tx_started_us = static_cast<uint64_t>(esp_timer_get_time());
            Nrf24_send(&m_dev, tx_buffer);
        }

        xSemaphoreGiveRecursive(m_radio_mutex);
    }
}

esp_err_t NRF24L01::send_raw(const uint8_t * data, uint8_t len)
{
    if (!m_initialized || m_tx_queue == nullptr)
        return ESP_ERR_INVALID_STATE;
    if (data == nullptr || len == 0 || len > 31)
    {
        ESP_LOGE(TAG, "send_raw: invalid length %d (must be 1..31)", len);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t frame[32] = {0};
    frame[0] = len;
    memcpy(&frame[1], data, len);

    if (xQueueSend(m_tx_queue, frame, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        ESP_LOGW(TAG, "TX queue full — dropping packet");
        return ESP_ERR_TIMEOUT;
    }

    uint32_t wake = static_cast<uint32_t>(m_irq_pin);
    xQueueSend(m_interrupt_queue, &wake, 0);
    return ESP_OK;
}

esp_err_t NRF24L01::send_to_sync(const uint8_t tx_addr[5],
                                 const uint8_t * data,
                                 uint8_t len,
                                 bool no_ack,
                                 TxResult * out_result,
                                 uint32_t timeout_ms)
{
    if (out_result != nullptr)
        *out_result = TxResult{};

    if (!m_initialized || m_radio_mutex == nullptr)
        return ESP_ERR_INVALID_STATE;
    if (tx_addr == nullptr || data == nullptr || len == 0 || len > 31)
        return ESP_ERR_INVALID_ARG;

    uint8_t frame[32] = {0};
    frame[0] = len;
    memcpy(&frame[1], data, len);

    xSemaphoreTakeRecursive(m_radio_mutex, portMAX_DELAY);

    if (m_dev.PTX)
        reconfigure_locked();

    // Never drop uplink replies sitting in the receive FIFO before switching to transmit.
    drain_rx_locked();

    const uint64_t t_start_us = static_cast<uint64_t>(esp_timer_get_time());

    Nrf24_ceLow(&m_dev);
    if (!m_last_tx_addr_valid || memcmp(m_last_tx_addr, tx_addr, 5) != 0)
    {
        if (Nrf24_setTADDR(&m_dev, const_cast<uint8_t *>(tx_addr)) != ESP_OK)
        {
            reconfigure_locked();
            xSemaphoreGiveRecursive(m_radio_mutex);
            return ESP_FAIL;
        }
        memcpy(m_last_tx_addr, tx_addr, 5);
        m_last_tx_addr_valid = true;
    }

    if (!no_ack)
    {
        uint8_t tx_pipes = static_cast<uint8_t>((1U << ERX_P0) | (1U << ERX_P1));
        if (m_broadcast_enabled)
            tx_pipes |= static_cast<uint8_t>(1U << ERX_P2);
        Nrf24_configRegister(&m_dev, EN_RXADDR, tx_pipes);
    }

    if (no_ack)
        Nrf24_sendNoAck(&m_dev, frame);
    else
        Nrf24_send(&m_dev, frame);

    const uint64_t deadline_us = t_start_us + static_cast<uint64_t>(timeout_ms) * 1000ULL;
    uint8_t status = 0;
    bool completed = false;

    while (static_cast<uint64_t>(esp_timer_get_time()) < deadline_us)
    {
        status = Nrf24_getStatus(&m_dev);
        if (status & ((1 << TX_DS) | (1 << MAX_RT)))
        {
            completed = true;
            break;
        }
        const uint64_t elapsed_us = static_cast<uint64_t>(esp_timer_get_time()) - t_start_us;
        if (elapsed_us < 1500ULL)
            ets_delay_us(50);
        else
            vTaskDelay(1);
    }

    const uint64_t elapsed_us = static_cast<uint64_t>(esp_timer_get_time()) - t_start_us;
    uint8_t observe_tx = 0;
    Nrf24_readRegister(&m_dev, OBSERVE_TX, &observe_tx, 1);

    TxResult result = {};
    result.retries = static_cast<uint8_t>(observe_tx & 0x0FU);
    result.tx_time_us = (elapsed_us > 65535ULL) ? 65535U : static_cast<uint16_t>(elapsed_us);

    if (!completed)
    {
        result.status = TxStatus::RADIO_ERR;
        reconfigure_locked();
    }
    else
    {
        if (status & (1 << MAX_RT))
        {
            flush_tx_locked();
            result.status = TxStatus::MAX_RETRIES;
        }
        else if (no_ack)
        {
            result.status = TxStatus::NO_ACK_SENT;
        }
        else
        {
            result.status = TxStatus::OK;
        }
        apply_idle_rx_pipes_locked();
        Nrf24_powerUpRx(&m_dev);
    }

    m_last_health_check_us = static_cast<uint64_t>(esp_timer_get_time());
    xSemaphoreGiveRecursive(m_radio_mutex);

    if (out_result != nullptr)
        *out_result = result;
    return completed ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t NRF24L01::scan_spectrum(uint8_t start_ch,
                                  uint8_t end_ch,
                                  uint8_t samples_per_ch,
                                  uint8_t * out_busy_pct)
{
    if (!m_initialized || m_radio_mutex == nullptr)
        return ESP_ERR_INVALID_STATE;
    if (out_busy_pct == nullptr || start_ch > MAX_RF_CHANNEL || end_ch > MAX_RF_CHANNEL ||
        start_ch > end_ch || samples_per_ch == 0)
        return ESP_ERR_INVALID_ARG;

    xSemaphoreTakeRecursive(m_radio_mutex, portMAX_DELAY);

    const uint8_t saved_channel = m_channel;
    Nrf24_ceLow(&m_dev);
    Nrf24_configRegister(&m_dev, CONFIG, mirf_CONFIG | ((1 << PWR_UP) | (1 << PRIM_RX)));

    const uint8_t count = static_cast<uint8_t>(end_ch - start_ch + 1);
    for (uint8_t i = 0; i < count; ++i)
    {
        const uint8_t ch = static_cast<uint8_t>(start_ch + i);
        Nrf24_ceLow(&m_dev);
        Nrf24_configRegister(&m_dev, RF_CH, ch);

        uint16_t hits = 0;
        for (uint8_t s = 0; s < samples_per_ch; ++s)
        {
            Nrf24_ceHi(&m_dev);
            ets_delay_us(170);  // 130 us PLL settle + 40 us RPD latch
            uint8_t rpd = 0;
            Nrf24_readRegister(&m_dev, RPD, &rpd, 1);
            Nrf24_ceLow(&m_dev);
            if (rpd & 0x01U)
                ++hits;
        }
        out_busy_pct[i] = static_cast<uint8_t>((hits * 100U + samples_per_ch / 2U) / samples_per_ch);
        // Yield periodically so watchdog stays fed during full 126-channel sweeps.
        if ((i & 0x0FU) == 0x0FU)
            vTaskDelay(1);
    }

    // Restore original channel and clean receive state.
    Nrf24_ceLow(&m_dev);
    m_dev.channel = saved_channel;
    Nrf24_configRegister(&m_dev, RF_CH, saved_channel);
    Nrf24_flushRx(&m_dev);
    Nrf24_configRegister(&m_dev, REG_STATUS, (1 << RX_DR) | (1 << TX_DS) | (1 << MAX_RT));
    Nrf24_powerUpRx(&m_dev);
    m_last_health_check_us = static_cast<uint64_t>(esp_timer_get_time());

    xSemaphoreGiveRecursive(m_radio_mutex);
    return ESP_OK;
}

bool NRF24L01::is_ready() const
{
    return m_initialized;
}

uint8_t NRF24L01::get_channel() const
{
    return m_channel;
}

rf24_datarate_e NRF24L01::get_data_rate() const
{
    return m_data_rate;
}

rf24_pa_dbm_e NRF24L01::get_pa_level() const
{
    return m_pa_level;
}

uint8_t NRF24L01::get_retransmit_delay() const
{
    return m_retransmit_delay;
}

uint8_t NRF24L01::get_retransmit_count() const
{
    return m_retransmit_count;
}

uint16_t NRF24L01::get_reinit_count() const
{
    return m_reinit_count.load(std::memory_order_relaxed);
}
