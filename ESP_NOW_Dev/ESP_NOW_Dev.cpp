#include "ESP_NOW_Dev.h"

#include <esp_check.h>
#include <esp_event.h>
#include <esp_timer.h>
#include <esp_wifi.h>
#include <freertos/task.h>

#include <cstring>

static const char * TAG = "ESP_NOW_DEV";

ESP_NOW_Dev & ESP_NOW_Dev::get_instance()
{
    static ESP_NOW_Dev link;
    return link;
}

esp_err_t ESP_NOW_Dev::init(uint8_t channel)
{
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "Create Wi-Fi event loop");
    wifi_init_config_t wifi = WIFI_INIT_CONFIG_DEFAULT();
    wifi.nvs_enable = 0;
    ESP_RETURN_ON_ERROR(esp_wifi_init(&wifi), TAG, "Initialize Wi-Fi");
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG, "Set Wi-Fi storage");
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "Set Wi-Fi station mode");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "Start Wi-Fi");
    ESP_RETURN_ON_ERROR(esp_wifi_set_ps(WIFI_PS_NONE), TAG, "Disable Wi-Fi power save");
    ESP_RETURN_ON_ERROR(esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE), TAG, "Set channel");
    ESP_RETURN_ON_ERROR(esp_now_init(), TAG, "Initialize ESP-NOW");
    return ESP_OK;
}

esp_err_t ESP_NOW_Dev::start(bool accept_broadcast, PacketHandler handler)
{
    if (m_queue != nullptr)
        return ESP_ERR_INVALID_STATE;
    if (handler == nullptr)
        return ESP_ERR_INVALID_ARG;

    m_queue = xQueueCreate(16, sizeof(Packet));
    if (m_queue == nullptr)
        return ESP_ERR_NO_MEM;

    m_handler = handler;
    m_accept_broadcast = accept_broadcast;
    TaskHandle_t task = nullptr;
    if (xTaskCreate(receive_task, "esp_now_rx", 6144, this, 5, &task) != pdPASS)
    {
        vQueueDelete(m_queue);
        m_queue = nullptr;
        return ESP_ERR_NO_MEM;
    }

    const esp_err_t result = esp_now_register_recv_cb(on_receive);
    if (result != ESP_OK)
    {
        vTaskDelete(task);
        vQueueDelete(m_queue);
        m_queue = nullptr;
    }
    return result;
}

bool ESP_NOW_Dev::is_receiving() const
{
    return m_queue != nullptr;
}

esp_err_t ESP_NOW_Dev::send(const uint8_t * destination, const uint8_t * payload, size_t length)
{
    if (destination == nullptr || payload == nullptr || length == 0 ||
        length > ESP_NOW_MAX_DATA_LEN)
        return ESP_ERR_INVALID_ARG;

    if (!esp_now_is_peer_exist(destination))
    {
        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, destination, sizeof(peer.peer_addr));
        peer.channel = 0;
        peer.ifidx = WIFI_IF_STA;
        ESP_RETURN_ON_ERROR(esp_now_add_peer(&peer), TAG, "Add ESP-NOW peer");
    }
    return esp_now_send(destination, payload, length);
}

void ESP_NOW_Dev::on_receive(const esp_now_recv_info_t * info, const uint8_t * data, int length)
{
    auto & link = get_instance();
    const bool broadcast = memcmp(info->des_addr, BROADCAST, ESP_NOW_ETH_ALEN) == 0;
    if ((broadcast && !link.m_accept_broadcast) || length <= 0 || length > ESP_NOW_MAX_DATA_LEN)
        return;

    Packet packet = {};
    packet.received_at_us = static_cast<uint64_t>(esp_timer_get_time());
    memcpy(packet.source, info->src_addr, sizeof(packet.source));
    packet.length = static_cast<uint8_t>(length);
    memcpy(packet.payload, data, packet.length);
    xQueueSend(link.m_queue, &packet, 0);
}

void ESP_NOW_Dev::receive_task(void * context)
{
    auto & link = *static_cast<ESP_NOW_Dev *>(context);
    Packet packet;
    while (true)
    {
        if (xQueueReceive(link.m_queue, &packet, portMAX_DELAY) == pdTRUE)
            link.m_handler(packet.payload, packet.length, packet.source, packet.received_at_us);
    }
}
