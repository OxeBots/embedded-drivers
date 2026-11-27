#ifndef _ESP_NOW_DEV_H_
#define _ESP_NOW_DEV_H_

#include <esp_now.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

class ESP_NOW_Dev
{
   public:
    static constexpr uint8_t BROADCAST[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    using PacketHandler = void (*)(const uint8_t * payload,
                                   uint8_t length,
                                   const uint8_t * source,
                                   uint64_t received_at_us);

    static ESP_NOW_Dev & get_instance();

    esp_err_t init(uint8_t channel);
    esp_err_t start(bool accept_broadcast, PacketHandler handler);
    bool is_receiving() const;
    esp_err_t send(const uint8_t * destination, const uint8_t * payload, size_t length);

   private:
    ESP_NOW_Dev() = default;

    struct Packet
    {
        uint64_t received_at_us;
        uint8_t source[ESP_NOW_ETH_ALEN];
        uint8_t length;
        uint8_t payload[ESP_NOW_MAX_DATA_LEN];
    };

    static void on_receive(const esp_now_recv_info_t * info, const uint8_t * data, int length);
    static void receive_task(void * context);

    QueueHandle_t m_queue = nullptr;
    PacketHandler m_handler = nullptr;
    bool m_accept_broadcast = false;
};

#endif  // _ESP_NOW_DEV_H_
