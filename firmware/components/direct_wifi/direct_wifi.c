#include "direct_wifi.h"

static const char *WIFI_TAG = "Direct_Wifi";

void (*wifi_recv_cb)(uint8_t src_mac[6], uint8_t *data, int len) = NULL;

esp_now_peer_info_t peer;
//static uint8_t s_example_broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
//static uint8_t s_example_broadcast_mac[ESP_NOW_ETH_ALEN] = {0x60, 0x01, 0x94, 0x2D, 0xA7, 0xD9};  //esp8266
//static uint8_t s_example_broadcast_mac[ESP_NOW_ETH_ALEN] = {0x98, 0x83, 0x89, 0x91, 0x45, 0xb3};  //samsung laptop
static uint8_t s_example_broadcast_mac[ESP_NOW_ETH_ALEN] = {0xB4, 0xB0, 0x24, 0xBC, 0xF1, 0xE9};  //tp-link TL-WN722N

void wifi_send_data(uint8_t *data, int len)
{
  //printf("Sending data to PC.\n");
  esp_now_send(peer.peer_addr, data, len);
}

static void wifi_recv_func(uint8_t src_mac[6], uint8_t *data, int len)
{
  if (wifi_recv_cb == NULL)
  {
    ESP_LOGW(WIFI_TAG, "Wifi frame received but no callback function is set on received...");
  }
  else
  {
    wifi_recv_cb(src_mac, data, len);
  }
}

static void wifi_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  if (status != ESP_OK)
  {
    //error_send++;
  }
}

void wifi_attach_recv_cb(void (*cb)(uint8_t src_mac[6], uint8_t *data, int len))
{
  wifi_recv_cb = cb;
}

void wifi_detach_recv_cb()
{
  wifi_recv_cb = NULL;
}

void wifi_init()
{
  /* Init NVS */
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  /* Init WiFi */
  tcpip_adapter_init();
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  cfg.ampdu_tx_enable = 0;
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  wifi_country_t country = {.cc = "JP", .schan = 1, .nchan = 14, .policy = WIFI_COUNTRY_POLICY_MANUAL};
  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_ERROR_CHECK(esp_wifi_set_country(&country));
  ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));
  ESP_ERROR_CHECK(esp_wifi_internal_set_fix_rate(ESP_IF_WIFI_STA, true, CONFIG_WIFI_DATARATE));

  /* Init ESPNOW */
  ESP_ERROR_CHECK(esp_now_init());
  ESP_ERROR_CHECK(esp_now_register_recv_cb(wifi_recv_func));

  memset(&peer, 0, sizeof(esp_now_peer_info_t));
  peer.channel = 1;
  peer.ifidx = ESP_IF_WIFI_STA;
  peer.encrypt = false;
  memcpy(peer.peer_addr, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
  ESP_ERROR_CHECK(esp_now_add_peer(&peer));
}

void wifi_deinit_func()
{
  ESP_ERROR_CHECK(esp_now_deinit());
  ESP_ERROR_CHECK(esp_wifi_stop());
  ESP_ERROR_CHECK(esp_wifi_deinit());
}

void wifi_change_channel(uint8_t wifi_channel)
{
  ESP_ERROR_CHECK(esp_wifi_set_channel(wifi_channel, WIFI_SECOND_CHAN_NONE));
  peer.channel = wifi_channel;
  ESP_ERROR_CHECK(esp_now_mod_peer(&peer));
}
