/* ESP-01S WiFi module: UART4 AT/MQTT driver for ESP8266 AT MQTT firmware. */
#ifndef __ESP01S_H__
#define __ESP01S_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define ESP01S_MQTT_LINK_ID 0U

typedef enum
{
  ESP01S_STATUS_OK = 0,
  ESP01S_STATUS_ERROR,
  ESP01S_STATUS_TIMEOUT,
  ESP01S_STATUS_INVALID_PARAM,
  ESP01S_STATUS_BUSY
} ESP01S_Status_t;

typedef struct
{
  char topic[64];
  char payload[128];
  uint32_t last_update_tick;
} ESP01S_MqttMessage_t;

/* UART4 is wired to ESP-01S: 9600 8N1, PC10 TX, PC11 RX. */
void ESP01S_Init(void);

/* Create the auto WiFi/MQTT connection task. */
void ESP01S_StartTask(void);

/* Drain received lines and update the latest MQTT subscription message cache. */
void ESP01S_Poll(void);

/* Generic AT command helper. The command string must not include trailing CRLF. */
ESP01S_Status_t ESP01S_SendCommand(const char *command, uint32_t timeout_ms);

ESP01S_Status_t ESP01S_Test(uint32_t timeout_ms);
ESP01S_Status_t ESP01S_Restart(uint32_t timeout_ms);
ESP01S_Status_t ESP01S_SetEcho(uint8_t enabled);
ESP01S_Status_t ESP01S_SetStationMode(void);
ESP01S_Status_t ESP01S_JoinAP(const char *ssid, const char *password, uint32_t timeout_ms);
ESP01S_Status_t ESP01S_MQTTUserConfig(const char *client_id, const char *username, const char *password);
ESP01S_Status_t ESP01S_MQTTConnect(const char *host, uint16_t port, uint16_t keepalive_seconds, uint32_t timeout_ms);
ESP01S_Status_t ESP01S_MQTTPublish(const char *topic, const char *payload, uint8_t qos, uint8_t retain);
ESP01S_Status_t ESP01S_MQTTSubscribe(const char *topic, uint8_t qos);
ESP01S_Status_t ESP01S_MQTTDisconnect(void);

uint8_t ESP01S_ReadLatestMessage(ESP01S_MqttMessage_t *message);
uint8_t ESP01S_IsWiFiConnected(void);
uint8_t ESP01S_IsMQTTConnected(void);

/* Called from NVIC.c HAL UART callback dispatchers. */
void ESP01S_RxCpltCallbackFromISR(UART_HandleTypeDef *huart);
void ESP01S_ErrorCallbackFromISR(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* __ESP01S_H__ */
