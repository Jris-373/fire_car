/* ESP-01S WiFi module implementation:
 * - UART4 interrupt receives ESP8266 AT response lines;
 * - blocking command helpers send ESP-AT MQTT commands;
 * - subscription notifications are cached for application polling.
 */
#include "ESP01S.h"

#include <stdio.h>
#include <string.h>

#include "cmsis_os.h"
#include "usart.h"

#define ESP01S_LINE_BUFFER_SIZE       192U
#define ESP01S_LINE_QUEUE_DEPTH       8U
#define ESP01S_COMMAND_BUFFER_SIZE    384U
#define ESP01S_ESCAPED_PAYLOAD_SIZE   256U
#define ESP01S_UART_TX_TIMEOUT_MS     200U
#define ESP01S_DEFAULT_TIMEOUT_MS     1000U
#define ESP01S_WIFI_JOIN_TIMEOUT_MS   15000U
#define ESP01S_MQTT_CONN_TIMEOUT_MS   5000U
#define ESP01S_RECONNECT_DELAY_MS     5000U
#define ESP01S_TASK_POLL_PERIOD_MS    20U

#define ESP01S_WIFI_SSID              "fire_car"
#define ESP01S_WIFI_PASSWORD          "zccznb123"
#define ESP01S_MQTT_CLIENT_ID         "fire_car_esp01s"
#define ESP01S_MQTT_USERNAME          ""
#define ESP01S_MQTT_PASSWORD          ""
#define ESP01S_MQTT_HOST              "192.168.137.1"
#define ESP01S_MQTT_PORT              1883U
#define ESP01S_MQTT_KEEPALIVE_SECONDS 60U
#define ESP01S_MQTT_SUB_TOPIC         "fire_car/down"
#define ESP01S_MQTT_COMPAT_SUB_TOPIC  "fire_car/mission"
#define ESP01S_MQTT_SUB_QOS           0U
#define ESP01S_MQTT_ONLINE_TOPIC      "fire_car/status"
#define ESP01S_MQTT_ONLINE_PAYLOAD    "online"

static uint8_t esp01s_rx_byte = 0U;
static char esp01s_rx_line[ESP01S_LINE_BUFFER_SIZE];
static volatile uint16_t esp01s_rx_line_length = 0U;

static char esp01s_line_queue[ESP01S_LINE_QUEUE_DEPTH][ESP01S_LINE_BUFFER_SIZE];
static volatile uint8_t esp01s_line_head = 0U;
static volatile uint8_t esp01s_line_tail = 0U;
static volatile uint8_t esp01s_command_busy = 0U;

static ESP01S_MqttMessage_t esp01s_latest_message;
static volatile uint8_t esp01s_message_available = 0U;
static volatile uint8_t esp01s_wifi_connected = 0U;
static volatile uint8_t esp01s_mqtt_connected = 0U;

static osThreadId_t esp01sTaskHandle = NULL;
static const osThreadAttr_t esp01sTaskAttributes = {
  .name = "esp01sTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t)osPriorityNormal,
};

static void ESP01S_Task(void *argument);

static void ESP01S_ArmReception(void)
{
  (void)HAL_UART_Receive_IT(&huart4, &esp01s_rx_byte, 1U);
}

static void ESP01S_RestoreIRQ(uint32_t primask)
{
  if (primask == 0U)
  {
    __enable_irq();
  }
}

static void ESP01S_ResetLineQueue(void)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  esp01s_line_head = 0U;
  esp01s_line_tail = 0U;
  esp01s_rx_line_length = 0U;
  ESP01S_RestoreIRQ(primask);
}

static uint8_t ESP01S_TryLockCommand(void)
{
  uint8_t locked = 0U;
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  if (esp01s_command_busy == 0U)
  {
    esp01s_command_busy = 1U;
    locked = 1U;
  }
  ESP01S_RestoreIRQ(primask);

  return locked;
}

static void ESP01S_UnlockCommand(void)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  esp01s_command_busy = 0U;
  ESP01S_RestoreIRQ(primask);
}

static void ESP01S_CopyBounded(char *destination, uint16_t destination_size, const char *source, uint16_t source_length)
{
  uint16_t copy_length = source_length;

  if ((destination == NULL) || (destination_size == 0U))
  {
    return;
  }

  if (source == NULL)
  {
    destination[0] = '\0';
    return;
  }

  if (copy_length >= destination_size)
  {
    copy_length = (uint16_t)(destination_size - 1U);
  }

  if (copy_length > 0U)
  {
    (void)memcpy(destination, source, copy_length);
  }
  destination[copy_length] = '\0';
}

static void ESP01S_SetConnectionState(uint8_t wifi_connected, uint8_t mqtt_connected)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  esp01s_wifi_connected = wifi_connected;
  esp01s_mqtt_connected = mqtt_connected;
  ESP01S_RestoreIRQ(primask);
}

static void ESP01S_PushLineFromISR(void)
{
  uint8_t next_head;
  uint16_t copy_length = esp01s_rx_line_length;

  if (copy_length == 0U)
  {
    return;
  }

  if (copy_length >= ESP01S_LINE_BUFFER_SIZE)
  {
    copy_length = (uint16_t)(ESP01S_LINE_BUFFER_SIZE - 1U);
  }

  next_head = (uint8_t)((esp01s_line_head + 1U) % ESP01S_LINE_QUEUE_DEPTH);
  if (next_head == esp01s_line_tail)
  {
    esp01s_line_tail = (uint8_t)((esp01s_line_tail + 1U) % ESP01S_LINE_QUEUE_DEPTH);
  }

  ESP01S_CopyBounded(esp01s_line_queue[esp01s_line_head], ESP01S_LINE_BUFFER_SIZE, esp01s_rx_line, copy_length);
  esp01s_line_head = next_head;
  esp01s_rx_line_length = 0U;
}

static void ESP01S_ProcessByteFromISR(uint8_t data)
{
  if (data == '\r')
  {
    return;
  }

  if (data == '\n')
  {
    ESP01S_PushLineFromISR();
    return;
  }

  if (esp01s_rx_line_length < (ESP01S_LINE_BUFFER_SIZE - 1U))
  {
    esp01s_rx_line[esp01s_rx_line_length] = (char)data;
    esp01s_rx_line_length++;
  }
  else
  {
    esp01s_rx_line_length = 0U;
  }
}

static uint8_t ESP01S_PopLine(char *line, uint16_t line_size)
{
  uint16_t index;
  uint8_t queue_index;
  uint32_t primask;

  if ((line == NULL) || (line_size == 0U))
  {
    return 0U;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  if (esp01s_line_tail == esp01s_line_head)
  {
    ESP01S_RestoreIRQ(primask);
    line[0] = '\0';
    return 0U;
  }

  queue_index = esp01s_line_tail;
  esp01s_line_tail = (uint8_t)((esp01s_line_tail + 1U) % ESP01S_LINE_QUEUE_DEPTH);

  for (index = 0U; index < (line_size - 1U); index++)
  {
    line[index] = esp01s_line_queue[queue_index][index];
    if (line[index] == '\0')
    {
      break;
    }
  }
  line[line_size - 1U] = '\0';

  ESP01S_RestoreIRQ(primask);
  return 1U;
}

static void ESP01S_DelayMs(uint32_t delay_ms)
{
  if (delay_ms == 0U)
  {
    return;
  }

  if (osKernelGetState() == osKernelRunning)
  {
    osDelay(delay_ms);
  }
  else
  {
    HAL_Delay(delay_ms);
  }
}

static uint8_t ESP01S_IsSafeATString(const char *value)
{
  const char *cursor = value;

  if (value == NULL)
  {
    return 0U;
  }

  while (*cursor != '\0')
  {
    if ((*cursor == '"') || (*cursor == '\\') || (*cursor == '\r') || (*cursor == '\n'))
    {
      return 0U;
    }
    cursor++;
  }

  return 1U;
}

static uint8_t ESP01S_EscapeATString(const char *value, char *escaped, uint16_t escaped_size)
{
  uint16_t write_index = 0U;
  const char *cursor = value;

  if ((value == NULL) || (escaped == NULL) || (escaped_size == 0U))
  {
    return 0U;
  }

  while (*cursor != '\0')
  {
    if ((*cursor == '\r') || (*cursor == '\n'))
    {
      return 0U;
    }

    if ((*cursor == '"') || (*cursor == '\\'))
    {
      if ((write_index + 2U) >= escaped_size)
      {
        return 0U;
      }
      escaped[write_index++] = '\\';
      escaped[write_index++] = *cursor;
    }
    else
    {
      if ((write_index + 1U) >= escaped_size)
      {
        return 0U;
      }
      escaped[write_index++] = *cursor;
    }

    cursor++;
  }

  escaped[write_index] = '\0';
  return 1U;
}

static uint8_t ESP01S_LineHasToken(const char *line, const char *token)
{
  if ((line == NULL) || (token == NULL))
  {
    return 0U;
  }

  return (strstr(line, token) != NULL) ? 1U : 0U;
}

static void ESP01S_ParseMqttSubscribeLine(const char *line)
{
  const char *topic_start;
  const char *topic_end;
  const char *length_start;
  const char *payload_start;
  uint16_t topic_length;
  uint16_t payload_length;
  uint32_t primask;

  if (strncmp(line, "+MQTTSUBRECV:", 13U) != 0)
  {
    return;
  }

  topic_start = strchr(line, '"');
  if (topic_start == NULL)
  {
    return;
  }
  topic_start++;

  topic_end = strchr(topic_start, '"');
  if (topic_end == NULL)
  {
    return;
  }

  length_start = strchr(topic_end + 1, ',');
  if (length_start == NULL)
  {
    return;
  }
  length_start++;

  payload_start = strchr(length_start, ',');
  if (payload_start == NULL)
  {
    return;
  }
  payload_start++;

  topic_length = (uint16_t)(topic_end - topic_start);
  payload_length = (uint16_t)strlen(payload_start);

  primask = __get_PRIMASK();
  __disable_irq();
  ESP01S_CopyBounded(esp01s_latest_message.topic, sizeof(esp01s_latest_message.topic), topic_start, topic_length);
  ESP01S_CopyBounded(esp01s_latest_message.payload, sizeof(esp01s_latest_message.payload), payload_start, payload_length);
  esp01s_latest_message.last_update_tick = HAL_GetTick();
  esp01s_message_available = 1U;
  ESP01S_RestoreIRQ(primask);
}

static void ESP01S_ParseAsyncLine(const char *line)
{
  if ((ESP01S_LineHasToken(line, "WIFI DISCONNECT") != 0U) ||
      (ESP01S_LineHasToken(line, "WIFI LOST") != 0U))
  {
    ESP01S_SetConnectionState(0U, 0U);
  }
  else if (ESP01S_LineHasToken(line, "WIFI GOT IP") != 0U)
  {
    ESP01S_SetConnectionState(1U, esp01s_mqtt_connected);
  }
  else if (ESP01S_LineHasToken(line, "+MQTTCONNECTED") != 0U)
  {
    ESP01S_SetConnectionState(esp01s_wifi_connected, 1U);
  }
  else if (ESP01S_LineHasToken(line, "+MQTTDISCONNECTED") != 0U)
  {
    ESP01S_SetConnectionState(esp01s_wifi_connected, 0U);
  }

  ESP01S_ParseMqttSubscribeLine(line);
}

static ESP01S_Status_t ESP01S_ConnectConfiguredNetwork(void)
{
  ESP01S_Status_t status;

  ESP01S_SetConnectionState(0U, 0U);

  status = ESP01S_Test(ESP01S_DEFAULT_TIMEOUT_MS);
  if (status != ESP01S_STATUS_OK)
  {
    (void)ESP01S_Restart(5000U);
    status = ESP01S_Test(ESP01S_DEFAULT_TIMEOUT_MS);
    if (status != ESP01S_STATUS_OK)
    {
      return status;
    }
  }

  status = ESP01S_SetEcho(0U);
  if (status != ESP01S_STATUS_OK)
  {
    return status;
  }

  status = ESP01S_SetStationMode();
  if (status != ESP01S_STATUS_OK)
  {
    return status;
  }

  status = ESP01S_JoinAP(ESP01S_WIFI_SSID, ESP01S_WIFI_PASSWORD, ESP01S_WIFI_JOIN_TIMEOUT_MS);
  if (status != ESP01S_STATUS_OK)
  {
    ESP01S_SetConnectionState(0U, 0U);
    return status;
  }
  ESP01S_SetConnectionState(1U, 0U);

  status = ESP01S_MQTTUserConfig(ESP01S_MQTT_CLIENT_ID,
                                 ESP01S_MQTT_USERNAME,
                                 ESP01S_MQTT_PASSWORD);
  if (status != ESP01S_STATUS_OK)
  {
    return status;
  }

  status = ESP01S_MQTTConnect(ESP01S_MQTT_HOST,
                              ESP01S_MQTT_PORT,
                              ESP01S_MQTT_KEEPALIVE_SECONDS,
                              ESP01S_MQTT_CONN_TIMEOUT_MS);
  if (status != ESP01S_STATUS_OK)
  {
    ESP01S_SetConnectionState(1U, 0U);
    return status;
  }
  ESP01S_SetConnectionState(1U, 1U);

  status = ESP01S_MQTTSubscribe(ESP01S_MQTT_SUB_TOPIC, ESP01S_MQTT_SUB_QOS);
  if (status != ESP01S_STATUS_OK)
  {
    ESP01S_SetConnectionState(1U, 0U);
    return status;
  }

  if (strcmp(ESP01S_MQTT_COMPAT_SUB_TOPIC, ESP01S_MQTT_SUB_TOPIC) != 0)
  {
    status = ESP01S_MQTTSubscribe(ESP01S_MQTT_COMPAT_SUB_TOPIC, ESP01S_MQTT_SUB_QOS);
    if (status != ESP01S_STATUS_OK)
    {
      ESP01S_SetConnectionState(1U, 0U);
      return status;
    }
  }

  (void)ESP01S_MQTTPublish(ESP01S_MQTT_ONLINE_TOPIC, ESP01S_MQTT_ONLINE_PAYLOAD, 0U, 0U);

  return ESP01S_STATUS_OK;
}

static ESP01S_Status_t ESP01S_WaitForResult(const char *ok_token, uint32_t timeout_ms)
{
  char line[ESP01S_LINE_BUFFER_SIZE];
  uint32_t start_tick = HAL_GetTick();
  const char *success_token = (ok_token == NULL) ? "OK" : ok_token;

  while ((HAL_GetTick() - start_tick) < timeout_ms)
  {
    while (ESP01S_PopLine(line, sizeof(line)) != 0U)
    {
      ESP01S_ParseAsyncLine(line);

      if (ESP01S_LineHasToken(line, success_token) != 0U)
      {
        return ESP01S_STATUS_OK;
      }

      if ((ESP01S_LineHasToken(line, "ERROR") != 0U) ||
          (ESP01S_LineHasToken(line, "FAIL") != 0U))
      {
        return ESP01S_STATUS_ERROR;
      }
    }

    ESP01S_DelayMs(1U);
  }

  return ESP01S_STATUS_TIMEOUT;
}

static ESP01S_Status_t ESP01S_RunCommandUnlocked(const char *command, const char *ok_token, uint32_t timeout_ms)
{
  uint16_t command_length;
  HAL_StatusTypeDef tx_status;
  static const uint8_t crlf[] = {'\r', '\n'};

  if (command == NULL)
  {
    return ESP01S_STATUS_INVALID_PARAM;
  }

  command_length = (uint16_t)strlen(command);
  if (command_length == 0U)
  {
    return ESP01S_STATUS_INVALID_PARAM;
  }

  ESP01S_ResetLineQueue();

  tx_status = HAL_UART_Transmit(&huart4, (uint8_t *)command, command_length, ESP01S_UART_TX_TIMEOUT_MS);
  if (tx_status == HAL_OK)
  {
    tx_status = HAL_UART_Transmit(&huart4, (uint8_t *)crlf, sizeof(crlf), ESP01S_UART_TX_TIMEOUT_MS);
  }

  if (tx_status != HAL_OK)
  {
    return ESP01S_STATUS_ERROR;
  }

  return ESP01S_WaitForResult(ok_token, timeout_ms);
}

static ESP01S_Status_t ESP01S_RunCommand(const char *command, const char *ok_token, uint32_t timeout_ms)
{
  ESP01S_Status_t status;

  if (ESP01S_TryLockCommand() == 0U)
  {
    return ESP01S_STATUS_BUSY;
  }

  status = ESP01S_RunCommandUnlocked(command, ok_token, timeout_ms);
  ESP01S_UnlockCommand();

  return status;
}

void ESP01S_Init(void)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  memset(&esp01s_latest_message, 0, sizeof(esp01s_latest_message));
  esp01s_message_available = 0U;
  esp01s_command_busy = 0U;
  esp01s_wifi_connected = 0U;
  esp01s_mqtt_connected = 0U;
  ESP01S_RestoreIRQ(primask);

  ESP01S_ResetLineQueue();
  ESP01S_ArmReception();
}

void ESP01S_StartTask(void)
{
  if (esp01sTaskHandle == NULL)
  {
    esp01sTaskHandle = osThreadNew(ESP01S_Task, NULL, &esp01sTaskAttributes);
  }
}

void ESP01S_Poll(void)
{
  char line[ESP01S_LINE_BUFFER_SIZE];

  if (esp01s_command_busy != 0U)
  {
    return;
  }

  while (ESP01S_PopLine(line, sizeof(line)) != 0U)
  {
    ESP01S_ParseAsyncLine(line);
  }
}

ESP01S_Status_t ESP01S_SendCommand(const char *command, uint32_t timeout_ms)
{
  if (timeout_ms == 0U)
  {
    timeout_ms = ESP01S_DEFAULT_TIMEOUT_MS;
  }

  return ESP01S_RunCommand(command, "OK", timeout_ms);
}

ESP01S_Status_t ESP01S_Test(uint32_t timeout_ms)
{
  if (timeout_ms == 0U)
  {
    timeout_ms = ESP01S_DEFAULT_TIMEOUT_MS;
  }

  return ESP01S_RunCommand("AT", "OK", timeout_ms);
}

ESP01S_Status_t ESP01S_Restart(uint32_t timeout_ms)
{
  if (timeout_ms == 0U)
  {
    timeout_ms = 5000U;
  }

  return ESP01S_RunCommand("AT+RST", "ready", timeout_ms);
}

ESP01S_Status_t ESP01S_SetEcho(uint8_t enabled)
{
  return ESP01S_RunCommand((enabled == 0U) ? "ATE0" : "ATE1", "OK", ESP01S_DEFAULT_TIMEOUT_MS);
}

ESP01S_Status_t ESP01S_SetStationMode(void)
{
  return ESP01S_RunCommand("AT+CWMODE=1", "OK", ESP01S_DEFAULT_TIMEOUT_MS);
}

ESP01S_Status_t ESP01S_JoinAP(const char *ssid, const char *password, uint32_t timeout_ms)
{
  char command[ESP01S_COMMAND_BUFFER_SIZE];
  int written_length;

  if ((ESP01S_IsSafeATString(ssid) == 0U) || (ESP01S_IsSafeATString(password) == 0U))
  {
    return ESP01S_STATUS_INVALID_PARAM;
  }

  written_length = snprintf(command, sizeof(command), "AT+CWJAP=\"%s\",\"%s\"", ssid, password);
  if ((written_length <= 0) || (written_length >= (int)sizeof(command)))
  {
    return ESP01S_STATUS_INVALID_PARAM;
  }

  if (timeout_ms == 0U)
  {
    timeout_ms = ESP01S_WIFI_JOIN_TIMEOUT_MS;
  }

  return ESP01S_RunCommand(command, "OK", timeout_ms);
}

ESP01S_Status_t ESP01S_MQTTUserConfig(const char *client_id, const char *username, const char *password)
{
  char command[ESP01S_COMMAND_BUFFER_SIZE];
  const char *safe_username = (username == NULL) ? "" : username;
  const char *safe_password = (password == NULL) ? "" : password;
  int written_length;

  if ((ESP01S_IsSafeATString(client_id) == 0U) ||
      (ESP01S_IsSafeATString(safe_username) == 0U) ||
      (ESP01S_IsSafeATString(safe_password) == 0U))
  {
    return ESP01S_STATUS_INVALID_PARAM;
  }

  written_length = snprintf(command,
                            sizeof(command),
                            "AT+MQTTUSERCFG=%u,1,\"%s\",\"%s\",\"%s\",0,0,\"\"",
                            (unsigned int)ESP01S_MQTT_LINK_ID,
                            client_id,
                            safe_username,
                            safe_password);
  if ((written_length <= 0) || (written_length >= (int)sizeof(command)))
  {
    return ESP01S_STATUS_INVALID_PARAM;
  }

  return ESP01S_RunCommand(command, "OK", ESP01S_DEFAULT_TIMEOUT_MS);
}

ESP01S_Status_t ESP01S_MQTTConnect(const char *host, uint16_t port, uint16_t keepalive_seconds, uint32_t timeout_ms)
{
  char command[ESP01S_COMMAND_BUFFER_SIZE];
  int written_length;

  if ((ESP01S_IsSafeATString(host) == 0U) || (port == 0U) || (keepalive_seconds == 0U))
  {
    return ESP01S_STATUS_INVALID_PARAM;
  }

  written_length = snprintf(command,
                            sizeof(command),
                            "AT+MQTTCONN=%u,\"%s\",%u,%u",
                            (unsigned int)ESP01S_MQTT_LINK_ID,
                            host,
                            (unsigned int)port,
                            (unsigned int)keepalive_seconds);
  if ((written_length <= 0) || (written_length >= (int)sizeof(command)))
  {
    return ESP01S_STATUS_INVALID_PARAM;
  }

  if (timeout_ms == 0U)
  {
    timeout_ms = ESP01S_MQTT_CONN_TIMEOUT_MS;
  }

  return ESP01S_RunCommand(command, "OK", timeout_ms);
}

ESP01S_Status_t ESP01S_MQTTPublish(const char *topic, const char *payload, uint8_t qos, uint8_t retain)
{
  char command[ESP01S_COMMAND_BUFFER_SIZE];
  char escaped_payload[ESP01S_ESCAPED_PAYLOAD_SIZE];
  int written_length;

  if ((ESP01S_IsSafeATString(topic) == 0U) ||
      (ESP01S_EscapeATString(payload, escaped_payload, sizeof(escaped_payload)) == 0U) ||
      (qos > 2U) ||
      (retain > 1U))
  {
    return ESP01S_STATUS_INVALID_PARAM;
  }

  written_length = snprintf(command,
                            sizeof(command),
                            "AT+MQTTPUB=%u,\"%s\",\"%s\",%u,%u",
                            (unsigned int)ESP01S_MQTT_LINK_ID,
                            topic,
                            escaped_payload,
                            (unsigned int)qos,
                            (unsigned int)retain);
  if ((written_length <= 0) || (written_length >= (int)sizeof(command)))
  {
    return ESP01S_STATUS_INVALID_PARAM;
  }

  return ESP01S_RunCommand(command, "OK", ESP01S_DEFAULT_TIMEOUT_MS);
}

ESP01S_Status_t ESP01S_MQTTSubscribe(const char *topic, uint8_t qos)
{
  char command[ESP01S_COMMAND_BUFFER_SIZE];
  int written_length;

  if ((ESP01S_IsSafeATString(topic) == 0U) || (qos > 2U))
  {
    return ESP01S_STATUS_INVALID_PARAM;
  }

  written_length = snprintf(command,
                            sizeof(command),
                            "AT+MQTTSUB=%u,\"%s\",%u",
                            (unsigned int)ESP01S_MQTT_LINK_ID,
                            topic,
                            (unsigned int)qos);
  if ((written_length <= 0) || (written_length >= (int)sizeof(command)))
  {
    return ESP01S_STATUS_INVALID_PARAM;
  }

  return ESP01S_RunCommand(command, "OK", ESP01S_DEFAULT_TIMEOUT_MS);
}

ESP01S_Status_t ESP01S_MQTTDisconnect(void)
{
  char command[32];
  int written_length;

  written_length = snprintf(command, sizeof(command), "AT+MQTTCLEAN=%u", (unsigned int)ESP01S_MQTT_LINK_ID);
  if ((written_length <= 0) || (written_length >= (int)sizeof(command)))
  {
    return ESP01S_STATUS_INVALID_PARAM;
  }

  return ESP01S_RunCommand(command, "OK", ESP01S_DEFAULT_TIMEOUT_MS);
}

uint8_t ESP01S_ReadLatestMessage(ESP01S_MqttMessage_t *message)
{
  uint8_t has_message;
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  has_message = esp01s_message_available;
  if (has_message != 0U)
  {
    if (message != NULL)
    {
      *message = esp01s_latest_message;
    }
    esp01s_message_available = 0U;
  }
  ESP01S_RestoreIRQ(primask);

  return has_message;
}

uint8_t ESP01S_IsWiFiConnected(void)
{
  return esp01s_wifi_connected;
}

uint8_t ESP01S_IsMQTTConnected(void)
{
  return esp01s_mqtt_connected;
}

static void ESP01S_Task(void *argument)
{
  (void)argument;

  for (;;)
  {
    if (ESP01S_ConnectConfiguredNetwork() == ESP01S_STATUS_OK)
    {
      while (esp01s_wifi_connected != 0U)
      {
        ESP01S_Poll();

        if (esp01s_mqtt_connected == 0U)
        {
          break;
        }

        ESP01S_DelayMs(ESP01S_TASK_POLL_PERIOD_MS);
      }
    }

    ESP01S_SetConnectionState(0U, 0U);
    ESP01S_DelayMs(ESP01S_RECONNECT_DELAY_MS);
  }
}

void ESP01S_RxCpltCallbackFromISR(UART_HandleTypeDef *huart)
{
  if ((huart != NULL) && (huart->Instance == UART4))
  {
    ESP01S_ProcessByteFromISR(esp01s_rx_byte);
    ESP01S_ArmReception();
  }
}

void ESP01S_ErrorCallbackFromISR(UART_HandleTypeDef *huart)
{
  if ((huart != NULL) && (huart->Instance == UART4))
  {
    __HAL_UART_CLEAR_OREFLAG(huart);
    esp01s_rx_line_length = 0U;
    ESP01S_ArmReception();
  }
}
