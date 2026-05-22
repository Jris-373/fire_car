/* MQTT command layer:
 * - accepts fire_car/down and fire_car/mission messages;
 * - parses a fixed carry command from zone 1 to zone 2/3;
 * - stores the latest valid command for the mission state machine;
 * - publishes received/error status replies.
 */
#include "MQTT_Command.h"

#include <stdio.h>
#include <string.h>

#include "cmsis_os.h"

#define MQTT_COMMAND_TASK_PERIOD_MS    50U
#define MQTT_COMMAND_ACK_RETRY_COUNT   3U
#define MQTT_COMMAND_ACK_RETRY_MS      100U
#define MQTT_COMMAND_STATUS_SIZE       160U

static osThreadId_t mqttCommandTaskHandle = NULL;
static const osThreadAttr_t mqttCommandTaskAttributes = {
  .name = "mqttCmdTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t)osPriorityBelowNormal,
};

static MQTT_Command_t mqtt_latest_command;
static volatile uint8_t mqtt_command_available = 0U;
static uint32_t mqtt_command_sequence = 0U;
static uint8_t mqtt_last_id_valid = 0U;
static uint32_t mqtt_last_id = 0U;

static void MQTT_Command_Task(void *argument);

static void MQTT_Command_RestoreIRQ(uint32_t primask)
{
  if (primask == 0U)
  {
    __enable_irq();
  }
}

static void MQTT_Command_CopyBounded(char *destination, uint16_t destination_size, const char *source)
{
  uint16_t copy_length;

  if ((destination == NULL) || (destination_size == 0U))
  {
    return;
  }

  if (source == NULL)
  {
    destination[0] = '\0';
    return;
  }

  copy_length = (uint16_t)strlen(source);
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

static const char *MQTT_Command_SkipSpaces(const char *cursor)
{
  while ((*cursor == ' ') || (*cursor == '\t') || (*cursor == '\r') || (*cursor == '\n'))
  {
    cursor++;
  }

  return cursor;
}

static uint8_t MQTT_Command_IsTopicAccepted(const char *topic)
{
  if (topic == NULL)
  {
    return 0U;
  }

  if (strcmp(topic, MQTT_COMMAND_TOPIC) == 0)
  {
    return 1U;
  }

  if (strcmp(topic, MQTT_COMMAND_COMPAT_TOPIC) == 0)
  {
    return 1U;
  }

  return 0U;
}

static uint8_t MQTT_Command_ZoneFromChar(char value, uint8_t *zone)
{
  if ((value >= '1') && (value <= '3'))
  {
    *zone = (uint8_t)(value - '0');
    return 1U;
  }

  if ((value >= 'a') && (value <= 'c'))
  {
    value = (char)(value - ('a' - 'A'));
  }

  if ((value >= 'A') && (value <= 'C'))
  {
    *zone = (uint8_t)(value - 'A' + 1);
    return 1U;
  }

  return 0U;
}

static uint8_t MQTT_Command_IsTargetZone(uint8_t zone)
{
  return ((zone >= MQTT_COMMAND_MIN_TARGET_ZONE) &&
          (zone <= MQTT_COMMAND_MAX_TARGET_ZONE)) ? 1U : 0U;
}

static const char *MQTT_Command_FindJsonValue(const char *payload, const char *field)
{
  char pattern[20];
  int written_length;
  const char *field_start;
  const char *colon;

  written_length = snprintf(pattern, sizeof(pattern), "\"%s\"", field);
  if ((written_length <= 0) || (written_length >= (int)sizeof(pattern)))
  {
    return NULL;
  }

  field_start = strstr(payload, pattern);
  if (field_start == NULL)
  {
    return NULL;
  }

  colon = strchr(field_start + written_length, ':');
  if (colon == NULL)
  {
    return NULL;
  }

  return MQTT_Command_SkipSpaces(colon + 1);
}

static uint8_t MQTT_Command_ReadZoneField(const char *payload, const char *field, uint8_t *zone, uint8_t *present)
{
  const char *value_start = MQTT_Command_FindJsonValue(payload, field);

  *present = 0U;

  if (value_start == NULL)
  {
    return 0U;
  }

  *present = 1U;
  if (*value_start == '"')
  {
    value_start++;
  }

  return MQTT_Command_ZoneFromChar(*value_start, zone);
}

static uint8_t MQTT_Command_ReadIdField(const char *payload, uint32_t *id, uint8_t *present)
{
  const char *value_start = MQTT_Command_FindJsonValue(payload, "id");
  uint32_t value = 0U;
  uint8_t has_digit = 0U;

  *present = 0U;
  *id = 0U;

  if (value_start == NULL)
  {
    return 1U;
  }

  *present = 1U;
  if (*value_start == '"')
  {
    value_start++;
  }

  while ((*value_start >= '0') && (*value_start <= '9'))
  {
    uint32_t digit = (uint32_t)(*value_start - '0');

    if ((value > 429496729U) ||
        ((value == 429496729U) && (digit > 5U)))
    {
      return 0U;
    }
    value = (value * 10U) + digit;
    value_start++;
    has_digit = 1U;
  }

  if (has_digit == 0U)
  {
    return 0U;
  }

  *id = value;
  return 1U;
}

static uint8_t MQTT_Command_StringFieldEquals(const char *payload, const char *field, const char *expected, uint8_t *present)
{
  const char *value_start = MQTT_Command_FindJsonValue(payload, field);
  const char *expected_cursor = expected;

  *present = 0U;
  if (value_start == NULL)
  {
    return 1U;
  }

  *present = 1U;
  if (*value_start != '"')
  {
    return 0U;
  }
  value_start++;

  while ((*value_start != '\0') && (*value_start != '"') && (*expected_cursor != '\0'))
  {
    if (*value_start != *expected_cursor)
    {
      return 0U;
    }
    value_start++;
    expected_cursor++;
  }

  return ((*value_start == '"') && (*expected_cursor == '\0')) ? 1U : 0U;
}

static uint8_t MQTT_Command_ParsePlainPayload(const char *payload, uint8_t *to_zone)
{
  const char *cursor = payload;

  while (*cursor != '\0')
  {
    if (MQTT_Command_ZoneFromChar(*cursor, to_zone) != 0U)
    {
      return 1U;
    }
    cursor++;
  }

  return 0U;
}

static uint8_t MQTT_Command_ParsePayload(const char *payload, MQTT_Command_t *command, const char **reason)
{
  uint8_t field_present;
  uint8_t cmd_present;
  uint8_t from_present;
  uint8_t to_present;

  memset(command, 0, sizeof(*command));
  command->from_zone = MQTT_COMMAND_SOURCE_ZONE;
  MQTT_Command_CopyBounded(command->raw_payload, sizeof(command->raw_payload), payload);

  if ((payload == NULL) || (payload[0] == '\0'))
  {
    *reason = "empty_payload";
    return 0U;
  }

  if (MQTT_Command_ReadIdField(payload, &command->id, &command->has_id) == 0U)
  {
    *reason = "invalid_id";
    return 0U;
  }

  if (MQTT_Command_SkipSpaces(payload)[0] == '{')
  {
    if (MQTT_Command_StringFieldEquals(payload, "cmd", "carry_ball", &cmd_present) == 0U)
    {
      *reason = "unsupported_cmd";
      return 0U;
    }
    (void)cmd_present;

    if (MQTT_Command_ReadZoneField(payload, "from", &command->from_zone, &from_present) == 0U)
    {
      if (from_present != 0U)
      {
        *reason = "invalid_from";
        return 0U;
      }
    }

    if ((from_present != 0U) && (command->from_zone != MQTT_COMMAND_SOURCE_ZONE))
    {
      *reason = "invalid_from";
      return 0U;
    }

    if (MQTT_Command_ReadZoneField(payload, "to", &command->to_zone, &to_present) == 0U)
    {
      *reason = (to_present == 0U) ? "missing_to" : "invalid_to";
      return 0U;
    }

    if (to_present == 0U)
    {
      *reason = "missing_to";
      return 0U;
    }
  }
  else
  {
    if (MQTT_Command_ParsePlainPayload(payload, &command->to_zone) == 0U)
    {
      *reason = "missing_to";
      return 0U;
    }
  }

  field_present = MQTT_Command_IsTargetZone(command->to_zone);
  if (field_present == 0U)
  {
    *reason = "invalid_to";
    return 0U;
  }

  *reason = NULL;
  return 1U;
}

static void MQTT_Command_StoreAccepted(const MQTT_Command_t *command)
{
  uint32_t primask = __get_PRIMASK();
  MQTT_Command_t stored = *command;

  stored.sequence = mqtt_command_sequence + 1U;
  if (stored.sequence == 0U)
  {
    stored.sequence = 1U;
  }
  mqtt_command_sequence = stored.sequence;
  stored.received_tick = HAL_GetTick();

  __disable_irq();
  mqtt_latest_command = stored;
  mqtt_command_available = 1U;
  if (stored.has_id != 0U)
  {
    mqtt_last_id = stored.id;
    mqtt_last_id_valid = 1U;
  }
  MQTT_Command_RestoreIRQ(primask);
}

static uint8_t MQTT_Command_IsDuplicate(const MQTT_Command_t *command)
{
  if ((command->has_id != 0U) &&
      (mqtt_last_id_valid != 0U) &&
      (command->id == mqtt_last_id))
  {
    return 1U;
  }

  return 0U;
}

static void MQTT_Command_ReportWithRetry(const MQTT_Command_t *command, const char *state, const char *reason)
{
  uint8_t retry;

  for (retry = 0U; retry < MQTT_COMMAND_ACK_RETRY_COUNT; retry++)
  {
    if (MQTT_Command_ReportState(command, state, reason) == ESP01S_STATUS_OK)
    {
      return;
    }

    osDelay(MQTT_COMMAND_ACK_RETRY_MS);
  }
}

static void MQTT_Command_HandleMessage(const ESP01S_MqttMessage_t *message)
{
  MQTT_Command_t parsed_command;
  const char *reason = NULL;

  if (MQTT_Command_IsTopicAccepted(message->topic) == 0U)
  {
    return;
  }

  if (MQTT_Command_ParsePayload(message->payload, &parsed_command, &reason) == 0U)
  {
    MQTT_Command_ReportWithRetry(NULL, "error", reason);
    return;
  }

  if (MQTT_Command_IsDuplicate(&parsed_command) != 0U)
  {
    MQTT_Command_ReportWithRetry(&parsed_command, "duplicate", NULL);
    return;
  }

  MQTT_Command_StoreAccepted(&parsed_command);
  MQTT_Command_ReportWithRetry(&parsed_command, "received", NULL);
}

void MQTT_Command_StartTask(void)
{
  if (mqttCommandTaskHandle == NULL)
  {
    mqttCommandTaskHandle = osThreadNew(MQTT_Command_Task, NULL, &mqttCommandTaskAttributes);
  }
}

uint8_t MQTT_Command_ReadLatest(MQTT_Command_t *command)
{
  uint8_t has_command;
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  has_command = mqtt_command_available;
  if (has_command != 0U)
  {
    if (command != NULL)
    {
      *command = mqtt_latest_command;
    }
    mqtt_command_available = 0U;
  }
  MQTT_Command_RestoreIRQ(primask);

  return has_command;
}

ESP01S_Status_t MQTT_Command_ReportState(const MQTT_Command_t *command, const char *state, const char *reason)
{
  char payload[MQTT_COMMAND_STATUS_SIZE];
  const char *safe_state = (state == NULL) ? "unknown" : state;
  const char *safe_reason = reason;
  int written_length;

  if (ESP01S_IsMQTTConnected() == 0U)
  {
    return ESP01S_STATUS_ERROR;
  }

  if (safe_reason == NULL)
  {
    safe_reason = "";
  }

  if (command != NULL)
  {
    if (safe_reason[0] != '\0')
    {
      if (command->has_id != 0U)
      {
        written_length = snprintf(payload,
                                  sizeof(payload),
                                  "{\"id\":%lu,\"state\":\"%s\",\"from\":%u,\"to\":%u,\"reason\":\"%s\"}",
                                  (unsigned long)command->id,
                                  safe_state,
                                  (unsigned int)command->from_zone,
                                  (unsigned int)command->to_zone,
                                  safe_reason);
      }
      else
      {
        written_length = snprintf(payload,
                                  sizeof(payload),
                                  "{\"state\":\"%s\",\"from\":%u,\"to\":%u,\"reason\":\"%s\"}",
                                  safe_state,
                                  (unsigned int)command->from_zone,
                                  (unsigned int)command->to_zone,
                                  safe_reason);
      }
    }
    else if (command->has_id != 0U)
    {
      written_length = snprintf(payload,
                                sizeof(payload),
                                "{\"id\":%lu,\"state\":\"%s\",\"from\":%u,\"to\":%u}",
                                (unsigned long)command->id,
                                safe_state,
                                (unsigned int)command->from_zone,
                                (unsigned int)command->to_zone);
    }
    else
    {
      written_length = snprintf(payload,
                                sizeof(payload),
                                "{\"state\":\"%s\",\"from\":%u,\"to\":%u}",
                                safe_state,
                                (unsigned int)command->from_zone,
                                (unsigned int)command->to_zone);
    }
  }
  else if (safe_reason[0] != '\0')
  {
    written_length = snprintf(payload,
                              sizeof(payload),
                              "{\"state\":\"%s\",\"reason\":\"%s\"}",
                              safe_state,
                              safe_reason);
  }
  else
  {
    written_length = snprintf(payload,
                              sizeof(payload),
                              "{\"state\":\"%s\"}",
                              safe_state);
  }

  if ((written_length <= 0) || (written_length >= (int)sizeof(payload)))
  {
    return ESP01S_STATUS_INVALID_PARAM;
  }

  return ESP01S_MQTTPublish(MQTT_COMMAND_STATUS_TOPIC, payload, 0U, 0U);
}

static void MQTT_Command_Task(void *argument)
{
  ESP01S_MqttMessage_t message;

  (void)argument;

  for (;;)
  {
    if (ESP01S_ReadLatestMessage(&message) != 0U)
    {
      MQTT_Command_HandleMessage(&message);
    }

    osDelay(MQTT_COMMAND_TASK_PERIOD_MS);
  }
}
