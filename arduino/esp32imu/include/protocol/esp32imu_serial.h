/**
 * @file esp32imu_serial.h
 * @brief Serial protocol for ACL esp32-imu 
 * @author Parker Lusk <plusk@mit.edu>
 * @date 21 Nov 2020
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#ifdef __GNUC__
#define PACKED_STRUCT(name) struct __attribute__((__packed__)) name
#else
#define PACKED_STRUCT(name) __pragma(pack(push, 1)) struct name __pragma(pack(pop))
#endif

//=============================================================================
// message types
//=============================================================================

typedef enum {
  ESP_SERIAL_MSG_IMU,
  ESP_SERIAL_MSG_RATE,
  ESP_SERIAL_NUM_MSGS
} esp_msg_type_t;

//=============================================================================
// message definitions
//=============================================================================

typedef struct {
  uint32_t t_us;
  uint32_t seq;
  float accel_x;
  float accel_y;
  float accel_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
  float mag_x;
  float mag_y;
  float mag_z;
} esp_serial_imu_msg_t;

typedef struct {
  uint16_t frequency;
} esp_serial_rate_msg_t;

// payload lengths
static constexpr float ESP_SERIAL_PAYLOAD_LEN[] = {
  sizeof(esp_serial_imu_msg_t),
  sizeof(esp_serial_rate_msg_t)
};

// manually indicate the largest msg payload
static constexpr size_t ESP_SERIAL_MAX_PAYLOAD_LEN = sizeof(esp_serial_imu_msg_t);

//=============================================================================
// generic message type
//=============================================================================

static constexpr uint8_t ESP_SERIAL_MAGIC = 0xA5;

PACKED_STRUCT(esp_serial_message_t) {
  uint8_t magic;
  uint8_t type;
  uint8_t payload[ESP_SERIAL_MAX_PAYLOAD_LEN];
  uint8_t crc;
};

static constexpr size_t ESP_SERIAL_MAX_MESSAGE_LEN = sizeof(esp_serial_message_t);

//=============================================================================
// utility functions
//=============================================================================

// source: http://www.nongnu.org/avr-libc/user-manual/group__util__crc.html#gab27eaaef6d7fd096bd7d57bf3f9ba083
inline uint8_t esp_serial_update_crc(uint8_t inCrc, uint8_t inData)
{
  uint8_t i;
  uint8_t data;

  data = inCrc ^ inData;

  for ( i = 0; i < 8; i++ )
  {
    if (( data & 0x80 ) != 0 )
    {
      data <<= 1;
      data ^= 0x07;
    }
    else
    {
      data <<= 1;
    }
  }
  return data;
}

inline void esp_serial_finalize_message(esp_serial_message_t *msg)
{
  msg->magic = ESP_SERIAL_MAGIC;

  uint8_t crc = 0;
  crc = esp_serial_update_crc(crc, msg->magic);
  crc = esp_serial_update_crc(crc, msg->type);
  for (size_t i=0; i<ESP_SERIAL_PAYLOAD_LEN[msg->type]; ++i)
  {
    crc = esp_serial_update_crc(crc, msg->payload[i]);
  }

  msg->crc = crc;
}

inline size_t esp_serial_send_to_buffer(uint8_t *dst, const esp_serial_message_t *src)
{
  size_t offset = 0;
  memcpy(dst + offset, &src->magic,  sizeof(src->magic)); offset += sizeof(src->magic);
  memcpy(dst + offset, &src->type,   sizeof(src->type));  offset += sizeof(src->type);
  memcpy(dst + offset, src->payload, ESP_SERIAL_PAYLOAD_LEN[src->type]); offset += ESP_SERIAL_PAYLOAD_LEN[src->type];
  memcpy(dst + offset, &src->crc,    sizeof(src->crc)); offset += sizeof(src->crc);
  return offset;
}

//=============================================================================
// IMU message
//=============================================================================

inline void esp_serial_imu_msg_pack(esp_serial_message_t *dst, const esp_serial_imu_msg_t *src)
{
  dst->type = ESP_SERIAL_MSG_IMU;
  size_t offset = 0;
  memcpy(dst->payload + offset, &src->t_us, sizeof(src->t_us)); offset += sizeof(src->t_us);
  memcpy(dst->payload + offset, &src->seq, sizeof(src->seq)); offset += sizeof(src->seq);
  memcpy(dst->payload + offset, &src->accel_x, sizeof(src->accel_x)); offset += sizeof(src->accel_x);
  memcpy(dst->payload + offset, &src->accel_y, sizeof(src->accel_y)); offset += sizeof(src->accel_y);
  memcpy(dst->payload + offset, &src->accel_z, sizeof(src->accel_z)); offset += sizeof(src->accel_z);
  memcpy(dst->payload + offset, &src->gyro_x,  sizeof(src->gyro_x));  offset += sizeof(src->gyro_x);
  memcpy(dst->payload + offset, &src->gyro_y,  sizeof(src->gyro_y));  offset += sizeof(src->gyro_y);
  memcpy(dst->payload + offset, &src->gyro_z,  sizeof(src->gyro_z));  offset += sizeof(src->gyro_z);
  memcpy(dst->payload + offset, &src->mag_x,  sizeof(src->mag_x));  offset += sizeof(src->mag_x);
  memcpy(dst->payload + offset, &src->mag_y,  sizeof(src->mag_y));  offset += sizeof(src->mag_y);
  memcpy(dst->payload + offset, &src->mag_z,  sizeof(src->mag_z));  offset += sizeof(src->mag_z);
  esp_serial_finalize_message(dst);
}

inline void esp_serial_imu_msg_unpack(esp_serial_imu_msg_t *dst, const esp_serial_message_t *src)
{
  size_t offset = 0;
  memcpy(&dst->t_us, src->payload + offset, sizeof(dst->t_us)); offset += sizeof(dst->t_us);
  memcpy(&dst->seq, src->payload + offset, sizeof(dst->seq)); offset += sizeof(dst->seq);
  memcpy(&dst->accel_x, src->payload + offset, sizeof(dst->accel_x)); offset += sizeof(dst->accel_x);
  memcpy(&dst->accel_y, src->payload + offset, sizeof(dst->accel_y)); offset += sizeof(dst->accel_y);
  memcpy(&dst->accel_z, src->payload + offset, sizeof(dst->accel_z)); offset += sizeof(dst->accel_z);
  memcpy(&dst->gyro_x,  src->payload + offset, sizeof(dst->gyro_x));  offset += sizeof(dst->gyro_x);
  memcpy(&dst->gyro_y,  src->payload + offset, sizeof(dst->gyro_y));  offset += sizeof(dst->gyro_y);
  memcpy(&dst->gyro_z,  src->payload + offset, sizeof(dst->gyro_z));  offset += sizeof(dst->gyro_z);
  memcpy(&dst->mag_x,  src->payload + offset, sizeof(dst->mag_x));  offset += sizeof(dst->mag_x);
  memcpy(&dst->mag_y,  src->payload + offset, sizeof(dst->mag_y));  offset += sizeof(dst->mag_y);
  memcpy(&dst->mag_z,  src->payload + offset, sizeof(dst->mag_z));  offset += sizeof(dst->mag_z);
}

inline size_t esp_serial_imu_msg_send_to_buffer(uint8_t *dst, const esp_serial_imu_msg_t *src)
{
  esp_serial_message_t msg;
  esp_serial_imu_msg_pack(&msg, src);
  return esp_serial_send_to_buffer(dst, &msg);
}

//=============================================================================
// Rate message
//=============================================================================

inline void esp_serial_rate_msg_pack(esp_serial_message_t *dst, const esp_serial_rate_msg_t *src)
{
  dst->type = ESP_SERIAL_MSG_RATE;
  size_t offset = 0;
  memcpy(dst->payload + offset, &src->frequency, sizeof(src->frequency)); offset += sizeof(src->frequency);
  esp_serial_finalize_message(dst);
}

inline void esp_serial_rate_msg_unpack(esp_serial_rate_msg_t *dst, const esp_serial_message_t *src)
{
  size_t offset = 0;
  memcpy(&dst->frequency, src->payload + offset, sizeof(dst->frequency)); offset += sizeof(dst->frequency);
}

inline size_t esp_serial_rate_msg_send_to_buffer(uint8_t *dst, const esp_serial_rate_msg_t *src)
{
  esp_serial_message_t msg;
  esp_serial_rate_msg_pack(&msg, src);
  return esp_serial_send_to_buffer(dst, &msg);
}

//==============================================================================
// parser
//==============================================================================

typedef enum
{
  ESP_SERIAL_PARSE_STATE_IDLE,
  ESP_SERIAL_PARSE_STATE_GOT_MAGIC,
  ESP_SERIAL_PARSE_STATE_GOT_TYPE,
  ESP_SERIAL_PARSE_STATE_GOT_PAYLOAD
} esp_serial_parse_state_t;

inline bool esp_serial_parse_byte(uint8_t byte, esp_serial_message_t *msg)
{
  static esp_serial_parse_state_t parse_state = ESP_SERIAL_PARSE_STATE_IDLE;
  static uint8_t crc_value = 0;
  static size_t payload_index = 0;
  static esp_serial_message_t msg_buffer;

  bool got_message = false;
  switch (parse_state)
  {
  case ESP_SERIAL_PARSE_STATE_IDLE:
    if (byte == ESP_SERIAL_MAGIC)
    {
      crc_value = 0;
      payload_index = 0;

      msg_buffer.magic = byte;
      crc_value = esp_serial_update_crc(crc_value, byte);

      parse_state = ESP_SERIAL_PARSE_STATE_GOT_MAGIC;
    }
    break;

  case ESP_SERIAL_PARSE_STATE_GOT_MAGIC:
    msg_buffer.type = byte;
    crc_value = esp_serial_update_crc(crc_value, byte);
    parse_state = ESP_SERIAL_PARSE_STATE_GOT_TYPE;
    break;

  case ESP_SERIAL_PARSE_STATE_GOT_TYPE:
    msg_buffer.payload[payload_index++] = byte;
    crc_value = esp_serial_update_crc(crc_value, byte);
    if (payload_index == ESP_SERIAL_PAYLOAD_LEN[msg_buffer.type])
    {
      parse_state = ESP_SERIAL_PARSE_STATE_GOT_PAYLOAD;
    }
    break;

  case ESP_SERIAL_PARSE_STATE_GOT_PAYLOAD:
    msg_buffer.crc = byte;
    if (msg_buffer.crc == crc_value)
    {
      got_message = true;
      memcpy(msg, &msg_buffer, sizeof(msg_buffer));
    }
    parse_state = ESP_SERIAL_PARSE_STATE_IDLE;
    break;
  }

  return got_message;
}
