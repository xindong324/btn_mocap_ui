#pragma once
// MESSAGE MAV_SET_GAIN PACKING

#define MAVLINK_MSG_ID_MAV_SET_GAIN 56


typedef struct __mavlink_mav_set_gain_t {
 float gain[30]; /*<  30 gain of uav 0-180*/
 uint8_t gain_num; /*<  num chaged of the gain*/
} mavlink_mav_set_gain_t;

#define MAVLINK_MSG_ID_MAV_SET_GAIN_LEN 121
#define MAVLINK_MSG_ID_MAV_SET_GAIN_MIN_LEN 121
#define MAVLINK_MSG_ID_56_LEN 121
#define MAVLINK_MSG_ID_56_MIN_LEN 121

#define MAVLINK_MSG_ID_MAV_SET_GAIN_CRC 122
#define MAVLINK_MSG_ID_56_CRC 122

#define MAVLINK_MSG_MAV_SET_GAIN_FIELD_GAIN_LEN 30

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MAV_SET_GAIN { \
    56, \
    "MAV_SET_GAIN", \
    2, \
    {  { "gain_num", NULL, MAVLINK_TYPE_UINT8_T, 0, 120, offsetof(mavlink_mav_set_gain_t, gain_num) }, \
         { "gain", NULL, MAVLINK_TYPE_FLOAT, 30, 0, offsetof(mavlink_mav_set_gain_t, gain) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MAV_SET_GAIN { \
    "MAV_SET_GAIN", \
    2, \
    {  { "gain_num", NULL, MAVLINK_TYPE_UINT8_T, 0, 120, offsetof(mavlink_mav_set_gain_t, gain_num) }, \
         { "gain", NULL, MAVLINK_TYPE_FLOAT, 30, 0, offsetof(mavlink_mav_set_gain_t, gain) }, \
         } \
}
#endif

/**
 * @brief Pack a mav_set_gain message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param gain_num  num chaged of the gain
 * @param gain  30 gain of uav 0-180
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mav_set_gain_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t gain_num, const float *gain)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAV_SET_GAIN_LEN];
    _mav_put_uint8_t(buf, 120, gain_num);
    _mav_put_float_array(buf, 0, gain, 30);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAV_SET_GAIN_LEN);
#else
    mavlink_mav_set_gain_t packet;
    packet.gain_num = gain_num;
    mav_array_memcpy(packet.gain, gain, sizeof(float)*30);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAV_SET_GAIN_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MAV_SET_GAIN;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MAV_SET_GAIN_MIN_LEN, MAVLINK_MSG_ID_MAV_SET_GAIN_LEN, MAVLINK_MSG_ID_MAV_SET_GAIN_CRC);
}

/**
 * @brief Pack a mav_set_gain message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gain_num  num chaged of the gain
 * @param gain  30 gain of uav 0-180
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mav_set_gain_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t gain_num,const float *gain)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAV_SET_GAIN_LEN];
    _mav_put_uint8_t(buf, 120, gain_num);
    _mav_put_float_array(buf, 0, gain, 30);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAV_SET_GAIN_LEN);
#else
    mavlink_mav_set_gain_t packet;
    packet.gain_num = gain_num;
    mav_array_memcpy(packet.gain, gain, sizeof(float)*30);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAV_SET_GAIN_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MAV_SET_GAIN;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MAV_SET_GAIN_MIN_LEN, MAVLINK_MSG_ID_MAV_SET_GAIN_LEN, MAVLINK_MSG_ID_MAV_SET_GAIN_CRC);
}

/**
 * @brief Encode a mav_set_gain struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mav_set_gain C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mav_set_gain_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mav_set_gain_t* mav_set_gain)
{
    return mavlink_msg_mav_set_gain_pack(system_id, component_id, msg, mav_set_gain->gain_num, mav_set_gain->gain);
}

/**
 * @brief Encode a mav_set_gain struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mav_set_gain C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mav_set_gain_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mav_set_gain_t* mav_set_gain)
{
    return mavlink_msg_mav_set_gain_pack_chan(system_id, component_id, chan, msg, mav_set_gain->gain_num, mav_set_gain->gain);
}

/**
 * @brief Send a mav_set_gain message
 * @param chan MAVLink channel to send the message
 *
 * @param gain_num  num chaged of the gain
 * @param gain  30 gain of uav 0-180
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mav_set_gain_send(mavlink_channel_t chan, uint8_t gain_num, const float *gain)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAV_SET_GAIN_LEN];
    _mav_put_uint8_t(buf, 120, gain_num);
    _mav_put_float_array(buf, 0, gain, 30);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_SET_GAIN, buf, MAVLINK_MSG_ID_MAV_SET_GAIN_MIN_LEN, MAVLINK_MSG_ID_MAV_SET_GAIN_LEN, MAVLINK_MSG_ID_MAV_SET_GAIN_CRC);
#else
    mavlink_mav_set_gain_t packet;
    packet.gain_num = gain_num;
    mav_array_memcpy(packet.gain, gain, sizeof(float)*30);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_SET_GAIN, (const char *)&packet, MAVLINK_MSG_ID_MAV_SET_GAIN_MIN_LEN, MAVLINK_MSG_ID_MAV_SET_GAIN_LEN, MAVLINK_MSG_ID_MAV_SET_GAIN_CRC);
#endif
}

/**
 * @brief Send a mav_set_gain message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mav_set_gain_send_struct(mavlink_channel_t chan, const mavlink_mav_set_gain_t* mav_set_gain)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mav_set_gain_send(chan, mav_set_gain->gain_num, mav_set_gain->gain);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_SET_GAIN, (const char *)mav_set_gain, MAVLINK_MSG_ID_MAV_SET_GAIN_MIN_LEN, MAVLINK_MSG_ID_MAV_SET_GAIN_LEN, MAVLINK_MSG_ID_MAV_SET_GAIN_CRC);
#endif
}

#if MAVLINK_MSG_ID_MAV_SET_GAIN_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mav_set_gain_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t gain_num, const float *gain)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 120, gain_num);
    _mav_put_float_array(buf, 0, gain, 30);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_SET_GAIN, buf, MAVLINK_MSG_ID_MAV_SET_GAIN_MIN_LEN, MAVLINK_MSG_ID_MAV_SET_GAIN_LEN, MAVLINK_MSG_ID_MAV_SET_GAIN_CRC);
#else
    mavlink_mav_set_gain_t *packet = (mavlink_mav_set_gain_t *)msgbuf;
    packet->gain_num = gain_num;
    mav_array_memcpy(packet->gain, gain, sizeof(float)*30);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_SET_GAIN, (const char *)packet, MAVLINK_MSG_ID_MAV_SET_GAIN_MIN_LEN, MAVLINK_MSG_ID_MAV_SET_GAIN_LEN, MAVLINK_MSG_ID_MAV_SET_GAIN_CRC);
#endif
}
#endif

#endif

// MESSAGE MAV_SET_GAIN UNPACKING


/**
 * @brief Get field gain_num from mav_set_gain message
 *
 * @return  num chaged of the gain
 */
static inline uint8_t mavlink_msg_mav_set_gain_get_gain_num(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  120);
}

/**
 * @brief Get field gain from mav_set_gain message
 *
 * @return  30 gain of uav 0-180
 */
static inline uint16_t mavlink_msg_mav_set_gain_get_gain(const mavlink_message_t* msg, float *gain)
{
    return _MAV_RETURN_float_array(msg, gain, 30,  0);
}

/**
 * @brief Decode a mav_set_gain message into a struct
 *
 * @param msg The message to decode
 * @param mav_set_gain C-struct to decode the message contents into
 */
static inline void mavlink_msg_mav_set_gain_decode(const mavlink_message_t* msg, mavlink_mav_set_gain_t* mav_set_gain)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mav_set_gain_get_gain(msg, mav_set_gain->gain);
    mav_set_gain->gain_num = mavlink_msg_mav_set_gain_get_gain_num(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MAV_SET_GAIN_LEN? msg->len : MAVLINK_MSG_ID_MAV_SET_GAIN_LEN;
        memset(mav_set_gain, 0, MAVLINK_MSG_ID_MAV_SET_GAIN_LEN);
    memcpy(mav_set_gain, _MAV_PAYLOAD(msg), len);
#endif
}
