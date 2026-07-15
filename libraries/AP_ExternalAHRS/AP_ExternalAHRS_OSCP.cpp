/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define AP_MATH_ALLOW_DOUBLE_FUNCTIONS 1
#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_OSCP_ENABLED

#include "AP_ExternalAHRS_OSCP.h"
#include "OSCP_common.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Compass/AP_Compass.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <cstring>
#include <cstdio>

extern const AP_HAL::HAL& hal;

AP_ExternalAHRS_OSCP::AP_ExternalAHRS_OSCP(AP_ExternalAHRS* _frontend, AP_ExternalAHRS::state_t& _state) : AP_ExternalAHRS_backend(_frontend, _state) {
    auto& sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);
    if (uart != nullptr) {
        send_cobs_command("CONFIG\r\n");
        hal.scheduler->delay(50);

        send_cobs_command("OMM\r\n");
        hal.scheduler->delay(50);

        send_cobs_command("SUF\r\n");
        hal.scheduler->delay(50);

        send_cobs_command("EXIT\r\n");
        hal.scheduler->delay(50);

        setup_complete = true;
    }

    set_default_sensors(uint16_t(AP_ExternalAHRS::AvailableSensor::IMU));
    set_default_sensors(uint16_t(AP_ExternalAHRS::AvailableSensor::COMPASS));

    baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_OSCP::update_thread, void), "AHRS", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        AP_HAL::panic("OSCP Failed to start ExternalAHRS update thread");
    }
}

const uint16_t AP_ExternalAHRS_OSCP::CRC_LUT[256] = {
    0x0000, 0xA2EB, 0xE73D, 0x45D6, 0x6C91, 0xCE7A, 0x8BAC, 0x2947,
    0xD922, 0x7BC9, 0x3E1F, 0x9CF4, 0xB5B3, 0x1758, 0x528E, 0xF065,
    0x10AF, 0xB244, 0xF792, 0x5579, 0x7C3E, 0xDED5, 0x9B03, 0x39E8,
    0xC98D, 0x6B66, 0x2EB0, 0x8C5B, 0xA51C, 0x07F7, 0x4221, 0xE0CA,
    0x215E, 0x83B5, 0xC663, 0x6488, 0x4DCF, 0xEF24, 0xAAF2, 0x0819,
    0xF87C, 0x5A97, 0x1F41, 0xBDAA, 0x94ED, 0x3606, 0x73D0, 0xD13B,
    0x31F1, 0x931A, 0xD6CC, 0x7427, 0x5D60, 0xFF8B, 0xBA5D, 0x18B6,
    0xE8D3, 0x4A38, 0x0FEE, 0xAD05, 0x8442, 0x26A9, 0x637F, 0xC194,
    0x42BC, 0xE057, 0xA581, 0x076A, 0x2E2D, 0x8CC6, 0xC910, 0x6BFB,
    0x9B9E, 0x3975, 0x7CA3, 0xDE48, 0xF70F, 0x55E4, 0x1032, 0xB2D9,
    0x5213, 0xF0F8, 0xB52E, 0x17C5, 0x3E82, 0x9C69, 0xD9BF, 0x7B54,
    0x8B31, 0x29DA, 0x6C0C, 0xCEE7, 0xE7A0, 0x454B, 0x009D, 0xA276,
    0x63E2, 0xC109, 0x84DF, 0x2634, 0x0F73, 0xAD98, 0xE84E, 0x4AA5,
    0xBAC0, 0x182B, 0x5DFD, 0xFF16, 0xD651, 0x74BA, 0x316C, 0x9387,
    0x734D, 0xD1A6, 0x9470, 0x369B, 0x1FDC, 0xBD37, 0xF8E1, 0x5A0A,
    0xAA6F, 0x0884, 0x4D52, 0xEFB9, 0xC6FE, 0x6415, 0x21C3, 0x8328,
    0x8578, 0x2793, 0x6245, 0xC0AE, 0xE9E9, 0x4B02, 0x0ED4, 0xAC3F,
    0x5C5A, 0xFEB1, 0xBB67, 0x198C, 0x30CB, 0x9220, 0xD7F6, 0x751D,
    0x95D7, 0x373C, 0x72EA, 0xD001, 0xF946, 0x5BAD, 0x1E7B, 0xBC90,
    0x4CF5, 0xEE1E, 0xABC8, 0x0923, 0x2064, 0x828F, 0xC759, 0x65B2,
    0xA426, 0x06CD, 0x431B, 0xE1F0, 0xC8B7, 0x6A5C, 0x2F8A, 0x8D61,
    0x7D04, 0xDFEF, 0x9A39, 0x38D2, 0x1195, 0xB37E, 0xF6A8, 0x5443,
    0xB489, 0x1662, 0x53B4, 0xF15F, 0xD818, 0x7AF3, 0x3F25, 0x9DCE,
    0x6DAB, 0xCF40, 0x8A96, 0x287D, 0x013A, 0xA3D1, 0xE607, 0x44EC,
    0xC7C4, 0x652F, 0x20F9, 0x8212, 0xAB55, 0x09BE, 0x4C68, 0xEE83,
    0x1EE6, 0xBC0D, 0xF9DB, 0x5B30, 0x7277, 0xD09C, 0x954A, 0x37A1,
    0xD76B, 0x7580, 0x3056, 0x92BD, 0xBBFA, 0x1911, 0x5CC7, 0xFE2C,
    0x0E49, 0xACA2, 0xE974, 0x4B9F, 0x62D8, 0xC033, 0x85E5, 0x270E,
    0xE69A, 0x4471, 0x01A7, 0xA34C, 0x8A0B, 0x28E0, 0x6D36, 0xCFDD,
    0x3FB8, 0x9D53, 0xD885, 0x7A6E, 0x5329, 0xF1C2, 0xB414, 0x16FF,
    0xF635, 0x54DE, 0x1108, 0xB3E3, 0x9AA4, 0x384F, 0x7D99, 0xDF72,
    0x2F17, 0x8DFC, 0xC82A, 0x6AC1, 0x4386, 0xE16D, 0xA4BB, 0x0650,
};

const uint8_t AP_ExternalAHRS_OSCP::OSCP_FRAME_LEN[8] = {
    61, // RAW
    25, // EULER
    29, // QUATERNION
    49, // ROT_MAT
    0,  // RESERVED
    58, // DEBUG1
    58, // DEBUG2
    40, // STARTUP
};

bool AP_ExternalAHRS_OSCP::initialised(void) const {
    if (setup_complete) {
        return true;
    }
    return false;
}

bool AP_ExternalAHRS_OSCP::healthy(void) const {
    const uint32_t now = AP_HAL::millis();

    if (initialised() && (now - last_raw_pkt) < 50) {
        return true;
    }

    return false;
} 

bool AP_ExternalAHRS_OSCP::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const {
    if (!setup_complete) {
        hal.util->snprintf(failure_msg, failure_msg_len, "OSCP setup failed");
        return false;
    }

    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "OSCP IMU Unhealthy");
        return false;
    }
    return true;
}

void AP_ExternalAHRS_OSCP::send_cobs_command(const char *cmd) {
    uint8_t enc[256];
    size_t enc_len = 0;

    if (oscp_cobs_encode((const uint8_t *)cmd, strlen(cmd), enc, sizeof(enc) - 1, &enc_len) != oscp_err_t::OSCP_OK) {
        return;
    }

    enc[enc_len++] = 0x00;
    uart->write(enc, enc_len);
}

void AP_ExternalAHRS_OSCP::update_thread(void) {
    while (!setup_complete) {
        hal.scheduler->delay(1);
    }
    uart->begin(baudrate);

    oscp_parser_t parser = { 0 };
    oscp_parser_init(&parser);

    while (1) {
        uint8_t buf[OSCP_FRAME_MAX_LEN];
        uint32_t n = uart->available();

        n = MIN(n, sizeof(buf));

        uint32_t bytes_read = uart->read(buf, n);

        if (bytes_read > 0) {
            oscp_parser_feed_buf(&parser, buf, bytes_read);
        } else {
            hal.scheduler->delay(1);
        }
    }
}

void AP_ExternalAHRS_OSCP::get_filter_status(nav_filter_status &status) const {
    memset(&status, 0, sizeof(status));
    status.flags.initalized = initialised();
    if (healthy()) {
        status.flags.attitude           = true;
        status.flags.vert_pos           = true;
        status.flags.horiz_vel          = true;
        status.flags.vert_vel           = true;
        status.flags.horiz_pos_rel      = true;
        status.flags.horiz_pos_abs      = true;
        status.flags.vert_pos           = true;
        status.flags.pred_horiz_pos_rel = true;
        status.flags.pred_horiz_pos_abs = true;
        status.flags.using_gps          = true;
    }
}

void AP_ExternalAHRS_OSCP::oscp_parser_feed_buf(oscp_parser_t* _parser, const uint8_t* buf, const size_t buf_len) {
    for (size_t i = 0; i < buf_len; i++) {
        oscp_parser_feed(_parser, buf[i]);
    }
}

void AP_ExternalAHRS_OSCP::oscp_parser_feed(oscp_parser_t* _parser, uint8_t byte) {
    if (byte == OSCP_FRAME_DELIM) {
        if (_parser->synced && _parser->buf_len > 0) {
            parser_buffer(_parser);
        }

        oscp_parser_go(_parser);
        return;
    }

    if (!_parser->synced) {
        return;
    }

    if (_parser->buf_len >= OSCP_FRAME_MAX_LEN) {
        _parser->stats.overflows++;
        oscp_parser_reset(_parser);
        return;
    }

    _parser->buf[_parser->buf_len++] = byte;
}

void AP_ExternalAHRS_OSCP::parser_buffer(oscp_parser_t* _parser) {
    uint8_t decode[OSCP_FRAME_MAX_LEN];
    size_t dec_len = 0;

    if (oscp_cobs_decode(_parser->buf, _parser->buf_len, decode, sizeof(decode), &dec_len) != oscp_err_t::OSCP_OK) {
        _parser->stats.cobs_errors++;
        return;
    }

    if (dec_len < 3) {
        _parser->stats.framing_errors++;
        return;
    }

    const uint8_t frame_type = decode[0] & 0x07u;
    if (frame_type > 7 || dec_len != OSCP_FRAME_LEN[frame_type]) {
        _parser->stats.framing_errors++;
        return;
    }

    if (!crc_check(decode, dec_len)) {
        _parser->stats.crc_errors++;
        return;
    }

    oscp_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    frame.type = (oscp_frame_type_t)frame_type;
    _parser->stats.frames_ok++;

    switch (frame.type) {
        case oscp_frame_type_t::OSCP_FRAME_RAW:
            decode_raw(&frame, decode);
            break;
        case oscp_frame_type_t::OSCP_FRAME_STARTUP:
            decode_startup(&frame, decode);
            break;
        default:
            break;
    }
}

AP_ExternalAHRS_OSCP::oscp_err_t AP_ExternalAHRS_OSCP::oscp_cobs_decode(const uint8_t* data, size_t data_len, uint8_t* decode, size_t dec_max_len, size_t* dec_len) {
    const cobs_decode_result result =
        cobs_decode(decode, dec_max_len, data, data_len);

    if (result.status != COBS_DECODE_OK) {
        return oscp_err_t::OSCP_ERR;
    }

    *dec_len = result.out_len;
    return oscp_err_t::OSCP_OK;
}

AP_ExternalAHRS_OSCP::oscp_err_t AP_ExternalAHRS_OSCP::oscp_cobs_encode(const uint8_t *data, size_t data_len, uint8_t *encode, size_t enc_max_len, size_t *enc_len) {
    const cobs_encode_result result = 
        cobs_encode(encode, enc_max_len, data, data_len);

    if (result.status != COBS_ENCODE_OK) {
        return oscp_err_t::OSCP_ERR;
    }

    *enc_len = result.out_len;
    return oscp_err_t::OSCP_OK;
}

uint16_t AP_ExternalAHRS_OSCP::oscp_crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (uint32_t i = 0; i < len; i++) {
        const uint8_t pos = (crc >> 8) ^ data[i];
        crc = (crc << 8) ^ CRC_LUT[pos];
    }
    return crc;
}

bool AP_ExternalAHRS_OSCP::crc_check(const uint8_t* frame, size_t len) {
    if (len < 3) {
        return false;
    }

    const uint16_t computed = oscp_crc16(frame, len - 2);
    const uint16_t received = (frame[len - 1] << 8) | frame[len - 2];
    return computed == received;
}

void AP_ExternalAHRS_OSCP::decode_raw(oscp_frame_t *frame, const uint8_t *data) {
    memcpy(&frame->content.raw, data, sizeof(oscp_raw_t));
    const oscp_raw_t *raw = &frame->content.raw;

    if (raw->status != OSCP_STATUS_OK) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Startup status error 0x%02X", (unsigned)raw->status);
        setup_complete = false;
        return;
    }

    last_raw_pkt = AP_HAL::millis();

    {
        WITH_SEMAPHORE(state.sem);
        state.accel = Vector3f{raw->accel_x, raw->accel_y, raw->accel_z} * GRAVITY_MSS;
        state.gyro = Vector3f{raw->gyro_x, raw->gyro_y, raw->gyro_z} * DEG_TO_RAD;
    }

    {
        AP_ExternalAHRS::ins_data_message_t ins;
        ins.accel = state.accel;
        ins.gyro = state.gyro;
        ins.temperature = raw->temp;
        AP::ins().handle_external(ins);

        AP_ExternalAHRS::mag_data_message_t mag;
        mag.field = Vector3f(raw->mag_x, raw->mag_y, raw->mag_z) * 10.0f;
        AP::compass().handle_external(mag);
    }
}

void AP_ExternalAHRS_OSCP::decode_startup(oscp_frame_t *frame, const uint8_t *data) {
    memcpy(&frame->content.startup, data, sizeof(oscp_startup_t));
    const oscp_startup_t *start_up = &frame->content.startup;

    memcpy(mark_number, start_up->mark_number, 10);
    mark_number[10] = '\0';

    hal.util->snprintf(oscp_device_name, sizeof(oscp_device_name), "%s-%u", mark_number, (unsigned)start_up->unit_number);
}

void AP_ExternalAHRS_OSCP::oscp_parser_go(oscp_parser_t* _parser) {
    _parser->buf_len = 0;
    _parser->synced = true;
}

void AP_ExternalAHRS_OSCP::oscp_parser_reset(oscp_parser_t* _parser) {
    _parser->buf_len = 0;
    _parser->synced = false;
}

void AP_ExternalAHRS_OSCP::oscp_parser_init(oscp_parser_t *_parser) {
    memset(_parser, 0, sizeof(oscp_parser_t));
}

const char* AP_ExternalAHRS_OSCP::get_name() const {
    if (oscp_device_name[0] != '\0') {
        return oscp_device_name;
    }
    return "OSCP";
}

int8_t AP_ExternalAHRS_OSCP::get_port(void) const {
    if (uart == nullptr) {
        return -1;
    }
    return port_num;
}

#endif  // AP_EXTERNAL_AHRS_OSCP_ENABLED