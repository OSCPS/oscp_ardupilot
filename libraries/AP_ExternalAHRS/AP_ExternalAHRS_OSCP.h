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

 /* Thank you for flying with OSCP. For integration steps, see https://docs.oscp.com/Integration/ArduPilot/ */

#pragma once

/* Includes */
#include "AP_ExternalAHRS_config.h"
#if AP_EXTERNAL_AHRS_OSCP_ENABLED
#include "AP_ExternalAHRS_backend.h"

class AP_ExternalAHRS_OSCP : public AP_ExternalAHRS_backend {

public:

    /* Static variables */
    static const uint16_t OSCP_CRC_LUT[256];
    static const uint8_t OSCP_FRAME_LEN[8];

    /* AP_ExternalAHRS_Backend overrides */
    AP_ExternalAHRS_OSCP(AP_ExternalAHRS* frontend, AP_ExternalAHRS::state_t& state);
    bool initialised(void) const override;
    bool healthy(void) const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    void update() override {}
    void update_thread();
    void get_filter_status(nav_filter_status &status) const override;
    const char* get_name(void) const override;
    int8_t get_port(void) const override;
    
    /* Returning 0 for the IMU devices - will be overriden with INS devices */
    uint8_t num_gps_sensors(void) const override { return 0; }

private:

    /* Frame type enum */
    enum class oscp_frame_type_t : uint8_t {
        OSCP_FRAME_RAW          = 0x00, // Raw operating frame
        OSCP_FRAME_EULER        = 0x01, // AHRS euler angles operating frame
        OSCP_FRAME_QUATERNION   = 0x02, // AHRS quaternions operating frame
        OSCP_FRAME_ROT_MATRIX   = 0x03, // AHRS rotation matrix operating frame
        OSCP_FRAME_DEBUG1       = 0x05, // Debug operating frame 1
        OSCP_FRAME_DEBUG2       = 0x06, // Debug operating frame 2
        OSCP_FRAME_STARTUP      = 0x07, // Startup frame
    };

    /* Operating mode enum */
    enum class oscp_operating_mode_t : uint8_t {
        OSCP_OP_MODE_IDLE   = 0x00,
        OSCP_OP_MODE_LOW    = 0x01,
        OSCP_OP_MODE_MEDIUM = 0x02,
    };

    /* Misalignment correction enum */
    enum class oscp_misalignment_corr_t : uint8_t {
        OSCP_MISALIGNMENT_CORR_DISABLED = 0x00,
        OSCP_MISALIGNMENT_CORR_ENABLED = 0x01,
    };

    /* Return code enum */
    enum class oscp_err_t : int8_t {
        OSCP_OK         = 0,
        OSCP_ERR        = -1,
        OSCP_ERR_NULL   = -2,
    };

    /* Raw Operating Frame - 61 bytes decoded */
    struct PACKED oscp_raw_t {
        uint8_t header_byte;    // Misalignment Correction (2B) - Operating Mode (2B) - Frame Type (3B) 
        uint8_t counter;        // Wrapping frame counter [0,255] 
        uint64_t timestamp_ms;  // Timestamp in ms since last power up or reset 
        float gyro_x;           // unit: dps 
        float gyro_y;           // unit: dps 
        float gyro_z;           // unit: dps 
        float accel_x;          // unit: g 
        float accel_y;          // unit: g 
        float accel_z;          // unit: g 
        float incl_x;           // unit: mg 
        float incl_y;           // unit: mg 
        float mag_x;            // unit: uT 
        float mag_y;            // unit: uT 
        float mag_z;            // unit: uT 
        float temp;             // unit: degC 
        uint8_t status;         // Status byte - use OSCP_STATUS_* masks 
        uint16_t crc;           // Checksum 
    };  

    /* Euler Angles Frame - 25 bytes decoded */
    struct PACKED oscp_euler_t {
        uint8_t header_byte;    // Misalignment Correction (2B) - Operating Mode (2B) - Frame Type (3B) 
        uint8_t counter;        // Wrapping frame counter [0,255] 
        uint64_t timestamp_ms;  // Timestamp in ms since last power up or reset 
        float roll;             // deg 
        float pitch;            // deg 
        float yaw;              // deg 
        uint8_t status;         // Status byte - use OSCP_STATUS_* masks 
        uint16_t crc;           // Checksum
    };
    
    /* Quaternion Frame - 29 bytes decoded */
    struct PACKED oscp_quat_t {
        uint8_t header_byte;    // Misalignment Correction (2b) - Operating Mode (2b) - Frame Type (3b)
        uint8_t counter;        // Wrapping frame counter [0,255] 
        uint64_t timestamp_ms;  // Timestamp in ms since last power up or reset 
        float w;
        float x;
        float y;
        float z;
        uint8_t status;         // Status byte - use OSCP_STATUS_* masks 
        uint16_t crc;           // Checksum 
    };
    
    /* Rotation Matrix Frame - 49 bytes decoded */
    struct PACKED oscp_rot_mat_t {
        uint8_t header_byte;    // Misalignment Correction (2b) - Operating Mode (2b) - Frame Type (3b)
        uint8_t counter;        // Wrapping frame counter [0,255] 
        uint64_t timestamp_ms;  // Timestamp in ms since last power up or reset
        float rm[3][3];
        uint8_t status;         // Status byte - use OSCP_STATUS_* masks
        uint16_t crc;           // Checksum data 
    };
    
    /* Startup Frame - 40 bytes decoded */
    struct PACKED oscp_startup_t {
        uint8_t header_byte;        // Misalignment Correction (2b) - Operating Mode (2b) - Frame Type (3b) 
        char mark_number[10];
        uint16_t unit_number;
        uint8_t sw_major_ver;
        uint8_t sw_minor_ver;
        uint8_t sw_patch_ver;
        uint8_t enabled_frames;     // Use OSCP_ENABLE_* masks
        uint8_t gyro_dr : 4;
        uint8_t accel_dr : 4;
        uint8_t gyro_filters : 2;
        uint8_t gyro_lpf : 3;
        uint8_t gyro_hpf : 3;
        uint8_t accel_filters : 2;
        uint8_t accel_lpf : 3;
        uint8_t accel_hpf : 3;
        uint8_t incl_dr : 4;
        uint8_t ahrs_convention : 2;
        uint8_t ahrs_heading_src : 2;
        float ahrs_gain;
        float ahrs_accel_rej;
        float ahrs_mag_rej;
        uint32_t ahrs_rec_trig_per; // Recovery Trigger Period in s
        uint8_t status;             // Status byte - use OSCP_STATUS_* masks
        uint16_t crc;               // Checksum
    };

    /* Tagged union for handling frame callback  */
    struct oscp_frame_t {
        oscp_frame_type_t type;
        union {
            oscp_raw_t     raw;
            oscp_euler_t   euler;
            oscp_quat_t    quat;
            oscp_rot_mat_t rot_mat;
            oscp_startup_t startup;
        } content;
    };

    /* Parser statistics */
    struct oscp_stats_t {
        uint32_t frames_ok;         // Number of frames successfully parsed
        uint32_t framing_errors;    // Number of frames dropped due to framing errors
        uint32_t crc_errors;        // Number of frames dropped due to CRC errors
        uint32_t cobs_errors;       // Number of frames dropped due to COBS errors
        uint32_t overflows;         // Number of frames dropped due to buffer overflow
    };

    /* OSCP parser */
    struct oscp_parser_t {
        uint8_t buf[72U];
        size_t buf_len;
        bool synced;
        oscp_stats_t stats;
    };

    /* Constant variables */
    static constexpr uint32_t OSCP_BAUD_RATE    = 921600U;
    static constexpr uint8_t OSCP_FRAME_DELIM   = 0x00U;
    static constexpr uint8_t OSCP_FRAME_MAX_LEN = 72U; 

    /* Status byte mask variables */
    static constexpr uint8_t OSCP_STATUS_OK         = 0x00U; // No bits are set
    static constexpr uint8_t OSCP_STATUS_OVERRUN    = 0x01U; // IMU real time controller overrun
    static constexpr uint8_t OSCP_STATUS_MEMS_ERR   = 0x02U; // MEMS sensors error
    static constexpr uint8_t OSCP_STATUS_INCL_ERR   = 0x04U; // Inclinometer error
    static constexpr uint8_t OSCP_STATUS_MAG_ERR    = 0x08U; // Magnetometer error
    static constexpr uint8_t OSCP_STATUS_TEMP_ERR   = 0x10U; // Temperature sensor error
    static constexpr uint8_t OSCP_STATUS_OG_ERR     = 0x40U; // Optical gyroscope error

    /* Enable frames mask variables */
    static constexpr uint8_t OSCP_ENABLE_RAW        = 0x01U;
    static constexpr uint8_t OSCP_ENABLE_EULER      = 0x02U;
    static constexpr uint8_t OSCP_ENABLE_QUAT       = 0x04U;
    static constexpr uint8_t OSCP_ENABLE_ROT_MAT    = 0x08U;

    /* Initialization parameters */
    bool setup_complete = false;
    AP_HAL::UARTDriver *uart = nullptr;
    oscp_operating_mode_t last_op_mode = oscp_operating_mode_t::OSCP_OP_MODE_IDLE; //Starting Point Reference = No Frame
    int8_t port_num = 0;
    uint32_t baudrate = 0;
    
    /* ASCII device print */
    char mark_number[11] = {};
    char full_device_name[32] = {};
    
    /* Raw IMU timer */
    uint32_t last_raw_pkt = 0;

    /* IMU configuration timer */ 
    uint32_t last_frame_ms = 0;         // Last CRC-check frame 
    uint32_t last_status_warn_ms = 0;   // Rate limiter For status byte warning(s)

    /* Accessors */
    inline oscp_frame_type_t get_frame_type(uint8_t hdr_byte) const {
        return (oscp_frame_type_t)(hdr_byte & 0x07U);
    };

    inline oscp_operating_mode_t get_operating_mode(uint8_t hdr_byte) const {
        return (oscp_operating_mode_t)((hdr_byte >> 3) & 0x07U);
    };

    inline oscp_misalignment_corr_t get_misalignment_corr(uint8_t hdr_byte) const {
        return (oscp_misalignment_corr_t)((hdr_byte >> 6) & 0x03U);
    };

    inline oscp_raw_t&     oscp_raw(oscp_frame_t *frame)     { return frame->content.raw; }
    inline oscp_euler_t&   oscp_euler(oscp_frame_t *frame)   { return frame->content.euler; }
    inline oscp_quat_t&    oscp_quat(oscp_frame_t *frame)    { return frame->content.quat; }
    inline oscp_rot_mat_t& oscp_rot_mat(oscp_frame_t *frame) { return frame->content.rot_mat; }
    inline oscp_startup_t& oscp_startup(oscp_frame_t *frame) { return frame->content.startup; }

    /* COBS utils */
    oscp_err_t oscp_cobs_decode(const uint8_t *data, size_t data_len, uint8_t *dec, size_t dec_max_len, size_t *dec_len);
    oscp_err_t oscp_cobs_encode(const uint8_t *data, size_t data_len, uint8_t *enc, size_t enc_max_len, size_t *enc_len); 
    void oscp_send_cobs_command(const char *cmd); 

    /* Serial utils */
    void drain_uart_data(oscp_parser_t &parser);

    /* IMU configuration */
    void oscp_configure_imu(oscp_parser_t &parser);

    /* Parser functions */
    static void oscp_parser_init(oscp_parser_t *parser);
    void oscp_parser_feed_buf(oscp_parser_t *parser, const uint8_t *buf, size_t buf_len);
    void oscp_parser_feed(oscp_parser_t *parser, uint8_t byte);
    void oscp_parser_buffer(oscp_parser_t *parser);
    void oscp_parser_go(oscp_parser_t *parser);
    void oscp_parser_reset(oscp_parser_t *parser);

    /* CRC utils*/
    uint16_t crc16(const uint8_t *data, size_t len);
    bool crc_check(const uint8_t *frame, size_t len);

    /* Parser decoding */
    void oscp_decode_raw(oscp_frame_t *frame, const uint8_t *data);
    void oscp_decode_startup(oscp_frame_t *frame, const uint8_t *data); 
};
 
#endif  // AP_EXTERNAL_AHRS_OSCP_ENABLED
