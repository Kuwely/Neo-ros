/*The MIT License (MIT)
 *
 * Copyright (c) 2017, micvision, LLC
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#ifndef _NEO_PROTOCOL_H_
#define _NEO_PROTOCOL_H_

#define NEO_RESPONSE_TYPE_DATA                'D'
#define NEO_RESPONSE_TYPE_INFO                'I'
#define NEO_RESPONSE_TYPE_MOTOR                'M'
#define NEO_RESPONSE_TYPE_LIDAR                'L'
#define NEO_RESPONSE_TYPE_RESET                'R'

#define NEO_RESPONSE_TYPE_DATA_START            'S'
#define NEO_RESPONSE_TYPE_DATA_STOP            'X'

#define NEO_RESPONSE_TYPE_INFO_MOTOR            'M'
#define NEO_RESPONSE_TYPE_INFO_VERSION        'V'
#define NEO_RESPONSE_TYPE_INFO_DEVICE            'D'

#define NEO_RESPONSE_TYPE_LIDAR_POWER            'P'
#define NEO_RESPONSE_TYPE_LIDAR_SETTINGS        'S'
#define NEO_RESPONSE_TYPE_LIDAR_INFO            'I'

#define NEO_RESPONSE_SCAN_NODE_SIZE 7

typedef struct _neo_cmd_packet_t
{
	uint8_t cmdByte1;
	uint8_t cmdByte2;
	uint8_t cmdParamTerm;
}__attribute__((packed)) neo_cmd_packet_t;

typedef struct _neo_cmd_param_packet_t
{
	uint8_t cmdByte1;
	uint8_t cmdByte2;
	uint8_t cmdParamByte1;
	uint8_t cmdParamByte2;
	uint8_t cmdParamTerm;
}__attribute__((packed)) neo_cmd_param_packet_t;

typedef struct _neo_response_header_t
{
	uint8_t cmdByte1;
	uint8_t cmdByte2;
	uint8_t cmdStatusByte1;
	uint8_t cmdStatusByte2;
	uint8_t cmdSum;
	uint8_t term1;

}__attribute__((packed)) neo_response_header_t;

typedef struct _neo_response_param_t
{
	uint8_t cmdByte1;
	uint8_t cmdByte2;
	uint8_t cmdParamByte1;
	uint8_t cmdParamByte2;
	uint8_t term1;
	uint8_t cmdStatusByte1;
	uint8_t cmdStatusByte2;
	uint8_t cmdSum;
	uint8_t term2;

}__attribute__((packed)) neo_response_param_t;

typedef struct _neo_response_scan_packet_t
{
	uint8_t sync_error;
	uint16_t angle;
	uint16_t distance;
	uint8_t signal_strength;
	uint8_t checksum;
}__attribute__((packed)) neo_response_scan_packet_t;

typedef struct _neo_response_info_device_t
{
	uint8_t cmdByte1;
	uint8_t cmdByte2;
	uint8_t bit_rate[6];
	uint8_t laser_state;
	uint8_t mode;
	uint8_t diagnostic;
	uint8_t motor_speed[2];
	uint8_t sample_rate[4];
	uint8_t term;

}__attribute__((packed)) neo_response_info_device_t;

typedef struct _neo_response_info_version_t
{
	uint8_t cmdByte1;
	uint8_t cmdByte2;
	uint8_t model[5];
	uint8_t protocol_major;
	uint8_t protocol_min;
	uint8_t firmware_major;
	uint8_t firmware_minor;
	uint8_t hardware_version;
	uint8_t serial_no[8];
	uint8_t term;

}__attribute__((packed)) neo_response_info_version_t;

typedef struct _neo_response_info_motor_t
{
	uint8_t cmdByte1;
	uint8_t cmdByte2;
	uint8_t motor_speed[2];
	uint8_t term;

}__attribute__((packed)) neo_response_info_motor_t;

#endif // _NEO_PROTOCOL_H_
