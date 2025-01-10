// Copyright (c) 2024，D-Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file camera_mod_sensor_emdata.h
 *
 * @NO{S10E02C04}
 * @ASIL{B}
 */

#ifndef __CAMERA_MOD_SENSOR_EMDATA_H__
#define __CAMERA_MOD_SENSOR_EMDATA_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct emode_data_s {
	uint8_t serial_addr;
	uint8_t sensor_addr;
	uint8_t eeprom_addr;
	uint8_t serial_rclk_out;
	uint32_t rclk_mfp;
#ifdef SENSOR_CUSTOM_EMODE_DATA
	SENSOR_CUSTOM_EMODE_DATA;
#endif
} emode_data_t;


#ifdef __cplusplus
}
#endif

#endif /* __CAMERA_MOD_SENSOR_EMDATA_H__ */

