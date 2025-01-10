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
 * @file camera_mod_poc_data.h
 *
 * @NO{S10E02C05}
 * @ASIL{B}
 */

#ifndef __CAMERA_MOD_POC_DATA_H__
#define __CAMERA_MOD_POC_DATA_H__

#include <stdint.h>

#include "cam_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define POC_LINK_TO_PORT(map, link) (((map) >> (link * 4)) & 0x3)

#ifdef CAM_CONFIG_INFO_LEGACY_COMPATIBLE
#define GPIO_NUMBER 6
#endif

/**
 * @struct poc_info_s
 * poc module data info struct
 * @NO{S10E02C05}
 */
typedef struct poc_info_s {
	uint32_t index;
	uint32_t bus_type;
	uint32_t bus_num;
	uint32_t poc_addr;
#ifdef CAM_CONFIG_INFO_LEGACY_COMPATIBLE
	uint32_t gpio_num;
	int32_t gpio_pin[GPIO_NUMBER];
	int32_t gpio_level[GPIO_NUMBER];
#endif
	uint32_t power_delay;
	uint32_t poc_map;
	char *poc_name;
	void *poc_ops;
	void *poc_fd;
	uint32_t reserved[16];
} poc_info_t;

#ifdef __cplusplus
}
#endif

#endif /* __CAMERA_MOD_POC_DATA_H__ */


