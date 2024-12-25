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
 * @file sensor_effect_common.h
 *
 * @NO{S10E02C07}
 * @ASIL{B}
 */

/* dummy for legacy header */
#include "camera_mod_sensor_data.h"

#include "camera_sensor_dev.h"

/* des pre operation for sensor lib */
#include "camera_deserial_dev.h"
#undef hb_vin_mipi_pre_request
#define hb_vin_mipi_pre_request(entry_num, type, timeout)	camera_deserial_dev_pre_req(entry_num, type, timeout)
#undef hb_vin_mipi_pre_result
#define hb_vin_mipi_pre_result(entry_num, type, result)		camera_deserial_dev_pre_result(entry_num, type, result)
