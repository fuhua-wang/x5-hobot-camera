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
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#ifndef UTILITY_SENSOR_INC_IMX390CSTD_SETTING_H_
#define UTILITY_SENSOR_INC_IMX390CSTD_SETTING_H_
#define POC_RESET_ADDR   (0x01)
#define DELAY_FLAG        (0xFFFF)
enum MODE_TYPE {
	SENSING_M24F120D4_S0R0T7,
	MODE_TYPE_MAX,
	MODE_TYPE_NUM,
};
#if 0
static uint32_t imx390cstd_init_setting[] = {
    0x0010, 0x21,
    0x02be, 0x00,
    0x02be, 0x18,
    0x0042, 0xa2,
    0x0043, 0x60,
    0x0044, 0xda,
    0x0045, 0x20,

    0x0330, 0x00,
    0x0331, 0x33,
    0x0332, 0xe0,
    0x0333, 0x04,
    0x0308, 0x64,
    0x0311, 0x40,
    0x0002, 0x43,
    0x0318, 0x5e,
    0x02d3, 0x00,
    0x02d3, 0x18,
    0x02d3, 0xe4,
    0x02d4, 0x67,
    0x02d5, 0x07,
    0x02d6, 0xe4,
    0x02d7, 0x68,
    0x02d8, 0x07,
};
#endif
static uint32_t imx390cstd_init_setting_1[] = {
    0x0042, 0xa2,
    0x0043, 0x60,
    0x0044, 0xda,
    0x0045, 0x20,
};
static uint32_t imx390cstd_init_setting_2[] = {
    0x0330, 0x00,
    0x0331, 0x33,
    0x0332, 0xe0,
    0x0333, 0x04,
    0x0308, 0x64,
    0x0311, 0x40,
    0x0002, 0x43,
    0x0318, 0x5e,
};
static uint32_t imx390cstd_init_setting_3[] = {
    0x02d3, 0xe4,
    0x02d4, 0x67,
    0x02d5, 0x07,
    0x02d6, 0xe4,
    0x02d7, 0x68,
    0x02d8, 0x07,
};
#endif  // UTILITY_SENSOR_INC_IMX390CSTD_SETTING_H_
