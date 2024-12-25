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
 * @file camera_gpio.h
 *
 * @NO{S10E02C07}
 * @ASIL{B}
 */

#ifndef __CAMERA_GPIO_H__
#define __CAMERA_GPIO_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CAMERA_GPIO_IN		(0)
#define CAMERA_GPIO_OUT		(1)
#define CAMERA_GPIO_LOW		(0)
#define CAMERA_GPIO_HIGH	(1)

extern int32_t camera_gpio_export(uint32_t gpio);
extern int32_t camera_gpio_unexport(uint32_t gpio);
extern int32_t camera_gpio_set_dir(uint32_t gpio, uint32_t out_flag);
extern int32_t camera_gpio_set_value(uint32_t gpio, uint32_t value);
extern int32_t camera_gpio_get_value(uint32_t gpio, uint32_t *value);
extern int32_t camera_gpio_set_edge(uint32_t gpio, const char *edge);

extern int32_t camera_gpio_power_ctrl(uint32_t gpio, int32_t on_off);

#ifdef __cplusplus
}
#endif

#endif /* __CAMERA_GPIO_H__ */
