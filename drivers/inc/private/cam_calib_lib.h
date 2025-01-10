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
 * @file cam_calib_lib.h
 *
 * @NO{S10E02C04}
 * @ASIL{B}
 */

#ifndef __CAM_CALIB_LIB_H__
#define __CAM_CALIB_LIB_H__

#include <stdint.h>

#include "camera_mod_calib.h"

#include "cam_handle.h"

#ifdef __cplusplus
extern "C" {
#endif

/* internal apis */
extern int32_t camera_calib_config_check(camera_module_lib_t *lib, const char *version);
extern int32_t camera_calib_ops_bind(camera_handle_st *hcam, calib_info_t *cal_if);
extern int32_t camera_calib_config_parse(camera_handle_st *hcam, calib_info_t *cal_if);

extern int32_t camera_calib_init(calib_info_t *cal_if);
extern int32_t camera_calib_deinit(calib_info_t *cal_if);
extern int32_t camera_calib_get_version(calib_info_t *cal_if, char *name, char *version);

extern int32_t camera_calib_set_cali_name_init(camera_module_lib_t *cal_lib);
extern int32_t camera_calib_set_cali_name_put(camera_module_lib_t *cal_lib, camera_calib_t *pcalib);
extern int32_t camera_calib_set_cali_name_deinit(camera_module_lib_t *cal_lib);

#ifdef __cplusplus
}
#endif

#endif /* __CAM_CALIB_LIB_H__ */


