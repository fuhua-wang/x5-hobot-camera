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
 * @file hb_cam_utility.h
 *
 * @NO{S10E02C07}
 * @ASIL{B}
 */

/* common data type */
#include "cam_common.h"

/* dummy for legacy header */
#include "camera_legacy.h"

#ifdef CAM_DIAG
/* diag enable */
#include "cam_diag.h"
#endif

// pre board id
#define	BOARD_ID_MATRIXDUO_A	(0x631)
#define	BOARD_ID_MATRIXDUO_B	(0x632)
#define	BOARD_ID_MATRIXDSOLO	(0x641)
// a sample board id
#define BOARD_ID_MATRIXDUO_A_V2	(0X651)
#define	BOARD_ID_MATRIXDUO_B_V2	(0X652)
#define	BOARD_ID_MATRIXDSOLO_V2	(0X642)
// b sample board id
#define BOARD_ID_MATRIXDUO_A_V3	(0X653)
#define BOARD_ID_MATRIXDUO_B_V3	(0X654)
#define BOARD_ID_MATRIXSOLO_V3	(0X643)
