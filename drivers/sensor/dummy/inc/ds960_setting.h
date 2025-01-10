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

/*
 *    COPYRIGHT NOTICE
 *   Copyright 2019 Horizon Robotics, Inc.
 *    All rights reserved.
 */

#ifndef UTILITY_SENSOR_INC_DS960_SETTING_H_
#define UTILITY_SENSOR_INC_DS960_SETTING_H_

#ifdef __cplusplus
extern "C" {
#endif

static uint32_t ds960_ar0233_x3_4lane_init_setting[] =
{
	0x01, 0x03,  // reset
	0x1f, 0x00,  // csi 1600M
	/***config csi port0****/
	0x32, 0x01,  // csi port0 config enable
	0x33, 0x03,  // 4 lane enable csi for csi port0
	/***config rx port0****/
	0x4c, 0x01,  // rx port0
	0x58, 0x5E,  // pass thou bc 50M  953
	0x5C, 0x30,  // 953 alias addr
	0x5D, 0x20,  // 0233 slave addr
	0x65, 0x20,  // 0233 alias addr
	0x6d, 0x44,  // csi mode
	0x6E, 0x99,
	0x72, 0x00,  // 00 00 00 00
	/***config rx port1****/
	0x4c, 0x12,  // rx port1
	0x58, 0x5E,  // pass thou bc 50M 953
	0x5C, 0x32,  // 953 alias addr
	0x5D, 0x20,  // 0233 slave addr
	0x65, 0x22,  // 0233 alias addr
	0x6d, 0x44,  // csi mode
	0x6E, 0x99,
	0x72, 0x55,  // 01 01 01 01
	/***config rx port2****/
	0x4c, 0x24,  // rx port2
	0x58, 0x5E,  // pass thou bc 50M
	0x5C, 0x34,  // 953 alias addr
	0x5D, 0x20,  // 0233 slave addr
	0x65, 0x24,  // 0233 alias addr
	0x6d, 0x44,  // csi mode
	0x6E, 0x99,
	0x72, 0xaa,  // 02 02 02 02
	/***config rx port3****/
	0x4c, 0x38,  // rx port3
	0x58, 0x5E,  // pass thou bc 50M
	0x5C, 0x36,  // 953 alias addr
	0x5D, 0x20,  // 0233 slave addr
	0x65, 0x26,  // 0233 alias addr
	0x6d, 0x44,  // csi mode
	0x6E, 0x99,
	0x72, 0xff,  // 03 03 03 03
	0x20, 0xf0   // disable all when init
};

static uint32_t ds960_ar0233_x3_2lane_init_setting[] =
{
	0x01, 0x03,  // reset
	0x1f, 0x00,  // csi 1600M
	/***config csi port0****/
	0x32, 0x01,  // csi port0 config enable
	0x33, 0x23,  // 2 lane enable csi for csi port0
	/***config rx port0****/
	0x4c, 0x01,  // rx port0
	0x58, 0x5E,  // pass thou bc 50M  953
	0x5C, 0x30,  // 953 alias addr
	0x5D, 0x20,  // 0233 slave addr
	0x65, 0x20,  // 0233 alias addr
	0x6d, 0x44,  // csi mode
	0x6E, 0x99,
	0x72, 0x00,  // 00 00 00 00
	/***config rx port1****/
	0x4c, 0x12,  // rx port1
	0x58, 0x5E,  // pass thou bc 50M 953
	0x5C, 0x32,  // 953 alias addr
	0x5D, 0x20,  // 0233 slave addr
	0x65, 0x22,  // 0233 alias addr
	0x6d, 0x44,  // csi mode
	0x6E, 0x99,
	0x72, 0x55,  // 01 01 01 01
	/***config rx port2****/
	0x4c, 0x24,  // rx port2
	0x58, 0x5E,  // pass thou bc 50M
	0x5C, 0x34,  // 953 alias addr
	0x5D, 0x20,  // 0233 slave addr
	0x65, 0x24,  // 0233 alias addr
	0x6d, 0x44,  // csi mode
	0x6E, 0x99,
	0x72, 0xaa,  // 02 02 02 02
	/***config rx port3****/
	0x4c, 0x38,  // rx port3
	0x58, 0x5E,  // pass thou bc 50M
	0x5C, 0x36,  // 953 alias addr
	0x5D, 0x20,  // 0233 slave addr
	0x65, 0x26,  // 0233 alias addr
	0x6d, 0x44,  // csi mode
	0x6E, 0x99,
	0x72, 0xff,  // 03 03 03 03

	0x20, 0xf0   // disable all when init
};

static uint32_t ds960_ar0233_zu3_init_setting[] =
{
	0x01, 0x03,  // reset
	0x1f, 0x03,  // csi 400M
	/***config csi port0****/
	0x32, 0x01,  // csi port0 config enable
	0x33, 0x03,  // 4 lane enable csi for csi port0
	/***config rx port0****/
	0x4c, 0x01,  // rx port0
	0x58, 0x5E,  // pass thou bc 50M  953
	0x5C, 0x30,  // 953 alias addr
	0x5D, 0x20,  // 0233 slave addr
	0x65, 0x20,  // 0233 alias addr
	0x6d, 0x44,  // csi mode
	0x6E, 0x99,
	0x72, 0x00,  // 00 00 00 00
	/***config rx port1****/
	0x4c, 0x12,  // rx port1
	0x58, 0x5E,  // pass thou bc 50M 953
	0x5C, 0x32,  // 953 alias addr
	0x5D, 0x20,  // 0233 slave addr
	0x65, 0x22,  // 0233 alias addr
	0x6d, 0x44,  // csi mode
	0x6E, 0x99,
	0x72, 0x55,  // 01 01 01 01
	/***config rx port2****/
	0x4c, 0x24,  // rx port2
	0x58, 0x5E,  // pass thou bc 50M
	0x5C, 0x34,  // 953 alias addr
	0x5D, 0x20,  // 0233 slave addr
	0x65, 0x24,  // 0233 alias addr
	0x6d, 0x44,  // csi mode
	0x6E, 0x99,
	0x72, 0xaa,  // 02 02 02 02
	/***config rx port3****/
	0x4c, 0x38,  // rx port3
	0x58, 0x5E,  // pass thou bc 50M
	0x5C, 0x36,  // 953 alias addr
	0x5D, 0x20,  // 0233 slave addr
	0x65, 0x26,  // 0233 alias addr
	0x6d, 0x44,  // csi mode
	0x6E, 0x99,
	0x72, 0xff,  // 03 03 03 03
	0x20, 0xf0   // disable all when init
};

static uint32_t ds960_ov10635_line_concatenated_setting[] =
{
	0x01, 0x03,  // reset
	0x1f, 0x00,  // csi 1600M
	/***config csi port0****/
	0x32, 0x03,  // csi Tx port0 ��port1 config enable
	0x33, 0x61,  // 2 lane enable csi for csi port0 continue clk
	/***config rx port0****/
	0x4c, 0x01,  // rx port0
	0x58, 0x58,  // pass thou bc 913  2.5MHZ
	0x5C, 0x38,  // 953 alias addr 0x1c
	0x5D, 0x60,  // 10635 slave addr
	0x65, 0x80,  // 10635 alias addr  0x40
	0x7c, 0x81 ,  // FV active low 8bit
	0x70, 0x1E ,  // VC0 and CSI0 datatype 0x1e yuv422_8b VC0->port0
	0x6d, 0x7F ,  // 913A 10-bit mode, FPD_MODE  bit0 bit 1
	0x72, 0x00,   // 00 00 00 00 mmap vc_0

	/***config rx port1****/
	0x4c, 0x12,  // rx port1
	0x58, 0x58,  // pass thou bc 913 2.5MHZ
	0x5C, 0x3A,  // 953 alias addr 0x1d
	0x5D, 0x60,  // 10635 slave addr
	0x65, 0x82,  // 10635 alias addr 0x41
	0x7c, 0x81 ,  // FV active low 8bit
	0x70, 0x1E ,  // VC1 and CSI0 datatype 0x1e yuv422_8b VC0->port1
	0x6d, 0x7F , // 913A 10-bit mode, FPD_MODE bit0 bit 1
	0x72, 0x00,  // 00 00 00 00 mmap vc_0

	/***config rx port2****/
	0x4c, 0x24,  // rx port2
	0x58, 0x58,  // pass thou bc 2.5MHZ
	0x5C, 0x3C,  // 953 alias addr 0x1E
	0x5D, 0x60,  // 10635 slave addr
	0x65, 0x84,  // 10635 alias addr 0x42
	0x7c, 0x81 ,  // FV active low 8bit  yuv
	0x70, 0x5E ,  // VC2 and CSI0 datatype 0x1e yuv422_8b  VC1->port2
	0x6d, 0x7F , // 913A 10-bit mode, FPD_MODE bit0 bit 1
	0x72, 0x00,  // 00 00 00 00 mmap vc_0

	/***config rx port3****/
	0x4c, 0x38,  // rx port3
	0x58, 0x58,  // pass thou bc 2.5MHZ
	0x5C, 0x3E,  // 953 alias addr  0x1F
	0x5D, 0x60,  // 10635 slave addr
	0x65, 0x86,  // 10635 alias addr 0x43
	0x7c, 0x81 ,  // FV active low 8bit
	0x70, 0x5E ,  // VC3 and CSI0 datatype 0x1e yuv422_8b  VC1->port3
	0x6d, 0x7F , // 913A RAW10-bit mode(yuv), FPD_MODE  bit0 bit 1
	0x72, 0x00,  // 00 00 00 00 mmap vc_0

	/***config rx port0****/
	0x4c, 0x01,  // rx port0
	0x6e, 0xA8,    // 913 GPIO1----10635 FSIN
	// 0x10, 0x91,   //

	/***config rx port1****/
	0x4c, 0x12,  // rx port0
	0x6e, 0xA8,    // 913 GPIO1----10635 FSIN

	/***config rx port2****/
	0x4c, 0x24,  // rx port2
	0x6e, 0xA8,    // 913 GPIO1----10635 FSIN

	/***config rx port3****/
	0x4c, 0x38,  // rx port3
	0x6e, 0xA8,    // 913 GPIO1----10635 FSIN

	0x19, 0x06,    // FS_HIGH_TIME1
	0x1A, 0x83,    // FS_HIGH_TIME0
	0x1B, 0x04,    // FS_LOW_TIME1
	0x1C, 0x57,    // FS_LOW_TIME0
    0x18, 0x01,    // FrameSync enable
	0x21, 0x3C,    // Synchronized Basic_FWD

	/***forward***/
	0x20, 0xfC	 // disable all when init, port0,port1 map CSI-0, port2,port3 map CSI-1
};

static uint32_t ds960_ov10635_init_setting[] =
{
	0x01, 0x03,  // reset
	0x1f, 0x00,  // csi 1600M
	/***config csi port0****/
	0x32, 0x01,  // csi Tx port0 config enable
	0x33, 0x61,  // 2 lane enable csi for csi port0 continue clk
	/***config rx port0****/
	0x4c, 0x01,  // rx port0
	0x58, 0x58,  // pass thou bc 913
	0x5C, 0x38,  // 953 alias addr 0x1c
	0x5D, 0x60,  // 10635 slave addr
	0x65, 0x80,  // 10635 alias addr  0x40
	0x7c, 0x81 ,  // FV active low 8bit
	0x70, 0x1E ,  // VC0 and CSI0 datatype 0x1e yuv422_8b VC0->port0
	0x6d, 0x7F ,  // 913A 10-bit mode, FPD_MODE  bit0 bit 1
	0x72, 0x00,  // 00 00 00 00 mmap vc_0
	0x6e, 0x08,  // 913 GPIO1----back channel
	/***config rx port1****/
	0x4c, 0x12,  // rx port1
	0x58, 0x58,  // pass thou bc 913
	0x5C, 0x3A,  // 953 alias addr 0x1d
	0x5D, 0x60,  // 10635 slave addr
	0x65, 0x82,  // 10635 alias addr 0x41
	0x7c, 0x81 ,  // FV active low 8bit
	0x70, 0x5E ,  // VC1 and CSI0 datatype 0x1e yuv422_8b VC1->port1
	0x6d, 0x7F , // 913A 10-bit mode, FPD_MODE bit0 bit 1
	0x72, 0x55,  // 01 01 01 01
	0x6e, 0x08,  // 913 GPIO1----back channel
	/***config rx port2****/
	0x4c, 0x24,  // rx port2
	0x58, 0x58,  // pass thou bc 50M
	0x5C, 0x3C,  // 953 alias addr 0x1E
	0x5D, 0x60,  // 10635 slave addr
	0x65, 0x84,  // 10635 alias addr 0x42
	0x7c, 0x81 ,  // FV active low 8bit  yuv
	0x70, 0x9E ,  // VC2 and CSI0 datatype 0x1e yuv422_8b  VC2->port2
	0x6d, 0x7F , // 913A 10-bit mode, FPD_MODE bit0 bit 1
	0x72, 0xaa,  // 02 02 02 02
	0x6e, 0x08,  // 913 GPIO1----back channel
	/***config rx port3****/
	0x4c, 0x38,  // rx port3
	0x58, 0x58,  // pass thou bc 50M
	0x5C, 0x3E,  // 953 alias addr  0x1F
	0x5D, 0x60,  // 10635 slave addr
	0x65, 0x86,  // 10635 alias addr 0x43
	0x7c, 0x81 ,  // FV active low 8bit
	0x70, 0xDE ,  // VC3 and CSI0 datatype 0x1e yuv422_8b  VC3->port3
	0x6d, 0x7F , // 913A RAW10-bit mode(yuv), FPD_MODE  bit0 bit 1
	0x72, 0xff,  // 03 03 03 03
	0x6e, 0x08,  // 913 GPIO1----back channel
	/***forward***/
	0x20, 0xf0	 // disable all when init
};

static uint32_t ds960_start_setting[] = {
	0x20, 0x00   // enable all RX port,forwarding csi port0
};

static uint32_t ds960_dummy_init_cs0_setting[] =
{
	0x01, 0x03,  // reset
	0x1f, 0x00,  // csi 1600M
	/***config csi port0****/
	0x32, 0x01,  // csi port0 config enable
	0x33, 0x03,  // 4 lane enable csi for csi port0
	/***config rx port0****/
	0x4c, 0x01,  // rx port0
	0x58, 0x5E,  // pass thou bc 50M
	0x5C, 0x30,  // 953 alias addr
	0x6d, 0x44,  // csi mode
	0x6E, 0x99,
	0x72, 0x00,  // 00 00 00 00
	/***config rx port1****/
	0x4c, 0x12,  // rx port1
	0x58, 0x5E,  // pass thou bc 50M
	0x5C, 0x32,  // 953 alias addr
	0x6d, 0x44,  // csi mode
	0x6E, 0x99,
	0x72, 0x55,  // 01 01 01 01
	/***config rx port2****/
	0x4c, 0x24,  // rx port2
	0x58, 0x5E,  // pass thou bc 50M
	0x5C, 0x34,  // 953 alias addr
	0x6d, 0x44,  // csi mode
	0x6E, 0x99,
	0x72, 0xaa,  // 10 10 10 10
	/***config rx port3****/
	0x4c, 0x38,  // rx port3
	0x58, 0x5E,  // pass thou bc 50M
	0x5C, 0x36,  // 953 alias addr
	0x6d, 0x44,  // csi mode
	0x6E, 0x99,
	0x72, 0xff,  // 11 11 11 11
	0x20, 0xf0   // disable all when init
};

static uint32_t ds960_dummy_start_cs0_setting[] = {
	0x20, 0x00   // enable all RX port,forwarding csi port0
};

static uint32_t ds960_dummy_init_cs1_setting[] =
{
	0x01, 0x03,  // reset
	0x1f, 0x00,  // csi 1600M
	/***config csi port0****/
	0x32, 0x12,  // csi port1 config enable
	0x33, 0x03,  // 4 lane enable csi for csi port1
	/***config rx port0****/
	0x4c, 0x01,  // rx port0
	0x58, 0x5E,  // pass thou bc 50M
	0x5C, 0x30,  // 953 alias addr
	0x6d, 0x44,  // csi mode
	0x6E, 0x99,
	0x72, 0x00,  // 00 00 00 00
	/***config rx port1****/
	0x4c, 0x12,  // rx port1
	0x58, 0x5E,  // pass thou bc 50M
	0x5C, 0x32,  // 953 alias addr
	0x6d, 0x44,  // csi mode
	0x6E, 0x99,
	0x72, 0x55,  // 01 01 01 01
	/***config rx port2****/
	0x4c, 0x24,  // rx port2
	0x58, 0x5E,  // pass thou bc 50M
	0x5C, 0x34,  // 953 alias addr
	0x6d, 0x44,  // csi mode
	0x6E, 0x99,
	0x72, 0xaa,  // 10 10 10 10
	/***config rx port3****/
	0x4c, 0x38,  // rx port3
	0x58, 0x5E,  // pass thou bc 50M
	0x5C, 0x36,  // 953 alias addr
	0x6d, 0x44,  // csi mode
	0x6E, 0x99,
	0x72, 0xff,  // 11 11 11 11
	0x20, 0xff   // disable all when init
};

static uint32_t ds960_dummy_start_cs1_setting[] = {
	0x20, 0x0f   // enable all RX port,forwarding csi port1
};

static uint32_t ds960_dummy_init_cs0p0_setting[] =
{
	0x01, 0x03,  // reset
	0x1f, 0x00,  // csi 1600M
	/***config csi port0****/
	0x32, 0x01,  // csi port0 config enable
	0x33, 0x41,  // 4 lane enable csi for csi port0
	0x34, 0x01,
	/***config rx port0****/
	0x4c, 0x01,  // rx port0
	0x58, 0x5E,  // pass thou bc 50M
	0x5C, 0x30,  // 953 alias addr
	0x6d, 0x44,  // csi mode
	0x6E, 0x99,
	0x72, 0xe4,  // 00 01 10 11
	0x20, 0xf0   // disable all when init
};

static uint32_t ds960_dummy_start_cs0p0_setting[] = {
	0x20, 0xe0   // enable RX port0,forwarding csi port0
};

static uint32_t ds960_dummy_init_cs1p0_setting[] =
{
	0x01, 0x03,  // reset
	0x1f, 0x00,  // csi 1600M
	/***config csi port0****/
	0x32, 0x12,  // csi port1 config enable
	0x33, 0x41,  // 4 lane enable csi for csi port1
	/***config rx port0****/
	0x4c, 0x01,  // rx port0
	0x58, 0x5E,  // pass thou bc 50M
	0x5C, 0x30,  // 953 alias addr
	0x6d, 0x44,  // csi mode
	0x6E, 0x99,
	0x72, 0xe4,  // 00 00 00 00
	0x20, 0xff   // disable all when init
};

static uint32_t ds960_dummy_start_cs1p0_setting[] = {
	0x20, 0xef   // enable RX port0,forwarding csi port1
};

static uint32_t ds960_dummy_stop_setting[] = {
	0x20, 0xf0   // disable all when init
};

static uint32_t ds960_ar0144AT_init_setting[] =
{
	0x01, 0x03,  // reset
	0x1F, 0x02,  // csi 800M
	/***config csi port0****/
	0x32, 0x01,  // csi Tx port0 config enable
	0x33, 0x41,  // 4 lane enable csi for csi port0
	/***config rx port0****/
	0x4C, 0x01,  // rx port0
	0x5C, 0x30,  // 953 alias addr 0x18
	0x5D, 0x20,  // ar0144AT slave addr
	0x65, 0x20,  // ar0144AT alias addr 0x10
	0x6E, 0x99,
	0x58, 0x58,  // pass thou bc 913
	0x6D, 0x7E,  // 913A 12-bit High Frequency mode, FPD_MODE  bit0 bit 1
	0x71, 0x2C,
	0x72, 0x00,  // 00 00 00 00 mmap vc_0
	/***config rx port1****/
	0x4C, 0x12,  // rx port1C
	0x5C, 0x32,  // 953 alias addr 0x19
	0x5D, 0x20,  // ar0144AT slave addr
	0x65, 0x22,  // ar0144AT alias addr 0x11
	0x6E, 0x99,
	0x58, 0x58,  // pass thou bc 913
	0x6D, 0x7E,  // 913AC 12-bit High Frequency mode, FPD_MODE  bit0 bit 1
	0x71, 0x6C,
	0x72, 0x55,  // 01 01 01 01
	/***config rAx port2****/
	0x4C, 0x24,  // rx port2
	0x5C, 0x34,  // 953 alias addr 0x1a
	0x5D, 0x20,  // ar0144AT slave addr
	0x65, 0x24,  // ar0144AT alias addr 0x12
	0x6E, 0x99,
	0x58, 0x58,  // pass thou bc 913
	0x6D, 0x7E,  // 913A 12-bit High Frequency mode, FPD_MODE  bit0 bit 1
	0x71, 0xAC,
	0x72, 0xAA,  // 02 02 02 02
	/***config rx port3****/
	0x4c, 0x38,  // rx port3
	0x5C, 0x36,  // 953 alias addr 0x1b
	0x5D, 0x20,  // ar0144AT slave addr
	0x65, 0x26,  // ar0144AT alias addr 0x13
	0x6E, 0x99,
	0x58, 0x58,  // pass thouFF bc 913
	0x6D, 0x7E,  // 913A 12-bit High Frequency mode, FPD_MODE  bit0 bit 1
	0x71, 0xEC,
	0x72, 0xFF,  // 03 03 03 03
	/***forward***/
	0x20, 0x00	 // enable all when init
};

#ifdef __cplusplus
}
#endif
#endif  // UTILITY_SENSOR_INC_DS960_SETTING_H_

