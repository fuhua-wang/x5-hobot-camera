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
 * Copyright 2024 D-Robotics.
 * All rights reserved.
 ***************************************************************************/
#define pr_fmt(fmt) "[irs2381c]:" fmt

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <assert.h>
#include <getopt.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/i2c-dev.h>
#include "hb_i2c.h"
#include "hb_cam_utility.h"
#include "inc/irs2381c_setting.h"
#include "inc/sensor_effect_common.h"
#include "hb_camera_data_config.h"

#define IRS2381C_MCLK (24000000)
static int sensor_write_reg(sensor_info_t *sensor_info, uint16_t addr, uint16_t val)
{
	int ret = RET_OK;
	int k = 0;

	if (!sensor_info)
		return -RET_ERROR;

	int bus = sensor_info->bus_num;
	int i2c_addr = sensor_info->sensor_addr;

	vin_dbg("sensor name: %s, i2c write, bus: %d, i2c_addr: %d, reg: 0x%x, val: 0x%x\n",
			sensor_info->sensor_name, bus, i2c_addr, addr, val);

	if (addr != REG_NULL)
	{
		k = CAM_I2C_RETRY_MAX;
		do {
			ret = camera_i2c_write_reg16_data16(bus, i2c_addr, addr, val);
			if (ret == RET_OK) {
				vin_dbg("write sensor %s register 0x%2x\n",
						sensor_info->sensor_name, addr);
				break;
			} else {
				vin_err("write sensor %s register 0x%2x fail try %d\n",
						sensor_info->sensor_name, addr, (CAM_I2C_RETRY_MAX - k));
			}
			camera_sys_msleep(20);
		}while (k--);

		if (ret < 0)
		{
			vin_err("write sensor %s register 0x%2x fail\n",
					sensor_info->sensor_name, addr);
		}
	}
	else
	{
		usleep(val * 1000);
	}

	return ret;
}

static int sensor_read_reg(sensor_info_t *sensor_info, uint16_t addr)
{
	int ret = RET_OK;

	if (!sensor_info)
		return -RET_ERROR;

	int bus = sensor_info->bus_num;
	int i2c_addr = sensor_info->sensor_addr;

	if (addr != REG_NULL)
	{
		int val = camera_i2c_read_reg16_data16(bus, i2c_addr, addr);
		vin_dbg("sensor name: %s, i2c read, bus: %d, i2c_addr: %d, reg: 0x%x, val: 0x%x\n",
				sensor_info->sensor_name, bus, i2c_addr, addr, val);
	}
	return ret;
}

static int sensor_write_array(sensor_info_t *sensor_info,
							  const struct regval *regs, int array_size)
{
	int ret = RET_OK;
	int i = 0;

	if (!sensor_info || !regs)
		return -RET_ERROR;

	while (i < array_size)
	{
		ret = sensor_write_reg(sensor_info, regs[i].addr, regs[i].val);
		if (ret < 0)
		{
			vin_err("sensor_write_array: Failed to write register\n");
			break;
		}
		i++;
	}

	return ret;
}

static int sensor_read_array(sensor_info_t *sensor_info,
							 struct regval *regs, int array_size)
{
	int ret = RET_OK;
	int i = 0;

	if (!sensor_info || !regs)
		return -RET_ERROR;

	while (i < array_size)
	{
		ret = sensor_read_reg(sensor_info, regs[i].addr);
		if (ret < 0)
		{
			vin_err("sensor_read_array: Failed to read register\n");
			break;
		}
		i++;
	}

	return ret;
}


static int irs2381c_linear_data_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	uint32_t  open_cnt = 0;
	sensor_turning_data_t turning_data;
	uint32_t *stream_on = turning_data.stream_ctrl.stream_on;
	uint32_t *stream_off = turning_data.stream_ctrl.stream_off;

	memset(&turning_data, 0, sizeof(sensor_turning_data_t));

	// common data
	turning_data.bus_num = sensor_info->bus_num;
	turning_data.bus_type = sensor_info->bus_type;
	turning_data.port = sensor_info->port;
	turning_data.reg_width = sensor_info->reg_width;
	//SLAVE MODE is just for custom, kernel should be keep same with NORMAL_M
	if (sensor_info->sensor_mode == SLAVE_M)
		turning_data.mode = NORMAL_M;  //NOTICE
	else
		turning_data.mode = sensor_info->sensor_mode;
	turning_data.sensor_addr = sensor_info->sensor_addr;
	strncpy(turning_data.sensor_name, sensor_info->sensor_name,
		sizeof(turning_data.sensor_name));

	turning_data.stream_ctrl.data_length = 2;

	if (sensor_info->sensor_mode == SLAVE_M) {
		if(sizeof(turning_data.stream_ctrl.stream_on) >= sizeof(irs2381c_slave_mode_stream_on_setting)) {
			memcpy(stream_on, irs2381c_slave_mode_stream_on_setting, sizeof(irs2381c_slave_mode_stream_on_setting));
		} else {
			vin_err("Number of registers on stream over 10\n");
			return -RET_ERROR;
		}
	} else  {
		if(sizeof(turning_data.stream_ctrl.stream_on) >= sizeof(irs2381c_stream_on_setting)) {
			memcpy(stream_on, irs2381c_stream_on_setting, sizeof(irs2381c_stream_on_setting));
		} else {
			vin_err("Number of registers on stream over 10\n");
			return -RET_ERROR;
		}
	}

	if (sensor_info->sensor_mode == SLAVE_M) {
		if(sizeof(turning_data.stream_ctrl.stream_off) >= sizeof(irs2381c_slave_mode_stream_off_setting)) {
			memcpy(stream_off, irs2381c_slave_mode_stream_off_setting, sizeof(irs2381c_slave_mode_stream_off_setting));
		} else {
			vin_err("Number of registers on stream over 10\n");
			return -RET_ERROR;
		}
	} else  {
		if(sizeof(turning_data.stream_ctrl.stream_off) >= sizeof(irs2381c_stream_off_setting)) {
			memcpy(stream_off, irs2381c_stream_off_setting, sizeof(irs2381c_stream_off_setting));
		} else {
			vin_err("Number of registers on stream over 10\n");
			return -RET_ERROR;
		}
	}

	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);
	if (ret < 0) {
		vin_err("%s sync turning param ioctl fail %d\n", sensor_info->sensor_name, ret);
		return -RET_ERROR;
	}

	return ret;
}

int sensor_poweroff(sensor_info_t *sensor_info)
{
	int gpio, ret = RET_OK;
	if (sensor_info->gpio_num > 0)
	{
		for (gpio = 0; gpio < sensor_info->gpio_num; gpio++)
		{
			vin_dbg("sensor gpio: %d/%d %d %d %d \n", gpio, sensor_info->gpio_num,
						sensor_info->gpio_pin[gpio],
						sensor_info->gpio_level[gpio],
						sensor_info->gpio_level[gpio]);
			if (sensor_info->gpio_pin[gpio] != -1)
			{
				ret = vin_power_ctrl(sensor_info->gpio_pin[gpio],
									 sensor_info->gpio_level[gpio]);
				if (ret < 0)
				{
					vin_err("vin_power_ctrl fail\n");
					return -1;
				}
			}
		}
	}

	return ret;
}

int sensor_poweron(sensor_info_t *sensor_info)
{
	int gpio, ret = RET_OK;
	if (sensor_info->gpio_num > 0)
	{
		for (gpio = 0; gpio < sensor_info->gpio_num; gpio++)
		{
			vin_dbg("sensor gpio: %d/%d %d %d %d \n", gpio, sensor_info->gpio_num,
						sensor_info->gpio_pin[gpio],
						sensor_info->gpio_level[gpio],
						sensor_info->gpio_level[gpio]);
			if (sensor_info->gpio_pin[gpio] != -1)
			{
				ret = vin_power_ctrl(sensor_info->gpio_pin[gpio],
									 sensor_info->gpio_level[gpio]);
				usleep(100 * 1000); // 100ms
				ret |= vin_power_ctrl(sensor_info->gpio_pin[gpio],
									  1 - sensor_info->gpio_level[gpio]);
				if (ret < 0)
				{
					vin_err("vin_power_ctrl fail\n");
					return -HB_CAM_SENSOR_POWERON_FAIL;
				}
				usleep(100 * 1000); // 100ms
			}
		}
	}

	return ret;
}

static int sensor_configure(sensor_info_t *sensor_info, const struct regval *regs, int array_size)
{
	int ret = RET_OK;

	vin_info("sensor %s enable fps: %d, setting_size = %d,bus_num = %d, sensor_addr = 0x%0x\n",
		sensor_info->sensor_name, sensor_info->fps, array_size,
		sensor_info->bus_num, sensor_info->sensor_addr);

	ret = sensor_write_array(sensor_info, regs, array_size);
	if (ret < 0)
	{
		vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
	}

	return ret;
}

int sensor_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

	ret = sensor_poweron(sensor_info);
	if (ret < 0)
	{
		vin_err("%d : sensor reset %s fail\n",
				__LINE__, sensor_info->sensor_name);
		return ret;
	}

	switch (sensor_info->fps)
	{
	case 5:
		ret = sensor_configure(sensor_info,
			irs2381c_224_173_5fps_setting,
			ARRAY_SIZE(irs2381c_224_173_5fps_setting));
		break;
	default:
		vin_err("sensor %s dose not support %d fps\n",
			sensor_info->sensor_name, sensor_info->fps);
		ret = -RET_ERROR;
		break;
	}
	ret = irs2381c_linear_data_init(sensor_info);
	if (ret < 0) {
		vin_err("%d : linear data init %s fail\n", __LINE__, sensor_info->sensor_name);
		return -HB_CAM_INIT_FAIL;
	}

	vin_info("sensor %s config success under fps %d\n",
		sensor_info->sensor_name, sensor_info->fps);

	return ret;
}
// start stream
int sensor_start(sensor_info_t *sensor_info)
{
	int ret = RET_OK;

	vin_info("sensor %s stream on, mode = %d\n", sensor_info->sensor_name, sensor_info->sensor_mode);
	if (sensor_info->sensor_mode == SLAVE_M) //SLAVE_M=6, NOTICE: you should config this mode in user program.
	{
		ret = sensor_write_array(sensor_info, irs2381c_slave_mode_stream_on_setting, ARRAY_SIZE(irs2381c_slave_mode_stream_on_setting));
		if (ret < 0)
		{
			vin_err("sensor %s stream on failed\n", sensor_info->sensor_name);
		}
	} else {  // for tof master mode
		ret = sensor_write_array(sensor_info, irs2381c_stream_on_setting, ARRAY_SIZE(irs2381c_stream_on_setting));
		if (ret < 0)
		{
			vin_err("sensor %s stream on failed\n", sensor_info->sensor_name);
		}
	}
	return ret;
}

int sensor_stop(sensor_info_t *sensor_info)
{
	int ret = RET_OK;

	vin_err("sensor %s stream off\n", sensor_info->sensor_name);
	if (sensor_info->sensor_mode == SLAVE_M) //SLAVE_M=6, NOTICE: you should config this mode in user program.
	{
		ret = sensor_write_array(sensor_info, irs2381c_slave_mode_stream_off_setting, ARRAY_SIZE(irs2381c_slave_mode_stream_off_setting));
		if (ret < 0)
		{
			vin_err("sensor %s stream on failed\n", sensor_info->sensor_name);
		}
	} else {  // for tof master mode
		ret = sensor_write_array(sensor_info, irs2381c_stream_off_setting, ARRAY_SIZE(irs2381c_stream_off_setting));
		if (ret < 0)
		{
			vin_err("sensor %s stream on failed\n", sensor_info->sensor_name);
		}
	}
	return ret;
}

int sensor_deinit(sensor_info_t *sensor_info)
{
	int ret = RET_OK;

	ret = sensor_poweroff(sensor_info);
	if (ret < 0)
	{
		vin_err("%d : deinit %s fail\n",
				__LINE__, sensor_info->sensor_name);
		return ret;
	}
	return ret;
}

#ifdef CAMERA_FRAMEWORK_HBN
SENSOR_MODULE_F(irs2381c, CAM_MODULE_FLAG_A16D16);
sensor_module_t irs2381c = {
	.module = SENSOR_MNAME(irs2381c),
#else
sensor_module_t irs2381c = {
	.module = "irs2381c",
#endif
	.init = sensor_init,
	.start = sensor_start,
	.stop = sensor_stop,
	.deinit = sensor_deinit,
	.power_on = sensor_poweron,
	.power_off = sensor_poweroff
};
