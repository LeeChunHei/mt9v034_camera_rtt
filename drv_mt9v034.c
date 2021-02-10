/*
 * Copyright (c) 2021 Lee Chun Hei, Leslie
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
 * OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <drv_mt9v034.h>

#ifdef PKG_USING_MT9V034_CAMERA
#include "fsl_common.h"

#define LOG_TAG              "drv.MT9V034"
#include <drv_log.h>

#define MT9V034_SLAVE_ADDR                  (0xB8 >> 1)
#define MT9V034_BYTEWISE_ADDR_REG_ADDR      0xF0

#define PKG_MT9V034_CSI_BASE            CSI

__attribute__((section("NonCacheable,\"aw\",%nobits @"))) static rt_uint8_t frame_buffer[CSI_DRIVER_QUEUE_SIZE][PKG_MT9V034_FRAME_WIDTH * PKG_MT9V034_FRAME_HEIGHT] __attribute__((aligned(64)));

void CSI_IRQHandler(void)
{
    CSI_DriverIRQHandler();
}

void mt9v034_csi_xfer_callback(CSI_Type *base, csi_handle_t *handle, status_t status, void *user_data)
{
    rt_mt9v034_device_t dev = (rt_mt9v034_device_t)user_data;
    if (dev->parent.rx_indicate)
        dev->parent.rx_indicate(dev, dev->width * dev->height);
}

static rt_err_t mt9v034_i2c_set_byte(struct rt_i2c_bus_device *bus, rt_uint8_t reg_addr, rt_uint8_t data)
{
    rt_uint8_t buff[2] = {reg_addr, data};
    struct rt_i2c_msg msgs;

    msgs.addr = MT9V034_SLAVE_ADDR;
    msgs.flags = RT_I2C_WR;
    msgs.buf = buff;
    msgs.len = 2;
    if (rt_i2c_transfer(bus, &msgs, 1) != 1)
    {
#ifdef PKG_MT9V034_ASSERT_ON_FAIL
        RT_ASSERT(RT_FALSE);
#endif
        return -RT_ERROR;
    }

    return RT_EOK;
}

static rt_err_t mt9v034_i2c_get_byte(struct rt_i2c_bus_device *bus, rt_uint8_t reg_addr, rt_uint8_t* result)
{
    struct rt_i2c_msg msgs;

    msgs.addr = MT9V034_SLAVE_ADDR;
    msgs.flags = RT_I2C_WR;
    msgs.buf = &reg_addr;
    msgs.len = 1;
    if (rt_i2c_transfer(bus, &msgs, 1) != 1)
    {
#ifdef PKG_MT9V034_ASSERT_ON_FAIL
        RT_ASSERT(RT_FALSE);
#endif
        return -RT_ERROR;
    }
    
    msgs.flags = RT_I2C_RD;
    msgs.buf = result;
    msgs.len = 1;
    if (rt_i2c_transfer(bus, &msgs, 1) != 1)
    {        
#ifdef PKG_MT9V034_ASSERT_ON_FAIL
        RT_ASSERT(RT_FALSE);
#endif
        return -RT_ERROR;
    }

    return RT_EOK;
}

static void mt9v034_reg_set(rt_mt9v034_device_t dev, rt_uint8_t reg_addr, rt_uint16_t value)
{
    rt_err_t status = mt9v034_i2c_set_byte(dev->i2c_bus, reg_addr, value >> 8) | \
            mt9v034_i2c_set_byte(dev->i2c_bus, MT9V034_BYTEWISE_ADDR_REG_ADDR, value & 0xFF);
    if (status != RT_EOK)
    {
#ifdef PKG_MT9V034_ASSERT_ON_FAIL
        RT_ASSERT(RT_FALSE);
#else
        LOG_E(LOG_TAG" failed to set register 0x%X to 0x%X", reg_addr, value);
#endif
    }
}

static void mt9v034_reg_get(rt_mt9v034_device_t dev, rt_uint8_t reg_addr, rt_uint16_t *result)
{
    rt_err_t status = RT_EOK;
    uint8_t temp_byte;
    status = mt9v034_i2c_get_byte(dev->i2c_bus, reg_addr, &temp_byte);
    *result = temp_byte << 8;
    status |= mt9v034_i2c_get_byte(dev->i2c_bus, MT9V034_BYTEWISE_ADDR_REG_ADDR, &temp_byte);
    *result += temp_byte;
    if (status != RT_EOK)
    {
#ifdef PKG_MT9V034_ASSERT_ON_FAIL
        RT_ASSERT(RT_FALSE);
#else
        LOG_E(LOG_TAG" failed to get register 0x%X", reg_addr);
#endif
    }
    return status;
}

static rt_err_t mt9v034_reg_init(rt_mt9v034_device_t dev)
{
    mt9v034_reg_set(dev, 0xFE, 0xBEEF);

    rt_uint16_t who_am_i;
    mt9v034_reg_get(dev, 0x00, &who_am_i);
#ifdef PKG_MT9V034_ASSERT_ON_FAIL
    RT_ASSERT(who_am_i == 0x1324);
#else
    if (who_am_i != 0x1324)
    {
        LOG_E(LOG_TAG" who am i value error");
    }
#endif

	//Reset control circuit
	mt9v034_reg_set(dev, 0x0C, 1);
	mt9v034_reg_set(dev, 0x0C, 0);

	//Load 
	mt9v034_reg_set(dev, 0x01, 0x0001);   //COL_WINDOW_START_CONTEXTA_REG
    mt9v034_reg_set(dev, 0x02, 0x0004);   //ROW_WINDOW_START_CONTEXTA_REG
    mt9v034_reg_set(dev, 0x03, 0x01E0);   //ROW_WINDOW_SIZE_CONTEXTA_REG
    mt9v034_reg_set(dev, 0x04, 0x02F0);   //COL_WINDOW_SIZE_CONTEXTA_REG
    mt9v034_reg_set(dev, 0x05, 0x005E);   //HORZ_BLANK_CONTEXTA_REG
    mt9v034_reg_set(dev, 0x06, 0x002D);   //VERT_BLANK_CONTEXTA_REG
    mt9v034_reg_set(dev, 0x07, 0x0188);   //CONTROL_MODE_REG
    mt9v034_reg_set(dev, 0x08, 0x01BB);   //COARSE_SHUTTER_WIDTH_1_CONTEXTA
    mt9v034_reg_set(dev, 0x09, 0x01D9);   //COARSE_SHUTTER_WIDTH_2_CONTEXTA
    mt9v034_reg_set(dev, 0x0A, 0x0164);   //SHUTTER_WIDTH_CONTROL_CONTEXTA
    mt9v034_reg_set(dev, 0x0B, 0x0000);   //COARSE_SHUTTER_WIDTH_TOTAL_CONTEXTA
    mt9v034_reg_set(dev, 0x0C, 0x0000);   //RESET_REG
    mt9v034_reg_set(dev, 0x0D, 0x0300);   //READ_MODE_REG
    mt9v034_reg_set(dev, 0x0E, 0x0000);   //READ_MODE2_REG
    mt9v034_reg_set(dev, 0x0F, 0x0000);   //PIXEL_OPERATION_MODE
    mt9v034_reg_set(dev, 0x10, 0x0040);   //RAMP_START_DELAY
    mt9v034_reg_set(dev, 0x11, 0x8042);   //OFFSET_CONTROL
    mt9v034_reg_set(dev, 0x12, 0x0022);   //AMP_RESET_BAR_CONTROL
    mt9v034_reg_set(dev, 0x13, 0x2D2E);   //5T_PIXEL_RESET_CONTROL
    mt9v034_reg_set(dev, 0x14, 0x0E02);   //4T_PIXEL_RESET_CONTROL
    mt9v034_reg_set(dev, 0x15, 0x0E32);   //TX_CONTROL
    mt9v034_reg_set(dev, 0x16, 0x2802);   //5T_PIXEL_SHS_CONTROL
    mt9v034_reg_set(dev, 0x17, 0x3E38);   //4T_PIXEL_SHS_CONTROL
    mt9v034_reg_set(dev, 0x18, 0x3E38);   //5T_PIXEL_SHR_CONTROL
    mt9v034_reg_set(dev, 0x19, 0x2802);   //4T_PIXEL_SHR_CONTROL
    mt9v034_reg_set(dev, 0x1A, 0x0428);   //COMPARATOR_RESET_CONTROL
    mt9v034_reg_set(dev, 0x1B, 0x0000);   //LED_OUT_CONTROL
    mt9v034_reg_set(dev, 0x1C, 0x0302);   //DATA_COMPRESSION
    mt9v034_reg_set(dev, 0x1D, 0x0040);   //ANALOG_TEST_CONTROL
    mt9v034_reg_set(dev, 0x1E, 0x0000);   //SRAM_TEST_DATA_ODD
    mt9v034_reg_set(dev, 0x1F, 0x0000);   //SRAM_TEST_DATA_EVEN
    mt9v034_reg_set(dev, 0x20, 0x03C7);   //BOOST_ROW_EN
    mt9v034_reg_set(dev, 0x21, 0x0020);   //I_VLN_CONTROL
    mt9v034_reg_set(dev, 0x22, 0x0020);   //I_VLN_AMP_CONTROL
    mt9v034_reg_set(dev, 0x23, 0x0010);   //I_VLN_CMP_CONTROL
    mt9v034_reg_set(dev, 0x24, 0x001B);   //I_OFFSET_CONTROL
    mt9v034_reg_set(dev, 0x26, 0x0004);   //I_VLN_VREF_ADC_CONTROL
    mt9v034_reg_set(dev, 0x27, 0x000C);   //I_VLN_STEP_CONTROL
    mt9v034_reg_set(dev, 0x28, 0x0010);   //I_VLN_BUF_CONTROL
    mt9v034_reg_set(dev, 0x29, 0x0010);   //I_MASTER_CONTROL
    mt9v034_reg_set(dev, 0x2A, 0x0020);   //I_VLN_AMP_60MHZ_CONTROL
    mt9v034_reg_set(dev, 0x2B, 0x0004);   //VREF_AMP_CONTROL
    mt9v034_reg_set(dev, 0x2C, 0x0004);   //VREF_ADC_CONTROL
    mt9v034_reg_set(dev, 0x2D, 0x0004);   //VBOOST_CONTROL
    mt9v034_reg_set(dev, 0x2E, 0x0007);   //V_HI_CONTROL
    mt9v034_reg_set(dev, 0x2F, 0x0003);   //V_LO_CONTROL
    mt9v034_reg_set(dev, 0x30, 0x0003);   //V_AMP_CAS_CONTROL
    mt9v034_reg_set(dev, 0x31, 0x0027);   //V1_CONTROL_CONTEXTA
    mt9v034_reg_set(dev, 0x32, 0x001A);   //V2_CONTROL_CONTEXTA
    mt9v034_reg_set(dev, 0x33, 0x0005);   //V3_CONTROL_CONTEXTA
    mt9v034_reg_set(dev, 0x34, 0x0003);   //V4_CONTROL_CONTEXTA
    mt9v034_reg_set(dev, 0x35, 0x0010);   //GLOBAL_GAIN_CONTEXTA_REG
    mt9v034_reg_set(dev, 0x36, 0x8010);   //GLOBAL_GAIN_CONTEXTB_REG
    mt9v034_reg_set(dev, 0x37, 0x0000);   //VOLTAGE_CONTROL
    mt9v034_reg_set(dev, 0x38, 0x0000);   //IDAC_VOLTAGE_MONITOR
    mt9v034_reg_set(dev, 0x39, 0x0027);   //V1_CONTROL_CONTEXTB
    mt9v034_reg_set(dev, 0x3A, 0x0026);   //V2_CONTROL_CONTEXTB
    mt9v034_reg_set(dev, 0x3B, 0x0005);   //V3_CONTROL_CONTEXTB
    mt9v034_reg_set(dev, 0x3C, 0x0003);   //V4_CONTROL_CONTEXTB
    mt9v034_reg_set(dev, 0x40, 0x0080);   //DARK_AVG_THRESHOLDS
    mt9v034_reg_set(dev, 0x46, 0x231D);   //CALIB_CONTROL_REG (AUTO)
    mt9v034_reg_set(dev, 0x47, 0x0080);   //STEP_SIZE_AVG_MODE
    mt9v034_reg_set(dev, 0x48, 0x0020);   //ROW_NOISE_CONTROL
    mt9v034_reg_set(dev, 0x4C, 0x0002);   //NOISE_CONSTANT
    mt9v034_reg_set(dev, 0x60, 0x0000);   //PIXCLK_CONTROL
    mt9v034_reg_set(dev, 0x67, 0x0000);   //TEST_DATA
    mt9v034_reg_set(dev, 0x6C, 0x0000);   //TILE_X0_Y0
    mt9v034_reg_set(dev, 0x70, 0x0000);   //TILE_X1_Y0
    mt9v034_reg_set(dev, 0x71, 0x002A);   //TILE_X2_Y0
    mt9v034_reg_set(dev, 0x72, 0x0000);   //TILE_X3_Y0
    mt9v034_reg_set(dev, 0x7F, 0x0000);   //TILE_X4_Y0
    mt9v034_reg_set(dev, 0x99, 0x0000);   //TILE_X0_Y1
    mt9v034_reg_set(dev, 0x9A, 0x0096);   //TILE_X1_Y1
    mt9v034_reg_set(dev, 0x9B, 0x012C);   //TILE_X2_Y1
    mt9v034_reg_set(dev, 0x9C, 0x01C2);   //TILE_X3_Y1
    mt9v034_reg_set(dev, 0x9D, 0x0258);   //TILE_X4_Y1
    mt9v034_reg_set(dev, 0x9E, 0x02F0);   //TILE_X0_Y2
    mt9v034_reg_set(dev, 0x9F, 0x0000);   //TILE_X1_Y2
    mt9v034_reg_set(dev, 0xA0, 0x0060);   //TILE_X2_Y2
    mt9v034_reg_set(dev, 0xA1, 0x00C0);   //TILE_X3_Y2
    mt9v034_reg_set(dev, 0xA2, 0x0120);   //TILE_X4_Y2
    mt9v034_reg_set(dev, 0xA3, 0x0180);   //TILE_X0_Y3
    mt9v034_reg_set(dev, 0xA4, 0x01E0);   //TILE_X1_Y3
    mt9v034_reg_set(dev, 0xA5, 0x003A);   //TILE_X2_Y3
    mt9v034_reg_set(dev, 0xA6, 0x0002);   //TILE_X3_Y3
    mt9v034_reg_set(dev, 0xA8, 0x0000);   //TILE_X4_Y3
    mt9v034_reg_set(dev, 0xA9, 0x0002);   //TILE_X0_Y4
    mt9v034_reg_set(dev, 0xAA, 0x0002);   //TILE_X1_Y4
    mt9v034_reg_set(dev, 0xAB, 0x0040);   //TILE_X2_Y4
    mt9v034_reg_set(dev, 0xAC, 0x0001);   //TILE_X3_Y4
    mt9v034_reg_set(dev, 0xAD, 0x01E0);   //TILE_X4_Y4
    mt9v034_reg_set(dev, 0xAE, 0x0014);   //X0_SLASH5
    mt9v034_reg_set(dev, 0xAF, 0x0000);   //X1_SLASH5
    mt9v034_reg_set(dev, 0xB0, 0xABE0);   //X2_SLASH5
    mt9v034_reg_set(dev, 0xB1, 0x0002);   //X3_SLASH5
    mt9v034_reg_set(dev, 0xB2, 0x0010);   //X4_SLASH5
    mt9v034_reg_set(dev, 0xB3, 0x0010);   //X5_SLASH5
    mt9v034_reg_set(dev, 0xB4, 0x0000);   //Y0_SLASH5
    mt9v034_reg_set(dev, 0xB5, 0x0000);   //Y1_SLASH5
    mt9v034_reg_set(dev, 0xB6, 0x0000);   //Y2_SLASH5
    mt9v034_reg_set(dev, 0xB7, 0x0000);   //Y3_SLASH5
    mt9v034_reg_set(dev, 0xBF, 0x0016);   //Y4_SLASH5
    mt9v034_reg_set(dev, 0xC0, 0x000A);   //Y5_SLASH5
    mt9v034_reg_set(dev, 0xC2, 0x18D0);   //DESIRED_BIN
    mt9v034_reg_set(dev, 0xC3, 0x007F);   //EXP_SKIP_FRM_H
    mt9v034_reg_set(dev, 0xC4, 0x007F);   //EXP_LPF
    mt9v034_reg_set(dev, 0xC5, 0x007F);   //GAIN_SKIP_FRM
    mt9v034_reg_set(dev, 0xC6, 0x0000);   //GAIN_LPF_H
    mt9v034_reg_set(dev, 0xC7, 0x4416);   //MAX_GAIN
    mt9v034_reg_set(dev, 0xC8, 0x4421);   //MIN_COARSE_EXPOSURE
    mt9v034_reg_set(dev, 0xC9, 0x0001);   //MAX_COARSE_EXPOSURE
    mt9v034_reg_set(dev, 0xCA, 0x0004);   //BIN_DIFF_THRESHOLD
    mt9v034_reg_set(dev, 0xCB, 0x01E0);   //AUTO_BLOCK_CONTROL
    mt9v034_reg_set(dev, 0xCC, 0x02F0);   //PIXEL_COUNT
    mt9v034_reg_set(dev, 0xCD, 0x005E);   //LVDS_MASTER_CONTROL
    mt9v034_reg_set(dev, 0xCE, 0x002D);   //LVDS_SHFT_CLK_CONTROL
    mt9v034_reg_set(dev, 0xCF, 0x01DE);   //LVDS_DATA_CONTROL
    mt9v034_reg_set(dev, 0xD0, 0x01DF);   //LVDS_DATA_STREAM_LATENCY
    mt9v034_reg_set(dev, 0xD1, 0x0164);   //LVDS_INTERNAL_SYNC
    mt9v034_reg_set(dev, 0xD2, 0x0001);   //LVDS_USE_10BIT_PIXELS
    mt9v034_reg_set(dev, 0xD3, 0x0000);   //STEREO_ERROR_CONTROL
    mt9v034_reg_set(dev, 0xD4, 0x0000);   //INTERLACE_FIELD_VBLANK
    mt9v034_reg_set(dev, 0xD5, 0x0104);   //IMAGE_CAPTURE_NUM
    mt9v034_reg_set(dev, 0xD6, 0x0000);   //ANALOG_CONTROLS
    mt9v034_reg_set(dev, 0xD7, 0x0000);   //AB_PULSE_WIDTH_REG
    mt9v034_reg_set(dev, 0xD8, 0x0000);   //TX_PULLUP_PULSE_WIDTH_REG
    mt9v034_reg_set(dev, 0xD9, 0x0000);   //RST_PULLUP_PULSE_WIDTH_REG
    mt9v034_reg_set(dev, 0xF0, 0x0000);   //NTSC_FV_CONTROL
    mt9v034_reg_set(dev, 0xFE, 0xBEEF);   //NTSC_HBLANK

	uint16_t data = 0;
	uint16_t width = PKG_MT9V034_FRAME_WIDTH;
	uint16_t height = PKG_MT9V034_FRAME_HEIGHT;
	uint16_t exposure;
    if ((height * 4) <= 480)
    {
        height *= 4;
        data |= 2;
		exposure = (PKG_MT9V034_FPS > 193) ? 193 : ((PKG_MT9V034_FPS < 1) ? 1 : PKG_MT9V034_FPS);        
        if (exposure > 132)
        {
            exposure = (uint16_t)(-2.0 * exposure + 504);
        }
        else
        {
            exposure = (uint16_t)(132.0 / exposure * 240);
        }
    }
    else if ((height * 2) <= 480)
    {
        height *= 2;
        data |= 1;
		exposure = (PKG_MT9V034_FPS > 112) ? 112 : ((PKG_MT9V034_FPS < 1) ? 1 : PKG_MT9V034_FPS);        
        if (exposure > 66)
        {
            exposure = (uint16_t)(-5.2 * exposure + 822);
        }
        else
        {
            exposure = (uint16_t)(66.0 / exposure * 480);
        }
    }
    else 
    {
		exposure = (PKG_MT9V034_FPS > 60) ? 60 : ((PKG_MT9V034_FPS < 1) ? 1 : PKG_MT9V034_FPS);
        exposure = (uint16_t)(60.0 / exposure * 480);
        
    }
    if ((width * 4) <= 752)
    {
        width *= 4;
        data |= 2 << 2;
    }
    else if ((width * 2) <= 752)
    {
        width *= 2;
        data |= 1 << 2;
    }
	//Row and column flip
    data |= 3 << 4;
	mt9v034_reg_set(dev, 0x0D, data);
    mt9v034_reg_set(dev, 0x04, width);
	mt9v034_reg_set(dev, 0x03, height);
	mt9v034_reg_set(dev, 0x01, (752 - width) / 2 + 1);
	mt9v034_reg_set(dev, 0x02, (480 - height) / 2 + 4);
	mt9v034_reg_set(dev, 0x0B, exposure);

	//Enable anti eclipse
	mt9v034_reg_set(dev, 0xC2, 0x18D0);
	mt9v034_reg_set(dev, 0xAB, 0x0040);
	mt9v034_reg_set(dev, 0xB0, width * height);
	mt9v034_reg_set(dev, 0x1C, 0x0303);

	//Reg fix
	mt9v034_reg_set(dev, 0x13, 0x2D2E);
	mt9v034_reg_set(dev, 0x20, 0x03C7);
	mt9v034_reg_set(dev, 0x24, 0x0010);
	mt9v034_reg_set(dev, 0x2B, 0x0003);
	mt9v034_reg_set(dev, 0x2F, 0x0003);

	//Coarse shutter image width control
	mt9v034_reg_set(dev, 0x0A, 0x0164);
	mt9v034_reg_set(dev, 0x32, 0x001A);
	mt9v034_reg_set(dev, 0x0F, 0x0103);
	mt9v034_reg_set(dev, 0xA5, 60);
	mt9v034_reg_set(dev, 0x35, 0x8010);

	//AEC AGC
	uint8_t af_value = 0;
#ifdef PKG_MT9V034_ENABLE_AEC
    af_value |= 1;
#endif
#ifdef PKG_MT9V034_ENABLE_AGC
    af_value |= 1 << 1;
#endif
	mt9v034_reg_set(dev, 0xAF, (af_value << 8) | af_value | 0);

    uint16_t coarse_shutter_width_total;
#ifdef PKG_MT9V034_HDR_100DB
    mt9v034_reg_set(dev, 0x0A, 0x0164);
    mt9v034_reg_get(dev, 0x0B, &coarse_shutter_width_total);
    if (coarse_shutter_width_total > 0x03E8)
        mt9v034_reg_set(dev, 0x0B, 0x03E8);
    mt9v034_reg_set(dev, 0x0F, 0x0103);
    mt9v034_reg_set(dev, 0x35, 0x8010);
#elif defined(PKG_MT9V034_HDR_80DB)
    mt9v034_reg_set(dev, 0x0A, 0x0164);
    mt9v034_reg_get(dev, 0x0B, &coarse_shutter_width_total);
    if (coarse_shutter_width_total > 0x03E8)
        mt9v034_reg_set(dev, 0x0B, 0x03E8);
    mt9v034_reg_set(dev, 0x0F, 0x0103);
    mt9v034_reg_set(dev, 0x35, 0x8010);
#else
    mt9v034_reg_set(dev, 0x08, 0x01BB);
    mt9v034_reg_set(dev, 0x09, 0x01D9);
    mt9v034_reg_set(dev, 0x0A, 0x0164);
    mt9v034_reg_get(dev, 0x0B, &coarse_shutter_width_total);
    if (coarse_shutter_width_total > 0x01E0)
        mt9v034_reg_set(dev, 0x0B, 0x01E0);
    mt9v034_reg_set(dev, 0x0F, 0x0100);
    mt9v034_reg_set(dev, 0x35, 0x0010);
#endif

	mt9v034_reg_set(dev, 0x70, 0x0303); //ROW_NOISE_CONTROL

	//Reset
	mt9v034_reg_set(dev, 0x0C, 0x03);

	//Lock all register
	mt9v034_reg_set(dev, 0xFE, 0xDEAD);

    return RT_EOK;
}


static rt_err_t mt9v034_init(rt_device_t _dev)
{
    rt_mt9v034_device_t dev = (rt_mt9v034_device_t)_dev;
    mt9v034_reg_init(dev);
    csi_config_t csi_config =
    {
        .width = dev->width,
        .height = dev->height,
        .polarityFlags = kCSI_HsyncActiveHigh | kCSI_VsyncActiveHigh | kCSI_DataLatchOnRisingEdge,
        .bytesPerPixel = 1,
        .linePitch_Bytes = dev->width,
        .workMode = kCSI_GatedClockMode,
        .dataBus = kCSI_DataBus8Bit,
        .useExtVsync = RT_TRUE,
    };
    CSI_Init(dev->csi_base, &csi_config);
    CSI_TransferCreateHandle(dev->csi_base, &dev->csi_handle, mt9v034_csi_xfer_callback, dev);
    for (rt_uint8_t i = 0; i < CSI_DRIVER_QUEUE_SIZE; ++i)
    {
        CSI_TransferSubmitEmptyBuffer(dev->csi_base, &dev->csi_handle, (rt_uint32_t)(frame_buffer[i]));
    }
    CSI_TransferStart(dev->csi_base, &dev->csi_handle);
    return RT_EOK;
}

static rt_err_t mt9v034_close(rt_device_t _dev)
{
    if(_dev->ref_count == 0)
    {
        rt_mt9v034_device_t dev = (rt_mt9v034_device_t)_dev;
        CSI_TransferStop(dev->csi_base, &dev->csi_handle);
    }
}

static rt_size_t mt9v034_read(rt_device_t _dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    rt_mt9v034_device_t dev = (rt_mt9v034_device_t)_dev;
    if (CSI_TransferGetFullBuffer(dev->csi_base, &dev->csi_handle, buffer) == kStatus_CSI_NoFullBuffer)
    {
        return 0;
    }
    else
    {
	    return dev->width * dev->height;
    }
}

rt_size_t mt9v034_write(rt_device_t _dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    rt_mt9v034_device_t dev = (rt_mt9v034_device_t)_dev;
    if (CSI_TransferSubmitEmptyBuffer(dev->csi_base, &dev->csi_handle, (rt_uint32_t)(*(const rt_uint32_t *)buffer)) == kStatus_CSI_QueueFull)
    {
        return 0;
    }
    else
    {
        return 4;
    }
}

#ifdef RT_USING_DEVICE_OPS
static struct rt_device_ops mt9v034_dev_ops =
{
    .init = mt9v034_init,
    .open = RT_NULL,
    .close = mt9v034_close,
    .read = mt9v034_read,
    .write = mt9v034_write,
    .control = RT_NULL,
#endif

static int mt9v034_hw_init(void)
{
    rt_mt9v034_device_t dev = rt_malloc(sizeof(struct rt_mt9v034_device));
    if (dev)
    {
        rt_memset(dev, 0, sizeof(struct rt_mt9v034_device));
        dev->i2c_bus = rt_device_find(PKG_MT9V034_I2C_BUS_NAME);
#ifdef PKG_MT9V034_ASSERT_ON_FAIL
        RT_ASSERT(dev->i2c_bus);
#else
        if (dev->i2c_bus == RT_NULL)
        {
            LOG_E(LOG_TAG" i2c bus %s not found", PKG_MT9V034_I2C_BUS_NAME);
            return -RT_ERROR;
        }
#endif
        dev->csi_base = PKG_MT9V034_CSI_BASE;
        dev->width = PKG_MT9V034_FRAME_WIDTH;
        dev->height = PKG_MT9V034_FRAME_HEIGHT;
#ifdef RT_USING_DEVICE_OPS
		dev->parent->ops = &mt9v034_dev_ops;
#else
		dev->parent.type = RT_Device_Class_Miscellaneous;
		dev->parent.init = mt9v034_init;
		dev->parent.open = RT_NULL;
		dev->parent.close = mt9v034_close;
		dev->parent.read = mt9v034_read;
		dev->parent.write = mt9v034_write;
		dev->parent.control = RT_NULL;
#endif
        rt_device_register(dev, "MT9V034", RT_DEVICE_FLAG_DEACTIVATE);
        return RT_EOK;
    }
    else
    {
        return -RT_ENOMEM;
    }
}
INIT_COMPONENT_EXPORT(mt9v034_hw_init);

#endif