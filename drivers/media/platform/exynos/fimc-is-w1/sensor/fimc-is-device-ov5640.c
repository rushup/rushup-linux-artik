#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/version.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <linux/videodev2.h>
#include <linux/videodev2_exynos_camera.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <mach/regs-gpio.h>
#include <mach/regs-clock.h>
#include <plat/clock.h>
#include <plat/gpio-cfg.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <mach/exynos-fimc-is-sensor.h>

#include "../fimc-is-core.h"
#include "../fimc-is-device-sensor.h"
#include "../fimc-is-resourcemgr.h"
#include "fimc-is-device-ov5640.h"

static struct i2c_client *g_ov5640_i2c_client;


/**
 * ov5640_reg_read - Read a value from a register in an ov5640 sensor device
 * @client: i2c driver client structure
 * @reg: register address / offset
 * @val: stores the value that gets read
 *
 * Read a value from a register in an ov5640 sensor device.
 * The value is returned in 'val'.
 * Returns zero if successful, or non-zero otherwise.
 */
static int ov5640_reg_read(struct i2c_client *client,  u16 reg, u8 *val)
{
    int ret;
    u8 data[2] = {0};
    struct i2c_msg msg = {
            .addr	= client->addr,
            .flags	= 0,
            .len	= 2,
            .buf	= data,
    };

    data[0] = (u8)(reg >> 8);
    data[1] = (u8)(reg & 0xff);

    ret = i2c_transfer(client->adapter, &msg, 1);
    if (ret < 0)
        goto err;

    msg.flags = I2C_M_RD|I2C_CLIENT_SCCB;
    msg.len = 1;
    ret = i2c_transfer(client->adapter, &msg, 1);
    if (ret < 0)
        goto err;

    *val = data[0];
    return 0;

    err:
    dev_err(&client->dev, "Failed reading register 0x%02x!\n", reg);
    return ret;
}

static int ov5640_i2c_read_twobyte(struct i2c_client *client,
                                   u16 subaddr, u16 *data)
{
    int err;
    unsigned char buf[2];
    struct i2c_msg msg[2];
    buf[0]=0;
    buf[1]=0;

    cpu_to_be16s(&subaddr);

    msg[0].addr = client->addr;
    msg[0].flags = I2C_CLIENT_SCCB;
    msg[0].len = 2;
    msg[0].buf = (u8 *)&subaddr;

    msg[1].addr = client->addr;
    msg[1].flags = I2C_M_RD|I2C_CLIENT_SCCB;
    msg[1].len = 2;
    msg[1].buf = buf;

    err = i2c_transfer(client->adapter, msg, 2);
    if (unlikely(err != 2)) {
        dev_err(&client->dev,
                "%s: register read fail (%d)\n", __func__,err);
        return -EIO;
    }

    *data = ((buf[0] << 8) | buf[1]);

    return 0;
}


/**
 * Write a value to a register in ov5640 sensor device.
 * @client: i2c driver client structure.
 * @reg: Address of the register to read value from.
 * @val: Value to be written to a specific register.
 * Returns zero if successful, or non-zero otherwise.
 */
static int ov5640_reg_write(struct i2c_client *client, u16 reg, u8 val)
{


    return fimc_is_sensor_write(client,reg,val);
}

/**
 * Initialize a list of ov5640 registers.
 * The list of registers is terminated by the pair of values
 * @client: i2c driver client structure.
 * @reglist[]: List of address of the registers to write data.
 * Returns zero if successful, or non-zero otherwise.
 */
static int ov5640_reg_writes(struct i2c_client *client,
                             const struct ov5640_reg reglist[],
                             int size)
{
    int err = 0, i;

    for (i = 0; i < size; i++) {
        if(reglist[i].reg == 0xFFFF){
            if(reglist[i].val != 0xFF){
                mdelay(reglist[i].val);
            }else{
                break;
            }
        }else{
        err = ov5640_reg_write(client, reglist[i].reg,
                         reglist[i].val);
        }
        if (err)
            return err;
    }
    return 0;
}

static int ov5640_reg_set(struct i2c_client *client, u16 reg, u8 val)
{
    int ret;
    u8 tmpval = 0;

    ret = ov5640_reg_read(client, reg, &tmpval);
    if (ret)
        return ret;

    return ov5640_reg_write(client, reg, tmpval | val);
}

static int ov5640_reg_clr(struct i2c_client *client, u16 reg, u8 val)
{
    int ret;
    u8 tmpval = 0;

    ret = ov5640_reg_read(client, reg, &tmpval);
    if (ret)
        return ret;

    return ov5640_reg_write(client, reg, tmpval & ~val);
}

static unsigned long ov5640_get_pclk(struct v4l2_subdev *sd)
{
    struct ov5640_s *ov5640 = to_ov5640(sd);
    unsigned long xvclk, vco, mipi_pclk;

    xvclk = ov5640->xvclk;

    vco = (xvclk / ov5640->clk_cfg.sc_pll_prediv) *
          ov5640->clk_cfg.sc_pll_mult;

    mipi_pclk = vco /
                ov5640->clk_cfg.sysclk_div /
                ov5640->clk_cfg.mipi_div;

    return mipi_pclk;
}

static void ov5640_set_brightness(struct v4l2_subdev *sd, int value)
{

}

static void ov5640_set_contrast(struct v4l2_subdev *sd, int value)
{

}

static void ov5640_set_sharpness(struct v4l2_subdev *sd, int value)
{

}

static void ov5640_set_saturation(struct v4l2_subdev *sd, int value)
{

}

static void ov5640_set_iso(struct v4l2_subdev *sd, int value)
{

}

static void ov5640_set_scenemode(struct v4l2_subdev *sd, int value)
{

}

static void ov5640_set_white_balance(struct v4l2_subdev *sd, int value)
{

}


static void ov5640_set_effect(struct v4l2_subdev *sd, int value)
{
    cam_info("%s\n", __func__);

}



static void ov5640_set_window(struct v4l2_subdev *sd)
{

}

static void ov5640_set_af(struct v4l2_subdev *sd)
{
    struct i2c_client *client = g_ov5640_i2c_client;

    cam_info("%s\n", __func__);

}

static void ov5640_cancel_af(struct v4l2_subdev *sd)
{
    struct i2c_client *client = g_ov5640_i2c_client;

    cam_info("%s\n", __func__);

}



static int ov5640_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{

    struct ov5640_s *ov5640  = to_ov5640(sd);
    int value = ctrl->value;

    cam_info("%s\n", __func__);

    switch (ctrl->id) {
        case V4L2_CID_SCENEMODE:

            break;
        case V4L2_CID_CAM_BRIGHTNESS:

            break;
        case V4L2_CID_WHITE_BALANCE_PRESET:

            break;
        case V4L2_CID_CAMERA_EFFECT:

            break;
        case V4L2_CID_CAM_CONTRAST:

            break;
        case V4L2_CID_CAM_SATURATION:

            break;
        case V4L2_CID_CAM_SHARPNESS:

            break;
        case V4L2_CID_CAM_ISO:

            break;
        case V4L2_CID_CAPTURE:

            break;
        case V4L2_CID_CAM_OBJECT_POSITION_X:

            break;
        case V4L2_CID_CAM_OBJECT_POSITION_Y:

            break;
        case V4L2_CID_CAM_SINGLE_AUTO_FOCUS:
        case V4L2_CID_CAM_SET_AUTO_FOCUS:

            break;
        case V4L2_CID_FOCUS_MODE:
        case V4L2_CID_JPEG_QUALITY:
        case V4L2_CID_CAM_FLASH_MODE:
            break;
        default:
            cam_err("[%s] Unidentified ID (%X)\n", __func__, ctrl->id);
            break;
    }

    return 0;
}

static int ov5640_get_af(struct v4l2_subdev *sd)
{

    return 0;
}

static int ov5640_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
    struct ov5640_s *ov5640 = to_ov5640(sd);
    int err = 0;

    cam_info("%s\n", __func__);

    switch (ctrl->id) {
        case V4L2_CID_CAMERA_WHITE_BALANCE:
            break;
        case V4L2_CID_CAMERA_CONTRAST:
            break;
        case V4L2_CID_CAMERA_SATURATION:
            break;
        case V4L2_CID_CAMERA_SHARPNESS:
            break;
        case V4L2_CID_CAM_AUTO_FOCUS_RESULT:
            break;
        case V4L2_CID_CAM_DATE_INFO_YEAR:
        case V4L2_CID_CAM_DATE_INFO_MONTH:
        case V4L2_CID_CAM_DATE_INFO_DATE:
        case V4L2_CID_CAM_SENSOR_VER:
        default:
            break;
    }

    return err;
}

static void ov5640_init_setting(struct v4l2_subdev *sd)
{
    struct ov5640_s *ov5640 = to_ov5640(sd);
    int value;

    cam_info("%s\n", __func__);


}




static int ov5640_s_power(struct v4l2_subdev *sd, int on)
{
    int ret= 0;

    return ret;
}




static int ov5640_init(struct v4l2_subdev *sd, u32 val)
{
    struct i2c_client *client = g_ov5640_i2c_client;
    struct ov5640_s *ov5640 = to_ov5640(sd);
    cam_info("%s\n", __func__);

    int ret = 0;
    u8 revision = 0;
    u8 msb;
    u8 lsb;

    ret = ov5640_s_power(sd, 1);
    if (ret < 0) {
        dev_err(&client->dev, "OV5640 power up failed\n");
        return ret;
    }

    ret = ov5640_reg_read(client, 0x302A, &revision);
    if (ret) {
        dev_err(&client->dev, "Failure to detect OV5640 chip\n");
        goto out;
    }

    revision &= 0xF;

    dev_info(&client->dev, "Detected a OV5640 chip, revision %x\n",
             revision);

    /* SW Reset */
    ret = ov5640_reg_set(client, 0x3008, 0x80);
    if (ret)
        goto out;

    msleep(2);

    ret = ov5640_reg_clr(client, 0x3008, 0x80);
    if (ret)
        goto out;

    /* SW Powerdown */
    ret = ov5640_reg_set(client, 0x3008, 0x40);
    if (ret)
        goto out;

    /* Check chip id */
    ret = ov5640_reg_read(client, 0x300A, &msb);
    if (!ret)
        ret = ov5640_reg_read(client, 0x300B, &lsb);
    if (!ret) {
        ov5640->id = (msb << 8) | lsb;
        if (ov5640->id == 0x5640)
            dev_info(&client->dev, "Sensor ID: %04X\n", ov5640->id);
        else
            dev_err(&client->dev, "Sensor detection failed (%04X, %d)\n",
                    ov5640->id, ret);
    }

   ret = ov5640_reg_writes(client, configscript_common1,
                            ARRAY_SIZE(configscript_common1));
    if (ret)
        goto out;

    ret = ov5640_reg_writes(client, configscript_common2,
                        ARRAY_SIZE(configscript_common2));
    if (ret)
      goto out;

    //ret = ov5640_reg_writes(client, configscript_common_XXX,
    //                        ARRAY_SIZE(configscript_common_XXX));
    //if (ret)
    //    goto out;

    cam_info("%s\n",__func__);
    /* Init controls */
    /*
    ret = v4l2_ctrl_handler_init(&ov5640->ctrl_handler, 1);
    if (ret)
        goto out;

    ov5640->pixel_rate = v4l2_ctrl_new_std(
            &ov5640->ctrl_handler, NULL,
            V4L2_CID_PIXEL_RATE,
            0, 0, 1, 0);

    sd->ctrl_handler = &ov5640->ctrl_handler;
    */
    out:
    ov5640_s_power(sd, 0);
    return ret;
}

static int ov5640_s_ext_ctrls(struct v4l2_subdev *sd,
                              struct v4l2_ext_controls *ctrls)
{
    cam_info("%s\n", __func__);

    return 0;
}




static int ov5640_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
    cam_info("%s\n", __func__);

    return 0;
}

static int ov5640_querymenu(struct v4l2_subdev *sd, struct v4l2_querymenu *qm)
{
    cam_info("%s\n", __func__);

    return 0;
}

static int ov5640_g_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *fmt)
{
    cam_info("%s\n", __func__);

    return 0;
}

static int ov5640_s_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *fmt)
{
    struct ov5640_s *ov5640 = to_ov5640(sd);
    ov5640->format.width = fmt->width;
    ov5640->format.height = fmt->height;

    //ov5640->format.code = fmt->code;

    return 0;
}


static int ov5640_config_timing(struct v4l2_subdev *sd)
{
    struct i2c_client *client = g_ov5640_i2c_client;;
    struct ov5640_s *ov5640 = to_ov5640(sd);
    int ret, i;
    u8 val;

    i = ov5640_find_framesize(ov5640->format.width, ov5640->format.height);
    ret = ov5640_reg_write(client,
                           0x3800,
                           (timing_cfg[i].x_addr_start & 0xFF00) >> 8);
    if (ret)
        return ret;

    ret = ov5640_reg_write(client,
                           0x3801,
                           timing_cfg[i].x_addr_start & 0xFF);
    if (ret)
        return ret;

    ret = ov5640_reg_write(client,
                           0x3802,
                           (timing_cfg[i].y_addr_start & 0xFF00) >> 8);
    if (ret)
        return ret;

    ret = ov5640_reg_write(client,
                           0x3803,
                           timing_cfg[i].y_addr_start & 0xFF);
    if (ret)
        return ret;

    ret = ov5640_reg_write(client,
                           0x3804,
                           (timing_cfg[i].x_addr_end & 0xFF00) >> 8);
    if (ret)
        return ret;

    ret = ov5640_reg_write(client,
                           0x3805,
                           timing_cfg[i].x_addr_end & 0xFF);
    if (ret)
        return ret;

    ret = ov5640_reg_write(client,
                           0x3806,
                           (timing_cfg[i].y_addr_end & 0xFF00) >> 8);
    if (ret)
        return ret;

    ret = ov5640_reg_write(client,
                           0x3807,
                           timing_cfg[i].y_addr_end & 0xFF);
    if (ret)
        return ret;

    ret = ov5640_reg_write(client,
                           0x3808,
                           (timing_cfg[i].h_output_size & 0xFF00) >> 8);
    if (ret)
        return ret;

    ret = ov5640_reg_write(client,
                           0x3809,
                           timing_cfg[i].h_output_size & 0xFF);
    if (ret)
        return ret;

    ret = ov5640_reg_write(client,
                           0x380A,
                           (timing_cfg[i].v_output_size & 0xFF00) >> 8);
    if (ret)
        return ret;

    ret = ov5640_reg_write(client,
                           0x380B,
                           timing_cfg[i].v_output_size & 0xFF);
    if (ret)
        return ret;

    ret = ov5640_reg_write(client,
                           0x380C,
                           (timing_cfg[i].h_total_size & 0xFF00) >> 8);
    if (ret)
        return ret;

    ret = ov5640_reg_write(client,
                           0x380D,
                           timing_cfg[i].h_total_size & 0xFF);
    if (ret)
        return ret;

    ret = ov5640_reg_write(client,
                           0x380E,
                           (timing_cfg[i].v_total_size & 0xFF00) >> 8);
    if (ret)
        return ret;

    ret = ov5640_reg_write(client,
                           0x380F,
                           timing_cfg[i].v_total_size & 0xFF);
    if (ret)
        return ret;

    ret = ov5640_reg_write(client,
                           0x3810,
                           (timing_cfg[i].isp_h_offset & 0xFF00) >> 8);
    if (ret)
        return ret;

    ret = ov5640_reg_write(client,
                           0x3811,
                           timing_cfg[i].isp_h_offset & 0xFF);
    if (ret)
        return ret;

    ret = ov5640_reg_write(client,
                           0x3812,
                           (timing_cfg[i].isp_v_offset & 0xFF00) >> 8);
    if (ret)
        return ret;

    ret = ov5640_reg_write(client,
                           0x3813,
                           timing_cfg[i].isp_v_offset & 0xFF);
    if (ret)
        return ret;

    ret = ov5640_reg_write(client,
                           0x3814,
                           ((timing_cfg[i].h_odd_ss_inc & 0xF) << 4) |
                           (timing_cfg[i].h_even_ss_inc & 0xF));
    if (ret)
        return ret;

    ret = ov5640_reg_write(client,
                           0x3815,
                           ((timing_cfg[i].v_odd_ss_inc & 0xF) << 4) |
                           (timing_cfg[i].v_even_ss_inc & 0xF));

    ov5640_reg_read(client, OV5640_TIMING_REG21, &val);
    if (timing_cfg[i].out_mode_sel & OV5640_BINNING_MASK)
        val |= OV5640_BINNING_MASK;
    else
        val &= ~(OV5640_BINNING_MASK);

    if (0)
        val |= OV5640_HFLIP_MASK;
    else
        val &= ~(OV5640_HFLIP_MASK);

    ret = ov5640_reg_write(client, OV5640_TIMING_REG21, val);
    if (ret)
        return ret;


    ret = ov5640_reg_write(client,
                           0x3108, timing_cfg[i].sclk_dividers & 0xFF);
    if (ret)
        return ret;

    ret = ov5640_reg_write(client, 0x3035, timing_cfg[i].sys_mipi_clk & 0xFF);
    if (ret)
        return ret;
/*
    ov5640_reg_writes(client, ov5640_fps_30,
                      ARRAY_SIZE(ov5640_fps_30));
*/
    return ret;
}


static int ov5640_s_stream(struct v4l2_subdev *sd, int enable)
{
    struct ov5640_s *ov5640 = to_ov5640(sd);
    struct i2c_client *client = g_ov5640_i2c_client;
    int ret = 0;

    if (enable) {
        ov5640_s_power(&ov5640->sd, 1);
        u8 fmtreg = 0, fmtmuxreg = 0;
        int i;

        switch ((u32)ov5640->format.code) {
            case V4L2_MBUS_FMT_UYVY8_2X8:
                fmtreg = 0x32;
                fmtmuxreg = 0;
                break;
            case V4L2_MBUS_FMT_YUYV8_2X8:
                fmtreg = 0x30;
                fmtmuxreg = 0;
                break;
            case V4L2_PIX_FMT_RGB565:
                fmtreg = 0x60;
                fmtmuxreg = 1;
            default:
                /* This shouldn't happen */
                ret = -EINVAL;
                return ret;
        }

        ret = ov5640_reg_write(client, 0x4300, fmtreg);
        if (ret)
            return ret;

        ret = ov5640_reg_write(client, 0x501F, fmtmuxreg);
        if (ret)
            return ret;

        ret = ov5640_config_timing(sd);
        if (ret)
            return ret;
       // ov5640_reg_write(client, 0x4202, 0x00);


        i = ov5640_find_framesize(ov5640->format.width, ov5640->format.height);
        cam_info("%s: %dx%d (%d)\n", __func__,ov5640->format.width, ov5640->format.height,i);
        if ((i == OV5640_SIZE_QVGA) ||
            (i == OV5640_SIZE_VGA) ||
            (i == OV5640_SIZE_720P)) {
            //ret = ov5640_reg_write(client, 0x3108,
            //                       (i == OV5640_SIZE_720P) ? 0x1 : 0);
            if (ret)
                return ret;
            //ret = ov5640_reg_set(client, 0x5001, 0x20);
        } else {
            //ret = ov5640_reg_clr(client, 0x5001, 0x20);
            if (ret)
                return ret;
            //ret = ov5640_reg_write(client, 0x3108, 0x2);
        }

        ret = ov5640_reg_clr(client, 0x3008, 0x40);
        if (ret)
            goto out;

    } else {

        u8 tmpreg = 0;

        ret = ov5640_reg_read(client, 0x3008, &tmpreg);
        if (ret)
            goto out;

        ret = ov5640_reg_write(client, 0x3008, tmpreg | 0x40);
        if (ret)
            goto out;

        //ov5640_reg_write(client, 0x4202, 0x0f);

        ov5640_s_power(&ov5640->sd, 0);
    }

    out:
    return ret;
}

static int ov5640_g_parm(struct v4l2_subdev *sd,
                         struct v4l2_streamparm *param)
{
    cam_info("%s\n", __func__);

    return 0;
}

static int ov5640_s_parm(struct v4l2_subdev *sd,
                         struct v4l2_streamparm *param)
{
    int ret = 0;
    cam_info("%s\n", __func__);
    struct fimc_is_module_enum *module;
    struct v4l2_captureparm *cp;
    struct v4l2_fract *tpf;
    u64 framerate;

    BUG_ON(!sd);
    BUG_ON(!param);

    cp = &param->parm.capture;
    tpf = &cp->timeperframe;

    if (!tpf->numerator) {
        err("numerator is 0");
        ret = -EINVAL;
        goto p_err;
    }

    framerate = tpf->denominator;

    module = (struct fimc_is_module_enum *)v4l2_get_subdevdata(sd);
    if (!module) {
        err("module is NULL");
        ret = -EINVAL;
        goto p_err;
    }

    ret = CALL_MOPS(module, s_duration, sd, framerate);
    if (ret) {
        err("s_duration is fail(%d)", ret);
        goto p_err;
    }
    p_err:

    return ret;
}

static int ov5640_enum_framesizes(struct v4l2_subdev *subdev,
                                  struct v4l2_subdev_fh *fh,
                                  struct v4l2_subdev_frame_size_enum *fse)
{
    cam_info("%s %d %d\n", __func__,fse->index,fse->code);
    if ((fse->index >= OV5640_SIZE_LAST) ||
        (fse->code != V4L2_MBUS_FMT_UYVY8_2X8 &&
         fse->code != V4L2_MBUS_FMT_YUYV8_2X8))
        return -EINVAL;
    cam_info("%s\n", __func__);
    fse->min_width = ov5640_frmsizes[fse->index].width;
    fse->max_width = fse->min_width;
    fse->min_height = ov5640_frmsizes[fse->index].height;
    fse->max_height = fse->min_height;

    return 0;
}

/*
 * Clock configuration
 * Configure expected MCLK from host and return EINVAL if not supported clock
 * frequency is expected
 *  freq : in Hz
 *  flag : not supported for now
 */
static int ov5640_s_crystal_freq(struct v4l2_subdev *sd,
                                 u32  freq, u32 flags)
{
    cam_info("%s\n", __func__);

    return 0;
}


static int ov5640_enum_frameintervals(struct v4l2_subdev *sd,
                                      struct v4l2_frmivalenum *interval)
{
    int size;

    if (interval->index >= 1)
        return -EINVAL;

    interval->type = V4L2_FRMIVAL_TYPE_DISCRETE;

    size = ov5640_find_framesize(interval->width, interval->height);

    switch (size) {
        case OV5640_SIZE_5MP:
            interval->discrete.numerator	= 2;
            interval->discrete.denominator	= 15;
            break;
        case OV5640_SIZE_720P:
            interval->discrete.numerator	= 1;
            interval->discrete.denominator	= 0;
            break;
        case OV5640_SIZE_VGA:
        case OV5640_SIZE_QVGA:
        default:
            interval->discrete.numerator	= 1;
            interval->discrete.denominator	= 24;
            break;
    }
    return 0;
}


/* get format by flite video device command */
static struct v4l2_mbus_framefmt *
__ov5640_get_pad_format(struct ov5640_s *ov5640, struct v4l2_subdev_fh *fh,
                        unsigned int pad, enum v4l2_subdev_format_whence which)
{
    switch (which) {
        case V4L2_SUBDEV_FORMAT_TRY:
            return v4l2_subdev_get_try_format(fh, pad);
        case V4L2_SUBDEV_FORMAT_ACTIVE:
            return &ov5640->format;
        default:
            return NULL;
    }
}

static int ov5640_get_fmt(struct v4l2_subdev *sd,
                        struct v4l2_subdev_fh *fh,
                        struct v4l2_subdev_format *format)
{
    struct ov5640_s *ov5640 = to_ov5640(sd);
    cam_info("%s\n", __func__);

    format->format = *__ov5640_get_pad_format(ov5640, fh, format->pad,
                                              format->which);

    return 0;
}


/* set format by flite video device command */

static int ov5640_set_fmt(struct v4l2_subdev *sd,
                        struct v4l2_subdev_fh *fh,
                        struct v4l2_subdev_format *format)
{
    struct ov5640_s *ov5640 = to_ov5640(sd);
    struct v4l2_mbus_framefmt *__format;

    __format = __ov5640_get_pad_format(ov5640, fh, format->pad,
                                       format->which);

    ov5640->pixel_rate->cur.val64 = ov5640_get_pclk(sd) / 16;

    return 0;
}


static int ov5640_enum_mbus_code(struct v4l2_subdev *sd,
                                 struct v4l2_subdev_fh *fh,
                                 struct v4l2_subdev_mbus_code_enum *code)
{
    cam_info("%s\n", __func__);
    if (code->index >= 2)
        return -EINVAL;

    switch (code->index) {
        case 0:
            code->code = V4L2_MBUS_FMT_UYVY8_2X8;
            break;
        case 1:
            code->code = V4L2_MBUS_FMT_YUYV8_2X8;
            break;
    }

    return 0;
}


static int ov5640_g_skip_frames(struct v4l2_subdev *sd, u32 *frames)
{
    /* Quantity of initial bad frames to skip. Revisit. */
    *frames = 3;

    return 0;
}

static const struct v4l2_subdev_video_ops ov5640_video_ops = {
        .g_mbus_fmt         = ov5640_g_fmt,
        .s_mbus_fmt         = ov5640_s_fmt,
        .g_parm             = ov5640_g_parm,
        .s_parm             = ov5640_s_parm,
        .s_stream           = ov5640_s_stream,
        .enum_framesizes    = ov5640_enum_framesizes,
        .s_crystal_freq     = ov5640_s_crystal_freq,
        .enum_frameintervals = ov5640_enum_frameintervals,
};

static const struct v4l2_subdev_core_ops ov5640_core_ops = {
        .g_ctrl         = ov5640_g_ctrl,
        .s_ctrl         = ov5640_s_ctrl,
        .init           = ov5640_init,  /* initializing API */
        .s_ext_ctrls    = ov5640_s_ext_ctrls,
        .s_power        = ov5640_s_power,
        .queryctrl      = ov5640_queryctrl,
        .querymenu      = ov5640_querymenu,
};


static struct v4l2_subdev_pad_ops ov5640_pad_ops = {
        .enum_mbus_code = ov5640_enum_mbus_code,
        .enum_frame_size = ov5640_enum_framesizes,
        .get_fmt        = ov5640_get_fmt,
        .set_fmt        = ov5640_set_fmt,
};

static struct v4l2_subdev_sensor_ops ov5640_sensor_ops = {
        .g_skip_frames	= ov5640_g_skip_frames,
};

static const struct v4l2_subdev_ops ov5640_ops = {
        .core   = &ov5640_core_ops,
        .pad    = &ov5640_pad_ops,
        .video  = &ov5640_video_ops,
        //.sensor	= &ov5640_sensor_ops,
};



/*
 * @ brief
 * frame duration time
 * @ unit
 * nano second
 * @ remarks
 */
int sensor_ov5640_s_duration(struct v4l2_subdev *subdev, u64 framerate)
{
    int ret = 0;
    u32 frametime;
    struct i2c_client *client = g_ov5640_i2c_client;
    struct ov5640_s *ov5640 = to_ov5640(subdev);
    cam_info("%s\n", __func__);



p_err:
    return ret;
}

int sensor_ov5640_g_min_duration(struct v4l2_subdev *subdev)
{
    int ret = 0;

    cam_info("%s\n", __func__);

    return ret;
}

int sensor_ov5640_g_max_duration(struct v4l2_subdev *subdev)
{
    int ret = 0;

    cam_info("%s\n", __func__);

    return ret;
}

int sensor_ov5640_s_exposure(struct v4l2_subdev *subdev, u64 exposure)
{
    int ret = 0;

    return ret;
}

int sensor_ov5640_g_min_exposure(struct v4l2_subdev *subdev)
{
    int ret = 0;

    cam_info("%s\n", __func__);

    return ret;
}

int sensor_ov5640_g_max_exposure(struct v4l2_subdev *subdev)
{
    int ret = 0;

    cam_info("%s\n", __func__);

    return ret;
}

int sensor_ov5640_s_again(struct v4l2_subdev *subdev, u64 sensitivity)
{
    int ret = 0;

    cam_info("%s\n", __func__);

    return ret;
}

int sensor_ov5640_g_min_again(struct v4l2_subdev *subdev)
{
    int ret = 0;

    cam_info("%s\n", __func__);

    return ret;
}

int sensor_ov5640_g_max_again(struct v4l2_subdev *subdev)
{
    int ret = 0;

    cam_info("%s\n", __func__);

    return ret;
}

int sensor_ov5640_s_dgain(struct v4l2_subdev *subdev)
{
    int ret = 0;

    cam_info("%s\n", __func__);

    return ret;
}

int sensor_ov5640_g_min_dgain(struct v4l2_subdev *subdev)
{
    int ret = 0;

    cam_info("%s\n", __func__);

    return ret;
}

int sensor_ov5640_g_max_dgain(struct v4l2_subdev *subdev)
{
    int ret = 0;

    cam_info("%s\n", __func__);

    return ret;
}



struct fimc_is_sensor_ops module_ov5640_ops = {
/*    	.stream_on		= sensor_ov5640_stream_on,
	    .stream_off		= sensor_ov5640_stream_off,
	    */
        .s_duration		= sensor_ov5640_s_duration,
        .g_min_duration	= sensor_ov5640_g_min_duration,
        .g_max_duration	= sensor_ov5640_g_max_duration,
        .s_exposure		= sensor_ov5640_s_exposure,
        .g_min_exposure	= sensor_ov5640_g_min_exposure,
        .g_max_exposure	= sensor_ov5640_g_max_exposure,
        .s_again		= sensor_ov5640_s_again,
        .g_min_again	= sensor_ov5640_g_min_again,
        .g_max_again	= sensor_ov5640_g_max_again,
        .s_dgain		= sensor_ov5640_s_dgain,
        .g_min_dgain	= sensor_ov5640_g_min_dgain,
        .g_max_dgain	= sensor_ov5640_g_max_dgain
};

static int ov5640_link_setup(struct media_entity *entity,
                             const struct media_pad *local,
                             const struct media_pad *remote, u32 flags)
{
    cam_info("%s\n", __func__);

    return 0;
}


static const struct media_entity_operations ov5640_media_ops = {
        .link_setup = ov5640_link_setup,
};

static int ov5640_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
    cam_info("%s\n", __func__);
    struct v4l2_mbus_framefmt *format;

    format = v4l2_subdev_get_try_format(fh, 0);
    format->code = V4L2_MBUS_FMT_UYVY8_2X8; //V4L2_MBUS_FMT_YUYV8_2X8;/*V4L2_MBUS_FMT_UYVY8_2X8;*/
    format->width = ov5640_frmsizes[OV5640_SIZE_5MP].width;
    format->height = ov5640_frmsizes[OV5640_SIZE_5MP].height;
    format->field = V4L2_FIELD_NONE;
    format->colorspace = V4L2_COLORSPACE_JPEG;

    return 0;
}

static int ov5640_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
    cam_info("%s\n", __func__);

    return 0;
}

static int ov5640_registered(struct v4l2_subdev *sd)
{
    cam_info("%s\n", __func__);

    return 0;
}

static void ov5640_unregistered(struct v4l2_subdev *sd)
{
    cam_info("%s\n", __func__);
}


static const struct v4l2_subdev_internal_ops ov5640_subdev_internal_ops = {
        .open           = ov5640_open,
        .close          = ov5640_close,
        .registered     = ov5640_registered,
        .unregistered   = ov5640_unregistered,
};


int sensor_ov5640_probe(struct i2c_client *client,
                        const struct i2c_device_id *id)
{
    int ret = 0;
    struct fimc_is_core *core;
    struct fimc_is_module_enum *module;
    struct fimc_is_device_sensor *device;
    struct sensor_open_extended *ext;
    struct v4l2_subdev *sd;

    struct ov5640_s *ov5640;

    cam_info("%s\n", __func__);

    BUG_ON(!fimc_is_dev);

    core = (struct fimc_is_core *)dev_get_drvdata(fimc_is_dev);
    if (!core) {
        err("core device is not yet probed");
        return -EPROBE_DEFER;
    }

    device = &core->sensor[SENSOR_OV5640_INSTANCE];

    ov5640 = kzalloc(sizeof(struct ov5640_s), GFP_KERNEL);
    if (ov5640 == NULL) {
        dev_err(&client->dev, "OV5640 probe error : kzalloc\n");
        return -ENOMEM;
    }

    /*
    ret = ov5640_get_resources(ov5640, &client->dev);
    if (ret) {
        kfree(ov5640);
        return ret;
    }
    */
    ov5640->xvclk= 24000000;
    ov5640->format.code = V4L2_MBUS_FMT_UYVY8_2X8;/*V4L2_MBUS_FMT_YUYV8_2X8;*/
    ov5640->format.width = ov5640_frmsizes[OV5640_SIZE_5MP].width;
    ov5640->format.height = ov5640_frmsizes[OV5640_SIZE_5MP].height;
    ov5640->format.field = V4L2_FIELD_NONE;
    ov5640->format.colorspace = V4L2_COLORSPACE_JPEG;

    ov5640->clk_cfg.sc_pll_prediv = 2;
    ov5640->clk_cfg.sc_pll_rdiv = 1;
    ov5640->clk_cfg.sc_pll_mult = 0x40 ;
    ov5640->clk_cfg.sysclk_div = 1;
    ov5640->clk_cfg.mipi_div = 1;

    sd = &ov5640->sd;
    g_ov5640_i2c_client = client;
    g_ov5640_i2c_client->flags |= I2C_CLIENT_SCCB;


    /* OV5640 */
    module = &device->module_enum[atomic_read
            (&core->resourcemgr.rsccount_module)];
    atomic_inc(&core->resourcemgr.rsccount_module);
    module->id = SENSOR_OV5640_NAME;
    module->subdev = sd;
    module->device = SENSOR_OV5640_INSTANCE;
    module->ops = &module_ov5640_ops;
    module->client = client;

    module->active_width = OV5640_SENSOR_SIZE_X;
    module->active_height = OV5640_SENSOR_SIZE_Y;
    module->pixel_width = module->active_width + BLANKING_EXTRA_WIDTH;
    module->pixel_height = BLANKING_MIN_HEIGHT;

    module->max_framerate = 120;

    module->position = SENSOR_POSITION_FRONT;

    module->setfile_name = "setfile_ov5640.bin";
    module->cfgs = ARRAY_SIZE(config_ov5640);
    module->cfg = config_ov5640;
    module->private_data = kzalloc(sizeof(struct fimc_is_module_ov5640),
                                   GFP_KERNEL);
    if (!module->private_data) {
        err("private_data is NULL");
        ret = -ENOMEM;
        goto p_err;
    }

    ext = &module->ext;
    ext->mipi_lane_num = 2;
    ext->I2CSclk = I2C_L1;
    ext->sensor_con.product_name = SENSOR_NAME_OV5640;
    ext->sensor_con.peri_type = SE_I2C;
    ext->sensor_con.peri_setting.i2c.channel = SENSOR_CONTROL_I2C1;
    ext->sensor_con.peri_setting.i2c.slave_address = 0x3c;
    ext->sensor_con.peri_setting.i2c.speed = 400000;

    ext->from_con.product_name = FROMDRV_NAME_NOTHING;
    ext->companion_con.product_name = COMPANION_NAME_NOTHING;

    v4l2_i2c_subdev_init(sd, client, &ov5640_ops);

    /* Registering subdev */
    ret = media_entity_init(&sd->entity, 1, &ov5640->pad, 0);
    if (ret) {
        dev_err(&client->dev, "OV5640 probe error : media entity\n");
        return ret;
    }
    sd->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
    sd->entity.ops = &ov5640_media_ops;
    sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
    sd->internal_ops = &ov5640_subdev_internal_ops;


    v4l2_set_subdevdata(sd, module);
    v4l2_set_subdev_hostdata(sd, device);


    snprintf(sd->name, V4L2_SUBDEV_NAME_SIZE,
             "sensor-subdev.%d", module->id);

    p_err:
    info("%s(%d)\n", __func__, ret);
    return ret;
}

static int sensor_ov5640_remove(struct i2c_client *client)
{
    int ret = 0;

    cam_info("%s\n", __func__);

    return ret;
}

#ifdef CONFIG_OF
static const struct of_device_id exynos_fimc_is_sensor_ov5640_match[] = {
	{
		.compatible = "samsung,exynos5-fimc-is-sensor-ov5640",
	},
	{},
};
#endif

static const struct i2c_device_id sensor_ov5640_idt[] = {
        { SENSOR_NAME, 0 },
};

static struct i2c_driver sensor_ov5640_driver = {
        .driver = {
                .name	= SENSOR_NAME,
                .owner	= THIS_MODULE,
#ifdef CONFIG_OF
                .of_match_table = exynos_fimc_is_sensor_ov5640_match
#endif
        },
        .probe	= sensor_ov5640_probe,
        .remove	= sensor_ov5640_remove,
        .id_table = sensor_ov5640_idt
};

static int __init sensor_ov5640_load(void)
{
    cam_info("%s\n", __func__);
    return i2c_add_driver(&sensor_ov5640_driver);
}

static void __exit sensor_ov5640_unload(void)
{
    cam_info("%s\n", __func__);
    i2c_del_driver(&sensor_ov5640_driver);
}

module_init(sensor_ov5640_load);
module_exit(sensor_ov5640_unload);

MODULE_AUTHOR("Danilo Sia");
MODULE_DESCRIPTION("Sensor LI-OV5640 driver");
MODULE_LICENSE("GPL v2");
