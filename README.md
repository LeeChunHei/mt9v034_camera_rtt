# MT9V034 camera Driver

[Chinese](README_ZH.md) | English

## 1. Introduction

This software package is a driver for MT9V034 camera. This driver provide access through IO device model.

### 1.1 License

MT9V034 camera driver package is licensed with MIT license.

### 1.2 Dependency

- RT-Thread 4.0+
- I2C device driver or soft I2C device driver
- NXP CSI peripheral driver

## 2. How to use MT9V034 camera Driver

To use MT9V034 camera driver package, you need to select it in the package manager of RT-Thread. The specific path is as follows:

```
RT-Thread online packages
    peripheral libraries and drivers --->
        [*] MT9V034 camera driver package --->
            (200)   Frame Per Second
            [ ]     Enable auto explosure control
            [ ]     Enable auto gain control
                    HDR (No HDR) --->
            (752)   Width
            (480)   Height
            (i2c10) Camera configuration I2C/SCCB bus name
            [ ]     Assert the program when faced configuration failure
```

The detailed description of the package options is as follows:

| Option | Description |
|-|-|
| Frame Per Second | The camera's frame per second, max 200 |
| Enable auto explosure control | The camera will automatically adjust the explosure time if this is selected |
| Enable auto gain control | The camera will automatically adjust the gain of the images if this is selected |
| HDR | High-Dynamic Range, options are No HDR, 80dB and 100dB |
| Width | Width of the camera image, max 752 and it needs to be divisble by 8 |
| Height | Height of the camera image, max 480 and it needs to be divisble by 8 |
| Camera configuration I2C/SCCB bus name | The I2C/SCCB bus name which connected to the camera's SDA and SCL pins |
| Assert the program when faced configuration failure | |

After selecting the options you need, use RT-Thread's package manager to automatically update, or use the `pkgs --update` command to update the package to the BSP.

## 3. Use MT9V034 camera Driver

After opening the MT9V034 camera driver package and selecting the corresponding function option, it will be added to the BSP project for compilation when the BSP is compiled.
Burn the program to the target development board, and the user can use the following method to interact with the camera:

| Function | Parameter | Action |
|---|---|---|
| `rt_size_t rt_device_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)` | `buffer`: pointer of a rt_uint8_t pointer | Request a camera frame and modifly `buffer` to point to the frame buffer, return the size of the frame buffer, return 0 if not frame buffer available yet |
| `rt_size_t rt_device_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)` | `buffer`: pointer of a rt_uint8_t pointer | return the previous requested frame buffer or add a new empty frame buffer into the queue, keep in mind that the buffer written need to be 64 byte aligned and non-cacheable |

## 4. Example
```
#include <rtdevice.h>

int main(void)
{
    /* Find the camera device */
    rt_device_t cam = rt_device_find("MT9V034");
    rt_device_open(cam, 0);

    rt_uint8_t* buffer;
    rt_size_t buffer_size;

    while (1)
    {
        /* request a frame buffer */
        buffer_size = rt_device_read(cam, 0, &buffer, 0);
        if (buffer_size != 0)
        {
            /* Loop through the camera image */
            for (rt_uint16_t i = 0; i < buffer_size; ++i)
            {
                /* Access one pixel in the frame */
                buffer[i];
            }
            /* return the frame buffer back to queue */
            rt_device_write(cam, 0, &buffer, 0);
        }
        else
        {
            rt_thread_mdelay(1);
        }
    }
}
```