/*
 * s1d13522_yotaphone.c -- Platform device for Yotaphone e-ink display
 *
 * Copyright (C) 2016, Yota Devices
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file COPYING in the main directory of this archive for
 * more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/of_gpio.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>

#include <video/s1d13522fb.h>
#include <linux/epd_sflash.h>

#define FIRMWARE_FILE_COMMAND  "epson/cmds1d13522.fw"

#define S1D13522_MAX_HRDY_TIMEOUT	1000

#define PANEL_WIDTH	540
#define PANEL_HEIGHT	960

static struct platform_device *s1d13522_pdev;
static struct platform_device *sflash_pdev;
static struct spi_device *spidev;

enum {
	/* HRDY_GPIO should be first! */
	HRDY_GPIO = 0,
	PWR_GPIO,
	HV_PWR_GPIO,
	PMIC_GPIO,
	RESET_GPIO,
	HDC_GPIO,

	GPIO_MAX
};

static char *gpio_names[GPIO_MAX] = {
	"qcom,hrdy",
	"qcom,power-enable",
	"qcom,hv-power-enable",
	"qcom,pmic-enable",
	"qcom,resetL-gpio",
	"qcom,hdc"
};
static int gpios[GPIO_MAX];


static int yphone_epd_wait_event(struct s1d13522fb_par *par, int timeoutms)
{
	int ret;

	if (gpio_get_value_cansleep(gpios[HRDY_GPIO]))
		return 0;

	enable_irq(gpio_to_irq(gpios[HRDY_GPIO]));

	ret = !wait_event_timeout(par->waitq,
				  gpio_get_value_cansleep(gpios[HRDY_GPIO]),
				  msecs_to_jiffies(timeoutms));

	disable_irq(gpio_to_irq(gpios[HRDY_GPIO]));

	return ret;
}

static u16 yphone_epd_read_data(struct s1d13522fb_par *par)
{
	u16 ret[2];

	gpio_set_value_cansleep(gpios[HDC_GPIO], 1);

	/* should read one dummy word */
	if (spi_read(spidev, &ret, sizeof(u16) * 2) < 0)
		dev_err(&spidev->dev, "%s: failed spi read\n", __func__);

	return cpu_to_be16(ret[1]);
}

static void yphone_epd_write_data(struct s1d13522fb_par *par, u16 data)
{
	data = cpu_to_be16(data);
	if (spi_write(spidev, &data, sizeof(u16)) < 0)
		dev_err(&spidev->dev, "%s: failed spi write\n", __func__);
}

#define PMIC_READ	0xD1
#define PMIC_WRITE	0xD0

static u16 yphone_epd_read_pmic(struct s1d13522fb_par *par, u16 reg)
{
	par->write_reg(par, S1D13522_REG021E_I2C_WRITE_DATA, PMIC_WRITE);
	par->write_reg(par, S1D13522_REG021A_I2C_COMMAND, 0x31);

	par->write_reg(par, S1D13522_REG021E_I2C_WRITE_DATA, reg);
	par->write_reg(par, S1D13522_REG021A_I2C_COMMAND, 0x01);

	par->write_reg(par, S1D13522_REG021E_I2C_WRITE_DATA, PMIC_READ);
	par->write_reg(par, S1D13522_REG021A_I2C_COMMAND, 0x31);

	par->write_reg(par, S1D13522_REG021A_I2C_COMMAND, 0x17);
	return par->read_reg(par, S1D13522_REG021C_I2C_READ_DATA);
}

static void yphone_epd_write_pmic(struct s1d13522fb_par *par, u16 reg,
					u16 data)
{
	par->write_reg(par, S1D13522_REG021E_I2C_WRITE_DATA, PMIC_WRITE);
	par->write_reg(par, S1D13522_REG021A_I2C_COMMAND, 0x31);

	par->write_reg(par, S1D13522_REG021E_I2C_WRITE_DATA, reg);
	par->write_reg(par, S1D13522_REG021A_I2C_COMMAND, 0x01);

	par->write_reg(par, S1D13522_REG021E_I2C_WRITE_DATA, data);
	par->write_reg(par, S1D13522_REG021A_I2C_COMMAND, 0x11);
}


static void
yphone_epd_write_burst(struct s1d13522fb_par *par, int size, u16 *data)
{
	struct spi_message message;
	struct spi_transfer transfer;
	dma_addr_t dma_map;

	if (size & 0x1) {
		dev_err(&spidev->dev,
		"%s: invalid transfer size [%d] should be power of 2\n",
		__func__, size);

		return;
	}

	/* check that controller in data mode */
	gpio_set_value_cansleep(gpios[HDC_GPIO], 1);

	spi_message_init(&message);

	memset(&transfer, 0 , sizeof(transfer));
	transfer.len = size;
	transfer.bits_per_word = 16;
	transfer.tx_buf = data;

	dma_map = dma_map_single(&spidev->dev, data, size, DMA_TO_DEVICE);
	if (dma_mapping_error(&spidev->dev, dma_map)) {
		dev_err(&spidev->dev, "%s: failed to map dma buf\n", __func__);
		goto no_dma;
	}

	transfer.tx_dma = dma_map;
	spi_message_add_tail(&transfer, &message);

	message.is_dma_mapped = 1;

	if (spi_sync(spidev, &message))
		dev_err(&spidev->dev, "%s: spi transfer failed with size %d\n",
			__func__, size);

	dma_unmap_single(&spidev->dev, transfer.tx_dma, size, DMA_TO_DEVICE);
	return;

no_dma:
	while (size) {
		yphone_epd_write_data(par, *data);
		size -= 2;
		data++;
	}
	return;
}

static int yphone_epd_send_cmdargs(struct s1d13522fb_par *par, u16 cmd,
				   int argc, u16 *argv)
{
	int i;

	if (yphone_epd_wait_event(par, S1D13522_MAX_HRDY_TIMEOUT)) {
		dev_err(&spidev->dev, "%s: timeout waiting HRDY\n", __func__);
		return -ETIMEDOUT;
	}

	gpio_set_value_cansleep(gpios[HDC_GPIO], 0);
	yphone_epd_write_data(par, cmd);
	gpio_set_value_cansleep(gpios[HDC_GPIO], 1);

	for (i = 0; i < argc; i++)
		yphone_epd_write_data(par, argv[i]);

	return 0;
}

static int yphone_epd_send_command(struct s1d13522fb_par *par, u16 cmd)
{
	return yphone_epd_send_cmdargs(par, cmd, 0, NULL);
}

/*
 * Warning!
 * Check that HRDY IRQ is enabled ONLY while
 * waiting ready response from epson s1d13522.
 * This errata should prevent multiple false IRQ's
 * On Yotaphone 2 platform.
 */
static irqreturn_t yphone_epd_handle_irq(int irq, void *dev_id)
{
	struct s1d13522fb_par *par = dev_id;

	wake_up(&par->waitq);
	return IRQ_HANDLED;
}

static int yphone_epd_setup_irq(struct fb_info *info)
{
	int ret;
	struct s1d13522fb_par *par = info->par;

	ret = of_get_named_gpio(spidev->dev.of_node, gpio_names[HRDY_GPIO], 0);
	if (ret < 0) {
		dev_err(&spidev->dev, "%s HRDY gpio (%s) not found, err = %d\n",
					__func__, gpio_names[HRDY_GPIO], ret);
		return ret;
	}
	gpios[HRDY_GPIO] = ret;

	ret = gpio_request(gpios[HRDY_GPIO], gpio_names[HRDY_GPIO]);
	if (ret)
		return ret;

	ret = request_irq(gpio_to_irq(gpios[HRDY_GPIO]), yphone_epd_handle_irq,
			  IRQF_TRIGGER_RISING,
			  "s1d13522_hrdy", par);
	if (ret) {
		dev_err(&spidev->dev, "%s: request_irq failed: %d\n",
			__func__, ret);
		gpio_free(gpios[HRDY_GPIO]);
	}

	disable_irq(gpio_to_irq(gpios[HRDY_GPIO]));

	return ret;
}

static void yphone_epd_set_power(unsigned char state)
{
	gpio_set_value_cansleep(gpios[RESET_GPIO], 0);

	/* power pins inverted */
	gpio_set_value_cansleep(gpios[PMIC_GPIO], !state);
	gpio_set_value_cansleep(gpios[HV_PWR_GPIO], !state);
	gpio_set_value_cansleep(gpios[PWR_GPIO], !state);

	msleep(20);
	gpio_set_value_cansleep(gpios[RESET_GPIO], 1);
}

static int yphone_epd_request_gpio(struct s1d13522fb_par *par)
{
	int ret;
	int i;

	/* start from second gpio, irq pin was configured while setup_irq */
	for (i = 1; i < GPIO_MAX; i++) {
		ret = of_get_named_gpio(spidev->dev.of_node, gpio_names[i], 0);
		if (ret < 0) {
			dev_err(&spidev->dev, "can't find gpio (%s) err = %d\n",
				gpio_names[i], ret);
			goto exit_err;
		}
		gpios[i] = ret;

		ret = gpio_request(ret, gpio_names[i]);
		if (ret) {
			dev_err(&spidev->dev,
				"failed to request gpio \"%s\"-[%d] err = %d\n",
				gpio_names[i], gpios[i], ret);
			goto exit_err;
		}

		gpio_direction_output(gpios[i], 0);
	}

	return 0;

exit_err:
	while (--i > 1)
		gpio_free(gpios[i]);
	return ret;
}

static int yphone_epd_init_board(struct s1d13522fb_par *par)
{
	int ret;
	u16 reg_val;
	u16 cmd_param[4];
	const struct firmware *firmware;
	u16 *fw_data;

	ret = yphone_epd_request_gpio(par);
	if (ret)
		return ret;

	yphone_epd_set_power(1);

	par->write_reg(par, S1D13522_REG000A_SYSTEM_STATUS, (0x01<<12));

	cmd_param[0] = 0x0003;
	cmd_param[1] = 0x5949;
	cmd_param[2] = 0x0040;
	ret = yphone_epd_send_cmdargs(par, S1D13522_CMD_INIT_PLL, 3, cmd_param);
	if (ret)
		return ret;

	/* Load firmware */
	ret = request_firmware(&firmware, FIRMWARE_FILE_COMMAND, &spidev->dev);
	if (ret) {
		dev_err(&spidev->dev, "%s: firmware not available\n", __func__);
		return ret;
	}

	fw_data = kmalloc(firmware->size, GFP_KERNEL);
	if (!fw_data) {
		dev_err(&spidev->dev, "%s: failed to allocate firmware memory \n",
			__func__);
		release_firmware(firmware);
		return -ENOMEM;
	}

	memcpy(fw_data, firmware->data, firmware->size);

	yphone_epd_send_command(par, S1D13522_CMD_INIT_CMD_SET);
	yphone_epd_write_burst(par, firmware->size, fw_data);
	release_firmware(firmware);
	kfree(fw_data);

	yphone_epd_send_command(par, S1D13522_CMD_INIT_SYS_RUN);

	/* setup waveform */
	cmd_param[0] = 0;
	yphone_epd_send_cmdargs(par, S1D13522_CMD_INIT_WAVE_DEV, 1 , cmd_param);

	cmd_param[0] = SF_WF_ADDR & 0x0000ffff;
	cmd_param[1] = (SF_WF_ADDR >> 16) & 0x000003ff;
	yphone_epd_send_cmdargs(par, S1D13522_CMD_RD_WFM_INFO, 2 , cmd_param);

	yphone_epd_send_command(par, S1D13522_CMD_UPD_GDRV_CLR);

	yphone_epd_send_command(par, S1D13522_CMD_WAIT_DSPE_TRG);

	/* Configure GPIO for temperature sensors */
	reg_val = par->read_reg(par, S1D13522_REG0250_GPIO_CONFIG);
	/* mark 3 and 4 pins as output */
	reg_val |= BIT(4) | BIT(3);
	par->write_reg(par, S1D13522_REG0250_GPIO_CONFIG, reg_val);

	par->board->fb_par = par;

	return 0;
}

static void yphone_epd_cleanup(struct s1d13522fb_par *par)
{
	int i;

	free_irq(gpio_to_irq(gpios[HRDY_GPIO]), par);

	if (gpios[RESET_GPIO] && gpios[PMIC_GPIO] &&
	    gpios[HV_PWR_GPIO] && gpios[PWR_GPIO])
		yphone_epd_set_power(0);

	for (i = 0; i < GPIO_MAX; i++)
		gpio_free(gpios[i]);

	par->board->fb_par = NULL;
}

static int yphone_epd_get_cfg(const char *config_name)
{
	if (!strcmp(config_name, "width"))
		return PANEL_WIDTH;

	if (!strcmp(config_name, "height"))
		return PANEL_HEIGHT;

	return -EINVAL;
}

static int yphone_epd_get_temp(struct s1d13522fb_par *par, int *result)
{
	const unsigned char count = 3;
	int temp[count];
	u16 gpio_ctl;
	int i, j = 0;

	*result = 0;

	mutex_lock(&par->io_lock);

	gpio_ctl = par->read_reg(par, S1D13522_REG0252_GPIO_CONTROL);

	for (i = 0 ; i < count ; i++) {
		/* clear pins 4 and 3 */
		gpio_ctl &= ~(BIT(3) | BIT(4));

		switch (i) {
		default:
		case 0:
			gpio_ctl |= BIT(3);
			break;
		case 1:
			gpio_ctl |= BIT(4);
			break;
		case 2:
			gpio_ctl |= BIT(3) | BIT(4);
			break;
		}

		par->write_reg(par, S1D13522_REG0252_GPIO_CONTROL,
						gpio_ctl);

		msleep(20);
		yphone_epd_write_pmic(par, 0x0d, 0x80);
		msleep(20);

		if (yphone_epd_read_pmic(par, 0x0d) & 0x80) {
			mutex_unlock(&par->io_lock);
			dev_err(par->info->dev, "%s: i2c read temp busy\n",
						__func__);
			return -EBUSY;
		}
		msleep(20);

		temp[i] = yphone_epd_read_pmic(par, 0x00);
		temp[i] = (temp[i] & 0x80) ? (temp[i] - 255 - 1) : temp[i];

		if (temp[i] != -10) {
			*result += temp[i];
			j++;
		}
	}

	mutex_unlock(&par->io_lock);

	if (j == 0) {
		dev_err(par->info->dev, "%s: no suitable sensors data found\n",
			__func__);
		return -EINVAL;
	}

	*result = *result / j;
	dev_dbg(par->info->dev, "read temp t1: %d, t2: %d, t3: %d avrg: %d\n",
		temp[0], temp[1], temp[2], *result);

	return 0;
}

static struct s1d13522fb_board s1d13522_board = {
	.owner			= THIS_MODULE,
	.get_panel_config	= yphone_epd_get_cfg,
	.init			= yphone_epd_init_board,
	.setup_irq		= yphone_epd_setup_irq,
	.wait_for_rdy		= yphone_epd_wait_event,
	.cleanup		= yphone_epd_cleanup,
	.write_data		= yphone_epd_write_data,
	.read_data		= yphone_epd_read_data,
	.send_cmdargs		= yphone_epd_send_cmdargs,
	.send_command		= yphone_epd_send_command,
	.burst_write		= yphone_epd_write_burst,
	.get_temp		= yphone_epd_get_temp,
};

static int __devinit yphone_epd_probe(struct spi_device *dev)
{
	int ret;

	/* double init is deprecated */
	if (spidev)
		return -EBUSY;

	dev->bits_per_word = 16;
	ret = spi_setup(dev);
	if (ret < 0) {
		dev_err(&dev->dev, "%s: couldn't setup spi\n", __func__);
		return ret;
	}

	/* request our platform independent driver */
	request_module("s1d13522fb");

	s1d13522_pdev = platform_device_alloc("s1d13522fb", -1);
	if (!s1d13522_pdev)
		return -ENOMEM;

	s1d13522_pdev->dev.platform_data = &s1d13522_board;
	spidev = dev;

	ret = platform_device_add(s1d13522_pdev);

	if (ret) {
		platform_device_put(s1d13522_pdev);
		spidev = NULL;
		return ret;
	}

	/* register EPD serial flash driver */
	request_module("epd_sflash");
	sflash_pdev = platform_device_alloc("epd_sflash", -1);
	if (!sflash_pdev) {
		dev_err(&dev->dev, "%s: alloc of epd_sflash device failed\n",
			__func__);
	} else {
		sflash_pdev->dev.platform_data = &s1d13522_board;
		ret = platform_device_add(sflash_pdev);

		if (ret) {
			dev_err(&dev->dev,
				"%s: failed to register EPD sflash driver\n",
				__func__);
			platform_device_put(sflash_pdev);
		}
	}

	printk(KERN_INFO "Yotaphone e-ink display board, "\
		"register platform device\n");

	return 0;
}

static int __devexit yphone_epd_remove(struct spi_device *spidev)
{
	spidev = NULL;
	s1d13522_pdev->dev.platform_data = NULL;
	sflash_pdev->dev.platform_data = NULL;

	platform_device_del(s1d13522_pdev);
	platform_device_put(s1d13522_pdev);

	platform_device_del(sflash_pdev);
	platform_device_put(sflash_pdev);

	return 0;
}

#ifdef CONFIG_PM
static int yphone_epd_suspend(struct spi_device *dev, pm_message_t msg)
{
	return platform_pm_suspend(&s1d13522_pdev->dev);
}

static int yphone_epd_resume(struct spi_device *dev)
{
	return platform_pm_resume(&s1d13522_pdev->dev);
}
#else
#define yphone_epd_suspend NULL
#define yphone_epd_resume NULL
#endif

static struct of_device_id epd_match_table[] = {
	{ .compatible = "qcom,s1d13522",},
	{ },
};

static struct spi_driver yphone_epd_driver = {
	.driver = {
		.name = "s1d13522_spi_driver",
		.owner = THIS_MODULE,
		.of_match_table = epd_match_table,
	},
	.probe = yphone_epd_probe,
	.remove = yphone_epd_remove,
	.suspend = yphone_epd_suspend,
	.resume = yphone_epd_resume,
};

static int __init yphone_epd_init(void)
{
	return spi_register_driver(&yphone_epd_driver);
}

static void __exit yphone_epd_exit(void)
{
	spi_unregister_driver(&yphone_epd_driver);
}

module_init(yphone_epd_init);
module_exit(yphone_epd_exit);

MODULE_DESCRIPTION("board driver for Yotaphone e-ink display");
MODULE_AUTHOR("Ivanov Dmitry");
MODULE_LICENSE("GPL");
