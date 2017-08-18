/* drivers/misc/epd_sflash.c
 *
 * (c) 2016 Yota Devices
 *
 * Driver for accessing & interpreting to panel's serial flash
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file COPYING in the main directory of this archive for
 * more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/printk.h>
#include <linux/delay.h>
#include <linux/platform_device.h>

#include <video/s1d13522fb.h>
#include <linux/epd_sflash.h>

#define SPI_WRITE_DUMMY		0
#define SPI_WRITE		(BIT(8))

static int epd_sflash_wait_for_ready(struct s1d13522fb_par *par)
{
	int cnt = 50;
	u16 val;

	do {
		val = par->read_reg(par, S1D13522_REG0206_SPI_STATUS);
		msleep(10);
	} while ((val & BIT(3)) && --cnt);

	if (cnt <= 0) {
		pr_warning("%s: serial flash timeout\n", __func__);
		return -EIO;
	}

	return 0;
}

static int epd_sflash_read(struct s1d13522fb_par *par, unsigned addr,
			   u8 *buf, size_t len)
{
	u16 spi_ctrl_reg;
	u16 read;

	if ((len == 0) || !buf)
		return -EINVAL;

	if (par->power_acquire)
		par->power_acquire(par->status);
	mutex_lock(&par->io_lock);

	/* switch to host access mode */
	spi_ctrl_reg = par->read_reg(par, S1D13522_REG0204_SPI_CONTROL);
	par->write_reg(par, S1D13522_REG0204_SPI_CONTROL,
					spi_ctrl_reg & ~BIT(7));

	/* enable cs 0 */
	par->write_reg(par, S1D13522_REG0208_SPI_CS_SELECT, BIT(0));

	par->write_reg(par, S1D13522_REG0202_SPI_WRITE, SPI_WRITE | 0x3);
	epd_sflash_wait_for_ready(par);

	par->write_reg(par, S1D13522_REG0202_SPI_WRITE,
			SPI_WRITE | ((addr >> 16) & 0xFF));
	epd_sflash_wait_for_ready(par);

	par->write_reg(par, S1D13522_REG0202_SPI_WRITE,
			SPI_WRITE | ((addr >> 8) & 0xFF));
	epd_sflash_wait_for_ready(par);

	par->write_reg(par, S1D13522_REG0202_SPI_WRITE,
			SPI_WRITE | (addr  & 0xFF));
	epd_sflash_wait_for_ready(par);

	while (len--) {
		par->write_reg(par, S1D13522_REG0202_SPI_WRITE, SPI_WRITE_DUMMY);
		epd_sflash_wait_for_ready(par);

		read = par->read_reg(par, S1D13522_REG0200_SPI_READ);

		*buf = (read & 0xff);
		buf++;
	}

	/* disable cs 0 */
	par->write_reg(par, S1D13522_REG0208_SPI_CS_SELECT, 0);

	/* switch back to display engine access mode */
	par->write_reg(par, S1D13522_REG0204_SPI_CONTROL, spi_ctrl_reg);

	mutex_unlock(&par->io_lock);
	if (par->power_release)
		par->power_release(par->status);

	return 0;
}

bool epd_sflash_is_case_white(struct s1d13522fb_par *par)
{
	u8 code;

	epd_sflash_read(par, SF_BORDER_ADDR, &code, 1);

	if (code == 0xfb)
		return true;
	else
		return false;
}
EXPORT_SYMBOL(epd_sflash_is_case_white);

int epd_sflash_get_vcom(struct s1d13522fb_par *par, int *val)
{
	u8 vcom_buf[5];

	epd_sflash_read(par, SF_VCOM_ADDR, vcom_buf, sizeof(vcom_buf));

	if (vcom_buf[0] != 0x0c)
		return -EINVAL;

	*val =  (vcom_buf[1] != 0xff) ? (vcom_buf[1] * 100) : 0;
	*val += (vcom_buf[3] != 0xff) ? (vcom_buf[3] * 10) : 0;
	*val += (vcom_buf[4] != 0xff) ? (vcom_buf[4] * 1) : 0;

	if ((vcom_buf[2] != 0x0b) && (vcom_buf[2] != 0xff))
		*val += -100;

	return 0;
}
EXPORT_SYMBOL(epd_sflash_get_vcom);


static ssize_t epd_sflash_read_case_color(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct s1d13522fb_board *board = dev_get_drvdata(dev);

	if (!board->fb_par) {
		dev_err(dev, "framebuffer deriver unregistered\n");
		return -ENODEV;
	}

	return snprintf(buf, PAGE_SIZE, "%s",
		epd_sflash_is_case_white(board->fb_par) ? "white" : "black");
}


static ssize_t epd_sflash_read_vcom(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct s1d13522fb_board *board = dev_get_drvdata(dev);
	int vcom;

	if (!board->fb_par) {
		dev_err(dev, "framebuffer deriver unregistered\n");
		return -ENODEV;
	}

	if (epd_sflash_get_vcom(board->fb_par, &vcom))
		return -EIO;
	else
		return snprintf(buf, PAGE_SIZE, "%d", vcom);
}

static DEVICE_ATTR(read_case_color, 0444, epd_sflash_read_case_color, NULL);
static DEVICE_ATTR(read_epd_vcom, 0444, epd_sflash_read_vcom, NULL);


static int __devinit epd_sflash_probe(struct platform_device *pdev)
{
	struct s1d13522fb_board *board;
	int ret;

	board = pdev->dev.platform_data;
	if (!board)
		return -EINVAL;

	if (!board->fb_par) {
		dev_err(&pdev->dev,
			"s1d13522 framebuffer driver should be inited first\n");
		return -EINVAL;
	}

	/* try to count device specific driver, if can't, platform recalls */
	if (!try_module_get(board->owner))
		return -ENODEV;

	platform_set_drvdata(pdev, board);

	ret = device_create_file(&pdev->dev, &dev_attr_read_case_color);
	ret |= device_create_file(&pdev->dev, &dev_attr_read_epd_vcom);

	if (ret) {
		dev_err(&pdev->dev,"failed register sysfs file (%d)\n", ret);
		device_remove_file(&pdev->dev, &dev_attr_read_case_color);
		device_remove_file(&pdev->dev, &dev_attr_read_epd_vcom);
		module_put(board->owner);
		return 0;
	}

	return 0;


}

static int __devexit epd_sflash_remove(struct platform_device *pdev)
{
	struct s1d13522fb_board *board = platform_get_drvdata(pdev);

	if (board) {
		device_remove_file(&pdev->dev, &dev_attr_read_case_color);
		device_remove_file(&pdev->dev, &dev_attr_read_epd_vcom);
		module_put(board->owner);
	}

	return 0;
}


static struct platform_driver epd_sflash_driver = {
	.probe		= epd_sflash_probe,
	.remove		= epd_sflash_remove,
	.driver = {
		.name	= "epd_sflash",
	},
};

module_platform_driver(epd_sflash_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Driver for EPD panel serial flash");
