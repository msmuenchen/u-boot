/*
 * (C) Copyright 2012-2013,2015 Stephen Warren
 *
 * SPDX-License-Identifier:	GPL-2.0
 */

#include <common.h>
#include <config.h>
#include <dm.h>
#include <fdt_support.h>
#include <fdt_simplefb.h>
#include <lcd.h>
#include <memalign.h>
#include <mmc.h>
#include <asm/gpio.h>
#include <asm/arch/mbox.h>
#include <asm/arch/sdhci.h>
#include <asm/global_data.h>
#include <dm/platform_data/serial_pl01x.h>

DECLARE_GLOBAL_DATA_PTR;

static const struct bcm2835_gpio_platdata gpio_platdata = {
	.base = BCM2835_GPIO_BASE,
};

U_BOOT_DEVICE(bcm2835_gpios) = {
	.name = "gpio_bcm2835",
	.platdata = &gpio_platdata,
};

static const struct pl01x_serial_platdata serial_platdata = {
#ifdef CONFIG_BCM2836
	.base = 0x3f201000,
#else
	.base = 0x20201000,
#endif
	.type = TYPE_PL011,
	.clock = 3000000,
};

U_BOOT_DEVICE(bcm2835_serials) = {
	.name = "serial_pl01x",
	.platdata = &serial_platdata,
};

struct msg_get_arm_mem {
	struct bcm2835_mbox_hdr hdr;
	struct bcm2835_mbox_tag_get_arm_mem get_arm_mem;
	u32 end_tag;
};

struct msg_get_board_rev {
	struct bcm2835_mbox_hdr hdr;
	struct bcm2835_mbox_tag_get_board_rev get_board_rev;
	u32 end_tag;
};

struct msg_get_mac_address {
	struct bcm2835_mbox_hdr hdr;
	struct bcm2835_mbox_tag_get_mac_address get_mac_address;
	u32 end_tag;
};

struct msg_set_power_state {
	struct bcm2835_mbox_hdr hdr;
	struct bcm2835_mbox_tag_set_power_state set_power_state;
	u32 end_tag;
};

struct msg_get_clock_rate {
	struct bcm2835_mbox_hdr hdr;
	struct bcm2835_mbox_tag_get_clock_rate get_clock_rate;
	u32 end_tag;
};

/* See comments in mbox.h for data source */
static const struct {
	const char *name;
	bool has_onboard_eth;
} models[] = {
	[0] = {
		"Unknown model",
		false,
	},
#ifdef CONFIG_BCM2836
	[BCM2836_BOARD_REV_2_B] = {
		"2 Model B",
		true,
	},
#else
	[BCM2835_BOARD_REV_B_I2C0_2] = {
		"Model B (no P5)",
		true,
	},
	[BCM2835_BOARD_REV_B_I2C0_3] = {
		"Model B (no P5)",
		true,
	},
	[BCM2835_BOARD_REV_B_I2C1_4] = {
		"Model B",
		true,
	},
	[BCM2835_BOARD_REV_B_I2C1_5] = {
		"Model B",
		true,
	},
	[BCM2835_BOARD_REV_B_I2C1_6] = {
		"Model B",
		true,
	},
	[BCM2835_BOARD_REV_A_7] = {
		"Model A",
		false,
	},
	[BCM2835_BOARD_REV_A_8] = {
		"Model A",
		false,
	},
	[BCM2835_BOARD_REV_A_9] = {
		"Model A",
		false,
	},
	[BCM2835_BOARD_REV_B_REV2_d] = {
		"Model B rev2",
		true,
	},
	[BCM2835_BOARD_REV_B_REV2_e] = {
		"Model B rev2",
		true,
	},
	[BCM2835_BOARD_REV_B_REV2_f] = {
		"Model B rev2",
		true,
	},
	[BCM2835_BOARD_REV_B_PLUS] = {
		"Model B+",
		true,
	},
	[BCM2835_BOARD_REV_CM] = {
		"Compute Module",
		false,
	},
	[BCM2835_BOARD_REV_A_PLUS] = {
		"Model A+",
		false,
	},
	[BCM2835_BOARD_REV_B_PLUS_13] = {
		"Model B+",
		true,
	},
	[BCM2835_BOARD_REV_CM_14] = {
		"Compute Module",
		false,
	},
	[BCM2835_BOARD_REV_A_PLUS_15] = {
		"Model A+",
		false,
	},
#endif
};

u32 rpi_board_rev = 0;

int dram_init(void)
{
	ALLOC_CACHE_ALIGN_BUFFER(struct msg_get_arm_mem, msg, 1);
	int ret;

	BCM2835_MBOX_INIT_HDR(msg);
	BCM2835_MBOX_INIT_TAG(&msg->get_arm_mem, GET_ARM_MEMORY);

	ret = bcm2835_mbox_call_prop(BCM2835_MBOX_PROP_CHAN, &msg->hdr);
	if (ret) {
		printf("bcm2835: Could not query ARM memory size\n");
		return -1;
	}

	gd->ram_size = msg->get_arm_mem.body.resp.mem_size;

	return 0;
}

static void set_usbethaddr(void)
{
	ALLOC_CACHE_ALIGN_BUFFER(struct msg_get_mac_address, msg, 1);
	int ret;

	if (!models[rpi_board_rev].has_onboard_eth)
		return;

	if (getenv("usbethaddr"))
		return;

	BCM2835_MBOX_INIT_HDR(msg);
	BCM2835_MBOX_INIT_TAG(&msg->get_mac_address, GET_MAC_ADDRESS);

	ret = bcm2835_mbox_call_prop(BCM2835_MBOX_PROP_CHAN, &msg->hdr);
	if (ret) {
		printf("bcm2835: Could not query MAC address\n");
		/* Ignore error; not critical */
		return;
	}

	eth_setenv_enetaddr("usbethaddr", msg->get_mac_address.body.resp.mac);

	return;
}

#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
static void set_board_info(void)
{
	char str_rev[11];
	sprintf(str_rev, "0x%X", rpi_board_rev);
	setenv("board_rev", str_rev);
	setenv("board_name", models[rpi_board_rev].name);
}
#endif /* CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG */

int misc_init_r(void)
{
	set_usbethaddr();
#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	set_board_info();
#endif

	/*
	 * The RPi start.elf passes important system configuration
	 * like memory sizes and LED GPIO assignments to the kernel
	 * via kernel command line.
	 *
	 * Therefore we have to parse the passed ATAG or FDT to get the
	 * commandline and pass it through to script-land via the env
	 * variable "bootargs_orig" so it can be passed on.
	 *
	 * By default, start.elf assumes a non-DT capable kernel and
	 * passes the commandline via ATAG; this requires u-boot to
	 * load the FDT from /boot and pass it on. This works if no
	 * overlays have been passed through, but once overlays are in
	 * the mix, stuff gets complicated to do in u-boot.
	 *
	 * To force start.elf to pass a processed, ready-to-go DT,
	 * you have to use the mkknlimg tool on the u-boot image:
	 * ./mkknlimg --dtok u-boot.bin /boot/u-boot.bin
	 *
	 * mkknlimg can be obtained from https://github.com/raspberrypi/tools/blob/master/mkimage/knlinfo
	 *
	 * User scripts can check for successful bootargs retrieval
	 * in the env variable pi_bootmode, which is either fdt, atag
	 * or unknown in case of an error.
	 *
	 * The location 0x100 is hard-coded in start.elf, and it is
	 * the same as in the default bootscripts, so you only have
	 * to omit the fatload command loading the raw FDT to get
	 * going.
	 */

	void* ptr = (char*) 0x100;
	struct tag_header* atag_ptr = ptr;
	setenv("pi_bootmode","unknown");

	if(atag_ptr->tag != ATAG_CORE) {
		if(atag_ptr->size == be32_to_cpu(FDT_MAGIC)) {
			set_working_fdt_addr((ulong) ptr);
			int nodeoffset = fdt_path_offset(working_fdt, "/chosen");
			int len;
			const void* nodep = fdt_getprop(working_fdt, nodeoffset, "bootargs", &len);
			if(len==0) {
				printf("WARNING: Could not determine bootargs from FDT!\n");
			} else {
				printf("Set bootargs_orig from FDT\n");
				setenv("bootargs_orig", (char*) nodep);
				setenv("pi_bootmode","fdt");
			}
		} else {
			printf("Warning: start.elf did not pass ATAG or FDT parameters\n");
		}
	} else {
		while(atag_ptr->tag != ATAG_NONE) {
			printf("at %p, have %x (len %x)\n",atag_ptr,atag_ptr->tag, atag_ptr->size);
			if(atag_ptr->tag == ATAG_CMDLINE) {
				char* old_cmdline = ptr + 8;
				printf("Set bootargs_orig from ATAG\n");
				setenv("bootargs_orig", old_cmdline);
				setenv("pi_bootmode", "atag");
			}
			ptr = ptr + (atag_ptr->size * 4);
			atag_ptr=ptr;
		}
	}

	return 0;
}

static int power_on_module(u32 module)
{
	ALLOC_CACHE_ALIGN_BUFFER(struct msg_set_power_state, msg_pwr, 1);
	int ret;

	BCM2835_MBOX_INIT_HDR(msg_pwr);
	BCM2835_MBOX_INIT_TAG(&msg_pwr->set_power_state,
			      SET_POWER_STATE);
	msg_pwr->set_power_state.body.req.device_id = module;
	msg_pwr->set_power_state.body.req.state =
		BCM2835_MBOX_SET_POWER_STATE_REQ_ON |
		BCM2835_MBOX_SET_POWER_STATE_REQ_WAIT;

	ret = bcm2835_mbox_call_prop(BCM2835_MBOX_PROP_CHAN,
				     &msg_pwr->hdr);
	if (ret) {
		printf("bcm2835: Could not set module %u power state\n",
		       module);
		return -1;
	}

	return 0;
}

static void get_board_rev(void)
{
	ALLOC_CACHE_ALIGN_BUFFER(struct msg_get_board_rev, msg, 1);
	int ret;
	const char *name;

	BCM2835_MBOX_INIT_HDR(msg);
	BCM2835_MBOX_INIT_TAG(&msg->get_board_rev, GET_BOARD_REV);

	ret = bcm2835_mbox_call_prop(BCM2835_MBOX_PROP_CHAN, &msg->hdr);
	if (ret) {
		printf("bcm2835: Could not query board revision\n");
		/* Ignore error; not critical */
		return;
	}

	/*
	 * For details of old-vs-new scheme, see:
	 * https://github.com/pimoroni/RPi.version/blob/master/RPi/version.py
	 * http://www.raspberrypi.org/forums/viewtopic.php?f=63&t=99293&p=690282
	 * (a few posts down)
	 *
	 * For the RPi 1, bit 24 is the "warranty bit", so we mask off just the
	 * lower byte to use as the board rev:
	 * http://www.raspberrypi.org/forums/viewtopic.php?f=63&t=98367&start=250
	 * http://www.raspberrypi.org/forums/viewtopic.php?f=31&t=20594
	 */
	rpi_board_rev = msg->get_board_rev.body.resp.rev;
	if (rpi_board_rev & 0x800000)
		rpi_board_rev = (rpi_board_rev >> 4) & 0xff;
	else
		rpi_board_rev &= 0xff;
	if (rpi_board_rev >= ARRAY_SIZE(models)) {
		printf("RPI: Board rev %u outside known range\n",
		       rpi_board_rev);
		rpi_board_rev = 0;
	}
	if (!models[rpi_board_rev].name) {
		printf("RPI: Board rev %u unknown\n", rpi_board_rev);
		rpi_board_rev = 0;
	}

	name = models[rpi_board_rev].name;
	printf("RPI %s\n", name);
}

int board_init(void)
{
	get_board_rev();

	gd->bd->bi_boot_params = 0x100;

	return power_on_module(BCM2835_MBOX_POWER_DEVID_USB_HCD);
}

int board_mmc_init(bd_t *bis)
{
	ALLOC_CACHE_ALIGN_BUFFER(struct msg_get_clock_rate, msg_clk, 1);
	int ret;

	power_on_module(BCM2835_MBOX_POWER_DEVID_SDHCI);

	BCM2835_MBOX_INIT_HDR(msg_clk);
	BCM2835_MBOX_INIT_TAG(&msg_clk->get_clock_rate, GET_CLOCK_RATE);
	msg_clk->get_clock_rate.body.req.clock_id = BCM2835_MBOX_CLOCK_ID_EMMC;

	ret = bcm2835_mbox_call_prop(BCM2835_MBOX_PROP_CHAN, &msg_clk->hdr);
	if (ret) {
		printf("bcm2835: Could not query eMMC clock rate\n");
		return -1;
	}

	return bcm2835_sdhci_init(BCM2835_SDHCI_BASE,
				  msg_clk->get_clock_rate.body.resp.rate_hz);
}

int ft_board_setup(void *blob, bd_t *bd)
{
	/*
	 * For now, we simply always add the simplefb DT node. Later, we
	 * should be more intelligent, and e.g. only do this if no enabled DT
	 * node exists for the "real" graphics driver.
	 */
	lcd_dt_simplefb_add_node(blob);

	return 0;
}
