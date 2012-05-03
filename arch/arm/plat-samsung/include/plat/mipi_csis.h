/*
 * Copyright (C) 2010 - 2012 Samsung Electronics Co., Ltd.
 *
 * S5P series MIPI CSI slave device support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __PLAT_SAMSUNG_MIPI_CSIS_H_
#define __PLAT_SAMSUNG_MIPI_CSIS_H_ __FILE__

struct platform_device;

/**
 * struct s5p_platform_mipi_csis - platform data for S5P MIPI-CSIS driver
 * @clk_rate:  bus clock frequency
 * @lanes:     number of data lanes used
 * @hs_settle: HS-RX settle time
 */
struct s5p_platform_mipi_csis {
	unsigned long clk_rate;
	u8 lanes;
	u8 hs_settle;
};

/**
 * s5p_csis_phy_enable - global MIPI-CSI receiver D-PHY control
 * @pdev:   MIPI-CSIS platform device
 * @id:     MIPI-CSIS harware instance index (0...1)
 * @on:     true to enable D-PHY and deassert its reset
 *          false to disable D-PHY
 * @return: 0 on success, or negative error code on failure
 */
int s5p_csis_phy_enable(struct platform_device *pdev, int id, bool on);

#endif /* __PLAT_SAMSUNG_MIPI_CSIS_H_ */
