/* Copyright (c) 2013, Pixelworks, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef _IRIS2_IO_H
#define _IRIS2_IO_H

void iris2_io_suspend(void);
void iris2_io_resume(void);
void iris2_io_reset(void);
void iris2_io_set_reset(int use_direction, int value);

enum{
	IRIS2_23,
	IRIS2_26,
	IRIS2_28,
	IRIS2P_50 = 50,
};
int iris2_get_chip_version(void);

#endif
