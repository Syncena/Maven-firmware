/*
 * Copyright (c) 2022, Steve C. Woodford.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>
#include <stdlib.h>

#include "hardware.h"

void
hw_init(void)
{

	/*
	 * Configure the seven buffer chips (SN74LVC1T45) in input mode
	 * by driving their direction pins low. This ensures their outputs
	 * are tri-stated so as not to interfere with any target hooked up.
	 * The pins have pull-downs so should power up in this safe mode.
	 *
	 * The downside of this is that the buffers will be driving the
	 * pins on our side so make sure all our signal pins are tri-state
	 * too (basically put them in input mode).
	 *
	 * Also switch off the TxD/RxD bridge.
	 */

	/* Pin 1: external buffer */
	HAL_GPIO_EXT1_DIR_clr();	/* Input direction */
	HAL_GPIO_EXT1_DIR_out();	/* Drive buffer control pin */

	/* Pin 3: external buffer */
	HAL_GPIO_EXT3_DIR_clr();	/* Input direction */
	HAL_GPIO_EXT3_DIR_out();	/* Drive buffer control pin */

	/* Pin 4: external buffer */
	HAL_GPIO_EXT4_DIR_clr();	/* Input direction */
	HAL_GPIO_EXT4_DIR_out();	/* Drive buffer control pin */

	/* Pin 5: external buffer */
	HAL_GPIO_EXT5_DIR_clr();	/* Input direction */
	HAL_GPIO_EXT5_DIR_out();	/* Drive buffer control pin */

	/* Pin 7: external buffer */
	HAL_GPIO_EXT7_DIR_clr();	/* Input direction */
	HAL_GPIO_EXT7_DIR_out();	/* Drive buffer control pin */

	/* Pin 8: external buffer */
	HAL_GPIO_EXT8_DIR_clr();	/* Input direction */
	HAL_GPIO_EXT8_DIR_out();	/* Drive buffer control pin */

	/* Pin 9: external buffer */
	HAL_GPIO_EXT9_DIR_clr();	/* Input direction */
	HAL_GPIO_EXT9_DIR_out();	/* Drive buffer control pin */

	/* Pin 9: external buffer */
	HAL_GPIO_BRIDGE_clr();		/* Low to disable */
	HAL_GPIO_BRIDGE_out();		/* Drive buffer control pin */
}
