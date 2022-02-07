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

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "rtos.h"
#include "platform.h"
#include "hardware.h"
#include "shell.h"
#include "sam_clocks.h"

#if (RELEASE_BUILD == 0)
SHELL_CMD_DECL(gpio, gpio_cmd, "Dump GPIO configuration");

struct gpio_mux {
	const char *desc[16];
};

static const struct gpio_mux gpioa_mux[32] = {
	[0]  ={{/* A */	"EIC/EXTINT[0]",
		/* B */	NULL,
		/* C */	NULL,
		/* D */	"SERCOM1/PAD[0]",
		/* E */	"TC2/WO[0]",
		/* F */	NULL,
		/* G */	NULL,
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	NULL,
	      }},
	[1]  ={{/* A */	"EIC/EXTINT[1]",
		/* B */	NULL,
		/* C */	NULL,
		/* D */	"SERCOM1/PAD[1]",
		/* E */	"TC2/WO[1]",
		/* F */	NULL,
		/* G */	NULL,
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	NULL,
	      }},
	[2]  ={{/* A */	"EIC/EXTINT[2]",
		/* B */	"ADC0/AIN[0] - DAC/VOUT[0]",
		/* C */	NULL,
		/* D */	NULL,
		/* E */	NULL,
		/* F */	NULL,
		/* G */	NULL,
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	NULL,
	      }},
	[3]  ={{/* A */	"EIC/EXTINT[3]",
		/* B */	"ANAREF/VREFA - ADC0/AIN[1] - X0/Y0",
		/* C */	NULL,
		/* D */	NULL,
		/* E */	NULL,
		/* F */	NULL,
		/* G */	NULL,
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	NULL,
	      }},
	[4]  ={{/* A */	"EIC/EXTINT[4]",
		/* B */	"ANAREF/VREFB - ADC0/AIN[5] - AC/AIN[0] - X3/Y3",
		/* C */	NULL,
		/* D */	"SERCOM0/PAD[0]",
		/* E */	"TC0/WO[0]",
		/* F */	NULL,
		/* G */	NULL,
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	"CCL/IN[0]"
	      }},
	[5]  ={{/* A */	"EIC/EXTINT[5]",
		/* B */	"ADC0/AIN[5] - AC/AIN[1] - DAC/VOUT[1]",
		/* C */	NULL,
		/* D */	"SERCOM0/PAD[1]",
		/* E */	"TC0/WO[1]",
		/* F */	NULL,
		/* G */	NULL,
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	"CCL/IN[1]"
	      }},
	[6]  ={{/* A */	"EIC/EXTINT[6]",
		/* B */	"ANAREF/VREFC - ADC0/AIN[6] - AC/AIN[2] - X4/Y4",
		/* C */	NULL,
		/* D */	"SERCOM0/PAD[2]",
		/* E */	"TC1/WO[0]",
		/* F */	NULL,
		/* G */	NULL,
		/* H */	NULL,
		/* I */	"SDHC0/SDCD",
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	"CCL/IN[2]"
	      }},
	[7]  ={{/* A */	"EIC/EXTINT[7]",
		/* B */	"ADC0/AIN[7] - AC/AIN[3] - X5/Y5",
		/* C */	NULL,
		/* D */	"SERCOM0/PAD[3]",
		/* E */	"TC1/WO[1]",
		/* F */	NULL,
		/* G */	NULL,
		/* H */	NULL,
		/* I */	"SDHC0/SDWP",
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	"CCL/OUT[0]"
	      }},
	[8]  ={{/* A */	"EIC/NMI",
		/* B */	"ADC0/AIN[8] - ADC1/AIN[2] - X6/Y6",
		/* C */	"SERCOM0/PAD[0]",
		/* D */	"SERCOM2/PAD[1]",
		/* E */	"TC0/WO[0]",
		/* F */	"TCC0/WO[0]",
		/* G */	"TCC1/WO[4]",
		/* H */	"QSPI/DATA[0]",
		/* I */	"SDHC0/SDCMD",
		/* J */	"I2S/MCK[0]",
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	"CCL/IN[3]"
	      }},
	[9]  ={{/* A */	"EIC/EXTINT[9]",
		/* B */	"ADC0/AIN[9] - ADC1/AIN[3] - X7/Y7",
		/* C */	"SERCOM0/PAD[1]",
		/* D */	"SERCOM2/PAD[0]",
		/* E */	"TC0/WO[1]",
		/* F */	"TCC0/WO[1]",
		/* G */	"TCC1/WO[5]",
		/* H */	"QSPI/DATA[1]",
		/* I */	"SDHC0/SDDAT[0]",
		/* J */	"I2S/FS[0]",
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	"CCL/IN[4]"
	      }},
	[10] ={{/* A */	"EIC/EXTINT[10]",
		/* B */	"ADC0/AIN[10] - X8/Y8",
		/* C */	"SERCOM0/PAD[2]",
		/* D */	"SERCOM2/PAD[2]",
		/* E */	"TC1/WO[0]",
		/* F */	"TCC0/WO[2]",
		/* G */	"TCC1/WO[6]",
		/* H */	"QSPI/DATA[2]",
		/* I */	"SDHC0/SDDAT[1]",
		/* J */	"I2S/SCK[0]",
		/* K */	NULL,
		/* L */	NULL,
		/* M */	"GCLK/IO[4]",
		/* N */	"CCL/IN[5]"
	      }},
	[11] ={{/* A */	"EIC/EXTINT[11]",
		/* B */	"ADC0/AIN[11] - X9/Y9",
		/* C */	"SERCOM0/PAD[3]",
		/* D */	"SERCOM2/PAD[3]",
		/* E */	"TC1/WO[1]",
		/* F */	"TCC0/WO[3]",
		/* G */	"TCC1/WO[7]",
		/* H */	"QSPI/DATA[3]",
		/* I */	"SDHC0/SDDAT[2]",
		/* J */	"I2S/SDO",
		/* K */	NULL,
		/* L */	NULL,
		/* M */	"GCLK/IO[5]",
		/* N */	"CCL/OUT[1]"
	      }},
	[12] ={{/* A */	"EIC/EXTINT[12]",
		/* B */	NULL,
		/* C */	"SERCOM2/PAD[0]",
		/* D */	"SERCOM4/PAD[1]",
		/* E */	"TC2/WO[0]",
		/* F */	"TCC0/WO[6]",
		/* G */	"TCC1/WO[2]",
		/* H */	NULL,
		/* I */	"SDHC0/SDCD",
		/* J */	NULL,
		/* K */	"PCC/DEN1",
		/* L */	"GMAC/GRX[1]",
		/* M */	"AC/CMP[0]",
		/* N */	NULL
	      }},
	[13] ={{/* A */	"EIC/EXTINT[13]",
		/* B */	NULL,
		/* C */	"SERCOM2/PAD[1]",
		/* D */	"SERCOM4/PAD[0]",
		/* E */	"TC2/WO[1]",
		/* F */	"TCC0/WO[7]",
		/* G */	"TCC1/WO[3]",
		/* H */	NULL,
		/* I */	"SDHC0/SDWP",
		/* J */	NULL,
		/* K */	"PCC/DEN2",
		/* L */	"GMAC/GRX[0]",
		/* M */	"AC/CMP[1]",
		/* N */	NULL
	      }},
	[14] ={{/* A */	"EIC/EXTINT[14]",
		/* B */	NULL,
		/* C */	"SERCOM2/PAD[2]",
		/* D */	"SERCOM4/PAD[2]",
		/* E */	"TC3/WO[0]",
		/* F */	"TCC2/WO[0]",
		/* G */	"TCC1/WO[2]",
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	"PCC/CLK",
		/* L */	"GMAC/GTXCK",
		/* M */	"GCLK/IO[0]",
		/* N */	NULL
	      }},
	[15] ={{/* A */	"EIC/EXTINT[15]",
		/* B */	NULL,
		/* C */	"SERCOM2/PAD[3]",
		/* D */	"SERCOM4/PAD[3]",
		/* E */	"TC3/WO[1]",
		/* F */	"TCC2/WO[1]",
		/* G */	"TCC1/WO[3]",
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	"GMAC/GRXERR",
		/* M */	"GCLK/IO[1]",
		/* N */	NULL
	      }},
	[16] ={{/* A */	"EIC/EXTINT[0]",
		/* B */	"X10/Y10",
		/* C */	"SERCOM1/PAD[0]",
		/* D */	"SERCOM3/PAD[1]",
		/* E */	"TC2/WO[0]",
		/* F */	"TCC1/WO[0]",
		/* G */	"TCC0/WO[4]",
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	"PCC/DATA[0]",
		/* L */	"GMAC/GCRS/GRXDV",
		/* M */	"GCLK/IO[2]",
		/* N */	"CCL/IN[0]"
	      }},
	[17] ={{/* A */	"EIC/EXTINT[1]",
		/* B */	"X11/Y11",
		/* C */	"SERCOM1/PAD[1]",
		/* D */	"SERCOM3/PAD[0]",
		/* E */	"TC2/WO[1]",
		/* F */	"TCC1/WO[1]",
		/* G */	"TCC0/WO[5]",
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	"PCC/DATA[1]",
		/* L */	"GMAC/GTXEN",
		/* M */	"GCLK/IO[3]",
		/* N */	"CCL/IN[1]"
	      }},
	[18] ={{/* A */	"EIC/EXTINT[2]",
		/* B */	"X12/Y12",
		/* C */	"SERCOM1/PAD[2]",
		/* D */	"SERCOM3/PAD[2]",
		/* E */	"TC3/WO[0]",
		/* F */	"TCC1/WO[2]",
		/* G */	"TCC0/WO[6]",
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	"PCC/DATA[2]",
		/* L */	"GMAC/GTX[0]",
		/* M */	"AC/CMP[0]",
		/* N */	"CCL/IN[2]"
	      }},
	[19] ={{/* A */	"EIC/EXTINT[3]",
		/* B */	"X13/Y13",
		/* C */	"SERCOM1/PAD[3]",
		/* D */	"SERCOM3/PAD[3]",
		/* E */	"TC3/WO[1]",
		/* F */	"TCC1/WO[3]",
		/* G */	"TCC0/WO[7]",
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	"PCC/DATA[3]",
		/* L */	"GMAC/GTX[1]",
		/* M */	"AC/CMP[1]",
		/* N */	"CCL/OUT[0]"
	      }},
	[20] ={{/* A */	"EIC/EXTINT[4]",
		/* B */	"X14/Y14",
		/* C */	"SERCOM5/PAD[2]",
		/* D */	"SERCOM3/PAD[2]",
		/* E */	"TC7/WO[0]",
		/* F */	"TCC1/WO[4]",
		/* G */	"TCC0/WO[0]",
		/* H */	NULL,
		/* I */	"SDHC1/SDCMD",
		/* J */	"I2S/FS[0]",
		/* K */	"PCC/DATA[4]",
		/* L */	"GMAC/GMDC",
		/* M */	NULL,
		/* N */	NULL
	      }},
	[21] ={{/* A */	"EIC/EXTINT[5]",
		/* B */	"X15/Y15",
		/* C */	"SERCOM5/PAD[3]",
		/* D */	"SERCOM3/PAD[3]",
		/* E */	"TC7/WO[1]",
		/* F */	"TCC1/WO[5]",
		/* G */	"TCC0/WO[1]",
		/* H */	NULL,
		/* I */	"SDHC1/SDCK",
		/* J */	"I2S/SDO",
		/* K */	"PCC/DATA[5]",
		/* L */	"GMAC/GMDIO",
		/* M */	NULL,
		/* N */	NULL
	      }},
	[22] ={{/* A */	"EIC/EXTINT[6]",
		/* B */	"X16/Y16",
		/* C */	"SERCOM3/PAD[0]",
		/* D */	"SERCOM5/PAD[1]",
		/* E */	"TC4/WO[0]",
		/* F */	"TCC1/WO[6]",
		/* G */	"TCC0/WO[2]",
		/* H */	NULL,
		/* I */	"CAN0/TX",
		/* J */	"I2S/SDI",
		/* K */	"PCC/DATA[6]",
		/* L */	NULL,
		/* M */	NULL,
		/* N */	"CCL/IN[6]"
	      }},
	[23] ={{/* A */	"EIC/EXTINT[7]",
		/* B */	"X17/Y17",
		/* C */	"SERCOM3/PAD[1]",
		/* D */	"SERCOM5/PAD[0]",
		/* E */	"TC4/WO[1]",
		/* F */	"TCC1/WO[7]",
		/* G */	"TCC0/WO[3]",
		/* H */	"USB/SOF_1KHZ",
		/* I */	"CAN0/RX",
		/* J */	"I2S/FS[1]",
		/* K */	"PCC/DATA[7]",
		/* L */	NULL,
		/* M */	NULL,
		/* N */	"CCL/IN[7]"
	      }},
	[24] ={{/* A */	"EIC/EXTINT[8]",
		/* B */	NULL,
		/* C */	"SERCOM3/PAD[2]",
		/* D */	"SERCOM5/PAD[2]",
		/* E */	"TC5/WO[0]",
		/* F */	"TCC2/WO[2]",
		/* G */	"PDEC/QDI[0]",
		/* H */	"USB/D-",
		/* I */	"CAN0/TX",
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	"CCL/IN[8]"
	      }},
	[25] ={{/* A */	"EIC/EXTINT[9]",
		/* B */	NULL,
		/* C */	"SERCOM3/PAD[3]",
		/* D */	"SERCOM5/PAD[3]",
		/* E */	"TC5/WO[1]",
		/* F */	NULL,
		/* G */	"PDEC/QDI[1]",
		/* H */	"USB/D+",
		/* I */	"CAN0/RX",
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	"CCL/OUT[2]"
	      }},
	[27] ={{/* A */	"EIC/EXTINT[11]",
		/* B */	"X18/Y18",
		/* C */	NULL,
		/* D */	NULL,
		/* E */	NULL,
		/* F */	NULL,
		/* G */	NULL,
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	"GCLK/IO[1]",
		/* N */	NULL
	      }},
	[30] ={{/* A */	"EIC/EXTINT[14]",
		/* B */	"X19/Y19",
		/* C */	NULL,
		/* D */	"SERCOM1/PAD[2]",
		/* E */	"TC6/WO[0]",
		/* F */	"TCC2/WO[0]",
		/* G */	NULL,
		/* H */	"SWCLK",
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	"GCLK/IO[0]",
		/* N */	"CCL/IN[3]"
	      }},
	[31] ={{/* A */	"EIC/EXTINT[15]",
		/* B */	NULL,
		/* C */	NULL,
		/* D */	"SERCOM1/PAD[3]",
		/* E */	"TC6/WO[1]",
		/* F */	"TCC2/WO[1]",
		/* G */	NULL,
		/* H */	"SWDIO",
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	"CCL/OUT[1]"
	      }},
};

static const struct gpio_mux gpiob_mux[32] = {
	[0]  ={{/* A */	"EIC/EXTINT[0]",
		/* B */	"ADC0/AIN[12] - X30/Y30",
		/* C */	NULL,
		/* D */	"SERCOM5/PAD[2]",
		/* E */	"TC7/WO[0]",
		/* F */	NULL,
		/* G */	NULL,
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	"CCL/IN[1]",
	      }},
	[1]  ={{/* A */	"EIC/EXTINT[1]",
		/* B */	"ADC0/AIN[13] - X31/Y31",
		/* C */	NULL,
		/* D */	"SERCOM5/PAD[3]",
		/* E */	"TC7/WO[1]",
		/* F */	NULL,
		/* G */	NULL,
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	"CCL/IN[2]",
	      }},
	[2]  ={{/* A */	"EIC/EXTINT[2]",
		/* B */	"ADC0/AIN[14] - X20/Y20",
		/* C */	NULL,
		/* D */	"SERCOM5/PAD[0]",
		/* E */	"TC6/WO[0]",
		/* F */	"TCC2/WO[2]",
		/* G */	NULL,
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	"CCL/OUT[0]",
	      }},
	[3]  ={{/* A */	"EIC/EXTINT[3]",
		/* B */	"ADC0/AIN[15] - X21/Y21",
		/* C */	NULL,
		/* D */	"SERCOM5/PAD[1]",
		/* E */	"TC6/WO[1]",
		/* F */	NULL,
		/* G */	NULL,
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	NULL,
	      }},
	[4]  ={{/* A */	"EIC/EXTINT[4]",
		/* B */	"ADC1/AIN[6] - X22/Y22",
		/* C */	NULL,
		/* D */	NULL,
		/* E */	NULL,
		/* F */	NULL,
		/* G */	NULL,
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	NULL,
	      }},
	[5]  ={{/* A */	"EIC/EXTINT[5]",
		/* B */	"ADC1/AIN[7] - X23/Y23",
		/* C */	NULL,
		/* D */	NULL,
		/* E */	NULL,
		/* F */	NULL,
		/* G */	NULL,
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	NULL,
	      }},
	[6]  ={{/* A */	"EIC/EXTINT[6]",
		/* B */	"ADC1/AIN[8] - X24/Y24",
		/* C */	NULL,
		/* D */	NULL,
		/* E */	NULL,
		/* F */	NULL,
		/* G */	NULL,
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	"CCL/IN[6]"
	      }},
	[7]  ={{/* A */	"EIC/EXTINT[7]",
		/* B */	"ADC1/AIN[9] - X25/Y25",
		/* C */	NULL,
		/* D */	NULL,
		/* E */	NULL,
		/* F */	NULL,
		/* G */	NULL,
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	"CCL/IN[7]"
	      }},
	[8]  ={{/* A */	"EIC/EXTINT[8]",
		/* B */	"ADC0/AIN[2] - ADC1/AIN[0] - X1/Y1",
		/* C */	NULL,
		/* D */	"SERCOM4/PAD[0]",
		/* E */	"TC4/WO[0]",
		/* F */	NULL,
		/* G */	NULL,
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	"CCL/IN[8]"
	      }},
	[9]  ={{/* A */	"EIC/EXTINT[9]",
		/* B */	"ADC0/AIN[3] - ADC1/AIN[1] - X2/Y2",
		/* C */	NULL,
		/* D */	"SERCOM4/PAD[1]",
		/* E */	"TC4/WO[1]",
		/* F */	NULL,
		/* G */	NULL,
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	"CCL/OUT[2]"
	      }},
	[10] ={{/* A */	"EIC/EXTINT[10]",
		/* B */	NULL,
		/* C */	NULL,
		/* D */	"SERCOM4/PAD[2]",
		/* E */	"TC5/WO[0]",
		/* F */	"TCC0/WO[4]",
		/* G */	"TCC1/WO[0]",
		/* H */	"QSPI/SCK",
		/* I */	"SDHC0/SDDAT[3]",
		/* J */	"I2S/SDI",
		/* K */	NULL,
		/* L */	NULL,
		/* M */	"GCLK/IO[4]",
		/* N */	"CCL/IN[11]"
	      }},
	[11] ={{/* A */	"EIC/EXTINT[11]",
		/* B */	NULL,
		/* C */	NULL,
		/* D */	"SERCOM4/PAD[3]",
		/* E */	"TC5/WO[1]",
		/* F */	"TCC0/WO[5]",
		/* G */	"TCC1/WO[1]",
		/* H */	"QSPI/CS",
		/* I */	"SDHC0/SDCK",
		/* J */	"I2S/FS[1]",
		/* K */	NULL,
		/* L */	NULL,
		/* M */	"GCLK/IO[5]",
		/* N */	"CCL/OUT[1]"
	      }},
	[12] ={{/* A */	"EIC/EXTINT[12]",
		/* B */	"X26/Y26",
		/* C */	"SERCOM4/PAD[0]",
		/* D */	NULL,
		/* E */	"TC4/WO[0]",
		/* F */	"TCC3/WO[0]",
		/* G */	"TCC0/WO[0]",
		/* H */	"CAN1/TX",
		/* I */	"SDHC0/SDCD",
		/* J */	"I2S/SCK[1]",
		/* K */	NULL,
		/* L */	NULL,
		/* M */	"GCLK/IO[6]",
		/* N */	NULL
	      }},
	[13] ={{/* A */	"EIC/EXTINT[13]",
		/* B */	"X27/Y27",
		/* C */	"SERCOM4/PAD[1]",
		/* D */	NULL,
		/* E */	"TC4/WO[1]",
		/* F */	"TCC3/WO[1]",
		/* G */	"TCC0/WO[1]",
		/* H */	"CAN1/RX",
		/* I */	"SDHC0/SDWP",
		/* J */	"I2S/MCK[1]",
		/* K */	NULL,
		/* L */	NULL,
		/* M */	"GCLK/IO[7]",
		/* N */	NULL
	      }},
	[14] ={{/* A */	"EIC/EXTINT[14]",
		/* B */	"X28/Y28",
		/* C */	"SERCOM4/PAD[2]",
		/* D */	NULL,
		/* E */	"TC5/WO[0]",
		/* F */	"TCC4/WO[0]",
		/* G */	"TCC0/WO[2]",
		/* H */	"CAN1/TX",
		/* I */	NULL,
		/* J */	NULL,
		/* K */	"PCC/DATA[8]",
		/* L */	"GMAC/GMDC",
		/* M */	"GCLK/IO[0]",
		/* N */	"CCL/IN[9]"
	      }},
	[15] ={{/* A */	"EIC/EXTINT[15]",
		/* B */	"X29/Y29",
		/* C */	"SERCOM4/PAD[3]",
		/* D */	NULL,
		/* E */	"TC5/WO[1]",
		/* F */	"TCC4/WO[1]",
		/* G */	"TCC0/WO[3]",
		/* H */	"CAN1/RX",
		/* I */	NULL,
		/* J */	NULL,
		/* K */	"PCC/DATA[9]",
		/* L */	"GMAC/GMDIO",
		/* M */	"GCLK/IO[1]",
		/* N */	"CCL/IN[10]"
	      }},
	[16] ={{/* A */	"EIC/EXTINT[0]",
		/* B */	NULL,
		/* C */	"SERCOM5/PAD[0]",
		/* D */	NULL,
		/* E */	"TC6/WO[0]",
		/* F */	"TCC3/WO[0]",
		/* G */	"TCC0/WO[4]",
		/* H */	NULL,
		/* I */	"SDHC1/SDCD",
		/* J */	"I2S/SCK[0]",
		/* K */	NULL,
		/* L */	NULL,
		/* M */	"GCLK/IO[2]",
		/* N */	"CCL/IN[11]"
	      }},
	[17] ={{/* A */	"EIC/EXTINT[1]",
		/* B */	NULL,
		/* C */	"SERCOM5/PAD[1]",
		/* D */	NULL,
		/* E */	"TC6/WO[1]",
		/* F */	"TCC3/WO[1]",
		/* G */	"TCC0/WO[5]",
		/* H */	NULL,
		/* I */	"SDHC1/SDWP",
		/* J */	"I2S/MCK[0]",
		/* K */	NULL,
		/* L */	NULL,
		/* M */	"GCLK/IO[3]",
		/* N */	"CCL/OUT[3]"
	      }},
	[18] ={{/* A */	"EIC/EXTINT[2]",
		/* B */	NULL,
		/* C */	"SERCOM5/PAD[2]",
		/* D */	"SERCOM7/PAD[2]",
		/* E */	NULL,
		/* F */	"TCC1/WO[0]",
		/* G */	"PDEC/QDI[0]",
		/* H */	NULL,
		/* I */	"SDHC1/SDDAT[0]",
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	"GCLK/IO[4]",
		/* N */	NULL
	      }},
	[19] ={{/* A */	"EIC/EXTINT[3]",
		/* B */	NULL,
		/* C */	"SERCOM5/PAD[3]",
		/* D */	"SERCOM7/PAD[3]",
		/* E */	NULL,
		/* F */	"TCC1/WO[1]",
		/* G */	"PDEC/QDI[1]",
		/* H */	NULL,
		/* I */	"SDHC1/SDDAT[1]",
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	"GCLK/IO[5]",
		/* N */	NULL,
	      }},
	[20] ={{/* A */	"EIC/EXTINT[4]",
		/* B */	NULL,
		/* C */	"SERCOM3/PAD[0]",
		/* D */	"SERCOM7/PAD[1]",
		/* E */	NULL,
		/* F */	"TCC1/WO[0]",
		/* G */	"PDEC/QDI[2]",
		/* H */	NULL,
		/* I */	"SDHC1/SDDAT[2]",
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	"GCLK/IO[6]",
		/* N */	NULL
	      }},
	[21] ={{/* A */	"EIC/EXTINT[5]",
		/* B */	NULL,
		/* C */	"SERCOM3/PAD[1]",
		/* D */	"SERCOM7/PAD[0]",
		/* E */	NULL,
		/* F */	"TCC1/WO[3]",
		/* G */	NULL,
		/* H */	NULL,
		/* I */	"SDHC1/SDDAT[3]",
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	"GCLK/IO[7]",
		/* N */	NULL
	      }},
	[22] ={{/* A */	"EIC/EXTINT[6]",
		/* B */	NULL,
		/* C */	"SERCOM1/PAD[2]",
		/* D */	"SERCOM5/PAD[2]",
		/* E */	"TC7/WO[0]",
		/* F */	NULL,
		/* G */	"PDEC/QDI[2]",
		/* H */	"USB/SOF_1KHZ",
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	"GCLK/IO[0]",
		/* N */	"CCL/IN[0]"
	      }},
	[23] ={{/* A */	"EIC/EXTINT[7]",
		/* B */	NULL,
		/* C */	"SERCOM1/PAD[3]",
		/* D */	"SERCOM5/PAD[3]",
		/* E */	"TC7/WO[1]",
		/* F */	NULL,
		/* G */	"PDEC/QDI[0]",
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	"GCLK/IO[1]",
		/* N */	"CCL/OUT[0]"
	      }},
	[24] ={{/* A */	"EIC/EXTINT[8]",
		/* B */	NULL,
		/* C */	"SERCOM0/PAD[0]",
		/* D */	"SERCOM2/PAD[1]",
		/* E */	NULL,
		/* F */	NULL,
		/* G */	"PDEC/QDI[1]",
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	"AC/CMP[0]",
		/* N */	NULL
	      }},
	[25] ={{/* A */	"EIC/EXTINT[9]",
		/* B */	NULL,
		/* C */	"SERCOM0/PAD[1]",
		/* D */	"SERCOM2/PAD[0]",
		/* E */	NULL,
		/* F */	NULL,
		/* G */	"PDEC/QDI[2]",
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	"AC/CMP[1]",
		/* N */	NULL
	      }},
	[26] ={{/* A */	"EIC/EXTINT[12]",
		/* B */	NULL,
		/* C */	"SERCOM2/PAD[0]",
		/* D */	"SERCOM4/PAD[1]",
		/* E */	NULL,
		/* F */	"TCC1/WO[2]",
		/* G */	NULL,
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	NULL
	      }},
	[27] ={{/* A */	"EIC/EXTINT[13]",
		/* B */	NULL,
		/* C */	"SERCOM2/PAD[1]",
		/* D */	"SERCOM4/PAD[0]",
		/* E */	NULL,
		/* F */	"TCC1/WO[3]",
		/* G */	NULL,
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	NULL
	      }},
	[28] ={{/* A */	"EIC/EXTINT[14]",
		/* B */	NULL,
		/* C */	"SERCOM2/PAD[2]",
		/* D */	"SERCOM4/PAD[2]",
		/* E */	NULL,
		/* F */	"TCC1/WO[4]",
		/* G */	NULL,
		/* H */	NULL,
		/* I */	NULL,
		/* J */	"I2S/SCK[1]",
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	NULL
	      }},
	[29] ={{/* A */	"EIC/EXTINT[15]",
		/* B */	NULL,
		/* C */	"SERCOM2/PAD[3]",
		/* D */	"SERCOM4/PAD[3]",
		/* E */	NULL,
		/* F */	"TCC1/WO[5]",
		/* G */	NULL,
		/* H */	NULL,
		/* I */	NULL,
		/* J */	"I2S/MCK[1]",
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	NULL
	      }},
	[30] ={{/* A */	"EIC/EXTINT[14]",
		/* B */	NULL,
		/* C */	NULL,
		/* D */	"SERCOM5/PAD[1]",
		/* E */	"TC0/WO[0]",
		/* F */	"TCC4/WO[0]",
		/* G */	"TCC0/WO[6]",
		/* H */	"SWO",
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	NULL
	      }},
	[31] ={{/* A */	"EIC/EXTINT[15]",
		/* B */	NULL,
		/* C */	NULL,
		/* D */	"SERCOM5/PAD[0]",
		/* E */	"TC0/WO[1]",
		/* F */	"TCC4/WO[1]",
		/* G */	"TCC0/WO[7]",
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	NULL
	      }},
};

#if (PORT_GROUPS >= 3)
static const struct gpio_mux gpioc_mux[32] = {
	[0]  ={{/* A */	"EIC/EXTINT[0]",
		/* B */	"ADC0/AIN[10]",
		/* C */	NULL,
		/* D */	NULL,
		/* E */	NULL,
		/* F */	NULL,
		/* G */	NULL,
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	NULL
	      }},
	[1]  ={{/* A */	"EIC/EXTINT[1]",
		/* B */	"ADC1/AIN[11]",
		/* C */	NULL,
		/* D */	NULL,
		/* E */	NULL,
		/* F */	NULL,
		/* G */	NULL,
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	NULL
	      }},
	[2]  ={{/* A */	"EIC/EXTINT[2]",
		/* B */	"ADC1/AIN[4]",
		/* C */	NULL,
		/* D */	NULL,
		/* E */	NULL,
		/* F */	NULL,
		/* G */	NULL,
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	NULL
	      }},
	[3]  ={{/* A */	"EIC/EXTINT[3]",
		/* B */	"ADC1/AIN[5]",
		/* C */	NULL,
		/* D */	NULL,
		/* E */	NULL,
		/* F */	NULL,
		/* G */	NULL,
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	NULL
	      }},
	[4]  ={{/* A */	"EIC/EXTINT[4]",
		/* B */	NULL,
		/* C */	"SERCOM6/PAD[0]",
		/* D */	NULL,
		/* E */	NULL,
		/* F */	"TCC0/WO[0]",
		/* G */	NULL,
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	NULL,
	      }},
	[5]  ={{/* A */	"EIC/EXTINT[5]",
		/* B */	NULL,
		/* C */	"SERCOM6/PAD[1]",
		/* D */	NULL,
		/* E */	NULL,
		/* F */	NULL,
		/* G */	NULL,
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	NULL,
	      }},
	[6]  ={{/* A */	"EIC/EXTINT[6]",
		/* B */	NULL,
		/* C */	"SERCOM6/PAD[2]",
		/* D */	NULL,
		/* E */	NULL,
		/* F */	NULL,
		/* G */	NULL,
		/* H */	NULL,
		/* I */	"SDHC0/SDCD",
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	NULL
	      }},
	[7]  ={{/* A */	"EIC/EXTINT[7]",
		/* B */	NULL,
		/* C */	"SERCOM6/PAD[3]",
		/* D */	NULL,
		/* E */	NULL,
		/* F */	NULL,
		/* G */	NULL,
		/* H */	NULL,
		/* I */	"SDHC0/SDWP",
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	NULL
	      }},
	[10] ={{/* A */	"EIC/EXTINT[10]",
		/* B */	NULL,
		/* C */	"SERCOM6/PAD[2]",
		/* D */	"SERCOM7/PAD[2]",
		/* E */	NULL,
		/* F */	"TCC0/WO[0]",
		/* G */	"TCC1/WO[4]",
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	NULL
	      }},
	[11] ={{/* A */	"EIC/EXTINT[11]",
		/* B */	NULL,
		/* C */	"SERCOM6/PAD[3]",
		/* D */	"SERCOM7/PAD[3]",
		/* E */	NULL,
		/* F */	"TCC0/WO[1]",
		/* G */	"TCC1/WO[5]",
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	"GMAC/GMDC",
		/* M */	NULL,
		/* N */	NULL
	      }},
	[12] ={{/* A */	"EIC/EXTINT[12]",
		/* B */	NULL,
		/* C */	"SERCOM7/PAD[0]",
		/* D */	"SERCOM6/PAD[1]",
		/* E */	NULL,
		/* F */	"TCC0/WO[2]",
		/* G */	"TCC1/WO[6]",
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	"PCC/DATA[10]",
		/* L */	"GMAC/GMDIO",
		/* M */	NULL,
		/* N */	NULL
	      }},
	[13] ={{/* A */	"EIC/EXTINT[13]",
		/* B */	NULL,
		/* C */	"SERCOM7/PAD[1]",
		/* D */	"SERCOM6/PAD[0]",
		/* E */	NULL,
		/* F */	"TCC0/WO[3]",
		/* G */	"TCC1/WO[7]",
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	"PCC/DATA[11]",
		/* L */	NULL,
		/* M */	NULL,
		/* N */	NULL
	      }},
	[14] ={{/* A */	"EIC/EXTINT[14]",
		/* B */	NULL,
		/* C */	"SERCOM7/PAD[2]",
		/* D */	"SERCOM6/PAD[2]",
		/* E */	NULL,
		/* F */	"TCC0/WO[4]",
		/* G */	"TCC1/WO[0]",
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	"PCC/DATA[21]",
		/* L */	"GMAC/GRX[3]",
		/* M */	NULL,
		/* N */	NULL
	      }},
	[15] ={{/* A */	"EIC/EXTINT[15]",
		/* B */	NULL,
		/* C */	"SERCOM7/PAD[3]",
		/* D */	"SERCOM6/PAD[3]",
		/* E */	NULL,
		/* F */	"TCC0/WO[5]",
		/* G */	"TCC1/WO[1]",
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	"PCC/DATA[13]",
		/* L */	"GMAC/GRX[2]",
		/* M */	NULL,
		/* N */	NULL
	      }},
	[16] ={{/* A */	"EIC/EXTINT[0]",
		/* B */	NULL,
		/* C */	"SERCOM6/PAD[0]",
		/* D */	"SERCOM0/PAD[1]",
		/* E */	NULL,
		/* F */	"TCC0/WO[0]",
		/* G */	"PDEC/QDI[0]",
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	"GMAC/GTX[2]",
		/* M */	NULL,
		/* N */	NULL
	      }},
	[17] ={{/* A */	"EIC/EXTINT[1]",
		/* B */	NULL,
		/* C */	"SERCOM6/PAD[1]",
		/* D */	"SERCOM0/PAD[0]",
		/* E */	NULL,
		/* F */	"TCC0/WO[1]",
		/* G */	"PDEC/QDI[1]",
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	"GMAC/GTX[3]",
		/* M */	NULL,
		/* N */	NULL
	      }},
	[18] ={{/* A */	"EIC/EXTINT[2]",
		/* B */	NULL,
		/* C */	"SERCOM6/PAD[2]",
		/* D */	"SERCOM0/PAD[2]",
		/* E */	NULL,
		/* F */	"TCC0/WO[2]",
		/* G */	"PDEC/QDI[2]",
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	"GMAC/GTXCK",
		/* M */	NULL,
		/* N */	NULL
	      }},
	[19] ={{/* A */	"EIC/EXTINT[3]",
		/* B */	NULL,
		/* C */	"SERCOM6/PAD[3]",
		/* D */	"SERCOM0/PAD[3]",
		/* E */	NULL,
		/* F */	"TCC0/WO[3]",
		/* G */	NULL,
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	"GMAC/GTXER",
		/* M */	NULL,
		/* N */	NULL
	      }},
	[20] ={{/* A */	"EIC/EXTINT[4]",
		/* B */	NULL,
		/* C */	NULL,
		/* D */	NULL,
		/* E */	NULL,
		/* F */	"TCC0/WO[4]",
		/* G */	NULL,
		/* H */	NULL,
		/* I */	"SDHC1/SDCD",
		/* J */	NULL,
		/* K */	NULL,
		/* L */	"GMAC/GRXDV",
		/* M */	NULL,
		/* N */	"CCL/IN[9]"
	      }},
	[21] ={{/* A */	"EIC/EXTINT[5]",
		/* B */	NULL,
		/* C */	NULL,
		/* D */	NULL,
		/* E */	NULL,
		/* F */	"TCC0/WO[5]",
		/* G */	NULL,
		/* H */	NULL,
		/* I */	"SDHC1/SDWP",
		/* J */	NULL,
		/* K */	NULL,
		/* L */	"GMAC/GCOL",
		/* M */	NULL,
		/* N */	"CCL/IN[10]"
	      }},
	[22] ={{/* A */	"EIC/EXTINT[6]",
		/* B */	NULL,
		/* C */	"SERCOM1/PAD[0]",
		/* D */	"SERCOM3/PAD[1]",
		/* E */	NULL,
		/* F */	"TCC0/WO[6]",
		/* G */	NULL,
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	"GMAC/GMDC",
		/* M */	NULL,
		/* N */	NULL
	      }},
	[23] ={{/* A */	"EIC/EXTINT[7]",
		/* B */	NULL,
		/* C */	"SERCOM1/PAD[1]",
		/* D */	"SERCOM3/PAD[0]",
		/* E */	NULL,
		/* F */	"TCC0/WO[7]",
		/* G */	NULL,
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	"GMAC/GMDIO",
		/* M */	NULL,
		/* N */	NULL
	      }},
	[24] ={{/* A */	"EIC/EXTINT[8]",
		/* B */	NULL,
		/* C */	"SERCOM0/PAD[2]",
		/* D */	"SERCOM2/PAD[2]",
		/* E */	NULL,
		/* F */	NULL,
		/* G */	NULL,
		/* H */	"TRACEDATA[3]",
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	NULL
	      }},
	[25] ={{/* A */	"EIC/EXTINT[9]",
		/* B */	NULL,
		/* C */	"SERCOM0/PAD[3]",
		/* D */	"SERCOM2/PAD[3]",
		/* E */	NULL,
		/* F */	NULL,
		/* G */	NULL,
		/* H */	"TRACEDATA[2]",
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	NULL
	      }},
	[26] ={{/* A */	"EIC/EXTINT[10]",
		/* B */	NULL,
		/* C */	NULL,
		/* D */	NULL,
		/* E */	NULL,
		/* F */	NULL,
		/* G */	NULL,
		/* H */	"TRACEDATA[1]",
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	NULL
	      }},
	[27] ={{/* A */	"EIC/EXTINT[11]",
		/* B */	NULL,
		/* C */	"SERCOM1/PAD[0]",
		/* D */	NULL,
		/* E */	NULL,
		/* F */	NULL,
		/* G */	NULL,
		/* H */	"TRACECLK",
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	"SWO",
		/* N */	"CCL/IN[4]"
	      }},
	[28] ={{/* A */	"EIC/EXTINT[12]",
		/* B */	NULL,
		/* C */	"SERCOM1/PAD[1]",
		/* D */	NULL,
		/* E */	NULL,
		/* F */	NULL,
		/* G */	NULL,
		/* H */	"TRACEDATA[[0]",
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	"CCL/IN[5]"
	      }},
	[30] ={{/* A */	"EIC/EXTINT[14]",
		/* B */	"ADC1/AIN[12]",
		/* C */	NULL,
		/* D */	NULL,
		/* E */	NULL,
		/* F */	NULL,
		/* G */	NULL,
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	NULL
	      }},
	[31] ={{/* A */	"EIC/EXTINT[15]",
		/* B */	"ADC1/AIN[13]",
		/* C */	NULL,
		/* D */	NULL,
		/* E */	NULL,
		/* F */	NULL,
		/* G */	NULL,
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	NULL
	      }},
};
#endif /* (PORT_GROUPS >= 3) */

#if (PORT_GROUPS == 4)
static const struct gpio_mux gpiod_mux[32] = {
	[0]  ={{/* A */	"EIC/EXTINT[0]",
		/* B */	"ADC1/AIN[14]",
		/* C */	NULL,
		/* D */	NULL,
		/* E */	NULL,
		/* F */	NULL,
		/* G */	NULL,
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	NULL
	      }},
	[1]  ={{/* A */	"EIC/EXTINT[1]",
		/* B */	"ADC1/AIN[15]",
		/* C */	NULL,
		/* D */	NULL,
		/* E */	NULL,
		/* F */	NULL,
		/* G */	NULL,
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	NULL
	      }},
	[8]  ={{/* A */	"EIC/EXTINT[3]",
		/* B */	NULL,
		/* C */	"SERCOM7/PAD[0]",
		/* D */	"SERCOM6/PAD[1]",
		/* E */	NULL,
		/* F */	"TCC0/WO[1]",
		/* G */	NULL,
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	NULL
	      }},
	[9]  ={{/* A */	"EIC/EXTINT[4]",
		/* B */	NULL,
		/* C */	"SERCOM7/PAD[1]",
		/* D */	"SERCOM6/PAD[0]",
		/* E */	NULL,
		/* F */	"TCC0/WO[2]",
		/* G */	NULL,
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	NULL
	      }},
	[10] ={{/* A */	"EIC/EXTINT[5]",
		/* B */	NULL,
		/* C */	"SERCOM7/PAD[2]",
		/* D */	"SERCOM6/PAD[2]",
		/* E */	NULL,
		/* F */	"TCC0/WO[3]",
		/* G */	NULL,
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	NULL
	      }},
	[11] ={{/* A */	"EIC/EXTINT[6]",
		/* B */	NULL,
		/* C */	"SERCOM7/PAD[3]",
		/* D */	"SERCOM6/PAD[3]",
		/* E */	NULL,
		/* F */	"TCC0/WO[4]",
		/* G */	NULL,
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	NULL
	      }},
	[12] ={{/* A */	"EIC/EXTINT[7]",
		/* B */	NULL,
		/* C */	NULL,
		/* D */	NULL,
		/* E */	NULL,
		/* F */	"TCC0/WO[5]",
		/* G */	NULL,
		/* H */	NULL,
		/* I */	NULL,
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	NULL
	      }},
	[20] ={{/* A */	"EIC/EXTINT[10]",
		/* B */	NULL,
		/* C */	"SERCOM1/PAD[2]",
		/* D */	"SERCOM3/PAD[2]",
		/* E */	NULL,
		/* F */	"TCC1/WO[0]",
		/* G */	NULL,
		/* H */	NULL,
		/* I */	"SDHC1/SDCD",
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	NULL
	      }},
	[21] ={{/* A */	"EIC/EXTINT[11]",
		/* B */	NULL,
		/* C */	"SERCOM1/PAD[3]",
		/* D */	"SERCOM3/PAD[3]",
		/* E */	NULL,
		/* F */	"TCC1/WO[3]",
		/* G */	NULL,
		/* H */	NULL,
		/* I */	"SDHC1/SDWP",
		/* J */	NULL,
		/* K */	NULL,
		/* L */	NULL,
		/* M */	NULL,
		/* N */	NULL
	      }},
};
#endif /* (PORT_GROUPS == 4) */

static void
dump_gpio_port(FILE *os, char group, const PortGroup *pg, uint32_t pins,
    const struct gpio_mux *mux)
{
	const char *pin_name;

	fprintf(os, "GPIO%c:\n", group);

	for (unsigned int pin = 0; pin < 32; pin++) {
		uint32_t mask;
		uint8_t cfg, mux_num;
		bool is_input, is_mux;

		mask = 1u << pin;
		if ((pins & mask) == 0)
			continue;

		pin_name = sam_gpio_pin_name(group, pin);
		if (pin_name == NULL)
			continue;

		fprintf(os, "  P%c%02u %-32s", group, pin, pin_name);
		cfg = pg->PINCFG[pin].reg;
		mux_num = pg->PMUX[pin / 2].reg;
		mux_num >>= (pin & 1u) * 4u;
		mux_num &= 0xfu;
		is_mux = (cfg & PORT_PINCFG_PMUXEN) != 0;

		if (is_mux) {
			if (mux[pin].desc[mux_num] != NULL)
				fprintf(os, "%s", mux[pin].desc[mux_num]);
			else
				fprintf(os, "BAD MUX-%c", 'A' + mux_num);

			if (cfg & PORT_PINCFG_DRVSTR)
				fprintf(os, ", strong");
			is_input = mux_num != 1;
		} else
		if (pg->DIR.reg & (1u << pin)) {
			fprintf(os, "Output");
			if (cfg & PORT_PINCFG_DRVSTR)
				fprintf(os, ", strong");
			fprintf(os, ", driving %u", (pg->OUT.reg & mask) != 0);
			is_input = false;
		} else {
			is_input = true;
			if ((cfg & PORT_PINCFG_INEN) == 0)
				fprintf(os, "Unconfigured");
			else
				fprintf(os, "Input");
		}

		if (is_input && (cfg & PORT_PINCFG_PULLEN) != 0) {
			if (pg->OUT.reg & (1u << pin))
				fprintf(os, ", pullup");
			else
				fprintf(os, ", pulldown");
		}

		if ((cfg & PORT_PINCFG_INEN) == 0) {
			if (is_input && !is_mux)
				fprintf(os, ", input disabled");
		} else {
			if (!is_mux || mux_num != 1) {
				fprintf(os, ", state %u",
				    (pg->IN.reg & mask) != 0);
			}
		}

		fputc('\n', os);
	}
}

static void
gpio_cmd(FILE *os, uint8_t argc, const char * const *argv)
{
#if (PORT_GROUPS >= 3)
	static const uint32_t port_bits[] = PORT_PIN_IMPLEMENTED;
#else
	static const uint32_t port_bits[] = {0xCBFFFFFF, 0xc0c3ffff};
#endif
	static const struct gpio_mux *mux_bits[] = {
		gpioa_mux,
		gpiob_mux,
#if (PORT_GROUPS >= 3)
		gpioc_mux,
#endif
#if (PORT_GROUPS == 4)
		gpiod_mux,
#endif
	};

	(void) argc;
	(void) argv;

	for (unsigned int i = 0; i < PORT_GROUPS; i++) {
		dump_gpio_port(os, 'A' + i, &PORT->Group[i], port_bits[i],
		    mux_bits[i]);
	}

	sam_clock_dump(os);
}
#endif /* (RELEASE_BUILD == 0) */

void
sam_gpio_cmd_init(void)
{

#if (RELEASE_BUILD == 0)
	SHELL_CMD_ADD(gpio);
#endif
}