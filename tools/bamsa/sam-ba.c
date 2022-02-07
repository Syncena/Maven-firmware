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

#include <fcntl.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>

#include <sys/filio.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/types.h>

#include "sam-ba.h"

#define	SAMBA_CMD_WRITE8	"O"
#define	SAMBA_CMD_READ8		"o"
#define	SAMBA_CMD_WRITE16	"H"
#define	SAMBA_CMD_READ16	"h"
#define	SAMBA_CMD_WRITE32	"W"
#define	SAMBA_CMD_READ32	"w"
#define	SAMBA_CMD_WRITE		"S"
#define	SAMBA_CMD_READ		"R"
#define	SAMBA_CMD_VERSION	'V'
#define	SAMBA_EXECUTE		'#'
#define	SAMBA_EOL		'>'

#define	SAMBA_BAUD		B115200

#define	DB_LOW_LEVEL		(1u << 0)
#define	DB_COMMANDS		(1u << 1)
#define	DB_SYSTEM		(1u << 2)
#define	DB_READWRITE		(1u << 3)

static unsigned int	debug_level = 0;
#define	DBPRINTF(lvl, fmt, ...)		\
	do {				\
		if (lvl & debug_level)  \
		fprintf(stderr, fmt, ##__VA_ARGS__); fflush(stderr); \
		fflush(stderr); \
	} while (0)

struct samba_state {
	int ss_fd;
	uint8_t ss_xbuff[1030];
};

static struct samba_state samba_state;

static int
samba_comms_read_byte(struct samba_state *ss, uint8_t *pv,
    struct timeval *tv)
{
	fd_set fds;
	int toread, sel_rv;

	if (ioctl(ss->ss_fd, FIONREAD, &toread) < 0) {
		perror("FIONREAD");
		return -1;
	}

	if (toread) {
		do {
			if (read(ss->ss_fd, pv, 1) == 1)
				return 1;
		} while (errno == EAGAIN);

		if (errno != EWOULDBLOCK) {
			perror("read");
			return -1;
		}
	}

	memset(&fds, 0, sizeof(fds));
	FD_SET(ss->ss_fd, &fds);

	sel_rv = select(ss->ss_fd + 1, &fds, NULL, NULL, tv);
	DBPRINTF(DB_LOW_LEVEL, "samba_comms_read_byte: select returned %d\n", sel_rv);

	if (sel_rv < 0) {
		perror("select");
		return (-1);
	}
	if (sel_rv == 0)
		return (0);

	DBPRINTF(DB_LOW_LEVEL, "samba_comms_read_byte: reading\n");

	sel_rv = (int) read(ss->ss_fd, pv, 1);

	DBPRINTF(DB_LOW_LEVEL, "samba_comms_read_byte: read returned %d, v %02x\n", sel_rv,
	    *pv);

	return (sel_rv);
}

static void
xm_flushinput(struct samba_state *ss)
{
	struct timeval tv;
	uint8_t c;
	int i;

	tv.tv_sec = 0;
	tv.tv_usec = 10000;

	for (i = 0; i < 10; i++) {
		while (samba_comms_read_byte(ss, &c, &tv) == 1)
			;
	}
}

static int
samba_comms_write_byte(struct samba_state *ss, const uint8_t *pv,
    struct timeval *tv)
{
	fd_set fds;
	int sel_rv;

	memset(&fds, 0, sizeof(fds));
	FD_SET(ss->ss_fd, &fds);

	DBPRINTF(DB_LOW_LEVEL, "samba_comms_write_byte: select fd %d\n", ss->ss_fd);

	sel_rv = select(ss->ss_fd + 1, NULL, &fds, NULL, tv);
	DBPRINTF(DB_LOW_LEVEL, "samba_comms_write_byte: select returned %d\n", sel_rv);

	if (sel_rv < 0) {
		perror("select");
		return (-1);
	}
	if (sel_rv == 0)
		return (0);

	DBPRINTF(DB_LOW_LEVEL, "samba_comms_write_byte: writing %02x\n", *pv);

	sel_rv = (int) write(ss->ss_fd, pv, 1);

	DBPRINTF(DB_LOW_LEVEL, "samba_comms_write_byte: write returned %d\n", sel_rv);

	return (sel_rv);
}

static int
samba_read_buffer(struct samba_state *ss, void *buff, size_t len,
    struct timeval *tv)
{
	uint8_t *p = buff;
	int rv = 0;

	DBPRINTF(DB_LOW_LEVEL, "samba_read_buffer: reading %d bytes\n", (int) len);

	while (len--) {
		switch (samba_comms_read_byte(ss, p++, tv)) {
		default:
			perror("samba_comms_read_byte");
			return (-1);
		case 0:
			DBPRINTF(0x10, "samba_read_buffer: Read timed out\n");
			return (rv);

		case 1:
			rv += 1;
			break;
		}
	}

	DBPRINTF(DB_LOW_LEVEL, "samba_read_buffer: done, returning %d\n", rv);
	return (rv);
}

static int
samba_write_buffer(struct samba_state *ss, const void *buff, size_t len,
    struct timeval *tv)
{
	const uint8_t *p = buff;
	int written = 0;
	int rv;

	DBPRINTF(DB_LOW_LEVEL, "samba_write_buffer: writing %d bytes\n", (int)len);

	while (len--) {
		if ((rv = samba_comms_write_byte(ss, p++, tv) < 0)) {
			perror("samba_comms_write_byte");
			return (-1);
		}

		DBPRINTF(DB_LOW_LEVEL, "samba_write_buffer: samba_comms_write_byte() returned %d\n", rv);

//		if (rv == 0)
//			break;

		written++;
	}

	DBPRINTF(DB_LOW_LEVEL, "samba_write_buffer: done. returning %d\n", written);
	return (written);
}

static int
samba_write_command(struct samba_state *ss, const char *cmd, char *resp)
{
	struct timeval tv;
	const char *resp_start = resp;
	int rv, len, retry;
	char v;

	DBPRINTF(0x10, "samba_write_command: writing command '%s'\n", cmd);

	tv.tv_sec = 4;
	tv.tv_usec = 0;
	rv = samba_write_buffer(ss, cmd, strlen(cmd), &tv);
	if (rv < 0) {
		perror("samba_write_command(buffer)");
		return (rv);
	}

	DBPRINTF(DB_COMMANDS, "samba_write_command: rv %d\n", rv);

	if (rv != strlen(cmd)) {
		fprintf(stderr, "samba_write_command: short write. %d != %d\n",
		    rv, (int) strlen(cmd));
		return (-1);
	}

	DBPRINTF(DB_COMMANDS, "samba_write_command: sending '#'\n");
	v = SAMBA_EXECUTE;
	rv = samba_comms_write_byte(ss, (uint8_t *)&v, &tv);
	if (rv < 0) {
		perror("samba_write_command(execute)");
		return (rv);
	}

	if (rv != 1) {
		fprintf(stderr, "samba_write_command: short write. %d != %d\n",
		    rv, (int) strlen(cmd));
		return (-1);
	}

	tv.tv_sec = 1;
	tv.tv_usec = 100000;

	DBPRINTF(DB_COMMANDS, "samba_write_command: waiting for response\n");

	len = 0;
	retry = 0;
	do {
		v = (char)0x1a;
		rv = samba_comms_read_byte(ss, (uint8_t *)&v, &tv);
		if (rv == 1) {
			len++;
			if (v >= ' ' && v < 0x7fu && resp != NULL)
				*resp++ = v;
		}
		if (rv == 0 && retry++ < 2)
			continue;
	} while (rv == 1 && v != SAMBA_EOL);

	if (rv != 1) {
		DBPRINTF(0x10, "\n\n\n\n**********************\nsamba_write_command: didn't get prompt (rv %d) cmd '%s'\n\n\n\n\n", rv, cmd);
		return (rv);
	}

	if (resp != NULL) {
		resp[-1] = '\0';

		DBPRINTF(0x10, "samba_write_command: response '%s'\n", resp_start);
	}

	DBPRINTF(0x10, "samba_write_command: '%s' done. len %d\n", cmd, len);

//	usleep(10000);

	return (len);
}

void *
samba_open(const char *ttypath, char *version)
{
	struct samba_state *ss = &samba_state;
	struct termios tent;
	struct timeval tv;
	char cmd[64];
	int rv;

	DBPRINTF(DB_SYSTEM, "samba_open: %s\n", ttypath);

	if ((ss->ss_fd = open(ttypath, O_RDWR | O_EXCL /*| O_NONBLOCK*/)) < 0) {
		perror(ttypath);
		return (NULL);
	}

	DBPRINTF(DB_SYSTEM, "samba_open: fd is %d. munging tty settings\n", ss->ss_fd);

	if (tcgetattr(ss->ss_fd, &tent) < 0) {
		perror("tcgetattr");
		close(ss->ss_fd);
		return (NULL);
	}

	cfmakeraw(&tent);

	if (cfsetispeed(&tent, SAMBA_BAUD) < 0 ||
	    cfsetospeed(&tent, SAMBA_BAUD) < 0) {
		perror("cfset[io]speed");
		close(ss->ss_fd);
		return (NULL);
	}

	tent.c_iflag |= IGNBRK;
	tent.c_iflag &= ~(IXON | IXOFF | IXANY | IMAXBEL | IGNPAR);
	tent.c_cflag |= CLOCAL;

	if (tcsetattr(ss->ss_fd, TCSANOW, &tent) < 0) {
		perror("tcsetattr");
		close(ss->ss_fd);
		return (NULL);
	}

#if 0
	int state;
	if (ioctl(ss->ss_fd, TIOCMGET, &state) < 0) {
		perror("TIOCMGET");
		close(ss->ss_fd);
		return (NULL);
	}

	DBPRINTF("samba_open: current TTY state:");
	if (state & TIOCM_LE)
		DBPRINTF(" TIOCM_LE");
	if (state & TIOCM_DTR)
		DBPRINTF(" TIOCM_DTR");
	if (state & TIOCM_RTS)
		DBPRINTF(" TIOCM_RTS");
	if (state & TIOCM_ST)
		DBPRINTF(" TIOCM_ST");
	if (state & TIOCM_SR)
		DBPRINTF(" TIOCM_SR");
	if (state & TIOCM_CTS)
		DBPRINTF(" TIOCM_CTS");
	if (state & TIOCM_CD)
		DBPRINTF(" TIOCM_CD");
	if (state & TIOCM_RI)
		DBPRINTF(" TIOCM_RI");
	if (state & TIOCM_DSR)
		DBPRINTF(" TIOCM_DSR");
	DBPRINTF("\n");

	DBPRINTF("samba_open: setting TTY state\n");

	state |= TIOCM_RTS | TIOCM_CTS | TIOCM_CD | TIOCM_DSR;
	if (ioctl(ss->ss_fd, TIOCMSET, &state) < 0) {
		perror("TIOCMSET");
		close(ss->ss_fd);
		return (NULL);
	}
#else
	if (ioctl(ss->ss_fd, TIOCSDTR, 0) < 0) {
		perror("TIOCSDTR");
		close(ss->ss_fd);
		return (NULL);
	}
#endif

	DBPRINTF(DB_SYSTEM, "samba_open: tty settings done\n");

	xm_flushinput(ss);

	cmd[0] = SAMBA_CMD_VERSION;
	cmd[1] = '\0';

	rv = samba_write_command(ss, cmd, version);
	if (rv <= 0) {
		perror("samba_open: Version cmd");
		return(NULL);
	}

	if (version[0] == '\0') {
		fprintf(stderr, "samba_open: No response to 'V' command\n");
		return (NULL);
	}

	DBPRINTF(DB_SYSTEM, "samba_open: Version '%s'\n", version);

	return (&samba_state);
}

void
samba_close(void *cookie)
{
	struct samba_state *ss = cookie;

	DBPRINTF(DB_SYSTEM, "samba_close: done\n");

	close(ss->ss_fd);
}

int
samba_write8(void *cookie, uint32_t addr, uint8_t v)
{
	struct samba_state *ss = cookie;
	char cmd[64];

	DBPRINTF(DB_READWRITE, "samba_write8: %02x -> %08x\n", v, addr);

	sprintf(cmd, SAMBA_CMD_WRITE8 "%" PRIX32 ",%" PRIX8, addr, v);

	return (samba_write_command(ss, cmd, NULL));
}

int
samba_write16(void *cookie, uint32_t addr, uint16_t v)
{
	struct samba_state *ss = cookie;
	char cmd[64];

	DBPRINTF(DB_READWRITE, "samba_write16: %04x -> %08x\n", v, addr);

	sprintf(cmd, SAMBA_CMD_WRITE16 "%" PRIX32 ",%" PRIX16, addr, v);

	return (samba_write_command(ss, cmd, NULL));
}

int
samba_write32(void *cookie, uint32_t addr, uint32_t v)
{
	struct samba_state *ss = cookie;
	char cmd[64];

	DBPRINTF(DB_READWRITE, "samba_write32: %08x -> %08x\n", v, addr);

	sprintf(cmd, SAMBA_CMD_WRITE32 "%" PRIX32 ",%" PRIX32, addr, v);

	return (samba_write_command(ss, cmd, NULL));
}

int
samba_read8(void *cookie, uint32_t addr, uint8_t *vp)
{
	struct samba_state *ss = cookie;
	char cmd[64], resp[64], *p;
	unsigned long v;
	int rv;

	sprintf(cmd, SAMBA_CMD_READ8 "%" PRIX32 ",", addr);

	rv = samba_write_command(ss, cmd, resp);
	if (rv <= 0)
		return(rv);
	if (resp[0] == '\0')
		return (0);

	v = strtoul(resp, &p, 16);
	if (p == resp || v > 255)
		return (0);

	*vp = (uint8_t)v;

	DBPRINTF(DB_READWRITE, "samba_read8: %08x = %02x\n", addr, *vp);

	return (1);
}

int
samba_read16(void *cookie, uint32_t addr, uint16_t *vp)
{
	struct samba_state *ss = cookie;
	char cmd[64], resp[64], *p;
	unsigned long v;
	int rv;

	sprintf(cmd, SAMBA_CMD_READ16 "%" PRIX32 ",", addr);

	rv = samba_write_command(ss, cmd, resp);
	if (rv <= 0)
		return(rv);
	if (resp[0] == '\0')
		return (0);

	v = strtoul(resp, &p, 16);
	if (p == resp || v > 65535u)
		return (0);

	*vp = (uint16_t)v;

	DBPRINTF(DB_READWRITE, "samba_read16: %08x = %04x\n", addr, *vp);

	return (1);
}

int
samba_read32(void *cookie, uint32_t addr, uint32_t *vp)
{
	struct samba_state *ss = cookie;
	char cmd[64], resp[64], *p;
	unsigned long v;
	int rv;

	sprintf(cmd, SAMBA_CMD_READ32 "%" PRIX32 ",", addr);

	rv = samba_write_command(ss, cmd, resp);
	if (rv <= 0)
		return(rv);
	if (resp[0] == '\0')
		return (0);

	v = strtoul(resp, &p, 16);
	if (p == resp)
		return (0);

	*vp = (uint32_t)v;

	DBPRINTF(DB_READWRITE, "samba_read32: %08x = %08x\n", addr, *vp);

	return (1);
}

static const uint16_t crc16tab[256]= {
	0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
	0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
	0x1231,0x0210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,
	0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
	0x2462,0x3443,0x0420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,
	0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
	0x3653,0x2672,0x1611,0x0630,0x76d7,0x66f6,0x5695,0x46b4,
	0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
	0x48c4,0x58e5,0x6886,0x78a7,0x0840,0x1861,0x2802,0x3823,
	0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
	0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0x0a50,0x3a33,0x2a12,
	0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
	0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0x0c60,0x1c41,
	0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
	0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0x0e70,
	0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
	0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,
	0x1080,0x00a1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
	0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,
	0x02b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
	0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,
	0x34e2,0x24c3,0x14a0,0x0481,0x7466,0x6447,0x5424,0x4405,
	0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,
	0x26d3,0x36f2,0x0691,0x16b0,0x6657,0x7676,0x4615,0x5634,
	0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,
	0x5844,0x4865,0x7806,0x6827,0x18c0,0x08e1,0x3882,0x28a3,
	0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,
	0x4a75,0x5a54,0x6a37,0x7a16,0x0af1,0x1ad0,0x2ab3,0x3a92,
	0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,
	0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0x0cc1,
	0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,
	0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0x0ed1,0x1ef0
};

static uint16_t
crc16_ccitt(const uint8_t *buf, int len)
{
	uint16_t crc = 0;

	while (len--)
		crc = (crc << 8) ^ crc16tab[((crc >> 8) ^ *buf++) & 0xffu];

	return (crc);
}

static int
crc_check(int crc, const uint8_t *buf, int sz)
{

	if (crc) {
		uint16_t crc = crc16_ccitt(buf, sz);
		uint16_t tcrc = (buf[sz] << 8) + buf[sz + 1];
		if (crc == tcrc)
			return 1;
	} else {
		uint8_t cks = 0;
		int i;

		for (i = 0; i < sz; i++)
			cks += buf[i];
		if (cks == buf[sz])
			return 1;
	}

	return 0;
}

#define XM_SOH		0x01
#define XM_STX		0x02
#define XM_EOT		0x04
#define XM_ACK		0x06
#define XM_NAK		0x15
#define XM_CAN		0x18
#define XM_CTRLZ	0x1A
#define MAXRETRANS	25

static int
samba_xmodem_transmit(struct samba_state *ss, const uint8_t *src, int srcsz)
{
	int bufsz, crc = -1;
	uint8_t c, seqno = 1;
	int i, l, retry, len = 0;
	struct timeval tv;

	tv.tv_usec = 0;

	for (;;) {
		for (retry = 16; retry > 0; retry--) {
			tv.tv_sec = 2;

			l = samba_comms_read_byte(ss, &c, &tv);
			if (l < 0) {
				perror("samba_write_memory(START_RD)");
				return (-1);
			}
			if (l == 0)
				continue;

			switch (c) {
			case 'C':
				crc = 1;
				goto start_trans;

			case XM_NAK:
				crc = 0;
				goto start_trans;

			case XM_CAN:
				l = samba_comms_read_byte(ss, &c, &tv);
				if (l < 0) {
					perror("samba_write_memory(START_RD)");
				}
				if (l == 0)
					continue;

				if (c == XM_CAN) {
					(void) samba_comms_write_byte(ss, &c,
					    NULL);
					xm_flushinput(ss);
					return -1; /* canceled by remote */
				}
				break;

			default:
				break;
			}
		}

		c = XM_CAN;
		(void) samba_comms_write_byte(ss, &c, NULL);
		(void) samba_comms_write_byte(ss, &c, NULL);
		(void) samba_comms_write_byte(ss, &c, NULL);
		xm_flushinput(ss);
		return -2; /* no sync */

		for (;;) {
		start_trans:
			ss->ss_xbuff[0] = XM_SOH;
			bufsz = 128;
			ss->ss_xbuff[1] = seqno;
			ss->ss_xbuff[2] = seqno ^ 0xffu;

			l = srcsz - len;

			if (l > bufsz)
			    l = bufsz;

			if (l >= 0) {
				memset(&ss->ss_xbuff[3], 0, bufsz);
				if (l == 0)
					ss->ss_xbuff[3] = XM_CTRLZ;
				else {
					memcpy(&ss->ss_xbuff[3], &src[len], l);
					if (l < bufsz)
						ss->ss_xbuff[3 + l] = XM_CTRLZ;
				}

				if (crc) {
					uint16_t ccrc = crc16_ccitt(
					    &ss->ss_xbuff[3], bufsz);
					ss->ss_xbuff[bufsz + 3] =
					    (ccrc >> 8) & 0xffu;
					ss->ss_xbuff[bufsz + 4] =
					    ccrc & 0xffu;
				} else {
					uint8_t ccks = 0;
					for (i = 3; i < (bufsz + 3); i++)
						ccks += ss->ss_xbuff[i];
					ss->ss_xbuff[bufsz + 3] = ccks;
				}

				for (retry = 0; retry < MAXRETRANS; retry++) {
					for (i = 0;
					    i < (bufsz + 4 + (crc ? 1 : 0));
					    i++) {
						l = samba_comms_write_byte(ss,
						    &ss->ss_xbuff[i], NULL);
						if (l < 0) {
							perror("samba_write_"
							    "memory(PAYLOAD)");
							return (-1);
						}
					}

					tv.tv_sec = 1;
					l = samba_comms_read_byte(ss, &c, &tv);
					if (l < 0) {
						perror("samba_write_memory"
						    "(ACK)");
						return (-1);
					}
					if (l == 0)
						continue;

					switch (c) {
					case XM_ACK:
						++seqno;
						len += bufsz;
						goto start_trans;

					case XM_CAN:
						l = samba_comms_read_byte(ss,
						    &c, &tv);
						if (l < 0) {
							perror("samba_write_"
							    "memory(XM_CAN)");
							return (-1);
						}
						if (l == 0)
							continue;

						if (c == XM_CAN) {
							samba_comms_write_byte(
							    ss, &c, NULL);
							xm_flushinput(ss);
							return -1; /* canceled by remote */
						}
						break;

					case XM_NAK:
					default:
						break;
					}
				}
				c = XM_CAN;
				samba_comms_write_byte(ss, &c, NULL);
				samba_comms_write_byte(ss, &c, NULL);
				samba_comms_write_byte(ss, &c, NULL);
				xm_flushinput(ss);
				return -4; /* xmit error */
			} else {
				for (retry = 0; retry < 10; retry++) {
					c = XM_EOT;
					samba_comms_write_byte(ss, &c, NULL);
					tv.tv_sec = 2;
					l = samba_comms_read_byte(ss, &c, &tv);
					if (l < 0) {
						perror("samba_write_memory(EOT)");
						return (-1);
					}
					if (l == 0)
						continue;

					if (c == XM_ACK)
						break;
				}

				xm_flushinput(ss);

				return ((c == XM_ACK) ? len : (-5));
			}
		}
	}

	return (0);
}

static int
samba_xmodem_receive(struct samba_state *ss, void *dest, int destsz)
{
	uint8_t *p;
	uint8_t startchar = 'C';
	uint8_t c, seqno = 1;
	int bufsz, crc, len;
	int retry, retrans = MAXRETRANS;
	int i, l;
	struct timeval tv;

	tv.tv_usec = 0;
	bufsz = crc = len = 0;
	retrans = MAXRETRANS;

	for (;;) {
		for (retry = 16; bufsz == 0 && retry > 0; retry--) {
			tv.tv_sec = 2;

			if (startchar) {
				l = samba_comms_write_byte(ss, &startchar,NULL);
				if (l < 0) {
					perror("samba_read_memory(START)");
					return (-1);
				}
			}

			l = samba_comms_read_byte(ss, &c, &tv);
			if (l < 0) {
				perror("samba_read_memory(HDR_RD)");
			}
			if (l == 0)
				continue;

			switch (c) {
			case XM_SOH:
				bufsz = 128;
				goto start_recv;

			case XM_STX:
				bufsz = 1024;
				goto start_recv;

			case XM_EOT:
				xm_flushinput(ss);
				c = XM_ACK;
				(void) samba_comms_write_byte(ss, &c, NULL);
				return len; /* normal end */

			case XM_CAN:
				tv.tv_sec = 1;
				l = samba_comms_read_byte(ss, &c, &tv);
				if (l < 0) {
					perror("samba_read_memory(CAN_RD)");
				}
				if (l == 0)
					continue;

				if (c == XM_CAN) {
					xm_flushinput(ss);
					c = XM_ACK;
					(void) samba_comms_write_byte(ss, &c,
					    NULL);
					return -1;
				}
				break;

			default:
				break;
			}
		}

		if (startchar == 'C') {
			startchar = XM_NAK;
			continue;
		}

		xm_flushinput(ss);
		c = XM_CAN;
		(void) samba_comms_write_byte(ss, &c, NULL);
		(void) samba_comms_write_byte(ss, &c, NULL);
		(void) samba_comms_write_byte(ss, &c, NULL);
		return -2;

	start_recv:
		if (startchar == 'C')
			crc = 1;

		startchar = 0;
		tv.tv_sec = 1;
		p = ss->ss_xbuff;
		*p++ = c;

		for (i = 0;  i < (bufsz + (crc ? 1 : 0) + 3); i++) {
			l = samba_comms_read_byte(ss, &c, &tv);
			if (i < 0) {
				perror("samba_read_memory(BUF_RD)");
			}
			if (i != 1)
				goto reject;
			*p++ = c;
		}

		if (ss->ss_xbuff[1] == (ss->ss_xbuff[2] ^ 0xffu) && 
		    (ss->ss_xbuff[1] == seqno || ss->ss_xbuff[1] == (seqno - 1)) &&
			crc_check(crc, &ss->ss_xbuff[3], bufsz)) {

			if (ss->ss_xbuff[1] == seqno)	{
				int count = destsz - len;
				if (count > bufsz)
					count = bufsz;
				if (count > 0) {
					memcpy(&dest[len], &ss->ss_xbuff[3],
					    count);
					len += count;
				}
				seqno++;
				retrans = MAXRETRANS + 1;
			}

			if (--retrans <= 0) {
				xm_flushinput(ss);
				c = XM_CAN;
				(void) samba_comms_write_byte(ss, &c, NULL);
				(void) samba_comms_write_byte(ss, &c, NULL);
				(void) samba_comms_write_byte(ss, &c, NULL);
				return -3;
			}
			c = XM_ACK;
			(void) samba_comms_write_byte(ss, &c, NULL);
			continue;
		}
	reject:
		xm_flushinput(ss);
		c = XM_NAK;
		(void) samba_comms_write_byte(ss, &c, NULL);
	}

	return (0);
}

int
samba_write_memory(void *cookie, uint32_t addr, const void *buffer, int len)
{
	struct samba_state *ss = cookie;
	char cmd[64];
	int rv;

	sprintf(cmd, SAMBA_CMD_WRITE "%" PRIX32 ",", addr);

	rv = samba_write_command(ss, cmd, NULL);
	if (rv <= 0)
		return(rv);

	if (samba_xmodem_transmit(ss, buffer, len) < 0)
		return (-1);

	return (len);
}

int
samba_read_memory(void *cookie, uint32_t addr, void *buffer, int len)
{
	struct samba_state *ss = cookie;
	char cmd[64];
	int rv;

	sprintf(cmd, SAMBA_CMD_READ "%" PRIX32 ",%" PRIX32, addr,
	    (uint32_t)len);

	rv = samba_write_command(ss, cmd, NULL);
	if (rv <= 0)
		return(rv);

	if (samba_xmodem_receive(ss, buffer, len) < 0)
		return (-1);

	return (len);
}
