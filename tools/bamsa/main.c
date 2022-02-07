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

#include <ctype.h>
#include <err.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <sys/types.h>
#include <sys/stat.h>

#include "sam-ba.h"

static const char *pname;
static int verbose;

struct flash_desc_header {
	uint32_t fl_id;
	uint32_t fl_size;
	uint32_t fl_page_size;
	uint32_t fl_nb_planes;
	uint32_t fl_nb_locks;
	uint32_t fl_lock0;
};

static struct flash_desc_header fl_header;
static uint32_t *fl_plane_sizes, *fl_lock_bits;
static int fl_pages_per_lock_bit;

static int
eefc_wait_ready(void *sam, uint32_t state)
{
	uint32_t sr;

	do {
		if (samba_read32(sam, 0x400e0a08, &sr) < 0) {
			perror("read sr");
			return (-1);
		}

		if ((sr & 0x0e) != 0)
			break;

		usleep(1000);
	} while ((sr & 1) != state);

	return ((sr & 0x0e) != 0) ? -1 : 0;
}

static int
eefc_send_command(void *sam, uint32_t cmd)
{
	uint32_t res, bit;
	int x = 0;

	if (cmd == 0x0e || cmd == 0x14)
		bit = 0;
	else
		bit = 1;

	cmd |= 0x5a000000;

	if (samba_write32(sam, 0x400e0a04, cmd) < 0) {
		perror("write cmd");
		return (-1);
	}

	if (samba_read32(sam, 0x400e0a08, &res) < 0) {
		perror("read status");
		return (-1);
	}

	return eefc_wait_ready(sam, bit);
}

static int
read_flash_descriptor(void *sam)
{
	uint32_t x;
	int i;

	if (samba_write32(sam, 0x400e0a00, 0x05000600) < 0) {
		fprintf(stderr, "Error writing fmr\n");
		return (-1);
	}

	if (eefc_send_command(sam, 0x00) < 0) {
		return (-1);
	}

	if (samba_read32(sam, 0x400e0a0c, &fl_header.fl_id) < 0) {
		fprintf(stderr, "Error reading fl_id\n");
		return (-1);
	}

	if (samba_read32(sam, 0x400e0a0c, &fl_header.fl_size) < 0) {
		fprintf(stderr, "Error reading fl_size\n");
		return (-1);
	}

	if (samba_read32(sam, 0x400e0a0c, &fl_header.fl_page_size) < 0) {
		fprintf(stderr, "Error reading fl_page_size\n");
		return (-1);
	}

	if (samba_read32(sam, 0x400e0a0c, &fl_header.fl_nb_planes) < 0) {
		fprintf(stderr, "Error reading fl_nb_planes\n");
		return (-1);
	}

	fl_plane_sizes = malloc(sizeof(uint32_t) * fl_header.fl_nb_planes);
	if (fl_plane_sizes == NULL) {
		perror("malloc planes");
		return (-1);
	}

	for (i = 0; i < fl_header.fl_nb_planes; i++) {
		if (samba_read32(sam, 0x400e0a0c, &fl_plane_sizes[i]) < 0) {
			fprintf(stderr, "Error reading plane %d\n", i);
			return (-1);
		}
	}

	if (samba_read32(sam, 0x400e0a0c, &fl_header.fl_nb_locks) < 0) {
		fprintf(stderr, "Error reading fl_nb_locks\n");
		return (-1);
	}

	if (samba_read32(sam, 0x400e0a0c, &fl_header.fl_lock0) < 0) {
		fprintf(stderr, "Error reading fl_lock0\n");
		return (-1);
	}

	x = fl_header.fl_size - fl_header.fl_lock0;
	x /= fl_header.fl_nb_locks - 1;
	x /= fl_header.fl_page_size;
	fl_pages_per_lock_bit = (int) x;

	fl_lock_bits = malloc(fl_header.fl_nb_locks / 32);
	if (fl_lock_bits == NULL) {
		perror("malloc locks");
		return (-1);
	}

	x = 0;
	for (i = 0; i < (fl_header.fl_nb_locks / 32); i++) {
		if (eefc_send_command(sam, 0xa | (x << 8)) < 0) {
			fprintf(stderr, "Error sending GLB cmd\n");
			return (-1);
		}
		if (samba_read32(sam, 0x400e0a0c, &fl_lock_bits[i]) < 0) {
			fprintf(stderr, "Error reading lockbit %d\n", i);
			return (-1);
		}

		x += fl_pages_per_lock_bit;
	}

	if (verbose) {
		printf("Flash ID: %08x\n", fl_header.fl_id);
		printf("Size: %08x\n", fl_header.fl_size);
		printf("Page Size: %08x\n", fl_header.fl_page_size);
		printf("Planes: %08x\n", fl_header.fl_nb_planes);
		for (i = 0; i < fl_header.fl_nb_planes; i++)
			printf("Plane %d Size: %08x\n", i, fl_plane_sizes[i]);
		printf("Locks: %08x\n", fl_header.fl_nb_locks);
		printf("Lock0: %08x\n", fl_header.fl_lock0);
		printf("Pages per lock bit: %d\n", fl_pages_per_lock_bit);
		for (i = 0; i < fl_header.fl_nb_locks / 32; i++)
			printf("Lockbits%d: %08x\n", i, fl_lock_bits[i]);
	}

	return (0);
}

static void
usage(void)
{

	fprintf(stderr, "usage: %s <tty-dev> [command <args> ...]\n", pname);
	exit(1);
}

static uint32_t
parse_number(const char *str)
{
	uint32_t rv;
	char *p;

	rv = strtoul(str, &p, 0);

	if (p == str || *p != '\0') {
		fprintf(stderr, "Malformed address: '%s'\n", str);
		exit(1);
	}

	return (rv);
}

static void
do_read8(void *sam, const char *addr_str)
{
	uint32_t addr;
	uint8_t value;

	addr = parse_number(addr_str);

	if (samba_read8(sam, addr, &value) < 0) {
		perror("samba_read8");
		return;
	}

	printf("Addr 0x%08" PRIx32 ": 0x%02" PRIx8 "\n", addr, value);
}

static void
do_read16(void *sam, const char *addr_str)
{
	uint32_t addr;
	uint16_t value;

	addr = parse_number(addr_str);

	if (samba_read16(sam, addr, &value) < 0) {
		perror("samba_read16");
		return;
	}

	printf("Addr 0x%08" PRIx32 ": 0x%04" PRIx16 "\n", addr, value);
}

static void
do_read32(void *sam, const char *addr_str)
{
	uint32_t addr;
	uint32_t value;

	addr = parse_number(addr_str);

	if (samba_read32(sam, addr, &value) < 0) {
		perror("samba_read32");
		return;
	}

	printf("Addr 0x%08" PRIx32 ": 0x%08" PRIx32 "\n", addr, value);
}

static void
do_write8(void *sam, const char *addr_str, const char *value_str)
{
	uint32_t addr, value;
	uint8_t value8;

	addr = parse_number(addr_str);
	value = parse_number(value_str);
	if (value > 255) {
		fprintf(stderr, "Too big for 8-bits\n");
		return;
	}

	value8 = (uint8_t) value;

	if (samba_write8(sam, addr, value8) < 0) {
		perror("samba_write8");
		return;
	}

	printf("0x%02" PRIx8 " -> Addr 0x%08" PRIx32 "\n", value8, addr);
}

static void
do_write16(void *sam, const char *addr_str, const char *value_str)
{
	uint32_t addr, value;
	uint16_t value16;

	addr = parse_number(addr_str);
	value = parse_number(value_str);
	if (value > 65535) {
		fprintf(stderr, "Too big for 16-bits\n");
		return;
	}

	value16 = (uint16_t) value;

	if (samba_write16(sam, addr, value16) < 0) {
		perror("samba_write16");
		return;
	}

	printf("0x%04" PRIx16 " -> Addr 0x%08" PRIx32 "\n", value16, addr);
}

static int
do_read_page(void *sam, int page, uint32_t *buffer)
{
	uint32_t addr;
	int i;

	addr = (page * fl_header.fl_page_size) + 0x400000;

	for (i = 0; i < (fl_header.fl_page_size / 4); i++) {
		if (samba_read32(sam, addr, buffer) < 0)
			return (-1);

		buffer++;
		addr += 4;
	}

	return (0);
}

static int
do_write_page(void *sam, int page, int last_page, const uint32_t *page_buff)
{
	const uint32_t *buff;
	uint32_t addr, cmd;
	int i;

	if ((page % 8) == 0) {
		if (page < 128 && (page % fl_pages_per_lock_bit) == 0) {
			/* Ensure lock bits clear */
			cmd = 0x9 | (page << 8);
			if (eefc_send_command(sam, cmd) < 0) {
				return (-1);
			}
		}

		/* Erase block of 8 pages */
		if (eefc_send_command(sam, 0x7 | ((page | 1) << 8)) < 0) {
			return (-1);
		}
	}

	/* Copy contents to page latch */
	addr = 0x400000;
	buff = page_buff;
	for (i = 0; i < (fl_header.fl_page_size / 4); i++) {
		if (samba_write32(sam, addr, *buff) < 0) {
			printf("Failed to write page buffer\n");
			return (-1);
		}
		buff++;
		addr += 4;
	}

	/* Write page */
	cmd = 0x1 | (page << 8);
	if (eefc_send_command(sam, cmd) < 0) {
		printf("Page write command failed (0x%08x)\n", cmd);
		return (-1);
	}

//	usleep(20000);

	if (page < 128 && (last_page || (page % fl_pages_per_lock_bit) ==
	    (fl_pages_per_lock_bit - 1))) {
		/* Lock set */
		cmd = 0x8 | (page << 8);
		if (eefc_send_command(sam, cmd) < 0) {
			printf("Failed to clear lock\n");
			return (-1);
		}
	}

#if 0
	usleep(20000);

	/* Verify */
	addr = 0x400000 + (page * fl_header.fl_page_size);
	buff = page_buff;
	for (i = 0; i < (fl_header.fl_page_size / 4); i++) {
		uint32_t v;

		if (samba_read32(sam, addr, &v) < 0) {
			printf("Verify failure: failed to read addr %08x\n", addr);
			return (-1);
		}

		if (v != *buff) {
			printf("Verify failure: %08x: %08x != %08x\n", addr, v, *buff);
			return (-1);
		}

		buff++;
		addr += 4;
	}
#endif
	return (0);
}

static void
do_write_flash(void *sam, uint32_t addr, size_t len, const uint8_t *buffer)
{
	uint32_t *page_buff;
	uint32_t plen, page;
	int i, p;

	if ((addr % fl_header.fl_page_size) != 0) {
		fprintf(stderr, "Address not multiple of page size\n");
		return;
	}

	if ((page_buff = malloc(fl_header.fl_page_size)) == NULL) {
		perror("page buffer");
		return;
	}

	page = (addr - 0x400000) / fl_header.fl_page_size;

	printf("Writing %ld pages to 0x%08x\n",
	    (len + (fl_header.fl_page_size - 1)) / fl_header.fl_page_size,
	    addr);

	p = 1;

	while (len) {
		printf("\rPage %d", p++);
		fflush(stdout);

		if (len < fl_header.fl_page_size) {
			memset(page_buff, 0xff, fl_header.fl_page_size);
			plen = len;
		} else
			plen = fl_header.fl_page_size;

		memcpy(page_buff, buffer, plen);

		if (do_write_page(sam, page, (len-plen) == 0, page_buff) < 0) {
			fflush(stdout);
			fprintf(stderr, " write failed\n");
			return;
		}

		fflush(stdout);

		buffer += plen;
		len -= plen;
		page += 1;
	}

	printf(" Finished\n");
}

static void
do_sendfile(void *sam, const char *filename, const char *addr_str)
{
	struct stat st;
	uint32_t addr;
	uint8_t *buffer;
	int fd;

	addr = parse_number(addr_str);
#if 0
	if (addr < 0x400000 || addr >= 0x800000) {
		fprintf(stderr, "Expecting Flash address 0x400000 -> "
		    "0x800000\n");
		return;
	}
#endif
	if (stat(filename, &st) < 0) {
		perror(filename);
		return;
	}

	if (!S_ISREG(st.st_mode)) {
		fprintf(stderr, "Not a regular file\n");
		return;
	}

	if (st.st_size == 0) {
		fprintf(stderr, "Empty file?\n");
		return;
	}

	if ((buffer = malloc(st.st_size)) == NULL) {
		perror(filename);
		return;
	}

	if ((fd = open(filename, O_RDONLY)) < 0) {
		perror("open");
		free(buffer);
		return;
	}

	if (read(fd, buffer, st.st_size) != st.st_size) {
		perror("read");
		free(buffer);
		return;
	}

	close(fd);

	/*
	 * Read the Flash descriptor
	 */
	if (read_flash_descriptor(sam) < 0) {
		fprintf(stderr, "Failed to read Flash descriptor\n");
		free(buffer);
		return;
	}

	if (addr >= 0x400000 && addr < (0x400000 + fl_header.fl_size))
		do_write_flash(sam, addr, st.st_size, buffer);
}

static void
do_recvfile(void *sam, const char *filename, const char *addr_str,
    const char *len_str)
{
	uint32_t addr, len, flen;
	uint32_t *buffer, *fbuff;
	int fd;

	addr = parse_number(addr_str);
	flen = len = parse_number(len_str);

	if ((addr % 4) != 0)
		errx(1, "address must be 32-bit aligned");

	if (len == 0 || (len % 4) != 0)
		errx(1, "length must be multiple of 32=bits");

	if ((fbuff = buffer = malloc(len)) == NULL) {
		perror("malloc");
		return;
	}

	while (len) {
		if (samba_read32(sam, addr, buffer) < 0) {
			errx(1, "Failed to read address %08x",
			    (unsigned int) addr);
		}

		addr += 4;
		len -= 4;
		buffer++;
	}

	fd = open(filename, O_CREAT | O_TRUNC | O_WRONLY, 0666);
	if (fd < 0)
		err(1, "Failed to create %s", filename);
	if (write(fd, fbuff, flen) != flen)
		err(1, "Failed to write %s", filename);
	close(fd);
	free(fbuff);
}

static void
do_nvmbits(void *sam, const char *which_bit)
{
	uint32_t v, bit = 0;

	if (which_bit != NULL) {
		bit = parse_number(which_bit);
		if (bit > 2) {
			fprintf(stderr, "Valid bits are 0, 1 or 2\n");
			exit(1);
		}

		if (eefc_send_command(sam, 0xb | (bit << 8)) < 0)
			fprintf(stderr, "Failed to configure NVMBIT%u\n", bit);
	}

	if (eefc_send_command(sam, 0xd) < 0) {
		fprintf(stderr, "Failed to send read NVMBITS command\n");
		exit(1);
	}
		
	if (samba_read32(sam, 0x400e0a0c, &v) < 0) {
		fprintf(stderr, "Failed to read NVMBITS result\n");
		exit(1);
	}

	if (which_bit == NULL)
		printf("NVMBITS: 0x%08x\n", v);
	else
	if ((v & (1u << bit)) == 0) {
		fprintf(stderr, "Failed to set GPNVM bit %u\n", (u_int)bit);
		exit(1);
	}
}

static int
validate_uid(const char *p)
{
	int i;

	if (p[0] != '\0')
		return (-1);

	for (i = 1; i < 16; i++) {
		if (!isalnum((unsigned char)p[i]))
			return (-1);
	}

	return (0);
}

static void
do_serial(void *sam)
{
	uint32_t serial[4];
	char *p;
	int i;

	if (eefc_send_command(sam, 0xe) < 0) {
		fprintf(stderr, "Failed to start signature read\n");
		return;
	}

	for (i = 0; i < 4; i++) {
		if (samba_read32(sam, 0x00400000 + (i * 4), &serial[i]) < 0) {
			perror("read serial");
			return;
		}
	}

	if (eefc_send_command(sam, 0xf) < 0) {
		fprintf(stderr, "Failed to stop signature read\n");
		return;
	}

	printf("Raw Serial: %08x %08x %08x %08x\n",
	    serial[0], serial[1], serial[2], serial[3]);

	p = (char *)(void * )serial;

	if (validate_uid(p) < 0) {
		printf("Whoa dude! They done changed the format!\n");
	} else {
		memmove(p, &p[1], 15);
		p[15] = '\0';
		printf("Serial: %s\n", p);
	}
}

static void
do_write32(void *sam, const char *addr_str, const char *value_str)
{
	uint32_t addr, value;

	addr = parse_number(addr_str);
	value = parse_number(value_str);

	if (samba_write32(sam, addr, value) < 0) {
		perror("samba_write32");
		return;
	}

	printf("0x%08" PRIx32 " -> Addr 0x%08" PRIx32 "\n", value, addr);
}

int
main(int argc, char **argv)
{
	char vstr[64];
	void *sam;

	pname = argv[0];

	if (argc > 1 && strcmp(argv[1], "-v") == 0) {
		verbose = 1;
		argc -= 1;
		argv += 1;
	}

	if (argc < 2)
		usage();

	sam = samba_open(argv[1], vstr);
	if (sam == NULL)
		exit(1);

	if (verbose)
		printf("SAM-BA Version: %s\n", vstr);

	if (argc >= 3) {
		argc -= 2;
		argv += 2;
		if (argc == 1 && strcasecmp(argv[0], "nvmbits") == 0)
			do_nvmbits(sam, NULL);
		else
		if (argc == 2 && strcasecmp(argv[0], "nvmbits") == 0)
			do_nvmbits(sam, argv[1]);
		else
		if (argc == 2 && strcasecmp(argv[0], "read8") == 0)
			do_read8(sam, argv[1]);
		else
		if (argc == 2 && strcasecmp(argv[0], "read16") == 0)
			do_read16(sam, argv[1]);
		else
		if (argc == 2 && strcasecmp(argv[0], "read32") == 0)
			do_read32(sam, argv[1]);
		else
		if (argc == 3 && strcasecmp(argv[0], "write8") == 0)
			do_write8(sam, argv[1], argv[2]);
		else
		if (argc == 3 && strcasecmp(argv[0], "write16") == 0)
			do_write16(sam, argv[1], argv[2]);
		else
		if (argc == 3 && strcasecmp(argv[0], "write32") == 0)
			do_write32(sam, argv[1], argv[2]);
		else
		if (argc == 3 && strcasecmp(argv[0], "sendfile") == 0)
			do_sendfile(sam, argv[1], argv[2]);
		else
		if (argc == 4 && strcasecmp(argv[0], "recvfile") == 0)
			do_recvfile(sam, argv[1], argv[2], argv[3]);
		else
		if (argc == 1 && strcasecmp(argv[0], "serial") == 0)
			do_serial(sam);
		else
			usage();
	}

	samba_close(sam);

	return (0);
}
