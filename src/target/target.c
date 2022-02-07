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
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rtos.h"
#include "target.h"
#include "target_comms.h"
#include "tmon.h"
#include "hardware.h"
#include "zone_alloc.h"
#ifdef CONFIG_USE_CONFIGDB
#include "configdb.h"
#endif
#ifdef CONFIG_USE_NETWORK
#include "network.h"
#ifdef NETWORK_OPT_HTTPD
#include "stringio.h"
#include "json_utils.h"
#define	TI_SIO_PRINTF(ti,fmt,...)					\
	do {								\
		if ((ti)->ti_sio != NULL)				\
			stringio_printf((ti)->ti_sio, fmt, ##__VA_ARGS__); \
	} while (0)
#else
#define	TI_SIO_PRINTF(ti,fmt,...)	do { /*nothing*/ } while (0)
#endif
#else	/* CONFIG_USE_NETWORK */
#define	TI_SIO_PRINTF(ti,fmt,...)	do { /*nothing*/ } while (0)
#endif	/* CONFIG_USE_NETWORK */

//#define DEBUG_FLAG_INIT	1
#include "debug.h"

/*
 * Target memory is described by the following data structure
 */
struct target_memory {
	TAILQ_ENTRY(target_memory) tm_qent;
	unsigned int tm_type;
	target_addr_t tm_base;
	target_addr_t tm_top;
	target_flash_t tm_flash;
	const char *tm_desc;
};

struct system_state {
	target_t ss_targets[CONFIG_MAX_TARGETS];
	u_int ss_target_count;
	bool ss_attaching;
	const struct target_arch *ss_arch;
	void *ss_arch_cookie;
	struct target_mem_qhead ss_mem;
	char ss_vendor_name[TARGET_NAME_LEN_MAX];
	char ss_soc_name[TARGET_NAME_LEN_MAX];
	char ss_soc_extra[2][TARGET_NAME_LEN_MAX];
};
static struct system_state system_state;

extern const struct target_arch target_arch;

target_t
target_alloc(target_link_ops_t lo, void *arg)
{
	struct system_state *ss = &system_state;
	target_t t;

	if (ss->ss_target_count == CONFIG_MAX_TARGETS) {
		DBFPRINTF("too many targets\n");
		tmon_log("Warning: This device has too many targets!\n");
		return NULL;
	}

	if ((t = zone_calloc(1, sizeof(*t))) == NULL) {
		DBFPRINTF("out of memory\n");
		tmon_log("Out of memory\n");
		return NULL;
	}

	t->t_link_ops = *lo;
	t->t_link_ops_arg = arg;
	t->t_flags = 0;
	TAILQ_INIT(&t->t_private_mem);

	/* Assign the next available target index */
	t->t_index = ss->ss_target_count++;
	ss->ss_targets[t->t_index] = t;

	return t;
}

void
target_free(target_t t)
{
	struct system_state *ss = &system_state;

	assert(ss->ss_target_count > 0);
	ss->ss_target_count--;
	ss->ss_targets[t->t_index] = NULL;
	if (t->t_flash_verify_buffer != NULL)
		zone_free(t->t_flash_verify_buffer);
	zone_free(t);
}

target_t *
target_probe(unsigned int *ntargets)
{
	struct system_state *ss = &system_state;
	int rv;

	DBFPRINTF("Probing for targets\n");
	tmon_log("Probing for targets.\n");

	if (ss->ss_arch->ta_probe(ss->ss_arch_cookie) == false) {
		DBFPRINTF("No targets found\n");
		tmon_log("No targets found.\n");
		return NULL;
	}

	DBFPRINTF("found possible targets\n");

	memset(ss->ss_targets, 0, sizeof(ss->ss_targets));
	ss->ss_target_count = 0;
	TAILQ_INIT(&ss->ss_mem);
	ss->ss_vendor_name[0] = '\0';
	ss->ss_soc_name[0] = '\0';
	ss->ss_soc_extra[0][0] = '\0';
	ss->ss_soc_extra[1][0] = '\0';

	ss->ss_attaching = true;

	/* Something was found. Go see if we can match it up */
	rv = ss->ss_arch->ta_attach(ss->ss_arch_cookie);

	if (rv == TARGET_ATTACH_NO_MATCH || rv == TARGET_ATTACH_RESCAN) {
		assert(ss->ss_target_count == 0);
		DBFPRINTF("no support for this device\n");
		ss->ss_arch->ta_detach(ss->ss_arch_cookie);
		ss->ss_attaching = false;
		return NULL;
	}

	assert(rv == TARGET_ATTACH_OK);
	assert(ss->ss_target_count != 0);
	assert(ss->ss_targets[0] != NULL);

	*ntargets = ss->ss_target_count;
	ss->ss_attaching = false;

	return ss->ss_targets;
}

void
target_cleanup(void)
{
	struct system_state *ss = &system_state;
	target_mem_t tm;
	target_t t;
	u_int idx;

	DBFPRINTF("running\n");

	/*
	 * Delete shared memory regions.
	 */
	while ((tm = TAILQ_FIRST(&system_state.ss_mem)) != NULL) {
		TAILQ_REMOVE(&system_state.ss_mem, tm, tm_qent);
		DBPRINTF("\tMemory @ %p\n", (void *)tm);
		if (tm->tm_flash != NULL) {
			zone_free(tm->tm_flash->tf_buffer);
			(tm->tm_flash->tf_free)(tm->tm_flash);
		}

		zone_free(tm);
	}

	for (idx = 0; idx < ss->ss_target_count; idx++) {
		t = ss->ss_targets[idx];

		/*
		 * Delete the private memory regions
		 */
		while ((tm = TAILQ_FIRST(&t->t_private_mem)) != NULL) {
			TAILQ_REMOVE(&t->t_private_mem, tm, tm_qent);
			DBPRINTF("\tPrivate Memory @ %p\n", (void *)tm);
			if (tm->tm_flash != NULL) {
				zone_free(tm->tm_flash->tf_buffer);
				(tm->tm_flash->tf_free)(tm->tm_flash);
			}

			zone_free(tm);
		}

		DBFPRINTF("removing target @ %p\n", (void *)t);
		if (t->t_free != NULL) {
			/* Invoke the core's free routine */
			DBPRINTF("\tCore\n");
			(t->t_free)(t);
		}

		/* Free the link interface */
		DBPRINTF("\tLink interface\n");
		(t->t_link_ops.lo_free)(t->t_link_ops_arg);

		DBPRINTF("\tTarget instance.\n");
		ss->ss_targets[idx] = NULL;
		if (t->t_flash_verify_buffer != NULL)
			zone_free(t->t_flash_verify_buffer);
		zone_free(t);
	}

	ss->ss_target_count = 0;

	ss->ss_arch->ta_detach(ss->ss_arch_cookie);

	ss->ss_vendor_name[0] = '\0';
	ss->ss_soc_name[0] = '\0';
	ss->ss_soc_extra[0][0] = '\0';
	ss->ss_soc_extra[1][0] = '\0';
}

int
target_ping(target_t t)
{
	struct system_state *ss = &system_state;

	/*
	 * If we're still in the process of attaching, fake success
	 * in case the target(s) is still in a state of flux.
	 */
	if (ss->ss_attaching)
		return 0;

	return t->t_link_ops.lo_ping(t->t_link_ops_arg);
}

void
target_register_name(const char *vendor, const char *soc,
    const char *extra1, const char *extra2)
{
	struct system_state *ss = &system_state;

	if (vendor != NULL)
		strlcpy(ss->ss_vendor_name, vendor, sizeof(ss->ss_vendor_name));

	if (soc != NULL)
		strlcpy(ss->ss_soc_name, soc, sizeof(ss->ss_soc_name));

	if (extra1 != NULL) {
		strlcpy(ss->ss_soc_extra[0], extra1,
		    sizeof(ss->ss_soc_extra[0]));
	}

	if (extra2 != NULL) {
		strlcpy(ss->ss_soc_extra[1], extra2,
		    sizeof(ss->ss_soc_extra[1]));
	}
}

static target_mem_t
target_add_memory_int(target_t t, unsigned int mem_type, target_addr_t mem_base,
    uint32_t mem_len, const char *desc)
{
	struct system_state *ss = &system_state;
	struct target_mem_qhead *qhead;
	target_mem_t tm, nsm;

	assert(mem_len > 0);

	if ((nsm = zone_calloc(1, sizeof(*nsm))) == NULL)
		return NULL;

	nsm->tm_type = mem_type;
	nsm->tm_base = mem_base;
	nsm->tm_top = (mem_base + mem_len) - 1;
	nsm->tm_desc = desc;

	if (mem_type & TARGET_MEM_EXCLUSIVE)
		qhead = &t->t_private_mem;
	else
		qhead = &ss->ss_mem;

	TAILQ_FOREACH(tm, qhead, tm_qent) {
		if (nsm->tm_base <= tm->tm_base)
			break;
	}

	/* Either list is empty or item belongs at the end */
	if (tm == NULL) {
		TAILQ_INSERT_TAIL(qhead, nsm, tm_qent);
		return nsm;
	}

	assert(nsm->tm_base != tm->tm_base);
	assert(nsm->tm_top < tm->tm_base);

	TAILQ_INSERT_BEFORE(tm, nsm, tm_qent);

	return nsm;
}

int
target_add_memory(target_t t, unsigned int mem_type, target_addr_t mem_base,
    uint32_t mem_len, const char *desc)
{

	assert(mem_len > 0);

	if ((mem_type & TARGET_MEM_TYPE_MASK) == TARGET_MEM_FLASH) {
		mem_type &= ~TARGET_MEM_TYPE_MASK;
		mem_type |= TARGET_MEM_ROM;
	}

	if (target_add_memory_int(t, mem_type, mem_base, mem_len,
	    desc) == NULL) {
		return -1;
	}

	return 0;
}

int
target_alloc_ram(target_t t, unsigned int ram_size, target_addr_t *base)
{
	target_mem_t tm;
	unsigned int size;

	/* Check target's private memory to start with */
	TAILQ_FOREACH(tm, &t->t_private_mem, tm_qent) {
		if ((tm->tm_type & TARGET_MEM_TYPE_MASK) != TARGET_MEM_RAM)
			continue;
		if ((tm->tm_type & TARGET_MEM_NO_APPLET) != 0)
			continue;

		size = (tm->tm_top + 1) - tm->tm_base;
		if (size >= ram_size)
			break;
	}

	if (tm == NULL) {
		/* Check shared memory. */
		TAILQ_FOREACH(tm, &system_state.ss_mem, tm_qent) {
			if ((tm->tm_type & TARGET_MEM_TYPE_MASK) !=
			    TARGET_MEM_RAM) {
				continue;
			}
			if ((tm->tm_type & TARGET_MEM_NO_APPLET) != 0)
				continue;

			size = (tm->tm_top + 1) - tm->tm_base;
			if (size >= ram_size)
				break;
		}
	}

	if (tm != NULL)
		*base = tm->tm_base;

	return tm ? 0 : -1;
}

int
target_degap_memory(target_addr_t start, target_addr_t end)
{
	target_mem_t tm, prev_tm;
	target_addr_t this_len;

	/* GDB will do the right thing here */
	if ((tm = TAILQ_FIRST(&system_state.ss_mem)) == NULL)
		return 0;

	do {
		prev_tm = NULL;
		TAILQ_FOREACH(tm, &system_state.ss_mem, tm_qent) {
			if (tm->tm_base > start)
				break;
			prev_tm = tm;
		}

		/* Gap between prev_tm and tm */
		if (prev_tm != NULL)
			start = prev_tm->tm_top + 1;

		if (tm != NULL) {
			if (tm->tm_base > end)
				this_len = (end - tm->tm_base) + 1;
			else
				this_len = tm->tm_base - start;
		} else {
			this_len = (end - start) + 1;
		}

		if (this_len) {
			if (target_add_memory(NULL,
			    TARGET_MEM_RAM | TARGET_MEM_FILLER,
			    start, this_len, NULL) < 0) {
				return -1;
			}

			start += this_len;
		} else
		if (tm != NULL) {
			start = tm->tm_top + 1;
		}
	} while (start && start < end && tm != NULL);

	return 0;
}

static target_mem_t
target_find_memory(target_t t, target_addr_t addr,
    target_mem_t *nearest_private)
{
	target_mem_t tm;

	/* Check target's private memory to start with */
	TAILQ_FOREACH(tm, &t->t_private_mem, tm_qent) {
		if (addr >= tm->tm_base && addr <= tm->tm_top)
			return tm;
		if (nearest_private != NULL && addr < tm->tm_base) {
			*nearest_private = tm;
			nearest_private = NULL;
		}
	}

	/* Check shared memory. */
	TAILQ_FOREACH(tm, &system_state.ss_mem, tm_qent) {
		if (addr >= tm->tm_base && addr <= tm->tm_top)
			return tm;
	}

	return NULL;
}

int
target_get_memory_type(target_t t, target_addr_t addr, unsigned int *ptype)
{
	target_mem_t tm;

	if ((tm = target_find_memory(t, addr, NULL)) != NULL) {
		if (ptype != NULL)
			*ptype = tm->tm_type & TARGET_MEM_TYPE_MASK;
		return 1;
	}

	return 0;
}

static int
target_xml_memory(target_t t, stringio_t s)
{
	target_mem_t tm, nearest_private;
	target_addr_t addr;
	unsigned int mtype;
	const char *mt;

	stringio_cat(s, "<memory-map>\n");
	addr = 0;

	for (;;) {
		nearest_private = NULL;
		tm = target_find_memory(t, addr, &nearest_private);
		if (tm == NULL)
			tm = nearest_private;
		if (tm == NULL)
			break;

		/*
		 * Filler regions in shared memory may be overlaid by private
		 * memory.
		 */
		if (tm->tm_type & TARGET_MEM_FILLER) {
			if (nearest_private == NULL ||
			    nearest_private->tm_base > tm->tm_top) {
				/*
				 * No private overlays. Skip over filler.
				 */
				addr = tm->tm_top + 1;

				/*
				 * If this is the last bit of filler, we're
				 * done.
				 */
				if (TAILQ_NEXT(tm, tm_qent) == NULL)
					break;

				continue;
			} else {
				/* There's a private memory overlay. */
				tm = nearest_private;
			}
		}

		addr = tm->tm_top + 1;
		mtype = tm->tm_type & TARGET_MEM_TYPE_MASK;

		switch (mtype) {
		case TARGET_MEM_FLASH:
			if (tm->tm_flash != NULL &&
			    (t->t_flags & TARGET_FLAG_FLASH_RDONLY) == 0) {
				mt = "flash";
				break;
			}
			mtype = TARGET_MEM_ROM;
			/*FALLTHROUGH*/

		case TARGET_MEM_ROM:
			mt = "rom";
			break;

		case TARGET_MEM_RAM:
			mt = "ram";
			break;

		default:
			/* Should never see this. */
			continue;
		}

		stringio_printf(s, " <memory type=\"%s\" start=\"0x%" PRIxTADDR
		    "\" length=\"0x%" PRIxTADDR "\"",
		    mt, tm->tm_base, (tm->tm_top - tm->tm_base) + 1);

		if (mtype == TARGET_MEM_FLASH && tm->tm_flash != NULL) {
			stringio_printf(s, ">\n  <property name=\"blocksize\">"
			    "0x%" PRIX32 "</property>\n </memory>\n",
			    tm->tm_flash->tf_max_page_size);
		} else {
			stringio_cat(s, "/>\n");
		}
	}

	stringio_cat(s, "</memory-map>\n");

	return 0;
}

int
target_add_flash(target_t t, target_flash_t tf, target_addr_t addr,
    uint32_t size, const char *desc, unsigned int mem_flags)
{
	target_mem_t tm;

	assert(size > 0);

	if ((tf->tf_buffer = zone_malloc(tf->tf_max_page_size)) == NULL)
		return -1;

	if (tf->tf_max_page_size > t->t_flash_max_page_size)
		t->t_flash_max_page_size = tf->tf_max_page_size;

	mem_flags &= TARGET_MEM_FLAGS_MASK;
	mem_flags |= TARGET_MEM_FLASH;

	/* Add this memory to the target */
	tm = target_add_memory_int(t, mem_flags, addr, size, desc);
	if (tm == NULL) {
		zone_free(tf->tf_buffer);
		return -1;
	}

	tm->tm_flash = tf;
	tf->tf_sector_cache.ce_base_offset = ~((target_addr_t)0);

	return 0;
}

static const struct target_flash_sector *
target_flash_find_sector(target_flash_t tf, target_addr_t *offset)
{
	struct target_flash_sector_cache_entry *ce = &tf->tf_sector_cache;
	target_addr_t curoff;

	curoff = *offset;

	/* Check if the sector descriptor is in the cache */
	if (curoff >= ce->ce_base_offset &&
	    curoff < (ce->ce_base_offset + ce->ce_sector.fs_size)) {
		/* Hit */
		*offset -= ce->ce_base_offset;
	} else {
		/* Otherwise query the SoC driver. */
		if ((tf->tf_sector_desc)(tf, offset, &ce->ce_sector) < 0)
			return NULL;

		/* Calculate the offset of the start of the sector */
		ce->ce_base_offset = curoff - *offset;
	}

	return &ce->ce_sector;
}

static int
target_flash_erase(target_t t, target_addr_t addr, uint32_t len)
{
	target_flash_t tf;
	target_mem_t tm;
	uint32_t this_len;

	DBFPRINTF("region %08" PRIxTADDR " -> %08" PRIxTADDR "\n", addr,
	    (addr + len) - 1);

	if (t->t_flags & TARGET_FLAG_FLASH_RDONLY) {
		DBFPRINTF("TARGET_FLAG_FLASH_RDONLY set\n");
		return -1;
	}

	t->t_flash_verify_enabled = tmon_flash_verify_enabled();
	if (t->t_flash_verify_enabled && t->t_flash_verify_buffer == NULL &&
	    t->t_flash_max_page_size > 0) {
		t->t_flash_verify_buffer = zone_malloc(t->t_flash_max_page_size);
	}

	/*
	 * The address/length may span more than one Flash region. We need
	 * to detect this since the regions may use different Flash
	 * controllers at the SoC level.
	 */
	do {
		if ((tm = target_find_memory(t, addr, NULL)) == NULL)
			return -1;

		/* We can only erase Flash regions with an SoC controller */
		if ((tf = tm->tm_flash) == NULL) {
			DBFPRINTF("not Flash!\n");
			return TARGET_MEM_ROM;
		}

		/* Ensure the page buffer is clear */
		tf->tf_buffer_dirty = 0;
		tf->tf_buffer_page = ~0u;
		tf->tf_buffer_sector.fs_number = ~0u;
		memset(tf->tf_buffer, tf->tf_erase_byte, tf->tf_max_page_size);

		/* Work out the remaining length in this region */
		this_len = (tm->tm_top + 1) - addr;
		if (this_len > len)
			this_len = len;
		len -= this_len;

		DBFPRINTF("erase_start %08" PRIxTADDR ", erase_end "
		    "%08" PRIxTADDR "\n", addr, addr + this_len);

		while (this_len) {
			const struct target_flash_sector *fs;
			target_addr_t offset;

			/* Fetch the descriptor for the current sector */
			offset = (target_addr_t)(addr - tm->tm_base);
			fs = target_flash_find_sector(tf, &offset);
			if (fs == NULL) {
				DBFPRINTF("sector not found for offset "
				    "0x%08" PRIxTADDR "\n", offset);
				return -1;
			}
			DBFPRINTF("size 0x%08" PRIx32 ", number %" PRIu32
			    ", pages %" PRIu32 "\n", fs->fs_size, fs->fs_number,
			    fs->fs_pages);

			/* Update 'addr', correcting any sector alignment */
			addr -= offset;
			addr += fs->fs_size;

			/* Erase the sector */
			if ((tf->tf_erase_sector)(tf, fs) < 0) {
				DBFPRINTF("SoC erase failed\n");
				return -1;
			}

			if (this_len > fs->fs_size)
				this_len -= fs->fs_size;
			else
				this_len = 0;
		}
	} while (len);

	return TARGET_MEM_FLASH;
}

static int
target_flash_do_write(target_t t, target_flash_t tf)
{
	uint32_t page_size;
	int s;

	s = tf->tf_write_page(tf, &tf->tf_buffer_sector, tf->tf_buffer_page);
	if (s == TARGET_FLASH_RESULT_ERROR) {
		DBFPRINTF("SoC write failed\n");
		return s;
	}
	if (s == TARGET_FLASH_RESULT_VERIFIED ||
	    t->t_flash_verify_enabled == false) {
		return s;
	}

	page_size = tf->tf_buffer_sector.fs_size /
	    tf->tf_buffer_sector.fs_pages;

	s = TARGET_MEM_READ(t, tf->tf_page_addr, page_size,
	    t->t_flash_verify_buffer);
	if (s == TARGET_FLASH_RESULT_ERROR) {
		DBFPRINTF("Failed to read to verify buffer.\n");
		return s;
	}

	if (memcmp(tf->tf_buffer, t->t_flash_verify_buffer,
	    (size_t)page_size) != 0) {
		DBFPRINTF("Verification failed for page @ %" PRIxTADDR ".\n",
		    tf->tf_page_addr);
		return TARGET_FLASH_RESULT_ERROR;
	}

	return TARGET_FLASH_RESULT_VERIFIED;
}

static int
target_flash_write(target_t t, target_addr_t addr, uint32_t len,
    const uint8_t *src)
{
	target_flash_t tf;
	target_mem_t tm;
	uint32_t this_len;

	if (t->t_flags & TARGET_FLAG_FLASH_RDONLY) {
		DBFPRINTF("TARGET_FLAG_FLASH_RDONLY set\n");
		return -1;
	}

	/*
	 * The address/length may span more than one Flash region. We need
	 * to detect this since the regions may use different Flash
	 * controllers at the SoC level.
	 */
	do {
		if ((tm = target_find_memory(t, addr, NULL)) == NULL) {
			DBFPRINTF("no matching Flash region for addr "
			    "%08" PRIxTADDR "\n", addr);
			return -1;
		}

		if ((tf = tm->tm_flash) == NULL) {
			DBFPRINTF("no SoC support!\n");
			return TARGET_MEM_ROM;
		}

		DBFPRINTF("addr %08" PRIxTADDR ", len %" PRIu32 "\n", addr,
		    len);

		/* Work out the remaining length in this region */
		this_len = (tm->tm_top + 1) - addr;
		if (this_len > len)
			this_len = len;
		len -= this_len;

		/* This outer loop handles sector-sized chunks */
		while (this_len) {
			const struct target_flash_sector *fs;
			target_addr_t offset, paddr = addr;
			uint32_t page_size, page;

			/* Fetch the descriptor for the current sector */
			offset = (target_addr_t)(addr - tm->tm_base);
			fs = target_flash_find_sector(tf, &offset);
			if (fs == NULL) {
				DBFPRINTF("sector not found for offset "
				    "0x%08" PRIxTADDR "\n", offset);
				return -1;
			}
			DBFPRINTF("Sector size 0x%08" PRIx32 ", number "
			    "%" PRIu32 ", pages %" PRIu32 ", offset "
			    "0x%08" PRIxTADDR "\n", fs->fs_size, fs->fs_number,
			    fs->fs_pages, offset);

			/* Update 'addr', correcting any sector alignment */
			addr -= offset;
			addr += fs->fs_size;

			/* Calculate details of the page(s) of interest */
			page_size = fs->fs_size / fs->fs_pages;
			if (page_size > tf->tf_max_page_size) {
				DBFPRINTF("page size exceeds max page size!\n");
				return -1;
			}
			page = offset / page_size;

			/* Inner loop handles pages */
			while (this_len && page < fs->fs_pages) {
				uint32_t page_len, page_off;

				/*
				 * Check if we need to flush the page
				 * buffer.
				 */
				if (tf->tf_buffer_dirty &&
				    (page != tf->tf_buffer_page ||
				    fs->fs_number != tf->tf_buffer_sector.fs_number)) {
					DBFPRINTF("flushing cache for sector "
					    "%" PRIu32 ", " "page %" PRIu32
					    "\n",
					    tf->tf_buffer_sector.fs_number,
					    tf->tf_buffer_page);

					if (target_flash_do_write(t, tf) ==
					    TARGET_FLASH_RESULT_ERROR) {
						DBFPRINTF("SoC write failed\n");
						return -1;
					}
					memset(tf->tf_buffer, tf->tf_erase_byte,
					    page_size);
				}

				page_len = (this_len < page_size) ?
				    this_len : page_size;
				page_off = offset % page_size;

				/* Are we writing a partial page? */
				if (page_off != 0 &&
				    page_len > (page_size - page_off)) {
					page_len = page_size - page_off;
				}

				/* Copy the source page */
				memcpy(&tf->tf_buffer[page_off], src, page_len);

				/* Record details of the now dirty page. */
				tf->tf_buffer_dirty = 1;
				tf->tf_buffer_page = page;
				tf->tf_page_addr = paddr & ~(page_size - 1);
				tf->tf_buffer_sector = *fs;

				/* Account for the number of bytes consumed */
				src += page_len;
				offset += page_len;
				this_len -= page_len;
				paddr += page_len;
				page += 1;
			}
		}
	} while (len);

	return TARGET_MEM_FLASH;
}

static int
target_flash_done(target_t t)
{
	target_mem_t tm;
	target_flash_t tf;
	int rv = 0;

	if (t->t_flags & TARGET_FLAG_FLASH_RDONLY) {
		DBFPRINTF("TARGET_FLAG_FLASH_RDONLY set\n");
		return -1;
	}

	/*
	 * Find all Flash regions with residual data in their page buffer
	 */
	TAILQ_FOREACH(tm, &t->t_private_mem, tm_qent) {
		if ((tf = tm->tm_flash) == NULL)
			continue;

		if (tf->tf_buffer_dirty) {
			tf->tf_buffer_dirty = 0;

			/* Write out the residual data */
			if (target_flash_do_write(t, tf) < 0)
				rv = -1;
		}
	}

	TAILQ_FOREACH(tm, &system_state.ss_mem, tm_qent) {
		if ((tf = tm->tm_flash) == NULL)
			continue;

		if (tf->tf_buffer_dirty) {
			tf->tf_buffer_dirty = 0;

			/* Write out the residual data */
			if (target_flash_do_write(t, tf) < 0)
				rv = -1;
		}
	}

	/*
	 * Iterate over Flash again to finalise.
	 * Must be done *after* all flushes are complete in case there are
	 * hidden data dependencies in SoC code.
	 */
	TAILQ_FOREACH(tm, &t->t_private_mem, tm_qent) {
		if ((tf = tm->tm_flash) != NULL) {
			/* Let the SoC code know we're done with this region. */
			if ((tf->tf_finish)(tf) == TARGET_FLASH_RESULT_ERROR)
				rv = -1;
		}
	}

	TAILQ_FOREACH(tm, &system_state.ss_mem, tm_qent) {
		if ((tf = tm->tm_flash) != NULL) {
			/* Let the SoC code know we're done with this region. */
			if ((tf->tf_finish)(tf) == TARGET_FLASH_RESULT_ERROR)
				rv = -1;
		}
	}

	return rv;
}

static void
target_print_memory(const target_mem_t tm, FILE *os,
#ifdef NETWORK_OPT_HTTPD
    stringio_t ss
#else
    void *ss
#endif
    )
{
	const char *suffix, *ftype;
	uint32_t size;
	char buff[24];

	size = (uint32_t)(tm->tm_top - tm->tm_base) + 1;
	if (size >= (1024 * 1024)) {
		size /= (1024 * 1024);
		suffix = "MB";
	} else
	if (size >= 1024) {
		size /= 1024;
		suffix = "KB";
	} else {
		suffix = "Bytes";
	}

	snprintf(buff, sizeof(buff), "%08" PRIxTADDR "-%08" PRIxTADDR,
	    tm->tm_base, tm->tm_top);

	switch (tm->tm_type & TARGET_MEM_TYPE_MASK) {
	case TARGET_MEM_FLASH:
		if (tm->tm_type & TARGET_MEM_OPTIONS)
			ftype = "Options";
		else
		if (tm->tm_type & TARGET_MEM_EEPROM)
			ftype = "EEPROM";
		else
			ftype = "Flash";
		break;

	case TARGET_MEM_ROM:
		ftype = "ROM";
		break;

	case TARGET_MEM_RAM:
		ftype = "RAM";
		break;

	default:
		ftype = "???";
		break;
	}

	if (os != NULL) {
		fprintf(os, "%7s: %s (%" PRIu32 " %s", ftype, buff, size,
		    suffix);

		if (tm->tm_type & TARGET_MEM_BEST_GUESS)
			fprintf(os, " [estimated]");

		if (tm->tm_desc != NULL)
			fprintf(os, ", %s)\n", tm->tm_desc);
		else
			fputs(")\n", os);
	}

#ifdef NETWORK_OPT_HTTPD
	if (ss != NULL) {
		stringio_printf(ss, JSON_STRING("type", ftype));
		stringio_printf(ss, JSON_SEPARATOR);

		stringio_printf(ss, JSON_STRING("range", buff));
		stringio_printf(ss, JSON_SEPARATOR);

		snprintf(buff, sizeof(buff), "%" PRIu32 " %s%s", size, suffix,
		    (tm->tm_type & TARGET_MEM_BEST_GUESS) ? " [estimated]" :"");

		stringio_printf(ss, JSON_STRING("size", buff));
		stringio_printf(ss, JSON_SEPARATOR);

		stringio_printf(ss, JSON_STRING("desc",
		    (tm->tm_desc != NULL) ? tm->tm_desc : ""));
	}
#else
	(void) ss;
#endif
}

static void
target_info(target_t t, const struct target_info *ti)
{
	target_mem_t tm;

	if (ti->ti_os != NULL && !TAILQ_EMPTY(&t->t_private_mem))
		fputs("Private Memories:\n", ti->ti_os);

	TI_SIO_PRINTF(ti, JSON_SEPARATOR);
	TI_SIO_PRINTF(ti, JSON_ARRAY_NAMED("memory"));

	TAILQ_FOREACH(tm, &t->t_private_mem, tm_qent) {
		TI_SIO_PRINTF(ti, JSON_OBJECT_START);

#ifdef NETWORK_OPT_HTTPD
		target_print_memory(tm, ti->ti_os, ti->ti_sio);
#else
		target_print_memory(tm, ti->ti_os, NULL);
#endif

		TI_SIO_PRINTF(ti, JSON_OBJECT_END);

		if (TAILQ_NEXT(tm, tm_qent) != NULL)
			TI_SIO_PRINTF(ti, JSON_SEPARATOR);
	}

	TI_SIO_PRINTF(ti, JSON_ARRAY_END);
}

void
target_info_all(FILE *os)
{
	struct system_state *ss = &system_state;
	struct target_info ti;
	target_mem_t tm;
	target_t t;
	u_int i;

	ti.ti_os = os;
#ifdef NETWORK_OPT_HTTPD
	ti.ti_sio = NULL;
#endif

	for (i = 0; i < ss->ss_target_count; i++) {
		if ((t = ss->ss_targets[i]) != NULL)
			target_ctl(t, TARGET_CTL_TARGET_INFO, &ti);
	}

	if (TAILQ_EMPTY(&ss->ss_mem))
		return;

	if (ss->ss_target_count != 1)
		fputs("Shared ", os);
	fputs("Memories:\n", os);

	TAILQ_FOREACH(tm, &ss->ss_mem, tm_qent) {
		if ((tm->tm_type & TARGET_MEM_FILLER) == 0)
			target_print_memory(tm, os, NULL);
	}
}

#ifdef NETWORK_OPT_HTTPD
void
target_json_all(stringio_t sio)
{
	struct system_state *ss = &system_state;
	struct target_info ti;
	target_mem_t tm;
	bool separator;
	target_t t;
	u_int i;

	stringio_printf(sio, JSON_ARRAY_NAMED("cores"));

	ti.ti_os = NULL;
	ti.ti_sio = sio;

	for (i = 0; i < ss->ss_target_count; i++) {
		if ((t = ss->ss_targets[i]) == NULL)
			continue;

		stringio_printf(sio, JSON_OBJECT_START);

		target_ctl(t, TARGET_CTL_TARGET_INFO, &ti);

		stringio_printf(sio, JSON_OBJECT_END);

		if ((i + 1) < ss->ss_target_count)
			stringio_printf(sio, JSON_SEPARATOR);
	}

	stringio_printf(sio, JSON_ARRAY_END);
	stringio_printf(sio, JSON_SEPARATOR);
	stringio_printf(sio, JSON_ARRAY_NAMED("memory"));

	separator = false;
	TAILQ_FOREACH(tm, &ss->ss_mem, tm_qent) {
		if (tm->tm_type & TARGET_MEM_FILLER)
			continue;

		if (separator)
			stringio_printf(sio, JSON_SEPARATOR);
		else
			separator = true;

		stringio_printf(sio, JSON_OBJECT_START);

		target_print_memory(tm, NULL, sio);

		stringio_printf(sio, JSON_OBJECT_END);
	}

	stringio_printf(sio, JSON_ARRAY_END);
}

void
target_debug_update(stringio_t ss)
{

	system_state.ss_arch->ta_debug_update(system_state.ss_arch_cookie, ss);
}

int
target_debug_update_settings(int nkeys,
    const struct network_http_furl_key_value *kv)
{

	return system_state.ss_arch->ta_debug_update_settings(
	    system_state.ss_arch_cookie, nkeys, kv);
}
#endif /* NETWORK_OPT_HTTPD */

int
target_ctl(target_t t, uint32_t cmd, void *arg)
{ 
	struct target_mem_readwrite *mr;
	int rv = 0;

	if (t->t_ctl != NULL) {
		rv = (t->t_ctl)(t, &cmd, arg);
		if (rv < 0)
			return rv;
	}

	switch (cmd) {
	case TARGET_CTL_NOP:
		break;

	case TARGET_CTL_GET_TARGET_NAME:
		{
			struct system_state *ss = &system_state;
			struct target_ctl_get_target_name *tn = arg;

			if (ss->ss_vendor_name[0] != '\0')
				tn->tn_vendor_name = ss->ss_vendor_name;
			else
				tn->tn_vendor_name = NULL;

			if (ss->ss_soc_name[0] != '\0')
				tn->tn_soc_name = ss->ss_soc_name;
			else
				tn->tn_soc_name = NULL;

			for (unsigned int i = 0; i < 2; i++) {
				if (ss->ss_soc_extra[i][0] != '\0') {
					tn->tn_soc_extra[i] =
					    ss->ss_soc_extra[i];
				} else {
					tn->tn_soc_extra[i] = NULL;
				}
			}
		}
		break;

	case TARGET_CTL_MEM_READWRITE:
		/*
		 * We assume the target-specific t->t_ctl() handler
		 * has intercepted this CTL to sync the cache, if
		 * present.
		 */
		mr = arg;

		if (mr->mr_write) {
			rv = TARGET_MEM_WRITE(t, mr->mr_target_addr,
			    mr->mr_length, mr->mr_src);
		} else {
			rv = TARGET_MEM_READ(t, mr->mr_target_addr,
			    mr->mr_length, mr->mr_dest);
		}
		if (rv < 0)
			t->t_link_error = TARGET_LINK_ERROR(t);
		else
		if (t->t_ctl != NULL) {
			uint32_t post_cmd = TARGET_CTL_MEM_POST_READWRITE;
			rv = (t->t_ctl)(t, &post_cmd, arg);
			if (rv < 0)
				t->t_link_error = TARGET_LINK_ERROR(t);
		}
		break;

	case TARGET_CTL_GET_XML_MEMORY:
		rv = target_xml_memory(t, (stringio_t)arg);
		break;

	case TARGET_CTL_FLASH_ERASE:
		mr = arg;
		rv = target_flash_erase(t, mr->mr_target_addr, mr->mr_length);
		if (rv < 0)
			t->t_link_error = TARGET_LINK_ERROR(t);
		break;

	case TARGET_CTL_FLASH_WRITE:
		mr = arg;
		rv = target_flash_write(t, mr->mr_target_addr, mr->mr_length,
		    mr->mr_src);
		if (rv < 0)
			t->t_link_error = TARGET_LINK_ERROR(t);
		break;

	case TARGET_CTL_FLASH_DONE:
		/*
		 * We assume the target-specific t->t_ctl() handler has
		 * intercepted this CTL to invalidate the cache, if present.
		 */
		rv = target_flash_done(t);
		t->t_link_error = TARGET_LINK_ERROR(t);
		if (t->t_link_error != 0)
			rv = -1;
		break;

	case TARGET_CTL_TARGET_INFO:
		target_info(t, arg);
		t->t_link_error = TARGET_LINK_ERROR(t);
		rv = 0;
		break;

	case TARGET_CTL_GET_LINK_ERROR:
		*(uint32_t *)arg = t->t_link_error;
		break;

	default:
		rv = -1;
		break;
	}

	return rv;
}

void
target_init(void)
{
	struct system_state *ss = &system_state;
	const struct target_arch *ta = &target_arch;

	memset(ss, 0, sizeof(*ss));
	TAILQ_INIT(&ss->ss_mem);

	/* Initialise the external interface. */
	hw_init();

	/* Target comms must be primed before attaching the target back-end. */
	target_comms_init();

	/* Attach target-specific back end. */
	ss->ss_arch = ta;
	ss->ss_arch_cookie = ta->ta_init();

	/* Target comms can now be started. */
	target_comms_start();
}

void
target_load_defaults(void)
{
	const struct target_arch *ta = &target_arch;

	if (ta->ta_load_defaults != NULL)
		ta->ta_load_defaults();
}
