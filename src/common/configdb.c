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
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "rtos.h"
#include "configdb.h"
#include "crypto_crc32.h"
#include "hexdump.h"
#include "shell.h"
#include "zone_alloc.h"

#define CF_TYPE_UINT8		0x00u
#define CF_TYPE_UINT16		0x01u
#define CF_TYPE_UINT32		0x02u
#define CF_TYPE_STRING		0x03u
#define CF_TYPE_DATA		0x04u
#define CF_TYPE_FLOAT		0x05u
#define CF_TYPE_DOUBLE		0x06u
#define CF_TYPE_FIX16		0x07u
#define CF_TYPE_EOT		0x0fu

typedef uint8_t cf_type_t;

/* DB entry header. Fits in 4 bytes. */
struct cf_entry {
	cf_id_t ce_id;
	cf_type_t ce_type;
	cf_len_t ce_len;
	uint8_t ce_data[];
};
#define	CF_TYPE_VALID(t)	((t) <= CF_TYPE_FIX16)
#define	CF_LEN_PAD(l)		(((size_t)(l) + (sizeof(void *) - 1)) & \
				 ~(sizeof(void *) - 1))

struct configdb {
	uint16_t cd_magic;
	uint16_t cd_version;
	uint32_t cd_generations;
	uint32_t cd_size;
	uint32_t cd_crc;
};
#define	CF_MAGIC	0xcafeu
#define	CF_VERSION	0x0001u
#define	CF_FIRST(cd)	((struct cf_entry *)(void *)(uintptr_t)((cd) + 1))
#define	CF_NEXT(ce)	((struct cf_entry *)(void *)(uintptr_t) \
			 &(ce)->ce_data[CF_LEN_PAD((ce)->ce_len)])

struct configdb_state {
	const struct configdb_driver *cs_driver;
	rtos_mutex_t cs_mutex;
	size_t cs_size;
	bool cs_locked;
	bool cs_dirty;
	struct configdb *cs_db;
};
static struct configdb_state configdb_state;

#if (RELEASE_BUILD == 0)
SHELL_CMD_DECL(configdb, configdb_cmd, "Show/manipulate the config DB");
#endif

static size_t
cf_read(struct configdb_state *cs, void *buff)
{
	const struct configdb_driver *cd = cs->cs_driver;

	if (cd == NULL || cd->cd_read == NULL) {
		memset(buff, 0, cs->cs_size);
		return cs->cs_size;
	}

	return cd->cd_read(cd->cd_cookie, buff);
}

#ifndef CONFIGDB_READ_ONLY
static int
cf_write(struct configdb_state *cs, const void *buff, size_t len)
{
	const struct configdb_driver *cd = cs->cs_driver;

	assert(len <= cs->cs_size);

	if (cd == NULL || cd->cd_write == NULL)
		return len;

	return cd->cd_write(cd->cd_cookie, buff, len);
}
#endif

static int
cf_fsck(struct configdb_state *cs)
{
	struct configdb *cd = cs->cs_db;
	struct cf_entry *ce;
	size_t size;
	uint32_t crc, calc_crc;

	if (cd == NULL)
		return -1;

	/*
	 * Validate the DB header.
	 */
	if (cd->cd_magic != CF_MAGIC || cd->cd_version != CF_VERSION ||
	    cd->cd_size < sizeof(struct cf_entry) ||
	    (cd->cd_size + sizeof(struct configdb)) > cs->cs_size) {
		return -1;
	}

	/*
	 * Header checks out. Go run a CRC over the whole DB.
	 */
	size = cd->cd_size + sizeof(struct configdb);
	crc = cd->cd_crc;
	cd->cd_crc = 0;
	calc_crc = crypto_crc32(cd, size);
	cd->cd_crc = crc;

	if (crc != calc_crc)
		return -1;

	/*
	 * CRC checks out. Validate each entry.
	 */
	size = sizeof(*ce);
	for (ce = CF_FIRST(cd); ce->ce_type != CF_TYPE_EOT; ) {
		size_t sz;

		switch (ce->ce_type) {
		case CF_TYPE_UINT8:
			sz = sizeof(uint8_t);
			break;
		case CF_TYPE_UINT16:
			sz = sizeof(uint16_t);
			break;
		case CF_TYPE_UINT32:
			sz = sizeof(uint32_t);
			break;
		case CF_TYPE_FLOAT:
			sz = sizeof(float);
			break;
		case CF_TYPE_DOUBLE:
			sz = sizeof(double);
			break;
		case CF_TYPE_STRING:
		case CF_TYPE_DATA:
			sz = (size_t) ce->ce_len;
			break;
		default:
			return -1;
		}

		/* Verify the size is appropriate for the type. */
		if (ce->ce_len != sz)
			return -1;

		/* Verify the size does not overflow the buffer. */
		size += CF_LEN_PAD(sz) + sizeof(*ce);
		if (size > cd->cd_size)
			return -1;

		ce = CF_NEXT(ce);
	}

	/* The EOT marker must have a length of zero. */
	if (ce->ce_len != 0)
		return -1;

	/* Finally, ensure the header size matches the actual size. */
	if (size != cd->cd_size)
		return -1;

	/* DB checks out. */
	return 0;
}

int
configdb_attach_driver(const struct configdb_driver *cd, size_t size)
{
	struct configdb_state *cs = &configdb_state;

	/* Sanity check the parameters. */
	assert(cs->cs_driver == NULL);
	assert(size > sizeof(struct configdb));

	/* XXX: This might be an unreasonable restriction. */
	assert(size <= 2048u);

	cs->cs_driver = cd;
	cs->cs_size = size;

	return 0;
}

static void
configdb_new(struct configdb_state *cs)
{
	struct configdb *cd = cs->cs_db;

	/* Construct a new DB. */
	cd->cd_magic = CF_MAGIC;
	cd->cd_version = CF_VERSION;
	cd->cd_generations = 0;
	cd->cd_size = sizeof(struct cf_entry);
	cd->cd_crc = 0;
	CF_FIRST(cd)->ce_id = (cf_id_t)(~0u);
	CF_FIRST(cd)->ce_type = CF_TYPE_EOT;
	CF_FIRST(cd)->ce_len = 0;
	cs->cs_dirty = true;
}

int
configdb_init(void)
{
	struct configdb_state *cs = &configdb_state;
	int rv = 1;

	/* Ensure this is called only once. */
	assert(cs->cs_db == NULL);

	/* Persistence is optional. Pick some defaults. */
	if (cs->cs_size == 0) {
		assert(cs->cs_driver == NULL);
		cs->cs_size = 256u;
	}

	/* Grab a buffer for the in-memory copy of the DB. */
	cs->cs_db = zone_malloc(cs->cs_size);
	assert(cs->cs_db != NULL);

	/* Locking. */
	cs->cs_mutex = rtos_mutex_create();
	assert(cs->cs_mutex != NULL);
	rtos_mutex_acquire(cs->cs_mutex);
	cs->cs_locked = true;

	/*
	 * Read the persistent copy of the DB, if it exists.
	 * If the read fails, proceed as if the driver never attached.
	 */
	if (cf_read(cs, cs->cs_db) != cs->cs_size)
		cs->cs_driver = NULL;

	/* Validate the DB. */
	if (cf_fsck(cs) < 0) {
		/* Validation failed. Construct a new DB. */
		configdb_new(cs);
		rv = 0;
	}

	/* Unlock. */
	cs->cs_locked = false;
	rtos_mutex_release(cs->cs_mutex);

#if (RELEASE_BUILD == 0)
	SHELL_CMD_ADD(configdb);
#endif

	return rv;
}

static void
cf_lock(struct configdb_state *cs)
{

	rtos_mutex_acquire(cs->cs_mutex);

	assert(cs->cs_locked == false);

	cs->cs_locked = true;
}

static void
cf_unlock(struct configdb_state *cs)
{

	assert(cs->cs_locked == true);

	cs->cs_locked = false;
	rtos_mutex_release(cs->cs_mutex);
}

static struct cf_entry *
cf_lookup_entry(struct configdb_state *cs, cf_type_t type, cf_id_t id)
{
	struct configdb *cd = cs->cs_db;
	struct cf_entry *ce;

	/* DB must be locked. */
	assert(cs->cs_locked == true);

	/* Catch bogus lookups. */
	assert(CF_TYPE_VALID(type));

#ifndef NDEBUG
	size_t size = sizeof(struct cf_entry);
#endif

	/* Scan the DB. */
	for (ce = CF_FIRST(cd); ce->ce_type != CF_TYPE_EOT; ce = CF_NEXT(ce)) {
		/* Make sure we don't run off the end... */
		assert((size + CF_LEN_PAD(ce->ce_len)) <= cd->cd_size);

		/* Found it? */
		if (ce->ce_type == type && ce->ce_id == id)
			return ce;

#ifndef NDEBUG
		size += CF_LEN_PAD(ce->ce_len);
#endif
	}

	return NULL;
}

static cf_len_t
cf_lookup(cf_id_t id, void *pv, cf_type_t type, size_t len)
{
	struct configdb_state *cs = &configdb_state;
	struct cf_entry *ce;

	cf_lock(cs);

	if ((ce = cf_lookup_entry(cs, type, id)) == NULL) {
		cf_unlock(cs);
		return 0;
	}

	/*
	 * 'len' has two uses...
	 *   = 0: Don't care - just copy the entire item.
	 *   > 0: Copy 'len' bytes, so long as it doesn't
	 *        exceed the item length.
	 */
	if (pv != NULL) {
		if (len == 0 || len > (size_t)ce->ce_len)
			len = (size_t)ce->ce_len;
		memcpy(pv, &ce->ce_data[0], len);
	}

	cf_unlock(cs);

	/* Always return the item's actual length. */
	return ce->ce_len;
}

static void
cf_delete_entry(struct configdb *cd, struct cf_entry *ce)
{
	struct cf_entry *nce;
	uintptr_t tbl_end;
	size_t len;

	/* Point to the first unused buffer location */
	tbl_end = (uintptr_t)(cd + 1) + cd->cd_size;

	/* Move the trailing entries into the vacated space. */
	nce = CF_NEXT(ce);
	memmove(ce, nce, tbl_end - (uintptr_t)nce);

	/* How many bytes did we delete? */
	len = (uintptr_t)nce - (uintptr_t)ce;

	/* Keep the stats up to date. */
	cd->cd_size -= len;
}

static cf_len_t
cf_insert(cf_id_t id, cf_type_t type, size_t len, const void *pv)
{
	struct configdb_state *cs = &configdb_state;
	struct configdb *cd = cs->cs_db;
	struct cf_entry *ce;
	size_t bytes_left, new_len;

	/* Be careful what we insert... */
	assert(CF_TYPE_VALID(type));
	assert(len < cs->cs_size);
	assert(len > 0);
	assert(len <= 255);
	assert(pv != NULL);

	/* Work out how much space is required for the new entry. */
	new_len = CF_LEN_PAD(len) + sizeof(*ce);

	cf_lock(cs);

	/*
	 * cs_size holds the total size of the allocated DB buffer.
	 * This is the same value passed in by the driver at attach time.
	 * Subtract the current DB size.
	 */
	assert(cs->cs_size > cd->cd_size);
	bytes_left = cs->cs_size - cd->cd_size;

	/*
	 * Now subtract the size of the DB header, yielding the amount
	 * of free space in the DB buffer.
	 */
	assert(bytes_left >= sizeof(struct configdb));
	bytes_left -= sizeof(struct configdb);

	/* Check if an entry of this type & id already exists. */
	if ((ce = cf_lookup_entry(cs, type, id)) != NULL) {
		/*
		 * It already exists.
		 */
		size_t old_len = (size_t)CF_LEN_PAD(ce->ce_len) + sizeof(*ce);

		if (new_len < old_len || (new_len > old_len &&
		    (new_len - old_len) <= bytes_left)) {
			/*
			 * New item occupies less space then old, or it
			 * occupies more AND there's space for it in the DB.
			 * Delete the old.
			 */
			cf_delete_entry(cd, ce);

			/* Mark the DB as modified. */
			cs->cs_dirty = true;

			/* Account for the new space. */
			bytes_left += old_len;

			/* Old item has gone. */
			ce = NULL;
		} else
		if (new_len > old_len) {
			/*
			 * New item occupies more space than the old, but
			 * even if we delete the old, there's insufficient
			 * space for it in the DB.
			 */
			len = 0;
		} else {
			/* Both items are the same size. */
			assert(new_len == old_len);
		}
	}

	if (len > 0) {
		/*
		 * Item didn't exist, or was deleted.
		 */
		if (ce == NULL) {
			/*
			 * Create some space at the start of the DB.
			 */
			uintptr_t st = (uintptr_t)CF_FIRST(cd);

			if (new_len <= bytes_left) {
				/* Shuffle up existing items. */
				memmove((void *)(st + new_len),
				    (const void *)st, cd->cd_size);
				ce = (struct cf_entry *)st;

				/* Record the item's header. */
				ce->ce_id = id;
				ce->ce_type = type;

				/* Include in stats. */
				cd->cd_size += new_len;
			} else {
				/* Exceeded the available size. */
				len = 0;
			}
		}

		/*
		 * We are overwriting an existing entry of the same size,
		 * or we're copying a new item into newly-opened up space.
		 */
		if (len > 0) {
			memcpy(&ce->ce_data[0], pv, len);

			/* Complete the header. */
			ce->ce_len = (cf_len_t)len;

			/* The DB has been modified. */
			cs->cs_dirty = true;
		}
	}

	cf_unlock(cs);

	return (cf_len_t)len;
}

static void
cf_delete(cf_id_t id, cf_type_t type)
{
	struct configdb_state *cs = &configdb_state;
	struct cf_entry *ce;

	/* Be careful what we delete... */
	assert(CF_TYPE_VALID(type));

	cf_lock(cs);

	/* Check if an entry of this type & id exists. */
	if ((ce = cf_lookup_entry(cs, type, id)) != NULL) {
		cf_delete_entry(cs->cs_db, ce);
		cs->cs_dirty = true;
	}

	cf_unlock(cs);
}

#ifndef CONFIGDB_READ_ONLY
void
configdb_erase_all(void)
{
	struct configdb_state *cs = &configdb_state;
	uint32_t oldgen;

	cf_lock(cs);

	/* Preserve the generations counter. */
	oldgen = cs->cs_db->cd_generations;

	/* Wipe the DB clean. */
	configdb_new(cs);

	/* Restore the generations counter. */
	cs->cs_db->cd_generations = oldgen;

	cf_unlock(cs);
}

int
configdb_persist(void)
{
	struct configdb_state *cs = &configdb_state;
	struct configdb *cd = cs->cs_db;
	size_t sz;
	int rv = 0;

	cf_lock(cs);

	if (cs->cs_dirty) {
		/* DB has been modified. Calculate the CRC and write it out. */
		sz = cd->cd_size + sizeof(struct configdb);
		cd->cd_crc = 0;
		cd->cd_generations++;
		cd->cd_crc = crypto_crc32(cd, sz);
		rv = cf_write(cs, cd, sz);
		if (rv >= 0)
			cs->cs_dirty = false;
	}

	cf_unlock(cs);

	return rv;
}
#endif /* CONFIGDB_READ_ONLY */

#if (RELEASE_BUILD == 0)
static void
configdb_dump(FILE *os)
{
	struct configdb_state *cs = &configdb_state;
	struct configdb *cd = cs->cs_db;
	struct cf_entry *ce;
	unsigned int count;
	void *p;
	static const char *cf_type_str[] = {
		"UINT8", "UINT16", "UINT32", "STRING",
		"DATA", "FLOAT", "DOUBLE", "FIX16"
	};

	cf_lock(cs);

	count = 0;
	for (ce = CF_FIRST(cd); ce->ce_type != CF_TYPE_EOT; ce = CF_NEXT(ce)) {
		if (CF_TYPE_VALID(ce->ce_type)) {
			fprintf(os, "%6s.%04" PRIx16 ": ",
			    cf_type_str[ce->ce_type], ce->ce_id);
		} else {
			fprintf(os, "%6u.%04" PRIx16 ": ",
			    ce->ce_type, ce->ce_id);
		}

		p = (void *)&ce->ce_data[0];

		switch (ce->ce_type) {
		case CF_TYPE_UINT8:
			fprintf(os, "0x%02x\n", (unsigned int)*(uint8_t *)p);
			break;
		case CF_TYPE_UINT16:
			fprintf(os, "0x%04" PRIx16 "\n", *(uint16_t *)p);
			break;
		case CF_TYPE_UINT32:
			fprintf(os, "0x%08" PRIx32 "\n", *(uint32_t *)p);
			break;
		case CF_TYPE_STRING:
			fprintf(os, "%s\n", (const char *)p);
			break;
		case CF_TYPE_DATA:
			fprintf(os, "Length %u\n", (unsigned int)ce->ce_len);
			hexdump(os, "             ", 8, p, ce->ce_len);
			break;
		case CF_TYPE_FLOAT:
			fprintf(os, "<float>\n");
			break;
		case CF_TYPE_DOUBLE:
			fprintf(os, "<double>\n");
			break;
		default:
			fprintf(os, "Uh oh, bogons!\n");
			break;
		}

		count++;
	}

	fprintf(os, "Total entries: %u, ", count);
	/* XXX: Can't use %zu here thanks to nano-specs. */
	fprintf(os, "DB size: %lu/%lu, ",
	    (unsigned long)(sizeof(*cd) + cd->cd_size),
	    (unsigned long)cs->cs_size);
	fprintf(os, "Persist count: %" PRIu32 "\n", cd->cd_generations);

	cf_unlock(cs);
}

static void
configdb_cmd(FILE *os, uint8_t argc, const char * const *argv)
{

	(void) argc;
	(void) argv;

	configdb_dump(os);
}
#endif /* (RELEASE_BUILD == 0) */

/* Get/Set an 8-bit integer */
cf_len_t
cf_get_uint8(cf_id_t id, uint8_t *pv)
{
	cf_len_t rv;

	rv = cf_lookup(id, pv, CF_TYPE_UINT8, sizeof(*pv));

	assert(rv == 0 || rv == sizeof(*pv));

	return rv;
}

cf_len_t
cf_set_uint8(cf_id_t id, uint8_t v)
{

	return cf_insert(id, CF_TYPE_UINT8, sizeof(v), &v);
}

void
cf_del_uint8(cf_id_t id)
{

	cf_delete(id, CF_TYPE_UINT8);
}

/* Get/Set a 16-bit integer */
cf_len_t
cf_get_uint16(cf_id_t id, uint16_t *pv)
{
	cf_len_t rv;

	rv = cf_lookup(id, pv, CF_TYPE_UINT16, sizeof(*pv));

	assert(rv == 0 || rv == sizeof(*pv));

	return rv;
}

cf_len_t
cf_set_uint16(cf_id_t id, uint16_t v)
{

	return cf_insert(id, CF_TYPE_UINT16, sizeof(v), &v);
}

void
cf_del_uint16(cf_id_t id)
{

	cf_delete(id, CF_TYPE_UINT16);
}

/* Get/Set a 32-bit integer */
cf_len_t
cf_get_uint32(cf_id_t id, uint32_t *pv)
{
	cf_len_t rv;

	rv = cf_lookup(id, pv, CF_TYPE_UINT32, sizeof(*pv));

	assert(rv == 0 || rv == sizeof(*pv));

	return rv;
}

cf_len_t
cf_set_uint32(cf_id_t id, uint32_t v)
{

	return cf_insert(id, CF_TYPE_UINT32, sizeof(v), &v);
}

void
cf_del_uint32(cf_id_t id)
{

	cf_delete(id, CF_TYPE_UINT32);
}

/* Get/Set a float. */
cf_len_t
cf_get_float(cf_id_t id, float *pv)
{
	cf_len_t rv;

	rv = cf_lookup(id, pv, CF_TYPE_FLOAT, sizeof(*pv));

	assert(rv == 0 || rv == sizeof(*pv));

	return rv;
}

cf_len_t
cf_set_float(cf_id_t id, float v)
{

	return cf_insert(id, CF_TYPE_FLOAT, sizeof(v), &v);
}

void
cf_del_float(cf_id_t id)
{

	cf_delete(id, CF_TYPE_FLOAT);
}

/* Get/Set a double. */
cf_len_t
cf_get_double(cf_id_t id, double *pv)
{
	cf_len_t rv;

	rv = cf_lookup(id, pv, CF_TYPE_DOUBLE, sizeof(*pv));

	assert(rv == 0 || rv == sizeof(*pv));

	return rv;
}

cf_len_t
cf_set_double(cf_id_t id, double v)
{

	return cf_insert(id, CF_TYPE_DOUBLE, sizeof(v), &v);
}

void
cf_del_double(cf_id_t id)
{

	cf_delete(id, CF_TYPE_DOUBLE);
}

/* Get/Set a nul-terminated C string. DB entry includes the trailing '\0'. */
cf_len_t
cf_get_string(cf_id_t id, size_t max_size, char *v)
{

	return cf_lookup(id, v, CF_TYPE_STRING, max_size);
}

cf_len_t
cf_set_string(cf_id_t id, const char *v)
{

	return cf_insert(id, CF_TYPE_STRING, strlen(v) + 1, v);
}

void
cf_del_string(cf_id_t id)
{

	cf_delete(id, CF_TYPE_STRING);
}

/* Get/Set a buffer of arbitrary data. */
cf_len_t
cf_get_data(cf_id_t id, size_t len, void *pv)
{

	return cf_lookup(id, pv, CF_TYPE_DATA, len);
}

cf_len_t
cf_set_data(cf_id_t id, size_t len, const void *pv)
{

	return cf_insert(id, CF_TYPE_DATA, len, pv);
}

void
cf_del_data(cf_id_t id)
{

	cf_delete(id, CF_TYPE_DATA);
}

#if 0
/* Get/Set a fixed-point value (fix16_t). */
cf_len_t
cf_get_fix16(cf_id_t id, fix16_t *pv)
{
	cf_len_t rv;

	rv = cf_lookup(id, pv, CF_TYPE_FIX16, sizeof(*pv));

	assert(rv == 0 || rv == sizeof(*pv));

	return rv;
}

cf_len_t
cf_set_fix16(cf_id_t id, fix16_t v)
{

	return cf_insert(id, CF_TYPE_FIX16, sizeof(v), &v);
}

void
cf_del_fix16(cf_id_t id)
{

	cf_delete(id, CF_TYPE_FIX16);
}
#endif
