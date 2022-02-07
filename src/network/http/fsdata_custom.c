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
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "rtos.h"
#include "fsdata_custom.h"
#include "linked-lists.h"
#include "zone_alloc.h"

#if (LWIP_HTTPD_FILE_STATE == 0)
#error "Need LWIP_HTTPD_FILE_STATE==1"
#endif
#if (HTTPD_PRECALCULATED_CHECKSUM != 0)
#error "Precalculated checksum not supported."
#endif

/* LwIP ought to have the following three prototypes in a header... */
extern int fs_open_custom(struct fs_file *file, const char *name);
extern void fs_close_custom(struct fs_file *file);
extern int fs_read_custom(struct fs_file *file, char *buffer, int count);

struct fsdata_custom_handler {
	fsdata_custom_open_t ch_open;
	fsdata_custom_read_t ch_read;
	fsdata_custom_close_t ch_close;
	void *ch_arg;
	LIST_ENTRY(fsdata_custom_handler) ch_qent;
};
LIST_HEAD(custom_handler_qhead, fsdata_custom_handler);

struct fsdata_custom_state {
	const struct fsdata_file *cs_static;
	struct fsdata_file *cs_dynamic;
	unsigned int cs_one_time_count;
	zone_t cs_zone;
	rtos_mutex_t cs_mutex;
	struct custom_handler_qhead cs_custom_handlers;
};
static struct fsdata_custom_state fsdata_custom_state;
#define	FSDATA_CUSTOM_ZONE_SIZE		8192
#define	FSDATA_FLAG_DYNAMIC_ONE_TIME	(1u << 6)
#define	FSDATA_FLAG_DYNAMIC_DUP_DATA	(1u << 7)
#define	FSDATA_FLAG_LOCAL_MASK		(FSDATA_FLAG_DYNAMIC_ONE_TIME | \
					 FSDATA_FLAG_DYNAMIC_DUP_DATA)

static void
fsdata_change_custom_root(struct fsdata_custom_state *cs,
    const struct fsdata_file *static_root)
{

	rtos_mutex_acquire(cs->cs_mutex);
	cs->cs_static = static_root;
	rtos_mutex_release(cs->cs_mutex);
}

void
fsdata_set_http_root_app(void)
{
	extern const struct fsdata_file * const fs_root_app;
	
	fsdata_change_custom_root(&fsdata_custom_state, fs_root_app);
}

void
fsdata_set_http_root_provision(void)
{
	extern const struct fsdata_file * const fs_root_provision;

	fsdata_change_custom_root(&fsdata_custom_state, fs_root_provision);
}

void *
fsdata_add_custom_handler(fsdata_custom_open_t open_func,
    fsdata_custom_read_t read_func, fsdata_custom_close_t close_func, void *arg)
{
	struct fsdata_custom_state *cs = &fsdata_custom_state;
	struct fsdata_custom_handler *ch;

	if ((ch = zone_malloc(sizeof(*ch))) == NULL)
		return NULL;

	ch->ch_open = open_func;
	ch->ch_read = read_func;
	ch->ch_close = close_func;
	ch->ch_arg = arg;

	rtos_mutex_acquire(cs->cs_mutex);
	LIST_INSERT_HEAD(&cs->cs_custom_handlers, ch, ch_qent);
	rtos_mutex_release(cs->cs_mutex);

	return ch;
}

void
fsdata_del_custom_handler(void *arg)
{
	struct fsdata_custom_state *cs = &fsdata_custom_state;
	struct fsdata_custom_handler *ch = arg;

	rtos_mutex_acquire(cs->cs_mutex);
	LIST_REMOVE(ch, ch_qent);
	rtos_mutex_release(cs->cs_mutex);

	zone_free(ch);
}

void
fsdata_custom_init(void)
{
	struct fsdata_custom_state *cs = &fsdata_custom_state;
	rtos_mutex_t zm;
	void *mem;
	extern const struct fsdata_file * const fs_root_app;

	cs->cs_mutex = rtos_mutex_create();
	assert(cs->cs_mutex != NULL);
	LIST_INIT(&cs->cs_custom_handlers);
	cs->cs_static = fs_root_app;
	cs->cs_dynamic = NULL;
	cs->cs_one_time_count = 0;

	mem = zone_malloc(FSDATA_CUSTOM_ZONE_SIZE);
	assert(mem != NULL);
	zm = rtos_mutex_create();
	assert(zm != NULL);
	cs->cs_zone = zone_create_named("fsdata", zm, mem,
	    FSDATA_CUSTOM_ZONE_SIZE, 0);
	assert(cs->cs_zone != NULL);
}

static const struct fsdata_file *
fs_search_custom(const char *name, const struct fsdata_file *f)
{

	for ( ; f != NULL; f = f->next) {
		if (strcmp(name, (const char *)f->name) == 0)
			break;
	}

	return f;
}

void
fsdata_create(const char *name, const void *data, size_t len, bool dup)
{
	struct fsdata_custom_state *cs = &fsdata_custom_state;
	const struct fsdata_file *oldf;
	struct fsdata_file *f;
	void *new_data;

	if (dup) {
		new_data = zone_malloc_private(cs->cs_zone, len);
		assert(new_data != NULL);
		if (new_data == NULL)
			return;
		memcpy(new_data, data, len);
		data = new_data;
	} else {
		new_data = NULL;
	}

	rtos_mutex_acquire(cs->cs_mutex);

	if ((oldf = fs_search_custom(name, cs->cs_dynamic)) != NULL) {
		/* Entry already exists; we can reuse the handle. */
		f = (struct fsdata_file *)(uintptr_t)oldf;
		if ((f->flags & FSDATA_FLAG_DYNAMIC_DUP_DATA) != 0) {
			zone_free_private(cs->cs_zone,
			    (void *)(uintptr_t)f->data);
		}
	} else {
		size_t nlen;
		char *n;

		f = zone_calloc_private(cs->cs_zone, 1, sizeof(*f));
		nlen = strlen(name) + 1;
		n = zone_malloc_private(cs->cs_zone, nlen);
		assert(f != NULL);
		assert(n != NULL);
		if (f == NULL || n == NULL) {
			if (n != NULL)
				zone_free_private(cs->cs_zone, n);
			if (f != NULL)
				zone_free_private(cs->cs_zone, f);
			if (new_data != NULL)
				zone_free_private(cs->cs_zone, new_data);
			rtos_mutex_release(cs->cs_mutex);
			return;
		}

		memcpy(n, name, nlen);
		f->name = (const unsigned char *)n;
		f->next = cs->cs_dynamic;
		cs->cs_dynamic = f;
	}

	f->data = data;
	f->len = (int)len;
	f->flags = dup ? FSDATA_FLAG_DYNAMIC_DUP_DATA : 0;

	rtos_mutex_release(cs->cs_mutex);
}

const char *
fsdata_create_one_time(const char *suff, const void *data, size_t len, bool dup)
{
	struct fsdata_custom_state *cs = &fsdata_custom_state;
	struct fsdata_file *f;
	void *new_data;
	char *name;

	if ((name = zone_malloc_private(cs->cs_zone, 32)) == NULL)
		return "/error.txt";

	if (dup) {
		new_data = zone_malloc_private(cs->cs_zone, len);
		assert(new_data != NULL);
		if (new_data == NULL) {
			zone_free_private(cs->cs_zone, name);
			return "/error.txt";
		}
		memcpy(new_data, data, len);
		data = new_data;
	} else {
		new_data = NULL;
	}

	rtos_mutex_acquire(cs->cs_mutex);

	sprintf(name, "/ot%u.%s", cs->cs_one_time_count++, suff);

	assert(fs_search_custom(name, cs->cs_dynamic) == NULL);

	f = zone_malloc_private(cs->cs_zone, sizeof(*f));
	assert(f != NULL);
	if (f == NULL) {
		if (new_data != NULL)
			zone_free_private(cs->cs_zone, new_data);
		zone_free_private(cs->cs_zone, name);
		return "/error.txt";
	}

	f->name = (const unsigned char *)name;
	f->data = data;
	f->len = len;
	f->flags = dup ? FSDATA_FLAG_DYNAMIC_DUP_DATA : 0;
	f->flags |= FSDATA_FLAG_DYNAMIC_ONE_TIME;
	f->next = cs->cs_dynamic;
	cs->cs_dynamic = f;

	rtos_mutex_release(cs->cs_mutex);

	return name;
}

void
fsdata_delete(const char *name)
{
	struct fsdata_custom_state *cs = &fsdata_custom_state;
	struct fsdata_file **pf, *f;

	rtos_mutex_acquire(cs->cs_mutex);

	f = NULL;
	for (pf = &cs->cs_dynamic; *pf != NULL;
	    pf = (struct fsdata_file **)(uintptr_t)&(*pf)->next) {
		if (strcmp(name, (const char *)(*pf)->name) == 0) {
			f = *pf;
			*pf = (struct fsdata_file *)(uintptr_t)f->next;
			break;
		}
	}

	if (f != NULL) {
		zone_free_private(cs->cs_zone, (void *)(uintptr_t)f->name);
		if (f->flags & FSDATA_FLAG_DYNAMIC_DUP_DATA) {
			zone_free_private(cs->cs_zone,
			    (void *)(uintptr_t)f->data);
		}
		zone_free_private(cs->cs_zone, f);
	}

	rtos_mutex_release(cs->cs_mutex);
}

/*
 * fs_*_custom() API for LwIP's httpd.
 */
void *
fs_state_init(struct fs_file *file, const char *name)
{
	(void) file;
	(void) name;

	/*
	 * This function is invoked *only* from fs_open() when
	 * our own fs_open_custom() does not find the required
	 * file. There is no local state in this case, so always
	 * return NULL.
	 */
	return NULL;
}

void
fs_state_free(struct fs_file *file, void *state)
{
	struct fsdata_file *f;

	(void) file;

	/*
	 * This function will be invoked for every file which is closed.
	 * In almost all cases, 'state' will be NULL. It will be non-NULL
	 * when dealing with a 'one-time' file, which must be deleted
	 * after it has been closed. In this case 'state' points to the
	 * corresponding struct fsdata_custom on the cs_dynamic list.
	 */
	if ((f = state) == NULL)
		return;

	assert((f->flags & FSDATA_FLAG_DYNAMIC_ONE_TIME) != 0);
	assert(file->state == state);

	file->state = NULL;

	fsdata_delete((const char *)f->name);
}

int
fs_open_custom(struct fs_file *file, const char *name)
{
	struct fsdata_custom_state *cs = &fsdata_custom_state;
	const struct fsdata_file *f;
	struct fsdata_custom_handler *ch;
	int rv = 0;

	/*
	 * Try all registered custom handlers first.
	 */
	rtos_mutex_acquire(cs->cs_mutex);
	LIST_FOREACH(ch, &cs->cs_custom_handlers, ch_qent) {
		if ((ch->ch_open)(file, name, ch->ch_arg)) {
			file->pextension = ch;
			rtos_mutex_release(cs->cs_mutex);
			return 1;
		}
	}

	/*
	 * Now try our dynamic and static custom roots.
	 */
	if ((f = fs_search_custom(name, cs->cs_dynamic)) != NULL ||
	    (f = fs_search_custom(name, cs->cs_static)) != NULL) {
		file->data = (const char *)f->data;
		file->len = f->len;
		file->index = f->len;
		file->flags = f->flags & ~FSDATA_FLAG_LOCAL_MASK;
		file->pextension = NULL;
		if (f->flags & FSDATA_FLAG_DYNAMIC_ONE_TIME)
			file->state = (void *)(uintptr_t)f;
		else
			file->state = NULL;
		rv = 1;
	}

	rtos_mutex_release(cs->cs_mutex);

	return rv;
}

void
fs_close_custom(struct fs_file *file)
{
	struct fsdata_custom_handler *ch;

	if ((ch = file->pextension) != NULL && ch->ch_close != NULL)
		(ch->ch_close)(file, ch->ch_arg);
}

int
fs_read_custom(struct fs_file *file, char *buffer, int count)
{
	struct fsdata_custom_handler *ch;
	int read_len;

	if ((ch = file->pextension) != NULL && ch->ch_read != NULL)
		return (ch->ch_read)(file, buffer, count, ch->ch_arg);

	read_len = file->len - file->index;
	if (read_len > count)
		read_len = count;

	memcpy(buffer, (file->data + file->index), read_len);
	file->index += read_len;

	return read_len;
}
