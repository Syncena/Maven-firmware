/* puff.h
  Copyright (C) 2002-2013 Mark Adler, all rights reserved
  version 2.3, 21 Jan 2013

  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the author be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.

  Mark Adler    madler@alumni.caltech.edu
 */


/*
 * See puff.c for purpose and usage.
 */
#ifndef NIL
#  define NIL ((void *)0)      /* for no output option */
#endif

struct puff_args {
	void *(*pa_calloc)(unsigned int, unsigned int);
	void (*pa_free)(void *);

	const unsigned char *pa_source;	/* pointer to source data */
	unsigned long pa_sourcelen;	/* amount of input available */

	union {
		struct {
			unsigned char *pa_dest;	/* dest pointer */
			unsigned long pa_destlen;	/* dest size */
		} /*upa_puff*/;
		struct {
			unsigned long pa_chunklen;
			int (*pa_cb)(void *, const unsigned char *,
				      unsigned long);
			void *pa_cb_arg;
		} /*upa_puff_cb*/;
	} /*pa_u*/;
};
//#define	pa_dest		pa_u.upa_puff.upa_dest
//#define	pa_destlen	pa_u.upa_puff.upa_destlen
//#define	pa_chunklen	pa_u.upa_puff_cb.upa_chunklen
//#define	pa_cb		pa_u.upa_puff_cb.upa_cb
//#define	pa_cb_arg	pa_u.upa_puff_cb.upa_cb_arg

extern int puff(struct puff_args *);
extern int puff_cb(struct puff_args *);
