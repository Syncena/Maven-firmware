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

#ifndef GDB_SIGNALS_H
#define GDB_SIGNALS_H

/*
 * Derived from binutils/include/gdb/signals.def
 */

#define	GDB_SIGNAL_0		0
#define	GDB_SIGNAL_HUP		1
#define	GDB_SIGNAL_INT		2
#define	GDB_SIGNAL_QUIT		3
#define	GDB_SIGNAL_ILL		4
#define	GDB_SIGNAL_TRAP		5
#define	GDB_SIGNAL_ABRT		6
#define	GDB_SIGNAL_EMT		7
#define	GDB_SIGNAL_FPE		8
#define	GDB_SIGNAL_KILL		9
#define	GDB_SIGNAL_BUS		10
#define	GDB_SIGNAL_SEGV		11
#define	GDB_SIGNAL_SYS		12
#define	GDB_SIGNAL_PIPE		13
#define	GDB_SIGNAL_ALRM		14
#define	GDB_SIGNAL_TERM		15
#define	GDB_SIGNAL_URG		16
#define	GDB_SIGNAL_STOP		17
#define	GDB_SIGNAL_TSTP		18
#define	GDB_SIGNAL_CONT		19
#define	GDB_SIGNAL_CHLD		20
#define	GDB_SIGNAL_TTIN		21
#define	GDB_SIGNAL_TTOU		22
#define	GDB_SIGNAL_IO		23
#define	GDB_SIGNAL_XCPU		24
#define	GDB_SIGNAL_XFSZ		25
#define	GDB_SIGNAL_VTALRM	26
#define	GDB_SIGNAL_PROF		27
#define	GDB_SIGNAL_WINCH	28
#define	GDB_SIGNAL_LOST		29
#define	GDB_SIGNAL_USR1		30
#define	GDB_SIGNAL_USR2		31
#define	GDB_SIGNAL_PWR		32

#endif /* GDB_SIGNALS_H */
