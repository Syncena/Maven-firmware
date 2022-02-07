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

#ifndef JSON_UTILS_H
#define JSON_UTILS_H

#define	JSON_STRING(k,v)	"\"" k "\":\"%s\"", (v)
#define	JSON_STRING_CH(k,v)	"\"" k "\":\"%c\"", (v)
#define	JSON_STRING_V(k,v)	"\"%s\":\"%s\"", (k), (v)

#define	JSON_NUMBER(k,v)	"\"" k "\":%d", (int)(v)
#define	JSON_NUMBER_V(k,v)	"\"%s\":%d", (k), (int)(v)

#define	JSON_BOOL(k,v)		"\"" k "\":%s", ((v)?"true":"false")
#define	JSON_BOOL_V(k,v)	"\"%s\":%s", (k), ((v)?"true":"false")

#define	JSON_ARRAY_START	"["
#define	JSON_ARRAY_NAMED(k)	"\"" k "\":["
#define	JSON_ARRAY_NAMED_V(k)	"\"%s\":[", (k)
#define	JSON_ARRAY_END		"]"

#define	JSON_OBJECT_START	"{"
#define	JSON_OBJECT_NAMED(k)	"\"" k "\":{"
#define	JSON_OBJECT_NAMED_V(k)	"\"%s\":{", (k)
#define	JSON_OBJECT_END		"}"

#define	JSON_SEPARATOR		","

#endif /* JSON_UTILS_H */
