﻿/**
 *
 * \file
 *
 * \brief This module contains debug APIs declarations.
 *
 * Copyright (c) 2016 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#ifndef _NM_DEBUG_H_
#define _NM_DEBUG_H_

#include "bsp/include/nm_bsp.h"
#include "bsp/include/nm_bsp_internal.h"

/**@defgroup  DebugDefines DebugDefines
 * @ingroup WlanDefines
 */
/**@{*/


#define M2M_LOG_NONE									0
#define M2M_LOG_ERROR									1
#define M2M_LOG_INFO									2
#define M2M_LOG_REQ										3
#define M2M_LOG_DBG										4

#if (defined __APS3_CORTUS__)
#define M2M_LOG_LEVEL									M2M_LOG_ERROR
#elif !defined(M2M_LOG_LEVEL)
#define M2M_LOG_LEVEL									M2M_LOG_REQ
#endif


#define M2M_ERR(...)				do { } while (0)
#define M2M_INFO(...)				do { } while (0)
#define M2M_REQ(...)				do { } while (0)
#define M2M_DBG(...)				do { } while (0)
#define M2M_PRINT(...)				do { } while (0)

#if (CONF_WINC_DEBUG == 1)
#ifndef PSTR
#define	PSTR(x)	x
#endif
#undef M2M_PRINT
#define M2M_PRINT(fmt,...)			do{CONF_WINC_PRINTF(PSTR(fmt), ##__VA_ARGS__);CONF_WINC_PRINTF(PSTR("\r"));}while(0)
#if (M2M_LOG_LEVEL >= M2M_LOG_ERROR)
#undef M2M_ERR
#define M2M_ERR(fmt,...)			do{CONF_WINC_PRINTF(PSTR("(APP)(ERR)[%s][%d]"),__FUNCTION__,__LINE__); CONF_WINC_PRINTF(PSTR(fmt), ##__VA_ARGS__);CONF_WINC_PRINTF(PSTR("\r"));}while(0)
#if (M2M_LOG_LEVEL >= M2M_LOG_INFO)
#undef M2M_INFO
#define M2M_INFO(fmt,...)			do{CONF_WINC_PRINTF(PSTR("(APP)(INFO)")); CONF_WINC_PRINTF(PSTR(fmt), ##__VA_ARGS__);CONF_WINC_PRINTF(PSTR("\r"));}while(0)
#if (M2M_LOG_LEVEL >= M2M_LOG_REQ)
#undef M2M_REQ
#define M2M_REQ(fmt,...)			do{CONF_WINC_PRINTF(PSTR("(APP)(R)")); CONF_WINC_PRINTF(PSTR(fmt), ##__VA_ARGS__);CONF_WINC_PRINTF(PSTR("\r"));}while(0)
#if (M2M_LOG_LEVEL >= M2M_LOG_DBG)
#undef M2M_DBG
#define M2M_DBG(fmt,...)			do{CONF_WINC_PRINTF(PSTR("(APP)(DBG)[%s][%d]"),__FUNCTION__,__LINE__); CONF_WINC_PRINTF(PSTR(fmt), ##__VA_ARGS__);CONF_WINC_PRINTF(PSTR("\r"));}while(0)
#endif /*M2M_LOG_DBG*/
#endif /*M2M_LOG_REQ*/
#endif /*M2M_LOG_INFO*/
#endif /*M2M_LOG_ERROR*/
#endif /*CONF_WINC_DEBUG */

/**@}*/
#endif /* _NM_DEBUG_H_ */
