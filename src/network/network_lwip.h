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

#ifndef NETWORK_LWIP_H
#define NETWORK_LWIP_H

/*
 * On no account must ANY calls to the driver entrypoints, below, be
 * made by code outside of network.c. Driver code is generally not
 * re-entrant and network.c goes to some lengths to ensure locking
 * around driver calls. There is usually some other way to achieve
 * what you are trying to do via the proper API.
 *
 * Failure to follow this process *WILL* cause weird problems and
 * crashes.
 */
struct netif;
struct network_state;

struct network_lwip_driver {
	struct netif *nld_netif;
	uint32_t (*nld_handle_interrupt)(struct netif *);
	void	(*nld_link_check)(struct netif *);
	void	(*nld_link_status)(struct netif *,
		    network_link_status_callback_t, void *);
	void	(*nld_get_mac)(struct netif *, uint8_t *);
	int8_t	(*nld_ioctl)(struct netif *, uint8_t, void *);
};
#define	NETWORK_IOCTL_LWIP_SCHEDULE	NETWORK_IOCTL_DRIVER_BASE

extern struct network_state *network_lwip_attach(struct network_lwip_driver *);

#endif /* NETWORK_LWIP_H */
