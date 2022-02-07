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
#include <stdlib.h>
#include <stdio.h>

#include "lwip/opt.h"
#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/sys.h"
#include "lwip/udp.h"
#include "lwip/prot/dns.h"

#include "dns_hijack.h"
#include "zone_alloc.h"

#ifdef PACK_STRUCT_USE_INCLUDES
#  include "arch/bpstruct.h"
#endif
PACK_STRUCT_BEGIN
struct dns_response {
	PACK_STRUCT_FIELD(uint16_t dr_name);
	PACK_STRUCT_FIELD(uint16_t dr_type);
	PACK_STRUCT_FIELD(uint16_t dr_class);
	PACK_STRUCT_FIELD(uint32_t dr_ttl);
	PACK_STRUCT_FIELD(uint16_t dr_length);
	PACK_STRUCT_FIELD(uint32_t dr_data);
} PACK_STRUCT_STRUCT;
PACK_STRUCT_END
#ifdef PACK_STRUCT_USE_INCLUDES
#  include "arch/epstruct.h"
#endif

struct dns_hijack_state {
	struct udp_pcb *hs_pcb;
	char hs_query[128];
	struct dns_response hs_resp;
};
static struct dns_hijack_state *dns_hijack_state;

static void
dns_hijack_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p,
    const ip_addr_t *addr, u16_t port)
{
	struct dns_hijack_state *hs = arg;
	struct dns_hdr *hdr;
	struct pbuf *rp;
	size_t rlen;
	char *q;

	LWIP_UNUSED_ARG(arg);
	LWIP_UNUSED_ARG(pcb);

	if (p->tot_len != p->len || p->tot_len > sizeof(hs->hs_query)) {
 bail:
		pbuf_free(p);
		return;
	}

	if (pbuf_copy_partial(p, hs->hs_query, p->tot_len, 0) < p->tot_len)
		goto bail;
	pbuf_free(p);

	hdr = (struct dns_hdr *)hs->hs_query;

	hdr->flags1 &= DNS_FLAG1_RD;
	hdr->flags1 |= DNS_FLAG1_RESPONSE | DNS_FLAG1_OPCODE_STANDARD |
	    DNS_FLAG1_AUTHORATIVE;
	hdr->flags2 = DNS_FLAG2_ERR_NONE;
	hdr->numanswers = hdr->numquestions;
	hdr->numauthrr = 0;
	hdr->numextrarr = 0;

	q = &hs->hs_query[sizeof(*hdr)];

	/* Skip QNAME */
	while (*q++ != '\0' && q < &hs->hs_query[sizeof(hs->hs_query)])
		;

	/* Skip QTYPE and QCLASS */
	q += 4;

	if (q >= &hs->hs_query[sizeof(hs->hs_query)])
		return;

	rlen = (size_t)(q - hs->hs_query);
	rp = pbuf_alloc(PBUF_TRANSPORT, rlen + sizeof(hs->hs_resp), PBUF_RAM);
	if (rp == NULL)
		return;

	if (pbuf_take(rp, hs->hs_query, rlen) == ERR_OK &&
	    pbuf_take_at(rp, &hs->hs_resp,sizeof(hs->hs_resp),rlen) == ERR_OK) {
		(void) udp_sendto(hs->hs_pcb, rp, addr, port);
	}

	pbuf_free(rp);
}

void
dns_hijack_start(const ip4_addr_t our_ip)
{
	struct dns_hijack_state *hs;
	err_t res;

	assert(dns_hijack_state == NULL);

	hs = zone_malloc(sizeof(*dns_hijack_state));
	assert(hs != NULL);

	hs->hs_resp.dr_name = lwip_htons(0xc00cu);
	hs->hs_resp.dr_type = lwip_htons(DNS_RRTYPE_A);
	hs->hs_resp.dr_class = lwip_htons(DNS_RRCLASS_IN);
	hs->hs_resp.dr_ttl = lwip_htons(0);
	hs->hs_resp.dr_length = lwip_htons(sizeof(our_ip));
	hs->hs_resp.dr_data = our_ip.addr;

	hs->hs_pcb = udp_new_ip_type(IPADDR_TYPE_ANY);
	LWIP_ASSERT("Failed to allocate pcb", hs->hs_pcb != NULL);

	res = udp_bind(hs->hs_pcb, IP_ANY_TYPE, DNS_SERVER_PORT);
	LWIP_UNUSED_ARG(res);
	LWIP_ASSERT("Failed to bind pcb", res == ERR_OK);
	dns_hijack_state = hs;
	udp_recv(hs->hs_pcb, dns_hijack_recv, hs);
}

void
dns_hijack_stop(void)
{
	struct dns_hijack_state *hs;

	assert(dns_hijack_state != NULL);
	hs = dns_hijack_state;
	dns_hijack_state = NULL;

	udp_remove(hs->hs_pcb);
	zone_free(hs);
}
