/*-
 * Copyright (c) 2006 Devicescape Software, Inc. All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce at minimum a disclaimer
 *    similar to the "NO WARRANTY" disclaimer below ("Disclaimer") and any
 *    redistribution must be conditioned upon including a substantially
 *    similar Disclaimer requirement for further binary redistribution.
 * 3. Neither the names of the above-listed copyright holders nor the names
 *    of any contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 *
 * NO WARRANTY
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF NONINFRINGEMENT, MERCHANTIBILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGES.
 *
 */

#ifndef _IF_ATH_H_
#define _IF_ATH_H_

#include <linux/netdevice.h>

int ath_init(struct ath_softc *);
int ath_stop(struct ath_softc *);
int ath_chan_set(struct ath_softc *, HAL_CHANNEL hchan);
int ath_reset(struct ath_softc *);
int ath_d80211_tx(struct ieee80211_hw *, struct sk_buff *skb,
		  struct ieee80211_tx_control *control);
int ath_descdma_setup(struct ath_softc *, struct ath_descdma *, ath_bufhead *,
		      const char *, int, int);
void ath_descdma_cleanup(struct ath_softc *, struct ath_descdma *,
			 ath_bufhead *, int);

int ath_key_alloc(struct ath_softc *sc, struct ieee80211_key_conf *key,
		  const u_int8_t addr[IEEE80211_ADDR_LEN]);
int ath_key_delete(struct ath_softc *sc, struct ieee80211_key_conf *key);
int ath_keyset(struct ath_softc *sc, struct ieee80211_key_conf *key,
	       const u_int8_t mac0[IEEE80211_ADDR_LEN]);

#define	AR_DEBUG

extern int	ath_debug;
#ifdef AR_DEBUG
enum {
	ATH_DEBUG_XMIT		= 0x00000001,	/* basic xmit operation */
	ATH_DEBUG_XMIT_DESC	= 0x00000002,	/* xmit descriptors */
	ATH_DEBUG_RECV		= 0x00000004,	/* basic recv operation */
	ATH_DEBUG_RECV_DESC	= 0x00000008,	/* recv descriptors */
	ATH_DEBUG_RATE		= 0x00000010,	/* rate control */
	ATH_DEBUG_RESET		= 0x00000020,	/* reset processing */
	/* 0x00000040 was ATH_DEBUG_MODE */
	ATH_DEBUG_BEACON 	= 0x00000080,	/* beacon handling */
	ATH_DEBUG_WATCHDOG 	= 0x00000100,	/* watchdog timeout */
	ATH_DEBUG_INTR		= 0x00001000,	/* ISR */
	ATH_DEBUG_TX_PROC	= 0x00002000,	/* tx ISR proc */
	ATH_DEBUG_RX_PROC	= 0x00004000,	/* rx ISR proc */
	ATH_DEBUG_BEACON_PROC	= 0x00008000,	/* beacon ISR proc */
	ATH_DEBUG_CALIBRATE	= 0x00010000,	/* periodic calibration */
	ATH_DEBUG_KEYCACHE	= 0x00020000,	/* key cache management */
	ATH_DEBUG_STATE		= 0x00040000,	/* 802.11 state transitions */
	ATH_DEBUG_NODE		= 0x00080000,	/* node management */
	ATH_DEBUG_LED		= 0x00100000,	/* led management */
	ATH_DEBUG_FF		= 0x00200000,	/* fast frames */
	ATH_DEBUG_TURBO		= 0x00400000,	/* turbo/dynamic turbo */
	ATH_DEBUG_UAPSD		= 0x00800000,	/* uapsd */
	ATH_DEBUG_DOTH		= 0x01000000,	/* 11.h */
	ATH_DEBUG_D80211	= 0x02000000,	/* d80211 interface */
	ATH_DEBUG_FATAL		= 0x80000000,	/* fatal errors */
	ATH_DEBUG_ANY		= 0xffffffff
};
#define	DPRINTF(sc, _m, _fmt, ...) do {				\
	if (sc->sc_debug & (_m))				\
		printk(_fmt, __VA_ARGS__);			\
} while (0)
#else /* defined(AR_DEBUG) */
#define	DPRINTF(sc, _m, _fmt, ...)
#endif /* defined(AR_DEBUG) */

#endif /* _IF_ATH_H_ */
