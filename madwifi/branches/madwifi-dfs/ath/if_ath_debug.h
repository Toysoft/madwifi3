/*
 * This software is distributed under the terms of the
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
 * $Id: if_ath_radar.h 2464 2007-06-15 22:51:56Z mtaylor $
 */
#ifndef _IF_ATH_DEBUG_H_

#ifdef AR_DEBUG
#define	DFLAG_ISSET(sc, _m) \
	((sc->sc_debug & _m))
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
	ATH_DEBUG_DOTHFILT	= 0x02000000,	/* 11.h radar pulse filter algorithm */
	ATH_DEBUG_DOTHFILTVBSE	= 0x04000000,	/* 11.h radar pulse filter algorithm - pulse level debug */
	ATH_DEBUG_DOTHFILTNOSC  = 0x08000000,	/* 11.h radar pulse filter algorithm - disable short circuit of processing after detection */
	ATH_DEBUG_DOTHPULSES    = 0x10000000,   /* 11.h radar pulse events */
	ATH_DEBUG_FATAL		= 0x80000000,	/* fatal errors */
	ATH_DEBUG_ANY		= 0xffffffff
};
#define	DPRINTF(sc, _m, _fmt, ...) do {				\
	if (sc->sc_debug & (_m))				\
		printk(_fmt, __VA_ARGS__);			\
} while (0)
#define	KEYPRINTF(sc, ix, hk, mac) do {				\
	if (sc->sc_debug & ATH_DEBUG_KEYCACHE)			\
		ath_keyprint(sc, __func__, ix, hk, mac);	\
} while (0)
#else /* defined(AR_DEBUG) */
#define	DFLAG_ISSET(sc, _m)		0
#define	DPRINTF(sc, _m, _fmt, ...)
#define	KEYPRINTF(sc, k, ix, mac)
#endif /* defined(AR_DEBUG) */
#define	IFF_DUMPPKTS(sc, _m) DFLAG_ISSET(sc,_m)

#endif /* #ifndef _IF_ATH_DEBUG_H_ */
