/*-
 * Copyright (c) 2002-2005 Sam Leffler, Errno Consulting
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 */
#ifndef EXPORT_SYMTAB
#define	EXPORT_SYMTAB
#endif

/*
 * IEEE 802.11 ap scanning support.
 */
#ifndef AUTOCONF_INCLUDED
#include <linux/config.h>
#endif
#include <linux/version.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/sort.h>

#include "if_media.h"

#include <net80211/ieee80211_var.h>

struct ap_state {
	int as_maxrssi[IEEE80211_CHAN_MAX];
	struct IEEE80211_TQ_STRUCT as_actiontq;	/* tasklet for "action" */
	struct ieee80211_scan_entry as_selbss;	/* selected bss for action tasklet */
	int (*as_action)(struct ieee80211vap *, const struct ieee80211_scan_entry *);
};

static int ap_flush(struct ieee80211_scan_state *);
static void action_tasklet(IEEE80211_TQUEUE_ARG);

/*
 * Attach prior to any scanning work.
 */
static int
ap_attach(struct ieee80211_scan_state *ss)
{
	struct ap_state *as;

	_MOD_INC_USE(THIS_MODULE, return 0);

	MALLOC(as, struct ap_state *, sizeof(struct ap_state),
		M_SCANCACHE, M_NOWAIT);
	ss->ss_priv = as;
	IEEE80211_INIT_TQUEUE(&as->as_actiontq, action_tasklet, ss);
	ap_flush(ss);
	return 1;
}

/*
 * Cleanup any private state.
 */
static int
ap_detach(struct ieee80211_scan_state *ss)
{
	struct ap_state *as = ss->ss_priv;

	if (as != NULL)
		FREE(as, M_SCANCACHE);

	_MOD_DEC_USE(THIS_MODULE);
	return 1;
}

/*
 * Flush all per-scan state.
 */
static int
ap_flush(struct ieee80211_scan_state *ss)
{
	struct ap_state *as = ss->ss_priv;

	memset(as->as_maxrssi, 0, sizeof(as->as_maxrssi));
	ss->ss_last = 0;		/* ensure no channel will be picked */
	return 0;
}

static int
find11gchannel(struct ieee80211com *ic, int i, int freq)
{
	const struct ieee80211_channel *c;
	int j;

	/*
	 * The normal ordering in the channel list is b channel
	 * immediately followed by g so optimize the search for
	 * this.  We'll still do a full search just in case.
	 */
	for (j = i+1; j < ic->ic_nchans; j++) {
		c = &ic->ic_channels[j];
		if (c->ic_freq == freq && IEEE80211_IS_CHAN_ANYG(c))
			return 1;
	}
	for (j = 0; j < i; j++) {
		c = &ic->ic_channels[j];
		if (c->ic_freq == freq && IEEE80211_IS_CHAN_ANYG(c))
			return 1;
	}
	return 0;
}

/*
 * Start an ap scan by populating the channel list.
 */
static int
ap_start(struct ieee80211_scan_state *ss, struct ieee80211vap *vap)
{
	struct ieee80211com *ic = vap->iv_ic;
	struct ieee80211_channel *c;
	int i;

	ss->ss_last = 0;
	if (vap->iv_des_mode == IEEE80211_MODE_AUTO) {
		for (i = 0; i < ic->ic_nchans; i++) {
			c = &ic->ic_channels[i];
			if (IEEE80211_IS_CHAN_TURBO(c)) {
				/* XR is not supported on turbo channels */
				if (vap->iv_ath_cap & IEEE80211_ATHC_XR)
					continue;
				/* dynamic channels are scanned in base mode */
				if (!IEEE80211_IS_CHAN_ST(c))
					continue;
			} else {
				/*
				 * Use any 11g channel instead of 11b one.
				 */
				if (IEEE80211_IS_CHAN_B(c) &&
				    find11gchannel(ic, i, c->ic_freq))
					continue;
			}
			if (c->ic_flags & IEEE80211_CHAN_RADAR)
				continue;
			if (ss->ss_last >= IEEE80211_SCAN_MAX)
				break;
			ss->ss_chans[ss->ss_last++] = c;
		}
	} else {
		static const u_int chanflags[] = {
			0,			/* IEEE80211_MODE_AUTO */
			IEEE80211_CHAN_A,	/* IEEE80211_MODE_11A */
			IEEE80211_CHAN_B,	/* IEEE80211_MODE_11B */
			IEEE80211_CHAN_PUREG,	/* IEEE80211_MODE_11G */
			IEEE80211_CHAN_FHSS,	/* IEEE80211_MODE_FH */
			IEEE80211_CHAN_108A,	/* IEEE80211_MODE_TURBO_A */
			IEEE80211_CHAN_108G,	/* IEEE80211_MODE_TURBO_G */
			IEEE80211_CHAN_ST,	/* IEEE80211_MODE_TURBO_STATIC_A */
		};
		u_int modeflags;

		modeflags = chanflags[vap->iv_des_mode];
		if (vap->iv_ath_cap & IEEE80211_ATHC_TURBOP && modeflags != IEEE80211_CHAN_ST) {
			if (vap->iv_des_mode == IEEE80211_MODE_11G)
				modeflags = IEEE80211_CHAN_108G;
			else
				modeflags = IEEE80211_CHAN_108A;
		}
		for (i = 0; i < ic->ic_nchans; i++) {
			c = &ic->ic_channels[i];
			if ((c->ic_flags & modeflags) != modeflags)
				continue;
			/* XR is not supported on turbo channels */
			if (IEEE80211_IS_CHAN_TURBO(c) && vap->iv_ath_cap & IEEE80211_ATHC_XR)
				continue;
			if (ss->ss_last >= IEEE80211_SCAN_MAX)
				break;
			/* 
			 * do not select static turbo channels if the mode is not
			 * static turbo .
			 */
			if (IEEE80211_IS_CHAN_STURBO(c) && vap->iv_des_mode != IEEE80211_MODE_MAX )
				continue;
			/* No dfs interference detected channels */
			if (c->ic_flags & IEEE80211_CHAN_RADAR)
				continue;
			ss->ss_chans[ss->ss_last++] = c;
		}
	}
	ss->ss_next = 0;
	/* XXX tunables */
	ss->ss_mindwell = msecs_to_jiffies(200);	/* 200ms */
	ss->ss_maxdwell = msecs_to_jiffies(300);	/* 300ms */

#ifdef IEEE80211_DEBUG
	if (ieee80211_msg_scan(vap)) {
		printf("%s: scan set ", vap->iv_dev->name);
		ieee80211_scan_dump_channels(ss);
		printf(" dwell min %ld max %ld\n",
			ss->ss_mindwell, ss->ss_maxdwell);
	}
#endif /* IEEE80211_DEBUG */

	return 0;
}

/*
 * Restart a bg scan.
 */
static int
ap_restart(struct ieee80211_scan_state *ss, struct ieee80211vap *vap)
{
	return 0;
}

/*
 * Cancel an ongoing scan.
 */
static int
ap_cancel(struct ieee80211_scan_state *ss, struct ieee80211vap *vap)
{
	struct ap_state *as = ss->ss_priv;

	IEEE80211_CANCEL_TQUEUE(&as->as_actiontq);
	return 0;
}

/*
 * Record max rssi on channel.
 */
static int
ap_add(struct ieee80211_scan_state *ss, const struct ieee80211_scanparams *sp,
	const struct ieee80211_frame *wh, int subtype, int rssi, u_int64_t rtsf)
{
	struct ap_state *as = ss->ss_priv;
	struct ieee80211vap *vap = ss->ss_vap;
	struct ieee80211com *ic = vap->iv_ic;
	int chan;

	chan = ieee80211_chan2ieee(ic, ic->ic_curchan);
	/* XXX better quantification of channel use? */
	/* XXX: count BSSs? */
	if (rssi > as->as_maxrssi[chan])
		as->as_maxrssi[chan] = rssi;
	/* XXX interference, turbo requirements */
	return 1;
}


/*
 * ap_end
 *
 * Choose the most optimal channel taking under consideration rssi of the
 * received packets and the current Spectrum Management algorithm.
 *
 * This can be called a stub since it is of no important use unless there is a
 * way to keep the scan results up-to-date without interrupting BSS operation.
 *
 */

struct ap_pc_params {
	struct ieee80211vap *vap;
	struct ieee80211_scan_state *ss;
	int flags;
};

struct channel {
	struct ieee80211_channel *chan;
	int orig;
	struct ap_pc_params *params;
};

static int
ap_pc_cmp_radar(struct ieee80211_channel *a, struct ieee80211_channel *b)
{
	/* a is better than b (return < 0) when b is marked while a is not */
	return !!IEEE80211_IS_CHAN_RADAR(a) - !!IEEE80211_IS_CHAN_RADAR(b);
}

static int
ap_pc_cmp_keepmode(struct ap_pc_params *params, struct ieee80211_channel *a,
		struct ieee80211_channel *b)
{
	struct ieee80211com *ic = params->vap->iv_ic;
	struct ieee80211_channel *cur = ic->ic_bsschan;

	if (!(params->flags & IEEE80211_SCAN_KEEPMODE))
		return 0;

	/* a is better than b (return < 0) when (a, cur) have the same mode and (b, cur) do not */
	return
		!!IEEE80211_ARE_CHANS_SAME_MODE(b, cur) -
		!!IEEE80211_ARE_CHANS_SAME_MODE(a, cur);
}

static int
ap_pc_cmp_sc(struct ieee80211com *ic, struct ieee80211_channel *a,
		struct ieee80211_channel *b)
{
	/* a is better than b (return < 0) when a has more chan nodes than b */
	return
		ic->ic_chan_nodes[b->ic_ieee] -
		ic->ic_chan_nodes[a->ic_ieee];
}

static int
ap_pc_cmp_rssi(struct ap_state *as, struct ieee80211_channel *a,
		struct ieee80211_channel *b)
{
	/* a is better than b (return < 0) when a has rssi less than b */
	return
		as->as_maxrssi[a->ic_ieee] -
		as->as_maxrssi[b->ic_ieee];
}

static int
ap_pc_cmp_samechan(struct ieee80211com *ic, struct ieee80211_channel *a,
		struct ieee80211_channel *b)
{
	struct ieee80211_channel *ic_bsschan = ic->ic_bsschan;
	if (ic_bsschan == IEEE80211_CHAN_ANYC)
		return 0;
	/* a is better than b (return < 0) when a is current (and b is not) */
	return (b == ic_bsschan) - (a == ic_bsschan);
}

/* use original scan list order */
static int
ap_pc_cmp_orig(struct channel *a, struct channel *b)
{
	return a->orig - b->orig;
}

static int
ap_pc_cmp(const void *_a, const void *_b)
{
	struct ieee80211_channel *a = ((struct channel *)_a)->chan;
	struct ieee80211_channel *b = ((struct channel *)_b)->chan;
	struct ap_pc_params *params = ((struct channel *)_a)->params;
	struct ieee80211com *ic = params->vap->iv_ic;
	int res;

#define EVALUATE_CRITERIUM(name, ...) do {			\
	if ((res = ap_pc_cmp_##name(__VA_ARGS__)) != 0) 	\
		return res;					\
} while (0)

	EVALUATE_CRITERIUM(radar, a, b);
	EVALUATE_CRITERIUM(keepmode, params, a, b);
	EVALUATE_CRITERIUM(sc, ic, a, b);
	EVALUATE_CRITERIUM(rssi, params->ss->ss_priv, a, b);		/* useless? ap_pick_channel evaluates it anyway */
	EVALUATE_CRITERIUM(samechan, ic, a, b);
	EVALUATE_CRITERIUM(orig, (struct channel *)_a, (struct channel *)_b);

#undef EVALUATE_CRITERIUM
	return res;
}

static void
ap_pc_swap(void *a, void *b, int n)
{
	struct ieee80211_channel *t = ((struct channel *)a)->chan;
	int i;

	((struct channel *)a)->chan = ((struct channel *)b)->chan;
	((struct channel *)b)->chan = t;

	i = ((struct channel *)a)->orig;
	((struct channel *)a)->orig = ((struct channel *)b)->orig;
	((struct channel *)b)->orig = i;

	/* (struct channel *)x->params doesn't have to be swapped, because it is the same across all channels */
}

/*
 * Pick a quiet channel to use for ap operation.
 */
static struct ieee80211_channel *
ap_pick_channel(struct ieee80211_scan_state *ss, struct ieee80211vap *vap, u_int32_t flags)
{
	struct ieee80211com *ic = vap->iv_ic;
	int i, best_rssi;
	int ss_last = ss->ss_last;
	struct ieee80211_channel *best;
	struct ap_state *as = ss->ss_priv;
	struct channel chans[ss_last]; /* actually ss_last-1 is required */
	struct ap_pc_params params = { vap, ss, flags };

	for (i = 0; i < ss_last; i++) {
		chans[i].chan = ss->ss_chans[i];
		chans[i].orig = i;
		chans[i].params = &params;
	}

	sort(chans, ss_last, sizeof(*chans), ap_pc_cmp, ap_pc_swap);

	for (i = 0; i < ss_last; i++) {
		int chan = ieee80211_chan2ieee(ic, chans[i].chan);

		IEEE80211_DPRINTF(vap, IEEE80211_MSG_SCAN,
				"%s: channel %u, rssi %d, radar %d, cn %d, km %d\n",
				__func__, chan, as->as_maxrssi[chan], IEEE80211_IS_CHAN_RADAR(chans[i].chan),
				ic->ic_chan_nodes[chans[i].chan->ic_ieee],
				!!IEEE80211_ARE_CHANS_SAME_MODE(chans[i].chan, ic->ic_bsschan));
	}

	best = chans[0].chan;
	best_rssi = -1;

	for (i = 0; i < ss_last; i++) {
		int benefit = best_rssi - as->as_maxrssi[chans[i].chan->ic_ieee];
		int sta_assoc = ic->ic_sta_assoc;

		if (benefit <= 0)
			continue;

		if ((flags & IEEE80211_SCAN_KEEPMODE) && !IEEE80211_ARE_CHANS_SAME_MODE(chans[i].chan, ic->ic_bsschan))
				/* break the loop as the subsequent chans won't be better */
				break;

		if (!IEEE80211_ARE_CHANS_SAME_MODE(chans[i].chan, ic->ic_bsschan))
			/* break the loop as the subsequent chans won't be better */
			break;

		if (sta_assoc != 0) {
			int sl = ic->ic_cn_total - ic->ic_chan_nodes[chans[i].chan->ic_ieee]; /* count */
			if (ic->ic_sc_algorithm == IEEE80211_SC_LOOSE) {
				int sl_max = ic->ic_sc_sldg * benefit;
				sl = 1000 * sl / sta_assoc; /* permil */
		IEEE80211_DPRINTF(vap, IEEE80211_MSG_SCAN,
						"%s: chan %d, dB gained: %d, STAs lost: %d permil (max %d)\n", __func__,
						chans[i].chan->ic_ieee, benefit, sl, sl_max);
				if (sl > sl_max)
					continue;
			} else if (ic->ic_sc_algorithm == IEEE80211_SC_TIGHT ||
					ic->ic_sc_algorithm == IEEE80211_SC_STRICT) {
				if (sl > 0)
					/* break the loop as the subsequent chans won't be better */
			break;
		}
	}
		best = chans[i].chan;
		best_rssi = as->as_maxrssi[best->ic_ieee];
	}

	i = best->ic_ieee;
	IEEE80211_DPRINTF(vap, IEEE80211_MSG_SCAN,
			"%s: best: channel %u rssi %d\n",
			__func__, i, as->as_maxrssi[i]);
	return best;
}

static int
ap_end(struct ieee80211_scan_state *ss, struct ieee80211vap *vap,
   int (*action)(struct ieee80211vap *, const struct ieee80211_scan_entry *),
   u_int32_t flags)
{
	struct ap_state *as = ss->ss_priv;
	struct ieee80211com *ic = vap->iv_ic;
	struct ieee80211_channel *bestchan;

	KASSERT(vap->iv_opmode == IEEE80211_M_HOSTAP,
		("wrong opmode %u", vap->iv_opmode));

	bestchan = ap_pick_channel(ss, vap, flags);
	if (bestchan == NULL) {
		if (ss->ss_last > 0) {
			/* no suitable channel, should not happen */
			IEEE80211_DPRINTF(vap, IEEE80211_MSG_SCAN,
				"%s: no suitable channel! (should not happen)\n", __func__);
			/* XXX print something? */
		}
		return 0; /* restart scan */
	} else {
		struct ieee80211_scan_entry se;
		/* XXX: notify all VAPs? */
		/* if this is a dynamic turbo frequency , start with normal mode first */
		if (IEEE80211_IS_CHAN_TURBO(bestchan) && !IEEE80211_IS_CHAN_STURBO(bestchan)) {
			if ((bestchan = ieee80211_find_channel(ic, bestchan->ic_freq,
				bestchan->ic_flags & ~IEEE80211_CHAN_TURBO)) == NULL) {
				/* should never happen ?? */
				return 0;
			}
		}
		memset(&se, 0, sizeof(se));
		se.se_chan = bestchan;

		as->as_action = ss->ss_ops->scan_default;
		if (action)
			as->as_action = action;
		as->as_selbss = se;

		/* 
		 * Must defer action to avoid possible recursive call through 80211
		 * state machine, which would result in recursive locking.
		 */
		IEEE80211_SCHEDULE_TQUEUE(&as->as_actiontq);
		return 1;
	}
}

static void
ap_age(struct ieee80211_scan_state *ss)
{
	/* XXX is there anything meaningful to do? */
}

static int
ap_iterate(struct ieee80211_scan_state *ss,
	ieee80211_scan_iter_func *f, void *arg)
{
	/* NB: nothing meaningful we can do */
	return 0;
}

static void
ap_assoc_success(struct ieee80211_scan_state *ss,
	const u_int8_t macaddr[IEEE80211_ADDR_LEN])
{
	/* should not be called */
}

static void
ap_assoc_fail(struct ieee80211_scan_state *ss,
	const u_int8_t macaddr[IEEE80211_ADDR_LEN], int reason)
{
	/* should not be called */
}

/*
 * Default action to execute when a scan entry is found for ap
 * mode.  Return 1 on success, 0 on failure
 */
static int
ap_default_action(struct ieee80211vap *vap,
	const struct ieee80211_scan_entry *se)
{
	ieee80211_create_ibss(vap, se->se_chan);
	return 1;
}

static void
action_tasklet(IEEE80211_TQUEUE_ARG data)
{
	struct ieee80211_scan_state *ss = (struct ieee80211_scan_state *)data;
	struct ap_state *as = (struct ap_state *)ss->ss_priv;
	struct ieee80211vap *vap = ss->ss_vap;

	(*ss->ss_ops->scan_default)(vap, &as->as_selbss);
}

/*
 * Module glue.
 */
MODULE_AUTHOR("Errno Consulting, Sam Leffler");
MODULE_DESCRIPTION("802.11 wireless support: default ap scanner");
#ifdef MODULE_LICENSE
MODULE_LICENSE("Dual BSD/GPL");
#endif

static const struct ieee80211_scanner ap_default = {
	.scan_name		= "default",
	.scan_attach		= ap_attach,
	.scan_detach		= ap_detach,
	.scan_start		= ap_start,
	.scan_restart		= ap_restart,
	.scan_cancel		= ap_cancel,
	.scan_end		= ap_end,
	.scan_flush		= ap_flush,
	.scan_add		= ap_add,
	.scan_age		= ap_age,
	.scan_iterate		= ap_iterate,
	.scan_assoc_success	= ap_assoc_success,
	.scan_assoc_fail	= ap_assoc_fail,
	.scan_default		= ap_default_action,
};

static int __init
init_scanner_ap(void)
{
	ieee80211_scanner_register(IEEE80211_M_HOSTAP, &ap_default);
	return 0;
}
module_init(init_scanner_ap);

static void __exit
exit_scanner_ap(void)
{
	ieee80211_scanner_unregister_all(&ap_default);
}
module_exit(exit_scanner_ap);
