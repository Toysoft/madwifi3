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

#include "opt_ah.h"

#ifndef AUTOCONF_INCLUDED
#include <linux/config.h>
#endif
#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/skbuff.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/if_arp.h>
#include <linux/etherdevice.h>

#include "if_athvar.h"
#include "if_ath_d80211.h"
#include "if_ath.h"
#include "if_ath_pci.h"

#include <net/mac80211.h>


static struct {
	u_int	hal_mode;	/* hal phy mode */
	int	d80211_mode;	/* d80211 phy mode */
} ath_mode_map[] = {
	{ AR5K_MODE_11A,  MODE_IEEE80211A	},
	{ AR5K_MODE_11B,  MODE_IEEE80211B	},
	{ AR5K_MODE_11G,  MODE_IEEE80211G	},
	{ AR5K_MODE_TURBO, MODE_ATHEROS_TURBO	},
	{ AR5K_MODE_108G, MODE_ATHEROS_TURBOG	},
};


/**
 * ath_hal_mode_to_d80211_mode - Convert a hal mode to an IEEE80211 mode.
 * @hal_mode: hal hardware mode (AR5K_MODE_11A, AR5K_MODE_11B, ...)
 *
 * Returns ieee80211 hardware mode (MODE_IEEE80211A, MODE_IEEE80211B ...)
 */
static int
ath_hal_mode_to_d80211_mode(u_int hal_mode)
{
	int i;

	for (i = 0; i < sizeof(ath_mode_map) / sizeof(ath_mode_map[0]); i++) {
		if (ath_mode_map[i].hal_mode == hal_mode)
			return ath_mode_map[i].d80211_mode;
	}
	printk(KERN_ERR "Invalid mode.\n");
	return ath_mode_map[0].d80211_mode;
}


/**
 * ath_d80211_add_channels - Setup channel array for a given hardware mode.
 * @sc: device in question
 * @hw_mode: ieee80211 hardware mode (MODE_IEEE80211A, MODE_IEEE80211B ...)
 * @hal_chans: pointer to an array of channels from the hal
 * @hal_nchan: number of total channels in @hal_chans
 * @hal_flags: hal channel flags which identify which channels pertain to the
 *             hardware mode in question (the array @hal_chans includes all
 *             channels supported by the device).
 *
 * Returns 0 on success or < 0 on error.
 */
int
ath_d80211_add_channels(struct ath_softc *sc, int hw_mode,
	       		AR5K_CHANNEL *hal_chans, int hal_nchan, int hal_flags)
{
//	struct ath_hal *ah = sc->sc_ah;
	struct ieee80211_hw_mode *mode;
	int error = 0;
	int i;

	for (i = 0; i < sc->sc_num_modes ; i++) {
		if (sc->sc_hw_modes[i].mode == hw_mode)
			break;
	}

	if (i == sc->sc_num_modes) {
		if (sc->sc_num_modes == ATH_MAX_HW_MODES) { 
			DPRINTF(sc, ATH_DEBUG_ANY,
				"%s: no free mode elements\n", __func__);
			return -1;
		}
		mode = &sc->sc_hw_modes[sc->sc_num_modes];
	} else {
		DPRINTF(sc, ATH_DEBUG_ANY,
			"%s: mode %d already initialized\n", __func__, hw_mode);
		return -1;
	}

	for (i = 0; i < hal_nchan; i++) {
		AR5K_CHANNEL *c = &hal_chans[i];

		if ((c->channel_flags & CHANNEL_ALL) == hal_flags) {
			struct ieee80211_channel *channel;
		
			if (mode->num_channels == ATH_MAX_CHANNELS) {
				printk(KERN_ERR "channel list truncated\n");
				error = -E2BIG;
				goto done;
			}

			channel = &mode->channels[mode->num_channels];

			channel->chan = ath_hal_mhz2ieee(c->freq, c->channel_flags);
			channel->freq = c->freq;
			channel->val = hal_flags;
			/* ? = c->private_flags; FIXME */
			/* ? = c->minTxPower; FIXME */
			/* channel->flag = ? FIXME */	
//			channel->power_level = c->maxRegTxPower; /* ??? FIXME */
//			channel->antenna_max = c->maxTxPower; /* ??? FIXME */

			mode->num_channels++;
		}
	}

done:
	if (mode->num_channels != 0) {
		DPRINTF(sc, ATH_DEBUG_D80211, "%s: hal_chan %x hal_flags %x\n", __func__,
			hal_nchan, hal_flags);
		mode->mode = hw_mode;
		sc->sc_num_modes++;
	}

	return error;
}	


/**
 * ath_d80211_rate_setup - Setup a rate array for a given hardware mode.
 * @sc: device in question
 * @hal_mode: hal hardware mode (AR5K_MODE_11A, AR5K_MODE_11B, ...)
 * @rt: hal rate table for the mode in question
 * 
 * Returns 0 on success or < 0 on error.
 *
 * XXX: This happens on every channel change? locking? 
 */
int
ath_d80211_rate_setup(struct ath_softc *sc, u_int hal_mode,
		      const AR5K_RATE_TABLE *rt)
{
	struct ieee80211_hw_mode *mode;
	int hw_mode;
	struct ieee80211_rate *rates;
	int i;

	hw_mode = ath_hal_mode_to_d80211_mode(hal_mode);

	for (i = 0; i < sc->sc_num_modes ; i++) {
		if (sc->sc_hw_modes[i].mode == hw_mode)
			break;
	}

	if (i == sc->sc_num_modes) {
		printk(KERN_ERR "cannot find mode element.\n");
		return -1;
	}

	DPRINTF(sc, ATH_DEBUG_D80211, "%s: hal_mode %x\n", __func__, hal_mode);

	mode = &sc->sc_hw_modes[i];
	mode->num_rates = 0;
	rates = mode->rates;

	for (i = 0; i < rt->rate_count; i++) {

		if (mode->num_rates == ATH_MAX_RATES) {
			printk(KERN_ERR "rate list truncated\n");
			return -1;
		}

		rates[i].rate = rt->rates[i].rate_kbps / 100;
		rates[i].val = rt->rates[i].rate_code;

		rates[i].flags = rt->rates[i].modulation;

		/* FIXME rates[i].min_rssi_ack = ?; */
		/* FIXME rates[i].min_rssi_ack_delta = ?; */
		mode->num_rates++;
	}

	return 0;
}


static int
ath_d80211_reset(struct ieee80211_hw *hw)
{
	struct ath_softc *sc = hw->priv;
	return ath_reset(sc);
}


static int
ath_d80211_open(struct ieee80211_hw *hw)
{
	struct ath_softc *sc = hw->priv;
	int rv;

	DPRINTF(sc, ATH_DEBUG_D80211, "%s\n", __func__);

	rv = ath_init(sc);

	if (rv == 0)
		sc->sc_dev_open = 1;

	return rv;
}


static int
ath_d80211_stop(struct ieee80211_hw *hw)
{
	struct ath_softc *sc = hw->priv;

	DPRINTF(sc, ATH_DEBUG_D80211, "%s\n", __func__);

	sc->sc_dev_open = 0;

	return ath_stop(sc);
}


/**
 * ath_d80211_calc_bssid_mask - Calculate the required BSSID mask.
 *
 * Note: Caller must hold ATH_LOCK
 *
 * Returns 1 if the bssidmask changed otherwise returns 0.
 */
static int
ath_d80211_calc_bssid_mask(struct ieee80211_hw *hw)
{
	struct ath_softc *sc = hw->priv;
	int i, j;
	struct net_device *dev;
	unsigned char mask[ETH_ALEN];

	memset(mask, 0xff, ETH_ALEN);

	for (i = 0; i < sc->sc_num_bss; i++) {
		dev = dev_get_by_index(sc->sc_bss[i].ab_if_id);

		for (j = 0; j < ETH_ALEN; j++) {
			mask[j] &= ~(hw->wiphy->perm_addr[j] ^ dev->dev_addr[j]);
		}

		dev_put(dev);
	}

	if (memcmp(sc->sc_bssidmask, mask, ETH_ALEN)) {
		memcpy(sc->sc_bssidmask, mask, ETH_ALEN);
		return 1;
	}

	return 0;
}


static int
ath_d80211_add_interface(struct ieee80211_hw *hw,
			 struct ieee80211_if_init_conf *conf)
{
	struct ath_softc *sc = hw->priv;
	int error = 0;
	int reset;

	DPRINTF(sc, ATH_DEBUG_D80211, "%s: if_id %d, type %d\n", __func__,
		conf->if_id, conf->type);

	ATH_LOCK(sc);

	switch (conf->type) {
	case IEEE80211_IF_TYPE_STA:
		sc->sc_opmode = AR5K_M_STA;
		break;
	case IEEE80211_IF_TYPE_IBSS:
		sc->sc_opmode = AR5K_M_IBSS;
		break;
	case IEEE80211_IF_TYPE_MNTR:
		sc->sc_opmode = AR5K_M_MONITOR;
		break;
	case IEEE80211_IF_TYPE_AP:

		if (sc->sc_num_alloced_bss < sc->sc_num_bss + 1) {
			struct ath_bss *bss;
			struct ath_descdma bdma;
			ath_bufhead bbuf;
			struct ath_hal *ah = sc->sc_ah;
			int i;

			bss = kzalloc((sc->sc_num_bss + 1) * sizeof(bss[0]),
				      GFP_KERNEL);

			if (!bss) {
				error = -ENOMEM;
				goto done;
			}

			error = ath_descdma_setup(sc, &bdma, &bbuf, "beacon",
						  sc->sc_num_bss + 1, 1);

			if (error) {
				kfree(bss);				
				goto done;
			}

			spin_lock_bh(&sc->sc_bss_lock);

			ath_hal_stoptxdma(ah, sc->sc_bhalq);
			ath_descdma_cleanup(sc, &sc->sc_bdma, &sc->sc_bbuf,
					    BUS_DMA_TODEVICE);
			sc->sc_bdma = bdma;
			sc->sc_bbuf = bbuf;

			memcpy(bss, sc->sc_bss,
			       sc->sc_num_bss * sizeof(bss[0]));
			kfree(sc->sc_bss);
			sc->sc_bss = bss;
			sc->sc_bss[sc->sc_num_bss].ab_if_id = conf->if_id;
			sc->sc_num_bss++;

			for (i = 0; i < sc->sc_num_bss; i++) {
				sc->sc_bss[i].ab_bcbuf = STAILQ_FIRST(&sc->sc_bbuf);
				STAILQ_REMOVE_HEAD(&sc->sc_bbuf, bf_list);
			}

			spin_unlock_bh(&sc->sc_bss_lock);

			sc->sc_num_alloced_bss++;

		} else {

			spin_lock_bh(&sc->sc_bss_lock);
				
			sc->sc_bss[sc->sc_num_bss].ab_bcbuf = STAILQ_FIRST(&sc->sc_bbuf);
			STAILQ_REMOVE_HEAD(&sc->sc_bbuf, bf_list);

			sc->sc_bss[sc->sc_num_bss].ab_if_id = conf->if_id;
			sc->sc_num_bss++;

			spin_unlock_bh(&sc->sc_bss_lock);

		}

		sc->sc_opmode = AR5K_M_HOSTAP;
		sc->sc_beacons = 1;
		break;
	default:
		error = -EINVAL;
		goto done;
	}

	reset = ath_d80211_calc_bssid_mask(hw);

	if (reset)
		error = ath_reset(sc);

done:
	ATH_UNLOCK(sc);
	return error;
}


static void
ath_d80211_remove_interface(struct ieee80211_hw *hw,
			    struct ieee80211_if_init_conf *conf)
{
	struct ath_softc *sc = hw->priv;
	int i;

	DPRINTF(sc, ATH_DEBUG_D80211, "%s: if_id %d, type %d\n", __func__,
		conf->if_id, conf->type);

	ATH_LOCK(sc);

	switch (conf->type) {
	case IEEE80211_IF_TYPE_AP:

		for (i = 0; i < sc->sc_num_bss; i++) {
			if (sc->sc_bss[i].ab_if_id == conf->if_id)
				break;
		}

		if (i == sc->sc_num_bss) {
			printk(KERN_ERR "%s: remove cannot find if_id %d\n",
			       sc->name, conf->if_id);
			goto done;
		}

		spin_lock_bh(&sc->sc_bss_lock);
			
		STAILQ_INSERT_TAIL(&sc->sc_bbuf, sc->sc_bss[i].ab_bcbuf,
				   bf_list);

		memmove(&sc->sc_bss[i], &sc->sc_bss[i+1],
			(sc->sc_num_bss - i - 1) * sizeof(sc->sc_bss[0]));
		sc->sc_num_bss--;

		spin_unlock_bh(&sc->sc_bss_lock);

		break;
	}

	if (sc->sc_num_bss == 0)
		sc->sc_beacons = 0;


	if (ath_d80211_calc_bssid_mask(hw))
		ath_reset(sc);

done:
	ATH_UNLOCK(sc);
}


static int
ath_d80211_config(struct ieee80211_hw *hw, struct ieee80211_conf *conf)
{
	/* FIXME: more to configure */
	struct ath_softc *sc = hw->priv;
	AR5K_CHANNEL hchan;
	int ret;

	DPRINTF(sc, ATH_DEBUG_D80211, "%s\n", __func__);

	sc->sc_ieee80211_channel = conf->channel;
	sc->sc_mode = conf->phymode;
	sc->sc_beacon_interval = (conf->beacon_int * 1000) >> 10;

	if (sc->sc_shortslottime !=
	    !!(conf->flags & IEEE80211_CONF_SHORT_SLOT_TIME)) {
		sc->sc_shortslottime =
			!!(conf->flags & IEEE80211_CONF_SHORT_SLOT_TIME);
		sc->sc_updateslot = UPDATE;
	}

	if (!sc->sc_dev_open || !conf->radio_enabled)
		return 0;

	hchan.freq = conf->freq;
	hchan.channel_flags = conf->channel_val;

	if ((ret = ath_chan_set(sc, hchan)))
		return ret;

	return 0;
}


static int
ath_d80211_config_interface(struct ieee80211_hw *hw, int if_id,
			    struct ieee80211_if_conf *conf)
{
	struct ath_softc *sc = hw->priv;
	struct ath_hal *ah = sc->sc_ah;

	DPRINTF(sc, ATH_DEBUG_D80211, "%s\n", __func__);

	if (conf->bssid)
		ath_hal_setassocid(ah, conf->bssid, 0 /* FIXME: aid */);

	return ath_reset(sc);
}

static int
ath_d80211_set_key(struct ieee80211_hw *hw, set_key_cmd cmd, u8 *addr,
		   struct ieee80211_key_conf *key, int aid)
{
	struct ath_softc *sc = hw->priv;
	struct ath_hal *ah = sc->sc_ah;
	int ret = 0;
	u_int keyix;
	int i;

	DPRINTF(sc, ATH_DEBUG_D80211,
		"%s %s%s%s hw %d %s%s%s%s%s " MAC_FMT " WEP %x, %s%s\n",
		__func__,
		cmd == SET_KEY ? "SET_KEY" : "",
		cmd == DISABLE_KEY ? "DISABLE_KEY" : "",
		cmd == REMOVE_ALL_KEYS ? "REMOVE_ALL_KEYS" : "",
		key->hw_key_idx,
		key->alg == ALG_NONE ? "ALG_NONE" : "",
		key->alg == ALG_WEP  ? "ALG_WEP"  : "",
		key->alg == ALG_TKIP ? "ALG_TKIP" : "",
		key->alg == ALG_CCMP ? "ALG_CCMP" : "",
		key->alg == ALG_NULL ? "ALG_NULL" : "",
		MAC_ARG(addr),
		key->keyidx,
		key->flags & IEEE80211_KEY_DEFAULT_TX_KEY ?
			" DEFAULT_TX_KEY" : "",
		key->flags & IEEE80211_KEY_DEFAULT_WEP_ONLY ?
			" DEFAULT_WEP_ONLY" : "");
	ATH_LOCK(sc);

	switch (cmd) {
	case SET_KEY:
		switch (key->alg) {
		case ALG_WEP:
			if (!ath_hal_ciphersupported(ah, AR5K_CIPHER_WEP)) {
				ret = -1;
				goto done;
			}
			break;
		case ALG_TKIP:
			if (!ath_hal_ciphersupported(ah, AR5K_CIPHER_TKIP)) {
				ret = -1;
				goto done;
			}
			break;
		case ALG_CCMP:
			if (!ath_hal_ciphersupported(ah, AR5K_CIPHER_AES_CCM)) {
				ret = -1;
				goto done;
			}
			break;
		case ALG_NONE:
		case ALG_NULL:
			break;
		}

		keyix = ath_key_alloc(sc, key, addr);

		if (keyix == IEEE80211_KEYIX_NONE) {
			ret = -1;
			goto done;
		}

		key->hw_key_idx = keyix;

		if (!ath_keyset(sc, key, addr)) {

			ath_key_delete(sc, key);

			ret = -1;
			goto done;
		}

		key->flags &= ~IEEE80211_KEY_FORCE_SW_ENCRYPT;
		sc->sc_ath_keys[keyix].ak_alg = key->alg;
		break;

	case DISABLE_KEY:
		ath_key_delete(sc, key);
		break;
	case REMOVE_ALL_KEYS:
		for (i = 0; i < sc->sc_keymax; i++) {
			ath_hal_keyreset(ah, i);
			clrbit(sc->sc_keymap, i);
		}
		memset(sc->sc_ath_keys, 0, sizeof(sc->sc_ath_keys));
	default:
		ret = -EOPNOTSUPP;
		break;
	}

done:
	ATH_UNLOCK(sc);
	return ret;
}


static u64
ath_d80211_get_tsf(struct ieee80211_hw *hw)
{
	struct ath_softc *sc = hw->priv;
	struct ath_hal *ah = sc->sc_ah;

	DPRINTF(sc, ATH_DEBUG_D80211, "%s\n", __func__);

	return ath_hal_gettsf64(ah);
}


static void
ath_d80211_reset_tsf(struct ieee80211_hw *hw)
{
	struct ath_softc *sc = hw->priv;
	struct ath_hal *ah = sc->sc_ah;

	DPRINTF(sc, ATH_DEBUG_D80211, "%s\n", __func__);

	ath_hal_resettsf(ah);
}


static struct ieee80211_ops ath_d80211_ops = {
	.tx = ath_d80211_tx,
	.reset = ath_d80211_reset,
	.open = ath_d80211_open,
	.stop = ath_d80211_stop,
	.add_interface = ath_d80211_add_interface,
	.remove_interface = ath_d80211_remove_interface,
	.config = ath_d80211_config,
	.config_interface = ath_d80211_config_interface,
	.set_key = ath_d80211_set_key,
	.get_tsf = ath_d80211_get_tsf,
	.reset_tsf = ath_d80211_reset_tsf,
};


/**
 * ath_d80211_alloc - Allocate and initialize hardware structure
 * @priv_size: size of private data
 *
 * Returns a pointer to the newly allocated struct ath_softc on success.
 * Returns NULL on error.
 */
struct ath_softc *
ath_d80211_alloc(size_t priv_size)
{
	struct ieee80211_hw *hw;
	struct ieee80211_hw_mode *modes;
	struct ath_softc *sc;
	int i;

	hw = ieee80211_alloc_hw(priv_size, &ath_d80211_ops);

	if (!hw) {
		printk("ath_d80211: Failed to allocate hw\n");
		return NULL;
	}

	sc = hw->priv;
	sc->sc_hw = hw;

	DPRINTF(sc, ATH_DEBUG_D80211, "%s\n", __func__);

	hw->flags = IEEE80211_HW_HOST_GEN_BEACON |
		 IEEE80211_HW_RX_INCLUDES_FCS |
		 IEEE80211_HW_HOST_BROADCAST_PS_BUFFERING |
		 IEEE80211_HW_WEP_INCLUDE_IV |
		 IEEE80211_HW_DATA_NULLFUNC_ACK;
	hw->extra_tx_headroom = 2;
	hw->channel_change_time = 5000;
	hw->max_rssi = 127; /* FIXME: get a real value for this. */
	hw->queues = 1;
	sc->sc_num_modes = 0;

	modes = &sc->sc_hw_modes[0];

	for (i = 0; i < ATH_MAX_HW_MODES; i++) {
		modes[i].num_channels = 0;
		modes[i].channels = &sc->sc_channels[i * ATH_MAX_CHANNELS];
		modes[i].num_rates = 0;
		modes[i].rates = &sc->sc_ieee80211_rates[i * ATH_MAX_RATES];
	}

	sc->sc_opmode = AR5K_M_STA;

	spin_lock_init(&sc->sc_bss_lock);

	return sc;
}


/**
 * ath_d80211_free - Free memory allocated by ath_d80211_alloc()
 * @sc: A pointer returned by ath_d80211_alloc().
 */
void ath_d80211_free(struct ath_softc *sc)
{
	ieee80211_free_hw(sc->sc_hw);
}


int
ath_d80211_attach(struct ath_softc *sc)
{
	struct ieee80211_hw *hw = sc->sc_hw;
	struct pci_dev *pdev = (struct pci_dev *)sc->sc_bdev;
	unsigned int i;
	int rv = 0;

	DPRINTF(sc, ATH_DEBUG_D80211, "%s\n", __func__);

	for (i = 0; i < sc->sc_num_modes; i++) {
		ieee80211_register_hwmode(hw, &sc->sc_hw_modes[i]);
		printk(KERN_DEBUG "register hw_mode %d\n",sc->sc_hw_modes[i].mode);
	}

	SET_IEEE80211_DEV(hw, &pdev->dev);

	rv = ieee80211_register_hw(hw);
	if (rv)
		printk(KERN_ERR "%s: device registration failed.\n", sc->name);

	return rv;
}

void
ath_d80211_detach(struct ath_softc *sc)
{
	kfree(sc->sc_bss);
}
