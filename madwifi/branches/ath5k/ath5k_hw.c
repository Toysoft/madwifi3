/*-
 * Copyright (c) 2006-2007 Nick Kossifidis <mickflemm@gmail.com>
 *
 *  This file is free software: you can copy, redistribute and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 2 of the License, or (at
 *  your option) any later version.
 *
 *  This file is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * This file incorporates work covered by the following copyright and
 * permission notice:
 *
 *     Copyright (c) 2007 Jiri Slaby <jirislaby@gmail.com>
 *     Copyright (c) 2004, 2005, 2006, 2007 Reyk Floeter <reyk@openbsd.org>
 * 
 *     Permission to use, copy, modify, and distribute this software for
 *     any purpose with or without fee is hereby granted, provided that
 *     the above copyright notice and this permission notice appear in all
 *     copies.
 *
 *     THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 *     WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 *     WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 *     AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR
 *     CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS
 *     OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 *     NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 *     CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

/*
 * HAL interface for Atheros Wireless LAN devices.
 */

#include <linux/pci.h>
#include <linux/delay.h>

#include "ath5k.h"
#include "ath5k_reg.h"

/*Rate tables*/
static const struct ath5k_rate_table ath5k_rt_11a = AR5K_RATES_11A;
static const struct ath5k_rate_table ath5k_rt_11b = AR5K_RATES_11B;
static const struct ath5k_rate_table ath5k_rt_11g = AR5K_RATES_11G;
static const struct ath5k_rate_table ath5k_rt_turbo = AR5K_RATES_TURBO;
static const struct ath5k_rate_table ath5k_rt_xr = AR5K_RATES_XR;

/*Prototypes*/
static int ath5k_hw_nic_reset(struct ath5k_hw *, u32);
static int ath5k_hw_nic_wakeup(struct ath5k_hw *, int, bool);
static int ath5k_hw_setup_4word_tx_desc(struct ath5k_hw *, struct ath5k_desc *,
	unsigned int, unsigned int, enum ath5k_pkt_type, unsigned int,
	unsigned int, unsigned int, unsigned int, unsigned int, unsigned int,
	unsigned int, unsigned int);
static bool ath5k_hw_setup_xr_tx_desc(struct ath5k_hw *, struct ath5k_desc *,
	unsigned int, unsigned int, unsigned int, unsigned int, unsigned int,
	unsigned int);
static int ath5k_hw_fill_4word_tx_desc(struct ath5k_hw *, struct ath5k_desc *,
	unsigned int, bool, bool);
static int ath5k_hw_proc_4word_tx_status(struct ath5k_hw *, struct ath5k_desc *);
static int ath5k_hw_setup_2word_tx_desc(struct ath5k_hw *, struct ath5k_desc *,
	unsigned int, unsigned int, enum ath5k_pkt_type, unsigned int,
	unsigned int, unsigned int, unsigned int, unsigned int, unsigned int,
	unsigned int, unsigned int);
static int ath5k_hw_fill_2word_tx_desc(struct ath5k_hw *, struct ath5k_desc *,
	unsigned int, bool, bool);
static int ath5k_hw_proc_2word_tx_status(struct ath5k_hw *, struct ath5k_desc *);
static int ath5k_hw_proc_new_rx_status(struct ath5k_hw *, struct ath5k_desc *);
static int ath5k_hw_proc_old_rx_status(struct ath5k_hw *, struct ath5k_desc *);
static int ath5k_hw_get_capabilities(struct ath5k_hw *);

static int ath5k_eeprom_init(struct ath5k_hw *);
static int ath5k_eeprom_read_mac(struct ath5k_hw *, u8 *);

static int ath5k_hw_enable_pspoll(struct ath5k_hw *, u8 *, u16);
static int ath5k_hw_disable_pspoll(struct ath5k_hw *);

/*
 * Enable to overwrite the country code (use "00" for debug)
 */
#if 0
#define COUNTRYCODE "00"
#endif

/*******************\
  General Functions
\*******************/

/*
 * Calculate transmition time of a frame
 * TODO: Left here for combatibility, change it in ath5k
 * TODO: Is this really hardware dependent?
 */
static u16 
ath_hal_computetxtime(struct ath5k_hw *hal, const struct ath5k_rate_table *rates,
		u32 frame_length, u16 rate_index, bool short_preamble)
{
	const struct ath5k_rate *rate;
	u32 value;

	AR5K_ASSERT_ENTRY(rate_index, rates->rate_count);

	/*
	 * Get rate by index
	 */
	rate = &rates->rates[rate_index];

	/*
	 * Calculate the transmission time by operation (PHY) mode
	 */
	switch (rate->modulation) {
	case IEEE80211_RATE_CCK:
		/*
		 * CCK / DS mode (802.11b)
		 */
		value = AR5K_CCK_TX_TIME(rate->rate_kbps, frame_length,
				(short_preamble && 
				 (rate->modulation == IEEE80211_RATE_CCK_2)));
		break;

	case IEEE80211_RATE_OFDM:
		/*
		 * Orthogonal Frequency Division Multiplexing
		 */
		if (AR5K_OFDM_NUM_BITS_PER_SYM(rate->rate_kbps) == 0)
			return 0;
		value = AR5K_OFDM_TX_TIME(rate->rate_kbps, frame_length);
		break;

	case IEEE80211_RATE_TURBO:
		/*
		 * Orthogonal Frequency Division Multiplexing
		 * Atheros "Turbo Mode" (doubled rates)
		 */
		if (AR5K_TURBO_NUM_BITS_PER_SYM(rate->rate_kbps) == 0)
			return 0;
		value = AR5K_TURBO_TX_TIME(rate->rate_kbps, frame_length);
		break;

	case MODULATION_XR:
		/*
		 * Orthogonal Frequency Division Multiplexing
		 * Atheros "eXtended Range" (XR)
		 */
		if (AR5K_XR_NUM_BITS_PER_SYM(rate->rate_kbps) == 0)
			return 0;
		value = AR5K_XR_TX_TIME(rate->rate_kbps, frame_length);
		break;

	default:
		return 0;
	}

	return value;
}

/*
 * Functions used internaly
 */

static inline unsigned int ath5k_hw_htoclock(unsigned int usec, bool turbo)
{
	return turbo == true ? (usec * 80) : (usec * 40);
}

static inline unsigned int ath5k_hw_clocktoh(unsigned int clock, bool turbo)
{
	return turbo == true ? (clock / 80) : (clock / 40);
}

/*
 * Check if a register write has been completed
 */
int ath5k_hw_register_timeout(struct ath5k_hw *hal, u32 reg, u32 flag, u32 val,
		bool is_set)
{
	int i;
	u32 data;

	for (i = AR5K_TUNE_REGISTER_TIMEOUT; i > 0; i--) {
		data = ath5k_hw_reg_read(hal, reg);
		if ((is_set == true) && (data & flag))
			break;
		else if ((data & flag) == val)
			break;
		udelay(15);
	}

	return (i <= 0) ? -EAGAIN : 0;
}

static const char *
ath5k_hw_get_part_name(enum ath5k_srev_type type, u_int32_t val)
{
	struct ath5k_srev_name names[] = AR5K_SREV_NAME;
	const char *name = "xxxx";
	int i;

	for (i = 0; i < ARRAY_SIZE(names); i++) {
		if (names[i].sr_type != type ||
		    names[i].sr_val == AR5K_SREV_UNKNOWN)
			continue;
		if ((val & 0xff) < names[i + 1].sr_val) {
			name = names[i].sr_name;
			break;
		}
	}

	return (name);
}

/***************************************\
	Attach/Detach Functions
\***************************************/

/*
 * Check if the device is supported and initialize the needed structs
 */
struct ath5k_hw *ath5k_hw_attach(u16 device, u8 mac_version, void *sc,
		void __iomem *sh)
{
	struct ath5k_hw *hal;
	u8 mac[ETH_ALEN];
	int ret;
	u32 srev;

	/*If we passed the test malloc a hal struct*/
	hal = kzalloc(sizeof(struct ath5k_hw), GFP_KERNEL);
	if (hal == NULL) {
		ret = -ENOMEM;
		AR5K_PRINT("out of memory\n");
		goto err;
	}

	hal->ah_sc = sc;
	hal->ah_sh = sh;

	/*
	 * HAL information
	 */

	/* Regulation Stuff */
	hal->ah_country_code = AR5K_TUNE_CTRY;
	ath5k_get_regdomain(hal);

	hal->ah_op_mode = IEEE80211_IF_TYPE_STA;
	hal->ah_radar.r_enabled = AR5K_TUNE_RADAR_ALERT;
	hal->ah_turbo = false;
	hal->ah_txpower.txp_tpc = AR5K_TUNE_TPC_TXPOWER;
	hal->ah_imr = 0;
	hal->ah_atim_window = 0;
	hal->ah_aifs = AR5K_TUNE_AIFS;
	hal->ah_cw_min = AR5K_TUNE_CWMIN;
	hal->ah_limit_tx_retries = AR5K_INIT_TX_RETRY;
	hal->ah_software_retry = false;
	hal->ah_ant_diversity = AR5K_TUNE_ANT_DIVERSITY;

	/*
	 * Set the mac version based on the pci id
	 */
	hal->ah_version = mac_version;

	/*
	 * Set the mac revision based by reading SREV
	 */
	srev = ath5k_hw_reg_read(hal, AR5K_SREV);
	hal->ah_mac_srev = srev;
	hal->ah_mac_version = AR5K_REG_MS(srev, AR5K_SREV_VER);
	hal->ah_mac_revision = AR5K_REG_MS(srev, AR5K_SREV_REV);

	/* Return on unsupported devices */
	if((srev >= AR5K_SREV_VER_AR5416) || ((srev >= AR5K_SREV_VER_AR2424)
					&& (srev < AR5K_SREV_VER_AR5413))){
		printk(KERN_ERR "ath5k: Device not supported (0x%x)\n", srev);
		ret = -ENODEV;
		goto err_free;
	}

	switch (srev) {
		case AR5K_SREV_VER_AR5413:
		case AR5K_SREV_VER_AR5414:
			/*
			 * Known single chip solutions
			 */
			hal->ah_single_chip = true;
			break;
		default:
			/*
			 * Multi chip solutions
			 */
			hal->ah_single_chip = false;
			break;
	}

	/*Fill the hal struct with the needed functions*/
	if (hal->ah_version == AR5K_AR5212)
		hal->ah_magic = AR5K_EEPROM_MAGIC_5212;
	else if (hal->ah_version == AR5K_AR5211)
		hal->ah_magic = AR5K_EEPROM_MAGIC_5211;

	if (hal->ah_version == AR5K_AR5212) {
		hal->ah_setup_tx_desc = ath5k_hw_setup_4word_tx_desc;
		hal->ah_setup_xtx_desc = ath5k_hw_setup_xr_tx_desc;
		hal->ah_fill_tx_desc = ath5k_hw_fill_4word_tx_desc;
		hal->ah_proc_tx_desc = ath5k_hw_proc_4word_tx_status;
	} else {
		hal->ah_setup_tx_desc = ath5k_hw_setup_2word_tx_desc;
		hal->ah_setup_xtx_desc = ath5k_hw_setup_xr_tx_desc;
		hal->ah_fill_tx_desc = ath5k_hw_fill_2word_tx_desc;
		hal->ah_proc_tx_desc = ath5k_hw_proc_2word_tx_status;
	}

	if (hal->ah_version == AR5K_AR5212)
		hal->ah_proc_rx_desc = ath5k_hw_proc_new_rx_status;
	else if (hal->ah_version <= AR5K_AR5211)
		hal->ah_proc_rx_desc = ath5k_hw_proc_old_rx_status;

	/* Bring device out of sleep and reset it's units */
	ret = ath5k_hw_nic_wakeup(hal, AR5K_INIT_MODE, true);
	if (ret)
		goto err_free;

	/* Get PHY and RADIO revisions */
	hal->ah_phy_revision = ath5k_hw_reg_read(hal, AR5K_PHY_CHIP_ID) &
			0xffffffff;
	hal->ah_radio_5ghz_revision = ath5k_hw_radio_revision(hal,
			CHANNEL_5GHZ);

	if (hal->ah_version == AR5K_AR5210)
		hal->ah_radio_2ghz_revision = 0;
	else
		hal->ah_radio_2ghz_revision = ath5k_hw_radio_revision(hal,
				CHANNEL_2GHZ);

	/* Single chip radio */
	if (hal->ah_radio_2ghz_revision == hal->ah_radio_5ghz_revision)
		hal->ah_radio_2ghz_revision = 0;

	/* Identify the radio chip*/
	if (hal->ah_version == AR5K_AR5210)
		hal->ah_radio = AR5K_RF5110;
	else
		hal->ah_radio = 
			(hal->ah_radio_5ghz_revision < AR5K_SREV_RAD_5112) ? 
			AR5K_RF5111 : AR5K_RF5112;

	hal->ah_phy = AR5K_PHY(0);

	/* Set MAC to bcast: ff:ff:ff:ff:ff:ff, this is using 'mac' as a
	 * temporary variable for setting our BSSID. Right bellow we update
	 * it with ath5k_hw_get_lladdr() */
	memset(mac, 0xff, ETH_ALEN);
	ath5k_hw_set_associd(hal, mac, 0);

	ath5k_hw_get_lladdr(hal, mac);
	ath5k_hw_set_opmode(hal);

#ifdef AR5K_DEBUG
	ath5k_hw_dump_state(hal);
#endif

	/*
	 * Get card capabilities, values, ...
	 */

	ret = ath5k_eeprom_init(hal);
	if (ret) {
		AR5K_PRINT("unable to init EEPROM\n");
		goto err_free;
	}

	/* Get misc capabilities */
	ret = ath5k_hw_get_capabilities(hal);
	if (ret) {
		AR5K_PRINTF("unable to get device capabilities: 0x%04x\n",
				device);
		goto err_free;
	}

	/* Get MAC address */
	ret = ath5k_eeprom_read_mac(hal, mac);
	if (ret) {
		AR5K_PRINTF("unable to read address from EEPROM: 0x%04x\n",
				device);
		goto err_free;
	}

	ath5k_hw_set_lladdr(hal, mac);

	ath5k_hw_set_rfgain_opt(hal);

	printk(KERN_INFO "ath5k: MAC revision: %s (0x%x)\n",
		ath5k_hw_get_part_name(AR5K_VERSION_VER,hal->ah_mac_srev),
					hal->ah_mac_srev);
	if(test_bit(MODE_IEEE80211B,hal->ah_capabilities.cap_mode) &&
	test_bit(MODE_IEEE80211A,hal->ah_capabilities.cap_mode)){
		printk(KERN_INFO "ath5k: PHY revision: %s (0x%x)\n",
			ath5k_hw_get_part_name(AR5K_VERSION_RAD,
						hal->ah_radio_5ghz_revision),
						hal->ah_radio_5ghz_revision);
	}
	if(test_bit(MODE_IEEE80211B,hal->ah_capabilities.cap_mode) &&
	!test_bit(MODE_IEEE80211A,hal->ah_capabilities.cap_mode)){
		printk(KERN_INFO "ath5k: 2Ghz PHY revision: %s (0x%x)\n",
			ath5k_hw_get_part_name(AR5K_VERSION_RAD,
						hal->ah_radio_2ghz_revision),
						hal->ah_radio_2ghz_revision);
	}
	if(!test_bit(MODE_IEEE80211B,hal->ah_capabilities.cap_mode) &&
	test_bit(MODE_IEEE80211A,hal->ah_capabilities.cap_mode)){
		printk(KERN_INFO "ath5k: 5Ghz PHY revision: %s (0x%x)\n",
			ath5k_hw_get_part_name(AR5K_VERSION_RAD,
						hal->ah_radio_5ghz_revision),
						hal->ah_radio_5ghz_revision);
	}
	printk(KERN_INFO "ath5k: EEPROM version: %x.%x\n",
		(hal->ah_ee_version & 0xF000) >> 12, hal->ah_ee_version & 0xFFF);

	return hal;
err_free:
	kfree(hal);
err:
	return ERR_PTR(ret);
}

/*
 * Bring up MAC + PHY Chips
 */
static int ath5k_hw_nic_wakeup(struct ath5k_hw *hal, int flags, bool initial)
{
	u32 turbo, mode, clock;
	int ret;

	turbo = 0;
	mode = 0;
	clock = 0;

	AR5K_TRACE;

	if (hal->ah_version != AR5K_AR5210) {
		/*
		 * Get channel mode flags
		 */

		if (hal->ah_radio >= AR5K_RF5112) {
			mode = AR5K_PHY_MODE_RAD_RF5112;
			clock = AR5K_PHY_PLL_RF5112;
		} else {
			mode = AR5K_PHY_MODE_RAD_RF5111;	/*Zero*/
			clock = AR5K_PHY_PLL_RF5111;		/*Zero*/
		}

		if (flags & CHANNEL_2GHZ) {
			mode |= AR5K_PHY_MODE_FREQ_2GHZ;
			clock |= AR5K_PHY_PLL_44MHZ;

			if (flags & CHANNEL_CCK) {
				mode |= AR5K_PHY_MODE_MOD_CCK;
			} else if (flags & CHANNEL_OFDM) {
				/* XXX Dynamic OFDM/CCK is not supported by the
				 * AR5211 so we set MOD_OFDM for plain g (no
				 * CCK headers) operation. We need to test
				 * this, 5211 might support ofdm-only g after
				 * all, there are also initial register values
				 * in the code for g mode (see ath5k_hw.h). */
				if (hal->ah_version == AR5K_AR5211)
					mode |= AR5K_PHY_MODE_MOD_OFDM;
				else
					mode |= AR5K_PHY_MODE_MOD_DYN;
			} else {
				AR5K_PRINT("invalid radio modulation mode\n");
				return -EINVAL;
			}
		} else if (flags & CHANNEL_5GHZ) {
			mode |= AR5K_PHY_MODE_FREQ_5GHZ;
			clock |= AR5K_PHY_PLL_40MHZ;

			if (flags & CHANNEL_OFDM)
				mode |= AR5K_PHY_MODE_MOD_OFDM;
			else {
				AR5K_PRINT("invalid radio modulation mode\n");
				return -EINVAL;
			}
		} else {
			AR5K_PRINT("invalid radio frequency mode\n");
			return -EINVAL;
		}

		if (flags & CHANNEL_TURBO)
			turbo = AR5K_PHY_TURBO_MODE | AR5K_PHY_TURBO_SHORT;
	} else { /* Reset and wakeup the device */
		if (initial == true) {
			/* ...reset hardware */
			if (ath5k_hw_nic_reset(hal, AR5K_RESET_CTL_PCI)) {
				AR5K_PRINT("failed to reset the PCI chipset\n");
				return -EIO;
			}

			mdelay(1);
		}

		/* ...wakeup */
		ret = ath5k_hw_set_power(hal, AR5K_PM_AWAKE, true, 0);
		if (ret) {
			AR5K_PRINT("failed to resume the MAC Chip\n");
			return ret;
		}

		/* ...enable Atheros turbo mode if requested */
		if (flags & CHANNEL_TURBO)
			ath5k_hw_reg_write(hal, AR5K_PHY_TURBO_MODE,
					AR5K_PHY_TURBO);

		/* ...reset chipset */
		if (ath5k_hw_nic_reset(hal, AR5K_RESET_CTL_CHIP)) {
			AR5K_PRINT("failed to reset the AR5210 chipset\n");
			return -EIO;
		}

		mdelay(1);
	}

	/* ...reset chipset and PCI device */
	if ((hal->ah_single_chip == false) && ath5k_hw_nic_reset(hal,
				AR5K_RESET_CTL_CHIP | AR5K_RESET_CTL_PCI)) {
		AR5K_PRINT("failed to reset the MAC Chip + PCI\n");
		return -EIO;
	}

	if (hal->ah_version == AR5K_AR5210)
		udelay(2300);

	/* ...wakeup */
	ret = ath5k_hw_set_power(hal, AR5K_PM_AWAKE, true, 0);
	if (ret) {
		AR5K_PRINT("failed to resume the MAC Chip\n");
		return ret;
	}

	/* ...final warm reset */
	if (ath5k_hw_nic_reset(hal, 0)) {
		AR5K_PRINT("failed to warm reset the MAC Chip\n");
		return -EIO;
	}

	if (hal->ah_version != AR5K_AR5210) {
		/* ...set the PHY operating mode */
		ath5k_hw_reg_write(hal, clock, AR5K_PHY_PLL);
		udelay(300);

		ath5k_hw_reg_write(hal, mode, AR5K_PHY_MODE);
		ath5k_hw_reg_write(hal, turbo, AR5K_PHY_TURBO);
	}

	return 0;
}

/*
 * Get the rate table for a specific operation mode
 */
const struct ath5k_rate_table *ath5k_hw_get_rate_table(struct ath5k_hw *hal,
		unsigned int mode)
{
	AR5K_TRACE;

	if (!test_bit(mode, hal->ah_capabilities.cap_mode))
		return NULL;

	/* Get rate tables */
	switch (mode) {
	case MODE_IEEE80211A:
		return &ath5k_rt_11a;
	case MODE_ATHEROS_TURBO:
		return &ath5k_rt_turbo;
	case MODE_IEEE80211B:
		return &ath5k_rt_11b;
	case MODE_IEEE80211G:
		return &ath5k_rt_11g;
	case MODE_ATHEROS_TURBOG:
		return &ath5k_rt_xr;
	}

	return NULL;
}

/*
 * Free the hal struct
 */
void ath5k_hw_detach(struct ath5k_hw *hal)
{
	AR5K_TRACE;

	if (hal->ah_rf_banks != NULL)
		kfree(hal->ah_rf_banks);

	/* assume interrupts are down */
	kfree(hal);
}

/*******************************\
	Reset Functions
\*******************************/

/*
 * Main reset function
 */
int ath5k_hw_reset(struct ath5k_hw *hal, enum ieee80211_if_types op_mode,
	struct ieee80211_channel *channel, bool change_channel)
{
	const struct ath5k_rate_table *rt;
	struct ath5k_eeprom_info *ee = &hal->ah_capabilities.cap_eeprom;
	u32 data, noise_floor, s_seq, s_ant, s_led[3];
	u8 mac[ETH_ALEN];
	unsigned int i, mode, freq, ee_mode, ant[2];
	int ret;

	AR5K_TRACE;

	s_seq = 0;
	s_ant = 1;
	ee_mode = 0;
	freq = 0;
	mode = 0;

	/*
	 * Save some registers before a reset
	 */
	/*DCU/Antenna selection not available on 5210*/
	if (hal->ah_version != AR5K_AR5210) {
		if (change_channel == true) {
			/* Seq number for queue 0 -do this for all queues ? */
			s_seq = ath5k_hw_reg_read(hal,
					AR5K_QUEUE_DFS_SEQNUM(0));
			/*Default antenna*/
			s_ant = ath5k_hw_reg_read(hal, AR5K_DEFAULT_ANTENNA);
		}
	}

	/*GPIOs*/
	s_led[0] = ath5k_hw_reg_read(hal, AR5K_PCICFG) & AR5K_PCICFG_LEDSTATE;
	s_led[1] = ath5k_hw_reg_read(hal, AR5K_GPIOCR);
	s_led[2] = ath5k_hw_reg_read(hal, AR5K_GPIODO);

	if (change_channel == true && hal->ah_rf_banks != NULL)
		ath5k_hw_get_rf_gain(hal);


	/*Wakeup the device*/
	ret = ath5k_hw_nic_wakeup(hal, channel->val, false);
	if (ret)
		return ret;

	/*
	 * Initialize operating mode
	 */
	hal->ah_op_mode = op_mode;

	/*
	 * 5111/5112 Settings
	 * 5210 only comes with RF5110
	 */
	if (hal->ah_version != AR5K_AR5210) {
		if ((hal->ah_radio != AR5K_RF5111) &&
				(hal->ah_radio != AR5K_RF5112)) {
			AR5K_PRINTF("invalid phy radio: %u\n", hal->ah_radio);
			return -EINVAL;
		}

		switch (channel->val & CHANNEL_MODES) {
		case CHANNEL_A:
			mode = AR5K_INI_VAL_11A;
			freq = AR5K_INI_RFGAIN_5GHZ;
			ee_mode = AR5K_EEPROM_MODE_11A;
			break;
		case CHANNEL_B:
			mode = AR5K_INI_VAL_11B;
			freq = AR5K_INI_RFGAIN_2GHZ;
			ee_mode = AR5K_EEPROM_MODE_11B;
			break;
		/* Is this ok on 5211 too ? */
		case CHANNEL_G:
			mode = AR5K_INI_VAL_11G;
			freq = AR5K_INI_RFGAIN_2GHZ;
			ee_mode = AR5K_EEPROM_MODE_11G;
			break;
		case CHANNEL_T:
			mode = AR5K_INI_VAL_11A_TURBO;
			freq = AR5K_INI_RFGAIN_5GHZ;
			ee_mode = AR5K_EEPROM_MODE_11A;
			break;
		/*Is this ok on 5211 too ?*/
		case CHANNEL_TG:
			mode = AR5K_INI_VAL_11G_TURBO;
			freq = AR5K_INI_RFGAIN_2GHZ;
			ee_mode = AR5K_EEPROM_MODE_11G;
			break;
		case CHANNEL_XR:
			if (hal->ah_version == AR5K_AR5211) {
				AR5K_PRINTF("XR mode not available on 5211");
				return -EINVAL;
			}
			mode = AR5K_INI_VAL_XR;
			freq = AR5K_INI_RFGAIN_5GHZ;
			ee_mode = AR5K_EEPROM_MODE_11A;
			break;
		default:
			AR5K_PRINTF("invalid channel: %d\n", channel->freq);
			return -EINVAL;
		}

		/* PHY access enable */
		ath5k_hw_reg_write(hal, AR5K_PHY_SHIFT_5GHZ, AR5K_PHY(0));
	}

	ath5k_hw_write_initvals(hal, mode, change_channel);

	/*
	 * 5211/5212 Specific
	 */
	if (hal->ah_version != AR5K_AR5210) {
		/*
		 * Write initial RF gain settings
		 * This should work for both 5111/5112
		 */
		ret = ath5k_hw_rfgain(hal, freq);
		if (ret)
			return ret;

		mdelay(1);

		/*
		 * Set rate duration table on 5212
		 */
		if (hal->ah_version == AR5K_AR5212) {
			/*For 802.11b*/
			if (!(channel->val & CHANNEL_B)) {
				/*Get rate table for this operation mode*/
				rt = ath5k_hw_get_rate_table(hal,
						MODE_IEEE80211B);

				/*Write rate duration table*/
				for (i = 0; i < rt->rate_count; i++) {
					data = AR5K_RATE_DUR(rt->rates[i].rate_code);
					ath5k_hw_reg_write(hal,
							ath_hal_computetxtime(hal, rt,
								14, rt->rates[i].control_rate,
								false), data);
					if (HAS_SHPREAMBLE(i))
						ath5k_hw_reg_write(hal,
								ath_hal_computetxtime(hal,
									rt, 14,
									rt->rates[i].control_rate,
									false), 
								data + 
								(AR5K_SET_SHORT_PREAMBLE << 2));
				}

			} else {
			/* For 802.11a/g Turbo/XR mode (AR5K_MODE_XR here is
			 * O.K. for both a/g - OFDM) */
				/* Get rate table for this operation mode */
				rt = ath5k_hw_get_rate_table(hal,
						channel->val & CHANNEL_TURBO ?
						MODE_ATHEROS_TURBO : 
						MODE_ATHEROS_TURBOG);

				/* Write rate duration table */
				for (i = 0; i < rt->rate_count; i++)
					ath5k_hw_reg_write(hal,
							ath_hal_computetxtime(hal, rt,
								14, rt->rates[i].control_rate,
								false),
							AR5K_RATE_DUR(rt->rates[i].rate_code));
			}
		}

		/* Fix for first revision of the RF5112 RF chipset */
		if ((hal->ah_radio >= AR5K_RF5112) &&
				(hal->ah_radio_5ghz_revision < 
				 AR5K_SREV_RAD_5112A)) {
			ath5k_hw_reg_write(hal, AR5K_PHY_CCKTXCTL_WORLD,
					AR5K_PHY_CCKTXCTL);
			if (channel->val & CHANNEL_A)
				data = 0xffb81020;
			else
				data = 0xffb80d20;
			ath5k_hw_reg_write(hal, data, AR5K_PHY_FRAME_CTL);
		}

		/*
		 * Set TX power (XXX use txpower from net80211)
		 */
		ret = ath5k_hw_txpower(hal, channel, AR5K_TUNE_DEFAULT_TXPOWER);
		if (ret)
			return ret;

		/*
		 * Write RF registers
		 * TODO:Does this work on 5211 (5111) ?
		 */
		ret = ath5k_hw_rfregs(hal, channel, mode);
		if (ret)
			return ret;

		/*
		 * Configure additional registers
		 */

		/* Write OFDM timings on 5212*/
		if (hal->ah_version == AR5K_AR5212) {
			if (channel->val & CHANNEL_OFDM) {
				u32 coef_scaled, coef_exp, coef_man,
				    ds_coef_exp, ds_coef_man, clock;

				clock = channel->val & CHANNEL_T ? 80 : 40;
				coef_scaled = ((5 * (clock << 24)) / 2) /
					channel->freq;

				for (coef_exp = 31; coef_exp > 0; coef_exp--)
					if ((coef_scaled >> coef_exp) & 0x1)
						break;

				if (!coef_exp)
					return -EINVAL;

				coef_exp = 14 - (coef_exp - 24);
				coef_man = coef_scaled +
					(1 << (24 - coef_exp - 1));
				ds_coef_man = coef_man >> (24 - coef_exp);
				ds_coef_exp = coef_exp - 16;

				AR5K_REG_WRITE_BITS(hal, AR5K_PHY_TIMING_3,
						AR5K_PHY_TIMING_3_DSC_MAN, ds_coef_man);
				AR5K_REG_WRITE_BITS(hal, AR5K_PHY_TIMING_3,
						AR5K_PHY_TIMING_3_DSC_EXP, ds_coef_exp);
			}
		}

		/* Enable/disable 802.11b mode on 5111
		 * (enable 2111 frequency converter + CCK) */
		if (hal->ah_radio == AR5K_RF5111) {
			if (channel->val & CHANNEL_B)
				AR5K_REG_ENABLE_BITS(hal, AR5K_TXCFG,
					AR5K_TXCFG_B_MODE);
			else
				AR5K_REG_DISABLE_BITS(hal, AR5K_TXCFG,
					AR5K_TXCFG_B_MODE);
		}

		/* Set antenna mode */
		AR5K_REG_MASKED_BITS(hal, AR5K_PHY(0x44),
				hal->ah_antenna[ee_mode][0], 0xfffffc06);

		if (freq == AR5K_INI_RFGAIN_2GHZ)
			ant[0] = ant[1] = AR5K_ANT_FIXED_B;
		else
			ant[0] = ant[1] = AR5K_ANT_FIXED_A;

		ath5k_hw_reg_write(hal, hal->ah_antenna[ee_mode][ant[0]],
				AR5K_PHY_ANT_SWITCH_TABLE_0);
		ath5k_hw_reg_write(hal, hal->ah_antenna[ee_mode][ant[1]],
				AR5K_PHY_ANT_SWITCH_TABLE_1);

		/* Commit values from EEPROM */
		if (hal->ah_radio == AR5K_RF5111)
			AR5K_REG_WRITE_BITS(hal, AR5K_PHY_FRAME_CTL,
					AR5K_PHY_FRAME_CTL_TX_CLIP, 
					ee->ee_tx_clip);

		ath5k_hw_reg_write(hal,
				AR5K_PHY_NF_SVAL(ee->ee_noise_floor_thr[ee_mode]),
				AR5K_PHY(0x5a));

		AR5K_REG_MASKED_BITS(hal, AR5K_PHY(0x11),
				(ee->ee_switch_settling[ee_mode] << 7) & 0x3f80,
				0xffffc07f);
		AR5K_REG_MASKED_BITS(hal, AR5K_PHY(0x12),
				(ee->ee_ant_tx_rx[ee_mode] << 12) & 0x3f000,
				0xfffc0fff);
		AR5K_REG_MASKED_BITS(hal, AR5K_PHY(0x14),
				(ee->ee_adc_desired_size[ee_mode] & 0x00ff) |
				((ee->ee_pga_desired_size[ee_mode] << 8) & 0xff00),
				0xffff0000);

		ath5k_hw_reg_write(hal,
				(ee->ee_tx_end2xpa_disable[ee_mode] << 24) |
				(ee->ee_tx_end2xpa_disable[ee_mode] << 16) |
				(ee->ee_tx_frm2xpa_enable[ee_mode] << 8) |
				(ee->ee_tx_frm2xpa_enable[ee_mode]), 
				AR5K_PHY(0x0d));

		AR5K_REG_MASKED_BITS(hal, AR5K_PHY(0x0a),
				ee->ee_tx_end2xlna_enable[ee_mode] << 8, 
				0xffff00ff);
		AR5K_REG_MASKED_BITS(hal, AR5K_PHY(0x19),
				(ee->ee_thr_62[ee_mode] << 12) & 0x7f000, 
				0xfff80fff);
		AR5K_REG_MASKED_BITS(hal, AR5K_PHY(0x49), 4, 0xffffff01);

		AR5K_REG_ENABLE_BITS(hal, AR5K_PHY_IQ,
			AR5K_PHY_IQ_CORR_ENABLE |
			(ee->ee_i_cal[ee_mode] << AR5K_PHY_IQ_CORR_Q_I_COFF_S) |
			ee->ee_q_cal[ee_mode]);

		if (hal->ah_ee_version >= AR5K_EEPROM_VERSION_4_1)
			AR5K_REG_WRITE_BITS(hal, AR5K_PHY_GAIN_2GHZ,
					AR5K_PHY_GAIN_2GHZ_MARGIN_TXRX,
					ee->ee_margin_tx_rx[ee_mode]);
	} else {
		mdelay(1);
		/* Disable phy and wait */
		ath5k_hw_reg_write(hal, AR5K_PHY_ACT_DISABLE, AR5K_PHY_ACT);
		mdelay(1);
	}

	/*
	 * Restore saved values
	 */
	/*DCU/Antenna selection not available on 5210*/
	if (hal->ah_version != AR5K_AR5210) {
		ath5k_hw_reg_write(hal, s_seq, AR5K_QUEUE_DFS_SEQNUM(0));
		ath5k_hw_reg_write(hal, s_ant, AR5K_DEFAULT_ANTENNA);
	}
	
	AR5K_REG_ENABLE_BITS(hal, AR5K_PCICFG, s_led[0]);
	ath5k_hw_reg_write(hal, s_led[1], AR5K_GPIOCR);
	ath5k_hw_reg_write(hal, s_led[2], AR5K_GPIODO);

	/*
	 * Misc.
	 */
	memset(mac, 0xff, ETH_ALEN);
	ath5k_hw_set_associd(hal, mac, 0);
	ath5k_hw_set_opmode(hal);
	/*PISR/SISR Not available on 5210*/
	if (hal->ah_version != AR5K_AR5210) {
		ath5k_hw_reg_write(hal, 0xffffffff, AR5K_PISR);
		/* XXX: AR5K_RSSI_THR has masks and shifts defined for it, so
		 * direct write using ath5k_hw_reg_write seems wrong. Test with:
		 * AR5K_REG_WRITE_BITS(hal, AR5K_RSSI_THR,
		 *   AR5K_RSSI_THR_BMISS, AR5K_TUNE_RSSI_THRES);
		 * with different variables and check results compared
		 * to ath5k_hw_reg_write(hal, )  */
		ath5k_hw_reg_write(hal, AR5K_TUNE_RSSI_THRES, AR5K_RSSI_THR);
	}

	/*
	 * Set Rx/Tx DMA Configuration
	 *(passing dma size not available on 5210)
	 */
	if (hal->ah_version != AR5K_AR5210) {
		AR5K_REG_WRITE_BITS(hal, AR5K_TXCFG, AR5K_TXCFG_SDMAMR,
				AR5K_DMASIZE_512B | AR5K_TXCFG_DMASIZE);
		AR5K_REG_WRITE_BITS(hal, AR5K_RXCFG, AR5K_RXCFG_SDMAMW,
				AR5K_DMASIZE_512B);
	}

	/*
	 * Set channel and calibrate the PHY
	 */
	ret = ath5k_hw_channel(hal, channel);
	if (ret)
		return ret;

	/*
	 * Enable the PHY and wait until completion
	 */
	ath5k_hw_reg_write(hal, AR5K_PHY_ACT_ENABLE, AR5K_PHY_ACT);

	/*
	 * 5111/5112 Specific
	 */
	if (hal->ah_version != AR5K_AR5210) {
		data = ath5k_hw_reg_read(hal, AR5K_PHY_RX_DELAY) &
			AR5K_PHY_RX_DELAY_M;
		data = (channel->val & CHANNEL_CCK) ?
			((data << 2) / 22) : (data / 10);

		udelay(100 + data);
	} else {
		mdelay(1);
	}

	/*
	 * Enable calibration and wait until completion
	 */
	AR5K_REG_ENABLE_BITS(hal, AR5K_PHY_AGCCTL,
				AR5K_PHY_AGCCTL_CAL);

	if (ath5k_hw_register_timeout(hal, AR5K_PHY_AGCCTL,
				AR5K_PHY_AGCCTL_CAL, 0, false)) {
		AR5K_PRINTF("calibration timeout (%uMHz)\n", channel->freq);
		return -EAGAIN;
	}

	/*
	 * Enable noise floor calibration and wait until completion
	 */
	AR5K_REG_ENABLE_BITS(hal, AR5K_PHY_AGCCTL,
				AR5K_PHY_AGCCTL_NF);

	if (ath5k_hw_register_timeout(hal, AR5K_PHY_AGCCTL,
			AR5K_PHY_AGCCTL_NF, 0, false)) {
		AR5K_PRINTF("noise floor calibration timeout (%uMHz)\n",
				channel->freq);
		return -EAGAIN;
	}

	/* Wait until the noise floor is calibrated and read the value */
	for (i = 20; i > 0; i--) {
		mdelay(1);
		noise_floor = ath5k_hw_reg_read(hal, AR5K_PHY_NF);

		if (AR5K_PHY_NF_RVAL(noise_floor) &
				AR5K_PHY_NF_ACTIVE)
			noise_floor = AR5K_PHY_NF_AVAL(noise_floor);

		if (noise_floor <= AR5K_TUNE_NOISE_FLOOR)
			break;
	}

	if (noise_floor > AR5K_TUNE_NOISE_FLOOR) {
		AR5K_PRINTF("noise floor calibration failed (%uMHz)\n",
				channel->freq);
		return -EIO;
	}

	hal->ah_calibration = false;

	if (!(channel->val & CHANNEL_B)) {
		hal->ah_calibration = true;
		AR5K_REG_WRITE_BITS(hal, AR5K_PHY_IQ,
				AR5K_PHY_IQ_CAL_NUM_LOG_MAX, 15);
		AR5K_REG_ENABLE_BITS(hal, AR5K_PHY_IQ,
				AR5K_PHY_IQ_RUN);
	}

	/*
	 * Reset queues and start beacon timers at the end of the reset routine
	 */
	for (i = 0; i < hal->ah_capabilities.cap_queues.q_tx_num; i++) {
		/*No QCU on 5210*/
		if (hal->ah_version != AR5K_AR5210)
			AR5K_REG_WRITE_Q(hal, AR5K_QUEUE_QCUMASK(i), i);

		ret = ath5k_hw_reset_tx_queue(hal, i);
		if (ret) {
			AR5K_PRINTF("failed to reset TX queue #%d\n", i);
			return ret;
		}
	}

	/* Pre-enable interrupts on 5211/5212*/
	if (hal->ah_version != AR5K_AR5210)
		ath5k_hw_set_intr(hal, AR5K_INT_RX | AR5K_INT_TX |
				AR5K_INT_FATAL);

	/*
	 * Set RF kill flags if supported by the device (read from the EEPROM)
	 * Disable gpio_intr for now since it results system hang.
	 * TODO: Handle this in ath_intr
	 */
#if 0
	if (AR5K_EEPROM_HDR_RFKILL(hal->ah_capabilities.cap_eeprom.ee_header)) {
		ath5k_hw_set_gpio_input(hal, 0);
		hal->ah_gpio[0] = ath5k_hw_get_gpio(hal, 0);
		if (hal->ah_gpio[0] == 0)
			ath5k_hw_set_gpio_intr(hal, 0, 1);
		else
			ath5k_hw_set_gpio_intr(hal, 0, 0);
	}
#endif

	/*
	 * Set the 32MHz reference clock on 5212 phy clock sleep register
	 */
	if (hal->ah_version == AR5K_AR5212) {
		ath5k_hw_reg_write(hal, AR5K_PHY_SCR_32MHZ, AR5K_PHY_SCR);
		ath5k_hw_reg_write(hal, AR5K_PHY_SLMT_32MHZ, AR5K_PHY_SLMT);
		ath5k_hw_reg_write(hal, AR5K_PHY_SCAL_32MHZ, AR5K_PHY_SCAL);
		ath5k_hw_reg_write(hal, AR5K_PHY_SCLOCK_32MHZ, AR5K_PHY_SCLOCK);
		ath5k_hw_reg_write(hal, AR5K_PHY_SDELAY_32MHZ, AR5K_PHY_SDELAY);
		ath5k_hw_reg_write(hal, hal->ah_radio == AR5K_RF5111 ?
			AR5K_PHY_SPENDING_RF5111 : AR5K_PHY_SPENDING_RF5112,
			AR5K_PHY_SPENDING);
	}

	/*
	 * Disable beacons and reset the register
	 */
	AR5K_REG_DISABLE_BITS(hal, AR5K_BEACON, AR5K_BEACON_ENABLE |
			AR5K_BEACON_RESET_TSF);

	return 0;
}

/*
 * Reset chipset
 */
static int ath5k_hw_nic_reset(struct ath5k_hw *hal, u32 val)
{
	int ret;
	u32 mask = val ? val : ~0;

	AR5K_TRACE;

	/* Read-and-clear RX Descriptor Pointer*/
	ath5k_hw_reg_read(hal, AR5K_RXDP);

	/*
	 * Reset the device and wait until success
	 */
	ath5k_hw_reg_write(hal, val, AR5K_RESET_CTL);

	/* Wait at least 128 PCI clocks */
	udelay(15);

	if (hal->ah_version == AR5K_AR5210) {
		val &= AR5K_RESET_CTL_CHIP;
		mask &= AR5K_RESET_CTL_CHIP;
	} else {
		val &= AR5K_RESET_CTL_PCU | AR5K_RESET_CTL_BASEBAND;
		mask &= AR5K_RESET_CTL_PCU | AR5K_RESET_CTL_BASEBAND;
	}

	ret = ath5k_hw_register_timeout(hal, AR5K_RESET_CTL, mask, val, false);

	/*
	 * Reset configuration register (for hw byte-swap)
	 */
	if ((val & AR5K_RESET_CTL_PCU) == 0)
		ath5k_hw_reg_write(hal, AR5K_INIT_CFG, AR5K_CFG);

	return ret;
}

/*
 * Power management functions
 */

/*
 * Sleep control
 */
int ath5k_hw_set_power(struct ath5k_hw *hal, enum ath5k_power_mode mode,
		bool set_chip, u16 sleep_duration)
{
	unsigned int i;
	u32 staid;

	AR5K_TRACE;
	staid = ath5k_hw_reg_read(hal, AR5K_STA_ID1);

	switch (mode) {
	case AR5K_PM_AUTO:
		staid &= ~AR5K_STA_ID1_DEFAULT_ANTENNA;
		/* fallthrough */
	case AR5K_PM_NETWORK_SLEEP:
		if (set_chip == true)
			ath5k_hw_reg_write(hal,
					AR5K_SLEEP_CTL_SLE | sleep_duration,
					AR5K_SLEEP_CTL);

		staid |= AR5K_STA_ID1_PWR_SV;
		break;

	case AR5K_PM_FULL_SLEEP:
		if (set_chip == true)
			ath5k_hw_reg_write(hal, AR5K_SLEEP_CTL_SLE_SLP,
					AR5K_SLEEP_CTL);

		staid |= AR5K_STA_ID1_PWR_SV;
		break;

	case AR5K_PM_AWAKE:
		if (set_chip == false)
			goto commit;

		ath5k_hw_reg_write(hal, AR5K_SLEEP_CTL_SLE_WAKE,
				AR5K_SLEEP_CTL);

		for (i = 5000; i > 0; i--) {
			/* Check if the chip did wake up */
			if ((ath5k_hw_reg_read(hal, AR5K_PCICFG) &
						AR5K_PCICFG_SPWR_DN) == 0)
				break;

			/* Wait a bit and retry */
			udelay(200);
			ath5k_hw_reg_write(hal, AR5K_SLEEP_CTL_SLE_WAKE,
					AR5K_SLEEP_CTL);
		}

		/* Fail if the chip didn't wake up */
		if (i <= 0)
			return -EIO;

		staid &= ~AR5K_STA_ID1_PWR_SV;
		break;

	default:
		return -EINVAL;
	}

commit:
	hal->ah_power_mode = mode;
	ath5k_hw_reg_write(hal, staid, AR5K_STA_ID1);

	return 0;
}

#if 0
/*
 * Get power mode (sleep state)
 * TODO:Remove ?
 */
enum ath5k_power_mode
ath5k_hw_get_power_mode(struct ath5k_hw *hal)
{
	AR5K_TRACE;
	return hal->ah_power_mode;
}
#endif

/***********************\
  DMA Related Functions
\***********************/

/*
 * Receive functions
 */

/*
 * Start DMA receive
 */
void ath5k_hw_start_rx(struct ath5k_hw *hal)
{
	AR5K_TRACE;
	ath5k_hw_reg_write(hal, AR5K_CR_RXE, AR5K_CR);
}

/*
 * Stop DMA receive
 */
int ath5k_hw_stop_rx_dma(struct ath5k_hw *hal)
{
	unsigned int i;

	AR5K_TRACE;
	ath5k_hw_reg_write(hal, AR5K_CR_RXD, AR5K_CR);

	/*
	 * It may take some time to disable the DMA receive unit
	 */
	for (i = 2000;
	     (i > 0) && ((ath5k_hw_reg_read(hal, AR5K_CR) & AR5K_CR_RXE) != 0);
	     i--)
		udelay(10);

	return i ? 0 : -EBUSY;
}

/*
 * Get the address of the RX Descriptor
 */
u32 ath5k_hw_get_rx_buf(struct ath5k_hw *hal)
{
	return ath5k_hw_reg_read(hal, AR5K_RXDP);
}

/*
 * Set the address of the RX Descriptor
 */
void ath5k_hw_put_rx_buf(struct ath5k_hw *hal, u32 phys_addr)
{
	AR5K_TRACE;

	/*TODO: Shouldn't we check if RX is enabled first?*/
	ath5k_hw_reg_write(hal, phys_addr, AR5K_RXDP);
}

/*
 * Transmit functions
 */

/*
 * Start DMA transmit for a specific queue
 * (see also QCU/DCU functions)
 */
int ath5k_hw_tx_start(struct ath5k_hw *hal, unsigned int queue)
{
	u32 tx_queue;

	AR5K_TRACE;
	AR5K_ASSERT_ENTRY(queue, hal->ah_capabilities.cap_queues.q_tx_num);

	/* Return if queue is declared inactive */
	if (hal->ah_txq[queue].tqi_type == AR5K_TX_QUEUE_INACTIVE)
		return -EIO;

	if (hal->ah_version == AR5K_AR5210) {
		tx_queue = ath5k_hw_reg_read(hal, AR5K_CR);

		/*
		 * Set the queue by type on 5210
		 */
		switch (hal->ah_txq[queue].tqi_type) {
		case AR5K_TX_QUEUE_DATA:
			tx_queue |= AR5K_CR_TXE0 & ~AR5K_CR_TXD0;
			break;
		case AR5K_TX_QUEUE_BEACON:
			tx_queue |= AR5K_CR_TXE1 & ~AR5K_CR_TXD1;
			ath5k_hw_reg_write(hal, AR5K_BCR_TQ1V | AR5K_BCR_BDMAE,
					AR5K_BSR);
			break;
		case AR5K_TX_QUEUE_CAB:
			tx_queue |= AR5K_CR_TXE1 & ~AR5K_CR_TXD1;
			ath5k_hw_reg_write(hal, AR5K_BCR_TQ1FV | AR5K_BCR_TQ1V |
				AR5K_BCR_BDMAE, AR5K_BSR);
			break;
		default:
			return -EINVAL;
		}
		/* Start queue */
		ath5k_hw_reg_write(hal, tx_queue, AR5K_CR);
	} else {
		/* Return if queue is disabled */
		if (AR5K_REG_READ_Q(hal, AR5K_QCU_TXD, queue))
			return -EIO;

		/* Start queue */
		AR5K_REG_WRITE_Q(hal, AR5K_QCU_TXE, queue);
	}

	return 0;
}

/*
 * Stop DMA transmit for a specific queue
 * (see also QCU/DCU functions)
 */
int ath5k_hw_stop_tx_dma(struct ath5k_hw *hal, unsigned int queue)
{
	unsigned int i = 100;
	u32 tx_queue, pending;

	AR5K_TRACE;
	AR5K_ASSERT_ENTRY(queue, hal->ah_capabilities.cap_queues.q_tx_num);

	/* Return if queue is declared inactive */
	if (hal->ah_txq[queue].tqi_type == AR5K_TX_QUEUE_INACTIVE)
		return -EIO;

	if (hal->ah_version == AR5K_AR5210) {
		tx_queue = ath5k_hw_reg_read(hal, AR5K_CR);

		/*
		 * Set by queue type
		 */
		switch (hal->ah_txq[queue].tqi_type) {
		case AR5K_TX_QUEUE_DATA:
			tx_queue |= AR5K_CR_TXD0 & ~AR5K_CR_TXE0;
			break;
		case AR5K_TX_QUEUE_BEACON:
		case AR5K_TX_QUEUE_CAB:
			/* XXX Fix me... */
			tx_queue |= AR5K_CR_TXD1 & ~AR5K_CR_TXD1;
			ath5k_hw_reg_write(hal, 0, AR5K_BSR);
			break;
		default:
			return -EINVAL;
		}

		/* Stop queue */
		ath5k_hw_reg_write(hal, tx_queue, AR5K_CR);
	} else {
		/*
		 * Schedule TX disable and wait until queue is empty
		 */
		AR5K_REG_WRITE_Q(hal, AR5K_QCU_TXD, queue);

		/*Check for pending frames*/
		do {
			pending = ath5k_hw_reg_read(hal,
				AR5K_QUEUE_STATUS(queue)) &
				AR5K_QCU_STS_FRMPENDCNT;
			udelay(100);
		} while (--i && pending);

		/* Clear register */
		ath5k_hw_reg_write(hal, 0, AR5K_QCU_TXD);
	}

	/* TODO: Check for success else return error */
	return 0;
}

/*
 * Get the address of the TX Descriptor for a specific queue
 * (see also QCU/DCU functions)
 */
u32 ath5k_hw_get_tx_buf(struct ath5k_hw *hal, unsigned int queue)
{
	u16 tx_reg;

	AR5K_TRACE;
	AR5K_ASSERT_ENTRY(queue, hal->ah_capabilities.cap_queues.q_tx_num);

	/*
	 * Get the transmit queue descriptor pointer from the selected queue
	 */
	/*5210 doesn't have QCU*/
	if (hal->ah_version == AR5K_AR5210) {
		switch (hal->ah_txq[queue].tqi_type) {
		case AR5K_TX_QUEUE_DATA:
			tx_reg = AR5K_NOQCU_TXDP0;
			break;
		case AR5K_TX_QUEUE_BEACON:
		case AR5K_TX_QUEUE_CAB:
			tx_reg = AR5K_NOQCU_TXDP1;
			break;
		default:
			return 0xffffffff;
		}
	} else {
		tx_reg = AR5K_QUEUE_TXDP(queue);
	}

	return ath5k_hw_reg_read(hal, tx_reg);
}

/*
 * Set the address of the TX Descriptor for a specific queue
 * (see also QCU/DCU functions)
 */
int ath5k_hw_put_tx_buf(struct ath5k_hw *hal, unsigned int queue, u32 phys_addr)
{
	u16 tx_reg;

	AR5K_TRACE;
	AR5K_ASSERT_ENTRY(queue, hal->ah_capabilities.cap_queues.q_tx_num);

	/*
	 * Set the transmit queue descriptor pointer register by type
	 * on 5210
	 */
	if (hal->ah_version == AR5K_AR5210) {
		switch (hal->ah_txq[queue].tqi_type) {
		case AR5K_TX_QUEUE_DATA:
			tx_reg = AR5K_NOQCU_TXDP0;
			break;
		case AR5K_TX_QUEUE_BEACON:
		case AR5K_TX_QUEUE_CAB:
			tx_reg = AR5K_NOQCU_TXDP1;
			break;
		default:
			return -EINVAL;
		}
	} else {
		/*
		 * Set the transmit queue descriptor pointer for
		 * the selected queue on QCU for 5211+
		 * (this won't work if the queue is still active)
		 */
		if (AR5K_REG_READ_Q(hal, AR5K_QCU_TXE, queue))
			return -EIO;

		tx_reg = AR5K_QUEUE_TXDP(queue);
	}

	/* Set descriptor pointer */
	ath5k_hw_reg_write(hal, phys_addr, tx_reg);

	return 0;
}

/*
 * Update tx trigger level
 */
int ath5k_hw_update_tx_triglevel(struct ath5k_hw *hal, bool increase)
{
	u32 trigger_level, imr;
	int ret = -EIO;

	AR5K_TRACE;

	/*
	 * Disable interrupts by setting the mask
	 */
	imr = ath5k_hw_set_intr(hal, hal->ah_imr & ~AR5K_INT_GLOBAL);

	/*TODO: Boundary check on trigger_level*/
	trigger_level = AR5K_REG_MS(ath5k_hw_reg_read(hal, AR5K_TXCFG),
			AR5K_TXCFG_TXFULL);

	if (increase == false) {
		if (--trigger_level < AR5K_TUNE_MIN_TX_FIFO_THRES)
			goto done;
	} else
		trigger_level +=
			((AR5K_TUNE_MAX_TX_FIFO_THRES - trigger_level) / 2);

	/*
	 * Update trigger level on success
	 */
	if (hal->ah_version == AR5K_AR5210)
		ath5k_hw_reg_write(hal, trigger_level, AR5K_TRIG_LVL);
	else
		AR5K_REG_WRITE_BITS(hal, AR5K_TXCFG,
				AR5K_TXCFG_TXFULL, trigger_level);

	ret = 0;

done:
	/*
	 * Restore interrupt mask
	 */
	ath5k_hw_set_intr(hal, imr);

	return ret;
}

/*
 * Interrupt handling
 */

/*
 * Check if we have pending interrupts
 */
bool ath5k_hw_is_intr_pending(struct ath5k_hw *hal)
{
	AR5K_TRACE;
	return ath5k_hw_reg_read(hal, AR5K_INTPEND);
}

/*
 * Get interrupt mask (ISR)
 */
int ath5k_hw_get_isr(struct ath5k_hw *hal, enum ath5k_int *interrupt_mask)
{
	u32 data;

	AR5K_TRACE;

	/*
	 * Read interrupt status from the Interrupt Status register
	 * on 5210
	 */
	if (hal->ah_version == AR5K_AR5210) {
		data = ath5k_hw_reg_read(hal, AR5K_ISR);
		if (unlikely(data == AR5K_INT_NOCARD)) {
			*interrupt_mask = data;
			return -ENODEV;
		}
	}

	/*
	 * Read interrupt status from the Read-And-Clear shadow register
	 */
	data = ath5k_hw_reg_read(hal, AR5K_RAC_PISR);

	/*
	 * Get abstract interrupt mask (HAL-compatible)
	 */
	*interrupt_mask = (data & AR5K_INT_COMMON) & hal->ah_imr;

	if (unlikely(data == AR5K_INT_NOCARD))
		return -ENODEV;

	if (data & (AR5K_ISR_RXOK | AR5K_ISR_RXERR))
		*interrupt_mask |= AR5K_INT_RX;

	if (data & (AR5K_ISR_TXOK | AR5K_ISR_TXERR))
		*interrupt_mask |= AR5K_INT_TX;

	if (hal->ah_version != AR5K_AR5210) {
		/*HIU = Host Interface Unit (PCI etc)*/
		if (unlikely(data & (AR5K_ISR_HIUERR)))
			*interrupt_mask |= AR5K_INT_FATAL;

		/*Beacon Not Ready*/
		if (unlikely(data & (AR5K_ISR_BNR)))
			*interrupt_mask |= AR5K_INT_BNR;
	}

	/*
	 * XXX: BMISS interrupts may occur after association.
	 * I found this on 5210 code but it needs testing
	 */
#if 0
	interrupt_mask &= ~AR5K_INT_BMISS;
#endif

	/*
	 * In case we didn't handle anything,
	 * print the register value.
	 */
	if (unlikely(*interrupt_mask == 0 && net_ratelimit()))
		AR5K_PRINTF("0x%08x\n", data);

	return 0;
}

/*
 * Set interrupt mask
 */
enum ath5k_int ath5k_hw_set_intr(struct ath5k_hw *hal, enum ath5k_int new_mask)
{
	enum ath5k_int old_mask, int_mask;

	/*
	 * Disable card interrupts to prevent any race conditions
	 * (they will be re-enabled afterwards).
	 */
	ath5k_hw_reg_write(hal, AR5K_IER_DISABLE, AR5K_IER);

	old_mask = hal->ah_imr;

	/*
	 * Add additional, chipset-dependent interrupt mask flags
	 * and write them to the IMR (interrupt mask register).
	 */
	int_mask = new_mask & AR5K_INT_COMMON;

	if (new_mask & AR5K_INT_RX)
		int_mask |= AR5K_IMR_RXOK | AR5K_IMR_RXERR | AR5K_IMR_RXORN |
			AR5K_IMR_RXDESC;

	if (new_mask & AR5K_INT_TX)
		int_mask |= AR5K_IMR_TXOK | AR5K_IMR_TXERR | AR5K_IMR_TXDESC |
			AR5K_IMR_TXURN;

	if (hal->ah_version != AR5K_AR5210) {
		if (new_mask & AR5K_INT_FATAL) {
			int_mask |= AR5K_IMR_HIUERR;
			AR5K_REG_ENABLE_BITS(hal, AR5K_SIMR2, AR5K_SIMR2_MCABT |
					AR5K_SIMR2_SSERR | AR5K_SIMR2_DPERR);
		}
	}

	ath5k_hw_reg_write(hal, int_mask, AR5K_PIMR);

	/* Store new interrupt mask */
	hal->ah_imr = new_mask;

	/* ..re-enable interrupts */
	ath5k_hw_reg_write(hal, AR5K_IER_ENABLE, AR5K_IER);

	return old_mask;
}

#if 0
/*
 * Enable HW radar detection
 */
void
ath5k_hw_radar_alert(struct ath5k_hw *hal, bool enable)
{

	AR5K_TRACE;
	/*
	 * Enable radar detection
	 */

	/*Disable interupts*/
	ath5k_hw_reg_write(hal, AR5K_IER_DISABLE, AR5K_IER);

	/*
	 * Set the RXPHY interrupt to be able to detect
	 * possible radar activity.
	 */
	if (hal->ah_version == AR5K_AR5210) {
		if (enable == true)
			AR5K_REG_ENABLE_BITS(hal, AR5K_IMR, AR5K_IMR_RXPHY);
		else
			AR5K_REG_DISABLE_BITS(hal, AR5K_IMR, AR5K_IMR_RXPHY);
	} else {
		/*Also set AR5K_PHY_RADAR register on 5111/5112*/
		if (enable == true) {
			ath5k_hw_reg_write(hal, AR5K_PHY_RADAR_ENABLE,
					AR5K_PHY_RADAR);
			AR5K_REG_ENABLE_BITS(hal, AR5K_PIMR,
					AR5K_IMR_RXPHY);
		} else {
			ath5k_hw_reg_write(hal, AR5K_PHY_RADAR_DISABLE,
					AR5K_PHY_RADAR);
			AR5K_REG_DISABLE_BITS(hal, AR5K_PIMR,
					AR5K_IMR_RXPHY);
		}
	}

	/*Re-enable interrupts*/
	ath5k_hw_reg_write(hal, AR5K_IER_ENABLE, AR5K_IER);
}
#endif



/*************************\
  EEPROM access functions
\*************************/

/*
 * Read from eeprom
 */
static int ath5k_hw_eeprom_read(struct ath5k_hw *hal, u32 offset, u16 *data)
{
	u32 status, timeout;

	AR5K_TRACE;
	/*
	 * Initialize EEPROM access
	 */
	if (hal->ah_version == AR5K_AR5210) {
		AR5K_REG_ENABLE_BITS(hal, AR5K_PCICFG, AR5K_PCICFG_EEAE);
		(void)ath5k_hw_reg_read(hal, AR5K_EEPROM_BASE + (4 * offset));
	} else {
		ath5k_hw_reg_write(hal, offset, AR5K_EEPROM_BASE);
		AR5K_REG_ENABLE_BITS(hal, AR5K_EEPROM_CMD,
				AR5K_EEPROM_CMD_READ);
	}

	for (timeout = AR5K_TUNE_REGISTER_TIMEOUT; timeout > 0; timeout--) {
		status = ath5k_hw_reg_read(hal, AR5K_EEPROM_STATUS);
		if (status & AR5K_EEPROM_STAT_RDDONE) {
			if (status & AR5K_EEPROM_STAT_RDERR)
				return -EIO;
			*data = (u16)(ath5k_hw_reg_read(hal, AR5K_EEPROM_DATA) &
					0xffff);
			return 0;
		}
		udelay(15);
	}

	return -ETIMEDOUT;
}

/*
 * Write to eeprom - currently disabled, use at your own risk
 */
static int ath5k_hw_eeprom_write(struct ath5k_hw *hal, u32 offset, u16 data)
{
#if 0
	u32 status, timeout;

	AR5K_TRACE;

	/*
	 * Initialize eeprom access
	 */

	if (hal->ah_version == AR5K_AR5210) {
		AR5K_REG_ENABLE_BITS(hal, AR5K_PCICFG, AR5K_PCICFG_EEAE);
	} else {
		AR5K_REG_ENABLE_BITS(hal, AR5K_EEPROM_CMD,
				AR5K_EEPROM_CMD_RESET);
	}

	/*
	 * Write data to data register
	 */

	if (hal->ah_version == AR5K_AR5210) {
		ath5k_hw_reg_write(hal, data, AR5K_EEPROM_BASE + (4 * offset));
	} else {
		ath5k_hw_reg_write(hal, offset, AR5K_EEPROM_BASE);
		ath5k_hw_reg_write(hal, data, AR5K_EEPROM_DATA);
		AR5K_REG_ENABLE_BITS(hal, AR5K_EEPROM_CMD,
				AR5K_EEPROM_CMD_WRITE);
	}

	/*
	 * Check status
	 */

	for (timeout = AR5K_TUNE_REGISTER_TIMEOUT; timeout > 0; timeout--) {
		status = ath5k_hw_reg_read(hal, AR5K_EEPROM_STATUS);
		if (status & AR5K_EEPROM_STAT_WRDONE) {
			if (status & AR5K_EEPROM_STAT_WRERR)
				return EIO;
			return 0;
		}
		udelay(15);
	}
#endif
	AR5K_PRINTF("EEPROM Write is disabled!");
	return -EIO;
}

/*
 * Translate binary channel representation in EEPROM to frequency
 */
static u16 ath5k_eeprom_bin2freq(struct ath5k_hw *hal, u16 bin, unsigned int mode)
{
	u16 val;

	if (bin == AR5K_EEPROM_CHANNEL_DIS)
		return bin;

	if (mode == AR5K_EEPROM_MODE_11A) {
		if (hal->ah_ee_version > AR5K_EEPROM_VERSION_3_2)
			val = (5 * bin) + 4800;
		else
			val = bin > 62 ?
				(10 * 62) + (5 * (bin - 62)) + 5100 :
				(bin * 10) + 5100;
	} else {
		if (hal->ah_ee_version > AR5K_EEPROM_VERSION_3_2)
			val = bin + 2300;
		else
			val = bin + 2400;
	}

	return val;
}

/*
 * Read antenna infos from eeprom
 */
static int ath5k_eeprom_read_ants(struct ath5k_hw *hal, u32 *offset,
		unsigned int mode)
{
	struct ath5k_eeprom_info *ee = &hal->ah_capabilities.cap_eeprom;
	u32 o = *offset;
	u16 val;
	int ret, i = 0;

	AR5K_EEPROM_READ(o++, val);
	ee->ee_switch_settling[mode]	= (val >> 8) & 0x7f;
	ee->ee_ant_tx_rx[mode]		= (val >> 2) & 0x3f;
	ee->ee_ant_control[mode][i]	= (val << 4) & 0x3f;

	AR5K_EEPROM_READ(o++, val);
	ee->ee_ant_control[mode][i++]	|= (val >> 12) & 0xf;
	ee->ee_ant_control[mode][i++]	= (val >> 6) & 0x3f;
	ee->ee_ant_control[mode][i++]	= val & 0x3f;

	AR5K_EEPROM_READ(o++, val);
	ee->ee_ant_control[mode][i++]	= (val >> 10) & 0x3f;
	ee->ee_ant_control[mode][i++]	= (val >> 4) & 0x3f;
	ee->ee_ant_control[mode][i]	= (val << 2) & 0x3f;

	AR5K_EEPROM_READ(o++, val);
	ee->ee_ant_control[mode][i++]	|= (val >> 14) & 0x3;
	ee->ee_ant_control[mode][i++]	= (val >> 8) & 0x3f;
	ee->ee_ant_control[mode][i++]	= (val >> 2) & 0x3f;
	ee->ee_ant_control[mode][i]	= (val << 4) & 0x3f;

	AR5K_EEPROM_READ(o++, val);
	ee->ee_ant_control[mode][i++]	|= (val >> 12) & 0xf;
	ee->ee_ant_control[mode][i++]	= (val >> 6) & 0x3f;
	ee->ee_ant_control[mode][i++]	= val & 0x3f;

	/* Get antenna modes */
	hal->ah_antenna[mode][0] =
		(ee->ee_ant_control[mode][0] << 4) | 0x1;
	hal->ah_antenna[mode][AR5K_ANT_FIXED_A] =
		ee->ee_ant_control[mode][1] 		|
		(ee->ee_ant_control[mode][2] << 6) 	|
		(ee->ee_ant_control[mode][3] << 12)	|
		(ee->ee_ant_control[mode][4] << 18)	|
		(ee->ee_ant_control[mode][5] << 24);
	hal->ah_antenna[mode][AR5K_ANT_FIXED_B] =
		ee->ee_ant_control[mode][6] 		|
		(ee->ee_ant_control[mode][7] << 6) 	|
		(ee->ee_ant_control[mode][8] << 12)	|
		(ee->ee_ant_control[mode][9] << 18)	|
		(ee->ee_ant_control[mode][10] << 24);

	/* return new offset */
	*offset = o;

	return 0;
}

/*
 * Read supported modes from eeprom
 */
static int ath5k_eeprom_read_modes(struct ath5k_hw *hal, u32 *offset,
		unsigned int mode)
{
	struct ath5k_eeprom_info *ee = &hal->ah_capabilities.cap_eeprom;
	u32 o = *offset;
	u16 val;
	int ret;

	AR5K_EEPROM_READ(o++, val);
	ee->ee_tx_end2xlna_enable[mode]	= (val >> 8) & 0xff;
	ee->ee_thr_62[mode]		= val & 0xff;

	if (hal->ah_ee_version <= AR5K_EEPROM_VERSION_3_2)
		ee->ee_thr_62[mode] = (mode == AR5K_EEPROM_MODE_11A) ? 15 : 28;

	AR5K_EEPROM_READ(o++, val);
	ee->ee_tx_end2xpa_disable[mode]	= (val >> 8) & 0xff;
	ee->ee_tx_frm2xpa_enable[mode]	= val & 0xff;

	AR5K_EEPROM_READ(o++, val);
	ee->ee_pga_desired_size[mode]	= (val >> 8) & 0xff;

	if ((val & 0xff) & 0x80)
		ee->ee_noise_floor_thr[mode] = -((((val & 0xff) ^ 0xff)) + 1);
	else
		ee->ee_noise_floor_thr[mode] = val & 0xff;

	if (hal->ah_ee_version <= AR5K_EEPROM_VERSION_3_2)
		ee->ee_noise_floor_thr[mode] =
			mode == AR5K_EEPROM_MODE_11A ? -54 : -1;

	AR5K_EEPROM_READ(o++, val);
	ee->ee_xlna_gain[mode]		= (val >> 5) & 0xff;
	ee->ee_x_gain[mode]		= (val >> 1) & 0xf;
	ee->ee_xpd[mode]		= val & 0x1;

	if (hal->ah_ee_version >= AR5K_EEPROM_VERSION_4_0)
		ee->ee_fixed_bias[mode] = (val >> 13) & 0x1;

	if (hal->ah_ee_version >= AR5K_EEPROM_VERSION_3_3) {
		AR5K_EEPROM_READ(o++, val);
		ee->ee_false_detect[mode] = (val >> 6) & 0x7f;

		if (mode == AR5K_EEPROM_MODE_11A)
			ee->ee_xr_power[mode] = val & 0x3f;
		else {
			ee->ee_ob[mode][0] = val & 0x7;
			ee->ee_db[mode][0] = (val >> 3) & 0x7;
		}
	}

	if (hal->ah_ee_version < AR5K_EEPROM_VERSION_3_4) {
		ee->ee_i_gain[mode] = AR5K_EEPROM_I_GAIN;
		ee->ee_cck_ofdm_power_delta = AR5K_EEPROM_CCK_OFDM_DELTA;
	} else {
		ee->ee_i_gain[mode] = (val >> 13) & 0x7;

		AR5K_EEPROM_READ(o++, val);
		ee->ee_i_gain[mode] |= (val << 3) & 0x38;

		if (mode == AR5K_EEPROM_MODE_11G)
			ee->ee_cck_ofdm_power_delta = (val >> 3) & 0xff;
	}

	if (hal->ah_ee_version >= AR5K_EEPROM_VERSION_4_0 &&
			mode == AR5K_EEPROM_MODE_11A) {
		ee->ee_i_cal[mode] = (val >> 8) & 0x3f;
		ee->ee_q_cal[mode] = (val >> 3) & 0x1f;
	}

	if (hal->ah_ee_version >= AR5K_EEPROM_VERSION_4_6 &&
			mode == AR5K_EEPROM_MODE_11G)
		ee->ee_scaled_cck_delta = (val >> 11) & 0x1f;

	/* return new offset */
	*offset = o;

	return 0;
}

/*
 * Initialize eeprom & capabilities structs
 */
static int ath5k_eeprom_init(struct ath5k_hw *hal)
{
	struct ath5k_eeprom_info *ee = &hal->ah_capabilities.cap_eeprom;
	unsigned int mode, i;
	int ret;
	u32 offset;
	u16 val;

	/* Initial TX thermal adjustment values */
	ee->ee_tx_clip = 4;
	ee->ee_pwd_84 = ee->ee_pwd_90 = 1;
	ee->ee_gain_select = 1;

	/*
	 * Read values from EEPROM and store them in the capability structure
	 */
	AR5K_EEPROM_READ_HDR(AR5K_EEPROM_MAGIC, ee_magic);
	AR5K_EEPROM_READ_HDR(AR5K_EEPROM_PROTECT, ee_protect);
	AR5K_EEPROM_READ_HDR(AR5K_EEPROM_REG_DOMAIN, ee_regdomain);
	AR5K_EEPROM_READ_HDR(AR5K_EEPROM_VERSION, ee_version);
	AR5K_EEPROM_READ_HDR(AR5K_EEPROM_HDR, ee_header);

	/* Return if we have an old EEPROM */
	if (hal->ah_ee_version < AR5K_EEPROM_VERSION_3_0)
		return 0;

#ifdef notyet
	/*
	 * Validate the checksum of the EEPROM date. There are some
	 * devices with invalid EEPROMs.
	 */
	for (cksum = 0, offset = 0; offset < AR5K_EEPROM_INFO_MAX; offset++) {
		AR5K_EEPROM_READ(AR5K_EEPROM_INFO(offset), val);
		cksum ^= val;
	}
	if (cksum != AR5K_EEPROM_INFO_CKSUM) {
		AR5K_PRINTF("Invalid EEPROM checksum 0x%04x\n", cksum);
		return -EIO;
	}
#endif

	AR5K_EEPROM_READ_HDR(AR5K_EEPROM_ANT_GAIN(hal->ah_ee_version),
			ee_ant_gain);

	if (hal->ah_ee_version >= AR5K_EEPROM_VERSION_4_0) {
		AR5K_EEPROM_READ_HDR(AR5K_EEPROM_MISC0, ee_misc0);
		AR5K_EEPROM_READ_HDR(AR5K_EEPROM_MISC1, ee_misc1);
	}

	if (hal->ah_ee_version < AR5K_EEPROM_VERSION_3_3) {
		AR5K_EEPROM_READ(AR5K_EEPROM_OBDB0_2GHZ, val);
		ee->ee_ob[AR5K_EEPROM_MODE_11B][0] = val & 0x7;
		ee->ee_db[AR5K_EEPROM_MODE_11B][0] = (val >> 3) & 0x7;

		AR5K_EEPROM_READ(AR5K_EEPROM_OBDB1_2GHZ, val);
		ee->ee_ob[AR5K_EEPROM_MODE_11G][0] = val & 0x7;
		ee->ee_db[AR5K_EEPROM_MODE_11G][0] = (val >> 3) & 0x7;
	}

	/*
	 * Get conformance test limit values
	 */
	offset = AR5K_EEPROM_CTL(hal->ah_ee_version);
	ee->ee_ctls = AR5K_EEPROM_N_CTLS(hal->ah_ee_version);

	for (i = 0; i < ee->ee_ctls; i++) {
		AR5K_EEPROM_READ(offset++, val);
		ee->ee_ctl[i] = (val >> 8) & 0xff;
		ee->ee_ctl[i + 1] = val & 0xff;
	}

	/*
	 * Get values for 802.11a (5GHz)
	 */
	mode = AR5K_EEPROM_MODE_11A;

	ee->ee_turbo_max_power[mode] =
		AR5K_EEPROM_HDR_T_5GHZ_DBM(ee->ee_header);

	offset = AR5K_EEPROM_MODES_11A(hal->ah_ee_version);

	ret = ath5k_eeprom_read_ants(hal, &offset, mode);
	if (ret)
		return ret;

	AR5K_EEPROM_READ(offset++, val);
	ee->ee_adc_desired_size[mode]	= (s8)((val >> 8) & 0xff);
	ee->ee_ob[mode][3]		= (val >> 5) & 0x7;
	ee->ee_db[mode][3]		= (val >> 2) & 0x7;
	ee->ee_ob[mode][2]		= (val << 1) & 0x7;

	AR5K_EEPROM_READ(offset++, val);
	ee->ee_ob[mode][2]		|= (val >> 15) & 0x1;
	ee->ee_db[mode][2]		= (val >> 12) & 0x7;
	ee->ee_ob[mode][1]		= (val >> 9) & 0x7;
	ee->ee_db[mode][1]		= (val >> 6) & 0x7;
	ee->ee_ob[mode][0]		= (val >> 3) & 0x7;
	ee->ee_db[mode][0]		= val & 0x7;

	ret = ath5k_eeprom_read_modes(hal, &offset, mode);
	if (ret)
		return ret;

	if (hal->ah_ee_version >= AR5K_EEPROM_VERSION_4_1) {
		AR5K_EEPROM_READ(offset++, val);
		ee->ee_margin_tx_rx[mode] = val & 0x3f;
	}

	/*
	 * Get values for 802.11b (2.4GHz)
	 */
	mode = AR5K_EEPROM_MODE_11B;
	offset = AR5K_EEPROM_MODES_11B(hal->ah_ee_version);

	ret = ath5k_eeprom_read_ants(hal, &offset, mode);
	if (ret)
		return ret;

	AR5K_EEPROM_READ(offset++, val);
	ee->ee_adc_desired_size[mode]	= (s8)((val >> 8) & 0xff);
	ee->ee_ob[mode][1]		= (val >> 4) & 0x7;
	ee->ee_db[mode][1]		= val & 0x7;

	ret = ath5k_eeprom_read_modes(hal, &offset, mode);
	if (ret)
		return ret;

	if (hal->ah_ee_version >= AR5K_EEPROM_VERSION_4_0) {
		AR5K_EEPROM_READ(offset++, val);
		ee->ee_cal_pier[mode][0] =
			ath5k_eeprom_bin2freq(hal, val & 0xff, mode);
		ee->ee_cal_pier[mode][1] =
			ath5k_eeprom_bin2freq(hal, (val >> 8) & 0xff, mode);

		AR5K_EEPROM_READ(offset++, val);
		ee->ee_cal_pier[mode][2] =
			ath5k_eeprom_bin2freq(hal, val & 0xff, mode);
	}

	if (hal->ah_ee_version >= AR5K_EEPROM_VERSION_4_1)
		ee->ee_margin_tx_rx[mode] = (val >> 8) & 0x3f;

	/*
	 * Get values for 802.11g (2.4GHz)
	 */
	mode = AR5K_EEPROM_MODE_11G;
	offset = AR5K_EEPROM_MODES_11G(hal->ah_ee_version);

	ret = ath5k_eeprom_read_ants(hal, &offset, mode);
	if (ret)
		return ret;

	AR5K_EEPROM_READ(offset++, val);
	ee->ee_adc_desired_size[mode]	= (s8)((val >> 8) & 0xff);
	ee->ee_ob[mode][1]		= (val >> 4) & 0x7;
	ee->ee_db[mode][1]		= val & 0x7;

	ret = ath5k_eeprom_read_modes(hal, &offset, mode);
	if (ret)
		return ret;

	if (hal->ah_ee_version >= AR5K_EEPROM_VERSION_4_0) {
		AR5K_EEPROM_READ(offset++, val);
		ee->ee_cal_pier[mode][0] =
			ath5k_eeprom_bin2freq(hal, val & 0xff, mode);
		ee->ee_cal_pier[mode][1] =
			ath5k_eeprom_bin2freq(hal, (val >> 8) & 0xff, mode);

		AR5K_EEPROM_READ(offset++, val);
		ee->ee_turbo_max_power[mode] = val & 0x7f;
		ee->ee_xr_power[mode] = (val >> 7) & 0x3f;

		AR5K_EEPROM_READ(offset++, val);
		ee->ee_cal_pier[mode][2] =
			ath5k_eeprom_bin2freq(hal, val & 0xff, mode);

		if (hal->ah_ee_version >= AR5K_EEPROM_VERSION_4_1)
			ee->ee_margin_tx_rx[mode] = (val >> 8) & 0x3f;

		AR5K_EEPROM_READ(offset++, val);
		ee->ee_i_cal[mode] = (val >> 8) & 0x3f;
		ee->ee_q_cal[mode] = (val >> 3) & 0x1f;

		if (hal->ah_ee_version >= AR5K_EEPROM_VERSION_4_2) {
			AR5K_EEPROM_READ(offset++, val);
			ee->ee_cck_ofdm_gain_delta = val & 0xff;
		}
	}

	/*
	 * Read 5GHz EEPROM channels
	 */

	return 0;
}

/*
 * Read the MAC address from eeprom
 */
static int ath5k_eeprom_read_mac(struct ath5k_hw *hal, u8 *mac)
{
	u8 mac_d[ETH_ALEN];
	u32 total, offset;
	u16 data;
	int octet, ret;

	memset(mac, 0, ETH_ALEN);
	memset(mac_d, 0, ETH_ALEN);

	ret = ath5k_hw_eeprom_read(hal, 0x20, &data);
	if (ret)
		return ret;

	for (offset = 0x1f, octet = 0, total = 0; offset >= 0x1d; offset--) {
		ret = ath5k_hw_eeprom_read(hal, offset, &data);
		if (ret)
			return ret;

		total += data;
		mac_d[octet + 1] = data & 0xff;
		mac_d[octet] = data >> 8;
		octet += 2;
	}

	memcpy(mac, mac_d, ETH_ALEN);

	if (!total || total == 3 * 0xffff)
		return -EINVAL;

	return 0;
}

/*
 * Read/Write regulatory domain
 */
static bool ath5k_eeprom_regulation_domain(struct ath5k_hw *hal, bool write,
	enum ath5k_regdom *regdomain)
{
	u16 ee_regdomain;

	/* Read current value */
	if (write != true) {
		ee_regdomain = hal->ah_capabilities.cap_eeprom.ee_regdomain;
		*regdomain = ath5k_regdom_to_ieee(ee_regdomain);
		return true;
	}

	ee_regdomain = ath5k_regdom_from_ieee(*regdomain);

	/* Try to write a new value */
	if (hal->ah_capabilities.cap_eeprom.ee_protect &
			AR5K_EEPROM_PROTECT_WR_128_191)
		return false;
	if (ath5k_hw_eeprom_write(hal, AR5K_EEPROM_REG_DOMAIN, 
				ee_regdomain) != 0)
		return false;

	hal->ah_capabilities.cap_eeprom.ee_regdomain = ee_regdomain;

	return true;
}

/*
 * Use the above to write a new regulatory domain
 */
int ath5k_hw_set_regdomain(struct ath5k_hw *hal, u16 regdomain)
{
	enum ath5k_regdom ieee_regdomain;

	ieee_regdomain = ath5k_regdom_to_ieee(regdomain);

	if (ath5k_eeprom_regulation_domain(hal, true, &ieee_regdomain) == true)
		return 0;

	return -EIO;
}

/*
 * Fill the capabilities struct
 */
static int ath5k_hw_get_capabilities(struct ath5k_hw *hal)
{
	u16 ee_header;

	AR5K_TRACE;
	/* Capabilities stored in the EEPROM */
	ee_header = hal->ah_capabilities.cap_eeprom.ee_header;

	if (hal->ah_version == AR5K_AR5210) {
		/*
		 * Set radio capabilities
		 * (The AR5110 only supports the middle 5GHz band)
		 */
		hal->ah_capabilities.cap_range.range_5ghz_min = 5120;
		hal->ah_capabilities.cap_range.range_5ghz_max = 5430;
		hal->ah_capabilities.cap_range.range_2ghz_min = 0;
		hal->ah_capabilities.cap_range.range_2ghz_max = 0;

		/* Set supported modes */
		set_bit(MODE_IEEE80211A, hal->ah_capabilities.cap_mode);
		set_bit(MODE_ATHEROS_TURBO, hal->ah_capabilities.cap_mode);
	} else {
		/*
		 * XXX The tranceiver supports frequencies from 4920 to 6100GHz
		 * XXX and from 2312 to 2732GHz. There are problems with the
		 * XXX current ieee80211 implementation because the IEEE
		 * XXX channel mapping does not support negative channel
		 * XXX numbers (2312MHz is channel -19). Of course, this
		 * XXX doesn't matter because these channels are out of range
		 * XXX but some regulation domains like MKK (Japan) will
		 * XXX support frequencies somewhere around 4.8GHz.
		 */

		/*
		 * Set radio capabilities
		 */

		if (AR5K_EEPROM_HDR_11A(ee_header)) {
			hal->ah_capabilities.cap_range.range_5ghz_min = 5005; /* 4920 */
			hal->ah_capabilities.cap_range.range_5ghz_max = 6100;

			/* Set supported modes */
			set_bit(MODE_IEEE80211A, hal->ah_capabilities.cap_mode);
			set_bit(MODE_ATHEROS_TURBO,
					hal->ah_capabilities.cap_mode);
			if (hal->ah_version == AR5K_AR5212)
				set_bit(MODE_ATHEROS_TURBOG,
						hal->ah_capabilities.cap_mode);
		}

		/* Enable  802.11b if a 2GHz capable radio (2111/5112) is
		 * connected */
		if (AR5K_EEPROM_HDR_11B(ee_header) ||
				AR5K_EEPROM_HDR_11G(ee_header)) {
			hal->ah_capabilities.cap_range.range_2ghz_min = 2412; /* 2312 */
			hal->ah_capabilities.cap_range.range_2ghz_max = 2732;

			if (AR5K_EEPROM_HDR_11B(ee_header))
				set_bit(MODE_IEEE80211B,
						hal->ah_capabilities.cap_mode);

			if (AR5K_EEPROM_HDR_11G(ee_header))
				set_bit(MODE_IEEE80211G,
						hal->ah_capabilities.cap_mode);
		}
	}

	/* GPIO */
	hal->ah_gpio_npins = AR5K_NUM_GPIO;

	/* Set number of supported TX queues */
	if (hal->ah_version == AR5K_AR5210)
		hal->ah_capabilities.cap_queues.q_tx_num =
			AR5K_NUM_TX_QUEUES_NOQCU;
	else
		hal->ah_capabilities.cap_queues.q_tx_num = AR5K_NUM_TX_QUEUES;

	return 0;
}

/*********************************\
  Protocol Control Unit Functions
\*********************************/

/*
 * Set Operation mode
 */
int ath5k_hw_set_opmode(struct ath5k_hw *hal)
{
	u32 pcu_reg, beacon_reg, low_id, high_id;

	pcu_reg = 0;
	beacon_reg = 0;

	AR5K_TRACE;

	switch (hal->ah_op_mode) {
	case IEEE80211_IF_TYPE_IBSS:
		pcu_reg |= AR5K_STA_ID1_ADHOC | AR5K_STA_ID1_DESC_ANTENNA |
			(hal->ah_version == AR5K_AR5210 ?
				AR5K_STA_ID1_NO_PSPOLL : 0);
		beacon_reg |= AR5K_BCR_ADHOC;
		break;

	case IEEE80211_IF_TYPE_AP:
		pcu_reg |= AR5K_STA_ID1_AP | AR5K_STA_ID1_RTS_DEF_ANTENNA |
			(hal->ah_version == AR5K_AR5210 ?
				AR5K_STA_ID1_NO_PSPOLL : 0);
		beacon_reg |= AR5K_BCR_AP;
		break;

	case IEEE80211_IF_TYPE_STA:
		pcu_reg |= AR5K_STA_ID1_DEFAULT_ANTENNA |
			(hal->ah_version == AR5K_AR5210 ?
				AR5K_STA_ID1_PWR_SV : 0);
	case IEEE80211_IF_TYPE_MNTR:
		pcu_reg |= AR5K_STA_ID1_DEFAULT_ANTENNA |
			(hal->ah_version == AR5K_AR5210 ?
				AR5K_STA_ID1_NO_PSPOLL : 0);
		break;

	default:
		return -EINVAL;
	}

	/*
	 * Set PCU registers
	 */
	low_id = AR5K_LOW_ID(hal->ah_sta_id);
	high_id = AR5K_HIGH_ID(hal->ah_sta_id);
	ath5k_hw_reg_write(hal, low_id, AR5K_STA_ID0);
	ath5k_hw_reg_write(hal, pcu_reg | high_id, AR5K_STA_ID1);

	/*
	 * Set Beacon Control Register on 5210
	 */
	if (hal->ah_version == AR5K_AR5210)
		ath5k_hw_reg_write(hal, beacon_reg, AR5K_BCR);

	return 0;
}

/*
 * BSSID Functions
 */

/*
 * Get station id
 */
void ath5k_hw_get_lladdr(struct ath5k_hw *hal, u8 *mac)
{
	AR5K_TRACE;
	memcpy(mac, hal->ah_sta_id, ETH_ALEN);
}

/*
 * Set station id
 */
int ath5k_hw_set_lladdr(struct ath5k_hw *hal, const u8 *mac)
{
	u32 low_id, high_id;

	AR5K_TRACE;
	/* Set new station ID */
	memcpy(hal->ah_sta_id, mac, ETH_ALEN);

	low_id = AR5K_LOW_ID(mac);
	high_id = AR5K_HIGH_ID(mac);

	ath5k_hw_reg_write(hal, low_id, AR5K_STA_ID0);
	ath5k_hw_reg_write(hal, high_id, AR5K_STA_ID1);

	return 0;
}

/*
 * Set BSSID
 */
void ath5k_hw_set_associd(struct ath5k_hw *hal, const u8 *bssid, u16 assoc_id)
{
	u32 low_id, high_id;
	u16 tim_offset = 0;

	/*
	 * Set simple BSSID mask on 5212
	 */
	if (hal->ah_version == AR5K_AR5212) {
		ath5k_hw_reg_write(hal, 0xfffffff, AR5K_BSS_IDM0);
		ath5k_hw_reg_write(hal, 0xfffffff, AR5K_BSS_IDM1);
	}

	/*
	 * Set BSSID which triggers the "SME Join" operation
	 */
	low_id = AR5K_LOW_ID(bssid);
	high_id = AR5K_HIGH_ID(bssid);
	ath5k_hw_reg_write(hal, low_id, AR5K_BSS_ID0);
	ath5k_hw_reg_write(hal, high_id | ((assoc_id & 0x3fff) <<
				AR5K_BSS_ID1_AID_S), AR5K_BSS_ID1);
	memcpy(&hal->ah_bssid, bssid, ETH_ALEN);

	if (assoc_id == 0) {
		ath5k_hw_disable_pspoll(hal);
		return;
	}

	AR5K_REG_WRITE_BITS(hal, AR5K_BEACON, AR5K_BEACON_TIM,
			tim_offset ? tim_offset + 4 : 0);

	ath5k_hw_enable_pspoll(hal, NULL, 0);
}

/*
 * Set BSSID mask on 5212
 */
int ath5k_hw_set_bssid_mask(struct ath5k_hw *hal, const u8 *mask)
{
	u32 low_id, high_id;
	AR5K_TRACE;

	if (hal->ah_version == AR5K_AR5212) {
		low_id = AR5K_LOW_ID(mask);
		high_id = AR5K_HIGH_ID(mask);

		ath5k_hw_reg_write(hal, low_id, AR5K_BSS_IDM0);
		ath5k_hw_reg_write(hal, high_id, AR5K_BSS_IDM1);

		return 0;
	}

	return -EIO;
}

/*
 * Receive start/stop functions
 */

/*
 * Start receive on PCU
 */
void ath5k_hw_start_rx_pcu(struct ath5k_hw *hal)
{
	AR5K_TRACE;
	AR5K_REG_DISABLE_BITS(hal, AR5K_DIAG_SW, AR5K_DIAG_SW_DIS_RX);
}

/*
 * Stop receive on PCU
 */
void ath5k_hw_stop_pcu_recv(struct ath5k_hw *hal)
{
	AR5K_TRACE;
	AR5K_REG_ENABLE_BITS(hal, AR5K_DIAG_SW, AR5K_DIAG_SW_DIS_RX);
}

/*
 * RX Filter functions
 */

/*
 * Set multicast filter
 */
void ath5k_hw_set_mcast_filter(struct ath5k_hw *hal, u32 filter0, u32 filter1)
{
	AR5K_TRACE;
	/* Set the multicat filter */
	ath5k_hw_reg_write(hal, filter0, AR5K_MCAST_FILTER0);
	ath5k_hw_reg_write(hal, filter1, AR5K_MCAST_FILTER1);
}

/*
 * Set multicast filter by index
 */
int ath5k_hw_set_mcast_filterindex(struct ath5k_hw *hal, u32 index)
{

	AR5K_TRACE;
	if (index >= 64)
		return -EINVAL;
	else if (index >= 32)
		AR5K_REG_ENABLE_BITS(hal, AR5K_MCAST_FILTER1,
				(1 << (index - 32)));
	else
		AR5K_REG_ENABLE_BITS(hal, AR5K_MCAST_FILTER0, (1 << index));

	return 0;
}

/*
 * Clear Multicast filter by index
 */
int ath5k_hw_clear_mcast_filter_idx(struct ath5k_hw *hal, u32 index)
{

	AR5K_TRACE;
	if (index >= 64)
		return -EINVAL;
	else if (index >= 32)
		AR5K_REG_DISABLE_BITS(hal, AR5K_MCAST_FILTER1,
				(1 << (index - 32)));
	else
		AR5K_REG_DISABLE_BITS(hal, AR5K_MCAST_FILTER0, (1 << index));

	return 0;
}

/*
 * Get current rx filter
 */
u32 ath5k_hw_get_rx_filter(struct ath5k_hw *ah)
{
	u32 data, filter = 0;

	AR5K_TRACE;
	filter = ath5k_hw_reg_read(ah, AR5K_RX_FILTER);

	/*Radar detection for 5212*/
	if (ah->ah_version == AR5K_AR5212) {
		data = ath5k_hw_reg_read(ah, AR5K_PHY_ERR_FIL);

		if (data & AR5K_PHY_ERR_FIL_RADAR)
			filter |= AR5K_RX_FILTER_RADARERR;
		if (data & (AR5K_PHY_ERR_FIL_OFDM | AR5K_PHY_ERR_FIL_CCK))
			filter |= AR5K_RX_FILTER_PHYERR;
	}

	return filter;
}

/*
 * Set rx filter
 */
void ath5k_hw_set_rx_filter(struct ath5k_hw *ah, u32 filter)
{
	u32 data = 0;

	AR5K_TRACE;

	/* Set PHY error filter register on 5212*/
	if (ah->ah_version == AR5K_AR5212) {
		if (filter & AR5K_RX_FILTER_RADARERR)
			data |= AR5K_PHY_ERR_FIL_RADAR;
		if (filter & AR5K_RX_FILTER_PHYERR)
			data |= AR5K_PHY_ERR_FIL_OFDM | AR5K_PHY_ERR_FIL_CCK;
	}

	/*
	 * The AR5210 uses promiscous mode to detect radar activity
	 */
	if ((ah->ah_version == AR5K_AR5210) &&
			(filter & AR5K_RX_FILTER_RADARERR)) {
		filter &= ~AR5K_RX_FILTER_RADARERR;
		filter |= AR5K_RX_FILTER_PROM;
	}

	/*Zero length DMA*/
	if (data)
		AR5K_REG_ENABLE_BITS(ah, AR5K_RXCFG, AR5K_RXCFG_ZLFDMA);
	else
		AR5K_REG_DISABLE_BITS(ah, AR5K_RXCFG, AR5K_RXCFG_ZLFDMA);

	/*Write RX Filter register*/
	ath5k_hw_reg_write(ah, filter & 0xff, AR5K_RX_FILTER);

	/*Write PHY error filter register on 5212*/
	if (ah->ah_version == AR5K_AR5212)
		ath5k_hw_reg_write(ah, data, AR5K_PHY_ERR_FIL);

}

/*
 * Beacon related functions
 */

/*
 * Get a 32bit TSF
 */
u32 ath5k_hw_get_tsf32(struct ath5k_hw *hal)
{
	AR5K_TRACE;
	return ath5k_hw_reg_read(hal, AR5K_TSF_L32);
}

/*
 * Get the full 64bit TSF
 */
u64 ath5k_hw_get_tsf64(struct ath5k_hw *hal)
{
	u64 tsf = ath5k_hw_reg_read(hal, AR5K_TSF_U32);
	AR5K_TRACE;

	return ath5k_hw_reg_read(hal, AR5K_TSF_L32) | (tsf << 32);
}

/*
 * Force a TSF reset
 */
void ath5k_hw_reset_tsf(struct ath5k_hw *hal)
{
	AR5K_TRACE;
	AR5K_REG_ENABLE_BITS(hal, AR5K_BEACON, AR5K_BEACON_RESET_TSF);
}

/*
 * Initialize beacon timers
 */
void ath5k_hw_init_beacon(struct ath5k_hw *hal, u32 next_beacon, u32 interval)
{
	u32 timer1, timer2, timer3;

	AR5K_TRACE;
	/*
	 * Set the additional timers by mode
	 */
	switch (hal->ah_op_mode) {
	case IEEE80211_IF_TYPE_STA:
		if (hal->ah_version == AR5K_AR5210) {
			timer1 = 0xffffffff;
			timer2 = 0xffffffff;
		} else {
			timer1 = 0x0000ffff;
			timer2 = 0x0007ffff;
		}
		break;

	default:
		timer1 = (next_beacon - AR5K_TUNE_DMA_BEACON_RESP) <<
			0x00000003;
		timer2 = (next_beacon - AR5K_TUNE_SW_BEACON_RESP) <<
			0x00000003;
	}

	timer3 = next_beacon + (hal->ah_atim_window ? hal->ah_atim_window : 1);

	/*
	 * Set the beacon register and enable all timers.
	 * (next beacon, DMA beacon, software beacon, ATIM window time)
	 */
	ath5k_hw_reg_write(hal, next_beacon, AR5K_TIMER0);
	ath5k_hw_reg_write(hal, timer1, AR5K_TIMER1);
	ath5k_hw_reg_write(hal, timer2, AR5K_TIMER2);
	ath5k_hw_reg_write(hal, timer3, AR5K_TIMER3);

	ath5k_hw_reg_write(hal, interval & 
			(AR5K_BEACON_PERIOD | AR5K_BEACON_RESET_TSF | 
			 AR5K_BEACON_ENABLE), AR5K_BEACON);
}

/*
 * Set beacon timers
 */
int ath5k_hw_set_beacon_timers(struct ath5k_hw *hal,
		const struct ath5k_beacon_state *state)
{
	u32 cfp_period, next_cfp, dtim, interval, next_beacon;

	/*
	 * TODO: should be changed through *state
	 * review struct ath5k_beacon_state struct
	 *
	 * XXX: These are used for cfp period bellow, are they
	 * ok ? Is it O.K. for tsf here to be 0 or should we use
	 * get_tsf ?
	 */
	u32 dtim_count = 0; /* XXX */
	u32 cfp_count = 0; /* XXX */
	u32 tsf = 0; /* XXX */

	AR5K_TRACE;
	/* Return on an invalid beacon state */
	if (state->bs_interval < 1)
		return -EINVAL;

	interval = state->bs_interval;
	dtim = state->bs_dtim_period;

	/*
	 * PCF support?
	 */
	if (state->bs_cfp_period > 0) {
		/*
		 * Enable PCF mode and set the CFP
		 * (Contention Free Period) and timer registers
		 */
		cfp_period = state->bs_cfp_period * state->bs_dtim_period *
			state->bs_interval;
		next_cfp = (cfp_count * state->bs_dtim_period + dtim_count) *
			state->bs_interval;

		AR5K_REG_ENABLE_BITS(hal, AR5K_STA_ID1,
				AR5K_STA_ID1_DEFAULT_ANTENNA |
				AR5K_STA_ID1_PCF);
		ath5k_hw_reg_write(hal, cfp_period, AR5K_CFP_PERIOD);
		ath5k_hw_reg_write(hal, state->bs_cfp_max_duration,
				AR5K_CFP_DUR);
		ath5k_hw_reg_write(hal, (tsf + (next_cfp == 0 ? cfp_period :
						next_cfp)) << 3, AR5K_TIMER2);
	} else {
		/* Disable PCF mode */
		AR5K_REG_DISABLE_BITS(hal, AR5K_STA_ID1,
				AR5K_STA_ID1_DEFAULT_ANTENNA |
				AR5K_STA_ID1_PCF);
	}

	/*
	 * Enable the beacon timer register
	 */
	ath5k_hw_reg_write(hal, state->bs_next_beacon, AR5K_TIMER0);

	/*
	 * Start the beacon timers
	 */
	ath5k_hw_reg_write(hal, (ath5k_hw_reg_read(hal, AR5K_BEACON) &
				~(AR5K_BEACON_PERIOD | AR5K_BEACON_TIM)) |
			AR5K_REG_SM(state->bs_tim_offset ? 
				state->bs_tim_offset + 4 : 0,
				AR5K_BEACON_TIM) | 
			AR5K_REG_SM(state->bs_interval, AR5K_BEACON_PERIOD), 
			AR5K_BEACON);

	/*
	 * Write new beacon miss threshold, if it appears to be valid
	 * XXX: Figure out right values for min <= bs_bmiss_threshold <= max
	 * and return if its not in range. We can test this by reading value and
	 * setting value to a largest value and seeing which values register.
	 */

	AR5K_REG_WRITE_BITS(hal, AR5K_RSSI_THR, AR5K_RSSI_THR_BMISS,
			state->bs_bmiss_threshold);

	/*
	 * Set sleep control register
	 * XXX: Didn't find this in 5210 code but since this register
	 * exists also in ar5k's 5210 headers i leave it as common code.
	 */
	AR5K_REG_WRITE_BITS(hal, AR5K_SLEEP_CTL, AR5K_SLEEP_CTL_SLDUR,
			(state->bs_sleep_duration - 3) << 3);

	/*
	 * Set enhanced sleep registers on 5212
	 */
	if (hal->ah_version == AR5K_AR5212) {
		if ((state->bs_sleep_duration > state->bs_interval) &&
				(roundup(state->bs_sleep_duration, interval) ==
				 state->bs_sleep_duration))
			interval = state->bs_sleep_duration;

		if ((state->bs_sleep_duration > dtim) && (dtim == 0 ||
				(roundup(state->bs_sleep_duration, dtim) ==
				 state->bs_sleep_duration)))
			dtim = state->bs_sleep_duration;

		if (interval > dtim)
			return -EINVAL;

		next_beacon = (interval == dtim) ? state->bs_next_dtim :
			state->bs_next_beacon;

		ath5k_hw_reg_write(hal,
			AR5K_REG_SM((state->bs_next_dtim - 3) << 3,
			AR5K_SLEEP0_NEXT_DTIM) |
			AR5K_REG_SM(10, AR5K_SLEEP0_CABTO) |
			AR5K_SLEEP0_ENH_SLEEP_EN |
			AR5K_SLEEP0_ASSUME_DTIM, AR5K_SLEEP0);

		ath5k_hw_reg_write(hal, AR5K_REG_SM((next_beacon - 3) << 3,
			AR5K_SLEEP1_NEXT_TIM) |
			AR5K_REG_SM(10, AR5K_SLEEP1_BEACON_TO), AR5K_SLEEP1);

		ath5k_hw_reg_write(hal,
			AR5K_REG_SM(interval, AR5K_SLEEP2_TIM_PER) |
			AR5K_REG_SM(dtim, AR5K_SLEEP2_DTIM_PER), AR5K_SLEEP2);
	}

	return 0;
}

/*
 * Reset beacon timers
 */
void ath5k_hw_reset_beacon(struct ath5k_hw *hal)
{
	AR5K_TRACE;
	/*
	 * Disable beacon timer
	 */
	ath5k_hw_reg_write(hal, 0, AR5K_TIMER0);

	/*
	 * Disable some beacon register values
	 */
	AR5K_REG_DISABLE_BITS(hal, AR5K_STA_ID1,
			AR5K_STA_ID1_DEFAULT_ANTENNA | AR5K_STA_ID1_PCF);
	ath5k_hw_reg_write(hal, AR5K_BEACON_PERIOD, AR5K_BEACON);
}

/*
 * Wait for beacon queue to finish
 * TODO: This function's name is misleading, rename
 */
int ath5k_hw_wait_for_beacon(struct ath5k_hw *hal, unsigned long phys_addr)
{
	unsigned int i;
	int ret;

	AR5K_TRACE;

	/* 5210 doesn't have QCU*/
	if (hal->ah_version == AR5K_AR5210) {
		/*
		 * Wait for beaconn queue to finish by checking
		 * Control Register and Beacon Status Register.
		 */
		for (i = AR5K_TUNE_BEACON_INTERVAL / 2; i > 0; i--) {
			if (!(ath5k_hw_reg_read(hal, AR5K_BSR) & 
						AR5K_BSR_TXQ1F) ||
			    !(ath5k_hw_reg_read(hal, AR5K_CR) & 
				    AR5K_BSR_TXQ1F))
				break;
			udelay(10);
		}

		/* Timeout... */
		if (i <= 0) {
			/*
			 * Re-schedule the beacon queue
			 */
			ath5k_hw_reg_write(hal, phys_addr, AR5K_NOQCU_TXDP1);
			ath5k_hw_reg_write(hal, AR5K_BCR_TQ1V | AR5K_BCR_BDMAE,
					AR5K_BCR);

			return -EIO;
		}
		ret = 0;
	} else {
		/*5211/5212*/
		ret = ath5k_hw_register_timeout(hal,
			AR5K_QUEUE_STATUS(AR5K_TX_QUEUE_ID_BEACON),
			AR5K_QCU_STS_FRMPENDCNT, 0, false);

		if (AR5K_REG_READ_Q(hal, AR5K_QCU_TXE, AR5K_TX_QUEUE_ID_BEACON))
			return -EIO;
	}

	return ret;
}

/*
 * Update mib counters (statistics)
 */
void ath5k_hw_update_mib_counters(struct ath5k_hw *hal,
		struct ath5k_mib_stats *statistics)
{
	AR5K_TRACE;
	/* Read-And-Clear */
	statistics->ackrcv_bad += ath5k_hw_reg_read(hal, AR5K_ACK_FAIL);
	statistics->rts_bad += ath5k_hw_reg_read(hal, AR5K_RTS_FAIL);
	statistics->rts_good += ath5k_hw_reg_read(hal, AR5K_RTS_OK);
	statistics->fcs_bad += ath5k_hw_reg_read(hal, AR5K_FCS_FAIL);
	statistics->beacons += ath5k_hw_reg_read(hal, AR5K_BEACON_CNT);

	/* Reset profile count registers on 5212*/
	if (hal->ah_version == AR5K_AR5212) {
		ath5k_hw_reg_write(hal, 0, AR5K_PROFCNT_TX);
		ath5k_hw_reg_write(hal, 0, AR5K_PROFCNT_RX);
		ath5k_hw_reg_write(hal, 0, AR5K_PROFCNT_RXCLR);
		ath5k_hw_reg_write(hal, 0, AR5K_PROFCNT_CYCLE);
	}
}

/*
 * ACK/CTS Timeouts
 */

/*
 * Set ACK timeout on PCU
 */
int ath5k_hw_set_ack_timeout(struct ath5k_hw *hal, unsigned int timeout)
{
	AR5K_TRACE;
	if (ath5k_hw_clocktoh(AR5K_REG_MS(0xffffffff, AR5K_TIME_OUT_ACK),
				hal->ah_turbo) <= timeout)
		return -EINVAL;

	AR5K_REG_WRITE_BITS(hal, AR5K_TIME_OUT, AR5K_TIME_OUT_ACK,
			ath5k_hw_htoclock(timeout, hal->ah_turbo));

	return 0;
}

/*
 * Read the ACK timeout from PCU
 */
unsigned int ath5k_hw_get_ack_timeout(struct ath5k_hw *hal)
{
	AR5K_TRACE;

	return ath5k_hw_clocktoh(AR5K_REG_MS(ath5k_hw_reg_read(hal, 
					AR5K_TIME_OUT), AR5K_TIME_OUT_ACK), 
			hal->ah_turbo);
}

/*
 * Set CTS timeout on PCU
 */
int ath5k_hw_set_cts_timeout(struct ath5k_hw *hal, unsigned int timeout)
{
	AR5K_TRACE;
	if (ath5k_hw_clocktoh(AR5K_REG_MS(0xffffffff, AR5K_TIME_OUT_CTS),
				hal->ah_turbo) <= timeout)
		return -EINVAL;

	AR5K_REG_WRITE_BITS(hal, AR5K_TIME_OUT, AR5K_TIME_OUT_CTS,
			ath5k_hw_htoclock(timeout, hal->ah_turbo));

	return 0;
}

/*
 * Read CTS timeout from PCU
 */
unsigned int ath5k_hw_get_cts_timeout(struct ath5k_hw *hal)
{
	AR5K_TRACE;
	return ath5k_hw_clocktoh(AR5K_REG_MS(ath5k_hw_reg_read(hal,
					AR5K_TIME_OUT), AR5K_TIME_OUT_CTS), 
			hal->ah_turbo);
}

/*
 * Key table (WEP) functions
 */

int ath5k_hw_reset_key(struct ath5k_hw *hal, u16 entry)
{
	unsigned int i;

	AR5K_TRACE;
	AR5K_ASSERT_ENTRY(entry, AR5K_KEYTABLE_SIZE);

	for (i = 0; i < AR5K_KEYCACHE_SIZE; i++)
		ath5k_hw_reg_write(hal, 0, AR5K_KEYTABLE_OFF(entry, i));

	/* Set NULL encryption on non-5210*/
	if (hal->ah_version != AR5K_AR5210)
		ath5k_hw_reg_write(hal, AR5K_KEYTABLE_TYPE_NULL,
				AR5K_KEYTABLE_TYPE(entry));

	return 0;
}

int ath5k_hw_is_key_valid(struct ath5k_hw *hal, u16 entry)
{
	AR5K_TRACE;
	AR5K_ASSERT_ENTRY(entry, AR5K_KEYTABLE_SIZE);

	/* Check the validation flag at the end of the entry */
	return ath5k_hw_reg_read(hal, AR5K_KEYTABLE_MAC1(entry)) &
		AR5K_KEYTABLE_VALID;
}

int ath5k_hw_set_key(struct ath5k_hw *hal, u16 entry,
		const struct ieee80211_key_conf *key, const u8 *mac)
{
	unsigned int i;
	__le32 key_v[5] = {};
	u32 keytype;

	AR5K_TRACE;
	AR5K_ASSERT_ENTRY(entry, AR5K_KEYTABLE_SIZE);

	switch (key->keylen) {
	case 40 / 8:
		memcpy(&key_v[0], key->key, 5);
		keytype = AR5K_KEYTABLE_TYPE_40;
		break;

	case 104 / 8:
		memcpy(&key_v[0], &key->key[0], 6);
		memcpy(&key_v[2], &key->key[6], 6);
		memcpy(&key_v[4], &key->key[12], 1);
		keytype = AR5K_KEYTABLE_TYPE_104;
		break;

	case 128 / 8:
		memcpy(&key_v[0], &key->key[0], 6);
		memcpy(&key_v[2], &key->key[6], 6);
		memcpy(&key_v[4], &key->key[12], 4);
		keytype = AR5K_KEYTABLE_TYPE_128;
		break;

	default:
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(key_v); i++)
		ath5k_hw_reg_write(hal, le32_to_cpu(key_v[i]),
				AR5K_KEYTABLE_OFF(entry, i));

	ath5k_hw_reg_write(hal, keytype, AR5K_KEYTABLE_TYPE(entry));

	return ath5k_hw_set_key_lladdr(hal, entry, mac);
}

int ath5k_hw_set_key_lladdr(struct ath5k_hw *hal, u16 entry, const u8 *mac)
{
	u32 low_id, high_id;

	AR5K_TRACE;
	 /* Invalid entry (key table overflow) */
	AR5K_ASSERT_ENTRY(entry, AR5K_KEYTABLE_SIZE);

	/* MAC may be NULL if it's a broadcast key. In this case no need to
	 * to compute AR5K_LOW_ID and AR5K_HIGH_ID as we already know it. */
	if (unlikely(mac == NULL)) {
		low_id = 0xffffffff;
		high_id = 0xffff | AR5K_KEYTABLE_VALID;
	} else {
		low_id = AR5K_LOW_ID(mac);
		high_id = AR5K_HIGH_ID(mac) | AR5K_KEYTABLE_VALID;
	}

	ath5k_hw_reg_write(hal, low_id, AR5K_KEYTABLE_MAC0(entry));
	ath5k_hw_reg_write(hal, high_id, AR5K_KEYTABLE_MAC1(entry));

	return 0;
}


/********************************************\
Queue Control Unit, DFS Control Unit Functions
\********************************************/

/*
 * Initialize a transmit queue
 */
int ath5k_hw_setup_tx_queue(struct ath5k_hw *hal, enum ath5k_tx_queue queue_type,
		struct ath5k_txq_info *queue_info)
{
	unsigned int queue;
	int ret;

	AR5K_TRACE;

	/*
	 * Get queue by type
	 */
	/*5210 only has 2 queues*/
	if (hal->ah_version == AR5K_AR5210) {
		switch (queue_type) {
		case AR5K_TX_QUEUE_DATA:
			queue = AR5K_TX_QUEUE_ID_NOQCU_DATA;
			break;
		case AR5K_TX_QUEUE_BEACON:
		case AR5K_TX_QUEUE_CAB:
			queue = AR5K_TX_QUEUE_ID_NOQCU_BEACON;
			break;
		default:
			return -EINVAL;
		}
	} else {
		switch (queue_type) {
		case AR5K_TX_QUEUE_DATA:
			for (queue = AR5K_TX_QUEUE_ID_DATA_MIN;
				hal->ah_txq[queue].tqi_type !=
				AR5K_TX_QUEUE_INACTIVE; queue++) {

				if (queue > AR5K_TX_QUEUE_ID_DATA_MAX)
					return -EINVAL;
			}
			break;
		case AR5K_TX_QUEUE_UAPSD:
			queue = AR5K_TX_QUEUE_ID_UAPSD;
			break;
		case AR5K_TX_QUEUE_BEACON:
			queue = AR5K_TX_QUEUE_ID_BEACON;
			break;
		case AR5K_TX_QUEUE_CAB:
			queue = AR5K_TX_QUEUE_ID_CAB;
			break;
		case AR5K_TX_QUEUE_XR_DATA:
			if (hal->ah_version != AR5K_AR5212)
				AR5K_PRINTF("XR data queues only supported in "
						"5212!\n");
			queue = AR5K_TX_QUEUE_ID_XR_DATA;
			break;
		default:
			return -EINVAL;
		}
	}

	/*
	 * Setup internal queue structure
	 */
	memset(&hal->ah_txq[queue], 0, sizeof(struct ath5k_txq_info));
	hal->ah_txq[queue].tqi_type = queue_type;

	if (queue_info != NULL) {
		queue_info->tqi_type = queue_type;
		ret = ath5k_hw_setup_tx_queueprops(hal, queue, queue_info);
		if (ret)
			return ret;
	}
	/*
	 * We use ah_txq_interrupts to hold a temp value for
	 * the Secondary interrupt mask registers on 5211+
	 * check out ath5k_hw_reset_tx_queue
	 */
	AR5K_Q_ENABLE_BITS(hal->ah_txq_interrupts, queue);

	return queue;
}

/*
 * Setup a transmit queue
 */
int ath5k_hw_setup_tx_queueprops(struct ath5k_hw *hal, int queue,
				const struct ath5k_txq_info *queue_info)
{
	AR5K_TRACE;
	AR5K_ASSERT_ENTRY(queue, hal->ah_capabilities.cap_queues.q_tx_num);

	if (hal->ah_txq[queue].tqi_type == AR5K_TX_QUEUE_INACTIVE)
		return -EIO;

	memcpy(&hal->ah_txq[queue], queue_info, sizeof(struct ath5k_txq_info));

	/*XXX: Is this supported on 5210 ?*/
	if ((queue_info->tqi_type == AR5K_TX_QUEUE_DATA &&
			((queue_info->tqi_subtype == AR5K_WME_AC_VI) ||
			(queue_info->tqi_subtype == AR5K_WME_AC_VO))) ||
			queue_info->tqi_type == AR5K_TX_QUEUE_UAPSD)
		hal->ah_txq[queue].tqi_flags |= AR5K_TXQ_FLAG_POST_FR_BKOFF_DIS;

	return 0;
}

/*
 * Get properties for a specific transmit queue
 */
int ath5k_hw_get_tx_queueprops(struct ath5k_hw *hal, int queue,
		struct ath5k_txq_info *queue_info)
{
	AR5K_TRACE;
	memcpy(queue_info, &hal->ah_txq[queue], sizeof(struct ath5k_txq_info));
	return 0;
}

/*
 * Set a transmit queue inactive
 */
void ath5k_hw_release_tx_queue(struct ath5k_hw *hal, unsigned int queue)
{
	AR5K_TRACE;
	if (WARN_ON(queue >= hal->ah_capabilities.cap_queues.q_tx_num))
		return;

	/* This queue will be skipped in further operations */
	hal->ah_txq[queue].tqi_type = AR5K_TX_QUEUE_INACTIVE;
	/*For SIMR setup*/
	AR5K_Q_DISABLE_BITS(hal->ah_txq_interrupts, queue);
}

/*
 * Set DFS params for a transmit queue
 */
int ath5k_hw_reset_tx_queue(struct ath5k_hw *hal, unsigned int queue)
{
	u32 cw_min, cw_max, retry_lg, retry_sh;
	struct ath5k_txq_info *tq = &hal->ah_txq[queue];

	AR5K_TRACE;
	AR5K_ASSERT_ENTRY(queue, hal->ah_capabilities.cap_queues.q_tx_num);

	tq = &hal->ah_txq[queue];

	if (tq->tqi_type == AR5K_TX_QUEUE_INACTIVE)
		return 0;

	if (hal->ah_version == AR5K_AR5210) {
		/* Only handle data queues, others will be ignored */
		if (tq->tqi_type != AR5K_TX_QUEUE_DATA)
			return -EINVAL;

		/* Set Slot time */
		ath5k_hw_reg_write(hal, hal->ah_turbo == true ?
			AR5K_INIT_SLOT_TIME_TURBO : AR5K_INIT_SLOT_TIME,
			AR5K_SLOT_TIME);
		/* Set ACK_CTS timeout */
		ath5k_hw_reg_write(hal, hal->ah_turbo == true ?
			AR5K_INIT_ACK_CTS_TIMEOUT_TURBO :
			AR5K_INIT_ACK_CTS_TIMEOUT, AR5K_SLOT_TIME);
		/* Set Transmit Latency */
		ath5k_hw_reg_write(hal, hal->ah_turbo == true ?
			AR5K_INIT_TRANSMIT_LATENCY_TURBO :
			AR5K_INIT_TRANSMIT_LATENCY, AR5K_USEC_5210);
		/* Set IFS0 */
		if (hal->ah_turbo == true)
			 ath5k_hw_reg_write(hal, ((AR5K_INIT_SIFS_TURBO +
				(hal->ah_aifs + tq->tqi_aifs) *
				AR5K_INIT_SLOT_TIME_TURBO) <<
				AR5K_IFS0_DIFS_S) | AR5K_INIT_SIFS_TURBO,
				AR5K_IFS0);
		else
			ath5k_hw_reg_write(hal, ((AR5K_INIT_SIFS +
				(hal->ah_aifs + tq->tqi_aifs) *
				AR5K_INIT_SLOT_TIME) << AR5K_IFS0_DIFS_S) |
				AR5K_INIT_SIFS, AR5K_IFS0);

		/* Set IFS1 */
		ath5k_hw_reg_write(hal, hal->ah_turbo == true ?
			AR5K_INIT_PROTO_TIME_CNTRL_TURBO :
			AR5K_INIT_PROTO_TIME_CNTRL, AR5K_IFS1);
		/* Set PHY register 0x9844 (??) */
		ath5k_hw_reg_write(hal, hal->ah_turbo == true ?
			(ath5k_hw_reg_read(hal, AR5K_PHY(17)) & ~0x7F) | 0x38 :
			(ath5k_hw_reg_read(hal, AR5K_PHY(17)) & ~0x7F) | 0x1C,
			AR5K_PHY(17));
		/* Set Frame Control Register */
		ath5k_hw_reg_write(hal, hal->ah_turbo == true ?
			(AR5K_PHY_FRAME_CTL_INI | AR5K_PHY_TURBO_MODE |
			AR5K_PHY_TURBO_SHORT | 0x2020) :
			(AR5K_PHY_FRAME_CTL_INI | 0x1020),
			AR5K_PHY_FRAME_CTL_5210);
	}

	/*
	 * Calculate cwmin/max by channel mode
	 */
	cw_min = hal->ah_cw_min = AR5K_TUNE_CWMIN;
	cw_max = hal->ah_cw_max = AR5K_TUNE_CWMAX;
	hal->ah_aifs = AR5K_TUNE_AIFS;
	/*XR is only supported on 5212*/
	if (IS_CHAN_XR(hal->ah_current_channel) &&
			(hal->ah_version == AR5K_AR5212)) {
		cw_min = hal->ah_cw_min = AR5K_TUNE_CWMIN_XR;
		cw_max = hal->ah_cw_max = AR5K_TUNE_CWMAX_XR;
		hal->ah_aifs = AR5K_TUNE_AIFS_XR;
	/*B mode is not supported on 5210*/
	} else if (IS_CHAN_B(hal->ah_current_channel) && 
			(hal->ah_version != AR5K_AR5210)) {
		cw_min = hal->ah_cw_min = AR5K_TUNE_CWMIN_11B;
		cw_max = hal->ah_cw_max = AR5K_TUNE_CWMAX_11B;
		hal->ah_aifs = AR5K_TUNE_AIFS_11B;
	}

	cw_min = 1;
	while (cw_min < hal->ah_cw_min)
		cw_min = (cw_min << 1) | 1;

	cw_min = tq->tqi_cw_min < 0 ? 
		(cw_min >> (-tq->tqi_cw_min)) :
		((cw_min << tq->tqi_cw_min) + (1 << tq->tqi_cw_min) - 1);
	cw_max = tq->tqi_cw_max < 0 ? 
		(cw_max >> (-tq->tqi_cw_max)) :
		((cw_max << tq->tqi_cw_max) + (1 << tq->tqi_cw_max) - 1);

	/*
	 * Calculate and set retry limits
	 */
	if (hal->ah_software_retry == true) {
		/* XXX Need to test this */
		retry_lg = hal->ah_limit_tx_retries;
		retry_sh = retry_lg = 
			(retry_lg > AR5K_DCU_RETRY_LMT_SH_RETRY) ?
			AR5K_DCU_RETRY_LMT_SH_RETRY : retry_lg;
	} else {
		retry_lg = AR5K_INIT_LG_RETRY;
		retry_sh = AR5K_INIT_SH_RETRY;
	}

	/*No QCU/DCU [5210]*/
	if (hal->ah_version == AR5K_AR5210) {
		ath5k_hw_reg_write(hal,
				(cw_min << AR5K_NODCU_RETRY_LMT_CW_MIN_S)	|
				AR5K_REG_SM(AR5K_INIT_SLG_RETRY,
					AR5K_NODCU_RETRY_LMT_SLG_RETRY)		|
				AR5K_REG_SM(AR5K_INIT_SSH_RETRY,
					AR5K_NODCU_RETRY_LMT_SSH_RETRY)		|
				AR5K_REG_SM(retry_lg, 
					AR5K_NODCU_RETRY_LMT_LG_RETRY)		|
				AR5K_REG_SM(retry_sh, 
					AR5K_NODCU_RETRY_LMT_SH_RETRY),
				AR5K_NODCU_RETRY_LMT);
	} else {
		/*QCU/DCU [5211+]*/
		ath5k_hw_reg_write(hal,
			AR5K_REG_SM(AR5K_INIT_SLG_RETRY,
				AR5K_DCU_RETRY_LMT_SLG_RETRY) |
			AR5K_REG_SM(AR5K_INIT_SSH_RETRY,
				AR5K_DCU_RETRY_LMT_SSH_RETRY) |
			AR5K_REG_SM(retry_lg, AR5K_DCU_RETRY_LMT_LG_RETRY) |
			AR5K_REG_SM(retry_sh, AR5K_DCU_RETRY_LMT_SH_RETRY),
			AR5K_QUEUE_DFS_RETRY_LIMIT(queue));

	/*===Rest is also for QCU/DCU only [5211+]===*/

		/*
		 * Set initial content window (cw_min/cw_max)
		 * and arbitrated interframe space (aifs)...
		 */
		ath5k_hw_reg_write(hal,
			AR5K_REG_SM(cw_min, AR5K_DCU_LCL_IFS_CW_MIN) |
			AR5K_REG_SM(cw_max, AR5K_DCU_LCL_IFS_CW_MAX) |
			AR5K_REG_SM(hal->ah_aifs + tq->tqi_aifs,
				AR5K_DCU_LCL_IFS_AIFS),
			AR5K_QUEUE_DFS_LOCAL_IFS(queue));

		/*
		 * Set misc registers
		 */
		ath5k_hw_reg_write(hal, AR5K_QCU_MISC_DCU_EARLY,
			AR5K_QUEUE_MISC(queue));

		if (tq->tqi_cbr_period) {
			ath5k_hw_reg_write(hal, AR5K_REG_SM(tq->tqi_cbr_period,
				AR5K_QCU_CBRCFG_INTVAL) |
				AR5K_REG_SM(tq->tqi_cbr_overflow_limit,
				AR5K_QCU_CBRCFG_ORN_THRES),
				AR5K_QUEUE_CBRCFG(queue));
			AR5K_REG_ENABLE_BITS(hal, AR5K_QUEUE_MISC(queue),
				AR5K_QCU_MISC_FRSHED_CBR);
			if (tq->tqi_cbr_overflow_limit)
				AR5K_REG_ENABLE_BITS(hal,
					AR5K_QUEUE_MISC(queue),
					AR5K_QCU_MISC_CBR_THRES_ENABLE);
		}

		if (tq->tqi_ready_time)
			ath5k_hw_reg_write(hal, AR5K_REG_SM(tq->tqi_ready_time,
				AR5K_QCU_RDYTIMECFG_INTVAL) |
				AR5K_QCU_RDYTIMECFG_ENABLE,
				AR5K_QUEUE_RDYTIMECFG(queue));

		if (tq->tqi_burst_time) {
			ath5k_hw_reg_write(hal, AR5K_REG_SM(tq->tqi_burst_time,
				AR5K_DCU_CHAN_TIME_DUR) |
				AR5K_DCU_CHAN_TIME_ENABLE,
				AR5K_QUEUE_DFS_CHANNEL_TIME(queue));

			if (tq->tqi_flags & AR5K_TXQ_FLAG_RDYTIME_EXP_POLICY_ENABLE)
				AR5K_REG_ENABLE_BITS(hal,
					AR5K_QUEUE_MISC(queue),
					AR5K_QCU_MISC_TXE);
		}

		if (tq->tqi_flags & AR5K_TXQ_FLAG_BACKOFF_DISABLE)
			ath5k_hw_reg_write(hal, AR5K_DCU_MISC_POST_FR_BKOFF_DIS,
					AR5K_QUEUE_DFS_MISC(queue));

		if (tq->tqi_flags & AR5K_TXQ_FLAG_FRAG_BURST_BACKOFF_ENABLE)
			ath5k_hw_reg_write(hal, AR5K_DCU_MISC_BACKOFF_FRAG,
					AR5K_QUEUE_DFS_MISC(queue));

		/*
		 * Set registers by queue type
		 */
		switch (tq->tqi_type) {
		case AR5K_TX_QUEUE_BEACON:
			AR5K_REG_ENABLE_BITS(hal, AR5K_QUEUE_MISC(queue),
				AR5K_QCU_MISC_FRSHED_DBA_GT |
				AR5K_QCU_MISC_CBREXP_BCN |
				AR5K_QCU_MISC_BCN_ENABLE);

			AR5K_REG_ENABLE_BITS(hal, AR5K_QUEUE_DFS_MISC(queue),
				(AR5K_DCU_MISC_ARBLOCK_CTL_GLOBAL <<
				AR5K_DCU_MISC_ARBLOCK_CTL_S) |
				AR5K_DCU_MISC_POST_FR_BKOFF_DIS |
				AR5K_DCU_MISC_BCN_ENABLE);

			ath5k_hw_reg_write(hal, ((AR5K_TUNE_BEACON_INTERVAL -
				(AR5K_TUNE_SW_BEACON_RESP -
				AR5K_TUNE_DMA_BEACON_RESP) -
				AR5K_TUNE_ADDITIONAL_SWBA_BACKOFF) * 1024) |
				AR5K_QCU_RDYTIMECFG_ENABLE,
				AR5K_QUEUE_RDYTIMECFG(queue));
			break;

		case AR5K_TX_QUEUE_CAB:
			AR5K_REG_ENABLE_BITS(hal, AR5K_QUEUE_MISC(queue),
				AR5K_QCU_MISC_FRSHED_DBA_GT |
				AR5K_QCU_MISC_CBREXP |
				AR5K_QCU_MISC_CBREXP_BCN);

			AR5K_REG_ENABLE_BITS(hal, AR5K_QUEUE_DFS_MISC(queue),
				(AR5K_DCU_MISC_ARBLOCK_CTL_GLOBAL <<
				AR5K_DCU_MISC_ARBLOCK_CTL_S));
			break;

		case AR5K_TX_QUEUE_UAPSD:
			AR5K_REG_ENABLE_BITS(hal, AR5K_QUEUE_MISC(queue),
				AR5K_QCU_MISC_CBREXP);
			break;

		case AR5K_TX_QUEUE_DATA:
		default:
			break;
		}

		/*
		 * Enable tx queue in the secondary interrupt mask registers
		 */
		ath5k_hw_reg_write(hal, AR5K_REG_SM(hal->ah_txq_interrupts,
			AR5K_SIMR0_QCU_TXOK) |
			AR5K_REG_SM(hal->ah_txq_interrupts,
			AR5K_SIMR0_QCU_TXDESC), AR5K_SIMR0);
		ath5k_hw_reg_write(hal, AR5K_REG_SM(hal->ah_txq_interrupts,
			AR5K_SIMR1_QCU_TXERR), AR5K_SIMR1);
		ath5k_hw_reg_write(hal, AR5K_REG_SM(hal->ah_txq_interrupts,
			AR5K_SIMR2_QCU_TXURN), AR5K_SIMR2);
	}

	return 0;
}

/*
 * Get number of pending frames
 * for a specific queue [5211+]
 */
u32 ath5k_hw_num_tx_pending(struct ath5k_hw *hal, unsigned int queue) {
	AR5K_TRACE;
	AR5K_ASSERT_ENTRY(queue, hal->ah_capabilities.cap_queues.q_tx_num);

	/* Return if queue is declared inactive */
	if (hal->ah_txq[queue].tqi_type == AR5K_TX_QUEUE_INACTIVE)
		return false;

	/* XXX: How about AR5K_CFG_TXCNT ? */
	if (hal->ah_version == AR5K_AR5210)
		return false;

	return AR5K_QUEUE_STATUS(queue) & AR5K_QCU_STS_FRMPENDCNT;
}

/*
 * Set slot time
 */
int ath5k_hw_set_slot_time(struct ath5k_hw *hal, unsigned int slot_time)
{
	AR5K_TRACE;
	if (slot_time < AR5K_SLOT_TIME_9 || slot_time > AR5K_SLOT_TIME_MAX)
		return -EINVAL;

	if (hal->ah_version == AR5K_AR5210)
		ath5k_hw_reg_write(hal, ath5k_hw_htoclock(slot_time,
				hal->ah_turbo), AR5K_SLOT_TIME);
	else
		ath5k_hw_reg_write(hal, slot_time, AR5K_DCU_GBL_IFS_SLOT);

	return 0;
}

/*
 * Get slot time
 */
unsigned int ath5k_hw_get_slot_time(struct ath5k_hw *hal)
{
	AR5K_TRACE;
	if (hal->ah_version == AR5K_AR5210)
		return ath5k_hw_clocktoh(ath5k_hw_reg_read(hal,
					AR5K_SLOT_TIME) & 0xffff, hal->ah_turbo);
	else
		return ath5k_hw_reg_read(hal, AR5K_DCU_GBL_IFS_SLOT) & 0xffff;
}


/******************************\
 Hardware Descriptor Functions
\******************************/

/*
 * TX Descriptor
 */

/*
 * Initialize the 2-word tx descriptor on 5210/5211
 */
static int
ath5k_hw_setup_2word_tx_desc(struct ath5k_hw *hal, struct ath5k_desc *desc,
	unsigned int pkt_len, unsigned int hdr_len, enum ath5k_pkt_type type,
	unsigned int tx_power, unsigned int tx_rate0, unsigned int tx_tries0,
	unsigned int key_index, unsigned int antenna_mode, unsigned int flags,
	unsigned int rtscts_rate, unsigned int rtscts_duration)
{
	u32 frame_type;
	struct ath5k_hw_2w_tx_desc *tx_desc;
	
	tx_desc = (struct ath5k_hw_2w_tx_desc *)&desc->ds_ctl0;

	if (tx_tries0 == 0)
		return -EINVAL;

	/* Initialize control descriptor */
	tx_desc->tx_control_0 = 0;
	tx_desc->tx_control_1 = 0;

	/* Setup control descriptor */

	/*Verify packet length*/
	tx_desc->tx_control_0 = pkt_len & AR5K_2W_TX_DESC_CTL0_FRAME_LEN;
	if (tx_desc->tx_control_0 != pkt_len)
		return -EINVAL;
	/*
	 * Verify header length
	 * XXX: I only found that on 5210 code, does it work on 5211 ?
	 */
	if (hal->ah_version == AR5K_AR5210) {
		tx_desc->tx_control_0 = hdr_len &
				AR5K_2W_TX_DESC_CTL0_HEADER_LEN;
		if (tx_desc->tx_control_0 != hdr_len)
			return -EINVAL;
	}

	/*Diferences between 5210-5211*/
	if (hal->ah_version == AR5K_AR5210) {
		switch (type) {
		case AR5K_PKT_TYPE_BEACON:
		case AR5K_PKT_TYPE_PROBE_RESP:
			frame_type = AR5K_AR5210_TX_DESC_FRAME_TYPE_NO_DELAY;
		case AR5K_PKT_TYPE_PIFS:
			frame_type = AR5K_AR5210_TX_DESC_FRAME_TYPE_PIFS;
		default:
			frame_type = type /*<< 2 ?*/;
		}

		tx_desc->tx_control_0 =
			AR5K_REG_SM(frame_type, AR5K_2W_TX_DESC_CTL0_FRAME_TYPE) |
			AR5K_REG_SM(tx_rate0, AR5K_2W_TX_DESC_CTL0_XMIT_RATE);
	} else {
		tx_desc->tx_control_0 |=
			AR5K_REG_SM(tx_rate0, AR5K_2W_TX_DESC_CTL0_XMIT_RATE) |
			AR5K_REG_SM(antenna_mode, AR5K_2W_TX_DESC_CTL0_ANT_MODE_XMIT);
		tx_desc->tx_control_1 =
			AR5K_REG_SM(type, AR5K_2W_TX_DESC_CTL1_FRAME_TYPE);
	}
#define _TX_FLAGS(_c, _flag)						\
	if (flags & AR5K_TXDESC_##_flag)				\
		tx_desc->tx_control_##_c |=				\
			AR5K_2W_TX_DESC_CTL##_c##_##_flag

	_TX_FLAGS(0, CLRDMASK);
	_TX_FLAGS(0, VEOL);
	_TX_FLAGS(0, INTREQ);
	_TX_FLAGS(0, RTSENA);
	_TX_FLAGS(1, NOACK);

#undef _TX_FLAGS

	/*
	 * WEP crap
	 */
	if (key_index != AR5K_TXKEYIX_INVALID) {
		tx_desc->tx_control_0 |=
			AR5K_2W_TX_DESC_CTL0_ENCRYPT_KEY_VALID;
		tx_desc->tx_control_1 |=
			AR5K_REG_SM(key_index,
			AR5K_2W_TX_DESC_CTL1_ENCRYPT_KEY_INDEX);
	}

	/*
	 * RTS/CTS Duration [5210 ?]
	 */
	if ((hal->ah_version == AR5K_AR5210) &&
			(flags & (AR5K_TXDESC_RTSENA | AR5K_TXDESC_CTSENA)))
		tx_desc->tx_control_1 |= rtscts_duration &
				AR5K_2W_TX_DESC_CTL1_RTS_DURATION;

	return 0;
}

/*
 * Initialize the 4-word tx descriptor on 5212
 */
static int ath5k_hw_setup_4word_tx_desc(struct ath5k_hw *hal,
	struct ath5k_desc *desc, unsigned int pkt_len, unsigned int hdr_len,
	enum ath5k_pkt_type type, unsigned int tx_power, unsigned int tx_rate0,
	unsigned int tx_tries0, unsigned int key_index,
	unsigned int antenna_mode, unsigned int flags, unsigned int rtscts_rate,
	unsigned int rtscts_duration)
{
	struct ath5k_hw_4w_tx_desc *tx_desc;

	AR5K_TRACE;

	tx_desc = (struct ath5k_hw_4w_tx_desc *)&desc->ds_ctl0;

	/*
	 * Validate input
	 */
	if (tx_tries0 == 0)
		return -EINVAL;

	/* Initialize status descriptor */
	tx_desc->tx_control_0 = 0;
	tx_desc->tx_control_1 = 0;
	tx_desc->tx_control_2 = 0;
	tx_desc->tx_control_3 = 0;

	/* Setup status descriptor */
	tx_desc->tx_control_0 = pkt_len & AR5K_4W_TX_DESC_CTL0_FRAME_LEN;
	if (tx_desc->tx_control_0 != pkt_len)
		return -EINVAL;

	tx_desc->tx_control_0 |=
		AR5K_REG_SM(tx_power, AR5K_4W_TX_DESC_CTL0_XMIT_POWER) |
		AR5K_REG_SM(antenna_mode, AR5K_4W_TX_DESC_CTL0_ANT_MODE_XMIT);
	tx_desc->tx_control_1 = AR5K_REG_SM(type,
					AR5K_4W_TX_DESC_CTL1_FRAME_TYPE);
	tx_desc->tx_control_2 = AR5K_REG_SM(tx_tries0 + AR5K_TUNE_HWTXTRIES,
					AR5K_4W_TX_DESC_CTL2_XMIT_TRIES0);
	tx_desc->tx_control_3 = tx_rate0 & AR5K_4W_TX_DESC_CTL3_XMIT_RATE0;

#define _TX_FLAGS(_c, _flag)			\
	if (flags & AR5K_TXDESC_##_flag)	\
		tx_desc->tx_control_##_c |=	\
			AR5K_4W_TX_DESC_CTL##_c##_##_flag

	_TX_FLAGS(0, CLRDMASK);
	_TX_FLAGS(0, VEOL);
	_TX_FLAGS(0, INTREQ);
	_TX_FLAGS(0, RTSENA);
	_TX_FLAGS(0, CTSENA);
	_TX_FLAGS(1, NOACK);

#undef _TX_FLAGS

	/*
	 * WEP crap
	 */
	if (key_index != AR5K_TXKEYIX_INVALID) {
		tx_desc->tx_control_0 |= AR5K_4W_TX_DESC_CTL0_ENCRYPT_KEY_VALID;
		tx_desc->tx_control_1 |= AR5K_REG_SM(key_index,
				AR5K_4W_TX_DESC_CTL1_ENCRYPT_KEY_INDEX);
	}

	/*
	 * RTS/CTS
	 */
	if (flags & (AR5K_TXDESC_RTSENA | AR5K_TXDESC_CTSENA)) {
		if ((flags & AR5K_TXDESC_RTSENA) &&
				(flags & AR5K_TXDESC_CTSENA))
			return -EINVAL;
		tx_desc->tx_control_2 |= rtscts_duration &
			AR5K_4W_TX_DESC_CTL2_RTS_DURATION;
		tx_desc->tx_control_3 |= AR5K_REG_SM(rtscts_rate,
				AR5K_4W_TX_DESC_CTL3_RTS_CTS_RATE);
	}

	return 0;
}

/*
 * Initialize a 4-word multirate tx descriptor on 5212
 */
static bool
ath5k_hw_setup_xr_tx_desc(struct ath5k_hw *hal, struct ath5k_desc *desc,
	unsigned int tx_rate1, u_int tx_tries1, u_int tx_rate2, u_int tx_tries2,
	unsigned int tx_rate3, u_int tx_tries3)
{
	struct ath5k_hw_4w_tx_desc *tx_desc;

	if (hal->ah_version == AR5K_AR5212) {
		tx_desc = (struct ath5k_hw_4w_tx_desc *)&desc->ds_ctl0;

#define _XTX_TRIES(_n)							\
	if (tx_tries##_n) {						\
		tx_desc->tx_control_2 |=				\
		AR5K_REG_SM(tx_tries##_n,				\
				AR5K_4W_TX_DESC_CTL2_XMIT_TRIES##_n);	\
		tx_desc->tx_control_3 |=				\
		AR5K_REG_SM(tx_rate##_n,				\
				AR5K_4W_TX_DESC_CTL3_XMIT_RATE##_n);	\
	}

		_XTX_TRIES(1);
		_XTX_TRIES(2);
		_XTX_TRIES(3);

#undef _XTX_TRIES

		return true;
	}

	return false;
}

/*
 * Fill the 2-word tx descriptor on 5210/5211
 */
static int ath5k_hw_fill_2word_tx_desc(struct ath5k_hw *hal,
	struct ath5k_desc *desc, unsigned int segment_length,
	bool first_segment, bool last_segment)
{
	struct ath5k_hw_2w_tx_desc *tx_desc;

	tx_desc = (struct ath5k_hw_2w_tx_desc *)&desc->ds_ctl0;

	/* Clear status descriptor */
	memset(desc->ds_hw, 0, sizeof(desc->ds_hw));

	/* Validate segment length and initialize the descriptor */
	tx_desc->tx_control_1 = segment_length & AR5K_2W_TX_DESC_CTL1_BUF_LEN;
	if (tx_desc->tx_control_1 != segment_length)
		return -EINVAL;

	if (first_segment != true)
		tx_desc->tx_control_0 &= ~AR5K_2W_TX_DESC_CTL0_FRAME_LEN;

	if (last_segment != true)
		tx_desc->tx_control_1 |= AR5K_2W_TX_DESC_CTL1_MORE;

	return 0;
}

/*
 * Fill the 4-word tx descriptor on 5212
 * XXX: Added an argument *last_desc -need revision
 */
static int ath5k_hw_fill_4word_tx_desc(struct ath5k_hw *hal,
	struct ath5k_desc *desc, unsigned int segment_length,
	bool first_segment, bool last_segment)
{
	struct ath5k_hw_4w_tx_desc *tx_desc;
	struct ath5k_hw_tx_status *tx_status;

	AR5K_TRACE;
	tx_desc = (struct ath5k_hw_4w_tx_desc *)&desc->ds_ctl0;
	tx_status = (struct ath5k_hw_tx_status *)&desc->ds_hw[2];

	/* Clear status descriptor */
	memset(tx_status, 0, sizeof(struct ath5k_hw_tx_status));

	/* Validate segment length and initialize the descriptor */
	tx_desc->tx_control_1 = segment_length & AR5K_4W_TX_DESC_CTL1_BUF_LEN;
	if (tx_desc->tx_control_1 != segment_length)
		return -EINVAL;

	if (first_segment != true)
		tx_desc->tx_control_0 &= ~AR5K_4W_TX_DESC_CTL0_FRAME_LEN;

	if (last_segment != true)
		tx_desc->tx_control_1 |= AR5K_4W_TX_DESC_CTL1_MORE;

	return 0;
}

/*
 * Proccess the tx status descriptor on 5210/5211
 */
static int ath5k_hw_proc_2word_tx_status(struct ath5k_hw *hal,
		struct ath5k_desc *desc)
{
	struct ath5k_hw_tx_status *tx_status;
	struct ath5k_hw_2w_tx_desc *tx_desc;

	tx_desc = (struct ath5k_hw_2w_tx_desc *)&desc->ds_ctl0;
	tx_status = (struct ath5k_hw_tx_status *)&desc->ds_hw[0];

	/* No frame has been send or error */
	if (unlikely((tx_status->tx_status_1 & AR5K_DESC_TX_STATUS1_DONE) == 0))
		return -EINPROGRESS;

	/*
	 * Get descriptor status
	 */
	desc->ds_us.tx.ts_tstamp = AR5K_REG_MS(tx_status->tx_status_0,
			AR5K_DESC_TX_STATUS0_SEND_TIMESTAMP);
	desc->ds_us.tx.ts_shortretry = AR5K_REG_MS(tx_status->tx_status_0,
			AR5K_DESC_TX_STATUS0_SHORT_RETRY_COUNT);
	desc->ds_us.tx.ts_longretry = AR5K_REG_MS(tx_status->tx_status_0,
			AR5K_DESC_TX_STATUS0_LONG_RETRY_COUNT);
	/*TODO: desc->ds_us.tx.ts_virtcol + test*/
	desc->ds_us.tx.ts_seqnum = AR5K_REG_MS(tx_status->tx_status_1,
			AR5K_DESC_TX_STATUS1_SEQ_NUM);
	desc->ds_us.tx.ts_rssi = AR5K_REG_MS(tx_status->tx_status_1,
			AR5K_DESC_TX_STATUS1_ACK_SIG_STRENGTH);
	desc->ds_us.tx.ts_antenna = 1;
	desc->ds_us.tx.ts_status = 0;
	desc->ds_us.tx.ts_rate = AR5K_REG_MS(tx_desc->tx_control_0,
			AR5K_2W_TX_DESC_CTL0_XMIT_RATE);

	if (!(tx_status->tx_status_0 & AR5K_DESC_TX_STATUS0_FRAME_XMIT_OK)) {
		if (tx_status->tx_status_0 &
				AR5K_DESC_TX_STATUS0_EXCESSIVE_RETRIES)
			desc->ds_us.tx.ts_status |= AR5K_TXERR_XRETRY;

		if (tx_status->tx_status_0 & AR5K_DESC_TX_STATUS0_FIFO_UNDERRUN)
			desc->ds_us.tx.ts_status |= AR5K_TXERR_FIFO;

		if (tx_status->tx_status_0 & AR5K_DESC_TX_STATUS0_FILTERED)
			desc->ds_us.tx.ts_status |= AR5K_TXERR_FILT;
	}

	return 0;
}

/*
 * Proccess a tx descriptor on 5212
 */
static int ath5k_hw_proc_4word_tx_status(struct ath5k_hw *hal,
		struct ath5k_desc *desc)
{
	struct ath5k_hw_tx_status *tx_status;
	struct ath5k_hw_4w_tx_desc *tx_desc;

	AR5K_TRACE;
	tx_desc = (struct ath5k_hw_4w_tx_desc *)&desc->ds_ctl0;
	tx_status = (struct ath5k_hw_tx_status *)&desc->ds_hw[2];

	/* No frame has been send or error */
	if (unlikely((tx_status->tx_status_1 & AR5K_DESC_TX_STATUS1_DONE) == 0))
		return -EINPROGRESS;

	/*
	 * Get descriptor status
	 */
	desc->ds_us.tx.ts_tstamp = AR5K_REG_MS(tx_status->tx_status_0,
			AR5K_DESC_TX_STATUS0_SEND_TIMESTAMP);
	desc->ds_us.tx.ts_shortretry = AR5K_REG_MS(tx_status->tx_status_0,
			AR5K_DESC_TX_STATUS0_SHORT_RETRY_COUNT);
	desc->ds_us.tx.ts_longretry = AR5K_REG_MS(tx_status->tx_status_0,
			AR5K_DESC_TX_STATUS0_LONG_RETRY_COUNT);
	desc->ds_us.tx.ts_seqnum = AR5K_REG_MS(tx_status->tx_status_1,
			AR5K_DESC_TX_STATUS1_SEQ_NUM);
	desc->ds_us.tx.ts_rssi = AR5K_REG_MS(tx_status->tx_status_1,
			AR5K_DESC_TX_STATUS1_ACK_SIG_STRENGTH);
	desc->ds_us.tx.ts_antenna = (tx_status->tx_status_1 &
			AR5K_DESC_TX_STATUS1_XMIT_ANTENNA) ? 2 : 1;
	desc->ds_us.tx.ts_status = 0;

	switch (AR5K_REG_MS(tx_status->tx_status_1,
			AR5K_DESC_TX_STATUS1_FINAL_TS_INDEX)) {
	case 0:
		desc->ds_us.tx.ts_rate = tx_desc->tx_control_3 &
			AR5K_4W_TX_DESC_CTL3_XMIT_RATE0;
		break;
	case 1:
		desc->ds_us.tx.ts_rate = AR5K_REG_MS(tx_desc->tx_control_3,
				AR5K_4W_TX_DESC_CTL3_XMIT_RATE1);
		desc->ds_us.tx.ts_longretry +=AR5K_REG_MS(tx_desc->tx_control_2,
				AR5K_4W_TX_DESC_CTL2_XMIT_TRIES1);
		break;
	case 2:
		desc->ds_us.tx.ts_rate = AR5K_REG_MS(tx_desc->tx_control_3,
				AR5K_4W_TX_DESC_CTL3_XMIT_RATE2);
		desc->ds_us.tx.ts_longretry +=AR5K_REG_MS(tx_desc->tx_control_2,
				AR5K_4W_TX_DESC_CTL2_XMIT_TRIES2);
		break;
	case 3:
		desc->ds_us.tx.ts_rate = AR5K_REG_MS(tx_desc->tx_control_3,
				AR5K_4W_TX_DESC_CTL3_XMIT_RATE3);
		desc->ds_us.tx.ts_longretry +=AR5K_REG_MS(tx_desc->tx_control_2,
				AR5K_4W_TX_DESC_CTL2_XMIT_TRIES3);
		break;
	}

	if ((tx_status->tx_status_0 & 
				AR5K_DESC_TX_STATUS0_FRAME_XMIT_OK) == 0) {
		if (tx_status->tx_status_0 &
				AR5K_DESC_TX_STATUS0_EXCESSIVE_RETRIES)
			desc->ds_us.tx.ts_status |= AR5K_TXERR_XRETRY;

		if (tx_status->tx_status_0 & AR5K_DESC_TX_STATUS0_FIFO_UNDERRUN)
			desc->ds_us.tx.ts_status |= AR5K_TXERR_FIFO;

		if (tx_status->tx_status_0 & AR5K_DESC_TX_STATUS0_FILTERED)
			desc->ds_us.tx.ts_status |= AR5K_TXERR_FILT;
	}

	return 0;
}

/*
 * RX Descriptor
 */

/*
 * Initialize an rx descriptor
 */
int ath5k_hw_setup_rx_desc(struct ath5k_hw *hal, struct ath5k_desc *desc,
			u32 size, unsigned int flags)
{
	struct ath5k_rx_desc *rx_desc;

	AR5K_TRACE;
	rx_desc = (struct ath5k_rx_desc *)&desc->ds_ctl0;

	/*
	 * Clear ds_hw
	 * If we don't clean the status descriptor,
	 * while scanning we get too many results,
	 * most of them virtual, after some secs
	 * of scanning system hangs. M.F.
	*/
	memset(desc->ds_hw, 0, sizeof(desc->ds_hw));

	/*Initialize rx descriptor*/
	rx_desc->rx_control_0 = 0;
	rx_desc->rx_control_1 = 0;

	/* Setup descriptor */
	rx_desc->rx_control_1 = size & AR5K_DESC_RX_CTL1_BUF_LEN;
	if (unlikely(rx_desc->rx_control_1 != size))
		return -EINVAL;

	if (flags & AR5K_RXDESC_INTREQ)
		rx_desc->rx_control_1 |= AR5K_DESC_RX_CTL1_INTREQ;

	return 0;
}

/*
 * Proccess the rx status descriptor on 5210/5211
 */
static int ath5k_hw_proc_old_rx_status(struct ath5k_hw *hal,
		struct ath5k_desc *desc)
{
	struct ath5k_hw_old_rx_status *rx_status;

	rx_status = (struct ath5k_hw_old_rx_status *)&desc->ds_hw[0];

	/* No frame received / not ready */
	if (unlikely((rx_status->rx_status_1 & AR5K_OLD_RX_DESC_STATUS1_DONE)
				== 0))
		return -EINPROGRESS;

	/*
	 * Frame receive status
	 */
	desc->ds_us.rx.rs_datalen = rx_status->rx_status_0 &
		AR5K_OLD_RX_DESC_STATUS0_DATA_LEN;
	desc->ds_us.rx.rs_rssi = AR5K_REG_MS(rx_status->rx_status_0,
			AR5K_OLD_RX_DESC_STATUS0_RECEIVE_SIGNAL);
	desc->ds_us.rx.rs_rate = AR5K_REG_MS(rx_status->rx_status_0,
			AR5K_OLD_RX_DESC_STATUS0_RECEIVE_RATE);
	desc->ds_us.rx.rs_antenna = rx_status->rx_status_0 &
		AR5K_OLD_RX_DESC_STATUS0_RECEIVE_ANTENNA;
	desc->ds_us.rx.rs_more = rx_status->rx_status_0 &
		AR5K_OLD_RX_DESC_STATUS0_MORE;
	desc->ds_us.rx.rs_tstamp = AR5K_REG_MS(rx_status->rx_status_1,
			AR5K_OLD_RX_DESC_STATUS1_RECEIVE_TIMESTAMP);
	desc->ds_us.rx.rs_status = 0;

	/*
	 * Key table status
	 */
	if (rx_status->rx_status_1 & AR5K_OLD_RX_DESC_STATUS1_KEY_INDEX_VALID)
		desc->ds_us.rx.rs_keyix = AR5K_REG_MS(rx_status->rx_status_1,
				AR5K_OLD_RX_DESC_STATUS1_KEY_INDEX);
	else
		desc->ds_us.rx.rs_keyix = AR5K_RXKEYIX_INVALID;

	/*
	 * Receive/descriptor errors
	 */
	if ((rx_status->rx_status_1 & AR5K_OLD_RX_DESC_STATUS1_FRAME_RECEIVE_OK)
			== 0) {
		if (rx_status->rx_status_1 & AR5K_OLD_RX_DESC_STATUS1_CRC_ERROR)
			desc->ds_us.rx.rs_status |= AR5K_RXERR_CRC;

		if (rx_status->rx_status_1 &
				AR5K_OLD_RX_DESC_STATUS1_FIFO_OVERRUN)
			desc->ds_us.rx.rs_status |= AR5K_RXERR_FIFO;

		if (rx_status->rx_status_1 &
				AR5K_OLD_RX_DESC_STATUS1_PHY_ERROR) {
			desc->ds_us.rx.rs_status |= AR5K_RXERR_PHY;
			desc->ds_us.rx.rs_phyerr =
				AR5K_REG_MS(rx_status->rx_status_1,
					AR5K_OLD_RX_DESC_STATUS1_PHY_ERROR);
		}

		if (rx_status->rx_status_1 &
				AR5K_OLD_RX_DESC_STATUS1_DECRYPT_CRC_ERROR)
			desc->ds_us.rx.rs_status |= AR5K_RXERR_DECRYPT;
	}

	return 0;
}

/*
 * Proccess the rx status descriptor on 5212
 */
static int ath5k_hw_proc_new_rx_status(struct ath5k_hw *hal,
		struct ath5k_desc *desc)
{
	struct ath5k_hw_new_rx_status *rx_status;
	struct ath5k_hw_rx_error *rx_err;

	AR5K_TRACE;
	rx_status = (struct ath5k_hw_new_rx_status *)&desc->ds_hw[0];

	/* Overlay on error */
	rx_err = (struct ath5k_hw_rx_error *)&desc->ds_hw[0];

	/* No frame received / not ready */
	if (unlikely((rx_status->rx_status_1 & AR5K_NEW_RX_DESC_STATUS1_DONE)
				== 0))
		return -EINPROGRESS;

	/*
	 * Frame receive status
	 */
	desc->ds_us.rx.rs_datalen = rx_status->rx_status_0 &
		AR5K_NEW_RX_DESC_STATUS0_DATA_LEN;
	desc->ds_us.rx.rs_rssi = AR5K_REG_MS(rx_status->rx_status_0,
		AR5K_NEW_RX_DESC_STATUS0_RECEIVE_SIGNAL);
	desc->ds_us.rx.rs_rate = AR5K_REG_MS(rx_status->rx_status_0,
		AR5K_NEW_RX_DESC_STATUS0_RECEIVE_RATE);
	desc->ds_us.rx.rs_antenna = rx_status->rx_status_0 &
		AR5K_NEW_RX_DESC_STATUS0_RECEIVE_ANTENNA;
	desc->ds_us.rx.rs_more = rx_status->rx_status_0 &
		AR5K_NEW_RX_DESC_STATUS0_MORE;
	desc->ds_us.rx.rs_tstamp = AR5K_REG_MS(rx_status->rx_status_1,
		AR5K_NEW_RX_DESC_STATUS1_RECEIVE_TIMESTAMP);
	desc->ds_us.rx.rs_status = 0;

	/*
	 * Key table status
	 */
	if (rx_status->rx_status_1 & AR5K_NEW_RX_DESC_STATUS1_KEY_INDEX_VALID)
		desc->ds_us.rx.rs_keyix = AR5K_REG_MS(rx_status->rx_status_1,
				AR5K_NEW_RX_DESC_STATUS1_KEY_INDEX);
	else
		desc->ds_us.rx.rs_keyix = AR5K_RXKEYIX_INVALID;

	/*
	 * Receive/descriptor errors
	 */
	if ((rx_status->rx_status_1 &
			AR5K_NEW_RX_DESC_STATUS1_FRAME_RECEIVE_OK) == 0) {
		if (rx_status->rx_status_1 & AR5K_NEW_RX_DESC_STATUS1_CRC_ERROR)
			desc->ds_us.rx.rs_status |= AR5K_RXERR_CRC;

		if (rx_status->rx_status_1 &
				AR5K_NEW_RX_DESC_STATUS1_PHY_ERROR) {
			desc->ds_us.rx.rs_status |= AR5K_RXERR_PHY;
			desc->ds_us.rx.rs_phyerr =
				AR5K_REG_MS(rx_err->rx_error_1,
					AR5K_RX_DESC_ERROR1_PHY_ERROR_CODE);
		}

		if (rx_status->rx_status_1 &
				AR5K_NEW_RX_DESC_STATUS1_DECRYPT_CRC_ERROR)
			desc->ds_us.rx.rs_status |= AR5K_RXERR_DECRYPT;

		if (rx_status->rx_status_1 & AR5K_NEW_RX_DESC_STATUS1_MIC_ERROR)
			desc->ds_us.rx.rs_status |= AR5K_RXERR_MIC;
	}

	return 0;
}


/****************\
  GPIO Functions
\****************/

/*
 * Set led state
 */
void ath5k_hw_set_ledstate(struct ath5k_hw *hal, unsigned int state)
{
	u32 led;
	/*5210 has different led mode handling*/
	u32 led_5210;

	AR5K_TRACE;

	/*Reset led status*/
	if (hal->ah_version != AR5K_AR5210)
		AR5K_REG_DISABLE_BITS(hal, AR5K_PCICFG,
			AR5K_PCICFG_LEDMODE |  AR5K_PCICFG_LED);
	else
		AR5K_REG_DISABLE_BITS(hal, AR5K_PCICFG, AR5K_PCICFG_LED);

	/*
	 * Some blinking values, define at your wish
	 */
	switch (state) {
	case AR5K_LED_SCAN:
	case AR5K_LED_AUTH:
		led = AR5K_PCICFG_LEDMODE_PROP | AR5K_PCICFG_LED_PEND;
		led_5210 = AR5K_PCICFG_LED_PEND | AR5K_PCICFG_LED_BCTL;
		break;

	case AR5K_LED_INIT:
		led = AR5K_PCICFG_LEDMODE_PROP | AR5K_PCICFG_LED_NONE;
		led_5210 = AR5K_PCICFG_LED_PEND;
		break;

	case AR5K_LED_ASSOC:
	case AR5K_LED_RUN:
		led = AR5K_PCICFG_LEDMODE_PROP | AR5K_PCICFG_LED_ASSOC;
		led_5210 = AR5K_PCICFG_LED_ASSOC;
		break;

	default:
		led = AR5K_PCICFG_LEDMODE_PROM | AR5K_PCICFG_LED_NONE;
		led_5210 = AR5K_PCICFG_LED_PEND;
		break;
	}

	/*Write new status to the register*/
	if (hal->ah_version != AR5K_AR5210)
		AR5K_REG_ENABLE_BITS(hal, AR5K_PCICFG, led);
	else
		AR5K_REG_ENABLE_BITS(hal, AR5K_PCICFG, led_5210);
}

/*
 * Set GPIO outputs
 */
int ath5k_hw_set_gpio_output(struct ath5k_hw *hal, u32 gpio)
{
	AR5K_TRACE;
	if (gpio > AR5K_NUM_GPIO)
		return -EINVAL;

	ath5k_hw_reg_write(hal, (ath5k_hw_reg_read(hal, AR5K_GPIOCR) & 
				~AR5K_GPIOCR_OUT(gpio)) | 
			AR5K_GPIOCR_OUT(gpio), AR5K_GPIOCR);

	return 0;
}

/*
 * Set GPIO inputs
 */
int ath5k_hw_set_gpio_input(struct ath5k_hw *hal, u32 gpio)
{
	AR5K_TRACE;
	if (gpio > AR5K_NUM_GPIO)
		return -EINVAL;

	ath5k_hw_reg_write(hal, (ath5k_hw_reg_read(hal, AR5K_GPIOCR) &
				~AR5K_GPIOCR_OUT(gpio)) | 
			AR5K_GPIOCR_IN(gpio), AR5K_GPIOCR);

	return 0;
}

/*
 * Get GPIO state
 */
u32 ath5k_hw_get_gpio(struct ath5k_hw *hal, u32 gpio)
{
	AR5K_TRACE;
	if (gpio > AR5K_NUM_GPIO)
		return 0xffffffff;

	/* GPIO input magic */
	return ((ath5k_hw_reg_read(hal, AR5K_GPIODI) & AR5K_GPIODI_M) >> gpio) &
		0x1;
}

/*
 * Set GPIO state
 */
int ath5k_hw_set_gpio(struct ath5k_hw *hal, u32 gpio, u32 val)
{
	u32 data;
	AR5K_TRACE;

	if (gpio > AR5K_NUM_GPIO)
		return -EINVAL;

	/* GPIO output magic */
	data = ath5k_hw_reg_read(hal, AR5K_GPIODO);

	data &= ~(1 << gpio);
	data |= (val & 1) << gpio;

	ath5k_hw_reg_write(hal, data, AR5K_GPIODO);

	return 0;
}

/*
 * Initialize the GPIO interrupt (RFKill switch)
 */
void ath5k_hw_set_gpio_intr(struct ath5k_hw *hal, unsigned int gpio,
		u32 interrupt_level)
{
	u32 data;

	AR5K_TRACE;
	if (gpio > AR5K_NUM_GPIO)
		return;

	/*
	 * Set the GPIO interrupt
	 */
	data = (ath5k_hw_reg_read(hal, AR5K_GPIOCR) &
			~(AR5K_GPIOCR_INT_SEL(gpio) | AR5K_GPIOCR_INT_SELH |
				AR5K_GPIOCR_INT_ENA | AR5K_GPIOCR_OUT(gpio))) |
		(AR5K_GPIOCR_INT_SEL(gpio) | AR5K_GPIOCR_INT_ENA);

	ath5k_hw_reg_write(hal, interrupt_level ? data :
			(data | AR5K_GPIOCR_INT_SELH), AR5K_GPIOCR);

	hal->ah_imr |= AR5K_IMR_GPIO;

	/* Enable GPIO interrupts */
	AR5K_REG_ENABLE_BITS(hal, AR5K_PIMR, AR5K_IMR_GPIO);
}


/*********************************\
 Regulatory Domain/Channels Setup
\*********************************/

u16 ath5k_get_regdomain(struct ath5k_hw *hal)
{
	u16 regdomain;
	enum ath5k_regdom ieee_regdomain;
#ifdef COUNTRYCODE
	u16 code;
#endif

	ath5k_eeprom_regulation_domain(hal, false, &ieee_regdomain);
	hal->ah_capabilities.cap_regdomain.reg_hw = ieee_regdomain;

#ifdef COUNTRYCODE
	/*
	 * Get the regulation domain by country code. This will ignore
	 * the settings found in the EEPROM.
	 */
	code = ieee80211_name2countrycode(COUNTRYCODE);
	ieee_regdomain = ieee80211_countrycode2regdomain(code);
#endif

	regdomain = ath5k_regdom_from_ieee(ieee_regdomain);
	hal->ah_capabilities.cap_regdomain.reg_current = regdomain;

	return regdomain;
}



/****************\
  Misc functions
\****************/

void /*O.K.*/
ath5k_hw_dump_state(struct ath5k_hw *hal)
{
#ifdef AR5K_DEBUG
#define AR5K_PRINT_REGISTER(_x)						\
	AR5K_PRINTF("(%s: %08x) ", #_x, ath5k_hw_reg_read(hal, AR5K_##_x));

	AR5K_PRINT("MAC registers:\n");
	AR5K_PRINT_REGISTER(CR);
	AR5K_PRINT_REGISTER(CFG);
	AR5K_PRINT_REGISTER(IER);
	AR5K_PRINT_REGISTER(TXCFG);
	AR5K_PRINT_REGISTER(RXCFG);
	AR5K_PRINT_REGISTER(MIBC);
	AR5K_PRINT_REGISTER(TOPS);
	AR5K_PRINT_REGISTER(RXNOFRM);
	AR5K_PRINT_REGISTER(RPGTO);
	AR5K_PRINT_REGISTER(RFCNT);
	AR5K_PRINT_REGISTER(MISC);
	AR5K_PRINT_REGISTER(PISR);
	AR5K_PRINT_REGISTER(SISR0);
	AR5K_PRINT_REGISTER(SISR1);
	AR5K_PRINT_REGISTER(SISR3);
	AR5K_PRINT_REGISTER(SISR4);
	AR5K_PRINT_REGISTER(DCM_ADDR);
	AR5K_PRINT_REGISTER(DCM_DATA);
	AR5K_PRINT_REGISTER(DCCFG);
	AR5K_PRINT_REGISTER(CCFG);
	AR5K_PRINT_REGISTER(CCFG_CUP);
	AR5K_PRINT_REGISTER(CPC0);
	AR5K_PRINT_REGISTER(CPC1);
	AR5K_PRINT_REGISTER(CPC2);
	AR5K_PRINT_REGISTER(CPCORN);
	AR5K_PRINT_REGISTER(QCU_TXE);
	AR5K_PRINT_REGISTER(QCU_TXD);
	AR5K_PRINT_REGISTER(DCU_GBL_IFS_SIFS);
	AR5K_PRINT_REGISTER(DCU_GBL_IFS_SLOT);
	AR5K_PRINT_REGISTER(DCU_FP);
	AR5K_PRINT_REGISTER(DCU_TXP);
	AR5K_PRINT_REGISTER(DCU_TX_FILTER);
	AR5K_PRINT_REGISTER(INTPEND);
	AR5K_PRINT_REGISTER(PCICFG);
	AR5K_PRINT_REGISTER(GPIOCR);
	AR5K_PRINT_REGISTER(GPIODO);
	AR5K_PRINT_REGISTER(SREV);
	AR5K_PRINT_REGISTER(EEPROM_BASE);
	AR5K_PRINT_REGISTER(EEPROM_DATA);
	AR5K_PRINT_REGISTER(EEPROM_CMD);
	AR5K_PRINT_REGISTER(EEPROM_CFG);
	AR5K_PRINT_REGISTER(PCU_MIN);
	AR5K_PRINT_REGISTER(STA_ID0);
	AR5K_PRINT_REGISTER(STA_ID1);
	AR5K_PRINT_REGISTER(BSS_ID0);
	AR5K_PRINT_REGISTER(SLOT_TIME);
	AR5K_PRINT_REGISTER(TIME_OUT);
	AR5K_PRINT_REGISTER(RSSI_THR);
	AR5K_PRINT_REGISTER(BEACON);
	AR5K_PRINT_REGISTER(CFP_PERIOD);
	AR5K_PRINT_REGISTER(TIMER0);
	AR5K_PRINT_REGISTER(TIMER2);
	AR5K_PRINT_REGISTER(TIMER3);
	AR5K_PRINT_REGISTER(CFP_DUR);
	AR5K_PRINT_REGISTER(MCAST_FILTER0);
	AR5K_PRINT_REGISTER(MCAST_FILTER1);
	AR5K_PRINT_REGISTER(DIAG_SW);
	AR5K_PRINT_REGISTER(TSF_U32);
	AR5K_PRINT_REGISTER(ADDAC_TEST);
	AR5K_PRINT_REGISTER(DEFAULT_ANTENNA);
	AR5K_PRINT_REGISTER(LAST_TSTP);
	AR5K_PRINT_REGISTER(NAV);
	AR5K_PRINT_REGISTER(RTS_OK);
	AR5K_PRINT_REGISTER(ACK_FAIL);
	AR5K_PRINT_REGISTER(FCS_FAIL);
	AR5K_PRINT_REGISTER(BEACON_CNT);
	AR5K_PRINT_REGISTER(TSF_PARM);
	AR5K_PRINT("\n");

	AR5K_PRINT("PHY registers:\n");
	AR5K_PRINT_REGISTER(PHY_TURBO);
	AR5K_PRINT_REGISTER(PHY_AGC);
	AR5K_PRINT_REGISTER(PHY_TIMING_3);
	AR5K_PRINT_REGISTER(PHY_CHIP_ID);
	AR5K_PRINT_REGISTER(PHY_AGCCTL);
	AR5K_PRINT_REGISTER(PHY_NF);
	AR5K_PRINT_REGISTER(PHY_SCR);
	AR5K_PRINT_REGISTER(PHY_SLMT);
	AR5K_PRINT_REGISTER(PHY_SCAL);
	AR5K_PRINT_REGISTER(PHY_RX_DELAY);
	AR5K_PRINT_REGISTER(PHY_IQ);
	AR5K_PRINT_REGISTER(PHY_PAPD_PROBE);
	AR5K_PRINT_REGISTER(PHY_TXPOWER_RATE1);
	AR5K_PRINT_REGISTER(PHY_TXPOWER_RATE2);
	AR5K_PRINT_REGISTER(PHY_RADAR);
	AR5K_PRINT_REGISTER(PHY_ANT_SWITCH_TABLE_0);
	AR5K_PRINT_REGISTER(PHY_ANT_SWITCH_TABLE_1);
	AR5K_PRINT("\n");
#endif
}

int ath5k_hw_get_capability(struct ath5k_hw *hal,
		enum ath5k_capability_type cap_type,
		u32 capability, u32 *result)
{
	AR5K_TRACE;

	switch (cap_type) {
	case AR5K_CAP_NUM_TXQUEUES:
		if (result) {
			if (hal->ah_version == AR5K_AR5210)
				*result = AR5K_NUM_TX_QUEUES_NOQCU;
			else
				*result = AR5K_NUM_TX_QUEUES;
			goto yes;
		}
	case AR5K_CAP_VEOL:
		goto yes;
	case AR5K_CAP_COMPRESSION:
		if (hal->ah_version == AR5K_AR5212)
			goto yes;
		else
			goto no;
	case AR5K_CAP_BURST:
		goto yes;
	case AR5K_CAP_TPC:
		goto yes;
	case AR5K_CAP_BSSIDMASK:
		if (hal->ah_version == AR5K_AR5212)
			goto yes;
		else
			goto no;
	case AR5K_CAP_XR:
		if (hal->ah_version == AR5K_AR5212)
			goto yes;
		else
			goto no;
	default:
		goto no;
	}

no:
	return -EINVAL;
yes:
	return 0;
}

#if 0
static bool ath5k_hw_query_pspoll_support(struct ath5k_hw *hal)
{
	AR5K_TRACE;

	if (hal->ah_version == AR5K_AR5210)
		return true;

	return false;
}
#endif

static int ath5k_hw_enable_pspoll(struct ath5k_hw *hal, u8 *bssid,
		u16 assoc_id)
{
	AR5K_TRACE;

	if (hal->ah_version == AR5K_AR5210) {
		AR5K_REG_DISABLE_BITS(hal, AR5K_STA_ID1,
			AR5K_STA_ID1_NO_PSPOLL | AR5K_STA_ID1_DEFAULT_ANTENNA);
		return 0;
	}

	return -EIO;
}

static int ath5k_hw_disable_pspoll(struct ath5k_hw *hal)
{
	AR5K_TRACE;

	if (hal->ah_version == AR5K_AR5210) {
		AR5K_REG_ENABLE_BITS(hal, AR5K_STA_ID1,
			AR5K_STA_ID1_NO_PSPOLL | AR5K_STA_ID1_DEFAULT_ANTENNA);
		return 0;
	}

	return -EIO;
}
