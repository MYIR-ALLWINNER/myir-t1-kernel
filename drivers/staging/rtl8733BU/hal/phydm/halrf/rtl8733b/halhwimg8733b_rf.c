/******************************************************************************
 *
 * Copyright(c) 2007 - 2017 Realtek Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * The full GNU General Public License is included in this distribution in the
 * file called LICENSE.
 *
 * Contact Information:
 * wlanfae <wlanfae@realtek.com>
 * Realtek Corporation, No. 2, Innovation Road II, Hsinchu Science Park,
 * Hsinchu 300, Taiwan.
 *
 * Larry Finger <Larry.Finger@lwfinger.net>
 *
 *****************************************************************************/

/*Image2HeaderVersion: R3 1.5.11*/
#include "mp_precomp.h"

#define ODM_WIN 0x08

#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
#if RT_PLATFORM == PLATFORM_MACOSX
#include "phydm_precomp.h"
#else
#include "../phydm_precomp.h"
#endif
#else
#include "../../phydm_precomp.h"
#endif

#define D_S_SIZE DELTA_SWINGIDX_SIZE
#define D_ST_SIZE DELTA_SWINTSSI_SIZE

#if (RTL8733B_SUPPORT == 1)
static boolean
check_positive(struct dm_struct *dm,
	       const u32	condition1,
	       const u32	condition2,
	       const u32	condition3,
	       const u32	condition4
)
{
	u32	cond1 = condition1, cond2 = condition2,
		cond3 = condition3, cond4 = condition4;

	u8	cut_version_for_para =
		(dm->cut_version ==  ODM_CUT_A) ? 15 : dm->cut_version;

	u8	pkg_type_for_para =
		(dm->package_type == 0) ? 15 : dm->package_type;

	u32	driver1 = cut_version_for_para << 24 |
			(dm->support_interface & 0xF0) << 16 |
			dm->support_platform << 16 |
			pkg_type_for_para << 12 |
			(dm->support_interface & 0x0F) << 8  |
			dm->rfe_type;

	u32	driver2 = (dm->type_glna & 0xFF) <<  0 |
			(dm->type_gpa & 0xFF)  <<  8 |
			(dm->type_alna & 0xFF) << 16 |
			(dm->type_apa & 0xFF)  << 24;

	u32	driver3 = 0;

	u32	driver4 = (dm->type_glna & 0xFF00) >>  8 |
			(dm->type_gpa & 0xFF00) |
			(dm->type_alna & 0xFF00) << 8 |
			(dm->type_apa & 0xFF00)  << 16;

	PHYDM_DBG(dm, ODM_COMP_INIT,
		  "===> %s (cond1, cond2, cond3, cond4) = (0x%X 0x%X 0x%X 0x%X)\n",
		  __func__, cond1, cond2, cond3, cond4);
	PHYDM_DBG(dm, ODM_COMP_INIT,
		  "===> %s (driver1, driver2, driver3, driver4) = (0x%X 0x%X 0x%X 0x%X)\n",
		  __func__, driver1, driver2, driver3, driver4);

	PHYDM_DBG(dm, ODM_COMP_INIT,
		  "	(Platform, Interface) = (0x%X, 0x%X)\n",
		  dm->support_platform, dm->support_interface);
	PHYDM_DBG(dm, ODM_COMP_INIT, "	(RFE, Package) = (0x%X, 0x%X)\n",
		  dm->rfe_type, dm->package_type);

	/*============== value Defined Check ===============*/
	/*cut version [27:24] need to do value check*/
	if (((cond1 & 0x0F000000) != 0) &&
	    ((cond1 & 0x0F000000) != (driver1 & 0x0F000000)))
		return false;

	/*pkg type [15:12] need to do value check*/
	if (((cond1 & 0x0000F000) != 0) &&
	    ((cond1 & 0x0000F000) != (driver1 & 0x0000F000)))
		return false;

	/*interface [11:8] need to do value check*/
	if (((cond1 & 0x00000F00) != 0) &&
	    ((cond1 & 0x00000F00) != (driver1 & 0x00000F00)))
		return false;
	/*=============== Bit Defined Check ================*/
	/* We don't care [31:28] */

	cond1 &= 0x000000FF;
	driver1 &= 0x000000FF;

	if (cond1 == driver1)
		return true;
	else
		return false;
}


/******************************************************************************
 *                           radioa.TXT
 ******************************************************************************/

const u32 array_mp_8733b_radioa[] = {
		0x000, 0x00030000,
		0x018, 0x00001C01,
		0x059, 0x000A0000,
		0x0FE, 0x00000000,
		0x01B, 0x00003A40,
		0x0EF, 0x00080000,
		0x033, 0x00000007,
		0x03E, 0x00000019,
		0x03F, 0x00007BD3,
		0x033, 0x0000000C,
		0x03E, 0x0000001F,
		0x03F, 0x000D2BFA,
		0x033, 0x0000000D,
		0x03E, 0x00000019,
		0x03F, 0x00006B50,
		0x0EF, 0x00000000,
		0x087, 0x00002007,
		0x0CC, 0x00080000,
		0x0A5, 0x000B0000,
		0x0B7, 0x00000208,
		0x0D5, 0x0004027A,
		0x0B1, 0x00069F7F,
		0x0ED, 0x00002000,
		0x033, 0x00000002,
		0x03D, 0x0004A883,
		0x03E, 0x00000000,
		0x03F, 0x00000001,
		0x033, 0x00000006,
		0x03D, 0x0004A883,
		0x03E, 0x00000000,
		0x03F, 0x00000001,
		0x0ED, 0x00000000,
		0x0ED, 0x00000800,
		0x033, 0x00000000,
		0x03F, 0x000001CE,
		0x033, 0x00000001,
		0x03F, 0x000001EF,
		0x033, 0x00000002,
		0x03F, 0x000001AD,
		0x033, 0x00000003,
		0x03F, 0x000001CE,
		0x033, 0x00000004,
		0x03F, 0x00000210,
		0x033, 0x00000005,
		0x03F, 0x00000631,
		0x033, 0x00000006,
		0x03F, 0x00000652,
		0x033, 0x00000007,
		0x03F, 0x00000673,
		0x033, 0x00000008,
		0x03F, 0x00000A94,
		0x033, 0x00000009,
		0x03F, 0x00000EB5,
		0x033, 0x0000000A,
		0x03F, 0x00000ED6,
		0x033, 0x0000000B,
		0x03F, 0x000012F7,
		0x033, 0x0000000C,
		0x03F, 0x00001318,
		0x033, 0x0000000D,
		0x03F, 0x00001739,
		0x033, 0x0000000E,
		0x03F, 0x00001B5A,
		0x033, 0x0000000F,
		0x03F, 0x00001B5A,
		0x033, 0x00000010,
		0x03F, 0x00001F7B,
		0x033, 0x00000011,
		0x03F, 0x00001F9C,
		0x033, 0x00000012,
		0x03F, 0x00000210,
		0x033, 0x00000013,
		0x03F, 0x00000231,
		0x0ED, 0x00000000,

};

void
odm_read_and_config_mp_8733b_radioa(struct dm_struct *dm)
{
	u32	i = 0;
	u8	c_cond;
	boolean	is_matched = true, is_skipped = false;
	u32	array_len =
			sizeof(array_mp_8733b_radioa) / sizeof(u32);
	u32	*array = (u32 *)array_mp_8733b_radioa;

	u32	v1 = 0, v2 = 0, pre_v1 = 0, pre_v2 = 0;
	u32	a1 = 0, a2 = 0, a3 = 0, a4 = 0;

	PHYDM_DBG(dm, ODM_COMP_INIT, "===> %s\n", __func__);

	while ((i + 1) < array_len) {
		v1 = array[i];
		v2 = array[i + 1];

		if (v1 & (BIT(31) | BIT(30))) {/*positive & negative condition*/
			if (v1 & BIT(31)) {/* positive condition*/
				c_cond  =
					(u8)((v1 & (BIT(29) | BIT(28))) >> 28);
				if (c_cond == COND_ENDIF) {/*end*/
					is_matched = true;
					is_skipped = false;
					PHYDM_DBG(dm, ODM_COMP_INIT, "ENDIF\n");
				} else if (c_cond == COND_ELSE) { /*else*/
					is_matched = is_skipped ? false : true;
					PHYDM_DBG(dm, ODM_COMP_INIT, "ELSE\n");
				} else {/*if , else if*/
					pre_v1 = v1;
					pre_v2 = v2;
					PHYDM_DBG(dm, ODM_COMP_INIT,
						  "IF or ELSE IF\n");
				}
			} else if (v1 & BIT(30)) { /*negative condition*/
				if (!is_skipped) {
					a1 = pre_v1; a2 = pre_v2;
					a3 = v1; a4 = v2;
					if (check_positive(dm,
							   a1, a2, a3, a4)) {
						is_matched = true;
						is_skipped = true;
					} else {
						is_matched = false;
						is_skipped = false;
					}
				} else {
					is_matched = false;
				}
			}
		} else {
			if (is_matched)
				odm_config_rf_radio_a_8733b(dm, v1, v2);
		}
		i = i + 2;
	}
}

u32
odm_get_version_mp_8733b_radioa(void)
{
		return 23;
}

/******************************************************************************
 *                           radiob.TXT
 ******************************************************************************/

const u32 array_mp_8733b_radiob[] = {
		0x0EF, 0x00004000,
		0x033, 0x00000002,
		0x03F, 0x00000B87,
		0x033, 0x00000001,
		0x03F, 0x00001A87,
		0x033, 0x00000000,
		0x03F, 0x00002987,
		0x033, 0x0000000F,
		0x03F, 0x00000705,
		0x033, 0x0000000E,
		0x03F, 0x00000785,
		0x033, 0x0000000D,
		0x03F, 0x00000585,
		0x033, 0x0000000C,
		0x03F, 0x00000485,
		0x033, 0x0000000B,
		0x03F, 0x00008B85,
		0x033, 0x0000000A,
		0x03F, 0x00000B85,
		0x033, 0x00000001,
		0x03F, 0x00001A85,
		0x033, 0x00000000,
		0x03F, 0x00002985,
		0x0EF, 0x00080000,
		0x033, 0x00000007,
		0x03F, 0x000341E3,
		0x033, 0x0000000C,
		0x03F, 0x0003F4AB,
		0x0EF, 0x00000000,
		0x087, 0x00002003,

};

void
odm_read_and_config_mp_8733b_radiob(struct dm_struct *dm)
{
	u32	i = 0;
	u8	c_cond;
	boolean	is_matched = true, is_skipped = false;
	u32	array_len =
			sizeof(array_mp_8733b_radiob) / sizeof(u32);
	u32	*array = (u32 *)array_mp_8733b_radiob;

	u32	v1 = 0, v2 = 0, pre_v1 = 0, pre_v2 = 0;
	u32	a1 = 0, a2 = 0, a3 = 0, a4 = 0;

	PHYDM_DBG(dm, ODM_COMP_INIT, "===> %s\n", __func__);

	while ((i + 1) < array_len) {
		v1 = array[i];
		v2 = array[i + 1];

		if (v1 & (BIT(31) | BIT(30))) {/*positive & negative condition*/
			if (v1 & BIT(31)) {/* positive condition*/
				c_cond  =
					(u8)((v1 & (BIT(29) | BIT(28))) >> 28);
				if (c_cond == COND_ENDIF) {/*end*/
					is_matched = true;
					is_skipped = false;
					PHYDM_DBG(dm, ODM_COMP_INIT, "ENDIF\n");
				} else if (c_cond == COND_ELSE) { /*else*/
					is_matched = is_skipped ? false : true;
					PHYDM_DBG(dm, ODM_COMP_INIT, "ELSE\n");
				} else {/*if , else if*/
					pre_v1 = v1;
					pre_v2 = v2;
					PHYDM_DBG(dm, ODM_COMP_INIT,
						  "IF or ELSE IF\n");
				}
			} else if (v1 & BIT(30)) { /*negative condition*/
				if (!is_skipped) {
					a1 = pre_v1; a2 = pre_v2;
					a3 = v1; a4 = v2;
					if (check_positive(dm,
							   a1, a2, a3, a4)) {
						is_matched = true;
						is_skipped = true;
					} else {
						is_matched = false;
						is_skipped = false;
					}
				} else {
					is_matched = false;
				}
			}
		} else {
			if (is_matched)
				odm_config_rf_radio_b_8733b(dm, v1, v2);
		}
		i = i + 2;
	}
}

u32
odm_get_version_mp_8733b_radiob(void)
{
		return 23;
}

/******************************************************************************
 *                           txpowertrack.TXT
 ******************************************************************************/

#ifdef CONFIG_8733B
const u8 delta_swingidx_mp_5gb_n_txpwrtrk_8733b[][D_S_SIZE] = {
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
};

const u8 delta_swingidx_mp_5gb_p_txpwrtrk_8733b[][D_S_SIZE] = {
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
};

const u8 delta_swingidx_mp_5ga_n_txpwrtrk_8733b[][D_S_SIZE] = {
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
};

const u8 delta_swingidx_mp_5ga_p_txpwrtrk_8733b[][D_S_SIZE] = {
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
};

const u8 delta_swingidx_mp_2gb_n_txpwrtrk_8733b[]    = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const u8 delta_swingidx_mp_2gb_p_txpwrtrk_8733b[]    = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const u8 delta_swingidx_mp_2ga_n_txpwrtrk_8733b[]    = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const u8 delta_swingidx_mp_2ga_p_txpwrtrk_8733b[]    = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const u8 delta_swingidx_mp_2g_cck_b_n_txpwrtrk_8733b[] = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const u8 delta_swingidx_mp_2g_cck_b_p_txpwrtrk_8733b[] = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const u8 delta_swingidx_mp_2g_cck_a_n_txpwrtrk_8733b[] = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const u8 delta_swingidx_mp_2g_cck_a_p_txpwrtrk_8733b[] = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
#endif

void
odm_read_and_config_mp_8733b_txpowertrack(struct dm_struct *dm)
{
#ifdef CONFIG_8733B

struct dm_rf_calibration_struct  *cali_info = &dm->rf_calibrate_info;

PHYDM_DBG(dm, ODM_COMP_INIT, "===> ODM_ReadAndConfig_MP_mp_8733b\n");

odm_move_memory(dm, cali_info->delta_swing_table_idx_2ga_p,
		(void *)delta_swingidx_mp_2ga_p_txpwrtrk_8733b,
		DELTA_SWINGIDX_SIZE);
odm_move_memory(dm, cali_info->delta_swing_table_idx_2ga_n,
		(void *)delta_swingidx_mp_2ga_n_txpwrtrk_8733b,
		DELTA_SWINGIDX_SIZE);
odm_move_memory(dm, cali_info->delta_swing_table_idx_2gb_p,
		(void *)delta_swingidx_mp_2gb_p_txpwrtrk_8733b,
		DELTA_SWINGIDX_SIZE);
odm_move_memory(dm, cali_info->delta_swing_table_idx_2gb_n,
		(void *)delta_swingidx_mp_2gb_n_txpwrtrk_8733b,
		DELTA_SWINGIDX_SIZE);

odm_move_memory(dm, cali_info->delta_swing_table_idx_2g_cck_a_p,
		(void *)delta_swingidx_mp_2g_cck_a_p_txpwrtrk_8733b,
		DELTA_SWINGIDX_SIZE);
odm_move_memory(dm, cali_info->delta_swing_table_idx_2g_cck_a_n,
		(void *)delta_swingidx_mp_2g_cck_a_n_txpwrtrk_8733b,
		DELTA_SWINGIDX_SIZE);
odm_move_memory(dm, cali_info->delta_swing_table_idx_2g_cck_b_p,
		(void *)delta_swingidx_mp_2g_cck_b_p_txpwrtrk_8733b,
		DELTA_SWINGIDX_SIZE);
odm_move_memory(dm, cali_info->delta_swing_table_idx_2g_cck_b_n,
		(void *)delta_swingidx_mp_2g_cck_b_n_txpwrtrk_8733b,
		DELTA_SWINGIDX_SIZE);

odm_move_memory(dm, cali_info->delta_swing_table_idx_5ga_p,
		(void *)delta_swingidx_mp_5ga_p_txpwrtrk_8733b,
		DELTA_SWINGIDX_SIZE * 3);
odm_move_memory(dm, cali_info->delta_swing_table_idx_5ga_n,
		(void *)delta_swingidx_mp_5ga_n_txpwrtrk_8733b,
		DELTA_SWINGIDX_SIZE * 3);
odm_move_memory(dm, cali_info->delta_swing_table_idx_5gb_p,
		(void *)delta_swingidx_mp_5gb_p_txpwrtrk_8733b,
		DELTA_SWINGIDX_SIZE * 3);
odm_move_memory(dm, cali_info->delta_swing_table_idx_5gb_n,
		(void *)delta_swingidx_mp_5gb_n_txpwrtrk_8733b,
		DELTA_SWINGIDX_SIZE * 3);
#endif
}

/******************************************************************************
 *                           txpowertracktssi.TXT
 ******************************************************************************/

#ifdef CONFIG_8733BTSSI
const u8 delta_swingidx_mp_5gb_n_txpwrtrktssi_8733b[][D_S_SIZE] = {
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
};

const u8 delta_swingidx_mp_5gb_p_txpwrtrktssi_8733b[][D_S_SIZE] = {
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
};

const u8 delta_swingidx_mp_5ga_n_txpwrtrktssi_8733b[][D_S_SIZE] = {
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
};

const u8 delta_swingidx_mp_5ga_p_txpwrtrktssi_8733b[][D_S_SIZE] = {
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
};

const u8 delta_swingidx_mp_2gb_n_txpwrtrktssi_8733b[]    = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const u8 delta_swingidx_mp_2gb_p_txpwrtrktssi_8733b[]    = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const u8 delta_swingidx_mp_2ga_n_txpwrtrktssi_8733b[]    = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const u8 delta_swingidx_mp_2ga_p_txpwrtrktssi_8733b[]    = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const u8 delta_swingidx_mp_2g_cck_b_n_txpwrtrktssi_8733b[] = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 3, 3, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5};
const u8 delta_swingidx_mp_2g_cck_b_p_txpwrtrktssi_8733b[] = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 3, 3, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5};
const u8 delta_swingidx_mp_2g_cck_a_n_txpwrtrktssi_8733b[] = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 3, 3, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5};
const u8 delta_swingidx_mp_2g_cck_a_p_txpwrtrktssi_8733b[] = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 3, 3, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5};
#endif

void
odm_read_and_config_mp_8733b_txpowertracktssi(struct dm_struct *dm)
{
#ifdef CONFIG_8733BTSSI

struct dm_rf_calibration_struct  *cali_info = &dm->rf_calibrate_info;

PHYDM_DBG(dm, ODM_COMP_INIT, "===> ODM_ReadAndConfig_MP_mp_8733b\n");

odm_move_memory(dm, cali_info->delta_swing_table_idx_2ga_p,
		(void *)delta_swingidx_mp_2ga_p_txpwrtrktssi_8733b,
		DELTA_SWINGIDX_SIZE);
odm_move_memory(dm, cali_info->delta_swing_table_idx_2ga_n,
		(void *)delta_swingidx_mp_2ga_n_txpwrtrktssi_8733b,
		DELTA_SWINGIDX_SIZE);
odm_move_memory(dm, cali_info->delta_swing_table_idx_2gb_p,
		(void *)delta_swingidx_mp_2gb_p_txpwrtrktssi_8733b,
		DELTA_SWINGIDX_SIZE);
odm_move_memory(dm, cali_info->delta_swing_table_idx_2gb_n,
		(void *)delta_swingidx_mp_2gb_n_txpwrtrktssi_8733b,
		DELTA_SWINGIDX_SIZE);

odm_move_memory(dm, cali_info->delta_swing_table_idx_2g_cck_a_p,
		(void *)delta_swingidx_mp_2g_cck_a_p_txpwrtrktssi_8733b,
		DELTA_SWINGIDX_SIZE);
odm_move_memory(dm, cali_info->delta_swing_table_idx_2g_cck_a_n,
		(void *)delta_swingidx_mp_2g_cck_a_n_txpwrtrktssi_8733b,
		DELTA_SWINGIDX_SIZE);
odm_move_memory(dm, cali_info->delta_swing_table_idx_2g_cck_b_p,
		(void *)delta_swingidx_mp_2g_cck_b_p_txpwrtrktssi_8733b,
		DELTA_SWINGIDX_SIZE);
odm_move_memory(dm, cali_info->delta_swing_table_idx_2g_cck_b_n,
		(void *)delta_swingidx_mp_2g_cck_b_n_txpwrtrktssi_8733b,
		DELTA_SWINGIDX_SIZE);

odm_move_memory(dm, cali_info->delta_swing_table_idx_5ga_p,
		(void *)delta_swingidx_mp_5ga_p_txpwrtrktssi_8733b,
		DELTA_SWINGIDX_SIZE * 3);
odm_move_memory(dm, cali_info->delta_swing_table_idx_5ga_n,
		(void *)delta_swingidx_mp_5ga_n_txpwrtrktssi_8733b,
		DELTA_SWINGIDX_SIZE * 3);
odm_move_memory(dm, cali_info->delta_swing_table_idx_5gb_p,
		(void *)delta_swingidx_mp_5gb_p_txpwrtrktssi_8733b,
		DELTA_SWINGIDX_SIZE * 3);
odm_move_memory(dm, cali_info->delta_swing_table_idx_5gb_n,
		(void *)delta_swingidx_mp_5gb_n_txpwrtrktssi_8733b,
		DELTA_SWINGIDX_SIZE * 3);
#endif
}

/******************************************************************************
 *                           txpwr_lmt.TXT
 ******************************************************************************/

#ifdef CONFIG_8733B
const char *array_mp_8733b_txpwr_lmt[] = {
	"FCC", "2.4G", "20M", "CCK", "1T", "01", "72",
	"ETSI", "2.4G", "20M", "CCK", "1T", "01", "56",
	"MKK", "2.4G", "20M", "CCK", "1T", "01", "68",
	"IC", "2.4G", "20M", "CCK", "1T", "01", "56",
	"KCC", "2.4G", "20M", "CCK", "1T", "01", "64",
	"ACMA", "2.4G", "20M", "CCK", "1T", "01", "56",
	"CHILE", "2.4G", "20M", "CCK", "1T", "01", "56",
	"UKRAINE", "2.4G", "20M", "CCK", "1T", "01", "56",
	"MEXICO", "2.4G", "20M", "CCK", "1T", "01", "56",
	"CN", "2.4G", "20M", "CCK", "1T", "01", "56",
	"FCC", "2.4G", "20M", "CCK", "1T", "02", "72",
	"ETSI", "2.4G", "20M", "CCK", "1T", "02", "56",
	"MKK", "2.4G", "20M", "CCK", "1T", "02", "68",
	"IC", "2.4G", "20M", "CCK", "1T", "02", "60",
	"KCC", "2.4G", "20M", "CCK", "1T", "02", "64",
	"ACMA", "2.4G", "20M", "CCK", "1T", "02", "56",
	"CHILE", "2.4G", "20M", "CCK", "1T", "02", "60",
	"UKRAINE", "2.4G", "20M", "CCK", "1T", "02", "56",
	"MEXICO", "2.4G", "20M", "CCK", "1T", "02", "60",
	"CN", "2.4G", "20M", "CCK", "1T", "02", "56",
	"FCC", "2.4G", "20M", "CCK", "1T", "03", "72",
	"ETSI", "2.4G", "20M", "CCK", "1T", "03", "56",
	"MKK", "2.4G", "20M", "CCK", "1T", "03", "68",
	"IC", "2.4G", "20M", "CCK", "1T", "03", "60",
	"KCC", "2.4G", "20M", "CCK", "1T", "03", "64",
	"ACMA", "2.4G", "20M", "CCK", "1T", "03", "56",
	"CHILE", "2.4G", "20M", "CCK", "1T", "03", "60",
	"UKRAINE", "2.4G", "20M", "CCK", "1T", "03", "56",
	"MEXICO", "2.4G", "20M", "CCK", "1T", "03", "60",
	"CN", "2.4G", "20M", "CCK", "1T", "03", "56",
	"FCC", "2.4G", "20M", "CCK", "1T", "04", "72",
	"ETSI", "2.4G", "20M", "CCK", "1T", "04", "56",
	"MKK", "2.4G", "20M", "CCK", "1T", "04", "68",
	"IC", "2.4G", "20M", "CCK", "1T", "04", "60",
	"KCC", "2.4G", "20M", "CCK", "1T", "04", "64",
	"ACMA", "2.4G", "20M", "CCK", "1T", "04", "56",
	"CHILE", "2.4G", "20M", "CCK", "1T", "04", "60",
	"UKRAINE", "2.4G", "20M", "CCK", "1T", "04", "56",
	"MEXICO", "2.4G", "20M", "CCK", "1T", "04", "60",
	"CN", "2.4G", "20M", "CCK", "1T", "04", "56",
	"FCC", "2.4G", "20M", "CCK", "1T", "05", "72",
	"ETSI", "2.4G", "20M", "CCK", "1T", "05", "56",
	"MKK", "2.4G", "20M", "CCK", "1T", "05", "68",
	"IC", "2.4G", "20M", "CCK", "1T", "05", "60",
	"KCC", "2.4G", "20M", "CCK", "1T", "05", "64",
	"ACMA", "2.4G", "20M", "CCK", "1T", "05", "56",
	"CHILE", "2.4G", "20M", "CCK", "1T", "05", "60",
	"UKRAINE", "2.4G", "20M", "CCK", "1T", "05", "56",
	"MEXICO", "2.4G", "20M", "CCK", "1T", "05", "60",
	"CN", "2.4G", "20M", "CCK", "1T", "05", "56",
	"FCC", "2.4G", "20M", "CCK", "1T", "06", "72",
	"ETSI", "2.4G", "20M", "CCK", "1T", "06", "56",
	"MKK", "2.4G", "20M", "CCK", "1T", "06", "68",
	"IC", "2.4G", "20M", "CCK", "1T", "06", "60",
	"KCC", "2.4G", "20M", "CCK", "1T", "06", "64",
	"ACMA", "2.4G", "20M", "CCK", "1T", "06", "56",
	"CHILE", "2.4G", "20M", "CCK", "1T", "06", "60",
	"UKRAINE", "2.4G", "20M", "CCK", "1T", "06", "56",
	"MEXICO", "2.4G", "20M", "CCK", "1T", "06", "60",
	"CN", "2.4G", "20M", "CCK", "1T", "06", "56",
	"FCC", "2.4G", "20M", "CCK", "1T", "07", "72",
	"ETSI", "2.4G", "20M", "CCK", "1T", "07", "56",
	"MKK", "2.4G", "20M", "CCK", "1T", "07", "68",
	"IC", "2.4G", "20M", "CCK", "1T", "07", "60",
	"KCC", "2.4G", "20M", "CCK", "1T", "07", "64",
	"ACMA", "2.4G", "20M", "CCK", "1T", "07", "56",
	"CHILE", "2.4G", "20M", "CCK", "1T", "07", "60",
	"UKRAINE", "2.4G", "20M", "CCK", "1T", "07", "56",
	"MEXICO", "2.4G", "20M", "CCK", "1T", "07", "60",
	"CN", "2.4G", "20M", "CCK", "1T", "07", "56",
	"FCC", "2.4G", "20M", "CCK", "1T", "08", "72",
	"ETSI", "2.4G", "20M", "CCK", "1T", "08", "56",
	"MKK", "2.4G", "20M", "CCK", "1T", "08", "68",
	"IC", "2.4G", "20M", "CCK", "1T", "08", "60",
	"KCC", "2.4G", "20M", "CCK", "1T", "08", "64",
	"ACMA", "2.4G", "20M", "CCK", "1T", "08", "56",
	"CHILE", "2.4G", "20M", "CCK", "1T", "08", "60",
	"UKRAINE", "2.4G", "20M", "CCK", "1T", "08", "56",
	"MEXICO", "2.4G", "20M", "CCK", "1T", "08", "60",
	"CN", "2.4G", "20M", "CCK", "1T", "08", "56",
	"FCC", "2.4G", "20M", "CCK", "1T", "09", "72",
	"ETSI", "2.4G", "20M", "CCK", "1T", "09", "56",
	"MKK", "2.4G", "20M", "CCK", "1T", "09", "68",
	"IC", "2.4G", "20M", "CCK", "1T", "09", "60",
	"KCC", "2.4G", "20M", "CCK", "1T", "09", "64",
	"ACMA", "2.4G", "20M", "CCK", "1T", "09", "56",
	"CHILE", "2.4G", "20M", "CCK", "1T", "09", "60",
	"UKRAINE", "2.4G", "20M", "CCK", "1T", "09", "56",
	"MEXICO", "2.4G", "20M", "CCK", "1T", "09", "60",
	"CN", "2.4G", "20M", "CCK", "1T", "09", "56",
	"FCC", "2.4G", "20M", "CCK", "1T", "10", "72",
	"ETSI", "2.4G", "20M", "CCK", "1T", "10", "56",
	"MKK", "2.4G", "20M", "CCK", "1T", "10", "68",
	"IC", "2.4G", "20M", "CCK", "1T", "10", "60",
	"KCC", "2.4G", "20M", "CCK", "1T", "10", "64",
	"ACMA", "2.4G", "20M", "CCK", "1T", "10", "56",
	"CHILE", "2.4G", "20M", "CCK", "1T", "10", "60",
	"UKRAINE", "2.4G", "20M", "CCK", "1T", "10", "56",
	"MEXICO", "2.4G", "20M", "CCK", "1T", "10", "60",
	"CN", "2.4G", "20M", "CCK", "1T", "10", "56",
	"FCC", "2.4G", "20M", "CCK", "1T", "11", "72",
	"ETSI", "2.4G", "20M", "CCK", "1T", "11", "56",
	"MKK", "2.4G", "20M", "CCK", "1T", "11", "68",
	"IC", "2.4G", "20M", "CCK", "1T", "11", "60",
	"KCC", "2.4G", "20M", "CCK", "1T", "11", "64",
	"ACMA", "2.4G", "20M", "CCK", "1T", "11", "56",
	"CHILE", "2.4G", "20M", "CCK", "1T", "11", "60",
	"UKRAINE", "2.4G", "20M", "CCK", "1T", "11", "56",
	"MEXICO", "2.4G", "20M", "CCK", "1T", "11", "60",
	"CN", "2.4G", "20M", "CCK", "1T", "11", "56",
	"FCC", "2.4G", "20M", "CCK", "1T", "12", "56",
	"ETSI", "2.4G", "20M", "CCK", "1T", "12", "56",
	"MKK", "2.4G", "20M", "CCK", "1T", "12", "68",
	"IC", "2.4G", "20M", "CCK", "1T", "12", "60",
	"KCC", "2.4G", "20M", "CCK", "1T", "12", "64",
	"ACMA", "2.4G", "20M", "CCK", "1T", "12", "56",
	"CHILE", "2.4G", "20M", "CCK", "1T", "12", "44",
	"UKRAINE", "2.4G", "20M", "CCK", "1T", "12", "56",
	"MEXICO", "2.4G", "20M", "CCK", "1T", "12", "44",
	"CN", "2.4G", "20M", "CCK", "1T", "12", "56",
	"FCC", "2.4G", "20M", "CCK", "1T", "13", "40",
	"ETSI", "2.4G", "20M", "CCK", "1T", "13", "56",
	"MKK", "2.4G", "20M", "CCK", "1T", "13", "68",
	"IC", "2.4G", "20M", "CCK", "1T", "13", "28",
	"KCC", "2.4G", "20M", "CCK", "1T", "13", "64",
	"ACMA", "2.4G", "20M", "CCK", "1T", "13", "56",
	"CHILE", "2.4G", "20M", "CCK", "1T", "13", "28",
	"UKRAINE", "2.4G", "20M", "CCK", "1T", "13", "56",
	"MEXICO", "2.4G", "20M", "CCK", "1T", "13", "28",
	"CN", "2.4G", "20M", "CCK", "1T", "13", "56",
	"FCC", "2.4G", "20M", "CCK", "1T", "14", "127",
	"ETSI", "2.4G", "20M", "CCK", "1T", "14", "127",
	"MKK", "2.4G", "20M", "CCK", "1T", "14", "68",
	"IC", "2.4G", "20M", "CCK", "1T", "14", "127",
	"KCC", "2.4G", "20M", "CCK", "1T", "14", "127",
	"ACMA", "2.4G", "20M", "CCK", "1T", "14", "127",
	"CHILE", "2.4G", "20M", "CCK", "1T", "14", "127",
	"UKRAINE", "2.4G", "20M", "CCK", "1T", "14", "127",
	"MEXICO", "2.4G", "20M", "CCK", "1T", "14", "127",
	"CN", "2.4G", "20M", "CCK", "1T", "14", "127",
	"FCC", "2.4G", "20M", "OFDM", "1T", "01", "64",
	"ETSI", "2.4G", "20M", "OFDM", "1T", "01", "56",
	"MKK", "2.4G", "20M", "OFDM", "1T", "01", "68",
	"IC", "2.4G", "20M", "OFDM", "1T", "01", "56",
	"KCC", "2.4G", "20M", "OFDM", "1T", "01", "60",
	"ACMA", "2.4G", "20M", "OFDM", "1T", "01", "56",
	"CHILE", "2.4G", "20M", "OFDM", "1T", "01", "56",
	"UKRAINE", "2.4G", "20M", "OFDM", "1T", "01", "56",
	"MEXICO", "2.4G", "20M", "OFDM", "1T", "01", "56",
	"CN", "2.4G", "20M", "OFDM", "1T", "01", "56",
	"FCC", "2.4G", "20M", "OFDM", "1T", "02", "64",
	"ETSI", "2.4G", "20M", "OFDM", "1T", "02", "56",
	"MKK", "2.4G", "20M", "OFDM", "1T", "02", "68",
	"IC", "2.4G", "20M", "OFDM", "1T", "02", "60",
	"KCC", "2.4G", "20M", "OFDM", "1T", "02", "64",
	"ACMA", "2.4G", "20M", "OFDM", "1T", "02", "56",
	"CHILE", "2.4G", "20M", "OFDM", "1T", "02", "60",
	"UKRAINE", "2.4G", "20M", "OFDM", "1T", "02", "56",
	"MEXICO", "2.4G", "20M", "OFDM", "1T", "02", "60",
	"CN", "2.4G", "20M", "OFDM", "1T", "02", "56",
	"FCC", "2.4G", "20M", "OFDM", "1T", "03", "64",
	"ETSI", "2.4G", "20M", "OFDM", "1T", "03", "56",
	"MKK", "2.4G", "20M", "OFDM", "1T", "03", "68",
	"IC", "2.4G", "20M", "OFDM", "1T", "03", "64",
	"KCC", "2.4G", "20M", "OFDM", "1T", "03", "64",
	"ACMA", "2.4G", "20M", "OFDM", "1T", "03", "56",
	"CHILE", "2.4G", "20M", "OFDM", "1T", "03", "64",
	"UKRAINE", "2.4G", "20M", "OFDM", "1T", "03", "56",
	"MEXICO", "2.4G", "20M", "OFDM", "1T", "03", "64",
	"CN", "2.4G", "20M", "OFDM", "1T", "03", "56",
	"FCC", "2.4G", "20M", "OFDM", "1T", "04", "64",
	"ETSI", "2.4G", "20M", "OFDM", "1T", "04", "56",
	"MKK", "2.4G", "20M", "OFDM", "1T", "04", "68",
	"IC", "2.4G", "20M", "OFDM", "1T", "04", "64",
	"KCC", "2.4G", "20M", "OFDM", "1T", "04", "64",
	"ACMA", "2.4G", "20M", "OFDM", "1T", "04", "56",
	"CHILE", "2.4G", "20M", "OFDM", "1T", "04", "64",
	"UKRAINE", "2.4G", "20M", "OFDM", "1T", "04", "56",
	"MEXICO", "2.4G", "20M", "OFDM", "1T", "04", "64",
	"CN", "2.4G", "20M", "OFDM", "1T", "04", "56",
	"FCC", "2.4G", "20M", "OFDM", "1T", "05", "64",
	"ETSI", "2.4G", "20M", "OFDM", "1T", "05", "56",
	"MKK", "2.4G", "20M", "OFDM", "1T", "05", "68",
	"IC", "2.4G", "20M", "OFDM", "1T", "05", "64",
	"KCC", "2.4G", "20M", "OFDM", "1T", "05", "64",
	"ACMA", "2.4G", "20M", "OFDM", "1T", "05", "56",
	"CHILE", "2.4G", "20M", "OFDM", "1T", "05", "64",
	"UKRAINE", "2.4G", "20M", "OFDM", "1T", "05", "56",
	"MEXICO", "2.4G", "20M", "OFDM", "1T", "05", "64",
	"CN", "2.4G", "20M", "OFDM", "1T", "05", "56",
	"FCC", "2.4G", "20M", "OFDM", "1T", "06", "64",
	"ETSI", "2.4G", "20M", "OFDM", "1T", "06", "56",
	"MKK", "2.4G", "20M", "OFDM", "1T", "06", "68",
	"IC", "2.4G", "20M", "OFDM", "1T", "06", "64",
	"KCC", "2.4G", "20M", "OFDM", "1T", "06", "64",
	"ACMA", "2.4G", "20M", "OFDM", "1T", "06", "56",
	"CHILE", "2.4G", "20M", "OFDM", "1T", "06", "64",
	"UKRAINE", "2.4G", "20M", "OFDM", "1T", "06", "56",
	"MEXICO", "2.4G", "20M", "OFDM", "1T", "06", "64",
	"CN", "2.4G", "20M", "OFDM", "1T", "06", "56",
	"FCC", "2.4G", "20M", "OFDM", "1T", "07", "64",
	"ETSI", "2.4G", "20M", "OFDM", "1T", "07", "56",
	"MKK", "2.4G", "20M", "OFDM", "1T", "07", "68",
	"IC", "2.4G", "20M", "OFDM", "1T", "07", "64",
	"KCC", "2.4G", "20M", "OFDM", "1T", "07", "64",
	"ACMA", "2.4G", "20M", "OFDM", "1T", "07", "56",
	"CHILE", "2.4G", "20M", "OFDM", "1T", "07", "64",
	"UKRAINE", "2.4G", "20M", "OFDM", "1T", "07", "56",
	"MEXICO", "2.4G", "20M", "OFDM", "1T", "07", "64",
	"CN", "2.4G", "20M", "OFDM", "1T", "07", "56",
	"FCC", "2.4G", "20M", "OFDM", "1T", "08", "64",
	"ETSI", "2.4G", "20M", "OFDM", "1T", "08", "56",
	"MKK", "2.4G", "20M", "OFDM", "1T", "08", "68",
	"IC", "2.4G", "20M", "OFDM", "1T", "08", "64",
	"KCC", "2.4G", "20M", "OFDM", "1T", "08", "64",
	"ACMA", "2.4G", "20M", "OFDM", "1T", "08", "56",
	"CHILE", "2.4G", "20M", "OFDM", "1T", "08", "64",
	"UKRAINE", "2.4G", "20M", "OFDM", "1T", "08", "56",
	"MEXICO", "2.4G", "20M", "OFDM", "1T", "08", "64",
	"CN", "2.4G", "20M", "OFDM", "1T", "08", "56",
	"FCC", "2.4G", "20M", "OFDM", "1T", "09", "64",
	"ETSI", "2.4G", "20M", "OFDM", "1T", "09", "56",
	"MKK", "2.4G", "20M", "OFDM", "1T", "09", "68",
	"IC", "2.4G", "20M", "OFDM", "1T", "09", "64",
	"KCC", "2.4G", "20M", "OFDM", "1T", "09", "64",
	"ACMA", "2.4G", "20M", "OFDM", "1T", "09", "56",
	"CHILE", "2.4G", "20M", "OFDM", "1T", "09", "64",
	"UKRAINE", "2.4G", "20M", "OFDM", "1T", "09", "56",
	"MEXICO", "2.4G", "20M", "OFDM", "1T", "09", "64",
	"CN", "2.4G", "20M", "OFDM", "1T", "09", "56",
	"FCC", "2.4G", "20M", "OFDM", "1T", "10", "64",
	"ETSI", "2.4G", "20M", "OFDM", "1T", "10", "56",
	"MKK", "2.4G", "20M", "OFDM", "1T", "10", "68",
	"IC", "2.4G", "20M", "OFDM", "1T", "10", "60",
	"KCC", "2.4G", "20M", "OFDM", "1T", "10", "64",
	"ACMA", "2.4G", "20M", "OFDM", "1T", "10", "56",
	"CHILE", "2.4G", "20M", "OFDM", "1T", "10", "60",
	"UKRAINE", "2.4G", "20M", "OFDM", "1T", "10", "56",
	"MEXICO", "2.4G", "20M", "OFDM", "1T", "10", "60",
	"CN", "2.4G", "20M", "OFDM", "1T", "10", "56",
	"FCC", "2.4G", "20M", "OFDM", "1T", "11", "60",
	"ETSI", "2.4G", "20M", "OFDM", "1T", "11", "56",
	"MKK", "2.4G", "20M", "OFDM", "1T", "11", "68",
	"IC", "2.4G", "20M", "OFDM", "1T", "11", "56",
	"KCC", "2.4G", "20M", "OFDM", "1T", "11", "64",
	"ACMA", "2.4G", "20M", "OFDM", "1T", "11", "56",
	"CHILE", "2.4G", "20M", "OFDM", "1T", "11", "56",
	"UKRAINE", "2.4G", "20M", "OFDM", "1T", "11", "56",
	"MEXICO", "2.4G", "20M", "OFDM", "1T", "11", "56",
	"CN", "2.4G", "20M", "OFDM", "1T", "11", "56",
	"FCC", "2.4G", "20M", "OFDM", "1T", "12", "48",
	"ETSI", "2.4G", "20M", "OFDM", "1T", "12", "56",
	"MKK", "2.4G", "20M", "OFDM", "1T", "12", "68",
	"IC", "2.4G", "20M", "OFDM", "1T", "12", "52",
	"KCC", "2.4G", "20M", "OFDM", "1T", "12", "64",
	"ACMA", "2.4G", "20M", "OFDM", "1T", "12", "56",
	"CHILE", "2.4G", "20M", "OFDM", "1T", "12", "52",
	"UKRAINE", "2.4G", "20M", "OFDM", "1T", "12", "56",
	"MEXICO", "2.4G", "20M", "OFDM", "1T", "12", "52",
	"CN", "2.4G", "20M", "OFDM", "1T", "12", "56",
	"FCC", "2.4G", "20M", "OFDM", "1T", "13", "24",
	"ETSI", "2.4G", "20M", "OFDM", "1T", "13", "56",
	"MKK", "2.4G", "20M", "OFDM", "1T", "13", "68",
	"IC", "2.4G", "20M", "OFDM", "1T", "13", "28",
	"KCC", "2.4G", "20M", "OFDM", "1T", "13", "60",
	"ACMA", "2.4G", "20M", "OFDM", "1T", "13", "56",
	"CHILE", "2.4G", "20M", "OFDM", "1T", "13", "28",
	"UKRAINE", "2.4G", "20M", "OFDM", "1T", "13", "56",
	"MEXICO", "2.4G", "20M", "OFDM", "1T", "13", "28",
	"CN", "2.4G", "20M", "OFDM", "1T", "13", "56",
	"FCC", "2.4G", "20M", "OFDM", "1T", "14", "127",
	"ETSI", "2.4G", "20M", "OFDM", "1T", "14", "127",
	"MKK", "2.4G", "20M", "OFDM", "1T", "14", "127",
	"IC", "2.4G", "20M", "OFDM", "1T", "14", "127",
	"KCC", "2.4G", "20M", "OFDM", "1T", "14", "127",
	"ACMA", "2.4G", "20M", "OFDM", "1T", "14", "127",
	"CHILE", "2.4G", "20M", "OFDM", "1T", "14", "127",
	"UKRAINE", "2.4G", "20M", "OFDM", "1T", "14", "127",
	"MEXICO", "2.4G", "20M", "OFDM", "1T", "14", "127",
	"CN", "2.4G", "20M", "OFDM", "1T", "14", "127",
	"FCC", "2.4G", "20M", "HT", "1T", "01", "64",
	"ETSI", "2.4G", "20M", "HT", "1T", "01", "56",
	"MKK", "2.4G", "20M", "HT", "1T", "01", "68",
	"IC", "2.4G", "20M", "HT", "1T", "01", "48",
	"KCC", "2.4G", "20M", "HT", "1T", "01", "60",
	"ACMA", "2.4G", "20M", "HT", "1T", "01", "56",
	"CHILE", "2.4G", "20M", "HT", "1T", "01", "48",
	"UKRAINE", "2.4G", "20M", "HT", "1T", "01", "56",
	"MEXICO", "2.4G", "20M", "HT", "1T", "01", "48",
	"CN", "2.4G", "20M", "HT", "1T", "01", "56",
	"FCC", "2.4G", "20M", "HT", "1T", "02", "64",
	"ETSI", "2.4G", "20M", "HT", "1T", "02", "56",
	"MKK", "2.4G", "20M", "HT", "1T", "02", "68",
	"IC", "2.4G", "20M", "HT", "1T", "02", "56",
	"KCC", "2.4G", "20M", "HT", "1T", "02", "64",
	"ACMA", "2.4G", "20M", "HT", "1T", "02", "56",
	"CHILE", "2.4G", "20M", "HT", "1T", "02", "56",
	"UKRAINE", "2.4G", "20M", "HT", "1T", "02", "56",
	"MEXICO", "2.4G", "20M", "HT", "1T", "02", "56",
	"CN", "2.4G", "20M", "HT", "1T", "02", "56",
	"FCC", "2.4G", "20M", "HT", "1T", "03", "64",
	"ETSI", "2.4G", "20M", "HT", "1T", "03", "56",
	"MKK", "2.4G", "20M", "HT", "1T", "03", "68",
	"IC", "2.4G", "20M", "HT", "1T", "03", "60",
	"KCC", "2.4G", "20M", "HT", "1T", "03", "64",
	"ACMA", "2.4G", "20M", "HT", "1T", "03", "56",
	"CHILE", "2.4G", "20M", "HT", "1T", "03", "60",
	"UKRAINE", "2.4G", "20M", "HT", "1T", "03", "56",
	"MEXICO", "2.4G", "20M", "HT", "1T", "03", "60",
	"CN", "2.4G", "20M", "HT", "1T", "03", "56",
	"FCC", "2.4G", "20M", "HT", "1T", "04", "64",
	"ETSI", "2.4G", "20M", "HT", "1T", "04", "56",
	"MKK", "2.4G", "20M", "HT", "1T", "04", "68",
	"IC", "2.4G", "20M", "HT", "1T", "04", "64",
	"KCC", "2.4G", "20M", "HT", "1T", "04", "64",
	"ACMA", "2.4G", "20M", "HT", "1T", "04", "56",
	"CHILE", "2.4G", "20M", "HT", "1T", "04", "64",
	"UKRAINE", "2.4G", "20M", "HT", "1T", "04", "56",
	"MEXICO", "2.4G", "20M", "HT", "1T", "04", "64",
	"CN", "2.4G", "20M", "HT", "1T", "04", "56",
	"FCC", "2.4G", "20M", "HT", "1T", "05", "64",
	"ETSI", "2.4G", "20M", "HT", "1T", "05", "56",
	"MKK", "2.4G", "20M", "HT", "1T", "05", "68",
	"IC", "2.4G", "20M", "HT", "1T", "05", "64",
	"KCC", "2.4G", "20M", "HT", "1T", "05", "64",
	"ACMA", "2.4G", "20M", "HT", "1T", "05", "56",
	"CHILE", "2.4G", "20M", "HT", "1T", "05", "64",
	"UKRAINE", "2.4G", "20M", "HT", "1T", "05", "56",
	"MEXICO", "2.4G", "20M", "HT", "1T", "05", "64",
	"CN", "2.4G", "20M", "HT", "1T", "05", "56",
	"FCC", "2.4G", "20M", "HT", "1T", "06", "64",
	"ETSI", "2.4G", "20M", "HT", "1T", "06", "56",
	"MKK", "2.4G", "20M", "HT", "1T", "06", "68",
	"IC", "2.4G", "20M", "HT", "1T", "06", "64",
	"KCC", "2.4G", "20M", "HT", "1T", "06", "64",
	"ACMA", "2.4G", "20M", "HT", "1T", "06", "56",
	"CHILE", "2.4G", "20M", "HT", "1T", "06", "64",
	"UKRAINE", "2.4G", "20M", "HT", "1T", "06", "56",
	"MEXICO", "2.4G", "20M", "HT", "1T", "06", "64",
	"CN", "2.4G", "20M", "HT", "1T", "06", "56",
	"FCC", "2.4G", "20M", "HT", "1T", "07", "64",
	"ETSI", "2.4G", "20M", "HT", "1T", "07", "56",
	"MKK", "2.4G", "20M", "HT", "1T", "07", "68",
	"IC", "2.4G", "20M", "HT", "1T", "07", "64",
	"KCC", "2.4G", "20M", "HT", "1T", "07", "64",
	"ACMA", "2.4G", "20M", "HT", "1T", "07", "56",
	"CHILE", "2.4G", "20M", "HT", "1T", "07", "64",
	"UKRAINE", "2.4G", "20M", "HT", "1T", "07", "56",
	"MEXICO", "2.4G", "20M", "HT", "1T", "07", "64",
	"CN", "2.4G", "20M", "HT", "1T", "07", "56",
	"FCC", "2.4G", "20M", "HT", "1T", "08", "64",
	"ETSI", "2.4G", "20M", "HT", "1T", "08", "56",
	"MKK", "2.4G", "20M", "HT", "1T", "08", "68",
	"IC", "2.4G", "20M", "HT", "1T", "08", "64",
	"KCC", "2.4G", "20M", "HT", "1T", "08", "64",
	"ACMA", "2.4G", "20M", "HT", "1T", "08", "56",
	"CHILE", "2.4G", "20M", "HT", "1T", "08", "64",
	"UKRAINE", "2.4G", "20M", "HT", "1T", "08", "56",
	"MEXICO", "2.4G", "20M", "HT", "1T", "08", "64",
	"CN", "2.4G", "20M", "HT", "1T", "08", "56",
	"FCC", "2.4G", "20M", "HT", "1T", "09", "64",
	"ETSI", "2.4G", "20M", "HT", "1T", "09", "56",
	"MKK", "2.4G", "20M", "HT", "1T", "09", "68",
	"IC", "2.4G", "20M", "HT", "1T", "09", "60",
	"KCC", "2.4G", "20M", "HT", "1T", "09", "64",
	"ACMA", "2.4G", "20M", "HT", "1T", "09", "56",
	"CHILE", "2.4G", "20M", "HT", "1T", "09", "60",
	"UKRAINE", "2.4G", "20M", "HT", "1T", "09", "56",
	"MEXICO", "2.4G", "20M", "HT", "1T", "09", "60",
	"CN", "2.4G", "20M", "HT", "1T", "09", "56",
	"FCC", "2.4G", "20M", "HT", "1T", "10", "60",
	"ETSI", "2.4G", "20M", "HT", "1T", "10", "56",
	"MKK", "2.4G", "20M", "HT", "1T", "10", "68",
	"IC", "2.4G", "20M", "HT", "1T", "10", "56",
	"KCC", "2.4G", "20M", "HT", "1T", "10", "64",
	"ACMA", "2.4G", "20M", "HT", "1T", "10", "56",
	"CHILE", "2.4G", "20M", "HT", "1T", "10", "56",
	"UKRAINE", "2.4G", "20M", "HT", "1T", "10", "56",
	"MEXICO", "2.4G", "20M", "HT", "1T", "10", "56",
	"CN", "2.4G", "20M", "HT", "1T", "10", "56",
	"FCC", "2.4G", "20M", "HT", "1T", "11", "56",
	"ETSI", "2.4G", "20M", "HT", "1T", "11", "56",
	"MKK", "2.4G", "20M", "HT", "1T", "11", "68",
	"IC", "2.4G", "20M", "HT", "1T", "11", "52",
	"KCC", "2.4G", "20M", "HT", "1T", "11", "64",
	"ACMA", "2.4G", "20M", "HT", "1T", "11", "56",
	"CHILE", "2.4G", "20M", "HT", "1T", "11", "52",
	"UKRAINE", "2.4G", "20M", "HT", "1T", "11", "56",
	"MEXICO", "2.4G", "20M", "HT", "1T", "11", "52",
	"CN", "2.4G", "20M", "HT", "1T", "11", "56",
	"FCC", "2.4G", "20M", "HT", "1T", "12", "48",
	"ETSI", "2.4G", "20M", "HT", "1T", "12", "56",
	"MKK", "2.4G", "20M", "HT", "1T", "12", "68",
	"IC", "2.4G", "20M", "HT", "1T", "12", "48",
	"KCC", "2.4G", "20M", "HT", "1T", "12", "64",
	"ACMA", "2.4G", "20M", "HT", "1T", "12", "56",
	"CHILE", "2.4G", "20M", "HT", "1T", "12", "48",
	"UKRAINE", "2.4G", "20M", "HT", "1T", "12", "56",
	"MEXICO", "2.4G", "20M", "HT", "1T", "12", "48",
	"CN", "2.4G", "20M", "HT", "1T", "12", "56",
	"FCC", "2.4G", "20M", "HT", "1T", "13", "24",
	"ETSI", "2.4G", "20M", "HT", "1T", "13", "56",
	"MKK", "2.4G", "20M", "HT", "1T", "13", "68",
	"IC", "2.4G", "20M", "HT", "1T", "13", "20",
	"KCC", "2.4G", "20M", "HT", "1T", "13", "60",
	"ACMA", "2.4G", "20M", "HT", "1T", "13", "56",
	"CHILE", "2.4G", "20M", "HT", "1T", "13", "20",
	"UKRAINE", "2.4G", "20M", "HT", "1T", "13", "56",
	"MEXICO", "2.4G", "20M", "HT", "1T", "13", "20",
	"CN", "2.4G", "20M", "HT", "1T", "13", "56",
	"FCC", "2.4G", "20M", "HT", "1T", "14", "127",
	"ETSI", "2.4G", "20M", "HT", "1T", "14", "127",
	"MKK", "2.4G", "20M", "HT", "1T", "14", "127",
	"IC", "2.4G", "20M", "HT", "1T", "14", "127",
	"KCC", "2.4G", "20M", "HT", "1T", "14", "127",
	"ACMA", "2.4G", "20M", "HT", "1T", "14", "127",
	"CHILE", "2.4G", "20M", "HT", "1T", "14", "127",
	"UKRAINE", "2.4G", "20M", "HT", "1T", "14", "127",
	"MEXICO", "2.4G", "20M", "HT", "1T", "14", "127",
	"CN", "2.4G", "20M", "HT", "1T", "14", "127",
	"FCC", "2.4G", "40M", "HT", "1T", "01", "127",
	"ETSI", "2.4G", "40M", "HT", "1T", "01", "127",
	"MKK", "2.4G", "40M", "HT", "1T", "01", "127",
	"IC", "2.4G", "40M", "HT", "1T", "01", "127",
	"KCC", "2.4G", "40M", "HT", "1T", "01", "127",
	"ACMA", "2.4G", "40M", "HT", "1T", "01", "127",
	"CHILE", "2.4G", "40M", "HT", "1T", "01", "127",
	"UKRAINE", "2.4G", "40M", "HT", "1T", "01", "127",
	"MEXICO", "2.4G", "40M", "HT", "1T", "01", "127",
	"CN", "2.4G", "40M", "HT", "1T", "01", "127",
	"FCC", "2.4G", "40M", "HT", "1T", "02", "127",
	"ETSI", "2.4G", "40M", "HT", "1T", "02", "127",
	"MKK", "2.4G", "40M", "HT", "1T", "02", "127",
	"IC", "2.4G", "40M", "HT", "1T", "02", "127",
	"KCC", "2.4G", "40M", "HT", "1T", "02", "127",
	"ACMA", "2.4G", "40M", "HT", "1T", "02", "127",
	"CHILE", "2.4G", "40M", "HT", "1T", "02", "127",
	"UKRAINE", "2.4G", "40M", "HT", "1T", "02", "127",
	"MEXICO", "2.4G", "40M", "HT", "1T", "02", "127",
	"CN", "2.4G", "40M", "HT", "1T", "02", "127",
	"FCC", "2.4G", "40M", "HT", "1T", "03", "52",
	"ETSI", "2.4G", "40M", "HT", "1T", "03", "56",
	"MKK", "2.4G", "40M", "HT", "1T", "03", "60",
	"IC", "2.4G", "40M", "HT", "1T", "03", "48",
	"KCC", "2.4G", "40M", "HT", "1T", "03", "56",
	"ACMA", "2.4G", "40M", "HT", "1T", "03", "56",
	"CHILE", "2.4G", "40M", "HT", "1T", "03", "48",
	"UKRAINE", "2.4G", "40M", "HT", "1T", "03", "56",
	"MEXICO", "2.4G", "40M", "HT", "1T", "03", "48",
	"CN", "2.4G", "40M", "HT", "1T", "03", "56",
	"FCC", "2.4G", "40M", "HT", "1T", "04", "52",
	"ETSI", "2.4G", "40M", "HT", "1T", "04", "56",
	"MKK", "2.4G", "40M", "HT", "1T", "04", "60",
	"IC", "2.4G", "40M", "HT", "1T", "04", "48",
	"KCC", "2.4G", "40M", "HT", "1T", "04", "56",
	"ACMA", "2.4G", "40M", "HT", "1T", "04", "56",
	"CHILE", "2.4G", "40M", "HT", "1T", "04", "48",
	"UKRAINE", "2.4G", "40M", "HT", "1T", "04", "56",
	"MEXICO", "2.4G", "40M", "HT", "1T", "04", "48",
	"CN", "2.4G", "40M", "HT", "1T", "04", "56",
	"FCC", "2.4G", "40M", "HT", "1T", "05", "56",
	"ETSI", "2.4G", "40M", "HT", "1T", "05", "56",
	"MKK", "2.4G", "40M", "HT", "1T", "05", "60",
	"IC", "2.4G", "40M", "HT", "1T", "05", "56",
	"KCC", "2.4G", "40M", "HT", "1T", "05", "56",
	"ACMA", "2.4G", "40M", "HT", "1T", "05", "56",
	"CHILE", "2.4G", "40M", "HT", "1T", "05", "56",
	"UKRAINE", "2.4G", "40M", "HT", "1T", "05", "56",
	"MEXICO", "2.4G", "40M", "HT", "1T", "05", "56",
	"CN", "2.4G", "40M", "HT", "1T", "05", "56",
	"FCC", "2.4G", "40M", "HT", "1T", "06", "56",
	"ETSI", "2.4G", "40M", "HT", "1T", "06", "56",
	"MKK", "2.4G", "40M", "HT", "1T", "06", "60",
	"IC", "2.4G", "40M", "HT", "1T", "06", "56",
	"KCC", "2.4G", "40M", "HT", "1T", "06", "56",
	"ACMA", "2.4G", "40M", "HT", "1T", "06", "56",
	"CHILE", "2.4G", "40M", "HT", "1T", "06", "56",
	"UKRAINE", "2.4G", "40M", "HT", "1T", "06", "56",
	"MEXICO", "2.4G", "40M", "HT", "1T", "06", "56",
	"CN", "2.4G", "40M", "HT", "1T", "06", "56",
	"FCC", "2.4G", "40M", "HT", "1T", "07", "56",
	"ETSI", "2.4G", "40M", "HT", "1T", "07", "56",
	"MKK", "2.4G", "40M", "HT", "1T", "07", "60",
	"IC", "2.4G", "40M", "HT", "1T", "07", "56",
	"KCC", "2.4G", "40M", "HT", "1T", "07", "56",
	"ACMA", "2.4G", "40M", "HT", "1T", "07", "56",
	"CHILE", "2.4G", "40M", "HT", "1T", "07", "56",
	"UKRAINE", "2.4G", "40M", "HT", "1T", "07", "56",
	"MEXICO", "2.4G", "40M", "HT", "1T", "07", "56",
	"CN", "2.4G", "40M", "HT", "1T", "07", "56",
	"FCC", "2.4G", "40M", "HT", "1T", "08", "52",
	"ETSI", "2.4G", "40M", "HT", "1T", "08", "56",
	"MKK", "2.4G", "40M", "HT", "1T", "08", "60",
	"IC", "2.4G", "40M", "HT", "1T", "08", "48",
	"KCC", "2.4G", "40M", "HT", "1T", "08", "56",
	"ACMA", "2.4G", "40M", "HT", "1T", "08", "56",
	"CHILE", "2.4G", "40M", "HT", "1T", "08", "48",
	"UKRAINE", "2.4G", "40M", "HT", "1T", "08", "56",
	"MEXICO", "2.4G", "40M", "HT", "1T", "08", "48",
	"CN", "2.4G", "40M", "HT", "1T", "08", "56",
	"FCC", "2.4G", "40M", "HT", "1T", "09", "48",
	"ETSI", "2.4G", "40M", "HT", "1T", "09", "56",
	"MKK", "2.4G", "40M", "HT", "1T", "09", "60",
	"IC", "2.4G", "40M", "HT", "1T", "09", "48",
	"KCC", "2.4G", "40M", "HT", "1T", "09", "56",
	"ACMA", "2.4G", "40M", "HT", "1T", "09", "56",
	"CHILE", "2.4G", "40M", "HT", "1T", "09", "48",
	"UKRAINE", "2.4G", "40M", "HT", "1T", "09", "56",
	"MEXICO", "2.4G", "40M", "HT", "1T", "09", "48",
	"CN", "2.4G", "40M", "HT", "1T", "09", "56",
	"FCC", "2.4G", "40M", "HT", "1T", "10", "40",
	"ETSI", "2.4G", "40M", "HT", "1T", "10", "56",
	"MKK", "2.4G", "40M", "HT", "1T", "10", "60",
	"IC", "2.4G", "40M", "HT", "1T", "10", "52",
	"KCC", "2.4G", "40M", "HT", "1T", "10", "56",
	"ACMA", "2.4G", "40M", "HT", "1T", "10", "56",
	"CHILE", "2.4G", "40M", "HT", "1T", "10", "52",
	"UKRAINE", "2.4G", "40M", "HT", "1T", "10", "56",
	"MEXICO", "2.4G", "40M", "HT", "1T", "10", "52",
	"CN", "2.4G", "40M", "HT", "1T", "10", "56",
	"FCC", "2.4G", "40M", "HT", "1T", "11", "36",
	"ETSI", "2.4G", "40M", "HT", "1T", "11", "56",
	"MKK", "2.4G", "40M", "HT", "1T", "11", "60",
	"IC", "2.4G", "40M", "HT", "1T", "11", "36",
	"KCC", "2.4G", "40M", "HT", "1T", "11", "56",
	"ACMA", "2.4G", "40M", "HT", "1T", "11", "56",
	"CHILE", "2.4G", "40M", "HT", "1T", "11", "36",
	"UKRAINE", "2.4G", "40M", "HT", "1T", "11", "56",
	"MEXICO", "2.4G", "40M", "HT", "1T", "11", "36",
	"CN", "2.4G", "40M", "HT", "1T", "11", "56",
	"FCC", "2.4G", "40M", "HT", "1T", "12", "127",
	"ETSI", "2.4G", "40M", "HT", "1T", "12", "127",
	"MKK", "2.4G", "40M", "HT", "1T", "12", "127",
	"IC", "2.4G", "40M", "HT", "1T", "12", "127",
	"KCC", "2.4G", "40M", "HT", "1T", "12", "127",
	"ACMA", "2.4G", "40M", "HT", "1T", "12", "127",
	"CHILE", "2.4G", "40M", "HT", "1T", "12", "127",
	"UKRAINE", "2.4G", "40M", "HT", "1T", "12", "127",
	"MEXICO", "2.4G", "40M", "HT", "1T", "12", "127",
	"CN", "2.4G", "40M", "HT", "1T", "12", "127",
	"FCC", "2.4G", "40M", "HT", "1T", "13", "127",
	"ETSI", "2.4G", "40M", "HT", "1T", "13", "127",
	"MKK", "2.4G", "40M", "HT", "1T", "13", "127",
	"IC", "2.4G", "40M", "HT", "1T", "13", "127",
	"KCC", "2.4G", "40M", "HT", "1T", "13", "127",
	"ACMA", "2.4G", "40M", "HT", "1T", "13", "127",
	"CHILE", "2.4G", "40M", "HT", "1T", "13", "127",
	"UKRAINE", "2.4G", "40M", "HT", "1T", "13", "127",
	"MEXICO", "2.4G", "40M", "HT", "1T", "13", "127",
	"CN", "2.4G", "40M", "HT", "1T", "13", "127",
	"FCC", "2.4G", "40M", "HT", "1T", "14", "127",
	"ETSI", "2.4G", "40M", "HT", "1T", "14", "127",
	"MKK", "2.4G", "40M", "HT", "1T", "14", "127",
	"IC", "2.4G", "40M", "HT", "1T", "14", "127",
	"KCC", "2.4G", "40M", "HT", "1T", "14", "127",
	"ACMA", "2.4G", "40M", "HT", "1T", "14", "127",
	"CHILE", "2.4G", "40M", "HT", "1T", "14", "127",
	"UKRAINE", "2.4G", "40M", "HT", "1T", "14", "127",
	"MEXICO", "2.4G", "40M", "HT", "1T", "14", "127",
	"CN", "2.4G", "40M", "HT", "1T", "14", "",
	"FCC", "5G", "20M", "OFDM", "1T", "36", "64",
	"ETSI", "5G", "20M", "OFDM", "1T", "36", "56",
	"MKK", "5G", "20M", "OFDM", "1T", "36", "66",
	"IC", "5G", "20M", "OFDM", "1T", "36", "58",
	"KCC", "5G", "20M", "OFDM", "1T", "36", "54",
	"ACMA", "5G", "20M", "OFDM", "1T", "36", "60",
	"CHILE", "5G", "20M", "OFDM", "1T", "36", "54",
	"UKRAINE", "5G", "20M", "OFDM", "1T", "36", "50",
	"MEXICO", "5G", "20M", "OFDM", "1T", "36", "58",
	"CN", "5G", "20M", "OFDM", "1T", "36", "56",
	"FCC", "5G", "20M", "OFDM", "1T", "40", "64",
	"ETSI", "5G", "20M", "OFDM", "1T", "40", "56",
	"MKK", "5G", "20M", "OFDM", "1T", "40", "66",
	"IC", "5G", "20M", "OFDM", "1T", "40", "58",
	"KCC", "5G", "20M", "OFDM", "1T", "40", "52",
	"ACMA", "5G", "20M", "OFDM", "1T", "40", "60",
	"CHILE", "5G", "20M", "OFDM", "1T", "40", "54",
	"UKRAINE", "5G", "20M", "OFDM", "1T", "40", "50",
	"MEXICO", "5G", "20M", "OFDM", "1T", "40", "58",
	"CN", "5G", "20M", "OFDM", "1T", "40", "56",
	"FCC", "5G", "20M", "OFDM", "1T", "44", "64",
	"ETSI", "5G", "20M", "OFDM", "1T", "44", "56",
	"MKK", "5G", "20M", "OFDM", "1T", "44", "66",
	"IC", "5G", "20M", "OFDM", "1T", "44", "58",
	"KCC", "5G", "20M", "OFDM", "1T", "44", "52",
	"ACMA", "5G", "20M", "OFDM", "1T", "44", "60",
	"CHILE", "5G", "20M", "OFDM", "1T", "44", "56",
	"UKRAINE", "5G", "20M", "OFDM", "1T", "44", "50",
	"MEXICO", "5G", "20M", "OFDM", "1T", "44", "58",
	"CN", "5G", "20M", "OFDM", "1T", "44", "56",
	"FCC", "5G", "20M", "OFDM", "1T", "48", "64",
	"ETSI", "5G", "20M", "OFDM", "1T", "48", "56",
	"MKK", "5G", "20M", "OFDM", "1T", "48", "66",
	"IC", "5G", "20M", "OFDM", "1T", "48", "58",
	"KCC", "5G", "20M", "OFDM", "1T", "48", "50",
	"ACMA", "5G", "20M", "OFDM", "1T", "48", "60",
	"CHILE", "5G", "20M", "OFDM", "1T", "48", "56",
	"UKRAINE", "5G", "20M", "OFDM", "1T", "48", "50",
	"MEXICO", "5G", "20M", "OFDM", "1T", "48", "58",
	"CN", "5G", "20M", "OFDM", "1T", "48", "56",
	"FCC", "5G", "20M", "OFDM", "1T", "52", "64",
	"ETSI", "5G", "20M", "OFDM", "1T", "52", "56",
	"MKK", "5G", "20M", "OFDM", "1T", "52", "66",
	"IC", "5G", "20M", "OFDM", "1T", "52", "60",
	"KCC", "5G", "20M", "OFDM", "1T", "52", "28",
	"ACMA", "5G", "20M", "OFDM", "1T", "52", "60",
	"CHILE", "5G", "20M", "OFDM", "1T", "52", "56",
	"UKRAINE", "5G", "20M", "OFDM", "1T", "52", "50",
	"MEXICO", "5G", "20M", "OFDM", "1T", "52", "58",
	"CN", "5G", "20M", "OFDM", "1T", "52", "56",
	"FCC", "5G", "20M", "OFDM", "1T", "56", "64",
	"ETSI", "5G", "20M", "OFDM", "1T", "56", "56",
	"MKK", "5G", "20M", "OFDM", "1T", "56", "66",
	"IC", "5G", "20M", "OFDM", "1T", "56", "60",
	"KCC", "5G", "20M", "OFDM", "1T", "56", "62",
	"ACMA", "5G", "20M", "OFDM", "1T", "56", "60",
	"CHILE", "5G", "20M", "OFDM", "1T", "56", "56",
	"UKRAINE", "5G", "20M", "OFDM", "1T", "56", "50",
	"MEXICO", "5G", "20M", "OFDM", "1T", "56", "62",
	"CN", "5G", "20M", "OFDM", "1T", "56", "56",
	"FCC", "5G", "20M", "OFDM", "1T", "60", "64",
	"ETSI", "5G", "20M", "OFDM", "1T", "60", "56",
	"MKK", "5G", "20M", "OFDM", "1T", "60", "66",
	"IC", "5G", "20M", "OFDM", "1T", "60", "60",
	"KCC", "5G", "20M", "OFDM", "1T", "60", "62",
	"ACMA", "5G", "20M", "OFDM", "1T", "60", "60",
	"CHILE", "5G", "20M", "OFDM", "1T", "60", "56",
	"UKRAINE", "5G", "20M", "OFDM", "1T", "60", "50",
	"MEXICO", "5G", "20M", "OFDM", "1T", "60", "62",
	"CN", "5G", "20M", "OFDM", "1T", "60", "56",
	"FCC", "5G", "20M", "OFDM", "1T", "64", "64",
	"ETSI", "5G", "20M", "OFDM", "1T", "64", "56",
	"MKK", "5G", "20M", "OFDM", "1T", "64", "66",
	"IC", "5G", "20M", "OFDM", "1T", "64", "56",
	"KCC", "5G", "20M", "OFDM", "1T", "64", "62",
	"ACMA", "5G", "20M", "OFDM", "1T", "64", "60",
	"CHILE", "5G", "20M", "OFDM", "1T", "64", "54",
	"UKRAINE", "5G", "20M", "OFDM", "1T", "64", "50",
	"MEXICO", "5G", "20M", "OFDM", "1T", "64", "56",
	"CN", "5G", "20M", "OFDM", "1T", "64", "56",
	"FCC", "5G", "20M", "OFDM", "1T", "100", "64",
	"ETSI", "5G", "20M", "OFDM", "1T", "100", "56",
	"MKK", "5G", "20M", "OFDM", "1T", "100", "66",
	"IC", "5G", "20M", "OFDM", "1T", "100", "60",
	"KCC", "5G", "20M", "OFDM", "1T", "100", "62",
	"ACMA", "5G", "20M", "OFDM", "1T", "100", "60",
	"CHILE", "5G", "20M", "OFDM", "1T", "100", "56",
	"UKRAINE", "5G", "20M", "OFDM", "1T", "100", "50",
	"MEXICO", "5G", "20M", "OFDM", "1T", "100", "56",
	"CN", "5G", "20M", "OFDM", "1T", "100", "127",
	"FCC", "5G", "20M", "OFDM", "1T", "104", "64",
	"ETSI", "5G", "20M", "OFDM", "1T", "104", "56",
	"MKK", "5G", "20M", "OFDM", "1T", "104", "66",
	"IC", "5G", "20M", "OFDM", "1T", "104", "62",
	"KCC", "5G", "20M", "OFDM", "1T", "104", "62",
	"ACMA", "5G", "20M", "OFDM", "1T", "104", "60",
	"CHILE", "5G", "20M", "OFDM", "1T", "104", "56",
	"UKRAINE", "5G", "20M", "OFDM", "1T", "104", "50",
	"MEXICO", "5G", "20M", "OFDM", "1T", "104", "62",
	"CN", "5G", "20M", "OFDM", "1T", "104", "127",
	"FCC", "5G", "20M", "OFDM", "1T", "108", "64",
	"ETSI", "5G", "20M", "OFDM", "1T", "108", "56",
	"MKK", "5G", "20M", "OFDM", "1T", "108", "66",
	"IC", "5G", "20M", "OFDM", "1T", "108", "62",
	"KCC", "5G", "20M", "OFDM", "1T", "108", "62",
	"ACMA", "5G", "20M", "OFDM", "1T", "108", "60",
	"CHILE", "5G", "20M", "OFDM", "1T", "108", "56",
	"UKRAINE", "5G", "20M", "OFDM", "1T", "108", "50",
	"MEXICO", "5G", "20M", "OFDM", "1T", "108", "62",
	"CN", "5G", "20M", "OFDM", "1T", "108", "127",
	"FCC", "5G", "20M", "OFDM", "1T", "112", "64",
	"ETSI", "5G", "20M", "OFDM", "1T", "112", "56",
	"MKK", "5G", "20M", "OFDM", "1T", "112", "66",
	"IC", "5G", "20M", "OFDM", "1T", "112", "62",
	"KCC", "5G", "20M", "OFDM", "1T", "112", "62",
	"ACMA", "5G", "20M", "OFDM", "1T", "112", "60",
	"CHILE", "5G", "20M", "OFDM", "1T", "112", "56",
	"UKRAINE", "5G", "20M", "OFDM", "1T", "112", "50",
	"MEXICO", "5G", "20M", "OFDM", "1T", "112", "62",
	"CN", "5G", "20M", "OFDM", "1T", "112", "127",
	"FCC", "5G", "20M", "OFDM", "1T", "116", "64",
	"ETSI", "5G", "20M", "OFDM", "1T", "116", "56",
	"MKK", "5G", "20M", "OFDM", "1T", "116", "66",
	"IC", "5G", "20M", "OFDM", "1T", "116", "62",
	"KCC", "5G", "20M", "OFDM", "1T", "116", "62",
	"ACMA", "5G", "20M", "OFDM", "1T", "116", "60",
	"CHILE", "5G", "20M", "OFDM", "1T", "116", "56",
	"UKRAINE", "5G", "20M", "OFDM", "1T", "116", "50",
	"MEXICO", "5G", "20M", "OFDM", "1T", "116", "62",
	"CN", "5G", "20M", "OFDM", "1T", "116", "127",
	"FCC", "5G", "20M", "OFDM", "1T", "120", "64",
	"ETSI", "5G", "20M", "OFDM", "1T", "120", "56",
	"MKK", "5G", "20M", "OFDM", "1T", "120", "66",
	"IC", "5G", "20M", "OFDM", "1T", "120", "127",
	"KCC", "5G", "20M", "OFDM", "1T", "120", "62",
	"ACMA", "5G", "20M", "OFDM", "1T", "120", "127",
	"CHILE", "5G", "20M", "OFDM", "1T", "120", "56",
	"UKRAINE", "5G", "20M", "OFDM", "1T", "120", "50",
	"MEXICO", "5G", "20M", "OFDM", "1T", "120", "62",
	"CN", "5G", "20M", "OFDM", "1T", "120", "127",
	"FCC", "5G", "20M", "OFDM", "1T", "124", "64",
	"ETSI", "5G", "20M", "OFDM", "1T", "124", "56",
	"MKK", "5G", "20M", "OFDM", "1T", "124", "66",
	"IC", "5G", "20M", "OFDM", "1T", "124", "127",
	"KCC", "5G", "20M", "OFDM", "1T", "124", "62",
	"ACMA", "5G", "20M", "OFDM", "1T", "124", "127",
	"CHILE", "5G", "20M", "OFDM", "1T", "124", "56",
	"UKRAINE", "5G", "20M", "OFDM", "1T", "124", "50",
	"MEXICO", "5G", "20M", "OFDM", "1T", "124", "62",
	"CN", "5G", "20M", "OFDM", "1T", "124", "127",
	"FCC", "5G", "20M", "OFDM", "1T", "128", "64",
	"ETSI", "5G", "20M", "OFDM", "1T", "128", "56",
	"MKK", "5G", "20M", "OFDM", "1T", "128", "66",
	"IC", "5G", "20M", "OFDM", "1T", "128", "127",
	"KCC", "5G", "20M", "OFDM", "1T", "128", "-128",
	"ACMA", "5G", "20M", "OFDM", "1T", "128", "127",
	"CHILE", "5G", "20M", "OFDM", "1T", "128", "56",
	"UKRAINE", "5G", "20M", "OFDM", "1T", "128", "50",
	"MEXICO", "5G", "20M", "OFDM", "1T", "128", "62",
	"CN", "5G", "20M", "OFDM", "1T", "128", "127",
	"FCC", "5G", "20M", "OFDM", "1T", "132", "64",
	"ETSI", "5G", "20M", "OFDM", "1T", "132", "56",
	"MKK", "5G", "20M", "OFDM", "1T", "132", "66",
	"IC", "5G", "20M", "OFDM", "1T", "132", "62",
	"KCC", "5G", "20M", "OFDM", "1T", "132", "-128",
	"ACMA", "5G", "20M", "OFDM", "1T", "132", "60",
	"CHILE", "5G", "20M", "OFDM", "1T", "132", "56",
	"UKRAINE", "5G", "20M", "OFDM", "1T", "132", "50",
	"MEXICO", "5G", "20M", "OFDM", "1T", "132", "62",
	"CN", "5G", "20M", "OFDM", "1T", "132", "127",
	"FCC", "5G", "20M", "OFDM", "1T", "136", "64",
	"ETSI", "5G", "20M", "OFDM", "1T", "136", "56",
	"MKK", "5G", "20M", "OFDM", "1T", "136", "66",
	"IC", "5G", "20M", "OFDM", "1T", "136", "62",
	"KCC", "5G", "20M", "OFDM", "1T", "136", "-128",
	"ACMA", "5G", "20M", "OFDM", "1T", "136", "60",
	"CHILE", "5G", "20M", "OFDM", "1T", "136", "56",
	"UKRAINE", "5G", "20M", "OFDM", "1T", "136", "127",
	"MEXICO", "5G", "20M", "OFDM", "1T", "136", "62",
	"CN", "5G", "20M", "OFDM", "1T", "136", "127",
	"FCC", "5G", "20M", "OFDM", "1T", "140", "56",
	"ETSI", "5G", "20M", "OFDM", "1T", "140", "56",
	"MKK", "5G", "20M", "OFDM", "1T", "140", "66",
	"IC", "5G", "20M", "OFDM", "1T", "140", "58",
	"KCC", "5G", "20M", "OFDM", "1T", "140", "-128",
	"ACMA", "5G", "20M", "OFDM", "1T", "140", "60",
	"CHILE", "5G", "20M", "OFDM", "1T", "140", "56",
	"UKRAINE", "5G", "20M", "OFDM", "1T", "140", "127",
	"MEXICO", "5G", "20M", "OFDM", "1T", "140", "58",
	"CN", "5G", "20M", "OFDM", "1T", "140", "127",
	"FCC", "5G", "20M", "OFDM", "1T", "144", "48",
	"ETSI", "5G", "20M", "OFDM", "1T", "144", "127",
	"MKK", "5G", "20M", "OFDM", "1T", "144", "127",
	"IC", "5G", "20M", "OFDM", "1T", "144", "56",
	"KCC", "5G", "20M", "OFDM", "1T", "144", "-128",
	"ACMA", "5G", "20M", "OFDM", "1T", "144", "127",
	"CHILE", "5G", "20M", "OFDM", "1T", "144", "56",
	"UKRAINE", "5G", "20M", "OFDM", "1T", "144", "127",
	"MEXICO", "5G", "20M", "OFDM", "1T", "144", "56",
	"CN", "5G", "20M", "OFDM", "1T", "144", "127",
	"FCC", "5G", "20M", "OFDM", "1T", "149", "64",
	"ETSI", "5G", "20M", "OFDM", "1T", "149", "-128",
	"MKK", "5G", "20M", "OFDM", "1T", "149", "127",
	"IC", "5G", "20M", "OFDM", "1T", "149", "56",
	"KCC", "5G", "20M", "OFDM", "1T", "149", "62",
	"ACMA", "5G", "20M", "OFDM", "1T", "149", "62",
	"CHILE", "5G", "20M", "OFDM", "1T", "149", "56",
	"UKRAINE", "5G", "20M", "OFDM", "1T", "149", "50",
	"MEXICO", "5G", "20M", "OFDM", "1T", "149", "62",
	"CN", "5G", "20M", "OFDM", "1T", "149", "-128",
	"FCC", "5G", "20M", "OFDM", "1T", "153", "64",
	"ETSI", "5G", "20M", "OFDM", "1T", "153", "-128",
	"MKK", "5G", "20M", "OFDM", "1T", "153", "127",
	"IC", "5G", "20M", "OFDM", "1T", "153", "62",
	"KCC", "5G", "20M", "OFDM", "1T", "153", "62",
	"ACMA", "5G", "20M", "OFDM", "1T", "153", "62",
	"CHILE", "5G", "20M", "OFDM", "1T", "153", "56",
	"UKRAINE", "5G", "20M", "OFDM", "1T", "153", "50",
	"MEXICO", "5G", "20M", "OFDM", "1T", "153", "62",
	"CN", "5G", "20M", "OFDM", "1T", "153", "-128",
	"FCC", "5G", "20M", "OFDM", "1T", "157", "64",
	"ETSI", "5G", "20M", "OFDM", "1T", "157", "-128",
	"MKK", "5G", "20M", "OFDM", "1T", "157", "127",
	"IC", "5G", "20M", "OFDM", "1T", "157", "62",
	"KCC", "5G", "20M", "OFDM", "1T", "157", "62",
	"ACMA", "5G", "20M", "OFDM", "1T", "157", "62",
	"CHILE", "5G", "20M", "OFDM", "1T", "157", "56",
	"UKRAINE", "5G", "20M", "OFDM", "1T", "157", "50",
	"MEXICO", "5G", "20M", "OFDM", "1T", "157", "62",
	"CN", "5G", "20M", "OFDM", "1T", "157", "-128",
	"FCC", "5G", "20M", "OFDM", "1T", "161", "64",
	"ETSI", "5G", "20M", "OFDM", "1T", "161", "-128",
	"MKK", "5G", "20M", "OFDM", "1T", "161", "127",
	"IC", "5G", "20M", "OFDM", "1T", "161", "62",
	"KCC", "5G", "20M", "OFDM", "1T", "161", "58",
	"ACMA", "5G", "20M", "OFDM", "1T", "161", "62",
	"CHILE", "5G", "20M", "OFDM", "1T", "161", "56",
	"UKRAINE", "5G", "20M", "OFDM", "1T", "161", "50",
	"MEXICO", "5G", "20M", "OFDM", "1T", "161", "62",
	"CN", "5G", "20M", "OFDM", "1T", "161", "-128",
	"FCC", "5G", "20M", "OFDM", "1T", "165", "60",
	"ETSI", "5G", "20M", "OFDM", "1T", "165", "-128",
	"MKK", "5G", "20M", "OFDM", "1T", "165", "127",
	"IC", "5G", "20M", "OFDM", "1T", "165", "62",
	"KCC", "5G", "20M", "OFDM", "1T", "165", "-128",
	"ACMA", "5G", "20M", "OFDM", "1T", "165", "62",
	"CHILE", "5G", "20M", "OFDM", "1T", "165", "56",
	"UKRAINE", "5G", "20M", "OFDM", "1T", "165", "50",
	"MEXICO", "5G", "20M", "OFDM", "1T", "165", "56",
	"CN", "5G", "20M", "OFDM", "1T", "165", "-128",
	"FCC", "5G", "20M", "HT", "1T", "36", "60",
	"ETSI", "5G", "20M", "HT", "1T", "36", "56",
	"MKK", "5G", "20M", "HT", "1T", "36", "66",
	"IC", "5G", "20M", "HT", "1T", "36", "56",
	"KCC", "5G", "20M", "HT", "1T", "36", "54",
	"ACMA", "5G", "20M", "HT", "1T", "36", "60",
	"CHILE", "5G", "20M", "HT", "1T", "36", "56",
	"UKRAINE", "5G", "20M", "HT", "1T", "36", "50",
	"MEXICO", "5G", "20M", "HT", "1T", "36", "56",
	"CN", "5G", "20M", "HT", "1T", "36", "56",
	"FCC", "5G", "20M", "HT", "1T", "40", "66",
	"ETSI", "5G", "20M", "HT", "1T", "40", "56",
	"MKK", "5G", "20M", "HT", "1T", "40", "66",
	"IC", "5G", "20M", "HT", "1T", "40", "58",
	"KCC", "5G", "20M", "HT", "1T", "40", "54",
	"ACMA", "5G", "20M", "HT", "1T", "40", "60",
	"CHILE", "5G", "20M", "HT", "1T", "40", "56",
	"UKRAINE", "5G", "20M", "HT", "1T", "40", "50",
	"MEXICO", "5G", "20M", "HT", "1T", "40", "58",
	"CN", "5G", "20M", "HT", "1T", "40", "56",
	"FCC", "5G", "20M", "HT", "1T", "44", "66",
	"ETSI", "5G", "20M", "HT", "1T", "44", "56",
	"MKK", "5G", "20M", "HT", "1T", "44", "66",
	"IC", "5G", "20M", "HT", "1T", "44", "58",
	"KCC", "5G", "20M", "HT", "1T", "44", "54",
	"ACMA", "5G", "20M", "HT", "1T", "44", "60",
	"CHILE", "5G", "20M", "HT", "1T", "44", "56",
	"UKRAINE", "5G", "20M", "HT", "1T", "44", "50",
	"MEXICO", "5G", "20M", "HT", "1T", "44", "58",
	"CN", "5G", "20M", "HT", "1T", "44", "56",
	"FCC", "5G", "20M", "HT", "1T", "48", "66",
	"ETSI", "5G", "20M", "HT", "1T", "48", "56",
	"MKK", "5G", "20M", "HT", "1T", "48", "66",
	"IC", "5G", "20M", "HT", "1T", "48", "58",
	"KCC", "5G", "20M", "HT", "1T", "48", "48",
	"ACMA", "5G", "20M", "HT", "1T", "48", "60",
	"CHILE", "5G", "20M", "HT", "1T", "48", "56",
	"UKRAINE", "5G", "20M", "HT", "1T", "48", "50",
	"MEXICO", "5G", "20M", "HT", "1T", "48", "58",
	"CN", "5G", "20M", "HT", "1T", "48", "56",
	"FCC", "5G", "20M", "HT", "1T", "52", "66",
	"ETSI", "5G", "20M", "HT", "1T", "52", "56",
	"MKK", "5G", "20M", "HT", "1T", "52", "66",
	"IC", "5G", "20M", "HT", "1T", "52", "60",
	"KCC", "5G", "20M", "HT", "1T", "52", "50",
	"ACMA", "5G", "20M", "HT", "1T", "52", "60",
	"CHILE", "5G", "20M", "HT", "1T", "52", "56",
	"UKRAINE", "5G", "20M", "HT", "1T", "52", "50",
	"MEXICO", "5G", "20M", "HT", "1T", "52", "62",
	"CN", "5G", "20M", "HT", "1T", "52", "56",
	"FCC", "5G", "20M", "HT", "1T", "56", "66",
	"ETSI", "5G", "20M", "HT", "1T", "56", "56",
	"MKK", "5G", "20M", "HT", "1T", "56", "66",
	"IC", "5G", "20M", "HT", "1T", "56", "60",
	"KCC", "5G", "20M", "HT", "1T", "56", "62",
	"ACMA", "5G", "20M", "HT", "1T", "56", "60",
	"CHILE", "5G", "20M", "HT", "1T", "56", "56",
	"UKRAINE", "5G", "20M", "HT", "1T", "56", "50",
	"MEXICO", "5G", "20M", "HT", "1T", "56", "58",
	"CN", "5G", "20M", "HT", "1T", "56", "56",
	"FCC", "5G", "20M", "HT", "1T", "60", "64",
	"ETSI", "5G", "20M", "HT", "1T", "60", "56",
	"MKK", "5G", "20M", "HT", "1T", "60", "66",
	"IC", "5G", "20M", "HT", "1T", "60", "60",
	"KCC", "5G", "20M", "HT", "1T", "60", "62",
	"ACMA", "5G", "20M", "HT", "1T", "60", "60",
	"CHILE", "5G", "20M", "HT", "1T", "60", "56",
	"UKRAINE", "5G", "20M", "HT", "1T", "60", "50",
	"MEXICO", "5G", "20M", "HT", "1T", "60", "58",
	"CN", "5G", "20M", "HT", "1T", "60", "56",
	"FCC", "5G", "20M", "HT", "1T", "64", "60",
	"ETSI", "5G", "20M", "HT", "1T", "64", "56",
	"MKK", "5G", "20M", "HT", "1T", "64", "66",
	"IC", "5G", "20M", "HT", "1T", "64", "56",
	"KCC", "5G", "20M", "HT", "1T", "64", "62",
	"ACMA", "5G", "20M", "HT", "1T", "64", "60",
	"CHILE", "5G", "20M", "HT", "1T", "64", "56",
	"UKRAINE", "5G", "20M", "HT", "1T", "64", "50",
	"MEXICO", "5G", "20M", "HT", "1T", "64", "56",
	"CN", "5G", "20M", "HT", "1T", "64", "56",
	"FCC", "5G", "20M", "HT", "1T", "100", "60",
	"ETSI", "5G", "20M", "HT", "1T", "100", "56",
	"MKK", "5G", "20M", "HT", "1T", "100", "66",
	"IC", "5G", "20M", "HT", "1T", "100", "56",
	"KCC", "5G", "20M", "HT", "1T", "100", "62",
	"ACMA", "5G", "20M", "HT", "1T", "100", "60",
	"CHILE", "5G", "20M", "HT", "1T", "100", "56",
	"UKRAINE", "5G", "20M", "HT", "1T", "100", "50",
	"MEXICO", "5G", "20M", "HT", "1T", "100", "56",
	"CN", "5G", "20M", "HT", "1T", "100", "127",
	"FCC", "5G", "20M", "HT", "1T", "104", "66",
	"ETSI", "5G", "20M", "HT", "1T", "104", "56",
	"MKK", "5G", "20M", "HT", "1T", "104", "66",
	"IC", "5G", "20M", "HT", "1T", "104", "62",
	"KCC", "5G", "20M", "HT", "1T", "104", "62",
	"ACMA", "5G", "20M", "HT", "1T", "104", "60",
	"CHILE", "5G", "20M", "HT", "1T", "104", "56",
	"UKRAINE", "5G", "20M", "HT", "1T", "104", "50",
	"MEXICO", "5G", "20M", "HT", "1T", "104", "62",
	"CN", "5G", "20M", "HT", "1T", "104", "127",
	"FCC", "5G", "20M", "HT", "1T", "108", "66",
	"ETSI", "5G", "20M", "HT", "1T", "108", "56",
	"MKK", "5G", "20M", "HT", "1T", "108", "66",
	"IC", "5G", "20M", "HT", "1T", "108", "62",
	"KCC", "5G", "20M", "HT", "1T", "108", "62",
	"ACMA", "5G", "20M", "HT", "1T", "108", "60",
	"CHILE", "5G", "20M", "HT", "1T", "108", "56",
	"UKRAINE", "5G", "20M", "HT", "1T", "108", "50",
	"MEXICO", "5G", "20M", "HT", "1T", "108", "62",
	"CN", "5G", "20M", "HT", "1T", "108", "127",
	"FCC", "5G", "20M", "HT", "1T", "112", "66",
	"ETSI", "5G", "20M", "HT", "1T", "112", "56",
	"MKK", "5G", "20M", "HT", "1T", "112", "66",
	"IC", "5G", "20M", "HT", "1T", "112", "62",
	"KCC", "5G", "20M", "HT", "1T", "112", "62",
	"ACMA", "5G", "20M", "HT", "1T", "112", "60",
	"CHILE", "5G", "20M", "HT", "1T", "112", "56",
	"UKRAINE", "5G", "20M", "HT", "1T", "112", "50",
	"MEXICO", "5G", "20M", "HT", "1T", "112", "62",
	"CN", "5G", "20M", "HT", "1T", "112", "127",
	"FCC", "5G", "20M", "HT", "1T", "116", "66",
	"ETSI", "5G", "20M", "HT", "1T", "116", "56",
	"MKK", "5G", "20M", "HT", "1T", "116", "66",
	"IC", "5G", "20M", "HT", "1T", "116", "62",
	"KCC", "5G", "20M", "HT", "1T", "116", "62",
	"ACMA", "5G", "20M", "HT", "1T", "116", "60",
	"CHILE", "5G", "20M", "HT", "1T", "116", "56",
	"UKRAINE", "5G", "20M", "HT", "1T", "116", "50",
	"MEXICO", "5G", "20M", "HT", "1T", "116", "62",
	"CN", "5G", "20M", "HT", "1T", "116", "127",
	"FCC", "5G", "20M", "HT", "1T", "120", "66",
	"ETSI", "5G", "20M", "HT", "1T", "120", "56",
	"MKK", "5G", "20M", "HT", "1T", "120", "66",
	"IC", "5G", "20M", "HT", "1T", "120", "127",
	"KCC", "5G", "20M", "HT", "1T", "120", "62",
	"ACMA", "5G", "20M", "HT", "1T", "120", "127",
	"CHILE", "5G", "20M", "HT", "1T", "120", "56",
	"UKRAINE", "5G", "20M", "HT", "1T", "120", "50",
	"MEXICO", "5G", "20M", "HT", "1T", "120", "62",
	"CN", "5G", "20M", "HT", "1T", "120", "127",
	"FCC", "5G", "20M", "HT", "1T", "124", "66",
	"ETSI", "5G", "20M", "HT", "1T", "124", "56",
	"MKK", "5G", "20M", "HT", "1T", "124", "66",
	"IC", "5G", "20M", "HT", "1T", "124", "127",
	"KCC", "5G", "20M", "HT", "1T", "124", "62",
	"ACMA", "5G", "20M", "HT", "1T", "124", "127",
	"CHILE", "5G", "20M", "HT", "1T", "124", "56",
	"UKRAINE", "5G", "20M", "HT", "1T", "124", "50",
	"MEXICO", "5G", "20M", "HT", "1T", "124", "62",
	"CN", "5G", "20M", "HT", "1T", "124", "127",
	"FCC", "5G", "20M", "HT", "1T", "128", "66",
	"ETSI", "5G", "20M", "HT", "1T", "128", "56",
	"MKK", "5G", "20M", "HT", "1T", "128", "66",
	"IC", "5G", "20M", "HT", "1T", "128", "127",
	"KCC", "5G", "20M", "HT", "1T", "128", "-128",
	"ACMA", "5G", "20M", "HT", "1T", "128", "127",
	"CHILE", "5G", "20M", "HT", "1T", "128", "56",
	"UKRAINE", "5G", "20M", "HT", "1T", "128", "50",
	"MEXICO", "5G", "20M", "HT", "1T", "128", "62",
	"CN", "5G", "20M", "HT", "1T", "128", "127",
	"FCC", "5G", "20M", "HT", "1T", "132", "66",
	"ETSI", "5G", "20M", "HT", "1T", "132", "56",
	"MKK", "5G", "20M", "HT", "1T", "132", "66",
	"IC", "5G", "20M", "HT", "1T", "132", "62",
	"KCC", "5G", "20M", "HT", "1T", "132", "-128",
	"ACMA", "5G", "20M", "HT", "1T", "132", "60",
	"CHILE", "5G", "20M", "HT", "1T", "132", "56",
	"UKRAINE", "5G", "20M", "HT", "1T", "132", "50",
	"MEXICO", "5G", "20M", "HT", "1T", "132", "62",
	"CN", "5G", "20M", "HT", "1T", "132", "127",
	"FCC", "5G", "20M", "HT", "1T", "136", "66",
	"ETSI", "5G", "20M", "HT", "1T", "136", "56",
	"MKK", "5G", "20M", "HT", "1T", "136", "66",
	"IC", "5G", "20M", "HT", "1T", "136", "62",
	"KCC", "5G", "20M", "HT", "1T", "136", "-128",
	"ACMA", "5G", "20M", "HT", "1T", "136", "60",
	"CHILE", "5G", "20M", "HT", "1T", "136", "56",
	"UKRAINE", "5G", "20M", "HT", "1T", "136", "127",
	"MEXICO", "5G", "20M", "HT", "1T", "136", "62",
	"CN", "5G", "20M", "HT", "1T", "136", "127",
	"FCC", "5G", "20M", "HT", "1T", "140", "58",
	"ETSI", "5G", "20M", "HT", "1T", "140", "56",
	"MKK", "5G", "20M", "HT", "1T", "140", "66",
	"IC", "5G", "20M", "HT", "1T", "140", "54",
	"KCC", "5G", "20M", "HT", "1T", "140", "-128",
	"ACMA", "5G", "20M", "HT", "1T", "140", "60",
	"CHILE", "5G", "20M", "HT", "1T", "140", "56",
	"UKRAINE", "5G", "20M", "HT", "1T", "140", "127",
	"MEXICO", "5G", "20M", "HT", "1T", "140", "54",
	"CN", "5G", "20M", "HT", "1T", "140", "127",
	"FCC", "5G", "20M", "HT", "1T", "144", "54",
	"ETSI", "5G", "20M", "HT", "1T", "144", "127",
	"MKK", "5G", "20M", "HT", "1T", "144", "127",
	"IC", "5G", "20M", "HT", "1T", "144", "50",
	"KCC", "5G", "20M", "HT", "1T", "144", "-128",
	"ACMA", "5G", "20M", "HT", "1T", "144", "127",
	"CHILE", "5G", "20M", "HT", "1T", "144", "56",
	"UKRAINE", "5G", "20M", "HT", "1T", "144", "127",
	"MEXICO", "5G", "20M", "HT", "1T", "144", "54",
	"CN", "5G", "20M", "HT", "1T", "144", "127",
	"FCC", "5G", "20M", "HT", "1T", "149", "64",
	"ETSI", "5G", "20M", "HT", "1T", "149", "-128",
	"MKK", "5G", "20M", "HT", "1T", "149", "127",
	"IC", "5G", "20M", "HT", "1T", "149", "62",
	"KCC", "5G", "20M", "HT", "1T", "149", "62",
	"ACMA", "5G", "20M", "HT", "1T", "149", "62",
	"CHILE", "5G", "20M", "HT", "1T", "149", "56",
	"UKRAINE", "5G", "20M", "HT", "1T", "149", "50",
	"MEXICO", "5G", "20M", "HT", "1T", "149", "62",
	"CN", "5G", "20M", "HT", "1T", "149", "-128",
	"FCC", "5G", "20M", "HT", "1T", "153", "64",
	"ETSI", "5G", "20M", "HT", "1T", "153", "-128",
	"MKK", "5G", "20M", "HT", "1T", "153", "127",
	"IC", "5G", "20M", "HT", "1T", "153", "62",
	"KCC", "5G", "20M", "HT", "1T", "153", "62",
	"ACMA", "5G", "20M", "HT", "1T", "153", "62",
	"CHILE", "5G", "20M", "HT", "1T", "153", "56",
	"UKRAINE", "5G", "20M", "HT", "1T", "153", "50",
	"MEXICO", "5G", "20M", "HT", "1T", "153", "62",
	"CN", "5G", "20M", "HT", "1T", "153", "-128",
	"FCC", "5G", "20M", "HT", "1T", "157", "64",
	"ETSI", "5G", "20M", "HT", "1T", "157", "-128",
	"MKK", "5G", "20M", "HT", "1T", "157", "127",
	"IC", "5G", "20M", "HT", "1T", "157", "62",
	"KCC", "5G", "20M", "HT", "1T", "157", "62",
	"ACMA", "5G", "20M", "HT", "1T", "157", "62",
	"CHILE", "5G", "20M", "HT", "1T", "157", "56",
	"UKRAINE", "5G", "20M", "HT", "1T", "157", "50",
	"MEXICO", "5G", "20M", "HT", "1T", "157", "62",
	"CN", "5G", "20M", "HT", "1T", "157", "-128",
	"FCC", "5G", "20M", "HT", "1T", "161", "64",
	"ETSI", "5G", "20M", "HT", "1T", "161", "-128",
	"MKK", "5G", "20M", "HT", "1T", "161", "127",
	"IC", "5G", "20M", "HT", "1T", "161", "62",
	"KCC", "5G", "20M", "HT", "1T", "161", "58",
	"ACMA", "5G", "20M", "HT", "1T", "161", "62",
	"CHILE", "5G", "20M", "HT", "1T", "161", "56",
	"UKRAINE", "5G", "20M", "HT", "1T", "161", "50",
	"MEXICO", "5G", "20M", "HT", "1T", "161", "62",
	"CN", "5G", "20M", "HT", "1T", "161", "-128",
	"FCC", "5G", "20M", "HT", "1T", "165", "64",
	"ETSI", "5G", "20M", "HT", "1T", "165", "-128",
	"MKK", "5G", "20M", "HT", "1T", "165", "127",
	"IC", "5G", "20M", "HT", "1T", "165", "62",
	"KCC", "5G", "20M", "HT", "1T", "165", "-128",
	"ACMA", "5G", "20M", "HT", "1T", "165", "62",
	"CHILE", "5G", "20M", "HT", "1T", "165", "56",
	"UKRAINE", "5G", "20M", "HT", "1T", "165", "50",
	"MEXICO", "5G", "20M", "HT", "1T", "165", "56",
	"CN", "5G", "20M", "HT", "1T", "165", "-128",
	"FCC", "5G", "40M", "HT", "1T", "38", "52",
	"ETSI", "5G", "40M", "HT", "1T", "38", "56",
	"MKK", "5G", "40M", "HT", "1T", "38", "64",
	"IC", "5G", "40M", "HT", "1T", "38", "40",
	"KCC", "5G", "40M", "HT", "1T", "38", "48",
	"ACMA", "5G", "40M", "HT", "1T", "38", "60",
	"CHILE", "5G", "40M", "HT", "1T", "38", "40",
	"UKRAINE", "5G", "40M", "HT", "1T", "38", "50",
	"MEXICO", "5G", "40M", "HT", "1T", "38", "40",
	"CN", "5G", "40M", "HT", "1T", "38", "56",
	"FCC", "5G", "40M", "HT", "1T", "46", "52",
	"ETSI", "5G", "40M", "HT", "1T", "46", "56",
	"MKK", "5G", "40M", "HT", "1T", "46", "64",
	"IC", "5G", "40M", "HT", "1T", "46", "60",
	"KCC", "5G", "40M", "HT", "1T", "46", "52",
	"ACMA", "5G", "40M", "HT", "1T", "46", "60",
	"CHILE", "5G", "40M", "HT", "1T", "46", "56",
	"UKRAINE", "5G", "40M", "HT", "1T", "46", "50",
	"MEXICO", "5G", "40M", "HT", "1T", "46", "58",
	"CN", "5G", "40M", "HT", "1T", "46", "56",
	"FCC", "5G", "40M", "HT", "1T", "54", "52",
	"ETSI", "5G", "40M", "HT", "1T", "54", "56",
	"MKK", "5G", "40M", "HT", "1T", "54", "64",
	"IC", "5G", "40M", "HT", "1T", "54", "60",
	"KCC", "5G", "40M", "HT", "1T", "54", "40",
	"ACMA", "5G", "40M", "HT", "1T", "54", "60",
	"CHILE", "5G", "40M", "HT", "1T", "54", "56",
	"UKRAINE", "5G", "40M", "HT", "1T", "54", "50",
	"MEXICO", "5G", "40M", "HT", "1T", "54", "60",
	"CN", "5G", "40M", "HT", "1T", "54", "56",
	"FCC", "5G", "40M", "HT", "1T", "62", "52",
	"ETSI", "5G", "40M", "HT", "1T", "62", "56",
	"MKK", "5G", "40M", "HT", "1T", "62", "64",
	"IC", "5G", "40M", "HT", "1T", "62", "42",
	"KCC", "5G", "40M", "HT", "1T", "62", "58",
	"ACMA", "5G", "40M", "HT", "1T", "62", "60",
	"CHILE", "5G", "40M", "HT", "1T", "62", "42",
	"UKRAINE", "5G", "40M", "HT", "1T", "62", "50",
	"MEXICO", "5G", "40M", "HT", "1T", "62", "42",
	"CN", "5G", "40M", "HT", "1T", "62", "56",
	"FCC", "5G", "40M", "HT", "1T", "102", "52",
	"ETSI", "5G", "40M", "HT", "1T", "102", "56",
	"MKK", "5G", "40M", "HT", "1T", "102", "64",
	"IC", "5G", "40M", "HT", "1T", "102", "38",
	"KCC", "5G", "40M", "HT", "1T", "102", "58",
	"ACMA", "5G", "40M", "HT", "1T", "102", "60",
	"CHILE", "5G", "40M", "HT", "1T", "102", "56",
	"UKRAINE", "5G", "40M", "HT", "1T", "102", "50",
	"MEXICO", "5G", "40M", "HT", "1T", "102", "38",
	"CN", "5G", "40M", "HT", "1T", "102", "127",
	"FCC", "5G", "40M", "HT", "1T", "110", "60",
	"ETSI", "5G", "40M", "HT", "1T", "110", "56",
	"MKK", "5G", "40M", "HT", "1T", "110", "64",
	"IC", "5G", "40M", "HT", "1T", "110", "60",
	"KCC", "5G", "40M", "HT", "1T", "110", "60",
	"ACMA", "5G", "40M", "HT", "1T", "110", "60",
	"CHILE", "5G", "40M", "HT", "1T", "110", "56",
	"UKRAINE", "5G", "40M", "HT", "1T", "110", "50",
	"MEXICO", "5G", "40M", "HT", "1T", "110", "60",
	"CN", "5G", "40M", "HT", "1T", "110", "127",
	"FCC", "5G", "40M", "HT", "1T", "118", "60",
	"ETSI", "5G", "40M", "HT", "1T", "118", "56",
	"MKK", "5G", "40M", "HT", "1T", "118", "64",
	"IC", "5G", "40M", "HT", "1T", "118", "127",
	"KCC", "5G", "40M", "HT", "1T", "118", "60",
	"ACMA", "5G", "40M", "HT", "1T", "118", "127",
	"CHILE", "5G", "40M", "HT", "1T", "118", "56",
	"UKRAINE", "5G", "40M", "HT", "1T", "118", "50",
	"MEXICO", "5G", "40M", "HT", "1T", "118", "60",
	"CN", "5G", "40M", "HT", "1T", "118", "127",
	"FCC", "5G", "40M", "HT", "1T", "126", "60",
	"ETSI", "5G", "40M", "HT", "1T", "126", "56",
	"MKK", "5G", "40M", "HT", "1T", "126", "64",
	"IC", "5G", "40M", "HT", "1T", "126", "127",
	"KCC", "5G", "40M", "HT", "1T", "126", "-128",
	"ACMA", "5G", "40M", "HT", "1T", "126", "127",
	"CHILE", "5G", "40M", "HT", "1T", "126", "56",
	"UKRAINE", "5G", "40M", "HT", "1T", "126", "50",
	"MEXICO", "5G", "40M", "HT", "1T", "126", "60",
	"CN", "5G", "40M", "HT", "1T", "126", "127",
	"FCC", "5G", "40M", "HT", "1T", "134", "60",
	"ETSI", "5G", "40M", "HT", "1T", "134", "56",
	"MKK", "5G", "40M", "HT", "1T", "134", "64",
	"IC", "5G", "40M", "HT", "1T", "134", "60",
	"KCC", "5G", "40M", "HT", "1T", "134", "-128",
	"ACMA", "5G", "40M", "HT", "1T", "134", "60",
	"CHILE", "5G", "40M", "HT", "1T", "134", "56",
	"UKRAINE", "5G", "40M", "HT", "1T", "134", "127",
	"MEXICO", "5G", "40M", "HT", "1T", "134", "60",
	"CN", "5G", "40M", "HT", "1T", "134", "127",
	"FCC", "5G", "40M", "HT", "1T", "142", "56",
	"ETSI", "5G", "40M", "HT", "1T", "142", "127",
	"MKK", "5G", "40M", "HT", "1T", "142", "127",
	"IC", "5G", "40M", "HT", "1T", "142", "54",
	"KCC", "5G", "40M", "HT", "1T", "142", "-128",
	"ACMA", "5G", "40M", "HT", "1T", "142", "127",
	"CHILE", "5G", "40M", "HT", "1T", "142", "56",
	"UKRAINE", "5G", "40M", "HT", "1T", "142", "127",
	"MEXICO", "5G", "40M", "HT", "1T", "142", "54",
	"CN", "5G", "40M", "HT", "1T", "142", "127",
	"FCC", "5G", "40M", "HT", "1T", "151", "60",
	"ETSI", "5G", "40M", "HT", "1T", "151", "-128",
	"MKK", "5G", "40M", "HT", "1T", "151", "127",
	"IC", "5G", "40M", "HT", "1T", "151", "60",
	"KCC", "5G", "40M", "HT", "1T", "151", "50",
	"ACMA", "5G", "40M", "HT", "1T", "151", "60",
	"CHILE", "5G", "40M", "HT", "1T", "151", "56",
	"UKRAINE", "5G", "40M", "HT", "1T", "151", "50",
	"MEXICO", "5G", "40M", "HT", "1T", "151", "60",
	"CN", "5G", "40M", "HT", "1T", "151", "-128",
	"FCC", "5G", "40M", "HT", "1T", "159", "60",
	"ETSI", "5G", "40M", "HT", "1T", "159", "-128",
	"MKK", "5G", "40M", "HT", "1T", "159", "127",
	"IC", "5G", "40M", "HT", "1T", "159", "60",
	"KCC", "5G", "40M", "HT", "1T", "159", "48",
	"ACMA", "5G", "40M", "HT", "1T", "159", "60",
	"CHILE", "5G", "40M", "HT", "1T", "159", "56",
	"UKRAINE", "5G", "40M", "HT", "1T", "159", "50",
	"MEXICO", "5G", "40M", "HT", "1T", "159", "60",
	"CN", "5G", "40M", "HT", "1T", "159", "-128"
};
#endif

void
odm_read_and_config_mp_8733b_txpwr_lmt(struct dm_struct *dm)
{
#ifdef CONFIG_8733B

	u32	i = 0;
#if (DM_ODM_SUPPORT_TYPE == ODM_IOT)
	u32	array_len =
			sizeof(array_mp_8733b_txpwr_lmt) / sizeof(u8);
	u8	*array = (u8 *)array_mp_8733b_txpwr_lmt;
#else
	u32	array_len =
			sizeof(array_mp_8733b_txpwr_lmt) / sizeof(u8 *);
	u8	**array = (u8 **)array_mp_8733b_txpwr_lmt;
#endif

#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	void	*adapter = dm->adapter;
	HAL_DATA_TYPE	*hal_data = GET_HAL_DATA(((PADAPTER)adapter));

	odm_memory_set(dm, hal_data->BufOfLinesPwrLmt, 0,
		       MAX_LINES_HWCONFIG_TXT *
		       MAX_BYTES_LINE_HWCONFIG_TXT);
	hal_data->nLinesReadPwrLmt = array_len / 7;
#endif

	PHYDM_DBG(dm, ODM_COMP_INIT, "===> %s\n", __func__);

	for (i = 0; i < array_len; i += 7) {
#if (DM_ODM_SUPPORT_TYPE == ODM_IOT)
		u8	regulation = array[i];
		u8	band = array[i + 1];
		u8	bandwidth = array[i + 2];
		u8	rate = array[i + 3];
		u8	rf_path = array[i + 4];
		u8	chnl = array[i + 5];
		u8	val = array[i + 6];
#else
		u8	*regulation = array[i];
		u8	*band = array[i + 1];
		u8	*bandwidth = array[i + 2];
		u8	*rate = array[i + 3];
		u8	*rf_path = array[i + 4];
		u8	*chnl = array[i + 5];
		u8	*val = array[i + 6];
#endif

		odm_config_bb_txpwr_lmt_8733b(dm, regulation, band, bandwidth,
					      rate, rf_path, chnl, val);
#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
		rsprintf((char *)hal_data->BufOfLinesPwrLmt[i / 7], 100, "\"%s\", \"%s\", \"%s\", \"%s\", \"%s\", \"%s\", \"%s\",",
			 regulation, band, bandwidth, rate, rf_path, chnl, val);
#endif
	}

#endif
}

/******************************************************************************
 *                           txxtaltrack.TXT
 ******************************************************************************/

const s8 delta_swing_xtal_mp_n_txxtaltrack_8733b[]    = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const s8 delta_swing_xtal_mp_p_txxtaltrack_8733b[]    = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -10, -20, -20,
	 -20, -30, -30, -30, -40, -40, -40, -40, -40, -40, -40};

void
odm_read_and_config_mp_8733b_txxtaltrack(struct dm_struct *dm)
{
	struct dm_rf_calibration_struct	*cali_info = &dm->rf_calibrate_info;

	PHYDM_DBG(dm, ODM_COMP_INIT, "===> ODM_ReadAndConfig_MP_mp_8733b\n");

	odm_move_memory(dm, cali_info->delta_swing_table_xtal_p,
			(void *)delta_swing_xtal_mp_p_txxtaltrack_8733b,
			DELTA_SWINGIDX_SIZE);
	odm_move_memory(dm, cali_info->delta_swing_table_xtal_n,
			(void *)delta_swing_xtal_mp_n_txxtaltrack_8733b,
			DELTA_SWINGIDX_SIZE);
}

#endif /* end of HWIMG_SUPPORT*/

