/*
* Sunxi SD/MMC host driver
*
* Copyright (C) 2015 AllWinnertech Ltd.
* Author: lixiang <lixiang@allwinnertech>
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed "as is" WITHOUT ANY WARRANTY of any
* kind, whether express or implied; without even the implied warranty
* of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*/

#ifdef CONFIG_ARCH_SUN8IW10P1

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/err.h>

#include <linux/clk.h>
#include <linux/clk/sunxi.h>

#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/reset.h>

#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/regulator/consumer.h>

#include <linux/mmc/host.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/core.h>
#include <linux/mmc/card.h>
#include <linux/mmc/slot-gpio.h>

#include "sunxi-smhc.h"
#include "sunxi-mmc-v5px.h"
#include "sunxi-mmc-debug.h"
#include "sunxi-mmc-export.h"

#ifdef CONFIG_REGULATOR
/**
 * mmc_regulator_get_ocrmask - return mask of supported voltages
 * @supply: regulator to use
 *
 * This returns either a negative errno, or a mask of voltages that
 * can be provided to MMC/SD/SDIO devices using the specified voltage
 * regulator.  This would normally be called before registering the
 * MMC host adapter.
 */
static int mmc_regulator_get_ocrmask(struct regulator *supply)
{
	int			result = 0;
	int			count;
	int			i;
	int			vdd_uV;
	int			vdd_mV;

	count = regulator_count_voltages(supply);
	if (count < 0)
		return count;

	for (i = 0; i < count; i++) {
		vdd_uV = regulator_list_voltage(supply, i);
		if (vdd_uV <= 0)
			continue;

		vdd_mV = vdd_uV / 1000;
		result |= mmc_vddrange_to_ocrmask(vdd_mV, vdd_mV);
	}

	if (!result) {
		vdd_uV = regulator_get_voltage(supply);
		if (vdd_uV <= 0)
			return vdd_uV;

		vdd_mV = vdd_uV / 1000;
		result = mmc_vddrange_to_ocrmask(vdd_mV, vdd_mV);
	}

	return result;
}

/**
 * mmc_regulator_set_ocr - set regulator to match host->ios voltage
 * @mmc: the host to regulate
 * @supply: regulator to use
 * @vdd_bit: zero for power off, else a bit number (host->ios.vdd)
 *
 * Returns zero on success, else negative errno.
 *
 * MMC host drivers may use this to enable or disable a regulator using
 * a particular supply voltage.  This would normally be called from the
 * set_ios() method.
 */
int sunxi_mmc_regulator_set_ocr(struct mmc_host *mmc,
			struct regulator *supply,
			unsigned short vdd_bit)
{
	int			result = 0;
	/*int			min_uV, max_uV;*/

	if (vdd_bit) {
		/*sunxi platform avoid set vcc voltage*/
		/*mmc_ocrbitnum_to_vdd(vdd_bit, &min_uV, &max_uV);*/

		/*result = regulator_set_voltage(supply, min_uV, max_uV);*/
		if (result == 0 && !mmc->regulator_enabled) {
			result = regulator_enable(supply);
			if (!result)
				mmc->regulator_enabled = true;
		}
	} else if (mmc->regulator_enabled) {
		result = regulator_disable(supply);
		if (result == 0)
			mmc->regulator_enabled = false;
	}

	if (result)
		dev_err(mmc_dev(mmc),
			"could not set regulator OCR (%d)\n", result);
	return result;
}
#else
static inline int mmc_regulator_get_ocrmask(struct regulator *supply)
{
	return 0;
}
static inline int sunxi_mmc_regulator_set_ocr(struct mmc_host *mmc,
			struct regulator *supply,
	i		unsigned short vdd_bit)
{
	return 0;
}
#endif

static int sunxi_wait_bit_clr(struct sunxi_mmc_host *smc_host, u32 reg_add,
			      u32 bit_map, s8 *reg_add_str, s8 *bit_map_str,
			      u32 timeout_ms)
{
	unsigned long expire = 0;
	u32 tmp = 0;
	s32 ret = 0;

	expire = jiffies + msecs_to_jiffies(timeout_ms);	/* 1ms timeout*/
	do {
		/*SMC_DBG(smc_host,"Wait reg %s(%x),bit %s(%x)\n",
		 *reg_add_str,reg_add,bit_map_str,bit_map);
		 */
		tmp = smhc_readl(smc_host, reg_add);
	} while (time_before(jiffies, expire) && (tmp & bit_map));

	tmp = smhc_readl(smc_host, reg_add);
	if (tmp & bit_map) {
		dev_err(mmc_dev(smc_host->mmc),
			"Wait reg %s(%x),bit %s(%x) %d ms timeout\n",
			reg_add_str, reg_add, bit_map_str, bit_map, timeout_ms);
		ret = 1;
	} else {
		ret = 0;
	}

	return ret;
}

static int sunxi_mmc_init_host(struct mmc_host *mmc)
{
	u32 tmp;
	struct sunxi_mmc_host *smc_host = mmc_priv(mmc);

	dev_dbg(mmc_dev(mmc), "MMC Driver init host p%d\n",
		smc_host->phy_index);

	/*reset control*/
	tmp = smhc_readl(smc_host, SMHC_RST_CLK_CTRL);
	smhc_writel(smc_host, SMHC_RST_CLK_CTRL, tmp | ResetAll);
	/*wait reset done*/
	if (sunxi_wait_bit_clr(smc_host,
			       SMHC_RST_CLK_CTRL, ResetAll,
			       "SMHC_RST_CLK_CTRL", "ResetAll", 1)) {
		return -1;
	}

	smhc_writel(smc_host, SMHC_INT_STA_EN, 0xffffffff);
	smhc_writel(smc_host, SMHC_INT_STA, 0xffffffff);
	dev_dbg(mmc_dev(mmc), "int sta %x\n",
		smhc_readl(smc_host, SMHC_INT_STA));

	/*Set Data & Response Timeout Value*/
#define  SMC_DATA_TIMEOUT	  0xffffffU
#define  SMC_RESP_TIMEOUT	  0xff
	smhc_writel(smc_host, SMHC_TO_CTRL2,
		    (SMC_DATA_TIMEOUT << 8) | SMC_RESP_TIMEOUT);

	/*stop clock at block gap, bit8*/
	tmp = smhc_readl(smc_host, SMHC_CTRL3);
	smhc_writel(smc_host, SMHC_CTRL3, tmp | StopReadClkAtBlkGap);

	/*disable dat3 int*/
	if (smc_host->dat3_imask) {
		/*enable data3 detect*/
		smhc_clr_bit(smc_host, SMHC_CTRL3, SWDebounceMode);
		smhc_set_bit(smc_host, SMHC_CTRL3, CdDat3En);

		/*enable data3 detect int*/
		smhc_set_bit(smc_host, SMHC_INT_STA_EN,
			     CardRemoveInt | CardInsertInt);
		smhc_set_bit(smc_host, SMHC_INT_SIG_EN,
			     CardRemoveInt | CardInsertInt);
	} else {
		smhc_clr_bit(smc_host, SMHC_INT_STA_EN,
			     CardRemoveInt | CardInsertInt);
		smhc_clr_bit(smc_host, SMHC_INT_SIG_EN,
			     CardRemoveInt | CardInsertInt);
	}

/*
*dumphex32(smc_host, "mmc",
			IOMEM(IO_ADDRESS(SMHC_BASE_ADDR)), 0x100);
*/
/*dumphex32(smc_host, "gpio", IO_ADDRESS(SUNXI_PIO_BASE), 0x120);
*/
	return 0;

}

static void sunxi_mmc_init_idma_des(struct sunxi_mmc_host *smc_host,
				    struct mmc_data *data)
{
	struct sdhc_idma_des *pdes = (struct sdhc_idma_des *)smc_host->sg_cpu;
/*    struct sdhc_idma_des* pdes_pa =
	*(struct sdhc_idma_des*)smc_host->sg_dma;
	*/
	struct scatterlist *sg = NULL;
	u32 i = 0;

	for_each_sg(data->sg, sg, data->sg_len, i) {
		memset((void *)(&pdes[i]), 0, sizeof(struct sdhc_idma_des));
		pdes[i].valid = 1;
		pdes[i].end = 0;
		pdes[i].int_en = 0;
		pdes[i].act = ACT_TRANS;
		pdes[i].length = sg->length;
		pdes[i].addr = sg_dma_address(sg);
	}

	pdes[i - 1].end = 1;
	pdes[i - 1].int_en = 1;
/******************************************************/
	smp_wmb();
	dev_dbg(mmc_dev(smc_host->mmc), "sg len %d end des index %d\n",
		data->sg_len, i - 1);

	for_each_sg(data->sg, sg, data->sg_len, i) {
		dev_dbg(mmc_dev(smc_host->mmc),
			"sg %d, des[%d](%08x): [0] = %08x, [1] = %08x\n", i, i,
			(u32) &pdes[i], (u32) ((u32 *) &pdes[i])[0],
			(u32) ((u32 *) &pdes[i])[1]);
	}
}

static enum dma_data_direction sunxi_mmc_get_dma_dir(struct mmc_data *data)
{
	if (data->flags & MMC_DATA_WRITE)
		return DMA_TO_DEVICE;
	else
		return DMA_FROM_DEVICE;
}

static int sunxi_mmc_map_dma(struct sunxi_mmc_host *host, struct mmc_data *data)
{
	u32 i, dma_len;
	struct scatterlist *sg;

	dma_len = dma_map_sg(mmc_dev(host->mmc), data->sg, data->sg_len,
			     sunxi_mmc_get_dma_dir(data));
	if (dma_len == 0) {
		dev_err(mmc_dev(host->mmc), "dma_map_sg failed\n");
		return -ENOMEM;
	}

	WARN_ON(dma_len != data->sg_len);
	WARN_ON(dma_len > host->mmc->max_segs);

	for_each_sg(data->sg, sg, data->sg_len, i) {
		if (sg->offset & 3 || sg->length & 3) {
			dev_err(mmc_dev(host->mmc),
				"unaligned scatterlist: os %x length %d\n",
				sg->offset, sg->length);
			return -EINVAL;
		}
	}

	return 0;
}

static void sunxi_mmc_start_dma(struct sunxi_mmc_host *smc_host,
				struct mmc_data *data)
{
	u32 tmp = 0;

	sunxi_mmc_init_idma_des(smc_host, data);
	/*dma access*/
	tmp = smhc_readl(smc_host, SMHC_CTRL3);
	tmp &= ~CPUAcessBuffEn;
	smhc_writel(smc_host, SMHC_CTRL3, tmp);

	/*dma select*/
	tmp = smhc_readl(smc_host, SMHC_CTRL1);
	tmp &= ~DmaSel;
	tmp |= Dma32BitSel;
	smhc_writel(smc_host, SMHC_CTRL1, tmp);
	smhc_writel(smc_host, SMHC_ADMA_ADDR, smc_host->sg_dma);

}

static void sunxi_mmc_send_manual_stop(struct sunxi_mmc_host *host,
				       struct mmc_request *req)
{
/*
*	u32 arg, cmd_val, ri;
*	unsigned long expire = jiffies + msecs_to_jiffies(1000);
*
*	cmd_val = SDXC_START | SDXC_RESP_EXPECT |
*		  SDXC_STOP_ABORT_CMD | SDXC_CHECK_RESPONSE_CRC;
*
*	if (req->cmd->opcode == SD_IO_RW_EXTENDED) {
*		cmd_val |= SD_IO_RW_DIRECT;
*		arg = (1 << 31) | (0 << 28) | (SDIO_CCCR_ABORT << 9) |
*		      ((req->cmd->arg >> 28) & 0x7);
*	} else {
*		cmd_val |= MMC_STOP_TRANSMISSION;
*		arg = 0;
*	}
*
*	mmc_writel(host, REG_CARG, arg);
*	mmc_writel(host, REG_CMDR, cmd_val);
*
*	do {
*		ri = mmc_readl(host, REG_RINTR);
*	} while (!(ri & (SDXC_COMMAND_DONE | SDXC_INTERRUPT_ERROR_BIT)) &&
*		 time_before(jiffies, expire));
*
*	if (!(ri & SDXC_COMMAND_DONE) || (ri & SDXC_INTERRUPT_ERROR_BIT)) {
*		dev_err(mmc_dev(host->mmc), "send stop command failed\n");
*		if (req->stop)
*			req->stop->resp[0] = -ETIMEDOUT;
*	} else {
*		if (req->stop)
*			req->stop->resp[0] = mmc_readl(host, REG_RESP0);
*	}
*
*	mmc_writel(host, REG_RINTR, 0xffff);
*/
	dev_info(mmc_dev(host->mmc), "no imple %s %d\n", __func__,
		 __LINE__);
}

static void sunxi_mmc_dump_errinfo(struct sunxi_mmc_host *host)
{

	/* For some cmds timeout is normal with sd/mmc cards */
/*
*      if ((host->int_sum & SDXC_INTERRUPT_ERROR_BIT) ==
*		SDXC_RESP_TIMEOUT && (cmd->opcode == SD_IO_SEND_OP_COND ||
*				cmd->opcode == SD_IO_RW_DIRECT))
*		 return;
*/
	dev_err(mmc_dev(host->mmc),
		"smc p%d err, cmd %d, %s%s%s%s%s%s%s%s%s%s !!\n",
		host->phy_index, host->mrq->cmd ? host->mrq->cmd->opcode : -1,
		host->int_sum & DSFOInt ? "SBE " : "",
		host->int_sum & DmaErrInt ? "DME " : "",
		host->int_sum & AcmdErrInt ? "ATE " : "",
		host->int_sum & DatEndBitErrInt ? "DEBE " : "",
		host->int_sum & DatCRCErrInt ? "DCE " : "",
		host->int_sum & DatTimeoutErrInt ? "DTO " : "",
		host->int_sum & CmdIdxErrInt ? "RIE " : "",
		host->int_sum & CmdEndBitErrInt ? "REBE " : "",
		host->int_sum & CmdCRCErrInt ? "RCE " : "",
		host->int_sum & CmdTimeoutErrInt ? "RTO " : "");

	/*sunxi_mmc_dumphex32(host,"sunxi mmc",host->reg_base,0x180);*/
	/*sunxi_mmc_dump_des(host,host->sg_cpu,PAGE_SIZE);*/
}

/* Called in interrupt context! */
static irqreturn_t sunxi_mmc_finalize_request(struct sunxi_mmc_host *smc_host)
{
	struct mmc_request *mrq = smc_host->mrq;
	struct mmc_data *data = mrq->data;
	u32 tmp = 0;

/*
*	mmc_writel(host, REG_IMASK, host->sdio_imask | host->dat3_imask);
*	mmc_writel(host, REG_IDIE, 0);
*	clear  int signal enable
*	tmp = smhc_readl(smc_host,SMHC_INT_SIG_EN);
*	tmp &= (CardInt|CardRemoveInt|CardInsertInt);
*	smhc_writel(smc_host,SMHC_INT_SIG_EN,tmp);
*/
	if (smc_host->int_sum & ErrIntBit) {
		sunxi_mmc_dump_errinfo(smc_host);
		mrq->cmd->error = -ETIMEDOUT;

		if (data) {
			data->error = -ETIMEDOUT;
			smc_host->manual_stop_mrq = mrq;
		}

		if (mrq->stop)
			mrq->stop->error = -ETIMEDOUT;
	} else {
		if (mrq->cmd->flags & MMC_RSP_136) {
			mrq->cmd->resp[0] =
			    (smhc_readl(smc_host, SMHC_RESP3) & 0xffffff) << 8;
			mrq->cmd->resp[0] |=
			    (smhc_readl(smc_host, SMHC_RESP2) >> 24) & 0xff;
			mrq->cmd->resp[1] =
			    (smhc_readl(smc_host, SMHC_RESP2) & 0xffffff) << 8;
			mrq->cmd->resp[1] |=
			    (smhc_readl(smc_host, SMHC_RESP1) >> 24) & 0xff;
			mrq->cmd->resp[2] =
			    (smhc_readl(smc_host, SMHC_RESP1) & 0xffffff) << 8;
			mrq->cmd->resp[2] |=
			    (smhc_readl(smc_host, SMHC_RESP0) >> 24) & 0xff;
			mrq->cmd->resp[3] =
			    (smhc_readl(smc_host, SMHC_RESP0) & 0xffffff) << 8;
		} else {
			mrq->cmd->resp[0] = smhc_readl(smc_host, SMHC_RESP0);
		}

		if (data)
			data->bytes_xfered = data->blocks * data->blksz;
/*
*		To avoid that "wait busy" and "maual stop" occur
*		at the same time,
*		We wait busy only on not error occur.
*		if(mrq->cmd->flags & MMC_RSP_BUSY){
*		      host->mrq_busy = host->mrq;
*		}
*/
	}

	if (data) {
		struct mmc_data *data = mrq->data;
		/*recover to cpu access*/
		/*
*		   tmp = smhc_readl(smc_host, SMHC_CTRL3);
*		   tmp |= CPUAcessBuffEn;
*		   smhc_writel(smc_host, SMHC_CTRL3, tmp);
		 */

		/* recover dma select*/
		tmp = smhc_readl(smc_host, SMHC_CTRL1);
		tmp &= ~DmaSel;
		smhc_writel(smc_host, SMHC_CTRL1, tmp);

		dma_unmap_sg(mmc_dev(smc_host->mmc), data->sg, data->sg_len,
			     sunxi_mmc_get_dma_dir(data));
	}
	/*clear int status*/
	smhc_writel(smc_host, SMHC_INT_STA,
		    0xffffffff & ~(CardInt | CardRemoveInt | CardInsertInt));
	/*clear  int signal enable*/
	tmp = smhc_readl(smc_host, SMHC_INT_SIG_EN);
	tmp &= (CardInt | CardRemoveInt | CardInsertInt);
	smhc_writel(smc_host, SMHC_INT_SIG_EN, tmp);

	dev_dbg(mmc_dev(smc_host->mmc), "final sta %x\n",
		smhc_readl(smc_host, SMHC_INT_STA));

	dev_dbg(mmc_dev(smc_host->mmc),
		"smc p%d done, resp %08x %08x %08x %08x\n", smc_host->phy_index,
		mrq->cmd->resp[0], mrq->cmd->resp[1], mrq->cmd->resp[2],
		mrq->cmd->resp[3]);

/*
*	//if(host->dat3_imask){
*	//      rval = mmc_readl(host,REG_GCTRL);
*	//      mmc_writel(host, REG_GCTRL, rval|SDXC_DEBOUNCE_ENABLE_BIT);
*	//}
*/

	smc_host->mrq = NULL;
	smc_host->int_sum = 0;
	smc_host->wait_dma = false;

	return (smc_host->manual_stop_mrq
		|| smc_host->mrq_busy) ? IRQ_WAKE_THREAD : IRQ_HANDLED;
}

static irqreturn_t sunxi_mmc_irq(int irq, void *dev_id)
{
	struct sunxi_mmc_host *host = dev_id;
	struct mmc_request *mrq;
	u32 msk_int = 0;
	bool finalize = false;
	bool sdio_int = false;
	irqreturn_t ret = IRQ_HANDLED;
	u32 int_sta = 0;
	u32 int_sig_en = 0;
	u32 int_sta_en = 0;

	spin_lock(&host->lock);

	int_sta = smhc_readl(host, SMHC_INT_STA);
	int_sig_en = smhc_readl(host, SMHC_INT_SIG_EN);
	int_sta_en = smhc_readl(host, SMHC_INT_STA_EN);
	msk_int = int_sta & int_sig_en;

	dev_dbg(mmc_dev(host->mmc),
		"smc p%d irq, sta %08x(%08x) sta_en %08x sig_en %08x,msk_int %x\n",
		host->phy_index, int_sta, host->int_sum, int_sta_en, int_sig_en,
		msk_int);

	/*
	*   if(host->dat3_imask){
	*   if(msk_int & SDXC_CARD_INSERT){
	*   mmc_writel(host, REG_RINTR, SDXC_CARD_INSERT);
	*   mmc_detect_change(host->mmc,msecs_to_jiffies(500));
	*   goto out;
	*   }
	*   if(msk_int & SDXC_CARD_REMOVE){
	*   mmc_writel(host, REG_RINTR, SDXC_CARD_REMOVE)
	*   mmc_detect_change(host->mmc,msecs_to_jiffies(50));
	*   goto out;
	*   }
	*   }
	 */
	/*
	*   if(host->dat3_imask){
	*   if(msk_int & CardInsertInt){
	*   smhc_writel(smc_host,SMHC_INT_STA,CardInsertInt);
	*   mmc_detect_change(host->mmc,msecs_to_jiffies(500));
	*   goto out;
	*   }
	*   if(msk_int  & CardRemoveInt){
	*   smhc_writel(smc_host,SMHC_INT_STA,CardRemoveInt);
	*   mmc_detect_change(host->mmc,msecs_to_jiffies(50));
	*   goto out;
	*   }
	*   }
	 */

	mrq = host->mrq;
	if (mrq) {
		if (msk_int & DmaInt)
			host->wait_dma = false;

		host->int_sum |= msk_int;

		/* Don't wait for dma on error */
		if (host->int_sum & ErrIntBit)
			finalize = true;
		else if ((host->int_sum & DoneIntBit) && !host->wait_dma)
			finalize = true;
	}

	if (msk_int & CardInt)
		sdio_int = true;

	smhc_writel(host, SMHC_INT_STA, msk_int);

	if (finalize)
		ret = sunxi_mmc_finalize_request(host);
/*out:*/
	spin_unlock(&host->lock);

	if (finalize && ret == IRQ_HANDLED)
		mmc_request_done(host->mmc, mrq);

	if (sdio_int)
		mmc_signal_sdio_irq(host->mmc);

	return ret;
}

/*
*static int sunxi_check_r1_ready(struct sunxi_mmc_host *smc_host, unsigned ms)
*{
*	//struct sunxi_mmc_host *smc_host = mmc_priv(mmc);
*dev_info(mmc_dev(smc_host->mmc),"no imple %s %d\n",
		__func__,__LINE__);
*	return 1;
*}
*/

static int sunxi_check_r1_ready_may_sleep(struct sunxi_mmc_host *smc_host,
					  unsigned ms)
{
	dev_info(mmc_dev(smc_host->mmc), "no imple %s %d\n", __func__,
		 __LINE__);
	return 1;
}

static irqreturn_t sunxi_mmc_handle_bottom_half(int irq, void *dev_id)
{
	struct sunxi_mmc_host *host = dev_id;
	struct mmc_request *mrq;
	struct mmc_request *mrq_busy = NULL;
	unsigned long iflags;

	spin_lock_irqsave(&host->lock, iflags);
	mrq = host->manual_stop_mrq;
	mrq_busy = host->mrq_busy;
	spin_unlock_irqrestore(&host->lock, iflags);
		/*
		*Here,we don't use the timeout value in mrq_busy->busy_timeout
		*Because this value may not right for example when useing TRIM
		*So we use max wait time and print time value every 1 second
		*/
	if (mrq_busy) {
		sunxi_check_r1_ready_may_sleep(host, 0x7ffffff);
		spin_lock_irqsave(&host->lock, iflags);
		host->mrq_busy = NULL;
		spin_unlock_irqrestore(&host->lock, iflags);
		mmc_request_done(host->mmc, mrq_busy);
		return IRQ_HANDLED;

	} else
		dev_dbg(mmc_dev(host->mmc), "no request for busy\n");

	if (!mrq) {
		dev_err(mmc_dev(host->mmc), "no request for manual stop\n");
		return IRQ_HANDLED;
	}

	dev_err(mmc_dev(host->mmc), "data error, sending stop command\n");

	/*
	 * We will never have more than one outstanding request,
	 * and we do not complete the request until after
	 * we've cleared host->manual_stop_mrq so we do not need to
	 * spin lock this function.
	 * Additionally we have wait states within this function
	 * so having it in a lock is a very bad idea.
	 */
	sunxi_mmc_send_manual_stop(host, mrq);

	spin_lock_irqsave(&host->lock, iflags);
	host->manual_stop_mrq = NULL;
	spin_unlock_irqrestore(&host->lock, iflags);

	mmc_request_done(host->mmc, mrq);

	return IRQ_HANDLED;
}

static void sunxi_mmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct sunxi_mmc_host *host = mmc_priv(mmc);
	u32 rval;
	u32 tmp = 0;
	static const char * const bus_mode[] = { "", "OD", "PP" };
	static const char * const pwr_mode[] = { "OFF", "UP", "ON" };
	static const char * const timing[] = { "LEGACY(SDR12)",
		"MMC-HS(SDR20)",
		"SD-HS(SDR25)", "UHS-SDR12",
		"UHS-SDR25",
		"UHS-SDR50", "UHS-SDR104", "UHS-DDR50", "MMC-HS200", "MMC-HS400"
	};
	static const char * const drv_type[] = { "B", "A", "C", "D" };

WARN_ON(ios->bus_mode >= ARRAY_SIZE(bus_mode) / ARRAY_SIZE(bus_mode[0]));
WARN_ON(ios->power_mode >= ARRAY_SIZE(pwr_mode) / ARRAY_SIZE(pwr_mode[0]));
WARN_ON(ios->timing >= ARRAY_SIZE(timing) / ARRAY_SIZE(timing[0]));
dev_info(mmc_dev(mmc), "sdc set ios: ",
		 "clk %dHz bm %s pm %s vdd %d width %d timing %s dt %s\n",
		 ios->clock, bus_mode[ios->bus_mode],
		 pwr_mode[ios->power_mode], ios->vdd,
		 1 << ios->bus_width, timing[ios->timing],
		 drv_type[ios->drv_type]);

	/* Set the power state */
	switch (ios->power_mode) {
	case MMC_POWER_ON:
		break;

	case MMC_POWER_UP:
		if (host->power_on)
			break;
		if (!IS_ERR(mmc->supply.vmmc)) {
			rval =
			    sunxi_mmc_regulator_set_ocr(mmc, mmc->supply.vmmc,
						  ios->vdd);
			if (rval)
				return;
		}
		if (!IS_ERR(mmc->supply.vqmmc)) {
			rval = regulator_enable(mmc->supply.vqmmc);
			if (rval < 0) {
				dev_err(mmc_dev(mmc),
					"failed to enable vqmmc regulator\n");
				return;
			}
		}

		rval = pinctrl_select_state(host->pinctrl, host->pins_default);
		if (rval) {
			dev_err(mmc_dev(mmc), "could not set default pins\n");
			return;
		}

		if (!IS_ERR(host->clk_rst)) {
			rval = clk_prepare_enable(host->clk_rst);
			if (rval) {
				dev_err(mmc_dev(mmc), "reset err %d\n", rval);
				return;
			}
		}

		rval = clk_prepare_enable(host->clk_ahb);
		if (rval) {
			dev_err(mmc_dev(mmc), "Enable ahb clk err %d\n", rval);
			return;
		}
		rval = clk_prepare_enable(host->clk_mmc);
		if (rval) {
			dev_err(mmc_dev(mmc), "Enable mmc clk err %d\n", rval);
			return;
		}

		host->ferror = sunxi_mmc_init_host(mmc);
		if (host->ferror)
			return;

		enable_irq(host->irq);

		host->power_on = 1;
		dev_dbg(mmc_dev(mmc), "power on!\n");
		break;

	case MMC_POWER_OFF:
		if (!host->power_on || host->dat3_imask)
			break;

		disable_irq(host->irq);
		/*sunxi_mmc_reset_host(host);*/

		clk_disable_unprepare(host->clk_mmc);
		clk_disable_unprepare(host->clk_ahb);

		if (!IS_ERR(host->clk_rst))
			clk_disable_unprepare(host->clk_rst);

		rval = pinctrl_select_state(host->pinctrl, host->pins_sleep);
		if (rval) {
			dev_err(mmc_dev(mmc), "could not set sleep pins\n");
			return;
		}
		if (!IS_ERR(mmc->supply.vqmmc)) {
			rval = regulator_disable(mmc->supply.vqmmc);
			if (rval) {
				dev_err(mmc_dev(mmc),
					"Could not disable vqmmc\n");
				return;
			}
		}

		if (!IS_ERR(mmc->supply.vmmc)) {
			rval = sunxi_mmc_regulator_set_ocr(mmc, mmc->supply.vmmc, 0);
			if (rval)
				return;
		}

		host->power_on = 0;
		dev_dbg(mmc_dev(mmc), "power off!\n");
		break;
	}

	/* set bus width */
	switch (ios->bus_width) {
	case MMC_BUS_WIDTH_1:
		tmp = smhc_readl(host, SMHC_CTRL1);
		tmp &= ~(BusWidth | ExtBusWidth);
		smhc_writel(host, SMHC_CTRL1, tmp);
		break;
	case MMC_BUS_WIDTH_4:
		tmp = smhc_readl(host, SMHC_CTRL1);
		tmp &= ~(BusWidth | ExtBusWidth);
		tmp |= BusWidth;	/*4bit bus*/
		smhc_writel(host, SMHC_CTRL1, tmp);
		break;
	case MMC_BUS_WIDTH_8:
		tmp = smhc_readl(host, SMHC_CTRL1);
		tmp &= ~(BusWidth | ExtBusWidth);
		tmp |= ExtBusWidth;	/*8bit bus*/
		smhc_writel(host, SMHC_CTRL1, tmp);
		break;
	}

	/*if set ddr mode in SMHC_ACMD_ERR_CTRL2,
	 *we  reset SD Clock Enable before changing this field to
	 *avoid generating clock glitch
	 */
	sunxi_mmc_oclk_onoff(host, 0);
	/* set ddr mode */
	if (sunxi_mmc_ddr_timing(ios->timing)) {
		dev_dbg(mmc_dev(mmc), "fun %s,line %d val%x\n", __func__,
			__LINE__, smhc_readl(host, SMHC_ACMD_ERR_CTRL2));
		tmp = smhc_readl(host, SMHC_ACMD_ERR_CTRL2);
		tmp &= ~DdrType;
		tmp |= (0x4 << DDR_SHIFT);
		smhc_writel(host, SMHC_ACMD_ERR_CTRL2, tmp);

		smhc_writel(host, SMHC_ATC, 0x50310000);

/*
*smhc_writel(smc_host, SMHC_RTC,   (1U<<3)|(0));
*smhc_writel(smc_host, SMHC_DITC0,
*(((1U<<3)|0)<<24) | (((1U<<3)|0)<<16) | (((1U<<3)|0)<<8) | (((1U<<3)|0)<<0) );
*smhc_writel(smc_host, SMHC_DITC1,
*(((1U<<3)|0)<<24) | (((1U<<3)|0)<<16) | (((1U<<3)|0)<<8) | (((1U<<3)|0)<<0) );
*/
		dev_dbg(mmc_dev(mmc), "fun %s,line %d val%x %x\n", __func__,
			__LINE__, smhc_readl(host, SMHC_ACMD_ERR_CTRL2),
			smhc_readl(host, SMHC_ATC));
	} else if (ios->timing == MMC_TIMING_MMC_HS400) {
		tmp = smhc_readl(host, SMHC_ACMD_ERR_CTRL2);
		tmp &= ~DdrType;
		tmp |= (0x5 << DDR_SHIFT);
		smhc_writel(host, SMHC_ACMD_ERR_CTRL2, tmp);

		smhc_writel(host, SMHC_ATC, 0x30330000);

/*
*   smhc_writel(smc_host, SMHC_RTC,   (1U<<3)|(3));
*   smhc_writel(smc_host, SMHC_DITC0,
*(((1U<<3)|3)<<24) | (((1U<<3)|3)<<16) | (((1U<<3)|3)<<8) | (((1U<<3)|3)<<0) );
*   smhc_writel(smc_host, SMHC_DITC1,
*(((1U<<3)|3)<<24) | (((1U<<3)|3)<<16) | (((1U<<3)|3)<<8) | (((1U<<3)|3)<<0) );
 */

		dev_dbg(mmc_dev(mmc), "fun %s,line %d val%x %x\n", __func__,
			__LINE__, smhc_readl(host, SMHC_ACMD_ERR_CTRL2),
			smhc_readl(host, SMHC_ATC));
	} else {
		tmp = smhc_readl(host, SMHC_ACMD_ERR_CTRL2);
		tmp &= ~DdrType;
		smhc_writel(host, SMHC_ACMD_ERR_CTRL2, tmp);

		smhc_writel(host, SMHC_ATC, 0x30330000);

/*
*   smhc_writel(smc_host, SMHC_RTC,   (1U<<3)|(3));
*   smhc_writel(smc_host, SMHC_DITC0,
(((1U<<3)|3)<<24) | (((1U<<3)|3)<<16) | (((1U<<3)|3)<<8) | (((1U<<3)|3)<<0) );
*   smhc_writel(smc_host, SMHC_DITC1,
(((1U<<3)|3)<<24) | (((1U<<3)|3)<<16) | (((1U<<3)|3)<<8) | (((1U<<3)|3)<<0) );
 */

		dev_dbg(mmc_dev(mmc), "fun %s,line %d val%x %x\n", __func__,
			__LINE__, smhc_readl(host, SMHC_ACMD_ERR_CTRL2),
			smhc_readl(host, SMHC_ATC));
	}
	sunxi_mmc_oclk_onoff(host, 1);

	/* set up clock */
	if (ios->power_mode && host->sunxi_mmc_clk_set_rate) {
		host->ferror = host->sunxi_mmc_clk_set_rate(host, ios);
		/* Android code had a usleep_range(50000, 55000); here */
	}
}

static void sunxi_mmc_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	struct sunxi_mmc_host *host = mmc_priv(mmc);
	unsigned long flags;
	u32 int_sig_en = 0;
	u32 int_sta_en = 0;

	spin_lock_irqsave(&host->lock, flags);
	int_sig_en = smhc_readl(host, SMHC_INT_SIG_EN);
	int_sta_en = smhc_readl(host, SMHC_INT_STA_EN);

	if (enable) {
		host->sdio_imask = CardInt;
		int_sta_en |= CardInt;
		int_sig_en |= CardInt;
		/*
		*smhc_writel(host,SMHC_INT_STA_EN,int_sta_en|CardInt);
		*smhc_writel(host,SMHC_INT_SIG_EN,int_sig_en|CardInt);
		*/
	} else {
		host->sdio_imask = 0;
		int_sta_en &= (~CardInt);
		int_sig_en &= (~CardInt);
		/*
		*smhc_writel(smc_host,SMHC_INT_STA_EN,int_sta_en & (~CardInt));
		*smhc_writel(smc_host,SMHC_INT_SIG_EN,int_sig_en & (~CardInt));
		*/
	}
	smhc_writel(host, SMHC_INT_STA_EN, int_sta_en);
	smhc_writel(host, SMHC_INT_SIG_EN, int_sig_en);

	spin_unlock_irqrestore(&host->lock, flags);
}

static void sunxi_mmc_hw_reset(struct mmc_host *mmc)
{
	dev_info(mmc_dev(mmc), "no imple %s %d\n", __func__, __LINE__);
}

static int sunxi_mmc_signal_voltage_switch(struct mmc_host *mmc,
					   struct mmc_ios *ios)
{
#ifdef CONFIG_REGULATOR
	int ret = 0;
	struct regulator *vqmmc = mmc->supply.vqmmc;
	struct device_node *np = NULL;
	bool disable_vol_switch = false;

	if (!mmc->parent || !mmc->parent->of_node) {
		dev_err(mmc_dev(mmc),
			"no dts to parse signal switch fun,use default\n");
		return 0;
	}

	np = mmc->parent->of_node;
	disable_vol_switch =
	    of_property_read_bool(np, "sunxi-dis-signal-vol-sw");

	/*For some emmc,io voltage will be fixed at 1.8v or other voltage,
	*so we can not switch io voltage
	*/
	/*Because mmc core will change the io voltage to 3.3v when power up,
	*so will must disable voltage switch
	*/
	if (disable_vol_switch) {
		dev_dbg(mmc_dev(mmc), "disable signal voltage-switch\n");
		return 0;
	}

	switch (ios->signal_voltage) {
	case MMC_SIGNAL_VOLTAGE_330:
		if (!IS_ERR(vqmmc)) {
			ret = regulator_set_voltage(vqmmc, 2700000, 3600000);
			if (ret) {
				dev_err(mmc_dev(mmc),
					"Switching to 3.3V signalling voltage ",
					" failed\n");
				return -EIO;
			}
		} else {
			dev_info(mmc_dev(mmc),
				 "no vqmmc,Check if there is regulator\n");
			return 0;
		}
		/* Wait for 5ms */
		/*usleep_range(5000, 5500);*/
		return 0;
	case MMC_SIGNAL_VOLTAGE_180:
		if (!IS_ERR(vqmmc)) {
			ret = regulator_set_voltage(vqmmc, 1700000, 1950000);
			if (ret) {
				dev_err(mmc_dev(mmc),
					"Switching to 1.8V signalling voltage ",
					" failed\n");
				return -EIO;
			}
		} else {
			dev_info(mmc_dev(mmc),
				 "no vqmmc,Check if there is regulator\n");
			return 0;
		}

		/* Wait for 5ms */
		/*usleep_range(5000, 5500);*/
		return 0;
	case MMC_SIGNAL_VOLTAGE_120:
		if (!IS_ERR(vqmmc)) {
			ret = regulator_set_voltage(vqmmc, 1100000, 1300000);
			if (ret) {
				dev_err(mmc_dev(mmc),
					"Switching to 1.2V signalling voltage ",
					" failed\n");
				return -EIO;
			}
		} else {
			dev_info(mmc_dev(mmc),
				 "no vqmmc,Check if there is regulator\n");
			return 0;
		}

		return 0;
	default:
		/* No signal voltage switch required */
		dev_err(mmc_dev(mmc),
			"unknown signal voltage switch request %x\n",
			ios->signal_voltage);
		return -1;
	}
#else
	return 0;
#endif
}

static int sunxi_mmc_card_busy(struct mmc_host *mmc)
{
	struct sunxi_mmc_host *host = mmc_priv(mmc);

	return !(smhc_readl(host, SMHC_STA) & Dat0LineSta);
}

static void sunxi_mmc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct sunxi_mmc_host *host = mmc_priv(mmc);
	struct mmc_command *cmd = mrq->cmd;
	struct mmc_data *data = mrq->data;
	unsigned long iflags;
	bool wait_dma = host->wait_dma;
	int ret;
	u32 cmd_val = 0;
	u32 int_sg_en = 0;
	u32 int_st_en = 0;
	u32 cmd_attr = 0;
	u32 tmp = 0;

	/* Check for set_ios errors (should never happen) */
	if (host->ferror) {
		mrq->cmd->error = host->ferror;
		mmc_request_done(mmc, mrq);
		return;
	}
	/*Wait for cmd line free*/
	/*
	*   if(sunxi_wait_bit_clr(host,                  \
	*   SMHC_STA,CmdInhibitCmd, \
	*   "SMHC_STA","CmdInhibitCmd",1)){
	*   dev_err(mmc_dev(mmc),"Wait cmd free timeout\n");
	*   mrq->cmd->error = -EINVAL;
	*   mmc_request_done(mmc, mrq);
	*   return;
	*   }
	 */

	/*************cmd val ***************/
	/*response*/
	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_136)
			cmd_val |= Rsp136;
		else if (cmd->flags & MMC_RSP_BUSY)
			cmd_val |= Rsp48b;
		else
			cmd_val |= Rsp48;

		if (cmd->flags & MMC_RSP_CRC)
			cmd_val |= CheckRspCRC;
		if (cmd->flags & MMC_RSP_OPCODE)
			cmd_val |= CheckRspIdx;
	}
	/*data*/
	if ((cmd->flags & MMC_CMD_MASK) == MMC_CMD_ADTC) {
		cmd_val |= DataExp;
		if (cmd->data->blocks > 1)
			cmd_val |= MultiBlkTrans | BlkCntEn;

		if (cmd->data->flags & MMC_DATA_READ)
			cmd_val |= Read;

/*if sbc is set,use cmd23 instead of cmd12 ,
 *regardless if stop is set
 */
		if (cmd->mrq->sbc != NULL)
			cmd_val |= AutoCmd23;
		else if (cmd->mrq->stop != NULL)
			cmd_val |= AutoCmd12;
		cmd_val |= DMAEn;
	}
	/*opcode*/
	cmd_val |= (cmd->opcode & 0x3f) << 24;

	/******************cmd attr val******************/
	if (cmd->opcode == MMC_GO_IDLE_STATE)
		cmd_attr |= SendInitSeq;
	else
		cmd_attr &= ~SendInitSeq;

	if (data) {
		/*Wait for data line free */
		/*
		*   if(sunxi_wait_bit_clr(host,          \
		*   SMHC_STA,CmdInhibitDat, \
		*   "SMHC_STA","CmdInhibitDat",1)){
		*   dev_err(mmc_dev(mmc),"Wait data free timeout\n");
		*   mrq->cmd->error = -EINVAL;
		*   mmc_request_done(mmc, mrq);
		*   return;
		*   }
		 */
		/*prepare dma */
		ret = sunxi_mmc_map_dma(host, data);
		if (ret < 0) {
			dev_err(mmc_dev(mmc), "map DMA failed\n");
			cmd->error = ret;
			data->error = ret;
			mmc_request_done(mmc, mrq);
			return;
		}
	}

	/***************irq setting and the wait in irq *********************/
	if ((cmd->flags & MMC_CMD_MASK) == MMC_CMD_ADTC) {
		int_sg_en = TransOverInt;
		if (cmd->data->flags & MMC_DATA_READ) {
			wait_dma = true;
			int_sg_en |= DmaInt;
		}
	} else {
		if (cmd->flags & MMC_RSP_BUSY)
			int_sg_en = TransOverInt;
		else
			int_sg_en = CmdOverInt;
	}

	spin_lock_irqsave(&host->lock, iflags);
	if (host->mrq || host->manual_stop_mrq || host->mrq_busy) {
		spin_unlock_irqrestore(&host->lock, iflags);

		if (data)
			dma_unmap_sg(mmc_dev(mmc), data->sg, data->sg_len,
				     sunxi_mmc_get_dma_dir(data));

		dev_err(mmc_dev(mmc), "request already pending\n");
		mrq->cmd->error = -EBUSY;
		mmc_request_done(mmc, mrq);
		return;
	}

	host->mrq = mrq;

	/**************irq setting*******************/
	/*enble all int state*/
	smhc_writel(host, SMHC_INT_STA_EN, 0xffffffff);
	int_st_en = smhc_readl(host, SMHC_INT_STA_EN);
	int_sg_en |= ErrIntBit;
	int_sg_en |= (smhc_readl(host, SMHC_INT_SIG_EN))
	    & (CardInt | CardRemoveInt | CardInsertInt);
	smhc_writel(host, SMHC_INT_SIG_EN, int_sg_en);
	host->wait_dma = wait_dma;

	if (data) {
	/**************DMA setting*******************/
		sunxi_mmc_start_dma(host, data);
	/**************blk size/blk cnt*******************/
		smhc_writel(host, SMHC_BLK_CFG,
			    (data->
			     blksz & 0xFFF) | ((data->blocks & 0xFFFF) << 16));
		host->sunxi_mmc_thld_ctl(host, &mmc->ios, data);
	}

	/**************arg*******************/
	smhc_writel(host, SMHC_CMD_ARG1, cmd->arg);
	dev_dbg(mmc_dev(mmc), "stop %x,sbc %x\n", (u32) (cmd->mrq->stop),
		(u32) (cmd->mrq->sbc));
	/*if sbc is set,use cmd23 instead of cmd12 ,regardless if stop is set*/
	if (cmd->mrq->sbc)
		smhc_writel(host, SMHC_CMD_ARG2, cmd->mrq->sbc->arg);
	else if (cmd->mrq->stop)
		smhc_writel(host, SMHC_CMD_ARG2, cmd->mrq->stop->arg);

	/**************cmd attr*******************/
	tmp = smhc_readl(host, SMHC_CMD_ATTR);
	tmp &= ~SendInitSeq;	/*clear Initialization first*/
	smhc_writel(host, SMHC_CMD_ATTR, cmd_attr | tmp);

	dev_dbg(mmc_dev(mmc),
		"smc p%d cmd %d(%08x) cmd_val %x in_sg_en 0x%08x int_st_en 0x%08x len %d\n",
		host->phy_index, cmd->opcode & 0x3f, cmd->arg, cmd_val,
		int_sg_en, int_st_en,
		host->mrq->data ? host->mrq->data->blksz *
		host->mrq->data->blocks : 0);

	if (host->dat3_imask) {
		dev_info(mmc_dev(host->mmc), "no imple %s %d\n", __func__,
			 __LINE__);
	}

	/******************exe cmd******************/
	smhc_writel(host, SMHC_CMD, cmd_val);

	spin_unlock_irqrestore(&host->lock, iflags);
}

/*we use our own mmc_regulator_get_supply
*because our platform regulator not support supply name,
*/
/*only support regulator ID,
*but linux mmc' own mmc_regulator_get_supply use supply name
*/
static int sunxi_mmc_regulator_get_supply(struct mmc_host *mmc)
{
	struct device *dev = mmc_dev(mmc);
	int ret = 0;
	struct sunxi_mmc_host *host = mmc_priv(mmc);

	mmc->supply.vmmc = regulator_get_optional(dev, "vmmc");
	mmc->supply.vqmmc = regulator_get_optional(dev, "vqmmc");
	host->supply.vdmmc = regulator_get_optional(dev, "vdmmc");

	if (IS_ERR(mmc->supply.vmmc)) {
		dev_info(dev, "No vmmc regulator found\n");
	} else {
		ret = mmc_regulator_get_ocrmask(mmc->supply.vmmc);
		if (ret > 0)
			mmc->ocr_avail = ret;
		else
			dev_warn(dev, "Failed getting OCR mask: %d\n", ret);
	}

	if (IS_ERR(mmc->supply.vqmmc))
		dev_info(dev, "No vqmmc regulator found\n");

	if (IS_ERR(host->supply.vdmmc))
		dev_info(dev, "No vdmmc regulator found\n");

	return 0;
}

/*
*Because our regulator driver does not support binding to device tree,
*so we can not binding it to our dev(for example
*regulator_get(dev, reg_str[0]) or devm_regulator_get(dev, reg_str[0]) )
*/
/*so we must release it manully*/
static void sunxi_mmc_regulator_release_supply(struct mmc_host *mmc)
{
	struct sunxi_mmc_host *host = mmc_priv(mmc);

	if (!IS_ERR(host->supply.vdmmc))
		regulator_put(host->supply.vdmmc);

	if (!IS_ERR(mmc->supply.vqmmc))
		regulator_put(mmc->supply.vqmmc);

	if (!IS_ERR(mmc->supply.vmmc))
		regulator_put(mmc->supply.vmmc);
}

static const struct of_device_id sunxi_mmc_of_match[] = {
	{.compatible = "allwinner,sunxi-mmc-v5px",},
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, sunxi_mmc_of_match);

static struct mmc_host_ops sunxi_mmc_ops = {
	.request = sunxi_mmc_request,
	.set_ios = sunxi_mmc_set_ios,
	.get_ro = mmc_gpio_get_ro,
	.get_cd = mmc_gpio_get_cd,
	.enable_sdio_irq = sunxi_mmc_enable_sdio_irq,
	.hw_reset = sunxi_mmc_hw_reset,
	.start_signal_voltage_switch = sunxi_mmc_signal_voltage_switch,
	.card_busy = sunxi_mmc_card_busy,
};

static int sunxi_mmc_resource_request(struct sunxi_mmc_host *host,
				      struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int ret;

	if (of_device_is_compatible(np, "allwinner,sunxi-mmc-v5px")) {
		int phy_index = 0;

		if (of_property_match_string(np, "device_type", "sdc0") == 0) {
			phy_index = 0;
		} else if (of_property_match_string(np, "device_type", "sdc1")
			   == 0) {
			phy_index = 1;
		} else if (of_property_match_string(np, "device_type", "sdc2")
			   == 0) {
			phy_index = 2;
		} else if (of_property_match_string(np, "device_type", "sdc3")
			   == 0) {
			phy_index = 3;
		} else {
			dev_err(&pdev->dev, "No sdmmc device,check dts\n");
		}
		sunxi_mmc_init_priv_v5px(host, pdev, phy_index);
	}
	/*ret = mmc_regulator_get_supply(host->mmc);*/
	ret = sunxi_mmc_regulator_get_supply(host->mmc);
	if (ret)
		return ret;
	/*Maybe in some platform,no regulator,so we set ocr_avail manully */
	if (!host->mmc->ocr_avail)
		host->mmc->ocr_avail =
		    MMC_VDD_28_29 | MMC_VDD_29_30 | MMC_VDD_30_31 |
		    MMC_VDD_31_32 | MMC_VDD_32_33 | MMC_VDD_33_34;

	/*enable card detect pin power*/
	if (!IS_ERR(host->supply.vdmmc)) {
		ret = regulator_enable(host->supply.vdmmc);
		if (ret < 0)
			dev_err(mmc_dev(host->mmc),
				"failed to enable vdmmc regulator\n");
			return ret;
	}

	host->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(host->pinctrl)) {
		ret = PTR_ERR(host->pinctrl);
		goto error_disable_regulator;
	}

	host->pins_default = pinctrl_lookup_state(host->pinctrl,
						  PINCTRL_STATE_DEFAULT);
	if (IS_ERR(host->pins_default)) {
		dev_err(&pdev->dev, "could not get default pinstate\n");
		ret = PTR_ERR(host->pins_default);
		goto error_disable_regulator;
	}

	host->pins_sleep = pinctrl_lookup_state(host->pinctrl,
						PINCTRL_STATE_SLEEP);
	if (IS_ERR(host->pins_sleep)) {
		dev_err(&pdev->dev, "could not get sleep pinstate\n");
		ret = PTR_ERR(host->pins_sleep);
		goto error_disable_regulator;
	}

	host->reg_base = devm_ioremap_resource(&pdev->dev,
					       platform_get_resource(pdev,
					       IORESOURCE_MEM,
								     0));
	if (IS_ERR(host->reg_base)) {
		ret = PTR_ERR(host->reg_base);
		goto error_disable_regulator;
	}

	host->clk_ahb = devm_clk_get(&pdev->dev, "ahb");
	if (IS_ERR(host->clk_ahb)) {
		dev_err(&pdev->dev, "Could not get ahb clock\n");
		ret = PTR_ERR(host->clk_ahb);
		goto error_disable_regulator;
	}

	host->clk_mmc = devm_clk_get(&pdev->dev, "mmc");
	if (IS_ERR(host->clk_mmc)) {
		dev_err(&pdev->dev, "Could not get mmc clock\n");
		ret = PTR_ERR(host->clk_mmc);
		goto error_disable_regulator;
	}

	host->clk_rst = devm_clk_get(&pdev->dev, "rst");
	if (IS_ERR(host->clk_rst))
		dev_warn(&pdev->dev, "Could not get mmc rst\n");

	if (!IS_ERR(host->clk_rst)) {
		ret = clk_prepare_enable(host->clk_rst);
		if (ret) {
			dev_err(&pdev->dev, "reset err %d\n", ret);
			goto error_disable_regulator;
		}
	}

	ret = clk_prepare_enable(host->clk_ahb);
	if (ret) {
		dev_err(&pdev->dev, "Enable ahb clk err %d\n", ret);
		goto error_assert_reset;
	}

	ret = clk_prepare_enable(host->clk_mmc);
	if (ret) {
		dev_err(&pdev->dev, "Enable mmc clk err %d\n", ret);
		goto error_disable_clk_ahb;
	}

	/*
	 * Sometimes the controller asserts the irq on boot for some reason,
	 * make sure the controller is in a sane state before enabling irqs.
	 */
	/*ret = sunxi_mmc_reset_host(host);*/
	ret = sunxi_mmc_init_host(host->mmc);
	if (ret)
		goto error_disable_clk_mmc;

	host->irq = platform_get_irq(pdev, 0);
	ret = devm_request_threaded_irq(&pdev->dev, host->irq, sunxi_mmc_irq,
					sunxi_mmc_handle_bottom_half, 0,
					"sunxi-mmc", host);
	if (ret) {
		dev_err(&pdev->dev, "failed to request irq %d\n", ret);
		goto error_disable_clk_mmc;
	}

	disable_irq(host->irq);

	clk_disable_unprepare(host->clk_mmc);
	clk_disable_unprepare(host->clk_ahb);
#if 0
	if (!IS_ERR(host->reset))
		reset_control_assert(host->reset);
#else
	if (!IS_ERR(host->clk_rst))
		clk_disable_unprepare(host->clk_rst);
#endif
/**
 *	ret = mmc_create_sys_fs(host,pdev);
 *	if(ret)
 *	      goto error_disable_regulator;
 */

	return ret;

error_disable_clk_mmc:
	clk_disable_unprepare(host->clk_mmc);
error_disable_clk_ahb:
	clk_disable_unprepare(host->clk_ahb);
error_assert_reset:
#if 0
	if (!IS_ERR(host->reset))
		reset_control_assert(host->reset);
#else
	if (!IS_ERR(host->clk_rst))
		clk_disable_unprepare(host->clk_rst);
#endif
error_disable_regulator:
	if (!IS_ERR(host->supply.vdmmc))
		regulator_disable(host->supply.vdmmc);
	sunxi_mmc_regulator_release_supply(host->mmc);

	return ret;
}

static int sunxi_mmc_probe(struct platform_device *pdev)
{
	struct sunxi_mmc_host *host;
	struct mmc_host *mmc;
	int ret;

	dev_info(&pdev->dev, "%s\n", DRIVER_VERSION);

	mmc = mmc_alloc_host(sizeof(struct sunxi_mmc_host), &pdev->dev);
	if (!mmc) {
		dev_err(&pdev->dev, "mmc alloc host failed\n");
		return -ENOMEM;
	}

	host = mmc_priv(mmc);
	host->mmc = mmc;
	spin_lock_init(&host->lock);

	ret = sunxi_mmc_resource_request(host, pdev);
	if (ret)
		goto error_free_host;

	host->dma_mask = DMA_BIT_MASK(32);
	pdev->dev.dma_mask = &host->dma_mask;
	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	host->sg_cpu = dma_alloc_coherent(&pdev->dev, SUNXI_REQ_PAGE_SIZE,
					  &host->sg_dma, GFP_KERNEL);
	if (!host->sg_cpu) {
		dev_err(&pdev->dev, "Failed to allocate DMA descriptor mem\n");
		ret = -ENOMEM;
		goto error_free_host;
	}

	mmc->ops = &sunxi_mmc_ops;
	mmc->max_blk_count = MAX_BLK_COUNT;
	mmc->max_blk_size = MAX_BLK_SIZE;
	mmc->max_segs = SUNXI_REQ_PAGE_SIZE / sizeof(struct sdhc_idma_des);
	mmc->max_seg_size = 1 << host->idma_des_size_bits;
	mmc->max_req_size = mmc->max_blk_size * mmc->max_blk_count;
	/* 400kHz ~ 50MHz */
	mmc->f_min = 400000;
	mmc->f_max = 50000000;
	/*
	 *  mmc->caps|= MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED \
	 *  | MMC_CAP_ERASE | MMC_CAP_WAIT_WHILE_BUSY;
	 */
	mmc->caps |=
	    MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED | MMC_CAP_ERASE;
	/*mmc->caps2      |= MMC_CAP2_HS400_1_8V;*/

#ifndef CONFIG_REGULATOR
	/*Because fpga has no regulator,so we add it manully*/
	mmc->ocr_avail =
	    MMC_VDD_28_29 | MMC_VDD_29_30 | MMC_VDD_30_31 | MMC_VDD_31_32 |
	    MMC_VDD_32_33 | MMC_VDD_33_34;
	dev_info(&pdev->dev,
		 "*******************set host ocr**************************\n");

#endif

	mmc_of_parse(mmc);
	if (mmc->sunxi_caps3 & MMC_SUNXI_CAP3_DAT3_DET) {
		/*host->dat3_imask = SDXC_CARD_INSERT|SDXC_CARD_REMOVE;*/
		dev_info(mmc_dev(host->mmc), "no imple %s %d\n", __func__,
			 __LINE__);
	}

	ret = mmc_add_host(mmc);
	if (ret)
		goto error_free_dma;

	ret = mmc_create_sys_fs(host, pdev);
	if (ret) {
		dev_err(&pdev->dev, "create sys fs failed\n");
		goto error_free_dma;
	}

	dev_info(&pdev->dev, "base:0x%p irq:%u\n", host->reg_base, host->irq);
	platform_set_drvdata(pdev, mmc);
	return 0;

error_free_dma:
	dma_free_coherent(&pdev->dev, SUNXI_REQ_PAGE_SIZE, host->sg_cpu,
			  host->sg_dma);
error_free_host:
	mmc_free_host(mmc);
	return ret;
}

static int sunxi_mmc_remove(struct platform_device *pdev)
{
	struct mmc_host *mmc = platform_get_drvdata(pdev);
	struct sunxi_mmc_host *host = mmc_priv(mmc);

	mmc_remove_host(mmc);
	disable_irq(host->irq);
/*      sunxi_mmc_reset_host(host);*/

	mmc_remove_sys_fs(host, pdev);

	if (!IS_ERR(host->supply.vdmmc))
		regulator_disable(host->supply.vdmmc);

	sunxi_mmc_regulator_release_supply(mmc);

	dma_free_coherent(&pdev->dev, SUNXI_REQ_PAGE_SIZE, host->sg_cpu,
			  host->sg_dma);
	mmc_free_host(mmc);

	return 0;
}

#ifdef CONFIG_PM

static void sunxi_mmc_regs_save(struct sunxi_mmc_host *host)
{
	struct sunxi_mmc_ctrl_regs *bak_regs = &host->bak_regs;

/*save public register */
/*dev_info(mmc_dev(host->mmc),"no imple %s %d\n",__func__,__LINE__);*/
	bak_regs->rst_clk_ctrl = smhc_readl(host, SMHC_RST_CLK_CTRL);
	bak_regs->int_sta_en = smhc_readl(host, SMHC_INT_STA_EN);
	bak_regs->to = smhc_readl(host, SMHC_TO_CTRL2);
	bak_regs->ctrl3 = smhc_readl(host, SMHC_CTRL3);
	bak_regs->int_sig_en = smhc_readl(host, SMHC_INT_SIG_EN);
	bak_regs->ctrl1 = smhc_readl(host, SMHC_CTRL1);
	bak_regs->acmd_err_ctrl2 = smhc_readl(host, SMHC_ACMD_ERR_CTRL2);
	bak_regs->atc = smhc_readl(host, SMHC_ATC);

	if (host->sunxi_mmc_save_spec_reg)
		host->sunxi_mmc_save_spec_reg(host);
	else
		dev_warn(mmc_dev(host->mmc), "no spec reg save\n");
}

static void sunxi_mmc_regs_restore(struct sunxi_mmc_host *host)
{
	struct sunxi_mmc_ctrl_regs *bak_regs = &host->bak_regs;

/*restore public register */
/*dev_info(mmc_dev(host->mmc),"no imple %s %d\n",__func__,__LINE__);*/
	smhc_writel(host, SMHC_RST_CLK_CTRL, bak_regs->rst_clk_ctrl);
	smhc_writel(host, SMHC_INT_STA_EN, bak_regs->rst_clk_ctrl);
	smhc_writel(host, SMHC_TO_CTRL2, bak_regs->rst_clk_ctrl);
	smhc_writel(host, SMHC_CTRL3, bak_regs->rst_clk_ctrl);
	smhc_writel(host, SMHC_INT_SIG_EN, bak_regs->rst_clk_ctrl);
	smhc_writel(host, SMHC_CTRL1, bak_regs->rst_clk_ctrl);
	smhc_writel(host, SMHC_ACMD_ERR_CTRL2, bak_regs->rst_clk_ctrl);
	smhc_writel(host, SMHC_ATC, bak_regs->rst_clk_ctrl);

	if (host->sunxi_mmc_restore_spec_reg)
		host->sunxi_mmc_restore_spec_reg(host);
	else
		dev_warn(mmc_dev(host->mmc), "no spec reg restore\n");

}

static int sunxi_mmc_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mmc_host *mmc = platform_get_drvdata(pdev);
	struct sunxi_mmc_host *host = mmc_priv(mmc);
	int ret = 0;

	dev_info(mmc_dev(host->mmc), "suspend start%s %d\n", __func__,
		 __LINE__);
	if (mmc) {
		ret = mmc_suspend_host(mmc);
		if (!ret) {
			if (!IS_ERR(host->supply.vdmmc)) {
				ret = regulator_disable(host->supply.vdmmc);
				if (ret) {
					dev_err(mmc_dev(mmc),
						"disable vdmmc failed in suspend\n");
					return ret;
				}
			}

			if (mmc_card_keep_power(mmc) || host->dat3_imask) {
				disable_irq(host->irq);
				sunxi_mmc_regs_save(host);

				clk_disable_unprepare(host->clk_mmc);
				clk_disable_unprepare(host->clk_ahb);

				if (!IS_ERR(host->clk_rst))
					clk_disable_unprepare(host->clk_rst);

				ret =
				    pinctrl_select_state(host->pinctrl,
							 host->pins_sleep);
				if (ret) {
					dev_err(mmc_dev(mmc),
						"could not set sleep pins in suspend\n");
					return ret;
				}
				if (!IS_ERR(mmc->supply.vqmmc))
					regulator_disable(mmc->supply.vqmmc);

				if (!IS_ERR(mmc->supply.vmmc)) {
					ret =
					    sunxi_mmc_regulator_set_ocr(mmc,
								  mmc->supply.
								  vmmc, 0);
					return ret;
				}
				dev_info(mmc_dev(mmc), "dat3_imask %x\n",
					 host->dat3_imask);
				/*dump_reg(host);  */
			}
		}
	}

	dev_info(mmc_dev(host->mmc), "suspend end %s %d\n", __func__,
		 __LINE__);
	return ret;
}

static int sunxi_mmc_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mmc_host *mmc = platform_get_drvdata(pdev);
	struct sunxi_mmc_host *host = mmc_priv(mmc);
	int ret = 0;

	dev_info(mmc_dev(host->mmc), "resume start%s %d\n", __func__,
		 __LINE__);
	if (mmc) {
		if (mmc_card_keep_power(mmc) || host->dat3_imask) {
			if (!IS_ERR(mmc->supply.vmmc)) {
				ret =
				    sunxi_mmc_regulator_set_ocr(mmc, mmc->supply.vmmc,
							  mmc->ios.vdd);
				if (ret)
					return ret;
			}

			if (!IS_ERR(mmc->supply.vqmmc)) {
				ret = regulator_enable(mmc->supply.vqmmc);
				if (ret < 0) {
					dev_err(mmc_dev(mmc),
						"failed to enable vqmmc regulator\n");
					return ret;
				}
			}

			ret =
			    pinctrl_select_state(host->pinctrl,
						 host->pins_default);
			if (ret) {
				dev_err(mmc_dev(mmc),
					"could not set default pins in resume\n");
				return ret;
			}

			if (!IS_ERR(host->clk_rst)) {
				ret = clk_prepare_enable(host->clk_rst);
				if (ret) {
					dev_err(mmc_dev(mmc), "reset err %d\n",
						ret);
					return ret;
				}
			}

			ret = clk_prepare_enable(host->clk_ahb);
			if (ret) {
				dev_err(mmc_dev(mmc), "Enable ahb clk err %d\n",
					ret);
				return ret;
			}
			ret = clk_prepare_enable(host->clk_mmc);
			if (ret) {
				dev_err(mmc_dev(mmc), "Enable mmc clk err %d\n",
					ret);
				return ret;
			}

			host->ferror = sunxi_mmc_init_host(mmc);
			if (host->ferror)
				return -1;

			sunxi_mmc_regs_restore(host);

			/*
			 * host->ferror = sunxi_mmc_update_clk(host);
			 * if (host->ferror)
			 *	return -1;
			 */
			enable_irq(host->irq);
			dev_info(mmc_dev(mmc), "dat3_imask %x\n",
				 host->dat3_imask);
			/*dump_reg(host);*/
		}
		/*enable card detect pin power*/
		if (!IS_ERR(host->supply.vdmmc)) {
			ret = regulator_enable(host->supply.vdmmc);
			if (ret < 0) {
				dev_err(mmc_dev(mmc),
					"failed to enable vdmmc regulator\n");
				return ret;
			}
		}
		ret = mmc_resume_host(mmc);
	}

	dev_info(mmc_dev(host->mmc), "resume end %s %d\n", __func__,
		 __LINE__);
	return ret;
}

static const struct dev_pm_ops sunxi_mmc_pm = {
	.suspend = sunxi_mmc_suspend,
	.resume = sunxi_mmc_resume,
};

#define sunxi_mmc_pm_ops (&sunxi_mmc_pm)

#else				/* CONFIG_PM */

#define sunxi_mmc_pm_ops NULL

#endif				/* CONFIG_PM */

static void sunxi_shutdown_mmc(struct platform_device *pdev)
{
	struct mmc_host *mmc = platform_get_drvdata(pdev);
	struct sunxi_mmc_host *host = mmc_priv(mmc);

	if (host->sunxi_mmc_shutdown)
		host->sunxi_mmc_shutdown(pdev);
}

static struct platform_driver sunxi_mmc_driver = {
	.driver = {
		   .name = "sunxi-smhc",
		   .of_match_table = of_match_ptr(sunxi_mmc_of_match),
		   .pm = sunxi_mmc_pm_ops,
		   },
	.probe = sunxi_mmc_probe,
	.remove = sunxi_mmc_remove,
	.shutdown = sunxi_shutdown_mmc,
};

module_platform_driver(sunxi_mmc_driver);

MODULE_DESCRIPTION("Allwinner's SD/MMC Card Controller Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("David Lanzend�rfer <david.lanzendoerfer@o2s.ch>");
MODULE_ALIAS("platform:sunxi-mmc");

#endif
