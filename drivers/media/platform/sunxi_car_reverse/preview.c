/*
 * Fast car reverse image preview module
 *
 * Copyright (C) 2015-2018 AllwinnerTech, Inc.
 *
 * Contacts:
 * Zeng.Yajian <zengyajian@allwinnertech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include "../../../video/fbdev/sunxi/disp2/disp/dev_disp.h"


#ifdef CONFIG_SUPPORT_BIRDVIEW
#include "BVKernal/G2dApi.h"
#include "BVKernal/birdview_api.h"
#endif
#include "colormap.h"
#include "car_reverse.h"
#include "include.h"
#include <linux/g2d_driver.h>
#include <linux/jiffies.h>
#include <linux/sunxi_tr.h>
#include <asm/cacheflush.h>
#if defined CONFIG_ARCH_SUN8IW17P1
#include <asm/glue-cache.h>
#endif
#undef _TRANSFORM_DEBUG

#define BUFFER_CNT (5)
#define BUFFER_MASK (0x03)
#define AUXLAYER_WIDTH (720)
#define AUXLAYER_HEIGHT (480)
#define AUXLAYER_SIZE (AUXLAYER_WIDTH * AUXLAYER_HEIGHT * 4)

#define SECOND_SCREEN_WIDTH (720)
#define SECOND_SCREEN_HEIGHT (480)

struct rect {
	int x, y;
	int w, h;
};

struct preview_private_data {
	struct device *dev;
	struct rect src;
	struct rect frame;
	struct rect screen;
	struct disp_layer_config config[4];
	int layer_cnt;

	int format;
	int input_src;
	int rotation;
	int mirror;
	int oview_mode;
	int oview_rdy;
	int count_render;
	int count_display;
	int viewthread;
	int is_enable;
	struct buffer_pool *disp_buffer;
	struct buffer_node *fb[BUFFER_CNT];
#ifdef CONFIG_SUPPORT_AUXILIARY_LINE
	struct buffer_pool *auxiliary_line;
	struct buffer_node *auxlayer;
	struct buffer_node *auxlayer_disp;
#endif
};

struct workqueue_struct *aut_preview_wq;

/* for auxiliary line */
#ifdef CONFIG_SUPPORT_AUXILIARY_LINE
extern int draw_auxiliary_line(void *base, int width,
								int height, int rotate,
								int lr);
extern void init_auxiliary_paramter(int ld, int hgap, int vgap, int sA, int mA);
int init_auxiliary_line(int rotate, int);
void deinit_auxiliary_line(int screen_id);
#endif

static struct mutex oview_mutex;
#ifdef CONFIG_SUNXI_G2D
extern int g2d_blit(g2d_blt *para);
extern int g2d_stretchblit(g2d_stretchblt *para);
extern int g2d_open(struct inode *inode, struct file *file);
extern int g2d_release(struct inode *inode, struct file *file);
#endif
static struct mutex g2d_mutex;

/* function from display driver */
extern struct disp_manager *disp_get_layer_manager(u32 disp);
extern int disp_get_num_screens(void);
extern s32 bsp_disp_shadow_protect(u32 disp, bool protect);
static struct preview_private_data preview[3];
// int image_rotate(struct buffer_node *frame, struct buffer_node **rotate,
		// int screen_id, int type);

int preview_output_config(struct disp_manager *mgr, int screen_id, int default_width, int default_height)
{
	int i = 0;
	struct rect perfect;

	if (mgr->device && mgr->device->get_resolution) {
		mgr->device->get_resolution(mgr->device, &preview[screen_id].screen.w,
				&preview[screen_id].screen.h);
			logdebug("screen%d size: %dx%d\n", screen_id, preview[screen_id].screen.w, preview[screen_id].screen.h);
	} else {
#ifdef CONFIG_SUPPORT_TD104
		preview[screen_id].screen.w = 720;
		preview[screen_id].screen.h = 576;
#else
		preview[screen_id].screen.w = default_width;
		preview[screen_id].screen.h = default_height;
#endif
		logerror("screen%d can't get screen size, use default: %dx%d : "
				 "rotation = %d\n", screen_id,
				 preview[screen_id].screen.w,
				 preview[screen_id].screen.h, preview[screen_id].rotation);
	}

	if (preview[screen_id].rotation == 1 || preview[screen_id].rotation == 3) {
		perfect.w = preview[screen_id].screen.h;
		perfect.h = preview[screen_id].screen.w;
	} else {
		perfect.w = preview[screen_id].screen.w;
		perfect.h = preview[screen_id].screen.h;
	}
	preview[screen_id].frame.w =
		(perfect.w > preview[screen_id].screen.w)
		? preview[screen_id].screen.w
		: perfect.w;
	preview[screen_id].frame.h =
		(perfect.h > preview[screen_id].screen.h)
		? preview[screen_id].screen.h
		: perfect.h;

	preview[screen_id].frame.x =
		(preview[screen_id].screen.w - preview[screen_id].frame.w) / 2;
	preview[screen_id].frame.y =
		(preview[screen_id].screen.h - preview[screen_id].frame.h) / 2;

#ifdef CONFIG_SUNXI_G2D
	if (preview[screen_id].rotation || preview[screen_id].mirror) {
		preview[screen_id].disp_buffer = alloc_buffer_pool(
			preview[screen_id].dev, BUFFER_CNT,
			preview[screen_id].src.w *
			preview[screen_id].src.h * 2);
		if (!preview[screen_id].disp_buffer) {
			logerror("request display buffer failed\n");
			return -1;
		}
		for (i = 0; i < BUFFER_CNT; i++) {
			preview[screen_id].fb[i] =
				preview[screen_id].disp_buffer->dequeue_buffer(
					preview[screen_id].disp_buffer);
			memset(preview[screen_id].fb[i]->vir_address,
					 0x10,
					 preview[screen_id].src.w * preview[screen_id].src.h);
			memset(preview[screen_id].fb[i]->vir_address +
						preview[screen_id].src.w * preview[screen_id].src.h,
					0x80,
					preview[screen_id].src.w * preview[screen_id].src.h);
		}
	}
#endif
	if (preview[screen_id].oview_mode) {
		if (!preview[screen_id].disp_buffer) {
			logdebug("preview[%d].disp_buffer alloc_buffer_pool\n", screen_id);
			preview[screen_id].disp_buffer = alloc_buffer_pool(preview[screen_id].dev,
					BUFFER_CNT,
					preview[screen_id].src.w * preview[screen_id].src.h * 2);

			if (!preview[screen_id].disp_buffer) {
				logerror("request display buffer failed\n");
				return -1;
			}
			for (i = 0; i < BUFFER_CNT; i++) {
				preview[screen_id].fb[i] = preview[screen_id].disp_buffer->dequeue_buffer(
						preview[screen_id].disp_buffer);
				memset(preview[screen_id].fb[i]->vir_address,
						0x10,
						preview[screen_id].src.w * preview[screen_id].src.h);
				memset(preview[screen_id].fb[i]->vir_address +
							preview[screen_id].src.w * preview[screen_id].src.h,
						0x80,
						preview[screen_id].src.w * preview[screen_id].src.h / 2);
			}
		}
	}
#ifdef CONFIG_SUPPORT_AUXILIARY_LINE
	if ((preview[screen_id].input_src == 0 || preview[screen_id].input_src == 1)
			&& (preview[screen_id].car_oview_mode == 0)) {
		init_auxiliary_paramter(800, 50, 200, 25, 80);
		init_auxiliary_line(preview[screen_id].rotation, screen_id);
	}
#endif
	preview[screen_id].is_enable = 1;
	return 0;
}

int preview_output_start(struct preview_params *params)
{
	int num_screens, screen_id;
	struct disp_manager *mgr = NULL; /*disp_get_layer_manager(0);*/

	num_screens = disp_get_num_screens();
	mutex_init(&g2d_mutex);
	mutex_init(&oview_mutex);
	for (screen_id = 0; screen_id < num_screens; screen_id++) {
		mgr = disp_get_layer_manager(screen_id);
		if (!mgr || !mgr->force_set_layer_config) {
			logerror("screen %d preview init error\n", screen_id);
			return -1;
		}
		memset(&(preview[screen_id]), 0, sizeof(preview[screen_id]));
		preview[screen_id].dev = params->dev;
		preview[screen_id].src.w = params->src_width;
		preview[screen_id].src.h = params->src_height;
		preview[screen_id].format = params->format;
		preview[screen_id].layer_cnt = 1;
		preview[screen_id].input_src = params->input_src;
		preview[screen_id].viewthread = params->viewthread;
		preview[screen_id].oview_mode = params->car_oview_mode;
		preview[screen_id].mirror = params->pr_mirror;
		preview[screen_id].rotation = params->rotation;

		if (mgr->device && mgr->device->is_enabled(mgr->device)) {
			preview_output_config(mgr, screen_id, params->screen_width, params->screen_height);
		}
	}
	return 0;
}

#ifdef CONFIG_SUPPORT_AUXILIARY_LINE
int init_auxiliary_line(int rotate, int screen_id)
{
	const struct rect _rect[2] = {
		{0, 0, AUXLAYER_HEIGHT, AUXLAYER_WIDTH},
		{0, 0, AUXLAYER_WIDTH, AUXLAYER_HEIGHT},

	};
	int idx;
	int buffer_size;
	struct disp_layer_config *config;
	void *start;
	void *end;
	if (preview[screen_id].rotation == 1 ||
			preview[screen_id].rotation == 3) {
		idx = 0;
	} else {
		idx = 1;
	}
	buffer_size = _rect[idx].w * _rect[idx].h * 4;
	preview[screen_id].auxiliary_line =
		alloc_buffer_pool(preview[screen_id].dev, 2, buffer_size);

	if (!preview[screen_id].auxiliary_line) {
		logerror("request auxiliary line buffer failed\n");
		return -1;
	}
	preview[screen_id].auxlayer =
		preview[screen_id].auxiliary_line->dequeue_buffer(
			preview[screen_id].auxiliary_line);
	if (!preview[screen_id].auxlayer) {
		logerror("no buffer in buffer pool\n");
		return -1;
	}

	preview[screen_id].auxlayer_disp =
		preview[screen_id].auxiliary_line->dequeue_buffer(
			preview[screen_id].auxiliary_line);
	if (!preview[screen_id].auxlayer_disp) {
		logerror("no buffer in buffer pool\n");
		return -1;
	}

	memset(preview[screen_id].auxlayer->vir_address, 0, AUXLAYER_SIZE);
	draw_auxiliary_line(preview[screen_id].auxlayer->vir_address,
				AUXLAYER_WIDTH, AUXLAYER_HEIGHT, 0, 0);
	start = preview[screen_id].auxlayer->vir_address;
	end = (void *)((unsigned long)start + preview[screen_id].auxlayer->size);
	__dma_flush_range(start, (size_t) end);

	start = preview[screen_id].auxlayer_disp->vir_address;
	end = (void *)((unsigned long)start + preview[screen_id].auxlayer_disp->size);
	__dma_flush_range(start, (size_t) end);

	//image_rotate(NULL, NULL, screen_id, 1);

	config = &preview[screen_id].config[1];
	memset(config, 0, sizeof(struct disp_layer_config));

	config->channel = 1;
	config->enable = 1;
	// fix linuxSDK DE invalid address for linuxapp ioctl fb0 (ch1 layer0)
	config->layer_id = 1;

	config->info.fb.addr[0] = (unsigned long long)preview[screen_id].auxlayer_disp->phy_address;
	config->info.fb.format = DISP_FORMAT_ARGB_8888;

	config->info.fb.size[0].width = _rect[idx].w;
	config->info.fb.size[0].height = _rect[idx].h;
	config->info.fb.size[1].width = _rect[idx].w;
	config->info.fb.size[1].height = _rect[idx].h;
	config->info.fb.size[2].width = _rect[idx].w;
	config->info.fb.size[2].height = _rect[idx].h;
	config->info.mode = LAYER_MODE_BUFFER;
	config->info.zorder = 1;
	config->info.fb.crop.width = (unsigned long long)_rect[idx].w << 32;
	config->info.fb.crop.height = (unsigned long long)_rect[idx].h << 32;

	config->info.alpha_mode = 0; /* pixel alpha */
	config->info.alpha_value = 0;
	config->info.screen_win.x = preview[screen_id].frame.x;
	config->info.screen_win.y = preview[screen_id].frame.y;
	config->info.screen_win.width = preview[screen_id].frame.w;
	config->info.screen_win.height = preview[screen_id].frame.h;
	preview[screen_id].layer_cnt++;
	return 0;
}

void deinit_auxiliary_line(int screen_id)
{
	if (preview[screen_id].auxlayer) {
		preview[screen_id].auxiliary_line->queue_buffer(
			preview[screen_id].auxiliary_line,
			preview[screen_id].auxlayer);
		preview[screen_id].auxlayer = 0;
	}
	if (preview[screen_id].auxlayer_disp) {
		preview[screen_id].auxiliary_line->queue_buffer(
			preview[screen_id].auxiliary_line,
			preview[screen_id].auxlayer_disp);
		preview[screen_id].auxlayer_disp = 0;
	}
	if (preview[screen_id].auxiliary_line)
		free_buffer_pool(preview[screen_id].dev,
				 preview[screen_id].auxiliary_line);
}
#endif

int preview_output_disable(void)
{
	int num_screens, screen_id;
	struct disp_manager *mgr = NULL;
	num_screens = disp_get_num_screens();

	for (screen_id = 0; screen_id < num_screens; screen_id++) {
		mgr = disp_get_layer_manager(screen_id);
		if (!mgr || !mgr->device)
			continue;
		preview[screen_id].config[0].enable = 0;
		preview[screen_id].config[1].enable = 0;
		preview[screen_id].config[2].enable = 0;
		preview[screen_id].config[3].enable = 0;

		mgr->force_set_layer_config(mgr, preview[screen_id].config,
					    preview[screen_id].layer_cnt);
		msleep(20);
	}
	return 0;
}

int preview_output_exit(struct preview_params *params)
{
	int i;
	int num_screens, screen_id;
	struct disp_manager *mgr = NULL;
	num_screens = disp_get_num_screens();
	for (screen_id = 0; screen_id < num_screens; screen_id++) {
		mgr = disp_get_layer_manager(screen_id);
		if (!mgr || !mgr->force_set_layer_config) {
			logerror("screen %d preview stop error\n", screen_id);
			return -1;
		}
		mgr->force_set_layer_config_exit(mgr);
		msleep(20);

		if (preview[screen_id].disp_buffer) {
			for (i = 0; i < BUFFER_CNT; i++) {
				if (!preview[screen_id].fb[i])
					continue;

				preview[screen_id].disp_buffer->queue_buffer(
					preview[screen_id].disp_buffer,
					preview[screen_id].fb[i]);
			}
			logdebug("preview[%d].disp_buffer free_buffer_pool\n", screen_id);
			free_buffer_pool(preview[screen_id].dev,
					 preview[screen_id].disp_buffer);
		}

#ifdef CONFIG_SUPPORT_AUXILIARY_LINE
		if ((params->input_src == 0 || params->input_src == 1)
				&& (params->car_oview_mode == 0)) {
			deinit_auxiliary_line(screen_id);
		}
#endif
	}
	return 0;
}

#if 0
int image_rotate(struct buffer_node *frame, struct buffer_node **rotate,
		 int screen_id, int type)
{
#if defined(CONFIG_SUNXI_G2D)
	static int active;
	struct buffer_node *node;
#endif
	int retval = -1;
	if (type == 0) {
#ifdef CONFIG_SUNXI_G2D
		g2d_stretchblt blit_para;
		struct file g2d_file;
		g2d_open(0, &g2d_file);
		active++;
		node = preview[screen_id].fb[active & BUFFER_MASK];
		if (!node || !frame) {
			logerror("%s, alloc buffer failed\n", __func__);
			return -1;
		}

		mutex_lock(&g2d_mutex);

		blit_para.src_image.addr[0] =
			(unsigned long)frame->phy_address;
		blit_para.src_image.addr[1] =
			(unsigned long)frame->phy_address +
			preview[screen_id].src.w * preview[screen_id].src.h;
		blit_para.src_image.w = preview[screen_id].src.w;
		blit_para.src_image.h = preview[screen_id].src.h;
		if (preview[screen_id].format == V4L2_PIX_FMT_NV21)
			blit_para.src_image.format = G2D_FMT_PYUV420UVC;
		else
			blit_para.src_image.format = G2D_FMT_PYUV422UVC;
		blit_para.src_image.pixel_seq = G2D_SEQ_NORMAL;
		blit_para.src_rect.x = 0;
		blit_para.src_rect.y = 0;
		blit_para.src_rect.w = preview[screen_id].src.w;
		blit_para.src_rect.h = preview[screen_id].src.h;

		blit_para.dst_image.addr[0] =
			(unsigned long)node->phy_address;
		blit_para.dst_image.addr[1] =
			(unsigned long)node->phy_address +
			preview[screen_id].src.w * preview[screen_id].src.h;
		if (preview[screen_id].format == V4L2_PIX_FMT_NV21)
			blit_para.dst_image.format = G2D_FMT_PYUV420UVC;
		else
			blit_para.dst_image.format = G2D_FMT_PYUV422UVC;
		blit_para.dst_image.pixel_seq = G2D_SEQ_NORMAL;
		blit_para.dst_rect.x = 0;
		blit_para.dst_rect.y = 0;
		if (preview[screen_id].rotation == 1 ||
			preview[screen_id].rotation == 3) {
			blit_para.dst_image.w = preview[screen_id].src.h;
			blit_para.dst_image.h = preview[screen_id].src.w;
			blit_para.dst_rect.w = preview[screen_id].src.h;
			blit_para.dst_rect.h = preview[screen_id].src.w;
		} else {
			blit_para.dst_image.w = preview[screen_id].src.w;
			blit_para.dst_image.h = preview[screen_id].src.h;
			blit_para.dst_rect.w = preview[screen_id].src.w;
			blit_para.dst_rect.h = preview[screen_id].src.h;
		}
		blit_para.color = 0xff;
		blit_para.alpha = 0xff;
		if (preview[screen_id].rotation == 1)
			blit_para.flag = G2D_BLT_ROTATE90;
		if (preview[screen_id].rotation == 2)
			blit_para.flag = G2D_BLT_ROTATE180;
		if (preview[screen_id].rotation == 3)
			blit_para.flag = G2D_BLT_ROTATE270;
		if (preview[screen_id].mirror) {
			blit_para.flag |= G2D_BLT_FLIP_HORIZONTAL;
		}
		retval = g2d_stretchblit(&blit_para);
		mutex_unlock(&g2d_mutex);
		if (retval < 0) {
			printk(KERN_ERR
					 "%s: g2d_blit G2D_BLT_ROTATE180 failed\n",
					 __func__);
			g2d_release(0, &g2d_file);
			return retval;
		}
		g2d_release(0, &g2d_file);
		*rotate = node;
#endif
	}
#ifdef CONFIG_SUPPORT_AUXILIARY_LINE
	if (type == 1) {
#ifdef CONFIG_SUNXI_G2D
		g2d_stretchblt blit_para;
		struct file g2d_file;
		g2d_open(0, &g2d_file);
		mutex_lock(&g2d_mutex);
		blit_para.src_image.addr[0] =
			(unsigned long long)preview[screen_id]
			.auxlayer->phy_address;
		blit_para.src_image.w = AUXLAYER_WIDTH;
		blit_para.src_image.h = AUXLAYER_HEIGHT;
		blit_para.src_image.format = G2D_FORMAT_ARGB8888;
		blit_para.src_image.pixel_seq = G2D_SEQ_NORMAL;
		blit_para.src_rect.x = 0;
		blit_para.src_rect.y = 0;
		blit_para.src_rect.w = AUXLAYER_WIDTH;
		blit_para.src_rect.h = AUXLAYER_HEIGHT;

		blit_para.dst_image.addr[0] =
			(unsigned long long)preview[screen_id]
			.auxlayer_disp->phy_address;
		blit_para.dst_image.format = G2D_FORMAT_ARGB8888;
		blit_para.dst_image.pixel_seq = G2D_SEQ_NORMAL;
		blit_para.dst_rect.x = 0;
		blit_para.dst_rect.y = 0;
		if (preview[screen_id].rotation == 1 ||
			preview[screen_id].rotation == 3) {
			blit_para.dst_image.w = AUXLAYER_HEIGHT;
			blit_para.dst_image.h = AUXLAYER_WIDTH;
			blit_para.dst_rect.w = AUXLAYER_HEIGHT;
			blit_para.dst_rect.h = AUXLAYER_WIDTH;
		} else {
			blit_para.dst_image.w = AUXLAYER_WIDTH;
			blit_para.dst_image.h = AUXLAYER_HEIGHT;
			blit_para.dst_rect.w = AUXLAYER_WIDTH;
			blit_para.dst_rect.h = AUXLAYER_HEIGHT;
		}
		blit_para.color = 0xff;
		blit_para.alpha = 0xff;
		if (preview[screen_id].rotation == 0)
			blit_para.flag = G2D_BLT_NONE;
		if (preview[screen_id].rotation == 1)
			blit_para.flag = G2D_BLT_ROTATE90;
		if (preview[screen_id].rotation == 2)
			blit_para.flag = G2D_BLT_ROTATE180;
		if (preview[screen_id].rotation == 3)
			blit_para.flag = G2D_BLT_ROTATE270;
		if (preview[screen_id].mirror)
			blit_para.flag |= G2D_BLT_FLIP_HORIZONTAL;
		retval = g2d_stretchblit(&blit_para);
		mutex_unlock(&g2d_mutex);

		if (retval < 0) {
			printk(KERN_ERR
					 "%s: g2d_blit  failed\n",
					 __func__);
			g2d_release(0, &g2d_file);
			return retval;
		}
		g2d_release(0, &g2d_file);
#endif
	}
#endif
	return retval;
}

int copy_frame(struct buffer_node *frame, struct buffer_node *node, int format,
			 int screen_id)
{
	int retval = -1;
#ifdef CONFIG_SUNXI_G2D
	g2d_stretchblt blit_para;
	struct file g2d_file;
	mutex_lock(&g2d_mutex);
	g2d_open(0, &g2d_file);
	blit_para.src_image.addr[0] =
		(unsigned long)frame->phy_address;
	blit_para.src_image.addr[1] =
		(unsigned long)frame->phy_address +
		preview[screen_id].src.w * preview[screen_id].src.h;
	blit_para.src_image.w = preview[screen_id].src.w;
	blit_para.src_image.h = preview[screen_id].src.h;
	blit_para.src_image.format = format;
	if (format == V4L2_PIX_FMT_NV21)
		blit_para.src_image.format = G2D_FMT_PYUV420UVC;
	else
		blit_para.src_image.format = G2D_FMT_PYUV422UVC;
	blit_para.src_image.pixel_seq = G2D_SEQ_NORMAL;
	blit_para.src_rect.x = 0;
	blit_para.src_rect.y = 0;
	blit_para.src_rect.w = preview[screen_id].src.w;
	blit_para.src_rect.h = preview[screen_id].src.h;

	blit_para.dst_image.addr[0] =
		(unsigned long)node->phy_address;
	blit_para.dst_image.addr[1] =
		(unsigned long)node->phy_address +
		preview[screen_id].src.w * preview[screen_id].src.h;
	if (format == V4L2_PIX_FMT_NV21)
		blit_para.dst_image.format = G2D_FMT_PYUV420UVC;
	else
		blit_para.dst_image.format = G2D_FMT_PYUV422UVC;
	blit_para.dst_image.pixel_seq = G2D_SEQ_NORMAL;
	blit_para.dst_rect.x = 0;
	blit_para.dst_rect.y = 0;
	blit_para.color = 0xff;
	blit_para.alpha = 0xff;
	blit_para.dst_image.w = preview[screen_id].src.w;
	blit_para.dst_image.h = preview[screen_id].src.h;
	blit_para.dst_rect.w = preview[screen_id].src.w;
	blit_para.dst_rect.h = preview[screen_id].src.h;
	blit_para.flag = G2D_BLT_NONE;
	retval = g2d_stretchblit(&blit_para);
	mutex_unlock(&g2d_mutex);
	if (retval < 0) {
		printk(KERN_ERR "%s: g2d_blit G2D_BLT_NONE failed\n", __func__);
		g2d_release(0, &g2d_file);
		return retval;
	}
	g2d_release(0, &g2d_file);
#endif
	return retval;
}

int merge_frame(struct buffer_node *frame, struct buffer_node *node, int format,
		int screen_id, int sequence)
{
	int retval = -1;
#ifdef CONFIG_SUNXI_G2D
	g2d_stretchblt blit_para;
	struct file g2d_file;
	g2d_rect _rect;
	if (sequence == 0) {
		_rect.x = 0;
		_rect.y = 0;
		_rect.w = preview[screen_id].src.w / 2;
		_rect.h = preview[screen_id].src.h / 2;
	} else if (sequence == 1) {
		_rect.x = preview[screen_id].src.w / 2;
		_rect.y = 0;
		_rect.w = preview[screen_id].src.w / 2;
		_rect.h = preview[screen_id].src.h / 2;
	} else if (sequence == 2) {
		_rect.x = 0;
		_rect.y = preview[screen_id].src.h / 2;
		_rect.w = preview[screen_id].src.w / 2;
		_rect.h = preview[screen_id].src.h / 2;
	} else {
		_rect.x = preview[screen_id].src.w / 2;
		;
		_rect.y = preview[screen_id].src.h / 2;
		_rect.w = preview[screen_id].src.w / 2;
		_rect.h = preview[screen_id].src.h / 2;
	}
	g2d_open(0, &g2d_file);
	mutex_lock(&g2d_mutex);
	blit_para.src_image.addr[0] =
		(unsigned long)frame->phy_address;
	blit_para.src_image.addr[1] =
		(unsigned long)frame->phy_address +
		preview[screen_id].src.w * preview[screen_id].src.h;
	blit_para.src_image.w = preview[screen_id].src.w;
	blit_para.src_image.h = preview[screen_id].src.h;
	if (format == V4L2_PIX_FMT_NV21)
		blit_para.src_image.format = G2D_FMT_PYUV420UVC;
	else
		blit_para.src_image.format = G2D_FMT_PYUV422UVC;
	blit_para.src_image.pixel_seq = G2D_SEQ_NORMAL;
	blit_para.src_rect.x = 0;
	blit_para.src_rect.y = 0;
	blit_para.src_rect.w = preview[screen_id].src.w;
	blit_para.src_rect.h = preview[screen_id].src.h;

	blit_para.dst_image.addr[0] =
		(unsigned long)node->phy_address;
	blit_para.dst_image.addr[1] =
		(unsigned long)node->phy_address +
		preview[screen_id].src.w * preview[screen_id].src.h;
	if (format == V4L2_PIX_FMT_NV21)
		blit_para.dst_image.format = G2D_FMT_PYUV420UVC;
	else
		blit_para.dst_image.format = G2D_FMT_PYUV422UVC;
	blit_para.dst_image.pixel_seq = G2D_SEQ_NORMAL;
#if 1
	blit_para.dst_image.w = preview[screen_id].src.w;
	blit_para.dst_image.h = preview[screen_id].src.h;
	blit_para.dst_rect.x = _rect.x;
	blit_para.dst_rect.y = _rect.y;
	blit_para.dst_rect.w = _rect.w;
	blit_para.dst_rect.h = _rect.h;
#else
	blit_para.dst_image.w = preview[screen_id].src.w;
	blit_para.dst_image.h = preview[screen_id].src.h;
	blit_para.dst_rect.x = 0;
	blit_para.dst_rect.y = 0;
	blit_para.dst_rect.w = preview[screen_id].src.w;
	blit_para.dst_rect.h = preview[screen_id].src.h;

#endif
	blit_para.color = 0xff;
	blit_para.alpha = 0xff;
	blit_para.flag = G2D_BLT_NONE;
	retval = g2d_stretchblit(&blit_para);
	mutex_unlock(&g2d_mutex);
	if (retval < 0) {
		printk(KERN_ERR "%s: g2d_blit G2D_BLT_NONE failed\n", __func__);
		g2d_release(0, &g2d_file);
		return retval;
	}
	g2d_release(0, &g2d_file);
#endif
	return retval;
}

#endif
void preview_update(struct buffer_node *frame, int orientation, int lr_direct)
{
	int num_screens, screen_id;
	struct disp_manager *mgr = NULL; /*disp_get_layer_manager(0);*/
	struct disp_layer_config *config = NULL;
	struct buffer_node *rotate = NULL;
	int buffer_format;
	int width;
	int height;
	if (frame == NULL) {
		logerror("preview frame is null\n");
		return;
	}
	num_screens = disp_get_num_screens();

	for (screen_id = 0; screen_id < num_screens; screen_id++) {
		mgr = disp_get_layer_manager(screen_id);
		if (!mgr || !mgr->force_set_layer_config) {
			logerror("preview update error\n");
			return;
		}
		if (!mgr->device || !mgr->device->is_enabled(mgr->device))
			continue;
		if (!preview[screen_id].is_enable) {
			preview_output_config(mgr, screen_id, SECOND_SCREEN_WIDTH, SECOND_SCREEN_HEIGHT);
		}
		config = &preview[screen_id].config[0];
		buffer_format = preview[screen_id].format;
		width = preview[screen_id].src.w;
		height = preview[screen_id].src.h;

		// if ((preview[screen_id].rotation || preview[screen_id].mirror) &&
		// 	image_rotate(frame, &rotate, screen_id, 0) == 0) {
		// 	if (!rotate) {
		// 		logerror("image rotate error\n");
		// 		return;
		// 	}
		// 	if (preview[screen_id].format == V4L2_PIX_FMT_NV21)
		// 		buffer_format = V4L2_PIX_FMT_YVU420;
		// 	else
		// 		buffer_format = V4L2_PIX_FMT_YUV422P;
		// 	if (preview[screen_id].rotation == 1 ||
		// 			preview[screen_id].rotation == 3) {
		// 		width = preview[screen_id].src.h;
		// 		height = preview[screen_id].src.w;
		// 		buffer_format = V4L2_PIX_FMT_YVU420;
		// 	}
		// }

		switch (buffer_format) {
		case V4L2_PIX_FMT_NV21:
			config->info.fb.format = DISP_FORMAT_YUV420_SP_VUVU;
#ifdef CONFIG_VIDEO_SUNXI_TVD_SPECIAL
			config->info.fb.addr[0] = (long)frame->phy_address_de;
			config->info.fb.addr[1] = (long)frame->phy_address_de + (width * height);
			config->info.fb.addr[2] = 0;
#else
			config->info.fb.addr[0] = (long)frame->phy_address;
			config->info.fb.addr[1] = (long)frame->phy_address + (width * height);
			config->info.fb.addr[2] = 0;
#endif
			config->info.fb.size[0].width = width;
			config->info.fb.size[0].height = height;
			config->info.fb.size[1].width = width / 2;
			config->info.fb.size[1].height = height / 2;
			config->info.fb.size[2].width = 0;
			config->info.fb.size[2].height = 0;
			config->info.fb.align[1] = 0;
			config->info.fb.align[2] = 0;

			config->info.fb.crop.x = (unsigned long long)40 << 32;
			config->info.fb.crop.y = (unsigned long long)0 << 32;
			config->info.fb.crop.width = (unsigned long long)(width - 80) << 32;
			config->info.fb.crop.height = (unsigned long long)height << 32;
			config->info.screen_win.x = preview[screen_id].frame.x;
			config->info.screen_win.y = preview[screen_id].frame.y;
			config->info.screen_win.width = preview[screen_id].frame.w;
			config->info.screen_win.height = preview[screen_id].frame.h;

			config->channel = 0;
			config->layer_id = 0;
			config->enable = 1;

			config->info.mode = LAYER_MODE_BUFFER;
			config->info.zorder = 0;
			config->info.alpha_mode = 1;
			config->info.alpha_value = 0xff;
			break;
		case V4L2_PIX_FMT_NV61:
			config->info.fb.format = DISP_FORMAT_YUV422_SP_VUVU;
			config->info.fb.addr[0] = (long)frame->phy_address;
			config->info.fb.addr[1] = config->info.fb.addr[0] + width * height;
			config->info.fb.addr[2] = 0;
			config->info.fb.size[0].width = width;
			config->info.fb.size[0].height = height;
			config->info.fb.size[1].width = width / 2;
			config->info.fb.size[1].height = height;
			config->info.fb.size[2].width = 0;
			config->info.fb.size[2].height = 0;
			config->info.fb.align[1] = 0;
			config->info.fb.align[2] = 0;

			config->info.fb.crop.x = (unsigned long long)40 << 32;
			config->info.fb.crop.y = (unsigned long long)0 << 32;
			config->info.fb.crop.width = (long)(width - 80) << 32;
			config->info.fb.crop.height = (unsigned long long)height << 32;
			config->info.screen_win.x = preview[screen_id].frame.x;
			config->info.screen_win.y = preview[screen_id].frame.y;
			config->info.screen_win.width = preview[screen_id].frame.w;
			config->info.screen_win.height = preview[screen_id].frame.h;

			config->channel = 0;
			config->layer_id = 0;
			config->enable = 1;

			config->info.mode = LAYER_MODE_BUFFER;
			config->info.zorder = 0;
			config->info.alpha_mode = 1;
			config->info.alpha_value = 0xff;
			break;
		case V4L2_PIX_FMT_YVU420:
			config->info.fb.format = DISP_FORMAT_YUV420_SP_VUVU;
			config->info.fb.addr[0] = (long)rotate->phy_address;
			config->info.fb.addr[1] = (long)rotate->phy_address + (width * height);
			config->info.fb.addr[2] = 0;
			config->info.fb.size[0].width = width;
			config->info.fb.size[0].height = height;
			config->info.fb.size[1].width = width / 2;
			config->info.fb.size[1].height = height / 2;
			config->info.fb.size[2].width = 0;
			config->info.fb.size[2].height = 0;
			config->info.fb.align[1] = 0;
			config->info.fb.align[2] = 0;

			config->info.fb.crop.width = (unsigned long long)width << 32;
			config->info.fb.crop.height = (unsigned long long)height << 32;
			config->info.screen_win.x = preview[screen_id].frame.x;
			config->info.screen_win.y = preview[screen_id].frame.y;
			config->info.screen_win.width = preview[screen_id].frame.w; /* FIXME */
			config->info.screen_win.height = preview[screen_id].frame.h;

			config->channel = 0;
			config->layer_id = 0;
			config->enable = 1;

			config->info.mode = LAYER_MODE_BUFFER;
			config->info.zorder = 0;
			config->info.alpha_mode = 1;
			config->info.alpha_value = 0xff;
			break;
		case V4L2_PIX_FMT_YUV422P:
			config->info.fb.format = DISP_FORMAT_YUV422_SP_VUVU;
			config->info.fb.addr[0] = (long)rotate->phy_address;
			config->info.fb.addr[1] = (long)rotate->phy_address + (width * height);
			config->info.fb.addr[2] = 0;
			config->info.fb.size[0].width = width;
			config->info.fb.size[0].height = height;
			config->info.fb.size[1].width = width / 2;
			config->info.fb.size[1].height = height;
			config->info.fb.size[2].width = 0;
			config->info.fb.size[2].height = 0;
			config->info.fb.align[1] = 0;
			config->info.fb.align[2] = 0;

			config->info.fb.crop.width = (unsigned long long)width << 32;
			config->info.fb.crop.height = (unsigned long long)height << 32;
			config->info.screen_win.x = preview[screen_id].frame.x;
			config->info.screen_win.y = preview[screen_id].frame.y;
			config->info.screen_win.width = preview[screen_id].frame.w; /* FIXME */
			config->info.screen_win.height = preview[screen_id].frame.h;

			config->channel = 0;
			config->layer_id = 0;
			config->enable = 1;

			config->info.mode = LAYER_MODE_BUFFER;
			config->info.zorder = 0;
			config->info.alpha_mode = 1;
			config->info.alpha_value = 0xff;
			break;
		default:
			logerror("%s: unknown pixel format, skip\n", __func__);
			break;
		}
		bsp_disp_shadow_protect(screen_id, true);
		mgr->force_set_layer_config(mgr, &preview[screen_id].config[0],
						preview[screen_id].layer_cnt);
		bsp_disp_shadow_protect(screen_id, false);
#ifdef CONFIG_SUPPORT_AUXILIARY_LINE
		if ((preview[screen_id].input_src == 0 || preview[screen_id].input_src == 1)
				&& (preview[screen_id].oview_mode == 0)) {
			draw_auxiliary_line(preview[screen_id].auxlayer->vir_address,
				AUXLAYER_WIDTH, AUXLAYER_HEIGHT, orientation,
				lr_direct);
			// image_rotate(NULL, NULL, screen_id, 1);
		}
#endif
	}
}

void preview_update_Ov(struct buffer_node **frame_input, int orientation,
		       int lr_direct, int oview_type)
{
	int num_screens, screen_id;
	struct disp_manager *mgr = NULL; /*disp_get_layer_manager(0);*/
	struct disp_layer_config *config = NULL;
	struct buffer_node *rotate = NULL;
	int buffer_format;
	int width;
	int height;
	int i = 0;
	unsigned layer_cnt = 0;
	struct buffer_node *frame = NULL;
	if (frame_input == NULL) {
		logerror("preview frame is null\n");
		return;
	} else {
		//printk(KERN_ERR "preview_update_Ov run\n");
		//preview_update(frame_input[2], orientation, lr_direct);
		//return;
	}
	mutex_lock(&oview_mutex);
	num_screens = disp_get_num_screens();

	for (screen_id = 0; screen_id < num_screens; screen_id++) {
		mgr = disp_get_layer_manager(screen_id);
		if (!mgr || !mgr->force_set_layer_config) {
			logerror("preview update error\n");
			return;
		}
		if (!mgr->device || !mgr->device->is_enabled(mgr->device))
			continue;
		config = &preview[screen_id].config[0];
		buffer_format = preview[screen_id].format;
		width = preview[screen_id].src.w;
		height = preview[screen_id].src.h;

		// if ((preview[screen_id].rotation || preview[screen_id].mirror) &&
		// 		image_rotate(frame, &rotate, screen_id, 0) == 0) {
		// 	if (!rotate) {
		// 		logerror("image rotate error\n");
		// 		return;
		// 	}
		// 	if (preview[screen_id].format == V4L2_PIX_FMT_NV21)
		// 		buffer_format = V4L2_PIX_FMT_YVU420;
		// 	else
		// 		buffer_format = V4L2_PIX_FMT_YUV422P;
		// 	if (preview[screen_id].rotation == 1 ||
		// 			preview[screen_id].rotation == 3) {
		// 		width = preview[screen_id].src.h;
		// 		height = preview[screen_id].src.w;
		// 		buffer_format = V4L2_PIX_FMT_YVU420;
		// 	}
		// }
		layer_cnt = 0;
		for (i = 0; i < CAR_MAX_CH && i < oview_type; i++) {
			frame = frame_input[i];
			config = &preview[screen_id].config[i];
			switch (buffer_format) {
			case V4L2_PIX_FMT_NV21:
				config->info.fb.format = DISP_FORMAT_YUV420_SP_VUVU;
				config->info.fb.addr[0] = (long) frame->phy_address;
				config->info.fb.addr[1] = (long) frame->phy_address + (width * height);
				config->info.fb.addr[2] = 0;
				config->info.fb.size[0].width = width;
				config->info.fb.size[0].height = height;
				config->info.fb.size[1].width = width / 2;
				config->info.fb.size[1].height = height / 2;
				config->info.fb.size[2].width = 0;
				config->info.fb.size[2].height = 0;
				config->info.fb.align[1] = 0;
				config->info.fb.align[2] = 0;

				config->info.fb.crop.x = (unsigned long long)40 << 32;
				config->info.fb.crop.y = (unsigned long long)0 << 32;
				config->info.fb.crop.width = (unsigned long long) (width - 80) << 32;
				config->info.fb.crop.height = (unsigned long long)height << 32;
				if (oview_type == 2) {
					if (i == 0) {
						config->info.screen_win.x = 0;
						config->info.screen_win.y = 0;
						config->info.screen_win.width = preview[screen_id].frame.w / 2;
						config->info.screen_win.height = preview[screen_id].frame.h;
					}
					if (i == 1) {
						config->info.screen_win.x = preview[screen_id].frame.w / 2;
						config->info.screen_win.y = 0;
						config->info.screen_win.width = preview[screen_id].frame.w / 2;
						config->info.screen_win.height = preview[screen_id].frame.h;
					}
					config->channel = 0;
					config->layer_id = i;
					config->info.zorder = i;
				} else if (oview_type == 3) { //not support , or it needs disp0 has 2 vi_ch,because of scale
					if (i == 0) {
						config->info.screen_win.x = 0;
						config->info.screen_win.y = 0;
						config->info.screen_win.width = preview[screen_id].frame.w / 2;
						config->info.screen_win.height = preview[screen_id].frame.h / 2;

						config->channel = 0;
						config->layer_id = 0;
						config->info.zorder = 0;
					}
					if (i == 1) {
						config->info.screen_win.x = preview[screen_id].frame.w / 2;
						config->info.screen_win.y = 0;
						config->info.screen_win.width = preview[screen_id].frame.w / 2;
						config->info.screen_win.height = preview[screen_id].frame.h / 2;

						config->channel = 0;
						config->layer_id = 1;
						config->info.zorder = 1;
					}
					if (i == 2) {
						config->info.screen_win.x = 0;
						config->info.screen_win.y = preview[screen_id].frame.h / 2;
						config->info.screen_win.width = preview[screen_id].frame.w/2;
						config->info.screen_win.height = preview[screen_id].frame.h / 2;

						config->channel = 1;
						config->layer_id = 1;
						config->info.zorder = 0;
					}
				} else if (oview_type == 4) {
					if (i == 0) {
						config->info.screen_win.x = 0;
						config->info.screen_win.y = 0;
						config->info.screen_win.width = preview[screen_id].frame.w / 2;
						config->info.screen_win.height = preview[screen_id].frame.h / 2;
					}
					if (i == 1) {
						config->info.screen_win.x = preview[screen_id].frame.w / 2;
						config->info.screen_win.y = 0;
						config->info.screen_win.width = preview[screen_id].frame.w / 2;
						config->info.screen_win.height = preview[screen_id].frame.h / 2;
					}
					if (i == 2) {
						config->info.screen_win.x = 0;
						config->info.screen_win.y = preview[screen_id].frame.h / 2;
						config->info.screen_win.width = preview[screen_id].frame.w / 2;
						config->info.screen_win.height = preview[screen_id].frame.h / 2;
					}
					if (i == 3) {
						config->info.screen_win.x = preview[screen_id].frame.w / 2;
						config->info.screen_win.y = preview[screen_id].frame.h / 2;
						config->info.screen_win.width = preview[screen_id].frame.w / 2;
						config->info.screen_win.height = preview[screen_id].frame.h / 2;
					}
					config->channel = 0;
					config->layer_id = i;
					config->info.zorder = i;
				} else {
					logerror("%s: unknown oview_type(%d), skip\n", __func__, oview_type);
					break;
				}
				config->enable = 1;
				config->info.mode = LAYER_MODE_BUFFER;
				config->info.alpha_mode = 0;
				config->info.alpha_value = 0xff;
				layer_cnt++;
				break;
			default:
				logerror("%s: unknown pixel format, skip\n", __func__);
				break;
			}
		}
		//printk("%s:%d end\n",__FUNCTION__,__LINE__);
		preview[screen_id].layer_cnt = layer_cnt;
		bsp_disp_shadow_protect(screen_id, true);

		mgr->force_set_layer_config(mgr, preview[screen_id].config, layer_cnt);
		bsp_disp_shadow_protect(screen_id, false);
	}
	mutex_unlock(&oview_mutex);
	return ;
}
