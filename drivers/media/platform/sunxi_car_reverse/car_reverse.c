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

#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/sched/rt.h>
#include <linux/slab.h>
#include <linux/delay.h>

#define VersionID "V1.0.4"

static int need_dump;

#if (defined CONFIG_SUNXI_DI && defined CONFIG_ARCH_SUN8IW20)
#include "../../../char/sunxi-di/drv_div1xx/di_client.h"
// #ifdef CONFIG_SUPPORT_TD104
#if 1
#define USE_SUNXI_DI_MODULE

#define ALIGN_16B(x) (((x) + (15)) & ~(15))

static struct di_client *car_client;

enum __di_pixel_fmt_t {
	DI_FORMAT_NV12 = 0x00,	/* 2-plane */
	DI_FORMAT_NV21 = 0x01,	/* 2-plane */
	DI_FORMAT_MB32_12 = 0x02, /* NOT SUPPORTED, UV mapping like NV12 */
	DI_FORMAT_MB32_21 = 0x03, /* NOT SUPPORTED, UV mapping like NV21 */
	DI_FORMAT_YV12 = 0x04,	/* 3-plane */
	DI_FORMAT_YUV422_SP_UVUV = 0x08, /* 2-plane, New in DI_V2.2 */
	DI_FORMAT_YUV422_SP_VUVU = 0x09, /* 2-plane, New in DI_V2.2 */
	DI_FORMAT_YUV422P = 0x0c,	/* 3-plane, New in DI_V2.2 */
	DI_FORMAT_MAX,
};

static unsigned int get_di_fb_format(unsigned int fmt)
{
	switch (fmt) {
	case DI_FORMAT_YUV422P:
		return DRM_FORMAT_YUV422;
	case DI_FORMAT_YV12:
		return DRM_FORMAT_YUV420;
	case DI_FORMAT_YUV422_SP_UVUV:
		return DRM_FORMAT_NV16;
	case DI_FORMAT_YUV422_SP_VUVU:
		return DRM_FORMAT_NV61;
	case DI_FORMAT_NV12:
		return DRM_FORMAT_NV12;
	case DI_FORMAT_NV21:
		return DRM_FORMAT_NV21;
	default:
		return 0;
	}

	return 0;
}

#else
#undef USE_SUNXI_DI_MODULE
#endif
#endif
static struct buffer_node *new_frame, *old_frame, *oold_frame;

#if defined(CONFIG_SWITCH) || defined(CONFIG_ANDROID_SWITCH)
#include <linux/switch.h>
#endif

#include "car_reverse.h"
#include "include.h"

#define MODULE_NAME "car-reverse"

#define SWAP_BUFFER_CNT (4)
#define SWAP_BUFFER_CNT_VIN (5)

#define THREAD_NEED_STOP (1 << 0)
#define THREAD_RUN (1 << 1)
#define CAR_REVSER_GPIO_LEVEL 1
/*#define USE_YUV422*/
#undef USE_YUV422
struct car_reverse_private_data {
	struct preview_params config;
	int reverse_gpio;

	struct buffer_pool *buffer_pool;
	struct buffer_pool *bufferOv_pool[CAR_MAX_CH];
	struct buffer_node *buffer_disp[2];

	struct work_struct status_detect;
	struct workqueue_struct *preview_workqueue;

	struct task_struct *display_update_task;
	struct task_struct *display_frame_task;
	struct list_head pending_frame;
	struct list_head pending_frameOv[CAR_MAX_CH];
	spinlock_t display_lock;

	int needexit;
	int needfree;
	int status;
	int disp_index;
	int debug;
	int format;
	int used_oview;
	int used_oview_type;
	int thread_mask;
	int discard_frame;
	int ov_sync;
	int ov_sync_algo;
	int ov_sync_frame;
	int sync_w;
	int sync_r;
	int standby;
	int algo_thread_start;
	int algo_mask;
	spinlock_t thread_lock;
	struct mutex free_lock;
};

static int rotate;

module_param(rotate, int, 0644);

static struct car_reverse_private_data *car_reverse;

#define UPDATE_STATE 1
#if defined(UPDATE_STATE) &&		\
	(defined(CONFIG_SWITCH) || defined(CONFIG_ANDROID_SWITCH))
static ssize_t print_dev_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", sdev->name);
}

static struct switch_dev car_reverse_switch = {
	.name = "parking-switch", .state = 0, .print_name = print_dev_name,
};

static void car_reverse_switch_register(void)
{
	switch_dev_register(&car_reverse_switch);
}

static void car_reverse_switch_unregister(void)
{
	switch_dev_unregister(&car_reverse_switch);
}

static void car_reverse_switch_update(int flag)
{
	switch_set_state(&car_reverse_switch, flag);
}
#else
static void car_reverse_switch_register(void)
{
}
static void car_reverse_switch_update(int flag)
{
}

static void car_reverse_switch_unregister(void)
{
}

#endif

static void of_get_value_by_name(struct platform_device *pdev, const char *name,
				 int *retval, unsigned int defval)
{
	if (of_property_read_u32(pdev->dev.of_node, name, retval) != 0) {
		dev_err(&pdev->dev, "missing property '%s', default value %d\n",
			name, defval);
		*retval = defval;
	}
}

static void of_get_gpio_by_name(struct platform_device *pdev, const char *name,
				int *retval)
{
	int gpio_index;
	enum of_gpio_flags config;

	gpio_index = of_get_named_gpio_flags(pdev->dev.of_node, name, 0, &config);
	if (!gpio_is_valid(gpio_index)) {
		dev_err(&pdev->dev, "failed to get gpio '%s'\n", name);
		*retval = 0;
		return;
	}
	*retval = gpio_index;

	dev_info(&pdev->dev, "%s: gpio=%d\n", name, gpio_index);
}

static void parse_config(struct platform_device *pdev,
			 struct car_reverse_private_data *priv)
{
	of_get_value_by_name(pdev, "tvd_id", &priv->config.tvd_id, 0);
#ifdef CONFIG_VIDEO_SUNXI_TVD_SPECIAL
	if (priv->config.tvd_id >= MAX_CSI_CAM_NUM) {
		priv->config.tvd_id = priv->config.tvd_id - MAX_CSI_CAM_NUM;
	}
#endif
	of_get_value_by_name(pdev, "screen_width", &priv->config.screen_width,
				0);
	of_get_value_by_name(pdev, "screen_height", &priv->config.screen_height,
				0);
	of_get_value_by_name(pdev, "rotation", &priv->config.rotation, 0);
	of_get_value_by_name(pdev, "source", &priv->config.input_src, 0);
	of_get_value_by_name(pdev, "oview", &priv->used_oview, 0);
	of_get_value_by_name(pdev, "oview_type", &priv->used_oview_type, 0);
	of_get_gpio_by_name(pdev, "reverse_pin", &priv->reverse_gpio);
}

static int tvd_fd_init;
static int tvd_fd_tmp;

//data callback
void car_reverse_display_update(int tvd_fd)
{
	int run_thread = 0;
	int n = 0;
	int tmp = 0;
	int start_camera_id = car_reverse->config.tvd_id;
	struct buffer_node *node = NULL;
	struct list_head *pending_frame = &car_reverse->pending_frame;

	spin_lock(&car_reverse->display_lock);

	if (tvd_fd_init == 0) {
		tvd_fd_init = 1;
		tvd_fd_tmp = tvd_fd;
	}

	if (car_reverse->used_oview) {
		tmp = car_reverse->sync_w - car_reverse->sync_r;
		if ((car_reverse->ov_sync & (1 << (tvd_fd - start_camera_id))) || tmp > 3) {
			for (n = 0; n < CAR_MAX_CH && n < car_reverse->used_oview_type; n++) {
				if (car_reverse->config.input_src) {
					node = video_source_dequeue_buffer_vin(n + start_camera_id);
				} else {
#ifdef CONFIG_VIDEO_SUNXI_TVD_SPECIAL
					node = video_source_dequeue_buffer(n + start_camera_id);
#endif
				}
				if (node) {
					if (car_reverse->config.input_src) {
						video_source_queue_buffer_vin(node, n + start_camera_id);
					} else {
#ifdef CONFIG_VIDEO_SUNXI_TVD_SPECIAL
						video_source_queue_buffer(node, n + start_camera_id);
#endif
					}
				}
			}
			car_reverse->ov_sync = 0;
		} else {
			car_reverse->ov_sync |= (1 << (tvd_fd - start_camera_id));
			if (car_reverse->used_oview_type == 4) {
				if ((car_reverse->ov_sync & 0xf) == 0xf) {
					run_thread = 1;
					car_reverse->ov_sync = 0;
				}
			} else if (car_reverse->used_oview_type == 3) {
				if ((car_reverse->ov_sync & 0x7) == 0x7) {
					run_thread = 1;
					car_reverse->ov_sync = 0;
				}
			} else if (car_reverse->used_oview_type == 2) {
				if ((car_reverse->ov_sync & 0x3) == 0x3) {
					run_thread = 1;
					car_reverse->ov_sync = 0;
				}
			} else {
				logerror("car_reverse used_oview_type(%d) not support\n", car_reverse->used_oview_type);
			}
		}
	}
	if (!car_reverse->used_oview) {
		/*
		while (!list_empty(pending_frame)) {
			node = list_entry(pending_frame->next,
				struct buffer_node, list);
			list_del(&node->list);
			if (car_reverse->config.input_src) {
				video_source_queue_buffer_vin(node, tvd_fd);
			} else {
#ifdef CONFIG_VIDEO_SUNXI_TVD_SPECIAL
				video_source_queue_buffer(node, tvd_fd);
				trace_printk("mv pending_frame to tvd, note->buffer_id : %d \n", node->buffer_id);
#endif
			}
		}
		*/
		if (car_reverse->config.input_src) {
			node = video_source_dequeue_buffer_vin(tvd_fd);
		} else {
#ifdef CONFIG_VIDEO_SUNXI_TVD_SPECIAL
			node = video_source_dequeue_buffer(tvd_fd);
			trace_printk("get buffer from tvd, note->buffer_id : %d \n", node->buffer_id);
#endif
		}
		if (node) {
			trace_printk("add tvd_buffer_node to pending_frame buffer_id : %d \n", node->buffer_id);
			list_add(&node->list, pending_frame);
		}
	} else {
		if (run_thread) {
			for (n = 0; n < CAR_MAX_CH && n < car_reverse->used_oview_type; n++) {
				pending_frame = &car_reverse->pending_frameOv[n];
				if (car_reverse->config.input_src) {
					node = video_source_dequeue_buffer_vin(n + start_camera_id);
				} else {
#ifdef CONFIG_VIDEO_SUNXI_TVD_SPECIAL
				node = video_source_dequeue_buffer(n + start_camera_id);
#endif
				}
				if (node) {
					list_add(&node->list, pending_frame);
				}
			}
		}
	}
	spin_unlock(&car_reverse->display_lock);

	if (car_reverse->used_oview) {
		if (run_thread) {
			if (car_reverse->display_update_task)
				wake_up_process(car_reverse->display_update_task);

			car_reverse->thread_mask |= THREAD_RUN;
			car_reverse->sync_w++;
			run_thread = 0;
		}
	} else {
		spin_lock(&car_reverse->thread_lock);
		if (car_reverse->thread_mask & THREAD_NEED_STOP) {
			spin_unlock(&car_reverse->thread_lock);
			return;
		}
		if (car_reverse->display_update_task)
			wake_up_process(car_reverse->display_update_task);

		car_reverse->thread_mask |= THREAD_RUN;
		spin_unlock(&car_reverse->thread_lock);
	}
}

void car_do_freemem(struct work_struct *work)
{
	int i = 0;
	struct buffer_pool *bp = 0;
	logdebug("into car_do_freemem\n");
	mutex_lock(&car_reverse->free_lock);
	if (car_reverse->needfree) {
		if (car_reverse->buffer_pool) {
			logdebug("car_reverse->buffer_pool free_buffer_pool\n");
			free_buffer_pool(car_reverse->config.dev,
				car_reverse->buffer_pool);
			car_reverse->buffer_pool = 0;
		}
		if (car_reverse->used_oview) {
			for (i = 0; i < CAR_MAX_CH && i < car_reverse->used_oview_type; i++) {
				bp = car_reverse->bufferOv_pool[i];
				if (bp) {
					logdebug("car_reverse->bufferOv_pool[%d] free_buffer_pool\n", i);
					free_buffer_pool(car_reverse->config.dev, bp);
					car_reverse->bufferOv_pool[i] = 0;
				}
			}
		}
		if (car_reverse->config.input_src == 0 || car_reverse->config.input_src == 1) {
			if (car_reverse->buffer_disp[0]) {
				logdebug("car_reverse->buffer_disp[0] __buffer_node_free\n");
				logdebug("__buffer_node_free: free %p\n", car_reverse->buffer_disp[0]->phy_address);
				__buffer_node_free(car_reverse->config.dev,
					car_reverse->buffer_disp[0]);
				car_reverse->buffer_disp[0] = 0;
			}
			if (car_reverse->buffer_disp[1]) {
				logdebug("car_reverse->buffer_disp[1] __buffer_node_free\n");
				logdebug("__buffer_node_free: free %p\n", car_reverse->buffer_disp[1]->phy_address);
				__buffer_node_free(car_reverse->config.dev,
					car_reverse->buffer_disp[1]);
				car_reverse->buffer_disp[1] = 0;
			}
		}
		logerror("car_reverse free buffer\n");
	} else {
		logdebug("no need free buffer! \n");
	}
	mutex_unlock(&car_reverse->free_lock);
}

static DECLARE_DELAYED_WORK(car_freework, car_do_freemem);

int algo_frame_work(void *data)
{
	struct buffer_pool *bp = 0;
	int i = 0;
	int tmp = 0;
	int count = 0;
	struct buffer_node *new_frameOv[CAR_MAX_CH];
	int oview_type = car_reverse->used_oview_type;
	while (!kthread_should_stop()) {
		car_reverse->algo_mask = 0;
		if (car_reverse->algo_thread_start) {
			tmp = car_reverse->ov_sync_frame - car_reverse->ov_sync_algo;
			if (car_reverse->ov_sync_frame > car_reverse->ov_sync_algo) {
				for (i = 0; i < CAR_MAX_CH && i < oview_type; i++) {
					new_frameOv[i] = 0;
					bp = car_reverse->bufferOv_pool[i];
					if (bp)
						new_frameOv[i] = bp->dequeue_buffer(bp);
				}
				if (tmp <= 4) {
					if ((oview_type == 2 && new_frameOv[0] && new_frameOv[1] && count) ||
						(oview_type == 3 && new_frameOv[0] && new_frameOv[1] && new_frameOv[2] && count) ||
						(oview_type == 4 && new_frameOv[0] && new_frameOv[1] && new_frameOv[2] && new_frameOv[3] && count)) {
						preview_update_Ov(new_frameOv,
							car_reverse->config.car_direct,
							car_reverse->config.lr_direct,
							oview_type);
					}

					if (count >= oview_type)
						count = 0;
					else
						count++;

					for (i = 0; i < CAR_MAX_CH && i < oview_type; i++) {
						if (new_frameOv[i]) {
								video_source_queue_buffer_vin(new_frameOv[i], i + car_reverse->config.tvd_id);
						}
					}
					car_reverse->ov_sync_algo++;
				} else {
					while (tmp > 0 && !kthread_should_stop()) {
						for (i = 0; i < CAR_MAX_CH && i < oview_type; i++) {
							new_frameOv[i] = 0;
							bp = car_reverse->bufferOv_pool[i];
							if (bp)
								new_frameOv[i] = bp->dequeue_buffer(bp);
							if (new_frameOv[i]) {
								video_source_queue_buffer_vin(new_frameOv[i], i + car_reverse->config.tvd_id);
							}
						}
						car_reverse->ov_sync_algo++;
						tmp = car_reverse->ov_sync_frame - car_reverse->ov_sync_algo;
						schedule_timeout(HZ / 10000);
					}
				}
			}
		}
		set_current_state(TASK_INTERRUPTIBLE);
		if (kthread_should_stop())
			set_current_state(TASK_RUNNING);
		if (!car_reverse->algo_thread_start) {
			car_reverse->algo_mask = 1;
			schedule();
		} else {
			schedule_timeout(HZ / 10000);
		}
	}
	return 0;
}

static int dump_buffer_in_user_space(char *file_name, const char *virt_addr, int size)
{
	struct file *pfile;
	mm_segment_t old_fs;
	loff_t pos = 0;

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pfile = filp_open(file_name, O_RDWR | O_CREAT | O_EXCL, 0755);
	set_fs(old_fs);
	kernel_write(pfile, virt_addr, size, &pos);
	filp_close(pfile, NULL);

	return 0;

}

static int current_disp_buffer_index;

static int display_update_thread(void *data)
{
	struct list_head *pending_frame = &car_reverse->pending_frame;
	struct buffer_pool *bp = car_reverse->buffer_pool;
	int ret = 0;
	int i = 0;
	struct buffer_node *new_frameOv[4], *node;
#ifdef USE_SUNXI_DI_MODULE
	unsigned int src_width, src_height;
	unsigned int dst_width, dst_height;
#endif
	new_frame = 0;
	old_frame = 0;
	logerror("%s start\n", __func__);
	while (!kthread_should_stop()) {
		bp = car_reverse->buffer_pool;
		if (!car_reverse->used_oview) {
			ret = spin_is_locked(&car_reverse->display_lock);
			if (ret) {
				goto disp_loop;
			}
			spin_lock(&car_reverse->display_lock);
		}

		if (car_reverse->config.input_src && car_reverse->used_oview) {
			if (car_reverse->sync_w != car_reverse->sync_r) {
				car_reverse->sync_r++;
				for (i = 0; i < CAR_MAX_CH && i < car_reverse->used_oview_type; i++) {
					new_frameOv[i] = NULL;
					pending_frame = &car_reverse->pending_frameOv[i];
					bp = car_reverse->bufferOv_pool[i];
					if (pending_frame->next != pending_frame) {
						new_frameOv[i] = list_entry(pending_frame->next,
							struct buffer_node, list);
						list_del(&new_frameOv[i]->list);
						bp->queue_buffer(bp, new_frameOv[i]);
					}
				}
				car_reverse->ov_sync_frame++;
			}
		} else {
			if (pending_frame->next != pending_frame) {
				new_frame = list_entry(pending_frame->next,
					struct buffer_node, list);
				list_del(&new_frame->list);
				bp->queue_buffer(bp, new_frame);
			}
		}
#ifdef USE_SUNXI_DI_MODULE
		if (car_reverse->config.input_src) {
			if (!car_reverse->used_oview) {
				old_frame = bp->dequeue_buffer(bp);
				list_add(&old_frame->list, pending_frame);
				spin_unlock(&car_reverse->display_lock);
				preview_update(new_frame,
					       car_reverse->config.car_direct,
					       car_reverse->config.lr_direct);
			}
		} else {
			if (old_frame) {
				oold_frame = old_frame;
				old_frame = bp->dequeue_buffer(bp);  // It is new_frame.
			} else {
				old_frame = bp->dequeue_buffer(bp);  // It is new_frame.
			}
			spin_unlock(&car_reverse->display_lock);
			if ((oold_frame) && (old_frame) && (new_frame)) {
				struct di_process_fb_arg fb_arg;
				struct di_fb *fb;

				src_height = car_reverse->config.src_height;
				src_width = car_reverse->config.src_width;
				dst_width = src_width;
				dst_height = src_height;
				memset(&fb_arg, 0, sizeof(fb_arg));

				fb_arg.is_interlace = 1;
				fb_arg.field_order = DI_TOP_FIELD_FIRST;

				if (car_reverse->config.format ==
					V4L2_PIX_FMT_NV61)
					fb_arg.pixel_format =
					get_di_fb_format(DI_FORMAT_YUV422_SP_VUVU);
				else
					fb_arg.pixel_format =
					get_di_fb_format(DI_FORMAT_NV21);
				// fb_arg.is_pulldown = 0;
				// fb_arg.top_field_first = 0;
				// Set di_size
				fb_arg.size.width = src_width;
				fb_arg.size.height = src_height;
				fb_arg.output_mode = DI_OUT_1FRAME;
				fb_arg.di_mode = DI_INTP_MODE_MOTION;
				// fb_arg.tnr_mode is not set.
/*set fb0*/
				fb = &fb_arg.in_fb0;
				fb->size.width = src_width;
				fb->size.height = src_height;
				// fb->buf.cstride = dst_width;
				// fb->buf.ystride = ALIGN_16B(dst_width);

				fb->dma_buf_fd = -1;
				fb->buf.addr.y_addr =
				*(unsigned long long *)(&(oold_frame->phy_address_de));
				fb->buf.addr.cb_addr =
					*(unsigned long long *)(&(
					oold_frame->phy_address_de)) +
					ALIGN_16B(src_width) * src_height;
				fb->buf.addr.cr_addr = 0;

/*set fb1*/
				fb = &fb_arg.in_fb1;
				fb->size.width = src_width;
				fb->size.height = src_height;
				// fb->buf.cstride = dst_width;
				// fb->buf.ystride = ALIGN_16B(dst_width);

				fb->dma_buf_fd = -1;
				fb->buf.addr.y_addr =
					*(unsigned long long *)(&(
					new_frame->phy_address_de));
				fb->buf.addr.cb_addr =
					*(unsigned long long *)(&(
					new_frame->phy_address_de)) +
					ALIGN_16B(src_width) * src_height;
				fb->buf.addr.cr_addr = 0;

/*set out_fb0*/
				fb = &fb_arg.out_fb0;
				fb->size.width = dst_width;
				fb->size.height = dst_height;
				// fb->buf.cstride = dst_width;
				// fb->buf.ystride = ALIGN_16B(dst_width);
				// if (car_reverse->config.format ==
				// 	V4L2_PIX_FMT_NV61)
				// 	fb->format =
				// 	get_di_fb_format(DI_FORMAT_YUV422_SP_VUVU);
				// else
				// 	fb->format =
				// 	get_di_fb_format(DI_FORMAT_NV21);
				fb->dma_buf_fd = -1;
				fb->buf.addr.y_addr = *(
					unsigned long long *)(&(
					car_reverse
					->buffer_disp[current_disp_buffer_index]
					->phy_address_de));
				fb->buf.addr.cb_addr =
					*(unsigned long long *)(&(
					car_reverse
						->buffer_disp[current_disp_buffer_index]
						->phy_address_de)) +
					ALIGN_16B(dst_width) * dst_height;
				fb->buf.addr.cr_addr = 0;
				trace_printk("in_fb0 oold_frame->buffer_id : %d. \n", oold_frame->buffer_id);
				trace_printk("in_fb1 new_frame->buffer_id : %d. \n", new_frame->buffer_id);

				ret = di_client_process_fb(car_client, &fb_arg);
				if (ret) {
					trace_printk("di_client_process_fb, ret = %d\n", ret);
				}

				if (car_reverse->discard_frame != 0) {
					car_reverse->discard_frame--;
				} else
					car_reverse->discard_frame = 0;
				if (car_reverse->discard_frame == 0) {
					trace_printk("current_disp_buffer_index : %d \n", current_disp_buffer_index);

					if (need_dump) {
						// dump_buffer()
						// out_fb0
						dump_buffer_in_user_space("/tmp/out_fb0.yuv420_p", car_reverse->buffer_disp[current_disp_buffer_index]->vir_address,
										src_width * src_height * 3 / 2);

						// in_fb0
						dump_buffer_in_user_space("/tmp/in_fb0_oold_frame.yuv420_p", oold_frame->vir_address, src_width * src_height * 3 / 2);
						// in_fb1
						dump_buffer_in_user_space("/tmp/in_fb1_new_frame.yuv420_p", new_frame->vir_address, src_width * src_height * 3 / 2);

						need_dump = 0;
					}

					preview_update(
						car_reverse->buffer_disp[current_disp_buffer_index],
						car_reverse->config.car_direct,
						car_reverse->config.lr_direct);

					// reverse disp buffer.
					if (current_disp_buffer_index == 0)
						current_disp_buffer_index = 1;
					else
						current_disp_buffer_index = 0;
				}

				if (oold_frame) {
					spin_lock(&car_reverse->display_lock);
					list_add(&oold_frame->list,
							pending_frame);

					while (!list_empty(pending_frame)) {
						node = list_entry(pending_frame->next,
								struct buffer_node, list);
						list_del(&node->list);
						if (car_reverse->config.input_src) {
							video_source_queue_buffer_vin(node, tvd_fd_tmp);
						} else {
#ifdef CONFIG_VIDEO_SUNXI_TVD_SPECIAL
							video_source_queue_buffer(node, tvd_fd_tmp);
							trace_printk("mv pending_frame to tvd, note->buffer_id : %d \n", node->buffer_id);
#endif
						}
					}

					spin_unlock(&car_reverse->display_lock);
				}
			} else {
				preview_update(new_frame,
					car_reverse->config.car_direct,
					car_reverse->config.lr_direct);
			}
		}
#else
		if (!car_reverse->used_oview) {
			old_frame = bp->dequeue_buffer(bp);
			list_add(&old_frame->list, pending_frame);
			spin_unlock(&car_reverse->display_lock);
			trace_printk("in_fb0 old_frame->buffer_id : %d. \n", old_frame->buffer_id);
			trace_printk("in_fb1 new_frame->buffer_id : %d. \n", new_frame->buffer_id);

			if (car_reverse->discard_frame != 0) {
				car_reverse->discard_frame--;
			} else {
				car_reverse->discard_frame = 0;
			}
			if (car_reverse->discard_frame == 0) {
				preview_update(new_frame, car_reverse->config.car_direct,
					car_reverse->config.lr_direct);
			}
			// preview_update(new_frame, car_reverse->config.car_direct,
			// 		car_reverse->config.lr_direct);
		}
#endif

disp_loop:
		car_reverse->thread_mask &= (~THREAD_RUN);
		if (car_reverse->config.input_src && car_reverse->used_oview) {
			if (car_reverse->sync_w == car_reverse->sync_r)
				schedule();
			else
				schedule_timeout(HZ / 10000);
			if (kthread_should_stop()) {
				break;
			}
		} else {
			set_current_state(TASK_INTERRUPTIBLE);
			if (kthread_should_stop()) {
				break;
			}
			schedule();
		}
	}
	logerror("%s stop\n", __func__);
	return 0;
}

static int car_reverse_preview_start(void)
{
	int retval = 0;
	int i, n;
	struct buffer_node *node;
	struct buffer_pool *bp = 0;
	unsigned int buf_cnt = 0;
	int heigth, width;
	int oview_type = 4;

#ifdef USE_SUNXI_DI_MODULE
	struct di_timeout_ns t;
	// struct di_dit_mode dit_mode;
	// struct di_fmd_enable fmd_en;
	unsigned int src_width, src_height;
	unsigned int dst_width, dst_height;
	int ret;
	// struct di_size src_size;
	// struct di_rect out_crop;
#endif
#ifdef CONFIG_SUPPORT_TD104
	heigth = 576;
	width = 720;
#else
	heigth = 720;
	width = 1280;
#endif

	mdelay(200);

	car_reverse->disp_index = 0;
	car_reverse->discard_frame = 0;
	car_reverse->needfree = 0;
	car_reverse->config.car_oview_mode = car_reverse->used_oview;
	oview_type = car_reverse->used_oview_type;
	cancel_delayed_work(&car_freework);
	car_reverse->display_update_task =
		kthread_create(display_update_thread, NULL, "sunxi-preview");
	if (!car_reverse->display_update_task) {
		printk(KERN_ERR "failed to create kthread\n");
		return -1;
	}
	mutex_lock(&car_reverse->free_lock);
	/* FIXME: Calculate buffer size by preview info */
	if (car_reverse->config.input_src) {
		if (car_reverse->buffer_pool == 0 && !car_reverse->used_oview) {
			logdebug("car_reverse->buffer_pool alloc_buffer_pool\n");
			car_reverse->buffer_pool = alloc_buffer_pool(
				car_reverse->config.dev, SWAP_BUFFER_CNT_VIN,
				width * heigth * 2);
		}
		if (car_reverse->buffer_disp[0] == 0) {
			logdebug("car_reverse->buffer_disp[0] __buffer_node_alloc\n");
			car_reverse->buffer_disp[0] = __buffer_node_alloc(
				car_reverse->config.dev, width * heigth * 2, 0);
			logdebug("__buffer_node_alloc:[0] vaddr:%p, paddr:%p\n",
				car_reverse->buffer_disp[0]->vir_address, car_reverse->buffer_disp[0]->phy_address);
		}
		if (car_reverse->buffer_disp[1] == 0) {
			logdebug("car_reverse->buffer_disp[1] __buffer_node_alloc\n");
			car_reverse->buffer_disp[1] = __buffer_node_alloc(
				car_reverse->config.dev, width * heigth * 2, 0);
			logdebug("__buffer_node_alloc:[1] vaddr:%p, paddr:%p\n",
				car_reverse->buffer_disp[1]->vir_address, car_reverse->buffer_disp[1]->phy_address);
		}

		buf_cnt = SWAP_BUFFER_CNT;
		if (car_reverse->used_oview) {
			for (i = 0; i < CAR_MAX_CH && i < oview_type; i++) {
				if (car_reverse->bufferOv_pool[i] == 0 && car_reverse->used_oview) {
					logdebug("car_reverse->bufferOv_pool[%d] alloc_buffer_pool\n", i);
					car_reverse->bufferOv_pool[i] = alloc_buffer_pool(car_reverse->config.dev,
							SWAP_BUFFER_CNT_VIN,
							width * heigth * 2);
					if (!car_reverse->bufferOv_pool[i]) {
						dev_err(car_reverse->config.dev,
							"alloc buffer memory bufferOv_pool failed\n");
						goto gc;
					}
				}
			}
		}
	} else {
		if (car_reverse->buffer_pool == 0)
			car_reverse->buffer_pool =
				alloc_buffer_pool(car_reverse->config.dev,
						SWAP_BUFFER_CNT, 720 * 576 * 2);
		if (car_reverse->buffer_disp[0] == 0) {
			car_reverse->buffer_disp[0] = __buffer_node_alloc(
				car_reverse->config.dev, 720 * 576 * 2, 0);
		}
		if (car_reverse->buffer_disp[1] == 0) {
			car_reverse->buffer_disp[1] = __buffer_node_alloc(
				car_reverse->config.dev, 720 * 576 * 2, 0);
		}
		buf_cnt = SWAP_BUFFER_CNT;
	}
	mutex_unlock(&car_reverse->free_lock);
	if (!car_reverse->buffer_pool && !car_reverse->used_oview) {
		dev_err(car_reverse->config.dev,
			"alloc buffer memory failed\n");
		goto gc;
	}

	if (car_reverse->config.input_src == 0 || car_reverse->config.input_src == 1) {
		if (!car_reverse->buffer_disp[0] ||
			!car_reverse->buffer_disp[1]) {
			dev_err(car_reverse->config.dev,
				"alloc buffer memory buffer_disp failed\n");
			goto gc;
		}
	}
	if (car_reverse->config.input_src) {
		car_reverse->config.format = V4L2_PIX_FMT_NV21;
	} else {
		if (car_reverse->format)
			car_reverse->config.format = V4L2_PIX_FMT_NV61;
		else
			car_reverse->config.format = V4L2_PIX_FMT_NV21;
	}
	if (car_reverse->used_oview && car_reverse->config.input_src) {

		for (n = 0; n < CAR_MAX_CH && n < oview_type; n++) {
			bp = car_reverse->bufferOv_pool[n];

			INIT_LIST_HEAD(&car_reverse->pending_frameOv[n]);
			retval = video_source_connect(&car_reverse->config, n + car_reverse->config.tvd_id);
			if (retval != 0) {
				logerror("can't connect to video source!\n");
				goto gc;
			}

			for (i = 0; i < buf_cnt; i++) {
				node = bp->dequeue_buffer(bp);
				if (car_reverse->config.input_src) {
					video_source_queue_buffer_vin(node, n + car_reverse->config.tvd_id);
				} else {
#ifdef CONFIG_VIDEO_SUNXI_TVD_SPECIAL
					video_source_queue_buffer(node, n + car_reverse->config.tvd_id);
#endif
				}
			}
		}
		car_reverse->ov_sync_frame = 0;
		car_reverse->ov_sync_algo = 0;
		car_reverse->sync_r = 0;
		car_reverse->sync_w = 0;

		car_reverse->display_frame_task =
			kthread_create(algo_frame_work, NULL, "algo-preview");
		if (!car_reverse->display_frame_task) {
			printk(KERN_ERR "failed to create kthread\n");
			goto gc;
		}

		printk(KERN_ERR "%s:%d\n", __FUNCTION__, __LINE__);

		if (car_reverse->display_frame_task) {
			struct sched_param param = {.sched_priority =
				MAX_RT_PRIO - 1};
			set_user_nice(car_reverse->display_frame_task, -20);
			sched_setscheduler(car_reverse->display_frame_task,
				SCHED_FIFO, &param);
		}

		car_reverse->config.viewthread = 0;
		car_reverse->algo_thread_start = 1;
		msleep(1);
		if (car_reverse->display_frame_task)
			wake_up_process(car_reverse->display_frame_task);

		preview_output_start(&car_reverse->config);
		for (n = 0; n < CAR_MAX_CH && n < oview_type; n++) {
			video_source_streamon_vin(n + car_reverse->config.tvd_id);
		}
	} else {
		bp = car_reverse->buffer_pool;

		INIT_LIST_HEAD(&car_reverse->pending_frame);
		retval = video_source_connect(&car_reverse->config,
						car_reverse->config.tvd_id);
		if (retval != 0) {
			logerror("can't connect to video source!\n");
			goto gc;
		}
		preview_output_start(&car_reverse->config);
		//preview_update(car_reverse->buffer_disp[0],
		//					car_reverse->config.car_direct,
		//					car_reverse->config.lr_direct);

		for (i = 0; i < buf_cnt; i++) {
			node = bp->dequeue_buffer(bp);
			node->buffer_id = i;
			if (car_reverse->config.input_src) {
				video_source_queue_buffer_vin(node, car_reverse->config.tvd_id);
			} else {
#ifdef CONFIG_VIDEO_SUNXI_TVD_SPECIAL
				video_source_queue_buffer(node, car_reverse->config.tvd_id);
#endif
			}
		}
		if (car_reverse->config.input_src) {
			video_source_streamon_vin(car_reverse->config.tvd_id);
		} else {
#ifdef CONFIG_VIDEO_SUNXI_TVD_SPECIAL
			video_source_streamon(car_reverse->config.tvd_id);
#endif
		}
	}
	car_reverse->status = CAR_REVERSE_START;
	car_reverse->thread_mask = 0;
#ifdef USE_SUNXI_DI_MODULE
	car_client = (struct di_client *)di_client_create("car_reverse");
	if (!car_client) {
		pr_err("di_client_create failed\n");
		goto gc;
	}

	// retval = di_client_reset(car_client, NULL);
	// if (retval) {
	// 	pr_err("di_client_reset failed\n");
	// 	goto gc;
	// }
	t.wait4start = 500000000ULL;
	t.wait4finish = 600000000ULL;
	retval = di_client_set_timeout(car_client, &t);
	if (retval) {
		pr_err("di_client_set_timeout failed\n");
		goto gc;
	}

	// dit_mode.intp_mode = DI_DIT_INTP_MODE_MOTION;
	// dit_mode.out_frame_mode = DI_DIT_OUT_1FRAME;
	// retval = di_client_set_dit_mode(car_client, &dit_mode);
	// if (retval) {
	// 	pr_err("di_client_set_dit_mode failed\n");
	// 	goto gc;
	// }
	// fmd_en.en = 0;
	// retval = di_client_set_fmd_enable(car_client, &fmd_en);
	// if (retval) {
	// 	pr_err("di_client_set_fmd_enable failed\n");
	// 	goto gc;
	// }

	// src_height = car_reverse->config.src_height;
	// src_width = car_reverse->config.src_width;
	// dst_width = src_width;
	// dst_height = src_height;
	// src_size.width = car_reverse->config.src_width;
	// src_size.height = car_reverse->config.src_height;
	// ret = di_client_set_video_size(car_client,
	// 				&src_size);
	// if (ret)
	// 	pr_err("di_client_set_video_size failed\n");

	// out_crop.left = 0;
	// out_crop.top = 0;
	// out_crop.right = src_size.width;
	// out_crop.bottom = src_size.height;
	// ret = di_client_set_video_crop(car_client,
	// 					&out_crop);

	// ret = di_client_check_para(car_client, NULL);
	// if (ret)
	// 	pr_err("di_client_check_para failed\n");
#endif
	return 0;
gc:
	car_reverse->needfree = 1;
	schedule_delayed_work(&car_freework, 2 * HZ);
	return -1;
}

static int car_reverse_preview_stop(void)
{
	struct buffer_node *node = NULL;
	int i;
	struct buffer_pool *bp = car_reverse->buffer_pool;
	struct list_head *pending_frame = &car_reverse->pending_frame;
	printk(KERN_ERR "car_reverse_preview_stop start\n");
	preview_output_disable();

	car_reverse->status = CAR_REVERSE_STOP;
	if (car_reverse->config.input_src && car_reverse->used_oview) {
		for (i = 0; i < CAR_MAX_CH && i < car_reverse->used_oview_type; i++)
			video_source_streamoff_vin(i + car_reverse->config.tvd_id);
	} else {
		if (car_reverse->config.input_src) {
			video_source_streamoff_vin(car_reverse->config.tvd_id);
		} else {
#ifdef CONFIG_VIDEO_SUNXI_TVD_SPECIAL
			video_source_streamoff(car_reverse->config.tvd_id);
#endif
		}
	}
	spin_lock(&car_reverse->thread_lock);
	car_reverse->thread_mask |= THREAD_NEED_STOP;
	spin_unlock(&car_reverse->thread_lock);
	if (car_reverse->config.input_src && car_reverse->used_oview) {
		struct sched_param param = {.sched_priority = MAX_RT_PRIO - 40};
		set_user_nice(car_reverse->display_frame_task, 0);
		sched_setscheduler(car_reverse->display_frame_task,
				SCHED_NORMAL, &param);
		car_reverse->algo_thread_start = 0;
		car_reverse->ov_sync_frame = 0;
		car_reverse->ov_sync_algo = 0;
		car_reverse->sync_r = 0;
		car_reverse->sync_w = 0;
		msleep(10);
		while (car_reverse->thread_mask & THREAD_RUN)
			msleep(1);
		while (!car_reverse->algo_mask &&
			car_reverse->display_frame_task)
			msleep(1);
		if (car_reverse->display_frame_task)
			kthread_stop(car_reverse->display_frame_task);
		while (car_reverse->thread_mask & THREAD_RUN)
			msleep(1);
		kthread_stop(car_reverse->display_update_task);
		car_reverse->display_frame_task = 0;
		car_reverse->display_update_task = 0;
	} else {
		while (car_reverse->thread_mask & THREAD_RUN)
			msleep(1);
		kthread_stop(car_reverse->display_update_task);
		car_reverse->display_update_task = 0;
	}
	preview_output_exit(&car_reverse->config);

__buffer_gc:
	if (car_reverse->config.input_src && car_reverse->used_oview) {
		for (i = 0; i < CAR_MAX_CH && i < car_reverse->used_oview_type; i++) {
			bp = car_reverse->bufferOv_pool[i];
			while (1) {
				node = video_source_dequeue_buffer_vin(i + car_reverse->config.tvd_id);
				if (node) {
					bp->queue_buffer(bp, node);
					logdebug("%s: collect %p\n", __func__, node->phy_address);
				} else {
					video_source_disconnect(&car_reverse->config, i + car_reverse->config.tvd_id);
					break;
				}
			}
		}

		msleep(100);
		for (i = 0; i < CAR_MAX_CH && i < car_reverse->used_oview_type; i++) {
			spin_lock(&car_reverse->display_lock);
			pending_frame = &car_reverse->pending_frameOv[i];
			bp = car_reverse->bufferOv_pool[i];
			while (!list_empty(pending_frame)) {
				node = list_entry(pending_frame->next,
						struct buffer_node, list);
				list_del(&node->list);
				bp->queue_buffer(bp, node);
			}
			spin_unlock(&car_reverse->display_lock);
			rest_buffer_pool(NULL, bp);
			//dump_buffer_pool(NULL, bp);
		}
	} else {
		if (car_reverse->config.input_src) {
			node = video_source_dequeue_buffer_vin(car_reverse->config.tvd_id);
		} else {
#ifdef CONFIG_VIDEO_SUNXI_TVD_SPECIAL
			node = video_source_dequeue_buffer(
				car_reverse->config.tvd_id);
#endif
		}
		if (node) {
			bp->queue_buffer(bp, node);
			logdebug("%s: collect %p\n", __func__, node->phy_address);
			goto __buffer_gc;
		}
		spin_lock(&car_reverse->display_lock);
		while (!list_empty(pending_frame)) {
			node = list_entry(pending_frame->next,
					struct buffer_node, list);
			list_del(&node->list);
			bp->queue_buffer(bp, node);
		}
		spin_unlock(&car_reverse->display_lock);
		rest_buffer_pool(NULL, bp);
		//dump_buffer_pool(NULL, bp);
		video_source_disconnect(&car_reverse->config,
					car_reverse->config.tvd_id);
	}

	car_reverse->needfree = 1;
	schedule_delayed_work(&car_freework, 2 * HZ);

	new_frame = 0;
	old_frame = 0;
	oold_frame = 0;
#ifdef USE_SUNXI_DI_MODULE
	di_client_destroy(car_client);
	car_client = NULL;
#endif
	printk(KERN_ERR "car_reverse_preview_stop finish\n");
	return 0;
}

// void car_reverse_set_int_ioin(void)
// {
// 	long unsigned int config;
// 	config = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_FUNC, 0xFFFF);
// 	pin_config_get(SUNXI_PINCTRL, car_irq_pin_name, &config);
// 	if (0 != SUNXI_PINCFG_UNPACK_VALUE(config)) {
// 		config = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_FUNC, 0);
// 		pin_config_set(SUNXI_PINCTRL, car_irq_pin_name, config);
// 	}
// }

// void car_reverse_set_io_int(void)
// {
// 	long unsigned int config;
// 	config = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_FUNC, 0xFFFF);
// 	pin_config_get(SUNXI_PINCTRL, car_irq_pin_name, &config);
// 	if (6 != SUNXI_PINCFG_UNPACK_VALUE(config)) {
// 		config = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_FUNC, 6);
// 		pin_config_set(SUNXI_PINCTRL, car_irq_pin_name, config);
// 	}
// }

static int car_reverse_gpio_status(void)
{
	int value = 1;

	if (car_reverse->reverse_gpio) {
		value = gpio_get_value(car_reverse->reverse_gpio);
	}

	if (value == 0) {
		return CAR_REVERSE_START;
	} else {
#ifdef _REVERSE_DEBUG_
		return (car_reverse->debug == CAR_REVERSE_START ? CAR_REVERSE_START
							: CAR_REVERSE_STOP);
#endif
		return CAR_REVERSE_STOP;
	}
}

/*
 *  current status | gpio status | next status
 *  ---------------+-------------+------------
 *		STOP	 |	STOP	 |	HOLD
 *  ---------------+-------------+------------
 *		STOP	 |	START	|	START
 *  ---------------+-------------+------------
 *		START	|	STOP	 |	STOP
 *  ---------------+-------------+------------
 *		START	|	START	|	HOLD
 *  ---------------+-------------+------------
 */
const int _transfer_table[3][3] = {
	[0] = {0, 0, 0},
	[CAR_REVERSE_START] = {0, CAR_REVERSE_HOLD, CAR_REVERSE_STOP},
	[CAR_REVERSE_STOP] = {0, CAR_REVERSE_START, CAR_REVERSE_HOLD},
};

static int car_reverse_get_next_status(void)
{
	int next_status;
	int gpio_status = car_reverse_gpio_status();
	int curr_status = car_reverse->status;
	car_reverse_switch_update(gpio_status == CAR_REVERSE_START ? 1 : 0);
	next_status = _transfer_table[curr_status][gpio_status];
	return next_status;
}

static void status_detect_func(struct work_struct *work)
{
	int retval;
	int status = car_reverse_get_next_status();

	if (car_reverse->standby)
		status = CAR_REVERSE_STOP;

	switch (status) {
	case CAR_REVERSE_START:
		if (!car_reverse->needexit) {
			retval = car_reverse_preview_start();
			logdebug("start car reverse, return %d\n", retval);
		}
		break;
	case CAR_REVERSE_STOP:
		retval = car_reverse_preview_stop();
		logdebug("stop car reverse, return %d\n", retval);
		break;
	case CAR_REVERSE_HOLD:
	default:
		break;
	}
	return;
}

static irqreturn_t reverse_irq_handle(int irqnum, void *data)
{
	queue_work(car_reverse->preview_workqueue, &car_reverse->status_detect);
	return IRQ_HANDLED;
}

static ssize_t car_reverse_status_show(struct class *class,
					struct class_attribute *attr, char *buf)
{
	int count = 0;

	if (car_reverse->status == CAR_REVERSE_STOP)
		count += sprintf(buf, "%s\n", "stop");
	else if (car_reverse->status == CAR_REVERSE_START)
		count += sprintf(buf, "%s\n", "start");
	else
		count += sprintf(buf, "%s\n", "unknow");
	return count;
}

static ssize_t car_reverse_format_store(struct class *class,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	if (!strncmp(buf, "1", 1))
		car_reverse->format = 1;
	else
		car_reverse->format = 0;
	return count;
}

static ssize_t car_reverse_format_show(struct class *class,
					struct class_attribute *attr, char *buf)
{
	int count = 0;
	count += sprintf(buf, "%d\n", car_reverse->format);
	return count;
}

static ssize_t car_reverse_oview_store(struct class *class,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	if (!strncmp(buf, "1", 1)) {
		car_reverse->used_oview = 1;
	} else {
		car_reverse->used_oview = 0;
	}
	return count;
}

static ssize_t car_reverse_oview_show(struct class *class,
					struct class_attribute *attr, char *buf)
{
	int count = 0;
	count += sprintf(buf, "%d\n", car_reverse->used_oview);
	return count;
}

static ssize_t car_reverse_oview_type_store(struct class *class,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	if (!strncmp(buf, "4", 1)) {
		car_reverse->used_oview_type = 4;
	} else if (!strncmp(buf, "3", 1)) {
		car_reverse->used_oview_type = 3;
	} else if (!strncmp(buf, "2", 1)) {
		car_reverse->used_oview_type = 2;
	} else {
		car_reverse->used_oview_type = 1;
	}
	return count;
}

static ssize_t car_reverse_oview_type_show(struct class *class,
					struct class_attribute *attr, char *buf)
{
	int count = 0;
	count += sprintf(buf, "%d\n", car_reverse->used_oview_type);
	return count;
}

static ssize_t car_reverse_tvd_id_store(struct class *class,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	if (!strncmp(buf, "5", 1)) {
		car_reverse->config.tvd_id = 5;
	} else if (!strncmp(buf, "4", 1)) {
		car_reverse->config.tvd_id = 4;
	} else if (!strncmp(buf, "3", 1)) {
		car_reverse->config.tvd_id = 3;
	} else if (!strncmp(buf, "2", 1)) {
		car_reverse->config.tvd_id = 2;
	} else if (!strncmp(buf, "1", 1)) {
		car_reverse->config.tvd_id = 1;
	} else {
		car_reverse->config.tvd_id = 0;
	}
	return count;
}

static ssize_t car_reverse_tvd_id_show(struct class *class,
					struct class_attribute *attr, char *buf)
{
	int count = 0;
	count += sprintf(buf, "%d\n", car_reverse->config.tvd_id);
	return count;
}

static ssize_t car_reverse_src_store(struct class *class,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	if (!strncmp(buf, "1", 1)) {
		car_reverse->config.input_src = 1;
	} else {
		car_reverse->config.input_src = 0;
	}
	return count;
}

static ssize_t car_reverse_src_show(struct class *class,
					struct class_attribute *attr, char *buf)
{
	int count = 0;
	count += sprintf(buf, "%d\n", car_reverse->config.input_src);
	return count;
}

static ssize_t car_reverse_rotation_store(struct class *class,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	int err;
	unsigned long val;
	err = kstrtoul(buf, 10, &val); /* strict_strtoul */
	if (err) {
		return err;
	}
	car_reverse->config.rotation = (unsigned int)val;
	return count;
}

static ssize_t car_reverse_rotation_show(struct class *class,
					struct class_attribute *attr,
					char *buf)
{
	int count = 0;
	count += sprintf(buf, "%d\n", car_reverse->config.rotation);
	return count;
}

static ssize_t car_reverse_needexit_store(struct class *class,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	if (!strncmp(buf, "1", 1))
		car_reverse->needexit = 1;
	else
		car_reverse->needexit = 0;
	return count;
}

static ssize_t car_reverse_needexit_show(struct class *class,
					struct class_attribute *attr,
					char *buf)
{
	int count = 0;
	if (car_reverse->needexit == 1)
		count += sprintf(buf, "needexit = %d\n", 1);
	else
		count += sprintf(buf, "needexit = %d\n", 0);
	return count;
}
#ifdef CONFIG_SUPPORT_AUXILIARY_LINE
static ssize_t car_reverse_orientation_store(struct class *class,
						struct class_attribute *attr,
						const char *buf, size_t count)
{
	int err;
	unsigned long val;
	err = kstrtoul(buf, 10, &val); /* strict_strtoul */
	if (err) {
		return err;
	}
	car_reverse->config.car_direct = (unsigned int)val;
	return count;
}

static ssize_t car_reverse_orientation_show(struct class *class,
						struct class_attribute *attr,
						char *buf)
{
	int count = 0;
	count += sprintf(buf, "%d\n", car_reverse->config.car_direct);
	return count;
}

static ssize_t car_reverse_lrdirect_store(struct class *class,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	int err;
	unsigned long val;
	err = kstrtoul(buf, 10, &val); /* strict_strtoul */
	if (err) {
		return err;
	}
	car_reverse->config.lr_direct = (unsigned int)val;
	return count;
}

static ssize_t car_reverse_lrdirect_show(struct class *class,
					struct class_attribute *attr,
					char *buf)
{
	int count = 0;
	count += sprintf(buf, "%d\n", car_reverse->config.lr_direct);
	return count;
}

static ssize_t car_reverse_mirror_store(struct class *class,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	int err;
	unsigned long val;
	err = kstrtoul(buf, 10, &val); /* strict_strtoul */
	if (err) {
		return err;
	}
	car_reverse->config.pr_mirror = (unsigned int)val;
	return count;
}

static ssize_t car_reverse_mirror_show(struct class *class,
					struct class_attribute *attr, char *buf)
{
	int count = 0;
	count += sprintf(buf, "%d\n", car_reverse->config.pr_mirror);
	return count;
}

#endif
#ifdef _REVERSE_DEBUG_
static ssize_t car_reverse_debug_store(struct class *class,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	if (!strncmp(buf, "stop", 4))
		car_reverse->debug = CAR_REVERSE_STOP;
	else if (!strncmp(buf, "start", 5))
		car_reverse->debug = CAR_REVERSE_START;
	queue_work(car_reverse->preview_workqueue, &car_reverse->status_detect);
	return count;
}
#endif

int save_frame(char *image_name, void *buffer_addr, int size)
{
	struct file *pfile;
	mm_segment_t old_fs;
	ssize_t bw;
	loff_t pos = 0;
	char file_name[20] = "/tmp/";

	strcat(file_name, image_name);
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pfile = filp_open(file_name, O_RDWR | O_CREAT | O_EXCL, 0755);
	set_fs(old_fs);
	if (IS_ERR(pfile)) {
		printk("%s, open %s err\n", __func__, file_name);
		goto OUT;
	}

	bw = kernel_write(pfile, (char *)buffer_addr, size, &pos);
	if (unlikely(bw != size))
		printk("%s, write %s err at byte offset %llu\n", __func__, file_name, pfile->f_pos);

	printk("save %s successful!\r\n", file_name);

	filp_close(pfile, NULL);

	return 0;
OUT:
	return 1;
}

static ssize_t car_reverse_cap_store(struct class *class,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	char *image_name = NULL;
	unsigned int size = 0, width = 0, height = 0;

	image_name = kmalloc(count, GFP_KERNEL | __GFP_ZERO);
	if (!image_name) {
		printk("kmalloc image name failed");
		goto OUT;
	}

	strncpy(image_name, buf, count);
	image_name[count - 1] = '\0';

#ifdef CONFIG_VIDEO_SUNXI_TVD_SPECIAL
	width = 720;
	height = 576;
	size = width * height * 2;
	if (save_frame(image_name, new_frame->vir_address, size)) {
		printk("save error!\r\n");
		goto FREE;
	}
#else
	width = 1280;
	height = 720;
	size = width * height * 2;
	if (save_frame(image_name, new_frame->vir_address, size)) {
		printk("save error!\r\n");
		goto FREE;
	}
#endif

FREE:
	kfree(image_name);
	image_name = NULL;

OUT:
	return count;
}

static struct class_attribute car_reverse_attrs[] = {
	__ATTR(status, 0775, car_reverse_status_show, NULL),
	__ATTR(needexit, 0775, car_reverse_needexit_show, car_reverse_needexit_store),
	__ATTR(format, 0775, car_reverse_format_show, car_reverse_format_store),
	__ATTR(rotation, 0775, car_reverse_rotation_show, car_reverse_rotation_store),
	__ATTR(src, 0775, car_reverse_src_show, car_reverse_src_store),
	__ATTR(oview, 0775, car_reverse_oview_show, car_reverse_oview_store),
	__ATTR(oview_type, 0775, car_reverse_oview_type_show, car_reverse_oview_type_store),
	__ATTR(tvd_id, 0775, car_reverse_tvd_id_show, car_reverse_tvd_id_store),
#ifdef CONFIG_SUPPORT_AUXILIARY_LINE
	__ATTR(car_mirror, 0775, car_reverse_mirror_show, car_reverse_mirror_store),
	__ATTR(car_direct, 0775, car_reverse_orientation_show, car_reverse_orientation_store),
	__ATTR(car_lr, 0775, car_reverse_lrdirect_show, car_reverse_lrdirect_store),
#endif
#ifdef _REVERSE_DEBUG_
	__ATTR(debug, S_IRUGO | S_IWUSR, NULL, car_reverse_debug_store),
#endif
	__ATTR(cap_dump, 0775, NULL, car_reverse_cap_store),
};

static struct class *car_reverse_class;

static int car_reverse_probe(struct platform_device *pdev)
{
	int retval = 0;
	int err = 0;
	int i;
	long reverse_pin_irqnum;
	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, "of_node is missing\n");
		retval = -EINVAL;
		goto _err_out;
	}
	car_reverse = devm_kzalloc(
		&pdev->dev, sizeof(struct car_reverse_private_data), GFP_KERNEL);
	if (!car_reverse) {
		dev_err(&pdev->dev, "kzalloc for private data failed\n");
		retval = -ENOMEM;
		goto _err_out;
	}
	parse_config(pdev, car_reverse);
#ifdef CONFIG_VIDEO_SUNXI_TVD_SPECIAL
	car_reverse->config.dev = &pdev->dev;
#else
	car_reverse->config.pdev = pdev;
	car_reverse->config.dev = vin_get_dev(car_reverse->config.tvd_id);
#endif

	platform_set_drvdata(pdev, car_reverse);
	INIT_LIST_HEAD(&car_reverse->pending_frame);
	spin_lock_init(&car_reverse->display_lock);
	spin_lock_init(&car_reverse->thread_lock);
	mutex_init(&car_reverse->free_lock);
	car_reverse->needexit = 0;
	car_reverse->status = CAR_REVERSE_STOP;

	if (car_reverse->reverse_gpio) {
		reverse_pin_irqnum = gpio_to_irq(car_reverse->reverse_gpio);
		if (IS_ERR_VALUE(reverse_pin_irqnum)) {
			dev_err(&pdev->dev,
				"map gpio [%d] to virq failed, errno = %ld\n",
				car_reverse->reverse_gpio, reverse_pin_irqnum);
			retval = -EINVAL;
			goto _err_out;
		}
	}
	car_reverse->preview_workqueue =
		create_singlethread_workqueue("car-reverse-wq");
	if (!car_reverse->preview_workqueue) {
		dev_err(&pdev->dev, "create workqueue failed\n");
		retval = -EINVAL;
		goto _err_out;
	}
	INIT_WORK(&car_reverse->status_detect, status_detect_func);
	/* sys/class/car_reverse */
	car_reverse_class = class_create(THIS_MODULE, "car_reverse");
	if (IS_ERR(car_reverse_class)) {
		pr_err("%s:%u class_create() failed\n", __func__, __LINE__);
		return PTR_ERR(car_reverse_class);
	}

	/* sys/class/car_reverse/xxx */
	for (i = 0; i < ARRAY_SIZE(car_reverse_attrs); i++) {
		err = class_create_file(car_reverse_class, &car_reverse_attrs[i]);
		if (err) {
			pr_err("%s:%u class_create_file() failed. err=%d\n", __func__, __LINE__, err);
			while (i--) {
				class_remove_file(car_reverse_class, &car_reverse_attrs[i]);
			}
			class_destroy(car_reverse_class);
			car_reverse_class = NULL;
			return err;
		}
	}

	car_reverse_switch_register();
	car_reverse->format = 0;
	car_reverse->standby = 0;
	car_reverse->config.car_oview_mode = car_reverse->used_oview;
	car_reverse->config.car_oview_type = car_reverse->used_oview_type;
	car_reverse->config.format = V4L2_PIX_FMT_NV21;
#ifdef USE_YUV422
	car_reverse->config.format = V4L2_PIX_FMT_NV61;
	car_reverse->format = 1;
#endif
	car_reverse->needfree = 0;
	printk(KERN_ERR "%s:%d curVersion:%s\n", __FUNCTION__, __LINE__, VersionID);
	if (car_reverse->reverse_gpio) {
		if (request_irq(reverse_pin_irqnum, reverse_irq_handle,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				"car-reverse", pdev)) {
			dev_err(&pdev->dev, "request irq %ld failed\n",
				reverse_pin_irqnum);
			retval = -EBUSY;
			goto _err_free_buffer;
		}
	}

	if (car_reverse->reverse_gpio) {
		car_reverse->debug = CAR_REVERSE_STOP;
	} else {
		car_reverse->debug = CAR_REVERSE_START; //CAR_REVERSE_START;
	}

	queue_work(car_reverse->preview_workqueue, &car_reverse->status_detect);
	dev_info(&pdev->dev, "car reverse module probe ok\n");
	return 0;
_err_free_buffer:

_err_out:
	dev_err(&pdev->dev, "car reverse module exit, errno %d!\n", retval);
	return retval;
}

static int car_reverse_remove(struct platform_device *pdev)
{
	struct car_reverse_private_data *priv = car_reverse;
	int i;

	car_reverse_switch_unregister();

	for (i = 0; i < ARRAY_SIZE(car_reverse_attrs); i++) {
		class_remove_file(car_reverse_class, &car_reverse_attrs[i]);
	}
	class_destroy(car_reverse_class);

	// free_irq(gpio_to_irq(priv->reverse_gpio), pdev);
	cancel_work_sync(&priv->status_detect);
	if (priv->preview_workqueue != NULL) {
		flush_workqueue(priv->preview_workqueue);
		destroy_workqueue(priv->preview_workqueue);
		priv->preview_workqueue = NULL;
	}
#ifdef USE_SUNXI_DI_MODULE

	di_client_destroy(car_client);
	car_client = NULL;
#endif
	dev_info(&pdev->dev, "car reverse module exit\n");
	return 0;
}

static int car_reverse_suspend(struct device *dev)
{
	int retval;
	if (car_reverse->status == CAR_REVERSE_START) {
		car_reverse->standby = 1;

#ifdef USE_SUNXI_DI_MODULE
		di_client_destroy(car_client);
		car_client = NULL;
#endif
		logerror("car_reverse_suspend\n");
		retval = car_reverse_preview_stop();
		flush_workqueue(car_reverse->preview_workqueue);
	} else {
		car_reverse->standby = 0;
	}
	return 0;
}

static int car_reverse_resume(struct device *dev)
{
	if (car_reverse->standby) {
		car_reverse->standby = 0;
		queue_work(car_reverse->preview_workqueue,
			&car_reverse->status_detect);
	}
	return 0;
}

static const struct dev_pm_ops car_reverse_pm_ops = {
	.suspend = car_reverse_suspend, .resume = car_reverse_resume,
};

static const struct of_device_id car_reverse_dt_ids[] = {
	{.compatible = "allwinner,sunxi-car-reverse"}, {},
};

static struct platform_driver car_reverse_driver = {
	.probe = car_reverse_probe,
	.remove = car_reverse_remove,
	.driver = {
		.name = MODULE_NAME,
		.pm = &car_reverse_pm_ops,
		.of_match_table = car_reverse_dt_ids,
	},
};

static int __init car_reverse_module_init(void)
{
	int ret;

	ret = platform_driver_register(&car_reverse_driver);
	if (ret) {
		pr_err("platform driver register failed\n");
		return ret;
	}
	return 0;
}

static void __exit car_reverse_module_exit(void)
{
	platform_driver_unregister(&car_reverse_driver);
}

late_initcall(car_reverse_module_init);
module_exit(car_reverse_module_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Zeng.Yajian <zengyajian@allwinnertech.com>");
MODULE_DESCRIPTION("Sunxi fast car reverse image preview");
