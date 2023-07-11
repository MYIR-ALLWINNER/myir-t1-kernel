PWD = $(shell pwd)
obj-y = sunxi_car_reverse.o

ifeq ($(CONFIG_SUNXI_DI_V3X),y)
di_dir := $(PWD)/../../../kernel/linux-5.4/drivers/char/sunxi-di/drv_div3x
ccflags-y := -I$(di_dir)
endif

ifeq ($(CONFIG_SUNXI_DI_V1XX),y)
di_dir := $(PWD)/../../../kernel/linux-5.4/drivers/char/sunxi-di/drv_div1xx
ccflags-y := -I$(di_dir)
endif

sunxi_car_reverse-y = buffer_pool.o \
	video_source.o \
	preview.o \
	car_reverse.o

ifeq ($(CONFIG_SUPPORT_AUXILIARY_LINE),y)
sunxi_car_reverse-y += auxiliary_line.o
endif

ifeq ($(CONFIG_SUPPORT_BIRDVIEW),y)
sunxi_car_reverse-y += BVKernal/birdview_algo \
			BVKernal/birdview_api \
			BVKernal/camera \
			BVKernal/fisheye_algo
endif

