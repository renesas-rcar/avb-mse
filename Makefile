#
# Makefile for the Renesas device drivers.
#

ifndef CONFIG_AVB_MSE
CONFIG_MSE_CORE ?= m
CONFIG_MSE_ADAPTER_EAVB ?= m
CONFIG_MSE_ADAPTER_ALSA ?= m
CONFIG_MSE_ADAPTER_V4L2 ?= m
CONFIG_MSE_ADAPTER_MCH ?= m

CONFIG_MSE_IOCTL ?= y

CONFIG_MSE_PACKETIZER_AAF ?= y
CONFIG_MSE_PACKETIZER_IEC61883_4 ?= y
CONFIG_MSE_PACKETIZER_IEC61883_6 ?= y
CONFIG_MSE_PACKETIZER_CVF_H264 ?= y
CONFIG_MSE_PACKETIZER_CVF_H264_SINGLE_NAL ?= y
CONFIG_MSE_PACKETIZER_CVF_MJPEG ?= y

ccflags-$(CONFIG_MSE_IOCTL) += -DCONFIG_MSE_IOCTL

ccflags-$(CONFIG_MSE_PACKETIZER_AAF) += -DCONFIG_MSE_PACKETIZER_AAF
ccflags-$(CONFIG_MSE_PACKETIZER_IEC61883_4) += -DCONFIG_MSE_PACKETIZER_IEC61883_4
ccflags-$(CONFIG_MSE_PACKETIZER_IEC61883_6) += -DCONFIG_MSE_PACKETIZER_IEC61883_6
ccflags-$(CONFIG_MSE_PACKETIZER_CVF_H264) += -DCONFIG_MSE_PACKETIZER_CVF_H264
ccflags-$(CONFIG_MSE_PACKETIZER_CVF_H264_SINGLE_NAL) += -DCONFIG_MSE_PACKETIZER_CVF_H264_SINGLE_NAL
ccflags-$(CONFIG_MSE_PACKETIZER_CVF_MJPEG) += -DCONFIG_MSE_PACKETIZER_CVF_MJPEG
endif

INCSHARED ?= drivers/staging/avb-streaming \
             drivers/staging/avb-mch

comma := ,
ccflags-y += $(addprefix -I,$(subst $(comma), ,$(INCSHARED)))

mse_core-objs := mse_core_main.o \
                 mse_packet_ctrl.o \
                 mse_config.o \
                 mse_sysfs.o \
                 avtp.o \
                 mse_packetizer.o \
                 mse_packetizer_crf.o \
                 mse_ptp_dummy.o
# configration interface
mse_core-$(CONFIG_MSE_IOCTL) += mse_ioctl.o
# packetizer objects
mse_core-$(CONFIG_MSE_PACKETIZER_AAF) += mse_packetizer_aaf.o
mse_core-$(CONFIG_MSE_PACKETIZER_IEC61883_4) += mse_packetizer_iec61883_4.o
mse_core-$(CONFIG_MSE_PACKETIZER_IEC61883_6) += mse_packetizer_iec61883_6.o
mse_core-$(CONFIG_MSE_PACKETIZER_CVF_H264) += mse_packetizer_cvf_h264.o
mse_core-$(CONFIG_MSE_PACKETIZER_CVF_MJPEG) += mse_packetizer_cvf_mjpeg.o jpeg.o
obj-$(CONFIG_MSE_CORE) += mse_core.o

# adapter
obj-$(CONFIG_MSE_ADAPTER_EAVB) += mse_adapter_eavb.o
obj-$(CONFIG_MSE_ADAPTER_ALSA) += mse_adapter_alsa.o
obj-$(CONFIG_MSE_ADAPTER_V4L2) += mse_adapter_v4l2.o
obj-$(CONFIG_MSE_ADAPTER_MCH)  += mse_adapter_mch.o

ifndef CONFIG_AVB_MSE
SRC := $(shell pwd)

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC)

%:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) $@

endif
