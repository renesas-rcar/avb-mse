#
# Makefile for the Renesas device drivers.
#

ifndef CONFIG_AVB_MSE
CONFIG_MSE_CORE ?= m
CONFIG_MSE_ADAPTER_EAVB ?= m
CONFIG_MSE_ADAPTER_ALSA ?= m
CONFIG_MSE_ADAPTER_V4L2 ?= m
CONFIG_MSE_ADAPTER_MCH ?= n
endif

INCSHARED ?= drivers/staging/avb-streaming

ccflags-y += -I$(INCSHARED)

mse_core-objs := mse_core_main.o \
                 mse_packet_ctrl.o \
                 mse_config.o \
                 mse_sysfs.o \
                 mse_ioctl.o \
                 avtp.o \
                 jpeg.o \
                 mse_packetizer.o \
                 mse_packetizer_aaf.o \
                 mse_packetizer_iec61883_6.o \
                 mse_packetizer_cvf_h264.o \
                 mse_packetizer_cvf_mjpeg.o \
                 mse_packetizer_iec61883_4.o \
                 mse_packetizer_crf.o \
                 mse_ptp_dummy.o
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
