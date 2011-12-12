#
# Makefile for the kernel multimedia device drivers.
#

obj-y += common/ IR/ video/

obj-$(CONFIG_VIDEO_DEV) += radio/
obj-$(CONFIG_DVB_CORE)  += dvb/
# VenturiGB_Usys_jypark 2011.08.08 - DMB 
ifeq ($(CONFIG_TDMB),y)
obj-$(CONFIG_TDMB)  += tdmb/
endif
