#
# Makefile for the input core drivers.
#

# Each configuration option enables a list of files.
# sensor drivers
ifneq ($(TARGET_SKU),CN)
obj-$(CONFIG_SENSORS_CAP1106)	+= cap1106/
endif
obj-y                           += hall/
