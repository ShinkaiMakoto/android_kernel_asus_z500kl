#GT928 touchpanel driver

#obj-$(CONFIG_TOUCHSCREEN_GT6108)         += gt6108.o
obj-y         += gt6108.o
obj-y         += gt6108_asus.o
obj-y         += gt6108_tool.o
obj-y         += gt6108_update.o
obj-y         += gt6108_openshort.o

ifeq ($(TARGET_BUILD_VARIANT),eng)
    KBUILD_CFLAGS += -DTOUCH_ENG_BUILD
endif

