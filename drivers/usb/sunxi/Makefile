include $(TOPDIR)/config.mk

LIB     := $(obj)libsunxi.o

COBJS-$(CONFIG_USB_SUNXI) += core.o gadget.o ep0.o
COBJS-$(CONFIG_USB_SUNXI) += misc.o sunxi_phy.o

COBJS   := $(COBJS-y)
SRCS    := $(COBJS:.o=.c)
OBJS    := $(addprefix $(obj),$(COBJS))

all:    $(LIB)

$(LIB): $(obj).depend $(OBJS)
	$(call cmd_link_o_target, $(OBJS))

include $(SRCTREE)/rules.mk
sinclude $(obj).depend
