# Note: Makefile is a bit hack-ish. Run ./build.sh to build.

BINARY = fancyblink
LDSCRIPT = embedded-pi.ld

################################################################################
# Platform specific options

LIBNAME		= opencm3_stm32f1
DEFS		= -DSTM32F1

FP_FLAGS	?= -msoft-float
ARCH_FLAGS	= -mthumb -mcpu=cortex-m3 $(FP_FLAGS) -mfix-cortex-m3-ldrd

CFLAGS		+= -std=gnu99

################################################################################
# OpenOCD specific variables

#OOCD		?= openocd
#OOCD_INTERFACE	?= flossjtag
#OOCD_BOARD	?= olimex_stm32_h103

################################################################################
# Black Magic Probe specific variables
# Set the BMP_PORT to a serial port and then BMP is used for flashing
#BMP_PORT	?=

################################################################################
# texane/stlink specific variables
#STLINK_PORT	?= :4242


include Makefile.rules

lib:
	$(Q)if [ ! "`ls -A libopencm3`" ] ; then \
		printf "######## ERROR ########\n"; \
		printf "\tlibopencm3 is not initialized.\n"; \
		printf "\tPlease run:\n"; \
		printf "\t$$ git submodule init\n"; \
		printf "\t$$ git submodule update\n"; \
		printf "\tbefore running make.\n"; \
		printf "######## ERROR ########\n"; \
		exit 1; \
		fi
	$(Q)$(MAKE) -C libopencm3 lib/stm32/f1 


