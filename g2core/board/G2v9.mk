# ----------------------------------------------------------------------------
# This file is part of the Synthetos g2core project


# To compile:
#   make BOARD=g2v9k

# You can also choose a CONFIG from boards.mk:
#   make CONFIG=ShapeokoDualY BOARD=g2v9k



##########
# BOARDs for use directly from the make command line (with default settings) or by the CONFIGS above.

ifeq ("$(BOARD)","g2v9i")
	BASE_BOARD=g2v9
	DEVICE_DEFINES += MOTATE_BOARD="G2v9i"
	DEVICE_DEFINES += SETTINGS_FILE=${SETTINGS_FILE}
endif

ifeq ("$(BOARD)","g2v9k")
	BASE_BOARD=g2v9
	DEVICE_DEFINES += MOTATE_BOARD="G2v9k"
	DEVICE_DEFINES += SETTINGS_FILE=${SETTINGS_FILE}
endif


##########
# The general g2v9 BASE_BOARD.

ifeq ("$(BASE_BOARD)","g2v9")
	_BOARD_FOUND = 1

	FIRST_LINK_SOURCES += $(sort $(wildcard ${MOTATE_PATH}/Atmel_sam_common/*.cpp)) $(sort $(wildcard ${MOTATE_PATH}/Atmel_sam3x/*.cpp))

	# Set CHIP and export it for GDB to see
	CHIP = SAM3X8C
	export CHIP
	CHIP_LOWERCASE = sam3x8c

	BOARD_PATH = ./board/G2v9
	SOURCE_DIRS += ${BOARD_PATH} device/step_dir_driver device/esc_spindle device/sd_card

	PLATFORM_BASE = ${MOTATE_PATH}/platform/atmel_sam

	include $(PLATFORM_BASE).mk

	# Binary link to a LinuxCNC host on J17 (USART2): make ... PI_LINK=1 [PI_LINK_BAUD=2000000]
	# Motion over the link needs PI_LINK_MOTION=1 too; without it every velocity is held at zero.
	ifeq ("$(PI_LINK)", "1")
		DEVICE_DEFINES += PI_LINK=1
		SOURCE_DIRS += device/pi_link
		ifneq ("$(PI_LINK_BAUD)", "")
			DEVICE_DEFINES += PI_LINK_BAUD=$(PI_LINK_BAUD)
		endif
		ifeq ("$(PI_LINK_MOTION)", "1")
			DEVICE_DEFINES += PI_LINK_MOTION=1
		endif
	endif

	ifeq ("$(BANTAM)", "1")
		DEVICE_DEFINES += BANTAM=1
		SOURCE_DIRS += device/bantam_safety_manager
		CUSTOM_LINKER_SCRIPT := board/G2v9/bantam-checksum-flash.ld
		ALL_ADDITIONS += $(OUTPUT_BIN)-summed.bin
define ADDITIONAL_RECIPES
# Note the use of $$$$
$(OUTPUT_BIN)-summed.bin: $(OUTPUT_BIN).bin
	$(QUIET)cp $(OUTPUT_BIN).bin $(OUTPUT_BIN)-summed.bin; \
	printf "%08x" `cksum $(OUTPUT_BIN).bin | awk '{print $$$$1}'` | xxd -r -p | dd of=$(OUTPUT_BIN)-summed.bin bs=1 count=4 seek=256 conv=notrunc; \
	printf "%08x" `wc -c "$(OUTPUT_BIN).bin" |  awk '{print $$$$1}'` | xxd -r -p | dd of=$(OUTPUT_BIN)-summed.bin bs=1 count=4 seek=260 conv=notrunc
endef
endif

endif
