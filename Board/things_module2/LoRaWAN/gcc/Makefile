TARGET_CHIP := NRF51822_QFAA_CA
BOARD := BOARD_AIRFY_THINGS

CFLAGS += -DSPI_MASTER_0_ENABLE

C_SOURCE_FILES += main.c
C_SOURCE_FILES += twi_sw_master.c
C_SOURCE_FILES += spi_master.c
C_SOURCE_FILES += nrf_delay.c
C_SOURCE_FILES += nrf_soc.c
C_SOURCE_FILES += sx1276.c
C_SOURCE_FILES += app_timer.c
C_SOURCE_FILES += app_scheduler.c
C_SOURCE_FILES += SEGGER_RTT.c
C_SOURCE_FILES += SEGGER_RTT_printf.c
C_SOURCE_FILES += LoRaMac.c
C_SOURCE_FILES += LoRaMacCrypto.c
C_SOURCE_FILES += aes.c
C_SOURCE_FILES += cmac.c

OUTPUT_FILENAME := LoraWAN
SDK_PATH = ../../../../


#Uncomment correct line if you have s110 programmed on the chip.
DEVICE_VARIANT := xxaa
#DEVICE_VARIANT := xxab

#USE_SOFTDEVICE := s110
#USE_SOFTDEVICE := s210

include $(SDK_PATH)Source/templates/gcc/Makefile.common
INCLUDEPATHS += -I"$(SDK_PATH)Include/app_common"
INCLUDEPATHS += -I"$(SDK_PATH)Include/sd_common"
INCLUDEPATHS += -I"$(SDK_PATH)Include/sdk_soc"
INCLUDEPATHS += -I"$(SDK_PATH)Include/RTT"
INCLUDEPATHS += -I"$(SDK_PATH)Include/sx1276"
#INCLUDEPATHS += -I"$(SDK_PATH)Include/s110"