TARGET_CHIP := NRF51822_QFAA_CA
BOARD := BOARD_AIRFY_THINGS

C_SOURCE_FILES += main.c
C_SOURCE_FILES += simple_uart.c
C_SOURCE_FILES += app_timer.c
C_SOURCE_FILES += nrf_soc.c

OUTPUT_FILENAME := app_timer_example
SDK_PATH = ../../../../


#Uncomment correct line if you have s110 programmed on the chip.
DEVICE_VARIANT := xxaa
#DEVICE_VARIANT := xxab

#USE_SOFTDEVICE := s110
#USE_SOFTDEVICE := s210

include $(SDK_PATH)Source/templates/gcc/Makefile.common
INCLUDEPATHS += -I"$(SDK_PATH)Include/app_common"
INCLUDEPATHS += -I"$(SDK_PATH)Include/sdk_soc"