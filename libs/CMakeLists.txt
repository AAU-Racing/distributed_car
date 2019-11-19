######## HAL ##########
file(GLOB src STM32L4xx_HAL_Driver/Src/*.c)
add_library(STM32l4xx_HAL_Driver ${src})

# There is a lot of unused variable warnings in some hal function-prototypes
target_compile_options(STM32l4xx_HAL_Driver PRIVATE -Wno-extra -Wno-pedantic)


######## CMSIS ########
add_library(CMSIS
	CMSIS/stm32l432xx/startup_stm32l432xx.s
	CMSIS/system_stm32l4xx.c
)

# There is a lot of unused variable warnings in some hal function-prototypes
target_compile_options(CMSIS PRIVATE -Wno-extra -Wno-pedantic)

######## USB DEVICE LIBRARY ########
#add_subdirectory(STM32_USB_Device_Library)
# add_library(STM32_USB_Device_Library
# 	STM32_USB_Device_Library/Core/Src/usbd_core.c
# 	STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
# 	STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
# )
#
# target_compile_options(STM32_USB_Device_Library PRIVATE -Wno-unused-parameter)


######## FatFs ########
#add_library(fatfs
#	FatFs/src/diskio.c
#	FatFs/src/ff_gen_drv.c
#	FatFs/src/ff.c
#
#	# drivers
#	FatFs/src/drivers/sd_diskio_template.c
#	#FatFs/src/drivers/sdram_diskio_template.c
#	#FatFs/src/drivers/sram_diskio_template.c
#	#FatFs/src/drivers/usbh_diskio_template.c
#
#	# option
#	FatFs/src/option/cc932.c
#	FatFs/src/option/cc936.c
#	FatFs/src/option/cc949.c
#	FatFs/src/option/cc950.c
#	FatFs/src/option/ccsbcs.c
#	FatFs/src/option/syscall.c
#	FatFs/src/option/unicode.c
#)
#
#target_link_libraries(fatfs
#	CMSIS
#	STM32l4xx_HAL_Driver
#)
