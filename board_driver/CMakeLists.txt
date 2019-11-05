add_custom_target(force_rtc_rebuild ALL
DEPENDS
	rm_rtc_obj
)

# This commands removes the rtc.c.obj so a rebuild og rtc.c is forced.
# This is done to ensure that the correct timestamp is compiled into the program.
add_custom_command(
	OUTPUT rm_rtc_obj
	COMMENT "Removing rtc.c.obj"
	COMMAND rm -f ${CMAKE_SOURCE_DIR}/build/board_driver/CMakeFiles/board_driver.dir/rtc.c.obj
)

add_library(board_driver
	init.c
	uart.c
	adc.c
	bkpsram.c
	flash.c
	can.c
	rng.c
	obdii.c
	usb/usbd_cdc_interface.c
	usb/usbd_conf.c
	usb/usbd_desc.c
	usb/usb.c
	gpio.c
	pwm.c
	i2c.c
	spi.c
	rtc.c
	crc.c
    iwdg.c
	TLC59116.c
)

add_dependencies(board_driver
	force_rtc_rebuild
)

target_link_libraries(board_driver
	CMSIS
	STM32f4xx_HAL_Driver
	STM32_USB_Device_Library
)

target_compile_options(board_driver PRIVATE -Wno-unused-parameter) # TODO remove this and fix the warnings
