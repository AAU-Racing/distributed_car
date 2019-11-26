add_library(shield_driver
	#mainboard/sd.c
	#mainboard/sd_setup.c
	mainboard/current_clamps.c
	mainboard/error.c
	mainboard/water_temp.c
	mainboard/wheel_sensor.c
	mainboard/gear_feedback.c
	mainboard/ignition_cut.c
	mainboard/gear.c
	mainboard/oil_pressure.c
	mainboard/neutral.c
	mainboard/actuator.c
	mainboard/current_clamps.c
	mainboard/power.c

	dashboard/buttons.c
	dashboard/steering_wheel_buttons.c
	dashboard/rpm.c
	dashboard/gear.c
	dashboard/oil.c
	dashboard/sensor_data.c
	dashboard/TLC59116.c
	dashboard/ea_dogxl160w-7.c
	dashboard/Fonts/font8.c
	dashboard/Fonts/font12.c
	dashboard/Fonts/font16.c
	dashboard/Fonts/font20.c
	dashboard/Fonts/font24.c

	ecu/ignition.c

	devboard/potmeter.c
	devboard/TLC59116.c
)

target_link_libraries(shield_driver
	CMSIS
	STM32l4xx_HAL_Driver
	#STM32_USB_Device_Library
	board_driver
	#fatfs
	m
)
