cmake_minimum_required(VERSION 3.5)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(CMAKE_SIZE arm-none-eabi-size)
find_program(GDB NAMES arm-none-eabi-gdb gdb-multiarch)

project(AAU-Racing C ASM)

# Following flags helps with dead code elimination.
# More info can found at http://stackoverflow.com/a/10809541
set(gc_flags "-ffunction-sections -fdata-sections -Wl,--gc-sections")
set(mcu_flags "-mcpu=cortex-m4 -mtune=cortex-m4 -mthumb -mlittle-endian -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb-interwork")
set(linker_flags "-Wl,-gc-sections -Wl,-LTO -nostartfiles -Wl,-Map=./result.map -lc -lrdimon -u _printf_float")
set(WARNINGS "-Wall -Wextra")
set(CFLAGS "-O0 -g3 -LTO -std=c11 ${WARNINGS} ${gc_flags} ${mcu_flags} ${linker_flags}")
set(CMAKE_C_FLAGS "${CFLAGS} ${CMAKE_C_FLAGS}")
add_definitions(-DLSE_VALUE=8000000 -DSTM32L432xx -DUSE_USB_FS)

include_directories(
	# Root
	${CMAKE_SOURCE_DIR}

	# CMSIS
	libs/CMSIS/Include
	libs/CMSIS/stm32l432xx

	# STM32 HAL
	libs/STM32L4xx_HAL_Driver/Inc

	# STM32 USB
	libs/STM32_USB_Device_Library/Class/CDC/Inc
	libs/STM32_USB_Device_Library/Core/Inc

  # FatFs
  libs/FatFs
  libs/FatFs/drivers
)

# Make it possible to Flash using openOCD
find_program(OPEN_OCD openocd)
get_filename_component(open_ocd_path ${OPEN_OCD} DIRECTORY)
set(OPEN_OCD_CONFIG "${open_ocd_path}/../share/openocd/scripts/board/stm32l4discovery.cfg")
function(open_ocd_write_flash elf_file)
	add_custom_target(${elf_file}_writeflash
		COMMAND ${OPEN_OCD}
			-f ${OPEN_OCD_CONFIG}
			-c "init; reset halt"
			-c "program ${elf_file}"
			-c "reset run; exit;"
		DEPENDS ${elf_file}
		VERBATIM
	)
endfunction()

function(open_ocd_start_debug elf_file)
	add_custom_target(${elf_file}_start_debug
		COMMAND ${OPEN_OCD}
			-f ${OPEN_OCD_CONFIG}
			-c "init; reset halt"
		DEPENDS ${elf_file}
		VERBATIM
	)
endfunction()

function(start_gdb elf_file)
	add_custom_target(${elf_file}_gdb
		COMMAND ${GDB} -ex "target remote localhost:3333" -ex "monitor reset halt" -ex "load" ${elf_file}
		VERBATIM
	)
endfunction()


add_subdirectory(board_driver)
add_subdirectory(shield_driver)
add_subdirectory(libs)
add_subdirectory(bootloader)

# The directory 'nodes' must be added last since it overides
# the 'add_executable' and 'add_library' functions.
add_subdirectory(nodes)

add_custom_target(rebuild
	COMMAND cmake -E echo
	COMMENT "asdsad"
)
