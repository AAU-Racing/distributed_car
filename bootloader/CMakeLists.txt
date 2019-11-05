set(target_name bootloader)
set(ldscript_path ${CMAKE_SOURCE_DIR}/bootloader/ldscript.ld)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wl,-T${ldscript_path}")

set(SRC_FILES
	newlib_calls.c
	main.c
)

add_executable(${target_name} ${SRC_FILES})
target_link_libraries(${target_name}
	board_driver
)

open_ocd_write_flash(${target_name})
