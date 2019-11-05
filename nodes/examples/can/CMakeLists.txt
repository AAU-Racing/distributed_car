set(target_name example_can1)

set(SRC_FILES
	main1.c
)

add_executable(${target_name} ${SRC_FILES})
target_link_libraries(${target_name}
	board_driver
	shield_driver
)

set(target_name example_can2)

set(SRC_FILES
	main2.c
)

add_executable(${target_name} ${SRC_FILES})
target_link_libraries(${target_name}
	board_driver
	shield_driver
)
