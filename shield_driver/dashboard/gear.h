#include <stdint.h>
#include <stdbool.h>

typedef enum {
	GEAR_N,
	GEAR_1,
	GEAR_2,
	GEAR_3,
	GEAR_4,
	GEAR_5,
	GEAR_6,
} gear_t;

bool get_gear(gear_t* gear);
int gear_init();
