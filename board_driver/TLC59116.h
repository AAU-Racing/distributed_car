#ifndef TLC59116_BOARD_H
#define TLC59116_BOARD_H

#include <stdbool.h>
#include <stdint.h>

#define ADDR(A3, A2, A1, A0) ((1 << 7) | (1 << 6) | (0 << 5) | (A3 << 4) | (A2 << 3) | (A1 << 2) | (A0 << 1) | 0) // Not used

// Warning because we shift here you cannot just iterate on the address
typedef enum {
	ADDR1 = ADDR(0,0,0,0),
	ADDR2 = ADDR(0,0,0,1),
	ADDR3 = ADDR(0,0,1,0),
	ADDR4 = ADDR(0,0,1,1),
	ADDR5 = ADDR(0,1,0,0),
	ADDR6 = ADDR(0,1,0,1),
	ADDR7 = ADDR(0,1,1,0),
	ADDR8 = ADDR(0,1,1,1),
	ADDR9 = ADDR(1,0,0,0),
	ADDR10 = ADDR(1,0,0,1),
	ADDR11 = ADDR(1,0,1,0),
	ADDR12 = ADDR(1,0,1,1),
	ADDR13 = ADDR(1,1,0,0),
	ADDR14 = ADDR(1,1,0,1),
} address;

int led_driver_init_base(bool);
void set_rgb(address addr, unsigned led, unsigned r, unsigned g, unsigned b);
address controller_lut(uint8_t x);
void init_controller(address ctrl);

#endif /* TLC59116_BOARD_H */
