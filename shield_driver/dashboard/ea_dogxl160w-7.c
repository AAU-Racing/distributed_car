#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

#include <stm32f4xx_hal.h>
#include <board_driver/spi.h>

#include "ea_dogxl160w-7.h"
#include "Fonts/fonts.h"
//#include "TLC59116.h"

#define COLUMNS	(PIXELS_X_AXIS)
#define PAGES	(PIXELS_Y_AXIS/4)


static void init_GPIO_pins(void) {
	// CS
	LCD_CS_GPIO_CLK_ENABLE();
	HAL_GPIO_Init(LCD_CS_GPIO_PORT, &(GPIO_InitTypeDef) {
		.Pin = LCD_CS_PIN,
		.Mode = GPIO_MODE_OUTPUT_PP,
		.Pull = GPIO_PULLUP,
		.Speed = GPIO_SPEED_FAST,
	});

	// CD
	LCD_CD_GPIO_CLK_ENABLE();
	HAL_GPIO_Init(LCD_CD_GPIO_PORT, &(GPIO_InitTypeDef) {
		.Pin = LCD_CD_PIN,
		.Mode = GPIO_MODE_OUTPUT_PP,
		.Pull = GPIO_PULLUP,
		.Speed = GPIO_SPEED_FAST,
	});
}

#define ARR_LEN(a) ( sizeof(a)/sizeof(a[0]) )

static void cmd_write_data_byte(uint8_t d) {
	LCD_CD_ON();
	SPI_transmit(&d, 1);
}

static void cmd_set_column_address(uint8_t c_addr) {
	const uint8_t lsb = c_addr & 0x0F;
	const uint8_t msb = (c_addr & 0xF0) >> 4;

	uint8_t payload[2] = {
		lsb,
		(1 << 4) | msb,
	};

	LCD_CD_OFF();
	SPI_transmit(payload, 2);
}

static void cmd_set_page_address(uint8_t p_addr) {
	p_addr |= 1 << 5;
	p_addr |= 1 << 6;

	LCD_CD_OFF();
	SPI_transmit(&p_addr, 1);
}

__attribute__ ((unused)) static void cmd_set_contrast(uint8_t contrast) {
	static uint8_t cmd = 0x81;
	uint8_t payload[2] = {cmd, contrast};

	LCD_CD_OFF();
	SPI_transmit(&payload, 2);
}

static void cmd_system_reset() {
	static uint8_t cmd = 0xE2;

	LCD_CD_OFF();
	SPI_transmit(&cmd, 1);
}

uint8_t frame_buffer[COLUMNS][PAGES];

static void setPixel(int x, int y, bool paint, bool clear) {
	const uint8_t bits_pr_pixel = 2;
	const uint8_t column_addr = x;
	const uint8_t page_addr = (y / 4); // 4 pixels per column
	const uint8_t page_offset = (y % 4) * bits_pr_pixel;

	const uint8_t val = ((1<<0)|(1<<1)) << page_offset;

	if (column_addr > COLUMNS || page_addr > PAGES) return;

	if (!clear) {
		frame_buffer[column_addr][page_addr] |= val;
	} else {
		frame_buffer[column_addr][page_addr] &= ~val;
	}
	if (paint) {
		cmd_set_column_address(column_addr);
		cmd_set_page_address(page_addr);
		cmd_write_data_byte(frame_buffer[column_addr][page_addr]);
	}
}

// see https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
// and http://rosettacode.org/wiki/Bitmap/Bresenham%27s_line_algorithm#C
void line(int x0, int y0, int x1, int y1, bool paint) {
	int dx = abs(x1-x0), sx = (x0 < x1) ? 1 : -1;
	int dy = abs(y1-y0), sy = (y0 < y1) ? 1 : -1;
	int err = ((dx > dy) ? dx : -dy) / 2, e2;

	while (1) {
		setPixel(x0, y0, paint, false);
		if (x0 == x1 && y0 == y1) break;
		e2 = err;
		if (e2 >-dx) { err -= dy; x0 += sx; }
		if (e2 < dy) { err += dx; y0 += sy; }
	}
}

void paint(void) {
	cmd_set_column_address(0);
	cmd_set_page_address(0);
	for (size_t j = 0; j < PAGES; j++) {
		for (size_t i = 0; i < COLUMNS; i++) {
			cmd_write_data_byte(frame_buffer[i][j]);
		}
	}
}

void clear(void) {
	memset(frame_buffer, 0, COLUMNS*PAGES);
	paint();
}

// use imagemagick to generate bitmapfonts
// convert  -font "$font_path"  -pointsize $point_size +antialias label:$i $i.xbm
// where $i is ascii index
// see http://ceronio.net/2009/07/bitmap-font-tileset-with-ruby-and-imagemagick/
void paintChar(int x_pos, int y_pos, char c, int scale, sFONT *font) {
	uint16_t width = font->Width;
	uint16_t height = font->Height;

	const uint16_t byte_width = (width / 8) + 1;
	const uint8_t  unused_bits = (8*byte_width) - width;

	const size_t char_offset = (c - ' ') * height * byte_width;
	const uint8_t *table = &font->table[char_offset];

	if (scale < 1) scale = 1;
	int scale_x_offset = 0;
	int scale_y_offset = 0;

	for (size_t h = 0; h < (size_t)(height * byte_width); h+= byte_width) {
		uint32_t row = 0;
		switch (byte_width) {
			case 1: row = table[h]; break;
			case 2: row = (table[h] << 8)  | (table[h+1]); break;
			case 3: row = (table[h] << 16) | (table[h+1] << 8) | (table[h+2]); break;
		}
		row >>= unused_bits;

		for (size_t w = 0; w < width; w++) {
			const size_t w_msb = width-w-1;
			// const int x = (x_pos + w);
			// const int y = (y_pos + (h/byte_width));
			const int x = x_pos + scale_x_offset;
			const int y = y_pos + scale_y_offset;

			const bool pixel_state = row & (1 << w_msb);
				for (int i = 0; i < scale; i++) {
					for (int j = 0; j < scale; j++) {
						setPixel(x + i, y + j, true, !pixel_state);
					}
				}
			scale_x_offset += scale;
		}
		scale_y_offset += scale;
		scale_x_offset = 0;
	}
}

void paintString(int x_pos, int y_pos, char *str, int scale, sFONT *font) {
	if (scale < 1) scale = 1;
	const uint16_t h = font->Height * scale;
	const uint16_t w = font->Width * scale;
	int line_len = 0;
	while(*str != '\0') {
		char c = *str++;

		if (c == '\n') {
			y_pos += h;
			x_pos -= line_len * w;
			line_len = 0;
		} else {
			paintChar(x_pos, y_pos, c, scale, font);
			x_pos += w;
			line_len++;
		}
	}
}

void init_lcd(void) {
	//set_led(0, 0, 0, 0xF);
	init_GPIO_pins();
	//set_led(1, 0, 0, 0xF);
	SPI_init();
	//set_led(2, 0, 0, 0xF);

	cmd_system_reset();
	//set_led(3, 0, 0, 0xF);

	HAL_Delay(1000);

	uint8_t init_seq[] = {
		0xF1, 0x67, 0xC0, 0x40, 0x50, 0x2B, 0xEB, 0x81, 0x5F, 0x89, 0xAF,
	};
	LCD_CD_OFF();

	//set_led(4, 0, 0, 0xF);

	for (uint8_t i = 0; i < ARR_LEN(init_seq); i++) {
		SPI_transmit(init_seq + i, 1);
		HAL_Delay(100);
	}

	//SPI_transmit(init_seq, ARR_LEN(init_seq));

	clear();
}
