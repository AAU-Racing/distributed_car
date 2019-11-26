#include <stdbool.h>
#include <string.h>

#include "i2c.h"
#include "gpio.h"

#define BUFFER_SIZE 256
#define WRITE 0
#define TIMINGR_CLEAR_MASK 0xF0FFFFFFU

typedef struct {
	uint16_t addr;
	uint8_t buf[32];
	size_t n;
} i2c_msg;

static I2C_TypeDef *handle;

static void init_sda_pin() {
	gpio_af_init(I2C_SDA_GPIO_PORT, I2C_SDA_PIN, GPIO_HIGH_SPEED, GPIO_OPENDRAIN, I2C_SDA_AF);
}

static void init_scl_pin() {
	gpio_af_init(I2C_SCL_GPIO_PORT, I2C_SCL_PIN, GPIO_HIGH_SPEED, GPIO_OPENDRAIN, I2C_SCL_AF);
}

static void i2c_enable() {
	SET_BIT(handle->CR1, I2C_CR1_PE);
}

static void i2c_set_timing(){
	handle->TIMINGR = 0x10909CEC & TIMINGR_CLEAR_MASK;
}

static void i2c_start_clock(){
	SET_BIT(RCC->APB1ENR1, RCC_APB1ENR1_I2C1EN);
}

int i2c_init(void) {
  init_sda_pin();
  init_scl_pin();

	handle = I2C1;
  i2c_start_clock();
	i2c_set_timing();
	i2c_enable();

	return 0;
}

int i2c_is_ready(uint16_t addr) {
	return 1;
}

static void start_condition(){
	SET_BIT(handle->CR2, I2C_CR2_START);
}

static void set_slave_addr(uint8_t addr) {
	MODIFY_REG(handle->CR2, I2C_CR2_SADD_Msk, addr << 1);
}

static void set_write() {
	CLEAR_BIT(handle->CR2, I2C_CR2_RD_WRN);
}

static void set_n_bytes(size_t n) {
	MODIFY_REG(handle->CR2, I2C_CR2_NBYTES_Msk, n << I2C_CR2_NBYTES_Pos);
}

static void wait_for_dr_empty(){
	//I2C_FLAG_TXIS: Data register empty flag (1 means empty)
	while(READ_BIT(handle->ISR, I2C_ISR_TXIS) == RESET)
		;
}

static void transmit_byte(uint8_t byte) { // Write data to DR
	handle->TXDR = byte;
}

static void wait_for_byte_transfer_finished() {
	//I2C_FLAG_TC: Transfer complete flag (1 means finished)
	while(READ_BIT(handle->ISR, I2C_ISR_TC) == RESET)
		;
}

static void stop_condition(){
	SET_BIT(handle->CR2, I2C_CR2_STOP);
}

int i2c_master_transmit(uint16_t addr, uint8_t *buf, size_t n) { // No DMA
	set_slave_addr(addr);
	set_write();
	set_n_bytes(n);

	start_condition();

	for (uint8_t i = 0; i < n; i++) {
		wait_for_dr_empty();
		transmit_byte(buf[i]);
	}

	wait_for_dr_empty();
	wait_for_byte_transfer_finished();
	stop_condition();

	return 0;
}
