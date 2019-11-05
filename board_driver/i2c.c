#include <stdbool.h>
#include <string.h>

#include "i2c.h"
#include "gpio.h"

#define BUFFER_SIZE 256
#define WRITE 0

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

static void i2c_freqrange(){
	handle->CR2 = 40; // 40 MHz
}

static void i2c_rise_time(){
	handle->TRISE = 0xD;
}

static void i2c_speed(){
	SET_BIT(handle->CCR, I2C_CCR_FS);
	SET_BIT(handle->CCR, I2C_CCR_DUTY);
	MODIFY_REG(handle->CCR, I2C_CCR_CCR_Msk, 4);
}

static void i2c_start_clock(){
	SET_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C2EN);
}

int i2c_init(void) {
    init_sda_pin();
    init_scl_pin();

	handle = I2C2;
    i2c_start_clock();
	i2c_freqrange();
	i2c_speed();
	i2c_rise_time();
	i2c_enable();

	return 0;
}

int i2c_is_ready(uint16_t addr) {
	return 1;
}

static void wait_for_start_generation() {
	// I2C_SR1_SB: 1 means start sequence was generated
	while(READ_BIT(handle->SR1, I2C_SR1_SB) == RESET) {}
}

static void start_condition(){
	SET_BIT(handle->CR1, I2C_CR1_START);
	wait_for_start_generation();
}

static void wait_for_addr_sent() {
	// I2C_SR1_ADDR: 1 means address was sent
	while(READ_BIT(handle->SR1, I2C_SR1_ADDR) == RESET) {}
}

static void clear_addr_sent() {
	handle->SR1;
	handle->SR2;
}

static void set_slave_addr(uint8_t addr) {
	// Clear EV5 by reading SR1 register
	handle->SR1;

	// Not for correct placement
	addr = addr & ~1;

	// Set slave address
	handle->DR = addr | WRITE;

	wait_for_addr_sent();
	clear_addr_sent();
}

static void wait_for_dr_empty(){
	//I2C_FLAG_TXE: Data register empty flag (1 means empty)
	while(READ_BIT(handle->SR1, I2C_SR1_TXE) == RESET) {}
}

static void transmit_byte(uint8_t byte) { // Write data to DR
	handle->DR = byte;
}

static void wait_for_byte_transfer_finished() {
	//I2C_FLAG_BTF: Byte transfer finished flag (1 means finished)
	while(READ_BIT(handle->SR1, I2C_SR1_BTF) == RESET) {}
}

static void stop_condition(){
	handle->CR1 |= I2C_CR1_STOP;
}

int i2c_master_transmit(uint16_t addr, uint8_t *buf, size_t n) { // No DMA

	start_condition();
	set_slave_addr(addr);

	for (uint8_t i = 0; i < n; i++) {
		wait_for_dr_empty();
		transmit_byte(buf[i]);
	}

	wait_for_dr_empty();
	wait_for_byte_transfer_finished();
	stop_condition();

	return 0;
}
