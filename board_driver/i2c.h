
#include <stm32f4xx.h>

#define I2C                             I2C2

/* Definition for I2C Pins */
#define I2C_SCL_PIN                    PIN_10
#define I2C_SCL_GPIO_PORT              GPIOB
#define I2C_SCL_AF                     GPIO_AF4
#define I2C_SDA_PIN                    PIN_11
#define I2C_SDA_GPIO_PORT              GPIOB
#define I2C_SDA_AF                     GPIO_AF4

int i2c_init(void);
int i2c_master_transmit(uint16_t addr, uint8_t *buf, size_t n);
