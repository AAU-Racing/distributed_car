#ifndef CAN_GUARD
#define CAN_GUARD

#include <stm32f4xx_hal.h>
#include <stdbool.h>

#include "gpio.h"

typedef struct {
    uint32_t last_receive;
    uint32_t transmit;
    uint32_t receive;
    uint32_t error_total;
    uint32_t error_ewg; // Error warning
    uint32_t error_epv; // Error passive (when transmit error counter, TEC, reaches 127)
    uint32_t error_bof; // Bus off (when TEC reaches 255)
    uint8_t error_stuff; // Bit stuffing error
    uint8_t error_form; // Form error (When fixed format bits are not correct)
    uint8_t error_ack; // Acknowledgement error
    uint8_t error_recess; // Bit recessive error (if monitoring opposite bit than transmitted after RTR)
    uint8_t error_dominant; // Bit dominant error (if monitoring opposite bit than transmitted after RTR)
    uint8_t error_crc; // CRC error
} CAN_Statistics;

typedef struct {
    uint16_t StdId;
    uint8_t FMI;
    uint8_t Length;
    uint8_t Msg[8];
} CAN_RxFrame;

typedef void (*CAN_RX_Callback) (CAN_RxFrame *msg);

#define	CAN_OK            0U
#define CAN_DRIVER_ERROR  1U
#define CAN_INVALID_ID    2U
#define	CAN_INVALID_FRAME 3U
#define CAN_INIT_TIMEOUT  4U
#define CAN_BUFFER_FULL   5U

// Definitions for CAN configurations
#define CAN_PA11 	0
#define CAN_PB5 	1
#define CAN_PB8		2
#define CAN_PB12	3
#define CAN_PD0 	4

// Definitions for ranges of ids for data and their individual ids
#define CAN_FATAL_INFO_ID_START 	0x000
#define CAN_OIL_PRESSURE 			0x000
#define CAN_FATAL_INFO_ID_END 		0x0ff
#define CAN_FATAL_INFO_MASK			0x780

#define CAN_MISSION_CRITICAL_ID_START 	0x1c0
#define CAN_GEAR_BUTTONS 				0x1d0
#define CAN_GEAR_NUMBER					0x1e0
#define CAN_GEAR_CHANGE_FAILED          0x1e1
#define CAN_IGNITION_CUT				0x1f0
#define CAN_DASHBOARD_STARTED			0x1f1
#define CAN_MAIN_BOARD_STARTED			0x1f2
#define CAN_MISSION_CRITICAL_ID_END 	0x1ff
#define CAN_MISSION_CRITICAL_MASK		0x7c0

#define CAN_ERROR_HANDLE_ID_START 	0x240
#define CAN_SPI_ERROR				0x242
#define CAN_UART_ERROR				0x244
#define CAN_I2C_ERROR				0x246
#define CAN_TELEMETRY_ERROR			0x248
#define CAN_PWM_ERROR				0x250
#define CAN_GPS_ERROR				0x252
#define CAN_SD_ERROR				0x254
#define CAN_LCD_ERROR				0x256
#define	CAN_LED_ERROR				0x258
#define CAN_ADC_ERROR				0x260
#define CAN_ERROR					0x262
#define CAN_ERROR_HANDLE_ID_END		0x27f
#define CAN_ERROR_HANDLE_MASK		0x7c0

#define CAN_SENSOR_DATA_ID_START 	0x400
#define CAN_FUEL_TEMPERATURE		0x400
#define CAN_WATER_TEMPERATURE_IN 	0x4b0
#define CAN_WATER_TEMPERATURE_OUT 	0x4b1
#define CAN_GEAR_FEEDBACK 			0x4b2
#define CAN_WHEEL_SPEED_RL 			0x4b4
#define CAN_WHEEL_SPEED_RR 			0x4b5
#define CAN_WHEEL_SPEED_FL 			0x4b6
#define CAN_WHEEL_SPEED_FR 			0x4b7
#define CAN_BRAKE_PRESSURE_FRONT 	0x4c0
#define CAN_BRAKE_PRESSURE_REAR		0x4c1
#define CAN_SPEEDER_POSITION		0x4c2
#define CAN_CLUTCH_POSITION			0x4c3
#define CAN_GYRO_X					0x4d0
#define CAN_GYRO_Y					0x4d1
#define CAN_GYRO_Z					0x4d2
#define CAN_SUSPENSION_TRAVEL_RL	0x4e0
#define CAN_SUSPENSION_TRAVEL_RR	0x4e1
#define CAN_SUSPENSION_TRAVEL_FL	0x4e2
#define CAN_SUSPENSION_TRAVEL_FR	0x4e3
#define CAN_STEERING_WHEEL_POSITION	0x4f0
#define CAN_BATTERY_CURRENT			0x4f2
#define CAN_INVERTER_CURRENT		0x4f3
#define CAN_NEUTRAL_SWITCH          0x4f4
#define CAN_SENSOR_DATA_ID_END 		0x4ff
#define CAN_SENSOR_DATA_MASK        0x700

#define CAN_OBD_ID_START 	0x7df
#define CAN_OBD_ID_END 		0x7ff

// Definitions for discrete valued data
#define CAN_GEAR_BUTTON_UP 		10
#define CAN_GEAR_BUTTON_DOWN 	11
#define CAN_GEAR_BUTTON_NEUTRAL 12
#define CAN_GEAR_BUTTON_OVERRIDE_NEUTRAL 100
#define CAN_GEAR_BUTTON_OVERRIDE_1       101
#define CAN_GEAR_BUTTON_OVERRIDE_2       102
#define CAN_GEAR_BUTTON_OVERRIDE_3       103
#define CAN_GEAR_BUTTON_OVERRIDE_4       104
#define CAN_GEAR_BUTTON_OVERRIDE_5       105
#define CAN_GEAR_BUTTON_OVERRIDE_6       106

#define CAN_OIL_PRESSURE_ON		1
#define CAN_OIL_PRESSURE_OFF	0

#define CAN_ADC_INIT_ERROR				1
#define CAN_ADC_START_ERROR				2
#define CAN_ADC_WATER_TEMP_ERROR 		3
#define CAN_ADC_GEAR_ERROR				4
#define CAN_ADC_CURRENT_CLAMPS_ERROR	5

#define CAN_NODE_COM_NODE_STARTED			0
#define CAN_NODE_TRACTION_CONTROL_STARTED	1
#define CAN_NODE_DASHBOARD_STARTED			2

uint8_t can_transmit(uint16_t id, uint8_t msg[], uint8_t length);
// !!!!!!!!!!!!!!!! Printf is not allow inside the callback !!!!!!!!!!!!!!!
uint8_t can_filter(uint16_t id, uint16_t mask, CAN_RX_Callback callback);
uint8_t can_start();
uint8_t can_init(uint8_t config);
CAN_Statistics can_get_stats();

#endif /* CAN_GUARD */
