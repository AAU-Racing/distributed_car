#include <stm32f4xx_hal.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include <board_driver/uart.h>
#include <board_driver/can.h>
#include <board_driver/obdii.h>

#include <shield_driver/dashboard/TLC59116.h>

#define RPM_TIMEOUT 1000
#define THROTTLE_POSITION_TIMEOUT 5

uint32_t lastRPMTick = 0;
uint32_t lastThrottleTick = 0;

void setup() {
	uart_init();

	printf("UART init complete\n");

	if (can_init(CAN_PB5) != CAN_OK) {
		printf("CAN init error\n");
	}

	if (obdii_init() != HAL_OK) {
		printf("CAN filter error\n");
	}

	HAL_Delay(100);

	if (can_start() != CAN_OK){
		printf("CAN start error\n");
	}
	else {
		printf("CAN send started\n");
	}

	// if (obdii_mode1_request(EngineRPM) != HAL_OK) {
	// 	printf("Initial rpm request failed\n");
	// }
	if (obdii_mode1_request(ThrottlePosition) != HAL_OK) {
		printf("Initial throttle position request failed\n");
	}

	// lastRPMTick = HAL_GetTick();
	lastThrottleTick = HAL_GetTick();
}

void loop() {
	// OBDII_Mode1_Frame frame = obdii_mode1_response(EngineRPM);
	//
	// if (frame.New) {
	// 	printf("Pid: %u\n", frame.Pid);
	// 	printf("Length: %u\n", frame.Length);
	// 	printf("Msg:");
	// 	for (uint32_t i = 0; i < frame.Length; i++) {
	// 		printf(" %d", frame.Msg[i]);
	// 	}
	// 	printf("\n");
	//
	// 	lastRPMTick = HAL_GetTick();
	// 	if (obdii_mode1_request(EngineRPM) != HAL_OK) {
	// 		printf("Update rpm request failed\n");
	// 	}
	// }

	OBDII_Mode1_Frame frame = obdii_mode1_response(ThrottlePosition);

	if (frame.New) {
		printf("Pid: %u\n", frame.Pid);
		printf("Length: %u\n", frame.Length);
		printf("Msg:");
		for (uint32_t i = 0; i < frame.Length; i++) {
			printf(" %d", frame.Msg[i]);
		}
		printf("\n");

		lastThrottleTick = HAL_GetTick();
		if (obdii_mode1_request(ThrottlePosition) != HAL_OK) {
			printf("Update throttle position request failed\n");
		}
	}

	// if (lastRPMTick + RPM_TIMEOUT < HAL_GetTick()) {
	// 	lastRPMTick = HAL_GetTick();
	// 	if (obdii_mode1_request(EngineRPM) != HAL_OK) {
	// 		printf("Timeout rpm request failed\n");
	// 	}
	// }

	if (lastThrottleTick + THROTTLE_POSITION_TIMEOUT < HAL_GetTick()) {
		lastThrottleTick = HAL_GetTick();
		if (obdii_mode1_request(ThrottlePosition) != HAL_OK) {
			printf("Timeout throttle position request failed\n");
		}
	}

	HAL_Delay(5);
}

int main(void) {
	setup();

	while(true) {
		loop();
	}
}
