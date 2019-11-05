#include <stm32f4xx_hal.h>
#include <stdbool.h>

#include "can.h"
#include "obdii.h"

#define FILL 0x55
#define MSG_LEN 8
#define DTC_LEN 2
#define DTC_FIRST_FRAME 2
#define DTC_PR_FRAME 3
#define MODE1_MAX_PID 0x7F

typedef enum {
	Single      = 0,
	First       = 1,
	Consecutive = 2,
	Flow        = 3,
	Unknown     = -1,
} MF_CANMode;

static OBDII_Mode1_Frame mode1_buffer[MODE1_MAX_PID];

static OBDII_Mode1_Pid pid_list[] = /*{ MonitorStatus, FuelSystemStatus,
	CalculatedEngineLoad, EngineCoolantTemperature, ShortTermFuelTrim,
	LongTermFuelTrim, IntakeManifoldPressure, EngineRPM, VehicleSpeed,
	TimingAdvance, IntakeAirTemperature, ThrottlePosition,
	OxygenSensorsPresent, RuntimeEngineStart, DistanceWithMIL,
	DistanceSinceClear, OxygenSensorFARatio, MonitorStatusDriveCycle,
	ControlModuleVoltage, FARatioCommanded, RelativeThrottlePosition,
	AbsoluteThrottlePosition, EngineFuelRate, DriverDemandTorque,
	EngineReferenceTorque, EnginePercentTorque };*/
	{ EngineRPM, EngineCoolantTemperature, TimingAdvance, IntakeManifoldPressure, ThrottlePosition, RuntimeEngineStart, ControlModuleVoltage, OxygenSensorFARatio, EngineFuelRate };
static uint8_t pid_list_length = 9;
static uint8_t index = 0;
static bool request_pending = false;
static uint32_t last_sequence_request = 0;

/////////////////////////////////////
// Convert to OBDII frame
////////////////////////////////////
static OBDII_Mode1_Frame can_to_obdii_mode1(const CAN_RxFrame* can_frame) {
	OBDII_Mode1_Frame frame = (OBDII_Mode1_Frame) {
		.Pid    = (OBDII_Mode1_Pid) can_frame->Msg[2],
		.Length = can_frame->Msg[0] - 2,
		.New 	= true,
	};
	for (uint8_t i = 0; i < frame.Length; i++) {
		frame.Msg[i] = can_frame->Msg[3 + i];
	}
	return frame;
}

/////////////////////////////////////////////
// Diagnostics Trouble Codes helper functions
/////////////////////////////////////////////
/*static MF_CANMode MultiFrameMode(const CanRxMsgTypeDef* can_frame) {
	return can_frame->Msg[0] >> 4 & 0xF;
}

static uint8_t SingleFrameMessageAmount(const CanRxMsgTypeDef* can_frame) {
	return (can_frame->Msg[0] & 0xF) / DTC_LEN;
}

static uint16_t MultiFrameMessageAmount(const CAN_Frame* can_frame) {
	return ((can_frame->Msg[0] & 0xF) << 4 | can_frame->Msg[1]) / DTC_LEN;
}

static DTC_Message ByteToDTC(const uint8_t *message) {
	return (DTC_Message) {
		.FirstChar  = message[0] >> 6 & 0x3,
		.SecondChar = message[0] >> 4 & 0x3,
		.ThirdChar  = message[0] & 0xF,
		.FourthChar = message[1] >> 4 & 0xF,
		.FifthChar  = message[1] & 0xF,
	};
}*/

////////////////////////////////////
// Receive callback
////////////////////////////////////
static void obdii_response_handler(CAN_RxFrame *msg) {
	if (msg->Msg[1] == 0x41) {
		OBDII_Mode1_Frame frame = can_to_obdii_mode1(msg);
		mode1_buffer[frame.Pid] = frame;
		request_pending = false;
	}
}

///////////////////////////////////
// Public functions
///////////////////////////////////
HAL_StatusTypeDef obdii_init() {
	for (int i = 0; i < MODE1_MAX_PID; i++) {
		mode1_buffer[i] = (OBDII_Mode1_Frame) {
			.Pid = MODE1_MAX_PID + 1,
			.Length = 0,
			.New = false,
		};
	}

	return can_filter(OBDII_RESPONSE_ID, 0x7F8, obdii_response_handler);
}

/*bool GetTroubleCodes(DTC_Message message[], size_t* len) {
	static bool running = false;
	static MF_CANMode frameType = Unknown;
	static size_t length = 0;
	static size_t messageCount = 0;
	static uint8_t frameIndex = 1;
	if (!running) {
		// Diagnostics trouble code request
		can_transmit(OBDII_REQUEST_ID, (uint8_t[])  {1, DTC, FILL, FILL, FILL, FILL, FILL, FILL}, MSG_LEN);
		running = true;
	}

	CAN_Frame received;
	if (frameType == Unknown) {
		if (CAN_Receive(OBDII_FilterNum, &received)) {
			frameType = MultiFrameMode(&received);
		}
		else {
			return false;
		}
	}

	if (frameType == Single) {
		*len = SingleFrameMessageAmount(&received);

		// Get DTC in single frame
		for (uint8_t i = 0; i < *len; i++) {
			message[i] = ByteToDTC(received.Msg + 1 + i * 2);
		}

		running = false;
		frameType = Unknown;
		return true;
	}
	else if (frameType == First) {
		// Send flow control
		can_transmit(OBDII_REQUEST_ID, (uint8_t[]) {Flow << 4, 0, 0, FILL, FILL, FILL, FILL, FILL}, MSG_LEN);

		// Handle DTC in first frame
		message[0] = ByteToDTC(received.Msg + 1);
		message[1] = ByteToDTC(received.Msg + 3);

		messageCount += 2;
		length = MultiFrameMessageAmount(&received);
	}

	// While there are more messages
	while (messageCount < length - 1) {
		if (CAN_Receive(OBDII_FilterNum, &received)) {
			if ((received.Msg[0] & 0xFF) != frameIndex) // Check that index order is correct
			{   } // Error
			frameIndex = (frameIndex + 1) % 0xF;

			// For all DTC in frame
			for (uint8_t j = 0; j < DTC_PR_FRAME; j++) {
				if (messageCount < length - 1) {
					message[messageCount++] = ByteToDTC(received.Msg + (j + 1) * DTC_LEN);
				}
			}
		}
		else {
			return false;
		}
	}

	*len = length;

	running = false;
	frameType = Unknown;
	length = 0;
	messageCount = 0;
	frameIndex = 1;
	return true;
}

void ClearTroubleCodes() {
	can_transmit(OBDII_REQUEST_ID, (uint8_t[]) {1, ClearDTC, FILL, FILL, FILL, FILL, FILL, FILL}, MSG_LEN);
}*/

HAL_StatusTypeDef obdii_mode1_request(OBDII_Mode1_Pid pid) {
	assert_param(pid != FreezeDTC);
	return can_transmit(OBDII_REQUEST_ID, (uint8_t[]) {2, ShowCurrent, pid, FILL, FILL, FILL, FILL, FILL}, MSG_LEN);
}

/*void OBDII_Mode2_Request(OBDII_Pid pid) {
	assert_param(pid != MonitorStatus);
	can_transmit(OBDII_REQUEST_ID, (uint8_t[]) {2, ShowFreeze, pid, FILL, FILL, FILL, FILL, FILL}, MSG_LEN);
}*/

OBDII_Mode1_Frame obdii_mode1_response(OBDII_Mode1_Pid pid) {
	OBDII_Mode1_Frame frame = mode1_buffer[pid];

	if (frame.New) {
		mode1_buffer[pid].New = false;
	}

	return frame;
}

uint32_t obdii_mode1_uid(OBDII_Mode1_Pid pid) {
	return (CAN_OBD_ID_START << 8) | pid;
}

static bool is_pending_but_not_timeout() {
	return request_pending && HAL_GetTick() - last_sequence_request < 200;
}

static bool first_pid_and_should_wait() {
	return index == 0 && HAL_GetTick() - last_sequence_request < 1;
}

void obdii_request_next(void) {
	if (is_pending_but_not_timeout()) {
		return;
	}
	else if (first_pid_and_should_wait()) {
		return;
	}

	if (index == 0) {
		last_sequence_request = HAL_GetTick();
	}

	obdii_mode1_request(pid_list[index]);
	request_pending = true;

	index++;

	if (index == pid_list_length) {
		index = 0;
	}
}
