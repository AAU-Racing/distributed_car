#ifndef OBDII_GUARD
#define OBDII_GUARD

#include <stm32f4xx_hal.h>

#include <stdbool.h>

#define OBDII_REQUEST_ID  0x7DF
#define OBDII_RESPONSE_ID 0x7E8

typedef enum{
	ShowCurrent  = 0x1,
	ShowFreeze   = 0x2,
	DTC          = 0x3,
	ClearDTC     = 0x4,
	OxygenSensor = 0x6,
} OBDII_Mode;

typedef enum{
	/* Supported pids bit encoded */
	SupportedPid0x01_0x20 = 0x00,
	SupportedPid0x21_0x40 = 0x20,
	SupportedPid0x41_0x60 = 0x40,
	SupportedPid0x61_0x80 = 0x60,

	/* General fields */
	MonitorStatus            = 0x01, // Mode 1 only
	//FreezeDTC                = 0x02, // Mode 2 only
	FuelSystemStatus         = 0x03,
	CalculatedEngineLoad     = 0x04,
	EngineCoolantTemperature = 0x05,
	ShortTermFuelTrim        = 0x06,
	LongTermFuelTrim         = 0x07,
	IntakeManifoldPressure   = 0x0B,
	EngineRPM                = 0x0C,
	VehicleSpeed             = 0x0D,
	TimingAdvance            = 0x0E,
	IntakeAirTemperature     = 0x0F,
	ThrottlePosition         = 0x11,
	OxygenSensorsPresent     = 0x13,
	OBDConformity            = 0x1C, // NO
	RuntimeEngineStart       = 0x1F,
	DistanceWithMIL          = 0x21,
	DistanceSinceClear       = 0x31,
	OxygenSensorFARatio      = 0x34,
	MonitorStatusDriveCycle  = 0x41,
	ControlModuleVoltage     = 0x42,
	FARatioCommanded         = 0x44,
	RelativeThrottlePosition = 0x45,
	AbsoluteThrottlePosition = 0x47,
	EngineFuelRate           = 0x5E,
	DriverDemandTorque       = 0x61,
	EngineReferenceTorque    = 0x63,
	EnginePercentTorque      = 0x64,
} OBDII_Mode1_Pid;

typedef enum{
	Powertrain = 0,
	Chassis    = 1,
	Body       = 2,
	Network    = 3,
} DTC_Location;

typedef struct{
	DTC_Location FirstChar;
	uint8_t SecondChar;
	uint8_t ThirdChar;
	uint8_t FourthChar;
	uint8_t FifthChar;
} DTC_Message;

typedef struct{
	OBDII_Mode1_Pid Pid;
	size_t Length;
	uint8_t Msg[4];
	bool New;
} OBDII_Mode1_Frame;

/*typedef struct{
	OBDII_Pid Pid;
	size_t Length;
	uint8_t Msg[4];
} OBDII_Mode2_Frame;*/

HAL_StatusTypeDef obdii_init();
//bool GetTroubleCodes(DTC_Message dtc[], size_t* len);
//void ClearTroubleCodes();
HAL_StatusTypeDef obdii_mode1_request(OBDII_Mode1_Pid pid);
OBDII_Mode1_Frame obdii_mode1_response(OBDII_Mode1_Pid pid);
void obdii_request_next();
uint32_t obdii_mode1_uid(OBDII_Mode1_Pid pid);

#endif
