#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include <shield_driver/com_node/sd.h>
#include <ff_gen_drv.h>
#include <sd_diskio.h>
#include <shield_driver/dashboard/oil.h>
#include <shield_driver/com_node/water_temperature.h>
#include <shield_driver/com_node/current_clamps.h>
#include <shield_driver/com_node/wheel_speed.h>
#include <shield_driver/com_node/error.h>
#include <board_driver/can.h>
#include <board_driver/obdii.h>

// Data log
static char dataBuf[SD_STANDARD_BLOCK_SIZE] = {'\0'};
static char* dataBufEnd = dataBuf;
static char data_file_name[16] = {'\0'};
static uint32_t last_data_write_time = 0;

// Error log
static char errorBuf[SD_STANDARD_BLOCK_SIZE] = {'\0'};
static char* errorBufEnd = errorBuf;
static char error_file_name[16] = { '\0' };
static uint32_t last_error_write_time = 0;

// General logging
static FATFS fs;
static char element[64] = {'\0'};
static uint32_t time_since_start = 0;
static uint32_t start_time = 0;
static bool sd_initialized = false;

static char* buf_end(char *file_name) {
	if (file_name == data_file_name) {
		return dataBufEnd;
	}
	else if (file_name == error_file_name) {
		return errorBufEnd;
	}

	return 0;
}

static char *buffer(char *file_name) {
	if (file_name == data_file_name) {
		return dataBuf;
	}
	else if (file_name == error_file_name) {
		return errorBuf;
	}

	return 0;
}

static void reset_buf_end(char *file_name) {
	if (file_name == data_file_name) {
		dataBufEnd = dataBuf;
	}
	else if (file_name == error_file_name) {
		errorBufEnd = errorBuf;
	}
}

static void increment_buf_end(char *file_name) {
	if (file_name == data_file_name) {
		dataBufEnd++;
	}
	else if (file_name == error_file_name) {
		errorBufEnd++;
	}
}

static void append_buffer_to_sd(char *file_name){
	uint16_t data_length = buf_end(file_name) - buffer(file_name);
	appendData(file_name, buffer(file_name), data_length);
	memset(buffer(file_name), '\0', SD_STANDARD_BLOCK_SIZE);
	reset_buf_end(file_name);
}

static void output_data(char *file_name, char data[]){
	for(uint8_t i = 0; data[i] != '\0'; i++){
		*buf_end(file_name) = data[i]; //Transfering to main buffer
		data[i] = '\0';
		increment_buf_end(file_name);

		if(buf_end(file_name) >= (buffer(file_name) + SD_STANDARD_BLOCK_SIZE)){
			append_buffer_to_sd(file_name);
		}
	}
}

static void handle_time_id(char *file_name, uint32_t id) {
	time_since_start = (HAL_GetTick() - start_time);

	sprintf(element, "%u;", (unsigned)time_since_start);
	output_data(file_name, element);

	sprintf(element, "%u;", (unsigned)id);
	output_data(file_name, element);
}

static void handle_uint_data(char *file_name, uint32_t id, uint32_t data){
	handle_time_id(file_name, id);

	sprintf(element, "%u\n", (unsigned)data);
	output_data(file_name, element);
}

static void handle_signed_data(char *file_name, uint32_t id, int data){
	handle_time_id(file_name, id);

	sprintf(element, "%d\n", data);
	output_data(file_name, element);
}

static void handle_float_data(char *file_name, uint32_t id, float data){
	handle_time_id(file_name, id);

	sprintf(element, "%f\n", data);
	output_data(file_name, element);
}

static void handle_string_data(char *file_name, uint32_t id, char *msg) {
	handle_time_id(file_name, id);

	sprintf(element, "%s\n", msg);
	output_data(file_name, element);
}

static void handle_oil_pressure() {
	bool oilPres = false;

	if (oilPressure_OK(&oilPres)) {
		handle_uint_data(data_file_name, CAN_OIL_PRESSURE, oilPres);
	}
}

static void handle_error() {
	uint16_t id = 0;
	uint32_t error_id = 0;

	while (get_error(&id, &error_id)) {
		handle_uint_data(error_file_name, id, error_id);
	}

	if (id != 0) {
		append_buffer_to_sd(error_file_name);
	}
}

static void handle_water_temperature(void){
	uint16_t water_in;
	uint16_t water_out;

	if (get_water_temp_in(&water_in)){
		handle_uint_data(data_file_name, CAN_WATER_TEMPERATURE_IN, water_in);
	}

	if (get_water_temp_out(&water_out)){
		handle_uint_data(data_file_name, CAN_WATER_TEMPERATURE_OUT, water_out);
	}
}

static void handle_current_clamps(void){
	uint16_t battery_current;
	uint16_t inverter_current;

	if (get_battery_current(&battery_current)){
		handle_uint_data(data_file_name, CAN_BATTERY_CURRENT, battery_current);
	}

	if (get_inverter_current(&inverter_current)){
		handle_uint_data(data_file_name, CAN_INVERTER_CURRENT, inverter_current);
	}
}

static void handle_wheel_speed(void){
	float FL_speed;
	float FR_speed;
	float RL_speed;
	float RR_speed;

	if (get_wheel_speed_FL(&FL_speed)){
		handle_uint_data(data_file_name, CAN_WHEEL_SPEED_FL, FL_speed);
	}

	if (get_wheel_speed_FR(&FR_speed)){
		handle_uint_data(data_file_name, CAN_WHEEL_SPEED_FR, FR_speed);
	}

	if (get_wheel_speed_RL(&RL_speed)){
		handle_uint_data(data_file_name, CAN_WHEEL_SPEED_RL, RL_speed);
	}

	if (get_wheel_speed_RR(&RR_speed)){
		handle_uint_data(data_file_name, CAN_WHEEL_SPEED_RR, RR_speed);
	}
}

static void handle_obdii_msg(OBDII_Mode1_Pid pid, void (*handle_data)(uint32_t, uint8_t*)) {
	OBDII_Mode1_Frame frame = obdii_mode1_response(pid);
	if (frame.New){
		uint32_t id = obdii_mode1_uid(frame.Pid);
		handle_data(id, frame.Msg);
	}
}

static void handle_obdii(void) {
	handle_obdii_msg(MonitorStatus, ({
		void translate_msg(uint32_t id, uint8_t *msg) {
			uint32_t data = 0;
			memcpy(&data, msg, 4);
			handle_uint_data(data_file_name, id, data);
		}
		translate_msg;
	}));

	handle_obdii_msg(FuelSystemStatus, ({
		void translate_msg(uint32_t id, uint8_t *msg) {
			uint32_t data = 0;
			memcpy(&data, msg, 2);
			handle_uint_data(data_file_name, id, data);
		}
		translate_msg;
	}));

	handle_obdii_msg(CalculatedEngineLoad, ({
		void translate_msg(uint32_t id, uint8_t *msg) {
			handle_uint_data(data_file_name, id, (100 * msg[0]) / 255);
		}
		translate_msg;
	}));

	handle_obdii_msg(EngineCoolantTemperature, ({
		void translate_msg(uint32_t id, uint8_t *msg) {
			handle_float_data(data_file_name, id, msg[0] - 40);
		}
		translate_msg;
	}));

	handle_obdii_msg(ShortTermFuelTrim, ({
		void translate_msg(uint32_t id, uint8_t *msg) {
			handle_float_data(data_file_name, id, (100 * msg[0]) / 128 - 100);
		}
		translate_msg;
	}));

	handle_obdii_msg(LongTermFuelTrim, ({
		void translate_msg(uint32_t id, uint8_t *msg) {
			handle_float_data(data_file_name, id, (100 * msg[0]) / 128 - 100);
		}
		translate_msg;
	}));

	handle_obdii_msg(IntakeManifoldPressure, ({
		void translate_msg(uint32_t id, uint8_t *msg) {
			handle_uint_data(data_file_name, id, (uint32_t) msg[0]);
		}
		translate_msg;
	}));

	handle_obdii_msg(EngineRPM, ({
		void translate_msg(uint32_t id, uint8_t *msg) {
			handle_float_data(data_file_name, id, ((256 * msg[0]) + msg[1]) / 4);
		}
		translate_msg;
	}));

	handle_obdii_msg(VehicleSpeed, ({
		void translate_msg(uint32_t id, uint8_t *msg) {
			handle_uint_data(data_file_name, id, (uint32_t) msg[0]);
		}
		translate_msg;
	}));

	handle_obdii_msg(TimingAdvance, ({
		void translate_msg(uint32_t id, uint8_t *msg) {
			handle_float_data(data_file_name, id, msg[0] / 2 - 64);
		}
		translate_msg;
	}));

	handle_obdii_msg(IntakeAirTemperature, ({
		void translate_msg(uint32_t id, uint8_t *msg) {
			handle_uint_data(data_file_name, id, msg[0] - 40);
		}
		translate_msg;
	}));

	handle_obdii_msg(ThrottlePosition, ({
		void translate_msg(uint32_t id, uint8_t *msg) {
			handle_float_data(data_file_name, id, (msg[0] * 100) / 255);
		}
		translate_msg;
	}));

	// Does not make sense to log continously
	// OBDII_Mode1_Frame frame = obdii_mode1_response(OxygenSensorPresent);

	handle_obdii_msg(RuntimeEngineStart, ({
		void translate_msg(uint32_t id, uint8_t *msg) {
			uint32_t seconds = 0;
			memcpy(&seconds, msg, 2);
			handle_uint_data(data_file_name, id, seconds);
		}
		translate_msg;
	}));

	handle_obdii_msg(DistanceWithMIL, ({
		void translate_msg(uint32_t id, uint8_t *msg) {
			uint32_t distance = 0;
			memcpy(&distance, msg, 2);
			handle_uint_data(data_file_name, id, distance);
		}
		translate_msg;
	}));

	handle_obdii_msg(DistanceSinceClear, ({
		void translate_msg(uint32_t id, uint8_t *msg) {
			uint32_t distance = 0;
			memcpy(&distance, msg, 2);
			handle_uint_data(data_file_name, id, distance);
		}
		translate_msg;
	}));

	handle_obdii_msg(OxygenSensorFARatio, ({
		void translate_msg(uint32_t id, uint8_t *msg) {
			float ratio = 2.0 / 65536 * (256 * msg[0] + msg[1]);
			float current = msg[2] + msg[3] / 256.0 - 128;
			handle_float_data(data_file_name, id << 4 | 1, ratio);
			handle_float_data(data_file_name, id << 4 | 2, current);
		}
		translate_msg;
	}));

	// Needs interpretation
	// OBDII_Mode1_Frame frame = obdii_mode1_response(MonitorStatusDriveCycle);

	handle_obdii_msg(ControlModuleVoltage, ({
		void translate_msg(uint32_t id, uint8_t *msg) {
			handle_float_data(data_file_name, id, (msg[0] * 256 + msg[1]) / 1000);
		}
		translate_msg;
	}));

	handle_obdii_msg(ControlModuleVoltage, ({
		void translate_msg(uint32_t id, uint8_t *msg) {
			handle_float_data(data_file_name, id, 2.0 / 65536 * (msg[0] * 256 + msg[1]));
		}
		translate_msg;
	}));

	handle_obdii_msg(RelativeThrottlePosition, ({
		void translate_msg(uint32_t id, uint8_t *msg) {
			handle_float_data(data_file_name, id, (msg[0] * 100) / 255);
		}
		translate_msg;
	}));

	handle_obdii_msg(AbsoluteThrottlePosition, ({
		void translate_msg(uint32_t id, uint8_t *msg) {
			handle_float_data(data_file_name, id, (msg[0] * 100) / 255);
		}
		translate_msg;
	}));

	handle_obdii_msg(EngineFuelRate, ({
		void translate_msg(uint32_t id, uint8_t *msg) {
			handle_float_data(data_file_name, id, (msg[0] * 256 + msg[1]) / 20);
		}
		translate_msg;
	}));

	handle_obdii_msg(DriverDemandTorque, ({
		void translate_msg(uint32_t id, uint8_t *msg) {
			handle_signed_data(data_file_name, id, msg[0] - 125);
		}
		translate_msg;
	}));

	handle_obdii_msg(EngineReferenceTorque, ({
		void translate_msg(uint32_t id, uint8_t *msg) {
			handle_uint_data(data_file_name, id, msg[0] * 256 + msg[1]);
		}
		translate_msg;
	}));

	handle_obdii_msg(EnginePercentTorque, ({
		void translate_msg(uint32_t id, uint8_t *msg) {
			int idle = msg[0] - 125;
			int enginePoint1 = msg[1] - 125;
			int enginePoint2 = msg[2] - 125;
			int enginePoint3 = msg[3] - 125;
			int enginePoint4 = msg[4] - 125;
			handle_signed_data(data_file_name, id << 4 | 0, idle);
			handle_signed_data(data_file_name, id << 4 | 1, enginePoint1);
			handle_signed_data(data_file_name, id << 4 | 2, enginePoint2);
			handle_signed_data(data_file_name, id << 4 | 3, enginePoint3);
			handle_signed_data(data_file_name, id << 4 | 4, enginePoint4);
		}
		translate_msg;
	}));
}

static HAL_StatusTypeDef init_fatfs(FATFS* fs) {
	char SDPath[4];

	// SD_Driver is exported from sd_diskio.h
	if (FATFS_LinkDriver(&SD_Driver, SDPath) != 0) {
		return HAL_ERROR;
	}
	if (f_mount(fs, SDPath, 1) != FR_OK) {
		return HAL_ERROR;
	}
	return HAL_OK;
}

void init_sd() {
	if (init_fatfs(&fs) == HAL_OK) {
		create_log(data_file_name);
		create_error(error_file_name);
		sd_initialized = true;
		printf("Successfully created file on SD\n");
	}

	start_time = HAL_GetTick();
	time_since_start = start_time;
}

void handle_logging() {
	// Only try to log if the SD-card is initialized
	if (sd_initialized) {
		//handle_oil_pressure();
		//handle_error();
		handle_water_temperature();
		//handle_current_clamps();
		//handle_wheel_speed();
		//handle_obdii();

		if(last_data_write_time + 1000 < HAL_GetTick()) { //If its been over a second since last data write to SD
			append_buffer_to_sd(data_file_name);

			last_data_write_time = HAL_GetTick();

			printf("Writing data to SD\n");
		}

		if(last_error_write_time + 1000 < HAL_GetTick()) { //If its been over a second since last error write to SD
			append_buffer_to_sd(error_file_name);

			last_error_write_time = HAL_GetTick();

			printf("Writing error log to SD\n");
		}
	}
}

void log_error(uint32_t id, char* error_msg) {
	handle_string_data(error_file_name, id, error_msg);
}
