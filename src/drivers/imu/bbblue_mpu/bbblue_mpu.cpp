#include <px4_platform_common/module.h>

#include "bbblue_mpu.h"

BBBlueMPU::BBBlueMPU() :
	ScheduledWorkItem(MODULE_NAME, px4::device_bus_to_wq(2)),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_px4_accel(1, ORB_PRIO_HIGH, (enum Rotation)0),
	_px4_gyro(1, ORB_PRIO_HIGH, (enum Rotation)0),
	_px4_mag(1, ORB_PRIO_DEFAULT, (enum Rotation)0)
{
}

BBBlueMPU::~BBBlueMPU()
{
	perf_free(_cycle_perf);
}

void BBBlueMPU::Run()
{
	if (should_exit()) {
		rc_mpu_power_off();
		return;
	}

	if (!_initialized) {
		rc_mpu_config_t conf = rc_mpu_default_config();
		conf.i2c_bus = 2;
		conf.enable_magnetometer = true;
		conf.show_warnings = false;

		conf.accel_fsr = rc_mpu_accel_fsr_t::ACCEL_FSR_16G;
		// conf.accel_dlpf = rc_mpu_accel_dlpf_t::ACCEL_DLPF_OFF;

		conf.gyro_fsr = rc_mpu_gyro_fsr_t::GYRO_FSR_2000DPS;
		// conf.gyro_dlpf = rc_mpu_gyro_dlpf_t::GYRO_DLPF_OFF;

		configure_accel(conf.accel_fsr);
		configure_gyro(conf.gyro_fsr);

		if (rc_mpu_initialize(&_data, conf)) {
			PX4_ERR("rc_mpu_initialize_failed");
			return;
		}

		_initialized = true;
	}

	perf_begin(_cycle_perf);

	hrt_abstime timestamp_sample = hrt_absolute_time();

	if (rc_mpu_read_accel(&_data) < 0) {
		PX4_ERR("read accel data failed");
		_px4_accel.increase_error_count();
	}
	if (rc_mpu_read_gyro(&_data) < 0) {
		PX4_ERR("read gyro data failed");
		_px4_gyro.increase_error_count();
	}
	if (rc_mpu_read_mag(&_data)) {
		PX4_ERR("read mag data failed");
		_px4_mag.increase_error_count();
	}
	if (rc_mpu_read_temp(&_data)) {
		PX4_ERR("read imu thermometer failed");
	}

	_px4_accel.set_temperature(_data.temp);
	_px4_gyro.set_temperature(_data.temp);
	_px4_mag.set_temperature(_data.temp);

	_px4_accel.update(timestamp_sample, _data.accel[0], -_data.accel[1], -_data.accel[2]);
	_px4_gyro.update(timestamp_sample, _data.gyro[0], -_data.gyro[1], -_data.gyro[2]);
	_px4_mag.update(timestamp_sample, _data.mag[0], _data.mag[1], _data.mag[2]);

	perf_end(_cycle_perf);
}

void BBBlueMPU::configure_accel(rc_mpu_accel_fsr_t fsr)
{
	switch (fsr) {
	case rc_mpu_accel_fsr_t::ACCEL_FSR_2G:
		_px4_accel.set_range(2 * CONSTANTS_ONE_G);
		break;

	case rc_mpu_accel_fsr_t::ACCEL_FSR_4G:
		_px4_accel.set_range(4 * CONSTANTS_ONE_G);
		break;

	case rc_mpu_accel_fsr_t::ACCEL_FSR_8G:
		_px4_accel.set_range(8 * CONSTANTS_ONE_G);
		break;

	case rc_mpu_accel_fsr_t::ACCEL_FSR_16G:
		_px4_accel.set_range(16 * CONSTANTS_ONE_G);
		break;
	}
}

void BBBlueMPU::configure_gyro(rc_mpu_gyro_fsr_t fsr)
{
	float range_dps = 0.f;

	switch (fsr) {
	case rc_mpu_gyro_fsr_t::GYRO_FSR_250DPS:
		range_dps = 250.f;
		break;

	case rc_mpu_gyro_fsr_t::GYRO_FSR_500DPS:
		range_dps = 500.f;
		break;

	case rc_mpu_gyro_fsr_t::GYRO_FSR_1000DPS:
		range_dps = 1000.f;
		break;

	case rc_mpu_gyro_fsr_t::GYRO_FSR_2000DPS:
		range_dps = 2000.f;
		break;
	}

	_px4_gyro.set_scale(math::radians(range_dps / 32768.f));
	_px4_gyro.set_range(math::radians(range_dps));
}

int BBBlueMPU::task_spawn(int argc, char *argv[])
{
	BBBlueMPU *instance = new BBBlueMPU();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	instance->ScheduleOnInterval(_current_update_interval);

	return PX4_OK;
}

int BBBlueMPU::custom_command(int argc, char *argv[])
{
	// Check if the driver is running.
	if (!is_running()) {
		PX4_INFO("not running");
		return PX4_ERROR;
	}

	return print_usage("unknown command");
}

int BBBlueMPU::print_status()
{
	PX4_INFO("Running");

	perf_print_counter(_cycle_perf);

	return 0;
}

int BBBlueMPU::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_USAGE_NAME("bbblue_bmp", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("imu");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int bbblue_mpu_main(int argc, char *argv[])
{
	return BBBlueMPU::main(argc, argv);
}
