#include <px4_platform_common/module.h>
#include <lib/drivers/device/Device.hpp>

#include "bbblue_bmp.h"

// choice of 1,2,4,8,16 oversampling. Here we use 16 and sample at 25hz which
// is close to the update rate specified in rc/bmp.h for that oversample.
#define OVERSAMPLE  BMP_OVERSAMPLE_16

// choice of OFF, 2, 4, 8, 16 filter constants. Here we turn off the filter and
// opt to use our own 2nd order filter instead.
#define INTERNAL_FILTER	BMP_FILTER_OFF

// our own low pass filter
#define ORDER		2
#define CUTOFF_FREQ	2.0f	// 2rad/s, about 0.3hz
#define BMP_CHECK_HZ	25
#define	DT		1.0f/BMP_CHECK_HZ

BBBlueBMP::BBBlueBMP() :
	ScheduledWorkItem(MODULE_NAME, px4::device_bus_to_wq(BBBLUE_BMP_DEVID)),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": comms errors")),
	_px4_baro(BBBLUE_BMP_DEVID)
{
}

BBBlueBMP::~BBBlueBMP()
{
	perf_free(_cycle_perf);
	perf_free(_comms_errors);
}

void BBBlueBMP::Run()
{
	if (should_exit()) {
		rc_bmp_power_off();
		return;
	}

	if (!_initialized) {
		if (rc_bmp_init(OVERSAMPLE, INTERNAL_FILTER)) {
			PX4_ERR("rc_bmp_init failed");
			return;
		}

		_initialized = true;
	}

	perf_begin(_cycle_perf);

	hrt_abstime timestamp_sample = hrt_absolute_time();

	if (rc_bmp_read(&_data) < 0) {
		PX4_ERR("read barometer data failed");
		perf_count(_comms_errors);
	}

	_px4_baro.set_error_count(perf_event_count(_comms_errors));
	_px4_baro.set_temperature(_data.temp_c);
	_px4_baro.update(timestamp_sample, _data.pressure_pa / 1000.0);

	perf_end(_cycle_perf);
}

int BBBlueBMP::task_spawn(int argc, char *argv[])
{
	BBBlueBMP *instance = new BBBlueBMP();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	instance->ScheduleOnInterval(1000000 / BMP_CHECK_HZ);

	return PX4_OK;
}

int BBBlueBMP::custom_command(int argc, char *argv[])
{
	// Check if the driver is running.
	if (!is_running()) {
		PX4_INFO("not running");
		return PX4_ERROR;
	}

	return print_usage("unknown command");
}

int BBBlueBMP::print_status()
{
	PX4_INFO("Running");

	perf_print_counter(_cycle_perf);
	perf_print_counter(_comms_errors);

	return 0;
}

int BBBlueBMP::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_USAGE_NAME("bbblue_bmp", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("baro");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int bbblue_bmp_main(int argc, char *argv[])
{
	return BBBlueBMP::main(argc, argv);
}
