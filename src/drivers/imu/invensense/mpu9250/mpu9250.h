#pragma once

#include <drivers/device/spi.h>
#include <inttypes.h>

namespace mpu9250
{

#pragma pack(push,1)
struct data_s {
	uint8_t ACCEL_XOUT_H;
	uint8_t ACCEL_XOUT_L;
	uint8_t ACCEL_YOUT_H;
	uint8_t ACCEL_YOUT_L;
	uint8_t ACCEL_ZOUT_H;
	uint8_t ACCEL_ZOUT_L;
	uint8_t GYRO_XOUT_H;
	uint8_t GYRO_XOUT_L;
	uint8_t GYRO_YOUT_H;
	uint8_t GYRO_YOUT_L;
	uint8_t GYRO_ZOUT_H;
	uint8_t GYRO_ZOUT_L;
};
#pragma pack(pop)

class IMPU9250
{
public:
	virtual ~IMPU9250() = default;

	virtual int init() = 0;

	// read reg value
	virtual uint8_t get_reg(uint8_t addr) = 0;

	// write reg value
	virtual int set_reg(uint8_t value, uint8_t addr) = 0;

	// bulk read of data into buffer, return same pointer
	virtual mpu9250::data_s *get_data(uint8_t addr, uint16_t num_samples) = 0;

	virtual uint32_t get_device_id() const = 0;

	virtual uint8_t get_device_address() const = 0;

	virtual int transfer(uint8_t *send, uint8_t *recv, unsigned len) = 0;

	virtual bool external() const = 0;
};

} /* namespace */

/* interface factories */
extern mpu9250::IMPU9250 *mpu9250_spi_interface(uint8_t busnum, uint32_t device, int bus_frequency, spi_mode_e spi_mode);
extern mpu9250::IMPU9250 *mpu9250_i2c_interface(uint8_t busnum, uint32_t device, int bus_frequency);
