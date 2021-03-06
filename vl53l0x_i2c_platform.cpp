/*
 * Cannon Falls FRC Team 6217 Bombbotz
 *
 * Function definitions for roborio i2c platform compatibility
 *
 */
#include <iostream>
#include <I2C.h>
#include <vl53l0x_i2c_platform.h>
#include <vl53l0x_def.h>

//#define I2C_DEBUG

//create static I2C class object
static frc::I2C* i2c;

int VL53L0X_i2c_init(frc::I2C::Port port, int deviceAddress){
	i2c = new frc::I2C(port, deviceAddress);
	return VL53L0X_ERROR_NONE;
}

int VL53L0X_write_multi(uint8_t deviceAddress, uint8_t index, uint8_t *pdata, uint32_t count) {
#ifdef I2C_DEBUG
	std::cout << "\tWriting " << count << " to addr 0x" << std::hex << index << std::dec << ": ";
#endif
	uint8_t buffer[count+1];
	buffer[0] = index;

	for(uint32_t i = 1; i < count+1; i++){
		buffer[i] = pdata[i-1];
#ifdef I2C_DEBUG
	std::cout << "0x" << std::hex << pdata[i-1] << std::dec << ", ";
#endif
	}
	i2c->Transaction(buffer, sizeof(buffer), NULL, 0);
#ifdef I2C_DEBUG
	std::cout << std::endl;
#endif
	return VL53L0X_ERROR_NONE;
}

int VL53L0X_read_multi(uint8_t deviceAddress, uint8_t index, uint8_t *pdata, uint32_t count) {
	i2c->Transaction(&index, sizeof(index), pdata, count);
#ifdef I2C_DEBUG
	std::cout << "\tReading " << count << " from addr 0x" << std::hex << index << std::dec <<": ";
	for(uint32_t i = 0; i < count; i++){
		std::cout << "0x" << std::hex << pdata[i] << std::dec << ", ";
	}
	std::cout << std::endl;
#endif
	return VL53L0X_ERROR_NONE;
}

int VL53L0X_write_byte(uint8_t deviceAddress, uint8_t index, uint8_t data) {
	return VL53L0X_write_multi(deviceAddress, index, &data, 1);
}

int VL53L0X_write_word(uint8_t deviceAddress, uint8_t index, uint16_t data) {
	uint8_t buff[2];
	buff[1] = data & 0xFF;
	buff[0] = data >> 8;
	return VL53L0X_write_multi(deviceAddress, index, buff, 2);
}

int VL53L0X_write_dword(uint8_t deviceAddress, uint8_t index, uint32_t data) {
	uint8_t buff[4];

	buff[3] = data & 0xFF;
	buff[2] = data >> 8;
	buff[1] = data >> 16;
	buff[0] = data >> 24;

	return VL53L0X_write_multi(deviceAddress, index, buff, 4);
}

int VL53L0X_read_byte(uint8_t deviceAddress, uint8_t index, uint8_t *data) {
	return VL53L0X_read_multi(deviceAddress, index, data, 1);
}

int VL53L0X_read_word(uint8_t deviceAddress, uint8_t index, uint16_t *data) {
	uint8_t buff[2];
	int r = VL53L0X_read_multi(deviceAddress, index, buff, 2);

	uint16_t tmp;
	tmp = buff[0];
	tmp <<= 8;
	tmp |= buff[1];
	*data = tmp;

	return r;
}

int VL53L0X_read_dword(uint8_t deviceAddress, uint8_t index, uint32_t *data) {
	uint8_t buff[4];
	int r = VL53L0X_read_multi(deviceAddress, index, buff, 4);

	uint32_t tmp;
	tmp = buff[0];
	tmp <<= 8;
	tmp |= buff[1];
	tmp <<= 8;
	tmp |= buff[2];
	tmp <<= 8;
	tmp |= buff[3];

	*data = tmp;

	return r;
}

