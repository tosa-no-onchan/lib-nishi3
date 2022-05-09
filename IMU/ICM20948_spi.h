/*
* ICM20948_spi.h
*/

#ifndef _ICM20948_SPI_H_
#define _ICM20948_SPI_H_

#include <inttypes.h>
#include <Arduino.h>
#include "Define.h"

#include "ICM20948_REGS.h"

// add by nishi
//#define SS_PIN   5
#define BDPIN_SPI_CS_IMU 5

#define SERIAL_PORT Serial


class cICM20948_spi
{

public:
	bool     bConnected;
	// AK8963 get calibration data
	int16_t AK8963_ASA[3];

public:
	cICM20948_spi();

	void AK09916_init(bool minimal=false);

	void selectBank(uint8_t bank);
	void spiRead(uint16_t addr, uint8_t *p_data, uint32_t length);
	uint8_t spiReadByte(uint16_t addr);
	void spiWriteByte(uint16_t addr, uint8_t data);
	void spiWrite(uint16_t addr, uint8_t *data,uint32_t length);

	int imu_spi_ak09916_reads(uint8_t akm_addr, uint8_t reg_addr, uint8_t len, uint8_t *data);
	int imu_spi_ak09916_writes(uint8_t akm_addr, uint8_t reg_addr, uint8_t len, uint8_t *data);
	int imu_spi_ak09916_write(uint8_t akm_addr, uint8_t reg_addr, uint8_t data);

	ICM_20948_Status_e ICM_20948_i2c_controller_periph4_txn(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len, bool Rw, bool send_reg_addr);

private:

};

#endif