/*
* ICM20948.h
*/

#ifndef _ICM20948_H_
#define _ICM20948_H_

#include <inttypes.h>
#include <Arduino.h>
#include "Define.h"

// add by nishi
#define ICM20948_IMU
#define USE_SPARK_LIB
#define USE_ACC_NISHI
//#define USE_GRYO_NISHI
#define USE_DMP_NISHI

// add by nishi 2021.10.7
#define IMU_SENSER6
//#define USE_MAG

#ifdef USE_SPARK_LIB
#include <ICM_20948.h> // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#else
#include "ICM20948_REGS.h"
#endif

#define ACC_1G     512

//#define MPU_SPI   SPI_IMU
// changed by nishi
#define MPU_SPI   SPI

// add by nishi
//#define SS_PIN   5
#define BDPIN_SPI_CS_IMU 5

#define SERIAL_PORT Serial


// for ICM_20948.h
#ifdef USE_SPARK_LIB
#define USE_SPI       // Uncomment this to use SPI
#define SERIAL_PORT Serial

//#define SPI_PORT SPI // Your desired SPI port.       Used only when "USE_SPI" is defined
//#define CS_PIN 2     // Which pin you connect CS to. Used only when "USE_SPI" is defined
#define CS_PIN 5     // Which pin you connect CS to. Used only when "USE_SPI" is defined

#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
#define AD0_VAL 1      // The value of the last bit of the I2C address.                \
                       // On the SparkFun 9DoF IMU breakout the default is 1, and when \
                       // the ADR jumper is closed the value becomes 0
#endif

/** Macro for micro tesla (uT) per LSB (1 LSB = 0.1uT) */
#define MAG_UT_LSB      (0.15F)

class cICM20948
{

public:
	bool     bConnected;

	int16_t  gyroADC[3];
	int16_t  gyroADC_BD[3];	// add by nishi from ICM-20948
	int16_t  gyroRAW[3];
	int16_t  gyroZero[3];

	int16_t  accADC[3];
	int16_t  accADC_BD[3];	// add by nishi from ICM-20948
	int16_t  accRAW[3];
	int16_t	 accZero[3];
	int32_t	 accZeroSum;
	float    accIMZero;			// add by nishi acc 内積

	//int16_t  magADC[3];
	float  magADC[3];		// changed by nishi 2021.11.8
	int16_t  magADC_BD[3];	// add by nishi from ICM-20948
	int16_t  magRAW[3];
	int16_t  magRAW_BD[3];	// add by nishi from ICM-20948
	int16_t  magZero[3];

	int16_t  gyroData[3];
	int16_t  accSmooth[3];

	uint16_t calibratingG;
	uint16_t calibratingA;
	uint16_t calibratingM;
	uint16_t calibratingD;

	char calibratingA_f;
	char calibratingD_f;

	// AK8963 get calibration data
	int16_t AK8963_ASA[3];

	double quat[4];		// add by nishi
	int32_t quatRAW[4];		// add by nishi
	int32_t quatZero[4];		// add by nishi

	#ifndef USE_SPARK_LIB
	#endif

public:
	cICM20948();

  bool begin( void );
  bool init( void );
	void gyro_init( void );
	void gyro_get_adc( void );
	void gyro_common();
	void gyro_cali_start();
	bool gyro_cali_get_done();

	void acc_init( void );
	void acc_get_adc( void );
	void acc_common();
	void acc_cali_start();
	bool acc_cali_get_done();

	#ifdef USE_DMP_NISHI
	void dmp_init( void );
	bool dmp_get_adc();
	bool dmp_cali_get_done();
	#endif

	void mag_init( void );
	bool mag_get_adc( void );
	void mag_common();
	void mag_cali_start();
	bool mag_cali_get_done();

	void AK09916_init(bool minimal=false);


private:
	void selectBank(uint8_t bank);
	void spiRead(uint16_t addr, uint8_t *p_data, uint32_t length);
	uint8_t spiReadByte(uint16_t addr);
	void spiWriteByte(uint16_t addr, uint8_t data);
	void spiWrite(uint16_t addr, uint8_t *data,uint32_t length);

	#ifdef USE_SPARK_LIB
	ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object
	#endif

	#ifndef USE_SPARK_LIB
	int imu_spi_ak09916_reads(uint8_t akm_addr, uint8_t reg_addr, uint8_t len, uint8_t *data);
	int imu_spi_ak09916_writes(uint8_t akm_addr, uint8_t reg_addr, uint8_t len, uint8_t *data);
	int imu_spi_ak09916_write(uint8_t akm_addr, uint8_t reg_addr, uint8_t data);

	ICM_20948_Status_e ICM_20948_i2c_controller_periph4_txn(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len, bool Rw, bool send_reg_addr);

	#endif
	float invSqrt(float x);
};

#endif