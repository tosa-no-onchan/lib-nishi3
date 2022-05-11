/*
* ICM20948.h
*/

#ifndef _ICM20948_H_
#define _ICM20948_H_

#include <inttypes.h>
#include <Arduino.h>
#include "Define.h"

//#include "use_def.h"
// add by nishi
#define ICM20948_IMU
//#define USE_SPARK_LIB
#define USE_ACC_NISHI
#define USE_GRYO_NISHI
//#define USE_DMP_NISHI
#define USE_MADWICK

// add by nishi 2021.10.7
#define IMU_SENSER6
//#define USE_MAG

#define USE_ACC_2G
//#define USE_ACC_4G
//#define USE_ACC_8G

// Full-Scale Range
//  ACCEL_FS=0  -> ±2 [G]
//  ACCEL_FS=1  -> ±4 [G]
//  ACCEL_FS=2  -> ±8 [G]
//  ACCEL_FS=3  -> ±16 [G]
// Sensitivity Scale Factor
//  ACCEL_FS=0 -> 16,384 [LSB/g]
//  ACCEL_FS=1 -> 8,192 [LSB/g]
//  ACCEL_FS=2 -> 4,096 [LSB/g]
//  ACCEL_FS=3 -> 2,048 [LSB/g]
// initial Tolerance
//  Component-level ±0.5[%]
// ZERO-G OUTPUT
//  Initial Tolerance Board-level, all axes ±50 [mg]
//aRes = 8.0/32768.0;      // 8g    16bit -> 8G

#define GYRO_NOISE_CUT_OFF 4

#if defined(USE_ACC_2G)
	#define ACC_1G 16384.0
	#define ACC_ZERO_OFF 	5.0		// + で上へ行く。 - で下に行く
    #define ACC_X_CUT_OFF 35.0    // 2G  with Low pass filter  ICM20948_ACCEL_BW_6HZ
    #define ACC_Y_CUT_OFF 35.0    // 
    #define ACC_Z_CUT_OFF_P 35.0  // 
	#define ACC_Z_CUT_OFF_M -35.0 // 

    //#define ACC_X_CUT_OFF 300.0    // 2G  without Low pass filter
    //#define ACC_Y_CUT_OFF 300.0    // 
    //#define ACC_Z_CUT_OFF_P 300.0  // 
    //#define ACC_Z_CUT_OFF_M -300.0 // 

    //#define ACC_X_CUT_OFF 150.0    // 2G  without Low pass filter
    //#define ACC_Y_CUT_OFF 150.0    // 
    //#define ACC_Z_CUT_OFF_P 150.0  // 
    //#define ACC_Z_CUT_OFF_M -150.0 // 

	#define ACC_ZERO_Z_OVER 3000

	// 速度
	// 普通にIMU を振って、 0.1 位

	// 速度を Cut Off
	//#define VX_CUT_OFF 0.3
	//#define VX_CUT_OFF 0.25
	#define VX_CUT_OFF 0.08
	//#define VX_CUT_OFF 0.1

	//#define VY_CUT_OFF 0.3
	//#define VY_CUT_OFF 0.25
	#define VY_CUT_OFF 0.08
	//#define VY_CUT_OFF 0.1

	//#define VZ_CUT_OFF 0.3
	//#define VZ_CUT_OFF 0.25
	#define VZ_CUT_OFF 0.08
	//#define VZ_CUT_OFF 0.1

	#define VX_MAX_CUT_OFF 0.35
	#define VY_MAX_CUT_OFF 0.35
	#define VZ_MAX_CUT_OFF 0.35


#elif defined(USE_ACC_4G)
	#define ACC_1G  8192.0
	#define ACC_ZERO_OFF -0.1		// + で上へ行く。 - で下に行く
    #define ACC_X_CUT_OFF 20.0    // 4G - 16G
    #define ACC_Y_CUT_OFF 20.0    // 4G - 16G
    #define ACC_Z_CUT_OFF_P 20.0  // 4G - 16G
    #define ACC_Z_CUT_OFF_M -20.0 // 4G - 16G

	#define ACC_ZERO_Z_OVER 1500


	// 速度を Cut Off
	//#define VX_CUT_OFF 0.3
	//#define VX_CUT_OFF 0.25
	//#define VX_CUT_OFF 0.08
	#define VX_CUT_OFF 0.05

	//#define VY_CUT_OFF 0.3
	//#define VY_CUT_OFF 0.25
	//#define VY_CUT_OFF 0.08
	#define VY_CUT_OFF 0.05

	//#define VZ_CUT_OFF 0.3
	//#define VZ_CUT_OFF 0.25
	//#define VZ_CUT_OFF 0.08
	#define VZ_CUT_OFF 0.05

#else
	#define ACC_1G  4096.0
	#define ACC_ZERO_OFF 0.15		// + で上へ行く。 - で下に行く
    #define ACC_X_CUT_OFF 20.0    // 4G - 16G
    #define ACC_Y_CUT_OFF 20.0    // 4G - 16G
    #define ACC_Z_CUT_OFF_P 20.0  // 4G - 16G
    #define ACC_Z_CUT_OFF_M -20.0 // 4G - 16G

	#define ACC_ZERO_Z_OVER 750

	// 速度を Cut Off
	//#define VX_CUT_OFF 0.3
	//#define VX_CUT_OFF 0.25
	//#define VX_CUT_OFF 0.08
	#define VX_CUT_OFF 0.05

	//#define VY_CUT_OFF 0.3
	//#define VY_CUT_OFF 0.25
	//#define VY_CUT_OFF 0.08
	#define VY_CUT_OFF 0.05

	//#define VZ_CUT_OFF 0.3
	//#define VZ_CUT_OFF 0.25
	//#define VZ_CUT_OFF 0.08
	#define VZ_CUT_OFF 0.05

#endif


#if defined(USE_SPARK_LIB)
	#include <ICM_20948.h>  		// Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#else
	#include "ICM20948_spi.h"
#endif

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
	float    accADC_BD[3];	// add by nishi from ICM-20948
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

	char calibratingG_f;
	char calibratingA_f;
	char calibratingD_f;

	// AK8963 get calibration data
	int16_t AK8963_ASA[3];

	double quat[4];		// add by nishi
	int32_t quatRAW[4];		// add by nishi
	int32_t quatZero[4];		// add by nishi

	//aRes = 9.80665/16384.0;    // 2g  -> 0.0005985504150390625
	//aRes = 9.80665/8192.0;     // 4g	-> 0.001197100830078125
	//aRes = 9.80665/4096.0;     // 8g	-> 0.00239420166015625
	//aRes = 9.80665/2048.0;     // 16g	-> 0.0047884033203125
	float aRes = 9.80665/ACC_1G;

	float gRes = 1.0/16.4;   // 2000dps  -> 0.06097560975609757
	float mRes = 0.15; // Sensitivity Scale Factor = 0.15
	float zero_off;

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

private:
	#ifdef USE_SPARK_LIB
		ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object
	#else
		cICM20948_spi _spi;
	#endif

	float invSqrt(float x);
};

#endif