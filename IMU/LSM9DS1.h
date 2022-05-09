/*
* LSM9DS1.h
*/

#ifndef _LSM9DS1_H_
#define _LSM9DS1_H_

#include <inttypes.h>
#include <Arduino.h>
#include "Define.h"

// add by nishi
#define LSM9DS1_IMU
#define USE_SPARK_LIB

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

// Sensitivity Scale Factor
// Linear acceleration FS = ±2 g  -> 0.061 [mg/LSB]
// Linear acceleration FS = ±4 g  -> 0.122 [mg/LSB]
// Linear acceleration FS = ±8 g  -> 0.244 [mg/LSB]
// Linear acceleration FS = ±16 g  -> 0.732 [mg/LSB]

//#define SENSITIVITY_ACCELEROMETER_2  0.000061	// [g/LSB]
//#define SENSITIVITY_ACCELEROMETER_4  0.000122
//#define SENSITIVITY_ACCELEROMETER_8  0.000244
//#define SENSITIVITY_ACCELEROMETER_16 0.000732

// Angular rate sensitivity
// Angular rate FS = ±245 dps -> 8.75 [mdps/LSB]
// Angular rate FS = ±500 dps -> 17.50 [mdps/LSB]
// Angular rate FS = ±2000 dps -> 70 [mdps/LSB]

// #define SENSITIVITY_GYROSCOPE_245    0.00875		// [dps/LSB]
// #define SENSITIVITY_GYROSCOPE_500    0.0175
// #define SENSITIVITY_GYROSCOPE_2000   0.07

//#define GYRO_RES_FAC 0.07
#define GYRO_RES_FAC 0.0175

//#define GYRO_RES_FAC 0.0609
//gRes = 1.0/16.4;   // 2000dps  -> 0.06097560975609757

#define GYRO_NOISE_CUT_OFF 6

#if defined(USE_ACC_2G)
	// 1G(9.8 m/s2)
	// 9.80665 / 0.00061 = 16076.475409836065
	// 実測 acc_x:200  acc_y:450  acc_z: 17600
	// 17600 - 16076 = 1524
	#define ACC_1G 16076
	#define ACC_RES_FAC  0.00061
	//#define ACC_ZERO_OFF 10.0		// + で上へ行く。 - で下に行く
	#define ACC_ZERO_OFF -10.0		// + で上へ行く。 - で下に行く
    #define ACC_X_CUT_OFF 80.0    // 2G  with Low pass filter  ICM20948_ACCEL_BW_6HZ
    #define ACC_Y_CUT_OFF 80.0    //  
    #define ACC_Z_CUT_OFF_P 85.0  // 
	#define ACC_Z_CUT_OFF_M -85.0 // 

    //#define ACC_X_CUT_OFF 300.0    // 2G  without Low pass filter
    //#define ACC_Y_CUT_OFF 300.0    // 
    //#define ACC_Z_CUT_OFF_P 300.0  // 
    //#define ACC_Z_CUT_OFF_M -300.0 // 

    //#define ACC_X_CUT_OFF 150.0    // 2G  without Low pass filter
    //#define ACC_Y_CUT_OFF 150.0    // 
    //#define ACC_Z_CUT_OFF_P 150.0  // 
    //#define ACC_Z_CUT_OFF_M -150.0 // 

	#define ACC_ZERO_Z_OVER 3000
	//#define ACC_ZERO_Z_OVER 4000

	// 速度を Cut Off
	//#define VX_CUT_OFF 0.3
	#define VX_CUT_OFF 0.005
	//#define VY_CUT_OFF 0.3
	#define VY_CUT_OFF 0.005
	//#define VZ_CUT_OFF 0.3
	#define VZ_CUT_OFF 0.005

	#define VX_CUT_OFF_PRE 0.4
	#define VY_CUT_OFF_PRE 0.4
	#define VZ_CUT_OFF_PRE 0.4

#elif defined(USE_ACC_4G)
	// 0.980665 / 0.000122 = 8038.237704918033
	#define ACC_1_G 8038
	#define ACC_RES_FAC  0.00122
	#define ACC_ZERO_OFF -0.1
    //#define ACC_X_CUT_OFF 20.0    // 4G - 16G
    //#define ACC_Y_CUT_OFF 20.0    // 4G - 16G
    //#define ACC_Z_CUT_OFF_P 20.0  // 4G - 16G
    //#define ACC_Z_CUT_OFF_M -20.0 // 4G - 16G

	#define ACC_ZERO_Z_OVER 1500

#elif defined(USE_ACC_8G)
	// 0.980665 / 0.000244 = 4019.1188524590166
	#define ACC_1_G 4019
	#define ACC_RES_FAC  0.00244
	#define ACC_ZERO_OFF 0.15
    //#define ACC_X_CUT_OFF 20.0    // 4G - 16G
    //#define ACC_Y_CUT_OFF 20.0    // 4G - 16G
    //#define ACC_Z_CUT_OFF_P 20.0  // 4G - 16G
    //#define ACC_Z_CUT_OFF_M -20.0 // 4G - 16G

	#define ACC_ZERO_Z_OVER 750

	// 速度を Cut Off
	//#define VX_CUT_OFF 0.3
	#define VX_CUT_OFF 0.005
	//#define VY_CUT_OFF 0.3
	#define VY_CUT_OFF 0.005
	//#define VZ_CUT_OFF 0.3
	#define VZ_CUT_OFF 0.005

	#define VX_CUT_OFF_PRE 0.4
	#define VY_CUT_OFF_PRE 0.4
	#define VZ_CUT_OFF_PRE 0.4

#else
	// 0.980665 / 0.000244 = 4019.1188524590166
	#define ACC_1_G 2048.0
	#define ACC_RES_FAC  0.00732
	#define ACC_ZERO_OFF 0.15
    //#define ACC_X_CUT_OFF 20.0    // 4G - 16G
    //#define ACC_Y_CUT_OFF 20.0    // 4G - 16G
    //#define ACC_Z_CUT_OFF_P 20.0  // 4G - 16G
    //#define ACC_Z_CUT_OFF_M -20.0 // 4G - 16G

	#define ACC_ZERO_Z_OVER 375

	// 速度を Cut Off
	//#define VX_CUT_OFF 0.3
	#define VX_CUT_OFF 0.005
	//#define VY_CUT_OFF 0.3
	#define VY_CUT_OFF 0.005
	//#define VZ_CUT_OFF 0.3
	#define VZ_CUT_OFF 0.005

	#define VX_CUT_OFF_PRE 0.4
	#define VY_CUT_OFF_PRE 0.4
	#define VZ_CUT_OFF_PRE 0.4

#endif
    //#define ACC_X_CUT_OFF 16.0
    //#define ACC_Y_CUT_OFF 16.0
    //#define ACC_Z_CUT_OFF_P 16.0
    //#define ACC_Z_CUT_OFF_M -16.0


#ifdef USE_SPARK_LIB
	#include <SparkFunLSM9DS1.h>
#else
	//#include "ICM20948_REGS.h"
#endif


//#define MPU_SPI   SPI_IMU
// changed by nishi
#define MPU_SPI   SPI

// add by nishi
//#define SS_PIN   5
#define BDPIN_SPI_CS_IMU 5
#define BDPIN_SPI_M_CS_IMU 16

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

class cLSM9DS1 : public LSM9DS1
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

	float aRes;
	float gRes;
	float mRes;          	// Magnetic FS = ±4 gaus -> Sensitivity Scale Factor = 0.14
	float zero_off;

	#ifndef USE_SPARK_LIB
	#endif

public:
	cLSM9DS1();

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

	void mag_init( void );
	bool mag_get_adc( void );
	void mag_common();
	void mag_cali_start();
	bool mag_cali_get_done();

	//void AK09916_init(bool minimal=false);


private:

	float invSqrt(float x);
};

#endif