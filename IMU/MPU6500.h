//----------------------------------------------------------------------------
//    프로그램명 	: MPU6050
//
//    만든이     	: Baram ( chcbaram@paran.com )
//
//    날  짜     :
//
//    최종 수정  	:
//
//    MPU_Type	:
//
//    파일명     	: MPU6050.h
//----------------------------------------------------------------------------
#ifndef _MPU6500_H_
#define _MPU6500_H_

#include <inttypes.h>
#include <Arduino.h>

#include "Define.h"

// add by nishi
#define MPU6500_IMU
#define USE_ACC_NISHI
#define USE_GRYO_NISHI
#define USE_MADWICK

// add by nishi 2021.10.7
#define IMU_SENSER6
//#define USE_MAG

#define USE_ACC_2G
//#define USE_ACC_4G
//#define USE_ACC_8G

#define GYRO_NOISE_CUT_OFF 4

#if defined(USE_ACC_2G)
	#define ACC_1G 16384.0
	#define ACC_X_CUT_OFF 16.0
	#define ACC_Y_CUT_OFF 16.0
	#define ACC_Z_CUT_OFF_P 16.0
	#define ACC_Z_CUT_OFF_M -16.0

	#define ACC_ZERO_Z_OVER 3000

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
	#define ACC_1G 8192.0
	#define ACC_X_CUT_OFF 16.0
	#define ACC_Y_CUT_OFF 16.0
	#define ACC_Z_CUT_OFF_P 16.0
	#define ACC_Z_CUT_OFF_M -16.0

	#define ACC_ZERO_Z_OVER 3000

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
	#define ACC_1G 4096.0
	#define ACC_X_CUT_OFF 16.0
	#define ACC_Y_CUT_OFF 16.0
	#define ACC_Z_CUT_OFF_P 16.0
	#define ACC_Z_CUT_OFF_M -16.0

	#define ACC_ZERO_Z_OVER 3000

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

#include "MPU6500_REGS.h"


#define GYRO_SCALE (4 / 16.4 * PI / 180.0 / 1000000.0) //16.4 LSB = 1 deg/s


//#define MPU_SPI   SPI_IMU
// changed by nishi
#define MPU_SPI   SPI


#define SERIAL_PORT Serial


void read_regs( uint8_t addr, uint8_t reg, uint8_t *p_data, uint32_t length );


class cMPU6500
{
 public:
  bool     bConnected;

    int16_t  gyroADC[3];
    int16_t  gyroRAW[3];
    int16_t  gyroZero[3];

    int16_t  accADC[3];
    int16_t  accRAW[3];
    int16_t  accZero[3];
	int32_t	 accZeroSum;
	float    accIMZero;			// add by nishi acc 内積

    int16_t  magADC[3];
    int16_t  magRAW[3];
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

	float aRes;
	float gRes;
	float mRes;
	float zero_off;

    int16_t AK8963_ASA[3];

 public:
	cMPU6500();

    bool begin( void );
    void init( void );
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
	void mag_get_adc( void );
	void mag_common();
	void mag_cali_start();
	bool mag_cali_get_done();

};


#endif
