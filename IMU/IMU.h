//----------------------------------------------------------------------------
//    프로그램명 	: IMU
//
//    만든이     	: Made by Baram ( chcbaram@paran.com )
//
//    날  짜     :
//
//    최종 수정  	:
//
//    MPU_Type	:
//
//    파일명     	: IMU.h
//----------------------------------------------------------------------------
#ifndef _IMU_H_
#define _IMU_H_

#include <inttypes.h>
#include <Arduino.h>

#include <SPI.h>
// changed by nishi
//#include "MPU9250.h"
//#include "MPU6500.h"
//#include "ICM20648.h"
#include "ICM20948.h"
#include "MadgwickAHRS.h"

// changed by nishi
//#include "imu_selector.h"

#define IMU_OK			  0x00
#define IMU_ERR_I2C		0x01

// IMU による、位置の計算
//#define USE_IMU_DIST

class cIMU
{
public:
	//cMPU9250 SEN;
	//cMPU6500 SEN;
  // changed by nishi
  //cIMUDevice SEN;
  //cICM20648 SEN;
  cICM20948 SEN;
  
  
	int16_t angle[3];
  float   rpy[3];
  float   quat[4];
  double  quat_dmp[4];  // add by nishi
  int16_t gyroData[3];
  int16_t gyroRaw[3];
  int16_t accData[3];
  int16_t accRaw[3];
  int16_t magData[3];
  int16_t magRaw[3];

  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;

	bool bConnected;

  float aRes;
  float gRes;
  float mRes;

  float tf_dlt[3];
  float v_acc[3];

public:
	cIMU();

	//uint8_t  begin( uint32_t hz = 200 );
	//uint8_t  begin( uint32_t hz = 100 );
	//uint8_t  begin( uint32_t hz = 400 );
	uint8_t  begin( uint32_t hz = 800 );
	uint16_t update( uint32_t option = 0 );

  void QuaternionToEulerAngles(double q0, double q1, double q2, double q3,
                             double& roll, double& pitch, double& yaw);


private:
  Madgwick filter;

  uint32_t update_hz;
  uint32_t update_us;

	void computeIMU( void );

  #ifdef ICM20948_IMU
  // Offsets applied to raw x/y/z mag values
  float mag_offsets[3]            = { -1.76F, 22.54F, 4.43F };
  // Soft iron error compensation matrix
  float mag_softiron_matrix[3][3] = { {  0.954,  -0.019,  0.003 },
                                      {  -0.019,  1.059, -0.009 },
                                      {  0.003,  -0.009,  0.990 } };
  float mag_field_strength        = 29.85F;

  #endif

  void computeTF( void);
  void compCBvBdt(float q[4],float vB[3],double dt,double *Pn);

};


#endif
