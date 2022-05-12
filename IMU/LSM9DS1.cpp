/*
* LSM9DS1.cpp
*/
#include <Arduino.h>
#include <SPI.h>
#include "LSM9DS1.h"


//#define SPI_CS_PIN          BDPIN_SPI_CS_IMU
                            
#define ICM20948_ADDRESS    0xEA
#define MPU_CALI_COUNT      512


//#define ACC_ORIENTATION(X, Y, Z)  {accADC[PITCH]  = -X; accADC[ROLL]  =  Y; accADC[YAW]  =   Z;}
//#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[PITCH] =  Y; gyroADC[ROLL] =  X; gyroADC[YAW] =   Z;}

#define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  X; accADC[PITCH]  =  -Y;  accADC[YAW]  =   Z;}
#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  X; gyroADC[PITCH] =  -Y;  gyroADC[YAW] =   Z;}


//#define DEBUG_M

float cLSM9DS1::invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	y = y * (1.5f - (halfx * y * y));
	return y;
}

/*----------------------------------
*  cLSM9DS1 constructor
-----------------------------------*/
cLSM9DS1::cLSM9DS1()
{
	calibratingG = 0;
	calibratingA = 0;
  calibratingM = 0;
  bConnected   = false;

	aRes = ACC_RES_FAC;
	gRes = GYRO_RES_FAC;
  mRes = 0.14;          // Magnetic FS = ±4 gaus -> Sensitivity Scale Factor = 0.14

  // ACC Z 軸の Zero 調整
  // Z 軸の停止時の中心が、 0 になるように調整します。
  zero_off = ACC_ZERO_OFF;
}

#ifdef USE_SPARK_LIB
/*----------------------------------
*  cLSM9DS1 begin()
-----------------------------------*/
bool cLSM9DS1::begin()
{
  int cnt;
  uint8_t data;

  bool initialized = false;
  cnt = 2;

  while (!initialized){
    //if (myICM.beginSPI(BDPIN_SPI_CS_IMU, BDPIN_SPI_M_CS_IMU) == false) // note, we need to sent this our CS pins (defined above)
    if (LSM9DS1::beginSPI(BDPIN_SPI_CS_IMU, BDPIN_SPI_M_CS_IMU) == false) // note, we need to sent this our CS pins (defined above)
    {
      Serial.println("Failed to communicate with LSM9DS1.");
      SERIAL_PORT.println("Trying again...");
      delay(500);
      //SERIAL_PORT.println("please Ent!");
      //WAITFORINPUT()
      cnt--;
      if(cnt <=0){
        return false;
      }
    }
    else{
      initialized = true;
    }
  }

  // init LSM9DS1 register
  bConnected = init();

  if(bConnected==true){
    // init cLSM9DS1 routine
    gyro_init();
    acc_init();
    mag_init();
    #ifdef USE_DMP_NISHI
    dmp_init();
    #endif
  }
  return bConnected;
}
#endif

#ifdef USE_SPARK_LIB
/*----------------------------------
* init LSM9DS1 register
-----------------------------------*/
bool cLSM9DS1::init( void ){
  uint8_t state;
  uint8_t data;
  uint8_t response[3] = {0, 0, 0};

  bool rc=true;

  // In this advanced example we'll cover how to do a more fine-grained setup of your sensor
  SERIAL_PORT.println("Device connected!");

  // この後で、セッティングを変更します。
	LSM9DS1::settings.gyro.enabled = true;
	LSM9DS1::settings.gyro.enableX = true;
	LSM9DS1::settings.gyro.enableY = true;
	LSM9DS1::settings.gyro.enableZ = true;
  
	// gyro scale can be 245, 500, or 2000
	//LSM9DS1::settings.gyro.scale = 2000;
	LSM9DS1::settings.gyro.scale = 500;
	// gyro sample rate: value between 1-6
	// 1 = 14.9    4 = 238
	// 2 = 59.5    5 = 476
	// 3 = 119     6 = 952
	LSM9DS1::settings.gyro.sampleRate = 6;
	// gyro cutoff frequency: value between 0-3
	// Actual value of cutoff frequency depends
	// on sample rate.
	LSM9DS1::settings.gyro.bandwidth = 0;
	LSM9DS1::settings.gyro.lowPowerEnable = false;
	LSM9DS1::settings.gyro.HPFEnable = false;
	// Gyro HPF cutoff frequency: value between 0-9
	// Actual value depends on sample rate. Only applies
	// if gyroHPFEnable is true.
	LSM9DS1::settings.gyro.HPFCutoff = 0;
	LSM9DS1::settings.gyro.flipX = false;
	LSM9DS1::settings.gyro.flipY = false;
	LSM9DS1::settings.gyro.flipZ = false;
	LSM9DS1::settings.gyro.orientation = 0;
	LSM9DS1::settings.gyro.latchInterrupt = true;

	LSM9DS1::settings.accel.enabled = true;
	LSM9DS1::settings.accel.enableX = true;
	LSM9DS1::settings.accel.enableY = true;
	LSM9DS1::settings.accel.enableZ = true;
	// accel scale can be 2, 4, 8, or 16
  #if defined(USE_ACC_2G)
	  LSM9DS1::settings.accel.scale = 2;
  #elif defined(USE_ACC_4G)
	  LSM9DS1::settings.accel.scale = 4;
  #elif defined(USE_ACC_8G)
	  LSM9DS1::settings.accel.scale = 8;
  #else
	  LSM9DS1::settings.accel.scale = 16;
  #endif

	// accel sample rate can be 1-6
	// 1 = 10 Hz    4 = 238 Hz
	// 2 = 50 Hz    5 = 476 Hz
	// 3 = 119 Hz   6 = 952 Hz
	LSM9DS1::settings.accel.sampleRate = 6;

	// Accel cutoff freqeuncy can be any value between -1 - 3. 
	// -1 = bandwidth determined by sample rate
	// 0 = 408 Hz   2 = 105 Hz
	// 1 = 211 Hz   3 = 50 Hz
	LSM9DS1::settings.accel.bandwidth = -1;
	LSM9DS1::settings.accel.highResEnable = false;
	// accelHighResBandwidth can be any value between 0-3
	// LP cutoff is set to a factor of sample rate
	// 0 = ODR/50    2 = ODR/9
	// 1 = ODR/100   3 = ODR/400
	LSM9DS1::settings.accel.highResBandwidth = 0;

	//LSM9DS1::settings.mag.enabled = true;
	LSM9DS1::settings.mag.enabled = false;

	// mag scale can be 4, 8, 12, or 16
	LSM9DS1::settings.mag.scale = 4;
	// mag data rate can be 0-7
	// 0 = 0.625 Hz  4 = 10 Hz
	// 1 = 1.25 Hz   5 = 20 Hz
	// 2 = 2.5 Hz    6 = 40 Hz
	// 3 = 5 Hz      7 = 80 Hz
	LSM9DS1::settings.mag.sampleRate = 7;
	LSM9DS1::settings.mag.tempCompensationEnable = false;
	// magPerformance can be any value between 0-3
	// 0 = Low power mode      2 = high performance
	// 1 = medium performance  3 = ultra-high performance
	LSM9DS1::settings.mag.XYPerformance = 3;
	LSM9DS1::settings.mag.ZPerformance = 3;
	LSM9DS1::settings.mag.lowPowerEnable = false;
	// magOperatingMode can be 0-2
	// 0 = continuous conversion
	// 1 = single-conversion
	// 2 = power down
	LSM9DS1::settings.mag.operatingMode = 0;

	//LSM9DS1::settings.temp.enabled = true;
	LSM9DS1::settings.temp.enabled = false;

  // Gyro and Acc を使う
  #if defined(USE_ACC_NISHI)
    LSM9DS1::initAccel();
  #endif
  #if defined(USE_GRYO_NISHI)
    LSM9DS1::initGyro();
  #endif
  // Choose whether or not to start the magnetometer
  #if defined(USE_MAG)
    LSM9DS1::initMag();
  #endif

  #ifdef USE_DMP_NISHI
  #endif

  return rc;
}

#endif

/*----------------------------------
*  cLSM9DS1 gyro_init()
-----------------------------------*/
void cLSM9DS1::gyro_init( void ){
	uint8_t i;
	for( i=0; i<3; i++ ){
    gyroADC[i]  = 0;
		gyroZero[i] = 0;
		gyroRAW[i]  = 0;
	}
  calibratingG_f = 0;
  calibratingG = 0; 
	//calibratingG = MPU_CALI_COUNT;
}

#ifdef USE_SPARK_LIB
/*----------------------------------
*  cLSM9DS1 gyro_get_adc()
-----------------------------------*/
void cLSM9DS1::gyro_get_adc( void ){
	int16_t x = 0;
	int16_t y = 0;
	int16_t z = 0;

  if( bConnected == true ){

    LSM9DS1::readGyro();

  	gyroRAW[0] = x = LSM9DS1::gx;
  	gyroRAW[1] = y = LSM9DS1::gy;
  	gyroRAW[2] = z = LSM9DS1::gz;

  	GYRO_ORIENTATION( x, y,z );
  }
  gyro_common();
}

#endif

/*----------------------------------
*  cLSM9DS1 gyro_cali_start()
-----------------------------------*/
void cLSM9DS1::gyro_cali_start(){
	//calibratingG = MPU_CALI_COUNT;
  calibratingG_f = 0;
  calibratingG = 0; 
}

/*----------------------------------
*  cLSM9DS1 gyro_common()
-----------------------------------*/
void cLSM9DS1::gyro_common(){
	static int32_t g[3];

  if(calibratingG_f == 0){
    calibratingG++;
    if(calibratingG >= 800){
      if(calibratingG == 800){
        g[0]=0;
        g[1]=0;
        g[2]=0;
      }
			g[0] += gyroADC[0];             // Sum up 512 readings
			g[1] += gyroADC[1];             // Sum up 512 readings
			g[2] += gyroADC[2];             // Sum up 512 readings

      if(calibratingG >= 1600){
        gyroZero[0] = g[0] / 800;
        gyroZero[1] = g[1] / 800;
        gyroZero[2] = g[2] / 800;
        //gyroZeroSum=gyroZero[0]+gyroZero[1]+gyroZero[2];
        calibratingG_f=1;
        calibratingG=0;
      }
    }
  }
  gyroADC[0] -= gyroZero[0];
  gyroADC[1] -= gyroZero[1];
  gyroADC[2] -= gyroZero[2];


  // GYRO 0 noise cut off
  // MadgwickAHRS.cpp and FusionAhrs.c では、gyro値 = 0.0 だとアクセスエラーと
  // 判定するみたいだ。 by nishi 2022.5.12
  //for (int axis = 0; axis < 3; axis++){
  //  if (abs(gyroADC[axis]) <= GYRO_NOISE_CUT_OFF){
  //    gyroADC[axis] = 0;
  //  }
  //}
}

/*----------------------------------
*  cLSM9DS1 acc_init()
-----------------------------------*/
void cLSM9DS1::acc_init( void ){
  uint8_t i;
  for( i=0; i<3; i++ ){
    accADC[i]   = 0;
		accZero[i]  = 0;
    accRAW[i]   = 0;
  }

  calibratingA_f = 0;
  calibratingA = 0;    // add by nishi 2022.1.23
  accIMZero=0.0;
}

#ifdef USE_SPARK_LIB
/*----------------------------------
*  cLSM9DS1 acc_get_adc()
-----------------------------------*/
void cLSM9DS1::acc_get_adc( void ){
	int16_t x = 0;
	int16_t y = 0;
	int16_t z = 0;
  //uint8_t rawADC[6];

  if( bConnected == true ){
    LSM9DS1::readAccel();
	}

  accRAW[0] = x = LSM9DS1::ax;
  accRAW[1] = y = LSM9DS1::ay;
  accRAW[2] = z = LSM9DS1::az;

  // レゾルーションを掛けた値?  agmt.fss.a を元に、G に変換した値
  //switch (agmt.fss.a){
  //case 0: -> (((float)axis_val) / 16.384);
  //case 1: -> (((float)axis_val) / 8.192);
  //case 2: -> (((float)axis_val) / 4.096);
  //case 3: -> (((float)axis_val) / 2.048);
  //accADC_BD[0] = LSM9DS1::accX();
  //accADC_BD[1] = LSM9DS1::accY();
  //accADC_BD[2] = LSM9DS1::accZ();

  ACC_ORIENTATION( x,	y, z );
	acc_common();
}

#endif

/*----------------------------------
*  cLSM9DS1 acc_common()
-----------------------------------*/
/*
* get Average of Acc value
* from start to 512 
* get Average for each acc x,y,z
* but z is set to zero;
*/
void cLSM9DS1::acc_common(){
	static int32_t a[3];

  if(calibratingA_f == 0){
    calibratingA++;
    if(calibratingA >= 800){
      // Z 軸のノイズを取り除く
      if(abs(accADC[2] - ACC_1G) > ACC_ZERO_Z_OVER){
        calibratingA--;
        return;
      }
      if(calibratingA == 800){
        a[0]=0;
        a[1]=0;
        a[2]=0;
      }
			a[0] += accADC[0];             // Sum up 512 readings
			a[1] += accADC[1];             // Sum up 512 readings
			a[2] += accADC[2];             // Sum up 512 readings

      if(calibratingA >= 1600){
        accZero[0] = a[0] / 800;
        accZero[1] = a[1] / 800;
        accZero[2] = a[2] / 800;
        accZeroSum=accZero[0]+accZero[1]+accZero[2];

        // 此処で、acc の内積を出す。
        //accIMZero = sqrt(accZero[0] * accZero[0] + accZero[1] * accZero[1] + accZero[2] * accZero[2]);
        //accZero[YAW] -= ACC_1G;
        //accZero[YAW] = 0;   // 注) これをすると、海抜からの標高値になる。しなければ、起動地点の標高が原点となる。

        calibratingA_f=1;
        calibratingA=0;
      }
    }
  }
}

/*----------------------------------
*  cLSM9DS1 mag_init()
-----------------------------------*/
void cLSM9DS1::mag_init( void )
{
  uint8_t i;

  for( i=0; i<3; i++ )
  {
    magADC[i]   = 0;
		magZero[i]  = 0;
    magRAW[i]   = 0;
  }
  calibratingM = MPU_CALI_COUNT;
}

#ifdef USE_SPARK_LIB
/*----------------------------------
*  cLSM9DS1 mag_get_adc()
-----------------------------------*/
bool cLSM9DS1::mag_get_adc( void )
{
	//int16_t x = 0;
	//int16_t y = 0;
	//int16_t z = 0;

  //uint8_t data[8];

  magADC[0]=magADC[1]=magADC[2]=0.0f;

  if( bConnected == true )
  {

    LSM9DS1::readMag();

    magRAW[0] =magRAW_BD[0] = LSM9DS1::mx;
  	magRAW[1] =magRAW_BD[1] = LSM9DS1::my;
  	magRAW[2] =magRAW_BD[2] = LSM9DS1::mz;
    //  AK8963_ASA[3] -> AK8963 キャリブレーションデータだが、AK09916 は、無し。
  	//magRAW[0] = ((long)magRAW[0] * AK8963_ASA[0]) >> 8;
  	//magRAW[1] = ((long)magRAW[1] * AK8963_ASA[1]) >> 8;
  	//magRAW[2] = ((long)magRAW[2] * AK8963_ASA[2]) >> 8;


    //SERIAL_PORT.print(F("magRAW[0]:"));
    //SERIAL_PORT.print(magRAW[0], 3);
    //SERIAL_PORT.print(F(" magRAW[1]:"));
    //SERIAL_PORT.print(magRAW[1], 3);
    //SERIAL_PORT.print(F(" magRAW[2]:"));
    //SERIAL_PORT.println(magRAW[2], 3);

	}

	mag_common();
  return true;
}

#endif

/*----------------------------------
*  cLSM9DS1 mag_common()
-----------------------------------*/
void cLSM9DS1::mag_common()
{

	static int32_t m[3];

	if (calibratingM>0)
	{
		calibratingM--;
		for (uint8_t axis = 0; axis < 3; axis++)
		{
			if (calibratingM ==(MPU_CALI_COUNT-1)) m[axis]=0;  // Reset a[axis] at start of calibration
			m[axis] += magRAW[axis];             // Sum up 512 readings
			magZero[axis] = m[axis]>>9;          // Calculate average, only the last itteration where (calibratingA == 0) is relevant
		}
		if (calibratingM == 0)
		{
			//accZero[YAW] -= ACC_1G;
      //magZero[YAW] = 0;
		}
	}

  // Sensitivity Scale Factor = 0.15 add by nishi 2021.11.4
  magADC[0] =  (float)(magRAW[0] - magZero[0]) * MAG_UT_LSB ;
  magADC[1] =  (float)(magRAW[1] - magZero[1]) * MAG_UT_LSB ;
  magADC[2] =  (float)(magRAW[2] - magZero[2]) * MAG_UT_LSB;

}

/*----------------------------------
*  cLSM9DS1 acc_cali_start()
-----------------------------------*/
void cLSM9DS1::acc_cali_start()
{
	//calibratingA = MPU_CALI_COUNT;
  calibratingA_f = 0;
  calibratingA = 0; 
}

/*----------------------------------
*  cLSM9DS1 acc_cali_get_done()
-----------------------------------*/
bool cLSM9DS1::acc_cali_get_done()
{
	if( calibratingA_f !=0 ) return true;
	else                    return false;
}

/*----------------------------------
*  cLSM9DS1 gyro_cali_get_done()
-----------------------------------*/
bool cLSM9DS1::gyro_cali_get_done()
{
	if( calibratingG_f != 0 ) return true;
	else                    return false;
}

