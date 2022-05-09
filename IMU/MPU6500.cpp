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
//    파일명     	: MPU6050.cpp
//----------------------------------------------------------------------------



#include <Arduino.h>
#include <SPI.h>
#include "MPU6500.h"
#include "imu_spi.h"


//#define MPU_CS_PIN          BDPIN_SPI_CS_IMU
// chande by nishi
#define MPU_CS_PIN          5
#define MPU6500_ADDRESS     0x68
#define MPU_CALI_COUNT      512


//#define ACC_ORIENTATION(X, Y, Z)  {accADC[PITCH]  = -X; accADC[ROLL]  =  Y; accADC[YAW]  =   Z;}
//#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[PITCH] =  Y; gyroADC[ROLL] =  X; gyroADC[YAW] =   Z;}

//#define ACC_ORIENTATION(X, Y, Z)  {accADC[PITCH]  =  Y; accADC[ROLL]  =  X; accADC[YAW]  =   Z;}
//#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[PITCH] =  Y; gyroADC[ROLL] =  X; gyroADC[YAW] =   Z;}

#define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  X; accADC[PITCH]  =  Y; accADC[YAW]  =   Z;}
#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  X; gyroADC[PITCH] =  Y; gyroADC[YAW] =   Z;}


/*---------------------------------------------------------------------------
     TITLE   : cMPU6500
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
cMPU6500::cMPU6500()
{
	calibratingG = 0;
	calibratingA = 0;
	calibratingM = 0;
	bConnected   = false;

  //aRes = 9.80665/16384.0;    // 2g
  //aRes = 9.80665/8192.0;     // 4g
  //aRes = 9.80665/4096.0;     // 8g
  //aRes = 9.80665/2048.0;     // 16g
  aRes = 9.80665/ACC_1G; 

  gRes = 1.0/16.4;   // 2000dps

  //mRes = 10.*4912./8190.;  // 14BIT
  mRes = 10.*4912./32760.; // 16BIT

  zero_off=0.0;

}

/*---------------------------------------------------------------------------
     TITLE   : cMPU6500
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
bool cMPU6500::begin(){

  pinMode( MPU_CS_PIN, OUTPUT );

  //MPU_SPI.begin();
  // changed by nishi
  MPU_SPI.begin(18,19,23,5);

  MPU_SPI.setDataMode( SPI_MODE3 );
  //MPU_SPI.setDataMode( SPI_MODE0 );     // changed by nishi 2022.5.22
  MPU_SPI.setBitOrder( MSBFIRST );

  // This defines are not representing the real Divider of the ESP32
  // the Defines match to an AVR Arduino on 16MHz for better compatibility
  //#define SPI_CLOCK_DIV2    0x00101001 //8 MHz
  //#define SPI_CLOCK_DIV4    0x00241001 //4 MHz
  //#define SPI_CLOCK_DIV8    0x004c1001 //2 MHz
  //#define SPI_CLOCK_DIV16   0x009c1001 //1 MHz
  //#define SPI_CLOCK_DIV32   0x013c1001 //500 KHz
  //#define SPI_CLOCK_DIV64   0x027c1001 //250 KHz
  //#define SPI_CLOCK_DIV128  0x04fc1001 //125 KHz

  // MPU-6500  6.5 SPI INTERFACE The maximum frequency of SCLK is 1MHz / 3.5.1 fSCLK = 20 MHz  どっち?
  //MPU_SPI.setClockDivider( SPI_CLOCK_DIV16 ); // 1 MHz  chaned by nishi 2022.5.2 
  //MPU_SPI.setClockDivider( SPI_CLOCK_DIV32 ); // 500 KHz
  MPU_SPI.setClockDivider( SPI_CLOCK_DIV128 ); // 500 KHz

  digitalWrite(MPU_CS_PIN, HIGH);
  //delay( 100 );
  delay( 300 );		// changed by nishi 2021.10.6

  //if( imu_spi_read(MPU6500_ADDRESS, 0x75) == 0x71 )
  // changed by nishi
  // whomai == 0x70 or 0x71 by nishi
  uint8_t whoami;
  int i=5;
  while(i>=0){
    whoami = imu_spi_read(MPU6500_ADDRESS, 0x75);
    if(whoami == 0x70) break;
    delay(100);
    i--;
  }

  if( whoami == 0x70 ){
    bConnected = true;
    init();
    #if defined(USE_GRYO_NISHI)
      gyro_init();
    #endif
    #if defined(USE_ACC_NISHI)
      acc_init();
    #endif
    #if defined(USE_MAG)
      mag_init();
    #endif

    MPU_SPI.setClockDivider( SPI_CLOCK_DIV16 ); // 1 MHz  chaned by nishi 2022.5.2 
  }
  else{
	Serial.print("cMPU6500::begin(): #1 ");
	Serial.println(whoami, HEX);
	//while(1){
	//	delay(100);
	//}
  }

  return bConnected;
}

/*
*
*  参考ページ
*   https://garberas.com/archives/206
*/
void cMPU6500::init( void ){
	uint8_t state;
	uint8_t data;
	uint8_t response[3] = {0, 0, 0};


	//MPU6500 Reset
	imu_spi_write(MPU6500_SPIx_ADDR, MPU6500_PWR_MGMT_1, MPU6500_RESET);
	delay(100);
	//MPU6500 Set Clock Source
	imu_spi_write(MPU6500_SPIx_ADDR, MPU6500_PWR_MGMT_1,  MPU6500_CLOCK_PLLGYROZ);
	//imu_spi_write(MPU6500_SPIx_ADDR, MPU6500_PWR_MGMT_1,  MPU6500_CLOCK_INTERNAL);  // changed by nishi 2022.5.1
	delay(1);


	//MPU6500 Set Interrupt
	imu_spi_write(MPU6500_SPIx_ADDR, MPU6500_INT_PIN_CFG,  MPU6500_INT_ANYRD_2CLEAR);
	delay(1);
	//imu_spi_write(MPU6500_SPIx_ADDR, MPU6500_INT_ENABLE, ENABLE);
	// changed by nishi ENABLE == 1 ?
	//imu_spi_write(MPU6500_SPIx_ADDR, MPU6500_INT_ENABLE, 1);
	imu_spi_write(MPU6500_SPIx_ADDR, MPU6500_INT_ENABLE, 0);    // changed by nishi 2022.5.1
	delay(1);
	//MPU6500 Set Sensors
	imu_spi_write(MPU6500_SPIx_ADDR, MPU6500_PWR_MGMT_2, MPU6500_XYZ_GYRO & MPU6500_XYZ_ACCEL);
	delay(1);

	//MPU6500 Set SampleRate
	//SAMPLE_RATE = Internal_Sample_Rate / (1 + SMPLRT_DIV)
	imu_spi_write(MPU6500_SPIx_ADDR, MPU6500_SMPLRT_DIV, SMPLRT_DIV);
	delay(1);

	//MPU6500 Set Full Scale Gyro Range
	//Fchoice_b[1:0] = [00] enable DLPF
	//  MPU6500_GYRO_CONFIG:0x1b -> ジャイロの感度設定 by nishi
	imu_spi_write(MPU6500_SPIx_ADDR, MPU6500_GYRO_CONFIG, (MPU6500_FSR_2000DPS << 3));
  delay(1);
	//MPU6500 Set Full Scale Accel Range PS:2G
	//  MPU6500_ACCEL_CONFIG:0x1c -> 加速度計の感度設定 by nishi
	//imu_spi_write(MPU6500_SPIx_ADDR, MPU6500_ACCEL_CONFIG, (MPU6500_FSR_2G << 3));
	imu_spi_write(MPU6500_SPIx_ADDR, MPU6500_ACCEL_CONFIG, (MPU6500_FSR_4G << 3));  // changed by nishi 2022.5.1
	delay(1);

	//MPU6500 Set Accel DLPF
	data = imu_spi_read(MPU6500_SPIx_ADDR, MPU6500_ACCEL_CONFIG2);
	//data |= MPU6500_ACCEL_DLPF_41HZ;
  data |=MPU6500_ACCEL_DLPF_5HZ;      // changed by nishi 2022.5.1
	delay(1);
	imu_spi_write(MPU6500_SPIx_ADDR, MPU6500_ACCEL_CONFIG2, data);
	delay(1);

	//MPU6500 Set Gyro DLPF
	data = imu_spi_read(MPU6500_SPIx_ADDR, MPU6500_CONFIG);
  //data |=MPU6500_GYRO_DLPF_41HZ;
  //data &= 0xc0;     // clear bit add by nishi 2022.5.1
  //data = 0x40;
  data = 0x02;
  //data |=MPU6500_GYRO_DLPF_5HZ;
	//imu_spi_write(MPU6500_SPIx_ADDR, MPU6500_CONFIG, MPU6500_GYRO_DLPF_41HZ);
	imu_spi_write(MPU6500_SPIx_ADDR, MPU6500_CONFIG, data); // changed by nishi 2022.5.1
  //delay(1);
  delay(10);

	data = imu_spi_read(MPU6500_SPIx_ADDR, MPU6500_CONFIG);
	Serial.print("cMPU6500::begin(): MPU6500_CONFIG=");
	Serial.println(MPU6500_CONFIG, HEX);    // 0x1a
  while(1){
    delay(100);
  }

	#ifdef USE_MPU6500_AK8963
		//MPU6500 Set SPI Mode
		state = imu_spi_read(MPU6500_ADDRESS, MPU6500_USER_CTRL);
		delay(1);
		imu_spi_write(MPU6500_ADDRESS, MPU6500_USER_CTRL, state | MPU6500_I2C_IF_DIS);
		delay(1);
		state = imu_spi_read(MPU6500_ADDRESS, MPU6500_USER_CTRL);
		delay(1);
		imu_spi_write(MPU6500_ADDRESS, MPU6500_USER_CTRL, state | MPU6500_I2C_MST_EN);
		delay(1);
		//////////////////////////////////////////////////////////////////////////
		//AK8963 Setup
		//reset AK8963
		imu_spi_ak8963_write(MPU6500_AK8963_I2C_ADDR, MPU6500_AK8963_CNTL2, MPU6500_AK8963_CNTL2_SRST);
		delay(1);

		imu_spi_ak8963_write(MPU6500_AK8963_I2C_ADDR, MPU6500_AK8963_CNTL, MPU6500_AK8963_POWER_DOWN);
		delay(1);
		imu_spi_ak8963_write(MPU6500_AK8963_I2C_ADDR, MPU6500_AK8963_CNTL, MPU6500_AK8963_FUSE_ROM_ACCESS);
		delay(1);

		imu_spi_ak8963_reads(MPU6500_AK8963_I2C_ADDR, MPU6500_AK8963_WIA, 3, response);
		//
		//AK8963 get calibration data
		imu_spi_ak8963_reads(MPU6500_AK8963_I2C_ADDR, MPU6500_AK8963_ASAX, 3, response);
		AK8963_ASA[0] = (int16_t)(response[0]) + 128;
		AK8963_ASA[1] = (int16_t)(response[1]) + 128;
		AK8963_ASA[2] = (int16_t)(response[2]) + 128;
		delay(1);

		imu_spi_ak8963_write(MPU6500_AK8963_I2C_ADDR, MPU6500_AK8963_CNTL, MPU6500_AK8963_POWER_DOWN);
		delay(1);
		//
		imu_spi_write(MPU6500_SPIx_ADDR, MPU6500_I2C_MST_CTRL, 0x5D);
		delay(1);
		imu_spi_write(MPU6500_SPIx_ADDR, MPU6500_I2C_SLV0_ADDR, MPU6500_AK8963_I2C_ADDR | MPU6500_I2C_READ);
		delay(1);
		imu_spi_write(MPU6500_SPIx_ADDR, MPU6500_I2C_SLV0_REG, MPU6500_AK8963_ST1);
		delay(1);
		imu_spi_write(MPU6500_SPIx_ADDR, MPU6500_I2C_SLV0_CTRL, 0x88);
		delay(1);
		//
		imu_spi_ak8963_write(MPU6500_AK8963_I2C_ADDR, MPU6500_AK8963_CNTL, MPU6500_AK8963_CONTINUOUS_MEASUREMENT);
		delay(1);
	#endif

	//
	//imu_spi_write(MPU6500_SPIx_ADDR, MPU6500_I2C_SLV4_CTRL, 0x09);
	//delay(1);
	//
	//imu_spi_write(MPU6500_SPIx_ADDR, MPU6500_I2C_MST_DELAY_CTRL, 0x81);
	//delay(100);
}

/*---------------------------------------------------------------------------
     TITLE   : gyro_init
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cMPU6500::gyro_init( void ){
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

/*---------------------------------------------------------------------------
     TITLE   : gyro_get_adc
     WORK    :
     ARG     : void
     RET     : void
     6. z軸のデータは16bitで上位ビットと下位ビットに分かれているので、アクセスをしなければならないアドレスは2つあります。
      GYRO_ZOUT_H : 0x47 , GYRO_ZOUT_H : 0x48
      上位ビットを8bitシフトをして、下位ビットと結合すれば値が取れそうだということがわかりました。
---------------------------------------------------------------------------*/
void cMPU6500::gyro_get_adc( void )
{
  int16_t x = 0;
  int16_t y = 0;
  int16_t z = 0;

  uint8_t rawADC[6];

  if( bConnected == true )
  {
    imu_spi_reads( MPU6500_ADDRESS, MPU6500_GYRO_XOUT_H, 6, rawADC );
    //imu_spi_reads( MPU6500_ADDRESS, MPU6500_GYRO_XOUT_H, 4, rawADC );
    //imu_spi_reads( MPU6500_ADDRESS, MPU6500_GYRO_ZOUT_H, 2, &rawADC[4] );

 	  x = (((int16_t)rawADC[0]) << 8) | rawADC[1];
  	y = (((int16_t)rawADC[2]) << 8) | rawADC[3];
  	z = (((int16_t)rawADC[4]) << 8) | rawADC[5];

  	gyroRAW[0] = x;
  	gyroRAW[1] = y;
  	gyroRAW[2] = z;

  	GYRO_ORIENTATION( x, y,z );
  }

  gyro_common();
}

/*---------------------------------------------------------------------------
     TITLE   : gyro_cali_start
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cMPU6500::gyro_cali_start()
{
  //calibratingG = MPU_CALI_COUNT;
  calibratingG_f = 0;
  calibratingG = 0; 

}

/*---------------------------------------------------------------------------
     TITLE   : acc_init
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cMPU6500::acc_init( void )
{
  uint8_t i;

  for( i=0; i<3; i++ )
  {
    accADC[i]   = 0;
    accZero[i]  = 0;
    accRAW[i]   = 0;
  }
  calibratingA_f = 0;
  calibratingA = 0;    // add by nishi 2022.1.23
  accIMZero=0.0;
}


/*---------------------------------------------------------------------------
     TITLE   : acc_get_adc
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cMPU6500::acc_get_adc( void )
{
	int16_t x = 0;
	int16_t y = 0;
	int16_t z = 0;
	uint8_t rawADC[6];

	if( bConnected == true )
	{
		imu_spi_reads( MPU6500_ADDRESS, MPU6500_ACCEL_XOUT_H, 6, rawADC );

		x = (((int16_t)rawADC[0]) << 8) | rawADC[1];
		y = (((int16_t)rawADC[2]) << 8) | rawADC[3];
		z = (((int16_t)rawADC[4]) << 8) | rawADC[5];

		accRAW[0] = x;
		accRAW[1] = y;
		accRAW[2] = z;

		ACC_ORIENTATION( x,	y, z );
	}

	acc_common();
}

/*---------------------------------------------------------------------------
     TITLE   : gyro_common
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cMPU6500::gyro_common(){
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
  for (int axis = 0; axis < 3; axis++){
    if (abs(gyroADC[axis]) <= GYRO_NOISE_CUT_OFF){
      gyroADC[axis] = 0;
    }
  }
}

/*---------------------------------------------------------------------------
     TITLE   : acc_common
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cMPU6500::acc_common(){
	static int32_t a[3];

  if(calibratingA_f == 0){
    calibratingA++;
    if(calibratingA >= 800){
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

/*---------------------------------------------------------------------------
     TITLE   : acc_init
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cMPU6500::mag_init( void )
{
  uint8_t i;

  for( i=0; i<3; i++ )
  {
    magADC[i]   = 0;
		magZero[i]  = 0;
    magRAW[i]   = 0;
  }
}

/*---------------------------------------------------------------------------
     TITLE   : mag_get_adc
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cMPU6500::mag_get_adc( void )
{
  return;
}







/*---------------------------------------------------------------------------
     TITLE   : mag_common
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cMPU6500::mag_common()
{
  magADC[0] = magRAW[0];
  magADC[1] = magRAW[1];
  magADC[2] = magRAW[2];
}





/*---------------------------------------------------------------------------
     TITLE   : acc_cali_start
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cMPU6500::acc_cali_start()
{
	calibratingA = MPU_CALI_COUNT;
}





/*---------------------------------------------------------------------------
     TITLE   : acc_cali_get_done
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
bool cMPU6500::acc_cali_get_done()
{
	if( calibratingA == 0 ) return true;
	else                    return false;
}




/*---------------------------------------------------------------------------
     TITLE   : gyro_cali_get_done
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
bool cMPU6500::gyro_cali_get_done()
{
	if( calibratingG == 0 ) return true;
	else                    return false;
}
