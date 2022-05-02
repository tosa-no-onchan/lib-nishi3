/*
* ICM20948.cpp
*/
#include <Arduino.h>
#include <SPI.h>
#include "ICM20948.h"


//#define SPI_CS_PIN          BDPIN_SPI_CS_IMU
                            
#define ICM20948_ADDRESS    0xEA
#define MPU_CALI_COUNT      512


//#define ACC_ORIENTATION(X, Y, Z)  {accADC[PITCH]  = -X; accADC[ROLL]  =  Y; accADC[YAW]  =   Z;}
//#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[PITCH] =  Y; gyroADC[ROLL] =  X; gyroADC[YAW] =   Z;}

#define ACC_ORIENTATION(X, Y, Z)  {accADC[PITCH]  =  Y; accADC[ROLL]  =  X; accADC[YAW]  =   Z;}
#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[PITCH] =  Y; gyroADC[ROLL] =  X; gyroADC[YAW] =   Z;}


//#define DEBUG_M

float cICM20948::invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	y = y * (1.5f - (halfx * y * y));
	return y;
}


cICM20948::cICM20948()
{
	calibratingG = 0;
	calibratingA = 0;
  calibratingM = 0;
  bConnected   = false;

  // ICM20948
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

  // ICM20948

  // 2G       16384.0
  // 4G       8192.0
  // 8G       4096.0
  // 16G      2048.0

  //aRes = 9.80665/16384.0;    // 2g
  //aRes = 9.80665/8192.0;     // 4g
  //aRes = 9.80665/4096.0;     // 8g
  //aRes = 9.80665/2048.0;     // 16g
  aRes = 9.80665/ACC_MAX_G; 

  gRes = 1.0/16.4;   // 2000dps
  // org setting
  // gRes = 2000.0/32768.0;   // 2000dps   -> 0.06103515625
  // gx = (float)SEN.gyroADC[0]*gRes;

  mRes = 0.15; // Sensitivity Scale Factor = 0.15

  // ACC Z 軸の Zero 調整
  // Z 軸の停止時の中心が、 0 になるように調整します。
  #if defined(USE_ACC_2G)
    //zero_off = 0.9;
    zero_off = 10.0*15.0;   // with low pass filter 
  #elif defined(USE_ACC_4G)
    zero_off = -0.1;
  #else
    zero_off = 0.15;   // +
    //zero_off = 0.12;   // +-+
    //zero_off = 0.1;    // -
  #endif

}

void cICM20948::selectBank(uint8_t bank)
{
  #ifndef USE_SPARK_LIB
    digitalWrite( BDPIN_SPI_CS_IMU, LOW);

    /* clear R/W bit - write, send the address */
    MPU_SPI.write((uint8_t)ICM20948_REG_BANK_SEL);
    MPU_SPI.write((uint8_t)(bank << 4));
  
    digitalWrite( BDPIN_SPI_CS_IMU, HIGH);
  #endif
}

void cICM20948::spiRead(uint16_t addr, uint8_t *p_data, uint32_t length)
{
  uint8_t reg_addr;
  uint8_t bank;
 
  reg_addr = (uint8_t) (addr & 0x7F);
  bank = (uint8_t) (addr >> 7);

  #ifndef USE_SPARK_LIB
    selectBank(bank);

    digitalWrite( BDPIN_SPI_CS_IMU, LOW);
  
    MPU_SPI.transfer(0x80 | reg_addr);
    //MPU_SPI.transfer(NULL, (void *)p_data, (size_t)length);
    // update by nishi
    MPU_SPI.transferBytes(NULL, (uint8_t *)p_data, (size_t)length);
  
    digitalWrite( BDPIN_SPI_CS_IMU, HIGH);
  #endif
}

uint8_t cICM20948::spiReadByte(uint16_t addr)
{
  uint8_t data;

  spiRead(addr, &data, 1);

	return data;
}

void cICM20948::spiWriteByte(uint16_t addr, uint8_t data)
{
  uint8_t reg_addr;
  uint8_t bank;
 
  reg_addr = (uint8_t) (addr & 0x7F);
  bank = (uint8_t) (addr >> 7);

  #ifndef USE_SPARK_LIB
    selectBank(bank);
    digitalWrite( BDPIN_SPI_CS_IMU, LOW); 

    MPU_SPI.transfer(reg_addr);
    MPU_SPI.transfer(data);
  
    digitalWrite( BDPIN_SPI_CS_IMU, HIGH); 
    //delay(1);
  #endif
}


//void cICM20948::spiWrite(uint16_t addr, uint8_t *data,uint32_t length)
//{
//  for(uint32_t i=0;i<length;i++){
//    spiWriteByte(addr+i, data[i]);
//  }
//}


//int  imu_spi_writes(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t *data)
void cICM20948::spiWrite(uint16_t addr, uint8_t *data, uint32_t length)
{

  uint8_t reg_addr;
  uint8_t bank;
 
  reg_addr = (uint8_t) (addr & 0x7F);
  bank = (uint8_t) (addr >> 7);

  uint32_t i;

  #ifndef USE_SPARK_LIB
    selectBank(bank);

    digitalWrite( BDPIN_SPI_CS_IMU, LOW);
    MPU_SPI.transfer( reg_addr );

    for( i=0; i<length; i++ )
    {
      MPU_SPI.transfer( data[i] );
    }
    digitalWrite( BDPIN_SPI_CS_IMU, HIGH);
    delay(1);
  #endif
}


#ifdef USE_SPARK_LIB
bool cICM20948::begin()
{
  int cnt;
  uint8_t data;
  // test by nishi MOSI(23) Pull UP 2023.12.13
  //pinMode(23,INPUT_PULLUP);
  MPU_SPI.begin(18,19,23,5);

  // test by nishi MOSI(23) Pull UP 2023.12.13
  //pinMode(23,INPUT_PULLUP);


  bool initialized = false;
  cnt = 2;
  while (!initialized)
  {
    myICM.begin(CS_PIN, MPU_SPI);

    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.println("Trying again...");
      delay(500);
      //SERIAL_PORT.println("please Ent!");
      //WAITFORINPUT()
      cnt--;
      if(cnt <=0){
        return false;
      }
    }
    else
    {
      initialized = true;
    }
  }

  // Disable I2C interface, use SPI
  //spiWriteByte(ICM20948_REG_USER_CTRL, ICM20948_BIT_I2C_IF_DIS);

  uint8_t whoami;
  //whoami = spiReadByte(ICM20948_REG_WHO_AM_I);
  whoami = myICM.getWhoAmI();
  // whoami = 0xEA  -> ICM20948
  //if(whoami == ICM20948_DEVICE_ID)
  if(whoami == 0xEA)
  {
    bConnected = init();
    if(bConnected==true){
      gyro_init();
      acc_init();
      mag_init();
      #ifdef USE_DMP_NISHI
      dmp_init();
      #endif
    }
    //MPU_SPI.setClockDivider( SPI_CLOCK_DIV4 ); // 6.5MHz
  }
  // add byy nishi
  else{
    SERIAL_PORT.print("cICM20948::begin(): #1 ");
    SERIAL_PORT.println(whoami, HEX);
    //while(1){
    //  delay(100);
    //}
  }

  return bConnected;
}

#else
bool cICM20948::begin()
{
  uint8_t data;

  pinMode( BDPIN_SPI_CS_IMU, OUTPUT );

  //MPU_SPI.begin();
  // changed by nishi
  MPU_SPI.begin(18,19,23,5);

  // test by nishi MOSI(23) Pull UP 2023.12.13
  //pinMode(23,INPUT_PULLUP);

  //MPU_SPI.setDataMode( SPI_MODE3 );
  MPU_SPI.setDataMode( SPI_MODE0 );   // changed by nishi
  MPU_SPI.setBitOrder( MSBFIRST );

  // Limit SPI frequency to 7MHz みたい。
  //#define SPI_CLOCK_DIV2    0x00101001 //8 MHz
  //#define SPI_CLOCK_DIV4    0x00241001 //4 MHz
  //#define SPI_CLOCK_DIV8    0x004c1001 //2 MHz
  //#define SPI_CLOCK_DIV16   0x009c1001 //1 MHz
  //#define SPI_CLOCK_DIV32   0x013c1001 //500 KHz
  //#define SPI_CLOCK_DIV64   0x027c1001 //250 KHz
  //#define SPI_CLOCK_DIV128  0x04fc1001 //125 KHz

  //MPU_SPI.setClockDivider( SPI_CLOCK_DIV128 ); // 108Mhz/128 = 0.8MHz  今までこちらを使用。
  MPU_SPI.setClockDivider( SPI_CLOCK_DIV4 ); // changed by nishi 2022.5.2
  digitalWrite(BDPIN_SPI_CS_IMU, HIGH);
  //delay( 100 );
  delay( 300 );		// changed by nishi 2021.10.6

  // mICM.sleep(false);
  data = spiReadByte(ICM20948_REG_PWR_MGMT_1);
  data &= 0xff - ICM20948_BIT_SLEEP;
  // mICM.lowPower(false);
  data &= 0xff - ICM20948_BIT_LP_EN;
  spiWriteByte(ICM20948_REG_PWR_MGMT_1, data);


  uint8_t whoami;
  int i=5;
  while(i>=0){
    whoami = spiReadByte(ICM20948_REG_WHO_AM_I);
    if(whoami == ICM20948_DEVICE_ID) break;
    delay(100);
    i--;
  }
  // whoami = 0xEA  -> ICM20948
  if(whoami == ICM20948_DEVICE_ID)
  {
    // Limit SPI frequency to 7MHz みたい。
    //MPU_SPI.setClockDivider( SPI_CLOCK_DIV4 ); // 4MHz  for ICM20948 これまで使用。 original

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

  }
  // add by nishi
  else{
    SERIAL_PORT.print("cICM20948::begin(): #10 ");
    SERIAL_PORT.println(whoami, HEX);
    //while(1){
    //  delay(100);
    //}
  }
  return bConnected;
}
#endif

#ifdef USE_SPARK_LIB
bool cICM20948::init( void ){
  uint8_t state;
  uint8_t data;
  uint8_t response[3] = {0, 0, 0};

  bool rc=true;

  // In this advanced example we'll cover how to do a more fine-grained setup of your sensor
  SERIAL_PORT.println("Device connected!");

  // Here we are doing a SW reset to make sure the device starts in a known state
  // リセットしたら、再度、myICM.startupMagnetometer() が必要です。
  //myICM.swReset();
  //if (myICM.status != ICM_20948_Stat_Ok)
  //{
  //  SERIAL_PORT.print(F("Software Reset returned: "));
  //  SERIAL_PORT.println(myICM.statusString());
  //}
  //delay(250);

  // Now wake the sensor up
  myICM.sleep(false);
  myICM.lowPower(false);

  // Gyro and Acc を使う
  #if defined(USE_ACC_NISHI) || defined(USE_GRYO_NISHI)

    // The next few configuration functions accept a bit-mask of sensors for which the settings should be applied.

    // Set Gyro and Accelerometer to a particular sample mode
    // options: ICM_20948_Sample_Mode_Continuous
    //          ICM_20948_Sample_Mode_Cycled
    myICM.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous);
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.print(F("setSampleMode returned: "));
      SERIAL_PORT.println(myICM.statusString());
      rc = false; // add by nishi 2022.4.23
    }

    // Set full scale ranges for both acc and gyr
    ICM_20948_fss_t myFSS; // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors

    //myFSS.a = gpm2; // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                  // gpm2
                  // gpm4
                  // gpm8
                  // gpm16
    #if defined(USE_ACC_2G)
      myFSS.a = gpm2;  // gpm2
    #elif defined(USE_ACC_4G)
      myFSS.a = gpm4;  // gpm4
    #else
      myFSS.a = gpm8;  // gpm8
    #endif

    myFSS.g = dps2000; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                      // dps250
                      // dps500
                      // dps1000
                      // dps2000

    myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.print(F("setFullScale returned: "));
      SERIAL_PORT.println(myICM.statusString());
      rc = false; // add by nishi 2022.4.23
    }


    // Set up Digital Low-Pass Filter configuration
    ICM_20948_dlpcfg_t myDLPcfg;    // Similar to FSS, this uses a configuration structure for the desired sensors
    //myDLPcfg.a = acc_d473bw_n499bw; // (ICM_20948_ACCEL_CONFIG_DLPCFG_e)
    //myDLPcfg.a = acc_d246bw_n265bw;     // acc_d246bw_n265bw      - means 3db bandwidth is 246 hz and nyquist bandwidth is 265 hz
    //myDLPcfg.a = acc_d111bw4_n136bw;   // acc_d111bw4_n136bw
    //myDLPcfg.a = acc_d50bw4_n68bw8; // acc_d50bw4_n68bw8
    //myDLPcfg.a = acc_d23bw9_n34bw4;  // acc_d23bw9_n34bw4
    //myDLPcfg.a =  acc_d11bw5_n17bw;  // acc_d11bw5_n17bw
    myDLPcfg.a =  acc_d5bw7_n8bw3;  // acc_d5bw7_n8bw3        - means 3 db bandwidth is 5.7 hz and nyquist bandwidth is 8.3 hz
                                    // acc_d473bw_n499bw

    //myDLPcfg.g = gyr_d361bw4_n376bw5; // (ICM_20948_GYRO_CONFIG_1_DLPCFG_e)
                                      // gyr_d196bw6_n229bw8
                                      // gyr_d151bw8_n187bw6
                                      // gyr_d119bw5_n154bw3
                                      // gyr_d51bw2_n73bw3
                                      // gyr_d23bw9_n35bw9
                                      // gyr_d11bw6_n17bw8
    myDLPcfg.g = gyr_d5bw7_n8bw9;     // gyr_d5bw7_n8bw9
                                      // gyr_d361bw4_n376bw5

    myICM.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg);
    //myICM.setDLPFcfg(ICM_20948_Internal_Acc , myDLPcfg);
    if (myICM.status != ICM_20948_Stat_Ok){
      SERIAL_PORT.print(F("setDLPcfg returned: "));
      SERIAL_PORT.println(myICM.statusString());
      rc = false; // add by nishi 2022.4.23
    }

    // Choose whether or not to use DLPF
    // Here we're also showing another way to access the status values, and that it is OK to supply individual sensor masks to these functions
    //ICM_20948_Status_e accDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Acc, false);
    ICM_20948_Status_e accDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Acc, true);
    //ICM_20948_Status_e gyrDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Gyr, false);
    ICM_20948_Status_e gyrDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Gyr, true);
    SERIAL_PORT.print(F("Enable DLPF for Accelerometer returned: "));
    SERIAL_PORT.println(myICM.statusString(accDLPEnableStat));
    SERIAL_PORT.print(F("Enable DLPF for Gyroscope returned: "));
    SERIAL_PORT.println(myICM.statusString(gyrDLPEnableStat));


    // Choose whether or not to start the magnetometer
    #if defined(USE_MAG)
      myICM.startupMagnetometer();
      if (myICM.status != ICM_20948_Stat_Ok){
        SERIAL_PORT.print(F("startupMagnetometer returned: "));
        SERIAL_PORT.println(myICM.statusString());
        rc = false; // add by nishi 2022.4.23
      }
    #endif

    // test by nishi
    myICM.setBank(2); // myICM.startupMagnetometer(); の後は、Bank が変わる。
    uint8_t reg_my;
    ICM_20948_Status_e rc_my = myICM.read(AGB2_REG_ACCEL_CONFIG, (uint8_t *)&reg_my, sizeof(reg_my));
    SERIAL_PORT.print("reg_my=");
    SERIAL_PORT.println(reg_my,HEX);

    SERIAL_PORT.println();
    SERIAL_PORT.println(F("Configuration complete!"));

      // DEBUG by nishi
      #ifdef DEBUG_NISHI_8
      // bank0 USER_CTRL 0x03
      myICM.setBank(0); // myICM.startupMagnetometer(); の後は、Bank が変わる。
      rc_my = myICM.read(0x03, (uint8_t *)&data, sizeof(data));
      SERIAL_PORT.print("USER_CTRL=");
      SERIAL_PORT.println(data,HEX);

      while(1){
        delay(100);
      }
      #endif
    //SERIAL_PORT.println("Please CR!");
    //WAITFORINPUT();
  #endif

  #ifdef USE_DMP_NISHI
    /*
    * add 9 axis fusion Quarternion with DMP3 start
    * 注) ICM220948C.h の #define ICM_20948_USE_DMP を有効にしないといけない。
    * 注2) 上記を、有効にすると、Magnet が取れなくなる。
    */

    bool success = true; // Use success to show if the DMP configuration was successful

    //#define QUAT_ANIMATION

    // Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to e.g. to change the sample rate
    success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);

    // DMP sensor options are defined in ICM_20948_DMP.h
    //    INV_ICM20948_SENSOR_ACCELEROMETER               (16-bit accel)
    //    INV_ICM20948_SENSOR_GYROSCOPE                   (16-bit gyro + 32-bit calibrated gyro)
    //    INV_ICM20948_SENSOR_RAW_ACCELEROMETER           (16-bit accel)
    //    INV_ICM20948_SENSOR_RAW_GYROSCOPE               (16-bit gyro + 32-bit calibrated gyro)
    //    INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED (16-bit compass)
    //    INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED      (16-bit gyro)
    //    INV_ICM20948_SENSOR_STEP_DETECTOR               (Pedometer Step Detector)
    //    INV_ICM20948_SENSOR_STEP_COUNTER                (Pedometer Step Detector)
    //    INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR        (32-bit 6-axis quaternion)
    //    INV_ICM20948_SENSOR_ROTATION_VECTOR             (32-bit 9-axis quaternion + heading accuracy)
    //    INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR (32-bit Geomag RV + heading accuracy)
    //    INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD           (32-bit calibrated compass)
    //    INV_ICM20948_SENSOR_GRAVITY                     (32-bit 6-axis quaternion)
    //    INV_ICM20948_SENSOR_LINEAR_ACCELERATION         (16-bit accel + 32-bit 6-axis quaternion)
    //    INV_ICM20948_SENSOR_ORIENTATION                 (32-bit 9-axis quaternion + heading accuracy)

    // Enable the DMP orientation sensor
    #ifndef IMU_SENSER6
      success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);   // original  2021.11.11
      //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_ROTATION_VECTOR) == ICM_20948_Stat_Ok);   // test 2
    #else
      //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GRAVITY) == ICM_20948_Stat_Ok);   // test 3
      success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);   // test 4
    #endif

    // Enable any additional sensors / features
    //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
    //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
    //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);

    // Configuring DMP to output data at multiple ODRs:
    // DMP is capable of outputting multiple sensor data at different rates to FIFO.
    // Setting value can be calculated as follows:
    // Value = (DMP running rate / ODR ) - 1
    // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
    #ifndef IMU_SENSER6
      success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok); // Set to the maximum
      //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok); // Set to the maximum
      //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 0) == ICM_20948_Stat_Ok); // Set to the maximum
      //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum
      //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass, 0) == ICM_20948_Stat_Ok); // Set to the maximum
      //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    #else
      success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok); // Set to the maximum
      //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok); // Set to the maximum
      //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 0) == ICM_20948_Stat_Ok); // Set to the maximum
      //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum
      //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass, 0) == ICM_20948_Stat_Ok); // Set to the maximum
      //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    #endif

    // Enable the FIFO
    success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);

    // Enable the DMP
    success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);

    // Reset DMP
    success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);

    // Reset FIFO
    success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

    // Check success
    if (success)
    {
      #ifndef QUAT_ANIMATION
        SERIAL_PORT.println(F("DMP enabled!"));
      #endif
    }
    else
    {
      rc=false;
      SERIAL_PORT.println(F("Enable DMP failed!"));
      SERIAL_PORT.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
      //while (1)
      //  ; // Do nothing more
    }
  #endif

  // end
  return rc;
}

#else
bool cICM20948::init( void )
{
  uint8_t state;
  uint8_t data;
  uint8_t response[3] = {0, 0, 0};
  bool rc=true;


	//ICM20948 Set Sensors
  spiWriteByte(ICM20948_REG_PWR_MGMT_2, 0x00); // Acc/Gyro Enable
	delay(1);

	//ICM20948 Set SampleRate
	//SAMPLE_RATE = Internal_Sample_Rate / (1 + SMPLRT_DIV)
  spiWriteByte(ICM20948_REG_GYRO_SMPLRT_DIV, 0x00); // 
	delay(1);

	//ICM20948 Gyro 
  // Gyro set GYRO_CONFIG_1
	// Set Full Scale Gyro Range
  data = ICM20948_GYRO_FULLSCALE_2000DPS;
  // Set Gyro DLPF
  //data |= ICM20948_GYRO_BW_51HZ;
  data |= ICM20948_GYRO_BW_6HZ;     // changed by nishi 2022.4.30
  spiWriteByte(ICM20948_REG_GYRO_CONFIG_1, data);
	delay(1);

	//ICM20948 Accel
	// Set Full Scale Accel Range
  #if defined(USE_ACC_2G)
    data = ICM20948_ACCEL_FULLSCALE_2G;
  #elif defined(USE_ACC_4G)
    data = ICM20948_ACCEL_FULLSCALE_4G;   // changed by nishi 2022.4.30
  #else
    data = ICM20948_ACCEL_FULLSCALE_8G;
  #endif

  // Set Accel DLPF
  //data |= ICM20948_ACCEL_BW_50HZ;
  data |= ICM20948_ACCEL_BW_6HZ;        // changed by nishi 2022.4.30
  spiWriteByte(ICM20948_REG_ACCEL_CONFIG, data);
	delay(1);

  #if defined(USE_MAG)
    AK09916_init();
  #endif
  return rc;
}

void cICM20948::AK09916_init(bool minimal){
	//////////////////////////////////////////////////////////////////////////
	//AK09916 Setup

  uint8_t data;

  int rc;
  ICM_20948_Status_e rc2;

  // i2cMasterPassthrough(false);
  data=spiReadByte(ICM20948_REG_INT_PIN_CFG);
  data &=0xff - ICM20948_BIT_BYPASS_EN;
  //SERIAL_PORT.print("cICM20948::AK09916_init() #10  INT_PIN_CFG=");
  //SERIAL_PORT.println(data,HEX);
  spiWriteByte(ICM20948_REG_INT_PIN_CFG,data);


  // i2cMasterEnable(true);
  // 1)
  // I2C_MST_CTRL - I2C_MST_CLK = 0x07
  //              - I2C_MST_P_NSR = 0x10
  data=spiReadByte(ICM20948_REG_I2C_MST_CTRL);
  data &=0xF0;
  data |=ICM20948_BIT_I2C_MST_P_NSR + 7;
  spiWriteByte(ICM20948_REG_I2C_MST_CTRL,data);
  // 2)
  // USERCTRL  - I2C_MST_EN = 1
  data=spiReadByte(ICM20948_REG_USER_CTRL);
  data &=0xF0;
  data |=ICM20948_BIT_I2C_MST_EN;
  spiWriteByte(ICM20948_REG_USER_CTRL,data);


  // DEBUG by nishi
  //data=spiReadByte(ICM20948_REG_I2C_MST_CTRL);
  //SERIAL_PORT.print("cICM20948::AK09916_init() #10  I2C_MST_CTRL=");
  //SERIAL_PORT.println(data,HEX);

  //#define TEST_X_1
  #ifdef TEST_X_1
    // mag who am i
    //ICM_20948_i2c_controller_periph4_txn(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len, bool Rw, bool send_reg_addr);
    data=0;
    rc2=ICM_20948_i2c_controller_periph4_txn(ICM20948_AK09916_I2C_ADDR, ICM20948_AK09916_WIA, &data, 1, true, true);
    SERIAL_PORT.print("cICM20948::AK09916_init() #19  rc2=");
    SERIAL_PORT.println(rc2);
    SERIAL_PORT.print("ICM20948_AK09916_WIA=");
    SERIAL_PORT.println(data,HEX);
    while(1){
      delay(100);
    }
  #endif

	//resetMag()
	//imu_spi_ak8963_write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL2, MPU9250_AK8963_CNTL2_SRST);
	rc=imu_spi_ak09916_write(ICM20948_AK09916_I2C_ADDR, ICM20948_AK09916_CNTL3, ICM20948_AK09916_CNTL3_SRST);    // CNTL3 SRST
  if(rc){
    SERIAL_PORT.print("cICM20948::AK09916_init() #20 error");
    SERIAL_PORT.println(rc);
    //while(1){
    //  delay(100);
    //}
  }
	delay(1);

  // magWhoIam()
  //imu_spi_ak8963_reads(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_WIA, 3, response);
  // WIA: DEVICE ID : 0x09
  imu_spi_ak09916_reads(ICM20948_AK09916_I2C_ADDR, ICM20948_AK09916_WIA2, 1, &data);
  if(data != AK09916_DIVICE_ID){
    SERIAL_PORT.print("WIA: DEVICE ID=");
    SERIAL_PORT.println(data,HEX);
  }

  // startupMagnetmeter(true) の時は、ここで終了
  if(minimal){
    return;
  }

  // ここから、EXT_SLV_SENS_DATA_00 - EXT_SLV_SENS_DATA_08 へ、自動取り込みの設定をします。


	//imu_spi_ak8963_write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_POWER_DOWN);
  // CNTL2 - Power-down mode
	rc=imu_spi_ak09916_write(ICM20948_AK09916_I2C_ADDR, ICM20948_AK09916_CNTL2, ICM20948_AK09916_POWER_DOWN); 
  if(rc){
    SERIAL_PORT.println("cICM20948::AK09916_init() #21 error");
    //while(1){
    //  delay(100);
    //}
  }

	delay(1);

  // 該当無し
	//imu_spi_ak09916_write(ICM20948_AK09916_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_FUSE_ROM_ACCESS);    // Fuse ROM access mode
	//delay(1);


	// 該当無し
	//AK8963 get calibration data
	//imu_spi_ak8963_reads(ICM20948_AK09916_I2C_ADDR, MPU9250_AK8963_ASAX, 3, response);
	//AK8963_ASA[0] = (int16_t)(response[0]) + 128;
	//AK8963_ASA[1] = (int16_t)(response[1]) + 128;
	//AK8963_ASA[2] = (int16_t)(response[2]) + 128;
	//delay(1);

  // 不要
  // CNTL2 - Power-down mode
	//imu_spi_ak8963_write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_POWER_DOWN);
  //delay(1);

  //#define TEST_NISHI_Y2
  #ifdef TEST_NISHI_Y2
    // ここは、ICM_20948_C.c ICM_20948_i2c_controller_configure_peripheral() を真似します。
    // でも、うまくいきません。

    //uint8_t periph_addr_reg;
    //uint8_t periph_reg_reg;
    //uint8_t periph_ctrl_reg;
    //uint8_t periph_do_reg;

    //periph_addr_reg = ICM20948_REG_I2C_SLV0_ADDR;
    //periph_reg_reg = ICM20948_REG_I2C_SLV0_REG;
    //periph_ctrl_reg = ICM20948_REG_I2C_SLV0_CTRL;
    //periph_do_reg = ICM20948_REG_I2C_SLV0_DO;

    // 1. Set the peripheral address and the Rw flag
    // AK09916_I2C_ADDR(0x0C) + Read Flag = 0x8C
    spiWriteByte(ICM20948_REG_I2C_SLV0_ADDR, ICM20948_AK09916_I2C_ADDR | ICM20948_I2C_READ);
    delay(1);
  
    // 2. Set the peripheral sub-address (register address)
    // reg : ST1(0x10)
    spiWriteByte(ICM20948_REG_I2C_SLV0_REG, ICM20948_AK09916_ST1);
    delay(1);

    // 3.Set up the control info
    //ICM_20948_I2C_PERIPHX_CTRL_t ctrl;
    //ctrl.LENG = len; -> 9
    //ctrl.EN = enable;  -> true
    //ctrl.REG_DIS = data_only; -> false
    //ctrl.GRP = grp; -> false
    //ctrl.BYTE_SW = swap;  -> false
    spiWriteByte(ICM20948_REG_I2C_SLV0_CTRL, 0x89);
    delay(1);

    return;
  #endif

	//imu_spi_write(MPU9250_SPIx_ADDR, MPU9250_I2C_MST_CTRL, 0x5D);
  // Bank3  I2C_MST_CTRL
	//spiWriteByte(ICM20948_REG_I2C_MST_CTRL, ICM20948_BIT_I2C_MST_P_NSR|15);     // Bank3  I2C_MST_CTRL
	spiWriteByte(ICM20948_REG_I2C_MST_CTRL, ICM20948_BIT_I2C_MST_P_NSR|7);     // Bank3  I2C_MST_CTRL
	delay(1);
	//imu_spi_write(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV0_ADDR, MPU9250_AK8963_I2C_ADDR | MPU9250_I2C_READ);
	spiWriteByte(ICM20948_REG_I2C_SLV0_ADDR, ICM20948_AK09916_I2C_ADDR | ICM20948_I2C_READ);
	delay(1);
	//imu_spi_write(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV0_REG, MPU9250_AK8963_ST1);
	spiWriteByte(ICM20948_REG_I2C_SLV0_REG, ICM20948_AK09916_ST1);
	delay(1);
	//imu_spi_write(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV0_CTRL, 0x88);
	spiWriteByte(ICM20948_REG_I2C_SLV0_CTRL, 0x89);
	delay(1);
	//
	//imu_spi_ak09916_write(ICM20948_AK09916_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_CONTINUOUS_MEASUREMENT);
	rc=imu_spi_ak09916_write(ICM20948_AK09916_I2C_ADDR, ICM20948_AK09916_CNTL2, ICM20948_AK09916_CONTINUOUS_MEASUREMENT4);
    if(rc){
    SERIAL_PORT.println("cICM20948::AK09916_init() #24 error");
    //while(1){
    //  delay(100);
    //}
  }

	delay(1);

	//
	//imu_spi_write(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_CTRL, 0x09);
	spiWriteByte(ICM20948_REG_I2C_SLV4_CTRL, 0x09);
	delay(1);
	//
	//imu_spi_write(MPU9250_SPIx_ADDR, MPU9250_I2C_MST_DELAY_CTRL, 0x81);
	spiWriteByte(ICM20948_REG_I2C_MST_DELAY_CTRL , 0x81);
	delay(100);

  //SERIAL_PORT.println("cICM20948::AK09916_init() #99");
}
#endif


void cICM20948::gyro_init( void ){
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
void cICM20948::gyro_get_adc( void ){
	int16_t x = 0;
	int16_t y = 0;
	int16_t z = 0;

  if( bConnected == true ){

  	gyroRAW[0] = x = myICM.agmt.gyr.axes.x;
  	gyroRAW[1] = y = myICM.agmt.gyr.axes.y;
  	gyroRAW[2] = z = myICM.agmt.gyr.axes.z;

  	GYRO_ORIENTATION( x, y,z );
  }
  gyro_common();
}

#else
void cICM20948::gyro_get_adc( void )
{
	int16_t x = 0;
	int16_t y = 0;
	int16_t z = 0;

  uint8_t rawADC[6];

  if( bConnected == true )
  {
    spiRead(ICM20948_REG_GYRO_XOUT_H_SH, &rawADC[0], 6);

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
#endif

void cICM20948::gyro_cali_start(){
	//calibratingG = MPU_CALI_COUNT;
  calibratingG_f = 0;
  calibratingG = 0; 
}

void cICM20948::gyro_common(){
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
}

void cICM20948::acc_init( void ){
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
void cICM20948::acc_get_adc( void ){
	int16_t x = 0;
	int16_t y = 0;
	int16_t z = 0;
  //uint8_t rawADC[6];

  if( bConnected == true ){    
    for (int i=0;i<=3;i++){
      if (myICM.dataReady()){
        myICM.getAGMT();         // The values are only updated when you call 'getAGMT'
        break;
      }
      //delayMicroseconds(10); 
    }
	}

  accRAW[0] = x = myICM.agmt.acc.axes.x;
  accRAW[1] = y = myICM.agmt.acc.axes.y;
  accRAW[2] = z = myICM.agmt.acc.axes.z;

  // レゾルーションを掛けた値?  agmt.fss.a を元に、G に変換した値
  //switch (agmt.fss.a){
  //case 0: -> (((float)axis_val) / 16.384);
  //case 1: -> (((float)axis_val) / 8.192);
  //case 2: -> (((float)axis_val) / 4.096);
  //case 3: -> (((float)axis_val) / 2.048);
  //accADC_BD[0] = myICM.accX();
  //accADC_BD[1] = myICM.accY();
  //accADC_BD[2] = myICM.accZ();

  ACC_ORIENTATION( x,	y, z );
	acc_common();
}

#else
void cICM20948::acc_get_adc( void ){
	int16_t x = 0;
	int16_t y = 0;
	int16_t z = 0;
  uint8_t rawADC[6];

  if( bConnected == true ){    
    spiRead(ICM20948_REG_ACCEL_XOUT_H_SH, &rawADC[0], 6);

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
#endif

/*
* get Average of Acc value
* from start to 512 
* get Average for each acc x,y,z
* but z is set to zero;
*/
void cICM20948::acc_common(){
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

void cICM20948::mag_init( void )
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
/*---------------------------------------------------------------------------
     TITLE   : mag_get_adc
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
bool cICM20948::mag_get_adc( void )
{
	//int16_t x = 0;
	//int16_t y = 0;
	//int16_t z = 0;

  //uint8_t data[8];

  magADC[0]=magADC[1]=magADC[2]=0.0f;

  if( bConnected == true )
  {
  	//imu_spi_reads(MPU9250_ADDRESS, MPU9250_EXT_SENS_DATA_00, 8, data);

  	//if (!(data[0] & MPU9250_AK8963_DATA_READY) || (data[0] & MPU9250_AK8963_DATA_OVERRUN))
    //{
  	//	return;
  	//}
  	//if (data[7] & MPU9250_AK8963_OVERFLOW)
    //{
  	//	return;
  	//}

  	
    //magRAW[0] = (data[2] << 8) | data[1];
  	//magRAW[1] = (data[4] << 8) | data[3];
  	//magRAW[2] = (data[6] << 8) | data[5];

  	//if (data[7] & MPU9250_AK8963_OVERFLOW)
  	if (myICM.agmt.magStat2 & 0x80)
    {
      //SERIAL_PORT.println("mag_get_adc() : #3 overflow");
  		return false;
    }

    magRAW[0] =magRAW_BD[0] = myICM.agmt.mag.axes.x;
  	magRAW[1] =magRAW_BD[1] = myICM.agmt.mag.axes.y;
  	magRAW[2] =magRAW_BD[2] = myICM.agmt.mag.axes.z;
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

#else
/*---------------------------------------------------------------------------
     TITLE   : mag_get_adc
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
bool cICM20948::mag_get_adc( void )
{
	// int16_t x = 0;
	// int16_t y = 0;
	// int16_t z = 0;

  uint8_t data[8];

  magADC[0]=magADC[1]=magADC[2]=0.0f;

  if( bConnected == true )
  {
    //imu_spi_reads(MPU9250_ADDRESS, MPU9250_EXT_SENS_DATA_00, 8, data);
    spiRead(ICM20948_REG_EXT_SLV_SENS_DATA_00, data,8);

    if (!(data[0] & ICM20948_AK09916_DATA_READY) || (data[0] & ICM20948_AK09916_DATA_OVERRUN))
    {
      return false;
    }
    if (data[7] & ICM20948_AK09916_OVERFLOW)
    {
      return false;
    }

  	magRAW[0] = (data[2] << 8) | data[1];   // x-axis
  	magRAW[1] = (data[4] << 8) | data[3];   // y-axis
  	magRAW[2] = (data[6] << 8) | data[5];   // z-axis

    //  AK8963_ASA[3] -> AK8963 キャリブレーションデータだが、AK09916 は、無し。
  	//magRAW[0] = ((long)magRAW[0] * AK8963_ASA[0]) >> 8;
  	//magRAW[1] = ((long)magRAW[1] * AK8963_ASA[1]) >> 8;
  	//magRAW[2] = ((long)magRAW[2] * AK8963_ASA[2]) >> 8;

  	mag_common();
	}
  else{
    return false;
  }

  return true;
}
#endif

void cICM20948::mag_common()
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

#ifdef USE_DMP_NISHI

void cICM20948::dmp_init( void )
{
  uint8_t i;

  for( i=0; i<4; i++ )
  {
    quat[i]   = 0.0;
		quatZero[i]  = 0;
    quatRAW[i]   = 0;
  }
  quat[0]   = 1.0;

  calibratingD_f = 0;
  calibratingD = 0;    // add by nishi 2022.1.23

}

bool cICM20948::dmp_cali_get_done()
{
	if( calibratingD_f == 1 ) return true;
	else                    return false;
}


bool cICM20948::dmp_get_adc(){
  /*
  * add 9 axis fusion Quarternion with DMP3
  */
  // Read any DMP data waiting in the FIFO
  // Note:
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFONoDataAvail if no data is available.
  //    If data is available, readDMPdataFromFIFO will attempt to read _one_ frame of DMP data.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOIncompleteData if a frame was present but was incomplete
  //    readDMPdataFromFIFO will return ICM_20948_Stat_Ok if a valid frame was read.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOMoreDataAvail if a valid frame was read _and_ the FIFO contains more (unread) data.
  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);

  bool rc=false;
  static int32_t d[4];
  
  static byte f=0;
  static double q[4]={1.0, 0.0, 0.0, 0.0};

  if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
  {
    //SERIAL_PORT.print(F("Received data! Header: 0x")); // Print the header in HEX so we can see what data is arriving in the FIFO
    //if ( data.header < 0x1000) SERIAL_PORT.print( "0" ); // Pad the zeros
    //if ( data.header < 0x100) SERIAL_PORT.print( "0" );
    //if ( data.header < 0x10) SERIAL_PORT.print( "0" );
    //SERIAL_PORT.println( data.header, HEX );

    #ifndef IMU_SENSER6
    if ((data.header & DMP_header_bitmap_Quat9) > 0) // We have asked for orientation data so we should receive Quat9
    {

      // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
      // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
      // ドリフトする時は、合計が、 1 になっていないので 正しい bias values で、 quaternion data を補正しないといけない。
      // The quaternion data is scaled by 2^30.

      //SERIAL_PORT.printf("Quat9 data is: Q1:%ld Q2:%ld Q3:%ld Accuracy:%d\r\n", data.Quat9.Data.Q1, data.Quat9.Data.Q2, data.Quat9.Data.Q3, data.Quat9.Data.Accuracy);

      // Scale to +/- 1
      double q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30  -- X
      double q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30  -- Y
      double q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30  -- Z

      double q0;
      int q0i,q1i,q2i,q3i;

      q1i =(int)round(q1*1000.0);
      q1 = (double)q1i/ 1000.0;

      q2i =(int)round(q2*1000.0);
      q2 = (double)q2i/ 1000.0;

      q3i =(int)round(q3*1000.0);
      q3 = (double)q3i/ 1000.0;

      //double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));    //  -- W 

      //double bias=-0.009;
      //q1+=bias;
      //q2+=bias;
      //q3+=bias;

      if(f==0){
        q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));    //  -- W 
        q0i =(int)round(q0*1000.0);
        q[0] = (double)q0i/ 1000.0;
        q[1]=q1;
        q[2]=q2;
        q[3]=q3;
        f=1;
      }
      else{
        //q[0] = q[0] - (q[1] - q1) - (q[2]-q2) - (q[3] - q3);
        q0 = q[0] - (q[1] - q1) - (q[2]-q2) - (q[3] - q3);
        q0i =(int)round(q0*1000.0);
        q[0] = (double)q0i/ 1000.0;

        //q[0] = q[0] - (q[1] - q1) - (q[2]-q2) + (q[3] - q3);
        //q[0] = q[0] - (q[1] - q1) + (q[2]-q2) - (q[3] - q3);
        //q[0] = q[0] - (q[1] - q1) + (q[2]-q2) + (q[3] - q3);
        //q[0] = q[0] + (q[1] - q1) - (q[2]-q2) - (q[3] - q3);
        //q[0] = q[0] + (q[1] - q1) - (q[2]-q2) + (q[3] - q3);
        //q[0] = q[0] + (q[1] - q1) + (q[2]-q2) - (q[3] - q3);
        //q[0] = q[0] + (q[1] - q1) + (q[2]-q2) + (q[3] - q3);
        //q0 = q0 + q1 -q2 + q3;
        q[1]=q1;
        q[2]=q2;
        q[3]=q3;
      }

      double recipNorm = invSqrt((q[0]*q[0])+ (q1 * q1) + (q2 * q2) + (q3 * q3));
      q[0] *= recipNorm;
      q1 *= recipNorm;
      q2 *= recipNorm;
      q3 *= recipNorm;

      //double gmag = sqrt((q0*q0)+ (q1 * q1) + (q2 * q2) + (q3 * q3));

      //if(fabs(gmag - 1.0) > 0.000001){
      //  q0 /= gmag;
      //  q1 /= gmag;
      //  q2 /= gmag;
      //  q3 /= gmag;
      //}


      //#define XTX1
      #if defined(XTX1)
      //double q0=1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3));
      if(q0 >= 0.0) q0 = sqrt(q0);
      else q0 = sqrt(q0 * -1.0) * -1.0;
      #endif
 
 
      #define TEST_W0_X
      #ifdef TEST_W0_X
      SERIAL_PORT.print(F("Q0:"));
      SERIAL_PORT.print(q[0], 3);
      SERIAL_PORT.print(F(" Q1:"));
      SERIAL_PORT.print(q1, 3);
      SERIAL_PORT.print(F(" Q2:"));
      SERIAL_PORT.print(q2, 3);
      SERIAL_PORT.print(F(" Q3:"));
      SERIAL_PORT.print(q3, 3);
      SERIAL_PORT.print(F(" Accuracy:"));
      SERIAL_PORT.println(data.Quat9.Data.Accuracy);
      #endif

      if(!isnan(q[0])){
                                            //   x x x x x
        quatRAW[1] = data.Quat9.Data.Q1;    // 1 1 2 2 3 3
        quatRAW[2] = data.Quat9.Data.Q2;    // 2 3 1 3 1 2
        quatRAW[3] = data.Quat9.Data.Q3;    // 3 2 3 1 2 1


        #ifndef QUAT_ANIMATION
        //SERIAL_PORT.print(F("Q1:"));
        //SERIAL_PORT.print(quatRAW[1]);
        //SERIAL_PORT.print(F(" Q2:"));
        //SERIAL_PORT.print(quatRAW[2]);
        //SERIAL_PORT.print(F(" Q3:"));
        //SERIAL_PORT.println(quatRAW[3]);
        //SERIAL_PORT.print(F(" Accuracy:"));
        //SERIAL_PORT.println(data.Quat9.Data.Accuracy);
        #endif

        if (calibratingD>0)
        {
          calibratingD--;
          for (uint8_t axis = 1; axis < 4; axis++)
          {
            if (calibratingD ==(MPU_CALI_COUNT-1)){
               d[axis]=0;  // Reset a[axis] at start of calibration
            }
            d[axis] += quatRAW[axis];
            quatZero[axis] = d[axis]>>8;          // Calculate average, only the last itteration where (calibratingA == 0) is relevant
          }
          if (calibratingD == 0)
          {
            //accZero[YAW] -= ACC_1G;
            //magZero[YAW] = 0;

            #ifndef QUAT_ANIMATION
            SERIAL_PORT.print(F("Q1:"));
            SERIAL_PORT.print(quatRAW[1]);
            SERIAL_PORT.print(F(" Q2:"));
            SERIAL_PORT.print(quatRAW[2]);
            SERIAL_PORT.print(F(" Q3:"));
            SERIAL_PORT.println(quatRAW[3]);

            SERIAL_PORT.print(F("QZ1:"));
            SERIAL_PORT.print(quatZero[1]);
            SERIAL_PORT.print(F(" QZ2:"));
            SERIAL_PORT.print(quatZero[2]);
            SERIAL_PORT.print(F(" QZ3:"));
            SERIAL_PORT.println(quatZero[3]);
            //SERIAL_PORT.print(F(" Accuracy:"));
            //SERIAL_PORT.println(data.Quat9.Data.Accuracy);
            #endif
          }


        }

        //quat[1] = ((double)(quatRAW[1] - quatZero[1])) / 1073741824.0; // Convert to double. Divide by 2^30  -- X
        //quat[2] = ((double)(quatRAW[2] - quatZero[2])) / 1073741824.0; // Convert to double. Divide by 2^30  -- Y
        //quat[3] = ((double)(quatRAW[3] - quatZero[3])) / 1073741824.0; // Convert to double. Divide by 2^30  -- Z
        //quat[0] = sqrt(1.0 - ((quat[1] * quat[1]) + (quat[2] * quat[2]) + (quat[3] * quat[3])));    //  -- W

        quat[0] = q[0];
        quat[1] = q1;
        quat[2] = q2;
        quat[3] = q3;

        rc=true;

        #ifndef QUAT_ANIMATION
        //SERIAL_PORT.print(F("Q1:"));
        //SERIAL_PORT.print(q1, 3);
        //SERIAL_PORT.print(F(" Q2:"));
        //SERIAL_PORT.print(q2, 3);
        //SERIAL_PORT.print(F(" Q3:"));
        //SERIAL_PORT.print(q3, 3);
        //SERIAL_PORT.print(F(" Accuracy:"));
        //SERIAL_PORT.println(data.Quat9.Data.Accuracy);
        #else
        // Output the Quaternion data in the format expected by ZaneL's Node.js Quaternion animation tool
        SERIAL_PORT.print(F("{\"quat_w\":"));
        SERIAL_PORT.print(q0, 3);
        SERIAL_PORT.print(F(", \"quat_x\":"));
        SERIAL_PORT.print(q1, 3);
        SERIAL_PORT.print(F(", \"quat_y\":"));
        SERIAL_PORT.print(q2, 3);
        SERIAL_PORT.print(F(", \"quat_z\":"));
        SERIAL_PORT.print(q3, 3);
        SERIAL_PORT.println(F("}"));
        #endif
      }
    }
    #else
    if((data.header & DMP_header_bitmap_Quat6) > 0){
      double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30  -- X
      double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30  -- Y
      double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30  -- Z
      //double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));    //  -- W    こいつが、バグとの事。

      double q0=1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3));
      if(q0 >= 0.0) q0 = sqrt(q0);
      else q0 = sqrt(q0 * -1.0) * -1.0;

      if(!isnan(q0)){
        if(calibratingD_f == 0){
          calibratingD++;
          if(calibratingD >= 200){
            if(calibratingD == 200){
              //d[1]=data.Quat6.Data.Q1;
              //d[2]=data.Quat6.Data.Q2;
              //d[3]=data.Quat6.Data.Q3;
            }
            //d[1] += data.Quat6.Data.Q1;             // Sum up 512 readings
            //d[2] += data.Quat6.Data.Q2;             // Sum up 512 readings
            //d[3] += data.Quat6.Data.Q3;             // Sum up 512 readings

            if(calibratingD >= 1600){
              //quatZero[1] = d[1] / 300;
              //quatZero[2] = d[2] / 300;
              //quatZero[3] = d[3] / 300;   // z だけキャリブレーションする。

              // 此処で、acc の内積を出す。
              //accIMZero = sqrt(accZero[0] * accZero[0] + accZero[1] * accZero[1] + accZero[2] * accZero[2]);
              //accZero[YAW] -= ACC_1G;
              //accZero[YAW] = 0;   // 注) これをすると、海抜からの標高値になる。しなければ、起動地点の標高が原点となる。

              calibratingD_f=1;
              calibratingD = 0;
            }
          }
        }
        //quatRAW[1] = data.Quat6.Data.Q1 - quatZero[1];
        //quatRAW[2] = data.Quat6.Data.Q2 - quatZero[2];
        //quatRAW[3] = data.Quat6.Data.Q3 - quatZero[3];

        //q1 = ((double)quatRAW[1]) / 1073741824.0; // Convert to double. Divide by 2^30  -- X
        //q2 = ((double)quatRAW[2]) / 1073741824.0; // Convert to double. Divide by 2^30  -- Y
        //q3 = ((double)quatRAW[3]) / 1073741824.0; // Convert to double. Divide by 2^30  -- Z

        //q0=1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3));
        //if(q0 >= 0.0) q0 = sqrt(q0);
        //else q0 = sqrt(q0 * -1.0) * -1.0;

        quat[0]=q0; // W
        quat[1]=q1; // X
        quat[2]=q2; // Y
        quat[3]=q3; // Z

        rc=true;

        //#define TEST_W0_X2
        #ifdef TEST_W0_X2
        SERIAL_PORT.print(F("Q0:"));
        SERIAL_PORT.print(q0, 3);
        SERIAL_PORT.print(F(" Q1:"));
        SERIAL_PORT.print(q1, 3);
        SERIAL_PORT.print(F(" Q2:"));
        SERIAL_PORT.print(q2, 3);
        SERIAL_PORT.print(F(" Q3:"));
        SERIAL_PORT.println(q3, 3);
        #endif
      }
    }
    #endif
  }
  return rc;
}

#endif  

void cICM20948::acc_cali_start()
{
	//calibratingA = MPU_CALI_COUNT;
  calibratingA_f = 0;
  calibratingA = 0; 
}

bool cICM20948::acc_cali_get_done()
{
	if( calibratingA_f !=0 ) return true;
	else                    return false;
}

bool cICM20948::gyro_cali_get_done()
{
	if( calibratingG_f != 0 ) return true;
	else                    return false;
}

#ifndef USE_SPARK_LIB
// add by nishi

#define TEST_XXXX
#ifdef TEST_XXXX

#define delay_ms delay

int cICM20948::imu_spi_ak09916_reads(uint8_t akm_addr, uint8_t reg_addr, uint8_t len, uint8_t *data)
{
	uint8_t index = 0;
	uint8_t status = 0;
	uint32_t timeout = 0;
	uint8_t tmp = 0;

	tmp = akm_addr | ICM20948_I2C_READ;
	//imu_spi_writes(ICM20948_SPIx_ADDR, ICM20948_REG_I2C_SLV4_ADDR, 1, &tmp);
  spiWrite(ICM20948_REG_I2C_SLV4_ADDR, &tmp,1);

  //tmp=spiReadByte(ICM20948_REG_I2C_SLV4_ADDR);
  //SERIAL_PORT.print("cICM20948::imu_spi_ak09916_reads() #1  I2C_SLV4_ADDR=");
  //SERIAL_PORT.println(tmp,HEX);


	delay(1);
	while(index < len){
		tmp = reg_addr + index;
		//imu_spi_writes(ICM20948_SPIx_ADDR, ICM20948_REG_I2C_SLV4_REG, 1, &tmp);
		spiWrite(ICM20948_REG_I2C_SLV4_REG, &tmp,1);
		delay(1);

    //tmp=spiReadByte(ICM20948_REG_I2C_SLV4_REG);
    //SERIAL_PORT.print("cICM20948::imu_spi_ak09916_reads() #2  I2C_SLV4_REG=");
    //SERIAL_PORT.println(tmp,HEX);


		tmp = ICM20948_I2C_SLV4_EN+0x10;
		//imu_spi_writes(ICM20948_SPIx_ADDR, ICM20948_REG_I2C_SLV4_CTRL, 1, &tmp);
		spiWrite(ICM20948_REG_I2C_SLV4_CTRL, &tmp, 1);
		delay(1);


    //tmp=spiReadByte(ICM20948_REG_I2C_SLV4_CTRL);
    //SERIAL_PORT.print("cICM20948::imu_spi_ak09916_reads() #3  I2C_SLV4_CTRL=");
    //SERIAL_PORT.println(tmp,HEX);

		do {
			if (timeout++ > 50){
				return -2;
			}
			//imu_spi_reads(ICM20948_SPIx_ADDR, ICM20948_REG_I2C_MST_STATUS, 1, &status);
			spiRead(ICM20948_REG_I2C_MST_STATUS, &status,1);

      //SERIAL_PORT.print("cICM20948::imu_spi_ak09916_reads() #4  ICM20948_REG_I2C_MST_STATUS=");
      //SERIAL_PORT.println(status,HEX);

			delay_ms(2);
		} while ((status & ICM20948_I2C_SLV4_DONE) == 0);
		//imu_spi_reads(ICM20948_SPIx_ADDR, ICM20948_REG_I2C_SLV4_DI, 1, data + index);
		spiRead(ICM20948_REG_I2C_SLV4_DI, data + index,1);
		delay_ms(1);
		index++;
	}
	return 0;
}

int cICM20948::imu_spi_ak09916_writes(uint8_t akm_addr, uint8_t reg_addr, uint8_t len, uint8_t *data)
{
  uint32_t timeout = 0;
	uint8_t status = 0;
	uint8_t tmp = 0;
	uint8_t index = 0;

	//imu_spi_writes(ICM20948_SPIx_ADDR, ICM20948_REG_I2C_SLV4_ADDR, 1, &tmp);
	spiWriteByte(ICM20948_REG_I2C_SLV4_ADDR, akm_addr);
	delay_ms(2);

  //SERIAL_PORT.print("cICM20948::imu_spi_ak09916_writes() #1  akm_addr=");
  //SERIAL_PORT.println(akm_addr,HEX);


	while(index < len){
		tmp = reg_addr + index;
		//imu_spi_writes(ICM20948_SPIx_ADDR, ICM20948_REG_I2C_SLV4_REG, 1, &tmp);
		spiWriteByte(ICM20948_REG_I2C_SLV4_REG, tmp);
		delay_ms(2);

    //SERIAL_PORT.print("cICM20948::imu_spi_ak09916_writes() #2  reg_addr + index=");
    //SERIAL_PORT.println(reg_addr + index,HEX);


		//imu_spi_writes(ICM20948_SPIx_ADDR, ICM20948_REG_I2C_SLV4_DO, 1, data + index);
		spiWriteByte(ICM20948_REG_I2C_SLV4_DO, data[index]);
		delay_ms(2);

    //SERIAL_PORT.print("cICM20948::imu_spi_ak09916_writes() #3  data[index]=");
    //SERIAL_PORT.println(data[index],HEX);


		//tmp = ICM20948_I2C_SLV4_EN;
		//imu_spi_writes(ICM20948_SPIx_ADDR, ICM20948_REG_I2C_SLV4_CTRL, 1, &tmp);
		spiWriteByte(ICM20948_REG_I2C_SLV4_CTRL, ICM20948_I2C_SLV4_EN+7);
		delay_ms(2);

		do {
			if (timeout++ > 50)
				return -2;
			//imu_spi_reads(ICM20948_SPIx_ADDR, ICM20948_REG_I2C_MST_STATUS, 1, &status);
			spiRead(ICM20948_REG_I2C_MST_STATUS, &status,1);

      //SERIAL_PORT.print("cICM20948::imu_spi_ak09916_writes() #4  I2C_MST_STATUS=");
      //SERIAL_PORT.println(status,HEX);

			delay_ms(2);
		} while ((status & ICM20948_I2C_SLV4_DONE) == 0);
		if (status & ICM20948_I2C_SLV4_NACK)
			return -3;
		index++;
	}
	return 0;
}
int cICM20948::imu_spi_ak09916_write(uint8_t akm_addr, uint8_t reg_addr, uint8_t data)
{
  uint8_t param[1];

  param[0] = data;

  return imu_spi_ak09916_writes(akm_addr,reg_addr, 1, param);
}

#endif

/*
* Rw : true / false -> Read / Write
*/
ICM_20948_Status_e cICM20948::ICM_20948_i2c_controller_periph4_txn(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len, bool Rw, bool send_reg_addr)
{
  // Thanks MikeFair! // https://github.com/kriswiner/MPU9250/issues/86
  ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

  uint8_t x,y;


  addr = (((Rw) ? 0x80 : 0x00) | addr);     // I2C の Slave アドレス

  //retval = ICM_20948_set_bank(pdev, 3);
  //retval = ICM_20948_execute_w(pdev, AGB3_REG_I2C_PERIPH4_ADDR, (uint8_t *)&addr, 1);
	spiWriteByte(ICM20948_REG_I2C_SLV4_ADDR, addr);

  // Debug by nishi
  spiRead(ICM20948_REG_I2C_SLV4_ADDR,&x, 1);
  SERIAL_PORT.print("cICM20948::ICM_20948_i2c_controller_periph4_txn() #1  addr=");
  SERIAL_PORT.print(addr,HEX);
  SERIAL_PORT.print(" ,x=");
  SERIAL_PORT.println(x,HEX);
  //while(1){
  //  delay(100);
  //}



  //if (retval != ICM_20948_Stat_Ok)
  //{
  //  return retval;
  //}

  //retval = ICM_20948_set_bank(pdev, 3);
  //retval = ICM_20948_execute_w(pdev, AGB3_REG_I2C_PERIPH4_REG, (uint8_t *)&reg, 1);
	spiWriteByte(ICM20948_REG_I2C_SLV4_REG, reg);


  // Debug by nishi
  spiRead(ICM20948_REG_I2C_SLV4_REG,&x, 1);
  SERIAL_PORT.print("cICM20948::ICM_20948_i2c_controller_periph4_txn() #2  reg=");
  SERIAL_PORT.print(reg,HEX);
  SERIAL_PORT.print(" ,x=");
  SERIAL_PORT.println(x,HEX);
  //while(1){
  //  delay(100);
  //}

  //if (retval != ICM_20948_Stat_Ok)
  //{
  //  return retval;
  //}

  ICM_20948_I2C_PERIPH4_CTRL_t ctrl;
  ctrl.EN = 1;
  ctrl.INT_EN = false;
  ctrl.REG_DIS = !send_reg_addr;
  ctrl.DLY = 0;

  SERIAL_PORT.print("sizeof(ICM_20948_I2C_PERIPH4_CTRL_t)=");
  SERIAL_PORT.print(sizeof(ICM_20948_I2C_PERIPH4_CTRL_t));
  SERIAL_PORT.print(" ,ctrl=");    // 0x80
  uint8_t *xp=(uint8_t *)&ctrl;
  x= *xp;
  SERIAL_PORT.println(x,HEX);

  ICM_20948_I2C_MST_STATUS_t i2c_mst_status;
  bool txn_failed = false;
  uint16_t nByte = 0;

  //x=0x90;

  while (nByte < len)
  {
    if (!Rw)
    {
      //retval = ICM_20948_set_bank(pdev, 3);
      //retval = ICM_20948_execute_w(pdev, AGB3_REG_I2C_PERIPH4_DO, (uint8_t *)&(data[nByte]), 1);
      spiWriteByte(ICM20948_REG_I2C_SLV4_DO,data[nByte]);

      //if (retval != ICM_20948_Stat_Ok)
      //{
      //  return retval;
      //}
    }

    // Kick off txn
    //retval = ICM_20948_set_bank(pdev, 3);
    //retval = ICM_20948_execute_w(pdev, AGB3_REG_I2C_PERIPH4_CTRL, (uint8_t *)&ctrl, sizeof(ICM_20948_I2C_PERIPH4_CTRL_t));
    spiWrite(ICM20948_REG_I2C_SLV4_CTRL,(uint8_t *)&ctrl, sizeof(ICM_20948_I2C_PERIPH4_CTRL_t));

    //if (retval != ICM_20948_Stat_Ok)
    //{
    //  return retval;
    //}

    // long tsTimeout = millis() + 3000;  // Emergency timeout for txn (hard coded to 3 secs)
    uint32_t max_cycles = 1000;
    uint32_t count = 0;
    bool peripheral4Done = false;
    while (!peripheral4Done)
    {
      //retval = ICM_20948_set_bank(pdev, 0);
      //retval = ICM_20948_execute_r(pdev, AGB0_REG_I2C_MST_STATUS, (uint8_t *)&i2c_mst_status, 1);
      spiRead(ICM20948_REG_I2C_MST_STATUS,(uint8_t *)&i2c_mst_status, 1);

      peripheral4Done = (i2c_mst_status.I2C_PERIPH4_DONE /*| (millis() > tsTimeout) */); //Avoid forever-loops
      peripheral4Done |= (count >= max_cycles);
      count++;
    }
    txn_failed = (i2c_mst_status.I2C_PERIPH4_NACK /*| (millis() > tsTimeout) */);
    txn_failed |= (count >= max_cycles);
    if (txn_failed)
      break;

    if (Rw)
    {
      //retval = ICM_20948_set_bank(pdev, 3);
      //retval = ICM_20948_execute_r(pdev, AGB3_REG_I2C_PERIPH4_DI, &data[nByte], 1);
      spiRead(ICM20948_REG_I2C_SLV4_DI,&data[nByte], 1);
    }

    nByte++;
  }

  if (txn_failed)
  {
    //We often fail here if mag is stuck
    return ICM_20948_Stat_Err;
  }

  return retval;
}


#endif