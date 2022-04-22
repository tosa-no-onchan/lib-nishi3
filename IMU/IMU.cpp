//----------------------------------------------------------------------------
//    프로그램명 	:
//
//    만든이     	: Made by Baram ( chcbaram@paran.com )
//
//    날  짜     :
//
//    최종 수정  	:
//
//    MPU_Type	:
//
//    파일명     	: IMU.ino
//----------------------------------------------------------------------------
#include <Arduino.h>
#include "IMU.h"

#define LED_BUILTIN 17
//#define LED_BUILTIN 4

/*---------------------------------------------------------------------------
     TITLE   : BLE
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
cIMU::cIMU(){
  uint8_t i;
  for( i=0; i<3; i++ ){
    rpy[i] = 0.;
  }
	bConnected = false;
}

/*---------------------------------------------------------------------------
     TITLE   : begin
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
uint8_t cIMU::begin( uint32_t hz ){
	uint8_t err_code = IMU_OK;
  uint32_t i;
  uint32_t pre_time;

  update_hz = hz;
  //update_us = 1000000/hz;
  update_us = 1000000UL/hz;

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
  #define ACC_MAX_G 8192.0

  //aRes = 9.80665/16384.0;     // 2g
  //aRes = 9.80665/8192.0;     // 4g
  //aRes = 9.80665/4096.0;     // 8g
  aRes = 9.80665/ACC_MAX_G; 

  gRes = 2000.0/32768.0;   // 2000dps

  #ifdef ICM20948_IMU
  //mRes = 0.15; // Sensitivity Scale Factor = 0.15
  #else
  //mRes = 10.*4912./8190.;  // 14BIT
  mRes = 10.*4912./32760.; // 16BIT
  #endif

  v_acc[0]=0.0;
  v_acc[1]=0.0;
  v_acc[2]=0.0;

  v_acc_pre[0]=0.0;
  v_acc_pre[1]=0.0;
  v_acc_pre[2]=0.0;

  tf_dlt[0]=0.0;
  tf_dlt[1]=0.0;
  tf_dlt[2]=0.0;

  quat[0]=1.0;
  quat[1]=0.0;
  quat[2]=0.0;
  quat[3]=0.0;

  cali_tf=5;

  for(i=0;i<3;i++){
    bConnected = SEN.begin();
    if(bConnected == true)
      break;
  }
  
 	digitalWrite(LED_BUILTIN, LOW);		// light OFF
  if( bConnected == true ){

    filter.begin(update_hz);
    //filter.begin();

    pre_time = millis();

   	#ifdef USE_DMP_NISHI
    while(!SEN.dmp_cali_get_done()){
      update();
      if (millis()-pre_time > 5000){
        break;
      }
    }
    #else
    while(!SEN.gyro_cali_get_done()){
      update();
      if (millis()-pre_time > 5000){
        break;
      }
    }
    #endif
  }
  else{
  	digitalWrite(LED_BUILTIN, HIGH);		// light ON
  }
	return err_code;
}

/*---------------------------------------------------------------------------
     TITLE   : update
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
uint16_t cIMU::update( uint32_t option ){
  // changed by nishi
  //UNUSED(option);

	uint16_t ret_time = 0;

	//static uint32_t tTime;
	static unsigned long tTime;

	if( (micros()-tTime) >= update_us ){
		ret_time = micros()-tTime;
    tTime = micros();
		computeIMU();
	}
	return ret_time;
}

#define FILTER_NUM    3

/*---------------------------------------------------------------------------
     TITLE   : computeIMU
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cIMU::computeIMU( void ){
  static unsigned long prev_process_time = micros();
  static unsigned long cur_process_time = 0;
  static unsigned long process_time = 0;
  uint32_t i;
  static int32_t gyroADC[3][FILTER_NUM] = {0,};
  int32_t gyroAdcSum;

  static int32_t accADC[4][FILTER_NUM] = {0,};
  int32_t accAdcSum;

  uint32_t axis;

  #if defined(USE_ACC_NISHI)
    // Get Acc data
	  SEN.acc_get_adc();
  #endif
  #if defined(USE_GRYO_NISHI)
    // Get Gyro data
	  SEN.gyro_get_adc();
  #endif

  #ifdef USE_DMP_NISHI
    // Get DMP data
    if(SEN.dmp_get_adc()!=true){
      //digitalWrite(LED_BUILTIN, HIGH-digitalRead(LED_BUILTIN));   // blink the blue
      return;
    }
    // DMP Caliburation not yet?
    if(SEN.calibratingD_f != 1){
      return;
    }
    #if defined(USE_ACC_NISHI)
      // Acc Caliburation not yet?
      if(SEN.calibratingA_f != 1){
        return;
      }
    #endif
  #else
    //if (SEN.mag_get_adc()==true){
    //  digitalWrite(LED_BUILTIN, HIGH-digitalRead(LED_BUILTIN));   // blink the blue
    //}
    if(SEN.calibratingA_f!=1){
      return;
    }
  #endif

  #if defined(USE_GRYO_NISHI)
    for (axis = 0; axis < 3; axis++){
      gyroADC[axis][0] = SEN.gyroADC[axis];
      gyroAdcSum = 0;
      for (i=0; i<FILTER_NUM; i++){
        gyroAdcSum += gyroADC[axis][i];
      }
      SEN.gyroADC[axis] = gyroAdcSum/FILTER_NUM;
      for (i=FILTER_NUM-1; i>0; i--){
        gyroADC[axis][i] = gyroADC[axis][i-1];
      }
      if (abs(SEN.gyroADC[axis]) <= 3){
        SEN.gyroADC[axis] = 0;
      }
    }
  #endif

  for( i=0; i<3; i++ ){
    #if defined(USE_ACC_NISHI)
    accRaw[i]   = SEN.accRAW[i];
    accData[i]  = SEN.accADC[i];
    #endif
    #if defined(USE_GRYO_NISHI)
      gyroRaw[i]  = SEN.gyroRAW[i];
      gyroData[i] = SEN.gyroADC[i];
    #endif

    #if defined(USE_MAG)
    magRaw[i]   = SEN.magRAW[i];
    magData[i]  = SEN.magADC[i];
    #endif
  }

  #if defined(USE_GRYO_NISHI)
    gx = (float)SEN.gyroADC[0]*gRes;
    gy = (float)SEN.gyroADC[1]*gRes;
    gz = (float)SEN.gyroADC[2]*gRes;
  #endif

  #if defined(USE_MAG)
    #ifdef ICM20948_IMU
      // Apply mag offset compensation (base values in uTesla)
      //float x = SEN.magADC[0] - mag_offsets[0];
      //float y = SEN.magADC[1] - mag_offsets[1];
      //float z = SEN.magADC[2] - mag_offsets[2];

      // Apply mag soft iron error compensation
      //mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
      //my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
      //mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

      mx = SEN.magADC[0];
      my = SEN.magADC[1];
      mz = SEN.magADC[2];

    #else
      mx = (float)SEN.magADC[0]*mRes;
      my = (float)SEN.magADC[1]*mRes;
      mz = (float)SEN.magADC[2]*mRes;
    #endif
  #endif

  cur_process_time  = micros();
  process_time      = cur_process_time-prev_process_time;
  prev_process_time = cur_process_time;

  #ifndef USE_DMP_NISHI
    #ifdef IMU_SENSER6
      if (SEN.calibratingG == 0 && SEN.calibratingA == 0)
      {
        filter.invSampleFreq = (float)process_time/1000000.0f;
        filter.updateIMU(gx, gy, gz, ax, ay, az);
      }
    #else
      if (SEN.calibratingG == 0 && SEN.calibratingA == 0 && SEN.calibratingM==0)
      {
        filter.invSampleFreq = (float)process_time/1000000.0f;
        filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
        //filter.update(gx, gy, gz*-1.0, ax, ay, az, mx, my, mz);
        //filter.update(gx, gy, gz, ax, ay, az, my, mx, mz*-1.0);
        //filter.update(gx, gy, gz, ax, ay, az, mx*-1.0, my, mz);
      }
    #endif

    rpy[0] = filter.getRoll();
    rpy[1] = filter.getPitch();
    rpy[2] = filter.getYaw()-180.;

    quat[0] = filter.q0;  // W
    quat[1] = filter.q1;  // X
    quat[2] = filter.q2;  // Y
    quat[3] = filter.q3;  // Z

    //#define TEST_NISHI_5_D
    #ifdef TEST_NISHI_5_D
      SERIAL_PORT.print(F("Q0:"));
      SERIAL_PORT.print(filter.q0, 4);
      SERIAL_PORT.print(F(" Q1:"));
      SERIAL_PORT.print(filter.q1, 4);
      SERIAL_PORT.print(F(" Q2:"));
      SERIAL_PORT.print(filter.q2, 4);
      SERIAL_PORT.print(F(" Q3:"));
      SERIAL_PORT.println(filter.q3, 4);
    #endif

    //filter.getQuaternion(&quat[0], &quat[1], &quat[2], &quat[3]);

    angle[0] = (int16_t)(rpy[0] * 10.);
    angle[1] = (int16_t)(rpy[1] * 10.);
    angle[2] = (int16_t)(rpy[1] * 1.);

  #else
    // use ICM20948 DMP Fusion.  add by nishi 2021.11.6
    quat[0] = SEN.quat[0];  // W
    quat[1] = SEN.quat[1];  // X
    quat[2] = SEN.quat[2];  // Y
    quat[3] = SEN.quat[3];  // Z
  #endif

  // Cumpute CB 
  compCB(quat,&cb);

  int16_t acc_Zero[3];

  // 1G の分配値を計算
  //acc_Zero[0] = (int16_t)(cb.dt[2][0]*(double)SEN.accZeroSum);
  //acc_Zero[1] = (int16_t)(cb.dt[2][1]*(double)SEN.accZeroSum);
  //acc_Zero[2] = (int16_t)(cb.dt[2][2]*(double)SEN.accZeroSum);

  //acc_Zero[0] = (int16_t)(cb.dt[2][0]*ACC_MAX_G);
  //acc_Zero[1] = (int16_t)(cb.dt[2][1]*ACC_MAX_G);
  //acc_Zero[2] = (int16_t)(cb.dt[2][2]*ACC_MAX_G);

  acc_Zero[0] = (int16_t)(cb.dt[2][0]*8192.0);  // accData[0]:2 accData[1]:-36 accData[2]:72
  acc_Zero[1] = (int16_t)(cb.dt[2][1]*8192.0);  // accData[0]:-1 accData[1]:-16 accData[2]:-12
  acc_Zero[2] = (int16_t)(cb.dt[2][2]*8192.0);

  //acc_Zero[0] = (int16_t)(cb.dt[2][0]*8698.0);    //
  //acc_Zero[1] = (int16_t)(cb.dt[2][1]*8698.0);
  //acc_Zero[2] = (int16_t)(cb.dt[2][2]*8698.0);    // accData[0]:-4 accData[1]:-21 accData[2]:-506

  //SEN.accZeroSum:8669
  //accData[0]:-30 accData[1]:-36 accData[2]:-469

  // acc 計測値から、1G をキャンセルします。
  accData[0] -=acc_Zero[0];
  accData[1] -=acc_Zero[1];
  accData[2] -=acc_Zero[2];

  // 2G -> SEN.accZeroSum:8698
  // 4G ->  SEN.accZeroSum:8723
  //SERIAL_PORT.print(F("SEN.accZeroSum:"));
  //SERIAL_PORT.println(SEN.accZeroSum);

  //SERIAL_PORT.print(F("accData[0]:"));
  //SERIAL_PORT.print(accData[0]);
  //SERIAL_PORT.print(F(" accData[1]:"));
  //SERIAL_PORT.print(accData[1]);
  //SERIAL_PORT.print(F(" accData[2]:"));
  //SERIAL_PORT.println(accData[2]);
  // 8192.0 を使った場合。
  // accData[0]:-6 accData[1]:0 accData[2]:18
  // accData[0]:7 accData[1]:-12 accData[2]:15
  // accData[0]:14 accData[1]:-3 accData[2]:27
  // accData[0]:2 accData[1]:-6 accData[2]:25

  // accData[0]:-5 accData[1]:-47 accData[2]:-6
  // accData[0]:-4 accData[1]:-48 accData[2]:-6

  // 4G full scale * 0.5[%] の 誤差 --> +-2G
  // 8192 * 4 * 0.005 = 163.84 / 2 = 81.92

  SERIAL_PORT.print(F("accData[0]:"));
  SERIAL_PORT.print(accData[0]);
  SERIAL_PORT.print(F(" accData[1]:"));
  SERIAL_PORT.print(accData[1]);
  SERIAL_PORT.print(F(" accData[2]:"));
  SERIAL_PORT.println(accData[2]);

  #define ACC_X_CUT_OFF 28
  #define ACC_Y_CUT_OFF 28
  #define ACC_Z_CUT_OFF 28

  //if(accData[0] <= ACC_X_CUT_OFF && accData[0] >= ACC_X_CUT_OFF * -1){
    //accData[0]=0;
  //}
  //if(accData[1] <= ACC_Y_CUT_OFF && accData[1] >= ACC_Y_CUT_OFF * -1){
    //accData[1]=0;
  //}
  //if(accData[2] <= ACC_Z_CUT_OFF && accData[2] >= ACC_Z_CUT_OFF * -1){
  //if(accData[2] <= 70 && accData[2] >= -70){
    //accData[2]=0;
  //}

  //SERIAL_PORT.print(F("accData[0]:"));
  //SERIAL_PORT.print(accData[0]);
  //SERIAL_PORT.print(F(" accData[1]:"));
  //SERIAL_PORT.print(accData[1]);
  //SERIAL_PORT.print(F(" accData[2]:"));
  //SERIAL_PORT.println(accData[2]);

  // Acc
  // initial Tolerance
  //  Component-level ±0.5[%]
  // ZERO-G OUTPUT
  //  Initial Tolerance Board-level, all axes ±50 [mg]
  // acc の ±0.5[%] は、ゼロとする。
  ax = (float)accData[0]*aRes;
  ay = (float)accData[1]*aRes;
  az = (float)accData[2]*aRes;

  #ifdef USE_IMU_DIST
    computeTF(process_time);
  #endif
}

/*---------------------------------------------------------------------------
     TITLE   : computeTF
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cIMU::computeTF(unsigned long process_time){
  double roll, pitch, yaw;

  double s = (double)process_time/1000000.0;

  //QuaternionToEulerAngles(quat[0], quat[1], quat[2], quat[3],roll, pitch, yaw);

  double v_acc_dlt[3];
  double dlt[3];    // 速度

  // 今回の加速度を計算
  //dlt[0]=(cb.dt[0][0]+cb.dt[0][1]+cb.dt[0][2])*(double)ax;
  //dlt[1]=(cb.dt[1][0]+cb.dt[1][1]+cb.dt[1][2])*(double)ay;
  //dlt[2]=(cb.dt[2][0]+cb.dt[2][1]+cb.dt[2][2])*(double)az;

  // 今回の加速度を計算 and 基準座標系に変換
  dlt[0]=cb.dt[0][0]*(double)ax+cb.dt[0][1]*(double)ay+cb.dt[0][2]*(double)az;
  dlt[1]=cb.dt[1][0]*(double)ax+cb.dt[1][1]*(double)ay+cb.dt[1][2]*(double)az;
  dlt[2]=cb.dt[2][0]*(double)ax+cb.dt[2][1]*(double)ay+cb.dt[2][2]*(double)az;

  // 速度を累計
  v_acc[0] += dlt[0]*s;
  v_acc[1] += dlt[1]*s;
  v_acc[2] += dlt[2]*s;

  //v_acc_dlt[0] =v_acc[0];
  //v_acc_dlt[1] =v_acc[1];
  //v_acc_dlt[2] =v_acc[2];

  // 速度を Cut Off
  //SERIAL_PORT.print(F("v_acc[0]:"));
  //SERIAL_PORT.print(v_acc[0],12);
  //SERIAL_PORT.print(F(" v_acc[1]:"));
  //SERIAL_PORT.print(v_acc[1],12);
  //SERIAL_PORT.print(F(" v_acc[2]:"));
  //SERIAL_PORT.println(v_acc[2],12);

  // v_acc[0]:0.001002981793 v_acc[1]:0.000000000000 v_acc[2]:0.000000000000
  // v_acc[0]:0.001002981793 v_acc[1]:0.000000000000 v_acc[2]:0.000000000000
  // v_acc[0]:0.000866365328 v_acc[1]:-0.000735460955 v_acc[2]:0.000000000000

  //#define VX_CUT_OFF 0.001
  //#define VY_CUT_OFF 0.001
  //#define VZ_CUT_OFF 0.001
  //if(v_acc[0] <= VX_CUT_OFF && v_acc[0] >= VX_CUT_OFF*-1.0){
    //v_acc[0]=0.0;
  //}
  //if(v_acc[1] <= VY_CUT_OFF && v_acc[1] >= VY_CUT_OFF*-1.0){
    //v_acc[1]=0.0;
  //}
  //if(v_acc[2] <= VZ_CUT_OFF && v_acc[2] >= VZ_CUT_OFF*-1.0){
    //v_acc[2]=0.0;
  //}

  // 定速速度で、加速度==0
  if((accData[0] <= ACC_X_CUT_OFF && accData[0] >= ACC_X_CUT_OFF * -1) &&
    (accData[1] <= ACC_Y_CUT_OFF && accData[1] >= ACC_Y_CUT_OFF * -1) &&
    (accData[2] <= ACC_Z_CUT_OFF && accData[2] >= ACC_Z_CUT_OFF * -1)){
      v_acc[0]=0.0;
      v_acc[1]=0.0;
      v_acc[2]=0.0;
  }
  //if(accData[0] == 0 && accData[1] == 0 && accData[2] == 0){
    //if(v_acc[0] != 0.0 && v_acc[0] == v_acc_pre[0]){
    //  v_acc[0]=0.0;
    //}
    //if(v_acc[1] != 0.0 && v_acc[1] == v_acc_pre[1]){
    //  v_acc[1]=0.0;
    //}
    //if(v_acc[2] != 0.0 && v_acc[2] == v_acc_pre[2]){
    //  v_acc[2]=0.0;
    //}
  //}
  
  //tf_dlt[0] += xyz[0] * v_acc[0] * s;   // delta x
  tf_dlt[0] += v_acc[0]*s;   // delta x
  //tf_dlt[1] += xyz[1] * v_acc[1] * s;   // delta y
  tf_dlt[1] += v_acc[1]*s;   // delta y
  //tf_dlt[2] += xyz[2] * v_acc[2] * s;  // delta z
  tf_dlt[2] += v_acc[2]*s;   // delta y

  v_acc_pre[0]=v_acc[0];
  v_acc_pre[1]=v_acc[1];
  v_acc_pre[2]=v_acc[2];

  //#define KKKK2
  #ifdef KKKK2
    SERIAL_PORT.print(F("tf_dlt[0]:"));
    SERIAL_PORT.print(tf_dlt[0],8);
    SERIAL_PORT.print(F(" tf_dlt[1]:"));
    SERIAL_PORT.println(tf_dlt[1],8);
  #endif

}

void cIMU::QuaternionToEulerAngles(double q0, double q1, double q2, double q3,
                             double& roll, double& pitch, double& yaw)
{
    double q0q0 = q0 * q0;
    double q0q1 = q0 * q1;
    double q0q2 = q0 * q2;
    double q0q3 = q0 * q3;
    double q1q1 = q1 * q1;
    double q1q2 = q1 * q2;
    double q1q3 = q1 * q3;
    double q2q2 = q2 * q2;
    double q2q3 = q2 * q3;
    double q3q3 = q3 * q3;
    roll = atan2(2.0 * (q2q3 + q0q1), q0q0 - q1q1 - q2q2 + q3q3);
    pitch = asin(2.0 * (q0q2 - q1q3));
    yaw = atan2(2.0 * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3);
}

/*
* 基準座標での単位移動距離を計算
*   P = [Px Py Pz] = CBvBdt
* Input
*  float q[4] : [q0 q1 q2 q3]
*  float vB[3] : [vBx vBy vBz] 今の速度
*  double dt: delta time
* Output
*  double Pn[3] : [Pxn Pyn Pzn] 基準座標での単位移動距離
*/
void cIMU::compCBvBdt(float q[4],float vB[3],double dt,double *Pn){

    double q0q0 = q[0] * q[0];
    double q0q1 = q[0] * q[1];
    double q0q2 = q[0] * q[2];
    double q0q3 = q[0] * q[3];
    double q1q1 = q[1] * q[1];
    double q1q2 = q[1] * q[2];
    double q1q3 = q[1] * q[3];
    double q2q2 = q[2] * q[2];
    double q2q3 = q[2] * q[3];
    double q3q3 = q[3] * q[3];

    double CB[3][3];
    CB[0][0] = q0q0+q1q1-q2q2-q3q3;
    CB[0][1] = 2.0*(q1q2-q0q3);
    CB[0][2] = 2.0*(q1q3+q0q2);
    CB[1][0] = 2.0*(q1q2+q0q3);
    CB[1][1] = q0q0-q1q1+q2q2-q3q3;
    CB[1][2] = 2.0*(q2q3-q0q1);
    CB[2][0] = 2.0*(q1q3-q0q2);
    CB[2][1] = 2.0*(q2q3+q0q1);
    CB[2][2] = q0q0-q1q1-q2q2+q3q3;

    Pn[0]=(CB[0][0]*vB[0]+CB[0][1]*vB[1]+CB[0][2]*vB[2])*dt;
    Pn[1]=(CB[1][0]*vB[0]+CB[1][1]*vB[1]+CB[1][2]*vB[2])*dt;
    Pn[2]=(CB[2][0]*vB[0]+CB[2][1]*vB[1]+CB[2][2]*vB[2])*dt;
}

/*
* クォータニオンを回転行列に変換して、X-Y-Z ベクトルを得る。
*/
void cIMU::CB2XYZ(float q[4],double *xyz){

    double q0q0 = q[0] * q[0];
    double q0q1 = q[0] * q[1];
    double q0q2 = q[0] * q[2];
    double q0q3 = q[0] * q[3];
    double q1q1 = q[1] * q[1];
    double q1q2 = q[1] * q[2];
    double q1q3 = q[1] * q[3];
    double q2q2 = q[2] * q[2];
    double q2q3 = q[2] * q[3];
    double q3q3 = q[3] * q[3];

    double CB[3][3];
    CB[0][0] = q0q0+q1q1-q2q2-q3q3;
    CB[0][1] = 2.0*(q1q2-q0q3);
    CB[0][2] = 2.0*(q1q3+q0q2);
    CB[1][0] = 2.0*(q1q2+q0q3);
    CB[1][1] = q0q0-q1q1+q2q2-q3q3;
    CB[1][2] = 2.0*(q2q3-q0q1);
    CB[2][0] = 2.0*(q1q3-q0q2);
    CB[2][1] = 2.0*(q2q3+q0q1);
    CB[2][2] = q0q0-q1q1-q2q2+q3q3;

    xyz[0]=(CB[0][0]+CB[0][1]+CB[0][2]);
    xyz[1]=(CB[1][0]+CB[1][1]+CB[1][2]);
    xyz[2]=(CB[2][0]+CB[2][1]+CB[2][2]);
}

/*
* compCB()
* クォータニオンを回転行列に変換する。
*/
void cIMU::compCB(float q[4],CB *cb){

    double q0q0 = q[0] * q[0];
    double q0q1 = q[0] * q[1];
    double q0q2 = q[0] * q[2];
    double q0q3 = q[0] * q[3];
    double q1q1 = q[1] * q[1];
    double q1q2 = q[1] * q[2];
    double q1q3 = q[1] * q[3];
    double q2q2 = q[2] * q[2];
    double q2q3 = q[2] * q[3];
    double q3q3 = q[3] * q[3];

    //double CB[3][3];
    cb->dt[0][0] = q0q0+q1q1-q2q2-q3q3;
    cb->dt[0][1] = 2.0*(q1q2-q0q3);
    cb->dt[0][2] = 2.0*(q1q3+q0q2);
    cb->dt[1][0] = 2.0*(q1q2+q0q3);
    cb->dt[1][1] = q0q0-q1q1+q2q2-q3q3;
    cb->dt[1][2] = 2.0*(q2q3-q0q1);
    cb->dt[2][0] = 2.0*(q1q3-q0q2);
    cb->dt[2][1] = 2.0*(q2q3+q0q1);
    cb->dt[2][2] = q0q0-q1q1-q2q2+q3q3;
}