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

  cali_tf=0;    // Madgwick Caliburation

 	digitalWrite(LED_BUILTIN, LOW);		// light OFF

  for(i=0;i<4;i++){
    bConnected = SEN.begin();
    if(bConnected == true)
      break;
    delayMicroseconds(100);        // 100us停止
  }
  
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
    err_code=1;
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

	computeIMU();

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

  static int32_t accADC[3][FILTER_NUM] = {0,};
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
    if(SEN.calibratingD_f == 0){
      return;
    }
  #endif
  #if defined(USE_ACC_NISHI)
    // Acc Caliburation not yet?
    if(SEN.calibratingA_f == 0){
      return;
    }
  #endif
  #if defined(USE_GRYO_NISHI)
    //if (SEN.mag_get_adc()==true){
    //  digitalWrite(LED_BUILTIN, HIGH-digitalRead(LED_BUILTIN));   // blink the blue
    //}
    if(SEN.calibratingG_f == 0){
      return;
    }
  #endif

  #if defined(USE_ACC_NISHI_X)
    // ACC 平滑化
    for (axis = 0; axis < 3; axis++){
      accADC[axis][0] = SEN.accADC[axis];
      accAdcSum = 0;
      for (i=0; i<FILTER_NUM; i++){
        accAdcSum += accADC[axis][i];
      }
      SEN.accADC[axis] = accAdcSum/FILTER_NUM;
      for (i=FILTER_NUM-1; i>0; i--){
        accADC[axis][i] = accADC[axis][i-1];
      }
    }
  #endif

  #if defined(USE_GRYO_NISHI_X)
    // GYRO 平滑化
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
    // GYRO 0 noise cut off
    for (axis = 0; axis < 3; axis++){
      if (abs(gyroData[axis]) <= 4){
        gyroData[axis] = 0;
      }
    }
  #endif

  //SERIAL_PORT.print(F("gyroData[0]:"));
  //SERIAL_PORT.print(gyroData[0]);
  //SERIAL_PORT.print(F(" gyroData[1]:"));
  //SERIAL_PORT.print(gyroData[1]);
  //SERIAL_PORT.print(F(" gyroData[2]:"));
  //SERIAL_PORT.println(gyroData[2]);

  //SERIAL_PORT.print(F("accData[0]:"));
  //SERIAL_PORT.print(accData[0]);
  //SERIAL_PORT.print(F(" accData[1]:"));
  //SERIAL_PORT.print(accData[1]);
  //SERIAL_PORT.print(F(" accData[2]:"));
  //SERIAL_PORT.println(accData[2]);


  #if defined(USE_ACC_NISHI)
    ax = (float)accData[0]*SEN.aRes;
    ay = (float)accData[1]*SEN.aRes;
    az = (float)accData[2]*SEN.aRes;
  #endif

  #if defined(USE_GRYO_NISHI)
    gx = (float)gyroData[0]*SEN.gRes;
    gy = (float)gyroData[1]*SEN.gRes;
    gz = (float)gyroData[2]*SEN.gRes;

    //SERIAL_PORT.print(F("gx:"));
    //SERIAL_PORT.print(gx,8);
    //SERIAL_PORT.print(F(" gy:"));
    //SERIAL_PORT.print(gy,8);
    //SERIAL_PORT.print(F(" gz:"));
    //SERIAL_PORT.println(gz,8);

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
      mx = (float)SEN.magADC[0]*SEN.mRes;
      my = (float)SEN.magADC[1]*SEN.mRes;
      mz = (float)SEN.magADC[2]*SEN.mRes;
    #endif
  #endif

  cur_process_time  = micros();
  process_time      = cur_process_time-prev_process_time;
  prev_process_time = cur_process_time;

  #ifdef USE_MADWICK
    #ifdef IMU_SENSER6
      if (SEN.calibratingG_f != 0 && SEN.calibratingA_f != 0){
        filter.invSampleFreq = (float)process_time/1000000.0f;
        filter.updateIMU(gx, gy, gz, ax, ay, az);
      }
      else{
        return;
      }
    #else
      if (SEN.calibratingG_f != 0 && SEN.calibratingA_f != 0 && SEN.calibratingM==0){
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

    quat_tmp[0] = filter.q0;  // W
    quat_tmp[1] = filter.q1;  // X
    quat_tmp[2] = filter.q2;  // Y
    quat_tmp[3] = filter.q3;  // Z

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
  #endif

  #ifdef USE_DMP_NISHI
    // use ICM20948 DMP Fusion.  add by nishi 2021.11.6
    quat[0] = SEN.quat[0];  // W
    quat[1] = SEN.quat[1];  // X
    quat[2] = SEN.quat[2];  // Y
    quat[3] = SEN.quat[3];  // Z
  #endif


  #ifdef USE_IMU_DIST
    // Cumpute CB 
    compCB(quat_tmp,&cb);

    double acc_Zero[3];

    // 1G の分配値を計算

    // 2G -> SEN.accZeroSum:8698
    // 4G ->  SEN.accZeroSum:8723
    //SERIAL_PORT.print(F("SEN.accZeroSum:"));
    //SERIAL_PORT.println(SEN.accZeroSum);    // 4238  -> 8 G

    //acc_Zero[0] = (int16_t)(cb.dt[2][0]*(double)SEN.accZeroSum);
    //acc_Zero[1] = (int16_t)(cb.dt[2][1]*(double)SEN.accZeroSum);
    //acc_Zero[2] = (int16_t)(cb.dt[2][2]*(double)SEN.accZeroSum);

    //acc_Zero[0] = (int16_t)(cb.dt[2][0]*ACC_MAX_G);
    //acc_Zero[1] = (int16_t)(cb.dt[2][1]*ACC_MAX_G);
    //acc_Zero[2] = (int16_t)(cb.dt[2][2]*ACC_MAX_G);


    acc_Zero[0] = cb.dt[2][0]*(ACC_MAX_G - SEN.zero_off);  
    acc_Zero[1] = cb.dt[2][1]*(ACC_MAX_G - SEN.zero_off);  
    acc_Zero[2] = cb.dt[2][2]*(ACC_MAX_G - SEN.zero_off);

 
    //acc_Zero[0] = cb.dt[2][0]*4238.0;  
    //acc_Zero[1] = cb.dt[2][1]*4238.0;  
    //acc_Zero[2] = cb.dt[2][2]*4238.0;

    //acc_Zero[0] = (int16_t)(cb.dt[2][0]*8698.0);    //
    //acc_Zero[1] = (int16_t)(cb.dt[2][1]*8698.0);
    //acc_Zero[2] = (int16_t)(cb.dt[2][2]*8698.0);    // accData[0]:-4 accData[1]:-21 accData[2]:-506

    //SEN.accZeroSum:8669
    //accData[0]:-30 accData[1]:-36 accData[2]:-469

    // acc 計測値から、1G をキャンセルします。 -> 読み込み値での計算
    ax = (double)accData[0] - acc_Zero[0];
    ay = (double)accData[1] - acc_Zero[1];
    az = (double)accData[2] - acc_Zero[2];

    //SERIAL_PORT.print(F("ax:"));
    //SERIAL_PORT.print(ax,8);
    //SERIAL_PORT.print(F(" ay:"));
    //SERIAL_PORT.print(ay,8);
    //SERIAL_PORT.print(F(" az:"));
    //SERIAL_PORT.println(az,8);


    // acc のノイズの 削除
    if (fabsf(ax) <= ACC_X_CUT_OFF) ax = 0;
    if (fabsf(ay) <= ACC_Y_CUT_OFF) ay = 0;
    if (fabsf(az) <= ACC_Z_CUT_OFF_P) az = 0;
    
    // ax,ay,az を、 表示して、 Arduino IDE の Serial plotter で、ACC_X_CUT_OFF の範囲を決めます。
    SERIAL_PORT.print(F("ax:"));
    SERIAL_PORT.print(ax,8);
    SERIAL_PORT.print(F(" ay:"));
    SERIAL_PORT.print(ay,8);
    SERIAL_PORT.print(F(" az:"));
    SERIAL_PORT.println(az,8);


    double dlt[3];    // acc_x,y,z 基準座標系 加速度

    // 今回の加速度 を 基準座標系に変換
    dlt[0]=cb.dt[0][0]*ax+cb.dt[0][1]*ay+cb.dt[0][2]*az;
    dlt[1]=cb.dt[1][0]*ax+cb.dt[1][1]*ay+cb.dt[1][2]*az;
    dlt[2]=cb.dt[2][0]*ax+cb.dt[2][1]*ay+cb.dt[2][2]*az;

    // dlt を、 表示して、 Arduino IDE の Serial plotter で、CUT_OFF の範囲を決めます。
    //SERIAL_PORT.print(F("dlt[0]:"));
    //SERIAL_PORT.print(dlt[0]);
    //SERIAL_PORT.print(F(" dlt[1]:"));
    //SERIAL_PORT.print(dlt[1]);
    //SERIAL_PORT.print(F(" dlt[2]:"));
    //SERIAL_PORT.println(dlt[2]);

    //SERIAL_PORT.print(F("ax:"));
    //SERIAL_PORT.print(ax,8);
    //SERIAL_PORT.print(F(" ay:"));
    //SERIAL_PORT.print(ay,8);
    //SERIAL_PORT.print(F(" az:"));
    //SERIAL_PORT.println(az,8);

    // Madgwick Caliburation OK?
    if(cali_tf >= 7000){
      //SERIAL_PORT.print(F("ax:"));
      //SERIAL_PORT.print(ax,8);
      //SERIAL_PORT.print(F(" ay:"));
      //SERIAL_PORT.print(ay,8);
      //SERIAL_PORT.print(F(" az:"));
      //SERIAL_PORT.println(az,8);


      //SERIAL_PORT.print(F("accData[0]:"));
      //SERIAL_PORT.print(accData[0],8);
      //SERIAL_PORT.print(F(" accData[1]:"));
      //SERIAL_PORT.print(accData[1],8);
      //SERIAL_PORT.print(F(" accData[2]:"));
      //SERIAL_PORT.println(accData[2],8);


      //SERIAL_PORT.print(F("gyroData[0]:"));
      //SERIAL_PORT.print(gyroData[0]);
      //SERIAL_PORT.print(F(" gyroData[1]:"));
      //SERIAL_PORT.print(gyroData[1]);
      //SERIAL_PORT.print(F(" gyroData[2]:"));
      //SERIAL_PORT.println(gyroData[2]);


      //SERIAL_PORT.print(F(" accRaw[0]:"));
      //SERIAL_PORT.println(accRaw[0],8);
      //SERIAL_PORT.print(F(" accRaw[1]:"));
      //SERIAL_PORT.println(accRaw[1],8);
      //SERIAL_PORT.print(F(" accRaw[2]:"));
      //SERIAL_PORT.println(accRaw[2],8);

      //SERIAL_PORT.print(F("dlt[0]:"));
      //SERIAL_PORT.print(dlt[0]);
      //SERIAL_PORT.print(F(" dlt[1]:"));
      //SERIAL_PORT.print(dlt[1]);
      //SERIAL_PORT.print(F(" dlt[2]:"));
      //SERIAL_PORT.println(dlt[2]);


      quat[0] = quat_tmp[0];  // W
      quat[1] = quat_tmp[1];  // X
      quat[2] = quat_tmp[2];  // Y
      quat[3] = quat_tmp[3];  // Z

      computeTF(process_time,dlt);
    }
    else{
      cali_tf ++;
    }

  #endif


  #ifdef XXX_0
  uint32_t t = millis();
  ac_cnt++;
  if(t > tTime[0]){

    uint32_t hz = ac_cnt/5;

    SERIAL_PORT.print(F("acc_hz:"));
    SERIAL_PORT.println(hz, 4);

    tTime[0] = t + 5000;	// set 1-cycle-time [ms] to odom Timer.
    ac_cnt=0;
  }
  #endif

}

/*---------------------------------------------------------------------------
     TITLE   : computeTF
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cIMU::computeTF(unsigned long process_time,double *dlt){
  double roll, pitch, yaw;

  double s = (double)process_time/1000000.0;

  //QuaternionToEulerAngles(quat[0], quat[1], quat[2], quat[3],roll, pitch, yaw);

  double v_acc_dlt[3];

  // 速度を累計
  v_acc[0] += dlt[0]*SEN.aRes*s;
  v_acc[1] += dlt[1]*SEN.aRes*s;
  v_acc[2] += dlt[2]*SEN.aRes*s;


  // 起動時にゴミが入ってくるみたい。
  //v_acc[0]:-1.15206158  v_acc_pre[0]:0.00000000
  //v_acc[1]:-0.33362532  v_acc_pre[1]:0.00000000
  //v_acc[2]:0.75112534  v_acc_pre[2]:0.00000000
  //v_acc[0]:-1.15281248  v_acc_pre[0]:-1.15206158
  //v_acc[1]:-0.33397377  v_acc_pre[1]:-0.33362532
  //v_acc[2]:0.75114322  v_acc_pre[2]:0.75112534


  // 速度を Cut Off
  //#define VX_CUT_OFF 0.003
  //#define VY_CUT_OFF 0.003
  //#define VZ_CUT_OFF 0.003
  //0.0002
  //#define VX_CUT_OFF 0.00012
  //#define VX_CUT_OFF 0.000012
  #define VX_CUT_OFF 0.000024
  //#define VX_CUT_OFF 0.0003

  //#define VY_CUT_OFF 0.00012
  //#define VY_CUT_OFF 0.000012
  #define VY_CUT_OFF 0.000024
  //#define VY_CUT_OFF 0.0003

  //#define VZ_CUT_OFF 0.00012
  //#define VZ_CUT_OFF 0.000012
  #define VZ_CUT_OFF 0.000024
  //#define VZ_CUT_OFF 0.0006

  if(ax == 0.0 && ay == 0.0 && az == 0.0){
    float v_acc_dl[3];
    v_acc_dl[0] = fabsf(v_acc[0] - v_acc_pre[0]);
    v_acc_dl[1] = fabsf(v_acc[1] - v_acc_pre[1]);
    v_acc_dl[2] = fabsf(v_acc[2] - v_acc_pre[2]);

    //#define KKKK3
    #ifdef KKKK3
      SERIAL_PORT.print(F("v_acc_dl[0]:"));
      SERIAL_PORT.print(v_acc_dl[0],8);
      SERIAL_PORT.print(F(" v_acc_dl[1]:"));
      SERIAL_PORT.print(v_acc_dl[1],8);
      SERIAL_PORT.print(F(" v_acc_dl[2]:"));
      SERIAL_PORT.println(v_acc_dl[2],8);
    #endif

    // v_acc_dl[0]:0.00009981 v_acc_dl[1]:0.00015712 v_acc_dl[2]:0.00057411
    // v_acc_dl[0]:0.00008539 v_acc_dl[1]:0.00040845 v_acc_dl[2]:0.00026176
    // v_acc_dl[0]:0.00078104 v_acc_dl[1]:0.00036857 v_acc_dl[2]:0.00020621
    // v_acc_dl[0]:0.00001029 v_acc_dl[1]:0.00034127 v_acc_dl[2]:0.00011996


    if(v_acc[0] != 0.0){
      if(v_acc_dl[0] <= VX_CUT_OFF){
        //SERIAL_PORT.println(F("v_acc[0]: Zero"));
        v_acc[0]=0.0;
      }
    }
    if(v_acc[1] != 0.0){
      if(v_acc_dl[1] <= VY_CUT_OFF){
        //SERIAL_PORT.println(F("v_acc[1]: Zero"));
        v_acc[1]=0.0;
      }
    }
    if(v_acc[2] != 0.0){
      if(v_acc_dl[2] <= VZ_CUT_OFF){
        //SERIAL_PORT.println(F("v_acc[2]: Zero"));
        v_acc[2]=0.0;
      }
    }
    //v_acc[0]=0.0;
    //v_acc[1]=0.0;
    //v_acc[2]=0.0;
  }
  
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
