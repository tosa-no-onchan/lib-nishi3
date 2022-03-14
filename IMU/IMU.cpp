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
cIMU::cIMU()
{
  uint8_t i;

  for( i=0; i<3; i++ )
  {
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
uint8_t cIMU::begin( uint32_t hz )
{
	uint8_t err_code = IMU_OK;
  uint32_t i;
  uint32_t pre_time;

  update_hz = hz;
  update_us = 1000000/hz;

  aRes = 8.0/32768.0;      // 8g
  //aRes = 2.0/32768.0;      // 2g
  
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

  tf_dlt[0]=0.0;
  tf_dlt[1]=0.0;
  tf_dlt[2]=0.0;

  quat[0]=1.0;
  quat[1]=0.0;
  quat[2]=0.0;
  quat[3]=0.0;

  bConnected = SEN.begin();
  
 	digitalWrite(LED_BUILTIN, LOW);		// light OFF
  if( bConnected == true )
  {
    filter.begin(update_hz);
    //filter.begin();

    for (i=0; i<32; i++)
    {
      update();
    }

    pre_time = millis();
   	#ifdef USE_DMP_NISHI
    while(!SEN.dmp_cali_get_done()){
      update();

      if (millis()-pre_time > 5000)
      {
        break;
      }
    }
    #else
    while(!SEN.gyro_cali_get_done())
    {
      update();

      if (millis()-pre_time > 5000)
      {
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
uint16_t cIMU::update( uint32_t option )
{
  // changed by nishi
  //UNUSED(option);

	uint16_t ret_time = 0;

	static uint32_t tTime;


	if( (micros()-tTime) >= update_us )
	{
		ret_time = micros()-tTime;
    tTime = micros();

		computeIMU();

		gyroData[0] = SEN.gyroADC[0];
		gyroData[1] = SEN.gyroADC[1];
		gyroData[2] = SEN.gyroADC[2];

    gyroRaw[0]  = SEN.gyroRAW[0];
    gyroRaw[1]  = SEN.gyroRAW[1];
    gyroRaw[2]  = SEN.gyroRAW[2];

    accData[0]  = SEN.accADC[0];
    accData[1]  = SEN.accADC[1];
    accData[2]  = SEN.accADC[2];

    accRaw[0]   = SEN.accRAW[0];
    accRaw[1]   = SEN.accRAW[1];
    accRaw[2]   = SEN.accRAW[2];

    magData[0]  = SEN.magADC[0];
    magData[1]  = SEN.magADC[1];
    magData[2]  = SEN.magADC[2];

    magRaw[0]   = SEN.magRAW[0];
    magRaw[1]   = SEN.magRAW[1];
    magRaw[2]   = SEN.magRAW[2];
	}

	return ret_time;
}


#define FILTER_NUM    3

/*---------------------------------------------------------------------------
     TITLE   : compute
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cIMU::computeIMU( void )
{
  static uint32_t prev_process_time = micros();
  static uint32_t cur_process_time = 0;
  static uint32_t process_time = 0;
  uint32_t i;
  static int32_t gyroADC[3][FILTER_NUM] = {0,};
  int32_t gyroAdcSum;

  static int32_t accADC[4][FILTER_NUM] = {0,};
  int32_t accAdcSum;


  uint32_t axis;


  #ifdef USE_DMP_NISHI
  #ifdef USE_IMU_DIST
	SEN.acc_get_adc();
	SEN.gyro_get_adc();
  #endif
  if(SEN.dmp_get_adc()!=true){
    //digitalWrite(LED_BUILTIN, HIGH-digitalRead(LED_BUILTIN));   // blink the blue
    return;
  }
  if(SEN.calibratingD_f!=1){
    return;
  }
  #else
	SEN.acc_get_adc();
	SEN.gyro_get_adc();
  //if (SEN.mag_get_adc()==true){
  //  digitalWrite(LED_BUILTIN, HIGH-digitalRead(LED_BUILTIN));   // blink the blue
  //}
  if(SEN.calibratingA_f!=1){
    return;
  }
  #endif

  for (axis = 0; axis < 3; axis++){
    gyroADC[axis][0] = SEN.gyroADC[axis];
    //accADC[axis][0] = SEN.accADC[axis];
    gyroAdcSum = 0;
    //accAdcSum = 0;
    for (i=0; i<FILTER_NUM; i++){
      gyroAdcSum += gyroADC[axis][i];
      //accAdcSum += accADC[axis][i];
    }
    SEN.gyroADC[axis] = gyroAdcSum/FILTER_NUM;
    //SEN.accADC[axis] = (accAdcSum/FILTER_NUM);
    for (i=FILTER_NUM-1; i>0; i--){
      gyroADC[axis][i] = gyroADC[axis][i-1];
      //accADC[axis][i] = accADC[axis][i-1];
    }
    if (abs(SEN.gyroADC[axis]) <= 3){
      SEN.gyroADC[axis] = 0;
    }
    //if (abs(SEN.accADC[axis]) <= 50){
    //  SEN.accADC[axis] = 0;
    //}
  }

  for( i=0; i<3; i++ )
  {
    accRaw[i]   = SEN.accRAW[i];
    accData[i]  = SEN.accADC[i];
    gyroRaw[i]  = SEN.gyroRAW[i];
    gyroData[i] = SEN.gyroADC[i];
    magRaw[i]   = SEN.magRAW[i];
    magData[i]  = SEN.magADC[i];
  }


  //SERIAL_PORT.print(F("ax:"));
  //SERIAL_PORT.print(accData[0]);
  //SERIAL_PORT.print(F(" ay:"));
  //SERIAL_PORT.print(accData[1]);
  //SERIAL_PORT.print(F(" az:"));
  //SERIAL_PORT.println(accData[2]);

  ax = (float)SEN.accADC[0]*aRes;
  ay = (float)SEN.accADC[1]*aRes;
  az = (float)SEN.accADC[2]*aRes;



  gx = (float)SEN.gyroADC[0]*gRes;
  gy = (float)SEN.gyroADC[1]*gRes;
  gz = (float)SEN.gyroADC[2]*gRes;


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

  // add by nishi
  //quat_dmp[0] = SEN.quat[0];
  //quat_dmp[1] = SEN.quat[1];
  //quat_dmp[2] = SEN.quat[2];
  //quat_dmp[3] = SEN.quat[3];

  #endif

  #ifdef USE_IMU_DIST
  computeTF();
  #endif


}
void cIMU::computeTF(void){
  static uint32_t tTime_TF;
  double roll, pitch, yaw;

  double s = (double)(micros()-tTime_TF)/1000000.0;

  tTime_TF = micros();

  //QuaternionToEulerAngles(quat[0], quat[1], quat[2], quat[3],roll, pitch, yaw);

  // そのまま計算すると、1G の影響が出る。
  // 今の速度を計算。

  float v_acc_x,v_acc_y,v_acc_z;


  //int axi,ayi,azi;
	//axi =(int)roundf(ax*100.0);
	//ax = (float)axi / 100.0;
	//ayi =(int)roundf(ay*100.0);
	//ay = (float)ayi / 100.0;
	//azi =(int)roundf(az*100.0);
	//az = (float)azi / 100.0;


  v_acc_x = ax*s;
  v_acc_y = ay*s;
  v_acc_z = az*s;

  //int v_acc_xi,v_acc_yi,v_acc_zi;
	//v_acc_xi =(int)roundf(v_acc_x*1000.0);
	//v_acc_x = (float)v_acc_xi / 1000.0;
	//v_acc_yi =(int)roundf(v_acc_y*1000.0);
	//v_acc_y = (float)v_acc_yi / 1000.0;
	//v_acc_zi =(int)roundf(v_acc_z*1000.0);
	//v_acc_z = (float)v_acc_zi / 1000.0;


  // 今の速度を計算
  v_acc[0] += v_acc_x;
  v_acc[1] += v_acc_y;
  v_acc[2] += v_acc_z;

  double Pn[3];
  compCBvBdt(quat,v_acc,s,Pn);

  int Pn_xi,Pn_yi,Pn_zi;
	Pn_xi =(int)roundf(Pn[0]*100.0);
	Pn[0] = (double)Pn_xi / 100.0;
	Pn_yi =(int)roundf(Pn[1]*100.0);
	Pn[1] = (double)Pn_yi / 100.0;
	//v_acc_zi =(int)roundf(v_acc_z*1000.0);
	//v_acc_z = (float)v_acc_zi / 1000.0;

  //Pn[0] /= 1000.0;
  //Pn[1] /= 1000.0;
  //今回の移動距離を計算。
  tf_dlt[0] += Pn[0];   // delta x
  tf_dlt[1] += Pn[1];   // delta y
  //tf_dlt[2] += Pn[2];   // delta z

  //tf_dlt[0] = 0.5 * v_acc[0] * s;   // delta x
  //tf_dlt[1] = 0.5 * v_acc[1] * s;   // delta y
  //tf_dlt[2] = 0.5 * v_acc[2] * s;   // delta z

  //int dlt0_i,dlt1_i,dlt2_i;
	//dlt0_i =(int)roundf(tf_dlt[0]*1000.0);
	//tf_dlt[0] = (float)dlt0_i / 1000.0;
	//dlt1_i =(int)roundf(tf_dlt[1]*1000.0);
	//tf_dlt[1] = (float)dlt1_i / 1000.0;
	//dlt2_i =(int)roundf(tf_dlt[2]*1000.0);
	//tf_dlt[2] = (float)dlt2_i / 1000.0;


  SERIAL_PORT.print(F("ax:"));
  SERIAL_PORT.print(v_acc[0],8);
  SERIAL_PORT.print(F(" ay:"));
  SERIAL_PORT.println(v_acc[1],8);
  //SERIAL_PORT.print(F(" az:"));
  //SERIAL_PORT.println(v_acc[2], 5);

  // ここで、IMU ローカルから、グローバルへ変換が必要。

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