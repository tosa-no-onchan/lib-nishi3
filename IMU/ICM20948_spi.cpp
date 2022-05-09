/*
* ICM20948_spi.cpp
*/
#include <Arduino.h>
#include <SPI.h>
#include "ICM20948_spi.h"


//#define SPI_CS_PIN          BDPIN_SPI_CS_IMU
                            
#define ICM20948_ADDRESS    0xEA
#define MPU_CALI_COUNT      512
#define MPU_SPI   SPI


//#define DEBUG_M

cICM20948_spi::cICM20948_spi(){}

/*----------------------------------
*  cICM20948_spi selectBank()
-----------------------------------*/
void cICM20948_spi::selectBank(uint8_t bank)
{
  digitalWrite( BDPIN_SPI_CS_IMU, LOW);

  /* clear R/W bit - write, send the address */
  MPU_SPI.write((uint8_t)ICM20948_REG_BANK_SEL);
  MPU_SPI.write((uint8_t)(bank << 4));

  digitalWrite( BDPIN_SPI_CS_IMU, HIGH);
}

/*----------------------------------
*  cICM20948_spi spiRead()
-----------------------------------*/
void cICM20948_spi::spiRead(uint16_t addr, uint8_t *p_data, uint32_t length)
{
  uint8_t reg_addr;
  uint8_t bank;

  reg_addr = (uint8_t) (addr & 0x7F);
  bank = (uint8_t) (addr >> 7);

  selectBank(bank);

  digitalWrite( BDPIN_SPI_CS_IMU, LOW);

  MPU_SPI.transfer(0x80 | reg_addr);
  //MPU_SPI.transfer(NULL, (void *)p_data, (size_t)length);
  // update by nishi
  MPU_SPI.transferBytes(NULL, (uint8_t *)p_data, (size_t)length);

  digitalWrite( BDPIN_SPI_CS_IMU, HIGH);
}

/*----------------------------------
*  cICM20948_spi spiReadByte()
-----------------------------------*/
uint8_t cICM20948_spi::spiReadByte(uint16_t addr)
{
  uint8_t data;

  spiRead(addr, &data, 1);

	return data;
}

/*----------------------------------
*  cICM20948_spi spiWriteByte()
-----------------------------------*/
void cICM20948_spi::spiWriteByte(uint16_t addr, uint8_t data)
{
  uint8_t reg_addr;
  uint8_t bank;
 
  reg_addr = (uint8_t) (addr & 0x7F);
  bank = (uint8_t) (addr >> 7);

  selectBank(bank);
  digitalWrite( BDPIN_SPI_CS_IMU, LOW); 

  MPU_SPI.transfer(reg_addr);
  MPU_SPI.transfer(data);

  digitalWrite( BDPIN_SPI_CS_IMU, HIGH); 
  //delay(1);
}


//void cICM20948_spi::spiWrite(uint16_t addr, uint8_t *data,uint32_t length)
//{
//  for(uint32_t i=0;i<length;i++){
//    spiWriteByte(addr+i, data[i]);
//  }
//}


//int  imu_spi_writes(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t *data)
void cICM20948_spi::spiWrite(uint16_t addr, uint8_t *data, uint32_t length)
{

  uint8_t reg_addr;
  uint8_t bank;
 
  reg_addr = (uint8_t) (addr & 0x7F);
  bank = (uint8_t) (addr >> 7);

  uint32_t i;

  selectBank(bank);

  digitalWrite( BDPIN_SPI_CS_IMU, LOW);
  MPU_SPI.transfer( reg_addr );

  for( i=0; i<length; i++ )
  {
    MPU_SPI.transfer( data[i] );
  }
  digitalWrite( BDPIN_SPI_CS_IMU, HIGH);
  delay(1);
}


void cICM20948_spi::AK09916_init(bool minimal){
	//////////////////////////////////////////////////////////////////////////
	//AK09916 Setup

  uint8_t data;

  int rc;
  ICM_20948_Status_e rc2;

  // i2cMasterPassthrough(false);
  data=spiReadByte(ICM20948_REG_INT_PIN_CFG);
  data &=0xff - ICM20948_BIT_BYPASS_EN;
  //SERIAL_PORT.print("cICM20948_spi::AK09916_init() #10  INT_PIN_CFG=");
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
  //SERIAL_PORT.print("cICM20948_spi::AK09916_init() #10  I2C_MST_CTRL=");
  //SERIAL_PORT.println(data,HEX);

  //#define TEST_X_1
  #ifdef TEST_X_1
    // mag who am i
    //ICM_20948_i2c_controller_periph4_txn(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len, bool Rw, bool send_reg_addr);
    data=0;
    rc2=ICM_20948_i2c_controller_periph4_txn(ICM20948_AK09916_I2C_ADDR, ICM20948_AK09916_WIA, &data, 1, true, true);
    SERIAL_PORT.print("cICM20948_spi::AK09916_init() #19  rc2=");
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
    SERIAL_PORT.print("cICM20948_spi::AK09916_init() #20 error");
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
    SERIAL_PORT.println("cICM20948_spi::AK09916_init() #21 error");
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
    SERIAL_PORT.println("cICM20948_spi::AK09916_init() #24 error");
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

  //SERIAL_PORT.println("cICM20948_spi::AK09916_init() #99");
}



#define TEST_XXXX
#ifdef TEST_XXXX

#define delay_ms delay

int cICM20948_spi::imu_spi_ak09916_reads(uint8_t akm_addr, uint8_t reg_addr, uint8_t len, uint8_t *data)
{
	uint8_t index = 0;
	uint8_t status = 0;
	uint32_t timeout = 0;
	uint8_t tmp = 0;

	tmp = akm_addr | ICM20948_I2C_READ;
	//imu_spi_writes(ICM20948_SPIx_ADDR, ICM20948_REG_I2C_SLV4_ADDR, 1, &tmp);
  spiWrite(ICM20948_REG_I2C_SLV4_ADDR, &tmp,1);

  //tmp=spiReadByte(ICM20948_REG_I2C_SLV4_ADDR);
  //SERIAL_PORT.print("cICM20948_spi::imu_spi_ak09916_reads() #1  I2C_SLV4_ADDR=");
  //SERIAL_PORT.println(tmp,HEX);


	delay(1);
	while(index < len){
		tmp = reg_addr + index;
		//imu_spi_writes(ICM20948_SPIx_ADDR, ICM20948_REG_I2C_SLV4_REG, 1, &tmp);
		spiWrite(ICM20948_REG_I2C_SLV4_REG, &tmp,1);
		delay(1);

    //tmp=spiReadByte(ICM20948_REG_I2C_SLV4_REG);
    //SERIAL_PORT.print("cICM20948_spi::imu_spi_ak09916_reads() #2  I2C_SLV4_REG=");
    //SERIAL_PORT.println(tmp,HEX);


		tmp = ICM20948_I2C_SLV4_EN+0x10;
		//imu_spi_writes(ICM20948_SPIx_ADDR, ICM20948_REG_I2C_SLV4_CTRL, 1, &tmp);
		spiWrite(ICM20948_REG_I2C_SLV4_CTRL, &tmp, 1);
		delay(1);


    //tmp=spiReadByte(ICM20948_REG_I2C_SLV4_CTRL);
    //SERIAL_PORT.print("cICM20948_spi::imu_spi_ak09916_reads() #3  I2C_SLV4_CTRL=");
    //SERIAL_PORT.println(tmp,HEX);

		do {
			if (timeout++ > 50){
				return -2;
			}
			//imu_spi_reads(ICM20948_SPIx_ADDR, ICM20948_REG_I2C_MST_STATUS, 1, &status);
			spiRead(ICM20948_REG_I2C_MST_STATUS, &status,1);

      //SERIAL_PORT.print("cICM20948_spi::imu_spi_ak09916_reads() #4  ICM20948_REG_I2C_MST_STATUS=");
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

int cICM20948_spi::imu_spi_ak09916_writes(uint8_t akm_addr, uint8_t reg_addr, uint8_t len, uint8_t *data)
{
  uint32_t timeout = 0;
	uint8_t status = 0;
	uint8_t tmp = 0;
	uint8_t index = 0;

	//imu_spi_writes(ICM20948_SPIx_ADDR, ICM20948_REG_I2C_SLV4_ADDR, 1, &tmp);
	spiWriteByte(ICM20948_REG_I2C_SLV4_ADDR, akm_addr);
	delay_ms(2);

  //SERIAL_PORT.print("cICM20948_spi::imu_spi_ak09916_writes() #1  akm_addr=");
  //SERIAL_PORT.println(akm_addr,HEX);


	while(index < len){
		tmp = reg_addr + index;
		//imu_spi_writes(ICM20948_SPIx_ADDR, ICM20948_REG_I2C_SLV4_REG, 1, &tmp);
		spiWriteByte(ICM20948_REG_I2C_SLV4_REG, tmp);
		delay_ms(2);

    //SERIAL_PORT.print("cICM20948_spi::imu_spi_ak09916_writes() #2  reg_addr + index=");
    //SERIAL_PORT.println(reg_addr + index,HEX);


		//imu_spi_writes(ICM20948_SPIx_ADDR, ICM20948_REG_I2C_SLV4_DO, 1, data + index);
		spiWriteByte(ICM20948_REG_I2C_SLV4_DO, data[index]);
		delay_ms(2);

    //SERIAL_PORT.print("cICM20948_spi::imu_spi_ak09916_writes() #3  data[index]=");
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

      //SERIAL_PORT.print("cICM20948_spi::imu_spi_ak09916_writes() #4  I2C_MST_STATUS=");
      //SERIAL_PORT.println(status,HEX);

			delay_ms(2);
		} while ((status & ICM20948_I2C_SLV4_DONE) == 0);
		if (status & ICM20948_I2C_SLV4_NACK)
			return -3;
		index++;
	}
	return 0;
}
int cICM20948_spi::imu_spi_ak09916_write(uint8_t akm_addr, uint8_t reg_addr, uint8_t data)
{
  uint8_t param[1];

  param[0] = data;

  return imu_spi_ak09916_writes(akm_addr,reg_addr, 1, param);
}

#endif

/*
* Rw : true / false -> Read / Write
*/
ICM_20948_Status_e cICM20948_spi::ICM_20948_i2c_controller_periph4_txn(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len, bool Rw, bool send_reg_addr)
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
  SERIAL_PORT.print("cICM20948_spi::ICM_20948_i2c_controller_periph4_txn() #1  addr=");
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
  SERIAL_PORT.print("cICM20948_spi::ICM_20948_i2c_controller_periph4_txn() #2  reg=");
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

