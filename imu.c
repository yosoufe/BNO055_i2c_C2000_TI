#include "imu.h"

struct bno055_t myIMU;

enum i2c_err_t i2c_error = NO_ERROR;

int imu_init(void){
	I2CB_Init();
	int com_error = 0;


	myIMU.bus_read = BNO055_I2C_bus_read;
	myIMU.bus_write = BNO055_I2C_bus_write;
	myIMU.delay_msec = BNO055_delay_msek;
	myIMU.dev_addr = IMU_I2C_ADD;
	com_error += bno055_init(&myIMU);
	com_error += bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
	return com_error;
}

/*	\Brief: The API is used as I2C bus read
 *	\Return : Status of the I2C read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register,
 *   will data is going to be read
 *	\param reg_data : This data read from the sensor,
 *   which is hold in an array
 *	\param cnt : The no of byte of data to be read
 */
s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt){
	//
	// Wait until the STP bit is cleared from any previous master communication.
	// Clearing of this bit by the module is delayed until after the SCD bit is
	// set. If this bit is not checked prior to initiating a new message, the
	// I2C could get confused.
	//

	Uint16 i=0;
	static unsigned long int counter = 1;

	I2cbRegs.I2CSAR.all = dev_addr; 	// Setup slave address

	while(  ( (++counter % COUNTER_TIMEOUT) != 0) && (I2cbRegs.I2CSTR.bit.BB == 1) ){}

	if (I2cbRegs.I2CSTR.bit.BB == 1)
	{
		i2c_error=BUSY_BUS;
		return i2c_send_error(&counter);// I2C_BUS_BUSY_ERROR;
	}
	counter = 1;

	I2cbRegs.I2CCNT = 1; 				// Setup number of bytes to send
	I2cbRegs.I2CDXR.all = reg_addr;
	// Send start as master transmitter
	DELAY_US(20);
	I2cbRegs.I2CMDR.all = 0x6620; // FREE, Start, Master, Transmitter, reset; (wihout STOP)


	while( ( (++counter % COUNTER_TIMEOUT) != 0) && (I2cbRegs.I2CFFTX.bit.TXFFST != 0) ){} // wait untill the send is complete

	if (I2cbRegs.I2CFFTX.bit.TXFFST != 0)
	{
		i2c_error=NO_TX_END;
		return i2c_send_error(&counter);
	}
	counter = 1;


	I2cbRegs.I2CCNT = cnt; 			// Setup number of bytes to send/rec
	DELAY_US(20);
	I2cbRegs.I2CMDR.all =  0x6C20;         // free, Start, stop, master, reset (reciever mode)
	while( ( (++counter % COUNTER_TIMEOUT) != 0) && (I2cbRegs.I2CFFRX.bit.RXFFST != cnt)  ){}
	if (I2cbRegs.I2CFFRX.bit.RXFFST != cnt)
	{
		i2c_error=NO_RX_FLAG;
		I2cbRegs.I2CFFRX.bit.RXFFINTCLR = 1;
		return i2c_send_error(&counter);// I2C_BUS_BUSY_ERROR;
	}
	counter = 1;
	for (i = 0 ; i < cnt ; i++){
		reg_data[i] = I2cbRegs.I2CDRR.all ;
	}
	return BNO055_SUCCESS;
}

/*	\Brief: The API is used as SPI bus write
 *	\Return : Status of the SPI write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register,
 *   will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *	will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt){
	//
	// Wait until the STP bit is cleared from any previous master communication.
	// Clearing of this bit by the module is delayed until after the SCD bit is
	// set. If this bit is not checked prior to initiating a new message, the
	// I2C could get confused.
	//
	static unsigned long int counter = 1;
	I2cbRegs.I2CSAR.all = dev_addr; 	// Setup slave address

	while(  ( (++counter % COUNTER_TIMEOUT) != 0) && (I2cbRegs.I2CSTR.bit.BB == 1) ){}
	if (I2cbRegs.I2CSTR.bit.BB == 1)
	{
		i2c_error=BUSY_BUS;
		return i2c_send_error(&counter);// I2C_BUS_BUSY_ERROR;
	}
	counter = 1;

	I2cbRegs.I2CCNT = cnt + 1; 				// Setup number of bytes to send
	Uint16 i;
	I2cbRegs.I2CDXR.all = reg_addr;
	for (i = 0 ; i < cnt ; i++){
		I2cbRegs.I2CDXR.all = reg_data[i];
	}
	DELAY_US(20);
	I2cbRegs.I2CMDR.all = 0x6E20; // FREE, Start, STOP, Master, Transmitter, reset;

	while( ( (++counter % COUNTER_TIMEOUT) != 0) && (I2cbRegs.I2CFFTX.bit.TXFFST != 0) ){} 
	if (I2cbRegs.I2CFFTX.bit.TXFFST != 0)
	{
		i2c_error=NO_TX_END;
		I2cbRegs.I2CFFTX.bit.TXFFINTCLR = 1;
		return i2c_send_error(&counter);
	}
	counter = 1;
	I2cbRegs.I2CFFTX.bit.TXFFINTCLR = 1;
	return BNO055_SUCCESS;
}

int i2c_send_error(unsigned long* counter){
	*counter = 0;
	return BNO055_ERROR;
}

void BNO055_delay_msek(u32 msek){
	DELAY_US(msek*1000);
}


void I2CB_Init(void)
{
	/*	Pin Configuration
	 *  SDA B		GPIO2		8J80
	 *  SCL B		GPIO3		8J79
	 */

	EALLOW;
	// TURN ON THE I2CB CLOCK
	CpuSysRegs.PCLKCR9.bit.I2C_B = 1;
	PieVectTable.I2CB_INT = &i2cBISR;
	EDIS;




	I2cBGpioConfig(I2C_B_GPIO2_GPIO3);

	/* register interrupt function
	EALLOW;	// This is needed to write to EALLOW protected registers
	PieVectTable.I2cb_INT = &i2c_int1a_isr;
	EDIS;   // This is needed to disable write to EALLOW protected registers
	*/

	// Initialize I2C
	I2cbRegs.I2CMDR.all = 0x4000;	// reset I2C
	I2cbRegs.I2CPSC.all = 19;		// Prescaler - need 7-12 Mhz on module clk here: 10 MhZ = 200/(19+1)
	/*
	 * Now the Module clock is 10 Mhz
	 * to calculate the min values according to imu datasheet:
	 * I2CCLKL = 1.3e-6 (from imu Datasheet) * modFq(10e6) - 5
	 * I2CCLKH = 0.6e-6 * modFq(10e6) - 5
	 *
	 * but the above values creates more than 500 khz data clock which is not supported
	 * to decrease it to 400 khz the following values are used.
	 * more details: datasheet of TI page 1937
	 *
	 * f_SCL = 1.0/( ((19+1) * (11+5 + 4+5) )/(200e6) ) / 1e3 = 400 (khz)
	 * f_SCL = 1.0/( ((I2CPSC+1) * (I2CCLKL+5 + I2CCLKH+5) )/(SYSCLK) ) / 1e3 (in khz)
	 */
	I2cbRegs.I2CCLKL = 11;;			// NOTE: must be non zero
	I2cbRegs.I2CCLKH = 4;			// NOTE: must be non zero
	I2cbRegs.I2CIER.all = 0x0;		// No interrupt


	I2cbRegs.I2CMDR.all = 0x4020;	// Take I2C out of reset
									// Free I2C when suspended

	I2cbRegs.I2CFFTX.all = 0x6000;	// Enable FIFO mode and TXFIFO, clear interrupt
	I2cbRegs.I2CFFRX.all = 0x2040;	// Enable RXFIFO, clear RXFFINT,
	I2cbRegs.I2CFFTX.bit.TXFFINTCLR = 1;
	I2cbRegs.I2CFFRX.bit.RXFFINTCLR = 1;

	return;
}
