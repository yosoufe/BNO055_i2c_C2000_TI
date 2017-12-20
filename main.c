#include "main.h"

void hardware_init(void){
	imu_init();
}

struct bno055_accel_float_t myAcc;
unsigned char accel_calib_status = 0;
unsigned char gyro_calib_status = 0;
unsigned char mag_calib_status = 0;
unsigned char sys_calib_status = 0;



void main(void) {
	// The Proccessor Specific Initilisation
	InitSysCtrl();
	InitGpio();
	InitPieCtrl();
	IER = 0x0000;
	IFR = 0x0000;
	InitPieVectTable();
	
	hardware_init();		// IMU initilisation

	// Processor Specific
	EnableInterrupts();		// enable all interrupts after  hardware initialisations
	ERTM;  					// Enable Global realtime interrupt DBGM

	// IMU readings
	volatile int com_error = 0;
	for (;;){
		com_error += bno055_get_accel_calib_stat(&accel_calib_status);
		com_error += bno055_get_mag_calib_stat(&mag_calib_status);
		com_error += bno055_get_gyro_calib_stat(&gyro_calib_status);
		com_error += bno055_get_sys_calib_stat(&sys_calib_status);

		com_error = bno055_convert_float_accel_xyz_msq(&myAcc);
		com_error = 0;
	}
}