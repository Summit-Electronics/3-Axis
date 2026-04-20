/*
 * SSM3ST4840C.c
 */
#include <SSM3ST4840C.h>

#define POSITION_TOLERANCE 256 // Adjust this value as needed, currently it's set to 0.5%
#define NUM_SPI_INTERFACES 3

/* SPI VARIABLES */
uint8_t ResetCount = 0;
uint8_t ResetWriteAction[MAX_WRITE_ACTIONS];
uint8_t SPICheckValue = 0;
SPI_WriteData spiWriteData[NUM_SPI_INTERFACES] = {
	    { .numWriteActions = 0 },
	    { .numWriteActions = 0 },
	    { .numWriteActions = 0 }
};

/* TMC VARIABLES */
uint8_t TMC_Boot = 0;
uint32_t SG_RESULTS[1000]; //for logging
int x = 0; // for logging



double ENC_Factor_A1 = 0;
double ENC_Factor_A2 = 0;
double ENC_Factor_A3 = 0;

static AxisConfig AxisCfg[NUM_SPI_INTERFACES];  // just allocate, no init here


void SSM_InitAxisConfig(void)
{
    AxisCfg[0].SPI_Handle = AXIS_1_SPI;
    AxisCfg[0].CS_Port = A1_TMC_CSN_GPIO_Port;
    AxisCfg[0].CS_Pin = A1_TMC_CSN_Pin;
    AxisCfg[0].DRV_ENN_Port = A1_DRV_ENN_GPIO_Port;
    AxisCfg[0].DRV_ENN_Pin = A1_DRV_ENN_Pin;
    AxisCfg[0].ENC_ENB = ENC_ENB_A1;
    AxisCfg[0].ENC_HOME = ENC_HOME_A1;
    AxisCfg[0].STG_ENB = STG_ENB_A1;
    AxisCfg[0].ENC_Resolution = ENC_Resolution_A1;

    AxisCfg[1].SPI_Handle = AXIS_2_SPI;
    AxisCfg[1].CS_Port = A2_TMC_CSN_GPIO_Port;
    AxisCfg[1].CS_Pin = A2_TMC_CSN_Pin;
    AxisCfg[1].DRV_ENN_Port = A2_DRV_ENN_GPIO_Port;
    AxisCfg[1].DRV_ENN_Pin = A2_DRV_ENN_Pin;
    AxisCfg[1].ENC_ENB = ENC_ENB_A2;
    AxisCfg[1].ENC_HOME = ENC_HOME_A2;
    AxisCfg[1].STG_ENB = STG_ENB_A2;
    AxisCfg[1].ENC_Resolution = ENC_Resolution_A2;

    AxisCfg[2].SPI_Handle = AXIS_3_SPI;
    AxisCfg[2].CS_Port = A3_TMC_CSN_GPIO_Port;
    AxisCfg[2].CS_Pin = A3_TMC_CSN_Pin;
    AxisCfg[2].DRV_ENN_Port = A3_DRV_ENN_GPIO_Port;
    AxisCfg[2].DRV_ENN_Pin = A3_DRV_ENN_Pin;
    AxisCfg[2].ENC_ENB = ENC_ENB_A3;
    AxisCfg[2].ENC_HOME = ENC_HOME_A3;
    AxisCfg[2].STG_ENB = STG_ENB_A3;
    AxisCfg[2].ENC_Resolution = ENC_Resolution_A3;
}

void TMC5160_Basic_Init(CurrentConfig *Current, int Axis)
{
	/* CURRENT SETTINGS
	 I_RUN, Max Run current = XX = ~3.0A
	 I_HOLD, Max Hold current = XX = ~3.0A
	*/

	TMC5160_Startup(Axis); //check if SPI communication is ok
	TMC5160_Stop(Axis); //after SPI ok, stop motor

	uint32_t IHOLD_IRUN = 0x00070000; // standard IHOLD DELAY value


	if(Current->IHOLD > 23) // set upper current limit ~3.0A
	{
		Current->IHOLD = 23;
	}

	if(Current->IRUN > 23) // set upper current limit ~3.0A
	{
		Current->IRUN = 23;
	}

	IHOLD_IRUN += Current->IHOLD + (Current->IRUN <<8);

	TMC5160_SPIWrite(0x03, 0x00000000, 1, &SPICheckValue, Axis); // writing value 0x00000000 = 0 = 0.0 to address 1 = 0x03(SLAVECONF)
	TMC5160_SPIWrite(0x05, 0x00000000, 1, &SPICheckValue, Axis); // writing value 0x00000000 = 0 = 0.0 to address 2 = 0x05(X_COMPARE)
	TMC5160_SPIWrite(0x06, 0x00000000, 1, &SPICheckValue, Axis); // writing value 0x00000000 = 0 = 0.0 to address 3 = 0x06(OTP_PROG)
	TMC5160_SPIWrite(0x08, 0x0000000F, 1, &SPICheckValue, Axis); // writing value 0x0000000F = 15 = 0.0 to address 4 = 0x08(FACTORY_CONF)
	TMC5160_SPIWrite(0x09, 0x00010606, 1, &SPICheckValue, Axis); // writing value 0x00010606 = 67078 = 0.0 to address 5 = 0x09(SHORT_CONF)
	TMC5160_SPIWrite(0x0A, 0x00080400, 1, &SPICheckValue, Axis); // writing value 0x00080400 = 525312 = 0.0 to address 6 = 0x0A(DRV_CONF)
	TMC5160_SPIWrite(0x0B, 0x00000000, 1, &SPICheckValue, Axis); // writing value 0x00000000 = 0 = 0.0 to address 7 = 0x0B(GLOBAL_SCALER)

	TMC5160_SPIWrite(0x10, IHOLD_IRUN, 1, &SPICheckValue, Axis); // writing value 0x00070A03 = 461315 = 0.0 to address 8 = 0x10(IHOLD_IRUN)

	TMC5160_SPIWrite(0x11, 0x0000000A, 1, &SPICheckValue, Axis); // writing value 0x0000000A = 10 = 0.0 to address 9 = 0x11(TPOWERDOWN)
	TMC5160_SPIWrite(0x13, 0x00000000, 1, &SPICheckValue, Axis); // writing value 0x00000000 = 0 = 0.0 to address 10 = 0x13(TPWMTHRS)
	TMC5160_SPIWrite(0x14, 0x00000000, 1, &SPICheckValue, Axis); // writing value 0x00000010 = 16 = 0.0 to address 11 = 0x14(TCOOLTHRS)
	TMC5160_SPIWrite(0x15, 0x00000000, 1, &SPICheckValue, Axis); // writing value 0x00000000 = 0 = 0.0 to address 12 = 0x15(THIGH)
	TMC5160_SPIWrite(0x2C, 0x00000000, 1, &SPICheckValue, Axis); // writing value 0x00000000 = 0 = 0.0 to address 23 = 0x2C(TZEROWAIT)
	TMC5160_SPIWrite(0x33, 0x00000000, 1, &SPICheckValue, Axis); // writing value 0x00000000 = 0 = 0.0 to address 25 = 0x33(VDCMIN)
	TMC5160_SPIWrite(0x34, 0x00000000, 1, &SPICheckValue, Axis); // writing value 0x00000000 = 0 = 0.0 to address 26 = 0x34(SW_MODE)
	TMC5160_SPIWrite(0x38, 0x00000000, 1, &SPICheckValue, Axis); // writing value 0x00000000 = 0 = 0.0 to address 27 = 0x38(ENCMODE)
	TMC5160_SPIWrite(0x3A, 0x00010000, 1, &SPICheckValue, Axis); // writing value 0x00010000 = 65536 = 0.0 to address 29 = 0x3A(ENC_CONST)
	TMC5160_SPIWrite(0x3D, 0x00000000, 1, &SPICheckValue, Axis); // writing value 0x00000000 = 0 = 0.0 to address 30 = 0x3D(ENC_DEVIATION)
	TMC5160_SPIWrite(0x60, 0xAAAAB554, 1, &SPICheckValue, Axis); // writing value 0xAAAAB554 = 0 = 0.0 to address 31 = 0x60(MSLUT[0])
	TMC5160_SPIWrite(0x61, 0x4A9554AA, 1, &SPICheckValue, Axis); // writing value 0x4A9554AA = 1251300522 = 0.0 to address 32 = 0x61(MSLUT[1])
	TMC5160_SPIWrite(0x62, 0x24492929, 1, &SPICheckValue, Axis); // writing value 0x24492929 = 608774441 = 0.0 to address 33 = 0x62(MSLUT[2])
	TMC5160_SPIWrite(0x63, 0x10104222, 1, &SPICheckValue, Axis); // writing value 0x10104222 = 269500962 = 0.0 to address 34 = 0x63(MSLUT[3])
	TMC5160_SPIWrite(0x64, 0xFBFFFFFF, 1, &SPICheckValue, Axis); // writing value 0xFBFFFFFF = 0 = 0.0 to address 35 = 0x64(MSLUT[4])
	TMC5160_SPIWrite(0x65, 0xB5BB777D, 1, &SPICheckValue, Axis); // writing value 0xB5BB777D = 0 = 0.0 to address 36 = 0x65(MSLUT[5])
	TMC5160_SPIWrite(0x66, 0x49295556, 1, &SPICheckValue, Axis); // writing value 0x49295556 = 1227445590 = 0.0 to address 37 = 0x66(MSLUT[6])
	TMC5160_SPIWrite(0x67, 0x00404222, 1, &SPICheckValue, Axis); // writing value 0x00404222 = 4211234 = 0.0 to address 38 = 0x67(MSLUT[7])
	TMC5160_SPIWrite(0x68, 0xFFFF8056, 1, &SPICheckValue, Axis); // writing value 0xFFFF8056 = 0 = 0.0 to address 39 = 0x68(MSLUTSEL)
	TMC5160_SPIWrite(0x69, 0x00F70000, 1, &SPICheckValue, Axis); // writing value 0x00F70000 = 16187392 = 0.0 to address 40 = 0x69(MSLUTSTART)
	TMC5160_SPIWrite(0x6C, 0x00410153, 1, &SPICheckValue, Axis); // writing value 0x00410153 = 4260099 = 0.0 to address 41 = 0x6C(CHOPCONF)
	TMC5160_SPIWrite(0x6D, 0x00000000, 1, &SPICheckValue, Axis); // writing value 0x00000000 = 0 = 0.0 to address 42 = 0x6D(COOLCONF)
	TMC5160_SPIWrite(0x6E, 0x00000000, 1, &SPICheckValue, Axis); // writing value 0x00000000 = 0 = 0.0 to address 43 = 0x6E(DCCTRL)
	TMC5160_SPIWrite(0x70, 0xC40C001E, 1, &SPICheckValue, Axis); // writing value 0xC40C001E = 0 = 0.0 to address 44 = 0x70(PWMCONF)

	if(Axis == 1)
	{
		ENC_Factor_A1 = (51200.0 / ENC_Resolution_A1);
		if(ENC_HOME_A1 == 1)
		  {
			TMC5160_SPIWrite(0x38, 0x00000134, 1, &SPICheckValue, Axis); // writing value 0x00000000 = 0 = 0.0 to address 27 = 0x38(ENCMODE)
								 //0x00000124 for clear ONCE at N-event

			//latch and clear encoder counter X_ENC ONCE at N-event
			//N-event occurs when polarities B and A ,match both set at 0
			//pol-N = 1 active high
			//encoder X_ENC will be set to 0 when N event occurs which occurs when crossing Home position.

			ENC_Start_position(Axis); // read encoder current position
		  }
	}

	if(Axis == 2)
	{
		ENC_Factor_A2 = (51200.0 / ENC_Resolution_A2);
		if(ENC_HOME_A2 == 1)
		  {
			TMC5160_SPIWrite(0x38, 0x00000134, 1, &SPICheckValue, Axis); // writing value 0x00000000 = 0 = 0.0 to address 27 = 0x38(ENCMODE)
								 //0x00000124 for clear ONCE at N-event

			//latch and clear encoder counter X_ENC ONCE at N-event
			//N-event occurs when polarities B and A ,match both set at 0
			//pol-N = 1 active high
			//encoder X_ENC will be set to 0 when N event occurs which occurs when crossing Home position.

			ENC_Start_position(Axis); // read encoder current position
		  }
	}

	if(Axis == 3)
	{
		ENC_Factor_A3 = (51200.0 / ENC_Resolution_A3);
		if(ENC_HOME_A3 == 1)
		  {
			TMC5160_SPIWrite(0x38, 0x00000134, 1, &SPICheckValue, Axis); // writing value 0x00000000 = 0 = 0.0 to address 27 = 0x38(ENCMODE)
								 //0x00000124 for clear ONCE at N-event

			//latch and clear encoder counter X_ENC ONCE at N-event
			//N-event occurs when polarities B and A ,match both set at 0
			//pol-N = 1 active high
			//encoder X_ENC will be set to 0 when N event occurs which occurs when crossing Home position.

			ENC_Start_position(Axis); // read encoder current position
		  }
	}

}

void TMC5160_Init_Stealthchop(int Axis)
{
	 	TMC5160_SPIWrite(0x00, 0x0000000C, 1, &SPICheckValue, Axis);
	 	TMC5160_SPIWrite(0x70, 0xC40D001E, 1, &SPICheckValue, Axis);
	    TMC5160_SPIWrite(0x6C, 0x00410153, 1, &SPICheckValue, Axis);
}

void TMC5160_Init_Stallguard(int reset, uint32_t SGT, int Axis)
{
	if(reset == 0)//basic stall setup
	{
		if((int32_t)SGT > 63)
		{
			SGT = 63; //SGT max value
		}

		else if((int32_t)SGT < -64)
		{
			SGT = -64; //SGT min value
		}

		uint32_t COOLCONF = 0;

		COOLCONF += (SGT <<16);

		TMC5160_SPIWrite(0x6D, COOLCONF, 1, &SPICheckValue, Axis); //0x6D(COOLCONF) //Stallguard Threshold
	}

	if(reset == 1)//Reset stall flag, to allow movement again
	{
		TMC5160_SPIWrite(0x35, 0x00000040, 1, &SPICheckValue, Axis); //0x35(RAMP_STAT)
	}

	// Stall is dependent on the used motor, current setting and speed settings.
	// Tuning must be done inside the application
	// first rotate the stepper at a speed appropriate for your application
	// Next read SG_RESULT
	// Apply load and monitor SG_RESULT
	// If SG_RESULT reaches 0 before "stall event" , increase SGT value (+63 highest) , if 0 is not reached lower SGT value (-64 lowest)
	// Sudden motor stops can cause a stall event.
	// In this case , increase TCOOLTHRS to increase the duration before stallguard is activated (not included in this function).
}

int TMC5160_Monitor_Stallguard(int Axis)
{
	uint32_t DRV_STATUS;
	int Stall_Flag = 0;

	DRV_STATUS = TMC5160_SPIWrite(0x6F, 0x00000000, 0, &SPICheckValue, Axis); //Read (DRV_STATUS)

	Stall_Flag = (DRV_STATUS & (1 << 24)); //bit 24 of DRV_STATUS is Stallguard flag
	Stall_Flag = (Stall_Flag >> 24); // bitshift stall flag is 0 or 1

	for(int y = 32; y > 10; y--) // clear other data but leave SG_RESULT
	{
		DRV_STATUS &= ~(1 << y);
	}

	SG_RESULTS[x] = DRV_STATUS; // see SG_RESULTS in explorer for tuning stallguard (SG_RESULT 0 = stall detected)

	if(Stall_Flag != 0 && SG_RESULTS[x] == 0) //stall detected -> stop motor
	{
		Stall_Flag = 1;
		TMC5160_Init_Stallguard(1,0,Axis); // clear stall flag
		TMC5160_Stop(Axis);
	}

	else
	{
		Stall_Flag = 0;
	}


	x++;

	if(x > 999) // prevent logging overflow
	{
		x = 0;
	}

	return Stall_Flag;
}

void TMC5160_Set_Home(int Axis)
{
	TMC5160_Drive_Enable(0, Axis);//disable motor
	TMC5160_SPIWrite(0x21, 0, 1, &SPICheckValue, Axis); //write current pos as home "0"
	TMC5160_Stop(Axis);
	TMC5160_Drive_Enable(1, Axis);//enable motor
}

void TMC5160_Basic_Rotate(uint8_t Mode, RampConfig *Ramp, int Axis) // 0 = Velocity Mode + , 1 = Velocity Mode -
{
	TMC5160_SPIWrite(0x11, 	0x0000000A, 1, &SPICheckValue, Axis); 	// writing value 0x0000000A = 10 = 0.0 to address 9 = 0x11(TPOWERDOWN)

	//Velocity mode , using VMAX and AMAX
	TMC5160_SPIWrite(0x23, Ramp->VSTART, 1, &SPICheckValue, Axis); 	// = 1000 = 0.0 to address 15 = 0x23(VSTART)
	TMC5160_SPIWrite(0x26, Ramp->AMAX, 1, &SPICheckValue, Axis); 		// = 12800 = 0.0 to address 18 = 0x26(AMAX)
	TMC5160_SPIWrite(0x27, Ramp->VMAX, 1, &SPICheckValue, Axis); 		// = 51200 = 0.0 to address 19 = 0x27(VMAX)
	TMC5160_SPIWrite(0x2B, Ramp->VSTOP, 1, &SPICheckValue, Axis); 	// = 10 = 0.0 to address 22 = 0x2B(VSTOP)

	if(Mode == 0)
	{
		TMC5160_SPIWrite(0x20, 	0x00000001, 1, &SPICheckValue, Axis); 		// writing value 0x00000001 = 0 = 0.0 to address 13 = 0x20(RAMPMODE)VM +
	}

	else if(Mode == 1)
	{
		TMC5160_SPIWrite(0x20, 	0x00000002, 1, &SPICheckValue, Axis); 		// writing value 0x00000002 = 0 = 0.0 to address 13 = 0x20(RAMPMODE) VM -
	}
}


void TMC5160_Rotate_To(uint32_t Position, RampConfig *Ramp, int Axis)
{
	uint32_t Target_Pos = 0;
	uint32_t ENC_Pos = 0;
	uint32_t TMC_Pos = 0;
	int Pos_Reached = 0;

	TMC5160_SPIWrite(0x11, 	0x0000000A, 1, &SPICheckValue, Axis); 	// writing value 0x0000000A = 10 = 0.0 to address 9 = 0x11(TPOWERDOWN)

	// setup for profile
	TMC5160_SPIWrite(0x23, Ramp->VSTART, 1, &SPICheckValue, Axis); 	// = 1000 = 0.0 to address 15 = 0x23(VSTART)
	TMC5160_SPIWrite(0x24, Ramp->A1, 1, &SPICheckValue, Axis); 		// = 5600 = 0.0 to address 16 = 0x24(A1)
	TMC5160_SPIWrite(0x25, Ramp->V1, 1, &SPICheckValue, Axis); 		// = 12800 = 0.0 to address 17 = 0x25(V1)
	TMC5160_SPIWrite(0x26, Ramp->AMAX, 1, &SPICheckValue, Axis); 		// = 12800 = 0.0 to address 18 = 0x26(AMAX)
	TMC5160_SPIWrite(0x27, Ramp->VMAX, 1, &SPICheckValue, Axis); 		// = 51200 = 0.0 to address 19 = 0x27(VMAX)
	TMC5160_SPIWrite(0x28, Ramp->DMAX, 1, &SPICheckValue, Axis); 		// = 700 = 0.0 to address 20 = 0x28(DMAX)
	TMC5160_SPIWrite(0x2A, Ramp->D1, 1, &SPICheckValue, Axis); 		// = 1400 = 0.0 to address 21 = 0x2A(D1)
	TMC5160_SPIWrite(0x2B, Ramp->VSTOP, 1, &SPICheckValue, Axis); 	// = 10 = 0.0 to address 22 = 0x2B(VSTOP)

	TMC5160_SPIWrite(0x20, 	0x00000000, 1, &SPICheckValue, Axis); 	// writing value 0x00000000 = 0 = 0.0 to address 13 = 0x20(RAMPMODE) MTP
	TMC5160_SPIWrite(0x2D, Position, 1, &SPICheckValue, Axis); 		// writing value to address 24 = 0x2D(XTARGET)  1 lap
	TMC5160_SPIWrite(0x21,	0x00000000, 0, &SPICheckValue, Axis);		// READ position register

	Target_Pos = Position;

	if(Position == 0) //To fix the first reading when target position = 0
	{
		ENC_Pos = 1;
		TMC_Pos = 1;
	}


	//Enter loop to check if position is reached
	while(Pos_Reached != 1) // loop until Position is reached
	//while(TMC_Pos != Target_Pos)
	{
		TMC_Pos = TMC5160_Get_Position(Axis);

		if(AxisCfg[Axis-1].ENC_ENB == 1)// if enabled, check Encoder position
		{
			ENC_Pos = ENC_Get_Position(Axis);
		}


		if (SPICheckValue & 0x20) //bit 5 = position reached
		{
			/*
			 *	Compare to AMS & ENC sensor data
			 */

			Pos_Reached = 1; //to break loop
		}
	}

	 (void)Target_Pos;
	 (void)TMC_Pos;
	 (void)ENC_Pos;
 }

void TMC5160_Rotate_All_To(uint32_t Position_1, uint32_t Position_2, uint32_t Position_3, RampConfig *Ramp)
{
	uint32_t Target_Pos_1 = 0;
	uint32_t Target_Pos_2 = 0;
	uint32_t Target_Pos_3 = 0;

	uint32_t ENC_Pos_1 = 0;
	uint32_t ENC_Pos_2 = 0;
	uint32_t ENC_Pos_3 = 0;

	uint32_t TMC_Pos_1 = 0;
	uint32_t TMC_Pos_2 = 0;
	uint32_t TMC_Pos_3 = 0;

	int Pos_Reached = 0;

	TMC5160_SPIWrite(0x11, 	0x0000000A, 1, &SPICheckValue, 1); 	// writing value 0x0000000A = 10 = 0.0 to address 9 = 0x11(TPOWERDOWN)

		// setup for profile
	TMC5160_SPIWrite(0x23, Ramp->VSTART, 1, &SPICheckValue, 1); 	// = 1000 = 0.0 to address 15 = 0x23(VSTART)
	TMC5160_SPIWrite(0x24, Ramp->A1, 1, &SPICheckValue, 1); 		// = 5600 = 0.0 to address 16 = 0x24(A1)
	TMC5160_SPIWrite(0x25, Ramp->V1, 1, &SPICheckValue, 1); 		// = 12800 = 0.0 to address 17 = 0x25(V1)
	TMC5160_SPIWrite(0x26, Ramp->AMAX, 1, &SPICheckValue, 1); 		// = 12800 = 0.0 to address 18 = 0x26(AMAX)
	TMC5160_SPIWrite(0x27, Ramp->VMAX, 1, &SPICheckValue, 1); 		// = 51200 = 0.0 to address 19 = 0x27(VMAX)
	TMC5160_SPIWrite(0x28, Ramp->DMAX, 1, &SPICheckValue, 1); 		// = 700 = 0.0 to address 20 = 0x28(DMAX)
	TMC5160_SPIWrite(0x2A, Ramp->D1, 1, &SPICheckValue, 1); 		// = 1400 = 0.0 to address 21 = 0x2A(D1)
	TMC5160_SPIWrite(0x2B, Ramp->VSTOP, 1, &SPICheckValue, 1); 	// = 10 = 0.0 to address 22 = 0x2B(VSTOP)

	TMC5160_SPIWrite(0x11, 	0x0000000A, 1, &SPICheckValue, 2); 	// writing value 0x0000000A = 10 = 0.0 to address 9 = 0x11(TPOWERDOWN)

		// setup for profile
	TMC5160_SPIWrite(0x23, Ramp->VSTART, 1, &SPICheckValue, 2); 	// = 1000 = 0.0 to address 15 = 0x23(VSTART)
	TMC5160_SPIWrite(0x24, Ramp->A1, 1, &SPICheckValue, 2); 		// = 5600 = 0.0 to address 16 = 0x24(A1)
	TMC5160_SPIWrite(0x25, Ramp->V1, 1, &SPICheckValue, 2); 		// = 12800 = 0.0 to address 17 = 0x25(V1)
	TMC5160_SPIWrite(0x26, Ramp->AMAX, 1, &SPICheckValue, 2); 		// = 12800 = 0.0 to address 18 = 0x26(AMAX)
	TMC5160_SPIWrite(0x27, Ramp->VMAX, 1, &SPICheckValue, 2); 		// = 51200 = 0.0 to address 19 = 0x27(VMAX)
	TMC5160_SPIWrite(0x28, Ramp->DMAX, 1, &SPICheckValue, 2); 		// = 700 = 0.0 to address 20 = 0x28(DMAX)
	TMC5160_SPIWrite(0x2A, Ramp->D1, 1, &SPICheckValue, 2); 		// = 1400 = 0.0 to address 21 = 0x2A(D1)
	TMC5160_SPIWrite(0x2B, Ramp->VSTOP, 1, &SPICheckValue, 2); 	// = 10 = 0.0 to address 22 = 0x2B(VSTOP)

	TMC5160_SPIWrite(0x11, 	0x0000000A, 1, &SPICheckValue, 3); 	// writing value 0x0000000A = 10 = 0.0 to address 9 = 0x11(TPOWERDOWN)

		// setup for profile
	TMC5160_SPIWrite(0x23, Ramp->VSTART, 1, &SPICheckValue, 3); 	// = 1000 = 0.0 to address 15 = 0x23(VSTART)
	TMC5160_SPIWrite(0x24, Ramp->A1, 1, &SPICheckValue, 3); 		// = 5600 = 0.0 to address 16 = 0x24(A1)
	TMC5160_SPIWrite(0x25, Ramp->V1, 1, &SPICheckValue, 3); 		// = 12800 = 0.0 to address 17 = 0x25(V1)
	TMC5160_SPIWrite(0x26, Ramp->AMAX, 1, &SPICheckValue, 3); 		// = 12800 = 0.0 to address 18 = 0x26(AMAX)
	TMC5160_SPIWrite(0x27, Ramp->VMAX, 1, &SPICheckValue, 3); 		// = 51200 = 0.0 to address 19 = 0x27(VMAX)
	TMC5160_SPIWrite(0x28, Ramp->DMAX, 1, &SPICheckValue, 3); 		// = 700 = 0.0 to address 20 = 0x28(DMAX)
	TMC5160_SPIWrite(0x2A, Ramp->D1, 1, &SPICheckValue, 3); 		// = 1400 = 0.0 to address 21 = 0x2A(D1)
	TMC5160_SPIWrite(0x2B, Ramp->VSTOP, 1, &SPICheckValue, 3); 		// = 10 = 0.0 to address 22 = 0x2B(VSTOP)

	TMC5160_SPIWrite(0x20, 	0x00000000, 1, &SPICheckValue, 1); 		// writing value 0x00000000 = 0 = 0.0 to address 13 = 0x20(RAMPMODE) MTP
	TMC5160_SPIWrite(0x2D, Position_1, 1, &SPICheckValue, 1); 		// writing value to address 24 = 0x2D(XTARGET)  1 lap
	TMC5160_SPIWrite(0x21,	0x00000000, 0, &SPICheckValue, 1);		// READ position register
	Target_Pos_1 = Position_1;

	TMC5160_SPIWrite(0x20, 	0x00000000, 1, &SPICheckValue, 2); 		// writing value 0x00000000 = 0 = 0.0 to address 13 = 0x20(RAMPMODE) MTP
	TMC5160_SPIWrite(0x2D, Position_2, 1, &SPICheckValue, 2); 		// writing value to address 24 = 0x2D(XTARGET)  1 lap
	TMC5160_SPIWrite(0x21,	0x00000000, 0, &SPICheckValue, 2);		// READ position register
	Target_Pos_2 = Position_2;

	TMC5160_SPIWrite(0x20, 	0x00000000, 1, &SPICheckValue, 3); 		// writing value 0x00000000 = 0 = 0.0 to address 13 = 0x20(RAMPMODE) MTP
	TMC5160_SPIWrite(0x2D, Position_3, 1, &SPICheckValue, 3); 		// writing value to address 24 = 0x2D(XTARGET)  1 lap
	TMC5160_SPIWrite(0x21,	0x00000000, 0, &SPICheckValue, 3);		// READ position register
	Target_Pos_3 = Position_3;

	if(Position_1 == 0) //To fix the first reading when target position = 0
	{
		ENC_Pos_1 = 1;
		TMC_Pos_1 = 1;
	}

	if(Position_2 == 0) //To fix the first reading when target position = 0
	{
		ENC_Pos_2 = 1;
		TMC_Pos_2 = 1;
	}

	if(Position_3 == 0) //To fix the first reading when target position = 0
	{
		ENC_Pos_3 = 1;
		TMC_Pos_3 = 1;
	}

	//write code to check all 3 positions and only when all 3 match, continue

	//Enter loop to check if position is reached
	while(Pos_Reached != 1) // loop until Position is reached
	//while(TMC_Pos != Target_Pos)
	{
		TMC_Pos_1 = TMC5160_Get_Position(1);
		TMC_Pos_2 = TMC5160_Get_Position(2);
		TMC_Pos_3 = TMC5160_Get_Position(3);

		if(TMC_Pos_1 == Position_1 && TMC_Pos_2 == Position_2 && TMC_Pos_3 == Position_3)
		{
			Pos_Reached = 1;
		}

		/*
		if (SPICheckValue & 0x20) //bit 5 = position reached
		{
			Add custom implementation here to compare to ENC sensor data

			Pos_Reached = 1; //to break loop
		} */
	}

	(void) Target_Pos_1;
	(void) Target_Pos_2;
	(void) Target_Pos_3;
	(void) ENC_Pos_1;
	(void) ENC_Pos_2;
	(void) ENC_Pos_3;
}

uint32_t TMC5160_Get_Position(int Axis)
{
	uint32_t AngleT = 0;

	AngleT = TMC5160_SPIWrite(0x21, 0x00000000, 0, &SPICheckValue, Axis); //read step counter from TMC5160

	return AngleT;
}

void TMC5160_Stop(int Axis)
{
	TMC5160_SPIWrite(0x27,0x00000000, 1, &SPICheckValue, Axis); //set VMAX to 0
}

void TMC5160_Drive_Enable(int state, int Axis)
{
	int Started = 0;
	uint32_t DRV_STATUS = 0;
	uint32_t IOIN = 0;

	if(state == 1) // Enable driver
	{
		while(Started == 0)
		{
			HAL_GPIO_WritePin(AxisCfg[Axis - 1].DRV_ENN_Port,AxisCfg[Axis - 1].DRV_ENN_Pin, 0); // LOW = ON
			HAL_Delay(10);
			DRV_STATUS = TMC5160_SPIWrite(0x6F, 0x00000000, 0, &SPICheckValue, Axis);

			if (DRV_STATUS & 0x18003000) // bit 28 / 27 / 13 / 12 short in H-bridge -> toggle DRV_enn
			{
				HAL_GPIO_WritePin(AxisCfg[Axis - 1].DRV_ENN_Port, AxisCfg[Axis - 1].DRV_ENN_Pin, 1); // High = OFF
				HAL_Delay(1000);

			continue;
			}

	    	if (SPICheckValue & 0x02) //bit 1 = driver_error
			{
				HAL_GPIO_WritePin(AxisCfg[Axis - 1].DRV_ENN_Port, AxisCfg[Axis - 1].DRV_ENN_Pin, 1); // HIGH = OFF
				HAL_Delay(1000);

				TMC5160_SPIWrite(0x01, 0x00000002, 1, &SPICheckValue, Axis); //clear drive error bit

				continue;
			}

	    	IOIN = TMC5160_SPIWrite(0x04, 0x00000000, 0, &SPICheckValue, Axis);


	    	if((IOIN & (1 <<4)) == 0) // check if DRV_ENN is set in software
	    	{
	    		Started = 1; //no issues during motor power up.
	    	}
	    	else
	    	{
	    		HAL_GPIO_WritePin(AxisCfg[Axis - 1].DRV_ENN_Port, AxisCfg[Axis - 1].DRV_ENN_Pin, 1);  // HIGH = OFF if not successfully enabled
	    		HAL_Delay(1000);  // Retry after delay
	    	}
		}
	}

	if(state == 0) // disable drive
	{
		HAL_GPIO_WritePin(AxisCfg[Axis - 1].DRV_ENN_Port, AxisCfg[Axis - 1].DRV_ENN_Pin, 1); // HIGH = OFF
		HAL_Delay(10);
	}
}

void TMC5160_Startup(int Axis)
{
	uint32_t GCONF = 0;
    uint8_t SPITxData[5];  // TX data array SPI2
    uint8_t SPIRxData[5];  // RX data array SPI2

	HAL_GPIO_WritePin(AxisCfg[Axis - 1].CS_Port, AxisCfg[Axis - 1].CS_Pin, 0); // set TMC CS low
	SPITxData[0] = 0x81;//81
	SPITxData[1] = 0x00;
	SPITxData[2] = 0x00;
	SPITxData[3] = 0x00;
	SPITxData[4] = 0x01;//01 // clear reset flag
	HAL_SPI_TransmitReceive(AxisCfg[Axis - 1].SPI_Handle, SPITxData, SPIRxData, 0x05, 100); //transmit, clear reset flag
	HAL_GPIO_WritePin(AxisCfg[Axis - 1].CS_Port, AxisCfg[Axis - 1].CS_Pin, 1); // set TMC CS high

	HAL_Delay(5);

	HAL_GPIO_WritePin(AxisCfg[Axis - 1].CS_Port, AxisCfg[Axis - 1].CS_Pin, 0); // set TMC CS low
	SPITxData[0] = 0x01;
	SPITxData[1] = 0x00;
	SPITxData[2] = 0x00;
	SPITxData[3] = 0x00;
	SPITxData[4] = 0x00;
	HAL_SPI_TransmitReceive(AxisCfg[Axis - 1].SPI_Handle, SPITxData, SPIRxData, 0x05, 100); //read, reset flag
	HAL_GPIO_WritePin(AxisCfg[Axis - 1].CS_Port, AxisCfg[Axis - 1].CS_Pin, 1); // set TMC CS high

	TMC5160_SPIWrite(0x00, 0x00000008, 1, &SPICheckValue, Axis); // writing value 0x00000008 = 8 = 0.0 to address 0 = 0x00(GCONF)
	GCONF = TMC5160_SPIWrite(0x00, 0x00000008, 0, &SPICheckValue, Axis); // writing value 0x00000008 = 8 = 0.0 to address 0 = 0x00(GCONF)

	if (!(GCONF & 0x00000008)) //0x00000008 is present, SPI is ok
	{
		//TMC not correctly written, try again.
		TMC5160_SPIWrite(0x00, 0x00000008, 1, &SPICheckValue, Axis); // writing value 0x00000008 = 8 = 0.0 to address 0 = 0x00(GCONF)
		GCONF = TMC5160_SPIWrite(0x00, 0x00000008, 0, &SPICheckValue, Axis); // writing value 0x00000008 = 8 = 0.0 to address 0 = 0x00(GCONF)
		while (!(GCONF & 0x00000008))
		{
			//TMC not correctly written, try again.
			TMC5160_SPIWrite(0x00, 0x00000008, 1, &SPICheckValue, Axis); // writing value 0x00000008 = 8 = 0.0 to address 0 = 0x00(GCONF)
			GCONF = TMC5160_SPIWrite(0x00, 0x00000008, 0, &SPICheckValue, Axis); // writing value 0x00000008 = 8 = 0.0 to address 0 = 0x00(GCONF)

			/* TODO add loop breaking function, f.e. error handler after x attempts*/
		}
	}

	//if reset bit is set, clear it and read if it's cleared
	if (SPIRxData[0] & 0x01) // reset bit set , rewrite all registers from backup
	{
		HAL_GPIO_WritePin(AxisCfg[Axis - 1].CS_Port, AxisCfg[Axis - 1].CS_Pin, 0); // set TMC CS low
		SPITxData[0] = 0x81;
		SPITxData[1] = 0x00;
		SPITxData[2] = 0x00;
		SPITxData[3] = 0x00;
		SPITxData[4] = 0x01; // clear reset flag
		HAL_SPI_TransmitReceive(AxisCfg[Axis - 1].SPI_Handle, SPITxData, SPIRxData, 0x05, 100); //transmit, clear reset flag
		HAL_GPIO_WritePin(AxisCfg[Axis - 1].CS_Port, AxisCfg[Axis - 1].CS_Pin, 1); // set TMC CS high

		HAL_Delay(5);

		HAL_GPIO_WritePin(AxisCfg[Axis - 1].CS_Port, AxisCfg[Axis - 1].CS_Pin, 0); // set TMC CS low
		SPITxData[0] = 0x01;
		SPITxData[1] = 0x00;
		SPITxData[2] = 0x00;
		SPITxData[3] = 0x00;
		SPITxData[4] = 0x00;
		HAL_SPI_TransmitReceive(AxisCfg[Axis - 1].SPI_Handle, SPITxData, SPIRxData, 0x05, 100); //read, reset flag
	}

	TMC_Boot = 1;
}

uint32_t TMC5160_SPIWrite(uint8_t Address, uint32_t Value, int Action, uint8_t *SPICheck, int Axis)
{
    uint8_t SPITxData[5];  // TX data array SPI2
    uint8_t SPIRxData[5];  // RX data array SPI2
    uint32_t SPIRx = 0;
    int index; //register index

    int SPIIndex = Axis - 1;//the SPI index
    SPI_WriteData *currentSPI = &spiWriteData[SPIIndex];

    HAL_GPIO_WritePin(AxisCfg[Axis - 1].CS_Port, AxisCfg[Axis - 1].CS_Pin, 0); // set TMC CS low

    if (Action == 1) // Write
    {
        SPITxData[0] = Address + 0x80;
    }
    else // Read
    {
        SPITxData[0] = Address;
    }

    SPITxData[1] = Value >> 24;
    SPITxData[2] = Value >> 16;
    SPITxData[3] = Value >> 8;
    SPITxData[4] = Value;

    HAL_SPI_TransmitReceive(AxisCfg[Axis - 1].SPI_Handle, SPITxData, SPIRxData, 0x05, 100);
    *SPICheck = SPIRxData[0]; // Update SPICheck

	if (Action != 1)
	{
		//ignore first response so toggle CS pin
		HAL_GPIO_WritePin(AxisCfg[Axis - 1].CS_Port, AxisCfg[Axis - 1].CS_Pin, 1); // set TMC CS high
		HAL_Delay(2); //5
		HAL_GPIO_WritePin(AxisCfg[Axis - 1].CS_Port, AxisCfg[Axis - 1].CS_Pin, 0); // set TMC CS low
		HAL_SPI_TransmitReceive(AxisCfg[Axis - 1].SPI_Handle, SPITxData, SPIRxData, 0x05, 100); // overwrite the data
		*SPICheck = SPIRxData[0]; // Update SPICheck

	}

    SPIRx += (SPIRxData[1] << 24);
    SPIRx += (SPIRxData[2] << 16);
    SPIRx += (SPIRxData[3] << 8);
    SPIRx += (SPIRxData[4] << 0);

    HAL_GPIO_WritePin(AxisCfg[Axis - 1].CS_Port, AxisCfg[Axis - 1].CS_Pin, 1); // set TMC CS high

    if (Action == 1) // Write action, so store adress and value for backup
	{
		// Check if the address already exists in the array
		for (index = 0; index < currentSPI->numWriteActions; index++)
		{
			if (currentSPI->writeAddresses[index] == Address)
			{
			    currentSPI->writeValues[index] = Value;  // Update existing value
			    break;
			}

		}

		// If the address doesn't exist, add it to the arrays
		if (index == currentSPI->numWriteActions)
		{
			if (currentSPI->numWriteActions < MAX_WRITE_ACTIONS)
			{
				currentSPI->writeAddresses[currentSPI->numWriteActions] = Address;
				currentSPI->writeValues[currentSPI->numWriteActions] = Value;
				currentSPI->numWriteActions++;
			}
			else
			{
				// Handle the case where the maximum number of write actions is exceeded
				// MAX_WRITE_ACTIONS is larger then number of registers on the TMC5160
			}
		}
	}

	if (SPIRxData[0] & 0x01 && TMC_Boot == 1) // reset bit set , rewrite all registers from backup
	{
		//CLEAR RESET FLAG
		HAL_GPIO_WritePin(AxisCfg[Axis - 1].CS_Port, AxisCfg[Axis - 1].CS_Pin, 0); // set TMC CS low
		SPITxData[0] = 0x81;
		SPITxData[1] = 0x00;
		SPITxData[2] = 0x00;
		SPITxData[3] = 0x00;
		SPITxData[4] = 0x01; // clear reset
		HAL_SPI_TransmitReceive(AxisCfg[Axis - 1].SPI_Handle, SPITxData, SPIRxData, 0x05, 100); // overwrite the data
		HAL_GPIO_WritePin(AxisCfg[Axis - 1].CS_Port, AxisCfg[Axis - 1].CS_Pin, 1); // set TMC CS high

		HAL_Delay(5);

		HAL_GPIO_WritePin(AxisCfg[Axis - 1].CS_Port, AxisCfg[Axis - 1].CS_Pin, 0); // set TMC CS low
		SPITxData[0] = 0x01;
		SPITxData[1] = 0x00;
		SPITxData[2] = 0x00;
		SPITxData[3] = 0x00;
		SPITxData[4] = 0x00;
		HAL_SPI_TransmitReceive(AxisCfg[Axis - 1].SPI_Handle, SPITxData, SPIRxData, 0x05, 100); // overwrite the data
		HAL_GPIO_WritePin(AxisCfg[Axis - 1].CS_Port, AxisCfg[Axis - 1].CS_Pin, 1); // set TMC CS high

		//WRITE BACKUP REGISTERS AS THEY HAVE BEEN RESET
		ResetWriteAction[ResetCount] = currentSPI->numWriteActions;
		ResetCount++; //times reset occured

		for (int i = 0; i < currentSPI->numWriteActions; i++)
		{
			SPITxData[0] = currentSPI->writeAddresses[i] + 0x80;
			SPITxData[1] = currentSPI->writeValues[i] >> 24;
			SPITxData[2] = currentSPI->writeValues[i] >> 16;
			SPITxData[3] = currentSPI->writeValues[i] >> 8;
			SPITxData[4] = currentSPI->writeValues[i];

			HAL_GPIO_WritePin(AxisCfg[Axis - 1].CS_Port, AxisCfg[Axis - 1].CS_Pin, 0); // set TMC CS low
			HAL_SPI_TransmitReceive(AxisCfg[Axis - 1].SPI_Handle, SPITxData, SPIRxData, 0x05, 100);
			HAL_GPIO_WritePin(AxisCfg[Axis - 1].CS_Port, AxisCfg[Axis - 1].CS_Pin, 1); // set TMC CS high
		}
	}

	return SPIRx;
}

int32_t ENC_Get_Position(int Axis)
{
	int32_t Enc_Position = TMC5160_SPIWrite(0x39, 0x00000000, 0, &SPICheckValue, Axis); // read encoder position
	float Fenc_Pos = Enc_Position;

	if(Axis == 1)
	{
		Fenc_Pos = Fenc_Pos * ENC_Factor_A1;
	}

	if(Axis == 2)
	{
		Fenc_Pos = Fenc_Pos * ENC_Factor_A2;
	}

	if(Axis == 3)
	{
		Fenc_Pos = Fenc_Pos * ENC_Factor_A3;
	}

	Enc_Position = (int32_t)Fenc_Pos;

	Enc_Position = ((Enc_Position) * (-1)); // to fix orientation

	return Enc_Position;
}

void ENC_Start_position(int Axis) //set current position to start position
{
	TMC5160_SPIWrite(0x39, 0x00000000, 1, &SPICheckValue, Axis); //write current position to 0
}


void Toggle_OUT(int port ,uint16_t time)
{
	if(port == 1)//Enable OUT1 for X sec (12V)
	{
	 HAL_GPIO_WritePin(EXT_OUT_1_GPIO_Port,EXT_OUT_1_Pin,1);
	 HAL_Delay(time);
	 HAL_GPIO_WritePin(EXT_OUT_1_GPIO_Port,EXT_OUT_1_Pin,0);
	}

	if(port == 2) // Enable OUT2 for X sec (12V)
	{
	 HAL_GPIO_WritePin(EXT_OUT_2_GPIO_Port,EXT_OUT_2_Pin,1);
	 HAL_Delay(time);
	 HAL_GPIO_WritePin(EXT_OUT_2_GPIO_Port,EXT_OUT_2_Pin,0);
	}

	if(port == 3) // Enable OUT3 for X sec (12V)
	{
	 HAL_GPIO_WritePin(EXT_OUT_3_GPIO_Port,EXT_OUT_3_Pin,1);
	 HAL_Delay(time);
	 HAL_GPIO_WritePin(EXT_OUT_3_GPIO_Port,EXT_OUT_3_Pin,0);
	}
}

int Read_IN(int port)
{
	int val = 0;

	if(port == 1)//Read IN1 (5 to 12V = 1)
	{
		val = HAL_GPIO_ReadPin(EXT_IN_1_GPIO_Port, EXT_IN_1_Pin);
	}

	if(port == 2)//Read IN1 (5 to 12V = 1)
	{
		val = HAL_GPIO_ReadPin(EXT_IN_2_GPIO_Port, EXT_IN_2_Pin);

	}
	if(port == 3)//Read IN1 (5 to 12V = 1)
	{
		val = HAL_GPIO_ReadPin(EXT_IN_3_GPIO_Port, EXT_IN_3_Pin);

	}
	if(port == 4)//Read IN1 (5 to 12V = 1)
	{
		val = HAL_GPIO_ReadPin(EXT_IN_4_GPIO_Port, EXT_IN_4_Pin);

	}
	if(port == 5)//Read IN1 (5 to 12V = 1)
	{
		val = HAL_GPIO_ReadPin(EXT_IN_5_GPIO_Port, EXT_IN_5_Pin);

	}
	if(port == 6)//Read IN1 (5 to 12V = 1)
	{
		val = HAL_GPIO_ReadPin(EXT_IN_6_GPIO_Port, EXT_IN_6_Pin);

	}

	return val;
}

uint16_t Read_AIN(void)
{
	uint16_t ADCReading = 0;

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 10);
	ADCReading = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	return ADCReading;
}


