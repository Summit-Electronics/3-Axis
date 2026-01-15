/*
 * SSM3ST4840C.h
 */

#ifndef INC_SUMMIT_SSM3ST4840C_H_
#define INC_SUMMIT_SSM3ST4840C_H_

#include "stm32g0xx_hal.h"
#include "main.h"

#define MAX_WRITE_ACTIONS 80 //TMC5160 register buffer

typedef struct {
	SPI_HandleTypeDef* SPI_Handle; //SPI handle for the axis
	GPIO_TypeDef* CS_Port; 		  //Chip select port, GPIOA,GPIOB, etc..
	uint16_t CS_Pin;	   		  //Chip select pin,  GPIO_PIN_2 , etc..
	GPIO_TypeDef* DRV_ENN_Port;   //Drive Enable port, GPIOA,GPIOB, etc..
	uint16_t DRV_ENN_Pin;	   	  //Drive Enable pin,  GPIO_PIN_2 , etc..
	int ENC_ENB; 				  //1 enable , 0 disable
	int ENC_HOME; 				  //1 reset X_ENC at Z , 0 disable reset
	int STG_ENB;				  //1 enable Stallguard, 0 disable
	int32_t ENC_Resolution;		  //Encoder Resolution
}AxisConfig;

typedef struct {
	uint32_t VSTART;
	uint32_t A1;
	uint32_t V1;
	uint32_t AMAX;
	uint32_t VMAX;
	uint32_t DMAX;
	uint32_t D1;
	uint32_t VSTOP;
} RampConfig;

typedef struct {
	uint32_t IRUN;
	uint32_t IHOLD;
} CurrentConfig;

typedef struct {
	uint8_t writeAddresses[MAX_WRITE_ACTIONS];
	uint32_t writeValues[MAX_WRITE_ACTIONS];
	int numWriteActions; // Counter for the number of write actions stored
}SPI_WriteData;


/* TMC5160 functions */
void SSM_InitAxisConfig(void);
/* AxisConfig pin definition and variables config
 * Call once at boot to set up correct pin config
 */
void TMC5160_Basic_Init(CurrentConfig *Current, int Axis);
/* Perform basic initialize of the TMC5160 and enable the DRV_Enable pin which controls the H-Bridge.
 * Also sets IHOLD, IRUN and default chopper settings
 * default stepper config = 256 microstepping
 */
void TMC5160_Basic_Rotate(uint8_t Mode, RampConfig *Ramp, int Axis);
/* Start stepper rotation in direction: "Mode"
 * "Mode" 0 or 1, depanding on motor windings left/right
 *  with Rampingprofile as set by RampConfig
 */
void TMC5160_Rotate_To(uint32_t Position, RampConfig *Ramp, int Axis);
/* Rotate Stepper to "Position"
 * "Position": uint32_t (51200 = 1 full 360 degree turn)
 * with Rampingprofile as set by RampConfig
 */
void TMC5160_Rotate_All_To(uint32_t Position_1, uint32_t Position_2, uint32_t Position_3, RampConfig *Ramp);
/* Rotate Stepper on axis 1 to position_1, axis 2 to position_2 and axis 3 to position_3
 * position_1 : uint32_t (51200 = 1 rotation)
 * position_2 : uint32_t (51200 = 1 rotation)
 * position_3 : uint32_t (51200 = 1 rotation)
 * with Rampingprofile as set by RampConfig
 */
void TMC5160_Stop(int Axis);
/* Stop all movement by writing VMAX = 0; but leave H-bridge enabled
 */
void TMC5160_Drive_Enable(int state, int Axis);
/* Enables the DRV_ENB pin, powering the motor and disabling free movement of the axis
 * "state": int (1 = enable drive , 0 = disable drive)
 */
void TMC5160_Startup(int Axis);
/* Performs an SPI check to make sure TMC is powered and SPI communication is ok
 */
uint32_t TMC5160_SPIWrite(uint8_t Address, uint32_t Value, int Action, uint8_t *SPICheck, int Axis);
/* SPI "action" on register at "Address" for "Value" on the TMC5160
 * "Action" 0 or 1, 0 = read, 1 = write
 * "Address", register adress of TMC5160
 * "Value", uint32_t value for the register
 * "SPICheck" unint8_t pointer to SPICheck
 * "Axis" int specify which Axis is contolled
 */
uint32_t TMC5160_Get_Position(int Axis);
/* Read the position register on the TMC5160 (32 bit) and return this value.
 */
void TMC5160_Init_Stallguard(int reset, uint32_t SGT, int Axis);
/* Initialize Stallguard
 * "reset" = 1 will reset the stall event and allow the motor to move again
 * "SGT" = Stallguard Tuning parameter (-64 to +63) a higher value makes Stallguard less sensitive and requires more torque to indicate a stall
 * Besides SGT, Current and speed settings also influence Stallguard
 */
int TMC5160_Monitor_Stallguard(int Axis);
/* monitor stallguard values
 * returns stallguard Flag value (1 = stall , 0 = no stall)
 */
void TMC5160_Set_Home(int Axis);
/*set current position as home "0" in the TMC5160 position register
 */
void TMC5160_Init_Stealthchop(int Axis);
/* Initialize Stealthchop to reduce motor vibration and sound
 * Current and speed settings influence Stealthchop
 */


/* Encoder functions */
int32_t ENC_Get_Position(int Axis);
/* Read the Encoder value register on TMC5160, and upscale the value to match it to the 256 microstepping position of the TMC5160
 */
void ENC_Start_position(int Axis);
/*set current position to 0 for homing
 */


/* GPIO functions */
void Toggle_OUT(int port,uint16_t time);
/* Toggle (12V) Output on "port" for duration of "time"
 * "port" 1,2 or 3 respectively OUT1, OUT2 or OUT3
 * "time" duration in ms
 */
int Read_IN(int port);
/* Read input (5 to 12V) on "port"
 * "port" 1 to 6, respectively IN1 to IN6
 */
uint16_t Read_AIN(void);
/* Read analog input, absolute maximum = 5.0V
 * A voltage divider is used to scale voltage to MCU voltage.
 * (Input voltage x (2K / 1K+2K)) = voltage on MCU pin
 */

#endif /* INC_SUMMIT_SSM3ST4840C_H_ */
