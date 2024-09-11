/*  
Rebeca Gaston
2DX3 Final Project
*/
#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"





#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          																// 7) disable analog functionality on PB2,3

                                                                            // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                        						// 8) configure for 100 kbps clock
        
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}
void PortM_Init(void){
	//Use PortM pins (PM0-PM3) for output
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;				// activate clock for Port M
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};	// allow time for clock to stabilize
	GPIO_PORTM_DIR_R |= 0x0F;        								// configure Port M pins (PM0-PM3) as output
  GPIO_PORTM_AFSEL_R &= ~0x0F;     								// disable alt funct on Port M pins (PM0-PM3)
  GPIO_PORTM_DEN_R |= 0x0F;        								// enable digital I/O on Port M pins (PM0-PM3)
																									// configure Port M as GPIO
  GPIO_PORTM_AMSEL_R &= ~0x0F;     								// disable analog functionality on Port M	pins (PM0-PM3)	
	return;
}


void PortN_Init(void){
	//Use PortN onboard LED	
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;				// activate clock for Port N
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R12) == 0){};	// allow time for clock to stabilize
	GPIO_PORTN_DIR_R |= 0x0B;        								// make PN0 & PN1 & PN3 out (PN0 built-in LED D2, PN1 LED D1, PN3 For Bus check)
	GPIO_PORTN_DIR_R &= 0xFFFB;        							// make PN2 input 
  GPIO_PORTN_AFSEL_R &= ~0x0F;     								// disable alt funct on PN0:PN3
  GPIO_PORTN_DEN_R |= 0x0F;        								// enable digital I/O on PN0:PN3
																									// configure PN0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF0F)+0x00000000;
  GPIO_PORTN_AMSEL_R &= ~0x0F;     								// disable analog functionality on PN0:PN3		
	
	GPIO_PORTN_DATA_R ^= 0b00000001; 								//hello world!
	SysTick_Wait10ms(10);														//.1s delay
	GPIO_PORTN_DATA_R ^= 0b00000001;	
	return;
}


void blinkLED2()                                   //Measurement status LED (PN0 D2)
{
	GPIO_PORTN_DATA_R |= 0b00000001; 								//ON
	SysTick_Wait10ms(10);														//.1s delay
	GPIO_PORTN_DATA_R ^= 0b00000001;                //OFF
	
	//   BUS FREQ CONFIRMATION CODE ----> UNCOMMENT TO TEST 
	GPIO_PORTN_DATA_R |= 0b00001000; 								//ON  PN3
	//1/Bus = 1/30MHz = 33.33 ns --> AD2 cant pick up on this so we can amplify it up to ms 
	SysTick_Wait(1000000);													//33.33nsec period * 1,000,000 = 33.33 ms delay 
	GPIO_PORTN_DATA_R ^= 0b00001000;                //OFF	PN3
	//
}
void ScanLED()           //Aditional Status LED (D1)
{
	GPIO_PORTN_DATA_R |= 0b00000010; 								//ON
	SysTick_Wait10ms(10);														//.1s delay
	GPIO_PORTN_DATA_R ^= 0b00000010;                //OFF
}

void step(int num)
{
	uint32_t delay = 1;
	if (num == 1)
	{GPIO_PORTM_DATA_R = 0b00000011;}
	else if (num == 2)
	{GPIO_PORTM_DATA_R = 0b00000110;}
	else if (num == 3)
	{GPIO_PORTM_DATA_R = 0b00001100;}
	else if (num == 4)
	{GPIO_PORTM_DATA_R = 0b00001001;}
	SysTick_Wait10ms(delay);
}


int rotate( int steps) {
	int num = 1;
	for (int i = 0; i < steps; i++)                       // 64 steps for one 11.25deg turn 
	{
		if((GPIO_PORTN_DATA_R&0b00000100) == 0)            // checks if there was a button press to stop process
		{
			SysTick_Wait10ms(100);                           //debounce button
			return 1;
		}
		else                                               //proceed with a single step 
		{
			step(num);
			num == 4 ? num = 1: num++;                     // CW: if num has reached 4 (max step loop), then set it back to 1
		}
	}
	blinkLED2();
	SysTick_Wait10ms(10);
	
	return 0;
}

void rotateback( int steps) {
	int num;
	
	num = 4;
	for (int i = 0; i < steps; i++)   //  64steps = 11.25deg turn 
	{
		step(num);
		num == 1 ? num = 4: num--;                     // CCW: if num has reached 1 (max step loop), then set it back to 4 
	}
}

//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}


//*********************************************************************************************************
//*********************************************************************************************************
//***********					MAIN Function				*****************************************************************
//*********************************************************************************************************
//*********************************************************************************************************
uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
int status=0;

int main(void) {
  uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  uint8_t RangeStatus;
  uint8_t dataReady;

	//initialize
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	PortM_Init();
	PortN_Init();	
	uint32_t delay = 1;
	int iteration = 32;
	int steps = 64;
	int input = 0;
	int flag = 0;
	
	
	// hello world!
	UART_printf("Program Begins\r\n");
	int mynumber = 1;
	sprintf(printf_buffer,"2DX ToF Program Studio Code %d\r\n",mynumber);
	UART_printf(printf_buffer);


/* Those basic I2C read functions can be used to check your own I2C functions */
	status = VL53L1X_GetSensorId(dev, &wordData);

	sprintf(printf_buffer,"(Model_ID, Module_Type)=0x%x\r\n",wordData);
	UART_printf(printf_buffer);

	// 1 Wait for device booted
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
  }
	FlashAllLEDs();
	UART_printf("ToF Chip Booted!\r\n Please Wait...\r\n");
	
	status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
	
  /* 2 Initialize the sensor with the default setting  */
  status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);
	
	while (1)                                       //main loop
	{
		if ((GPIO_PORTN_DATA_R&0b00000100) == 0)
		{
			SysTick_Wait10ms(100);                    //wait a little
			ScanLED();              //scan LED
			status = VL53L1X_StartRanging(dev) ;   // This function has to be called to enable the ranging
			for (int i = 0; i < iteration; i++)             //loop 32 times for 32*11.25deg = 360 full rotation
			{ 
				flag = rotate(steps);							         // Call function spin for 1 11.25deg rotation	
				if (flag == 1)                            //checks if button was pressed when rotating to stop 
				{break;}	
			
				//data collection
				while (dataReady == 0){
					status = VL53L1X_CheckForDataReady(dev, &dataReady);
							VL53L1_WaitMs(dev, 5);
				}
				dataReady = 0;
				
				//7 read the data values from ToF sensor
				status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
				status = VL53L1X_GetDistance(dev, &Distance) ;					//The Measured Distance value

				status = VL53L1X_ClearInterrupt(dev); /* 8 clear interrupt has to be called to enable next interrupt*/
				
				sprintf(printf_buffer, "%u\r\n", Distance);
				UART_printf(printf_buffer);
				SysTick_Wait10ms(20);
			}
			VL53L1X_StopRanging(dev);
			for (int j = 0; j < iteration; j++)
			{
				rotateback(steps);
			}
		}	
	}
	return 0;
}

