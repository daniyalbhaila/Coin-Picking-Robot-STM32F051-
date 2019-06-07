/*
* Project 2, Elec 291 University of British Columbia Electrical and Computer Engineering
* Professor Dr. Jes�s Calvi�o-Fraga.
* 
* Introduction:
* Design, build, program, and test robot that detects and picks coins.  
* The robot must operate within a predefined perimeter defined by a wire carrying an AC current.
*  
* Processor: ST Microelectronics, ARM, STM32F051 
*
* Group members: 
* Daniyal Bhaila, Erick Chmelyk, Mark Gnocato, Braedon Norman, Jian Yong (Alex) Wang,Timothy Wriglesworth.  
*
* This project does not use the STM32CubeMX code generator tool. 
*/

#include "stm32f05xxx.h"
#include <stdio.h>
#include <stdlib.h>
#include "serial.h"

#define SYSCLK 48000000L
#define DEF_F 100000L // To achieve a tick of 10 us
#define F_CPU 48000000L
//#define THRESHOLD_FREQ 58600  //**
#define THRESHOLD_VOLT 0.55

#define PIN_PERIOD (GPIOA_IDR&BIT14)

int THRESHOLD_FREQ= 500000; //way to big to start to stop false detects
int counter=0;

//function prototypes
void MotorLeftTurn(void);
void metal_detect(void);
int egets(char *s, int Max);
long int GetPeriod (int n);
void delay_ms (int msecs);
void AutoCalib(void);

//ISR variables
volatile int ISR_pw=0, ISR_cnt=0, ISR_frc;

volatile int flag=0;



void AutoCalib(void)
{
 float period;
 
    //should turnLED 
 period= GetPeriod(100);
 period=period/(F_CPU*100.0);	
 delay_ms(1000); //Delay for LED
 THRESHOLD_FREQ= (1/period)+ 200;  //set threshold
 
 
 //pin 27 PB4 for LED
 
}

// The following ISR happens at a rate of 100kHz.  It is used
// to generate the standard hobby servo 50Hz signal with a pulse
// width of 0.6ms to 2.4ms.
void Timer1ISR(void) 
{
	TIM1_SR &= ~BIT0; // clear update interrupt flag
	ISR_cnt++;

	if(flag)
	{
		if(ISR_cnt<ISR_pw)
		{
			GPIOA_ODR |= BIT0; // PA0=1
		}
		else
		{
			GPIOA_ODR &= ~BIT0; // PA0=0
		
		}
	
	}
	
	else
	{
		if(ISR_cnt<ISR_pw)
		{
			GPIOA_ODR |= BIT1; // PA1=1
		}
		else
		{
			GPIOA_ODR &= ~BIT1; // PA1=0
		
		}
	}
	
	
	
	if(ISR_cnt>=2000)
	{
		ISR_cnt=0; // 2000 * 10us=20ms
		ISR_frc++;
	}
	
	
}	

void delay(int dly)
{
	while( dly--);
}


void SysInit(void)
{
	// Set up output port bit for blinking LED
	RCC_AHBENR |= 0x00020000;  // peripheral clock enable for port A
	GPIOA_MODER |= 0x00000001; // Make pin PA0 output
	GPIOA_MODER &= ~(BIT3); //PA1 is output
	GPIOA_MODER |= BIT2;
	
	//Metal Detector
	RCC_AHBENR |= 0x00020000; // peripheral clock enable for port A
	// Set up Metal Detector Pins
	GPIOA_MODER &= ~(BIT28 | BIT29); // Make pin PA14 input
	
	// Activate pull up for pin PA14:
	GPIOA_PUPDR |= BIT28; 
	GPIOA_PUPDR &= ~(BIT29);
	
	//miniServos
	GPIOA_MODER &= ~(BIT17); //PA1 is output
	GPIOA_MODER |= BIT16;
	
	GPIOB_MODER &= ~(BIT3); //Pin 15 ie red LED
	GPIOB_MODER |= BIT2;
	
	/*
	* set up motor pins
	*/
	GPIOA_MODER &= ~(BIT11); //PA5
	GPIOA_MODER |= BIT10;
	GPIOA_MODER &= ~(BIT5); //PA2
	GPIOA_MODER |= BIT4;
	
	GPIOA_MODER &= ~(BIT7); //PA3  pin9
	GPIOA_MODER |= BIT6;
	GPIOA_MODER &= ~(BIT9); //PA4  pin10
	GPIOA_MODER |= BIT8;
	
	
	
	//LED pin PA7 pin 13 for config pin
	GPIOA_MODER &= ~(BIT14); 	
	GPIOA_MODER |= BIT15;
	
	
	
	// Set up timer
	RCC_APB2ENR |= BIT11; // turn on clock for timer1
	TIM1_ARR = SYSCLK/DEF_F;  //timer counts from this vlaue to 0 (downcounting): from the data sheet
	ISER |= BIT13;        // enable timer interrupts in the NVIC
	TIM1_CR1 |= BIT4;     // Downcounting    
	TIM1_CR1 |= BIT0;     // enable counting    
	TIM1_DIER |= BIT0;    // enable update event (reload event) interrupt  
	enable_interrupts();
}

void delay_ms (int msecs)
{	
	int ticks;
	ISR_frc=0;
	ticks=msecs/20;
	while(ISR_frc<ticks);
}


void Servo1(int pow1)
{
 ISR_pw=pow1;
}

void Servo2(int pow2)
{
	ISR_pw=pow2;

}

void SlowServo1(int start, int pow1, int speed)
{
	while(1)
	{
		if(start<pow1){
		start += speed;
		}
		if(start>pow1){
		start -= speed;
		}
		if(abs(start - pow1)<speed){
			ISR_pw=pow1;
			break;
		}
		ISR_pw=start;
		delay_ms(35);
	}
}

void SlowServo2(int start, int pow2, int speed)
{
	while(1)
	{
		if(start<pow2){
		start += speed;
		}
		if(start>pow2){
		start -= speed;
		}
		if(abs(start - pow2)<speed){
			ISR_pw=pow2;
			break;
		}
		ISR_pw=start;
		delay_ms(35);
	}
}

/*
* H-Bridge desired motor direction Control operations
*/


void MotorBackward(void)
{
       //MoveBackward
		GPIOA_ODR |= BIT5; // PA5=1
		GPIOA_ODR |= BIT3; // PA2=1
		GPIOA_ODR &= ~(BIT2); // PAx=0
		GPIOA_ODR &= ~(BIT4); // PAx=0
}



void MotorForward(void)
{
 //MoveForward
		GPIOA_ODR &= ~BIT5; // PA5=1
		GPIOA_ODR &= ~BIT3; // PA2=1
		GPIOA_ODR |= (BIT2); // PAx=0
		GPIOA_ODR |= (BIT4); // PAx=0

}


void MotorKill(void)
{
 //turn wheels off
		GPIOA_ODR &= ~BIT2; // PA2=1
		GPIOA_ODR &= ~BIT4; // PA2=1
		GPIOA_ODR &= ~(BIT3); // PAx=0
		GPIOA_ODR &= ~(BIT5); // PA5=0

}


/*
* Allow robot to detect coins while turning. 
*/

void MotorLeftTurnSlow(int delay_time, int cycles) {
	while(cycles--){    
	MotorLeftTurn();
	metal_detect();
	delay_ms(delay_time);
	metal_detect();
	}
}

/*
* upon perimeter detection robot must back away from perimiter and
* turn around ~120 degrees. Also while turning robot will still be detecting coins.
*
*/

void TurnAround(void)
{
	MotorKill();
	delay_ms(50);
	MotorBackward();
	delay_ms(800); // move back for xxx time
	MotorLeftTurnSlow(50,20);    //~120 degrees
	MotorKill();
	delay_ms(100);

}
void MotorLeftTurn(void)
{
		GPIOA_ODR |= BIT5; // PA5=1
		GPIOA_ODR &= ~BIT3; // PA2=1
		GPIOA_ODR &= ~(BIT2); // PAx=0
		GPIOA_ODR |= (BIT4); // PAx=0
}
/*
* period measurement from sensor
*/

void wait_1ms(void)
{
	// For SysTick info check the STM32F0xxx Cortex-M0 programming manual page 85.
	STK_RVR = (F_CPU/1000L) - 1;  // set reload register, counter rolls over from zero, hence -1
	STK_CVR = 0; // load the SysTick counter
	STK_CSR = 0x05; // Bit 0: ENABLE, BIT 1: TICKINT, BIT 2:CLKSOURCE
	while((STK_CSR & BIT16)==0); // Bit 16 is the COUNTFLAG.  True when counter rolls over from zero.
	STK_CSR = 0x00; // Disable Systick counter
}

void waitms(int len)
{
	while(len--) wait_1ms();
}

long int GetPeriod (int n)
{
	int i;
	unsigned int saved_TCNT1a, saved_TCNT1b;
	
	STK_RVR = 0xffffff;  // 24-bit counter set to check for signal present
	STK_CVR = 0xffffff; // load the SysTick counter
	STK_CSR = 0x05; // Bit 0: ENABLE, BIT 1: TICKINT, BIT 2:CLKSOURCE
	while (PIN_PERIOD!=0) // Wait for square wave to be 0
	{
		if(STK_CSR & BIT28) return 0;
	}
	STK_CSR = 0x00; // Disable Systick counter

	STK_RVR = 0xffffff;  // 24-bit counter set to check for signal present
	STK_CVR = 0xffffff; // load the SysTick counter
	STK_CSR = 0x05; // Bit 0: ENABLE, BIT 1: TICKINT, BIT 2:CLKSOURCE
	while (PIN_PERIOD==0) // Wait for square wave to be 1
	{
		if(STK_CSR & BIT28) return 0;
	}
	STK_CSR = 0x00; // Disable Systick counter
	
	STK_RVR = 0xffffff;  // 24-bit counter reset
	STK_CVR = 0xffffff; // load the SysTick counter to initial value
	STK_CSR = 0x05; // Bit 0: ENABLE, BIT 1: TICKINT, BIT 2:CLKSOURCE
	for(i=0; i<n; i++) // Measure the time of 'n' periods
	{
		while (PIN_PERIOD!=0) // Wait for square wave to be 0
		{
			if(STK_CSR & BIT28) return 0;
		}
		while (PIN_PERIOD==0) // Wait for square wave to be 1
		{
			if(STK_CSR & BIT28) return 0;
		}
	}
	STK_CSR = 0x00; // Disable Systick counter

	return 0xffffff-STK_CVR;
}

int readADC(void)
{
	ADC_CR |=  BIT2;            // Trigger a conversion
	while ( (ADC_CR & BIT2) );  // Wait for End of Conversion
	return ADC_DR;              // ADC_DR has the 12 bits out of the ADC
}

/*
* Time varying current produces a magnetic field that induces a voltage spike across the sensor circuitry.
* if Voltage goes above threshold turn around.
*/

void PerimeterDetection(void)
{
	float j, a;
	
	j=readADC();
	a=(j*3.3)/0x1000;
	if (a > THRESHOLD_VOLT)
	{
		TurnAround(); 
	}
}

void initADC(void)
{
	RCC_AHBENR |= BIT18;        // Turn on GPIOB
	RCC_APB2ENR |= BIT9;        // Turn on ADC 
	GPIOB_MODER |= (BIT2+BIT3); // Select analog mode for PB1 (pin 15 of LQFP32 package)
	ADC_CR |= BIT31;            // Begin ADCCalibration
	while ((ADC_CR & BIT31));   // Wait for calibration complete
	ADC_SMPR=7;                 // Long sampling time for more stable measurements
	//ADC_CHSELR |= BIT17;      // Select Channel 17, internal reference
	ADC_CHSELR |= BIT9;         // Select Channel 9
	ADC_CCR |= BIT22;	        // Enable the reference voltage
	ADC_CR |= BIT0;             // Enable the ADC
}


/*
*  after coin detected- position correctly, then pick up the coin
*
*/

void electromagnet(void)
{
		MotorKill();
		delay_ms(200);
		MotorBackward();
		delay_ms(200); // back distance
		MotorKill();
		delay_ms(200);
		flag =1;	
		Servo1(58);  //starting position: Pin 6 is for bottom servo
		delay_ms(200);
		flag=0;      
		Servo2(55);  //starting position for Pin 7. Arm Servo
		delay_ms(200);
		
		flag =1;
		Servo1(137);  //move to sweep position
		delay_ms(200); 
		flag=0;
	    delay_ms(200);
		Servo2(220);  //move e-mag to floor
		delay_ms(200); 

		GPIOA_ODR |= BIT8;   // pin18. Turn e-magnet on  
		flag =1;	
	
		SlowServo1(137,235,7);
		delay_ms(200);
		flag=0;
	
		SlowServo2(220,90,5);
		delay_ms(200);	
		flag =1;
		
		SlowServo1(235,112,5);
		delay_ms(200);
		flag =0;
		SlowServo2(90,105,5);
		GPIOA_ODR &= ~BIT8;   // pin18: turn e-Mag off
		delay_ms(200);
		Servo2(70);
		delay_ms(200);
		
		flag =1;	
		Servo1(58);  //starting position: Pin 6 is for bottom servo
		delay_ms(100);
		flag=0;      
		Servo2(55);  //starting position for Pin 7. Arm Servo
		delay_ms(500);
		
}


/*
* tank circuit sensor frequency characteristic will change upon detection of a coin
* if change is large-> Metal detected!
*/

void metal_detect(void){
	long int count;
	float T,f;

	count=GetPeriod(100);
	if(count>0)
	{
		T=count/(F_CPU*100.0);
		f=1/T;

		if(f>THRESHOLD_FREQ){
			
			GPIOA_ODR &= ~BIT7;
			electromagnet();
			GPIOA_ODR |= BIT7;
			counter++;
			// stop robot, pick up coin
		}
	}
	// otherwise return to main, continue moving

}



/*
* Behaviour definition:
*
* Robot moves forward until detection of coin: where it stops, picks up coin and continues.
* Upon detection of perimeter it will stop, backup and turn around. Robot will also detect coins while
* Turning. It will pick up the coin, then finish it's turn, and continue forward. 
*
*/


int main(void)
{
//intialize/configure microcontroller to operation mode
	int LED_light=10;
	SysInit();
	initADC();
   GPIOA_ODR &= ~BIT7;
   AutoCalib();
   delay_ms(50);
	GPIOA_ODR |= BIT7; 	
	
	
	//GPIOA_ODR |= BIT8; // pin18. Turn e-magnet off
	delay_ms(200);
		flag =1;	
		Servo1(58);  //starting position: Pin 6 is for bottom servo
		delay_ms(50);
		flag=0;      
		Servo2(55);  //starting position for Pin 7. Arm Servo
		delay_ms(50);
		
		
		
	

	while(counter<20)
	{
		
	//Program Control	
		MotorForward();
		delay_ms(50);
		metal_detect();
		delay_ms(50);
		PerimeterDetection();
		delay_ms(50);
			
	}
	
		flag =1;	
		Servo1(58);  //starting position: Pin 6 is for bottom servo
		delay_ms(200);
		flag=0;      
		Servo2(55);  //starting position for Pin 7. Arm Servo
		delay_ms(200);
	
		while(LED_light<10)
		{
		LED_light--;
		
		GPIOA_ODR |= BIT7; // PA8=1
		delay_ms(300);
		GPIOA_ODR &= ~(BIT7); // PA8=0
		delay_ms(300);
		
		}

			delay_ms(1000000);
}
