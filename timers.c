/*
 * SPM Controller Code
 *
 * Author: Chris de Torres
 *
 * Program Description:
 * This code was made to interface three RE-MAX29 motors for a Spherical Parallel Manipulator (SPM).
 *
 * The code uses a TM4C123G Tiva C Evaluation Board to control the three motors. Simulink is
 * to provide the reference encoder pulse position of the motors. The controller uses a discrete PI
 * control law running at the frequency of CONTROL_FREQ. Hardware interrupt ISR are used to keep
 * track of the encoder pulses.
 *
 * The main loop of the controller is used for communication between the controller and the host
 * program. The host program sends three 4-digit numbers in one ASCII string that represents the
 * reference position of the motors in encoder pulses. The program returns the current position
 * of the motors as well as the current reference positions.
 *
 * Input string to the controller
 * "#### #### ####\r"
 *   1    2    3
 *
 * 1 - motor A reference position
 * 2 - motor B reference position
 * 3 - motor C reference position
 *
 * Output string to the controller
 * "#### #### #### #### #### ####\n\r"
 *   1    2    3    4    5    6
 *
 * 1 - motor A current position
 * 2 - motor B current position
 * 3 - motor C current position
 * 4 - motor A reference position
 * 5 - motor B reference position
 * 6 - motor C reference position

Microcontroller Connection Pinouts
------------
 Motor A
------------
 PWM0 - PB7
 PWM1 - PB6

 QEIA - PD7
 QEIB - PD6

------------
 Motor B
------------
 PWM0 - PB4
 PWM1 - PB5

 QEIA - PC5
 QEIB - PC6

------------
 Motor C
------------
 PWM0 - PE4
 PWM1 - PE5

 QEIA - PA6
 QEIB - PA5

 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/qei.h"
#include "utils/uartstdio.h"

#define PWM_FREQUENCY 18000
#define CONTROL_FREQ 4000

#define MAX_PULSE 500

// PID constants
#define Kp 3.0
#define Kd 0.0
#define Ki 0.0

#define MAX_ERROR 1000.0
#define MID_POS 5000
#define MAX_POS 9999
#define UART_SPEED 921600

// global variables for the setpoints and current position
volatile int32_t A_ref = MID_POS;
volatile int32_t B_ref = MID_POS;
volatile int32_t C_ref = MID_POS;

volatile int32_t aQEI_count = MID_POS;
volatile int32_t bQEI_count = MID_POS;
volatile int32_t cQEI_count = MID_POS;

volatile uint32_t ui32Load;
volatile float I_c_prev = 0;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// The interrupt handler for Motor A quadrature encoder
//
//*****************************************************************************
void qeiDIntHandler(void){
	static int8_t lookup_table[] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};
	static uint8_t last_state = 0;
	volatile uint8_t status=0;
	status = GPIOIntStatus(GPIO_PORTD_BASE,true);
	GPIOIntClear(GPIO_PORTD_BASE,status);

	uint8_t current_state = GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_7 | GPIO_PIN_6) >> 6;
	last_state = last_state << 2 | current_state;

	aQEI_count = aQEI_count + lookup_table[last_state & 0b1111];
//	aQEI_count = aQEI_count % (MAX_POS + 1);
}

//*****************************************************************************
//
// The interrupt handler for Motor B quadrature encoder
//
//*****************************************************************************
void qeiCIntHandler(void){
	static int8_t lookup_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
	static uint8_t last_state = 0;
	volatile uint8_t status = GPIOIntStatus(GPIO_PORTC_BASE,true);
	GPIOIntClear(GPIO_PORTC_BASE,status);

	uint8_t current_state = GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_6 | GPIO_PIN_5) >> 5;
	last_state = last_state << 2 | current_state;

	bQEI_count = bQEI_count + lookup_table[last_state & 0b1111];
//	bQEI_count = bQEI_count % (MAX_POS + 1);
}

//*****************************************************************************
//
// The interrupt handler for Motor C quadrature encoder
//
//*****************************************************************************
void qeiAIntHandler(void){
	static int8_t lookup_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
	static uint8_t last_state = 0;
	volatile uint8_t status = GPIOIntStatus(GPIO_PORTA_BASE,true);
	GPIOIntClear(GPIO_PORTA_BASE,status);

	uint8_t current_state = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_5) >> 5;
	last_state = (last_state << 2) | current_state;

	cQEI_count = cQEI_count + lookup_table[last_state & 0b1111];
//	cQEI_count = cQEI_count % (MAX_POS + 1);
}

//*****************************************************************************
//
// Timed control ISR
// TODO: Write a Description of the ISR
//
//*****************************************************************************
void ControlIntHandler(void){
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

	volatile float a_error, b_error, c_error;
	volatile uint32_t Load = ui32Load;
	volatile float P_a, P_b, P_c; // Proportional terms
	volatile float D_a, D_b, D_c; // Derivative terms
	volatile float I_a, I_b, I_c; // Integral terms
	volatile float a_duty, b_duty, c_duty; // output of PID
	volatile float a_mag, b_mag, c_mag;
	volatile uint32_t load_A, load_B, load_C;
	static float Ti = 1.0/CONTROL_FREQ;
	static float MAX_ERROR_INV = 1.0/MAX_ERROR;

	// Update all the PID parameters
    volatile float acurr = aQEI_count;
    volatile float bcurr = bQEI_count;
    volatile float ccurr = cQEI_count;

    // Calculate the PID values (PWM strength)
    // TODO: Complete the PID section. Make the discrete version of this.
    a_error = A_ref - aQEI_count;
    if(a_error >= MAX_PULSE) a_error = MAX_PULSE;
    if(a_error <= -MAX_PULSE) a_error = -MAX_PULSE;
    P_a = Kp * a_error;

    b_error = B_ref - bQEI_count;
    if(b_error >= MAX_PULSE) b_error = MAX_PULSE;
    if(b_error <= -MAX_PULSE) b_error = -MAX_PULSE;
    P_b = Kp * b_error;

    // C Motor Calculation
    c_error = C_ref - cQEI_count;
    if(c_error >= MAX_PULSE) c_error = MAX_PULSE;
    if(c_error <= -MAX_PULSE) c_error = -MAX_PULSE;
    P_c = Kp * c_error;

//    if(I_c <= MAX_ERROR && I_c >= -MAX_ERROR){
//    	I_c = Ki * c_error * Ti + I_c_prev;
//    	I_c_prev = I_c;
//    }
//    else{
//    	I_c = 0;
//    	I_c_prev = 0;
//    }
//
//    if(I_c > MAX_ERROR) I_c = MAX_ERROR;
//    if(I_c < -MAX_ERROR)I_c = -MAX_ERROR;

    a_duty = P_a * MAX_ERROR_INV;
    b_duty = P_b * MAX_ERROR_INV;
    c_duty = P_c * MAX_ERROR_INV;

    // Make an absolute value
    if(a_duty >= 0) a_mag = a_duty;
    else a_mag = -1.0 * a_duty;

    if(b_duty >= 0) b_mag = b_duty;
    else b_mag = -1.0* b_duty;

    if(c_duty >= 0) c_mag = c_duty;
    else c_mag = -1.0 * c_duty;

    load_A = a_mag * ui32Load;
    load_B = b_mag * ui32Load;
    load_C = c_mag * ui32Load;

    // Set the PID values to the PWM
	if (a_duty > 0){
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, load_A); 						// PB6
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 0); 			// PB7
	}
	else{
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 0); 			// PB6
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, load_A); 	// PB7
	}

	if(b_duty >= 0){
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0); // PB4
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, load_B); // PB5
	}
	else{
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, load_B); // PB4
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, 0); // PB5
	}

	if(c_duty >= 0){
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, 0); // PE4
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, load_C); // PE5
	}
	else{
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, load_C); // PE4
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, 0); // PE5
	}
}// end control

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void ConfigureUART(void){
	// Setup for UART module in the microcontroller
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    // Assign port A pin 0, pin 1 as the receive and transmit pin for communication
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Setup the clock source of UART
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    // Setup the baud rate of UART communication
    UARTStdioConfig(0, UART_SPEED, 16000000);
}


//*****************************************************************************
//
// Configure the PWM modules and PWM outputs.
//
//*****************************************************************************

void ConfigurePWM(void){
	volatile uint32_t ui32PWMClock;
	volatile uint32_t pulse_per_period;

	SysCtlPWMClockSet(SYSCTL_PWMDIV_1);			// Initialize the PWM clock
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

	GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);    //Configure PB6 as PWM 0
	GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4|GPIO_PIN_5);    //Configure PE4 as PWM 4

	GPIOPinConfigure(GPIO_PB6_M0PWM0);
	GPIOPinConfigure(GPIO_PB7_M0PWM1);
	GPIOPinConfigure(GPIO_PB4_M0PWM2);
	GPIOPinConfigure(GPIO_PB5_M0PWM3);
	GPIOPinConfigure(GPIO_PE4_M0PWM4);
	GPIOPinConfigure(GPIO_PE5_M0PWM5);

	PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT|PWM_OUT_1_BIT|PWM_OUT_2_BIT|
							  PWM_OUT_3_BIT|PWM_OUT_4_BIT|PWM_OUT_5_BIT, true); // PWM output bits

	ui32PWMClock = SysCtlClockGet();
	pulse_per_period = ui32PWMClock / PWM_FREQUENCY;
	ui32Load = pulse_per_period - 1;					// Calculate the period

	PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN);
	PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_UP_DOWN);
	PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_UP_DOWN);

	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32Load);
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, ui32Load);
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, ui32Load);

	PWMGenEnable(PWM0_BASE, PWM_GEN_0);
	PWMGenEnable(PWM0_BASE, PWM_GEN_1);
	PWMGenEnable(PWM0_BASE, PWM_GEN_2);

	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 0); // PB6
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 0); // PB7

	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0); // PB4
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, 0); // PB5

	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, 0); // PE4 pin U3 - 12
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, 0); // PE5 pin U3 - 16
}

//*****************************************************************************
//
// Configure the QEI
//
//*****************************************************************************

void ConfigureQEI(void){
	// PD7 = NMI = Non-Maskable Interrupt = Unlock GPIO Commit Control Register
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
	HWREG(GPIO_PORTD_BASE + GPIO_O_AFSEL) &= ~0x80;
	HWREG(GPIO_PORTD_BASE + GPIO_O_DEN) |= 0x80;
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

	// Motor A setup
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_7); // Configure Phase B to PC
	GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_6);
	GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_7 | GPIO_PIN_6, GPIO_BOTH_EDGES);
	GPIOIntRegister(GPIO_PORTD_BASE, qeiDIntHandler);
	GPIOIntEnable(GPIO_PORTD_BASE, GPIO_INT_PIN_7 | GPIO_INT_PIN_6);

	// Motor B setup
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_6); // Configure Phase B to PC
	GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_5);
	GPIOIntTypeSet(GPIO_PORTC_BASE, GPIO_PIN_6 | GPIO_PIN_5, GPIO_BOTH_EDGES);
	GPIOIntRegister(GPIO_PORTC_BASE, qeiCIntHandler);
	GPIOIntEnable(GPIO_PORTC_BASE, GPIO_INT_PIN_6 | GPIO_INT_PIN_5);

	// Motor C setup PCA - PHA | PCA - PHB
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_6); // Configure Phase B to PC
	GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_5);
	GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_5, GPIO_BOTH_EDGES);
	GPIOIntRegister(GPIO_PORTA_BASE, qeiAIntHandler);
	GPIOIntEnable(GPIO_PORTA_BASE, GPIO_INT_PIN_6 | GPIO_INT_PIN_5);
}

void ConfigureCtrlTimer(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    IntMasterEnable();
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet()/CONTROL_FREQ);
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER0_BASE, TIMER_A);
}

//*****************************************************************************
//
// Takes in a string that contains three unsigned integer values in ASCII format
// and places them in a three unsigned integer values.
//Input:
//rx - the string containing the three reference values.
//a - a pointer to an unsigned int for motor A
//b - a pointer to an unsigned int for motor B
//c - a pointer to an unsigned int for motor C
//
//*****************************************************************************
void ThreeStrtoInt(char* rx, int32_t* a, int32_t* b, int32_t* c){
	uint32_t i;
	char* strCopy = rx;
	char A_str[16], B_str[16], C_str[16];

	for(i = 0; *strCopy != ' '; i++){
		A_str[i] = *strCopy;
		strCopy++;
	}
	A_str[i] = NULL;

	strCopy++;

	for(i = 0; *strCopy != ' '; i++){
		B_str[i] = *strCopy;
		strCopy++;
	}
	B_str[i] = NULL;

	strCopy++;

	for(i = 0; *strCopy != NULL && *strCopy != '\n' && *strCopy != '\r'; i++){
		C_str[i] = *strCopy;
		strCopy++;
	}
	C_str[i] = NULL;

	*a = atoi(A_str);
	*b = atoi(B_str);
	*c = atoi(C_str);

	// A faster implementation of the conversion

}

//*****************************************************************************
//
// This example application demonstrates the use of the timers to generate
// periodic interrupts.
//
//*****************************************************************************
int main(void){
	volatile char RxString[32];
	volatile int32_t Aset = MID_POS, Bset = MID_POS, Cset = MID_POS;
	volatile int32_t acurr, bcurr, ccurr;

	FPUEnable();
	FPULazyStackingEnable();

    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    ConfigureUART();
    ConfigurePWM();
    ConfigureQEI();
    ConfigureCtrlTimer();

    // Main program loop
    while(1){
		UARTgets(RxString,32);
		ThreeStrtoInt(RxString, &Aset, &Bset, &Cset);

		A_ref=Aset;
		B_ref=Bset;
		C_ref=Cset;

		UARTprintf("%4d %4d %4d %4d %4d %4d\n",aQEI_count,Aset,bQEI_count,Bset,cQEI_count,Cset);
//		UARTprintf("%4d %4d %4d\n",aQEI_count,bQEI_count,cQEI_count);
    }// end while

}// end main
