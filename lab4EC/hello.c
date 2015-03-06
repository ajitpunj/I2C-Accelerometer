//Ajit Punj and Baban Malhi
//997210013 and ID

//#include <stdint.h>
//#include <stdbool.h>
//#include "inc/hw_memmap.h"
//#include "inc/hw_types.h"
//#include "inc/hw_gpio.h"
//#include "inc/hw_ints.h"
//#include "inc/hw_nvic.h"
//#include "driverlib/debug.h"
//#include "driverlib/fpu.h"
//#include "driverlib/gpio.h"
//#include "driverlib/pin_map.h"
//#include "driverlib/rom.h"
//#include "driverlib/sysctl.h"
//#include "driverlib/interrupt.h"
//#include "driverlib/uart.h"
//#include "utils/uartstdio.h"

//#include "inc/tm4c123gh6pm.h"
//#include "driverlib/timer.h"
//#include "driverlib/systick.h"

// global integers for edge 1 and edge 2, bit counter, pulse width, and array position
//second declaration is for the iTick flag which keeps a track of the first pulse to hit the interrupt handler
// last declaration is for the array of bit values which is of size 32
volatile int edge1, edge2, counter, pulse, arrayVals;
volatile int iTick;
volatile int arrayValues[32];

#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
// Configure the UART and its pins.  This must be called before UARTprintf().
//***************************************************************************
void
ConfigureUART(void)
{

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); // Enable the GPIO Peripheral used by the UART.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0); // Enable UART0
    GPIOPinConfigure(GPIO_PA0_U0RX); // Configure GPIO Pins for UART mode.
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC); // Use the internal 16MHz oscillator as the UART clock source.
    UARTStdioConfig(0, 115200, 16000000);  // Initialize the UART for console I/O.
}


int main(void){
	
}

//*****************************************************************************
// Systick Interrupt handler on falling edge of PB5, also declared as extern in bitvector
// startup_rvmdk.S file, and declared as DCD SysTick_Handler for port B in the same file
//*****************************************************************************

/* TODO: USE THIS CODE FOR PART2
void SysTick_Handler (void) {
 
    //clear interrupts
    GPIOIntClear(GPIO_PORTB_BASE, GPIO_INT_PIN_5);
	counter++;
    //first fallin edge of the first pulse hit since iTick == 0
	if(iTick==0){
		//reset systick, set edge1, set iTick=1, then edge2 gets set at the next falling edge
		SysTickPeriodSet (16777000);
		NVIC_ST_CURRENT_R=0;
		edge1=SysTickValueGet();
		iTick=1;
	}
    //every pulse after the first is handled here
	else if(iTick==1){
		//get edge2 value and store it before re-clearing systick and getting the cleared value again for the first edge of the next pulse
		edge2=SysTickValueGet();
        //since both edge values are obtained, calculate the pulse width
		pulse=edge1-edge2;
        //store the pulse width raw value to the array, and increment the array position tracker
		arrayValues[arrayVals]=pulse;
		arrayVals++;
		//reset systick and get edge1, on next falling edge it will get edge2 and calculate from here to next falling edge
		SysTickPeriodSet (16777000);
		NVIC_ST_CURRENT_R=0;
		edge1=SysTickValueGet();
	}
	
} 


int main(void)
{
        ConfigureUART();
        //set clock frequency to 80 MHz
        SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
    
        //enable ports F and B for LEDs and interrupt port
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);    
        //configure PB5 as input 
        GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_5);
    
        //enable LED outputs
        GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
    
        //configure interrupt type for PB5 (falling edge)
		GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_FALLING_EDGE);
        //enable interrupts after configuring
		GPIOIntEnable(GPIO_PORTB_BASE, GPIO_INT_PIN_5);
		IntEnable(INT_GPIOB);
		IntMasterEnable();
        //initialize edge global variables and turn on the red LED 
		edge1=0;
		edge2=0;
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_1);//red led
        int bitValues[32];
        int pulseWidth=0;
        int arrayPos=0;
        //set the systick period to close to the max value and enable it
		SysTickPeriodSet (16777000);
		SysTickEnable();
		int resetInt=0;
		iTick=0;
		arrayVals=0;

		while(1)
        {
                //main infinite while loop, will repeat and wait for the 32nd bit to be recorded in the interrupt handler
				if(arrayVals==32){
                    //clear interrupts and disable interrupts while printing
					GPIOIntClear(GPIO_PORTB_BASE, GPIO_INT_PIN_5);
					GPIOIntDisable(GPIO_PORTB_BASE, GPIO_INT_PIN_5);
                    //loop through bit values
					int loopcount;
					for(loopcount=0;loopcount<32;loopcount++){
							//take the current raw pulse width value, and decide if it is a 1 or 0
							int tempbitval=arrayValues[loopcount];
                            //if pulse time is in the range 178000<width<180000, it is a 1 since 180000/80000000=2.25 ms which is the recorded period for a 1 from the logic analyzer
							if(tempbitval>178000 && tempbitval<180000)
									arrayValues[loopcount]=1;
                            //likewise, if pulse time is in the range 87000<width<89000 it is a 0 since 89000/80000000=1.1125 which is the recorded period for a 0 from the logic analyzer
							else if(tempbitval>87000 && tempbitval<89000)
									arrayValues[loopcount]=0;
					}
                    //calculate the binary value of the last 8 bits from the array in a decimal representation by taking the value from the array
                    //multiplying remoteVal by 10 and then adding the array value to remoteVal
					int remoteVal=0;
					for(loopcount=24;loopcount<32;loopcount++){
						remoteVal*=10;
						remoteVal+=arrayValues[loopcount];
						
					}
					UARTprintf("remote val=%d\n",remoteVal);
                    //remoteVal is a 8 bit number represented in decimal (so beginning 0's omitted)
                    //which represents the last 8 bits of the pattern, minus the final bit (so bits 23:30)
                    //and map these using a switch case to the recorded logic analyzer values
					switch(remoteVal){
						case 1101111: //0000 1101 1111 last 12 bits of "1", so this takes 01101111 (first 0 omitted)
							UARTprintf("1\n");
							break;
						case 101111:
							UARTprintf("2\n");
							break;
						case 1001111:
							UARTprintf("3\n");
							break;
						case 1110111:
							UARTprintf("4\n");
							break;
						case 110111:
							UARTprintf("5\n");
							break;
						case 1010111:
							UARTprintf("6\n");
							break;
						case 1100111:
							UARTprintf("7\n");
							break;
						case 100111:
							UARTprintf("8\n");
							break;
						case 1000111:
							UARTprintf("9\n");
							break;
						case 111011:
							UARTprintf("0\n");
							break;
                        case 111100:
							UARTprintf("Down\n");
							break;
                        case 1111100:
							UARTprintf("Up\n");
							break;
                        case 101100:
							UARTprintf("Left\n");
							break;
                        case 1011100:
							UARTprintf("Right\n");
							break;
                        case 111:
							UARTprintf("Mute\n");
							break;
						default://default case when an error is found due to invalid button pressed or noise detected
							UARTprintf("error, try again\n");
							break;
					}
                    //reset flags and necessary variables for next measurement
					arrayPos=0;
					arrayVals=0;
					iTick=0;
                    //1/3 second delay before enabling interrupt again to remove duplicate button presses
					SysCtlDelay(SysCtlClockGet()/3/3);
                    //clear interrupts and enable again to calculate next value
					GPIOIntClear(GPIO_PORTB_BASE, GPIO_INT_PIN_5);
					GPIOIntEnable(GPIO_PORTB_BASE, GPIO_INT_PIN_5);
				}
			
   
    }
}

*/


