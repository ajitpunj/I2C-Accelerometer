/*
 * Ajit Punj and Baban Malhi
 * lab4 extra credit-read accelerometer values and display the ball movement on the
 * Oled display and send positions to remote processor via Xbee.
 */

// Timing Delays
#define SSD1351_DELAYS_HWFILL	    (3)
#define SSD1351_DELAYS_HWLINE       (1)

//colors
#define	BLACK           0x0000
#define	BLUE            0x001F
#define	RED             0xF800
#define	GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define WHITE           0xFFFF

// SSD1351 Commands
#define SSD1351_CMD_SETCOLUMN 		0x15
#define SSD1351_CMD_SETROW    		0x75
#define SSD1351_CMD_WRITERAM   		0x5C
#define SSD1351_CMD_READRAM   		0x5D
#define SSD1351_CMD_SETREMAP 		0xA0
#define SSD1351_CMD_STARTLINE 		0xA1
#define SSD1351_CMD_DISPLAYOFFSET 	0xA2
#define SSD1351_CMD_DISPLAYALLOFF 	0xA4
#define SSD1351_CMD_DISPLAYALLON  	0xA5
#define SSD1351_CMD_NORMALDISPLAY 	0xA6
#define SSD1351_CMD_INVERTDISPLAY 	0xA7
#define SSD1351_CMD_FUNCTIONSELECT 	0xAB
#define SSD1351_CMD_DISPLAYOFF 		0xAE
#define SSD1351_CMD_DISPLAYON     	0xAF
#define SSD1351_CMD_PRECHARGE 		0xB1
#define SSD1351_CMD_DISPLAYENHANCE	0xB2
#define SSD1351_CMD_CLOCKDIV 		0xB3
#define SSD1351_CMD_SETVSL 		0xB4
#define SSD1351_CMD_SETGPIO 		0xB5
#define SSD1351_CMD_PRECHARGE2 		0xB6
#define SSD1351_CMD_SETGRAY 		0xB8
#define SSD1351_CMD_USELUT 		0xB9
#define SSD1351_CMD_PRECHARGELEVEL 	0xBB
#define SSD1351_CMD_VCOMH 		0xBE
#define SSD1351_CMD_CONTRASTABC		0xC1
#define SSD1351_CMD_CONTRASTMASTER	0xC7
#define SSD1351_CMD_MUXRATIO            0xCA
#define SSD1351_CMD_COMMANDLOCK         0xFD
#define SSD1351_CMD_HORIZSCROLL         0x96
#define SSD1351_CMD_STOPSCROLL          0x9E
#define SSD1351_CMD_STARTSCROLL         0x9F
#define SSD1351WIDTH 128
#define SSD1351HEIGHT 128

#define SLAVE_ADDRESS 0x4C

int WIDTH=128;
int HEIGHT=128;

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_nvic.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

#include "inc/hw_i2c.h"
#include "driverlib/i2c.h"

#include "inc/tm4c123gh6pm.h"
#include "driverlib/timer.h"
#include "driverlib/systick.h"

#include "driverlib/ssi.h"

#include "glcdfont.c"


volatile int edge1, edge2, counter, pulse, arrayVals;
volatile int iTick;

volatile bool Tx_done;
//paddle and ball globals

int ballX=128/2;
int ballY=128/2;//middle of screen
int ballXDirection=0;//ball direction either 0 (left) or 1 (right)
int ballYDirection=0;//ball direction either 0 (down) or 1 (up)
const int dx=3;
const int dy=3;

//signed int accelX, accelY, accelZ;//global ints for accelerometer position
int8_t accelX,accelY,accelZ;

volatile bool systickDone;
int Xoffset, Yoffset,Zoffset;

void
InitConsole(void)
{
    // Enable GPIO port A which is used for UART0 pins.
    // Enable UART0.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    
    // Configure the pin muxing for UART0 functions on port A0 and A1.
    // Select the alternate (UART) function for these pins.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

//ssi/spi functions
void writeCommand(int c) {
    //wait until tx fifo empty
    while(SSIBusy(SSI0_BASE))
    {
    }
    //set data command signal low
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0);
    //send 8 bit command
    SSIDataPut(SSI0_BASE, c);
    //wait until fifo empty and transmit data
    while(SSIBusy(SSI0_BASE))
    {
    }
}


void writeData(int c) {
    //set dc high
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7);
    //send 8 bit data using blocking put function
    SSIDataPut(SSI0_BASE, c);
}

void initializePins(){
    
    //
    // Set the clocking to run directly from the external crystal/oscillator.
    //
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    
    // Display the setup on the console.
    //
    UARTprintf("\nSSI ->\n CLOCK=%d\n",SysCtlClockGet());
    UARTprintf("  Mode: SPI\n");
    UARTprintf("  Data: 8-bit\n\n");
    
    // The SSI0 peripheral must be enabled for use.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    
    // For this example SSI0 is used with PortA[5:2].
    // GPIO port A needs to be enabled so these pins can be used.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    
    // Configure the pin muxing for SSI0 functions on port A2, A3, A4, and A5.
    //
    GPIOPinConfigure(GPIO_PA2_SSI0CLK);//pa2 is clock
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);//pa3 is frameslave select
    GPIOPinConfigure(GPIO_PA4_SSI0RX);//pa4 is receive
    GPIOPinConfigure(GPIO_PA5_SSI0TX);//pa5 is transmit
    
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7);//PA6=reset, PA7=DC
    
    //
    // Configure the GPIO settings for the SSI pins.  This function also gives
    // control of these pins to the SSI hardware.  Consult the data sheet to
    // see which functions are allocated per pin.
    // The pins are assigned as follows:
    //      PA5 - SSI0Tx
    //      PA4 - SSI0Rx
    //      PA3 - SSI0Fss
    //      PA2 - SSI0CLK
    //			PA6=reset
    //			PA7=DC
    //
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 |
                   GPIO_PIN_2);
    
    //
    // Configure and enable the SSI port for SPI master mode.  Use SSI0,
    // system clock supply, idle clock level low and active low clock in
    // freescale SPI mode, master mode, 1MHz SSI frequency, and 8-bit data.
    
    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                       SSI_MODE_MASTER, 1000000, 8);
    
    //
    // Enable the SSI0 module.
    //
    SSIEnable(SSI0_BASE);
}



//UART0 used for console, UART1 used for remote processor communication
void configureUART1(void){
    // Enable the GPIO Peripheral used by the UART1 and enable UART1
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    
	
    // Configure GPIO Pins for UART mode.
    //
    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART1_BASE, UART_CLOCK_PIOSC);
    
    // Initialize UART1 with 9600 baud, 8 bit data, one stop, no parity
    UARTConfigSetExpClk(UART1_BASE, 16000000, 9600,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));

}

//oled display functions
void goTo(int x, int y) {
  if ((x >= SSD1351WIDTH) || (y >= SSD1351HEIGHT)) return;
  
  // set x and y coordinate
  writeCommand(SSD1351_CMD_SETCOLUMN);
  writeData(x);
  writeData(SSD1351WIDTH-1);

  writeCommand(SSD1351_CMD_SETROW);
  writeData(y);
  writeData(SSD1351HEIGHT-1);

  writeCommand(SSD1351_CMD_WRITERAM);  
}

int Color565(int r, int g, int b) {
  int c;
  c = r >> 3;
  c <<= 6;
  c |= g >> 2;
  c <<= 5;
  c |= b >> 3;

  return c;
}

void fillScreen(int fillcolor) {
  fillRect(0, 0, SSD1351WIDTH, SSD1351HEIGHT, fillcolor);
}

// Draw a filled rectangle with no rotation.
void rawFillRect(int x, int y, int w, int h, int fillcolor) {
  // Bounds check
  if ((x >= SSD1351WIDTH) || (y >= SSD1351HEIGHT))
    return;

  // Y bounds check
  if (y+h > SSD1351HEIGHT)
  {
    h = SSD1351HEIGHT - y - 1;
  }

  // X bounds check
  if (x+w > SSD1351WIDTH)
  {
    w = SSD1351WIDTH - x - 1;
  }
  
  /*
  Serial.print(x); Serial.print(", ");
  Serial.print(y); Serial.print(", ");
  Serial.print(w); Serial.print(", ");
  Serial.print(h); Serial.println(", ");
*/

  // set location
  writeCommand(SSD1351_CMD_SETCOLUMN);
  writeData(x);
  writeData(x+w-1);
  writeCommand(SSD1351_CMD_SETROW);
  writeData(y);
  writeData(y+h-1);
  // fill!
  writeCommand(SSD1351_CMD_WRITERAM);  

  for (int i=0; i < w*h; i++) {
    writeData(fillcolor >> 8);
    writeData(fillcolor);
  }
}

void swap(int x, int y){
	int temp=x;
	x=y;
	y=temp;
}

int getRotation(void){
	return 0;
}

/**************************************************************************/
/*!
    @brief  Draws a filled rectangle using HW acceleration
*/
/**************************************************************************/
int fillRect(int x, int y, int w, int h, int fillcolor) {
	//int temp;
  // Transform x and y based on current rotation.
  switch (getRotation()) {
	//switch(0){
  case 0:  // No rotation
          //UARTprintf("case0 for fill rect");
    rawFillRect(x, y, w, h, fillcolor);
    break;
  case 1:  // Rotated 90 degrees clockwise.
    swap(x, y);
//		temp=x;
//		x=y;
//		y=temp;
    x = WIDTH - x - h;
    rawFillRect(x, y, h, w, fillcolor);
    break;
  case 2:  // Rotated 180 degrees clockwise.
    x = WIDTH - x - w;
    y = HEIGHT - y - h;
    rawFillRect(x, y, w, h, fillcolor);
    break;
  case 3:  // Rotated 270 degrees clockwise.
    swap(x, y);
    y = HEIGHT - y - w;
    rawFillRect(x, y, h, w, fillcolor);
    break;
  }
	return 0;
}

// Draw a horizontal line ignoring any screen rotation.
void rawFastHLine(int x, int y, int w, int color) {
  // Bounds check
  if ((x >= SSD1351WIDTH) || (y >= SSD1351HEIGHT))
    return;

  // X bounds check
  if (x+w > SSD1351WIDTH)
  {
    w = SSD1351WIDTH - x - 1;
  }

  if (w < 0) return;

  // set location
  writeCommand(SSD1351_CMD_SETCOLUMN);
  writeData(x);
  writeData(x+w-1);
  writeCommand(SSD1351_CMD_SETROW);
  writeData(y);
  writeData(y);
  // fill!
  writeCommand(SSD1351_CMD_WRITERAM);  

  for (int i=0; i < w; i++) {
    writeData(color >> 8);
    writeData(color);
  }
}

// Draw a vertical line ignoring any screen rotation.
void rawFastVLine(int x, int y, int h, int color) {
  // Bounds check
  if ((x >= SSD1351WIDTH) || (y >= SSD1351HEIGHT))
  return;

  // X bounds check
  if (y+h > SSD1351HEIGHT)
  {
    h = SSD1351HEIGHT - y - 1;
  }

  if (h < 0) return;

  // set location
  writeCommand(SSD1351_CMD_SETCOLUMN);
  writeData(x);
  writeData(x);
  writeCommand(SSD1351_CMD_SETROW);
  writeData(y);
  writeData(y+h-1);
  // fill!
  writeCommand(SSD1351_CMD_WRITERAM);  

  for (int i=0; i < h; i++) {
    writeData(color >> 8);
    writeData(color);
  }
}



void drawFastVLine(int x, int y, int h, int color) {
  // Transform x and y based on current rotation.
  switch (getRotation()) {
  case 0:  // No rotation
    rawFastVLine(x, y, h, color);
    break;
  case 1:  // Rotated 90 degrees clockwise.
    swap(x, y);
    x = WIDTH - x - h;
    rawFastHLine(x, y, h, color);
    break;
  case 2:  // Rotated 180 degrees clockwise.
    x = WIDTH - x - 1;
    y = HEIGHT - y - h;
    rawFastVLine(x, y, h, color);
    break;
  case 3:  // Rotated 270 degrees clockwise.
    swap(x, y);
    y = HEIGHT - y - 1;
    rawFastHLine(x, y, h, color);
    break;
  }
}

void drawFastHLine(int x, int y, int w, int color) {
  // Transform x and y based on current rotation.
  switch (getRotation()) {
  case 0:  // No rotation.
    rawFastHLine(x, y, w, color);
    break;
  case 1:  // Rotated 90 degrees clockwise.
    swap(x, y);
    x = WIDTH - x - 1;
    rawFastVLine(x, y, w, color);
    break;
  case 2:  // Rotated 180 degrees clockwise.
    x = WIDTH - x - w;
    y = HEIGHT - y - 1;
    rawFastHLine(x, y, w, color);
    break;
  case 3:  // Rotated 270 degrees clockwise.
    swap(x, y);
    y = HEIGHT - y - w;
    rawFastVLine(x, y, w, color);
    break;
  }
}

// Used to do circles and roundrects
void fillCircleHelper(int x0, int y0, int r,
                      int cornername, int delta, int color) {
    
    int f     = 1 - r;
    int ddF_x = 1;
    int ddF_y = -2 * r;
    int x     = 0;
    int y     = r;
    
    while (x<y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f     += ddF_y;
        }
        x++;
        ddF_x += 2;
        f     += ddF_x;
        
        if (cornername & 0x1) {
            drawFastVLine(x0+x, y0-y, 2*y+1+delta, color);
            drawFastVLine(x0+y, y0-x, 2*x+1+delta, color);
        }
        if (cornername & 0x2) {
            drawFastVLine(x0-x, y0-y, 2*y+1+delta, color);
            drawFastVLine(x0-y, y0-x, 2*x+1+delta, color);
        }
    }
}

void fillCircle(int x0, int y0, int r,
                              int color) {
    drawFastVLine(x0, y0-r, 2*r+1, color);
    fillCircleHelper(x0, y0, r, 3, 0, color);
}


void begin(void) {

    // Initialization Sequence
    writeCommand(SSD1351_CMD_COMMANDLOCK);  // set command lock
    writeData(0x12);  
    writeCommand(SSD1351_CMD_COMMANDLOCK);  // set command lock
    writeData(0xB1);

    writeCommand(SSD1351_CMD_DISPLAYOFF);  		// 0xAE

    writeCommand(SSD1351_CMD_CLOCKDIV);  		// 0xB3
    writeCommand(0xF1);  						// 7:4 = Oscillator Frequency, 3:0 = CLK Div Ratio (A[3:0]+1 = 1..16)
    
    writeCommand(SSD1351_CMD_MUXRATIO);
    writeData(127);
    
    writeCommand(SSD1351_CMD_SETREMAP);
    writeData(0x74);
  
    writeCommand(SSD1351_CMD_SETCOLUMN);
    writeData(0x00);
    writeData(0x7F);
    writeCommand(SSD1351_CMD_SETROW);
    writeData(0x00);
    writeData(0x7F);

    writeCommand(SSD1351_CMD_STARTLINE); 		// 0xA1
    if (SSD1351HEIGHT == 96) {
      writeData(96);
    } else {
      writeData(0);
    }


    writeCommand(SSD1351_CMD_DISPLAYOFFSET); 	// 0xA2
    writeData(0x0);

    writeCommand(SSD1351_CMD_SETGPIO);
    writeData(0x00);
    
    writeCommand(SSD1351_CMD_FUNCTIONSELECT);
    writeData(0x01); // internal (diode drop)
    //writeData(0x01); // external bias

//    writeCommand(SSSD1351_CMD_SETPHASELENGTH);
//    writeData(0x32);

    writeCommand(SSD1351_CMD_PRECHARGE);  		// 0xB1
    writeCommand(0x32);
 
    writeCommand(SSD1351_CMD_VCOMH);  			// 0xBE
    writeCommand(0x05);

    writeCommand(SSD1351_CMD_NORMALDISPLAY);  	// 0xA6

    writeCommand(SSD1351_CMD_CONTRASTABC);
    writeData(0xC8);
    writeData(0x80);
    writeData(0xC8);

    writeCommand(SSD1351_CMD_CONTRASTMASTER);
    writeData(0x0F);

    writeCommand(SSD1351_CMD_SETVSL );
    writeData(0xA0);
    writeData(0xB5);
    writeData(0x55);
    
    writeCommand(SSD1351_CMD_PRECHARGE2);
    writeData(0x01);
    
    writeCommand(SSD1351_CMD_DISPLAYON);		//--turn on oled panel    
}
void drawPixel(int x, int y, int color)
{
  // Transform x and y based on current rotation.
  switch (getRotation()) {
  // Case 0: No rotation
  case 1:  // Rotated 90 degrees clockwise.
    swap(x, y);
    x = WIDTH - x - 1;
    break;
  case 2:  // Rotated 180 degrees clockwise.
    x = WIDTH - x - 1;
    y = HEIGHT - y - 1;
    break;
  case 3:  // Rotated 270 degrees clockwise.
    swap(x, y);
    y = HEIGHT - y - 1;
    break;
  }

  // Bounds check.
  if ((x >= SSD1351WIDTH) || (y >= SSD1351HEIGHT)) return;
  if ((x < 0) || (y < 0)) return;

  goTo(x, y);
  
  // setup for data
  //*rsport |= rspinmask;
  //*csport &= ~ cspinmask;
  
  writeData(color >> 8);    
  writeData(color);
  
 // *csport |= cspinmask;
}
void drawCircle(int x0, int y0, int r,
                int color) {
    int f = 1 - r;
    int ddF_x = 1;
    int ddF_y = -2 * r;
    int x = 0;
    int y = r;
    
    drawPixel(x0  , y0+r, color);
    drawPixel(x0  , y0-r, color);
    drawPixel(x0+r, y0  , color);
    drawPixel(x0-r, y0  , color);
    
    while (x<y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;
        
        drawPixel(x0 + x, y0 + y, color);
        drawPixel(x0 - x, y0 + y, color);
        drawPixel(x0 + x, y0 - y, color);
        drawPixel(x0 - x, y0 - y, color);
        drawPixel(x0 + y, y0 + x, color);
        drawPixel(x0 - y, y0 + x, color);
        drawPixel(x0 + y, y0 - x, color);
        drawPixel(x0 - y, y0 - x, color);
    }
}

void drawChar(int x, int y, char c,
			    int color, int bg, int size) {

  if((x >= WIDTH)            || // Clip right
     (y >= HEIGHT)           || // Clip bottom
     ((x + 6 * size - 1) < 0) || // Clip left
     ((y + 8 * size - 1) < 0))   // Clip top
    return;
	int i;
  for (i=0; i<6; i++ ) {
    int line;
    if (i == 5) 
      line = 0x0;
    else 
      line = font[(c*5)+i];
    for (int8_t j = 0; j<8; j++) {
      if (line & 0x1) {
        if (size == 1) // default size
          drawPixel(x+i, y+j, color);
        else {  // big size
          fillRect(x+(i*size), y+(j*size), size, size, color);
        } 
      } else if (bg != color) {
        if (size == 1) // default size
          drawPixel(x+i, y+j, bg);
        else {  // big size
          fillRect(x+i*size, y+j*size, size, size, bg);
        }
      }
      line >>= 1;
    }
  }
}
					
void drawLine(int x0, int y0,
			    int x1, int y1,
			    int color) {
  int steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) {
    swap(x0, y0);
    swap(x1, y1);
  }

  if (x0 > x1) {
    swap(x0, x1);
    swap(y0, y1);
  }

  int dx, dy;
  dx = x1 - x0;
  dy = abs(y1 - y0);

  int err = dx / 2;
  int ystep;

  if (y0 < y1) {
    ystep = 1;
  } else {
    ystep = -1;
  }

  for (; x0<=x1; x0++) {
    if (steep) {
      drawPixel(y0, x0, color);
    } else {
      drawPixel(x0, y0, color);
    }
    err -= dy;
    if (err < 0) {
      y0 += ystep;
      err += dx;
    }
  }
}
//end oled functions
					
//send positions via xbee on uart1
void SendPositions(int positions[]) {
    
    for (int i=0; i<2; i++) {
        UARTCharPut(UART1_BASE,positions[i]);
    }
}


void SysTick_Handler (void) {
 
    //clear interrupts
    GPIOIntClear(GPIO_PORTB_BASE, GPIO_INT_PIN_5);
    GPIOIntDisable(GPIO_PORTB_BASE, GPIO_INT_PIN_5);
    iTick++;//increment counter
    //int generated interrupt from accelerometer, now print x,y,z
    
    //get x value by writing reg 0 to slave address, then reading the register
    //with a single receive. burst send start is used to avoid the stop bit.
    I2CMasterSlaveAddrSet(I2C0_BASE,SLAVE_ADDRESS,false);
    I2CMasterDataPut(I2C0_BASE,0x00);//reg 0 address sent
    I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_BURST_SEND_START);
    while (I2CMasterBusy(I2C0_BASE)) {;}
    I2CMasterSlaveAddrSet(I2C0_BASE,SLAVE_ADDRESS,true);
    I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_RECEIVE);
    while (I2CMasterBusy(I2C0_BASE)) {;}
    accelX=I2CMasterDataGet(I2C0_BASE);
    int readInteger=accelX & 0x1F;//get ls 5 bits
    if((accelX & 0x20)==32){//if 5th bit is 1, the number is negative (bits[7:0])
        accelX=-readInteger;
    }
    else{
        accelX=readInteger;
    }
    
    //get y value by writing reg 0 to slave address, then reading the register
    //with a single receive. burst send start is used to avoid the stop bit.
    I2CMasterSlaveAddrSet(I2C0_BASE,SLAVE_ADDRESS,false);
    I2CMasterDataPut(I2C0_BASE,0x01);//reg 1 address sent
    I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_BURST_SEND_START);
    while (I2CMasterBusy(I2C0_BASE)) {;}
    I2CMasterSlaveAddrSet(I2C0_BASE,SLAVE_ADDRESS,true);
    I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_RECEIVE);
    while (I2CMasterBusy(I2C0_BASE)) {;}
    accelY=I2CMasterDataGet(I2C0_BASE);
    readInteger=accelY & 0x1F;
    if((accelY & 0x20)==32){//if 5th bit is 1, the number is negative (bits[7:0])
        accelY=-readInteger;
    }
    else{
        accelY=readInteger;
    }
    
    //get z value by writing reg 0 to slave address, then reading the register
    //with a single receive. burst send start is used to avoid the stop bit.
    I2CMasterSlaveAddrSet(I2C0_BASE,SLAVE_ADDRESS,false);
    I2CMasterDataPut(I2C0_BASE,0x02);//reg 2 address sent
    I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_BURST_SEND_START);
    while (I2CMasterBusy(I2C0_BASE)) {;}
    I2CMasterSlaveAddrSet(I2C0_BASE,SLAVE_ADDRESS,true);
    I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_RECEIVE);
    while (I2CMasterBusy(I2C0_BASE)) {;}
    accelZ=I2CMasterDataGet(I2C0_BASE);
    readInteger=accelZ & 0x1F;
    if((accelZ & 0x20)==32){//if 5th bit is 1, the number is negative (bits[7:0])
        accelZ=-readInteger;
    }
    else{
        accelZ=readInteger;
    }
    //print to console for debug
    UARTprintf("X=%d, Y=%d, Z=%d\n",accelX-Xoffset,accelY-Yoffset,accelZ-Zoffset);
    //erase ball and redraw
    fillCircle(ballX,ballY,4,WHITE);
    if (accelX>-30 && accelX<32) {
        if (accelX<0) {//when x is in the negative direction
            if (ballX<120) {//if ball is not on right edge, subtract accelX/5 to control the speed of movement. subtract because this value is negative
                ballX=ballX-accelX/5;
            }
            
        }
        else{//direction is positive
            if(ballX>8){//if not on the left edge of screen
                ballX-=accelX/5;//subtract accelX/5 since this is positive X, and dividing accelX by 5 controls speed
            }
        }
    }
    if (accelY>-28 && accelY<32) {//repeat ball coordinate update for Y values
        if (accelY<0) {
            if (ballY>8) {//negative Y, less than top edge
                ballY=ballY+accelY/5;//add to ballY to control speed
            }
            
        }
        else{
            if(ballY<120){
                ballY+=accelY/5;
            }
        }
    }
    //redraw black ball
    fillCircle(ballX,ballY,4,BLACK);
    int positions[2];
    positions[0]=ballX;
    positions[1]=ballY;
    SendPositions(positions);//send 2 ball coordinates to remote via xbee
    UARTprintf("hit\n");
    SysCtlDelay(SysCtlClockGet()/3/10);
    GPIOIntClear(GPIO_PORTB_BASE, GPIO_INT_PIN_5);
    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_INT_PIN_5);
	
}



void callibrateAccelerometer(void){
    //get x,y,z values like in the interrupt handler at the first value to callibrate when reset is pressed, and board is flat
    int tempaccelX, tempaccelY, tempaccelZ;
    I2CMasterSlaveAddrSet(I2C0_BASE,SLAVE_ADDRESS,false);
    I2CMasterDataPut(I2C0_BASE,0x00);//reg 0 address sent
    I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_BURST_SEND_START);
    while (I2CMasterBusy(I2C0_BASE)) {;}
    I2CMasterSlaveAddrSet(I2C0_BASE,SLAVE_ADDRESS,true);
    I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_RECEIVE);
    while (I2CMasterBusy(I2C0_BASE)) {;}
    tempaccelX=I2CMasterDataGet(I2C0_BASE);
    int readInteger=accelX & 0x1F;//get ls 5 bits
    if((tempaccelX & 0x20)==32){//if 5th bit is 1, the number is negative (bits[7:0])
        tempaccelX=-readInteger;
    }
    else{
        tempaccelX=readInteger;
    }
    
    //get y
    I2CMasterSlaveAddrSet(I2C0_BASE,SLAVE_ADDRESS,false);
    I2CMasterDataPut(I2C0_BASE,0x01);//reg 0 address sent
    I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_BURST_SEND_START);
    while (I2CMasterBusy(I2C0_BASE)) {;}
    I2CMasterSlaveAddrSet(I2C0_BASE,SLAVE_ADDRESS,true);
    I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_RECEIVE);
    while (I2CMasterBusy(I2C0_BASE)) {;}
    tempaccelY=I2CMasterDataGet(I2C0_BASE);
    readInteger=accelY & 0x1F;
    if((tempaccelY & 0x20)==32){//if 5th bit is 1, the number is negative (bits[7:0])
        tempaccelY=-readInteger;
    }
    else{
        tempaccelY=readInteger;
    }
    
    //get z
    I2CMasterSlaveAddrSet(I2C0_BASE,SLAVE_ADDRESS,false);
    I2CMasterDataPut(I2C0_BASE,0x02);//reg 0 address sent
    I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_BURST_SEND_START);
    while (I2CMasterBusy(I2C0_BASE)) {;}
    I2CMasterSlaveAddrSet(I2C0_BASE,SLAVE_ADDRESS,true);
    I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_RECEIVE);
    while (I2CMasterBusy(I2C0_BASE)) {;}
    accelZ=I2CMasterDataGet(I2C0_BASE);
    readInteger=accelZ & 0x1F;
    if((tempaccelZ & 0x20)==32){//if 5th bit is 1, the number is negative (bits[7:0])
        tempaccelZ=-readInteger;
    }
    else{
        tempaccelZ=readInteger;
    }
    
    Xoffset=tempaccelX;
    Yoffset=tempaccelY;
    Zoffset=tempaccelZ;
    
}

void initI2C(void){
    //pb5 for interrupts, pb2 for scl pb3 for sda
    //initialize i2c pins and interrupts
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);//reset?
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);
    
    //write to reg 0x07 for standby mode
    
    I2CMasterSlaveAddrSet(I2C0_BASE,SLAVE_ADDRESS,false);//false to send
    I2CMasterDataPut(I2C0_BASE,0x07);
    I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_BURST_SEND_START);
    while (I2CMasterBusy(I2C0_BASE)) {;}//wait until done sending
    //I2CMasterSlaveAddrSet(I2C0_BASE,SLAVE_ADDRESS,false);//false to send
    I2CMasterDataPut(I2C0_BASE,0);//set to standby mode by writing to d0
    I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_BURST_SEND_FINISH);
    while (I2CMasterBusy(I2C0_BASE)) {;}
    
    
    //write to register 0x08to set sampling rate to 120
    I2CMasterSlaveAddrSet(I2C0_BASE,SLAVE_ADDRESS,false);//false to send
    I2CMasterDataPut(I2C0_BASE,0x08);
    I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_BURST_SEND_START);
    while (I2CMasterBusy(I2C0_BASE)) {;}//wait until done sending
    //I2CMasterSlaveAddrSet(I2C0_BASE,SLAVE_ADDRESS,false);//false to send
    I2CMasterDataPut(I2C0_BASE,0X01);
    I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_BURST_SEND_FINISH);
    while (I2CMasterBusy(I2C0_BASE)) {;}
    
    //write to reg 0x06 to control gint interrupts
    I2CMasterSlaveAddrSet(I2C0_BASE,SLAVE_ADDRESS,false);//false to send
    I2CMasterDataPut(I2C0_BASE,0x06);
    I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_BURST_SEND_START);
    while (I2CMasterBusy(I2C0_BASE)) {;}//wait until done sending
    //I2CMasterSlaveAddrSet(I2C0_BASE,SLAVE_ADDRESS,false);//false to send
    I2CMasterDataPut(I2C0_BASE,0x10);//set only gint to 1
    I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_BURST_SEND_FINISH);
    while (I2CMasterBusy(I2C0_BASE)) {;}
    
    
    
    //write 1 to reg 0x07 for active mode
    I2CMasterSlaveAddrSet(I2C0_BASE,SLAVE_ADDRESS,false);//false to send
    I2CMasterDataPut(I2C0_BASE,0x07);
    I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_BURST_SEND_START);
    while (I2CMasterBusy(I2C0_BASE)) {;}//wait until done sending
    //I2CMasterSlaveAddrSet(I2C0_BASE,SLAVE_ADDRESS,false);//false to send
    I2CMasterDataPut(I2C0_BASE,1);//set to active mode by writing to d0
    I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_BURST_SEND_FINISH);
    while (I2CMasterBusy(I2C0_BASE)) {;}
    
    
    
    callibrateAccelerometer();//call when board is flat and being reset
    
    //configure interrupt type for PB5 after i2c setup
    GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_LOW_LEVEL);
    //enable interrupts after configuring
    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_INT_PIN_5);
    IntEnable(INT_GPIOB);
    IntMasterEnable();
    
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_1);//red led
    
    //reset global int counter
    iTick=0;
    arrayVals=0;
    
}

					
int main(void){
	InitConsole();//uart
	initializePins();//set spi/ssi pins
    //toggle reset on display to turn on, set to high during actions
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6);
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, 0);//reset is low
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6);//reset is low
	begin();
    fillScreen(0xffff);
	initializeIRInterrupts();//lab1 code
    configureUART1();
    initI2C();
    
    
	int buttonPressed;
	int oldButtonPressed=-1;
	int amountPressed=0;
	char toPrint;
	int charXValue=0;
	int charYValue=0;
   
	int bitValues[32];
        int pulseWidth=0;
        int arrayPos=0;
    //int buttonBreak=0;
   /*
	iTick=0;
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6);
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, 0);//reset is low
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6);//reset is low
    */
    //fill screen white and draw a line halfway through the screen
	fillScreen(0xffff);
    
    
    fillCircle(ballX,ballY,4,BLACK);
    //drawChar(0,0,'t',BLACK,WHITE,1);
    //drawChar(0,123,'b',BLACK,WHITE,1);
    
    int positions[2];
    systickDone=true;
    int debugstatus;
    //int stupidcount=0;
	while(1){
        //wait for interrupts
        
	}
	return 0;
}