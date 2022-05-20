#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include<stdio.h>
#include<math.h>

#define NU32_DESIRED_BAUD 230400    // Baudrate for RS232
#define NU32_SYS_FREQ 48000000ul    // 48 million Hz

// DEVCFG0
#pragma config DEBUG = OFF // disable debugging
#pragma config JTAGEN = OFF // disable jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // disable flash write protect
#pragma config BWP = OFF // disable boot write protect
#pragma config CP = OFF // disable code protect

// DEVCFG1
#pragma config FNOSC = FRCPLL // use fast frc oscillator with pll
#pragma config FSOSCEN = OFF // disable secondary oscillator
#pragma config IESO = OFF // disable switching clocks
#pragma config POSCMOD = OFF // primary osc disabled
#pragma config OSCIOFNC = OFF // disable clock output
#pragma config FPBDIV = DIV_1 // divide sysclk freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // disable clock switch and FSCM
#pragma config WDTPS = PS1 // use largest wdt value
#pragma config WINDIS = OFF // use non-window mode wdt
#pragma config FWDTEN = OFF // wdt disabled
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the sysclk clock to 48MHz from the 8MHz fast rc internal oscillator
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz

// DEVCFG3
#pragma config USERID = 0xffff // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations

char msg[500];

void ReadUART1(char * message, int maxLength) {
  char data = 0;
  int complete = 0, num_bytes = 0;
  // loop until you get a '\r' or '\n'
  while (!complete) {
    if (U1STAbits.URXDA) { // if data is available
      data = U1RXREG;      // read the data
      if ((data == '\n') || (data == '\r')) {
        complete = 1;
      } else {
        message[num_bytes] = data;
        ++num_bytes;
        // roll over if the array is too small
        if (num_bytes >= maxLength) {
          num_bytes = 0;
        }
      }
    }
  }
  // end the string
  message[num_bytes] = '\0';
}

// Write a character array using UART3
void WriteUART1(const char * string) {
  while (*string != '\0') {
    while (U1STAbits.UTXBF) {
      ; // wait until tx buffer isn't full
    }
    U1TXREG = *string;
    ++string;
  }
}

void StartUART1(){
    __builtin_disable_interrupts(); // disable interrupts while initializing things
    // UART ?
    U1MODEbits.BRGH = 0; // set baud to NU32_DESIRED_BAUD
    U1BRG = ((NU32_SYS_FREQ / NU32_DESIRED_BAUD) / 16) - 1;

    // 8 bit, no parity bit, and 1 stop bit (8N1 setup)
    U1MODEbits.PDSEL = 0;
    U1MODEbits.STSEL = 0;

    // configure TX & RX pins as output & input pins
    U1STAbits.UTXEN = 1;
    U1STAbits.URXEN = 1;
    // configure hardware flow control using RTS and CTS
    U1MODEbits.UEN = 2;

    // enable the uart
    U1MODEbits.ON = 1;
}

/////////////////////////////////////////////////SPI DAC

#define CS LATBbits.LATB15

#define SINLEN 100              //  length of sin wave values
#define TRILEN 100              // 	length of tri wave values

#define pi 3.14159

void initSPI() {
    // Pin B14 has to be SCK1
    // Turn of analog pins
    //...
    // Make an output pin for CS
    //...
    //...
    // Set SDO1
    RPB13Rbits.RPB13R = 0b0011; // SD01 on pin B13
    //...
    // Set SDI1
    //...

    // setup SPI1
    SPI1CON = 0; // turn off the spi module and reset it
    SPI1BUF; // clear the rx buffer by reading from it
    SPI1BRG = 4799; // 1000 for 24kHz, 1 for 12MHz; // baud rate to 10 MHz [SPI1BRG = (48000000/(2*desired))-1]
    SPI1STATbits.SPIROV = 0; // clear the overflow bit
    SPI1CONbits.CKE = 1; // data changes when clock goes from hi to lo (since CKP is 0)
    SPI1CONbits.MSTEN = 1; // master operation
    SPI1CONbits.ON = 1; // turn on spi 
}


// send a byte via spi and return the response
unsigned char spi_io(unsigned char o) {
  SPI1BUF = o;
  while(!SPI1STATbits.SPIRBF) { // wait to receive the byte
    ;
  }
  return SPI1BUF;
}

void setVoltage(char channel, unsigned char voltage){
    unsigned short data = voltage;
    if (channel == 0) {//0 is channel A
        data = (data << 4) | 0x7000;
    }
    else if (channel == 1){ // 1 is channel B
        data = (data << 4) | 0xF000;
    }
    CS = 0;
    spi_io(data >> 8 ); // most significant byte of data
    spi_io(data);
    CS = 1;
}


unsigned short sinwave(void)  {
    int i;
    static unsigned short sinvec[100];
    for(i=0; i<SINLEN; i++) {
		sinvec[i] = 127.5*sin((double)2*pi*i/(SINLEN))+127.5;    
	}
    return sinvec;
}

unsigned short triwave(void)  {	
    int j;
    static unsigned short trivec[100];
    for(j=0; j<TRILEN/2; j++) {
		trivec[j] = 255*((double)j/TRILEN/2);                     
    }
    for(j=TRILEN/2; j<TRILEN; j++) {
        int i = 0;
		trivec[j] = 255 - 255*((double)i/TRILEN/2);  
        i++;
    }
    return trivec;
}
/////////////////////////////////////////////


int main() {

    __builtin_disable_interrupts(); // disable interrupts while initializing things

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    // do your TRIS and LAT commands here
    TRISAbits.TRISA4 = 0 ;// output
    LATAbits.LATA4 = 0; //high
    TRISBbits.TRISB4 = 1; // input
    
    // uart stuff
    StartUART1();
    U1RXRbits.U1RXR = 0b0000; // Set A2 to U1RX
    RPB3Rbits.RPB3R = 0b0001; // Set B3 to U1TX
    int i = 0;

    __builtin_enable_interrupts();
    
    
    unsigned short sinv[100];
    unsigned short triv[100];
    
    initSPI();             //  initialize spi1
    sinv = sinwave();
    triv = triwave();
    
    __builtin_enable_interrupts();
    
    //  loop should cycle through sin and tri vectors infinitely
    
    int sincount = 0;   //  counters to keep track of sinv and triv indexing
    int tricount = 0;
    char channel;

    while (1) {
        
        channel = 0;
        setVoltage(channel,(sinv[sincount]));
        
        //  send tri wave to channel B
        channel = 1;
        setVoltage(channel,(triv[tricount]));
        
        sincount++;
        tricount++;
        
        if  (sincount==SINLEN) {
            sincount = 0;       //  reset sincount to start next sin wave period 
        }
        if  (tricount==TRILEN) {
            tricount = 0;       //  reset tricount to start next tri wave period
        }
        


    }
}
