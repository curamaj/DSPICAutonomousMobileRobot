// Robocar.c : Main prigram as well as all needed library
// Support in a single file.
#include <xc.h>
#include "./robocar.h"

// I have no clue where this pragma magic is defined.
// All these are got as bits and pieces from other code bases on interne.
// Pls do not delete any of these !!!

#pragma config BWRP = WRPROTECT_OFF     // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH           // Boot Segment Program Flash Code Protection (No Boot program Flash segment)
#pragma config RBS = NO_RAM             // Boot Segment RAM Protection (No Boot RAM)

// FSS
#pragma config SWRP = WRPROTECT_OFF     // Secure Segment Program Write Protect (Secure segment may be written)
#pragma config SSS = NO_FLASH           // Secure Segment Program Flash Code Protection (No Secure Segment)
#pragma config RSS = NO_RAM             // Secure Segment Data RAM Protection (No Secure RAM)

// FGS
#pragma config GWRP = OFF               // General Code Segment Write Protect (User program memory is not write-protected)
#pragma config GSS = OFF                // General Segment Code Protection (User program memory is not code-protected)

// FOSCSEL
#pragma config FNOSC = FRCPLL           // Oscillator Mode (Internal Fast RC (FRC) w/ PLL)
#pragma config IESO = ON                // Internal External Switch Over Mode (Start-up device with FRC, then automatically switch to user-selected oscillator source when ready)

// FOSC
#pragma config POSCMD = NONE            // Primary Oscillator Source (Primary Oscillator Disabled)
#pragma config OSCIOFNC = OFF           // OSC2 Pin Function (OSC2 pin has clock out function)
#pragma config IOL1WAY = ON             // Peripheral Pin Select Configuration (Allow Only One Re-configuration)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor (Both Clock Switching and Fail-Safe Clock Monitor are disabled)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler (1:32,768)
#pragma config WDTPRE = PR128           // WDT Prescaler (1:128)
#pragma config WINDIS = OFF             // Watchdog Timer Window (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (Watchdog timer enabled/disabled by user software)

// FPOR
#pragma config FPWRT = PWR128           // POR Timer Value (128ms)
#pragma config ALTI2C = OFF             // Alternate I2C  pins (I2C mapped to SDA1/SCL1 pins)
#pragma config LPOL = ON                // Motor Control PWM Low Side Polarity bit (PWM module low side output pins have active-high output polarity)
#pragma config HPOL = ON                // Motor Control PWM High Side Polarity bit (PWM module high side output pins have active-high output polarity)
#pragma config PWMPIN = ON              // Motor Control PWM Module Pin Mode bit (PWM module pins controlled by PORT register at device Reset)

// FICD
#pragma config ICS = PGD1               // Comm Channel Select (Communicate on PGC1/EMUC1 and PGD1/EMUD1)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG is Disabled)

#define PPSUnLock __builtin_write_OSCCONL(OSCCON & 0xBF)
#define PPSLock __builtin_write_OSCCONL(OSCCON | 0x40)

// I cannot find bzero() defined anywhere, hence  wrote one.
void crk_bzero(char *s, int n)
{
	int i;
	for (i=0; i < n; i++) *s++ = 0;
}

//Generic SPI interface Mostly from the text book.
unsigned char SPI_Transmit(unsigned char TxValue)
{
    while (SPI1STATbits.SPITBF == 1)
			;// wait until TX buffer is empty due to a prior process
    SPI1BUF = TxValue; // when empty, send the byte to the tx buffer
    while (SPI1STATbits.SPIRBF == 0)
			;//As valid bits shifts out of SDO, junk bits are recieved from SDI,			 // wait till the RX buffer is full of junk data
    return SPI1BUF; //when full, read the junk data in RX buffer from SPI1BUF
}

unsigned char SPI_Receive()
{
    printf( "DEBUG SPI STAT Bits = (%x) \n",SPI1STATbits.SPITBF);
    while (SPI1STATbits.SPITBF == 1)
			;// wait until TX buffer is empty due to a prior process
    SPI1BUF = 0x00; // when empty, send the junk byte (0x00) to the TC buffer.
    while (SPI1STATbits.SPIRBF == 0)
			; // As junk bits shifts out of SDO, vald bits are recieved from SDI
			  // wait till the RX buffer is full of valid data
    return SPI1BUF; // when full, read the valid data in RX buffer from SPI1BUF
}

void Write_Status_Reg(unsigned char WriteStatusByte)
{
    LATBbits.LATB6 = 0;
    SPI_Transmit(0x01);
    SPI_Transmit(WriteStatusByte);
    LATBbits.LATB6 = 1;
}

unsigned char Read_Status_Reg()
{
    LATBbits.LATB6 = 0;
    SPI_Transmit(0x05);
    unsigned char status = SPI_Receive();
    LATBbits.LATB6 = 1;
    return status;
}

//Sram specific SPI interface
void Write_Byte_Sram(unsigned int Address, unsigned char SentByte)
{
    LATBbits.LATB6 = 0;
    SPI_Transmit (0x02);
    SPI_Transmit((Address >> 8)& 0xFF);
    SPI_Transmit (Address & 0xFF);
    SPI_Transmit (SentByte);
    LATBbits.LATB6 = 1;
}

unsigned char Read_Byte_Sram(unsigned int Address)
{
    LATBbits.LATB6 = 0;
    SPI_Transmit (0x03);
    SPI_Transmit((Address >> 8)& 0xFF);
    SPI_Transmit (Address & 0xFF);
    unsigned char RecievedByte = SPI_Receive();
    LATBbits.LATB6 = 1;
    return RecievedByte;
}
//Flash Interface follows

unsigned char Write_Byte_Flash(unsigned char cmd, 
				long int address, unsigned char SendByte)
{
	LATBbits.LATB6 = 0;
	SPI_Transmit(cmd);
	SPI_Transmit((address >> 16)&0xFF);
	SPI_Transmit((address >> 8)&0xFF);
	SPI_Transmit(address & 0xFF);
    SPI_Transmit(SendByte);
	unsigned char junk = SPI_Receive();
    LATBbits.LATB6 = 1;
	return(junk);
}

unsigned char Read_Byte_Flash(unsigned char cmd, long int address)
{
	LATBbits.LATB6 = 0;
	SPI_Transmit(cmd);
	SPI_Transmit((address >> 16)&0xFF);
	SPI_Transmit((address >> 8)&0xFF);
	SPI_Transmit(address & 0xFF);
	unsigned char ReceivedByte = SPI_Receive();
    LATBbits.LATB6 = 1;
	return(ReceivedByte);
}

void WREN()
{
	LATBbits.LATB6 = 0; // Lower SS for instruction delivery
	SPI_Transmit(0x06); // Send WREN
	LATBbits.LATB6 = 1; // Raise SS for instruction completiomn
}

void BulkErase()
{
	LATBbits.LATB6 = 0; // Lower SS for instruction delivery
	SPI_Transmit(0xC7); // Send Bulk Erase 
	LATBbits.LATB6 = 1; // Raise SS for instruction completiomn
}

unsigned char WriteStatusReg(unsigned char SendByte)
{
	LATBbits.LATB6 = 0;
	SPI_Transmit(0x01);
    SPI_Transmit(SendByte);
	unsigned char junk = SPI_Receive();
    LATBbits.LATB6 = 1;
	return(junk);
}

unsigned char ReadStatusReg()
{
	LATBbits.LATB6 = 0; 
	SPI_Transmit(0x05); // Send Read Status register instruction
	unsigned char status = SPI_Receive();
	LATBbits.LATB6 = 1;
	return(status);
}	
//These delay functions needs tuning, once I freee on CPU frequency
// For now this seems to work. I get the delay accurately.
void delay_ms(int n)
{
	int i,j, k, l;
	for (i=0; i < n; i++)
		for(j=0; j < 17; j++)
			for(k=0; k < 4; k++)
			l = i+j;
			l = i+j;

}

void DelayInSeconds(int seconds)
{
	int i;
	int j;
    
    for(i=0; i < seconds; i++) {
		for ( j = 0; j < 1000; j++) // 100 ms = 1 sec.
        	delay_ms(1); //One second approx delay
    }
}

//* PWM1 H1 controls channel CH1 
void	TimeOfTravel(double rc_travel_distance, double *time, 
									int *seconds, int *milliseconds)
{
	*time =  rc_travel_distance * TIMETOTRAVELAMETER; // This is adhooc !!!
							// In the calibration we have made it takes 2 sec
							// to travel a meter !!!
	*seconds = (int) *time; /// Get the intpart
	*milliseconds = (*time - *seconds) * 1000;
}
void TurnLeft(void)
{
	int n, i;

	rc_angle = GetFloatArg();
	n = rc_angle/rc_angle_per_pulse;

	printf("\n\r DEBUG rc_angle = (%f) no of pulses = (%d)", rc_angle, n);
  	printf("DEBUG Servo test Left P1TPER = (%d) P1DC1 = (%d) \n", P1TPER, P1DC1);
	for(i=0; i < n; i++) {
		P1DC1 = rc_left; 
		P1TCONbits.PTEN = 1;
	}
}

void Middle(void)
{
	rc_angle = 0.0;
	printf("\n\rDEBUG rc_angle = (%f)", rc_angle);
	printf("DEBUG Servo test Middle P1TPER = (%d) P1DC1 = (%d) \n", P1TPER, P1DC1);
		P1DC1 = rc_middle; 
		P1TCONbits.PTEN = 1;
}

void TurnRight(void)
{
	int n, i;

	rc_angle = GetFloatArg();
	n = rc_angle/rc_angle_per_pulse;

	printf("\n\r DEBUG rc_angle = (%f) no of pulses = (%d)", rc_angle, n);
	printf("DEBUG Servo test Right P1TPER = (%d) P1DC1 = (%d) \n", P1TPER, P1DC1);
	for (i=0; i < n; i++) {
		P1DC1 = rc_right; 
		P1TCONbits.PTEN = 1;
	}
}

void MoveFwd(void)
{
	double time;
	int seconds;
	int milliseconds;
	double travel_distance;

	travel_distance = GetFloatArg();
	
	printf("\n\rDEBUG distance = (%f)", travel_distance);
	printf("DEBUG Servo test FWD !!! P2TPER = (%d) P2DC1 = (%d) rc_fwd = (%d) \n", P2TPER, P2DC1, rc_fwd);
	TimeOfTravel(travel_distance, &time, &seconds, &milliseconds);

	Stop(); // Always stop before moving forward ... otherwise the car does not work
	P2DC1 = rc_fwd; 
	P2TCONbits.PTEN = 1;
 	DelayInSeconds(seconds);
	delay_ms(milliseconds);
	Stop(); 
}


void MoveBack(void)
{
	double time;
	int seconds;
	int milliseconds;
	double travel_distance;

	travel_distance = GetFloatArg();
	printf("\n\rDEBUG distance = (%f))", travel_distance);
	printf("SDEBUG ervo test Back !!! P2TPER = (%d) P2DC1 = (%d) rc_back = (%d) \n", P2TPER, P2DC1, rc_back);


	TimeOfTravel(travel_distance, &time, &seconds, &milliseconds);

	Stop(); 
	P2DC1 = rc_back; 
	P2TCONbits.PTEN = 1;
	DelayInSeconds(seconds);
	delay_ms(milliseconds);
	Stop(); 
}


void Stop(void)
{
		P2DC1 = rc_still; 
		P2TCONbits.PTEN = 1;
printf("Servo test Stop !!! P2TPER = (%d) P2DC1 = (%d) stop = (%d) \n", P2TPER, P2DC1,rc_still);
}

void ServoInit(void)
{

	rc_full_cycle =  FREQ62_5HZ; // Use this to get 62.5 Hz 

	P1TCONbits.PTEN = 0; // DO not enable clock deliverto PWM1 timer yet
	P1TCONbits.PTCKPS = 3; // Prescale is 1:1, Timer clock = FP ( 57.6 KHz ??)
	P1TCONbits.PTMOD = 0; // PWM1 is free running mode

	//PWM1 counter and period definitions
	P1TMRbits.PTMR = 0; // Initial value in PWM1 counter

	P1TPER = rc_full_cycle; //PWM1 perios register produces 20 msec PWM period
	
	//PWM1 control register
	PWM1CON1bits.PMOD3 = 1; //PWM1H3 and PWM1L3 outouts are independent
	PWM1CON1bits.PMOD2 = 1; //PWM1H2 and PWM1L2 outouts are independent
	PWM1CON1bits.PMOD1 = 1; //PWM1H1 and PWM1L1 outouts are independent

	PWM1CON1bits.PEN3H = 0; //PWM1H3 is disabled
	PWM1CON1bits.PEN2H = 0; //PWM1H3 is disabled
	PWM1CON1bits.PEN1H = 1; //PWM1H1 is enabled
	PWM1CON1bits.PEN3L = 0; //PWM1L3 is disabled
	PWM1CON1bits.PEN2L = 0; //PWM1L2 is disabled
	PWM1CON1bits.PEN1L = 0; //PWM1L1 is disabled

	// PWM control 2 registers
	PWM1CON2bits.IUE = 0; //Updates are synchronized with timnebase
	PWM1CON2bits.UDIS = 0; // Updates from perios and suty cycle registers are enabled

	// Duty cycle for PWM CH1 Forward, backword, still
	rc_left = 2 *11.3 * rc_full_cycle / 100;
	rc_right = 2 *6.6 * rc_full_cycle / 100;
	rc_middle = 2 *9.0 * rc_full_cycle / 100;

	// Channel CH2 which controls FWD, Backword and Still is on PWM2H (Pin 23)
	P2TCONbits.PTEN = 0; // DO not enable clock deliverto PWM1 timer yet
	P2TCONbits.PTCKPS = 3; // Prescale is 1:1, Timer clock = FP ( 57.6 KHz ??)
	P2TCONbits.PTMOD = 0; // PWM1 is free running mode

	//PWM1 counter and period definitions
	P2TMRbits.PTMR = 0; // Initial value in PWM1 counter

	P2TPER = rc_full_cycle; //PWM1 perios register produces 20 msec PWM period
	
	//PWM1 control register
	PWM2CON1bits.PMOD1 = 1; //PWM1H1 and PWM1L1 outouts are independent

	PWM2CON1bits.PEN1H = 1; //PWM1H1 is enabled
	PWM2CON1bits.PEN1L = 0; //PWM1L1 is disabled

	// PWM control 2 registers
	PWM2CON2bits.IUE = 0; //Updates are synchronized with timnebase
	PWM2CON2bits.UDIS = 0; // Updates from perios and suty cycle registers are enabled
//We need to fine tune these !!!
//	rc_fwd = 2 * 10.0 * full_cycle / 100;
	rc_fwd = 2 * 9.8 * rc_full_cycle / 100;
	rc_still = 2 *  9.4 * rc_full_cycle / 100;
	rc_back = 2 * 8.8 * rc_full_cycle / 100;	

}

unsigned char U1Rx(void)
{
	char ch;
	int i =0;
	char data[128];

	U1STAbits.OERR = 0;
	U1STAbits.FERR = 0;

	while(i < 128) {
		/* Check for receive errors */
		i++;
		if(U1STAbits.FERR == 1) {
			continue;
		}
		/* Must clear the overrun error to keep UART receiving */
		if(U1STAbits.OERR == 1) {
			U1STAbits.OERR = 0;
			continue;
		}
		/* Get the data */
		if(U1STAbits.URXDA == 1) {
			ch = U1RXREG;
			data[i-1] = ch;
		}
	}
	return (ch);
}

void U1Tx(unsigned char ch)

{
   	while (U1STAbits.TRMT  == 0)
		; // Wait till the transmit buffer is empty
	U1TXREG  = ch;
}

unsigned char U2Rx(void)
{
	unsigned char ch;
	while(U2STAbits.URXDA != 1) 
			;
	ch = U2RXREG;
	return(ch);
}

void U2Tx(unsigned char ch)

{
   	while (U2STAbits.TRMT  == 0)
		; // Wait till the transmit buffer is empty
	U2TXREG  = ch;
}

void BlueToothTest(void)
{
	int i;

	char *buf = "Hello World\n Enter character ";
	printf("DEBUG BlueTooth Test !!!\n");
	for (i=0; i < strlen(buf); i++) {
		U2Tx(buf[i]);
	}
}
		
//TBD
void SPITestMagnet(void)
{
    unsigned char status;

	printf("DEBUG Now dping SPI Test !!!\n");
	while (1) {
		status = SPI_Receive();
		printf("DEBUG SPI receceive = (%c)\n", status);
		DelayInSeconds(1);
	}
	
}

void SPIInit()
{
	//SPI channel setting.
    SPI1CON1bits.DISSCK = 0;
    SPI1CON1bits.DISSDO = 0;        
    SPI1CON1bits.MODE16 = 0;
    SPI1CON1bits.SSEN = 0;
    SPI1CON1bits.MSTEN = 1;
    SPI1CON1bits.SMP = 0;
    SPI1CON1bits.CKE = 1;
    SPI1CON1bits.CKP = 0;
    SPI1CON1bits.PPRE = 1;
    SPI1CON1bits.SPRE = 7;
    SPI1STATbits.SPIROV = 0;
    SPI1STATbits.SPIEN = 1;

    PPSUnLock;
    RPOR1bits.RP3R = 8;
    RPINR20bits.SCK1R = 8;
    RPOR1bits.RP2R = 7;
    RPINR20bits.SDI1R = 7;
    RPOR3bits.RP6R = 9;
    PPSLock;

    TRISBbits.TRISB6 = 0;
    TRISBbits.TRISB3 = 0;
    TRISBbits.TRISB2 = 0;
    TRISBbits.TRISB7 = 1;
}


void UART1Init(void)
{
    PLLFBDbits.PLLDIV = 64; // My Chosen value

	CLKDIVbits.DOZE = 0x5; 
	CLKDIVbits.DOZEN = 1; 
    OSCCONbits.IOLOCK = 0;

	RPINR18bits.U1RXR =5; // RP5 -> Pin14
	RPOR2bits.RP4R = 3; // UART1 TX PR4 -> Pin 11

    OSCCONbits.IOLOCK = 1;

    TRISBbits.TRISB5 = 1;
    TRISBbits.TRISB4 = 0;

  	U1BRG = 191; // (Fp/(16*baudrate)) - 1 
    U1MODE = 0;
    U1MODEbits.UARTEN = 1; // Enable UART
    U1MODEbits.UEN    = 0;
    U1MODEbits.PDSEL  = 0; // 8-bit data, no parity // MUST
    U1MODEbits.STSEL  = 0; // 1 stop bit// MUST
    U1MODEbits.ABAUD  = 0; // No auto baud 
    U1MODEbits.LPBACK = 0; // No loopback mode
    U1STA             = 0;
    U1STAbits.UTXEN   = 1; // Enable transmit
	U1STAbits.URXISEL = 0; // Interrupt after one RX character is received; 
}

void UART2Init(void)
{
    OSCCONbits.IOLOCK = 0;
	RPINR19bits.U2RXR =11; // RP11 -> Pin22
	RPOR5bits.RP10R = 5; // UART2 TX RP10 -> Pin21 
    OSCCONbits.IOLOCK = 1;

    TRISBbits.TRISB11 = 1;
    TRISBbits.TRISB10 = 0;

  	U2BRG =15; //  192/12 - 1; // (Fp/(16*baudrate)) - 1 
    U2MODE = 0;
    U2MODEbits.UARTEN = 1; // Enable UART
    U2MODEbits.UEN    = 0;
    U2MODEbits.PDSEL  = 0; // 8-bit data, no parity // MUST
    U2MODEbits.STSEL  = 0; // 1 stop bit// MUST
    U2MODEbits.ABAUD  = 0; // No auto baud 
    U2MODEbits.LPBACK = 0; // No loopback mode
    U2STA             = 0;
    U2STAbits.UTXEN   = 1; // Enable transmit
	U2STAbits.URXISEL = 0; // Interrupt after one RX character is received; 
}

void UARTTest(void)
{
    printf("DEBUG Hello 1 World Clock details COSC (%x) NOSC = (%x) \n", OSCCONbits.COSC, OSCCONbits.NOSC); 
        DelayInSeconds(2);

}

double GetFloatArg(void)
{
	double parm;
	char ch;
	char buf[12];
	int i = 0;

	/* Print on the UART2 Need more work to simulate interrupt driven */
	for(i=0; i < 12; i++) {buf[i]=0; }
	while ((ch=GetInp()) == ' ') 
			; // Skip Spaces
	i = 0;
	buf[i++] = ch; // Collect  number entered
	while (((ch=GetInp()) != ' ') && (ch != '\r'))
		buf[i++] = ch; // Collect number entered

	sscanf(buf, "%lf", &parm);

	return(parm);
}

void printfBT(char *buf )
{
	int i;
	for (i=0; i < strlen(buf); i++) {
    	while (U2STAbits.TRMT  == 0)
			; // Wait till the transmit buffer is empty
		U2Tx(buf[i]);
	}
}

unsigned char GetBTInput(void)
{
	char ch;
	/* Print on the UART2 Need more work to simulate interrupt driven */
	while (1) {
		char *buf = "\n\rEnter Command:  ";

		crk_bzero(btbuf, BTBUF); sprintf(btbuf, "%s", buf);printfBT(btbuf);
		ch = U2Rx();
		U2Tx(ch); // Echo back what you read
		return(toupper(ch)); // All commands are single character ones
	}
}

void ParseCmd(int cmd)
{
	parse_function[cmd].func();
}

void Parser(void)
{
	unsigned char cmd; 
	exit_parser = 0;
	while (1) {
		Help();
		cmd = GetBTInput();
		ParseCmd(cmd);
		U2Tx('\n');
		U2Tx('\r');
		if(exit_parser == 1) {
			return;
		}
	}
}

void NotImplemented(void)
{
	printfBT("Not Implemented\n");
}


void Ignition(void)
{
	double time;
	int seconds;
	int milliseconds;

	printf("\n\rDEBUG distance = (%f)", rc_dist2target);
	printf("DEBUG Servo test FWD !!! P2TPER = (%d) P2DC1 = (%d) rc_fwd = (%d) \n", P2TPER, P2DC1, rc_fwd);
	TimeOfTravel(rc_dist2target, &time, &seconds, &milliseconds);

	
	Stop(); // Always stop before moving forward ... otherwise the car does not work
	P2DC1 = rc_fwd; 
	P2TCONbits.PTEN = 1;
 	DelayInSeconds(seconds);
	delay_ms(milliseconds);
	Stop(); 
}
void ExitParser(void)
{
	exit_parser = 1;
	crk_bzero(btbuf, BTBUF); sprintf(btbuf, "%s","\n\r"); printfBT(btbuf);
}
void SystemHalt(void)
{
	crk_bzero(btbuf, BTBUF); sprintf(btbuf, "%s","\n\r"); printfBT(btbuf);
	exit(0);
}
void Help(void)
{
	int i;
	for(i=0; i < CMDS; i++) {
		if ( parse_function[i].func != NotImplemented) {
			crk_bzero(btbuf, BTBUF); sprintf(btbuf, "%s", parse_function[i].help); printfBT(btbuf);
		}
	}
}

void Trace(void)
{
	boolean status;
	log_data = 1;
	nmea_rec gps_record;
	crk_bzero(gps_record.raw,MAXLINELENGTH);
	
	char buf[MAXLINELENGTH];

	crk_bzero(buf, MAXLINELENGTH);
	status = GetGPSLine(buf);
	if (status == false) {
		printfBT("DEBUG Could not track satellite or get GPRMC or GGPA records. Try later\n");
		return;
	}
	printfBT(buf);
	status = false;
	status = GetGPRMCorGPGGA(buf, &gps_record);
	if (status == false) {
		printfBT("Could not find GPRMC or GGPA records. Try later\n");
	} else {
		printfBT("Successfully found GPS record of interest\n");
		printfBT(gps_record.raw);
	}
	return;
}

void GPSFix(void)
{
	boolean status;
	log_data = 1;
	int i;
	int max_tries = 4; // Pick some number :(
	
	char buf[MAXLINELENGTH];

	for (i =0; i < max_tries; i++) { 
		crk_bzero(buf, MAXLINELENGTH);
		status = GetGPSLine(buf);
		if (status == false) {
			printfBT("DEBUG Could not track satellite or get GPRMC or GGPA records. Try later\n");
			return;
		}
		printfBT(buf);
		status = false;
		crk_bzero(rc_src.raw,MAXLINELENGTH);
		status = GetGPRMCorGPGGA(buf, &rc_src);
		if (status == true) {
			if(rc_src.fix == true || rc_src.fixquality != 0) { 
				//=> We found record with a fix ='A' for GPRMC, qual = 1 for GPGGA
				printfBT("GPSFIX: Successfully found GPS record of with fix.\n");
				printfBT("GPS Raw data:" );
				printfBT(rc_src.raw);
				printfBT("\n\r");
				crk_bzero(buf, 128); sprintf(buf, "    Latitude = (%lf) Longitude = (%lf)\n\r",rc_src.latitude2, rc_src.longitude2);printfBT(buf);
				
				return;
			}
		}
	}

	printfBT("Could not find GPS record with a fix. Please try later \n");
	return;
}


unsigned char GetInp(void)
{
	unsigned char ch;
	ch = U2Rx();
	U2Tx(ch); // Echo back what you read
	return(ch);
}

void GetDestination(void)
{
	double latitude;
	double longitude;
	char ch;
	char lat, lon;
	char buf1[12];
	char buf2[12];
	char buf[128];
	int i = 0;

	/* Print on the UART2 Need more work to simulate interrupt driven */
	for(i=0; i < 12; i++) {buf1[i]=0; buf2[i] = 0;}
	while ((ch=GetInp()) == ' ') 
		;
	i = 0;
	buf1[i++] = ch;
	while ((ch=GetInp()) != ' ') {
		buf1[i++] = ch; // Collect longitude number entered
	}
	while ((ch=GetInp()) == ' ') {
		;
	}
	lat = toupper(ch);	
	while ((ch=GetInp()) == ' ') 
		; 
	i = 0;
	buf2[i++] = ch;
	while (((ch=GetInp()) != ' ') && (ch != '\r'))
		buf2[i++] = ch; // Collect latitude number entered
	while ((ch=GetInp()) == ' ') 
			; // Skip Spaces

	lon = toupper(ch);	
	sscanf(buf1, "%lf", &latitude);
	sscanf(buf2, "%lf", &longitude);

	rc_dst.latitude2 =  latitude;
	rc_dst.lat =  lat;
	rc_dst.longitude2 =  longitude;
	rc_dst.lon =  lon;

	if(lat == 'S')
		rc_dst.latitude2 =  (-1.0) * latitude;

	if(lon == 'W')
		rc_dst.longitude2 =  (-1.0) * longitude;

    rc_dist2target =  distance2(rc_src.latitude2, rc_src.longitude2,
            rc_dst.latitude2, rc_dst.longitude2,'M'); //  distance meters.

	crk_bzero(buf, 128); sprintf(buf, "\n\rSource: Latitude = (%lf) (%lf) ", rc_src.latitude2, rc_src.longitude2);printfBT(buf);
	crk_bzero(buf, 128); sprintf(buf, "\n\rDestination: Latitude = (%lf) (%lf) ", rc_dst.latitude2, rc_dst.longitude2);printfBT(buf);
	crk_bzero(buf, 128); sprintf(buf, "\n\rDstance to travel = (%lf meters)",rc_dist2target);printfBT(buf);

    if (rc_dist2target <= rc_proximity_distance) {
            printf("Reached Destination with in (%lf)\n",rc_proximity_distance);
			crk_bzero(buf, 128); 
			sprintf(buf, "\n\rReached Destination with in (%lf)\n",
						rc_proximity_distance);
			printfBT(buf);
    }

}

void ParserInit(void)
{
	int i;
	for(i=0; i < CMDS; i++) {
		parse_function[i].func = NotImplemented;
	}

	parse_function['I'].func = Ignition; 
			strcpy(parse_function['I'].help,"	I: Turn on Ignition to run the Car\n\r");
	parse_function['F'].func = MoveFwd;
			strcpy(parse_function['F'].help,"	F <n> : Move Forward the Car by 'n' meters\n\r");
	parse_function['B'].func = MoveBack;
			strcpy(parse_function['B'].help,"	B <n> : Move Back the Car by 'n' meters\n\r");
	parse_function['S'].func = Stop;
			strcpy(parse_function['S'].help,"	S : Stop the Car\n\r");

	parse_function['L'].func = TurnLeft;
			strcpy(parse_function['L'].help,"	L <n> : Turn the Car towards left by <n> degress ( 0 - 90)\n\r");
	parse_function['R'].func = TurnRight;
			strcpy(parse_function['R'].help,"	R <n> : Turn the Car towards right by <n> degress ( 0 - 90)\n\r");
	parse_function['M'].func = Middle;
			strcpy(parse_function['M'].help,"	M <n> : Turn the Car to Middle\n\r");

	parse_function['H'].func = Help;
			strcpy(parse_function['H'].help,"	H  : Help \n\r");
	parse_function['?'].func = Help;
			strcpy(parse_function['?'].help,"	?  : Help \n\r");
	parse_function['T'].func = Trace;
			strcpy(parse_function['T'].help,"	T  : Start tracing the car\n\r");
	parse_function['X'].func = Trace;
			strcpy(parse_function['X'].help,"	X  : Stop logging\n\r");

	parse_function['G'].func = GPSFix;
			strcpy(parse_function['G'].help,"	G  : Get GPS co-ordinates of Car\n\r");
	parse_function['D'].func = GetDestination;
			strcpy(parse_function['D'].help,"	D  : <longitude> <latitude>: Destination co-ordinates <latitude> <longitude>\n\r");
	parse_function['E'].func = ExitParser;
			strcpy(parse_function['E'].help,"	E  : Exit and continue\n\r");
	parse_function['Z'].func = SystemHalt;
			strcpy(parse_function['Z'].help,"	Z  : Halt the Car and reset system\n\r");
	
}

boolean GetGPSLine(char *buffer)
{
	int i = 0;
	int j = 0;
	char buf[MAXLINELENGTH*2];

    printfBT("\n\rTesting GPS ... if this you do not see any output in 60 secomds ... Reset the system\r\n");
	U1STAbits.URXISEL = 0;
	U1STAbits.FERR = 0;
	U1STAbits.OERR = 0;
	j = 0;
	while (1) {
	/* Get the data */
		if(U1STAbits.URXDA == 1) {
			buf[i++]  = U1RXREG;
			if(i == MAXLINELENGTH) {
				buf[MAXLINELENGTH-1] = 0;
				strcpy(buffer, buf);
				return true;
			}
		}
	}
	return(false);
}

// This is our GPS library

double deg2rad(double deg) {
	return (deg * pi / 180);
}

double rad2deg(double rad) {
	return (rad * 180 / pi);
}


//Note : I see the definition as in distance() on the ionternel, this
//       formula for some reason is not at all accurate for small distances
//       I used proper formula and defined distance2() as a new function
//       I will keep both and experiment with time.
double distance2(double lat1, double lon1, double lat2, double lon2, char unit) {

	double d1, d2, pi1, pi2, la1, la2, dist;
	pi1 = deg2rad(lat1);
	pi2 = deg2rad(lat2);
	la1 = deg2rad(lon1);
	la2 = deg2rad(lon2);

	d1 = sin((pi2-pi1)/2)*sin((pi2-pi1)/2) + cos(pi1)*cos(pi2)*sin((la2-la1)/2) * sin((la2-la1)/2);

	d2 =sqrt(d1);

	dist = 2 * 6371000.0 * asin(d2);
printf("%s: src: lat(%lf) lon(%lf)  dst: lat(%lf)  lon(%lf)  returns (%lf)\n", __FUNCTION__, lat1, lon1, lat2, lon2, dist);
  	return (dist);
}


double distance(double lat1, double lon1, double lat2, double lon2, char unit) {
  	double theta, dist;
  	theta = lon1 - lon2;
  	dist = sin(deg2rad(lat1)) * sin(deg2rad(lat2)) + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * cos(deg2rad(theta));
  	dist = acos(dist);
  	dist = rad2deg(dist);
  	dist = dist * 60 * 1.1515;
  	switch(unit) {
    	case 'M':
     	 	dist = dist * 1.609344 * 1000;
     	 	break;
    	case 'K':
      		dist = dist * 1.609344;
      		break;
    	case 'N':
      		dist = dist * 0.8684;
     	 	break;
  	}
  	return (dist);
}

// I probably will not need the one below. If needed I will have to
//  print this to BlueTooth device using printfBT() function
//
void gps_print_NMEA_line(nmea_rec *record)
{
	char *ptr = record->raw;
	printf("RAW = (%s)\n", ptr);
    printf("\nTime: ");
    printf("%d", record->hour); printf(":");
    printf("%d", record->minute); printf(":");
    printf("%d",record->seconds); printf(".");
    printf("%d\n",record->milliseconds);
    printf("Date: ");
    printf("%d",record->day); printf("/");
    printf("%d",record->month);printf("/20");
    printf("%d\n", record->year);
    printf("Fix: "); printf("%d",(int)record->fix);
    printf(" quality: "); printf("%d\n", (int)record->fixquality); 
    if (record->fix) {
      printf("Location:\n");
      printf("  Latitude2 (%lf):  Degrees:%f Minutes = %f",record->latitude2, record->latitudeDegrees2, record->latitudeMinutes2); printf("%c\n",record->lat);
      printf("  Longitude2 (%lf):  Degrees:%f Minutes = %f",record->longitude2, record->longitudeDegrees2, record->longitudeMinutes2); printf("%c\n",record->lat);
      
      printf("Speed (knots): "); printf("%f\n", record->speed);
      printf("Angle: "); printf("%f\n",record->angle);
      printf("Altitude: "); printf("%f\n",record->altitude);
      printf("Satellites: "); printf("%d\n", record->satellites);
    }
}

void GetLatitude(char *p, double *latitude, double* degrees, double *minutes)
{  
	char *p1, *p2;
	char buf[12];
	char buf2[12];
	int i;
	int ft2;
    p1 = p;
    char  c2;
    p2 = strchr(p, ',');
    int n = p2-p1;
    for(i=0; i < n; i++) {
		buf[i] = p[i];
    }
	
    buf[i] = 0;
	
// First work on latitude
    buf2[0] = buf[0];
    buf2[1] = buf[1];
    buf2[2] = '.';
    buf2[3] = buf[2];
    buf2[4] = buf[3];
    buf2[5] = buf[5];
    buf2[6] = buf[6];
    buf2[7] = buf[7];
    buf2[8] = buf[8];
    buf2[9] = 0;
sscanf(buf2, "%lf", latitude); 
	
	c2 = buf[2];
	buf[2] = 0;
	sscanf(buf, "%d", &ft2);
	*degrees = (double)ft2;
	buf[0] = '0'; buf[1] = '0'; buf[2] = c2;
	sscanf(buf+2, "%lf", minutes);
printf("DEBUG %s: buf= (%s) latitude2  = (%lf)\n", __FUNCTION__, buf2,  *latitude);

}

void GetLongitude(char *p, double *longitude, double* degrees, double *minutes)
{  
	char *p1, *p2;
	char buf[12];
	char buf2[12];
	int i;
	int ft2;
    p1 = p;
    char c3;
    p2 = strchr(p, ',');
    int n = p2-p1;
    for(i=0; i < n; i++)
	buf[i] = p[i];
    buf[i] = 0;

// First work on longitude
    buf2[0] = buf[0];
    buf2[1] = buf[1];
    buf2[2] = buf[2];
    buf2[3] = '.';
    buf2[4] = buf[3];
    buf2[5] = buf[4];
    buf2[6] = buf[6];
    buf2[7] = buf[7];
    buf2[8] = buf[8];
    buf2[9] = buf[9];
    buf2[10] = 0;
	
sscanf(buf2, "%lf", longitude); 

	c3 = buf[3];
	buf[3] = 0;
	sscanf(buf, "%d", &ft2);
	*degrees = (double)ft2;
	buf[0] = '0'; buf[1] = '0'; buf[2] = 0; buf[3]= c3;
	sscanf(buf+3, "%lf", minutes);

printf("DEBUG %s: buf= (%s) longitude2  = (%lf)\n", __FUNCTION__, buf2,  *longitude);

}

boolean parse_NMEA(nmea_rec *rec) {
  // do checksum check
    uint8_t i;
  uint8_t hour, minute, seconds, year, month, day;
  uint16_t milliseconds = 0;
  double latitude, longitude;
  double latitude2, longitude2;
  int32_t latitude_fixed, longitude_fixed;
  double latitudeDegrees, longitudeDegrees;
  double latitudeDegrees1, longitudeDegrees1;
  double latitudeMinutes, longitudeMinutes;
  double latitudeDegrees2, longitudeDegrees2;
  double latitudeMinutes2, longitudeMinutes2;
  double altitude = 0.0;
  double  geoidheight = 0.0;
  double speed, angle, magvariation, HDOP;
  char lat, lon, mag;
  boolean fix;
  uint8_t fixquality, satellites;
  char gpa[8];

  hour= minute =  seconds =  year =  month =  day = 0;
  latitude_fixed =  longitude_fixed = 0;
  latitude =  longitude = 0.0;
  latitude2 =  longitude2 = 0.0;
  latitudeDegrees =  longitudeDegrees = 0.0;
  latitudeDegrees1 =  longitudeDegrees1 = 0.0;
  latitudeMinutes =  longitudeMinutes = 0.0;
  latitudeDegrees2 =  longitudeDegrees2 = 0.0;
  latitudeMinutes2 =  longitudeMinutes2 = 0.0;
  altitude = 0.0;
  geoidheight = 0.0;
  speed =  angle =  magvariation =  HDOP = 0.0;
  fix = 0;
  fixquality =  satellites = 0;
  lat =  lon =  mag = 0;

  char *nmea = rec->raw;

  strncpy(gpa, nmea, 6);
  gpa[6] = 0; gpa[7] = 0;
  if((strncmp(gpa, "$GPRMC",6) != 0) && (strncmp(gpa, "$GPGGA",6) != 0)) {
	// In this case we are not interested at all
	printf("Found uin-interesting (%s)\n", gpa);
	rec->valid = false;
	return false;
  }

  // first look if we even have one
  if (nmea[strlen(nmea)-4] == '*') {
    uint16_t sum = parseHex(nmea[strlen(nmea)-3]) * 16;
    sum += parseHex(nmea[strlen(nmea)-2]);
    
    // check checksum 
    for (i=2; i < (strlen(nmea)-4); i++) {
      sum ^= nmea[i];
    }
    if (sum != 0) {
      // bad checksum :(
	  printf("DEBUG Bad checksum for (%s)\n", gpa);
	  rec->valid = false;
      return false;
    }
  }
  int32_t degree;
  long minutes;
  char degreebuff[10];
  // look for a few common sentences
  if (strstr(nmea, "$GPGGA")) {
printf("DEBUG Found GPGGA\n");
    // found GGA
    char *p = nmea;
    // get time
    p = strchr(p, ',')+1;
    double timef = atof(p);
    uint32_t time = timef;
    hour = time / 10000;
    minute = (time % 10000) / 100;
    seconds = (time % 100);

    milliseconds = (uint16_t)fmod(timef, 1.0) * 1000;

    // parse out latitude
    p = strchr(p, ',')+1;
	GetLatitude(p, &latitude2, &latitudeDegrees2, &latitudeMinutes2);
    if (',' != *p)
    {
      strncpy(degreebuff, p, 2);
      p += 2;
      degreebuff[2] = '\0';
      degree = atol(degreebuff) * 10000000;
      strncpy(degreebuff, p, 2); // minutes
      p += 3; // skip decimal point
      strncpy(degreebuff + 2, p, 4);
      degreebuff[6] = '\0';
//printf("DEBUG degree buf = (%s)\n", degreebuff);
      minutes = 50 * atol(degreebuff) / 3;
      latitude_fixed = degree + minutes;
      latitude = degree / 100000 + minutes * 0.000006F;
      latitudeDegrees =  (int)(latitude-100*(int)(latitude/100))/60.0;
      latitudeDegrees += (int)(latitude/100);
    }
    
    p = strchr(p, ',')+1;
    if (',' != *p)
    {
      if (p[0] == 'S') latitudeDegrees *= -1.0;
      if (p[0] == 'N') lat = 'N';
      else if (p[0] == 'S') lat = 'S';
      else if (p[0] == ',') lat = 0;
      else {
		rec->valid = false;
		return false;
	  }
    }
    
    // parse out longitude
    p = strchr(p, ',')+1;
	GetLongitude(p, &longitude2, &longitudeDegrees2, &longitudeMinutes2);
    if (',' != *p)
    {
      strncpy(degreebuff, p, 3);
      p += 3;
      degreebuff[3] = '\0';
      degree = atol(degreebuff) * 10000000;
      strncpy(degreebuff, p, 2); // minutes
      p += 3; // skip decimal point
      strncpy(degreebuff + 2, p, 4);
      degreebuff[6] = '\0';
      minutes = 50 * atol(degreebuff) / 3;
      longitude_fixed = degree + minutes;
      longitude = degree / 100000 + minutes * 0.000006F;
      longitudeDegrees = (int)(longitude-100*(int)(longitude/100))/60.0;
      longitudeDegrees += (int)(longitude/100);

		longitudeMinutes = modf(longitude, &longitudeDegrees1);
		longitudeDegrees1 = longitudeDegrees1/100;
		longitudeMinutes = 100*longitudeMinutes;
    }
    
    p = strchr(p, ',')+1;
    if (',' != *p)
    {
//      if (p[0] == 'W') longitudeDegrees *= -1.0;
      if (p[0] == 'W') lon = 'W';
      else if (p[0] == 'E') lon = 'E';
      else if (p[0] == ',') lon = 0;
      else {
		rec->valid = false;
		return false;
	  }
    }
    
    p = strchr(p, ',')+1;
    if (',' != *p)
    {
      fixquality = atoi(p);
    }
    
    p = strchr(p, ',')+1;
    if (',' != *p)
    {
      satellites = atoi(p);
    }
    
    p = strchr(p, ',')+1;
    if (',' != *p)
    {
      HDOP = atof(p);
    }
    
    p = strchr(p, ',')+1;
    if (',' != *p)
    {
      altitude = atof(p);
    }
    
    p = strchr(p, ',')+1;
    p = strchr(p, ',')+1;
    if (',' != *p)
    {
      geoidheight = atof(p);
    }
  rec->hour =  hour; 
  rec->minute =  minute; 
  rec->seconds =  seconds; 
  rec->year =  year; 
  rec->month = month; 
  rec->day = day;
  rec->milliseconds = milliseconds;
  rec->latitude =  latitude;
  rec->longitude =  longitude;

  if (lat == 'S')
  	rec->latitude2 =  latitude2 * -1.0;
  else
  	rec->latitude2 =  latitude2;

  if (lon == 'W')
  	rec->longitude2 =  longitude2 * -1.0;
  else
  	rec->longitude2 =  longitude2;

  rec->latitude_fixed =  latitude_fixed;
  rec->longitude_fixed = longitude_fixed;

  rec->latitudeDegrees =  latitudeDegrees;
  rec->longitudeDegrees =  longitudeDegrees;

  rec->latitudeDegrees1 =  latitudeDegrees1;
  rec->latitudeMinutes =  latitudeMinutes;
  rec->longitudeDegrees1 =  longitudeDegrees1;
  rec->longitudeMinutes =  longitudeMinutes;
  rec->latitudeDegrees2 =  latitudeDegrees2;
  rec->latitudeMinutes2 =  latitudeMinutes2;
// printf("  Latitudexx:  Degrees:%d Minutes = %f",rec->latitudeDegrees2, rec->latitudeMinutes2); 
  rec->longitudeDegrees2 =  longitudeDegrees2;
  rec->longitudeMinutes2 =  longitudeMinutes2;

  rec->geoidheight =  geoidheight;
  rec->altitude =  altitude;
  rec->speed =  speed;
  rec->angle =  angle;
  rec->magvariation =  magvariation;
  rec->HDOP =  HDOP;
  rec->lat =  lat;
  rec->lon =  lon;
  rec->mag = mag;
  rec->fix =  fix;
  rec->fixquality =  fixquality;
  rec->satellites =  satellites;
  rec->valid = true;
    return true;
	 
  }
  if (strstr(nmea, "$GPRMC")) {
   // found RMC
    char *p = nmea;
    // get time
    p = strchr(p, ',')+1;
    double timef = atof(p);
    uint32_t time = timef;
    hour = time / 10000;
    minute = (time % 10000) / 100;
    seconds = (time % 100);
printf("DEBUG Found GPRMC !!!\n");

    milliseconds = fmod(timef, 1.0) * 1000;

    p = strchr(p, ',')+1;
    // Serial.println(p);
    if (p[0] == 'A') 
      fix = true;
    else if (p[0] == 'V')
      fix = false;
    else {
	  rec->valid = false;
      return false;
    }

    // parse out latitude
    p = strchr(p, ',')+1;
	GetLatitude(p, &latitude2, &latitudeDegrees2, &latitudeMinutes2);
    if (',' != *p)
    {
      strncpy(degreebuff, p, 2);
      p += 2;
      degreebuff[2] = '\0';
      long degree = atol(degreebuff) * 10000000;
      strncpy(degreebuff, p, 2); // minutes
      p += 3; // skip decimal point
      strncpy(degreebuff + 2, p, 4);
      degreebuff[6] = '\0';
      long minutes = 50 * atol(degreebuff) / 3;
      latitude_fixed = degree + minutes;
      latitude = degree / 100000 + minutes * 0.000006F;
      latitudeDegrees = (latitude-100*(int)(latitude/100))/60.0;
      latitudeDegrees += (int)(latitude/100);


	latitudeMinutes = modf(latitude, &latitudeDegrees1);
	latitudeDegrees1 = latitudeDegrees1/100;
	latitudeMinutes = 100*latitudeMinutes;


    }
    
    p = strchr(p, ',')+1;
    if (',' != *p)
    {
//      if (p[0] == 'S') latitudeDegrees *= -1.0;
      if (p[0] == 'N') lat = 'N';
      else if (p[0] == 'S') lat = 'S';
      else if (p[0] == ',') lat = 0;
      else {
		rec->valid = false;
		return false;
	  }
    }
    
    // parse out longitude
    p = strchr(p, ',')+1;
    GetLongitude(p,&longitude2, &longitudeDegrees2, &longitudeMinutes2); 
    if (',' != *p)
    {
      strncpy(degreebuff, p, 3);
      p += 3;
      degreebuff[3] = '\0';
      degree = atol(degreebuff) * 10000000;
      strncpy(degreebuff, p, 2); // minutes
      p += 3; // skip decimal point
      strncpy(degreebuff + 2, p, 4);
      degreebuff[6] = '\0';
      minutes = 50 * atol(degreebuff) / 3;
      longitude_fixed = degree + minutes;
      longitude = degree / 100000 + minutes * 0.000006F;
      longitudeDegrees = (longitude-100*(int)(longitude/100))/60.0;
      longitudeDegrees += (int)(longitude/100);

		longitudeMinutes = modf(longitude, &longitudeDegrees1);
		longitudeDegrees1 = longitudeDegrees1/100;
		longitudeMinutes = 100*longitudeMinutes;
    }
    
    p = strchr(p, ',')+1;
    if (',' != *p)
    {
//      if (p[0] == 'W') longitudeDegrees *= -1.0;
      if (p[0] == 'W') lon = 'W';
      else if (p[0] == 'E') lon = 'E';
      else if (p[0] == ',') lon = 0;
      else  {
	    rec->valid = false;
		return false;
	  }
    }
    // speed
    p = strchr(p, ',')+1;
    if (',' != *p)
    {
      speed = atof(p);
    }
    
    // angle
    p = strchr(p, ',')+1;
    if (',' != *p)
    {
      angle = atof(p);
    }
    
    p = strchr(p, ',')+1;
    if (',' != *p)
    {
      uint32_t fulldate = atof(p);
      day = fulldate / 10000;
      month = (fulldate % 10000) / 100;
      year = (fulldate % 100);
    }
    // we dont parse the remaining, yet!
  rec->hour =  hour; 
  rec->minute =  minute; 
  rec->seconds =  seconds; 
  rec->year =  year; 
  rec->month = month; 
  rec->day = day;
  rec->milliseconds = milliseconds;
  rec->latitude =  latitude;
  rec->longitude =  longitude;
  if (lat == 'S')
  	rec->latitude2 =  latitude2 * -1.0;
  else
  	rec->latitude2 =  latitude2;

  if (lon == 'W')
  	rec->longitude2 =  longitude2 * -1.0;
  else
  	rec->longitude2 =  longitude2;

  rec->latitude_fixed =  latitude_fixed;
  rec->longitude_fixed = longitude_fixed;
  rec->latitudeDegrees =  latitudeDegrees;
  rec->longitudeDegrees =  longitudeDegrees;
  rec->latitudeDegrees1 =  latitudeDegrees1;
  rec->longitudeDegrees1 =  longitudeDegrees1;
  rec->latitudeDegrees2 =  latitudeDegrees2;
  rec->longitudeDegrees2 =  longitudeDegrees2;
  rec->latitudeMinutes =  latitudeMinutes;
  rec->longitudeMinutes =  longitudeMinutes;
  rec->latitudeMinutes2 =  latitudeMinutes2;
  rec->longitudeMinutes2 =  longitudeMinutes2;

  rec->geoidheight =  geoidheight;
  rec->altitude =  altitude;
  rec->speed =  speed;
  rec->angle =  angle;
  rec->magvariation =  magvariation;
  rec->HDOP =  HDOP;
  rec->lat =  lat;
  rec->lon =  lon;
  rec->mag = mag;
  rec->fix =  fix;
  rec->fixquality =  fixquality;
  rec->satellites =  satellites;
  rec->valid = true;
    return true;
  }

  rec->valid = false;
  return false;
}

// read a Hex value and return the decimal equivalent
uint8_t parseHex(char c) 
{
    if (c < '0')
      return 0;
    if (c <= '9')
      return c - '0';
    if (c < 'A')
       return 0;
    if (c <= 'F')
       return (c - 'A')+10;
    // if (c > 'F')
    return 0;
}

boolean GetGPRMCorGPGGA(char *buf,  nmea_rec *rec )
{
	int i = 0;
	int j = 0;
	unsigned char ch;
	char *raw = rec->raw;

	// Read between two $ signs. 
    while ((ch=buf[i++]) != '$')
            ; // Skip Spaces
    raw[j++] = ch;
    while ((ch=buf[i++]) != '$') {
        raw[j++] = ch; // Collect record data
	}
	raw[j++] = 0;
	if((strncmp(raw, "$GPRMC", 6) == 0) || 
				(strncmp(raw, "$GPGGA", 6) == 0)) {
printf("(%s) DEBUG !!! FOund record (%s)\n", __FUNCTION__, raw);
		parse_NMEA(rec);
		if (rec->valid == true) {
			gps_print_NMEA_line(rec);
			return (true);
		}
	}
	return (false);
}

/* Basic hard ware wiring:
	UART1 : 
		Tx:Used for printf ( via tty serial cable for debug purpose )
		Rx: GPS Tx is connected to this Rc, so that UART1 can receive and
		    process GPS data
	UART2:
		Tx: Transmit to Bluetooth 
		Rx: Reveivce ( Parser input )  from Bluetooth.
	SPI:
		INterface is implemennted but not tested with Magnetometer.
		SPI worked weill with Flash and SRAM

   Software: Main routine and alogorithm
		- Initialize the hardware
		- Initialize GPS ( To get current co-ordinates of the Car PointA)
		- Initialize Bluetooth ( To get information on where the target is )
		- Initialize magnetometer ( To get the alignment with North pole )
		- Initialize Servo Motor ( To turn and drive to the  target Point B)
		- Note: if needed we can add optical sensor to see the way to adjust.

	Initialize high level routines 
		- Set init values such as : When  the car needs to stop etc
		- Get current location of the car ( From GPS  Point A)
		- Get the location of the target ( from Bluetoth Point B )
		- Get the current of alignment of the car  to Northpole (Magnetometer)
		- Calculate straight line distance between A & B
		- Calculate the angle we need to turn to face the target
		- Start the car to move towards destination
		- Execute parser in loop.

		- Parser exists/Halts the system.

*/
int main(void) 
{
	AD1PCFGL = 0xFFFF; // Make all Analog as Digital pinouts
	
	ParserInit();
	UART1Init();
	UART2Init();
	printfBT("\n\rIf You see this, that means BlueTooth Initialized\n");
	SPIInit();
	ServoInit();
	printfBT("\n\rSystem Initialized and Read to go\n");
	printfBT("\n\rBasic testing in progress\n");

	BlueToothTest();
// SPITestFlash(); : // Not needed for Car
//	SPITestMagnet(); // Not implemented
	Parser();
					// TBD we can do much more later.	
	
	fflush(stdout);
    return 0;
}
