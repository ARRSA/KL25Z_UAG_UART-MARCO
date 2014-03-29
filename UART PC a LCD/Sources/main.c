#include "derivative.h" /* include peripheral declarations */


#define GPIO_PIN_MASK 0x1Fu
#define GPIO_PIN(x) (((1)<<(x & GPIO_PIN_MASK)))

#define	nBit0	0x01	//'00000001'
#define	nBit1	0x02	//'00000010'
#define	nBit2	0x04	//'00000100'
#define	nBit3	0x08	//'00001000'
#define	nBit4	0x10	//'00010000'
#define	nBit5	0x20	//'00100000'
#define	nBit6	0x40	//'01000000'
#define	nBit7	0x80	//'10000000'

//Time definitions
#define nt15_msec	3500
#define nt40_usec	100
#define ntdelay 600000

//LCD Control
#define nIns	0
#define nData	1

#define PortLCD    	GPIOD_PDOR
//Enable connected to portb_01
#define Enable_1	GPIOB_PDOR |= 0x01
#define Enable_0	GPIOB_PDOR &= 0xFE
#define RS_1   		GPIOB_PDOR |= 0x02
#define RS_0   		GPIOB_PDOR &= 0xFD


#define	Set_GPIOB_PDOR(x)	(GPIOB_PDOR |= (1 << (x-1)))

//Cursor Blink off initialization
const unsigned char InitializeLCD[5] = {0x38, 0x38, 0x38, 0x0C, 0x01};
//--------------------------------------------------------------
//Declare Prototypes
/* Functions */
void cfgPorts(void);
void initLCD(void);
void delay(long time);
void sendCode(int Code, int Data);
void InitUART(void);

unsigned char A;

int main(void)
{
	//Configure ports
	cfgPorts();
	//Initialize LCD
	initLCD();
	//Set position to print character
	InitUART();
	sendCode(nIns,0x80);
	
	for(;;)
	{
		if(UART0_S1 & 0x20)
		{
			A = UART0_D;
			sendCode(nData,A);
		}
	}
		
	return 0;
}

void cfgPorts(void)
{
	//Turn on clock for portb
	SIM_SCGC5 = SIM_SCGC5_PORTB_MASK;	
	//Turn on clock for portd
	SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;	
	////Turn on clock for portc
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
	////Turn on clock for porta
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
	
	/* Set pins of PORTB as GPIO */
	PORTB_PCR0= PORT_PCR_MUX(1);
	PORTB_PCR1= PORT_PCR_MUX(1);
	PORTB_PCR2=(0|PORT_PCR_MUX(1));
	PORTB_PCR3=(0|PORT_PCR_MUX(1));
	PORTB_PCR4=(0|PORT_PCR_MUX(1));
	PORTB_PCR5=(0|PORT_PCR_MUX(1));
	PORTB_PCR6=(0|PORT_PCR_MUX(1));
	PORTB_PCR7=(0|PORT_PCR_MUX(1));
	
	/* Set pins of PORTC as GPIO */
	PORTC_PCR0= PORT_PCR_MUX(1);
	PORTC_PCR1= PORT_PCR_MUX(1);
	PORTC_PCR2= PORT_PCR_MUX(1);
	PORTC_PCR3= PORT_PCR_MUX(1);
	
	/* Set pins of PORTD as GPIO */
	PORTD_PCR0= PORT_PCR_MUX(1);
	PORTD_PCR1= PORT_PCR_MUX(1);
	PORTD_PCR2=(0|PORT_PCR_MUX(1));
	PORTD_PCR3=(0|PORT_PCR_MUX(1));
	PORTD_PCR4=(0|PORT_PCR_MUX(1));
	PORTD_PCR5=(0|PORT_PCR_MUX(1));
	PORTD_PCR6=(0|PORT_PCR_MUX(1));
	PORTD_PCR7=(0|PORT_PCR_MUX(1));
	
	/* Set pins of PORTA as GPIO */
	PORTA_PCR1= PORT_PCR_MUX(2);
	PORTA_PCR2= PORT_PCR_MUX(2);
	
	//Initialize Ports
	GPIOB_PDOR = 0x00;
	GPIOD_PDOR = 0x00;
	GPIOA_PDOR = 0x00;

	//Configure PortB as outputs
	GPIOB_PDDR = 0xFF;
	
	//Configure PortD as outputs
	GPIOD_PDDR = 0xFF;
	
	//Configure PortC as inputs
	GPIOC_PDDR = 0x00;
}

void initLCD(void)
{
	int i;
	delay(nt15_msec);
	
	/* Send initialization instructions */
	/* Loop for sending each character from the array */
	for(i=0;i<5;i++)
	{										
		sendCode(nIns, InitializeLCD[i]);	/* send initialization instructions */			
	}
	
}

void sendCode(int Code, int Data)
{
	//Assign a value to pin RS
	/*HINT: When RS is 1, then the LCD receives a data
	when RS is 0, then the LCD receives an instruction */
	// Initialize RS and Enable with 0
	RS_0;
	Enable_0;
	//Assign the value we want to send to the LCD
	PortLCD = Data;	
	
	//We make the algorithm to establish if its an instruction we start with 0 on RS value, otherwise if its a data command we start with RS as 1;
	if (Code == nIns)
	{
		Enable_1;
		delay(nt40_usec);
		Enable_0;
		RS_0;
	}		
	else if(Code == nData)
	{
		RS_1;
		Enable_1;
		delay(nt40_usec);
		Enable_0;
		RS_0;
	}
}
void delay(long time)
{
	while (time > 0)
	{
		time--;
	}
}

void InitUART(void)
{
	 SIM_SCGC4 |= SIM_SCGC4_UART0_MASK;                                                   
		
	 /* PORTA_PCR1: ISF=0,MUX=2 */
	 PORTA_PCR1 = (uint32_t)((PORTA_PCR1 & (uint32_t)~0x01000500UL) | (uint32_t)0x0200UL);
	 /* PORTA_PCR2: ISF=0,MUX=2 */
	 PORTA_PCR2 = (uint32_t)((PORTA_PCR2 & (uint32_t)~0x01000500UL) | (uint32_t)0x0200UL); 
	
	 /* Disable TX & RX while we configure settings */
	 UART0_C2 &= ~(UART0_C2_TE_MASK); //disable transmitter
	 UART0_C2 &= ~(UART0_C2_RE_MASK); //disable receiver
	 
	 /* UART0_C1: LOOPS=0,DOZEEN=0,RSRC=0,M=0,WAKE=0,ILT=0,PE=0,PT=0 */
	 UART0_C1 = 0x00U; /* Set the C1 register */
	 /* UART0_C3: R8T9=0,R9T8=0,TXDIR=0,TXINV=0,ORIE=0,NEIE=0,FEIE=0,PEIE=0 */
	 UART0_C3 = 0x00U; /* Set the C3 register */
	 /* UART0_S2: LBKDIF=0,RXEDGIF=0,MSBF=0,RXINV=0,RWUID=0,BRK13=0,LBKDE=0,RAF=0 */
	 UART0_S2 = 0x00U; /* Set the S2 register */
	 
	 SIM_SOPT2 |= SIM_SOPT2_UART0SRC(1); //set clock source to be from PLL/FLL
	 SIM_SOPT2 |= SIM_SOPT2_CLKOUTSEL(0b100);
	 unsigned SBR = 546;//137; //Set the baud rate register, SBR = 137
	 UART0_BDH |= (~UART0_BDH_SBR_MASK) | SBR >> 8;
	 UART0_BDL |= (~UART0_BDL_SBR_MASK) | SBR;
	 
	 char OSR = 3; //set the oversampling ratio to option #3 = 4x
	 UART0_C4 &= (~UART0_C4_OSR_MASK) | OSR;
		 
	 UART0_C5 |= UART0_C5_BOTHEDGE_MASK; //enable sampling on both edges of the clock
	 UART0_C2 |= UART0_C2_TE_MASK; //enable transmitter
	 UART0_C2 |= UART0_C2_RE_MASK; //enable receiver	
}






