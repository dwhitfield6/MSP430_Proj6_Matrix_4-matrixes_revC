#include <msp430.h> 

#define CS BIT0  //1.5 is SPI clock 1.7 is MOSI
#define MOSI BIT7  //1.5 is SPI clock 1.7 is MOSI
#define SCLK BIT5  //1.5 is SPI clock 1.7 is MOSI
#define LED1 BIT4  //1.5 is SPI clock 1.7 is MOSI
#define UART_TXD 0x02                                  // TXD on P1.1 (Timer0_A.OUT0)
#define UART_RXD 0x04                                  // RXD on P1.2 (Timer0_A.CCI1A)

#define UART_TBIT_DIV_2 (1000000 / (9600 * 2))         // Conditions for 9600 Baud SW UART, SMCLK = 1MHz
#define UART_TBIT (1000000 / 9600)

void TimerA_UART_tx(unsigned char byte);               // Function prototypes
void TimerA_UART_print(char *string);

void Init_MAX7219(void);
void SPI_Init(void); //SPI initialization
void SPI_Write(unsigned char);
void SPI_Write2(unsigned char, unsigned char);
void SPI_Write3(unsigned char, unsigned char, unsigned char);

int rxBuffer[40];
int resetcount =1005;
int iiii =0;
int speedchange =0;
int speed=400;
int scrollspeed =600;
int print =0;
int speedloop =990;
int digits =1;
unsigned int ii,iii,iiiii,i,j,ji,jji;
unsigned int txData;

unsigned const char dispFRONT[28][8]={
  {
    0	,	0	,	0	,	0	,	0	,	0	,	0	,	0	  }
  ,	//	space
  {
    31	,	40	,	72	,	136	,	72	,	40	,	31	,	0	  }
  ,	//	A
  {
    255	,	145	,	145	,	145	,	145	,	145	,	110	,	0	  }
  ,	//	B
  {
    60	,	66	,	129	,	129	,	129	,	129	,	129	,	0	  }
  ,	//	C
  {
    255	,	129	,	129	,	129	,	66	,	60	,	0	,	0	  }
  ,	//	D
  {
    255	,	145	,	145	,	145	,	145	,	129	,	129	,	0	  }
  ,	//	E
  {
    255	,	144	,	144	,	144	,	144	,	128	,	128	,	0	  }
  ,	//	F
  {
    60	,	66	,	129	,	129	,	133	,	71	,	4	,	0	  }
  ,	//	G
  {
    255	,	8	,	8	,	8	,	8	,	8	,	255	,	0	  }
  ,	//	H
  {
    129	,	129	,	129	,	255	,	129	,	129	,	129	,	0	  }
  ,	//	I
  {
    130	,	129	,	129	,	129	,	255	,	128	,	128	,	0	  }
  ,	//	J
  {
    255	,	16	,	40	,	68	,	130	,	1	,	0	,	0	  }
  ,	//	K
  {
    255	,	1	,	1	,	1	,	1	,	1	,	1	,	0	  }
  ,	//	L
  {
    255	,	64	,	32	,	16	,	32	,	64	,	255	,	0	  }
  ,	//	M
  {
    255	,	64	,	32	,	16	,	8	,	4	,	255	,	0	  }
  ,	//	N
  {
    255	,	129	,	129	,	129	,	129	,	129	,	255	,	0	  }
  ,	//	O
  {
    255	,	144	,	144	,	144	,	144	,	144	,	96	,	0	  }
  ,	//	P
  {
    254	,	130	,	130	,	134	,	130	,	131	,	254	,	0	  }
  ,	//	Q
  {
    255	,	144	,	144	,	152	,	148	,	146	,	241	,	0	  }
  ,	//	R
  {
    241	,	145	,	145	,	145	,	145	,	145	,	159	,	0	  }
  ,	//	S
  {
    128	,	128	,	128	,	255	,	128	,	128	,	128	,	0	  }
  ,	//	T
  {
    254	,	1	,	1	,	1	,	1	,	1	,	254	,	0	  }
  ,	//	U
  {
    248	,	4	,	2	,	1	,	2	,	4	,	248	,	0	  }
  ,	//	V
  {
    255	,	1	,	2	,	4	,	2	,	1	,	255	,	0	  }
  ,	//	W
  {
    65	,	34	,	20	,	8	,	20	,	34	,	65	,	0	  }
  ,	//	X
  {
    128	,	64	,	32	,	31	,	32	,	64	,	128	,	0	  }
  ,	//	Y
  {
    0	,	131	,	133	,	137	,	145	,	161	,	193	,	0	  }
  ,	//	Z
  {
    0	,	0	,	0	,	0	,	0	,	0	,	0	,	0	  }
  ,	//	space
};

unsigned const char dispBACK[28][8]={
  {
    0	,	0	,	0	,	0	,	0	,	0	,	0	,	0	  }
  ,	//	space
  {
    248	,	20	,	18	,	17	,	18	,	20	,	248	,	0	  }
  ,	//	A
  {
    255	,	137	,	137	,	137	,	137	,	137	,	118	,	0	  }
  ,	//	B
  {
    60	,	66	,	129	,	129	,	129	,	129	,	129	,	0	  }
  ,	//	C
  {
    255	,	129	,	129	,	129	,	66	,	60	,	0	,	0	  }
  ,	//	D
  {
    255	,	137	,	137	,	137	,	137	,	129	,	129	,	0	  }
  ,	//	E
  {
    255	,	9	,	9	,	9	,	9	,	1	,	1	,	0	  }
  ,	//	F
  {
    60	,	66	,	129	,	129	,	161	,	226	,	32	,	0	  }
  ,	//	G
  {
    255	,	16	,	16	,	16	,	16	,	16	,	255	,	0	  }
  ,	//	H
  {
    129	,	129	,	129	,	255	,	129	,	129	,	129	,	0	  }
  ,	//	I
  {
    65	,	129	,	129	,	129	,	255	,	1	,	1	,	0	  }
  ,	//	J
  {
    255	,	8	,	20	,	34	,	65	,	128	,	0	,	0	  }
  ,	//	K
  {
    255	,	128	,	128	,	128	,	128	,	128	,	128	,	0	  }
  ,	//	L
  {
    255	,	2	,	4	,	8	,	4	,	2	,	255	,	0	  }
  ,	//	M
  {
    255	,	2	,	4	,	8	,	16	,	32	,	255	,	0	  }
  ,	//	N
  {
    255	,	129	,	129	,	129	,	129	,	129	,	255	,	0	  }
  ,	//	O
  {
    255	,	9	,	9	,	9	,	9	,	9	,	6	,	0	  }
  ,	//	P
  {
    127	,	65	,	65	,	97	,	65	,	193	,	127	,	0	  }
  ,	//	Q
  {
    255	,	9	,	9	,	25	,	41	,	73	,	143	,	0	  }
  ,	//	R
  {
    143	,	137	,	137	,	137	,	137	,	137	,	249	,	0	  }
  ,	//	S
  {
    1	,	1	,	1	,	255	,	1	,	1	,	1	,	0	  }
  ,	//	T
  {
    127	,	128	,	128	,	128	,	128	,	128	,	127	,	0	  }
  ,	//	U
  {
    31	,	32	,	64	,	128	,	64	,	32	,	31	,	0	  }
  ,	//	V
  {
    255	,	128	,	64	,	32	,	64	,	128	,	255	,	0	  }
  ,	//	W
  {
    130	,	68	,	40	,	16	,	40	,	68	,	130	,	0	  }
  ,	//	X
  {
    1	,	2	,	4	,	248	,	4	,	2	,	1	,	0	  }
  ,	//	Y
  {
    0	,	193	,	161	,	145	,	137	,	133	,	131	,	0	  }
  ,	//	Z
  {
    0	,	0	,	0	,	0	,	0	,	0	,	0	,	0	  }
  ,	//	space
};


void SPI_Init(void) //SPI initialization
{
  P2DIR |= CS;
  P1SEL |= MOSI + SCLK;
  P1SEL2 |= MOSI + SCLK;
  UCB0CTL1 = UCSWRST;
  UCB0CTL0 |= UCMSB + UCMST + UCSYNC + UCCKPH; // 4-pin, 8-bit SPI master
  UCB0CTL1 |= UCSSEL_2;                     // SMCLK
  UCB0BR0 = 10;                          // /2
  UCB0BR1 = 0;                              //
  UCB0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**

  __enable_interrupt(); // enable all interrupts
}

void SPI_Write2(unsigned char MSB, unsigned char LSB) //SPI write one byte
{

  P2OUT &= ~CS;
  _delay_cycles(50);
  UCB0TXBUF = MSB ;
  while (UCB0STAT & UCBUSY);
  UCB0TXBUF = LSB ;
  while (UCB0STAT & UCBUSY);
  P2OUT |= CS;
}
void SPI_Write1(unsigned char MSB) //SPI write one byte
{

  P2OUT &= ~CS;
  _delay_cycles(10);
  UCB0TXBUF = MSB ;
  while (UCB0STAT & UCBUSY);
  P2OUT |= CS;
}
void SPI_Write3(unsigned char MSB,unsigned char MMSB,unsigned char LSB ) //SPI write one byte
{

  P2OUT &= ~CS;
  _delay_cycles(10);
  UCB0TXBUF = MSB ;
  while (UCB0STAT & UCBUSY);
  UCB0TXBUF = MMSB ;
  while (UCB0STAT & UCBUSY);
  UCB0TXBUF = LSB ;
  while (UCB0STAT & UCBUSY);
  P2OUT |= CS;
}

void Matrix1(unsigned char address,unsigned char data ) //SPI write one byte
{

  P2OUT &= ~CS;
  _delay_cycles(10);
  UCB0TXBUF = address ;
  while (UCB0STAT & UCBUSY);
  UCB0TXBUF = data ;
  while (UCB0STAT & UCBUSY);
  UCB0TXBUF = 0x00 ;
  while (UCB0STAT & UCBUSY);
  UCB0TXBUF = 0x00 ;
  while (UCB0STAT & UCBUSY);
  UCB0TXBUF = 0x00 ;
  while (UCB0STAT & UCBUSY);
  UCB0TXBUF = 0x00 ;
  while (UCB0STAT & UCBUSY);
  UCB0TXBUF = 0x00 ;
  while (UCB0STAT & UCBUSY);
  UCB0TXBUF = 0x00 ;
  while (UCB0STAT & UCBUSY);
  P2OUT |= CS;
}
void Matrix2(unsigned char address,unsigned char data ) //SPI write one byte
{

  P2OUT &= ~CS;
  _delay_cycles(10);
  UCB0TXBUF = 0 ;
  while (UCB0STAT & UCBUSY);
  UCB0TXBUF = 0 ;
  while (UCB0STAT & UCBUSY);
  UCB0TXBUF = address ;
  while (UCB0STAT & UCBUSY);
  UCB0TXBUF = data ;
  while (UCB0STAT & UCBUSY);
  UCB0TXBUF = 0 ;
  while (UCB0STAT & UCBUSY);
  UCB0TXBUF = 0 ;
  while (UCB0STAT & UCBUSY);
  UCB0TXBUF = 0 ;
  while (UCB0STAT & UCBUSY);
  UCB0TXBUF = 0 ;
  while (UCB0STAT & UCBUSY);
  P2OUT |= CS;
}

void Matrix3(unsigned char address,unsigned char data ) //SPI write one byte
{

  P2OUT &= ~CS;
  _delay_cycles(10);
  UCB0TXBUF = 0 ;
  while (UCB0STAT & UCBUSY);
  UCB0TXBUF = 0 ;
  while (UCB0STAT & UCBUSY);
  UCB0TXBUF = 0 ;
  while (UCB0STAT & UCBUSY);
  UCB0TXBUF = 0 ;
  while (UCB0STAT & UCBUSY);
  UCB0TXBUF = address ;
  while (UCB0STAT & UCBUSY);
  UCB0TXBUF = data ;
  while (UCB0STAT & UCBUSY);
  UCB0TXBUF = 0 ;
  while (UCB0STAT & UCBUSY);
  UCB0TXBUF = 0 ;
  while (UCB0STAT & UCBUSY);
  P2OUT |= CS;
}

void Matrix4(unsigned char address,unsigned char data ) //SPI write one byte
{

  P2OUT &= ~CS;
  _delay_cycles(10);
  UCB0TXBUF = 0 ;
  while (UCB0STAT & UCBUSY);
  UCB0TXBUF = 0 ;
  while (UCB0STAT & UCBUSY);
  UCB0TXBUF = 0 ;
  while (UCB0STAT & UCBUSY);
  UCB0TXBUF = 0 ;
  while (UCB0STAT & UCBUSY);
  UCB0TXBUF = 0 ;
  while (UCB0STAT & UCBUSY);
  UCB0TXBUF = 0 ;
  while (UCB0STAT & UCBUSY);
  UCB0TXBUF = address ;
  while (UCB0STAT & UCBUSY);
  UCB0TXBUF = data ;
  while (UCB0STAT & UCBUSY);
  P2OUT |= CS;
}

void Init_MAX7219(void)
{
  Matrix1(0x09, 0x00);       //
  Matrix1(0x0A, 0x0F);       //
  Matrix1(0x0B, 0x0F);       //
  Matrix1(0x0C, 0x01);       //
  Matrix1(0x0F, 0x0F);       //
  Matrix1(0x0F, 0x00);       //

  Matrix2(0x09, 0x00);       //
  Matrix2(0x0A, 0x0F);       //
  Matrix2(0x0B, 0x0F);       //
  Matrix2(0x0C, 0x01);       //
  Matrix2(0x0F, 0x0F);       //
  Matrix2(0x0F, 0x00);       //

  Matrix3(0x09, 0x00);       //
  Matrix3(0x0A, 0x0F);       //
  Matrix3(0x0B, 0x0F);       //
  Matrix3(0x0C, 0x01);       //
  Matrix3(0x0F, 0x0F);       //
  Matrix3(0x0F, 0x00);       //

  Matrix4(0x09, 0x00);       //
  Matrix4(0x0A, 0x0F);       //
  Matrix4(0x0B, 0x0F);       //
  Matrix4(0x0C, 0x01);       //
  Matrix4(0x0F, 0x0F);       //
  Matrix4(0x0F, 0x00);       //

}

int main(void)
{
  P1SEL |= UART_TXD + UART_RXD;
  P1DIR |= (UART_TXD);
  P2DIR |= (LED1);
  P2OUT &= ~(LED1);

  TA0CCTL0 = OUT;                                      // Set TXD Idle as Mark = '1'
  TA0CCTL1 = SCS + CM1 + CAP + CCIE;                   // Sync, Neg Edge, Capture, Int
  TA0CTL = TASSEL_2 + MC_2;                            // SMCLK, start in continuous mode


  WDTCTL = WDTPW | WDTHOLD;

  SPI_Init();
  _delay_cycles(100000);
  Init_MAX7219();
  _delay_cycles(1000);
  __enable_interrupt(); // enable all interrupts
  while(1)
  {
    resetcount++;
    if(resetcount == 5 && speedloop>200)
    {
      digits = iiii;
      for(ji=0;ji<digits;ji++)
      {
        if(rxBuffer[ji] <208 && rxBuffer[ji] > 192)
        {
          rxBuffer[ji] -=192;
        }// Store in global variable
        else if (rxBuffer[ji] <251 && rxBuffer[ji] > 239)
        {
          rxBuffer[ji]  -= 224;
        }
        else if (rxBuffer[ji] == 64)
        {
          rxBuffer[ji] = 27;
        }
      }
      for(jji=digits;jji<=39;jji++)
      {
        rxBuffer[jji] =0;
      }
      digits+=5;
      P2OUT |= LED1;
      __delay_cycles(10000);
      iiii=0;
      P2OUT &= ~LED1;

    }
    if(resetcount >scrollspeed+300)
    {
      resetcount =200;
    }

    if(speedchange ==1)
    {
    	speedchange =0;
    	speedloop =0;
    	scrollspeed = speed;
    	__delay_cycles(1000);
    }
speedloop++;
if(speedloop>1000)
{
	speedloop =990;
}
    __delay_cycles(100);
    	    if(resetcount==250)
    	    {
        for(i=1;i<9;i++)
        {
          Matrix4(i,dispBACK[rxBuffer[j]][i-1]);
        }
        if(j< 1)
        {
          for(ii=1;ii<9;ii++)
          {
            Matrix3(ii,dispFRONT[0][8-ii]);
          }
        }
        else
        {
          for(ii=1;ii<9;ii++)
          {
            Matrix3(ii,dispFRONT[rxBuffer[j-1]][8-ii]);
          }
        }
        if(j<2)
        {
          for(iii=1;iii<9;iii++)
          {
            Matrix2(iii,dispBACK[0][iii-1]);
          }
        }
        else
        {
          for(iii=1;iii<9;iii++)
          {
            Matrix2(iii,dispBACK[rxBuffer[j-2]][iii-1]);
          }
        }
        if(j<3)
        {
          for(iiiii=1;iiiii<9;iiiii++)
          {
            Matrix1(iiiii,dispFRONT[0][8-iiiii]);
          }
        }
        else
        {
          for(iiiii=1;iiiii<9;iiiii++)
          {
            Matrix1(iiiii,dispFRONT[rxBuffer[j-3]][8-iiiii]);
          }
        }
        j++;
        if(j>digits)
        	{
        	j=0;
    	    }
      }

    }

  }





#pragma vector = TIMER0_A1_VECTOR                      // Timer_A UART - Receive Interrupt Handler

__interrupt void Timer_A1_ISR(void) {

  static unsigned char rxBitCnt = 8;

  static int rxData = 0;
  __disable_interrupt(); // enable all interrupts
  switch (__even_in_range(TA0IV, TA0IV_TAIFG)) {       // Use calculated branching

  case TA0IV_TACCR1:                                 // TACCR1 CCIFG - UART RX

    TA0CCR1 += UART_TBIT;                         // Add Offset to CCRx

    if (TA0CCTL1 & CAP) {                         // Capture mode = start bit edge

      TA0CCTL1 &= ~CAP;                           // Switch capture to compare mode

      TA0CCR1 += UART_TBIT_DIV_2;                 // Point CCRx to middle of D0
    }
    else {
      rxData >>= 1;

      if (TA0CCTL1 & SCCI)                        // Get bit waiting in receive latch
        rxData |= 0x80;
      rxBitCnt--;

      if (rxBitCnt == 0) {                        // All bits RXed?
        if(iiii<40)
        {
          if(rxData == 10)
          {
        	  if( speed <3000)
        	  {
        	  speed+=100;
        	  }
        	  speedchange =1;
          }
          else if(rxData == 52)
          {
        	  if(speed> 200)
        	  {
        	  speed-=100;
        	  }
        	  speedchange =1;
          }
          else
          {
          rxBuffer[iiii] = rxData;
          iiii++;
          }
        }
        rxBitCnt = 8;                             // Re-load bit counter

        TA0CCTL1 |= CAP;                          // Switch compare to capture mode

      }
    }
    break;
  }
  if(speedchange != 1)
  {
  resetcount =0;
  }
  __enable_interrupt(); // enable all interrupts
}

