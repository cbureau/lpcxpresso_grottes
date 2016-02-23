/*
===============================================================================
 Name        : main.c
 Author      : 
 Version     :
 Copyright   : Copyright (C) 
 Description : main definition
===============================================================================
*/

#ifdef __USE_CMSIS
#include "LPC17xx.h"
#endif
#include "system_LPC17xx.h"
#include <cr_section_macros.h>
#include <NXP/crp.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "ethernet.h"
#include "rfid_indus_def.h"
// Variable to store CRP value in. Will be placed automatically
// by the linker when "Enable Code Read Protect" selected.
// See crp.h header for more information
__CRP const unsigned int CRP_WORD = CRP_NO_CRP ;
/* Common headers */
int main(void) {
char buff2[16];
char buff3[16];
int nb_car,NbByte,i;
LPC_SC->CLKSRCSEL = 0x00; // selection oscillateur interne
 SystemClockUpdate();
 Init();
 LPC_GPIO0->FIOSET |= RELAIS;  // RELAIS on
  Tempo(1000000);
  LPC_GPIO0->FIOCLR |= RELAIS;  // RELAIS off
  //while(1){
  buff2[0]='v';
  		 ser_Write (1,buff2,1); // envoi vers sortie
  		 Tempo(1000000);
  		 buff2[0]='V';
  		 ser_Write (1,buff2,1); // envoi vers sortie
  		 Tempo(1000000);
  		NbByte = lecture_serie(buff2);
  //}
  		 /************************************MIFARD****************************/
 	while(1) {
 		Tempo(10000);
 		NbByte=0;
 		ser_AvailChar2(&NbByte);
 		if (NbByte > 0){
 		ser_Read2(buff2, NbByte);  // lecture ethernet
 		ser_Write (1,buff2,NbByte); // envoi vers sortie
 		if (buff2[0]=='%'){
 			switch (buff2[1]){
 			case 'L' :{
 				LPC_GPIO1->FIOSET |= RELAIS;
 				break;
 			}
 			case 'l' :{
 				LPC_GPIO1->FIOCLR |= RELAIS;
 				break;
 			 			}
 			case 'A' :{
 				buff3[0]=lect_io(I0)+'0';
 				break;
 			 			}
 			case 'B' :{
 				buff3[0]=lect_io(I1)+'0';
 				break;
 			 			}
 			case 'C' :{
 				buff3[0]=lect_io(I2)+'0';
 				break;
 			 			}
 			case 'D' :{
 				buff3[0]=lect_io(I3)+'0';
 				break;
 			 			}
 			case 'E' :{
 				buff3[0]=lect_io(I4)+'0';
 				break;
 			 		}
 			}
 			ser_Write (2,buff3,1); // envoi vers ethernet
 		for(i=0;i<NbByte;i++)
 		printf("%c",buff2[i]);
 		printf("\n");
 			}
 		}
 	}
 	return 0 ;
 }

 /****************Initialisation**********************/
 void Init(void){

 	LPC_SC->PCLKSEL0 |=  (0x01<<8);	// fclk  initialise clk à fclk pour uart1
 	LPC_SC->PCLKSEL1 |= (0x01<<16); // initialise clk uart2 à fclk
 	LPC_GPIO1->FIODIR |= RELAIS; // en sortie
 	LPC_GPIO1->FIOCLR |= RELAIS;  // RELAIS off
 	LPC_GPIO0->FIODIR &= ~I0; // en entree
 	LPC_GPIO0->FIODIR &= ~I1; // en entree
 	LPC_GPIO0->FIODIR &= ~I2; // en entree
 	LPC_GPIO0->FIODIR &= ~I3; // en entree
 	LPC_GPIO0->FIODIR &= ~I4; // en entree
 	LPC_GPIO0->FIODIR |= RST_IP; // reset ip en sortie
 	LPC_GPIO0->FIOSET |= RST_IP; // à 1
 	//LPC_GPIO0->FIODIR |= ISP;
 	//LPC_GPIO0->FIOSET |= ISP;
 	LPC_SC->PCONP |= (1<<24);	// valide uart2
 	ser_OpenPort (1);
 	// init série cpu
 	ser_InitPort1 (115200,8,0,0);
 	ser_OpenPort (2);
 	ser_InitPort2 (115200,8,0,0);
 }
 /****************tempo**********************/
 void Tempo(long delai){
 	long i;
 	for(i=0;i<=delai;i++);
 }

/*----------------------------------------------------------------------------
  open the serial port
 *---------------------------------------------------------------------------*/
void ser_OpenPort (char portNum) {

  if ( portNum == 1 )
  {
	  /* Port 1 */
	  	NVIC_DisableIRQ(UART1_IRQn);
	  	LPC_PINCON->PINSEL4 &= ~0x0000000F;
	  	LPC_PINCON->PINSEL4 |= TXRXD1;    /* Enable RxD1 P2.1, TxD1 P2.0 */
  }
  else if ( portNum == 2 )
  {
	  /* Port 2 */
	  	NVIC_DisableIRQ(UART2_IRQn);
	  	LPC_PINCON->PINSEL0 &= ~TXRXD2;
	  	LPC_PINCON->PINSEL0 |= TXRXD2;    /* Enable RxD2 P0.10, TxD3 P0.11 */
  }
  return;
}
/*----------------------------------------------------------------------------
  close the serial port
 *---------------------------------------------------------------------------*/
void ser_ClosePort (char portNum ) {
  if ( portNum == 1 )
  {
	  /* Port 1 */
	  	LPC_PINCON->PINSEL4 &= ~0x0000000F;
	  	/* Disable the interrupt in the VIC and UART controllers */
	  	LPC_UART1->IER = 0;
	  	NVIC_DisableIRQ(UART1_IRQn);
  }
  else if ( portNum == 2 )
    {
  	  /* Port 3 */
  	  	LPC_PINCON->PINSEL0 &= ~TXRXD2;
  	  	/* Disable the interrupt in the VIC and UART controllers */
  	  	LPC_UART2->IER = 0;
  	  	NVIC_DisableIRQ(UART2_IRQn);
    }
  return;
}
/*----------------------------------------------------------------------------
  initialize the serial port
 *---------------------------------------------------------------------------*/
void ser_InitPort1 (unsigned long baudrate, unsigned int  databits,
                  unsigned int  parity,   unsigned int  stopbits) {

  unsigned char lcr_p, lcr_s, lcr_d;
  unsigned int dll;
  unsigned int pclkdiv, pclk;

  switch (databits) {
    case 5:                                            // 5 Data bits
      lcr_d = 0x00;
    break;
    case 6:                                            // 6 Data bits
      lcr_d = 0x01;
    break;
    case 7:                                            // 7 Data bits
      lcr_d = 0x02;
    break;
    case 8:                                            // 8 Data bits
    default:
      lcr_d = 0x03;
    break;
  }

  switch (stopbits) {
    case 1:                                            // 1,5 Stop bits
    case 2:                                            // 2   Stop bits
      lcr_s = 0x04;
    break;
    case 0:                                            // 1   Stop bit
    default:
      lcr_s = 0x00;
    break;
  }

  switch (parity) {
    case 1:                                            // Parity Odd
      lcr_p = 0x08;
    break;
    case 2:                                            // Parity Even
      lcr_p = 0x18;
    break;
    case 3:                                            // Parity Mark
      lcr_p = 0x28;
    break;
    case 4:                                            // Parity Space
      lcr_p = 0x38;
    break;
    case 0:                                            // Parity None
    default:
      lcr_p = 0x00;
    break;
  }

  SER_BUF_RESET(ser_out1);                              // reset out buffer
  SER_BUF_RESET(ser_in1);                               // reset in buffer

  /* Bit 8,9 are for UART1 */
  pclkdiv = (LPC_SC->PCLKSEL0 >> 8) & 0x03;

  switch ( pclkdiv )
  {
	case 0x00:
	default:
	  pclk = SystemFrequency/4;
	  break;
	case 0x01:
	  pclk = SystemFrequency;
	  break;
	case 0x02:
	  pclk = SystemFrequency/2;
	  break;
	case 0x03:
	  pclk = SystemFrequency/8;
	  break;
  }

  dll = (pclk/16)/baudrate ;	/*baud rate */
  LPC_UART1->FDR = 0;                             // Fractional divider not used
  LPC_UART1->LCR = 0x80 | lcr_d | lcr_p | lcr_s;  // Data bits, Parity,   Stop bit
  LPC_UART1->DLL = dll;                           // Baud Rate depending on PCLK
  LPC_UART1->DLM = (dll >> 8);                    // High divisor latch
  LPC_UART1->LCR = 0x00 | lcr_d | lcr_p | lcr_s;  // DLAB = 0
  LPC_UART1->IER = 0x03;                          // Enable TX/RX interrupts

  LPC_UART1->FCR = 0x07;				/* Enable and reset TX and RX FIFO. */
  ser_txRestart = 1;                                   // TX fifo is empty

  /* Enable the UART Interrupt */
  NVIC_EnableIRQ(UART1_IRQn);
  return;
}
/*----------------------------------------------------------------------------
  initialize the serial port
 *---------------------------------------------------------------------------*/
void ser_InitPort2 (unsigned long baudrate, unsigned int  databits,
                  unsigned int  parity,   unsigned int  stopbits) {

  unsigned char lcr_p, lcr_s, lcr_d;
  unsigned int dll;
  unsigned int pclkdiv, pclk;

  switch (databits) {
    case 5:                                            // 5 Data bits
      lcr_d = 0x00;
    break;
    case 6:                                            // 6 Data bits
      lcr_d = 0x01;
    break;
    case 7:                                            // 7 Data bits
      lcr_d = 0x02;
    break;
    case 8:                                            // 8 Data bits
    default:
      lcr_d = 0x03;
    break;
  }

  switch (stopbits) {
    case 1:                                            // 1,5 Stop bits
    case 2:                                            // 2   Stop bits
      lcr_s = 0x04;
    break;
    case 0:                                            // 1   Stop bit
    default:
      lcr_s = 0x00;
    break;
  }

  switch (parity) {
    case 1:                                            // Parity Odd
      lcr_p = 0x08;
    break;
    case 2:                                            // Parity Even
      lcr_p = 0x18;
    break;
    case 3:                                            // Parity Mark
      lcr_p = 0x28;
    break;
    case 4:                                            // Parity Space
      lcr_p = 0x38;
    break;
    case 0:                                            // Parity None
    default:
      lcr_p = 0x00;
    break;
  }

  SER_BUF_RESET(ser_out2);                              // reset out buffer
  SER_BUF_RESET(ser_in2);                               // reset in buffer

  /* Bit 16,17 are for UART2 */
  pclkdiv = (LPC_SC->PCLKSEL1 >> 16) & 0x03;

  switch ( pclkdiv )
  {
	case 0x00:
	default:
	  pclk = SystemFrequency/4;
	  break;
	case 0x01:
	  pclk = SystemFrequency;
	  break;
	case 0x02:
	  pclk = SystemFrequency/2;
	  break;
	case 0x03:
	  pclk = SystemFrequency/8;
	  break;
  }

  dll = (pclk/16)/baudrate ;	/*baud rate */
  LPC_UART2->FDR = 0;                             // Fractional divider not used
  LPC_UART2->LCR = 0x80 | lcr_d | lcr_p | lcr_s;  // Data bits, Parity,   Stop bit
  LPC_UART2->DLL = dll;                           // Baud Rate depending on PCLK
  LPC_UART2->DLM = (dll >> 8);                    // High divisor latch
  LPC_UART2->LCR = 0x00 | lcr_d | lcr_p | lcr_s;  // DLAB = 0
  LPC_UART2->IER = 0x03;                          // Enable TX/RX interrupts

  LPC_UART2->FCR = 0x07;				/* Enable and reset TX and RX FIFO. */
  ser_txRestart = 1;                                   // TX fifo is empty

  /* Enable the UART Interrupt */
  NVIC_EnableIRQ(UART2_IRQn);
  return;
}
/*----------------------------------------------------------------------------
  read data from serial port 1
 *---------------------------------------------------------------------------*/
int ser_Read1 (char *buffer, const int length) {
  int bytesToRead, bytesRead;

  /* Read *length bytes, block if *bytes are not avaialable	*/
  bytesToRead = length;
  bytesToRead = (bytesToRead < length) ? bytesToRead : (length);
  bytesRead = bytesToRead;

  while (bytesToRead--) {
   // while (SER_BUF_EMPTY(ser_in));                     // Block until data is available if none
    *buffer++ = SER_BUF_RD(ser_in1);
  }
  return (bytesRead);
}
/*************************lecture port 3************************/
int ser_Read2 (char *buffer, const int length) {
  int bytesToRead, bytesRead;

  /* Read *length bytes, block if *bytes are not avaialable	*/
  bytesToRead = length;
  bytesToRead = (bytesToRead < length) ? bytesToRead : (length);
  bytesRead = bytesToRead;

  while (bytesToRead--) {
   // while (SER_BUF_EMPTY(ser_in));                     // Block until data is available if none
    *buffer++ = SER_BUF_RD(ser_in2);
  }
  return (bytesRead);
}
/*----------------------------------------------------------------------------
  write data to the serial port
 *---------------------------------------------------------------------------*/
int ser_Write (char portNum, const char *buffer, int length) {
  int  bytesToWrite, bytesWritten;

  // Write *length bytes
  bytesToWrite = length;
  bytesWritten = bytesToWrite;
  if ( portNum == 1 ){
	  //while (!SER_BUF_EMPTY(ser_out1));               // Block until space is available if none
	  	  while (bytesToWrite) {
	  		  SER_BUF_WR(ser_out1, *buffer++);            // Read Rx FIFO to buffer
	  		  bytesToWrite--;
	  	  }
  }
  else if ( portNum == 2 ){
	  //while (!SER_BUF_EMPTY(ser_out3));               // Block until space is available if none
	  	  while (bytesToWrite) {
	  		  SER_BUF_WR(ser_out2, *buffer++);            // Read Rx FIFO to buffer
	  		  bytesToWrite--;
	  	  }
  }
  if (ser_txRestart)
    ser_txRestart = 0;
	if ( portNum == 1 )
	{
		 LPC_UART1->THR = SER_BUF_RD(ser_out1);             // Write to the Tx Register
    }
	else if ( portNum == 2 )
	{
      LPC_UART2->THR = SER_BUF_RD(ser_out2);             // Write to the Tx Register
	}

  return (bytesWritten);
}
/*----------------------------------------------------------------------------
  check if character(s) are available at the serial interface
 *---------------------------------------------------------------------------*/
void ser_AvailChar1 (int *availChar) {

  *availChar = SER_BUF_COUNT(ser_in1);

}
void ser_AvailChar2 (int *availChar) {

  *availChar = SER_BUF_COUNT(ser_in2);

}
/*----------------------------------------------------------------------------
  read the line state of the serial port
 *---------------------------------------------------------------------------*/
void ser_LineState (unsigned short *lineState) {

  *lineState = ser_lineState;
  ser_lineState = 0;

}
/*******************lecture des infos**********************/
int lecture_serie(char *buffer){
	int nbBytes,i;
	nbBytes=0;
	ser_AvailChar1(&nbBytes);
	if (nbBytes>0){
	ser_Read1 (buffer, nbBytes);
	for(i=0;i<nbBytes;i++)
	printf("%c",buffer[i]);
	printf("\n");
	}
	return nbBytes;
}
/***********************lecture io ********************/
unsigned char lect_io(unsigned int io){
	if ((LPC_GPIO0->FIOPIN & io)==0)
			return 1;
		else
			return 0;

}
