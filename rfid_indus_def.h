/*
 * ID_7xx_s_v1_def.h
 *
 *  Created on: 2 avr. 2012
 *      Author: cb
 */
int lecture_serie(char *buffer);
void Init(void);		// Initialisation des entrées sorties
void Tempo(long delai);
void  ser_OpenPort  (char portNum); // ouverture port uart
void  ser_ClosePort (char portNum);
void  ser_InitPort2  (unsigned long baudrate, unsigned int databits, unsigned int parity, unsigned int stopbits);
void  ser_InitPort1  (unsigned long baudrate, unsigned int databits, unsigned int parity, unsigned int stopbits);
void  ser_AvailChar1 (int *availChar);
void  ser_AvailChar2 (int *availChar);
int   ser_Write     (char portNum, const char *buffer, int length);
int   ser_Read1      (char *buffer, const int length);
int   ser_Read2     (char *buffer, const int length);
void  ser_LineState (unsigned short *lineState);
unsigned char lect_io(unsigned int io);
/*----------------------------------------------------------------------------
  serial port 1 interrupt
 *---------------------------------------------------------------------------*/
void UART1_IRQHandler(void)
{
  volatile unsigned long iir;

  iir = LPC_UART1->IIR;

  if ((iir & 0x4) || (iir & 0xC)) {            // RDA or CTI pending
    while (LPC_UART1->LSR & 0x01) {                 // Rx FIFO is not empty
      SER_BUF_WR(ser_in1, LPC_UART1->RBR);           // Read Rx FIFO to buffer
    }
  }
  if ((iir & 0x2)) {                           // TXMIS pending
	if (SER_BUF_COUNT(ser_out1) != 0) {
      LPC_UART1->THR = SER_BUF_RD(ser_out1);         // Write to the Tx FIFO
      ser_txRestart = 0;
    }
	else {
      ser_txRestart = 1;
	}
  }
  ser_lineState = ((LPC_UART1->MSR<<8)|LPC_UART1->LSR) & 0xE01E;    // update linestate
  return;
}
/*----------------------------------------------------------------------------
  serial port 2 interrupt
 *---------------------------------------------------------------------------*/
void UART2_IRQHandler(void)
{
  volatile unsigned long iir;

  iir = LPC_UART2->IIR;

  if ((iir & 0x4) || (iir & 0xC)) {            // RDA or CTI pending
    while (LPC_UART2->LSR & 0x01) {                 // Rx FIFO is not empty
      SER_BUF_WR(ser_in2, LPC_UART2->RBR);           // Read Rx FIFO to buffer
    }
  }
  if ((iir & 0x2)) {                           // TXMIS pending
	if (SER_BUF_COUNT(ser_out2) != 0) {
      LPC_UART2->THR = SER_BUF_RD(ser_out2);         // Write to the Tx FIFO
      ser_txRestart = 0;
    }
	else {
      ser_txRestart = 1;
	}
  }
  ser_lineState = LPC_UART2->LSR & 0x1E;            // update linestate
  return;
}


//volatile uint32_t interrupt0RxStat = 0;
//volatile uint32_t interrupt0OverRunStat = 0;
//volatile uint32_t interrupt0RxTimeoutStat = 0;
//volatile uint32_t interrupt1RxStat = 0;
//volatile uint32_t interrupt1OverRunStat = 0;
//volatile uint32_t interrupt1RxTimeoutStat = 0;

