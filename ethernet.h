/*
 * ethernet.h
 *
 *  Created on: 19 mai 2013
 *      Author: cb
 */

#ifndef ETHERNET_H_
#define ETHERNET_H_

#define RELAIS			(1<<30)		// P1.30
#define I0			(1<<9)		// P0.9
#define I1			(1<<8)		// P0.8
#define I2			(1<<7)		// P0.7
#define I3			(1<<25)		// P0.25
#define I4			(1<<26)		// P0.26
#define RST_IP		(1<<29)		// P0.29
#define ISP			(1<<30)		// P0.30
#define CTS			(1<<28)		// P1.28
#define RTS			(1<<29)		// P1.29
#define PORT_NUM1	1				// port uart CPU
#define PORT_NUM2	2				// port uart CSE-M53
#define TXRXD1		(0x0A<<0)		// P2.0,P2.1 txd1 rxd1
#define TXRXD2		(0x05<<20)		// P0.10,P0.11 txd2 rxd2
#define NB_RECUS	8 				// caractres envoyés par l'antenne
#define SER_BUF_SIZE               (128)               // serial buffer in bytes (power 2)
#define SER_BUF_MASK               (SER_BUF_SIZE-1ul)  // buffer size mask

/* Buffer read / write macros */
#define SER_BUF_RESET(serBuf)      (serBuf.rdIdx = serBuf.wrIdx = 0)
#define SER_BUF_WR(serBuf, dataIn) (serBuf.data[SER_BUF_MASK & serBuf.wrIdx++] = (dataIn))
#define SER_BUF_RD(serBuf)         (serBuf.data[SER_BUF_MASK & serBuf.rdIdx++])
#define SER_BUF_EMPTY(serBuf)      (serBuf.rdIdx == serBuf.wrIdx)
#define SER_BUF_FULL(serBuf)       (serBuf.rdIdx == serBuf.wrIdx+1)
#define SER_BUF_COUNT(serBuf)      (SER_BUF_MASK & (serBuf.wrIdx - serBuf.rdIdx))

// buffer type
typedef struct __SER_BUF_T {
  unsigned char data[SER_BUF_SIZE];
  unsigned int wrIdx;
  unsigned int rdIdx;
} SER_BUF_T;

unsigned long          ser_txRestart;                  // NZ if TX restart is required
unsigned short         ser_lineState;                  // ((msr << 8) | (lsr))
SER_BUF_T              ser_out1;                        // Serial data buffers port 1
SER_BUF_T              ser_in1;
SER_BUF_T              ser_out2;                        // Serial data buffers port 2
SER_BUF_T              ser_in2;
#endif /* ETHERNET_H_ */
