#ifndef __LPC17xx_H
#define __LPC17xx_H

/* System Control Block (SCB) includes:
   Flash Accelerator Module, Clocking and Power Control, External Interrupts,
   Reset, System Control and Status
*/
#define SCB_BASE_ADDR   0x400FC000

#define PCONP_PCTIM0    0x00000002
#define PCONP_PCTIM1    0x00000004
#define PCONP_PCUART0   0x00000008
#define PCONP_PCUART1   0x00000010
#define PCONP_PCPWM1    0x00000040
#define PCONP_PCI2C0    0x00000080
#define PCONP_PCSPI     0x00000100
#define PCONP_PCRTC     0x00000200
#define PCONP_PCSSP1    0x00000400
#define PCONP_PCAD      0x00001000
#define PCONP_PCCAN1    0x00002000
#define PCONP_PCCAN2    0x00004000
#define PCONP_PCGPIO    0x00008000
#define PCONP_PCRIT     0x00010000
#define PCONP_PCMCPWM   0x00020000
#define PCONP_PCQEI     0x00040000
#define PCONP_PCI2C1    0x00080000
#define PCONP_PCSSP0    0x00200000
#define PCONP_PCTIM2    0x00400000
#define PCONP_PCTIM3    0x00800000
#define PCONP_PCUART2   0x01000000
#define PCONP_PCUART3   0x02000000
#define PCONP_PCI2C2    0x04000000
#define PCONP_PCI2S     0x08000000
#define PCONP_PCGPDMA   0x20000000
#define PCONP_PCENET    0x40000000
#define PCONP_PCUSB     0x80000000

#define PLLCON_PLLE     0x00000001
#define PLLCON_PLLC     0x00000002
#define PLLCON_MASK     0x00000003

#define PLLCFG_MUL1     0x00000000
#define PLLCFG_MUL2     0x00000001
#define PLLCFG_MUL3     0x00000002
#define PLLCFG_MUL4     0x00000003
#define PLLCFG_MUL5     0x00000004
#define PLLCFG_MUL6     0x00000005
#define PLLCFG_MUL7     0x00000006
#define PLLCFG_MUL8     0x00000007
#define PLLCFG_MUL9     0x00000008
#define PLLCFG_MUL10    0x00000009
#define PLLCFG_MUL11    0x0000000A
#define PLLCFG_MUL12    0x0000000B
#define PLLCFG_MUL13    0x0000000C
#define PLLCFG_MUL14    0x0000000D
#define PLLCFG_MUL15    0x0000000E
#define PLLCFG_MUL16    0x0000000F
#define PLLCFG_MUL17    0x00000010
#define PLLCFG_MUL18    0x00000011
#define PLLCFG_MUL19    0x00000012
#define PLLCFG_MUL20    0x00000013
#define PLLCFG_MUL21    0x00000014
#define PLLCFG_MUL22    0x00000015
#define PLLCFG_MUL23    0x00000016
#define PLLCFG_MUL24    0x00000017
#define PLLCFG_MUL25    0x00000018
#define PLLCFG_MUL26    0x00000019
#define PLLCFG_MUL27    0x0000001A
#define PLLCFG_MUL28    0x0000001B
#define PLLCFG_MUL29    0x0000001C
#define PLLCFG_MUL30    0x0000001D
#define PLLCFG_MUL31    0x0000001E
#define PLLCFG_MUL32    0x0000001F
#define PLLCFG_MUL33    0x00000020
#define PLLCFG_MUL34    0x00000021
#define PLLCFG_MUL35    0x00000022
#define PLLCFG_MUL36    0x00000023

#define PLLCFG_DIV1     0x00000000
#define PLLCFG_DIV2     0x00010000
#define PLLCFG_DIV3     0x00020000
#define PLLCFG_DIV4     0x00030000
#define PLLCFG_DIV5     0x00040000
#define PLLCFG_DIV6     0x00050000
#define PLLCFG_DIV7     0x00060000
#define PLLCFG_DIV8     0x00070000
#define PLLCFG_DIV9     0x00080000
#define PLLCFG_DIV10    0x00090000
#define PLLCFG_MASK		0x00FF7FFF

#define PLLSTAT_MSEL_MASK	0x00007FFF
#define PLLSTAT_NSEL_MASK	0x00FF0000

#define PLLSTAT_PLLE	(1 << 24)
#define PLLSTAT_PLLC	(1 << 25)
#define PLLSTAT_PLOCK	(1 << 26)

#define PLLFEED_FEED1   0x000000AA
#define PLLFEED_FEED2   0x00000055

#define NVIC_IRQ_WDT         0u         // IRQ0,  exception number 16
#define NVIC_IRQ_TIMER0      1u         // IRQ1,  exception number 17
#define NVIC_IRQ_TIMER1      2u         // IRQ2,  exception number 18
#define NVIC_IRQ_TIMER2      3u         // IRQ3,  exception number 19
#define NVIC_IRQ_TIMER3      4u         // IRQ4,  exception number 20
#define NVIC_IRQ_UART0       5u         // IRQ5,  exception number 21
#define NVIC_IRQ_UART1       6u         // IRQ6,  exception number 22
#define NVIC_IRQ_UART2       7u         // IRQ7,  exception number 23
#define NVIC_IRQ_UART3       8u         // IRQ8,  exception number 24
#define NVIC_IRQ_PWM1        9u         // IRQ9,  exception number 25
#define NVIC_IRQ_I2C0        10u        // IRQ10, exception number 26
#define NVIC_IRQ_I2C1        11u        // IRQ11, exception number 27
#define NVIC_IRQ_I2C2        12u        // IRQ12, exception number 28
#define NVIC_IRQ_SPI         13u        // IRQ13, exception number 29
#define NVIC_IRQ_SSP0        14u        // IRQ14, exception number 30
#define NVIC_IRQ_SSP1        15u        // IRQ15, exception number 31
#define NVIC_IRQ_PLL0        16u        // IRQ16, exception number 32
#define NVIC_IRQ_RTC         17u        // IRQ17, exception number 33
#define NVIC_IRQ_EINT0       18u        // IRQ18, exception number 34
#define NVIC_IRQ_EINT1       19u        // IRQ19, exception number 35
#define NVIC_IRQ_EINT2       20u        // IRQ20, exception number 36
#define NVIC_IRQ_EINT3       21u        // IRQ21, exception number 37
#define NVIC_IRQ_ADC         22u        // IRQ22, exception number 38
#define NVIC_IRQ_BOD         23u        // IRQ23, exception number 39
#define NVIC_IRQ_USB         24u        // IRQ24, exception number 40
#define NVIC_IRQ_CAN         25u        // IRQ25, exception number 41
#define NVIC_IRQ_GPDMA       26u        // IRQ26, exception number 42
#define NVIC_IRQ_I2S         27u        // IRQ27, exception number 43
#define NVIC_IRQ_ETHERNET    28u        // IRQ28, exception number 44
#define NVIC_IRQ_RIT         29u        // IRQ29, exception number 45
#define NVIC_IRQ_MCPWM       30u        // IRQ30, exception number 46
#define NVIC_IRQ_QE          31u        // IRQ31, exception number 47
#define NVIC_IRQ_PLL1        32u        // IRQ32, exception number 48
#define NVIC_IRQ_USB_ACT     33u        // IRQ33, exception number 49
#define NVIC_IRQ_CAN_ACT     34u        // IRQ34, exception number 50


#endif  // __LPC17xx_H


#ifndef CMSIS_17xx_H
#define CMSIS_17xx_H

/******************************************************************************
 * @file:    LPC17xx.h
 * @purpose: CMSIS Cortex-M3 Core Peripheral Access Layer Header File for
 *           NXP LPC17xx Device Series
 * @version: V1.1
 * @date:    14th May 2009
 *----------------------------------------------------------------------------
 *
 * Copyright (C) 2008 ARM Limited. All rights reserved.
 *
 * ARM Limited (ARM) is supplying this software for use with Cortex-M3
 * processor based microcontrollers.  This file can be freely distributed
 * within development tools that are supporting such ARM based processors.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ******************************************************************************/


#ifndef __LPC17xx_H__
#define __LPC17xx_H__

/*
 * ==========================================================================
 * ---------- Interrupt Number Definition -----------------------------------
 * ==========================================================================
 */

typedef enum IRQn
{
/******  Cortex-M3 Processor Exceptions Numbers ***************************************************/
  NonMaskableInt_IRQn           = -14,      /*!< 2 Non Maskable Interrupt                         */
  MemoryManagement_IRQn         = -12,      /*!< 4 Cortex-M3 Memory Management Interrupt          */
  BusFault_IRQn                 = -11,      /*!< 5 Cortex-M3 Bus Fault Interrupt                  */
  UsageFault_IRQn               = -10,      /*!< 6 Cortex-M3 Usage Fault Interrupt                */
  SVCall_IRQn                   = -5,       /*!< 11 Cortex-M3 SV Call Interrupt                   */
  DebugMonitor_IRQn             = -4,       /*!< 12 Cortex-M3 Debug Monitor Interrupt             */
  PendSV_IRQn                   = -2,       /*!< 14 Cortex-M3 Pend SV Interrupt                   */
  SysTick_IRQn                  = -1,       /*!< 15 Cortex-M3 System Tick Interrupt               */

/******  LPC17xx Specific Interrupt Numbers *******************************************************/
  WDT_IRQn                      = 0,        /*!< Watchdog Timer Interrupt                         */
  TIMER0_IRQn                   = 1,        /*!< Timer0 Interrupt                                 */
  TIMER1_IRQn                   = 2,        /*!< Timer1 Interrupt                                 */
  TIMER2_IRQn                   = 3,        /*!< Timer2 Interrupt                                 */
  TIMER3_IRQn                   = 4,        /*!< Timer3 Interrupt                                 */
  UART0_IRQn                    = 5,        /*!< UART0 Interrupt                                  */
  UART1_IRQn                    = 6,        /*!< UART1 Interrupt                                  */
  UART2_IRQn                    = 7,        /*!< UART2 Interrupt                                  */
  UART3_IRQn                    = 8,        /*!< UART3 Interrupt                                  */
  PWM1_IRQn                     = 9,        /*!< PWM1 Interrupt                                   */
  I2C0_IRQn                     = 10,       /*!< I2C0 Interrupt                                   */
  I2C1_IRQn                     = 11,       /*!< I2C1 Interrupt                                   */
  I2C2_IRQn                     = 12,       /*!< I2C2 Interrupt                                   */
  SPI_IRQn                      = 13,       /*!< SPI Interrupt                                    */
  SSP0_IRQn                     = 14,       /*!< SSP0 Interrupt                                   */
  SSP1_IRQn                     = 15,       /*!< SSP1 Interrupt                                   */
  PLL0_IRQn                     = 16,       /*!< PLL0 Lock (Main PLL) Interrupt                   */
  RTC_IRQn                      = 17,       /*!< Real Time Clock Interrupt                        */
  EINT0_IRQn                    = 18,       /*!< External Interrupt 0 Interrupt                   */
  EINT1_IRQn                    = 19,       /*!< External Interrupt 1 Interrupt                   */
  EINT2_IRQn                    = 20,       /*!< External Interrupt 2 Interrupt                   */
  EINT3_IRQn                    = 21,       /*!< External Interrupt 3 Interrupt                   */
  ADC_IRQn                      = 22,       /*!< A/D Converter Interrupt                          */
  BOD_IRQn                      = 23,       /*!< Brown-Out Detect Interrupt                       */
  USB_IRQn                      = 24,       /*!< USB Interrupt                                    */
  CAN_IRQn                      = 25,       /*!< CAN Interrupt                                    */
  DMA_IRQn                      = 26,       /*!< General Purpose DMA Interrupt                    */
  I2S_IRQn                      = 27,       /*!< I2S Interrupt                                    */
  ENET_IRQn                     = 28,       /*!< Ethernet Interrupt                               */
  RIT_IRQn                      = 29,       /*!< Repetitive Interrupt Timer Interrupt             */
  MCPWM_IRQn                    = 30,       /*!< Motor Control PWM Interrupt                      */
  QEI_IRQn                      = 31,       /*!< Quadrature Encoder Interface Interrupt           */
  PLL1_IRQn                     = 32,       /*!< PLL1 Lock (USB PLL) Interrupt                    */
} IRQn_Type;


/*
 * ==========================================================================
 * ----------- Processor and Core Peripheral Section ------------------------
 * ==========================================================================
 */

/* Configuration of the Cortex-M3 Processor and Core Peripherals */
#define __MPU_PRESENT             1         /*!< MPU present or not                               */
#define __NVIC_PRIO_BITS          5         /*!< Number of Bits used for Priority Levels          */
#define __Vendor_SysTickConfig    0         /*!< Set to 1 if different SysTick Config is used     */


//#include "..\core_cm3.h"                    /* Cortex-M3 processor and core peripherals           */
#include "core_cm3.h"
#include "system_LPC17xx.h"                 /* System Header


 * Initialize the system clock
 *
 * @param  none
 * @return none
 *
 * @brief  Setup the microcontroller system.
 *         Initialize the System and update the SystemFrequency variable.
 */
extern void SystemInit (void);


/******************************************************************************/
/*                Device Specific Peripheral registers structures             */
/******************************************************************************/

/*------------- System Control (SC) ------------------------------------------*/
typedef struct
{
  __IO uint32_t FLASHCFG;               /* Flash Accelerator Module           */
       uint32_t RESERVED0[31];
  __IO uint32_t PLL0CON;                /* Clocking and Power Control         */
  __IO uint32_t PLL0CFG;
  __I  uint32_t PLL0STAT;
  __O  uint32_t PLL0FEED;
       uint32_t RESERVED1[4];
  __IO uint32_t PLL1CON;
  __IO uint32_t PLL1CFG;
  __I  uint32_t PLL1STAT;
  __O  uint32_t PLL1FEED;
       uint32_t RESERVED2[4];
  __IO uint32_t PCON;
  __IO uint32_t PCONP;
       uint32_t RESERVED3[15];
  __IO uint32_t CCLKCFG;
  __IO uint32_t USBCLKCFG;
  __IO uint32_t CLKSRCSEL;
       uint32_t RESERVED4[12];
  __IO uint32_t EXTINT;                 /* External Interrupts                */
       uint32_t RESERVED5;
  __IO uint32_t EXTMODE;
  __IO uint32_t EXTPOLAR;
       uint32_t RESERVED6[12];
  __IO uint32_t RSID;                   /* Reset                              */
       uint32_t RESERVED7[7];
  __IO uint32_t SCS;                    /* Syscon Miscellaneous Registers     */
  __IO uint32_t IRCTRIM;                /* Clock Dividers                     */
  __IO uint32_t PCLKSEL0;
  __IO uint32_t PCLKSEL1;
       uint32_t RESERVED8[4];
  __IO uint32_t USBIntSt;               /* USB Device/OTG Interrupt Register  */
       uint32_t RESERVED9;
  __IO uint32_t CLKOUTCFG;              /* Clock Output Configuration         */
 } SC_TypeDef;

/*------------- Pin Connect Block (PINCON) -----------------------------------*/
typedef struct
{
  __IO uint32_t PINSEL0;
  __IO uint32_t PINSEL1;
  __IO uint32_t PINSEL2;
  __IO uint32_t PINSEL3;
  __IO uint32_t PINSEL4;
  __IO uint32_t PINSEL5;
  __IO uint32_t PINSEL6;
  __IO uint32_t PINSEL7;
  __IO uint32_t PINSEL8;
  __IO uint32_t PINSEL9;
  __IO uint32_t PINSEL10;
       uint32_t RESERVED0[5];
  __IO uint32_t PINMODE0;
  __IO uint32_t PINMODE1;
  __IO uint32_t PINMODE2;
  __IO uint32_t PINMODE3;
  __IO uint32_t PINMODE4;
  __IO uint32_t PINMODE5;
  __IO uint32_t PINMODE6;
  __IO uint32_t PINMODE7;
  __IO uint32_t PINMODE8;
  __IO uint32_t PINMODE9;
  __IO uint32_t PINMODE_OD0;
  __IO uint32_t PINMODE_OD1;
  __IO uint32_t PINMODE_OD2;
  __IO uint32_t PINMODE_OD3;
  __IO uint32_t PINMODE_OD4;
} PINCON_TypeDef;

/*------------- General Purpose Input/Output (GPIO) --------------------------*/
typedef struct
{
  __IO uint32_t FIODIR;
       uint32_t RESERVED0[3];
  __IO uint32_t FIOMASK;
  __IO uint32_t FIOPIN;
  __IO uint32_t FIOSET;
  __O  uint32_t FIOCLR;
} GPIO_TypeDef;

typedef struct
{
  __I  uint32_t IntStatus;
  __I  uint32_t IO0IntStatR;
  __I  uint32_t IO0IntStatF;
  __O  uint32_t IO0IntClr;
  __IO uint32_t IO0IntEnR;
  __IO uint32_t IO0IntEnF;
       uint32_t RESERVED0[3];
  __I  uint32_t IO2IntStatR;
  __I  uint32_t IO2IntStatF;
  __O  uint32_t IO2IntClr;
  __IO uint32_t IO2IntEnR;
  __IO uint32_t IO2IntEnF;
} GPIOINT_TypeDef;

/*------------- Timer (TIM) --------------------------------------------------*/
typedef struct
{
  __IO uint32_t IR;
  __IO uint32_t TCR;
  __IO uint32_t TC;
  __IO uint32_t PR;
  __IO uint32_t PC;
  __IO uint32_t MCR;
  __IO uint32_t MR0;
  __IO uint32_t MR1;
  __IO uint32_t MR2;
  __IO uint32_t MR3;
  __IO uint32_t CCR;
  __I  uint32_t CR0;
  __I  uint32_t CR1;
       uint32_t RESERVED0[2];
  __IO uint32_t EMR;
       uint32_t RESERVED1[24];
  __IO uint32_t CTCR;
} TIM_TypeDef;

/*------------- Pulse-Width Modulation (PWM) ---------------------------------*/
typedef struct
{
  __IO uint32_t IR;
  __IO uint32_t TCR;
  __IO uint32_t TC;
  __IO uint32_t PR;
  __IO uint32_t PC;
  __IO uint32_t MCR;
  __IO uint32_t MR0;
  __IO uint32_t MR1;
  __IO uint32_t MR2;
  __IO uint32_t MR3;
  __IO uint32_t CCR;
  __I  uint32_t CR0;
  __I  uint32_t CR1;
  __I  uint32_t CR2;
  __I  uint32_t CR3;
  __IO uint32_t MR4;
  __IO uint32_t MR5;
  __IO uint32_t MR6;
  __IO uint32_t PCR;
  __IO uint32_t LER;
       uint32_t RESERVED0[7];
  __IO uint32_t CTCR;
} PWM_TypeDef;

/*------------- Universal Asynchronous Receiver Transmitter (UART) -----------*/
/** @brief  Universal Asynchronous Receiver Transmitter (UART) register structure definition */
typedef struct
{
  union {
  __I  uint32_t RBR;                   /*!< Offset: 0x000 Receiver Buffer  Register (R/ ) */
  __O  uint32_t THR;                   /*!< Offset: 0x000 Transmit Holding Register ( /W) */
  __IO uint32_t DLL;                   /*!< Offset: 0x000 Divisor Latch LSB (R/W) */
  };
  union {
  __IO uint32_t DLM;                   /*!< Offset: 0x004 Divisor Latch MSB (R/W) */
  __IO uint32_t IER;                   /*!< Offset: 0x004 Interrupt Enable Register (R/W) */
  };
  union {
  __I  uint32_t IIR;                   /*!< Offset: 0x008 Interrupt ID Register (R/ ) */
  __O  uint32_t FCR;                   /*!< Offset: 0x008 FIFO Control Register ( /W) */
  };
  __IO uint32_t LCR;                   /*!< Offset: 0x00C Line Control Register (R/W) */
       uint32_t RESERVED0;
  __I  uint32_t LSR;                   /*!< Offset: 0x014 Line Status Register (R/ ) */
       uint32_t RESERVED1;
  __IO uint32_t SCR;                   /*!< Offset: 0x01C Scratch Pad Register (R/W) */
  __IO uint32_t ACR;                   /*!< Offset: 0x020 Auto-baud Control Register (R/W) */
  __IO uint32_t ICR;                   /*!< Offset: 0x024 IrDA Control Register (R/W) */
  __IO uint32_t FDR;                   /*!< Offset: 0x028 Fractional Divider Register (R/W) */
       uint32_t RESERVED2;
  __IO uint32_t TER;                   /*!< Offset: 0x030 Transmit Enable Register (R/W) */
} LPC_UART_TypeDef;

/** @brief  Universal Asynchronous Receiver Transmitter 0 (UART0) register structure definition */
typedef struct
{
  union {
  __I  uint32_t  RBR;                   /*!< Offset: 0x000 Receiver Buffer  Register (R/ ) */
  __O  uint32_t  THR;                   /*!< Offset: 0x000 Transmit Holding Register ( /W) */
  __IO uint32_t  DLL;                   /*!< Offset: 0x000 Divisor Latch LSB (R/W) */
  };
  union {
  __IO uint32_t  DLM;                   /*!< Offset: 0x004 Divisor Latch MSB (R/W) */
  __IO uint32_t  IER;                   /*!< Offset: 0x000 Interrupt Enable Register (R/W) */
  };
  union {
  __I  uint32_t  IIR;                   /*!< Offset: 0x008 Interrupt ID Register (R/ ) */
  __O  uint32_t  FCR;                   /*!< Offset: 0x008 FIFO Control Register ( /W) */
  };
  __IO uint32_t  LCR;                   /*!< Offset: 0x00C Line Control Register (R/W) */
  __IO uint32_t  MCR;                   /*!< Offset: 0x010 Modem control Register (R/W) */
  __I  uint32_t  LSR;                   /*!< Offset: 0x014 Line Status Register (R/ ) */
  __I  uint32_t  MSR;                   /*!< Offset: 0x018 Modem status Register (R/ ) */
  __IO uint32_t  SCR;                   /*!< Offset: 0x01C Scratch Pad Register (R/W) */
  __IO uint32_t  ACR;                   /*!< Offset: 0x020 Auto-baud Control Register (R/W) */
       uint32_t  RESERVED0;
  __IO uint32_t  FDR;                   /*!< Offset: 0x028 Fractional Divider Register (R/W) */
       uint32_t  RESERVED1;
  __IO uint32_t  TER;                   /*!< Offset: 0x030 Transmit Enable Register (R/W) */
       uint32_t  RESERVED2[6];
  __IO uint32_t  RS485CTRL;             /*!< Offset: 0x04C RS-485/EIA-485 Control Register (R/W) */
  __IO uint32_t  ADRMATCH;              /*!< Offset: 0x050 RS-485/EIA-485 address match Register (R/W) */
  __IO uint32_t  RS485DLY;              /*!< Offset: 0x054 RS-485/EIA-485 direction control delay Register (R/W) */
} LPC_UART1_TypeDef;


typedef struct
{
  union {
  __I  uint8_t  RBR;
  __O  uint8_t  THR;
  __IO uint8_t  DLL;
       uint32_t RESERVED0;
  };
  union {
  __IO uint8_t  DLM;
  __IO uint32_t IER;
  };
  union {
  __I  uint32_t IIR;
  __O  uint8_t  FCR;
  };
  __IO uint8_t  LCR;
       uint8_t  RESERVED1[3];
  __IO uint8_t  MCR;
       uint8_t  RESERVED2[3];
  __IO uint8_t  LSR;
       uint8_t  RESERVED3[3];
  __IO uint8_t  MSR;
       uint8_t  RESERVED4[3];
  __IO uint8_t  SCR;
       uint8_t  RESERVED5[3];
  __IO uint32_t ACR;
       uint32_t RESERVED6;
  __IO uint32_t FDR;
       uint32_t RESERVED7;
  __IO uint8_t  TER;
       uint8_t  RESERVED8[27];
  __IO uint8_t  RS485CTRL;
       uint8_t  RESERVED9[3];
  __IO uint8_t  ADRMATCH;
       uint8_t  RESERVED10[3];
  __IO uint8_t  RS485DLY;
} UART1_TypeDef;

/*------------- Serial Peripheral Interface (SPI) ----------------------------*/
typedef struct
{
  __IO uint32_t SPCR;
  __I  uint32_t SPSR;
  __IO uint32_t SPDR;
  __IO uint32_t SPCCR;
       uint32_t RESERVED0[3];
  __IO uint32_t SPINT;
} SPI_TypeDef;

/*------------- Synchronous Serial Communication (SSP) -----------------------*/
typedef struct
{
  __IO uint32_t CR0;
  __IO uint32_t CR1;
  __IO uint32_t DR;
  __I  uint32_t SR;
  __IO uint32_t CPSR;
  __IO uint32_t IMSC;
  __IO uint32_t RIS;
  __IO uint32_t MIS;
  __IO uint32_t ICR;
  __IO uint32_t DMACR;
} SSP_TypeDef;
/*------------- Inter-Integrated Circuit (I2C) -------------------------------*/
/** @brief  Inter-Integrated Circuit (I2C) register structure definition */
typedef struct
{
  __IO uint32_t CONSET;                     /*!< Offset: 0x000 (R/W)  I2C Control Set Register */
  __I  uint32_t STAT;                       /*!< Offset: 0x004 (R/ )  I2C Status Register */
  __IO uint32_t DAT;                        /*!< Offset: 0x008 (R/W)  I2C Data Register */
  __IO uint32_t ADR0;                       /*!< Offset: 0x00C (R/W)  I2C Slave Address Register 0 */
  __IO uint32_t SCLH;                       /*!< Offset: 0x010 (R/W)  SCH Duty Cycle Register High Half Word */
  __IO uint32_t SCLL;                       /*!< Offset: 0x014 (R/W)  SCL Duty Cycle Register Low Half Word */
  __O  uint32_t CONCLR;                     /*!< Offset: 0x018 (R/W)  I2C Control Clear Register */
  __IO uint32_t MMCTRL;                     /*!< Offset: 0x01C (R/W)  Monitor mode control register */
  __IO uint32_t ADR1;                       /*!< Offset: 0x020 (R/W)  I2C Slave Address Register 1 */
  __IO uint32_t ADR2;                       /*!< Offset: 0x024 (R/W)  I2C Slave Address Register 2 */
  __IO uint32_t ADR3;                       /*!< Offset: 0x028 (R/W)  I2C Slave Address Register 3 */
  __I  uint32_t DATA_BUFFER;                /*!< Offset: 0x02C (R/ )  Data buffer Register */
  __IO uint32_t MASK0;                      /*!< Offset: 0x030 (R/W)  I2C Slave address mask register 0 */
  __IO uint32_t MASK1;                      /*!< Offset: 0x034 (R/W)  I2C Slave address mask register 1 */
  __IO uint32_t MASK2;                      /*!< Offset: 0x038 (R/W)  I2C Slave address mask register 2 */
  __IO uint32_t MASK3;                      /*!< Offset: 0x03C (R/W)  I2C Slave address mask register 3 */
} LPC_I2C_TypeDef;

/*------------- Inter IC Sound (I2S) -----------------------------------------*/
typedef struct
{
  __IO uint32_t I2SDAO;
  __IO  uint32_t I2SDAI;
  __O  uint32_t I2STXFIFO;
  __I  uint32_t I2SRXFIFO;
  __I  uint32_t I2SSTATE;
  __IO uint32_t I2SDMA1;
  __IO uint32_t I2SDMA2;
  __IO uint32_t I2SIRQ;
  __IO uint32_t I2STXRATE;
  __IO uint32_t I2SRXRATE;
  __IO uint32_t I2STXBITRATE;
  __IO uint32_t I2SRXBITRATE;
  __IO uint32_t I2STXMODE;
  __IO uint32_t I2SRXMODE;
} I2S_TypeDef;

/*------------- Repetitive Interrupt Timer (RIT) -----------------------------*/
typedef struct
{
  __IO uint32_t RICOMPVAL;
  __IO uint32_t RIMASK;
  __IO uint8_t  RICTRL;
       uint8_t  RESERVED0[3];
  __IO uint32_t RICOUNTER;
} RIT_TypeDef;

/*------------- Real-Time Clock (RTC) ----------------------------------------*/
typedef struct
{
  __IO uint8_t  ILR;
       uint8_t  RESERVED0[3];
  __IO uint8_t  CCR;
       uint8_t  RESERVED1[3];
  __IO uint8_t  CIIR;
       uint8_t  RESERVED2[3];
  __IO uint8_t  AMR;
       uint8_t  RESERVED3[3];
  __I  uint32_t CTIME0;
  __I  uint32_t CTIME1;
  __I  uint32_t CTIME2;
  __IO uint8_t  SEC;
       uint8_t  RESERVED4[3];
  __IO uint8_t  MIN;
       uint8_t  RESERVED5[3];
  __IO uint8_t  HOUR;
       uint8_t  RESERVED6[3];
  __IO uint8_t  DOM;
       uint8_t  RESERVED7[3];
  __IO uint8_t  DOW;
       uint8_t  RESERVED8[3];
  __IO uint16_t DOY;
       uint16_t RESERVED9;
  __IO uint8_t  MONTH;
       uint8_t  RESERVED10[3];
  __IO uint16_t YEAR;
       uint16_t RESERVED11;
  __IO uint32_t CALIBRATION;
  __IO uint32_t GPREG0;
  __IO uint32_t GPREG1;
  __IO uint32_t GPREG2;
  __IO uint32_t GPREG3;
  __IO uint32_t GPREG4;
  __IO uint8_t  WAKEUPDIS;
       uint8_t  RESERVED12[3];
  __IO uint8_t  PWRCTRL;
       uint8_t  RESERVED13[3];
  __IO uint8_t  ALSEC;
       uint8_t  RESERVED14[3];
  __IO uint8_t  ALMIN;
       uint8_t  RESERVED15[3];
  __IO uint8_t  ALHOUR;
       uint8_t  RESERVED16[3];
  __IO uint8_t  ALDOM;
       uint8_t  RESERVED17[3];
  __IO uint8_t  ALDOW;
       uint8_t  RESERVED18[3];
  __IO uint16_t ALDOY;
       uint16_t RESERVED19;
  __IO uint8_t  ALMON;
       uint8_t  RESERVED20[3];
  __IO uint16_t ALYEAR;
       uint16_t RESERVED21;
} RTC_TypeDef;

/*------------- Watchdog Timer (WDT) -----------------------------------------*/
typedef struct
{
  __IO uint8_t  WDMOD;
       uint8_t  RESERVED0[3];
  __IO uint32_t WDTC;
  __O  uint8_t  WDFEED;
       uint8_t  RESERVED1[3];
  __I  uint32_t WDTV;
  __IO uint32_t WDCLKSEL;
} WDT_TypeDef;

/*------------- Analog-to-Digital Converter (ADC) ----------------------------*/
typedef struct
{
  __IO uint32_t ADCR;
  __IO uint32_t ADGDR;
       uint32_t RESERVED0;
  __IO uint32_t ADINTEN;
  __I  uint32_t ADDR0;
  __I  uint32_t ADDR1;
  __I  uint32_t ADDR2;
  __I  uint32_t ADDR3;
  __I  uint32_t ADDR4;
  __I  uint32_t ADDR5;
  __I  uint32_t ADDR6;
  __I  uint32_t ADDR7;
  __I  uint32_t ADSTAT;
  __IO uint32_t ADTRM;
} ADC_TypeDef;

/*------------- Digital-to-Analog Converter (DAC) ----------------------------*/
typedef struct
{
  __IO uint32_t DACR;
  __IO uint32_t DACCTRL;
  __IO uint16_t DACCNTVAL;
} DAC_TypeDef;

/*------------- Motor Control Pulse-Width Modulation (MCPWM) -----------------*/
typedef struct
{
  __I  uint32_t MCCON;
  __O  uint32_t MCCON_SET;
  __O  uint32_t MCCON_CLR;
  __I  uint32_t MCCAPCON;
  __O  uint32_t MCCAPCON_SET;
  __O  uint32_t MCCAPCON_CLR;
  __IO uint32_t MCTIM0;
  __IO uint32_t MCTIM1;
  __IO uint32_t MCTIM2;
  __IO uint32_t MCPER0;
  __IO uint32_t MCPER1;
  __IO uint32_t MCPER2;
  __IO uint32_t MCPW0;
  __IO uint32_t MCPW1;
  __IO uint32_t MCPW2;
  __IO uint32_t MCDEADTIME;
  __IO uint32_t MCCCP;
  __IO uint32_t MCCR0;
  __IO uint32_t MCCR1;
  __IO uint32_t MCCR2;
  __I  uint32_t MCINTEN;
  __O  uint32_t MCINTEN_SET;
  __O  uint32_t MCINTEN_CLR;
  __I  uint32_t MCCNTCON;
  __O  uint32_t MCCNTCON_SET;
  __O  uint32_t MCCNTCON_CLR;
  __I  uint32_t MCINTFLAG;
  __O  uint32_t MCINTFLAG_SET;
  __O  uint32_t MCINTFLAG_CLR;
  __O  uint32_t MCCAP_CLR;
} MCPWM_TypeDef;

/*------------- Quadrature Encoder Interface (QEI) ---------------------------*/
typedef struct
{
  __O  uint32_t QEICON;
  __I  uint32_t QEISTAT;
  __IO uint32_t QEICONF;
  __I  uint32_t QEIPOS;
  __IO uint32_t QEIMAXPOS;
  __IO uint32_t CMPOS0;
  __IO uint32_t CMPOS1;
  __IO uint32_t CMPOS2;
  __I  uint32_t INXCNT;
  __IO uint32_t INXCMP;
  __IO uint32_t QEILOAD;
  __I  uint32_t QEITIME;
  __I  uint32_t QEIVEL;
  __I  uint32_t QEICAP;
  __IO uint32_t VELCOMP;
  __IO uint32_t FILTER;
       uint32_t RESERVED0[998];
  __O  uint32_t QEIIEC;
  __O  uint32_t QEIIES;
  __I  uint32_t QEIINTSTAT;
  __I  uint32_t QEIIE;
  __O  uint32_t QEICLR;
  __O  uint32_t QEISET;
} QEI_TypeDef;

/*------------- Controller Area Network (CAN) --------------------------------*/
typedef struct
{
  __IO uint32_t mask[512];              /* ID Masks                           */
} CANAF_RAM_TypeDef;

typedef struct                          /* Acceptance Filter Registers        */
{
  __IO uint32_t AFMR;
  __IO uint32_t SFF_sa;
  __IO uint32_t SFF_GRP_sa;
  __IO uint32_t EFF_sa;
  __IO uint32_t EFF_GRP_sa;
  __IO uint32_t ENDofTable;
  __I  uint32_t LUTerrAd;
  __I  uint32_t LUTerr;
} CANAF_TypeDef;

typedef struct                          /* Central Registers                  */
{
  __I  uint32_t CANTxSR;
  __I  uint32_t CANRxSR;
  __I  uint32_t CANMSR;
} CANCR_TypeDef;

typedef struct                          /* Controller Registers               */
{
  __IO uint32_t MOD;
  __O  uint32_t CMR;
  __IO uint32_t GSR;
  __I  uint32_t ICR;
  __IO uint32_t IER;
  __IO uint32_t BTR;
  __IO uint32_t EWL;
  __I  uint32_t SR;
  __IO uint32_t RFS;
  __IO uint32_t RID;
  __IO uint32_t RDA;
  __IO uint32_t RDB;
  __IO uint32_t TFI1;
  __IO uint32_t TID1;
  __IO uint32_t TDA1;
  __IO uint32_t TDB1;
  __IO uint32_t TFI2;
  __IO uint32_t TID2;
  __IO uint32_t TDA2;
  __IO uint32_t TDB2;
  __IO uint32_t TFI3;
  __IO uint32_t TID3;
  __IO uint32_t TDA3;
  __IO uint32_t TDB3;
} CAN_TypeDef;

/*------------- General Purpose Direct Memory Access (GPDMA) -----------------*/
typedef struct                          /* Common Registers                   */
{
  __I  uint32_t DMACIntStat;
  __I  uint32_t DMACIntTCStat;
  __O  uint32_t DMACIntTCClear;
  __I  uint32_t DMACIntErrStat;
  __O  uint32_t DMACIntErrClr;
  __I  uint32_t DMACRawIntTCStat;
  __I  uint32_t DMACRawIntErrStat;
  __I  uint32_t DMACEnbldChns;
  __IO uint32_t DMACSoftBReq;
  __IO uint32_t DMACSoftSReq;
  __IO uint32_t DMACSoftLBReq;
  __IO uint32_t DMACSoftLSReq;
  __IO uint32_t DMACConfig;
  __IO uint32_t DMACSync;
} GPDMA_TypeDef;

typedef struct                          /* Channel Registers                  */
{
  __IO uint32_t DMACCSrcAddr;
  __IO uint32_t DMACCDestAddr;
  __IO uint32_t DMACCLLI;
  __IO uint32_t DMACCControl;
  __IO uint32_t DMACCConfig;
} GPDMACH_TypeDef;

/*------------- Universal Serial Bus (USB) -----------------------------------*/
typedef struct
{
  __I  uint32_t HcRevision;             /* USB Host Registers                 */
  __IO uint32_t HcControl;
  __IO uint32_t HcCommandStatus;
  __IO uint32_t HcInterruptStatus;
  __IO uint32_t HcInterruptEnable;
  __IO uint32_t HcInterruptDisable;
  __IO uint32_t HcHCCA;
  __I  uint32_t HcPeriodCurrentED;
  __IO uint32_t HcControlHeadED;
  __IO uint32_t HcControlCurrentED;
  __IO uint32_t HcBulkHeadED;
  __IO uint32_t HcBulkCurrentED;
  __I  uint32_t HcDoneHead;
  __IO uint32_t HcFmInterval;
  __I  uint32_t HcFmRemaining;
  __I  uint32_t HcFmNumber;
  __IO uint32_t HcPeriodicStart;
  __IO uint32_t HcLSTreshold;
  __IO uint32_t HcRhDescriptorA;
  __IO uint32_t HcRhDescriptorB;
  __IO uint32_t HcRhStatus;
  __IO uint32_t HcRhPortStatus1;
  __IO uint32_t HcRhPortStatus2;
       uint32_t RESERVED0[40];
  __I  uint32_t Module_ID;

  __I  uint32_t OTGIntSt;               /* USB On-The-Go Registers            */
  __IO uint32_t OTGIntEn;
  __O  uint32_t OTGIntSet;
  __O  uint32_t OTGIntClr;
  __IO uint32_t OTGStCtrl;
  __IO uint32_t OTGTmr;
       uint32_t RESERVED1[58];

  __I  uint32_t USBDevIntSt;            /* USB Device Interrupt Registers     */
  __IO uint32_t USBDevIntEn;
  __O  uint32_t USBDevIntClr;
  __O  uint32_t USBDevIntSet;

  __O  uint32_t USBCmdCode;             /* USB Device SIE Command Registers   */
  __I  uint32_t USBCmdData;

  __I  uint32_t USBRxData;              /* USB Device Transfer Registers      */
  __O  uint32_t USBTxData;
  __I  uint32_t USBRxPLen;
  __O  uint32_t USBTxPLen;
  __IO uint32_t USBCtrl;
  __O  uint32_t USBDevIntPri;

  __I  uint32_t USBEpIntSt;             /* USB Device Endpoint Interrupt Regs */
  __IO uint32_t USBEpIntEn;
  __O  uint32_t USBEpIntClr;
  __O  uint32_t USBEpIntSet;
  __O  uint32_t USBEpIntPri;

  __IO uint32_t USBReEp;                /* USB Device Endpoint Realization Reg*/
  __O  uint32_t USBEpInd;
  __IO uint32_t USBMaxPSize;

  __I  uint32_t USBDMARSt;              /* USB Device DMA Registers           */
  __O  uint32_t USBDMARClr;
  __O  uint32_t USBDMARSet;
       uint32_t RESERVED2[9];
  __IO uint32_t USBUDCAH;
  __I  uint32_t USBEpDMASt;
  __O  uint32_t USBEpDMAEn;
  __O  uint32_t USBEpDMADis;
  __I  uint32_t USBDMAIntSt;
  __IO uint32_t USBDMAIntEn;
       uint32_t RESERVED3[2];
  __I  uint32_t USBEoTIntSt;
  __O  uint32_t USBEoTIntClr;
  __O  uint32_t USBEoTIntSet;
  __I  uint32_t USBNDDRIntSt;
  __O  uint32_t USBNDDRIntClr;
  __O  uint32_t USBNDDRIntSet;
  __I  uint32_t USBSysErrIntSt;
  __O  uint32_t USBSysErrIntClr;
  __O  uint32_t USBSysErrIntSet;
       uint32_t RESERVED4[15];

  __I  uint32_t I2C_RX;                 /* USB OTG I2C Registers              */
  __O  uint32_t I2C_WO;
  __I  uint32_t I2C_STS;
  __IO uint32_t I2C_CTL;
  __IO uint32_t I2C_CLKHI;
  __O  uint32_t I2C_CLKLO;
       uint32_t RESERVED5[823];

  union {
  __IO uint32_t USBClkCtrl;             /* USB Clock Control Registers        */
  __IO uint32_t OTGClkCtrl;
  } ;
  union {
  __I  uint32_t USBClkSt;
  __I  uint32_t OTGClkSt;
  };
} USB_TypeDef;

/*------------- Ethernet Media Access Controller (EMAC) ----------------------*/
typedef struct
{
  __IO uint32_t MAC1;                   /* MAC Registers                      */
  __IO uint32_t MAC2;
  __IO uint32_t IPGT;
  __IO uint32_t IPGR;
  __IO uint32_t CLRT;
  __IO uint32_t MAXF;
  __IO uint32_t SUPP;
  __IO uint32_t TEST;
  __IO uint32_t MCFG;
  __IO uint32_t MCMD;
  __IO uint32_t MADR;
  __O  uint32_t MWTD;
  __I  uint32_t MRDD;
  __I  uint32_t MIND;
       uint32_t RESERVED0[2];
  __IO uint32_t SA0;
  __IO uint32_t SA1;
  __IO uint32_t SA2;
       uint32_t RESERVED1[45];
  __IO uint32_t Command;                /* Control Registers                  */
  __I  uint32_t Status;
  __IO uint32_t RxDescriptor;
  __IO uint32_t RxStatus;
  __IO uint32_t RxDescriptorNumber;
  __I  uint32_t RxProduceIndex;
  __IO uint32_t RxConsumeIndex;
  __IO uint32_t TxDescriptor;
  __IO uint32_t TxStatus;
  __IO uint32_t TxDescriptorNumber;
  __IO uint32_t TxProduceIndex;
  __I  uint32_t TxConsumeIndex;
       uint32_t RESERVED2[10];
  __I  uint32_t TSV0;
  __I  uint32_t TSV1;
  __I  uint32_t RSV;
       uint32_t RESERVED3[3];
  __IO uint32_t FlowControlCounter;
  __I  uint32_t FlowControlStatus;
       uint32_t RESERVED4[34];
  __IO uint32_t RxFilterCtrl;           /* Rx Filter Registers                */
  __IO uint32_t RxFilterWoLStatus;
  __IO uint32_t RxFilterWoLClear;
       uint32_t RESERVED5;
  __IO uint32_t HashFilterL;
  __IO uint32_t HashFilterH;
       uint32_t RESERVED6[882];
  __I  uint32_t IntStatus;              /* Module Control Registers           */
  __IO uint32_t IntEnable;
  __O  uint32_t IntClear;
  __O  uint32_t IntSet;
       uint32_t RESERVED7;
  __IO uint32_t PowerDown;
       uint32_t RESERVED8;
  __IO uint32_t Module_ID;
} EMAC_TypeDef;
/*------------- System Control (SC) ------------------------------------------*/
/** @brief System Control (SC) register structure definition */
typedef struct
{
  __IO uint32_t FLASHCFG;                   /*!< Offset: 0x000 (R/W)  Flash Accelerator Configuration Register */
       uint32_t RESERVED0[31];
  __IO uint32_t PLL0CON;                    /*!< Offset: 0x080 (R/W)  PLL0 Control Register */
  __IO uint32_t PLL0CFG;                    /*!< Offset: 0x084 (R/W)  PLL0 Configuration Register */
  __I  uint32_t PLL0STAT;                   /*!< Offset: 0x088 (R/ )  PLL0 Status Register */
  __O  uint32_t PLL0FEED;                   /*!< Offset: 0x08C ( /W)  PLL0 Feed Register */
       uint32_t RESERVED1[4];
  __IO uint32_t PLL1CON;                    /*!< Offset: 0x0A0 (R/W)  PLL1 Control Register */
  __IO uint32_t PLL1CFG;                    /*!< Offset: 0x0A4 (R/W)  PLL1 Configuration Register */
  __I  uint32_t PLL1STAT;                   /*!< Offset: 0x0A8 (R/ )  PLL1 Status Register */
  __O  uint32_t PLL1FEED;                   /*!< Offset: 0x0AC ( /W)  PLL1 Feed Register */
       uint32_t RESERVED2[4];
  __IO uint32_t PCON;                       /*!< Offset: 0x0C0 (R/W)  Power Control Register */
  __IO uint32_t PCONP;                      /*!< Offset: 0x0C4 (R/W)  Power Control for Peripherals Register */
       uint32_t RESERVED3[15];
  __IO uint32_t CCLKCFG;                    /*!< Offset: 0x104 (R/W)  CPU Clock Configure Register  */
  __IO uint32_t USBCLKCFG;                  /*!< Offset: 0x108 (R/W)  USB Clock Configure Register */
  __IO uint32_t CLKSRCSEL;                  /*!< Offset: 0x10C (R/W)  Clock Source Select Register */
  __IO uint32_t CANSLEEPCLR;                /*!< Offset: 0x110 (R/W)  CAN Sleep Clear Register */
  __IO uint32_t CANWAKEFLAGS;               /*!< Offset: 0x114 (R/W)  CAN Wake-up Flags Register */
       uint32_t RESERVED4[10];
  __IO uint32_t EXTINT;                     /*!< Offset: 0x140 (R/W)  External Interrupt Flag Register */
       uint32_t RESERVED5[1];
  __IO uint32_t EXTMODE;                    /*!< Offset: 0x148 (R/W)  External Interrupt Mode Register */
  __IO uint32_t EXTPOLAR;                   /*!< Offset: 0x14C (R/W)  External Interrupt Polarity Register */
       uint32_t RESERVED6[12];
  __IO uint32_t RSID;                       /*!< Offset: 0x180 (R/W)  Reset Source Identification Register */
       uint32_t RESERVED7[7];
  __IO uint32_t SCS;                        /*!< Offset: 0x1A0 (R/W)  System Controls and Status Register */
  __IO uint32_t IRCTRIM;                /* Clock Dividers                     */
  __IO uint32_t PCLKSEL0;                   /*!< Offset: 0x1A8 (R/W)  Peripheral Clock Select 0 Register */
  __IO uint32_t PCLKSEL1;                   /*!< Offset: 0x1AC (R/W)  Peripheral Clock Select 1 Register */
       uint32_t RESERVED8[4];
  __IO uint32_t USBIntSt;                   /*!< Offset: 0x1C0 (R/W)  USB Interrupt Status Register */
  __IO uint32_t DMAREQSEL;                  /*!< Offset: 0x1C4 (R/W)  DMA Request Select Register */
  __IO uint32_t CLKOUTCFG;                  /*!< Offset: 0x1C8 (R/W)  Clock Output Configuration Register */

 } LPC_SC_TypeDef;

/*------------- Pin Connect Block (PINCON) -----------------------------------*/
/** @brief Pin Connect Block (PINCON) register structure definition */
typedef struct
{
  __IO uint32_t PINSEL0;				 /* !< Offset: 0x000 PIN Select0 (R/W) */
  __IO uint32_t PINSEL1;				 /* !< Offset: 0x004 PIN Select1 (R/W) */
  __IO uint32_t PINSEL2;				 /* !< Offset: 0x008 PIN Select2 (R/W) */
  __IO uint32_t PINSEL3;				 /* !< Offset: 0x00C PIN Select3 (R/W) */
  __IO uint32_t PINSEL4;				 /* !< Offset: 0x010 PIN Select4 (R/W) */
  __IO uint32_t PINSEL5;				 /* !< Offset: 0x014 PIN Select5 (R/W) */
  __IO uint32_t PINSEL6;				 /* !< Offset: 0x018 PIN Select6 (R/W) */
  __IO uint32_t PINSEL7;				 /* !< Offset: 0x01C PIN Select7 (R/W) */
  __IO uint32_t PINSEL8;				 /* !< Offset: 0x020 PIN Select8 (R/W) */
  __IO uint32_t PINSEL9;				 /* !< Offset: 0x024 PIN Select9 (R/W) */
  __IO uint32_t PINSEL10;				 /* !< Offset: 0x028 PIN Select20 (R/W) */
       uint32_t RESERVED0[5];
  __IO uint32_t PINMODE0;				 /* !< Offset: 0x040 PIN Mode0 (R/W) */
  __IO uint32_t PINMODE1;				 /* !< Offset: 0x044 PIN Mode1 (R/W) */
  __IO uint32_t PINMODE2;				 /* !< Offset: 0x048 PIN Mode2 (R/W) */
  __IO uint32_t PINMODE3;				 /* !< Offset: 0x04C PIN Mode3 (R/W) */
  __IO uint32_t PINMODE4;				 /* !< Offset: 0x050 PIN Mode4 (R/W) */
  __IO uint32_t PINMODE5;				 /* !< Offset: 0x054 PIN Mode5 (R/W) */
  __IO uint32_t PINMODE6;				 /* !< Offset: 0x058 PIN Mode6 (R/W) */
  __IO uint32_t PINMODE7;				 /* !< Offset: 0x05C PIN Mode7 (R/W) */
  __IO uint32_t PINMODE8;				 /* !< Offset: 0x060 PIN Mode8 (R/W) */
  __IO uint32_t PINMODE9;				 /* !< Offset: 0x064 PIN Mode9 (R/W) */
  __IO uint32_t PINMODE_OD0;			 /* !< Offset: 0x068 Open Drain PIN Mode0 (R/W) */
  __IO uint32_t PINMODE_OD1;			 /* !< Offset: 0x06C Open Drain PIN Mode1 (R/W) */
  __IO uint32_t PINMODE_OD2;			 /* !< Offset: 0x070 Open Drain PIN Mode2 (R/W) */
  __IO uint32_t PINMODE_OD3;			 /* !< Offset: 0x074 Open Drain PIN Mode3 (R/W) */
  __IO uint32_t PINMODE_OD4;			 /* !< Offset: 0x078 Open Drain PIN Mode4 (R/W) */
  __IO uint32_t I2CPADCFG;				 /* !< Offset: 0x07C I2C Pad Configure (R/W) */
} LPC_PINCON_TypeDef;

/*------------- General Purpose Input/Output (GPIO) --------------------------*/
/** @brief General Purpose Input/Output (GPIO) register structure definition */
typedef struct
{
  union {
    __IO uint32_t FIODIR;				 /* !< Offset: 0x00 Port direction (R/W) */
    struct {
      __IO uint16_t FIODIRL;
      __IO uint16_t FIODIRH;
    };
    struct {
      __IO uint8_t  FIODIR0;
      __IO uint8_t  FIODIR1;
      __IO uint8_t  FIODIR2;
      __IO uint8_t  FIODIR3;
    };
  };
  uint32_t RESERVED0[3];
  union {
    __IO uint32_t FIOMASK;				 /* !< Offset: 0x10 Port mask (R/W) */
    struct {
      __IO uint16_t FIOMASKL;
      __IO uint16_t FIOMASKH;
    };
    struct {
      __IO uint8_t  FIOMASK0;
      __IO uint8_t  FIOMASK1;
      __IO uint8_t  FIOMASK2;
      __IO uint8_t  FIOMASK3;
    };
  };
  union {
    __IO uint32_t FIOPIN;				 /* !< Offset: 0x14 Port value (R/W) */
    struct {
      __IO uint16_t FIOPINL;
      __IO uint16_t FIOPINH;
    };
    struct {
      __IO uint8_t  FIOPIN0;
      __IO uint8_t  FIOPIN1;
      __IO uint8_t  FIOPIN2;
      __IO uint8_t  FIOPIN3;
    };
  };
  union {
    __IO uint32_t FIOSET;				 /* !< Offset: 0x18 Port output set (R/W) */
    struct {
      __IO uint16_t FIOSETL;
      __IO uint16_t FIOSETH;
    };
    struct {
      __IO uint8_t  FIOSET0;
      __IO uint8_t  FIOSET1;
      __IO uint8_t  FIOSET2;
      __IO uint8_t  FIOSET3;
    };
  };
  union {
    __O  uint32_t FIOCLR;				 /* !< Offset: 0x1C Port output clear (R/W) */
    struct {
      __O  uint16_t FIOCLRL;
      __O  uint16_t FIOCLRH;
    };
    struct {
      __O  uint8_t  FIOCLR0;
      __O  uint8_t  FIOCLR1;
      __O  uint8_t  FIOCLR2;
      __O  uint8_t  FIOCLR3;
    };
  };
} LPC_GPIO_TypeDef;

/** @brief General Purpose Input/Output interrupt (GPIOINT) register structure definition */
typedef struct
{
  __I  uint32_t IntStatus;                  /*!< Offset: 0x000 (R/ )  GPIO overall Interrupt Status Register */
  __I  uint32_t IO0IntStatR;                /*!< Offset: 0x004 (R/ )  GPIO Interrupt Status Register 0 for Rising edge */
  __I  uint32_t IO0IntStatF;                /*!< Offset: 0x008 (R/ )  GPIO Interrupt Status Register 0 for Falling edge */
  __O  uint32_t IO0IntClr;                  /*!< Offset: 0x00C (R/W)  GPIO Interrupt Clear  Register 0 */
  __IO uint32_t IO0IntEnR;                  /*!< Offset: 0x010 ( /W)  GPIO Interrupt Enable Register 0 for Rising edge */
  __IO uint32_t IO0IntEnF;                  /*!< Offset: 0x014 (R/W)  GPIO Interrupt Enable Register 0 for Falling edge */
       uint32_t RESERVED0[3];
  __I  uint32_t IO2IntStatR;                /*!< Offset: 0x000 (R/ )  GPIO Interrupt Status Register 2 for Rising edge */
  __I  uint32_t IO2IntStatF;                /*!< Offset: 0x000 (R/ )  GPIO Interrupt Status Register 2 for Falling edge */
  __O  uint32_t IO2IntClr;                  /*!< Offset: 0x000 ( /W)  GPIO Interrupt Clear  Register 2 */
  __IO uint32_t IO2IntEnR;                  /*!< Offset: 0x000 (R/W)  GPIO Interrupt Enable Register 2 for Rising edge */
  __IO uint32_t IO2IntEnF;                  /*!< Offset: 0x000 (R/W)  GPIO Interrupt Enable Register 2 for Falling edge */
} LPC_GPIOINT_TypeDef;

/*------------- Timer (TIM) --------------------------------------------------*/
/** @brief Timer (TIM) register structure definition */
typedef struct
{
  __IO uint32_t IR;                         /*!< Offset: 0x000 (R/W)  Interrupt Register */
  __IO uint32_t TCR;                        /*!< Offset: 0x004 (R/W)  Timer Control Register */
  __IO uint32_t TC;                         /*!< Offset: 0x008 (R/W)  Timer Counter Register */
  __IO uint32_t PR;                         /*!< Offset: 0x00C (R/W)  Prescale Register */
  __IO uint32_t PC;                         /*!< Offset: 0x010 (R/W)  Prescale Counter Register */
  __IO uint32_t MCR;                        /*!< Offset: 0x014 (R/W)  Match Control Register */
  __IO uint32_t MR0;                        /*!< Offset: 0x018 (R/W)  Match Register 0 */
  __IO uint32_t MR1;                        /*!< Offset: 0x01C (R/W)  Match Register 1 */
  __IO uint32_t MR2;                        /*!< Offset: 0x020 (R/W)  Match Register 2 */
  __IO uint32_t MR3;                        /*!< Offset: 0x024 (R/W)  Match Register 3 */
  __IO uint32_t CCR;                        /*!< Offset: 0x028 (R/W)  Capture Control Register */
  __I  uint32_t CR0;                        /*!< Offset: 0x02C (R/ )  Capture Register 0 */
  __I  uint32_t CR1;                        /*!< Offset: 0x030 (R/ )  Capture Register */
       uint32_t RESERVED0[2];
  __IO uint32_t EMR;                        /*!< Offset: 0x03C (R/W)  External Match Register */
       uint32_t RESERVED1[12];
  __IO uint32_t CTCR;                       /*!< Offset: 0x070 (R/W)  Count Control Register */
} LPC_TIM_TypeDef;

/*------------- Pulse-Width Modulation (PWM) ---------------------------------*/
/** @brief Pulse-Width Modulation (PWM) register structure definition */
typedef struct
{
  __IO uint32_t IR;                         /*!< Offset: 0x000 (R/W)  Interrupt Register */
  __IO uint32_t TCR;                        /*!< Offset: 0x004 (R/W)  Timer Control Register. Register */
  __IO uint32_t TC;                         /*!< Offset: 0x008 (R/W)  Timer Counter Register */
  __IO uint32_t PR;                         /*!< Offset: 0x00C (R/W)  Prescale Register */
  __IO uint32_t PC;                         /*!< Offset: 0x010 (R/W)  Prescale Counter Register */
  __IO uint32_t MCR;                        /*!< Offset: 0x014 (R/W)  Match Control Register */
  __IO uint32_t MR0;                        /*!< Offset: 0x018 (R/W)  Match Register 0 */
  __IO uint32_t MR1;                        /*!< Offset: 0x01C (R/W)  Match Register 1 */
  __IO uint32_t MR2;                        /*!< Offset: 0x020 (R/W)  Match Register 2 */
  __IO uint32_t MR3;                        /*!< Offset: 0x024 (R/W)  Match Register 3 */
  __IO uint32_t CCR;                        /*!< Offset: 0x028 (R/W)  Capture Control Register */
  __I  uint32_t CR0;                        /*!< Offset: 0x02C (R/ )  Capture Register 0 */
  __I  uint32_t CR1;                        /*!< Offset: 0x030 (R/ )  Capture Register 1 */
  __I  uint32_t CR2;                        /*!< Offset: 0x034 (R/ )  Capture Register 2 */
  __I  uint32_t CR3;                        /*!< Offset: 0x038 (R/ )  Capture Register 3 */
       uint32_t RESERVED0;
  __IO uint32_t MR4;                        /*!< Offset: 0x040 (R/W)  Match Register 4 */
  __IO uint32_t MR5;                        /*!< Offset: 0x044 (R/W)  Match Register 5 */
  __IO uint32_t MR6;                        /*!< Offset: 0x048 (R/W)  Match Register 6 */
  __IO uint32_t PCR;                        /*!< Offset: 0x04C (R/W)  PWM Control Register */
  __IO uint32_t LER;                        /*!< Offset: 0x050 (R/W)  Load Enable Register */
       uint32_t RESERVED1[7];
  __IO uint32_t CTCR;                       /*!< Offset: 0x070 (R/W)  Count Control Register */
} LPC_PWM_TypeDef;
/******************************************************************************/
/*                         Peripheral memory map                              */
/******************************************************************************/
/* Base addresses                                                             */
#define FLASH_BASE            (0x00000000UL)
#define RAM_BASE              (0x10000000UL)
#define GPIO_BASE             (0x2009C000UL)
#define APB0_BASE             (0x40000000UL)
#define APB1_BASE             (0x40080000UL)
#define AHB_BASE              (0x50000000UL)
#define CM3_BASE              (0xE0000000UL)

/* APB0 peripherals                                                           */
#define WDT_BASE              (APB0_BASE + 0x00000)
#define TIM0_BASE             (APB0_BASE + 0x04000)
#define TIM1_BASE             (APB0_BASE + 0x08000)
#define UART0_BASE            (APB0_BASE + 0x0C000)
#define UART1_BASE            (APB0_BASE + 0x10000)
#define PWM1_BASE             (APB0_BASE + 0x18000)
#define I2C0_BASE             (APB0_BASE + 0x1C000)
#define SPI_BASE              (APB0_BASE + 0x20000)
#define RTC_BASE              (APB0_BASE + 0x24000)
#define GPIOINT_BASE          (APB0_BASE + 0x28080)
#define PINCON_BASE           (APB0_BASE + 0x2C000)
#define SSP1_BASE             (APB0_BASE + 0x30000)
#define ADC_BASE              (APB0_BASE + 0x34000)
#define CANAF_RAM_BASE        (APB0_BASE + 0x38000)
#define CANAF_BASE            (APB0_BASE + 0x3C000)
#define CANCR_BASE            (APB0_BASE + 0x40000)
#define CAN1_BASE             (APB0_BASE + 0x44000)
#define CAN2_BASE             (APB0_BASE + 0x48000)
#define I2C1_BASE             (APB0_BASE + 0x5C000)

/* APB1 peripherals                                                           */
#define SSP0_BASE             (APB1_BASE + 0x08000)
#define DAC_BASE              (APB1_BASE + 0x0C000)
#define TIM2_BASE             (APB1_BASE + 0x10000)
#define TIM3_BASE             (APB1_BASE + 0x14000)
#define UART2_BASE            (APB1_BASE + 0x18000)
#define UART3_BASE            (APB1_BASE + 0x1C000)
#define I2C2_BASE             (APB1_BASE + 0x20000)
#define I2S_BASE              (APB1_BASE + 0x28000)
#define RIT_BASE              (APB1_BASE + 0x30000)
#define MCPWM_BASE            (APB1_BASE + 0x38000)
#define QEI_BASE              (APB1_BASE + 0x3C000)
#define SC_BASE               (APB1_BASE + 0x7C000)

/* AHB peripherals                                                            */
#define EMAC_BASE             (AHB_BASE  + 0x00000)
#define GPDMA_BASE            (AHB_BASE  + 0x04000)
#define GPDMACH0_BASE         (AHB_BASE  + 0x04100)
#define GPDMACH1_BASE         (AHB_BASE  + 0x04120)
#define GPDMACH2_BASE         (AHB_BASE  + 0x04140)
#define GPDMACH3_BASE         (AHB_BASE  + 0x04160)
#define GPDMACH4_BASE         (AHB_BASE  + 0x04180)
#define GPDMACH5_BASE         (AHB_BASE  + 0x041A0)
#define GPDMACH6_BASE         (AHB_BASE  + 0x041C0)
#define GPDMACH7_BASE         (AHB_BASE  + 0x041E0)
#define USB_BASE              (AHB_BASE  + 0x0C000)

/* GPIOs                                                                      */
#define GPIO0_BASE            (GPIO_BASE + 0x00000)
#define GPIO1_BASE            (GPIO_BASE + 0x00020)
#define GPIO2_BASE            (GPIO_BASE + 0x00040)
#define GPIO3_BASE            (GPIO_BASE + 0x00060)
#define GPIO4_BASE            (GPIO_BASE + 0x00080)


/******************************************************************************/
/*                         Peripheral declaration                             */
/******************************************************************************/
#define SC                    ((       SC_TypeDef *)        SC_BASE)
#define GPIO0                 ((     GPIO_TypeDef *)     GPIO0_BASE)
#define GPIO1                 ((     GPIO_TypeDef *)     GPIO1_BASE)
#define GPIO2                 ((     GPIO_TypeDef *)     GPIO2_BASE)
#define GPIO3                 ((     GPIO_TypeDef *)     GPIO3_BASE)
#define GPIO4                 ((     GPIO_TypeDef *)     GPIO4_BASE)
#define WDT                   ((      WDT_TypeDef *)       WDT_BASE)
#define TIM0                  ((      TIM_TypeDef *)      TIM0_BASE)
#define TIM1                  ((      TIM_TypeDef *)      TIM1_BASE)
#define TIM2                  ((      TIM_TypeDef *)      TIM2_BASE)
#define TIM3                  ((      TIM_TypeDef *)      TIM3_BASE)
#define RIT                   ((      RIT_TypeDef *)       RIT_BASE)
#define UART0                 ((     UART_TypeDef *)     UART0_BASE)
#define UART1                 ((    UART1_TypeDef *)     UART1_BASE)
#define UART2                 ((     UART_TypeDef *)     UART2_BASE)
#define UART3                 ((     UART_TypeDef *)     UART3_BASE)
#define PWM1                  ((      PWM_TypeDef *)      PWM1_BASE)
#define I2C0                  ((      I2C_TypeDef *)      I2C0_BASE)
#define I2C1                  ((      I2C_TypeDef *)      I2C1_BASE)
#define I2C2                  ((      I2C_TypeDef *)      I2C2_BASE)
#define I2S                   ((      I2S_TypeDef *)       I2S_BASE)
#define SPI                   ((      SPI_TypeDef *)       SPI_BASE)
#define RTC                   ((      RTC_TypeDef *)       RTC_BASE)
#define GPIOINT               ((  GPIOINT_TypeDef *)   GPIOINT_BASE)
#define PINCON                ((   PINCON_TypeDef *)    PINCON_BASE)
#define SSP0                  ((      SSP_TypeDef *)      SSP0_BASE)
#define SSP1                  ((      SSP_TypeDef *)      SSP1_BASE)
#define ADC                   ((      ADC_TypeDef *)       ADC_BASE)
#define DAC                   ((      DAC_TypeDef *)       DAC_BASE)
#define CANAF_RAM             ((CANAF_RAM_TypeDef *) CANAF_RAM_BASE)
#define CANAF                 ((    CANAF_TypeDef *)     CANAF_BASE)
#define CANCR                 ((    CANCR_TypeDef *)     CANCR_BASE)
#define CAN1                  ((      CAN_TypeDef *)      CAN1_BASE)
#define CAN2                  ((      CAN_TypeDef *)      CAN2_BASE)
#define MCPWM                 ((    MCPWM_TypeDef *)     MCPWM_BASE)
#define QEI                   ((      QEI_TypeDef *)       QEI_BASE)
#define EMAC                  ((     EMAC_TypeDef *)      EMAC_BASE)
#define GPDMA                 ((    GPDMA_TypeDef *)     GPDMA_BASE)
#define GPDMACH0              ((  GPDMACH_TypeDef *)  GPDMACH0_BASE)
#define GPDMACH1              ((  GPDMACH_TypeDef *)  GPDMACH1_BASE)
#define GPDMACH2              ((  GPDMACH_TypeDef *)  GPDMACH2_BASE)
#define GPDMACH3              ((  GPDMACH_TypeDef *)  GPDMACH3_BASE)
#define GPDMACH4              ((  GPDMACH_TypeDef *)  GPDMACH4_BASE)
#define GPDMACH5              ((  GPDMACH_TypeDef *)  GPDMACH5_BASE)
#define GPDMACH6              ((  GPDMACH_TypeDef *)  GPDMACH6_BASE)
#define GPDMACH7              ((  GPDMACH_TypeDef *)  GPDMACH7_BASE)
#define USB                   ((      USB_TypeDef *)       USB_BASE)

/******************************************************************************/
/*                         Peripheral memory map                              */
/******************************************************************************/
/* Base addresses                                                             */
#define LPC_FLASH_BASE        (0x00000000UL)
#define LPC_RAM_BASE          (0x10000000UL)
#ifdef __LPC17XX_REV00
#define LPC_AHBRAM0_BASE      (0x20000000UL)
#define LPC_AHBRAM1_BASE      (0x20004000UL)
#else
#define LPC_AHBRAM0_BASE      (0x2007C000UL)
#define LPC_AHBRAM1_BASE      (0x20080000UL)
#endif
#define LPC_GPIO_BASE         (0x2009C000UL)
#define LPC_APB0_BASE         (0x40000000UL)
#define LPC_APB1_BASE         (0x40080000UL)
#define LPC_AHB_BASE          (0x50000000UL)
#define LPC_CM3_BASE          (0xE0000000UL)

/* APB0 peripherals                                                           */
#define LPC_WDT_BASE          (LPC_APB0_BASE + 0x00000)
#define LPC_TIM0_BASE         (LPC_APB0_BASE + 0x04000)
#define LPC_TIM1_BASE         (LPC_APB0_BASE + 0x08000)
#define LPC_UART0_BASE        (LPC_APB0_BASE + 0x0C000)
#define LPC_UART1_BASE        (LPC_APB0_BASE + 0x10000)
#define LPC_PWM1_BASE         (LPC_APB0_BASE + 0x18000)
#define LPC_I2C0_BASE         (LPC_APB0_BASE + 0x1C000)
#define LPC_SPI_BASE          (LPC_APB0_BASE + 0x20000)
#define LPC_RTC_BASE          (LPC_APB0_BASE + 0x24000)
#define LPC_GPIOINT_BASE      (LPC_APB0_BASE + 0x28080)
#define LPC_PINCON_BASE       (LPC_APB0_BASE + 0x2C000)
#define LPC_SSP1_BASE         (LPC_APB0_BASE + 0x30000)
#define LPC_ADC_BASE          (LPC_APB0_BASE + 0x34000)
#define LPC_CANAF_RAM_BASE    (LPC_APB0_BASE + 0x38000)
#define LPC_CANAF_BASE        (LPC_APB0_BASE + 0x3C000)
#define LPC_CANCR_BASE        (LPC_APB0_BASE + 0x40000)
#define LPC_CAN1_BASE         (LPC_APB0_BASE + 0x44000)
#define LPC_CAN2_BASE         (LPC_APB0_BASE + 0x48000)
#define LPC_I2C1_BASE         (LPC_APB0_BASE + 0x5C000)

/* APB1 peripherals                                                           */
#define LPC_SSP0_BASE         (LPC_APB1_BASE + 0x08000)
#define LPC_DAC_BASE          (LPC_APB1_BASE + 0x0C000)
#define LPC_TIM2_BASE         (LPC_APB1_BASE + 0x10000)
#define LPC_TIM3_BASE         (LPC_APB1_BASE + 0x14000)
#define LPC_UART2_BASE        (LPC_APB1_BASE + 0x18000)
#define LPC_UART3_BASE        (LPC_APB1_BASE + 0x1C000)
#define LPC_I2C2_BASE         (LPC_APB1_BASE + 0x20000)
#define LPC_I2S_BASE          (LPC_APB1_BASE + 0x28000)
#define LPC_RIT_BASE          (LPC_APB1_BASE + 0x30000)
#define LPC_MCPWM_BASE        (LPC_APB1_BASE + 0x38000)
#define LPC_QEI_BASE          (LPC_APB1_BASE + 0x3C000)
#define LPC_SC_BASE           (LPC_APB1_BASE + 0x7C000)

/* AHB peripherals                                                            */
#define LPC_EMAC_BASE         (LPC_AHB_BASE  + 0x00000)
#define LPC_GPDMA_BASE        (LPC_AHB_BASE  + 0x04000)
#define LPC_GPDMACH0_BASE     (LPC_AHB_BASE  + 0x04100)
#define LPC_GPDMACH1_BASE     (LPC_AHB_BASE  + 0x04120)
#define LPC_GPDMACH2_BASE     (LPC_AHB_BASE  + 0x04140)
#define LPC_GPDMACH3_BASE     (LPC_AHB_BASE  + 0x04160)
#define LPC_GPDMACH4_BASE     (LPC_AHB_BASE  + 0x04180)
#define LPC_GPDMACH5_BASE     (LPC_AHB_BASE  + 0x041A0)
#define LPC_GPDMACH6_BASE     (LPC_AHB_BASE  + 0x041C0)
#define LPC_GPDMACH7_BASE     (LPC_AHB_BASE  + 0x041E0)
#define LPC_USB_BASE          (LPC_AHB_BASE  + 0x0C000)

/* GPIOs                                                                      */
#define LPC_GPIO0_BASE        (LPC_GPIO_BASE + 0x00000)
#define LPC_GPIO1_BASE        (LPC_GPIO_BASE + 0x00020)
#define LPC_GPIO2_BASE        (LPC_GPIO_BASE + 0x00040)
#define LPC_GPIO3_BASE        (LPC_GPIO_BASE + 0x00060)
#define LPC_GPIO4_BASE        (LPC_GPIO_BASE + 0x00080)


/******************************************************************************/
/*                         Peripheral declaration                             */
/******************************************************************************/
#define LPC_SC                ((LPC_SC_TypeDef        *) LPC_SC_BASE       )
#define LPC_GPIO0             ((LPC_GPIO_TypeDef      *) LPC_GPIO0_BASE    )
#define LPC_GPIO1             ((LPC_GPIO_TypeDef      *) LPC_GPIO1_BASE    )
#define LPC_GPIO2             ((LPC_GPIO_TypeDef      *) LPC_GPIO2_BASE    )
#define LPC_GPIO3             ((LPC_GPIO_TypeDef      *) LPC_GPIO3_BASE    )
#define LPC_GPIO4             ((LPC_GPIO_TypeDef      *) LPC_GPIO4_BASE    )
#define LPC_WDT               ((LPC_WDT_TypeDef       *) LPC_WDT_BASE      )
#define LPC_TIM0              ((LPC_TIM_TypeDef       *) LPC_TIM0_BASE     )
#define LPC_TIM1              ((LPC_TIM_TypeDef       *) LPC_TIM1_BASE     )
#define LPC_TIM2              ((LPC_TIM_TypeDef       *) LPC_TIM2_BASE     )
#define LPC_TIM3              ((LPC_TIM_TypeDef       *) LPC_TIM3_BASE     )
#define LPC_RIT               ((LPC_RIT_TypeDef       *) LPC_RIT_BASE      )
#define LPC_UART0             ((LPC_UART_TypeDef      *) LPC_UART0_BASE    )
#define LPC_UART1             ((LPC_UART1_TypeDef     *) LPC_UART1_BASE    )
#define LPC_UART2             ((LPC_UART_TypeDef      *) LPC_UART2_BASE    )
#define LPC_UART3             ((LPC_UART_TypeDef      *) LPC_UART3_BASE    )
#define LPC_PWM1              ((LPC_PWM_TypeDef       *) LPC_PWM1_BASE     )
#define LPC_I2C0              ((LPC_I2C_TypeDef       *) LPC_I2C0_BASE     )
#define LPC_I2C1              ((LPC_I2C_TypeDef       *) LPC_I2C1_BASE     )
#define LPC_I2C2              ((LPC_I2C_TypeDef       *) LPC_I2C2_BASE     )
#define LPC_I2S               ((LPC_I2S_TypeDef       *) LPC_I2S_BASE      )
#define LPC_SPI               ((LPC_SPI_TypeDef       *) LPC_SPI_BASE      )
#define LPC_RTC               ((LPC_RTC_TypeDef       *) LPC_RTC_BASE      )
#define LPC_GPIOINT           ((LPC_GPIOINT_TypeDef   *) LPC_GPIOINT_BASE  )
#define LPC_PINCON            ((LPC_PINCON_TypeDef    *) LPC_PINCON_BASE   )
#define LPC_SSP0              ((LPC_SSP_TypeDef       *) LPC_SSP0_BASE     )
#define LPC_SSP1              ((LPC_SSP_TypeDef       *) LPC_SSP1_BASE     )
#define LPC_ADC               ((LPC_ADC_TypeDef       *) LPC_ADC_BASE      )
#define LPC_DAC               ((LPC_DAC_TypeDef       *) LPC_DAC_BASE      )
#define LPC_CANAF_RAM         ((LPC_CANAF_RAM_TypeDef *) LPC_CANAF_RAM_BASE)
#define LPC_CANAF             ((LPC_CANAF_TypeDef     *) LPC_CANAF_BASE    )
#define LPC_CANCR             ((LPC_CANCR_TypeDef     *) LPC_CANCR_BASE    )
#define LPC_CAN1              ((LPC_CAN_TypeDef       *) LPC_CAN1_BASE     )
#define LPC_CAN2              ((LPC_CAN_TypeDef       *) LPC_CAN2_BASE     )
#define LPC_MCPWM             ((LPC_MCPWM_TypeDef     *) LPC_MCPWM_BASE    )
#define LPC_QEI               ((LPC_QEI_TypeDef       *) LPC_QEI_BASE      )
#define LPC_EMAC              ((LPC_EMAC_TypeDef      *) LPC_EMAC_BASE     )
#define LPC_GPDMA             ((LPC_GPDMA_TypeDef     *) LPC_GPDMA_BASE    )
#define LPC_GPDMACH0          ((LPC_GPDMACH_TypeDef   *) LPC_GPDMACH0_BASE )
#define LPC_GPDMACH1          ((LPC_GPDMACH_TypeDef   *) LPC_GPDMACH1_BASE )
#define LPC_GPDMACH2          ((LPC_GPDMACH_TypeDef   *) LPC_GPDMACH2_BASE )
#define LPC_GPDMACH3          ((LPC_GPDMACH_TypeDef   *) LPC_GPDMACH3_BASE )
#define LPC_GPDMACH4          ((LPC_GPDMACH_TypeDef   *) LPC_GPDMACH4_BASE )
#define LPC_GPDMACH5          ((LPC_GPDMACH_TypeDef   *) LPC_GPDMACH5_BASE )
#define LPC_GPDMACH6          ((LPC_GPDMACH_TypeDef   *) LPC_GPDMACH6_BASE )
#define LPC_GPDMACH7          ((LPC_GPDMACH_TypeDef   *) LPC_GPDMACH7_BASE )
#define LPC_USB               ((LPC_USB_TypeDef       *) LPC_USB_BASE      )

#endif  // __LPC17xx_H__


#endif
