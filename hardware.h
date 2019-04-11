/* *****************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 LeafLabs LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 * ****************************************************************************/

#ifndef __HARDWARE_H
#define __HARDWARE_H

#include "stm32f10x_type.h"
#include "cortexm3_macro.h"
#include "common.h"

//#define USE_USART           // В основном используется для целей отладки
#define SPI_SPEED 0x00      // Скорость SPI порта
#define SS_PIN_PA4 // SS_PIN_PA4
//#define USE_HW_CRC        // TODO: А надо ли это в загрузчике?
//#define USE_BACKUP_REGS   // TODO: А надо ли это в загрузчике?

typedef unsigned int size_t;   /* see <stddef.h> */

typedef unsigned         int  uint32_t;
typedef unsigned short   int  uint16_t;
typedef unsigned         char uint8_t;
#define     __IO    volatile             /*!< Defines 'read / write' permissions */

#ifdef USE_USART
    #include <string.h>
    extern char *myMessages[];
        
    #define  RCC_APB2ENR_USART1EN                ((uint32_t)0x00004000)         /*!< USART1 clock enable */

    #define  GPIO_CRH_MODE9                      ((uint32_t)0x00000030)        /*!< MODE9[1:0] bits (Port x mode bits, pin 9) */
    #define  GPIO_CRH_MODE9_0                    ((uint32_t)0x00000010)        /*!< Bit 0 */
    #define  GPIO_CRH_MODE9_1                    ((uint32_t)0x00000020)        /*!< Bit 1 */
    #define  GPIO_CRH_MODE10                     ((uint32_t)0x00000300)        /*!< MODE10[1:0] bits (Port x mode bits, pin 10) */
    #define  GPIO_CRH_MODE10_0                   ((uint32_t)0x00000100)        /*!< Bit 0 */
    #define  GPIO_CRH_MODE10_1                   ((uint32_t)0x00000200)        /*!< Bit 1 */

    #define  GPIO_CRH_CNF9                       ((uint32_t)0x000000C0)        /*!< CNF9[1:0] bits (Port x configuration bits, pin 9) */
    #define  GPIO_CRH_CNF9_0                     ((uint32_t)0x00000040)        /*!< Bit 0 */
    #define  GPIO_CRH_CNF9_1                     ((uint32_t)0x00000080)        /*!< Bit 1 */
    #define  GPIO_CRH_CNF10                      ((uint32_t)0x00000C00)        /*!< CNF10[1:0] bits (Port x configuration bits, pin 10) */
    #define  GPIO_CRH_CNF10_0                    ((uint32_t)0x00000400)        /*!< Bit 0 */
    #define  GPIO_CRH_CNF10_1                    ((uint32_t)0x00000800)        /*!< Bit 1 */

    #define USART1_BASE           (APB2PERIPH_BASE + 0x3800)
    //    #define USART3_BASE           (APB1PERIPH_BASE + 0x4800)
  
    /** 
     * @brief Universal Synchronous Asynchronous Receiver Transmitter
     */
    
    typedef struct
    {
    __IO uint16_t SR;
    uint16_t  RESERVED0;
    __IO uint16_t DR;
    uint16_t  RESERVED1;
    __IO uint16_t BRR;
    uint16_t  RESERVED2;
    __IO uint16_t CR1;
    uint16_t  RESERVED3;
    __IO uint16_t CR2;
    uint16_t  RESERVED4;
    __IO uint16_t CR3;
    uint16_t  RESERVED5;
    __IO uint16_t GTPR;
    uint16_t  RESERVED6;
    } USART_TypeDef;

    #define USART_CR1_UE                        ((uint16_t)0x2000)            /*!< USART Enable */
    #define USART_CR1_TE                        ((uint16_t)0x0008)            /*!< Transmitter Enable */
    #define USART_SR_TXE                        ((uint16_t)0x0080)            /*!< Transmit Data Register Empty */
    #define APB1PERIPH_BASE       PERIPH_BASE
    #define USART1              ((USART_TypeDef *) USART1_BASE)
    
 #endif // USE_USART


/* macro'd register and peripheral definitions */
    #define RCC   ((u32)0x40021000)
    #define FLASH ((u32)0x40022000)
    #define GPIOA ((u32)0x40010800)
    #define GPIOB ((u32)0x40010C00)
    #define GPIOC ((u32)0x40011000)
    #define GPIOD ((u32)0x40011400)
    #define GPIOE ((u32)0x40011800)
    #define GPIOF ((u32)0x40011C00)
    #define GPIOG ((u32)0x40012000)
    #define AFIO_BASE               ((u32)0x40010000)
    #define AFIO_MAPR               (AFIO_BASE + 0x04)


    #define RCC_CR      RCC
    #define RCC_CFGR    (RCC + 0x04)
    #define RCC_CIR     (RCC + 0x08)
    #define RCC_AHBENR  (RCC + 0x14)
    #define RCC_APB2ENR (RCC + 0x18)
    #define RCC_APB1ENR (RCC + 0x1C)

    #define FLASH_ACR     (FLASH + 0x00)
    #define FLASH_KEYR    (FLASH + 0x04)
    #define FLASH_OPTKEYR (FLASH + 0x08)
    #define FLASH_SR      (FLASH + 0x0C)
    #define FLASH_CR      (FLASH + 0x10)
    #define FLASH_AR      (FLASH + 0x14)
    #define FLASH_OBR     (FLASH + 0x1C)
    #define FLASH_WRPR    (FLASH + 0x20)

    #define FLASH_KEY1     0x45670123
    #define FLASH_KEY2     0xCDEF89AB
    #define FLASH_RDPRT    0x00A5
    #define FLASH_SR_BSY   0x01
    #define FLASH_CR_PER   0x02
    #define FLASH_CR_PG    0x01
    #define FLASH_CR_START 0x40

    #define GPIO_CRL(port)  port
    #define GPIO_CRH(port)  (port+0x04)
    #define GPIO_IDR(port)  (port+0x08)
    #define GPIO_ODR(port)  (port+0x0c)
    #define GPIO_BSRR(port) (port+0x10)
    #define GPIO_CR(port,pin) (port + (0x04*(pin>7)))

    #define CR_OUTPUT_OD        0x05
    #define CR_OUTPUT_PP        0x01
    #define CR_INPUT            0x04
    #define CR_INPUT_PU_PD      0x08

    #define SCS_BASE   ((u32)0xE000E000)
    #define NVIC_BASE  (SCS_BASE + 0x0100)
    #define SCB_BASE   (SCS_BASE + 0x0D00)


    #define SCS      0xE000E000
    #define NVIC     (SCS+0x100)
    #define SCB      (SCS+0xD00)
    #define STK      (SCS+0x10)

    #define SCB_VTOR (SCB+0x08)
    #define STK_CTRL (STK+0x00)

/* AIRCR  */
    #define AIRCR_RESET         0x05FA0000
    #define AIRCR_RESET_REQ     (AIRCR_RESET | (u32)0x04);

// SWD and JTAG DEBUGGING
    #define AFIO_MAPR_SWJ_CFG                      (0x7 << 24)
    #define AFIO_MAPR_SWJ_CFG_FULL_SWJ             (0x0 << 24)
    #define AFIO_MAPR_SWJ_CFG_FULL_SWJ_NO_NJRST    (0x1 << 24)
    #define AFIO_MAPR_SWJ_CFG_NO_JTAG_SW           (0x2 << 24)
    #define AFIO_MAPR_SWJ_CFG_NO_JTAG_NO_SW        (0x4 << 24)

// more bit twiddling to set Control register bits
    #define CR_SHITF(pin) ((pin - 8*(pin>7))<<2)

    #define SET_REG(addr,val) do { *(vu32*)(addr)=val; } while(0)
    #define GET_REG(addr)     (*(vu32*)(addr))

//COMMANDS SPI Flash
    #define W_EN 	    0x06	//write enable
    #define W_DE	    0x04	//write disable
    #define R_SR1	    0x05	//read status reg 1
    #define R_SR2	    0x35	//read status reg 2
    #define W_SR	    0x01	//write status reg
    #define PAGE_PGM	0x02	//page program
    #define QPAGE_PGM	0x32	//quad input page program
    #define BLK_E_64K	0xD8	//block erase 64KB
    #define BLK_E_32K	0x52	//block erase 32KB
    #define SECTOR_E	0x20	//sector erase 4KB
    #define CHIP_ERASE	0xc7	//chip erase
    #define CHIP_ERASE2	0x60	//=CHIP_ERASE
    #define E_SUSPEND	0x75	//erase suspend
    #define E_RESUME	0x7a	//erase resume
    #define PDWN		0xb9	//power down
    #define HIGH_PERF_M	0xa3	//high performance mode
    #define CONT_R_RST	0xff	//continuous read mode reset
    #define RELEASE		0xab	//release power down or HPM/Dev ID (deprecated)
    #define R_MANUF_ID	0x90	//read Manufacturer and Dev ID (deprecated)
    #define R_UNIQUE_ID	0x4b	//read unique ID (suggested)
    #define R_JEDEC_ID	0x9f	//read JEDEC ID = Manuf+ID (suggested)
    #define READ		0x03
    #define FAST_READ	0x0b

    #define SR1_BUSY_MASK	0x01
    #define SR1_WEN_MASK	0x02

    #define DEFAULT_TIMEOUT 200

// Загрузчик заточен под микросхему m25p40
    #define WINBOND_MANUF	  0x20
    #define flash_id()        0x2013
    #define flash_bytes()     524288
    #define flash_pages()     2048
    #define flash_sectors()   256
    #define flash_blocks()    8

#if defined   (SS_PIN_PA0)
    #define		CS_LOW 	GPIO_A->BSRR = GPIO_BSRR_BR0;
    #define 	CS_HIGH GPIO_A->BSRR = GPIO_BSRR_BS0;
#elif defined (SS_PIN_PA4)
    #define		CS_LOW 	GPIO_A->BSRR = GPIO_BSRR_BR4;
    #define 	CS_HIGH GPIO_A->BSRR = GPIO_BSRR_BS4;
#endif

#define     select()    CS_LOW 
#define     deselect()  CS_HIGH
#define     bool    uint8_t
#define     true    1
#define     false   0

#define FIRMWARE_BLOCK_SIZE		(16u)				//!< Size of each firmware block
//#define FIRMWARE_START_OFFSET	(10u)				//!< Start offset for firmware in flash (DualOptiboot wants to keeps a signature first)
#define FIRMWARE_START_OFFSET	(16u)
#define FIRMWARE_START_ADDRESS  0x1000              // Сдвигаемся на размер загрузчика

//TODO: #define SPI_port PA0
char mybuff[0x10];
uint32_t SPIflash_CRC;
uint32_t flash_CRC;

#define PERIPH_BASE           0x40000000U /*!< Peripheral base address in the alias region */

#ifdef USE_HW_CRC
    #define RCC_AHBENR_CRCEN                    ((uint16_t)0x0040)            /*!< CRC clock enable */
    #define AHBPERIPH_BASE        (PERIPH_BASE + 0x00020000U)
    #define CRC_BASE              (AHBPERIPH_BASE + 0x00003000U)

    /********************  Bit definition for CRC_CR register  ********************/
    #define CRC_CR_RESET_Pos                    (0U)                               
    #define CRC_CR_RESET_Msk                    (0x1U << CRC_CR_RESET_Pos)         /*!< 0x00000001 */
    #define CRC_CR_RESET                        CRC_CR_RESET_Msk                   /*!< RESET bit */

    /** 
     * @brief CRC calculation unit 
     */

    typedef struct
    {
    __IO uint32_t DR;           /*!< CRC Data register,                           Address offset: 0x00 */
    __IO uint8_t  IDR;          /*!< CRC Independent data register,               Address offset: 0x04 */
    uint8_t       RESERVED0;    /*!< Reserved,                                    Address offset: 0x05 */
    uint16_t      RESERVED1;    /*!< Reserved,                                    Address offset: 0x06 */  
    __IO uint32_t CR;           /*!< CRC Control register,                        Address offset: 0x08 */ 
    } CRC_TypeDef;

    #define CRC                 ((CRC_TypeDef *)CRC_BASE)
 #endif // USE_HW_CRC

#define  RCC_APB2ENR_AFIOEN                  ((uint32_t)0x00000001)         /*!< Alternate Function I/O clock enable */
#define  RCC_APB2ENR_IOPAEN                  ((uint32_t)0x00000004)         /*!< I/O port A clock enable */

#define  GPIO_CRL_MODE0                      ((uint32_t)0x00000003)        /*!< MODE0[1:0] bits (Port x mode bits, pin 0) */
#define  GPIO_CRL_MODE0_0                    ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  GPIO_CRL_MODE0_1                    ((uint32_t)0x00000002)        /*!< Bit 1 */

#define  GPIO_CRL_MODE4                      ((uint32_t)0x00030000)        /*!< MODE4[1:0] bits (Port x mode bits, pin 4) */
#define  GPIO_CRL_MODE4_0                    ((uint32_t)0x00010000)        /*!< Bit 0 */
#define  GPIO_CRL_MODE4_1                    ((uint32_t)0x00020000)        /*!< Bit 1 */

#define  GPIO_CRL_MODE5                      ((uint32_t)0x00300000)        /*!< MODE5[1:0] bits (Port x mode bits, pin 5) */
#define  GPIO_CRL_MODE5_0                    ((uint32_t)0x00100000)        /*!< Bit 0 */
#define  GPIO_CRL_MODE5_1                    ((uint32_t)0x00200000)        /*!< Bit 1 */

#define  GPIO_CRL_MODE6                      ((uint32_t)0x03000000)        /*!< MODE6[1:0] bits (Port x mode bits, pin 6) */
#define  GPIO_CRL_MODE6_0                    ((uint32_t)0x01000000)        /*!< Bit 0 */
#define  GPIO_CRL_MODE6_1                    ((uint32_t)0x02000000)        /*!< Bit 1 */

#define  GPIO_CRL_MODE7                      ((uint32_t)0x30000000)        /*!< MODE7[1:0] bits (Port x mode bits, pin 7) */
#define  GPIO_CRL_MODE7_0                    ((uint32_t)0x10000000)        /*!< Bit 0 */
#define  GPIO_CRL_MODE7_1                    ((uint32_t)0x20000000)        /*!< Bit 1 */

#define  GPIO_CRL_CNF0                       ((uint32_t)0x0000000C)        /*!< CNF0[1:0] bits (Port x configuration bits, pin 0) */
#define  GPIO_CRL_CNF0_0                     ((uint32_t)0x00000004)        /*!< Bit 0 */
#define  GPIO_CRL_CNF0_1                     ((uint32_t)0x00000008)        /*!< Bit 1 */

#define  GPIO_CRL_CNF4                       ((uint32_t)0x000C0000)        /*!< CNF4[1:0] bits (Port x configuration bits, pin 4) */
#define  GPIO_CRL_CNF4_0                     ((uint32_t)0x00040000)        /*!< Bit 0 */
#define  GPIO_CRL_CNF4_1                     ((uint32_t)0x00080000)        /*!< Bit 1 */

#define  GPIO_CRL_CNF5                       ((uint32_t)0x00C00000)        /*!< CNF5[1:0] bits (Port x configuration bits, pin 5) */
#define  GPIO_CRL_CNF5_0                     ((uint32_t)0x00400000)        /*!< Bit 0 */
#define  GPIO_CRL_CNF5_1                     ((uint32_t)0x00800000)        /*!< Bit 1 */

#define  GPIO_CRL_CNF6                       ((uint32_t)0x0C000000)        /*!< CNF6[1:0] bits (Port x configuration bits, pin 6) */
#define  GPIO_CRL_CNF6_0                     ((uint32_t)0x04000000)        /*!< Bit 0 */
#define  GPIO_CRL_CNF6_1                     ((uint32_t)0x08000000)        /*!< Bit 1 */

#define  GPIO_CRL_CNF7                       ((uint32_t)0xC0000000)        /*!< CNF7[1:0] bits (Port x configuration bits, pin 7) */
#define  GPIO_CRL_CNF7_0                     ((uint32_t)0x40000000)        /*!< Bit 0 */
#define  GPIO_CRL_CNF7_1                     ((uint32_t)0x80000000)        /*!< Bit 1 */

#define GPIO_BSRR_BS0                        ((uint32_t)0x00000001)        /*!< Port x Set bit 0 */
#define GPIO_BSRR_BS4                        ((uint32_t)0x00000010)        /*!< Port x Set bit 4 */
#define GPIO_BSRR_BS5                        ((uint32_t)0x00000020)        /*!< Port x Set bit 5 */
#define GPIO_BSRR_BS6                        ((uint32_t)0x00000040)        /*!< Port x Set bit 6 */
#define GPIO_BSRR_BS7                        ((uint32_t)0x00000080)        /*!< Port x Set bit 7 */

//TODO: Сделать определение ноги SS в одном месте
#define GPIO_BSRR_BR0                        ((uint32_t)0x00010000)        /*!< Port x Reset bit 0 */
#define GPIO_BSRR_BR4                        ((uint32_t)0x00100000)        /*!< Port x Reset bit 4 */
#define GPIO_BSRR_BR5                        ((uint32_t)0x00200000)        /*!< Port x Reset bit 5 */
#define GPIO_BSRR_BR6                        ((uint32_t)0x00400000)        /*!< Port x Reset bit 6 */
#define GPIO_BSRR_BR7                        ((uint32_t)0x00800000)        /*!< Port x Reset bit 7 */

#define RCC_APB2ENR_SPI1EN_Pos               (12U)                             
#define RCC_APB2ENR_SPI1EN_Msk               (0x1U << RCC_APB2ENR_SPI1EN_Pos)  /*!< 0x00001000 */
#define RCC_APB2ENR_SPI1EN                   RCC_APB2ENR_SPI1EN_Msk            /*!< SPI 1 clock enable */

#define SPI_SR_BSY_Pos                      (7U)                               
#define SPI_SR_BSY_Msk                      (0x1U << SPI_SR_BSY_Pos)           /*!< 0x00000080 */
#define SPI_SR_BSY                          SPI_SR_BSY_Msk                     /*!< Busy flag */

#define SPI_SR_TXE_Pos                      (1U)                               
#define SPI_SR_TXE_Msk                      (0x1U << SPI_SR_TXE_Pos)           /*!< 0x00000002 */
#define SPI_SR_TXE                          SPI_SR_TXE_Msk                     /*!< Transmit buffer Empty */

#define SPI_SR_RXNE_Pos                     (0U)                               
#define SPI_SR_RXNE_Msk                     (0x1U << SPI_SR_RXNE_Pos)          /*!< 0x00000001 */
#define SPI_SR_RXNE                         SPI_SR_RXNE_Msk                    /*!< Receive buffer Not Empty */

#define SPI_CR1_BR_Pos                      (3U)                               
#define SPI_CR1_BR_Msk                      (0x7U << SPI_CR1_BR_Pos)           /*!< 0x00000038 */
#define SPI_CR1_BR                          SPI_CR1_BR_Msk                     /*!< BR[2:0] bits (Baud Rate Control) */
#define SPI_CR1_BR_0                        (0x1U << SPI_CR1_BR_Pos)           /*!< 0x00000008 */
#define SPI_CR1_BR_1                        (0x2U << SPI_CR1_BR_Pos)           /*!< 0x00000010 */
#define SPI_CR1_BR_2                        (0x4U << SPI_CR1_BR_Pos)           /*!< 0x00000020 */

#define SPI_CR1_SSI_Pos                     (8U)                               
#define SPI_CR1_SSI_Msk                     (0x1U << SPI_CR1_SSI_Pos)          /*!< 0x00000100 */
#define SPI_CR1_SSI                         SPI_CR1_SSI_Msk                    /*!< Internal slave select */
#define SPI_CR1_SSM_Pos                     (9U)                               
#define SPI_CR1_SSM_Msk                     (0x1U << SPI_CR1_SSM_Pos)          /*!< 0x00000200 */
#define SPI_CR1_SSM                         SPI_CR1_SSM_Msk                    /*!< Software slave management */

#define SPI_CR1_MSTR_Pos                    (2U)                               
#define SPI_CR1_MSTR_Msk                    (0x1U << SPI_CR1_MSTR_Pos)         /*!< 0x00000004 */
#define SPI_CR1_MSTR                        SPI_CR1_MSTR_Msk                   /*!< Master Selection */

#define SPI_CR1_SPE_Pos                     (6U)                               
#define SPI_CR1_SPE_Msk                     (0x1U << SPI_CR1_SPE_Pos)          /*!< 0x00000040 */
#define SPI_CR1_SPE                         SPI_CR1_SPE_Msk                    /*!< SPI Enable */

typedef struct
 {
  __IO uint32_t CR1;
  __IO uint32_t CR2;
  __IO uint32_t SR;
  __IO uint32_t DR;
  __IO uint32_t CRCPR;
  __IO uint32_t RXCRCR;
  __IO uint32_t TXCRCR;
  __IO uint32_t I2SCFGR;
 } SPI_TypeDef;

#define SPI1                ((SPI_TypeDef *)SPI1_BASE)
#define SPI1_BASE             (APB2PERIPH_BASE + 0x00003000U)

/** 
  * @brief General Purpose I/O
  */

typedef struct
 {
  __IO uint32_t CRL;
  __IO uint32_t CRH;
  __IO uint32_t IDR;
  __IO uint32_t ODR;
  __IO uint32_t BSRR;
  __IO uint32_t BRR;
  __IO uint32_t LCKR;
 } GPIO_TypeDef;


#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000U)
#define GPIOA_BASE            (APB2PERIPH_BASE + 0x00000800U)
#define GPIOB_BASE            (APB2PERIPH_BASE + 0x0C00)
#define GPIO_A               ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIO_B               ((GPIO_TypeDef *)GPIOB_BASE)

#define GPIO_CRL_CNF5                       ((uint32_t)0x00C00000)        /*!< CNF5[1:0] bits (Port x configuration bits, pin 5) */
#define GPIO_CRL_CNF5_0                     ((uint32_t)0x00400000)        /*!< Bit 0 */
#define GPIO_CRL_CNF5_1                     ((uint32_t)0x00800000)        /*!< Bit 1 */

#define GPIO_CRL_CNF7                       ((uint32_t)0xC0000000)        /*!< CNF7[1:0] bits (Port x configuration bits, pin 7) */
#define GPIO_CRL_CNF7_0                     ((uint32_t)0x40000000)        /*!< Bit 0 */
#define GPIO_CRL_CNF7_1                     ((uint32_t)0x80000000)        /*!< Bit 1 */

// #define GPIO_CRL_MODE5_Pos                   (20U)                             
// #define GPIO_CRL_MODE5_Msk                   (0x3U << GPIO_CRL_MODE5_Pos)      /*!< 0x00300000 */
// #define GPIO_CRL_MODE5                       GPIO_CRL_MODE5_Msk                /*!< MODE5[1:0] bits (Port x mode bits, pin 5) */
// #define GPIO_CRL_MODE5_0                     (0x1U << GPIO_CRL_MODE5_Pos)      /*!< 0x00100000 */
// #define GPIO_CRL_MODE5_1                     (0x2U << GPIO_CRL_MODE5_Pos)      /*!< 0x00200000 */

// #define GPIO_CRL_MODE7_Pos                   (28U)                             
// #define GPIO_CRL_MODE7_Msk                   (0x3U << GPIO_CRL_MODE7_Pos)      /*!< 0x30000000 */
// #define GPIO_CRL_MODE7                       GPIO_CRL_MODE7_Msk                /*!< MODE7[1:0] bits (Port x mode bits, pin 7) */
// #define GPIO_CRL_MODE7_0                     (0x1U << GPIO_CRL_MODE7_Pos)      /*!< 0x10000000 */
// #define GPIO_CRL_MODE7_1                     (0x2U << GPIO_CRL_MODE7_Pos)      /*!< 0x20000000 */

/* todo: there must be some major misunderstanding in how we access
   regs. The direct access approach (GET_REG) causes the usb init to
   fail upon trying to activate RCC_APB1 |= 0x00800000. However, using
   the struct approach from ST, it works fine...temporarily switching
   to that approach */
typedef struct {
    vu32 CR;
    vu32 CFGR;
    vu32 CIR;
    vu32 APB2RSTR;
    vu32 APB1RSTR;
    vu32 AHBENR;
    vu32 APB2ENR;
    vu32 APB1ENR;
    vu32 BDCR;
    vu32 CSR;
 } RCC_RegStruct;
#define pRCC ((RCC_RegStruct *) RCC)

typedef struct {
    vu32 ISER[2];
    u32  RESERVED0[30];
    vu32 ICER[2];
    u32  RSERVED1[30];
    vu32 ISPR[2];
    u32  RESERVED2[30];
    vu32 ICPR[2];
    u32  RESERVED3[30];
    vu32 IABR[2];
    u32  RESERVED4[62];
    vu32 IPR[15];
 } NVIC_TypeDef;

typedef struct {
    u8 NVIC_IRQChannel;
    u8 NVIC_IRQChannelPreemptionPriority;
    u8 NVIC_IRQChannelSubPriority;
    bool NVIC_IRQChannelCmd; /* TRUE for enable */
 } NVIC_InitTypeDef;

typedef struct {
    vuc32 CPUID;
    vu32 ICSR;
    vu32 VTOR;
    vu32 AIRCR;
    vu32 SCR;
    vu32 CCR;
    vu32 SHPR[3];
    vu32 SHCSR;
    vu32 CFSR;
    vu32 HFSR;
    vu32 DFSR;
    vu32 MMFAR;
    vu32 BFAR;
    vu32 AFSR;
 } SCB_TypeDef;

/** Power interface register map. */
typedef struct pwr_reg_map {
    vu32 CR;      /**< Control register */
    vu32 CSR;     /**< Control and status register */
 } pwr_reg_map;


/** Power peripheral register map base pointer. */
#define pPWR                        ((struct pwr_reg_map*)0x40007000)

#ifdef USE_BACKUP_REGS
    /** Disable backup domain write protection bit */
    #define PWR_CR_DBP                  (1 << 8)

    /** Backup peripheral register map type. */
    typedef struct bkp_reg_map {
        const u32 RESERVED1; ///< Reserved
        vu16 DR1;            ///< Data register 1
        const u16 RESERVED2;
        vu16 DR2;            ///< Data register 2
        const u16 RESERVED3;
        vu16 DR3;            ///< Data register 3
        const u16 RESERVED4;
        vu16 DR4;            ///< Data register 4
        const u16 RESERVED5;
        vu16 DR5;            ///< Data register 5
        const u16 RESERVED6;
        vu16 DR6;            ///< Data register 6
        const u16 RESERVED7;
        vu16 DR7;            ///< Data register 7
        const u16 RESERVED8;
        vu16 DR8;            ///< Data register 8
        const u16 RESERVED9;
        vu16 DR9;            ///< Data register 9
        const u16 RESERVED10;
        vu16 DR10;           ///< Data register 10
        const u16 RESERVED11;
        vu32 RTCCR;          ///< RTC control register
        vu32 CR;             ///< Control register
        vu32 CSR;            ///< Control and status register
    } bkp_reg_map;

    /** Backup peripheral register map base pointer. */
    #define pBKP                        ((struct bkp_reg_map*)0x40006C00)

    void bkp10Write(u16 value);
 #endif // USE_BACKUP_REGS

//void setPin(u32 bank, u8 pin);
//void resetPin(u32 bank, u8 pin);
void gpio_write_bit(u32 bank, u8 pin, u8 val);
unsigned int crMask(int pin);

bool readPin(u32 bank, u8 pin);
void strobePin(u32 bank, u8 pin, u8 count, u32 rate,u8 onState);

void systemHardReset(void);
void systemReset(void);
void setupCLK(void);
void setupLEDAndButton(void);
void setupFLASH(void);
//TODO: void setupSPI (uint8_t);
void setupSPI (void);
#ifdef USE_USART        
 void USART_init(void);
#endif // USE_USART    
#ifdef USE_HW_CRC
    void CRC_init(void);
#endif // USE_HW_CRC
uint8_t CheckFlashImage(void);
void send_string_USART(char*);
void Hex2Ascii (uint8_t);
bool readButtonState(void);
uint16_t fast_read (uint32_t, char *, uint16_t);
bool busy(void);
void erase64kBlock(uint32_t);
void setWriteEnable(bool);


bool checkUserCode(u32 usrAddr);
void jumpToUser(u32 usrAddr);
#ifdef USE_BACKUP_REGS
    int  checkAndClearBootloaderFlag();
#endif // USE_BACKUP_REGS
bool flashWriteWord(u32 addr, u32 word);
bool flashErasePage(u32 addr);
bool flashErasePages(u32 addr, u16 n);
void flashLock(void);
void flashUnlock(void);
void nvicInit(NVIC_InitTypeDef *);
void nvicDisableInterrupts(void);

int getFlashEnd(void);
int getFlashPageSize(void);

#endif
