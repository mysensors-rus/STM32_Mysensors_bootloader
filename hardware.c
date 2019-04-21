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

/**
 *  @file hardware.c
 *
 *  @brief init routines to setup clocks, interrupts, also destructor functions.
 *  does not include USB stuff. EEPROM read/write functions.
 *
 */
#include "common.h"
#include "hardware.h"

//int wTransferSize = 0x400;

#ifdef USE_USART
    char *myMessages[] = {"Bootloader started.\n", "SPI flash found!\n", "SPI flash not found!\n", "New firmware found!\n", 
    "New firmware not found!\n", "Started main program...\n", "Firmware started...\n", "Reboot...\n", "Firmaware complite!\n", "Firmaware flashing error!\n"};

 void Hex2Ascii (uint8_t numb) {
    uint8_t number = numb;
    number = number >> 4;

    if (number > 0x09) 
        number += 0x38;
    else 
        number += 0x30;
    while ((USART1->SR & USART_SR_TXE) != USART_SR_TXE);
    USART1->DR = number;

    number = numb & 0x0F;
    //number = ((number << 4) >> 4);

    if (number > 0x09) 
        number += 0x38;
    else 
        number += 0x30;
    while ((USART1->SR & USART_SR_TXE) != USART_SR_TXE);
    USART1->DR = number;
 }


 #endif // USE_USART

#ifdef USE_HW_CRC
    //FIXME: ругается что функция ничего не возвращает, она не void
    uint32_t __RBIT(uint32_t value)
    {
    asm volatile("rbit r0, r0");
    asm volatile("bx lr");
    }
    #endif // USE_HW_CRC

uint8_t transfer(uint8_t byte)
    {
        SPI1->DR;
        SPI1->DR = byte;
        while(!(SPI1->SR & SPI_SR_TXE));
        while((SPI1->SR & SPI_SR_BSY) != 0);
        return (uint8_t)SPI1->DR; // "... and read the last received data."
    }

bool checkPartNo(uint16_t _partno)
 {
  uint8_t manuf;
  uint16_t id;
  
  select();
  transfer(R_JEDEC_ID);
  manuf = transfer(0x00);
  id = transfer(0x00) << 8;
  id |= transfer(0x00);
  deselect();

  if(manuf != WINBOND_MANUF)
    return false;

  if(id != _partno)
    return false;
  
  return true;
  }

bool begin(uint16_t _partno)
    {
    select();
    transfer(RELEASE);
    deselect();
    uint16_t volatile i;
    for (i=0; i<30; i++); //TODO: Потом отрихтовать задержку
    //  delay_us(5);//>3us
    
    if(!checkPartNo(_partno))
        return false;
    
    return true;
    }

void end()
    {
    select();
    transfer(PDWN);
    deselect();
    //  delayMicroseconds(5);//>3us
    uint8_t i;
    for (i=0; i<255; i++);  //TODO: Потом отрихтовать задержку  
    }

bool busy()
    {
    uint8_t r1;
    select();
    transfer(R_SR1);
    r1 = transfer(0xff);
    deselect();
    if(r1 & SR1_BUSY_MASK)
        return true;
    return false;
    }

uint16_t readByte (uint32_t addr)
    {
    if(busy())
        return 0;
    
    select();
    transfer(READ);
    transfer(addr>>16);
    transfer(addr>>8);
    transfer(addr);
    uint8_t res = transfer(0x00);
    deselect();
    
    return res;
    }

uint16_t read(uint32_t addr,uint8_t *buf,uint16_t n)
 {
  if(busy())
    return 0;
  
  select();
  transfer(READ);
  transfer(addr>>16);
  transfer(addr>>8);
  transfer(addr);
  uint16_t i;
  for(i=0;i<n;i++)
  {
    buf[i] = transfer(0x00);
  }
  deselect();
  
  return n;
 }

uint16_t fast_read (uint32_t addr,char *buf,uint16_t n)
 {
  if(busy())
    return 0;
  
  select();
  transfer(FAST_READ);
  transfer(addr>>16);
  transfer(addr>>8);
  transfer(addr);
  transfer(0x00);
  uint16_t i;
  for(i=0;i<n;i++)
  {
    buf[i] = transfer(0x00);
  }
  deselect();
  
  return n;
 }

void writePage(uint32_t addr_start,uint8_t *buf)
    {
    select();
    transfer(PAGE_PGM);
    transfer(addr_start>>16);
    transfer(addr_start>>8);
    transfer(0x00);
    uint8_t i=0;
    do {
        transfer(buf[i]);
        i++;
    }while(i!=0);
    deselect();
    }

void eraseSector(uint32_t addr_start)
    {
    select();
    transfer(SECTOR_E);
    transfer(addr_start>>16);
    transfer(addr_start>>8);
    transfer(addr_start);
    deselect();
    }

void erase32kBlock(uint32_t addr_start)
    {
    select();
    transfer(BLK_E_32K);
    transfer(addr_start>>16);
    transfer(addr_start>>8);
    transfer(addr_start);
    deselect();
    }

void erase64kBlock(uint32_t addr_start)
    {
    select();
    transfer(BLK_E_64K);
    transfer(addr_start>>16);
    transfer(addr_start>>8);
    transfer(addr_start);
    deselect();
    }

void eraseAll()
    {
    select();
    transfer(CHIP_ERASE);
    deselect();
    }

void eraseSuspend()
    {
    select();
    transfer(E_SUSPEND);
    deselect();
    }

void eraseResume()
    {
    select();
    transfer(E_RESUME);
    deselect();
    }

uint8_t readManufacturer()
    {
    uint8_t c;
    select();
    transfer(R_JEDEC_ID);
    c = transfer(0x00);
    transfer(0x00);
    transfer(0x00);
    deselect();
    return c;
    }

void setWriteEnable(bool cmd)
 {
  select();
  transfer( cmd ? W_EN : W_DE );
  deselect();
 }

uint16_t Spi_Write_Data(uint8_t data)
 {
    //ждём пока опустошится Tx буфер
	while(!(SPI1->SR & SPI_SR_TXE));
	//активируем Chip Select
	CS_LOW   
    //отправляем данные     
	SPI1->DR = data;  

    //ждём пока придёт ответ
	while(!(SPI1->SR & SPI_SR_RXNE));
    //считываем полученные данные
	data = SPI1->DR;  
	//деактивируем Chip Select
	CS_HIGH   
    //возвращаем то, что прочитали
    return data;  
    }

// uint16_t CRC16(uint32_t size) {

    // 	// init crc
    // 	uint16_t crc = ~0;
    //     uint32_t i;
    // 	for (i = 0; i < size; ++i) {
    // 		crc ^= readByte(i + FIRMWARE_START_OFFSET);
    //         uint8_t j;
    // 		for (j = 0; j < 8; ++j) {
    // 			if (crc & 1) {
    // 				crc = (crc >> 1) ^ 0xA001;
    // 			} else {
    // 				crc = (crc >> 1);
    // 			}
    // 		}
    // 	}

    // return crc;
    // }

#ifdef USE_HW_CRC
    //Аппаратный подсчёт CRC
    uint32_t HW_CRC32(const uint8_t* pData, size_t count, uint32_t init) {
        uint32_t crc;
        uint32_t *p32 =  (uint32_t*) pData; //FIXME:
        size_t count32 = count >> 2;
        if (0xFFFFFFFF == init)
                CRC->CR |= CRC_CR_RESET;

        while (count32--) {
                CRC->DR = __RBIT(*p32++);
        }

        crc = __RBIT(CRC->DR);
        count = count % 4;
        if (count) {
                CRC->DR = __RBIT(crc);
                switch (count) {
                case 1:
                        CRC->DR = __RBIT((*p32 & 0x000000FF) ^ crc) >> 24;
                        crc = (crc >> 8) ^ __RBIT(CRC->DR);
                        break;
                case 2:
                        CRC->DR = (__RBIT((*p32 & 0x0000FFFF) ^ crc) >> 16);
                        crc = (crc >> 16) ^ __RBIT(CRC->DR);
                        break;
                case 3:
                        CRC->DR = __RBIT((*p32 & 0x00FFFFFF) ^ crc) >> 8;
                        crc = (crc >> 24) ^ __RBIT(CRC->DR);
                        break;
                }
        }
        return ~crc;
 }
 #endif // USE_HW_CRC

uint8_t CheckFlashImage() {
    if (!begin(flash_id())) {

    #ifdef USE_USART
        send_string_USART(myMessages[2]);
    #endif // USE_USART  

    return 1; // Не найдена флешка
    }

    #ifdef USE_USART
        send_string_USART(myMessages[1]);
    #endif // USE_USART  
      
      //fast_read(FIRMWARE_START_ADDRESS,&mybuff[0],0x10);
      fast_read(0,&mybuff[0],0x10);


      if (mybuff[0]==0x46 && mybuff[1]==0x4c && mybuff[2]==0x58 && mybuff[3]==0x49 && mybuff[4]==0x4d && mybuff[5]==0x47 && mybuff[6]==0x3a)
	{
        // Сигнатура прошивки найдена
        #ifdef USE_USART
            send_string_USART(myMessages[3]);
        #endif // USE_USART  
    
    } else {
        //  // Сигнатура прошивки не найдена
        #ifdef USE_USART
            send_string_USART(myMessages[4]);
        #endif // USE_USART  

    return 2;
    }
    // Всё хорошо, контрольные суммы совпадают
    // Можно обновляться
    return 0;
 } 


void gpio_write_bit(u32 bank, u8 pin, u8 val) {
    val = !val;          // "set" bits are lower than "reset" bits
    SET_REG(GPIO_BSRR(bank), (1U << pin) << (16 * val));
 }

bool readPin(u32 bank, u8 pin) {
    // todo, implement read
    if (GET_REG(GPIO_IDR(bank)) & (0x01 << pin)) {
        return TRUE;
    } else {
        return FALSE;
    }
 }

bool readButtonState() {
    // todo, implement read
    bool state=FALSE;
    #if defined(BUTTON_BANK) && defined (BUTTON_PIN) && defined (BUTTON_PRESSED_STATE)
    if (GET_REG(GPIO_IDR(BUTTON_BANK)) & (0x01 << BUTTON_PIN))
    {
        state = TRUE;
    }

    if (BUTTON_PRESSED_STATE==0)
    {
        state=!state;
    }
    #endif
    return state;
 }

void strobePin(u32 bank, u8 pin, u8 count, u32 rate, u8 onState)
    {
        gpio_write_bit(bank,pin,1-onState);

        u32 c;
        while (count-- > 0)
        {
            for (c = rate; c > 0; c--)
            {
                asm volatile("nop");
            }

            gpio_write_bit( bank,pin,onState);

            for (c = rate; c > 0; c--)
            {
                asm volatile("nop");
            }
            gpio_write_bit( bank,pin,1-onState);
        }
    }

void systemReset(void) {
    SET_REG(RCC_CR, GET_REG(RCC_CR)     | 0x00000001);
    SET_REG(RCC_CFGR, GET_REG(RCC_CFGR) & 0xF8FF0000);
    SET_REG(RCC_CR, GET_REG(RCC_CR)     & 0xFEF6FFFF);
    SET_REG(RCC_CR, GET_REG(RCC_CR)     & 0xFFFBFFFF);
    SET_REG(RCC_CFGR, GET_REG(RCC_CFGR) & 0xFF80FFFF);

    SET_REG(RCC_CIR, 0x00000000);  /* disable all RCC interrupts */
 }

void setupCLK(void) {
    unsigned int StartUpCounter=0;
    /* enable HSE */
    SET_REG(RCC_CR, GET_REG(RCC_CR) | 0x00010001);
    while ((GET_REG(RCC_CR) & 0x00020000) == 0); /* for it to come on */

    /* enable flash prefetch buffer */
    SET_REG(FLASH_ACR, 0x00000012);

    /* Configure PLL */
    #ifdef XTAL12M
    SET_REG(RCC_CFGR, GET_REG(RCC_CFGR) | 0x00110400); /* pll=72Mhz(x6),APB1=36Mhz,AHB=72Mhz */
    #else
    SET_REG(RCC_CFGR, GET_REG(RCC_CFGR) | 0x001D0400); /* pll=72Mhz(x9),APB1=36Mhz,AHB=72Mhz */
    #endif

    SET_REG(RCC_CR, GET_REG(RCC_CR)     | 0x01000000); /* enable the pll */


    #if !defined  (HSE_STARTUP_TIMEOUT)
        #define HSE_STARTUP_TIMEOUT    ((unsigned int)0x0500)   /*!< Time out for HSE start up */
    #endif /* HSE_STARTUP_TIMEOUT */

    while ((GET_REG(RCC_CR) & 0x03000000) == 0 && StartUpCounter < HSE_STARTUP_TIMEOUT)
    {
    //      StartUpCounter++; // This is commented out, so other changes can be committed. It will be uncommented at a later date
    }   /* wait for it to come on */

    if (StartUpCounter>=HSE_STARTUP_TIMEOUT)
    {
        // HSE has not started. Try restarting the processor
        systemHardReset();
    }

    /* Set SYSCLK as PLL */
    SET_REG(RCC_CFGR, GET_REG(RCC_CFGR) | 0x00000002);
    while ((GET_REG(RCC_CFGR) & 0x00000008) == 0); /* wait for it to come on */

    pRCC->APB2ENR |= 0B111111100;// Enable All GPIO channels (A to G)
    pRCC->APB1ENR |= RCC_APB1ENR_USB_CLK;
 }


#ifdef USE_USART

    void USART_init(void)
    {
            pRCC->APB2ENR |= RCC_APB2ENR_USART1EN; // такты на USART1
            GPIO_A->CRH &= (~(GPIO_CRH_CNF9_0)); //Настройка порта передатчика
            GPIO_A->CRH |= (GPIO_CRH_MODE9 | GPIO_CRH_CNF9_1); // AF Push-Pull out (TX)

            USART1->CR1 |= USART_CR1_UE; // Разрешить USART
            USART1->BRR = 0x0271; // 115200 baud
            USART1->CR1 |= USART_CR1_TE; //Разрешить передатчик
    }

    void send_string_USART(char* buff) {
        uint8_t count = strlen(buff);
        uint8_t i;
        for (i=0; i<count; i++) {
            while ((USART1->SR & USART_SR_TXE) != USART_SR_TXE);
            USART1->DR = buff[i];  
        }
    }

 #endif // USE_USART

#ifdef USE_HW_CRC
    void CRC_init(void) {
        pRCC->AHBENR |= RCC_AHBENR_CRCEN;                //Разрешить тактирование CRC-юнита

        CRC->CR = 1;
        __asm("nop");             //Аппаратная готовность за 4 такта, жду...
        __asm("nop");
        __asm("nop");
    }
 #endif // USE_HW_CRC

void setupSPI (void) {    

    //инициализация порта A*************************
    
    pRCC->APB2ENR |=  RCC_APB2ENR_AFIOEN;     //включить тактирование альтернативных функций        /
    pRCC->APB2ENR |=  RCC_APB2ENR_IOPAEN;     //включить тактирование порта А
    
    //вывод управления SS: выход двухтактный, общего назначения,50MHz    
    #if defined (SS_PIN_PA4)                         
        GPIO_A->CRL   |=  GPIO_CRL_MODE4;    //
        GPIO_A->CRL   &= ~GPIO_CRL_CNF4;     //
        GPIO_A->BSRR   =  GPIO_BSRR_BS4;     //
    #elif defined (SS_PIN_PA0)
        GPIO_A->CRL   |=  GPIO_CRL_MODE0;    //
        GPIO_A->CRL   &= ~GPIO_CRL_CNF0;     //
        GPIO_A->BSRR   =  GPIO_BSRR_BS0;     //
    #endif // SS_PIN_PA0

    //вывод SCK: выход двухтактный, альтернативная функция, 50MHz
    GPIO_A->CRL   |=  GPIO_CRL_MODE5;    //
    GPIO_A->CRL   &= ~GPIO_CRL_CNF5;     //
    GPIO_A->CRL   |=  GPIO_CRL_CNF5_1;   //
    
    //вывод MISO: вход цифровой с подтягивающим резистором, подтяжка к плюсу
    GPIO_A->CRL   &= ~GPIO_CRL_MODE6;    //
    GPIO_A->CRL   &= ~GPIO_CRL_CNF6;     //
    GPIO_A->CRL   |=  GPIO_CRL_CNF6_1;   //
    GPIO_A->BSRR   =  GPIO_BSRR_BS6;     //
    
    //вывод MOSI: выход двухтактный, альтернативная функция, 50MHz
    GPIO_A->CRL   |=  GPIO_CRL_MODE7;    //
    GPIO_A->CRL   &= ~GPIO_CRL_CNF7;     //
    GPIO_A->CRL   |=  GPIO_CRL_CNF7_1;   //

    pRCC->APB2ENR |= RCC_APB2ENR_SPI1EN; //подать тактирование                                      /
    SPI1->CR1     = 0x0000;             //очистить первый управляющий регистр
    SPI1->CR2     = 0x0000;             //очистить второй управляющий регистр
    SPI1->CR1    |= SPI_CR1_MSTR;       //контроллер должен быть мастером    
    SPI1->CR1    |= SPI_SPEED;         //задаем скорость
    SPI1->CR1    |= SPI_CR1_SSI;        //обеспечить высокий уровень программного NSS
    SPI1->CR1    |= SPI_CR1_SSM;        //разрешить программное формирование NSS
    SPI1->CR1    |= SPI_CR1_SPE;        //разрешить работу модуля SPI
 }


void setupLEDAndButton (void) {
    // SET_REG(AFIO_MAPR,(GET_REG(AFIO_MAPR) & ~AFIO_MAPR_SWJ_CFG) | AFIO_MAPR_SWJ_CFG_NO_JTAG_NO_SW);// Try to disable SWD AND JTAG so we can use those pins (not sure if this works).

    #if defined(BUTTON_BANK) && defined (BUTTON_PIN) && defined (BUTTON_PRESSED_STATE)
        SET_REG(GPIO_CR(BUTTON_BANK,BUTTON_PIN),(GPIO_CR(BUTTON_BANK,BUTTON_PIN) & crMask(BUTTON_PIN)) | BUTTON_INPUT_MODE << CR_SHITF(BUTTON_PIN));

        gpio_write_bit(BUTTON_BANK, BUTTON_PIN,1-BUTTON_PRESSED_STATE);// set pulldown resistor in case there is no button.
    #endif
        SET_REG(GPIO_CR(LED_BANK,LED_PIN),(GET_REG(GPIO_CR(LED_BANK,LED_PIN)) & crMask(LED_PIN)) | CR_OUTPUT_PP << CR_SHITF(LED_PIN));
 }

void setupFLASH() {
    /* configure the HSI oscillator */
    if ((pRCC->CR & 0x01) == 0x00) {
        u32 rwmVal = pRCC->CR;
        rwmVal |= 0x01;
        pRCC->CR = rwmVal;
    }

    /* wait for it to come on */
    while ((pRCC->CR & 0x02) == 0x00) {}
 }

bool checkUserCode(u32 usrAddr) {
    u32 sp = *(vu32 *) usrAddr;

    if ((sp & 0x2FFE0000) == 0x20000000) {
        return (TRUE);
    } else {
        return (FALSE);
    }
 }

void setMspAndJump(u32 usrAddr) {
    // Dedicated function with no call to any function (appart the last call)
    // This way, there is no manipulation of the stack here, ensuring that GGC
    // didn't insert any pop from the SP after having set the MSP.
    typedef void (*funcPtr)(void);
    u32 jumpAddr = *(vu32 *)(usrAddr + 0x04); /* reset ptr in vector table */
    funcPtr usrMain = (funcPtr) jumpAddr;
    SET_REG(SCB_VTOR, (vu32) (usrAddr));
    asm volatile("msr msp, %0"::"g"(*(volatile u32 *)usrAddr));
    usrMain();                                /* go! */
 }


void jumpToUser(u32 usrAddr) {

    /* tear down all the dfu related setup */
    // disable usb interrupts, clear them, turn off usb, set the disc pin
    // todo pick exactly what we want to do here, now its just a conservative
    flashLock();
    nvicDisableInterrupts();

    // Does nothing, as PC12 is not connected on teh Maple mini according to the schemmatic     setPin(GPIOC, 12); // disconnect usb from host. todo, macroize pin
    systemReset(); // resets clocks and periphs, not core regs
    setMspAndJump(usrAddr);
 }

#ifdef USE_BACKUP_REGS
    void bkp10Write(u16 value)
    {
            // Enable clocks for the backup domain registers
            pRCC->APB1ENR |= (RCC_APB1ENR_PWR_CLK | RCC_APB1ENR_BKP_CLK);
            // Disable backup register write protection
            pPWR->CR |= PWR_CR_DBP;
            // store value in pBK DR10
            pBKP->DR10 = value;
            // Re-enable backup register write protection
            pPWR->CR &=~ PWR_CR_DBP;
    }

    int checkAndClearBootloaderFlag()
    {
        bool flagSet = 0x00;// Flag not used

        // Enable clocks for the backup domain registers
        pRCC->APB1ENR |= (RCC_APB1ENR_PWR_CLK | RCC_APB1ENR_BKP_CLK);

        switch (pBKP->DR10)
        {
            case RTC_BOOTLOADER_FLAG:
                flagSet = 0x01;
                break;
            case RTC_BOOTLOADER_JUST_UPLOADED:
                flagSet = 0x02;
                break;
        }

        if (flagSet!=0x00)
        {
            bkp10Write(0x0000);// Clear the flag
            // Disable clocks
            pRCC->APB1ENR &= ~(RCC_APB1ENR_PWR_CLK | RCC_APB1ENR_BKP_CLK);
        }
        return flagSet;
    }
 #endif // USE_BACKUP_REGS



void nvicInit(NVIC_InitTypeDef *NVIC_InitStruct) {
    u32 tmppriority = 0x00;
    u32 tmpreg      = 0x00;
    u32 tmpmask     = 0x00;
    u32 tmppre      = 0;
    u32 tmpsub      = 0x0F;

    SCB_TypeDef *rSCB = (SCB_TypeDef *) SCB_BASE;
    NVIC_TypeDef *rNVIC = (NVIC_TypeDef *) NVIC_BASE;


    /* Compute the Corresponding IRQ Priority --------------------------------*/
    tmppriority = (0x700 - (rSCB->AIRCR & (u32)0x700)) >> 0x08;
    tmppre = (0x4 - tmppriority);
    tmpsub = tmpsub >> tmppriority;

    tmppriority = (u32)NVIC_InitStruct->NVIC_IRQChannelPreemptionPriority << tmppre;
    tmppriority |=  NVIC_InitStruct->NVIC_IRQChannelSubPriority & tmpsub;

    tmppriority = tmppriority << 0x04;
    tmppriority = ((u32)tmppriority) << ((NVIC_InitStruct->NVIC_IRQChannel & (u8)0x03) * 0x08);

    tmpreg = rNVIC->IPR[(NVIC_InitStruct->NVIC_IRQChannel >> 0x02)];
    tmpmask = (u32)0xFF << ((NVIC_InitStruct->NVIC_IRQChannel & (u8)0x03) * 0x08);
    tmpreg &= ~tmpmask;
    tmppriority &= tmpmask;
    tmpreg |= tmppriority;

    rNVIC->IPR[(NVIC_InitStruct->NVIC_IRQChannel >> 0x02)] = tmpreg;

    /* Enable the Selected IRQ Channels --------------------------------------*/
    rNVIC->ISER[(NVIC_InitStruct->NVIC_IRQChannel >> 0x05)] =
        (u32)0x01 << (NVIC_InitStruct->NVIC_IRQChannel & (u8)0x1F);
 }

void nvicDisableInterrupts() {
    NVIC_TypeDef *rNVIC = (NVIC_TypeDef *) NVIC_BASE;
    rNVIC->ICER[0] = 0xFFFFFFFF;
    rNVIC->ICER[1] = 0xFFFFFFFF;
    rNVIC->ICPR[0] = 0xFFFFFFFF;
    rNVIC->ICPR[1] = 0xFFFFFFFF;

    SET_REG(STK_CTRL, 0x04); /* disable the systick, which operates separately from nvic */
 }

void systemHardReset(void) {
    SCB_TypeDef *rSCB = (SCB_TypeDef *) SCB_BASE;

    /* Reset  */
    rSCB->AIRCR = (u32)AIRCR_RESET_REQ;

    /*  should never get here */
    while (1) {
        asm volatile("nop");
    }
 }

bool flashErasePage(u32 pageAddr) {
    u32 rwmVal = GET_REG(FLASH_CR);
    rwmVal = FLASH_CR_PER;
    SET_REG(FLASH_CR, rwmVal);

    while (GET_REG(FLASH_SR) & FLASH_SR_BSY) {}
    SET_REG(FLASH_AR, pageAddr);
    SET_REG(FLASH_CR, FLASH_CR_START | FLASH_CR_PER);
    while (GET_REG(FLASH_SR) & FLASH_SR_BSY) {}

    /* todo: verify the page was erased */

    rwmVal = 0x00;
    SET_REG(FLASH_CR, rwmVal);

    return TRUE;
 }

bool flashErasePages(u32 pageAddr, u16 n) {
    while (n-- > 0) {
        if (!flashErasePage(pageAddr + wTransferSize * n)) {
            return FALSE;
        }
    }

    return TRUE;
 }

bool flashWriteWord(u32 addr, u32 word) {
    vu16 *flashAddr = (vu16 *)addr;
    vu32 lhWord = (vu32)word & 0x0000FFFF;
    vu32 hhWord = ((vu32)word & 0xFFFF0000) >> 16;

    u32 rwmVal = GET_REG(FLASH_CR);
    SET_REG(FLASH_CR, FLASH_CR_PG);

    /* apparently we need not write to FLASH_AR and can
       simply do a native write of a half word */
    while (GET_REG(FLASH_SR) & FLASH_SR_BSY) {}
    *(flashAddr + 0x01) = (vu16)hhWord;
    while (GET_REG(FLASH_SR) & FLASH_SR_BSY) {}
    *(flashAddr) = (vu16)lhWord;
    while (GET_REG(FLASH_SR) & FLASH_SR_BSY) {}

    rwmVal &= 0xFFFFFFFE;
    SET_REG(FLASH_CR, rwmVal);

    /* verify the write */
    if (*(vu32 *)addr != word) {
        return FALSE;
    }

    return TRUE;
 }

void flashLock() {
    /* take down the HSI oscillator? it may be in use elsewhere */

    /* ensure all FPEC functions disabled and lock the FPEC */
    SET_REG(FLASH_CR, 0x00000080);
 }

void flashUnlock() {
    /* unlock the flash */
    SET_REG(FLASH_KEYR, FLASH_KEY1);
    SET_REG(FLASH_KEYR, FLASH_KEY2);
 }


// Used to create the control register masking pattern, when setting control register mode.
unsigned int crMask(int pin)
    {
        unsigned int mask;
        if (pin>=8)
        {
            pin-=8;
        }
        mask = 0x0F << (pin<<2);
        return ~mask;
    }

#define FLASH_SIZE_REG 0x1FFFF7E0
int getFlashEnd(void)
    {
        unsigned short *flashSize = (unsigned short *) (FLASH_SIZE_REG);// Address register
        return ((int)(*flashSize & 0xffff) * 1024) + 0x08000000;
    }

int getFlashPageSize(void)
    {

        unsigned short *flashSize = (unsigned short *) (FLASH_SIZE_REG);// Address register
        if ((*flashSize & 0xffff) > 128)
        {
            return 0x800;
        }
        else
        {
            return 0x400;
        }
    }
