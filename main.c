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
 *  @file main.c
 *
 *  @brief main loop and calling any hardware init stuff. timing hacks for EEPROM
 *  writes not to block usb interrupts. logic to handle 2 second timeout then
 *  jump to user code.
 *
 */

#include "common.h"

int main()
{
    SPIflash_CRC = 0;
    flash_CRC = 0;

    systemReset(); // peripherals but not PC
    setupCLK();
    setupLEDAndButton();
    //TODO: setupSPI(PA0);
    setupSPI();

#ifdef USE_USART
    USART_init();
    send_string_USART(myMessages[0]);
#endif // USE_USART

    //    CRC_init();   // Проверка контрольной суммы

// Можно реализовать действие по нажатию кнопки
//    if (readButtonState())
//    {

        uint8_t checkFlash;
        checkFlash = CheckFlashImage();
        if (!checkFlash)
        { // Начинаем процесс обновления

            setupFLASH();  //
            flashUnlock(); // Разблокируем флэш
            uint32_t flashSize = (mybuff[5] << 24) | (mybuff[6] << 16) | (mybuff[7] << 8) | mybuff[8];

            // #ifdef USE_USART             // Выводим размер прошивки
            //         Hex2Ascii(mybuff[5]);
            //         Hex2Ascii(mybuff[6]);
            //         Hex2Ascii(mybuff[7]);
            //         Hex2Ascii(mybuff[8]);
            // #endif // USE_USART

            // Очищаем потребное место
            uint32_t index;
            for (index = USER_CODE_FLASH0X8001000; index < (USER_CODE_FLASH0X8001000 + flashSize); index += 0x0400)
                flashErasePage(index);

            // MYSController пишет с учётом отступа 0x1000 загрузчика, т.е. так как указано в HEX файле,
            // поэтому чтобы не изгаляться с переадресовкой оставляем вначале spi флэша пустое пространство
            // и сдвигаемся на размер загрузчика вперед
            uint32_t flashAddress = USER_CODE_FLASH0X8001000; // адрес флэш памяти МК, куда мы пишем данные
            uint32_t i;

            //TODO: Доделать проверку на размер < 256 ??
            for (i = 0; i < (flashSize - FIRMWARE_START_ADDRESS); i = i + 4)
            {
                fast_read(FIRMWARE_START_ADDRESS + i + FIRMWARE_START_OFFSET, &mybuff[0], 0x04);
                if (!flashWriteWord(flashAddress + i, *(uint32_t *)&mybuff[0]))
                {
                    flashLock();

#ifdef USE_USART
                    send_string_USART(myMessages[9]); // Firmaware flashing error!
#endif // USE_USART
        //TODO: Доделать корректную дальнейшую работу при ошибке записи

                    strobePin(LED_BANK, LED_PIN, checkFlash, BLINK_SLOW, LED_ON_STATE);    
#ifdef USE_USART
        send_string_USART(myMessages[7]); // Reboot
#endif  
                    systemHardReset();
                }
            }
            flashLock();

#ifdef USE_USART
            send_string_USART(myMessages[8]);
#endif                                             // USE_USART

            // Очищаем spi флэш
	        while(busy());
    	    setWriteEnable(true);
            //TODO: Сделать очистку второго блока, если прошивка больше 64к
            erase64kBlock(FIRMWARE_START_ADDRESS);

            strobePin(LED_BANK, LED_PIN, 10, BLINK_FAST, LED_ON_STATE);
        }
        else
        {
            strobePin(LED_BANK, LED_PIN, checkFlash, BLINK_SLOW, LED_ON_STATE);
        }
//    }

    if (checkUserCode(USER_CODE_FLASH0X8001000))
    {

#ifdef USE_USART
        send_string_USART(myMessages[5]); // jump to user prog
#endif // USE_USART

        jumpToUser(USER_CODE_FLASH0X8001000);
    }
    else
    {

#ifdef USE_USART
        send_string_USART(myMessages[7]); // Reboot
#endif                                    // USE_USART
        strobePin(LED_BANK, LED_PIN, 5, BLINK_SLOW, LED_ON_STATE);
        systemHardReset();
    }

    while (1)
        ;

    return 0; // Added to please the compiler
}
