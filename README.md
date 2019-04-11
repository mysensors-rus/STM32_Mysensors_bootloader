# STM32-mysensors-bootloader

Создан на основе STM32duino-bootloader.
Выкинуто всё лишнее (DFU, USB), добавлено необходимое.
Позволяет собирать ноды STM32 с OTA.
Для работоспособности OTA необхдимо наличие SPI флэш на первом SPI порту (впрочем это при нужде можно и поменять). Также необходим контроллер (например MYSController) умеющий шить более 32кб кода.
Вывод SS назначен на ногу PA4.
Проект уже вполне рабочий. Но т.к. в STM32 флеша горахдо больше 32кб, то требует допиливания библиотеки MySensors. По хорошему полноценного пулл реквеста. В проекте порой наличествует лишний мусор от предыдущего проекта.

Also Note. Use GCC 4.8 (not 4.9 or newer, as these versions have more aggressive optimisation which causes hardware registers not be read correctly and consequently the bootloader does not work)

Bootloader for STM32F103 boards.
