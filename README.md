# STM32F103C8-TUTORIAL
ADC DMA 10 kHz tutorial sample with USB and PWM output
This is simple example of code for STM32F103C8
My program uses ADC via DMA. One channel with 10 kHz sampling due to timer.
Each 10 mc average value from ADC appear with PWM. And each 1c via USB.
Небольшая учебная программа для тестовой платы на STM32F103C8T6.
Назначение программы: 
1. С частотой 10 кГц, по таймеру, программа опрашивает один канал ADC 
2. Результаты измерений помещаются в буфер с использованием DMA
3. По заполнению буфера (100 слов), вычисляется среднее значение за 10 мс
4. Полученное среднее значение выводится PWM на один из выводов.
5. Вычисляется среднее значение измерений за 1с и выводится через USB, на VCP в текстовом виде
6. Это же среднее значение выводится через SWO (по отладке) в среду Keil uVision.
