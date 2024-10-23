### Описание
Представляю часть кода для `STM32F411RE`.
В код не включен ассемблерный `startup` и прочее из проекта `CudeIDE`, только папка `src`.
Программа описывает взаимодейтсвие `MCU` с силовой электроникой (`IGBT` 6 пар), для управления спаренным 3-х фазным `SRM` двигателем.
На основе данных с датчика положения ротора (датчик реализован на основе микросхемы `AS5048`), подключен по `SPI` и выбранного режиме работы - задается внутренней переменной производится управление двигателем.
Управление `IGBT` также реализованно цифровым интерфейсом (`I2C`), на базе пары микросхем `PCF8574A`.
В силовой схеме присутсвует токовая защита в виде датчика тока (`ПИТ-УА..`) измеряющего ток на фае, в момент включения и `ЦАП` `mcp4725`, также подключенным по `I2C` ( на той же шине что `PCF8574A`). Если ток превышает уставноленное значение (сравниваем компаратором), формируется дискретный сигнал, который через прерывание останавливает работу.

### Структура
Глобальные константы лежат в `cm_srm_global_define.h`
Остальное постарался разбить по "смысловым группам":
`cm_angle_sensor.c/h` - описание логики взаимодействия с датчиком положения, и обработка полученного значения, вместе с конфигурацией `SPI` и таймера `TIM2` прерывания для коммуникации.
`cm_srm_current_sensor.c/h` - legacy. Остался с тех времен, когда значение датчика еще черес `АЦП` заводилось в `MCU`. Удалять не стал, для будущей реализации...
`cm_srm_switch_IGBT.c/h` - описание процесса переключения `IGBT`, иначе говоря `ШИМ` с управлением его параметрами, реализованный на `TIM3` таймере, также кофигурация `I2C` с таймером коммуникации `TIM4`
`cm_srm_throttle.c\h` - алгоритм установки задания, для `ЦАП`. Название происходит от того, что здесь раньше был алгоритм для работы с аналоговой педалью газа, которая ограничивала ток, тем самым регулировала момент. В итоге от аналоговой педали отказались, новую (цифровую) подобради, но файлик остался для будущей реализации.
`main.c` - объявление общей структуры управления, конфигурация `GPIO` для приема дискретных сигналов с компаратора, тоже как-то здесь оказалась... Также, раньше я использовал кнопку на отладочной плате, для запуска двигателя в нужном режиме, сейчас делаю "виртуальную", чтобы "дребезга контактов" избегать.
`stm32f4xx_it.c` - тут наверно нечего комменитровать, обработчик прерываний. Единственно, как видно, в текущей реализации функции посылки `I2C` в отдельные функции не завернуты, на тот момент их функционал не до конца сформирован, тоже относится к прерываниям от компараторов.

P.S.
Комментариев не добавлял в код, кроме тех, что уже имелись. В коде также довольно много просто закомментированого кода осталось...сейчас я так почти не делаю, но здесь оставил.