# Модуль MicroPython для управления однооборотным магнитным энкодером AS5600 от AMS OSRAM. В дальнейшем энкодер.
AS5600 — это простой в программировании магнитный датчик положения с 12-битным аналоговым или ШИМ-выходом высокого разрешения. 
Эта бесконтактная система измеряет абсолютный угол диаметрально намагниченного осевого магнита. Этот AS5600 предназначен для применения в бесконтактных потенциометрах, 
а его прочная конструкция исключает влияние любых однородных внешних рассеянных магнитных полей.
Стандартный интерфейс I²C поддерживает простое пользовательское программирование параметров без необходимости использования программатора.

По умолчанию выходные данные представляют собой диапазон от 0 до 360 градусов. Также возможно определить меньший диапазон выходного сигнала, 
установив угол начального положения и угол конечного положения (конечное положение).

# Применения

Применения
AS5600 идеально подходит для бесконтактных потенциометров, бесконтактных ручек, педалей, радиоуправляемых сервоприводов и других решений для измерения углового положения.

## Адрес датчика
Контроллер доступен по адресу 0x36 на шине I2C. Адрес фиксирован, изменению не подлежит! 

## Питание
Напряжение питания AS5600 от 3,3 В до 5,0 В!

## Шина I2C
Просто подключите контакты (VCC, GND, SDA, SCL) платы с AS5600 к соответствующим контактам MicroPython платы, 
ESP или любой другой с залитой прошивкой MicroPython!

## Загрузка ПО в плату
Загрузите прошивку micropython на плату NANO(ESP и т. д.), а затем файлы: main.py, as5600mod.py и папку sensor_pack_2 полностью!
Затем откройте main.py в своей IDE и запустите/выполните его.

# Картинки
## Плата AS5600
![alt text](https://github.com/octaprog7/veml6040/blob/master/pics/6040_led_off.jpg)
![alt text](https://github.com/octaprog7/veml6040/blob/master/pics/6040_led_on.jpg)
## Среда разработки
![alt text](https://github.com/octaprog7/veml6040/blob/master/pics/ide_6040.png)
