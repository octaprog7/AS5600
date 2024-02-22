# Модуль MicroPython для управления однооборотным магнитным энкодером AS5600 от AMS OSRAM
AS5600, в дальнейшем энкодер, это простой в программировании магнитный датчик положения с 12-битным аналоговым или ШИМ-выходом высокого разрешения. 
Эта бесконтактная система измеряет абсолютный угол диаметрально намагниченного осевого магнита. Энкодер предназначен для применения в бесконтактных потенциометрах, 
а его конструкция исключает влияние любых однородных внешних рассеянных магнитных полей.
Стандартный интерфейс I²C поддерживает простая настройка параметров без необходимости использования программатора.
## Угловой диапазон
По умолчанию выходные данные представляют собой диапазон от 0 до 360 градусов. Также возможно определить меньший диапазон выходного сигнала, 
установив угол начального положения и угол конечного положения.

## Применения

Применения
AS5600 идеально подходит для бесконтактных потенциометров, бесконтактных ручек, педалей, радиоуправляемых сервоприводов и других решений для измерения углового положения.

## Адрес датчика
Контроллер доступен по адресу 0x36 на шине I2C. Адрес фиксирован, изменению не подлежит! 

## Питание
Напряжение питания AS5600 от 3,3 В до 5,0 В!

## Шина I2C
Просто подключите контакты (VCC, GND, SDA, SCL) платы энкодера к соответствующим контактам MicroPython платы, 
ESP или любой другой с залитой прошивкой MicroPython!

## Загрузка ПО в плату
Загрузите прошивку micropython на плату NANO(ESP и т. д.), а затем файлы: main.py, as5600mod.py и папку sensor_pack_2 полностью!
Затем откройте main.py в своей IDE и запустите/выполните его.

# Картинки
## Плата AS5600
![alt text](https://github.com/octaprog7/as5600/blob/master/pics/board/board_0.jpg)
![alt text](https://github.com/octaprog7/as5600/blob/master/pics/board/board_1.jpg)
![alt text](https://github.com/octaprog7/as5600/blob/master/pics/board/board_2.jpg)
![alt text](https://github.com/octaprog7/as5600/blob/master/pics/board/board_3.jpg)
![alt text](https://github.com/octaprog7/as5600/blob/master/pics/board/board_4.jpg)
![alt text](https://github.com/octaprog7/as5600/blob/master/pics/board/board_5.jpg)
![alt text](https://github.com/octaprog7/as5600/blob/master/pics/board/shaft.jpg)
## Среда разработки
![alt text](https://github.com/octaprog7/as5600/blob/master/pics/ide_1.png)
![alt text](https://github.com/octaprog7/as5600/blob/master/pics/ide_2.png)
![alt text](https://github.com/octaprog7/as5600/blob/master/pics/ide_3.png)

