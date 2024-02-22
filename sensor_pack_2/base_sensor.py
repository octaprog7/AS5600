# micropython
# MIT license
# Copyright (c) 2022 Roman Shevchik   goctaprog@gmail.com
import struct
import micropython
from sensor_pack_2 import bus_service
from machine import Pin


@micropython.native
def check_value(value: [int, None], valid_range, error_msg: str) -> [int, None]:
    if value is None:
        return value
    if value not in valid_range:
        raise ValueError(error_msg)
    return value


def all_none(*args):
    """возвращает Истина, если все входные параметры в None.
    Добавил 25.01.2024"""
    for element in args:
        if element is not None:
            return False
    return True


class Device:
    """Класс - основа датчика"""

    def __init__(self, adapter: bus_service.BusAdapter, address: [int, Pin], big_byte_order: bool):
        """Базовый класс Устройство.
        Если big_byte_order равен True -> порядок байтов в регистрах устройства «big»
        (Порядок от старшего к младшему), в противном случае порядок байтов в регистрах "little"
        (Порядок от младшего к старшему)
        address - адрес устройства на шине.

        Base device class. if big_byte_order is True -> register values byteorder is 'big'
        else register values byteorder is 'little'
        address - address of the device on the bus."""
        self.adapter = adapter
        self.address = address
        # for I2C. byte order in register of device
        self.big_byte_order = big_byte_order
        # for SPI ONLY. При передаче данных по SPI: SPI.firstbit can be SPI.MSB or SPI.LSB
        # передавать первым битом старший или младший
        # для каждого устройства!
        self.msb_first = True

    def _get_byteorder_as_str(self) -> tuple:
        """Return byteorder as string"""
        if self.is_big_byteorder():
            return 'big', '>'
        return 'little', '<'

    def pack(self, fmt_char: str, *values) -> bytes:
        if not fmt_char:
            raise ValueError("Invalid fmt_char parameter!")
        bo = self._get_byteorder_as_str()[1]
        return struct.pack(bo + fmt_char, values)

    def unpack(self, fmt_char: str, source: bytes, redefine_byte_order: str = None) -> tuple:
        """распаковка массива, считанного из датчика.
        Если redefine_byte_order != None, то bo (смотри ниже) = redefine_byte_order
        fmt_char: c, b, B, h, H, i, I, l, L, q, Q. pls see: https://docs.python.org/3/library/struct.html"""
        if not fmt_char:
            raise ValueError("Invalid fmt_char parameter!")
        bo = self._get_byteorder_as_str()[1]
        if redefine_byte_order is not None:
            bo = redefine_byte_order[0]
        return struct.unpack(bo + fmt_char, source)

    @micropython.native
    def is_big_byteorder(self) -> bool:
        return self.big_byte_order


class DeviceEx(Device):
    """Класс - основа датчика. Добавил общие методы доступа к шине. 30.01.2024"""
    def read_reg(self, reg_addr: int, bytes_count=2) -> bytes:
        """считывает из регистра датчика значение.
        bytes_count - размер значения в байтах.
        Должна быть реализована во всех классах - адаптерах шин, наследников BusAdapter.
        Добавил 25.01.2024"""
        return self.adapter.read_register(self.address, reg_addr, bytes_count)

    # BaseSensor
    def write_reg(self, reg_addr: int, value: [int, bytes, bytearray], bytes_count) -> int:
        """записывает данные value в датчик, по адресу reg_addr.
        bytes_count - кол-во записываемых данных.
        Добавил 25.01.2024"""
        byte_order = self._get_byteorder_as_str()[0]
        return self.adapter.write_register(self.address, reg_addr, value, bytes_count, byte_order)

    def read(self, n_bytes: int) -> bytes:
        """Читает из устройства n_bytes байт. Добавил 25.01.2024"""
        return self.adapter.read(self.address, n_bytes)

    def read_to_buf(self, buf) -> bytes:
        """Чтение из устройства в буфер"""
        return self.adapter.read_to_buf(self.address, buf)

    def write(self, buf: bytes):
        """Записывает в устройство информацию из buf. Добавил 25.01.2024"""
        return self.adapter.write(self.address, buf)

    def read_buf_from_mem(self, address: int, buf, address_size: int = 1):
        """Читает из устройства, начиная с адреса address в буфер.
        Кол-во читаемых байт равно "длине" буфера в байтах!
        address_size - определяет размер адреса в байтах."""
        return self.adapter.read_buf_from_memory(self.address, address, buf, address_size)

    def write_buf_to_mem(self, mem_addr, buf):
        """Записывает в устройство все байты из буфера buf.
        Запись начинается с адреса в устройстве: mem_addr."""
        return self.adapter.write_buf_to_memory(self.address, mem_addr, buf)


class BaseSensor(Device):
    """Класс - основа датчика с дополнительными методами"""
    def get_id(self):
        raise NotImplementedError

    def soft_reset(self):
        raise NotImplementedError


class BaseSensorEx(DeviceEx):
    """Класс - основа датчика"""
    def get_id(self):
        raise NotImplementedError

    def soft_reset(self):
        raise NotImplementedError


class Iterator:
    def __iter__(self):
        return self

    def __next__(self):
        raise NotImplementedError


class TemperatureSensor:
    """Вспомогательный или основной датчик температуры"""
    def enable_temp_meas(self, enable: bool = True):
        """Включает измерение температуры при enable в Истина
        Для переопределения программистом!!!"""
        raise NotImplementedError

    def get_temperature(self) -> [int, float]:
        """Возвращает температуру корпуса датчика в градусах Цельсия!
        Для переопределения программистом!!!"""
        raise NotImplementedError
