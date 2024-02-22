from sensor_pack_2.bus_service import BusAdapter
from sensor_pack_2.base_sensor import BaseSensor


def bitmask(bit_mask_range: range) -> int:
    """возвращает битовую маску по занимаемым битам."""
    return sum(map(lambda _x: 1 << _x, bit_mask_range))


def get_bf(source: int, mask: range) -> [bool, int]:
    """Возвращает битовое поле из source c использованием диапазона битовой маски mask"""
    if not mask:    # длина битовой маски не может быть меньше одного бита!
        raise ValueError(f"Неверный mask! {mask}")
    val = (source & bitmask(mask)) >> mask.start
    if 1 == len(mask):
        return bool(val)
    return val


def get_bf_gen(source: int, masks: iter[range]) -> [bool, int]:
    """Функция-генератор. Возвращает битовое поле из source c использованием диапазона битовой маски mask"""
    for mask in masks:
        yield get_bf(source, mask)


class Aliased:
    def __init__(self, alias: [str]):
        self._alias = alias

    @property
    def alias(self) -> str:
        """Возвращает псевдоним"""
        return self._alias

    def __repr__(self) -> str:
        return str(f"Alias: {self._alias}; id: {id(self)}; type: {type(self)}")


class AliasedStore:
    def __init__(self, items: tuple[Aliased]):
        self._items = items

    def __getitem__(self, item: [str]) -> [Aliased, None]:
        if isinstance(item, str):
            for item in self._items:
                if item.alias == item:
                    return item


class BitField(Aliased):
    """Класс для удобной работы с битовым полем."""

    def __init__(self, alias: [str, None], rng: range):
        """alias - псевдоним (для удобства, например "work_mode3:0")
        rng: номера бит бит битового поля. Например range(3) - биты 0, 1, 2; range(2, 4) - биты 2, 3!"""
        super().__init__(alias)
        if not len(rng) or rng.step > 1:
            raise ValueError("Неверный параметр rng!")
        self._alias = alias
        self._start = rng.start         # номер младшего бита маски
        self._bitmask = bitmask(rng)   # вычисление маски

    def set(self, source: int, value: int) -> int:
        """Записывает value в битовый диапазон в source, начиная с бита self._start.
        Возвращает source с установленным value в битовом диапазоне"""
        _tmp = source & ~self._bitmask                    # чистка битового диапазона в source
        _tmp |= self._bitmask & (value << self._start)    # установка битов в заданном диапазоне
        return _tmp

    def get(self, source: int) -> int:
        """Возвращает значение, находящееся в битовом диапазоне в source."""
        return (source & self._bitmask) >> self._start     # выделение маской битового диапазона и его сдвиг вправо

    @property
    def mask(self) -> int:
        """Возвращает битовую маску"""
        return self._bitmask


class Registry(Aliased):
    """Типа аппаратный регистр"""
    def __init__(self, alias: str, address: int, bytes_width=1,
                 readable: bool = True, writable: bool = True):
        """alias - псевдоним
        address - адрес регистра в памяти устройства.
        bytes_width - разрядность в байтах"""
        super().__init__(alias)
        if address < 0 or bytes_width > 2:
            raise ValueError("Неверный параметр address или bytes_width!")
        self._fields = None
        self._address = address
        self._readable = readable   # доступ на чтение
        self._writable = writable   # доступ на запись
        self._length_in_bytes = bytes_width     # разрядность регистра в байтах, 1 - 8 бит, 2 - 16 бит
        # считанное из устройства значение хранит это поле!
        self._value = 0

    def init(self, fields: tuple[BitField]):
        """Заполняет регистр битовыми полями"""
        for field in fields:
            if not isinstance(field, BitField):
                raise ValueError("Неверный тип параметра field!")
        self._fields = fields

    @property
    def value(self) -> int:
        return self._value

#    @value.setter
#    def value(self, new_val):
#        self._value = new_val

    def __getitem__(self, item: [str, int]) -> [BitField, None]:
        """возвращает BitField класс регистра, по его alias или индексу"""
        if isinstance(item, int):
            return self._fields[item]
        if isinstance(item, str):
            for field in self._fields:
                if field.alias == item:
                    return field
            return None

    @property
    def readable(self) -> bool:
        return self._readable

    @property
    def writable(self) -> bool:
        return self._writable

    @property
    def address(self) -> int:
        return self._address


class BusDevice(BaseSensor):
    def __init__(self, address: int, adapter: BusAdapter, big_byte_ord: bool):
        super().__init__(adapter, address, big_byte_ord)
        self._regs = None

    def init(self, regs: tuple[Registry]):
        for reg in regs:
            if not isinstance(reg, Registry):
                raise ValueError('Ошибка. Элемент неверного типа передан в метод init!')
        self._regs = regs
