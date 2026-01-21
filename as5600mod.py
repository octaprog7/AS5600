"""Модуль MicroPython для управления 12-ти битным однооборотным магнитным энкодером AS5600 от AMS OSRAM."""
# micropython
# mail: goctaprog@gmail.com
# MIT license
import time
import micropython
from micropython import const
from collections import namedtuple
from sensor_pack_2 import bus_service
from sensor_pack_2.base_sensor import IBaseSensorEx, Iterator, DeviceEx, check_value, all_none


@micropython.native
def _bitmask(bit_mask_range: range) -> int:
    """Возвращает битовую маску по занимаемым битам."""
    mask = 0
    for bit in bit_mask_range:
        mask |= (1 << bit)
    return mask

def _get_bf(source: int, mask: range) -> bool | int:
    """Возвращает битовое поле из source c использованием диапазона битовой маски mask"""
    if not mask:    # длина битовой маски не может быть меньше одного бита!
        raise ValueError(f"Неверный mask! {mask}")
    val = (source & _bitmask(mask)) >> mask.start
    if 1 == len(mask):
        return bool(val)
    return val

_mask_12: int = const(_bitmask(range(12)))
_all_round: int = const(360)
_raw_per_degrees: float = const((1+_mask_12)/_all_round)

def _get_bf_gen(source: int, masks: iter) -> iter:
    """Функция-генератор. Возвращает битовое поле из source c использованием диапазона битовой маски mask"""
    for mask in masks:
        yield _get_bf(source, mask)

def _degrees_to_raw(degrees: int) -> int:
    """Преобразует градусы от 0 до 360 в сырые значения для записи в регистры датчика"""
    check_value(degrees, range(1+_all_round), f"Значение параметра degrees вне допустимого диапазона: {degrees}!")
    val = int(round(_raw_per_degrees * degrees))
    if val > _mask_12:
        return _mask_12
    return val


def _raw_to_degrees(raw: int) -> float:
    """Преобразует сырые значения углов из регистров датчика в градусы от 0 до 360"""
    check_value(raw, range(1 + _mask_12), f"Значение параметра raw вне допустимого диапазона: {raw}!")
    return raw / _raw_per_degrees


# маски для битовых полей регистра CONF
#               WatchDog(bool),   fast_filter_threshold(int), slow_filter(int), pwm_freq(int), output_stage(int),
#               hysteresis(int), power_mode(int)
_conf_masks = range(13, 14), range(10, 13), range(8, 10), range(6, 8), range(4, 6), range(2, 4), range(2)

# именованный кортеж конфигурации для регистра CONF
config_as5600 = namedtuple("config_as5600",
                           ("watchdog", "fast_filter_threshold", "slow_filter",
                            "pwm_freq", "output_stage", "hysteresis", "power_mode"))

# Превышение минимального усиления АРУ, слишком сильный магнит (min_gain_ovf)
# Превышение максимального усиления АРУ, слишком слабый магнит (max_gain_ovf)
# Магнит обнаружен (mag_detected)
_status_masks = range(5, 6), range(4, 5), range(3, 4)
# именованный кортеж - состояние
# для метода get_status
status_as5600 = namedtuple("status_as5600", "mag_detected max_gain_ovf min_gain_ovf")
# для хранения угловых положений: ZPOS, MPOS, MANG
angle_positions = namedtuple("angle_positions", "start stop angular_range")


def _check_slow_filter(sf: int) -> int:
    """Проверяет параметр slow_filter (0..3) на правильность
    Соответствие параметр slow_filter и времени интегрирования в мс
    0 - 2.2 мс
    1 - 1.1 мс
    2 - 0.55 мс
    3 - 0.286 мс"""
    return check_value(sf, range(4), f"Неверное значение параметра slow_filter: {sf}")


class AS5600(IBaseSensorEx, Iterator):
    """Класс MicroPython для управления 12-ти битным однооборотным магнитным энкодером AS5600 от AMS OSRAM."""

    def __init__(self, adapter: bus_service.BusAdapter, address=0x36):
        """i2c - объект класса I2C; address - адрес датчика на шине. Он не изменяется!"""
        # self._connection = DeviceEx(adapter=adapter, address=address, big_byte_order=True)
        # super().__init__(adapter, address, True)
        self._connection = DeviceEx(adapter=adapter, address=address, big_byte_order=True)
        self._buf_6 = bytearray((0 for _ in range(6)))  # буфер для угловых положений
        # magnet. состояние внешнего магнита энкодера. обновляются методом get_status()
        self._mag_detected = self._max_gain_ovf = self._min_gain_ovf = None
        # конфигурация/настройки датчика. обновляются методом get_config
        # Сторожевой таймер позволяет экономить электроэнергию путем переключения в LMP3, если угол остается в пределах
        # сторожевого порога 4 младших разрядов в течение как минимум одной(!) минуты.
        self._watchdog = None
        # Для быстрого реагирования на скачки и низкого уровня шума после стабилизации можно включить fast filter.
        # Он работает только в том случае, если изменение угла превышает(!) порог fast filter, в противном случае
        # выходной отклик определяется только slow filter.
        self._fast_filter_threshold = None  # bit 12..10
        # Если быстрый фильтр выключен, переходная характеристика выходного (угла) сигнала контролируется
        # медленным линейным фильтром.
        self._slow_filter = 0  # bit 9..8,
        # частота ШИМ. 00 = 115 Hz; 01 = 230 Hz; 10 = 460 Hz; 11 = 920 Hz
        self._pwm_freq = None  # bit 7..6,
        # 0x00 - аналоговый выход (выходное напряжение в пределах 0..100% VDD)
        # 0x01 - аналоговый выход (выходное напряжение в пределах 10..90% VDD)
        # 0x02 - на выводе микросхемы датчика ШИМ(!)
        self._output_stage = None  # bit 5..4,
        # Чтобы избежать любого переключения выхода, когда магнит не движется, можно включить гистерезис от 1 до 3
        # младших разрядов 12-битного разрешения.
        self._hysteresis = None  # bit 3..2,
        # Цифровой конечный автомат автоматически управляет режимами пониженного энергопотребления, чтобы снизить
        # среднее потребление тока. Доступны и могут быть включены три(!) режима пониженного энергопотребления!
        # PM0 - датчик всегда(!) включен
        # PM1 - период опроса 5 мс
        # PM2 - период опроса 20 мс
        # PM3 - период опроса 100 мс
        self._power_mode = None  # bit 1..0
        #
        self.get_status()  # читаю состояние датчика
        self.get_config()  # читаю настройки датчика

    def get_status(self, noreturn: bool = True) -> status_as5600 | None:
        """Возвращает содержимое регистра состояния(STATUS) в виде именованного кортежа (MD, ML, MH)
        MH - минимальное усиления АРУ, слишком сильный магнит(min_gain_ovf).
        ML - Превышение максимального усиления АРУ, слишком слабый магнит(max_gain_ovf).
        МD - Магнит обнаружен.(mag_detected)"""
        val = self._connection.read_reg(reg_addr=0x0B, bytes_count=1)[0]
        # return 0 != (val & 0b10_0000), 0 != (val & 0b01_0000), 0 != (val & 0b00_1000)
        #               магнит в норме          магнит слишком слабый  магнит слишком сильный
        # return status_t(0 != (val & 0b10_0000), 0 != (val & 0b01_0000), 0 != (val & 0b00_1000))
        stat = status_as5600(*_get_bf_gen(val, _status_masks))
        # сохраняю состояние магнита энкодера
        self._mag_detected, self._max_gain_ovf, self._min_gain_ovf = stat
        if noreturn:
            return None
        return stat

    @property
    def status(self) -> status_as5600:
        return self.get_status(noreturn=False)

    def _get_gain(self) -> int:
        """Возвращает содержимое регистра Automatic Gain Control (ACG)"""
        return self._connection.read_reg(reg_addr=0x1A, bytes_count=1)[0]

    def _get_magnitude(self) -> int:
        """Возвращает содержимое регистра MAGNITUDE"""
        conn = self._connection
        return _mask_12 & conn.unpack(f"H", conn.read_reg(reg_addr=0x1B, bytes_count=2))[0]

    def get_raw_angle(self, raw: bool = False) -> int:
        """Возвращает сырой(raw) угол поворота магнита относительно микросхемы.
        Если raw Истина, то возвращенное значение не масштабировано (сырое)!
        Регистр ANGLE имеет гистерезис 10-LSB на пределе диапазона 360 градусов,
        чтобы избежать точек разрыва или переключения выхода в течение одного оборота."""
        conn = self._connection
        return _mask_12 & conn.unpack(f"H", conn.read_reg(0x0C if raw else 0x0E, bytes_count=2))[0]

    def get_config(self, noreturn: bool = True) -> config_as5600 | None:
        """Возвращает текущие настройки датчика, регистр CONF, в виде именованного кортежа"""
        if noreturn:
            return None
        _conf = self._conf()
        (self._watchdog, self._fast_filter_threshold, self._slow_filter, self._pwm_freq, self._output_stage,
         self._hysteresis, self._power_mode) = _conf
        return _conf

    def get_angle_pos(self, raw: bool = True) -> angle_positions:
        """Возвращает сырые(raw is True) или в градусах(raw is False) значения:
        начального углового положения(ZPOS/start)
        конечного углового положения(MPOS/stop)
        размера углового диапазона(MANG/angular_range)"""
        conn = self._connection
        buf = self._buf_6
        conn.read_buf_from_mem(address=0x01, buf=buf)
        if raw:
            itr = map(lambda x: _mask_12 & x, conn.unpack(fmt_char="HHH", source=buf))
        else:
            itr = map(lambda x: _raw_to_degrees(_mask_12 & x), conn.unpack(fmt_char="HHH", source=buf))
        return angle_positions(*itr)

    def set_angle_pos(self, start: int = 0, stop: int | None = 360, angular_range: int | None = None):
        """Записывает в регистры датчика начальное угловое положение(start), конечное угловое положение(stop).
        Угловой диапазон вычисляется автоматически!
        Задавайте пару start и stop, или пару start и angular_range.
        Задавать start, stop и angular_range бессмысленно! Датчик самостоятельно обнулит stop,
        если вы зададите angular_range. Вот такие пошли датчики умные!"""
        err_str = "Неверное значение параметра"
        if stop is None and angular_range is None:
            raise ValueError("Один из параметров 'stop' или 'angular_range' должен быть задан!")
        rng = range(1 + _all_round)
        check_value(start, rng, err_str)
        check_value(stop, rng, err_str)
        check_value(angular_range, rng, err_str)
        # минимальный угловой диапазон у меня 19 градусов. В документации на датчик это значение равно 18 градусам!
        if angular_range is not None and angular_range < 20:
            raise ValueError("'angular_range' не может быть меньше 20!")
        #
        # vals = degrees_to_raw(start), degrees_to_raw(stop)      # , degrees_to_raw(angle_range)
        conn = self._connection
        conn.write_reg(reg_addr=0x01, value=_degrees_to_raw(start), bytes_count=2)
        time.sleep_ms(1)  # ожидаю 1 мс
        # Диапазон задается путем программирования начальной позиции (ZPOS) и либо конечной позиции (MPOS),
        # либо(!) размера углового диапазона (MANG). То есть, либо пара ZPOS, MPOS, либо пара ZPOS, MANG!
        # я выбрал пару ZPOS, MPOS. Если после MPOS записать MANG, то MPOS обнулится!!!
        if stop is not None:
            conn.write_reg(reg_addr=0x03, value=_degrees_to_raw(stop), bytes_count=2)
            return
        if angular_range is not None:
            conn.write_reg(reg_addr=0x05, value=_degrees_to_raw(angular_range), bytes_count=2)

    def burn(self, angle_or_settings: bool = True):
        """Сохраняет настройки углов(angle_or_settings is True) или настройки(angle_or_settings is False) из
        регистров датчика в энергонезависимую память (OTP).
        Эту команду (BURN_ANGLE) можно выполнить только при наличии
        бита "магнит обнаружен" (MD = 1/status_as5600.mag_detected == True)"""
        conn = self._connection
        conn.write_reg(reg_addr=0xFF, value=0x80 if angle_or_settings else 0x40, bytes_count=1)

    def get_counter(self) -> int:
        """Возвращает кол-во операций записи в энергонезависимую память. В датчике на это отведено ДВА бита (ZMCO(1:0)),
        то есть результат будет в диапазоне 0..3"""
        conn = self._connection
        return 0b11 & conn.read_reg(reg_addr=0x00, bytes_count=1)[0]

    @micropython.native
    def get_conversion_cycle_time(self) -> int:
        """Возвращает время преобразования в [мкс] датчиком данных цвета. В микросекундах!"""
        k = self._slow_filter
        _check_slow_filter(k)
        return 11 + (2200 // (1 << k))

    def _conf(self,
              watchdog: bool | None = None,  # bit 13,
              fast_filter_threshold: int | None = None,  # bit 12..10
              slow_filter: int | None = None,  # bit 9..8,
              pwm_freq: int | None = None,  # bit 7..6,
              output_stage: int | None = None,  # bit 5..4,
              hysteresis: int | None = None,  # bit 3..2,
              power_mode: int | None = None,  # bit 1..0
              ) -> config_as5600 | None:
        """Регистр CONF. Если все параметры в None, возвращает содержимое регистра"""
        conn = self._connection
        val = conn.unpack(f"H", conn.read_reg(reg_addr=0x07, bytes_count=2))[0]
        if all_none(watchdog, fast_filter_threshold, slow_filter, pwm_freq,
                    output_stage, hysteresis, power_mode):
            return config_as5600(*_get_bf_gen(val, _conf_masks))
        #
        if watchdog is not None:
            _mask = _conf_masks[0]
            val &= ~_bitmask(_mask)  # mask
            val |= watchdog << _mask.start
        if fast_filter_threshold is not None:
            _mask = _conf_masks[1]
            val &= ~_bitmask(_mask)  # mask
            val |= fast_filter_threshold << _mask.start
        if slow_filter is not None:
            _mask = _conf_masks[2]
            val &= ~_bitmask(_mask)  # mask
            val |= slow_filter << _mask.start
        if pwm_freq is not None:
            _mask = _conf_masks[3]
            val &= ~_bitmask(_mask)  # mask
            val |= pwm_freq << _mask.start
        if output_stage is not None:
            _mask = _conf_masks[4]
            val &= ~_bitmask(_mask)  # mask
            val |= output_stage << _mask.start
        if hysteresis is not None:
            _mask = _conf_masks[5]
            val &= ~_bitmask(_mask)  # mask
            val |= hysteresis << _mask.start
        if power_mode is not None:
            _mask = _conf_masks[6]
            val &= ~_bitmask(_mask)  # mask
            val |= power_mode << _mask.start
        # print(f"DBG _settings after: 0x{val:x}")
        conn = self._connection
        conn.write_reg(reg_addr=0x07, value=val, bytes_count=2)
        return None

    @property
    def magnitude(self) -> int:
        """Возвращает """
        return self._get_magnitude()

    @property
    def gain(self) -> int:
        """Возвращает """
        return self._get_gain()

    @property
    def angle(self) -> float:
        """Возвращает угол поворота магнита относительно корпуса микросхемы"""
        return _raw_to_degrees(self.get_raw_angle(raw=False))

    def start_measurement(self, slow_filter: int = 1, fast_filter_threshold: int = 0, watchdog: bool = False,
                          pwm_freq: int = 2, output_stage: int = 2, hysteresis: int = 1, power_mode: int = 1):
        """Настраивает параметры датчика"""
        def get_err_str(val_name: str, val: int, rng: range) -> str:
            """Возвращает подробное сообщение об ошибке"""
            return f"Значение {val} параметра {val_name} вне диапазона [{rng.start}..{rng.stop-1}]!"
        r4 = range(4)
        check_value(slow_filter, r4, get_err_str("slow filter", slow_filter, r4))
        check_value(fast_filter_threshold, range(8), get_err_str("fast filter threshold",
                                                                 fast_filter_threshold, range(8)))
        check_value(pwm_freq, r4, get_err_str("pwm freq", pwm_freq, r4))
        check_value(output_stage, range(3), get_err_str("output stage", output_stage, range(3)))
        check_value(hysteresis, r4, get_err_str("hysteresis", hysteresis, r4))
        check_value(power_mode, r4, get_err_str("power_mode", power_mode, r4))
        #
        self._conf(watchdog, fast_filter_threshold, slow_filter, pwm_freq, output_stage, hysteresis, power_mode)

    # Iterator
    def __next__(self) -> float | None:
        """Часть протокола итератора"""
        return self.angle
