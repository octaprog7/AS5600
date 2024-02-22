"""Модуль MicroPython для управления 12-ти битным однооборотным магнитным энкодером AS5600 от AMS OSRAM."""
# micropython
# mail: goctaprog@gmail.com
# MIT license
# import struct
import time
from micropython import const
from sensor_pack_2 import bus_service
from sensor_pack_2.base_sensor import DeviceEx, Iterator, check_value, all_none
import micropython
from collections import namedtuple
from sensor_pack_2.aliased import get_bf_gen, bitmask

_mask_12 = bitmask(range(12))
_all_round = const(360)
raw_per_degrees: float = const((1+_mask_12)/_all_round)


def degrees_to_raw(degrees: int) -> int:
    """Преобразует градусы от 0 до 360 в сырые значения для записи в регистры датчика"""
    check_value(degrees, range(1+_all_round), f"Значение параметра degrees вне допустимого диапазона: {degrees}!")
    val = int(round(raw_per_degrees * degrees))
    if val > _mask_12:
        return _mask_12
    return val


def raw_to_degrees(raw: int) -> float:
    """Преобразует сырые значения углов из регистров датчика в градусы от 0 до 360"""
    check_value(raw, range(1 + _mask_12), f"Значение параметра raw вне допустимого диапазона: {raw}!")
    return raw / raw_per_degrees


# маски для битовых полей регистра CONF
#               WatchDog(bool),   fast_filter_threshold(int), slow_filter(int), pwm_freq(int), output_stage(int),
#               hysteresis(int), power_mode(int)
_conf_masks = range(13, 14), range(10, 13), range(8, 10), range(6, 8), range(4, 6), range(2, 4), range(2)

# именованный кортеж конфигурации для регистра CONF
config_as5600 = namedtuple("config_t",
                           ("watchdog", "fast_filter_threshold", "slow_filter",
                            "pwm_freq", "output_stage", "hysteresis", "power_mode"))

# Превышение минимального усиления АРУ, слишком сильный магнит (min_gain_ovf)
# Превышение максимального усиления АРУ, слишком слабый магнит (max_gain_ovf)
# Магнит обнаружен (mag_detected)
_status_masks = range(5, 6), range(4, 5), range(3, 4)
# именованный кортеж - состояние"""
# для метода get_status
status_as5600 = namedtuple("status_t", "mag_detected max_gain_ovf min_gain_ovf")
# для хранения угловых положений: ZPOS, MPOS, MANG
angle_positions = namedtuple("angle_positions", "start stop angular_range")


def _check_slow_filter(sf: int):
    """Проверяет параметр slow_filter (0..3) на правильность
    Соответствие параметр slow_filter и времени интегрирования в мс
    0 - 2.2 мс
    1 - 1.1 мс
    2 - 0.55 мс
    3 - 0.286 мс"""
    check_value(sf, range(4), f"Неверное значение параметра slow_filter: {sf}")


class AS5600(DeviceEx, Iterator):
    """Класс MicroPython для управления 12-ти битным однооборотным магнитным энкодером AS5600 от AMS OSRAM."""

    def __init__(self, adapter: bus_service.BusAdapter, address=0x36):
        """i2c - объект класса I2C; address - адрес датчика на шине. Он не изменяется!"""
        super().__init__(adapter, address, True)
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
        self._fast_filter_threshold = None,  # bit 12..10
        # Если быстрый фильтр выключен, переходная характеристика выходного (угла) сигнала контролируется
        # медленным линейным фильтром.
        self._slow_filter = 0,  # bit 9..8,
        # частота ШИМ. 00 = 115 Hz; 01 = 230 Hz; 10 = 460 Hz; 11 = 920 Hz
        self._pwm_freq = None,  # bit 7..6,
        # 0x00 - аналоговый выход (выходное напряжение в пределах 0..100% VDD)
        # 0x01 - аналоговый выход (выходное напряжение в пределах 10..90% VDD)
        # 0x02 - на выводе микросхемы датчика ШИМ(!)
        self._output_stage = None,  # bit 5..4,
        # Чтобы избежать любого переключения выхода, когда магнит не движется, можно включить гистерезис от 1 до 3
        # младших разрядов 12-битного разрешения.
        self._hysteresis = None,  # bit 3..2,
        # Цифровой конечный автомат автоматически управляет режимами пониженного энергопотребления, чтобы снизить
        # среднее потребление тока. Доступны и могут быть включены три(!) режима пониженного энергопотребления!
        # PM0 - датчик всегда(!) включен
        # PM1 - период опроса 5 мс
        # PM2 - период опроса 20 мс
        # PM3 - период опроса 100 мс
        self._power_mode = None,  # bit 1..0
        #
        self.get_status()  # читаю состояние датчика
        self.get_config()  # читаю настройки датчика

    def get_status(self, noreturn: bool = True) -> [status_as5600, None]:
        """Возвращает содержимое регистра состояния(STATUS) в виде именованного кортежа (MD, ML, MH)
        MH - минимальное усиления АРУ, слишком сильный магнит(min_gain_ovf).
        ML - Превышение максимального усиления АРУ, слишком слабый магнит(max_gain_ovf).
        МD - Магнит обнаружен.(mag_detected)"""
        val = self.read_reg(0x0B, 1)[0]
        # return 0 != (val & 0b10_0000), 0 != (val & 0b01_0000), 0 != (val & 0b00_1000)
        #               магнит в норме          магнит слишком слабый  магнит слишком сильный
        # return status_t(0 != (val & 0b10_0000), 0 != (val & 0b01_0000), 0 != (val & 0b00_1000))
        stat = status_as5600(*get_bf_gen(val, _status_masks))
        # сохраняю состояние магнита энкодера
        self._mag_detected, self._max_gain_ovf, self._min_gain_ovf = stat
        if noreturn:
            return
        return stat

    @property
    def status(self) -> status_as5600:
        return self.get_status(noreturn=False)

    def _get_gain(self) -> int:
        """Возвращает содержимое регистра Automatic Gain Control (ACG)"""
        return self.read_reg(0x1A, 1)[0]

    def _get_magnitude(self) -> int:
        """Возвращает содержимое регистра MAGNITUDE"""
        return _mask_12 & self.unpack(f"H", self.read_reg(0x1B, 2))[0]

    def get_raw_angle(self, raw: bool = False) -> int:
        """Возвращает сырой(raw) угол поворота магнита относительно микросхемы.
        Если raw в Истина, то возвращенное значение не масштабировано (сырое)!
        Регистр ANGLE имеет гистерезис 10-LSB на пределе диапазона 360 градусов,
        чтобы избежать точек разрыва или переключения выхода в течение одного оборота."""
        return _mask_12 & self.unpack(f"H", self.read_reg(0x0C if raw else 0x0E, 2))[0]

    def get_config(self, noreturn: bool = True) -> [config_as5600, None]:
        """Возвращает текущие настройки датчика, регистр CONF, в виде именованного кортежа"""
        _conf = self._conf()
        (self._watchdog, self._fast_filter_threshold, self._slow_filter, self._pwm_freq, self._output_stage,
         self._hysteresis, self._power_mode) = _conf
        if noreturn:
            return
        return _conf

    def get_angle_pos(self, raw: bool = True) -> angle_positions:
        """Возвращает сырые(raw is True) или в градусах(raw is False) значения:
        начального углового положения(ZPOS/start)
        конечного углового положения(MPOS/stop)
        размера углового диапазона(MANG/angular_range)"""
        buf = self._buf_6
        self.read_buf_from_mem(address=0x01, buf=buf)
        if raw:
            itr = map(lambda x: _mask_12 & x, self.unpack(fmt_char="HHH", source=buf))
        else:
            itr = map(lambda x: raw_to_degrees(_mask_12 & x), self.unpack(fmt_char="HHH", source=buf))
        # print(f"DBG:get_angle_pos {list(itr)}")
        return angle_positions(*itr)

    def set_angle_pos(self, start: int = 0, stop: [int, None] = 360, angular_range: [int, None] = None):
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
        self.write_reg(0x01, degrees_to_raw(start), 2)
        time.sleep_ms(1)  # ожидаю 1 мс
        # Диапазон задается путем программирования начальной позиции (ZPOS) и либо конечной позиции (MPOS),
        # либо(!) размера углового диапазона (MANG). То есть, либо пара ZPOS, MPOS, либо пара ZPOS, MANG!
        # я выбрал пару ZPOS, MPOS. Если после MPOS записать MANG, то MPOS обнулится!!!
        if stop is not None:
            self.write_reg(0x03, degrees_to_raw(stop), 2)
            return
        if angular_range is not None:
            self.write_reg(0x05, degrees_to_raw(angular_range), 2)

    def burn(self, angle_or_settings: bool = True):
        """Сохраняет настройки углов(angle_or_settings is True) или настройки(angle_or_settings is False) из
        регистров датчика в энергонезависимую память (OTP).
        Эту команду (BURN_ANGLE) можно выполнить только при наличии
        бита "магнит обнаружен" (MD = 1/status_as5600.mag_detected == True)"""
        self.write_reg(0xFF, 0x80 if angle_or_settings else 0x40, 1)

    def get_counter(self) -> int:
        """Возвращает кол-во операций записи в энергонезависимую память. В датчике на это отведено ДВА бита (ZMCO(1:0)),
        то есть результат будет в диапазоне 0..3"""
        return 0b11 & self.read_reg(0x00, 1)[0]

    @micropython.native
    def get_conversion_cycle_time(self) -> int:
        """Возвращает время преобразования в [мкc] датчиком данных цвета. В микросекундах!"""
        k = self._slow_filter
        _check_slow_filter(k)
        return 11 + (2200 // (1 << k))

    def _conf(self,
              watchdog: [bool, None] = None,  # bit 13,
              fast_filter_threshold: [int, None] = None,  # bit 12..10
              slow_filter: [int, None] = None,  # bit 9..8,
              pwm_freq: [int, None] = None,  # bit 7..6,
              output_stage: [int, None] = None,  # bit 5..4,
              hysteresis: [int, None] = None,  # bit 3..2,
              power_mode: [int, None] = None,  # bit 1..0
              ) -> config_as5600:
        """Регистр CONF. Если все параметры в None, возвращает содержимое регистра"""
        val = self.unpack(f"H", self.read_reg(0x07, 2))[0]
        if all_none(watchdog, fast_filter_threshold, slow_filter, pwm_freq,
                    output_stage, hysteresis, power_mode):
            # print(f"DBG _conf before: 0x{val:x}")
            return config_as5600(*get_bf_gen(val, _conf_masks))
        #
        if watchdog is not None:
            _mask = _conf_masks[0]
            val &= ~bitmask(_mask)  # mask
            val |= watchdog << _mask.start
        if fast_filter_threshold is not None:
            _mask = _conf_masks[1]
            val &= ~bitmask(_mask)  # mask
            val |= fast_filter_threshold << _mask.start
        if slow_filter is not None:
            _mask = _conf_masks[2]
            val &= ~bitmask(_mask)  # mask
            val |= slow_filter << _mask.start
        if pwm_freq is not None:
            _mask = _conf_masks[3]
            val &= ~bitmask(_mask)  # mask
            val |= pwm_freq << _mask.start
        if output_stage is not None:
            _mask = _conf_masks[4]
            val &= ~bitmask(_mask)  # mask
            val |= output_stage << _mask.start
        if hysteresis is not None:
            _mask = _conf_masks[5]
            val &= ~bitmask(_mask)  # mask
            val |= hysteresis << _mask.start
        if power_mode is not None:
            _mask = _conf_masks[6]
            val &= ~bitmask(_mask)  # mask
            val |= power_mode << _mask.start
        # print(f"DBG _settings after: 0x{val:x}")
        self.write_reg(0x07, val, 2)

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
        return raw_to_degrees(self.get_raw_angle(raw=False))

    def start_measurement(self, slow_filter: int = 1, fast_filter_threshold: int = 0, watchdog: bool = False,
                          pwm_freq: int = 2, output_stage: int = 2, hysteresis: int = 1, power_mode: int = 1):
        """Настраивает параметры датчика"""
        self._conf(watchdog, fast_filter_threshold, slow_filter, pwm_freq, output_stage, hysteresis, power_mode)

    # Iterator
    def __next__(self) -> [int, None]:
        """Часть протокола итератора"""
        return self.angle
