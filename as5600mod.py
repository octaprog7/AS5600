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
    if 360 == degrees:
        return _mask_12
    check_value(degrees, range(_all_round), f"Значение параметра degrees вне допустимого диапазона: {degrees}!")
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

    def get_angle(self, raw: bool = False) -> int:
        """
        Возвращает 12-битное значение угла с датчика AS5600.

        Параметры:
            raw (bool):
                - Если True — возвращает «сырое» значение из регистра RAW ANGLE (0x0C).
                Это абсолютное положение магнита (0–4095, полный оборот), не зависящее от настроек ZPOS и MPOS.
                - Если False (по умолчанию) — возвращает обработанное значение из регистра ANGLE (0x0E).
                Это значение масштабировано в диапазон между ZPOS и MPOS
                и содержит гистерезис в 10 LSB около границы 0°/360° при использовании полного диапазона.

        Возвращает:
            int: 12-битное целое число в диапазоне [0, 4095].

        Примечания:
            - Оба регистра (RAW ANGLE и ANGLE) всегда доступны; задержка преобразования отсутствует.
            - Полярность вывода DIR влияет на оба значения одинаково (инверсия на аппаратном уровне).
        """
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
        размера углового диапазона(MANG/angular_range).
        Если датчик настроен через ZPOS/MPOS, поле angular_range может быть некорректным!"""
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
        # Минимальный угловой диапазон у меня 19 градусов. В документации на датчик это значение равно 18 градусам!
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
        """
        Записывает текущие значения в энергонезависимую память OTP (One-Time Programmable).

        Параметры:
            angle_or_settings (bool):
                - Если True — выполняется команда BURN_ANGLE: сохраняются значения ZPOS и MPOS.
                  Может быть выполнена до 3 раз. Требует наличия магнита (бит MD=1 в регистре STATUS).
                - Если False — выполняется команда BURN_SETTING: сохраняются регистры CONF и MANG.
                  Может быть выполнена только один раз и только если ZPOS/MPOS ещё не записывались (ZMCO=0).

        Важно:
            - Перед BURN_ANGLE обязательно убедитесь, что магнит обнаружен (self.status.mag_detected == True),
              иначе команда будет проигнорирована.
            - После записи в OTP изменить эти параметры программно невозможно.
            - Во время записи требуется стабильное питание (см. Electrical Characteristics, стр. 5 даташита).

        Исключения:
            RuntimeError: если вызвано с angle_or_settings=True, но магнит не обнаружен.

        Пример:
            enc.get_status()
            if enc.status.mag_detected:
                enc.set_angle_pos(0, 180)
                enc.burn(angle_or_settings=True)  # Сохранить углы
        """
        if angle_or_settings:
            # Обновляем статус, чтобы убедиться, что данные актуальны
            self.get_status()
            if not self._mag_detected:
                raise RuntimeError("BURN_ANGLE не выполнен: магнит не обнаружен (MD=0). Убедитесь, что магнит установлен и находится в пределах рабочего диапазона.")

        conn = self._connection
        conn.write_reg(reg_addr=0xFF, value=0x80 if angle_or_settings else 0x40, bytes_count=1)

    def get_counter(self) -> int:
        """Возвращает кол-во операций записи в энергонезависимую память. В датчике на это отведено ДВА бита (ZMCO(1:0)),
        то есть результат будет в диапазоне 0..3"""
        conn = self._connection
        return 0b11 & conn.read_reg(reg_addr=0x00, bytes_count=1)[0]

    @micropython.native
    def get_conversion_cycle_time(self) -> int:
        """Возвращает время цикла измерения в микросекундах!"""
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
        #
        conn = self._connection
        conn.write_reg(reg_addr=0x07, value=val, bytes_count=2)
        return None

    def configure(self,
                  watchdog: bool | None = None,
                  fast_filter_threshold: int | None = None,
                  slow_filter: int | None = None,
                  pwm_freq: int | None = None,
                  output_stage: int | None = None,
                  hysteresis: int | None = None,
                  power_mode: int | None = None) -> None:
        """
        Настраивает регистр CONF (0x07–0x08) датчика AS5600.
        Параметры, переданные как None, не изменяются.

        Параметры:
            watchdog (bool | None):
                Включение сторожевого таймера. При активации (True) датчик переходит в режим LPM3,
                если угол остаётся в пределах ±4 LSB в течение 1 минуты.
            fast_filter_threshold (int | None):
                Порог срабатывания быстрого фильтра (биты FTH[2:0], 0–7).
                Значения: 0 — только медленный фильтр; 1–7 — порог в LSB (см. даташит, стр. 29).
            slow_filter (int | None):
                Настройка медленного фильтра (биты SF[1:0], 0–3):
                    0 — 2.2 мс, 0.015° шума,
                    1 — 1.1 мс, 0.021°,
                    2 — 0.55 мс, 0.030°,
                    3 — 0.286 мс, 0.043°.
            pwm_freq (int | None):
                Частота ШИМ-выхода (биты PWMF[1:0], 0–3):
                    0 — 115 Гц, 1 — 230 Гц, 2 — 460 Гц, 3 — 920 Гц.
                Игнорируется, если выбран аналоговый выход.
            output_stage (int | None):
                Тип выходного сигнала (биты OUTS[1:0], 0–2):
                    0 — аналоговый (0–100% VDD),
                    1 — аналоговый (10–90% VDD),
                    2 — цифровой ШИМ.
            hysteresis (int | None):
                Гистерезис выхода (биты HYST[1:0], 0–3):
                    0 — отключён,
                    1–3 — гистерезис 1–3 младших разряда (LSB).
            power_mode (int | None):
                Режим энергопотребления (биты PM[1:0], 0–3):
                    0 — всегда включён (NOM),
                    1 — опрос каждые 5 мс (LPM1),
                    2 — опрос каждые 20 мс (LPM2),
                    3 — опрос каждые 100 мс (LPM3).

        Исключения:
            ValueError: если любой из переданных параметров выходит за допустимый диапазон.

        Примечание:
            Все изменения применяются немедленно и могут быть сохранены в OTP с помощью burn(angle_or_settings=False).
        """
        def get_err_str(val_name: str, val: int, rng: range) -> str:
            """Возвращает подробное сообщение об ошибке"""
            return f"Значение {val} параметра {val_name} вне диапазона [{rng.start}..{rng.stop-1}]!"

        r4 = range(4)
        #
        check_value(slow_filter, r4, get_err_str(val_name="slow filter", val=slow_filter, rng=r4))
        check_value(fast_filter_threshold, range(8), get_err_str(val_name="fast filter threshold", val=fast_filter_threshold, rng=range(8)))
        check_value(pwm_freq, r4, get_err_str(val_name="pwm freq", val=pwm_freq, rng=r4))
        check_value(output_stage, range(3), get_err_str(val_name="output stage", val=output_stage, rng=range(3)))
        check_value(hysteresis, r4, get_err_str(val_name="hysteresis", val=hysteresis, rng=r4))
        check_value(power_mode, r4, get_err_str(val_name="power_mode", val=power_mode, rng=r4))
        # запись в регистр CONF
        self._conf(
            watchdog=watchdog,
            fast_filter_threshold=fast_filter_threshold,
            slow_filter=slow_filter,
            pwm_freq=pwm_freq,
            output_stage=output_stage,
            hysteresis=hysteresis,
            power_mode=power_mode
        )


    @property
    def magnitude(self) -> int:
        """Возвращает величину магнитного поля (регистр MAGNITUDE, 0x1B–0x1C)."""
        return self._get_magnitude()

    @property
    def gain(self) -> int:
        """Возвращает значение коэффициента АРУ (Automatic Gain Control, регистр 0x1A)."""
        return self._get_gain()

    @property
    def angle(self) -> float:
        """Возвращает угол поворота магнита относительно корпуса микросхемы"""
        return _raw_to_degrees(self.get_angle(raw=False))

    def start_measurement(self, mode: int | None = None) -> None:
        """Совместимость с IBaseSensorEx. AS5600 всегда готов к чтению."""
        pass

    def get_data_status(self, raw=False):
        """Данные всегда готовы для чтения."""
        return True

    def soft_reset(self) -> None:
        """
        Программный сброс сырых угловых значений в значения по умолчанию:
        - ZPOS = 0°
        - MPOS = 360° (полный оборот)

        Эквивалентно состоянию после включения питания (если OTP не запрограммирован).
        Не затрагивает другие параметры (CONF, гистерезис и т.д.).
        """
        conn = self._connection
        conn.write_reg(reg_addr=0x01, value=0, bytes_count=2)  # ZPOS = 0
        conn.write_reg(reg_addr=0x03, value=_mask_12, bytes_count=2)  # MPOS = 4095


    # Iterator
    def __next__(self) -> float | None:
        """Часть протокола итератора"""
        return self.angle