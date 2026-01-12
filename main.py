import sys
from machine import I2C, Pin
from sensor_pack_2.bus_service import I2cAdapter
import as5600mod
import time


def decode_magnet_status(source: as5600mod.status_as5600, _gain: int):
    """Выводит в stdout состояние магнита и усиление АРУ"""
    if source.min_gain_ovf:
        print("Магнит слишком сильный! Замените его на более слабый!!!")
        return
    print(f"Магнит установлен: {source.mag_detected}; Предел усиления АРУ достигнут: {source.max_gain_ovf}; Минимальное усиление: {source.min_gain_ovf}; Усиление: {_gain}")


def decode_config(source: as5600mod.config_as5600):
    """Выводит в stdout настройки датчика"""
    def decode_pm(pm: int) -> str:
        if 0 == pm:
            return "нормальный режим"
        return f"LPM{pm}"

    def decode_hyst(hyst: int) -> str:
        if 0 == hyst:
            return "выключен"
        return f"{hyst}"

    def decode_output_stage(stage: int) -> str | None:
        if 0 == stage:
            return "аналоговый выход. диапазон GND..VDD"
        if 1 == stage:
            return "аналоговый выход. диапазон 10% VDD..90% VDD"
        if 2 == stage:
            return "Цифровой выход. ШИМ."
        return None

    def decode_pwmf(pwmf: int) -> int:
        vals = 115, 230, 460, 920   # Гц
        return vals[pwmf]

    def decode_slow_filter(sf: int) -> str:
        vals = "16x", "8x", "4x", "2x"
        return vals[sf]

    def decode_ffh(ffh: int) -> int:
        vals = 0, 6, 7, 9, 18, 21, 24, 10
        return vals[ffh]

    print(f"Watchdog: {source.watchdog}")
    print(f"fast filter threshold: {decode_ffh(source.fast_filter_threshold)}; slow filter: {decode_slow_filter(source.slow_filter)}")
    print(f"PWM frequency: {decode_pwmf(source.pwm_freq)}; output stage: {decode_output_stage(source.output_stage)}")
    print(f"hysteresis: {decode_hyst(source.hysteresis)}; power mode: {decode_pm(source.power_mode)}")


def decode_angle_pos(source: as5600mod.angle_positions, _raw: bool):
    """Выводит в stdout угловые настройки датчика"""
    print(f"Угловые положения raw: {_raw}. старт: {source.start}; стоп: {source.stop}; диапазон: {source.angular_range}")


if __name__ == '__main__':
    i2c = I2C(id=1, scl=Pin(7), sda=Pin(6), freq=400_000)  # on Raspberry Pi Pico
    adapter = I2cAdapter(i2c)
    sensor = as5600mod.AS5600(adapter)
    print(f"Время преобразования [мкс]: {sensor.get_conversion_cycle_time()}")
    print("---Настройки датчика---")
    decode_config(sensor.get_config(noreturn=False))
    print("---Угловые настройки---")
    decode_angle_pos(sensor.get_angle_pos(raw=False), _raw=False)
    sensor.set_angle_pos(0, 360)
    decode_angle_pos(sensor.get_angle_pos(raw=False), _raw=False)
    #
    check_magnet_time = 15
    print("---Проверка наличия магнита---")
    wait_func = time.sleep_us
    for _ in range(check_magnet_time):
        decode_magnet_status(sensor.status, sensor.gain)
        time.sleep_ms(1000)
    #
    counter, _max = 0, 10_000
    sensor.start_measurement()
    for angle in sensor:
        print(f"Угол: {angle}; raw angle: {sensor.get_raw_angle(raw=True)}; коефф. усиления: {sensor.gain}; magnitude: {sensor.magnitude}")
        wait_func(100_000)
        if counter > _max:
            sys.exit(0)
        counter += 1


