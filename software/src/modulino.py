"""Modulino Knob (encodeur rotatif I2C).

Lecture via read_i2c_block_data : envoie d'abord un octet registre 0x00
qui triggere le reset du compteur interne, puis lit 5 octets.
Format: [0x74_status, delta_lo, delta_hi, button, 0xFF]
"""

import struct
import time
import threading

try:
    import smbus2
    _bus = smbus2.SMBus(1)
    _has_bus = True
except Exception:
    _bus = None
    _has_bus = False

from config import MODULINO_I2C_ADDR

INVERT_DIR = False

_MAX_DELTA = 5   # plafond par lecture pour eviter les sauts


class ModulinoKnob:
    def __init__(self, fan, state, state_lock):
        self._fan = fan
        self._state = state
        self._lock = state_lock
        self._saved_duty = 30.0
        self._btn_prev = False
        threading.Thread(target=self._loop, daemon=True).start()

    def _read(self):
        if not _has_bus:
            return 0, False
        try:
            b = _bus.read_i2c_block_data(MODULINO_I2C_ADDR, 0, 5)
            # [0x74, delta_lo, delta_hi, button, 0xFF]
            delta = struct.unpack_from('<h', bytes(b[1:3]))[0]
            pressed = b[3] != 0
            # plafonne pour eviter sauts dus a accumulation
            if delta > _MAX_DELTA:
                delta = _MAX_DELTA
            elif delta < -_MAX_DELTA:
                delta = -_MAX_DELTA
            if INVERT_DIR:
                delta = -delta
            with self._lock:
                self._state["modulino_raw"] = list(b)
                self._state["modulino_last_delta"] = delta
                self._state["modulino_btn"] = pressed
                self._state["modulino_err"] = ""
            return delta, pressed
        except Exception as e:
            with self._lock:
                self._state["modulino_raw"] = [0, 0, 0, 0, 0]
                self._state["modulino_err"] = str(e)[:40]
            return 0, False

    def _set_duty(self, duty):
        duty = max(0.0, min(100.0, duty))
        self._fan.set_speed(duty)
        with self._lock:
            self._state["fan_duty"] = duty

    def _loop(self):
        while True:
            delta, pressed = self._read()
            step = self._state.get("encoder_step", 2.0)

            if delta != 0:
                current = self._fan.duty
                if current <= 0 and delta < 0:
                    self._set_duty(step)
                else:
                    self._set_duty(current + delta * step)

            if pressed and not self._btn_prev:
                if self._fan.duty > 0:
                    self._saved_duty = self._fan.duty
                    self._set_duty(0.0)
                else:
                    self._set_duty(self._saved_duty if self._saved_duty > 0 else 30.0)

            self._btn_prev = pressed
            time.sleep(0.005)
