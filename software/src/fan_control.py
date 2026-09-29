import time
import threading
import RPi.GPIO as GPIO
from config import PIN_FAN_PWM, PIN_FAN_TACH, PWM_FREQUENCY


class FanController:
    """Contrôle PWM du ventilateur + mesure RPM via polling TACH."""

    def __init__(self):
        GPIO.setup(PIN_FAN_PWM, GPIO.OUT)
        GPIO.setup(PIN_FAN_TACH, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        self.pwm = GPIO.PWM(PIN_FAN_PWM, PWM_FREQUENCY)
        self.pwm.start(0)
        self._duty = 0.0
        self._rpm = 0
        threading.Thread(target=self._rpm_loop, daemon=True).start()

    def _rpm_loop(self):
        last = GPIO.input(PIN_FAN_TACH)
        count = 0
        t0 = time.time()
        while True:
            cur = GPIO.input(PIN_FAN_TACH)
            if cur == 0 and last == 1:  # front descendant
                count += 1
            last = cur
            now = time.time()
            if now - t0 >= 1.0:
                self._rpm = count * 30  # 2 impulsions/tour → ×60/2
                count = 0
                t0 = now
            time.sleep(0.001)

    def set_speed(self, duty):
        duty = max(0.0, min(100.0, float(duty)))
        if duty == 0.0:
            # Forcer le pin LOW — évite les micro-impulsions du soft PWM à 0%
            self.pwm.ChangeDutyCycle(0)
            GPIO.output(PIN_FAN_PWM, GPIO.LOW)
        else:
            self.pwm.ChangeDutyCycle(duty)
        self._duty = duty

    @property
    def duty(self):
        return self._duty

    @property
    def rpm(self):
        return self._rpm

    def stop(self):
        self.set_speed(0)

    def cleanup(self):
        self.pwm.stop()
