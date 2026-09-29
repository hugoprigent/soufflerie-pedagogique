"""BMP280 / BME280 pressure sensor driver via I2C (smbus2 — no extra deps).

Compatible BMP280 (chip_id=0x58) ET BME280 (chip_id=0x60).
Les deux ont les mêmes registres T/P et les mêmes formules de compensation.
Le BME280 a en plus l'humidité (ignorée ici).

Wiring (same I2C bus as Modulino Knob, no address conflict):
  SDA -> GPIO 2 / Pin 3
  SCL -> GPIO 3 / Pin 5
  VCC -> 3.3V   / Pin 1
  GND -> GND    / Pin 6
  SDO -> GND    => address 0x76  (SDO->VCC => 0x77)
  CSB -> 3.3V   => force mode I2C

Usage:
  bmp = BMP280(bus=1, addr=0x76)
  bmp.set_reference()           # fan off, measure ambient baseline
  dp = bmp.delta_pa()           # ΔP = P_ref - P_static (Pa), >0 in flow
  V  = sqrt(2 * dp / 1.204)    # airspeed from static pressure drop
"""

import struct
import time

_CHIP_IDS_OK = {0x58: "BMP280", 0x60: "BME280"}

_REG_CHIPID    = 0xD0
_REG_CTRL_MEAS = 0xF4   # osrs_t[7:5] osrs_p[4:2] mode[1:0]
_REG_CONFIG    = 0xF5   # t_sb[7:5]   filter[4:2] spi3w_en[0]
_REG_DATA      = 0xF7   # 6 bytes: press_msb..temp_xlsb
_REG_CALIB     = 0x88   # 24 bytes calibration (T1..T3, P1..P9)


class BMP280:
    def __init__(self, bus=1, addr=0x76):
        self._addr = addr
        self._ok   = False
        self.error = ""
        self._p_ref = None
        self.chip_name = "?"

        try:
            import smbus2
            self._bus = smbus2.SMBus(bus)
            cid = self._bus.read_byte_data(addr, _REG_CHIPID)
            if cid not in _CHIP_IDS_OK:
                self.error = f"chip_id=0x{cid:02X} inconnu (attendu 0x58=BMP280 ou 0x60=BME280)"
                return
            self.chip_name = _CHIP_IDS_OK[cid]
            # Mode normal, oversampling ×4 pression + ×2 temp, filtre IIR ×4
            self._bus.write_byte_data(addr, _REG_CTRL_MEAS, 0b10010011)
            self._bus.write_byte_data(addr, _REG_CONFIG,    0b00010000)
            self._read_calib()
            time.sleep(0.1)
            self._ok = True
        except Exception as e:
            self.error = str(e)

    @property
    def ok(self):
        return self._ok

    # ------------------------------------------------------------------
    def _read_calib(self):
        raw = self._bus.read_i2c_block_data(self._addr, _REG_CALIB, 24)
        vals = struct.unpack('<HhhHhhhhhhhh', bytes(raw))
        T1, T2, T3 = vals[0], vals[1], vals[2]
        P1 = vals[3]; P2 = vals[4]; P3 = vals[5]; P4 = vals[6]
        P5 = vals[7]; P6 = vals[8]; P7 = vals[9]; P8 = vals[10]; P9 = vals[11]
        self._T = (T1, T2, T3)
        self._P = (P1, P2, P3, P4, P5, P6, P7, P8, P9)

    def _compensate(self, adc_t, adc_p):
        T1, T2, T3 = self._T
        var1 = (adc_t / 16384.0 - T1 / 1024.0) * T2
        var2 = (adc_t / 131072.0 - T1 / 8192.0) ** 2 * T3
        t_fine = var1 + var2
        temp_c = t_fine / 5120.0

        P1, P2, P3, P4, P5, P6, P7, P8, P9 = self._P
        v1 = t_fine / 2.0 - 64000.0
        v2 = v1 * v1 * P6 / 32768.0 + v1 * P5 * 2.0
        v2 = v2 / 4.0 + P4 * 65536.0
        v1 = (P3 * v1 * v1 / 524288.0 + P2 * v1) / 524288.0
        v1 = (1.0 + v1 / 32768.0) * P1
        if v1 == 0.0:
            return None, temp_c
        p = 1048576.0 - adc_p
        p = (p - v2 / 4096.0) * 6250.0 / v1
        v1 = P9 * p * p / 2147483648.0
        v2 = p * P8 / 32768.0
        p += (v1 + v2 + P7) / 16.0
        return p, temp_c

    # ------------------------------------------------------------------
    def read(self):
        """Lecture unique. Retourne (pression_Pa, temperature_C) ou (None, None)."""
        if not self._ok:
            return None, None
        try:
            raw = self._bus.read_i2c_block_data(self._addr, _REG_DATA, 6)
            adc_p = (raw[0] << 12) | (raw[1] << 4) | (raw[2] >> 4)
            adc_t = (raw[3] << 12) | (raw[4] << 4) | (raw[5] >> 4)
            return self._compensate(adc_t, adc_p)
        except Exception as e:
            self.error = str(e)
            return None, None

    def read_avg(self, n=20, dt=0.05):
        """Moyenne de n lectures (dt secondes entre chaque). Retourne (p_Pa, t_C)."""
        ps, ts = [], []
        for _ in range(n):
            p, t = self.read()
            if p is not None:
                ps.append(p); ts.append(t)
            time.sleep(dt)
        if not ps:
            return None, None
        return sum(ps) / len(ps), sum(ts) / len(ts)

    # ------------------------------------------------------------------
    def set_reference(self, n=40):
        """Mesure la pression ambiante de référence (ventilateur ARRÊTÉ).
        Appeler avant le sweep de calibration.
        """
        p, _ = self.read_avg(n=n, dt=0.05)
        if p is not None:
            self._p_ref = p
        return p

    @property
    def p_ref(self):
        return self._p_ref

    def delta_pa(self, n_avg=5):
        """ΔP = P_ref - P_section (Pa). Positif quand le ventilateur souffle
        (pression statique en section < pression ambiante).
        Retourne 0 si référence non définie ou capteur KO.
        """
        if not self._ok or self._p_ref is None:
            return 0.0
        ps = []
        for _ in range(n_avg):
            p, _ = self.read()
            if p is not None:
                ps.append(p)
        if not ps:
            return 0.0
        return self._p_ref - sum(ps) / len(ps)

    # ------------------------------------------------------------------
    def status(self):
        p, t = self.read() if self._ok else (None, None)
        return {
            "ok":    self._ok,
            "error": self.error,
            "p_pa":  round(p, 2) if p else None,
            "t_c":   round(t, 2) if t else None,
            "p_ref": round(self._p_ref, 2) if self._p_ref else None,
            "dp_pa": round(self.delta_pa(), 3),
        }
