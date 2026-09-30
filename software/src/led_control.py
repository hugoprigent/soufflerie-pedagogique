"""Controleur ruban LED WS2812B via SPI (GPIO10 / MOSI / pin 19).

Utilise /dev/spidev0.0 ; l'utilisateur du Pi doit avoir accès au groupe spi.
Encodage : 1 SPI byte = 1 bit WS2812B a 8 MHz.
  0-bit : 0xC0 -> T0H=250ns  T0L=750ns
  1-bit : 0xF8 -> T1H=625ns  T1L=375ns
"""

import math
import threading
import time

SPI_BUS     = 0
SPI_DEVICE  = 0
SPI_SPEED   = 8_000_000   # 8 MHz

_B0 = 0xC0   # WS2812B 0-bit : 2 high + 6 low
_B1 = 0xF8   # WS2812B 1-bit : 5 high + 3 low
_RESET_BYTES = 60   # >50 µs de LOW = reset strip


def _encode_pixel(r, g, b):
    """Encode (R,G,B) en 24 bytes SPI (ordre GRB WS2812B)."""
    out = bytearray(24)
    idx = 0
    for byte in (g, r, b):
        for bit in range(7, -1, -1):
            out[idx] = _B1 if (byte >> bit) & 1 else _B0
            idx += 1
    return out


def _build_frame(pixels):
    buf = bytearray()
    for r, g, b in pixels:
        buf += _encode_pixel(r, g, b)
    buf += bytearray(_RESET_BYTES)
    return bytes(buf)


def _hsv_to_rgb(h_deg, s, v):
    h = (h_deg % 360) / 60.0
    i = int(h)
    f = h - i
    p, q, t = v*(1-s), v*(1-f*s), v*(1-(1-f)*s)
    r, g, b = [(v,t,p),(q,v,p),(p,v,t),(p,q,v),(t,p,v),(v,p,q)][i % 6]
    return int(r*255), int(g*255), int(b*255)


_ENSAM_MAUVE  = (148, 60, 200)  # violet/mauve ENSAM
_ENSAM_ORANGE = (255, 110, 0)   # orange ENSAM
_ENSAM_WHITE  = (230, 230, 255) # blanc chaud


class LedController:
    ANIMATIONS = ("off", "solid", "pulse", "rainbow", "strobe", "wipe", "exposition", "boite_nuit")

    def __init__(self, n_leds=30):
        self.n          = n_leds
        self.animation  = "solid"
        self.color      = (0, 120, 255)
        self.brightness = 80           # 0-255 (applique en scale sur les couleurs)
        self.speed      = 50           # 0-100
        self._lock      = threading.Lock()
        self._spi       = None
        self.error      = ""
        self._startup_done = False

        try:
            import spidev
            spi = spidev.SpiDev()
            spi.open(SPI_BUS, SPI_DEVICE)
            spi.max_speed_hz = SPI_SPEED
            spi.mode = 0
            self._spi = spi
        except Exception as e:
            self.error = str(e)

        threading.Thread(target=self._loop, daemon=True).start()

    @property
    def ok(self):
        return self._spi is not None

    def set(self, **kw):
        with self._lock:
            if "animation"  in kw: self.animation  = kw["animation"]
            if "color"      in kw: self.color       = tuple(kw["color"])
            if "brightness" in kw: self.brightness  = max(0, min(255, int(kw["brightness"])))
            if "speed"      in kw: self.speed       = max(1, min(100, int(kw["speed"])))
            if "n_leds"     in kw: self.n           = max(1, int(kw["n_leds"]))

    def _write(self, pixels):
        if self._spi:
            try:
                self._spi.writebytes2(_build_frame(pixels))
            except Exception as e:
                self.error = str(e)

    def _fill(self, r, g, b):
        bri = self.brightness / 255.0
        self._write([(int(r*bri), int(g*bri), int(b*bri))] * self.n)

    def _run_startup(self):
        """Animation demarrage : wipe bleu ENSAM + eclair arc-en-ciel + fondu vers saved."""
        if not self._spi:
            return
        with self._lock:
            n = self.n
            bri = self.brightness / 255.0
            r_s, g_s, b_s = self.color

        # 1) tout eteint
        self._write([(0, 0, 0)] * n)
        time.sleep(0.15)

        # 2) wipe mauve ENSAM des deux bouts vers le centre
        r0, g0, b0 = _ENSAM_MAUVE
        c_mauve = (int(r0 * bri), int(g0 * bri), int(b0 * bri))
        for step in range(n // 2 + 2):
            pix = [(0, 0, 0)] * n
            for i in range(min(step + 1, n)):
                pix[i] = c_mauve
            for i in range(max(n - step - 1, 0), n):
                pix[i] = c_mauve
            self._write(pix)
            time.sleep(0.025)

        # 3) flash orange ENSAM
        rw, gw, bw = _ENSAM_ORANGE
        self._write([(int(rw * bri), int(gw * bri), int(bw * bri))] * n)
        time.sleep(0.15)
        # flash blanc
        rw2, gw2, bw2 = _ENSAM_WHITE
        self._write([(int(rw2 * bri), int(gw2 * bri), int(bw2 * bri))] * n)
        time.sleep(0.12)

        # 4) burst arc-en-ciel rapide (16 frames)
        for frame in range(16):
            pix = []
            for i in range(n):
                hue = (frame * 22.5 + i * 360 / max(n, 1)) % 360
                rc, gc, bc = _hsv_to_rgb(hue, 1.0, bri)
                pix.append((rc, gc, bc))
            self._write(pix)
            time.sleep(0.04)

        # 5) fondu arco-en-ciel → couleur enregistree (10 frames)
        for step in range(10):
            t_f = step / 9.0
            pix = []
            for i in range(n):
                hue = (16 * 22.5 + i * 360 / max(n, 1)) % 360
                rc, gc, bc = _hsv_to_rgb(hue, 1.0, bri)
                pix.append((
                    int(rc * (1 - t_f) + r_s * bri * t_f),
                    int(gc * (1 - t_f) + g_s * bri * t_f),
                    int(bc * (1 - t_f) + b_s * bri * t_f),
                ))
            self._write(pix)
            time.sleep(0.04)

    def _loop(self):
        self._run_startup()
        self._startup_done = True

        t = 0.0
        wipe_pos = 0
        prev_anim = None

        while True:
            try:
                if not self._spi:
                    time.sleep(0.5)
                    continue

                with self._lock:
                    anim = self.animation
                    r, g, b = self.color
                    bri = self.brightness / 255.0
                    spd = max(0.01, self.speed / 100.0)
                    n   = self.n

                if anim != prev_anim:
                    t = 0.0; wipe_pos = 0
                    prev_anim = anim

                if anim == "off":
                    self._write([(0, 0, 0)] * n)
                    time.sleep(0.2)
                    continue

                elif anim == "solid":
                    self._fill(r, g, b)
                    time.sleep(0.1)
                    continue

                elif anim == "pulse":
                    f = (math.sin(t * math.pi * 2) + 1) / 2
                    self._write([(int(r*bri*f), int(g*bri*f), int(b*bri*f))] * n)
                    t += 0.04 * spd * 3

                elif anim == "rainbow":
                    pixels = []
                    for i in range(n):
                        hue = (t * 360 + i * 360 / max(n, 1)) % 360
                        rc, gc, bc = _hsv_to_rgb(hue, 1.0, bri)
                        pixels.append((rc, gc, bc))
                    self._write(pixels)
                    t += 0.8 * spd

                elif anim == "strobe":
                    on = int(t * spd * 8) % 2 == 0
                    c = (int(r*bri), int(g*bri), int(b*bri)) if on else (0, 0, 0)
                    self._write([c] * n)
                    t += 0.05

                elif anim == "wipe":
                    pixels = []
                    for i in range(n):
                        if i <= wipe_pos:
                            pixels.append((int(r*bri), int(g*bri), int(b*bri)))
                        else:
                            pixels.append((0, 0, 0))
                    self._write(pixels)
                    wipe_pos = (wipe_pos + 1) % (n + 1)
                    t += 0.02

                elif anim == "exposition":
                    # 3 bandes mauve / blanc / orange ENSAM qui défilent
                    # Une bande = 1/3 du ruban, limites nettes
                    PALETTE = [_ENSAM_MAUVE, _ENSAM_WHITE, _ENSAM_ORANGE]
                    NC = 3
                    pix = []
                    for i in range(n):
                        # position dans le cycle couleur (0..NC), défile avec t
                        pos = (i / max(n, 1) * NC - t * spd * 0.25) % NC
                        idx = int(pos) % NC
                        frac = pos - int(pos)
                        # très court fondu (~10%) aux jonctions pour éviter le crénelage dur
                        if frac > 0.90:
                            f2 = (frac - 0.90) / 0.10
                            r_a, g_a, b_a = PALETTE[idx]
                            r_b, g_b, b_b = PALETTE[(idx + 1) % NC]
                            pix.append((
                                int((r_a * (1-f2) + r_b * f2) * bri),
                                int((g_a * (1-f2) + g_b * f2) * bri),
                                int((b_a * (1-f2) + b_b * f2) * bri),
                            ))
                        else:
                            r_c, g_c, b_c = PALETTE[idx]
                            pix.append((int(r_c * bri), int(g_c * bri), int(b_c * bri)))
                    self._write(pix)
                    t += 0.02 * spd

                elif anim == "boite_nuit":
                    # Rythme : beat toutes les beat_s secondes, 12 beats par cycle
                    beat_s   = max(0.25, 1.5 / spd)
                    beat_num = int(t / beat_s) % 12
                    bphase   = (t % beat_s) / beat_s          # 0..1 dans le beat
                    hue_t    = (t * 60.0 * spd) % 360.0

                    pix = [(0, 0, 0)] * n

                    if beat_num < 3:
                        # Strobe segmenté : 3 tiers en couleurs décalées de 120°
                        if math.sin(t * math.pi * spd * 18) > 0:
                            seg = max(1, n // 3)
                            cols = [
                                _hsv_to_rgb(int(hue_t) % 360,       1.0, bri),
                                _hsv_to_rgb(int(hue_t + 120) % 360, 1.0, bri),
                                _hsv_to_rgb(int(hue_t + 240) % 360, 1.0, bri),
                            ]
                            for i in range(n):
                                pix[i] = cols[(i // seg) % 3]

                    elif beat_num < 6:
                        # Comète arc-en-ciel avec trainée lumineuse
                        trail = max(5, n // 5)
                        pos   = (t * spd * n * 1.2) % n
                        for i in range(n):
                            dist = (pos - i) % n
                            if dist < trail:
                                fade = (1.0 - dist / trail) ** 1.5
                                rc, gc, bc = _hsv_to_rgb(
                                    (hue_t + i * 360.0 / max(n, 1)) % 360, 1.0, bri * fade)
                                pix[i] = (rc, gc, bc)

                    elif beat_num < 9:
                        # Double explosion depuis les deux bords vers le centre
                        spread = bphase * (n // 2 + 2)
                        for i in range(n):
                            d = min(i, n - 1 - i)
                            if d < spread:
                                fade = 1.0 - d / max(spread, 1)
                                rc, gc, bc = _hsv_to_rgb(
                                    (hue_t + d * 15) % 360, 1.0, bri * fade)
                                pix[i] = (rc, gc, bc)

                    else:
                        # Sparkle dense pseudo-aléatoire (LCG sans import random)
                        seed = int(t * spd * 1000)
                        for k in range(max(1, n // 2)):
                            s = (seed + k * 1664525 + 1013904223) & 0x7FFFFFFF
                            idx_l = s % n
                            h = (s // 997 + int(hue_t)) % 360
                            fl = (math.sin(t * spd * 15 + k * 0.73) + 1) * 0.5
                            rc, gc, bc = _hsv_to_rgb(h, 1.0, bri * fl)
                            pix[idx_l] = (rc, gc, bc)

                    self._write(pix)
                    t += 0.025

                time.sleep(max(0.02, 0.08 - spd * 0.06))

            except Exception:
                time.sleep(0.5)

    def stop(self):
        if self._spi:
            try:
                self._write([(0, 0, 0)] * self.n)
                self._spi.close()
            except Exception:
                pass

    def status(self):
        return {
            "ok":         self.ok,
            "error":      self.error,
            "animation":  self.animation,
            "color":      list(self.color),
            "brightness": self.brightness,
            "speed":      self.speed,
            "n_leds":     self.n,
        }
