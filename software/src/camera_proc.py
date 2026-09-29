"""Traitement image pour visualisation fumee + flux optique.

Optimisations:
- accepte un stream gris basse resolution (lores) pour le flux Farneback
  -> 4x plus rapide qu'un calcul full-res, sans perte de qualite visuelle
- ROI optionnelle : limite l'analyse a une region (ex: zone de sillage)
- mode vorticite : omega = dV/dx - dU/dy, met en evidence les tourbillons

Modes:
- raw       : image brute
- contrast  : CLAHE local
- arrows    : champ de vecteurs decime
- heatmap   : flux en HSV (teinte=direction, brillance=magnitude)
- vorticity : carte de vorticite rouge/bleu
- bgsub     : soustraction de fond MOG2
"""

import cv2
import numpy as np

from config import (
    FLOW_PYR_SCALE, FLOW_LEVELS, FLOW_WINSIZE,
    FLOW_ITERATIONS, FLOW_POLY_N, FLOW_POLY_SIGMA, FLOW_FLAGS,
)


class FlowProcessor:
    MODES = ("raw", "contrast", "arrows", "heatmap", "vorticity", "streamlines", "bgsub")

    def __init__(self):
        self._prev_gray = None
        self._bgsub = cv2.createBackgroundSubtractorMOG2(
            history=200, varThreshold=25, detectShadows=False,
        )
        self._clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        # CLAHE agressif pour la fumée (amplification locale forte)
        self._clahe_smoke = cv2.createCLAHE(clipLimit=8.0, tileGridSize=(4, 4))
        self.mode = "raw"
        self.last_flow_mag = 0.0
        self.last_flow_std = 0.0
        self.last_flow_median = 0.0
        self.last_vort_max = 0.0
        self.use_mask = False
        self.smoke_enhance = True   # prétraitement fumée avant Farneback
        self.hsv_low = np.array([0, 0, 180], dtype=np.uint8)
        self.hsv_high = np.array([180, 60, 255], dtype=np.uint8)
        # ROI normalisee [0..1] : x, y, w, h. None = toute l image
        self.roi = None

    def set_mode(self, mode):
        if mode in self.MODES:
            self.mode = mode
            self._prev_gray = None

    def set_hsv(self, h_lo=None, s_lo=None, v_lo=None,
                h_hi=None, s_hi=None, v_hi=None):
        if h_lo is not None: self.hsv_low[0] = int(h_lo)
        if s_lo is not None: self.hsv_low[1] = int(s_lo)
        if v_lo is not None: self.hsv_low[2] = int(v_lo)
        if h_hi is not None: self.hsv_high[0] = int(h_hi)
        if s_hi is not None: self.hsv_high[1] = int(s_hi)
        if v_hi is not None: self.hsv_high[2] = int(v_hi)

    def set_roi(self, x, y, w, h):
        if w <= 0 or h <= 0:
            self.roi = None
        else:
            self.roi = (max(0.0, min(1.0, x)), max(0.0, min(1.0, y)),
                        max(0.01, min(1.0, w)), max(0.01, min(1.0, h)))

    def clear_roi(self):
        self.roi = None

    def _crop_roi(self, arr):
        if self.roi is None:
            return arr, None
        h, w = arr.shape[:2]
        x0 = int(self.roi[0] * w); y0 = int(self.roi[1] * h)
        x1 = int((self.roi[0] + self.roi[2]) * w)
        y1 = int((self.roi[1] + self.roi[3]) * h)
        x1 = min(w, x1); y1 = min(h, y1)
        return arr[y0:y1, x0:x1], (x0, y0, x1, y1)

    def _enhance_smoke(self, gray):
        """Prétraitement pour amplifier la texture de la fumée avant Farneback.
        1. CLAHE agressif : booste les gradients locaux faibles
        2. Filtre bilatéral : réduit le bruit haute fréquence sans lisser les bords
        3. Normalisation pleine dynamique : utilise tout le 0-255
        """
        enhanced = self._clahe_smoke.apply(gray)
        # filtre bilatéral : préserve les bords (filets de fumée) tout en lissant les zones uniformes
        enhanced = cv2.bilateralFilter(enhanced, d=5, sigmaColor=30, sigmaSpace=5)
        # normalisation contraste pleine dynamique
        mn, mx = int(enhanced.min()), int(enhanced.max())
        if mx > mn:
            enhanced = ((enhanced.astype(np.int32) - mn) * 255 // (mx - mn)).astype(np.uint8)
        return enhanced

    def process(self, rgb_display, gray_analyze=None):
        """rgb_display : image RGB pleine resolution pour affichage.
        gray_analyze   : image gris basse res pour analyse (sinon derivee de display).
        Retourne BGR pret pour JPEG.
        """
        bgr = cv2.cvtColor(rgb_display, cv2.COLOR_RGB2BGR)
        if gray_analyze is None:
            gray_analyze = cv2.cvtColor(rgb_display, cv2.COLOR_RGB2GRAY)
            gray_analyze = cv2.resize(gray_analyze, (0, 0), fx=0.4, fy=0.4)

        if self.use_mask:
            hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
            m = cv2.inRange(hsv, self.hsv_low, self.hsv_high)
            bgr = cv2.bitwise_and(bgr, bgr, mask=m)

        mode = self.mode

        if mode == "contrast":
            g_full = cv2.cvtColor(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
                                  if self.use_mask else
                                  cv2.cvtColor(rgb_display, cv2.COLOR_RGB2GRAY),
                                  cv2.COLOR_GRAY2GRAY) if False else cv2.cvtColor(rgb_display, cv2.COLOR_RGB2GRAY)
            eq = self._clahe.apply(g_full)
            out = cv2.cvtColor(eq, cv2.COLOR_GRAY2BGR)
            self._draw_roi(out)
            return out

        if mode == "bgsub":
            fg = self._bgsub.apply(bgr)
            fg3 = cv2.cvtColor(fg, cv2.COLOR_GRAY2BGR)
            out = cv2.addWeighted(bgr, 0.4, fg3, 0.6, 0)
            self._draw_roi(out)
            return out

        if mode in ("arrows", "heatmap", "vorticity", "streamlines"):
            # crop ROI pour analyse
            ga_roi, roi_box = self._crop_roi(gray_analyze)
            # prétraitement fumée : booste texture avant Farneback
            ga_proc = self._enhance_smoke(ga_roi) if self.smoke_enhance else ga_roi
            if self._prev_gray is None or self._prev_gray.shape != ga_proc.shape:
                self._prev_gray = ga_proc
                self._draw_roi(bgr)
                return bgr

            flow = cv2.calcOpticalFlowFarneback(
                self._prev_gray, ga_proc, None,
                FLOW_PYR_SCALE, FLOW_LEVELS, FLOW_WINSIZE,
                FLOW_ITERATIONS, FLOW_POLY_N, FLOW_POLY_SIGMA, FLOW_FLAGS,
            )
            self._prev_gray = ga_proc
            mag, ang = cv2.cartToPolar(flow[..., 0], flow[..., 1])
            self.last_flow_mag = float(mag.mean())
            self.last_flow_std = float(mag.std())
            self.last_flow_median = float(np.median(mag))

            h_d, w_d = bgr.shape[:2]
            if roi_box is None:
                roi_box = (0, 0, w_d, h_d)
            rx0, ry0, rx1, ry1 = roi_box
            roi_w = rx1 - rx0; roi_h = ry1 - ry0

            if mode == "heatmap":
                hsv = np.zeros((*mag.shape, 3), dtype=np.uint8)
                hsv[..., 0] = (ang * 90 / np.pi).astype(np.uint8)
                hsv[..., 1] = 255
                # gain adaptatif : normalise par le 95e percentile pour ne pas saturer
                mag_p95 = float(np.percentile(mag, 95)) if mag.max() > 0 else 1.0
                gain = min(255.0 / max(mag_p95, 0.05), 120.0)
                hsv[..., 2] = np.clip(mag * gain, 0, 255).astype(np.uint8)
                color = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
                color = cv2.resize(color, (roi_w, roi_h))
                bgr[ry0:ry1, rx0:rx1] = cv2.addWeighted(
                    bgr[ry0:ry1, rx0:rx1], 0.4, color, 0.6, 0)

            elif mode == "vorticity":
                # omega = dv/dx - du/dy
                fy, fx = np.gradient(flow[..., 1]), np.gradient(flow[..., 0])
                vort = fy[1] - fx[0]   # d(v)/dx - d(u)/dy
                vmax = max(np.abs(vort).max(), 1e-6)
                self.last_vort_max = float(vmax)
                # normalise [-1,1] et map en BGR : rouge (CCW+) bleu (CW-)
                norm = np.clip(vort / vmax, -1, 1)
                r = np.clip(norm * 255, 0, 255).astype(np.uint8)
                b = np.clip(-norm * 255, 0, 255).astype(np.uint8)
                color = np.zeros((*vort.shape, 3), dtype=np.uint8)
                color[..., 0] = b   # canal B
                color[..., 2] = r   # canal R
                color = cv2.resize(color, (roi_w, roi_h))
                bgr[ry0:ry1, rx0:rx1] = cv2.addWeighted(
                    bgr[ry0:ry1, rx0:rx1], 0.5, color, 0.5, 0)

            elif mode == "streamlines":
                # Integration RK2 du champ de vitesse depuis une grille de seeds.
                # Chaque streamline = serie de positions parcourues en suivant flow.
                hs, ws = flow.shape[:2]
                sx_disp = roi_w / ws; sy_disp = roi_h / hs
                n_seeds_x = 18
                n_seeds_y = 12
                step_size = 0.7   # facteur d'avance (px de flow ratio)
                n_steps = 24      # longueur max d'une streamline
                # Colormap par magnitude
                vmax_local = max(self.last_flow_mag * 3.0, 1.0)
                for sy_i in np.linspace(1, hs - 2, n_seeds_y):
                    for sx_i in np.linspace(1, ws - 2, n_seeds_x):
                        x, y = float(sx_i), float(sy_i)
                        pts = []
                        for _ in range(n_steps):
                            ix, iy = int(x), int(y)
                            if not (0 <= ix < ws and 0 <= iy < hs):
                                break
                            fx_, fy_ = flow[iy, ix]
                            m = fx_ * fx_ + fy_ * fy_
                            if m < 0.005:  # seuil abaisse pour fumee lente
                                break
                            pts.append((int(rx0 + x * sx_disp),
                                        int(ry0 + y * sy_disp)))
                            # RK2 mid-point
                            x_m = x + 0.5 * fx_ * step_size
                            y_m = y + 0.5 * fy_ * step_size
                            jx, jy = int(x_m), int(y_m)
                            if not (0 <= jx < ws and 0 <= jy < hs):
                                break
                            fxm, fym = flow[jy, jx]
                            x += fxm * step_size
                            y += fym * step_size
                        if len(pts) >= 3:
                            # Couleur selon vitesse moyenne de la streamline
                            mid = pts[len(pts) // 2]
                            ix0, iy0 = int((mid[0] - rx0) / sx_disp), int((mid[1] - ry0) / sy_disp)
                            ix0 = max(0, min(ws - 1, ix0)); iy0 = max(0, min(hs - 1, iy0))
                            mm = float(np.sqrt(flow[iy0, ix0][0] ** 2 + flow[iy0, ix0][1] ** 2))
                            t = min(mm / vmax_local, 1.0)
                            # bleu (lent) -> cyan -> jaune -> rouge (rapide)
                            r_c = int(255 * t)
                            g_c = int(255 * (1 - abs(2 * t - 1)))
                            b_c = int(255 * (1 - t))
                            poly = np.array(pts, dtype=np.int32).reshape(-1, 1, 2)
                            cv2.polylines(bgr, [poly], False, (b_c, g_c, r_c), 1, cv2.LINE_AA)
                            # tete d'arrowhead
                            if len(pts) >= 2:
                                cv2.circle(bgr, pts[-1], 2, (b_c, g_c, r_c), -1)

            else:  # arrows
                step = max(4, ga_roi.shape[1] // 24)
                hs, ws = mag.shape
                sx = roi_w / ws; sy = roi_h / hs
                for y in range(step // 2, hs, step):
                    for x in range(step // 2, ws, step):
                        fx_, fy_ = flow[y, x]
                        if fx_ * fx_ + fy_ * fy_ < 0.05:  # seuil abaisse pour fumee lente
                            continue
                        p1 = (int(rx0 + x * sx), int(ry0 + y * sy))
                        p2 = (int(rx0 + (x + fx_ * 3) * sx),
                              int(ry0 + (y + fy_ * 3) * sy))
                        cv2.arrowedLine(bgr, p1, p2, (0, 255, 255), 1, tipLength=0.4)
            self._draw_roi(bgr)
            return bgr

        self._draw_roi(bgr)
        return bgr

    def _draw_roi(self, bgr):
        if self.roi is None:
            return
        h, w = bgr.shape[:2]
        x0 = int(self.roi[0] * w); y0 = int(self.roi[1] * h)
        x1 = int((self.roi[0] + self.roi[2]) * w)
        y1 = int((self.roi[1] + self.roi[3]) * h)
        cv2.rectangle(bgr, (x0, y0), (x1, y1), (255, 200, 50), 1)
