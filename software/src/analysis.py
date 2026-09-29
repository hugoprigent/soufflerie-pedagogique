"""Analyse signal + formules aerodynamiques.

RollingBuffer    : anneau O(1) avec stats (mean/std/rms/median/MAD) et PSD.
                   Detection de regime stationnaire (drift faible).

PSD (Welch)      : meilleure SNR que FFT brute, divise le signal en
                   segments avec recouvrement et moyenne les periodogrammes.

Stats robustes   : median + MAD (median absolute deviation) sont insensibles
                   aux outliers. MAD ≈ 0.67 σ pour distrib gaussienne.

Formules aero    : Reynolds, Strouhal, coefficients, conversion ADU<->N.
"""

from collections import deque
import math

try:
    import numpy as np
    _HAS_NP = True
except Exception:
    _HAS_NP = False


class RollingBuffer:
    def __init__(self, size):
        self.buf = deque(maxlen=size)

    def push(self, v):
        self.buf.append(float(v))

    def clear(self):
        self.buf.clear()

    def __len__(self):
        return len(self.buf)

    # --- stats classiques ---
    def mean(self):
        return sum(self.buf) / len(self.buf) if self.buf else 0.0

    def std(self):
        n = len(self.buf)
        if n < 2:
            return 0.0
        m = self.mean()
        return math.sqrt(sum((x - m) ** 2 for x in self.buf) / (n - 1))

    def rms(self):
        n = len(self.buf)
        if not n:
            return 0.0
        return math.sqrt(sum(x * x for x in self.buf) / n)

    def min_max(self):
        if not self.buf:
            return 0.0, 0.0
        return min(self.buf), max(self.buf)

    # --- stats robustes ---
    def median(self):
        if not self.buf:
            return 0.0
        s = sorted(self.buf)
        n = len(s)
        return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])

    def mad(self):
        """Median Absolute Deviation (robuste aux outliers)."""
        if not self.buf:
            return 0.0
        med = self.median()
        s = sorted(abs(x - med) for x in self.buf)
        n = len(s)
        return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])

    # --- analyse spectrale ---
    def peak_freq(self, fs):
        """FFT brute avec Hanning. Retourne (freq_Hz, magnitude)."""
        n = len(self.buf)
        if not _HAS_NP or n < 32 or fs <= 0:
            return 0.0, 0.0
        a = np.array(self.buf, dtype=float)
        a = a - a.mean()
        a = a * np.hanning(n)
        spec = np.abs(np.fft.rfft(a))
        freqs = np.fft.rfftfreq(n, 1.0 / fs)
        if len(spec) < 3:
            return 0.0, 0.0
        i = 1 + int(np.argmax(spec[1:]))
        return float(freqs[i]), float(spec[i])

    def welch_psd(self, fs, nperseg=64):
        """Densite spectrale par methode de Welch (segments + overlap 50%).
        Retourne (freqs[], psd[]). Meilleure SNR que peak_freq brute.
        """
        n = len(self.buf)
        if not _HAS_NP or n < nperseg or fs <= 0:
            return [], []
        a = np.array(self.buf, dtype=float)
        a = a - a.mean()
        win = np.hanning(nperseg)
        wnorm = (win * win).sum() * fs
        step = nperseg // 2
        segs = []
        for start in range(0, n - nperseg + 1, step):
            seg = a[start:start + nperseg] * win
            spec = np.abs(np.fft.rfft(seg)) ** 2
            segs.append(spec / wnorm)
        if not segs:
            return [], []
        psd = np.mean(segs, axis=0)
        freqs = np.fft.rfftfreq(nperseg, 1.0 / fs)
        return freqs.tolist(), psd.tolist()

    def peak_freq_welch(self, fs, nperseg=64):
        """Freq dominante via Welch (plus stable que FFT brute)."""
        freqs, psd = self.welch_psd(fs, nperseg)
        if not freqs or len(psd) < 3:
            return 0.0, 0.0
        i = 1 + int(np.argmax(psd[1:])) if _HAS_NP else 1
        return float(freqs[i]), float(psd[i])

    # --- regime stationnaire ---
    def is_steady(self, tol_rel=0.02, min_n=20):
        """Retourne True si le signal est stationnaire :
        ecart median(premiere moitie) vs median(seconde moitie) < tol_rel * median(total).
        """
        n = len(self.buf)
        if n < min_n:
            return False
        a = list(self.buf)
        mid = n // 2
        sa = sorted(a[:mid]); sb = sorted(a[mid:])
        ma = sa[len(sa) // 2]
        mb = sb[len(sb) // 2]
        med = sorted(a)[n // 2]
        if abs(med) < 1e-9:
            return abs(mb - ma) < tol_rel
        return abs(mb - ma) / abs(med) < tol_rel

    def drift_rate(self):
        """Vitesse de derive [unite/s implicite] : pente lineaire du signal."""
        n = len(self.buf)
        if n < 4:
            return 0.0
        if _HAS_NP:
            a = np.array(self.buf)
            x = np.arange(n)
            return float(np.polyfit(x, a, 1)[0])
        # fallback python pur
        xs = list(range(n))
        ys = list(self.buf)
        mx = sum(xs) / n; my = sum(ys) / n
        num = sum((xs[i] - mx) * (ys[i] - my) for i in range(n))
        den = sum((xs[i] - mx) ** 2 for i in range(n))
        return num / den if den else 0.0


# ============================================================
# Formules aerodynamiques
# ============================================================

def reynolds(v_ms, chord_m, rho=1.204, mu=1.82e-5):
    if mu <= 0 or chord_m <= 0:
        return 0.0
    return rho * v_ms * chord_m / mu


def strouhal(f_hz, chord_m, v_ms):
    if v_ms <= 0:
        return 0.0
    return f_hz * chord_m / v_ms


def aero_coeff(force_N, v_ms, area_m2, rho=1.204):
    q = 0.5 * rho * v_ms * v_ms
    denom = q * area_m2
    if denom <= 1e-9:
        return 0.0
    return force_N / denom


def dynamic_pressure(v_ms, rho=1.204):
    return 0.5 * rho * v_ms * v_ms


def airspeed_from_flow(mag_px_per_frame, fps, px_per_m):
    if px_per_m <= 0 or fps <= 0:
        return 0.0
    return mag_px_per_frame * fps / px_per_m


def airspeed_from_fan(rpm, kv, k0=0.0):
    """Modele lineaire V = kv * RPM + k0.
    kv : m/s par RPM (a calibrer empiriquement).
    k0 : offset (typiquement 0).
    Si RPM = 0, retourne 0 (pas de flux).
    """
    if rpm <= 0:
        return 0.0
    v = kv * rpm + k0
    return max(0.0, v)


def fan_calib_kv(rpm, target_v_ms, k0=0.0):
    """Calcule kv tel que la formule donne target_v_ms au RPM actuel."""
    if rpm <= 0:
        return 0.0
    return (target_v_ms - k0) / rpm


def airspeed_from_pressure(dp_pa, rho=1.204):
    """V = sqrt(2 * ΔP / ρ) depuis la chute de pression statique en section.
    dp_pa : ΔP = P_ref_ambiant - P_section (Pa), doit être > 0 en présence de flux.
    rho   : densité de l'air (kg/m³), défaut 1.204 @ 20°C.
    Retourne 0 si ΔP ≤ 0.
    """
    if dp_pa <= 0.0:
        return 0.0
    return math.sqrt(2.0 * dp_pa / rho)


def pressure_calib_kv(rpm, dp_pa, rho=1.204, k0=0.0):
    """Calcule kv empirique depuis une mesure (RPM, ΔP) du BMP280."""
    v = airspeed_from_pressure(dp_pa, rho)
    return fan_calib_kv(rpm, v, k0)


# ============================================================
# Filtre de Kalman 1D pour fusion d'estimateurs de vitesse
# ============================================================

class KalmanV:
    """Fusion bayesienne de plusieurs estimateurs scalaires.

    Modele d'evolution : V(t+dt) = V(t) + w,  w ~ N(0, Q)
    Mesures : z = V + v,  v ~ N(0, R_i) selon le capteur.

    Donne plus de poids au capteur le plus precis (R faible).
    Lisse naturellement les bruits et fait converger les estimations.
    """

    def __init__(self, sigma_process=0.3, sigma_flow=1.5, sigma_fan=0.8):
        self.x = 0.0
        self.P = 100.0
        self.Q = sigma_process ** 2     # variance de procede
        self.R_flow = sigma_flow ** 2   # variance flux optique
        self.R_fan = sigma_fan ** 2     # variance estimation moteur

    def predict(self, dt=0.1):
        self.P += self.Q * dt

    def update_flow(self, z):
        if z is None or z <= 0:
            return
        K = self.P / (self.P + self.R_flow)
        self.x += K * (z - self.x)
        self.P *= (1.0 - K)

    def update_fan(self, z):
        if z is None or z < 0:
            return
        K = self.P / (self.P + self.R_fan)
        self.x += K * (z - self.x)
        self.P *= (1.0 - K)

    def reset(self):
        self.x = 0.0; self.P = 100.0

    @property
    def value(self):
        return self.x

    @property
    def sigma(self):
        return math.sqrt(self.P) if self.P > 0 else 0.0


# ============================================================
# Spectrogramme temps reel (waterfall)
# ============================================================

class SpectrogramBuffer:
    """Stocke les N derniers vecteurs PSD pour affichage waterfall."""

    def __init__(self, n_cols=80, n_bins=24):
        self.n_cols = n_cols
        self.n_bins = n_bins
        self.cols = deque(maxlen=n_cols)
        self.fmax = 0.0

    def push_psd(self, freqs, psd):
        if not freqs or not psd:
            return
        bins = min(self.n_bins, len(psd))
        self.cols.append([float(psd[i]) for i in range(bins)])
        self.fmax = freqs[bins - 1] if bins else 0.0

    def matrix(self):
        """Retourne (cols x bins) liste de listes, plus haute valeur globale."""
        return list(self.cols), self.fmax


# ============================================================
# Bootstrap : intervalle de confiance sur la moyenne
# ============================================================

def bootstrap_mean_ci(samples, n_boot=200, alpha=0.05):
    """Intervalle de confiance sur la moyenne par bootstrap non-parametrique.
    Retourne (mean, ci_low, ci_high).
    """
    n = len(samples)
    if n < 5 or not _HAS_NP:
        if n == 0:
            return 0.0, 0.0, 0.0
        m = sum(samples) / n
        return m, m, m
    arr = np.array(samples, dtype=float)
    means = np.empty(n_boot)
    rng = np.random.default_rng(42)
    for i in range(n_boot):
        idx = rng.integers(0, n, size=n)
        means[i] = arr[idx].mean()
    lo = float(np.percentile(means, 100 * alpha / 2))
    hi = float(np.percentile(means, 100 * (1 - alpha / 2)))
    return float(arr.mean()), lo, hi


# ============================================================
# Wake survey : estimation de la trainee depuis u(y) dans le sillage
# ============================================================

def wake_drag_per_span(u_profile, v_inf, rho=1.204, dy_m=1e-3):
    """Estime F_D / span depuis le profil de vitesse u(y) dans le sillage.

    F_D/span = rho * sum( u(y) * (V_inf - u(y)) ) * dy

    u_profile : liste/array de vitesses [m/s] le long d'une coupe verticale
    v_inf     : vitesse a l'infini amont [m/s]
    dy_m      : pas spatial entre echantillons [m]
    Retourne F_D par unite d'envergure [N/m].
    """
    if not _HAS_NP or v_inf <= 0:
        return 0.0
    u = np.array(u_profile, dtype=float)
    deficit = u * (v_inf - u)
    return float(rho * np.trapz(deficit, dx=dy_m))


def adu_to_grams(delta_adu, scale_adu_per_g):
    if abs(scale_adu_per_g) < 1e-9:
        return 0.0
    return delta_adu / scale_adu_per_g


def grams_to_newtons(g):
    return g * 9.80665e-3


# ============================================================
# Correction de blocage (Maskell)
# ============================================================

def blockage_correction(v_measured, area_model, area_tunnel):
    """Correction de Maskell pour un modele dans tunnel ferme.
    V_corr = V_mes * (1 + eps),  eps = (1/4) * (S_model / S_tunnel)
    Valide pour S_model/S_tunnel < ~0.1 ; au-dela, surestime.
    """
    if area_tunnel <= 0:
        return v_measured
    eps = 0.25 * area_model / area_tunnel
    return v_measured * (1.0 + eps)


# ============================================================
# Incertitudes
# ============================================================

def cl_uncertainty(F, V, A, rho, dF, dV, dA, dRho):
    """Propagation d'incertitude relative sur CL = 2F/(rho*V^2*A).
    Retourne dCL/CL en valeur absolue.
    """
    CL = aero_coeff(F, V, A, rho)
    if abs(CL) < 1e-9 or V <= 0 or A <= 0 or rho <= 0:
        return 0.0
    rel = math.sqrt(
        (dF / F) ** 2 if F else 0
        + (2 * dV / V) ** 2
        + (dA / A) ** 2
        + (dRho / rho) ** 2
    )
    return rel
