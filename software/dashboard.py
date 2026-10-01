"""Dashboard soufflerie.

Page principale: camera + force + ventilateur + enregistrement.
Page /settings: calibration par capteur (force, ventilo, camera, aero).
"""

import collections
import csv
import io
import json
import math
import os
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import cv2
import RPi.GPIO as GPIO
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
from picamera2.outputs import FileOutput

from config import (
    PIN_HX711_DT, PIN_HX711_SCK, HX711_OFFSET, HX711_SCALE,
    FORCE_FILTER_WINDOW, FORCE_SAMPLE_HZ, SUPPORT_DRAG_N,
    CAM_WIDTH, CAM_HEIGHT, CAM_FPS, CAM_PX_PER_M,
    CAM_BRIGHTNESS, CAM_CONTRAST, CAM_SATURATION,
    CAM_AE_ENABLE, CAM_AWB_ENABLE, CAM_EXPOSURE_US, CAM_GAIN,
    AIR_DENSITY, AIR_VISCOSITY,
    PROFILE_CHORD_M, PROFILE_SPAN_M, PROFILE_AOA_DEG,
    FAN_KV, FAN_K0,
    A_FAN_M2, A_TEST_M2, CONTRACTION,
    PRESSURE_I2C_BUS, PRESSURE_I2C_ADDR,
)
from src.fan_control import FanController
from src.modulino import ModulinoKnob
from src.camera_proc import FlowProcessor
from src.led_control import LedController
from src.pressure_sensor import BMP280
from src.analysis import (
    RollingBuffer, reynolds, strouhal, aero_coeff,
    airspeed_from_flow, airspeed_from_fan, fan_calib_kv,
    airspeed_from_pressure, pressure_calib_kv,
    adu_to_grams, grams_to_newtons,
    KalmanV, SpectrogramBuffer, bootstrap_mean_ci, wake_drag_per_span,
)
from src.http_security import request_allowed

# ---------- Calibration persistante ----------
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_ROOT = os.path.abspath(os.path.expanduser(os.environ.get("SOUFFLERIE_DATA_DIR", os.path.join(BASE_DIR, "data"))))
CALIB_FILE = os.path.join(DATA_ROOT, "calib.json")
HTTP_BIND_HOST = os.environ.get("SOUFFLERIE_BIND_HOST", "127.0.0.1")
WIFI_CONTROL_ENABLED = os.environ.get("SOUFFLERIE_ENABLE_WIFI_CONTROL") == "1"

# ---------- GPIO global ----------
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# ---------- Etat partage ----------
state = {
    # force
    "raw": 0,
    "delta": 0.0,           # delta adu (brut - offset)
    "force_g": 0.0,         # filtre (moyenne glissante)
    "force_N": 0.0,
    "offset": HX711_OFFSET,
    "scale": HX711_SCALE,
    "filter_window": FORCE_FILTER_WINDOW,
    "force_mean": 0.0,      # stats buffer
    "force_std": 0.0,
    "force_rms": 0.0,
    "force_net_N": 0.0,
    "support_drag_N": SUPPORT_DRAG_N,
    "support_drag_enabled": False,
    "fan_drag_table": [],          # [{duty, F_N}] courbe calibration ventilo
    "peak_freq_hz": 0.0,
    "status": "init",
    # fan
    "fan_duty": 0.0,
    "fan_rpm": 0,
    "encoder_step": 2.0,
    # modulino debug
    "modulino_raw": [0, 0, 0, 0, 0],
    "modulino_last_delta": 0,
    "modulino_btn": False,
    "modulino_err": "",
    # camera
    "cam_mode": "raw",
    "cam_mask": False,
    "cam_brightness": CAM_BRIGHTNESS,
    "cam_contrast": CAM_CONTRAST,
    "cam_saturation": CAM_SATURATION,
    "cam_ae": CAM_AE_ENABLE,
    "cam_awb": CAM_AWB_ENABLE,
    "cam_colour_gain_r": 2.0,   # gain rouge manuel (AWB off)
    "cam_colour_gain_b": 1.5,   # gain bleu  manuel (AWB off)
    "cam_fps": 0.0,
    "hsv_low": [0, 0, 180],
    "hsv_high": [180, 60, 255],
    "px_per_m": CAM_PX_PER_M,
    "cam_dist_m": 0.5,           # distance camera-objet (m)
    "flow_mag": 0.0,
    "flow_std": 0.0,
    "flow_median": 0.0,
    "vort_max": 0.0,
    "roi": None,           # [x, y, w, h] normalise
    "airspeed_ms": 0.0,         # estime depuis le flux optique
    "airspeed_fan_ms": 0.0,     # estime depuis RPM ventilo (lineaire)
    "airspeed_pressure_ms": 0.0,# estime depuis BMP280 (ΔP Bernoulli)
    "airspeed_avg_ms": 0.0,     # moyenne arithmetique des deux
    "airspeed_kalman_ms": 0.0,  # fusion Kalman des deux estimateurs
    "airspeed_kalman_sigma": 0.0,
    "fan_kv": FAN_KV,
    "fan_k0": FAN_K0,
    # BMP280
    "pressure_pa": 0.0,         # pression absolue (Pa)
    "pressure_delta_pa": 0.0,   # ΔP = P_ref - P_section (Pa)
    "pressure_ref_pa": 0.0,     # pression de reference (ventilo arrete)
    "pressure_ok": False,
    # Source de vitesse utilisee dans les experiences
    "airspeed_source": "kalman",  # "kalman" | "fan" | "pressure" | "manual"
    "airspeed_manual_ms": 0.0,
    # Courbe de calibration vitesse (liste de {duty,rpm,v_fan,v_pressure,dp_pa})
    "speed_calib_table": [],
    # Bootstrap CI sur la force moyenne
    "force_ci_low": 0.0,
    "force_ci_high": 0.0,
    # Experience automatique
    "exp_running": False,
    "exp_progress": 0.0,
    "exp_results": [],
    "force_median": 0.0,
    "force_mad": 0.0,
    "force_drift": 0.0,
    "steady": False,
    # aero
    "object_type": "airfoil",   # airfoil | plate | cube | sphere | cylinder
    "chord_m": PROFILE_CHORD_M,
    "span_m": PROFILE_SPAN_M,
    "aoa_deg": PROFILE_AOA_DEG,
    "rho": AIR_DENSITY,
    "reynolds": 0.0,
    "strouhal": 0.0,
    "CL": 0.0,
    "CD": 0.0,
    # recording
    "recording": False,
    "record_file": "",
}
state_lock = threading.Lock()
frame_lock = threading.Lock()
frame_event = threading.Event()   # signale qu'un nouveau JPEG est disponible


# ---------- Fonctions calibration (definies tot : utilisees avant HTTP) ----------

def _interp_duty(duty, table):
    """Interpolation lineaire dans la table de trainee. Pure function, sans lock."""
    if not table:
        return 0.0
    duties = [p["duty"] for p in table]
    forces = [p["F_N"]  for p in table]
    if duty <= duties[0]:  return forces[0]
    if duty >= duties[-1]: return forces[-1]
    for i in range(len(duties) - 1):
        if duties[i] <= duty <= duties[i + 1]:
            t = (duty - duties[i]) / (duties[i + 1] - duties[i])
            return forces[i] + t * (forces[i + 1] - forces[i])
    return 0.0


def _save_calib():
    """Sauvegarde TOUT dans data/calib.json — persiste entre redémarrages et déploiements SCP."""
    try:
        os.makedirs(os.path.dirname(CALIB_FILE), exist_ok=True)
        with state_lock:
            d = {
                "version": 4,
                "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                # HX711
                "hx711_offset":          state["offset"],
                "hx711_scale":           state["scale"],
                "filter_window":         state.get("filter_window", FORCE_FILTER_WINDOW),
                # Support aéro
                "support_drag_N":        state["support_drag_N"],
                "support_drag_enabled":  state["support_drag_enabled"],
                "fan_drag_table":        state.get("fan_drag_table", []),
                # Profil / aéro
                "chord_m":               state.get("chord_m", PROFILE_CHORD_M),
                "span_m":                state.get("span_m", PROFILE_SPAN_M),
                "aoa_deg":               state.get("aoa_deg", PROFILE_AOA_DEG),
                "rho":                   state.get("rho", AIR_DENSITY),
                "object_type":           state.get("object_type", "airfoil"),
                # Vitesse
                "fan_kv":                state.get("fan_kv", FAN_KV),
                "fan_k0":                state.get("fan_k0", FAN_K0),
                "airspeed_source":       state.get("airspeed_source", "kalman"),
                "speed_calib_table":     state.get("speed_calib_table", []),
                # Caméra
                "px_per_m":              state.get("px_per_m", CAM_PX_PER_M),
                "cam_dist_m":            state.get("cam_dist_m", 0.5),
                "cam_colour_gain_r":     state.get("cam_colour_gain_r", 2.0),
                "cam_colour_gain_b":     state.get("cam_colour_gain_b", 1.5),
                # LED
                "led_animation":         led.animation,
                "led_color":             list(led.color),
                "led_brightness":        led.brightness,
                "led_speed":             led.speed,
                "led_n_leds":            led.n,
            }
        with open(CALIB_FILE, "w") as f:
            json.dump(d, f, indent=2)
        return True, ""
    except Exception as e:
        return False, str(e)


def _load_calib():
    """Charge calib.json au demarrage. Prioritaire sur config.py."""
    try:
        with open(CALIB_FILE) as f:
            d = json.load(f)
        with state_lock:
            # HX711
            if "hx711_offset"         in d: state["offset"]               = int(d["hx711_offset"])
            if "hx711_scale"          in d: state["scale"]                = float(d["hx711_scale"])
            if "filter_window"        in d: state["filter_window"]        = int(d["filter_window"])
            # Support aéro
            if "support_drag_N"       in d: state["support_drag_N"]       = float(d["support_drag_N"])
            if "support_drag_enabled" in d: state["support_drag_enabled"] = bool(d["support_drag_enabled"])
            if "fan_drag_table"       in d: state["fan_drag_table"]       = list(d["fan_drag_table"])
            # Profil / aéro
            if "chord_m"              in d: state["chord_m"]              = float(d["chord_m"])
            if "span_m"               in d: state["span_m"]               = float(d["span_m"])
            if "aoa_deg"              in d: state["aoa_deg"]              = float(d["aoa_deg"])
            if "rho"                  in d: state["rho"]                  = float(d["rho"])
            if "object_type"          in d: state["object_type"]          = str(d["object_type"])
            # Vitesse
            if "fan_kv"               in d: state["fan_kv"]               = float(d["fan_kv"])
            if "fan_k0"               in d: state["fan_k0"]               = float(d["fan_k0"])
            if "airspeed_source"      in d: state["airspeed_source"]      = str(d["airspeed_source"])
            if "airspeed_manual_ms"   in d: state["airspeed_manual_ms"]   = float(d["airspeed_manual_ms"])
            if "speed_calib_table"    in d: state["speed_calib_table"]    = list(d["speed_calib_table"])
            # Caméra
            if "px_per_m"             in d: state["px_per_m"]             = float(d["px_per_m"])
            if "cam_dist_m"           in d: state["cam_dist_m"]           = float(d["cam_dist_m"])
            if "cam_colour_gain_r"    in d: state["cam_colour_gain_r"]    = float(d["cam_colour_gain_r"])
            if "cam_colour_gain_b"    in d: state["cam_colour_gain_b"]    = float(d["cam_colour_gain_b"])
        # LED
        led_kw = {}
        if "led_animation"  in d: led_kw["animation"]  = d["led_animation"]
        if "led_color"      in d: led_kw["color"]       = tuple(d["led_color"])
        if "led_brightness" in d: led_kw["brightness"]  = int(d["led_brightness"])
        if "led_speed"      in d: led_kw["speed"]       = int(d["led_speed"])
        if "led_n_leds"     in d: led_kw["n_leds"]      = int(d["led_n_leds"])
        if led_kw:
            led.set(**led_kw)
        print(f"[calib] chargé v{d.get('version','?')} du {d.get('saved_at','?')}")
        return True
    except FileNotFoundError:
        print("[calib] calib.json absent — valeurs config.py utilisées")
        return False
    except Exception as e:
        print(f"[calib] erreur chargement: {e}")
        return False
latest_jpeg = b""
force_buf = RollingBuffer(256)
force_avg = RollingBuffer(FORCE_FILTER_WINDOW)

# Buffer export CSV : 10 min @ 20 Hz = 12000 points
data_buffer = collections.deque(maxlen=12000)
session_start = time.time()
recording_start = [0.0]

# Etat instruments avances
kalman_v = KalmanV()
spectro = SpectrogramBuffer(n_cols=80, n_bins=24)

# Etat experience automatique
experiment = {"running": False, "progress": 0.0, "results": [], "msg": ""}

# Analyse flux optique sur video enregistree
_video_analysis_jobs = {}   # fname -> {"progress": 0..1, "done": bool, "err": ""}
_video_analysis_lock = threading.Lock()

# Conversion H264 -> MP4 en tache de fond
_conversions = {}    # mp4_basename -> {"done": bool, "err": ""}
_conv_lock = threading.Lock()

# Etat enregistrement video
recording = {
    "active": False,
    "h264": None,           # encoder
    "ffmpeg": None,         # output (mp4)
    "video_file": "",
    "csv_file": "",
    "start_time": 0.0,
    "start_t_rel": 0.0,     # session_start relatif
}
DATA_DIR_VIDEO  = os.path.join(DATA_ROOT, "videos")
DATA_DIR_CSV    = os.path.join(DATA_ROOT, "raw")
DATA_DIR_PHOTOS = os.path.join(DATA_ROOT, "photos")
DATA_DIR_ETUDES = os.path.join(DATA_ROOT, "etudes")
os.makedirs(DATA_DIR_VIDEO,  exist_ok=True)
os.makedirs(DATA_DIR_CSV,    exist_ok=True)
os.makedirs(DATA_DIR_PHOTOS, exist_ok=True)
os.makedirs(DATA_DIR_ETUDES, exist_ok=True)


def _svg_chart(pts, title="", xlabel="", ylabel="", width=400, height=155, default_color="#4af"):
    """SVG line chart. pts = list of (x, y) or (x, y, color)."""
    if not pts:
        return (f'<svg width="{width}" height="{height}" style="background:#060616;border-radius:4px">'
                f'<text x="{width//2}" y="{height//2}" fill="#555" font-size="11" '
                f'text-anchor="middle" font-family="monospace">Pas de données</text></svg>')
    pl, pr, pt_, pb = 52, 10, 20, 28
    w, h = width - pl - pr, height - pt_ - pb
    xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
    cols = [p[2] if len(p) > 2 else default_color for p in pts]
    xmin, xmax = min(xs), max(xs); ymin, ymax = min(ys), max(ys)
    xr = xmax - xmin or 1; yr = ymax - ymin or 1

    def svx(x): return pl + (x - xmin) / xr * w
    def svy(y): return pt_ + h - (y - ymin) / yr * h

    el = [f'<rect width="{width}" height="{height}" fill="#060616" rx="4"/>']
    for i in range(5):
        yv = ymin + yr * i / 4; py_g = svy(yv)
        el.append(f'<line x1="{pl}" y1="{py_g:.1f}" x2="{pl+w}" y2="{py_g:.1f}" stroke="#12122a" stroke-width="1"/>')
        el.append(f'<text x="{pl-3}" y="{py_g+3:.1f}" fill="#555" font-size="9" font-family="monospace" text-anchor="end">{yv:.3f}</text>')
    for i in range(5):
        xv = xmin + xr * i / 4; px_g = svx(xv)
        el.append(f'<text x="{px_g:.1f}" y="{pt_+h+18}" fill="#555" font-size="9" font-family="monospace" text-anchor="middle">{xv:.3f}</text>')
    el.append(f'<line x1="{pl}" y1="{pt_}" x2="{pl}" y2="{pt_+h}" stroke="#2a2a4a" stroke-width="1"/>')
    el.append(f'<line x1="{pl}" y1="{pt_+h}" x2="{pl+w}" y2="{pt_+h}" stroke="#2a2a4a" stroke-width="1"/>')
    groups: dict = {}
    for p, c in zip(pts, cols):
        groups.setdefault(c, []).append(p)
    for col, gp in groups.items():
        gp.sort(key=lambda p: p[0])
        poly = " ".join(f"{svx(p[0]):.1f},{svy(p[1]):.1f}" for p in gp)
        el.append(f'<polyline points="{poly}" fill="none" stroke="{col}" stroke-width="1.5" opacity=".75"/>')
        for p in gp:
            el.append(f'<circle cx="{svx(p[0]):.1f}" cy="{svy(p[1]):.1f}" r="3" fill="{col}"/>')
    el.append(f'<text x="{pl+w/2:.1f}" y="13" fill="#888" font-size="10" font-family="monospace" text-anchor="middle">{title}</text>')
    el.append(f'<text x="{pl+w/2:.1f}" y="{pt_+h+26}" fill="#777" font-size="9" font-family="monospace" text-anchor="middle">{xlabel}</text>')
    el.append(f'<text x="10" y="{pt_+h/2:.1f}" fill="#777" font-size="9" font-family="monospace" text-anchor="middle" transform="rotate(-90,10,{pt_+h/2:.1f})">{ylabel}</text>')
    return f'<svg width="{width}" height="{height}" style="background:#060616;border-radius:4px">{"".join(el)}</svg>'


_OBJ_LABELS = {"airfoil":"Profil portant","plate":"Plaque plane","cube":"Cube","sphere":"Sphère","cylinder":"Cylindre"}
_MODE_LABELS = {"single":"Point unique","sweep":"Sweep ↑","sweep_double":"Sweep ↑↓ double","polar":"Polaire α"}


def _build_etude_report(data):
    mode       = data.get("mode", "single")
    obj_name   = data.get("object", "—")
    saved_at   = data.get("saved_at", "—")
    params_d   = data.get("params", {})
    results    = data.get("results", {})
    sweep      = data.get("sweep", [])
    polar      = data.get("polar", [])
    smoke_photo= data.get("smoke_photo", "")
    obj_disp   = _OBJ_LABELS.get(obj_name, obj_name)
    mode_disp  = _MODE_LABELS.get(mode, mode)

    if mode == "single":
        coef_lbl = "CD" if obj_name in ("cube","sphere","cylinder","plate") else "CL"
        table_html = f"""<table>
<tr><th>Grandeur</th><th>Valeur</th><th>Unité</th></tr>
<tr><td>Force brute</td><td>{results.get('F_mean_N',0):.5f}</td><td>N</td></tr>
<tr><td>Support drag</td><td>{results.get('support_drag_N',0):.5f}</td><td>N</td></tr>
<tr><td>Force nette</td><td>{results.get('F_net_N',0):.5f}</td><td>N</td></tr>
<tr><td>Vitesse air</td><td>{results.get('V_ms',0):.3f}</td><td>m/s</td></tr>
<tr><td>Reynolds Re</td><td>{int(results.get('Re',0))}</td><td>—</td></tr>
<tr><td>{coef_lbl}</td><td>{results.get('CL',0):.4f}</td><td>—</td></tr>
</table>"""
        charts_html = ""

    elif mode == "polar":
        coef_lbl = "CD" if obj_name in ("cube","sphere","cylinder","plate") else "CL"
        rows = "".join(
            f'<tr><td style="text-align:center"><b>{r.get("angle",0)}°</b></td>'
            f'<td>{r.get("duty",0)}%</td>'
            f'<td>{r.get("rpm",0):.0f}</td>'
            f'<td>{r.get("V_ms",0):.3f}</td>'
            f'<td>{r.get("F_net_N",0):.5f}</td>'
            f'<td>{r.get("CL",0):.4f}</td>'
            f'<td>{r.get("CD",0):.4f}</td>'
            f'<td>{int(r.get("Re",0))}</td></tr>'
            for r in polar)
        table_html = f"""<table>
<tr><th>α (°)</th><th>Duty %</th><th>RPM</th><th>V (m/s)</th><th>F nette (N)</th><th>CL</th><th>CD</th><th>Re</th></tr>
{rows}</table>"""

        pts_cl  = [(r.get("angle",0), r.get("CL",0), "#4af") for r in polar]
        pts_cd  = [(r.get("angle",0), r.get("CD",0), "#f84") for r in polar]
        pts_clcd= [(r.get("CD",0),    r.get("CL",0), "#4d4") for r in polar]
        c_cl   = _svg_chart(pts_cl,   "CL vs angle d'attaque",      "α (°)", "CL",      default_color="#4af")
        c_cd   = _svg_chart(pts_cd,   "CD vs angle d'attaque",      "α (°)", "CD",      default_color="#f84")
        c_polar= _svg_chart(pts_clcd, "Polaire — CL vs CD",         "CD",    "CL",      default_color="#4d4")

        photo_cards = ""
        for r in polar:
            snap = r.get("snapUrl","") or r.get("snap","")
            if snap:
                photo_cards += (f'<div style="text-align:center;margin:4pt">'
                                f'<img src="{snap}" '
                                f'style="width:160px;height:120px;object-fit:cover;border-radius:4px;border:1px solid #ddd">'
                                f'<p style="font-size:8pt;color:#666;margin:2pt 0">α = {r.get("angle",0)}°</p>'
                                f'</div>')

        photos_section = (f'<h2>Photos par angle</h2>'
                          f'<div style="display:flex;flex-wrap:wrap;gap:8pt;margin:6pt 0">{photo_cards}</div>') if photo_cards else ""

        charts_html = (f'<h2>Courbes polaires</h2>'
                       f'<div style="display:flex;gap:12pt;flex-wrap:wrap;margin:6pt 0">{c_cl}{c_cd}{c_polar}</div>'
                       f'{photos_section}')

    else:
        rows = "".join(
            f'<tr><td style="color:{"#fa0" if r.get("dir")=="↓" else "#4af"}">{r.get("dir","↑")}</td>'
            f'<td>{r["duty"]}%</td><td>{r["V_ms"]:.3f}</td><td>{r["F_net_N"]:.5f}</td>'
            f'<td>{r["CL"]:.4f}</td><td>{int(r["Re"])}</td></tr>'
            for r in sweep)
        table_html = f"""<table>
<tr><th>Dir</th><th>Duty %</th><th>V (m/s)</th><th>F nette (N)</th><th>CL/CD</th><th>Re</th></tr>
{rows}</table>"""
        pts1 = [(r["V_ms"],  r["F_net_N"], "#fa0" if r.get("dir")=="↓" else "#4af") for r in sweep]
        pts2 = [(r["Re"],    r["CL"],      "#fa0" if r.get("dir")=="↓" else "#f84") for r in sweep]
        c1 = _svg_chart(pts1, "Force nette vs Vitesse", "V (m/s)", "F nette (N)")
        c2 = _svg_chart(pts2, "CL/CD vs Reynolds",      "Re",      "CL/CD",      default_color="#f84")
        charts_html = f'<h2>Courbes</h2><div style="display:flex;gap:12pt;flex-wrap:wrap;margin:6pt 0">{c1}{c2}</div><p style="font-size:8pt;color:#999;margin:4pt 0">Bleu = montée &nbsp; Orange = descente</p>'

    photo_html = (f'<h2>Visualisation fumée</h2><img src="/media/file/{smoke_photo}" '
                  f'style="max-width:100%;max-height:220px;border-radius:4px;margin:6pt 0">')  if smoke_photo else ""

    aoa_line = "" if mode == "polar" else f' &nbsp;·&nbsp; <b>AoA :</b> {params_d.get("aoa",0)}°'

    return f"""<!DOCTYPE html>
<html lang="fr"><head><meta charset="UTF-8">
<title>Rapport étude — {saved_at}</title>
<style>
*{{box-sizing:border-box;margin:0;padding:0}}
body{{font-family:Arial,Helvetica,sans-serif;background:#fff;color:#111;padding:18mm;font-size:10pt;line-height:1.4}}
h1{{font-size:15pt;margin-bottom:5pt}}h2{{font-size:11pt;margin:13pt 0 5pt;border-bottom:1px solid #ddd;padding-bottom:3pt}}
table{{width:100%;border-collapse:collapse;font-size:9.5pt;margin:5pt 0}}
th{{background:#f0f4f8;padding:4pt 7pt;text-align:left;border:1px solid #ccc}}
td{{padding:4pt 7pt;border:1px solid #ccc}}
.meta{{color:#555;font-size:9pt;margin-bottom:10pt}}
.np{{}} @media print{{.np{{display:none!important}}body{{padding:10mm}}}}
</style></head><body>
<div class="np" style="margin-bottom:16pt">
  <button onclick="window.print()" style="padding:6px 16px;background:#1a5fd0;color:#fff;border:none;border-radius:4px;cursor:pointer;font-size:10pt">🖨 Imprimer / PDF</button>
</div>
<h1>Rapport d'étude — Soufflerie ENSAM</h1>
<p class="meta"><b>Date :</b> {saved_at} &nbsp;·&nbsp; <b>Objet :</b> {obj_disp} &nbsp;·&nbsp; <b>Mode :</b> {mode_disp}{aoa_line}</p>
<h2>Résultats</h2>
{table_html}
{charts_html}
{photo_html}
<p style="margin-top:18pt;font-size:8pt;color:#bbb">Généré par Soufflerie ENSAM Dashboard — {saved_at}</p>
</body></html>"""

# Auto-tare : si fan_duty=0 ET signal stable pendant idle_s, retare auto
auto_tare = {
    "enabled": False,
    "idle_s": 5.0,           # secondes de stabilite requises
    "std_threshold_g": 0.3,  # ecart-type max pour considerer stable
    "last_trigger": 0.0,
    "count": 0,
}

# ---------- Camera ----------
picam = Picamera2()
# Stream dual : main (affichage HD) + lores (analyse rapide).
# Le flux optique tourne sur lores 320x240 : ~16x plus rapide qu'en pleine resolution.
LORES_W, LORES_H = 320, 240
picam.configure(picam.create_video_configuration(
    main={"size": (CAM_WIDTH, CAM_HEIGHT), "format": "RGB888"},
    lores={"size": (LORES_W, LORES_H), "format": "YUV420"},
    # min=1/30s (30fps max) max=1/10s (10fps min, adapte a la luminosite)
    controls={"FrameDurationLimits": (int(1e6 / 30), int(1e6 / 10))},
))
picam.start()
time.sleep(1)

flow_proc = FlowProcessor()


def apply_cam_controls():
    try:
        ctrl = {
            "Brightness": float(state["cam_brightness"]),
            "Contrast": float(state["cam_contrast"]),
            "Saturation": float(state["cam_saturation"]),
            "AeEnable": bool(state["cam_ae"]),
            "AwbEnable": bool(state["cam_awb"]),
        }
        if not state["cam_ae"]:
            ctrl["ExposureTime"] = int(CAM_EXPOSURE_US)
            ctrl["AnalogueGain"] = float(CAM_GAIN)
        # Gains couleur manuels (actifs seulement si AWB désactivé)
        if not state["cam_awb"]:
            rg = float(state.get("cam_colour_gain_r", 2.0))
            bg = float(state.get("cam_colour_gain_b", 1.5))
            ctrl["ColourGains"] = (rg, bg)
        picam.set_controls(ctrl)
    except Exception as e:
        with state_lock:
            state["status"] = f"camctl err: {e}"


apply_cam_controls()


# Diagnostics camera
_cam_fps_buf = collections.deque(maxlen=60)


def camera_loop():
    """Capture atomique main + lores du meme frame.
    Le flux optique tourne sur lores (rapide), l affichage utilise main (HD).
    """
    global latest_jpeg
    while True:
        t_cap = time.time()
        try:
            req = picam.capture_request()
            try:
                rgb = req.make_array("main")
                yuv = req.make_array("lores")
            finally:
                req.release()
        except Exception:
            time.sleep(0.05)
            continue
        _cam_fps_buf.append(t_cap)
        # mise a jour fps dans state toutes les 30 frames
        if len(_cam_fps_buf) > 5:
            _dt = _cam_fps_buf[-1] - _cam_fps_buf[0]
            _fps_val = round((len(_cam_fps_buf) - 1) / _dt, 1) if _dt > 0 else 0
            with state_lock:
                state["cam_fps"] = _fps_val
        # plan Y du YUV420 = grayscale lores
        gray_lores = yuv[:LORES_H, :LORES_W]
        bgr = flow_proc.process(rgb, gray_lores)
        # Redimensionne pour l'affichage (640×480) — encode 4× plus vite, flux plus léger.
        # La capture main reste en pleine résolution pour l'enregistrement vidéo.
        bgr_disp = cv2.resize(bgr, (640, 480), interpolation=cv2.INTER_LINEAR)
        ok, buf = cv2.imencode(".jpg", bgr_disp, [cv2.IMWRITE_JPEG_QUALITY, 62])
        if ok:
            with frame_lock:
                latest_jpeg = buf.tobytes()
            frame_event.set()   # réveille les clients /camera en attente
        with state_lock:
            state["flow_mag"] = round(flow_proc.last_flow_mag, 3)
            state["flow_std"] = round(flow_proc.last_flow_std, 3)
            state["flow_median"] = round(flow_proc.last_flow_median, 3)
            state["vort_max"] = round(flow_proc.last_vort_max, 3)
            state["airspeed_ms"] = round(airspeed_from_flow(
                flow_proc.last_flow_mag, CAM_FPS, state["px_per_m"]), 3)


threading.Thread(target=camera_loop, daemon=True).start()

# ---------- HX711 ----------
GPIO.setup(PIN_HX711_DT,  GPIO.IN)                     # DT : entree (manquait)
GPIO.setup(PIN_HX711_SCK, GPIO.OUT, initial=GPIO.LOW)
GPIO.output(PIN_HX711_SCK, GPIO.HIGH)
time.sleep(0.15)
GPIO.output(PIN_HX711_SCK, GPIO.LOW)
time.sleep(0.4)


# Thread unique HX711 : un seul thread accede au chip
_hx_latest = {"raw": None, "t": 0.0, "ok": False}
_hx_state_lock = threading.Lock()


def _hx_wakeup():
    """Power-cycle HX711 via SCK sans bloquer."""
    try:
        GPIO.output(PIN_HX711_SCK, GPIO.HIGH)
        time.sleep(0.15)
        GPIO.output(PIN_HX711_SCK, GPIO.LOW)
        time.sleep(0.4)
    except Exception:
        pass


def _hx_read_bits():
    """Lit 24 bits + 1 pulse gain depuis HX711 par bit-banging direct.
    Appelee uniquement quand DOUT est deja LOW.
    Retourne la valeur signee 24 bits.

    PAS de time.sleep() ici : sous Linux Python, sleep(2µs) dure en realite
    50-150µs — assez pour que HX711 entre en power-down (seuil = 60µs SCK HIGH).
    GPIO.output() prend ~2µs naturellement, ce qui satisfait le minimum HX711 (0.2µs).
    """
    val = 0
    for _ in range(24):
        GPIO.output(PIN_HX711_SCK, GPIO.HIGH)
        val = (val << 1) | GPIO.input(PIN_HX711_DT)
        GPIO.output(PIN_HX711_SCK, GPIO.LOW)
    # 25eme pulse : canal A gain 128
    GPIO.output(PIN_HX711_SCK, GPIO.HIGH)
    GPIO.output(PIN_HX711_SCK, GPIO.LOW)
    if val & 0x800000:
        val -= 0x1000000
    # Valeurs extremes = lecture corrompue (DT flottant ou power-down)
    if val in (0x7FFFFF, -1, 0x7FFFF, 0x3FFFF):
        return None
    return val


def _hx_reader():
    """Thread daemon HX711 — bit-banging direct, sans librairie bloquante."""
    # Priorite temps-reel FIFO : reduit les context-switches OS pendant la lecture 24 bits.
    # Necessite d'etre lance en root (sudo). Si non-root, continue sans RT.
    try:
        import os as _os
        _os.sched_setscheduler(
            0, _os.SCHED_FIFO,
            _os.sched_param(_os.sched_get_priority_max(_os.SCHED_FIFO)),
        )
    except Exception:
        pass
    _hx_wakeup()
    fails = 0
    while True:
        try:
            # Attendre DOUT LOW avec timeout 2s
            t_end = time.time() + 2.0
            while GPIO.input(PIN_HX711_DT) == 1:
                if time.time() > t_end:
                    _hx_wakeup()
                    fails += 1
                    break
                time.sleep(0.002)
            else:
                # DOUT est LOW : lire
                val = _hx_read_bits()
                if val is None:          # lecture corrompue, ignorer
                    fails += 1
                    continue
                with _hx_state_lock:
                    _hx_latest["raw"] = float(val)
                    _hx_latest["t"] = time.time()
                    _hx_latest["ok"] = True
                fails = 0
        except Exception:
            fails += 1
            time.sleep(0.1)


threading.Thread(target=_hx_reader, daemon=True).start()


def raw_mean(max_age=3.0):
    """Retourne la derniere lecture HX711 si recente, sinon None."""
    with _hx_state_lock:
        if _hx_latest["ok"] and (time.time() - _hx_latest["t"]) < max_age:
            return _hx_latest["raw"]
    return None


def force_loop():
    import collections
    period = 1.0 / FORCE_SAMPLE_HZ
    consecutive_errors = 0
    while True:
        t0 = time.time()
        try:
            raw = raw_mean()
            if raw is None:
                consecutive_errors += 1
                if consecutive_errors >= 3:
                    force_avg.clear()
                    force_buf.clear()
                    consecutive_errors = 0
                with state_lock:
                    state["status"] = "HX711 attente..."
                time.sleep(0.2)
                continue
            consecutive_errors = 0
            with state_lock:
                offset = state["offset"]
                scale = state["scale"]
                fw = int(state["filter_window"])
            if force_avg.buf.maxlen != fw:
                force_avg.buf = collections.deque(force_avg.buf, maxlen=max(1, fw))
            force_avg.push(raw)
            raw_f = force_avg.mean()
            delta = raw_f - offset
            g = adu_to_grams(delta, scale)
            N = grams_to_newtons(g)
            with state_lock:
                _table   = state.get("fan_drag_table", [])
                _enabled = state.get("support_drag_enabled", False)
                _duty    = state.get("fan_duty", 0.0)
                _drag_fixed = state["support_drag_N"]
            if _enabled:
                drag = _interp_duty(_duty, _table) if _table else _drag_fixed
            else:
                drag = 0.0
            net_N = N - drag
            force_buf.push(N)
            data_buffer.append({
                "t": round(time.time() - session_start, 3),
                "duty": state.get("fan_duty", 0),
                "rpm": fan.rpm,
                "delta_adu": round(delta, 1),
                "force_g": round(g, 3),
                "force_N": round(N, 5),
                "flow_mag": state.get("flow_mag", 0),
                "airspeed_ms": state.get("airspeed_ms", 0),
                "airspeed_fan_ms": state.get("airspeed_fan_ms", 0),
            })
            with state_lock:
                state["raw"] = int(raw)
                state["delta"] = round(delta, 1)
                state["force_g"] = round(g, 3)
                state["force_N"] = round(N, 5)
                state["force_net_N"] = round(net_N, 5)
                state["fan_rpm"] = fan.rpm
                state["status"] = "ok"
        except Exception as e:
            with state_lock:
                state["status"] = f"force err: {str(e)[:40]}"
        dt = time.time() - t0
        if dt < period:
            time.sleep(period - dt)


# ---------- Ventilateur ----------
fan = FanController()

# ---------- Etude complete ----------
etude = {
    "running": False, "phase": "idle", "progress": 0.0, "msg": "",
    "F_mean_N": 0.0, "F_net_N": 0.0, "V_ms": 0.0,
    "Re": 0, "CL": 0.0, "CD": 0.0, "n_samples": 0,
    "sweep_partial": [],
}
etude_lock = threading.Lock()

# ---------- Calibration ventilateur (sweep) ----------
fan_calib_state = {
    "running": False, "phase": "idle", "progress": 0.0,
    "current_duty": 0, "current_F": 0.0, "msg": "",
    "table": [], "total_points": 0,
}
fan_calib_lock = threading.Lock()

# ---------- LED strip ----------
led = LedController(n_leds=30)

# ---------- Capteur pression BMP280 ----------
pressure = BMP280(PRESSURE_I2C_BUS, PRESSURE_I2C_ADDR)
if pressure.ok:
    print(f"[{pressure.chip_name}] OK @ 0x{PRESSURE_I2C_ADDR:02X}")
else:
    print(f"[BMP/BME280] KO : {pressure.error}")

# Charge la calibration persistante (calib.json > config.py)
_load_calib()
# ModulinoKnob(fan, state, state_lock)  # desactive
threading.Thread(target=force_loop, daemon=True).start()


def analysis_loop():
    """Stats + FFT + Re/St/CL/CD toutes les 0.5s."""
    while True:
        time.sleep(0.5)
        try:
            m = force_buf.mean()
            s = force_buf.std()
            r = force_buf.rms()
            med = force_buf.median()
            mad = force_buf.mad()
            drift = force_buf.drift_rate()
            steady = force_buf.is_steady()
            # Welch PSD : plus stable que FFT brute pour detecter le pic
            f_hz, _ = force_buf.peak_freq_welch(FORCE_SAMPLE_HZ, nperseg=64)
            with state_lock:
                v_flow = state["airspeed_ms"]
                rpm = state["fan_rpm"]
                kv = state["fan_kv"]
                k0 = state["fan_k0"]
                c = state["chord_m"]
                sp = state["span_m"]
                rho = state["rho"]
                F = state["force_N"]
                asrc = state["airspeed_source"]
            # Vitesse depuis le ventilo (lineaire avec RPM)
            v_fan = airspeed_from_fan(rpm, kv, k0)
            # Vitesse depuis BMP280 (Bernoulli)
            dp = pressure.delta_pa(n_avg=3) if pressure.ok else 0.0
            v_pres = airspeed_from_pressure(dp, rho)
            p_abs, _ = pressure.read() if pressure.ok else (None, None)
            # Moyenne arithmetique simple (legacy display)
            if v_flow > 0.1 and v_fan > 0.1:
                v_avg = 0.5 * (v_flow + v_fan)
            elif v_fan > 0:
                v_avg = v_fan
            else:
                v_avg = v_flow
            # Fusion Kalman (statistiquement optimale)
            kalman_v.predict(dt=0.5)
            if v_flow > 0.05:
                kalman_v.update_flow(v_flow)
            if v_fan > 0.0:
                kalman_v.update_fan(v_fan)
            if v_pres > 0.05:
                kalman_v.update_fan(v_pres)   # meme bruit que ventilo
            v_kal = kalman_v.value
            v_kal_sigma = kalman_v.sigma
            # Vitesse effective selon source choisie par l'utilisateur
            if asrc == "fan":
                v = v_fan if v_fan > 0.0 else v_avg
            elif asrc == "pressure":
                v = v_pres if v_pres > 0.05 else v_avg
            elif asrc == "manual":
                v = state.get("airspeed_manual_ms", 0.0)
            else:   # "kalman" (defaut)
                v = v_kal if v_kal > 0.1 else v_avg
            # Spectrogramme : push PSD recente
            freqs, psd = force_buf.welch_psd(FORCE_SAMPLE_HZ, nperseg=32)
            spectro.push_psd(freqs, psd)
            # Bootstrap CI sur force moyenne (echantillon recent)
            samples_recent = list(force_buf.buf)[-64:] if len(force_buf) > 32 else list(force_buf.buf)
            f_mean, f_lo, f_hi = bootstrap_mean_ci(samples_recent, n_boot=100)
            A = c * sp
            Re = reynolds(v, c, rho, AIR_VISCOSITY)
            St = strouhal(f_hz, c, v)
            # On suppose F mesure = portance ou trainee selon orientation
            # Coefficient generique a interpreter selon le montage.
            C = aero_coeff(F, v, A, rho)
            with state_lock:
                state["force_mean"] = round(m, 5)
                state["force_std"] = round(s, 5)
                state["force_rms"] = round(r, 5)
                state["force_median"] = round(med, 5)
                state["force_mad"] = round(mad, 5)
                state["force_drift"] = round(drift, 6)
                state["steady"] = steady
                state["peak_freq_hz"] = round(f_hz, 2)
                state["reynolds"] = round(Re, 0)
                state["strouhal"] = round(St, 4)
                state["CL"] = round(C, 4)
                state["CD"] = round(C, 4)
                state["airspeed_fan_ms"] = round(v_fan, 3)
                state["airspeed_pressure_ms"] = round(v_pres, 3)
                state["airspeed_avg_ms"] = round(v_avg, 3)
                state["airspeed_kalman_ms"] = round(v_kal, 3)
                state["airspeed_kalman_sigma"] = round(v_kal_sigma, 3)
                state["pressure_delta_pa"] = round(dp, 3)
                state["pressure_ok"] = pressure.ok
                if p_abs is not None:
                    state["pressure_pa"] = round(p_abs, 2)
                    state["pressure_ref_pa"] = round(pressure.p_ref, 2) if pressure.p_ref else 0.0
                state["force_ci_low"] = round(f_lo, 5)
                state["force_ci_high"] = round(f_hi, 5)
                state["exp_running"] = experiment["running"]
                state["exp_progress"] = round(experiment["progress"], 1)
                state["exp_results"] = list(experiment["results"])
        except Exception as e:
            with state_lock:
                state["status"] = f"analysis err: {e}"


threading.Thread(target=analysis_loop, daemon=True).start()


def auto_tare_loop():
    """Surveille en continu : si fan OFF et signal stable depuis idle_s,
    declenche un tare automatique. Evite la derive thermique cumulative.
    """
    stable_since = None
    while True:
        time.sleep(1.0)
        try:
            if not auto_tare["enabled"]:
                stable_since = None
                continue
            with state_lock:
                duty = state["fan_duty"]
                # ecart-type force en grammes (sigma_N -> sigma_g)
                scale = state["scale"]
                sig_N = state["force_std"]
                f_g = state["force_g"]
            # Convertit sigma_N en sigma_g pour comparer au seuil
            sig_g = (sig_N / 9.80665e-3) if sig_N > 0 else 0  # car N = g * 9.81e-3
            # Conditions pour considerer "idle stable"
            cond_off = (duty <= 0.5)
            cond_stable = (sig_g < auto_tare["std_threshold_g"])
            cond_far_from_zero = (abs(f_g) > 0.5)  # force non triviale -> retare utile
            if cond_off and cond_stable and cond_far_from_zero:
                if stable_since is None:
                    stable_since = time.time()
                elif (time.time() - stable_since) >= auto_tare["idle_s"]:
                    # Cooldown : pas plus d'1 tare auto par 30s
                    if (time.time() - auto_tare["last_trigger"]) > 30.0:
                        # Moyenne de 10 lectures via le lecteur interne (evite la lib hx711)
                        samples = []
                        for _ in range(10):
                            v = raw_mean(max_age=1.0)
                            if v is not None:
                                samples.append(v)
                            time.sleep(0.08)
                        vals = samples
                        if vals:
                            new_off = int(sum(vals) / len(vals))
                            force_avg.clear(); force_buf.clear()
                            with state_lock:
                                state["offset"] = new_off
                                state["status"] = "auto-tare OK"
                            auto_tare["last_trigger"] = time.time()
                            auto_tare["count"] += 1
                            stable_since = None
            else:
                stable_since = None
        except Exception:
            stable_since = None


threading.Thread(target=auto_tare_loop, daemon=True).start()


def run_duty_sweep(d_from, d_to, d_step, dwell_s):
    """Experience automatique : balaye le duty, mesure F et V a chaque palier."""
    experiment["running"] = True
    experiment["progress"] = 0.0
    experiment["results"] = []
    experiment["msg"] = "demarrage"
    duties = list(range(int(d_from), int(d_to) + 1, int(d_step)))
    try:
        for i, duty in enumerate(duties):
            if not experiment["running"]:
                experiment["msg"] = "stoppe"
                break
            experiment["msg"] = f"palier {duty}% ({i+1}/{len(duties)})"
            fan.set_speed(duty)
            with state_lock:
                state["fan_duty"] = duty
            time.sleep(dwell_s)  # attendre stabilisation
            # Moyenner 2s de mesures stables
            time.sleep(2.0)
            with state_lock:
                F = state["force_mean"]
                F_lo = state["force_ci_low"]
                F_hi = state["force_ci_high"]
                V = state["airspeed_kalman_ms"] or state["airspeed_avg_ms"]
                rpm = state["fan_rpm"]
                steady = state["steady"]
            experiment["results"].append({
                "duty": duty, "F_N": round(F, 5), "F_lo": round(F_lo, 5),
                "F_hi": round(F_hi, 5), "V_ms": round(V, 3),
                "rpm": rpm, "steady": steady,
            })
            experiment["progress"] = round((i + 1) / len(duties) * 100, 1)
        if experiment["running"]:
            experiment["msg"] = "termine"
        # Coupe le ventilo a la fin
        fan.set_speed(0)
        with state_lock:
            state["fan_duty"] = 0
    finally:
        experiment["running"] = False


# ==================== HTML ====================
HTML = """<!DOCTYPE html>
<html lang="fr"><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Soufflerie</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#0a0a0a;color:#e0e0e0;font-family:ui-monospace,Menlo,monospace;height:100vh;overflow:hidden;display:flex;flex-direction:column}
/* HEADER */
header{display:flex;align-items:center;justify-content:space-between;padding:6px 10px;background:#111;border-bottom:1px solid #222;flex-shrink:0;height:44px}
.logo{font-size:14px;font-weight:bold;letter-spacing:3px;color:#4af;text-transform:uppercase}
.hdr-mid{display:flex;align-items:center;gap:8px}
.hdr-right{display:flex;align-items:center;gap:6px}
.btn-hdr{padding:5px 10px;border:1px solid #333;background:#1a1a1a;color:#ccc;border-radius:4px;cursor:pointer;font-family:inherit;font-size:11px;letter-spacing:1px;transition:border-color .2s}
.btn-hdr:hover{border-color:#4af;color:#fff}
.btn-rec{border-color:#500;color:#f55}
.btn-rec.on{background:#300;border-color:#f44;color:#f44;animation:blink 1s infinite}
@keyframes blink{0%,100%{opacity:1}50%{opacity:.5}}
#rec-timer{font-size:12px;color:#f55;min-width:40px}
/* MAIN GRID */
.main{display:grid;grid-template-columns:1fr 310px;flex:1;gap:5px;padding:5px;overflow:hidden}
/* CAMERA */
.cam-wrap{background:#000;border:1px solid #1e1e1e;border-radius:6px;overflow:hidden;position:relative}
.cam-wrap img{width:100%;height:100%;object-fit:contain;display:block}
.cam-hud{position:absolute;top:8px;left:8px;background:rgba(0,0,0,.6);backdrop-filter:blur(4px);padding:5px 9px;border-radius:4px;font-size:11px;line-height:1.8;pointer-events:none}
.cam-hud span{color:#888}
.cam-hud b{color:#4af}
.cam-mode-badge{position:absolute;bottom:8px;left:8px;background:rgba(0,0,0,.6);padding:3px 8px;border-radius:3px;font-size:10px;letter-spacing:1px;color:#4af;text-transform:uppercase}
/* SIDE */
.side{display:flex;flex-direction:column;gap:5px;overflow-y:auto}
.card{background:#141414;border:1px solid #222;border-radius:6px;padding:10px}
h2{font-size:9px;letter-spacing:2.5px;text-transform:uppercase;color:#4af;margin-bottom:8px;display:flex;justify-content:space-between;align-items:center}
/* FORCE CARD */
.force-top{display:flex;align-items:baseline;gap:8px;margin-bottom:2px}
.big-val{font-size:36px;font-weight:bold;font-variant-numeric:tabular-nums;color:#4f4;line-height:1;transition:color .3s}
.big-val.neg{color:#f84}
.unit-grp{display:flex;gap:2px;margin-left:auto}
.u-btn{padding:2px 6px;font-size:9px;border:1px solid #333;background:transparent;color:#666;border-radius:2px;cursor:pointer;font-family:inherit;transition:all .15s}
.u-btn.on{background:#4af22;border-color:#4af;color:#4af}
.force-sub{font-size:11px;color:#555;margin-bottom:4px}
canvas{width:100%;height:50px;display:block;border-radius:2px}
.stats-row{display:flex;gap:12px;font-size:10px;color:#555;margin-top:3px}
.stats-row span b{color:#888}
.btn-tare{width:100%;margin-top:6px;padding:8px;border:1px solid #1a4a99;background:#0d2040;color:#4af;border-radius:4px;cursor:pointer;font-family:inherit;font-size:11px;letter-spacing:1px;font-weight:bold;transition:background .2s}
.btn-tare:hover{background:#1a4a99}
.btn-tare:active{opacity:.7}
/* FAN CARD */
.fan-layout{display:flex;align-items:center;gap:10px}
.gauge-wrap{flex-shrink:0}
svg.gauge{width:90px;height:90px}
.g-bg{fill:none;stroke:#1e1e1e;stroke-width:7}
.g-arc{fill:none;stroke:#f90;stroke-width:7;stroke-linecap:round;transition:stroke-dasharray .3s,stroke .3s}
.g-pct{font-size:20px;font-weight:bold;fill:#f90;font-family:ui-monospace,Menlo,monospace;transition:fill .3s}
.g-rpm{font-size:9px;fill:#666;font-family:ui-monospace,Menlo,monospace}
.fan-btns{flex:1}
.btn-row{display:flex;gap:3px;margin-top:6px}
.btn-row button{flex:1;padding:8px 2px;border:1px solid #2a2a2a;background:#1a1a1a;color:#ccc;border-radius:3px;font-size:11px;font-weight:bold;cursor:pointer;font-family:inherit;transition:all .15s}
.btn-row button:hover{border-color:#f90;color:#f90}
.btn-tog{background:#0d2000 !important;border-color:#2a4a00 !important;color:#4f4 !important}
.btn-tog.off{background:#200 !important;border-color:#500 !important;color:#f55 !important}
/* AERO */
.aero-grid{display:grid;grid-template-columns:1fr 1fr;gap:4px 12px}
.aero-item{font-size:11px;color:#666}
.aero-item b{color:#ccc;float:right}
/* STATUS BAR */
.statusbar{display:flex;align-items:center;gap:14px;padding:3px 10px;background:#0d0d0d;border-top:1px solid #1a1a1a;font-size:10px;color:#444;flex-shrink:0}
.sb-dot{width:6px;height:6px;border-radius:50%;background:#4f4;flex-shrink:0}
.sb-dot.err{background:#f44}
#sb-st{color:#666}
.sb-sep{color:#2a2a2a}
/* TOAST */
#toasts{position:fixed;bottom:36px;right:12px;display:flex;flex-direction:column;gap:6px;z-index:100}
.toast{padding:7px 14px;border-radius:4px;font-size:11px;background:#1a4a99;color:#fff;border-left:3px solid #4af;animation:tin .25s ease,tout .4s 1.6s forwards}
.toast.ok{background:#1a3a00;border-color:#4f4}
.toast.err{background:#3a0000;border-color:#f44}
@keyframes tin{from{opacity:0;transform:translateX(20px)}to{opacity:1;transform:none}}
@keyframes tout{to{opacity:0;pointer-events:none}}
/* SHORTCUTS */
#shortcuts{display:none;position:fixed;inset:0;background:rgba(0,0,0,.8);z-index:200;align-items:center;justify-content:center}
#shortcuts.show{display:flex}
.sc-box{background:#171717;border:1px solid #333;border-radius:8px;padding:20px 28px;min-width:280px}
.sc-box h3{color:#4af;font-size:13px;letter-spacing:2px;margin-bottom:12px}
.sc-row{display:flex;justify-content:space-between;gap:20px;padding:3px 0;font-size:12px;color:#aaa}
.sc-key{background:#222;border:1px solid #333;border-radius:2px;padding:1px 6px;color:#4af;font-size:11px}
</style></head>
<body>

<header>
 <div class="logo">Soufflerie</div>
 <div class="hdr-mid">
  <button class="btn-hdr btn-rec" id="rec-btn" onclick="toggleRec()">&#9679; REC</button>
  <span id="rec-timer"></span>
 </div>
 <div class="hdr-right">
  <button class="btn-hdr" onclick="doSnapshot()" title="Snapshot [P]">&#128247; Photo</button>
  <button class="btn-hdr" onclick="doExport()" title="Export CSV [E]">&#8595; CSV</button>
  <button class="btn-hdr" onclick="doPDF()" title="Rapport PDF [G]">&#128196; PDF</button>
  <a href="/settings" style="text-decoration:none"><button class="btn-hdr">&#9881; Params</button></a>
  <a href="/calib" style="text-decoration:none"><button class="btn-hdr" style="background:#0a1a30;color:#8af;border:1px solid #1a3a60">&#9670; Calib</button></a>
  <a href="/etude" style="text-decoration:none"><button class="btn-hdr" style="background:#0a2a4a;color:#4af;border:1px solid #1a4a99">&#9654; Etude</button></a>
  <a href="/sensors" style="text-decoration:none"><button class="btn-hdr" style="background:#0a1a0a;color:#4f4;border:1px solid #1a4a1a">&#9889; Capteurs</button></a>
  <a href="/media" style="text-decoration:none"><button class="btn-hdr" style="background:#1a0a1a;color:#c8f;border:1px solid #3a1a5a">&#127916; Médias</button></a>
  <button class="btn-hdr" onclick="showSC()" title="Raccourcis [?]">?</button>
 </div>
</header>

<div class="main">
 <div class="cam-wrap">
  <img src="/camera" alt="camera">
  <div class="cam-hud">
   <span>Mode</span> <b id="h-mode">raw</b><br>
   <span>V&#8331;</span> <b id="h-v">0.00</b> <span>m/s</span> &nbsp;
   <span>V&#8348;</span> <b id="h-vf">0.00</b> <span>m/s</span><br>
   <span>Re</span> <b id="h-re">0</b> &nbsp;
   <span>Flow</span> <b id="h-fl">0.00</b>
  </div>
  <div class="cam-mode-badge" id="mode-badge">raw</div>
 </div>

 <div class="side">
  <!-- Force -->
  <div class="card">
   <h2>FORCE
    <div class="unit-grp">
     <button class="u-btn" id="u-adu" onclick="setUnit('adu')">ADU</button>
     <button class="u-btn" id="u-g" onclick="setUnit('g')">g</button>
     <button class="u-btn on" id="u-N" onclick="setUnit('N')">N</button>
    </div>
   </h2>
   <div class="force-top">
    <div class="big-val" id="f-val">+0</div>
   </div>
   <div class="force-sub" id="f-sub">0.00 g &mdash; 0.00000 N</div>
   <canvas id="spark"></canvas>
   <div class="stats-row">
    <span><b id="f-mu">0</b> &mu;</span>
    <span><b id="f-sig">0</b> &sigma;</span>
    <span><b id="f-fft">0 Hz</b> fft</span>
   </div>
   <button class="btn-tare" onclick="doTare()">TARE &nbsp;[T]</button>
   <div id="tare-st" style="font-size:10px;color:#4f4;text-align:center;min-height:12px;margin-top:3px"></div>
  </div>

  <!-- Fan -->
  <div class="card">
   <h2>VENTILATEUR</h2>
   <div class="fan-layout">
    <div class="gauge-wrap">
     <svg class="gauge" viewBox="0 0 100 100">
      <circle class="g-bg" cx="50" cy="50" r="40"/>
      <circle class="g-arc" id="g-arc" cx="50" cy="50" r="40"
       stroke-dasharray="0 251.3" transform="rotate(-90 50 50)"/>
      <text class="g-pct" id="g-pct" x="50" y="46" text-anchor="middle">0%</text>
      <text class="g-rpm" id="g-rpm" x="50" y="58" text-anchor="middle">0 RPM</text>
     </svg>
    </div>
    <div class="fan-btns" style="flex:1">
     <div style="font-size:10px;color:#555;margin-bottom:4px">duty cycle</div>
     <div class="btn-row">
      <button onclick="fanStep(-10)">&minus;10</button>
      <button onclick="fanStep(-5)">&minus;5</button>
     </div>
     <div class="btn-row" style="margin-top:3px">
      <button class="btn-tog off" id="fan-tog" onclick="fanToggle()">OFF</button>
     </div>
     <div class="btn-row" style="margin-top:3px">
      <button onclick="fanStep(+5)">+5</button>
      <button onclick="fanStep(+10)">+10</button>
     </div>
     <div style="margin-top:8px;display:flex;align-items:center;gap:4px">
      <input id="fan-v-inp" type="number" min="0" max="20" step="0.1" placeholder="0.0"
       style="width:60px;background:#0a0a1a;border:1px solid #2a2a4a;color:#eee;padding:3px 5px;border-radius:4px;font-size:12px"
       onkeydown="if(event.key==='Enter')fanSetV()">
      <button id="fan-v-unit-btn" onclick="fanVToggleUnit()"
       style="padding:2px 6px;font-size:11px;background:#1a1a3a;border:1px solid #3a3a6a;color:#aaf;border-radius:4px;cursor:pointer">m/s</button>
      <button onclick="fanSetV()" style="padding:2px 8px;font-size:11px">&#8594;</button>
     </div>
    </div>
   </div>
  </div>

  <!-- Aero -->
  <div class="card">
   <h2>AERO <span style="font-size:8px;color:#666;letter-spacing:1px;margin-left:6px;text-transform:none">Kalman fusion</span></h2>
   <div class="aero-grid">
    <div class="aero-item">V flux <b id="a-vf">0.00 m/s</b></div>
    <div class="aero-item">V fan <b id="a-vfn">0.00 m/s</b></div>
    <div class="aero-item">V Kalman <b id="a-vk">0.00 m/s</b></div>
    <div class="aero-item">&sigma;(V) <b id="a-vks">0.0</b></div>
    <div class="aero-item">Reynolds <b id="a-re">0</b></div>
    <div class="aero-item">Strouhal <b id="a-st">0</b></div>
    <div class="aero-item">C=F/qA <b id="a-c">0</b></div>
    <div class="aero-item">Pic FFT <b id="a-fft">0 Hz</b></div>
   </div>
  </div>

  <!-- Spectrogramme waterfall -->
  <div class="card">
   <h2>SPECTROGRAMME <span style="font-size:8px;color:#666;letter-spacing:1px;margin-left:6px;text-transform:none">force / freq</span></h2>
   <canvas id="spectro" style="height:80px"></canvas>
   <div style="font-size:9px;color:#555;display:flex;justify-content:space-between;margin-top:2px">
    <span>0 Hz</span><span id="spectro-fmax">10 Hz</span>
   </div>
  </div>

  <!-- Polaire CL/CD live -->
  <div class="card">
   <h2>POLAIRE F vs V&sup2;<span style="font-size:8px;color:#666;letter-spacing:1px;margin-left:6px;text-transform:none">live + sweep</span></h2>
   <canvas id="polar" style="height:120px"></canvas>
   <div style="font-size:9px;color:#555;display:flex;justify-content:space-between;margin-top:2px">
    <span id="pol-xl">V&sup2; →</span>
    <span><span style="color:#4af">•</span> sweep <span style="color:#f90">•</span> live</span>
    <button onclick="clearPolar()" style="padding:1px 6px;font-size:9px;background:#222;color:#666;border:1px solid #333;border-radius:2px;cursor:pointer">clear</button>
   </div>
  </div>

  <!-- Auto experiment -->
  <div class="card">
   <h2>BALAYAGE AUTO</h2>
   <div id="exp-idle">
    <div style="display:grid;grid-template-columns:1fr 1fr;gap:4px;font-size:11px;margin-bottom:4px">
     <label>De <input id="exp-from" type="number" value="20" min="0" max="100" style="width:50px;background:#0a0a0a;color:#eee;border:1px solid #333;padding:2px 4px"></label>
     <label>A <input id="exp-to" type="number" value="80" min="0" max="100" style="width:50px;background:#0a0a0a;color:#eee;border:1px solid #333;padding:2px 4px"></label>
     <label>Pas <input id="exp-step" type="number" value="10" min="1" max="50" style="width:50px;background:#0a0a0a;color:#eee;border:1px solid #333;padding:2px 4px"></label>
     <label>Dwell <input id="exp-dwell" type="number" value="5" min="1" max="60" style="width:50px;background:#0a0a0a;color:#eee;border:1px solid #333;padding:2px 4px"></label>
    </div>
    <div class="btn-row">
     <button class="btn-tare" style="padding:6px;font-size:11px" onclick="startExp()">DEMARRER F(V&sup2;)</button>
    </div>
   </div>
   <div id="exp-run" style="display:none">
    <div style="font-size:11px;color:#4af;margin-bottom:3px" id="exp-msg">...</div>
    <div style="background:#222;height:4px;border-radius:2px;overflow:hidden;margin-bottom:5px">
     <div id="exp-bar" style="background:#4af;height:100%;width:0%;transition:width .3s"></div>
    </div>
    <div class="btn-row">
     <button onclick="stopExp()" style="background:#a11;color:#fff">STOP</button>
    </div>
   </div>
   <div id="exp-done" style="display:none;font-size:11px;color:#4f4;margin-top:5px">
    <div id="exp-result"></div>
    <button onclick="exportExp()" style="margin-top:4px;background:#1a4a99;color:#fff;padding:5px;width:100%">&#8595; Export CSV</button>
   </div>
  </div>

  <!-- Diagnostics systeme -->
  <div class="card">
   <h2>DIAGNOSTICS <span style="font-size:8px;color:#666;letter-spacing:1px;margin-left:6px;text-transform:none">live</span></h2>
   <div class="aero-grid">
    <div class="aero-item">CPU <b id="h-cpu">--°C</b></div>
    <div class="aero-item">RAM <b id="h-ram">--%</b></div>
    <div class="aero-item">FPS cam <b id="h-fps">--</b></div>
    <div class="aero-item">Uptime <b id="h-up">0s</b></div>
    <div class="aero-item">Buffer <b id="h-buf">0%</b></div>
    <div class="aero-item">Disk libre <b id="h-disk">-- GB</b></div>
   </div>
   <div style="display:flex;align-items:center;gap:6px;margin-top:5px;padding-top:5px;border-top:1px solid #222;font-size:11px">
    <label style="color:#888"><input type="checkbox" id="at-en" onchange="setAutoTare()"> Auto-tare</label>
    <span style="color:#555">déclenché <b id="at-cnt" style="color:#aaa">0</b>×</span>
   </div>
  </div>

  <!-- Mesure certifiee -->
  <div class="card">
   <h2>MESURE CERTIFIEE</h2>
   <div style="display:flex;gap:5px;align-items:center;margin-bottom:5px">
    <input id="meas-dur" type="number" value="10" min="2" max="60" style="width:50px;background:#0a0a0a;color:#eee;border:1px solid #333;padding:3px 5px;font-size:11px"> s
    <button class="btn-tare" style="padding:6px 10px;font-size:11px" onclick="doMeasure()">LANCER</button>
   </div>
   <div id="meas-status" style="font-size:10px;color:#666"></div>
   <pre id="meas-result" style="font-size:10px;color:#aaa;white-space:pre-wrap;margin-top:5px;max-height:120px;overflow:auto"></pre>
  </div>
 </div>
</div>

<div class="statusbar">
 <div class="sb-dot" id="sb-dot"></div>
 <span id="sb-st">init</span>
 <span class="sb-sep">|</span>
 <span id="sb-hz">-- Hz</span>
 <span class="sb-sep">|</span>
 <span>cam 30fps</span>
 <span class="sb-sep">|</span>
 <span id="sb-buf">0 pts</span>
 <span style="margin-left:auto;color:#333">? = raccourcis</span>
</div>

<div id="toasts"></div>

<div id="shortcuts">
 <div class="sc-box" onclick="event.stopPropagation()">
  <h3>RACCOURCIS CLAVIER</h3>
  <div class="sc-row"><span>Tare</span><span class="sc-key">T</span></div>
  <div class="sc-row"><span>Fan ON/OFF</span><span class="sc-key">Espace</span></div>
  <div class="sc-row"><span>Fan +10%</span><span class="sc-key">&#8593;</span></div>
  <div class="sc-row"><span>Fan &minus;10%</span><span class="sc-key">&#8595;</span></div>
  <div class="sc-row"><span>Fan +5%</span><span class="sc-key">+</span></div>
  <div class="sc-row"><span>Fan &minus;5%</span><span class="sc-key">&minus;</span></div>
  <div class="sc-row"><span>Export CSV</span><span class="sc-key">E</span></div>
  <div class="sc-row"><span>Photo snapshot</span><span class="sc-key">P</span></div>
  <div class="sc-row"><span>Rapport PDF</span><span class="sc-key">G</span></div>
  <div class="sc-row"><span>REC on/off</span><span class="sc-key">R</span></div>
  <div class="sc-row"><span>Fermer</span><span class="sc-key">Echap</span></div>
 </div>
</div>

<script>
// ---- state ----
let fanDuty=0,savedDuty=30,unit='N',recStart=0,recInt=null;
const C2PI=251.3; // 2*pi*r=2*pi*40

// ---- sparkline ----
const cvs=document.getElementById("spark"),ctx=cvs.getContext("2d");
const hist=[];const HIST=150;
function draw(){
 cvs.width=cvs.offsetWidth;cvs.height=50;
 const w=cvs.width,h=cvs.height;
 if(hist.length<2){ctx.clearRect(0,0,w,h);return;}
 const mx=Math.max(...hist.map(Math.abs),1e-9);
 // gradient fill
 const gr=ctx.createLinearGradient(0,0,0,h);
 gr.addColorStop(0,"rgba(68,255,68,.25)");gr.addColorStop(1,"rgba(68,255,68,0)");
 ctx.fillStyle=gr;
 ctx.beginPath();
 hist.forEach((v,i)=>{const x=(i/(HIST-1))*w,y=h/2-(v/mx)*(h/2-3);
  i?ctx.lineTo(x,y):ctx.moveTo(x,y)});
 ctx.lineTo(w,h/2);ctx.lineTo(0,h/2);ctx.closePath();ctx.fill();
 // zero line
 ctx.strokeStyle="#222";ctx.lineWidth=1;
 ctx.beginPath();ctx.moveTo(0,h/2);ctx.lineTo(w,h/2);ctx.stroke();
 // signal line
 ctx.strokeStyle="#4f4";ctx.lineWidth=1.5;
 ctx.beginPath();
 hist.forEach((v,i)=>{const x=(i/(HIST-1))*w,y=h/2-(v/mx)*(h/2-3);
  i?ctx.lineTo(x,y):ctx.moveTo(x,y)});
 ctx.stroke();
 // last point dot
 const lx=w,ly=h/2-(hist[hist.length-1]/mx)*(h/2-3);
 ctx.fillStyle="#4f4";ctx.beginPath();ctx.arc(lx-1,ly,2.5,0,Math.PI*2);ctx.fill();
}

// ---- gauge ----
function updateGauge(duty,rpm){
 const fill=(duty/100)*C2PI;
 const arc=document.getElementById("g-arc");
 arc.style.strokeDasharray=fill+" "+(C2PI-fill);
 const col=duty>70?"#f44":duty>40?"#f90":"#4f4";
 arc.style.stroke=col;
 document.getElementById("g-pct").style.fill=col;
 document.getElementById("g-pct").textContent=duty.toFixed(0)+"%";
 document.getElementById("g-rpm").textContent=rpm+" RPM";
 const t=document.getElementById("fan-tog");
 t.textContent=duty>0?"ON":"OFF";
 t.className="btn-tog"+(duty>0?"":" off");
}

// ---- unit toggle ----
function setUnit(u){
 unit=u;
 ["adu","g","N"].forEach(x=>{
  document.getElementById("u-"+x).className="u-btn"+(u===x?" on":"")});
}

function fmtVal(d){
 if(unit==="g") return (d.force_g>=0?"+":"")+d.force_g.toFixed(2)+" g";
 if(unit==="N") return (d.force_N>=0?"+":"")+d.force_N.toFixed(4)+" N";
 return (d.delta>=0?"+":"")+d.delta.toFixed(0)+" adu";
}
function isNeg(d){return unit==="g"?d.force_g<0:unit==="N"?d.force_N<0:d.delta<0;}
function histVal(d){return unit==="g"?d.force_g:unit==="N"?d.force_N:d.delta;}

// ---- SSE ----
const evs=new EventSource("/events");
let hz_count=0,hz_last=Date.now();
evs.onmessage=e=>{
 const d=JSON.parse(e.data);
 // force
 document.getElementById("f-val").textContent=fmtVal(d);
 document.getElementById("f-val").className="big-val"+(isNeg(d)?" neg":"");
 document.getElementById("f-sub").textContent=d.force_g.toFixed(3)+" g — "+d.force_N.toFixed(5)+" N";
 document.getElementById("f-mu").textContent=(+d.force_mean).toFixed(2);
 document.getElementById("f-sig").textContent=(+d.force_std).toFixed(2);
 document.getElementById("f-fft").textContent=d.peak_freq_hz.toFixed(1)+" Hz";
 hist.push(histVal(d));if(hist.length>HIST)hist.shift();draw();
 // fan
 fanDuty=d.fan_duty;
 updateGauge(d.fan_duty,d.fan_rpm);
 // aero
 document.getElementById("a-vf").textContent=d.airspeed_ms.toFixed(2)+" m/s";
 document.getElementById("a-vfn").textContent=(d.airspeed_fan_ms||0).toFixed(2)+" m/s";
 document.getElementById("a-vk").textContent=(d.airspeed_kalman_ms||0).toFixed(2)+" m/s";
 document.getElementById("a-vks").textContent="±"+(d.airspeed_kalman_sigma||0).toFixed(2);
 document.getElementById("a-re").textContent=Math.round(d.reynolds);
 // append au polar plot live (1x par seconde max)
 const now=Date.now();
 if(now-lastPolarTime>1000){
  const vk=d.airspeed_kalman_ms||d.airspeed_avg_ms||d.airspeed_ms;
  pushPolarLive(vk,d.force_N);
  lastPolarTime=now;
 }
 document.getElementById("a-st").textContent=(+d.strouhal).toFixed(3);
 document.getElementById("a-c").textContent=(+d.CL).toFixed(3);
 document.getElementById("a-fft").textContent=d.peak_freq_hz.toFixed(1)+" Hz";
 // cam hud
 document.getElementById("h-mode").textContent=d.cam_mode;
 document.getElementById("h-v").textContent=d.airspeed_ms.toFixed(2);
 document.getElementById("h-vf").textContent=(d.airspeed_fan_ms||0).toFixed(2);
 document.getElementById("h-re").textContent=Math.round(d.reynolds);
 document.getElementById("h-fl").textContent=(+d.flow_mag).toFixed(2);
 document.getElementById("mode-badge").textContent=d.cam_mode;
 // status bar
 const ok=d.status==="ok";
 document.getElementById("sb-dot").className="sb-dot"+(ok?"":" err");
 document.getElementById("sb-st").textContent=d.status;
 hz_count++;
 if(Date.now()-hz_last>1000){
  document.getElementById("sb-hz").textContent=hz_count*10+" Hz";
  hz_count=0;hz_last=Date.now();}
};

// ---- fan ----
async function fanSet(v){await fetch("/fan/set?duty="+Math.max(0,Math.min(100,v)),{method:"POST"})}
function fanStep(s){fanSet(fanDuty+s)}
function fanToggle(){if(fanDuty>0){savedDuty=fanDuty;fanSet(0)}else fanSet(savedDuty||30)}
let _fanVUnit="ms"; // "ms" or "pct"
function fanVToggleUnit(){
 _fanVUnit=_fanVUnit==="ms"?"pct":"ms";
 const btn=document.getElementById("fan-v-unit-btn");
 const inp=document.getElementById("fan-v-inp");
 if(_fanVUnit==="ms"){
  btn.textContent="m/s"; inp.max=20; inp.step=0.1; inp.placeholder="0.0";
 } else {
  btn.textContent="%"; inp.max=100; inp.step=0.1; inp.placeholder="0.0";
 }
 inp.value="";
}
async function fanSetV(){
 const val=parseFloat(document.getElementById("fan-v-inp").value);
 if(isNaN(val)||val<0) return;
 if(_fanVUnit==="ms"){
  const r=await fetch("/fan/set_v?v="+val.toFixed(2),{method:"POST"}).then(x=>x.json()).catch(()=>null);
  if(r&&r.duty!=null) fanDuty=r.duty;
 } else {
  await fanSet(val);
  fanDuty=Math.max(0,Math.min(100,val));
 }
}

// ---- tare ----
async function doTare(){
 const el=document.getElementById("tare-st");el.textContent="tare...";
 hist.length=0;
 const r=await (await fetch("/tare",{method:"POST"})).json();
 el.textContent="zero OK (offset="+r.offset+")";
 toast("Tare OK","ok");
 setTimeout(()=>el.textContent="",2500);
}

// ---- REC ----
async function toggleRec(){
 const r=await (await fetch("/record/toggle",{method:"POST"})).json();
 const btn=document.getElementById("rec-btn");
 const timer=document.getElementById("rec-timer");
 if(r.recording){
  recStart=Date.now();btn.classList.add("on");
  recInt=setInterval(()=>{
   const s=Math.floor((Date.now()-recStart)/1000);
   timer.textContent=String(Math.floor(s/60)).padStart(2,"0")+":"+String(s%60).padStart(2,"0");
  },500);
  toast("REC: "+(r.video?r.video.split("/").pop():"started"),"ok");
 } else {
  btn.classList.remove("on");clearInterval(recInt);timer.textContent="";
  if(r.err){toast("Err: "+r.err,"err");return;}
  const dur=r.duration_s||0;
  toast("REC sauve: "+dur.toFixed(1)+"s, "+r.samples+" pts","ok");
 }
}

// ---- export ----
async function doExport(){
 toast("Export CSV...");
 window.location.href="/export/csv";
}
async function doSnapshot(){
 toast("Snapshot...");
 window.location.href="/export/snapshot";
}
async function doPDF(){
 toast("Generation PDF (peut prendre 3-5s)...");
 window.location.href="/export/pdf";
}

// ---- toast ----
function toast(msg,type){
 const div=document.createElement("div");
 div.className="toast"+(type?" "+type:"");
 div.textContent=msg;
 document.getElementById("toasts").appendChild(div);
 setTimeout(()=>div.remove(),2200);
}

// ---- shortcuts ----
function showSC(){document.getElementById("shortcuts").classList.add("show")}
document.getElementById("shortcuts").onclick=()=>document.getElementById("shortcuts").classList.remove("show");
document.addEventListener("keydown",e=>{
 if(e.target.tagName==="INPUT"||e.target.tagName==="SELECT")return;
 if(e.key==="Escape")document.getElementById("shortcuts").classList.remove("show");
 if(e.key==="?")showSC();
 if(e.key==="t"||e.key==="T")doTare();
 if(e.key===" "){e.preventDefault();fanToggle();}
 if(e.key==="ArrowUp"){e.preventDefault();fanStep(10);}
 if(e.key==="ArrowDown"){e.preventDefault();fanStep(-10);}
 if(e.key==="+"||e.key==="=")fanStep(5);
 if(e.key==="-")fanStep(-5);
 if(e.key==="e"||e.key==="E")doExport();
 if(e.key==="p"||e.key==="P")doSnapshot();
 if(e.key==="g"||e.key==="G")doPDF();
 if(e.key==="r"||e.key==="R")toggleRec();
});

// status bar buffer count
setInterval(async()=>{
 const r=await fetch("/export/count").catch(()=>null);
 if(r&&r.ok){const j=await r.json();document.getElementById("sb-buf").textContent=j.n+" pts";}
},2000);

// ---- DIAGNOSTICS ----
async function updateHealth(){
 const r=await fetch("/health").catch(()=>null);
 if(!r||!r.ok)return;
 const h=await r.json();
 const fmt=(v,u,d)=>v==null?"--":(+v).toFixed(d===undefined?1:d)+u;
 document.getElementById("h-cpu").textContent=fmt(h.cpu_temp_c,"°C");
 const cpu=document.getElementById("h-cpu");
 cpu.style.color=h.cpu_temp_c>75?"#f44":h.cpu_temp_c>65?"#f90":"#4f4";
 document.getElementById("h-ram").textContent=fmt(h.mem_used_pct,"%");
 document.getElementById("h-fps").textContent=fmt(h.cam_fps,"");
 const up=h.uptime_s;
 const hh=Math.floor(up/3600),mm=Math.floor((up%3600)/60),ss=Math.floor(up%60);
 document.getElementById("h-up").textContent=(hh>0?hh+"h":"")+mm+"m"+ss+"s";
 document.getElementById("h-buf").textContent=fmt(h.data_buffer.pct,"%");
 document.getElementById("h-disk").textContent=fmt(h.disk_free_gb," GB");
 if(h.auto_tare_enabled!==undefined)document.getElementById("at-en").checked=h.auto_tare_enabled;
 if(h.auto_tare_count!==undefined)document.getElementById("at-cnt").textContent=h.auto_tare_count;
}
setInterval(updateHealth,2000);
updateHealth();

async function setAutoTare(){
 const en=document.getElementById("at-en").checked?1:0;
 await fetch("/auto_tare?enable="+en,{method:"POST"});
 toast("Auto-tare "+(en?"ON":"OFF"),"ok");
}

// ---- SPECTROGRAMME WATERFALL ----
const spc=document.getElementById("spectro"),spx=spc.getContext("2d");
async function drawSpectro(){
 const r=await fetch("/spectrogram").catch(()=>null);
 if(!r||!r.ok)return;
 const d=await r.json();
 spc.width=spc.offsetWidth;spc.height=80;
 const w=spc.width,h=spc.height;
 spx.fillStyle="#0a0a0a";spx.fillRect(0,0,w,h);
 if(!d.cols||d.cols.length===0)return;
 const N=d.cols.length,B=d.n_bins;
 let vmax=1e-9;
 for(const c of d.cols)for(const v of c)if(v>vmax)vmax=v;
 const cw=w/d.n_cols,bh=h/B;
 for(let i=0;i<N;i++){
  const col=d.cols[i];
  const xi=w-(N-i)*cw;
  for(let j=0;j<col.length;j++){
   const t=Math.min(1,Math.log10(1+9*col[j]/vmax));
   const rr=Math.round(255*Math.max(0,Math.min(1,1.5*t-0.5)));
   const gg=Math.round(255*Math.max(0,Math.sin(Math.PI*t)));
   const bb=Math.round(255*Math.max(0,Math.min(1,1.5-2*t)));
   spx.fillStyle="rgb("+rr+","+gg+","+bb+")";
   const yi=h-(j+1)*bh;
   spx.fillRect(xi,yi,Math.ceil(cw)+1,Math.ceil(bh)+1);
  }
 }
 document.getElementById("spectro-fmax").textContent=d.fmax.toFixed(1)+" Hz";
}
setInterval(drawSpectro,500);

// ---- EXPERIENCE AUTO ----
async function startExp(){
 const f=+document.getElementById("exp-from").value;
 const t=+document.getElementById("exp-to").value;
 const s=+document.getElementById("exp-step").value;
 const dw=+document.getElementById("exp-dwell").value;
 const r=await (await fetch("/experiment/start?from="+f+"&to="+t+"&step="+s+"&dwell="+dw,{method:"POST"})).json();
 if(r.ok){toast("Experience demarree","ok");} else {toast("Err: "+r.err,"err");}
}
async function stopExp(){await fetch("/experiment/stop",{method:"POST"});toast("Stop","err");}
async function exportExp(){window.location.href="/experiment/export";}
async function pollExp(){
 const r=await fetch("/experiment/status").catch(()=>null);
 if(!r||!r.ok)return;
 const s=await r.json();
 if(s.running){
  document.getElementById("exp-idle").style.display="none";
  document.getElementById("exp-run").style.display="block";
  document.getElementById("exp-done").style.display="none";
  document.getElementById("exp-msg").textContent=s.msg;
  document.getElementById("exp-bar").style.width=s.progress+"%";
 } else if(s.results&&s.results.length>0){
  document.getElementById("exp-idle").style.display="block";
  document.getElementById("exp-run").style.display="none";
  document.getElementById("exp-done").style.display="block";
  const last=s.results[s.results.length-1];
  document.getElementById("exp-result").textContent=s.results.length+" points - dernier: "+last.duty+"% V="+last.V_ms+" F="+last.F_N+"N";
  // populate sweep series du polar plot
  polarSweep.length=0;
  for(const r of s.results){if(r.V_ms>0)polarSweep.push({x:r.V_ms*r.V_ms,y:r.F_N});}
 } else {
  document.getElementById("exp-idle").style.display="block";
  document.getElementById("exp-run").style.display="none";
  document.getElementById("exp-done").style.display="none";
 }
}
setInterval(pollExp,1000);

// ---- POLAIRE F vs V^2 ----
const pol=document.getElementById("polar"),polx=pol.getContext("2d");
const polarLive=[];const polarSweep=[];const POL_MAX=400;
let lastPolarTime=0;
function pushPolarLive(v,f){
 if(v<0.2)return;
 polarLive.push({x:v*v,y:f});
 if(polarLive.length>POL_MAX)polarLive.shift();
}
function drawPolar(){
 pol.width=pol.offsetWidth;pol.height=120;
 const w=pol.width,h=pol.height;
 polx.fillStyle="#0a0a0a";polx.fillRect(0,0,w,h);
 const all=polarLive.concat(polarSweep);
 if(all.length<2)return;
 const xmax=Math.max(...all.map(p=>p.x),0.1)*1.1;
 const ymin=Math.min(...all.map(p=>p.y),0);
 const ymax=Math.max(...all.map(p=>p.y),1e-6)*1.1;
 const xrange=xmax||1,yrange=(ymax-ymin)||1;
 const mx=w-8,my=h-12,ox=4,oy=4;
 // grid
 polx.strokeStyle="#1a1a1a";polx.lineWidth=1;
 for(let i=1;i<5;i++){
  polx.beginPath();polx.moveTo(ox+i*mx/5,oy);polx.lineTo(ox+i*mx/5,oy+my);polx.stroke();
  polx.beginPath();polx.moveTo(ox,oy+i*my/5);polx.lineTo(ox+mx,oy+i*my/5);polx.stroke();
 }
 // zero line (y=0)
 if(ymin<0){
  const yz=oy+my*(1-(0-ymin)/yrange);
  polx.strokeStyle="#333";
  polx.beginPath();polx.moveTo(ox,yz);polx.lineTo(ox+mx,yz);polx.stroke();
 }
 // live (orange)
 polx.fillStyle="#f90";
 for(const p of polarLive){
  const x=ox+mx*p.x/xrange;
  const y=oy+my*(1-(p.y-ymin)/yrange);
  polx.fillRect(x-1,y-1,2,2);
 }
 // sweep (bleu)
 polx.fillStyle="#4af";
 polx.strokeStyle="#4af";polx.lineWidth=2;
 polx.beginPath();
 polarSweep.forEach((p,i)=>{
  const x=ox+mx*p.x/xrange;
  const y=oy+my*(1-(p.y-ymin)/yrange);
  if(i===0)polx.moveTo(x,y);else polx.lineTo(x,y);
 });
 polx.stroke();
 for(const p of polarSweep){
  const x=ox+mx*p.x/xrange;
  const y=oy+my*(1-(p.y-ymin)/yrange);
  polx.beginPath();polx.arc(x,y,3,0,Math.PI*2);polx.fill();
 }
 // axes labels
 polx.fillStyle="#444";polx.font="9px monospace";
 polx.fillText("V²="+xmax.toFixed(1),ox+mx-50,h-2);
 polx.fillText("F="+ymax.toFixed(3)+"N",ox+2,oy+8);
 polx.fillText(ymin.toFixed(3),ox+2,oy+my-2);
}
function clearPolar(){polarLive.length=0;polarSweep.length=0;drawPolar();toast("Polar cleared","ok");}
setInterval(drawPolar,500);

// ---- MESURE CERTIFIEE ----
async function doMeasure(){
 const dur=+document.getElementById("meas-dur").value;
 document.getElementById("meas-status").textContent="acquisition ("+dur+"s)...";
 document.getElementById("meas-result").textContent="";
 const r=await (await fetch("/measure?dur="+dur)).json();
 document.getElementById("meas-status").textContent="OK ("+r.n_samples+" echantillons)";
 const fmt=function(v,d){if(d===undefined)d=4;return (+v).toFixed(d);};
 const txt=
  "F   = "+fmt(r.F_mean_N)+" N\\n"+
  "    CI95 = ["+fmt(r.F_ci95[0])+", "+fmt(r.F_ci95[1])+"] N\\n"+
  "V   = "+fmt(r.V_mean_ms,3)+" m/s\\n"+
  "Re  = "+r.Re+"\\n"+
  "St  = "+fmt(r.St,3)+"\\n"+
  "C   = "+fmt(r.CL_or_CD,3)+"\\n"+
  "    CI95 = ["+fmt(r.CL_or_CD_ci95[0],3)+", "+fmt(r.CL_or_CD_ci95[1],3)+"]\\n"+
  "FFT = "+fmt(r.FFT_peak_Hz,2)+" Hz\\n"+
  "alpha="+r.aoa_deg+" corde="+r.chord_m+"m";
 document.getElementById("meas-result").textContent=txt.replace(/\\\\n/g,"\\n");
 toast("Mesure certifiee OK","ok");
}
</script></body></html>"""


SETTINGS_HTML = r"""<!DOCTYPE html>
<html lang="fr"><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Parametres - Soufflerie</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#0d0d0d;color:#eee;font-family:ui-monospace,Menlo,monospace;padding:14px;max-width:900px;margin:auto}
a.back{color:#4af;text-decoration:none;font-size:12px}
h1{color:#4af;font-size:16px;letter-spacing:2px;text-transform:uppercase;margin:10px 0 16px}
.tabs{display:flex;gap:4px;border-bottom:1px solid #333;margin-bottom:12px;flex-wrap:wrap}
.tabs button{background:transparent;color:#888;border:none;padding:8px 12px;cursor:pointer;font-family:inherit;font-size:12px;letter-spacing:1px;text-transform:uppercase;border-bottom:2px solid transparent}
.tabs button.on{color:#4af;border-bottom-color:#4af}
.panel{display:none}.panel.on{display:block}
.card{background:#171717;border:1px solid #2a2a2a;border-radius:6px;padding:14px;margin-bottom:10px}
h2{color:#4af;font-size:11px;letter-spacing:2px;text-transform:uppercase;margin-bottom:8px}
.row{display:flex;justify-content:space-between;align-items:center;padding:4px 0;font-size:13px;gap:10px}
.row .lab{color:#888;flex:1}
.row .val{color:#4af;font-weight:bold}
.row input[type=number]{width:110px;background:#0d0d0d;color:#eee;border:1px solid #333;border-radius:3px;padding:4px 6px;font-family:inherit}
input[type=range]{width:100%;accent-color:#4af}
button{padding:7px 12px;background:#2a2a2a;color:#fff;border:none;border-radius:3px;cursor:pointer;font-family:inherit;font-size:12px;font-weight:bold}
button:active{opacity:.7}
button.primary{background:#1a4a99}
button.danger{background:#a11}
select{background:#0d0d0d;color:#eee;border:1px solid #333;border-radius:3px;padding:4px 6px;font-family:inherit}
.hex{font-family:monospace;color:#fa4;font-size:14px;letter-spacing:2px}
.led{display:inline-block;width:10px;height:10px;border-radius:50%;background:#333}.led.on{background:#4f4;box-shadow:0 0 6px #4f4}
.note{font-size:11px;color:#666;margin-top:6px;line-height:1.5}
.btn-sm{padding:4px 10px;background:#0d1520;color:#4af;border:1px solid #1a4a70;border-radius:3px;cursor:pointer;font-family:inherit;font-size:11px}
.grid2{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.status{font-size:10px;color:#4f4;min-height:12px;margin-top:4px}
.hsv{display:grid;grid-template-columns:60px 1fr 40px;gap:6px;align-items:center;font-size:11px}
</style></head><body>
<a class="back" href="/">&larr; retour</a>
<h1>Parametres &amp; calibration</h1>

<div class="tabs">
 <button class="on" data-p="p-force">Force</button>
 <button data-p="p-fan">Ventilo</button>
 <button data-p="p-led">LED</button>
 <button data-p="p-cam">Camera</button>
 <button data-p="p-aero">Aero</button>
 <button data-p="p-dbg">Debug</button>
 <button data-p="p-info">Info</button>
 <button data-p="p-wifi">WiFi</button>
</div>

<!-- FORCE -->
<div id="p-force" class="panel on">
 <div class="card">
  <h2>Calibration cellule HX711</h2>
  <div class="row"><span class="lab">Raw brut</span><b class="val" id="s-raw">0</b></div>
  <div class="row"><span class="lab">Delta (raw - offset)</span><b class="val" id="s-delta">0</b></div>
  <div class="row"><span class="lab">Offset (tare)</span><b class="val" id="s-off">0</b></div>
  <div class="row"><span class="lab">Scale (ADU / gramme)</span><b class="val" id="s-sc">1.0</b></div>
  <div class="row">
   <button class="primary" onclick="doTare()">TARER (zero)</button>
   <span style="flex:1"></span>
   Masse etalon: <input id="mref" type="number" step="0.1" value="100"> g
   <button onclick="doCalib()">Calibrer</button>
  </div>
  <div class="status" id="cal-st"></div>
  <div class="note">1) retirer la charge, cliquer TARER. 2) poser une masse connue (ex 100g),
   taper sa valeur et cliquer Calibrer. Le gain sera sauvegarde dans scale.</div>
 </div>

 <div class="card">
  <h2>Filtre force</h2>
  <div class="row"><span class="lab">Fenetre moyenne glissante</span>
   <input id="fw" type="number" min="1" max="200" value="10" onchange="setFilter()"> echantillons</div>
  <div class="note">Force brute HX711 a 20 Hz. 10 = ~0.5s de lissage.
   Plus grand = moins de bruit mais reponse lente.</div>
 </div>

 <div class="card">
  <h2>Stats temps reel</h2>
  <div class="row"><span class="lab">Force filtree</span><b class="val" id="s-fN">0 N</b></div>
  <div class="row"><span class="lab">Moyenne (buffer)</span><b class="val" id="s-fm">0</b></div>
  <div class="row"><span class="lab">Ecart-type</span><b class="val" id="s-fstd">0</b></div>
  <div class="row"><span class="lab">RMS</span><b class="val" id="s-frms">0</b></div>
  <div class="row"><span class="lab">Pic FFT (lacher tourbillonnaire?)</span><b class="val" id="s-fft">0 Hz</b></div>
 </div>

 <div class="card">
  <h2>Tare trainee support (aero)</h2>
  <div class="row">
   <span class="lab">Trainee support actuelle</span>
   <b class="val" id="s-drag">0.000 N</b>
   <label style="display:flex;align-items:center;gap:6px;cursor:pointer">
    <input type="checkbox" id="drag-en" onchange="setDragEnable()" style="accent-color:#4af">
    <span style="font-size:11px;color:#4af">Soustraire</span>
   </label>
  </div>
  <div class="row">
   <span class="lab">Duty ventilo pour tare</span>
   <input id="drag-duty" type="number" min="0" max="100" value="50" style="width:70px"> %
  </div>
  <button class="primary" onclick="doTareSupport()" id="btn-drag-tare">TARER LE SUPPORT</button>
  <div class="status" id="drag-st"></div>
  <div class="note">Support monte SANS profil. Le ventilateur demarre automatiquement, mesure la trainee parasite, s'arrete. Activer "Soustraire" pour retrancher cette valeur de la force brute.</div>
 </div>

 <div class="card">
  <button class="primary" onclick="doSave()">Sauvegarder offset + scale dans config.py</button>
  <div class="status" id="save-st"></div>
 </div>
</div>

<!-- FAN -->
<div id="p-fan" class="panel">
 <div class="card">
  <h2>Sensibilite encodeur Modulino</h2>
  <div class="row"><span class="lab">% par cran</span><b class="val" id="es">1.0</b></div>
  <input id="erng" type="range" min="0.1" max="10" step="0.1" value="1" oninput="setStep(+this.value)">
  <div class="status" id="es-st"></div>
  <div class="note">Augmente si tu veux moins tourner. Diminue pour plus de precision.</div>
 </div>
</div>

<!-- LED -->
<div id="p-led" class="panel">
 <div class="card">
  <h2>Ruban LED <span id="led-status-badge" style="font-size:9px;padding:2px 6px;border-radius:3px;background:#1a3a00;color:#4f4;margin-left:6px">OK</span></h2>
  <div class="row">
   <span class="lab">Marche</span>
   <label style="display:flex;align-items:center;gap:8px;cursor:pointer">
    <input type="checkbox" id="led-on" onchange="ledToggle()" checked style="width:16px;height:16px;accent-color:#4af">
    <span id="led-on-lbl" style="color:#4af;font-size:11px">ON</span>
   </label>
  </div>
  <div class="row">
   <span class="lab">Couleur</span>
   <input type="color" id="led-color" value="#0078ff" oninput="ledApply()"
    style="width:48px;height:28px;border:none;background:none;cursor:pointer;padding:0">
  </div>
  <div class="row">
   <span class="lab">Luminosite</span>
   <input type="range" id="led-bri" min="0" max="255" value="80" oninput="ledApply();document.getElementById('led-bri-val').textContent=this.value" style="flex:1">
   <b class="val" id="led-bri-val" style="min-width:28px;text-align:right">80</b>
  </div>
  <div class="row">
   <span class="lab">Vitesse</span>
   <input type="range" id="led-spd" min="1" max="100" value="50" oninput="ledApply();document.getElementById('led-spd-val').textContent=this.value" style="flex:1">
   <b class="val" id="led-spd-val" style="min-width:28px;text-align:right">50</b>
  </div>
  <div class="row">
   <span class="lab">Animation</span>
   <select id="led-anim" onchange="ledApply()" style="flex:1;background:#0d0d0d;color:#eee;border:1px solid #333;border-radius:3px;padding:4px">
    <option value="solid">Solide</option>
    <option value="pulse">Pulse (respiration)</option>
    <option value="rainbow">Arc-en-ciel</option>
    <option value="strobe">Stroboscope</option>
    <option value="wipe">Color wipe</option>
    <option value="exposition">Exposition ENSAM (bleu/or/blanc)</option>
    <option value="boite_nuit">🪩 Boîte de nuit</option>
    <option value="off">Eteint</option>
   </select>
  </div>
  <div class="row">
   <span class="lab">Nb LEDs</span>
   <input type="number" id="led-n" value="30" min="1" max="300"
    style="width:70px;background:#0d0d0d;color:#eee;border:1px solid #333;border-radius:3px;padding:4px"
    onchange="ledApply()">
  </div>
  <div id="led-err" class="note" style="color:#f55;display:none;margin-top:4px"></div>
  <div class="note" style="margin-top:6px">GPIO10 / MOSI SPI (pin 19) → DATA. Alim 5V externe sur VCC strip. GND commun avec Pi.</div>
 </div>

 <!-- Palette rapide -->
 <div class="card">
  <h2>Couleurs rapides</h2>
  <div style="display:flex;flex-wrap:wrap;gap:6px">
   <button onclick="ledColor(255,0,0)"    style="background:#f00;color:#fff;padding:6px 12px;border:none;border-radius:3px;cursor:pointer">Rouge</button>
   <button onclick="ledColor(0,255,0)"    style="background:#0f0;color:#000;padding:6px 12px;border:none;border-radius:3px;cursor:pointer">Vert</button>
   <button onclick="ledColor(0,0,255)"    style="background:#00f;color:#fff;padding:6px 12px;border:none;border-radius:3px;cursor:pointer">Bleu</button>
   <button onclick="ledColor(255,255,0)"  style="background:#ff0;color:#000;padding:6px 12px;border:none;border-radius:3px;cursor:pointer">Jaune</button>
   <button onclick="ledColor(255,128,0)"  style="background:#f80;color:#fff;padding:6px 12px;border:none;border-radius:3px;cursor:pointer">Orange</button>
   <button onclick="ledColor(128,0,255)"  style="background:#80f;color:#fff;padding:6px 12px;border:none;border-radius:3px;cursor:pointer">Violet</button>
   <button onclick="ledColor(0,255,255)"  style="background:#0ff;color:#000;padding:6px 12px;border:none;border-radius:3px;cursor:pointer">Cyan</button>
   <button onclick="ledColor(255,255,255)"style="background:#fff;color:#000;padding:6px 12px;border:none;border-radius:3px;cursor:pointer">Blanc</button>
   <button onclick="ledExposition()"      style="background:linear-gradient(90deg,#94003c78,#9444cc,#e66e00,#e6e6ff);color:#fff;padding:6px 12px;border:none;border-radius:3px;cursor:pointer;font-weight:bold">ENSAM</button>
  </div>
 </div>
</div>

<!-- CAMERA -->
<div id="p-cam" class="panel">
 <div class="card" style="padding:0;overflow:hidden;background:#000;max-width:640px">
  <img src="/camera" style="width:100%;display:block" alt="camera live">
 </div>
 <div class="card" style="display:flex;justify-content:space-between;align-items:center">
  <span style="font-size:12px;color:#666">Remet tous les parametres camera aux valeurs par defaut</span>
  <button class="danger" onclick="camReset()" style="white-space:nowrap;margin-left:12px">&#8635; Reset camera</button>
 </div>
 <div class="card">
  <h2>Mode affichage</h2>
  <select id="cm" onchange="setMode(this.value)">
   <option value="raw">raw (brut)</option>
   <option value="contrast">CLAHE (contraste local, fumee visible)</option>
   <option value="arrows">Fleches (flux optique)</option>
   <option value="heatmap">Heatmap (couleur = direction, brillance = vitesse)</option>
   <option value="vorticity">Vorticite omega (rouge=CCW, bleu=CW, tourbillons)</option>
   <option value="streamlines">Streamlines (lignes de courant integrees)</option>
   <option value="bgsub">Soustraction fond (fumee isolee)</option>
  </select>
  <div class="note">Le flux optique (Farneback) calcule le champ de vitesse entre 2 images.
   Heatmap: teinte = sens (rouge/cyan = horizontal, vert/violet = vertical).
   Soustraction fond: apprentissage MOG2, isole ce qui bouge (fumee).</div>
 </div>

 <div class="card">
  <h2>Couleurs &amp; exposition</h2>
  <div class="row"><span class="lab">Brightness</span>
   <input type="range" min="-1" max="1" step="0.05" value="0" oninput="setCtl('brightness',+this.value)">
   <b class="val" id="cb">0.00</b></div>
  <div class="row"><span class="lab">Contrast</span>
   <input type="range" min="0" max="4" step="0.1" value="1" oninput="setCtl('contrast',+this.value)">
   <b class="val" id="cc">1.0</b></div>
  <div class="row"><span class="lab">Saturation</span>
   <input type="range" min="0" max="4" step="0.1" value="1" oninput="setCtl('saturation',+this.value)">
   <b class="val" id="csa">1.0</b></div>
  <div class="row"><span class="lab">Auto-Exposure</span>
   <label><input type="checkbox" id="ae" checked onchange="setCtl('ae',this.checked?1:0)"></label></div>
  <div class="row"><span class="lab">Auto-WhiteBalance</span>
   <label><input type="checkbox" id="awb" checked onchange="setCtl('awb',this.checked?1:0);toggleColourGains()"></label></div>
  <div id="colour-gains-row" style="display:none">
   <div class="row"><span class="lab">Gain Rouge (R)</span>
    <input type="range" id="cgr" min="0.1" max="8" step="0.05" value="2.0" oninput="setColourGain()">
    <b class="val" id="cgr-v">2.0</b></div>
   <div class="row"><span class="lab">Gain Bleu (B)</span>
    <input type="range" id="cgb" min="0.1" max="8" step="0.05" value="1.5" oninput="setColourGain()">
    <b class="val" id="cgb-v">1.5</b></div>
   <div style="display:flex;gap:6px;margin-top:6px">
    <button onclick="setColourPreset(1.0,1.0)" class="btn-sm">Neutre (1,1)</button>
    <button onclick="setColourPreset(2.0,1.5)" class="btn-sm">Auto (2,1.5)</button>
    <button onclick="setColourPreset(1.5,1.8)" class="btn-sm">Chaud</button>
   </div>
   <div class="note">Gains actifs uniquement si AWB desactive. R>1 = plus chaud, B>1 = plus froid.<br>Pour couleurs fideles : desactiver AWB + ajuster R/B en visant gris neutre.</div>
  </div>
 </div>

 <div class="card">
  <h2>Detection fumee (flux optique)</h2>
  <div class="row"><span class="lab">Amplification fumee</span>
   <label style="display:flex;align-items:center;gap:6px"><input type="checkbox" id="smoke-enh" checked onchange="setSmokeEnhance(this.checked)">
   <span style="font-size:11px;color:#4af">CLAHE + bilateral + normalisation</span></label></div>
  <div class="note">Ameliore la detection des filets de fumee peu contrastes. Desactiver si artefacts sur fond tres texture.</div>
 </div>

 <div class="card">
  <h2>Masque HSV (isoler fumee blanche)</h2>
  <div class="row"><span class="lab">Activer masque</span>
   <label><input type="checkbox" id="mk" onchange="setMask(this.checked?1:0)"></label></div>
  <div class="hsv"><span>H low</span><input type="range" min="0" max="180" value="0" oninput="setHSV('h_lo',+this.value)"><b class="val" id="h_lo">0</b></div>
  <div class="hsv"><span>H high</span><input type="range" min="0" max="180" value="180" oninput="setHSV('h_hi',+this.value)"><b class="val" id="h_hi">180</b></div>
  <div class="hsv"><span>S low</span><input type="range" min="0" max="255" value="0" oninput="setHSV('s_lo',+this.value)"><b class="val" id="s_lo">0</b></div>
  <div class="hsv"><span>S high</span><input type="range" min="0" max="255" value="60" oninput="setHSV('s_hi',+this.value)"><b class="val" id="s_hi">60</b></div>
  <div class="hsv"><span>V low</span><input type="range" min="0" max="255" value="180" oninput="setHSV('v_lo',+this.value)"><b class="val" id="v_lo">180</b></div>
  <div class="hsv"><span>V high</span><input type="range" min="0" max="255" value="255" oninput="setHSV('v_hi',+this.value)"><b class="val" id="v_hi">255</b></div>
  <div class="note">Fumee blanche = V haut, S bas. Defaut OK pour fumee sur fond sombre.</div>
 </div>

 <div class="card">
  <h2>ROI (region d'interet flux optique)</h2>
  <div class="note">Limite le calcul de vitesse a une zone (ex: derriere le profil pour le sillage).
   Valeurs normalisees 0..1 (0=gauche/haut, 1=droite/bas).</div>
  <div class="row"><span class="lab">X</span><input id="rx" type="number" step="0.05" min="0" max="1" value="0"></div>
  <div class="row"><span class="lab">Y</span><input id="ry" type="number" step="0.05" min="0" max="1" value="0"></div>
  <div class="row"><span class="lab">Largeur</span><input id="rw" type="number" step="0.05" min="0.05" max="1" value="1"></div>
  <div class="row"><span class="lab">Hauteur</span><input id="rh" type="number" step="0.05" min="0.05" max="1" value="1"></div>
  <div class="row">
   <button onclick="setROI()">Appliquer</button>
   <button onclick="clearROI()">Annuler ROI</button>
  </div>
 </div>

 <div class="card">
  <h2>Calibration px/metre (pour airspeed)</h2>
  <div class="row"><span class="lab">px / m</span>
   <input id="pxm" type="number" min="1" value="1000" onchange="setPxm(+this.value)"></div>
  <div class="row"><span class="lab">Distance cam-objet [m]</span>
   <input id="cam-dist" type="number" min="0.05" max="20" step="0.05" value="0.5"
    onchange="fetch('/cam/dist?v='+this.value)" style="width:80px"></div>
  <div class="note">Methode 1 (manuelle): place une regle dans la scene, mesure combien de pixels
   fait 1 m reel (ex: 10 cm = 300 px -> 3000 px/m).</div>
  <hr style="border:none;border-top:1px solid #333;margin:10px 0">
  <h2 style="font-size:11px">Methode 2: auto via damier OpenCV</h2>
  <div class="row">
   <span class="lab">Damier (coins int.)</span>
   <input id="cbc" type="number" value="9" style="width:50px"> x
   <input id="cbr" type="number" value="6" style="width:50px">
  </div>
  <div class="row">
   <span class="lab">Carre [mm]</span>
   <input id="cbsq" type="number" value="20" step="0.1" style="width:70px">
  </div>
  <button onclick="calibChessboard()">Detecter et calibrer</button>
  <div id="cb-st" class="status"></div>
  <div class="note">Imprime un damier (echiquier) sur papier, place-le dans le plan de symetrie
   de la veine, clique. OpenCV detecte les coins et calcule px/m precisement.</div>
 </div>

 <div class="card">
  <h2>Caracterisation bruit HX711</h2>
  <div class="row">
   <span class="lab">Duree [s]</span>
   <input id="nt-dur" type="number" value="10" min="2" max="60" style="width:70px">
   <button onclick="noiseTest()">Lancer (cellule au repos)</button>
   <button onclick="allanPlot()" style="background:#1a4a99;color:#fff">Allan multi-tau</button>
  </div>
  <div id="nt-st" class="status"></div>
  <pre id="nt-result" style="font-size:10px;color:#aaa;white-space:pre-wrap;margin-top:5px"></pre>
  <canvas id="allan-cv" style="width:100%;height:160px;display:none;background:#0a0a0a;margin-top:8px;border-radius:3px"></canvas>
  <div class="note">Allan deviation σ_A(τ) en log-log. Pente −½ = bruit blanc (ideal).
   Plat = instabilite de biais. +½ = marche aleatoire / derive. Le minimum = τ optimal d integration.</div>
 </div>
</div>

<!-- AERO -->
<div id="p-aero" class="panel">
 <div class="card">
  <h2>Type d objet</h2>
  <div style="display:grid;grid-template-columns:repeat(5,1fr);gap:6px;margin-bottom:12px">
   <div class="obj-card" id="oc-airfoil" onclick="setObjType('airfoil')" style="background:#0d0d0d;border:2px solid #1a4a99;border-radius:5px;padding:8px 4px;text-align:center;cursor:pointer">
    <svg viewBox="0 0 60 30" width="100%" height="30"><path d="M5,15 Q20,3 40,14 Q50,18 55,15 Q50,23 40,17 Q20,28 5,15Z" fill="#4af" opacity=".7"/></svg>
    <div style="font-size:9px;color:#4af;margin-top:3px">PROFIL</div>
   </div>
   <div class="obj-card" id="oc-plate" onclick="setObjType('plate')" style="background:#0d0d0d;border:2px solid #333;border-radius:5px;padding:8px 4px;text-align:center;cursor:pointer">
    <svg viewBox="0 0 60 30" width="100%" height="30"><rect x="8" y="9" width="44" height="12" rx="1" fill="#4af" opacity=".6"/><line x1="8" y1="15" x2="52" y2="15" stroke="#0d0d0d" stroke-width="1" stroke-dasharray="4,2"/></svg>
    <div style="font-size:9px;color:#888;margin-top:3px">PLAQUE</div>
   </div>
   <div class="obj-card" id="oc-cube" onclick="setObjType('cube')" style="background:#0d0d0d;border:2px solid #333;border-radius:5px;padding:8px 4px;text-align:center;cursor:pointer">
    <svg viewBox="0 0 60 40" width="100%" height="30"><path d="M15,22 L30,14 L45,22 L45,34 L30,42 L15,34Z" fill="#4af" opacity=".3"/><path d="M15,22 L30,30 L30,42 L15,34Z" fill="#4af" opacity=".5"/><path d="M30,14 L45,22 L45,34 L30,42 L30,30Z" fill="#4af" opacity=".4"/><path d="M15,22 L30,30 L45,22 L30,14Z" fill="#4af" opacity=".7"/></svg>
    <div style="font-size:9px;color:#888;margin-top:3px">CUBE</div>
   </div>
   <div class="obj-card" id="oc-sphere" onclick="setObjType('sphere')" style="background:#0d0d0d;border:2px solid #333;border-radius:5px;padding:8px 4px;text-align:center;cursor:pointer">
    <svg viewBox="0 0 60 40" width="100%" height="30"><circle cx="30" cy="20" r="16" fill="#4af" opacity=".4"/><ellipse cx="30" cy="20" rx="16" ry="6" stroke="#4af" stroke-width="1" fill="none" opacity=".6"/><ellipse cx="30" cy="20" rx="6" ry="16" stroke="#4af" stroke-width="1" fill="none" opacity=".4"/></svg>
    <div style="font-size:9px;color:#888;margin-top:3px">SPHERE</div>
   </div>
   <div class="obj-card" id="oc-cylinder" onclick="setObjType('cylinder')" style="background:#0d0d0d;border:2px solid #333;border-radius:5px;padding:8px 4px;text-align:center;cursor:pointer">
    <svg viewBox="0 0 60 40" width="100%" height="30"><ellipse cx="30" cy="10" rx="16" ry="5" fill="#4af" opacity=".7"/><rect x="14" y="10" width="32" height="20" fill="#4af" opacity=".4"/><ellipse cx="30" cy="30" rx="16" ry="5" fill="#4af" opacity=".5"/></svg>
    <div style="font-size:9px;color:#888;margin-top:3px">CYLINDRE</div>
   </div>
  </div>
  <h2 id="aero-params-title">Parametres profil</h2>
  <div class="row" id="aero-p1-row"><span class="lab" id="aero-p1-lab">Corde c [m]</span>
   <input id="chord" type="number" step="0.001" value="0.10" onchange="setAero()"></div>
  <div class="row" id="aero-p2-row"><span class="lab" id="aero-p2-lab">Envergure b [m]</span>
   <input id="span" type="number" step="0.001" value="0.15" onchange="setAero()"></div>
  <div class="row" id="aero-aoa-row"><span class="lab">Angle d attaque &alpha; [deg]</span>
   <input id="aoa" type="number" step="0.5" value="0" onchange="setAero()"></div>
  <div class="row"><span class="lab">rho air [kg/m3]</span>
   <input id="rho" type="number" step="0.001" value="1.204" onchange="setAero()"></div>
  <div class="status" id="obj-st" style="margin-top:6px"></div>
 </div>

 <div class="card">
  <h2>Calibration vitesse vent (modele V = kv * RPM + k0)</h2>
  <div class="row"><span class="lab">kv (m/s par RPM)</span><b class="val" id="fkv">0.003</b></div>
  <div class="row"><span class="lab">k0 (offset m/s)</span><b class="val" id="fk0">0</b></div>
  <div class="row"><span class="lab">RPM actuel</span><b class="val" id="frpm">0</b></div>
  <div class="row"><span class="lab">V depuis flux optique</span><b class="val" id="vflow">0 m/s</b></div>
  <div class="row"><span class="lab">V depuis ventilo</span><b class="val" id="vfan">0 m/s</b></div>
  <div class="row" style="margin-top:8px">
   <button onclick="calibFan('flow')">Calibrer depuis flux optique</button>
  </div>
  <div class="row">
   V mesuree manuellement: <input id="vman" type="number" step="0.1" value="5"> m/s
   <button onclick="calibFan('manual')">Appliquer</button>
  </div>
  <div class="row">
   ou kv direct: <input id="kvman" type="number" step="0.0001" value="0.003">
   <button onclick="calibFan('kv')">Appliquer</button>
  </div>
  <div class="status" id="fan-cal-st"></div>
  <div class="note">Procedure recommandee: 1) avec un anemometre ou tube de Pitot, mesurer V reel
   a un RPM donne. 2) Entrer cette V dans "V mesuree" et cliquer Appliquer. 3) Sauvegarder.
   Alternative: si la calib px/m est fiable, utiliser le flux optique comme reference.</div>
 </div>

 <!-- === CALIBRATION VITESSE PAR PRESSION BMP280 === -->
 <div class="card">
  <h2>Calibration vitesse — BMP280 (&#916;P Bernoulli)</h2>
  <div class="row"><span class="lab">Capteur BMP280</span><b class="val" id="bmp-ok">—</b></div>
  <div class="row"><span class="lab">P absolue</span><b class="val" id="bmp-pa">— Pa</b></div>
  <div class="row"><span class="lab">P réf (ventilo 0)</span><b class="val" id="bmp-ref">— Pa</b></div>
  <div class="row"><span class="lab">&#916;P</span><b class="val" id="bmp-dp">— Pa</b></div>
  <div class="row"><span class="lab">V mesurée (Bernoulli)</span><b class="val" id="bmp-v">— m/s</b></div>
  <div style="margin:10px 0 6px">
   <button onclick="bmpSetRef()" style="margin-right:6px">&#127968; Prendre référence (ventilo ARRÊTÉ)</button>
   <span id="bmp-ref-st" style="font-size:11px;color:#fa0"></span>
  </div>
  <div style="background:#0a1a2a;border-radius:6px;padding:12px;margin:10px 0">
   <div style="font-size:9px;color:#556;letter-spacing:2px;text-transform:uppercase;margin-bottom:8px">Sweep de calibration</div>
   <div style="display:grid;grid-template-columns:1fr 1fr;gap:8px">
    <div><div class="sw-lbl">Duty min %</div><input id="bcs-dmin" type="number" min="20" max="80" value="20" style="width:100%"></div>
    <div><div class="sw-lbl">Duty max %</div><input id="bcs-dmax" type="number" min="30" max="95" value="80" style="width:100%"></div>
    <div><div class="sw-lbl">Nb points</div><input id="bcs-pts" type="number" min="4" max="15" value="8" style="width:100%"></div>
    <div><div class="sw-lbl">Stab. (s)</div><input id="bcs-stab" type="number" min="3" max="20" value="6" style="width:100%"></div>
   </div>
   <button onclick="bcsLaunch()" id="bcs-btn" style="margin-top:10px;width:100%;padding:9px;background:#0a2a1a;color:#4f4;border:1px solid #2a5a3a;border-radius:4px;cursor:pointer;font-family:inherit;font-size:12px;font-weight:bold">
    &#9658; Lancer sweep calibration
   </button>
   <div class="progress-outer" id="bcs-prog-outer" style="display:none"><div class="progress-inner" id="bcs-prog" style="width:0%"></div></div>
   <div id="bcs-st" style="font-size:12px;color:#fa0;margin-top:6px;min-height:16px"></div>
  </div>
  <!-- Courbes théorique vs mesurée -->
  <div id="bcs-chart-wrap" style="display:none;margin-top:12px">
   <div style="font-size:9px;color:#556;letter-spacing:2px;text-transform:uppercase;margin-bottom:6px">RPM → Vitesse : théorique vs mesuré</div>
   <canvas id="bcs-chart" height="180" style="width:100%;border-radius:4px;background:#060e18"></canvas>
   <div style="display:flex;gap:16px;margin-top:6px;font-size:11px">
    <span style="color:#4af">&#9632; Théorique (KV×RPM)</span>
    <span style="color:#4f4">&#9632; Mesuré (BMP280)</span>
   </div>
   <div id="bcs-kv-result" style="margin-top:8px;font-size:12px;color:#aaa"></div>
   <button id="bcs-apply-btn" onclick="bcsApplyKv()" style="display:none;margin-top:8px;padding:6px 14px;background:#0a2a4a;color:#4af;border:1px solid #1a4a8a;border-radius:4px;cursor:pointer;font-family:inherit;font-size:12px">
    Appliquer KV mesuré
   </button>
  </div>
 </div>

 <div class="card">
  <h2>Calculs live</h2>
  <div class="row"><span class="lab">Airspeed V</span><b class="val" id="av">0 m/s</b></div>
  <div class="row"><span class="lab">Reynolds = rho V L / mu</span><b class="val" id="are">0</b></div>
  <div class="row"><span class="lab">Pression dyn q = rho V^2 / 2</span><b class="val" id="aq">0 Pa</b></div>
  <div class="row"><span class="lab">Strouhal = f L / V</span><b class="val" id="ast">0</b></div>
  <div class="row"><span class="lab">C = F / (q A)</span><b class="val" id="acc">0</b></div>
  <div class="note">Selon orientation de la cellule: C = CL si force verticale (portance),
   C = CD si force horizontale (trainee). Adapter l'interpretation au montage.</div>
 </div>
</div>

<!-- DEBUG -->
<div id="p-dbg" class="panel">
 <div class="card">
  <h2>Modulino Knob</h2>
  <div class="row"><span class="lab">Octets I2C (5)</span><span class="hex" id="d-raw">-- -- -- -- --</span></div>
  <div class="row"><span class="lab">Dernier delta</span><b class="val" id="d-dlt">0</b></div>
  <div class="row"><span class="lab">Bouton</span><span><span class="led" id="d-led"></span> <b class="val" id="d-btn">OFF</b></span></div>
  <div class="row"><span class="lab">Erreur I2C</span><b class="val" id="d-err">&mdash;</b></div>
  <div class="row"><span class="lab">Adresse</span><b class="val">0x3A</b></div>
 </div>
 <div class="card">
  <h2>Systeme</h2>
  <div class="row"><span class="lab">Status</span><b class="val" id="d-st">ok</b></div>
  <div class="row"><span class="lab">Fan duty</span><b class="val" id="d-fd">0%</b></div>
  <div class="row"><span class="lab">Fan RPM</span><b class="val" id="d-fr">0</b></div>
 </div>
</div>

<!-- INFO -->
<div id="p-info" class="panel">
 <div class="card">
  <h2>Que peut-on extraire scientifiquement ?</h2>
  <div class="note" style="color:#ccc;font-size:12px">
<b>1. Cellule de charge (force)</b><br>
- Moyenne &rarr; portance/trainee steady-state.<br>
- Ecart-type &rarr; niveau de turbulence / unsteadiness.<br>
- FFT &rarr; frequence du lacher tourbillonnaire (derriere le profil en decrochage).<br>
- Coefficients CL/CD = 2F / (&rho;V&sup2;A). Courbe polaire CL vs CD.<br>
- Courbe CL vs &alpha; &rarr; angle de decrochage.<br>
<br>
<b>2. Fumee + flux optique (Farneback dense)</b><br>
- Champ de vitesse image (vecteurs) &rarr; visualiser circulation autour du profil.<br>
- Magnitude moyenne &rarr; vitesse globale (calibrer px/m).<br>
- Heatmap HSV &rarr; identifier zones rapides / lentes, points d arret, sillage.<br>
- Soustraction fond &rarr; isoler les filets de fumee pour streaklines.<br>
- Vorticite &omega; = &part;Vy/&part;x - &part;Vx/&part;y &rarr; localiser les tourbillons.<br>
- Point de decollement = ou le flux parallele a la paroi s annule.<br>
<br>
<b>3. Nombres adimensionnels</b><br>
- Reynolds Re = &rho;VL/&mu; &rarr; regime (laminaire &lt; 2000, transition, turbulent).<br>
- Strouhal St = fL/V &rarr; compare avec theorie (~0.2 pour cylindre).<br>
- Mach negligeable (V &lt; 30 m/s).<br>
<br>
<b>4. Experiences stylees a faire</b><br>
- Balayage vitesse &rarr; tracer F = f(V&sup2;) pour verifier la loi quadratique.<br>
- Balayage angle d attaque &rarr; CL(&alpha;), trouver decrochage.<br>
- Comparer profil symetrique vs cambre &rarr; portance a &alpha;=0.<br>
- Ajouter volet/flap &rarr; mesurer gain de CL_max.<br>
- Deposer un cylindre &rarr; mesurer St pour verifier ~0.2.<br>
- Couche limite: injecter fumee pres de la paroi, visualiser l epaisseur.<br>
  </div>
 </div>
</div>

<!-- WIFI -->
<div id="p-wifi" class="panel">
 <div class="card">
  <h2>Statut WiFi</h2>
  <div class="row"><span class="lab">Mode</span><b class="val" id="wifi-mode">—</b></div>
  <div class="row"><span class="lab">IP wlan0</span><b class="val" id="wifi-ip">—</b></div>
  <div class="row"><span class="lab">Réseau</span><b class="val" id="wifi-ssid">—</b></div>
  <button onclick="refreshWifi()" style="margin-top:6px">Actualiser</button>
  <div class="status" id="wifi-st"></div>
 </div>

 <div class="card">
  <h2>Hotspot Soufflerie-ENSAM</h2>
  <div class="row"><span class="lab">SSID</span><b class="val">Soufflerie-ENSAM</b></div>
  <div class="row"><span class="lab">Mot de passe</span><b class="val">défini sur le Pi</b></div>
  <div class="row"><span class="lab">IP Pi</span><b class="val">192.168.8.1</b></div>
  <div class="row"><span class="lab">Dashboard</span><b class="val">http://192.168.8.1:8080</b></div>
  <div style="display:flex;gap:8px;margin-top:10px">
   <button class="primary" onclick="hotspot(true)">Activer</button>
   <button class="danger" onclick="hotspot(false)">Desactiver</button>
  </div>
  <div class="status" id="hotspot-st"></div>
  <div class="note">Commandes Wi-Fi désactivées par défaut. Pour un accès réseau, configurer explicitement SOUFFLERIE_BIND_HOST et les droits NetworkManager sur le Pi.</div>
 </div>

 <div class="card">
  <h2>Connexion a un réseau WiFi</h2>
  <div class="row"><span class="lab">SSID</span><input id="wifi-new-ssid" type="text" placeholder="nom du reseau" style="flex:1;background:#0d0d0d;color:#eee;border:1px solid #333;border-radius:3px;padding:4px 6px;font-family:inherit"></div>
  <div class="row"><span class="lab">Mot de passe</span><input id="wifi-new-pwd" type="password" placeholder="laisser vide si ouvert" style="flex:1;background:#0d0d0d;color:#eee;border:1px solid #333;border-radius:3px;padding:4px 6px;font-family:inherit"></div>
  <button class="primary" onclick="connectWifi()" style="margin-top:8px">Connecter</button>
  <div class="status" id="conn-st"></div>
  <div class="note">Coupe le hotspot automatiquement. Cliquer sur un réseau scanné remplit l'SSID automatiquement.</div>
 </div>

 <div class="card">
  <h2>Réseaux disponibles</h2>
  <button onclick="scanWifi()">Scanner</button>
  <div id="wifi-scan-list" style="margin-top:10px;font-size:12px"></div>
 </div>
</div>

<script>
document.querySelectorAll(".tabs button").forEach(b=>b.onclick=()=>{
 document.querySelectorAll(".tabs button").forEach(x=>x.classList.remove("on"));
 document.querySelectorAll(".panel").forEach(x=>x.classList.remove("on"));
 b.classList.add("on");document.getElementById(b.dataset.p).classList.add("on");
});

async function post(u){return fetch(u,{method:"POST"})}
async function doTare(){
 const st=document.getElementById("cal-st");
 st.style.color="#fa0"; st.textContent="tare en cours...";
 const r=await (await post("/tare")).json();
 st.style.color="#4f4"; st.textContent="offset = "+r.offset+" — sauvegarde dans config.py ✓";}
async function doCalib(){const m=+document.getElementById("mref").value;
 const st=document.getElementById("cal-st");
 st.style.color="#fa0"; st.textContent="calibration "+m+"g...";
 const r=await (await post("/calibrate?mass_g="+m)).json();
 st.style.color="#4f4"; st.textContent="scale = "+r.scale.toFixed(4)+" ADU/g — sauvegarde dans config.py ✓";}
async function doSave(){const r=await (await post("/save")).json();
 document.getElementById("save-st").textContent=r.ok?"Sauvegarde dans config.py":"err: "+r.err;}
async function setFilter(){const n=+document.getElementById("fw").value;
 await post("/force/filter?n="+n);}
async function setStep(v){document.getElementById("es").textContent=v.toFixed(1);
 await post("/settings/set?step="+v);
 const s=document.getElementById("es-st");s.textContent="applique: "+v.toFixed(1);
 setTimeout(()=>s.textContent="",1200);}
async function setMode(m){await post("/cam/mode?mode="+m);}
async function setCtl(k,v){
 await post("/cam/controls?"+k+"="+v);
 if(k==="brightness")document.getElementById("cb").textContent=(+v).toFixed(2);
 if(k==="contrast")document.getElementById("cc").textContent=(+v).toFixed(1);
 if(k==="saturation")document.getElementById("csa").textContent=(+v).toFixed(1);}
async function setMask(v){await post("/cam/mask?enable="+v);}
function toggleColourGains(){
 const awb=document.getElementById("awb").checked;
 document.getElementById("colour-gains-row").style.display=awb?"none":"";
}
async function setColourGain(){
 const r=+document.getElementById("cgr").value;
 const b=+document.getElementById("cgb").value;
 document.getElementById("cgr-v").textContent=r.toFixed(2);
 document.getElementById("cgb-v").textContent=b.toFixed(2);
 await post("/cam/colour?r="+r+"&b="+b);
}
async function setColourPreset(r,b){
 document.getElementById("cgr").value=r;
 document.getElementById("cgb").value=b;
 document.getElementById("cgr-v").textContent=r.toFixed(2);
 document.getElementById("cgb-v").textContent=b.toFixed(2);
 await post("/cam/colour?r="+r+"&b="+b);
}
async function setSmokeEnhance(en){await post("/cam/smoke_enhance?enable="+(en?1:0));}
async function setHSV(k,v){document.getElementById(k).textContent=v;
 await post("/cam/hsv?"+k+"="+v);}
async function setPxm(v){await post("/cam/pxm?v="+v);}
async function camReset(){
 const r=await post("/cam/reset").then(x=>x.json()).catch(()=>null);
 if(!r||!r.ok)return;
 // Mettre a jour les sliders et inputs
 const d=r.defaults;
 const q=(id,v)=>{const el=document.getElementById(id);if(el)el.value=v;};
 const qb=(id,v)=>{const el=document.getElementById(id);if(el)el.checked=v;};
 q("cm","raw"); setMode("raw");
 q("cb",d.brightness.toFixed(2)); document.querySelector('[oninput*="brightness"]').value=d.brightness;
 q("cc",d.contrast.toFixed(1));   document.querySelector('[oninput*="contrast"]').value=d.contrast;
 q("csa",d.saturation.toFixed(1));document.querySelector('[oninput*="saturation"]').value=d.saturation;
 qb("ae",d.ae); qb("awb",d.awb);
 qb("mk",false); setMask(0);
 // HSV reset
 [{k:"h_lo",v:0},{k:"h_hi",v:180},{k:"s_lo",v:0},{k:"s_hi",v:60},{k:"v_lo",v:180},{k:"v_hi",v:255}]
  .forEach(({k,v})=>{
   document.getElementById(k).textContent=v;
   document.querySelector('[oninput*="\''+k+'\'"]').value=v;
   post("/cam/hsv?"+k+"="+v);
  });
 // ROI reset
 clearROI();
 q("rx","0");q("ry","0");q("rw","1");q("rh","1");
 // px/m
 q("pxm",d.px_per_m); setPxm(d.px_per_m);
}
async function setROI(){
 const x=+document.getElementById("rx").value;
 const y=+document.getElementById("ry").value;
 const w=+document.getElementById("rw").value;
 const h=+document.getElementById("rh").value;
 await post("/cam/roi?x="+x+"&y="+y+"&w="+w+"&h="+h);
}
async function clearROI(){await post("/cam/roi?clear=1");}
async function calibChessboard(){
 const c=+document.getElementById("cbc").value;
 const r=+document.getElementById("cbr").value;
 const sq=+document.getElementById("cbsq").value;
 document.getElementById("cb-st").textContent="detection...";
 const j=await (await post("/cam/calib_chessboard?cols="+c+"&rows="+r+"&square_mm="+sq)).json();
 document.getElementById("cb-st").textContent=j.ok
  ? "OK : "+j.px_per_m+" px/m ("+j.px_per_mm+" px/mm pour carre "+j.square_mm+"mm)"
  : "Erreur : "+j.err;
 if(j.ok)document.getElementById("pxm").value=j.px_per_m;
}
async function noiseTest(){
 const d=+document.getElementById("nt-dur").value;
 document.getElementById("nt-st").textContent="acquisition "+d+"s...";
 document.getElementById("nt-result").textContent="";
 const j=await (await post("/noise_test?dur="+d)).json();
 if(!j.ok){document.getElementById("nt-st").textContent="Err: "+j.err;return;}
 document.getElementById("nt-st").textContent="OK ("+j.n_samples+" echantillons)";
 document.getElementById("nt-result").textContent=
  "Mean    = "+j.mean_adu+" ADU\n"+
  "Std     = "+j.std_adu+" ADU ("+j.noise_rms_g+" g RMS)\n"+
  "Allan σ = "+j.allan_std_adu+" ADU ("+j.allan_g+" g)\n"+
  "SNR @ 100g = "+j.snr_db_if_force_100g+" dB";
}
async function allanPlot(){
 const stEl=document.getElementById("nt-st");
 stEl.textContent="calcul Allan multi-tau...";
 const r=await fetch("/allan").catch(()=>null);
 if(!r){stEl.textContent="erreur reseau";return;}
 const j=await r.json();
 if(!j.ok){stEl.textContent="Err: "+j.err;return;}
 const data=j.data;
 stEl.textContent=`Allan : ${data.length} pts — ${j.n_samples} echantillons (${j.duration_s}s @ ${j.fs_hz}Hz)`;

 const cv=document.getElementById("allan-cv");
 cv.style.display="block";
 cv.width=cv.offsetWidth; cv.height=160;
 const cx=cv.getContext("2d"),W=cv.width,H=cv.height;
 cx.fillStyle="#0a0a0a"; cx.fillRect(0,0,W,H);

 if(data.length<2){
  cx.fillStyle="#555"; cx.font="11px monospace";
  cx.fillText("pas assez de donnees",W/2-80,H/2); return;
 }

 const ML=52,MR=12,MT=14,MB=22;
 const PW=W-ML-MR, PH=H-MT-MB;

 // domaine log
 const taus=data.map(d=>d.tau_s);
 const devs=data.map(d=>d.allan_dev_g);
 const lx0=Math.log10(Math.min(...taus));
 const lx1=Math.log10(Math.max(...taus));
 const ly0=Math.log10(Math.min(...devs))-0.4;
 const ly1=Math.log10(Math.max(...devs))+0.4;
 const LXR=lx1-lx0||1, LYR=ly1-ly0||1;

 function px(t){ return ML+PW*(Math.log10(t)-lx0)/LXR; }
 function py(d){ return MT+PH*(1-(Math.log10(d)-ly0)/LYR); }

 // --- grille decennie X ---
 cx.font="8px monospace"; cx.textAlign="center";
 for(let e=Math.floor(lx0);e<=Math.ceil(lx1);e++){
  // sub-decennie (2..9)
  for(let s=2;s<=9;s++){
   const lv=e+Math.log10(s);
   if(lv<lx0||lv>lx1) continue;
   const x=ML+PW*(lv-lx0)/LXR;
   cx.strokeStyle="#111"; cx.lineWidth=1;
   cx.beginPath(); cx.moveTo(x,MT); cx.lineTo(x,H-MB); cx.stroke();
  }
  // decennie principale
  const lv=e; if(lv<lx0-0.05||lv>lx1+0.05) continue;
  const x=ML+PW*(lv-lx0)/LXR;
  cx.strokeStyle="#1e1e1e"; cx.lineWidth=1;
  cx.beginPath(); cx.moveTo(x,MT); cx.lineTo(x,H-MB); cx.stroke();
  const val=Math.pow(10,e);
  cx.fillStyle="#555";
  cx.fillText(val>=1?val.toFixed(0)+"s":val.toFixed(2)+"s", x, H-MB+10);
 }

 // --- grille decennie Y ---
 cx.textAlign="right";
 for(let e=Math.floor(ly0);e<=Math.ceil(ly1);e++){
  const y=MT+PH*(1-(e-ly0)/LYR);
  cx.strokeStyle="#1a1a1a"; cx.lineWidth=1;
  cx.beginPath(); cx.moveTo(ML,y); cx.lineTo(W-MR,y); cx.stroke();
  cx.fillStyle="#555";
  cx.fillText(Math.pow(10,e).toExponential(0)+"g", ML-3, y+3);
 }

 // labels axes
 cx.fillStyle="#444"; cx.font="8px monospace"; cx.textAlign="left";
 cx.fillText("σ_A",2,MT+8);
 cx.textAlign="center"; cx.fillText("τ (s)",ML+PW/2,H);

 // --- pentes de reference (ancrées sur le 1er point) ---
 const anchX=Math.log10(taus[0]), anchY=Math.log10(devs[0]);
 function drawSlope(lsx,lsy,slope,color,label){
  cx.strokeStyle=color; cx.setLineDash([3,4]); cx.lineWidth=1;
  const ex=lx1, ey=lsy+slope*(ex-lsx);
  cx.beginPath();
  cx.moveTo(ML+PW*(lsx-lx0)/LXR, MT+PH*(1-(lsy-ly0)/LYR));
  cx.lineTo(ML+PW*(ex-lx0)/LXR,  MT+PH*(1-(ey-ly0)/LYR));
  cx.stroke(); cx.setLineDash([]);
  const mx=(lsx+ex)/2, my=lsy+slope*(mx-lsx);
  const xx=ML+PW*(mx-lx0)/LXR, yy=MT+PH*(1-(my-ly0)/LYR);
  cx.fillStyle=color; cx.font="8px monospace"; cx.textAlign="left";
  cx.fillText(label,xx+3,yy-2);
 }
 drawSlope(anchX,anchY,-0.5,"#2a3a2a","−½ blanc");
 drawSlope(anchX,anchY, 0.0,"#2a2a3a","0 flicker");
 drawSlope(anchX,anchY,+0.5,"#3a2a2a","+½ derive");

 // --- courbe Allan ---
 cx.strokeStyle="#4af"; cx.lineWidth=2; cx.lineJoin="round";
 cx.beginPath();
 data.forEach((d,i)=>{
  const x=px(d.tau_s),y=py(d.allan_dev_g);
  i?cx.lineTo(x,y):cx.moveTo(x,y);
 });
 cx.stroke();

 // points
 cx.fillStyle="#4af";
 data.forEach(d=>{
  cx.beginPath(); cx.arc(px(d.tau_s),py(d.allan_dev_g),2.5,0,Math.PI*2); cx.fill();
 });

 // --- minimum : tau optimal ---
 const iMin=devs.indexOf(Math.min(...devs));
 const xMin=px(taus[iMin]), yMin=py(devs[iMin]);
 cx.strokeStyle="#4f4"; cx.lineWidth=2;
 cx.beginPath(); cx.arc(xMin,yMin,5,0,Math.PI*2); cx.stroke();
 // ligne verticale pointillee
 cx.strokeStyle="#234"; cx.lineWidth=1; cx.setLineDash([2,3]);
 cx.beginPath(); cx.moveTo(xMin,MT); cx.lineTo(xMin,H-MB); cx.stroke();
 cx.setLineDash([]);
 // label
 cx.fillStyle="#4f4"; cx.font="9px monospace"; cx.textAlign="left";
 const tauLabel=taus[iMin]>=1?taus[iMin].toFixed(1)+"s":taus[iMin].toFixed(2)+"s";
 const devLabel=devs[iMin].toExponential(2)+"g";
 const lblX=xMin+(xMin>W*0.7?-130:8), lblY=yMin-6;
 cx.fillText("τ_opt="+tauLabel,lblX,lblY);
 cx.fillText("σ_min="+devLabel,lblX,lblY+10);
}
// ---- LED ----
function _hexToRgb(hex){
 const r=parseInt(hex.slice(1,3),16),g=parseInt(hex.slice(3,5),16),b=parseInt(hex.slice(5,7),16);
 return {r,g,b};
}
function _rgbToHex(r,g,b){
 return "#"+[r,g,b].map(v=>v.toString(16).padStart(2,"0")).join("");
}
async function ledApply(){
 const on=document.getElementById("led-on").checked;
 const anim=on?document.getElementById("led-anim").value:"off";
 const rgb=_hexToRgb(document.getElementById("led-color").value);
 const bri=+document.getElementById("led-bri").value;
 const spd=+document.getElementById("led-spd").value;
 const n=+document.getElementById("led-n").value;
 const url=`/led/set?anim=${anim}&r=${rgb.r}&g=${rgb.g}&b=${rgb.b}&bri=${bri}&speed=${spd}&n=${n}`;
 const j=await (await post(url)).json();
 _ledUpdateUI(j);
}
async function ledToggle(){
 const on=document.getElementById("led-on").checked;
 document.getElementById("led-on-lbl").textContent=on?"ON":"OFF";
 await ledApply();
}
function ledColor(r,g,b){
 document.getElementById("led-color").value=_rgbToHex(r,g,b);
 document.getElementById("led-on").checked=true;
 document.getElementById("led-on-lbl").textContent="ON";
 ledApply();
}
function ledExposition(){
 document.getElementById("led-anim").value="exposition";
 document.getElementById("led-on").checked=true;
 document.getElementById("led-on-lbl").textContent="ON";
 ledApply();
}
function _ledUpdateUI(j){
 const badge=document.getElementById("led-status-badge");
 const errEl=document.getElementById("led-err");
 if(!j.ok){
  badge.style.background="#3a0000";badge.style.color="#f55";badge.textContent="ERR";
  errEl.style.display="block";errEl.textContent=j.error||"erreur LED";
 } else {
  badge.style.background="#1a3a00";badge.style.color="#4f4";badge.textContent="OK";
  errEl.style.display="none";
 }
}
async function _ledPoll(populate){
 const j=await fetch("/led").then(r=>r.json()).catch(()=>null);
 if(!j) return;
 _ledUpdateUI(j);
 if(populate){
  // sync UI controls to server state (e.g. after reboot loads calib.json)
  const animEl=document.getElementById("led-anim");
  const colorEl=document.getElementById("led-color");
  const briEl=document.getElementById("led-bri");
  const spdEl=document.getElementById("led-spd");
  const nEl=document.getElementById("led-n");
  if(animEl && j.animation) animEl.value=j.animation;
  if(colorEl && j.color) colorEl.value=_rgbToHex(j.color[0],j.color[1],j.color[2]);
  if(briEl && j.brightness!=null){briEl.value=j.brightness;document.getElementById("led-bri-val").textContent=j.brightness;}
  if(spdEl && j.speed!=null){spdEl.value=j.speed;document.getElementById("led-spd-val").textContent=j.speed;}
  if(nEl && j.n_leds!=null) nEl.value=j.n_leds;
  const onEl=document.getElementById("led-on");
  const lblEl=document.getElementById("led-on-lbl");
  if(onEl){onEl.checked=j.animation!=="off";lblEl.textContent=j.animation!=="off"?"ON":"OFF";}
 }
}
setInterval(()=>_ledPoll(false),5000);
_ledPoll(true);

async function doTareSupport(){
 const duty=+document.getElementById("drag-duty").value;
 const st=document.getElementById("drag-st");
 const btn=document.getElementById("btn-drag-tare");
 st.textContent="Démarrage ventilateur ("+duty+"%)... attendre ~8s";
 btn.disabled=true;
 try{
  const r=await (await post("/tare_support?duty="+duty)).json();
  if(r.ok){st.textContent="Trainee support = "+r.support_drag_N.toFixed(4)+" N — sauvegardé";}
  else{st.textContent="Erreur : "+(r.err||"pas de données");}
 }catch(e){st.textContent="Erreur réseau";}
 btn.disabled=false;
}
async function setDragEnable(){
 const en=document.getElementById("drag-en").checked;
 await post("/tare_support/enable?v="+en);
}
// ---- BMP280 calibration ----
let _bcsCalibTable=[];
let _bcsKvFit=null;

async function bmpSetRef(){
 const st=document.getElementById("bmp-ref-st");
 st.textContent="Mesure référence en cours (~3s)..."; st.style.color="#fa0";
 const r=await post("/calibrate/pressure_ref").then(x=>x.json()).catch(()=>({ok:false,err:"timeout"}));
 if(r.ok){
  st.textContent="Référence prise : "+r.p_ref.toFixed(1)+" Pa"; st.style.color="#4f4";
  document.getElementById("bmp-ref").textContent=r.p_ref.toFixed(2)+" Pa";
 } else {
  st.textContent="Erreur : "+r.err; st.style.color="#f55";
 }
}

async function bcsLaunch(){
 if(!await post("/calibrate/pressure_ref").then(x=>x.json().then(r=>r.ok)).catch(()=>false)){
  document.getElementById("bcs-st").textContent="Erreur : impossible de prendre la référence";
  return;
 }
 const dmin=+document.getElementById("bcs-dmin").value;
 const dmax=+document.getElementById("bcs-dmax").value;
 const pts =+document.getElementById("bcs-pts").value;
 const stab=+document.getElementById("bcs-stab").value;
 const url=`/calibrate/pressure_sweep?dmin=${dmin}&dmax=${dmax}&pts=${pts}&stab=${stab}`;
 document.getElementById("bcs-btn").disabled=true;
 document.getElementById("bcs-prog-outer").style.display="";
 document.getElementById("bcs-chart-wrap").style.display="none";
 _bcsCalibTable=[];
 const es=new EventSource(url);
 es.onmessage=e=>{
  const d=JSON.parse(e.data);
  if(d.done){
   es.close();
   document.getElementById("bcs-btn").disabled=false;
   document.getElementById("bcs-st").textContent="Sweep terminé — "+_bcsCalibTable.length+" points";
   _bcsDrawChart();
   return;
  }
  if(d.error){
   es.close();
   document.getElementById("bcs-btn").disabled=false;
   document.getElementById("bcs-st").textContent="Erreur : "+d.error; return;
  }
  document.getElementById("bcs-prog").style.width=(d.progress*100).toFixed(0)+"%";
  document.getElementById("bcs-st").textContent="Point "+d.idx+" — duty="+d.duty+"%  RPM="+d.rpm+"  ΔP="+d.dp.toFixed(1)+"Pa  V_mes="+d.v_pres.toFixed(2)+"m/s";
  _bcsCalibTable.push(d);
 };
 es.onerror=()=>{es.close();document.getElementById("bcs-btn").disabled=false;};
}

function _bcsDrawChart(){
 if(!_bcsCalibTable.length) return;
 const canvas=document.getElementById("bcs-chart");
 const ctx=canvas.getContext("2d");
 canvas.width=canvas.offsetWidth||400; canvas.height=180;
 const W=canvas.width, H=canvas.height;
 const pad={l:42,r:16,t:16,b:32};
 const W_=W-pad.l-pad.r, H_=H-pad.t-pad.b;
 ctx.fillStyle="#060e18"; ctx.fillRect(0,0,W,H);
 // data ranges
 const rpms=_bcsCalibTable.map(p=>p.rpm);
 const vfans=_bcsCalibTable.map(p=>p.v_fan);
 const vpres=_bcsCalibTable.map(p=>p.v_pres);
 const rmax=Math.max(...rpms)||1;
 const vmax=Math.max(...vfans,...vpres)*1.1||1;
 const rx=rpm=>(rpm/rmax)*W_+pad.l;
 const vy=v=>H-pad.b-(v/vmax)*H_;
 // grid
 ctx.strokeStyle="#1a2a3a"; ctx.lineWidth=1;
 for(let i=0;i<=4;i++){
  const y=vy(vmax*i/4);
  ctx.beginPath();ctx.moveTo(pad.l,y);ctx.lineTo(W-pad.r,y);ctx.stroke();
  ctx.fillStyle="#556";ctx.font="9px monospace";ctx.textAlign="right";
  ctx.fillText((vmax*i/4).toFixed(1),pad.l-3,y+3);
 }
 for(let i=0;i<=4;i++){
  const x=rx(rmax*i/4);
  ctx.beginPath();ctx.moveTo(x,pad.t);ctx.lineTo(x,H-pad.b);ctx.stroke();
  ctx.fillStyle="#556";ctx.font="9px monospace";ctx.textAlign="center";
  ctx.fillText(Math.round(rmax*i/4),x,H-pad.b+14);
 }
 // axes labels
 ctx.fillStyle="#888";ctx.font="9px monospace";ctx.textAlign="center";
 ctx.fillText("RPM",W/2,H-2);
 ctx.save();ctx.translate(10,H/2);ctx.rotate(-Math.PI/2);ctx.fillText("V m/s",0,0);ctx.restore();
 // theoretical line (blue)
 ctx.strokeStyle="#4af"; ctx.lineWidth=2;
 ctx.beginPath();
 _bcsCalibTable.forEach((p,i)=>{ const x=rx(p.rpm),y=vy(p.v_fan); i===0?ctx.moveTo(x,y):ctx.lineTo(x,y); });
 ctx.stroke();
 // measured points + line (green)
 ctx.strokeStyle="#4f4"; ctx.lineWidth=2;
 ctx.beginPath();
 _bcsCalibTable.forEach((p,i)=>{ const x=rx(p.rpm),y=vy(p.v_pres); i===0?ctx.moveTo(x,y):ctx.lineTo(x,y); });
 ctx.stroke();
 ctx.fillStyle="#4f4";
 _bcsCalibTable.forEach(p=>{
  ctx.beginPath();ctx.arc(rx(p.rpm),vy(p.v_pres),4,0,2*Math.PI);ctx.fill();
 });
 // linear regression on measured data for KV
 const n=_bcsCalibTable.length;
 const sumR=rpms.reduce((a,b)=>a+b,0);
 const sumV=vpres.reduce((a,b)=>a+b,0);
 const sumR2=rpms.reduce((a,b)=>a+b*b,0);
 const sumRV=_bcsCalibTable.reduce((a,p)=>a+p.rpm*p.v_pres,0);
 _bcsKvFit=(n*sumRV-sumR*sumV)/(n*sumR2-sumR*sumR)||null;
 document.getElementById("bcs-chart-wrap").style.display="";
 const kvFan=_bcsCalibTable[0]?.kv_fan||0;
 document.getElementById("bcs-kv-result").innerHTML=
  `KV théorique : <b style="color:#4af">${kvFan.toFixed(6)}</b> m/s/RPM &nbsp;|&nbsp; ` +
  `KV mesuré (régression) : <b style="color:#4f4">${(_bcsKvFit||0).toFixed(6)}</b> m/s/RPM`;
 document.getElementById("bcs-apply-btn").style.display=_bcsKvFit?"":"none";
}

async function bcsApplyKv(){
 if(!_bcsKvFit) return;
 const r=await post("/fan/calib?kv="+_bcsKvFit).then(x=>x.json()).catch(()=>({ok:false}));
 document.getElementById("bcs-kv-result").innerHTML+=
  r.ok?' &nbsp;<span style="color:#4f4">✓ KV appliqué et sauvegardé</span>':' <span style="color:#f55">Erreur</span>';
 document.getElementById("bcs-apply-btn").style.display="none";
}

async function calibFan(mode){
 let url="/fan/calib?";
 if(mode==="flow")url+="from_flow=1";
 else if(mode==="manual")url+="v="+(+document.getElementById("vman").value);
 else if(mode==="kv")url+="kv="+(+document.getElementById("kvman").value);
 const r=await (await post(url)).json();
 const s=document.getElementById("fan-cal-st");
 if(r.ok){
  s.textContent="OK : kv="+(+r.kv).toFixed(6)+(r.rpm?(" (RPM="+r.rpm+", V="+r.v+")"):"");
 } else {
  s.textContent="Erreur : "+r.err;
 }
}
const OBJ_DEFS={
 airfoil:{title:"Parametres profil",p1:"Corde c [m]",p2:"Envergure b [m]",angle:true,coef:"CL / CD"},
 plate:  {title:"Parametres plaque",p1:"Largeur w [m]",p2:"Hauteur h [m]",angle:true,coef:"CD / CL"},
 cube:   {title:"Parametres cube",p1:"Cote a [m]",p2:null,angle:false,coef:"CD"},
 sphere: {title:"Parametres sphere",p1:"Diametre d [m]",p2:null,angle:false,coef:"CD"},
 cylinder:{title:"Parametres cylindre",p1:"Diametre d [m]",p2:"Longueur L [m]",angle:false,coef:"CD, St"},
};
let _curObjType="airfoil";
function setObjType(t){
 _curObjType=t;
 const d=OBJ_DEFS[t];
 document.querySelectorAll(".obj-card").forEach(c=>{
  c.style.borderColor="#333";
  c.querySelector("div").style.color="#888";
 });
 const sel=document.getElementById("oc-"+t);
 if(sel){sel.style.borderColor="#4af";sel.querySelector("div").style.color="#4af";}
 if(document.getElementById("aero-params-title"))document.getElementById("aero-params-title").textContent=d.title;
 if(document.getElementById("aero-p1-lab"))document.getElementById("aero-p1-lab").textContent=d.p1;
 const p2row=document.getElementById("aero-p2-row");
 if(p2row)p2row.style.display=d.p2?"":"none";
 if(d.p2&&document.getElementById("aero-p2-lab"))document.getElementById("aero-p2-lab").textContent=d.p2;
 const aoarow=document.getElementById("aero-aoa-row");
 if(aoarow)aoarow.style.display=d.angle?"":"none";
 if(document.getElementById("obj-st"))document.getElementById("obj-st").textContent="→ Mesure : "+d.coef;
 post("/aero/set?type="+t);
}
async function setAero(){
 const q=new URLSearchParams({type:_curObjType,chord:+document.getElementById("chord").value,
  span:+document.getElementById("span").value,
  aoa:+document.getElementById("aoa").value,
  rho:+document.getElementById("rho").value}).toString();
 await post("/aero/set?"+q);}

async function refreshWifi(){
 const st=document.getElementById("wifi-st");
 st.textContent="...";
 const r=await fetch("/wifi/status",{method:"POST"}).then(x=>x.json()).catch(()=>null);
 if(!r){st.textContent="erreur";return;}
 st.textContent="";
 document.getElementById("wifi-mode").textContent=r.hotspot_active?"HOTSPOT (Soufflerie-ENSAM)":r.client_ssid?"CLIENT — "+r.client_ssid:"non connecte";
 document.getElementById("wifi-ip").textContent=r.wlan0_ip||"—";
 document.getElementById("wifi-ssid").textContent=r.client_ssid||(r.hotspot_active?"Soufflerie-ENSAM":"—");
}
async function hotspot(en){
 const st=document.getElementById("hotspot-st");
 st.style.color="#fa0"; st.textContent=(en?"Activation":"Désactivation")+" en cours (~5s)...";
 const r=await post("/wifi/hotspot?enable="+(en?1:0)).then(x=>x.json()).catch(()=>({ok:false,err:"timeout"}));
 st.style.color=r.ok?"#4f4":"#f44";
 st.textContent=r.ok?"OK — "+(en?"Hotspot actif (192.168.8.1:8080)":"Hotspot arrêté"):"Erreur: "+r.err;
 refreshWifi();
}
async function connectWifi(){
 const ssid=document.getElementById("wifi-new-ssid").value.trim();
 const pwd=document.getElementById("wifi-new-pwd").value;
 if(!ssid)return;
 const st=document.getElementById("conn-st");
 st.style.color="#fa0"; st.textContent="Connexion a "+ssid+"... (~15s)";
 const r=await fetch("/wifi/connect",{method:"POST",headers:{"Content-Type":"application/x-www-form-urlencoded"},body:"ssid="+encodeURIComponent(ssid)+"&password="+encodeURIComponent(pwd)}).then(x=>x.json()).catch(()=>({ok:false,err:"timeout"}));
 st.style.color=r.ok?"#4f4":"#f44";
 st.textContent=r.ok?"Connecte a "+ssid+" — IP: "+(r.ip||"?"):"Erreur: "+r.err;
 refreshWifi();
}
async function scanWifi(){
 const d=document.getElementById("wifi-scan-list");
 d.style.color="#fa0"; d.textContent="scan en cours...";
 const r=await fetch("/wifi/scan",{method:"POST"}).then(x=>x.json()).catch(()=>null);
 if(!r||!r.ok){d.style.color="#f44";d.textContent="Erreur: "+(r&&r.err||"reseau");return;}
 d.style.color="#eee";
 d.replaceChildren();
 for(const n of r.networks){
  const bars=n.signal>-55?"▮▮▮▮":n.signal>-65?"▮▮▮▯":n.signal>-75?"▮▮▯▯":"▮▯▯▯";
  const row=document.createElement("div");
  row.style.cssText="padding:5px 0;border-bottom:1px solid #1e1e1e;cursor:pointer;display:flex;justify-content:space-between";
  row.addEventListener("click",()=>{document.getElementById("wifi-new-ssid").value=n.ssid;});
  const name=document.createElement("span"); name.style.color="#4af"; name.textContent=n.ssid;
  const info=document.createElement("span"); info.style.cssText="color:#888;font-size:11px";
  info.textContent=`${bars} ${n.signal}dBm ${n.security?"🔒":""}`;
  row.append(name,info); d.append(row);
 }
 if(!r.networks.length)d.textContent="aucun réseau trouvé";
}
refreshWifi();

const evs=new EventSource("/events");let init=false;
evs.onmessage=e=>{const d=JSON.parse(e.data);
 // force
 document.getElementById("s-raw").textContent=d.raw;
 document.getElementById("s-delta").textContent=d.delta;
 document.getElementById("s-off").textContent=d.offset;
 document.getElementById("s-sc").textContent=(+d.scale).toFixed(3);
 document.getElementById("s-fN").textContent=d.force_N.toFixed(5)+" N";
 document.getElementById("s-fm").textContent=d.force_mean.toFixed(5);
 document.getElementById("s-fstd").textContent=d.force_std.toFixed(5);
 document.getElementById("s-frms").textContent=d.force_rms.toFixed(5);
 document.getElementById("s-fft").textContent=d.peak_freq_hz.toFixed(2)+" Hz";
 if(document.getElementById("s-drag"))document.getElementById("s-drag").textContent=(d.support_drag_N||0).toFixed(4)+" N";
 if(document.getElementById("drag-en")&&!init)document.getElementById("drag-en").checked=d.support_drag_enabled||false;
 // aero live
 const v=d.airspeed_avg_ms||d.airspeed_ms;
 document.getElementById("av").textContent=v.toFixed(2)+" m/s";
 document.getElementById("fkv").textContent=(+d.fan_kv).toFixed(6);
 document.getElementById("fk0").textContent=(+d.fan_k0).toFixed(2);
 document.getElementById("frpm").textContent=d.fan_rpm;
 document.getElementById("vflow").textContent=d.airspeed_ms.toFixed(3)+" m/s";
 document.getElementById("vfan").textContent=(d.airspeed_fan_ms||0).toFixed(3)+" m/s";
 // BMP280
 const bmpOk=document.getElementById("bmp-ok");
 if(bmpOk){
  bmpOk.textContent=d.pressure_ok?"OK ✓":"KO"; bmpOk.style.color=d.pressure_ok?"#4f4":"#f55";
  document.getElementById("bmp-pa").textContent=d.pressure_pa?(d.pressure_pa.toFixed(1)+" Pa"):"—";
  document.getElementById("bmp-ref").textContent=d.pressure_ref_pa?(d.pressure_ref_pa.toFixed(1)+" Pa"):"— (prendre réf.)";
  document.getElementById("bmp-dp").textContent=d.pressure_delta_pa?(d.pressure_delta_pa.toFixed(2)+" Pa"):"0 Pa";
  document.getElementById("bmp-v").textContent=d.airspeed_pressure_ms?(d.airspeed_pressure_ms.toFixed(2)+" m/s"):"— m/s";
 }
 document.getElementById("are").textContent=Math.round(d.reynolds);
 document.getElementById("aq").textContent=(0.5*d.rho*v*v).toFixed(3)+" Pa";
 document.getElementById("ast").textContent=d.strouhal.toFixed(3);
 document.getElementById("acc").textContent=d.CL.toFixed(4);
 // modulino
 const raw=d.modulino_raw||[0,0,0,0,0];
 document.getElementById("d-raw").textContent=raw.map(b=>b.toString(16).padStart(2,"0").toUpperCase()).join(" ");
 document.getElementById("d-dlt").textContent=d.modulino_last_delta;
 document.getElementById("d-led").className="led"+(d.modulino_btn?" on":"");
 document.getElementById("d-btn").textContent=d.modulino_btn?"PRESSED":"OFF";
 document.getElementById("d-err").textContent=d.modulino_err||"\u2014";
 document.getElementById("d-st").textContent=d.status;
 document.getElementById("d-fd").textContent=d.fan_duty.toFixed(0)+"%";
 document.getElementById("d-fr").textContent=d.fan_rpm;
 if(!init){init=true;
  document.getElementById("fw").value=d.filter_window;
  document.getElementById("erng").value=d.encoder_step;
  document.getElementById("es").textContent=(+d.encoder_step).toFixed(1);
  document.getElementById("cm").value=d.cam_mode;
  document.getElementById("chord").value=d.chord_m;
  document.getElementById("span").value=d.span_m;
  document.getElementById("aoa").value=d.aoa_deg;
  document.getElementById("rho").value=d.rho;
  document.getElementById("pxm").value=d.px_per_m;
  if(d.cam_dist_m!=null){const el=document.getElementById("cam-dist");if(el)el.value=d.cam_dist_m;}
  document.getElementById("mk").checked=d.cam_mask;}
};
</script></body></html>
"""


# ==================== CALIBRATION HTML ====================

CALIB_HTML = r"""<!DOCTYPE html>
<html lang="fr"><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Calibration — Soufflerie ENSAM</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#08080f;color:#dde;font-family:ui-monospace,Menlo,monospace;min-height:100vh}
header{display:flex;align-items:center;gap:12px;padding:12px 16px;border-bottom:1px solid #1a2030;background:#0a0a14}
header h1{flex:1;color:#8af;font-size:15px;letter-spacing:3px;text-transform:uppercase}
a.back{color:#8af;text-decoration:none;font-size:12px;opacity:.7}
a.back:hover{opacity:1}

/* Steps */
.stepbar{display:flex;align-items:center;padding:16px 20px;gap:0;background:#0a0a18;border-bottom:1px solid #1a2030}
.si{display:flex;align-items:center;flex:1}
.sn{width:32px;height:32px;border-radius:50%;display:flex;align-items:center;justify-content:center;font-size:13px;font-weight:bold;border:2px solid #222;background:#111;color:#444;flex-shrink:0;transition:all .4s}
.sl{font-size:10px;color:#444;margin-left:8px;letter-spacing:1px;text-transform:uppercase;line-height:1.3;transition:color .4s}
.sconn{flex:1;height:1px;background:#1a2030;margin:0 6px}
.si.done .sn{background:#061a10;color:#3f3;border-color:#2a5a3a}
.si.done .sl{color:#3f3}
.si.active .sn{background:#0a1a30;color:#8af;border-color:#4af;box-shadow:0 0 12px #4af4}
.si.active .sl{color:#8af}

/* Layout */
.content{max-width:680px;margin:0 auto;padding:16px}
.panel{display:none}.panel.on{display:block}
.card{background:#0e0e1c;border:1px solid #1a1a2e;border-radius:10px;padding:18px;margin-bottom:14px}
.card h2{color:#8af;font-size:10px;letter-spacing:3px;text-transform:uppercase;margin-bottom:14px;display:flex;align-items:center;gap:8px}
.card h2::after{content:"";flex:1;height:1px;background:#1a1a2e}

/* Instr box */
.instr{background:#0a0a18;border-left:3px solid #4af;padding:14px 16px;border-radius:0 8px 8px 0;margin-bottom:16px;font-size:13px;line-height:1.7;color:#aac}
.instr b,.instr strong{color:#8af}

/* Live display */
.bigval{text-align:center;padding:14px 0 10px;font-size:36px;font-weight:bold;letter-spacing:4px;color:#8af;font-variant-numeric:tabular-nums}
.bigval .unit{font-size:14px;color:#446;margin-left:6px}
.bigval.warn{color:#f93}
.bigval.ok{color:#3f3}

/* Stability gauge */
.gauge-row{display:flex;align-items:center;gap:10px;margin:4px 0 12px}
.gauge-bg{flex:1;height:6px;background:#111;border-radius:3px;overflow:hidden}
.gauge-fill{height:100%;border-radius:3px;transition:width .4s,background .4s}
.gauge-lbl{font-size:10px;color:#556;min-width:60px;text-align:right}

/* Form rows */
.row{display:flex;justify-content:space-between;align-items:center;padding:7px 0;font-size:13px;gap:12px}
.lab{color:#778;flex:1}
.val{color:#8af;font-weight:bold}
input[type=number]{width:100px;background:#060616;color:#dde;border:1px solid #2a2a40;border-radius:4px;padding:6px 10px;font-family:inherit;font-size:13px}
input[type=range]{accent-color:#8af;width:100%}

/* Buttons */
.btn{padding:12px 0;border:none;border-radius:6px;cursor:pointer;font-family:inherit;font-size:13px;font-weight:bold;letter-spacing:1px;transition:all .25s;width:100%}
.btn:disabled{opacity:.35;cursor:not-allowed}
.btn-go{background:linear-gradient(135deg,#1a3a8a,#2a5ab0);color:#fff}
.btn-go:hover:not(:disabled){background:linear-gradient(135deg,#2a4a9a,#3a6ac0);box-shadow:0 4px 16px #4af3}
.btn-sec{background:#0e0e1c;color:#8af;border:1px solid #2a2a40}
.btn-sec:hover:not(:disabled){background:#1a1a2e}
.btn-danger{background:#1c0a0a;color:#f55;border:1px solid #4a1a1a}

/* Progress */
.prog-outer{background:#0a0a18;border-radius:6px;height:10px;overflow:hidden;margin:10px 0}
.prog-inner{height:100%;border-radius:6px;background:linear-gradient(90deg,#1a3a8a,#4af);transition:width .6s}

/* Status chips */
.chip{display:inline-flex;align-items:center;gap:5px;font-size:11px;padding:3px 9px;border-radius:12px;font-weight:bold}
.chip.ok{background:#061a10;color:#3f3;border:1px solid #1a4a2a}
.chip.warn{background:#1a1000;color:#f93;border:1px solid #4a2a00}
.chip.err{background:#1a0a0a;color:#f55;border:1px solid #4a1a1a}
.chip.idle{background:#0e0e1c;color:#556;border:1px solid #2a2a40}

/* Fan sweep table */
.sweep-table{width:100%;border-collapse:collapse;font-size:12px;margin-top:8px}
.sweep-table th{color:#556;text-align:center;padding:5px 8px;border-bottom:1px solid #1a1a2e;font-size:9px;letter-spacing:2px;text-transform:uppercase}
.sweep-table td{padding:5px 8px;text-align:center;border-bottom:1px solid #0e0e1c;transition:background .3s}
.sweep-table tr.current td{background:#0a1a30;color:#8af}
.sweep-table .f-val{color:#3f3;font-weight:bold}

/* Canvas chart */
#chart{display:block;width:100%;border-radius:6px;background:#060616;margin-top:10px}

/* Nav */
.nav{max-width:680px;margin:0 auto;padding:0 16px 20px;display:flex;gap:10px}
.nav .btn{flex:1}
.nav .btn-go{flex:2}

/* Calib state summary */
.state-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:4px}
.state-item{background:#060616;border:1px solid #1a1a2e;border-radius:6px;padding:10px 12px}
.state-item .si-lbl{font-size:9px;color:#556;letter-spacing:2px;text-transform:uppercase;margin-bottom:4px}
.state-item .si-val{font-size:15px;font-weight:bold;color:#8af}
.state-item.done-item .si-val{color:#3f3}
</style></head><body>
<header>
 <a class="back" href="/">&larr; Dashboard</a>
 <h1>&#9670; Calibration capteur</h1>
 <a class="back" href="/etude">Etude &rarr;</a>
</header>

<!-- Steps -->
<div class="stepbar">
 <div class="si active" id="si-1"><div class="sn">1</div><div class="sl">Tare<br>zero</div></div>
 <div class="sconn"></div>
 <div class="si" id="si-2"><div class="sn">2</div><div class="sl">Masse<br>etalon</div></div>
 <div class="sconn"></div>
 <div class="si" id="si-3"><div class="sn">3</div><div class="sl">Courbe<br>ventilo</div></div>
</div>

<div class="content">

<!-- Etat global calibration -->
<div id="calib-summary" class="card" style="display:none">
 <h2>Etat calibration actuelle</h2>
 <div class="state-grid">
  <div class="state-item" id="sum-tare"><div class="si-lbl">Tare (offset)</div><div class="si-val" id="sum-offset">—</div></div>
  <div class="state-item" id="sum-scale"><div class="si-lbl">Scale (ADU/g)</div><div class="si-val" id="sum-sc">—</div></div>
  <div class="state-item" id="sum-drag"><div class="si-lbl">Table trainee</div><div class="si-val" id="sum-drag-pts">—</div></div>
  <div class="state-item"><div class="si-lbl">Sauvegarde</div><div class="si-val" id="sum-saved" style="font-size:11px;color:#556">—</div></div>
 </div>
</div>

<!-- STEP 1 : Tare -->
<div id="p-1" class="panel on">
 <div class="card">
  <h2>1 — Tare de reference (zero electrique)</h2>
  <div class="instr">
   <strong>&#9888; Action :</strong> Retirez <strong>tout</strong> du capteur et du tunnel.<br>
   Ventilateur <strong>eteint</strong>. Aucune masse, aucun support.<br>
   Cette etape capture le zero electrique de la cellule HX711.
  </div>
  <div class="bigval" id="raw-disp1">— <span class="unit">ADU</span></div>
  <div class="gauge-row">
   <span style="font-size:10px;color:#556;min-width:55px">Stabilite</span>
   <div class="gauge-bg"><div class="gauge-fill" id="stab-fill1" style="width:0%;background:#f93"></div></div>
   <span class="gauge-lbl" id="stab-lbl1">σ = —</span>
  </div>
  <div style="display:flex;align-items:center;justify-content:center;gap:8px;margin-bottom:12px">
   <span id="fan-chip1" class="chip idle">&#11044; Ventilo ?</span>
   <span id="stab-chip1" class="chip idle">&#9632; Signal ?</span>
  </div>
  <button class="btn btn-go" onclick="doStep1()" id="btn1">TARER — CAPTURER LE ZERO</button>
  <div style="margin-top:10px;font-size:12px;min-height:16px" id="st1"></div>
 </div>
</div>

<!-- STEP 2 : Masse -->
<div id="p-2" class="panel">
 <div class="card">
  <h2>2 — Etalonnage masse (ADU / gramme)</h2>
  <div class="instr">
   <strong>&#9888; Action :</strong> Mettez la soufflerie a la <strong>verticale</strong>.<br>
   Posez un poids de reference connu sur le capteur.<br>
   Entrez sa masse exacte ci-dessous, puis calibrez.
  </div>
  <div class="bigval" id="raw-disp2">— <span class="unit">ADU</span></div>
  <div class="row">
   <span class="lab">Delta (raw - offset)</span>
   <b class="val" id="delta-disp">— ADU</b>
  </div>
  <div class="row">
   <span class="lab">Scale calcule en temps reel</span>
   <b class="val" id="sc-live">— ADU/g</b>
  </div>
  <div class="row" style="margin-top:8px">
   <span class="lab" style="font-size:14px;font-weight:bold;color:#8af">Masse etalon</span>
   <input id="mass-inp" type="number" step="0.1" min="10" max="5000" value="100" style="width:110px;font-size:16px;text-align:center"> <span style="color:#556;margin-left:4px">g</span>
  </div>
  <div style="margin-top:14px"></div>
  <button class="btn btn-go" onclick="doStep2()" id="btn2">CALIBRER — CALCULER ADU/g</button>
  <div style="margin-top:10px;font-size:12px;min-height:16px" id="st2"></div>
 </div>
</div>

<!-- STEP 3 : Courbe ventilo -->
<div id="p-3" class="panel">
 <div class="card">
  <h2>3 — Courbe de trainee ventilateur (sweep automatique)</h2>
  <div class="instr">
   <strong>&#9888; Action :</strong> <strong>Fermez le tunnel</strong> (sans profil, sans support).<br>
   Le ventilateur va sweeper automatiquement sur la plage choisie.<br>
   Il mesure la force de base (bruit aero + vibrations) a chaque vitesse.<br>
   Cette courbe sera <strong>soustraite automatiquement</strong> lors des mesures.
  </div>
  <div style="display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-bottom:12px">
   <div>
    <div style="font-size:9px;color:#556;letter-spacing:2px;text-transform:uppercase;margin-bottom:4px">Duty min</div>
    <input id="sw-min" type="number" min="10" max="80" value="20" style="width:100%;text-align:center"> %
   </div>
   <div>
    <div style="font-size:9px;color:#556;letter-spacing:2px;text-transform:uppercase;margin-bottom:4px">Duty max</div>
    <input id="sw-max" type="number" min="30" max="100" value="80" style="width:100%;text-align:center"> %
   </div>
   <div>
    <div style="font-size:9px;color:#556;letter-spacing:2px;text-transform:uppercase;margin-bottom:4px">Nb points</div>
    <input id="sw-pts" type="number" min="3" max="15" value="7" style="width:100%;text-align:center">
   </div>
   <div>
    <div style="font-size:9px;color:#556;letter-spacing:2px;text-transform:uppercase;margin-bottom:4px">Stabilisation (s)</div>
    <input id="sw-stab" type="number" min="2" max="15" value="5" style="width:100%;text-align:center">
   </div>
  </div>

  <!-- Current measurement display -->
  <div id="sweep-running" style="display:none">
   <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:6px">
    <span style="font-size:11px;color:#556">Duty actuel</span>
    <b style="font-size:22px;color:#8af" id="sw-cur-duty">—%</b>
    <span style="font-size:11px;color:#556">Force mesuree</span>
    <b style="font-size:22px;color:#3f3" id="sw-cur-F">— N</b>
   </div>
   <div class="prog-outer"><div class="prog-inner" id="sw-prog" style="width:0%"></div></div>
   <div style="font-size:11px;color:#556;text-align:center" id="sw-phase-lbl">—</div>
  </div>

  <button class="btn btn-go" onclick="doSweep()" id="btn3" style="margin-top:12px">LANCER LE SWEEP AUTOMATIQUE</button>
  <button class="btn btn-danger" onclick="resetSweep()" id="btn-reset" style="margin-top:6px;display:none">Effacer la courbe</button>
  <div style="margin-top:10px;font-size:12px;min-height:16px" id="st3"></div>
 </div>

 <!-- Results table + chart -->
 <div class="card" id="sweep-results" style="display:none">
  <h2>Courbe de trainee mesuree</h2>
  <canvas id="chart" height="140"></canvas>
  <table class="sweep-table" id="sw-table">
   <thead><tr><th>Duty %</th><th>RPM</th><th>F brute N</th><th>Statut</th></tr></thead>
   <tbody id="sw-tbody"></tbody>
  </table>
  <div style="margin-top:10px;font-size:11px;color:#3f3;text-align:center" id="sw-saved-lbl" style="display:none"></div>
 </div>
</div>

</div><!-- /content -->

<!-- Nav -->
<div class="nav">
 <button class="btn btn-sec" id="btn-prev" onclick="prevStep()" style="display:none">&larr; Precedent</button>
 <button class="btn btn-go" id="btn-next" onclick="nextStep()">Etape suivante &rarr;</button>
</div>

<script>
let step=1;
let calib={offset:0,scale:1,fanDragTable:[]};

// SSE pour lecture live
const evs=new EventSource("/events");
let lastState={};
evs.onmessage=e=>{
 lastState=JSON.parse(e.data);
 updateLive();
};

function updateLive(){
 const s=lastState;
 if(!s.raw) return;
 // Step 1
 document.getElementById("raw-disp1").innerHTML=s.raw+' <span class="unit">ADU</span>';
 const fanOk=(s.fan_duty||0)<2;
 const sigma=s.force_std||1;
 const sigmaG=sigma/9.80665e-3;
 const stabPct=Math.max(0,Math.min(100,(1-sigmaG/5)*100));
 document.getElementById("stab-fill1").style.width=stabPct+"%";
 document.getElementById("stab-fill1").style.background=stabPct>70?"#3f3":stabPct>40?"#fa0":"#f55";
 document.getElementById("stab-lbl1").textContent="σ="+sigmaG.toFixed(2)+"g";
 document.getElementById("fan-chip1").className="chip "+(fanOk?"ok":"warn");
 document.getElementById("fan-chip1").textContent=(fanOk?"✓ Ventilo OFF":"✗ Ventilo ON "+Math.round(s.fan_duty||0)+"%");
 document.getElementById("stab-chip1").className="chip "+(stabPct>70?"ok":stabPct>40?"warn":"err");
 document.getElementById("stab-chip1").textContent=(stabPct>70?"✓ Stable":"≈ Instable");
 // Step 2
 if(s.raw){
  document.getElementById("raw-disp2").innerHTML=s.raw+' <span class="unit">ADU</span>';
  const delta=s.raw-(+document.getElementById("off-used")?.textContent||calib.offset);
  document.getElementById("delta-disp").textContent=(s.delta||0).toFixed(0)+" ADU";
  const mass=+document.getElementById("mass-inp").value||100;
  const scLive=mass>0?(s.delta||0)/mass:0;
  document.getElementById("sc-live").textContent=scLive.toFixed(2)+" ADU/g";
 }
}

function goStep(n){
 document.querySelectorAll(".panel").forEach(p=>p.classList.remove("on"));
 document.getElementById("p-"+n).classList.add("on");
 for(let i=1;i<=3;i++){
  const el=document.getElementById("si-"+i);
  el.className="si"+(i===n?" active":i<n?" done":"");
 }
 document.getElementById("btn-prev").style.display=n>1?"":"none";
 document.getElementById("btn-next").style.display=n<3?"":"none";
 step=n;
 if(n===3) loadSweepTable();
}
function nextStep(){if(step<3)goStep(step+1);}
function prevStep(){if(step>1)goStep(step-1);}

// Step 1 : tare
async function doStep1(){
 const btn=document.getElementById("btn1");
 const st=document.getElementById("st1");
 btn.disabled=true;st.style.color="#fa0";st.textContent="Capture en cours...";
 const r=await fetch("/calib/tare",{method:"POST"}).then(x=>x.json()).catch(()=>null);
 btn.disabled=false;
 if(!r){st.style.color="#f55";st.textContent="Erreur réseau";return;}
 calib.offset=r.offset;
 st.style.color="#3f3";st.textContent="✓ Zero capture : offset = "+r.offset+" ADU — sauvegarde dans calib.json";
 document.getElementById("si-1").className="si done";
 refreshSummary();
}

// Step 2 : scale
async function doStep2(){
 const m=+document.getElementById("mass-inp").value;
 if(!m||m<=0){alert("Entrez une masse valide");return;}
 const btn=document.getElementById("btn2");
 const st=document.getElementById("st2");
 btn.disabled=true;st.style.color="#fa0";st.textContent="Calibration "+m+"g...";
 const r=await fetch("/calib/scale?mass_g="+m,{method:"POST"}).then(x=>x.json()).catch(()=>null);
 btn.disabled=false;
 if(!r){st.style.color="#f55";st.textContent="Erreur réseau";return;}
 calib.scale=r.scale;
 st.style.color="#3f3";st.textContent="✓ Scale = "+r.scale.toFixed(4)+" ADU/g — sauvegarde dans calib.json";
 document.getElementById("si-2").className="si done";
 refreshSummary();
}

// Step 3 : sweep
async function doSweep(){
 const dmin=+document.getElementById("sw-min").value;
 const dmax=+document.getElementById("sw-max").value;
 const pts=+document.getElementById("sw-pts").value;
 const stab=+document.getElementById("sw-stab").value;
 const btn=document.getElementById("btn3");
 const st=document.getElementById("st3");
 btn.disabled=true;
 document.getElementById("sweep-running").style.display="block";
 st.style.color="#fa0";st.textContent="Sweep en cours... (environ "+(pts*(stab+3))+"s)";
 await fetch("/calib/fan_sweep?dmin="+dmin+"&dmax="+dmax+"&pts="+pts+"&stab="+stab,{method:"POST"});
 const pollId=setInterval(async()=>{
  const s=await fetch("/calib/status").then(r=>r.json()).catch(()=>null);
  if(!s)return;
  document.getElementById("sw-prog").style.width=Math.round(s.progress*100)+"%";
  document.getElementById("sw-cur-duty").textContent=(s.current_duty||0)+"%";
  document.getElementById("sw-cur-F").textContent=(s.current_F||0).toFixed(4)+" N";
  document.getElementById("sw-phase-lbl").textContent=s.msg||"";
  if(s.table&&s.table.length>0) renderTable(s.table);
  if(s.phase==="done"){
   clearInterval(pollId);
   calib.fanDragTable=s.table;
   document.getElementById("sweep-running").style.display="none";
   document.getElementById("sw-saved-lbl").textContent="✓ Courbe sauvegardee ("+s.table.length+" points) — calib.json";
   document.getElementById("sw-saved-lbl").style.display="block";
   document.getElementById("btn-reset").style.display="block";
   document.getElementById("sweep-results").style.display="block";
   st.style.color="#3f3";st.textContent="✓ Sweep termine — "+s.table.length+" points — soustraction activee automatiquement";
   btn.disabled=false;
   document.getElementById("si-3").className="si done";
   refreshSummary();
   drawChart(s.table);
  }else if(s.phase==="error"){
   clearInterval(pollId);
   document.getElementById("sweep-running").style.display="none";
   st.style.color="#f55";st.textContent="Erreur : "+s.msg;
   btn.disabled=false;
  }
 },700);
}

function renderTable(table){
 const tbody=document.getElementById("sw-tbody");
 tbody.innerHTML="";
 table.forEach(p=>{
  tbody.innerHTML+=`<tr><td>${p.duty}%</td><td>${p.rpm||"—"}</td><td class="f-val">${(+p.F_N).toFixed(4)}</td><td class="chip ok" style="font-size:10px">✓</td></tr>`;
 });
 document.getElementById("sweep-results").style.display="block";
}

function drawChart(table){
 if(!table||table.length<2)return;
 const cv=document.getElementById("chart");
 cv.width=cv.offsetWidth||600;cv.height=140;
 const cx=cv.getContext("2d"),W=cv.width,H=cv.height;
 cx.fillStyle="#060616";cx.fillRect(0,0,W,H);
 const ML=52,MR=14,MT=14,MB=24;
 const PW=W-ML-MR,PH=H-MT-MB;
 const duties=table.map(p=>p.duty);
 const forces=table.map(p=>+p.F_N);
 const xmin=Math.min(...duties),xmax=Math.max(...duties);
 const ymin=0,ymax=Math.max(...forces)*1.2||0.1;
 const px=d=>ML+PW*(d-xmin)/(xmax-xmin||1);
 const py=f=>MT+PH*(1-f/ymax);
 // grid
 cx.strokeStyle="#111";cx.lineWidth=1;
 [0.25,0.5,0.75,1].forEach(t=>{
  const y=MT+PH*t;cx.beginPath();cx.moveTo(ML,y);cx.lineTo(W-MR,y);cx.stroke();
  cx.fillStyle="#334";cx.font="9px monospace";cx.textAlign="right";
  cx.fillText((ymax*(1-t)).toExponential(1)+"N",ML-3,y+3);
 });
 // labels
 cx.fillStyle="#445";cx.font="9px monospace";cx.textAlign="center";
 duties.forEach(d=>cx.fillText(d+"%",px(d),H-6));
 // area fill
 cx.beginPath();cx.moveTo(px(duties[0]),py(0));
 table.forEach(p=>cx.lineTo(px(p.duty),py(+p.F_N)));
 cx.lineTo(px(duties[duties.length-1]),py(0));cx.closePath();
 cx.fillStyle="rgba(100,170,255,0.07)";cx.fill();
 // line
 cx.beginPath();cx.strokeStyle="#4af";cx.lineWidth=2;
 table.forEach((p,i)=>i===0?cx.moveTo(px(p.duty),py(+p.F_N)):cx.lineTo(px(p.duty),py(+p.F_N)));
 cx.stroke();
 // dots
 table.forEach(p=>{
  cx.beginPath();cx.arc(px(p.duty),py(+p.F_N),4,0,Math.PI*2);
  cx.fillStyle="#4af";cx.fill();
 });
}

async function resetSweep(){
 if(!confirm("Effacer la courbe de trainee ?"))return;
 await fetch("/calib/reset_sweep",{method:"POST"});
 document.getElementById("sw-tbody").innerHTML="";
 document.getElementById("sweep-results").style.display="none";
 document.getElementById("btn-reset").style.display="none";
 document.getElementById("sw-saved-lbl").style.display="none";
 document.getElementById("st3").textContent="";
 refreshSummary();
}

async function loadSweepTable(){
 const s=await fetch("/calib/status").then(r=>r.json()).catch(()=>null);
 if(s&&s.table&&s.table.length>0){
  renderTable(s.table);drawChart(s.table);
  document.getElementById("btn-reset").style.display="block";
  document.getElementById("sw-saved-lbl").textContent="✓ Courbe existante chargee ("+s.table.length+" points)";
  document.getElementById("sw-saved-lbl").style.display="block";
 }
}

async function refreshSummary(){
 const s=await fetch("/events",{headers:{"Accept":"text/event-stream"}}).catch(()=>null);
 // Use lastState instead
 const d=lastState;
 if(!d.offset&&d.offset!==0)return;
 document.getElementById("calib-summary").style.display="block";
 document.getElementById("sum-offset").textContent=d.offset||"—";
 document.getElementById("sum-sc").textContent=d.scale?(+d.scale).toFixed(4):"—";
 const pts=d.fan_drag_table?d.fan_drag_table.length:0;
 document.getElementById("sum-drag-pts").textContent=pts>0?(pts+" points"):"Non calibree";
 if(pts>0){document.getElementById("sum-drag").className="state-item done-item";}
}

// Init
setTimeout(refreshSummary,1000);
goStep(1);

// Annuler la calibration si l'utilisateur quitte la page
window.addEventListener("beforeunload",()=>{navigator.sendBeacon("/calib/cancel");});
document.addEventListener("visibilitychange",()=>{if(document.hidden)navigator.sendBeacon("/calib/cancel");});
</script></body></html>"""


# ==================== ETUDE COMPLETE HTML ====================

ETUDE_HTML = r"""<!DOCTYPE html>
<html lang="fr"><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Etude — Soufflerie ENSAM</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#080c10;color:#dde;font-family:ui-monospace,Menlo,monospace;min-height:100vh;padding:0}
header{display:flex;align-items:center;gap:12px;padding:12px 16px;border-bottom:1px solid #1a2a3a;background:#0a1018}
header h1{flex:1;color:#4af;font-size:15px;letter-spacing:2px;text-transform:uppercase}
a.back{color:#4af;text-decoration:none;font-size:12px}
a.etude-dash{color:#4af;font-size:11px;text-decoration:none;border:1px solid #1a4a99;padding:4px 10px;border-radius:3px}
.stepbar{display:flex;padding:14px 16px;gap:0;background:#0d1520;border-bottom:1px solid #1a2a3a;overflow-x:auto}
.stepitem{display:flex;align-items:center;flex:1;min-width:55px}
.stepnum{width:24px;height:24px;border-radius:50%;display:flex;align-items:center;justify-content:center;font-size:11px;font-weight:bold;background:#1a2030;color:#555;border:2px solid #222;flex-shrink:0;transition:all .3s}
.steplabel{font-size:9px;color:#555;margin-left:5px;letter-spacing:1px;text-transform:uppercase;transition:color .3s}
.stepitem.done .stepnum{background:#0a3a1a;color:#4f4;border-color:#2a6a3a}
.stepitem.done .steplabel{color:#4f4}
.stepitem.active .stepnum{background:#1a4a99;color:#fff;border-color:#4af;box-shadow:0 0 8px #4af5}
.stepitem.active .steplabel{color:#4af}
.stepconn{flex:1;height:2px;background:#1a2030;margin:0 3px;min-width:6px}
.content{padding:16px;max-width:700px;margin:auto}
.panel{display:none}.panel.on{display:block}
.card{background:#0f1720;border:1px solid #1a2a3a;border-radius:8px;padding:16px;margin-bottom:12px}
.card h2{color:#4af;font-size:11px;letter-spacing:2px;text-transform:uppercase;margin-bottom:10px;padding-bottom:8px;border-bottom:1px solid #1a2a3a}
.instr{background:#060e18;border-left:3px solid #4af;padding:12px;border-radius:0 4px 4px 0;margin-bottom:12px;font-size:13px;line-height:1.6;color:#bcd}
.instr b{color:#4af}
.instr-smoke{background:#060e18;border-left:3px solid #4f4;padding:12px;border-radius:0 4px 4px 0;margin-bottom:12px;font-size:13px;line-height:1.6;color:#bcd}
.instr-smoke b{color:#4f4}
.row{display:flex;justify-content:space-between;align-items:center;padding:6px 0;font-size:13px;gap:10px}
.lab{color:#888;flex:1}
.val{color:#4af;font-weight:bold}
input[type=number]{width:90px;background:#060e18;color:#eee;border:1px solid #2a3a4a;border-radius:3px;padding:5px 8px;font-family:inherit;font-size:13px}
.btn-primary{padding:10px 20px;background:#1a4a99;color:#fff;border:none;border-radius:5px;cursor:pointer;font-family:inherit;font-size:13px;font-weight:bold;letter-spacing:1px;transition:all .2s;width:100%}
.btn-primary:hover:not(:disabled){background:#2a5ab0}
.btn-primary:disabled{opacity:.4;cursor:not-allowed}
.btn-secondary{padding:8px 16px;background:#1a2030;color:#4af;border:1px solid #2a4a70;border-radius:5px;cursor:pointer;font-family:inherit;font-size:12px}
.btn-ready{padding:14px 20px;background:#0a2a1a;color:#4f4;border:2px solid #2a6a3a;border-radius:5px;cursor:pointer;font-family:inherit;font-size:14px;font-weight:bold;width:100%;margin-top:8px;letter-spacing:1px;transition:all .2s}
.btn-ready:hover:not(:disabled){background:#0a3a2a}
.btn-ready:disabled{opacity:.4;cursor:not-allowed}
.status{font-size:12px;min-height:16px;margin-top:8px;transition:color .3s}
.st-ok{color:#4f4}.st-err{color:#f55}.st-wait{color:#fa0}
.check{display:flex;align-items:center;gap:8px;font-size:12px;padding:4px 0}
.check-icon{width:18px;height:18px;border-radius:50%;background:#1a2030;display:flex;align-items:center;justify-content:center;font-size:11px}
.check-icon.ok{background:#0a2a0a;color:#4f4}
.check-icon.ko{background:#2a0a0a;color:#f55}
.check-icon.wait{background:#1a1a2a;color:#fa0}
.progress-outer{background:#1a2030;border-radius:4px;height:8px;overflow:hidden;margin:10px 0}
.progress-inner{height:100%;background:linear-gradient(90deg,#1a4a99,#4af);border-radius:4px;transition:width .4s}
.progress-smoke{height:100%;background:linear-gradient(90deg,#1a6a3a,#4f4);border-radius:4px;transition:width .3s}
.rtable{width:100%;border-collapse:collapse;font-size:12px}
.rtable th{color:#4af;text-align:left;padding:5px 8px;border-bottom:1px solid #1a2a3a;font-size:9px;letter-spacing:1px;text-transform:uppercase}
.rtable td{padding:5px 8px;border-bottom:1px solid #111}
.rtable tr:last-child td{border:none}
.rtable .unit{font-size:10px;color:#555}
.rtable .dir-up{color:#4af;font-weight:bold}.rtable .dir-dn{color:#fa0;font-weight:bold}
.nav{display:flex;gap:10px;padding:16px;max-width:700px;margin:auto}
.nav .btn-secondary{flex:1}
.nav .btn-primary{flex:2}
.live-box{flex:1;text-align:center;background:#060e18;border-radius:6px;padding:8px}
.live-lbl{font-size:9px;color:#556;letter-spacing:2px;text-transform:uppercase;margin-bottom:4px}
.live-val{font-size:16px;font-weight:bold;color:#4af;padding:2px;letter-spacing:1px}
.mode-chip{display:inline-block;padding:3px 10px;border-radius:12px;font-size:10px;font-weight:bold;letter-spacing:1px;text-transform:uppercase;border:1px solid #2a4a70;color:#4af;background:#0a1a30;cursor:pointer;margin:3px}
.mode-chip.sel{background:#1a4a99;border-color:#4af;color:#fff}
.smoke-cd{font-size:32px;font-weight:bold;color:#4f4;text-align:center;padding:16px;letter-spacing:4px;font-variant-numeric:tabular-nums}
.snap-img{width:100%;border-radius:4px;border:1px solid #2a3a4a;margin-bottom:10px;display:block}
.video-wrap{background:#000;border-radius:4px;overflow:hidden;margin-bottom:10px}
.video-wrap video{width:100%;display:block;max-height:220px}
</style></head><body>
<header>
 <a class="back" href="/">&larr; Dashboard</a>
 <h1>Etude Complete</h1>
 <a class="etude-dash" href="/settings">Parametres</a>
</header>

<div class="stepbar">
 <div class="stepitem active" id="si-1"><div class="stepnum">1</div><div class="steplabel">Config</div></div>
 <div class="stepconn"></div>
 <div class="stepitem" id="si-2"><div class="stepnum">2</div><div class="steplabel">Tare objet</div></div>
 <div class="stepconn"></div>
 <div class="stepitem" id="si-3"><div class="stepnum">3</div><div class="steplabel">Mesure</div></div>
 <div class="stepconn"></div>
 <div class="stepitem" id="si-4"><div class="stepnum">4</div><div class="steplabel">Fumee</div></div>
 <div class="stepconn"></div>
 <div class="stepitem" id="si-5"><div class="stepnum">5</div><div class="steplabel">Resultats</div></div>
</div>

<div class="content">

<!-- ===== STEP 1 : Config ===== -->
<div id="p-1" class="panel on">
 <div class="card">
  <h2>1 — Choisir l objet</h2>
  <div style="display:grid;grid-template-columns:repeat(5,1fr);gap:8px;margin-bottom:16px">
   <div class="eobj" id="eo-airfoil" onclick="etudeSetObj('airfoil')" style="background:#0f1720;border:2px solid #1a4a99;border-radius:8px;padding:10px 4px;text-align:center;cursor:pointer">
    <svg viewBox="0 0 60 30" width="100%" height="36"><path d="M5,15 Q20,3 40,14 Q50,18 55,15 Q50,23 40,17 Q20,28 5,15Z" fill="#4af" opacity=".8"/></svg>
    <div style="font-size:9px;color:#4af;margin-top:4px">PROFIL</div><div style="font-size:8px;color:#4af;margin-top:2px">CL/CD</div>
   </div>
   <div class="eobj" id="eo-plate" onclick="etudeSetObj('plate')" style="background:#0f1720;border:2px solid #222;border-radius:8px;padding:10px 4px;text-align:center;cursor:pointer">
    <svg viewBox="0 0 60 30" width="100%" height="36"><rect x="8" y="9" width="44" height="12" rx="1" fill="#4af" opacity=".6"/></svg>
    <div style="font-size:9px;color:#666;margin-top:4px">PLAQUE</div><div style="font-size:8px;color:#555;margin-top:2px">CD</div>
   </div>
   <div class="eobj" id="eo-cube" onclick="etudeSetObj('cube')" style="background:#0f1720;border:2px solid #222;border-radius:8px;padding:10px 4px;text-align:center;cursor:pointer">
    <svg viewBox="0 0 60 44" width="100%" height="36"><path d="M15,22 L30,14 L45,22 L45,34 L30,42 L15,34Z" fill="#4af" opacity=".2"/><path d="M15,22 L30,30 L30,42 L15,34Z" fill="#4af" opacity=".4"/><path d="M30,14 L45,22 L45,34 L30,42 L30,30Z" fill="#4af" opacity=".35"/><path d="M15,22 L30,30 L45,22 L30,14Z" fill="#4af" opacity=".65"/></svg>
    <div style="font-size:9px;color:#666;margin-top:4px">CUBE</div><div style="font-size:8px;color:#555;margin-top:2px">CD≈1.05</div>
   </div>
   <div class="eobj" id="eo-sphere" onclick="etudeSetObj('sphere')" style="background:#0f1720;border:2px solid #222;border-radius:8px;padding:10px 4px;text-align:center;cursor:pointer">
    <svg viewBox="0 0 60 44" width="100%" height="36"><circle cx="30" cy="22" r="17" fill="#4af" opacity=".35"/><ellipse cx="30" cy="22" rx="17" ry="6" stroke="#4af" stroke-width="1" fill="none" opacity=".7"/><circle cx="30" cy="22" r="17" stroke="#4af" stroke-width="1.5" fill="none" opacity=".5"/></svg>
    <div style="font-size:9px;color:#666;margin-top:4px">SPHERE</div><div style="font-size:8px;color:#555;margin-top:2px">CD≈0.47</div>
   </div>
   <div class="eobj" id="eo-cylinder" onclick="etudeSetObj('cylinder')" style="background:#0f1720;border:2px solid #222;border-radius:8px;padding:10px 4px;text-align:center;cursor:pointer">
    <svg viewBox="0 0 60 44" width="100%" height="36"><ellipse cx="30" cy="10" rx="16" ry="5" fill="#4af" opacity=".7"/><rect x="14" y="10" width="32" height="22" fill="#4af" opacity=".35"/><ellipse cx="30" cy="32" rx="16" ry="5" fill="#4af" opacity=".5"/></svg>
    <div style="font-size:9px;color:#666;margin-top:4px">CYLINDRE</div><div style="font-size:8px;color:#555;margin-top:2px">CD≈1.2</div>
   </div>
  </div>
  <h2 id="ecfg-title">Parametres</h2>
  <div class="row"><span class="lab" id="ecfg-p1-lab">Corde c</span>
   <input id="cfg-p1" type="number" step="0.001" value="0.100"> m</div>
  <div class="row" id="ecfg-p2-row"><span class="lab" id="ecfg-p2-lab">Envergure b</span>
   <input id="cfg-p2" type="number" step="0.001" value="0.150"> m</div>
  <div class="row" id="ecfg-aoa-row"><span class="lab">Angle attaque &#945;</span>
   <input id="cfg-aoa" type="number" step="0.5" value="0"> deg</div>
 </div>

 <div class="card">
  <h2>Mode mesure</h2>
  <div style="display:flex;flex-wrap:wrap;gap:6px;margin-bottom:12px">
   <span class="mode-chip sel" id="mc-single"       onclick="setMode('single')">Vitesse unique</span>
   <span class="mode-chip"     id="mc-sweep"        onclick="setMode('sweep')">Sweep &#8593;</span>
   <span class="mode-chip"     id="mc-sweep_double" onclick="setMode('sweep_double')">Sweep &#8593;&#8595; double</span>
   <span class="mode-chip"     id="mc-polar"        onclick="setMode('polar')">Polaire &#945;</span>
  </div>
  <!-- Single -->
  <div id="cfg-single">
   <div class="row"><span class="lab">Duty ventilo</span>
    <input id="cfg-duty" type="number" min="20" max="100" value="50"> %</div>
   <div class="row"><span class="lab">Duree acquisition</span>
    <input id="cfg-dur" type="number" min="5" max="120" value="30"> s</div>
  </div>
  <!-- Sweep (simple ou double) -->
  <div id="cfg-sweep" style="display:none">
   <!-- Variable de sweep -->
   <div style="margin-bottom:10px">
    <div style="font-size:9px;color:#556;letter-spacing:2px;text-transform:uppercase;margin-bottom:5px">Variable</div>
    <div style="display:flex;gap:6px">
     <span id="sv-duty" onclick="setSweepVar('duty')" style="padding:4px 12px;border-radius:4px;cursor:pointer;font-size:12px;border:1px solid #5af;color:#5af;background:#0a1e30">Duty %</span>
     <span id="sv-rpm"  onclick="setSweepVar('rpm')"  style="padding:4px 12px;border-radius:4px;cursor:pointer;font-size:12px;border:1px solid #2a2a2a;color:#888;background:#111">RPM cible ⚡</span>
    </div>
    <div style="font-size:10px;color:#556;margin-top:4px" id="sv-hint">Mode RPM : asservissement tachymètre — points régulièrement espacés en vitesse.</div>
   </div>
   <div style="display:grid;grid-template-columns:1fr 1fr;gap:8px">
    <!-- Duty inputs -->
    <div id="sw-row-dmin"><div class="sw-lbl">Duty min %</div>
     <input id="sw-dmin" type="number" min="20" max="80"  value="20" style="width:100%"></div>
    <div id="sw-row-dmax"><div class="sw-lbl">Duty max %</div>
     <input id="sw-dmax" type="number" min="30" max="95"  value="80" style="width:100%"></div>
    <!-- RPM inputs (cachés par défaut) -->
    <div id="sw-row-rmin" style="display:none"><div class="sw-lbl">RPM min</div>
     <input id="sw-rmin" type="number" min="500"  max="3000" value="800"  style="width:100%"></div>
    <div id="sw-row-rmax" style="display:none"><div class="sw-lbl">RPM max</div>
     <input id="sw-rmax" type="number" min="1000" max="3500" value="3000" style="width:100%"></div>
    <!-- Communs -->
    <div><div class="sw-lbl">Nb points</div>
     <input id="sw-pts"  type="number" min="3" max="12" value="6"  style="width:100%"></div>
    <div><div class="sw-lbl">Stab. (s)</div>
     <input id="sw-stab" type="number" min="2" max="15" value="5"  style="width:100%"></div>
    <div><div class="sw-lbl">Acq. / pt (s)</div>
     <input id="sw-dur"  type="number" min="5" max="60"  value="10" style="width:100%"></div>
    <div><div class="sw-lbl" title="Plafond duty — évite le saut brutal à 100%">Max duty % ⚠</div>
     <input id="sw-maxduty" type="number" min="50" max="100" value="95" style="width:100%"></div>
   </div>
   <div id="double-note" style="display:none;font-size:11px;color:#fa0;margin-top:8px;padding:6px;background:#060e18;border-radius:3px">
    &#8593;&#8595; Double sweep : montée dmin&#8594;dmax puis descente dmax&#8594;dmin. Détecte l'hystérésis.
   </div>
  </div>
  <!-- Polaire -->
  <div id="cfg-polar" style="display:none">
   <div style="font-size:11px;color:#778;margin-bottom:10px;line-height:1.5">
    Balayage en angle d'attaque &#945; — chaque angle est réglé <b>manuellement</b>, puis validé.<br>
    Photo prise automatiquement à chaque point.
   </div>
   <div style="display:grid;grid-template-columns:1fr 1fr;gap:8px">
    <div><div class="sw-lbl">&#945; min °</div><input id="polar-amin" type="number" step="1" value="-10" style="width:100%" oninput="updatePolarPreview()"></div>
    <div><div class="sw-lbl">&#945; max °</div><input id="polar-amax" type="number" step="1" value="20" style="width:100%" oninput="updatePolarPreview()"></div>
    <div><div class="sw-lbl">Pas °</div><input id="polar-step" type="number" step="0.5" min="0.5" value="5" style="width:100%" oninput="updatePolarPreview()"></div>
    <div><div class="sw-lbl">Duty ventilo %</div><input id="polar-duty" type="number" min="20" max="95" value="60" style="width:100%"></div>
    <div><div class="sw-lbl">Stab. (s)</div><input id="polar-stab" type="number" min="2" max="20" value="6" style="width:100%"></div>
    <div><div class="sw-lbl">Acq. / angle (s)</div><input id="polar-dur" type="number" min="5" max="60" value="15" style="width:100%"></div>
   </div>
   <div class="row" style="margin-top:8px">
    <span class="lab">Tare force entre chaque angle</span>
    <input type="checkbox" id="polar-tare" checked style="width:auto;accent-color:#4af">
   </div>
   <div id="polar-angles-preview" style="font-size:11px;color:#4af;margin-top:8px;line-height:1.6"></div>
  </div>
 </div>

 <div class="card">
  <h2>&#128168; Fumee (etape 4)</h2>
  <div class="row"><span class="lab">Duty ventilo (ecoulement lent)</span>
   <input id="cfg-smoke-duty" type="number" min="0" max="40" value="15"> %</div>
  <div class="row"><span class="lab">Duree enregistrement</span>
   <input id="cfg-smoke-dur"  type="number" min="5" max="120" value="20"> s</div>
  <div style="font-size:11px;color:#556;margin-top:4px">Apres la mesure : ventilateur lent, vous preparez la fumee, puis enregistrement video automatique.</div>
 </div>

 <div class="card">
  <h2>Source de vitesse</h2>
  <div style="display:flex;flex-wrap:wrap;gap:6px;margin-bottom:8px">
   <span class="mode-chip sel" id="vs-kalman"  onclick="setVSrc('kalman')">Kalman (fusion)</span>
   <span class="mode-chip"    id="vs-fan"     onclick="setVSrc('fan')">Ventilateur (KV&#215;RPM)</span>
   <span class="mode-chip"    id="vs-pressure" onclick="setVSrc('pressure')">Pression BMP280</span>
   <span class="mode-chip"    id="vs-manual"   onclick="setVSrc('manual')">Manuel</span>
  </div>
  <div id="vs-manual-row" style="display:none;margin:6px 0;align-items:center;gap:8px">
   <span style="font-size:12px;color:#aaa">Vitesse :</span>
   <input id="vs-manual-val" type="number" min="0" max="50" step="0.1" value="0"
    style="width:80px;background:#0a0a1a;border:1px solid #2a2a4a;color:#eee;padding:3px 6px;border-radius:4px;font-size:13px"
    oninput="applyManualV()">
   <span style="font-size:12px;color:#aaa">m/s</span>
  </div>
  <div id="vs-desc" style="font-size:11px;color:#778;line-height:1.5"></div>
  <div id="vs-bmp-warn" style="display:none;font-size:11px;color:#fa0;margin-top:4px">
   &#9888; BMP280 non détecté — source pression indisponible
  </div>
 </div>

 <div class="card">
  <h2>Protocole</h2>
  <div class="instr">
   <b>Etape 2</b> — Monter l objet sur le support, ventilo eteint &#8594; tare mecanique auto.<br>
   <b>Etape 3</b> — Mesure aero (vitesse unique, sweep &#8593; ou sweep &#8593;&#8595;).<br>
   <b>Etape 4</b> — Fumee : ventilateur lent, preparez la fumee, enregistrement video.<br>
   <b>Etape 5</b> — Rapport : donnees + capture fumee + video + analyse flux optique.
  </div>
  <div id="proto-drag-status" style="margin-top:8px;font-size:12px;padding:8px;border-radius:4px;background:#060e18"></div>
 </div>
</div>

<!-- ===== STEP 2 : Tare avec objet ===== -->
<div id="p-2" class="panel">
 <div class="card">
  <h2>2 — Tare avec objet monte</h2>
  <div class="instr">
   <b>Action :</b> Montez l <b id="obj-name-step2">objet</b> sur le support dans le tunnel.<br>
   Ventilateur eteint. La tare capture le poids propre de l objet (zero mecanique).
  </div>
  <div id="ck-fan2" class="check"><span class="check-icon wait" id="ck-fan2-i">?</span> Ventilateur eteint</div>
  <div id="ck-sig2" class="check"><span class="check-icon wait" id="ck-sig2-i">?</span> Signal force stable</div>
  <div id="auto-tare-cd2" style="display:none;margin-top:8px;font-size:13px;color:#fa0;text-align:center">
   Tare dans <b id="cd2-secs">3</b> s...
  </div>
  <div style="margin-top:12px"></div>
  <button class="btn-primary" onclick="doTare2()" id="btn-tare2">TARER AVEC OBJET</button>
  <div class="status" id="st-tare2"></div>
 </div>
</div>

<!-- ===== STEP 3 : Mesure ===== -->
<div id="p-3" class="panel">
 <div class="card" style="padding:0;overflow:hidden;background:#000">
  <img src="/camera" style="width:100%;display:block;max-height:240px;object-fit:contain" alt="camera live">
 </div>
 <div class="card">
  <h2 id="step3-title">3 — Mesure</h2>
  <div class="instr" id="step3-instr">Ventilateur demarre — lancez quand le flux est etabli.</div>
  <div style="display:flex;gap:10px;margin-bottom:12px">
   <div class="live-box"><div class="live-lbl">Force nette</div><div class="live-val" id="live-F">— N</div></div>
   <div class="live-box"><div class="live-lbl">Vitesse air</div><div class="live-val" id="live-V">— m/s</div></div>
   <div class="live-box"><div class="live-lbl">Ventilo</div><div class="live-val" id="live-duty" style="color:#4f4">— %</div></div>
  </div>
  <button class="btn-primary" onclick="doMeasure()" id="btn-measure">LANCER LA MESURE</button>
  <div class="progress-outer" id="pg-meas" style="display:none">
   <div class="progress-inner" id="pg-meas-bar" style="width:0%"></div>
  </div>
  <div class="status" id="st-meas"></div>
 </div>
 <!-- Sweep live -->
 <div class="card" id="sweep-live-card" style="display:none">
  <h2>Sweep en cours</h2>
  <div style="font-size:12px;color:#fa0;margin-bottom:6px" id="sweep-live-msg">—</div>
  <div class="progress-outer"><div class="progress-inner" id="sw-live-bar" style="width:0%"></div></div>
  <table class="rtable" style="margin-top:8px">
   <thead><tr><th></th><th>Duty %</th><th>V m/s</th><th>F nette N</th><th>CD/CL</th><th>Re</th></tr></thead>
   <tbody id="sw-live-tbody"></tbody>
  </table>
 </div>

 <!-- Polaire : carte angle par angle -->
 <div class="card" id="polar-angle-card" style="display:none;border-color:#2a6a3a">
  <h2 id="polar-angle-title" style="color:#4f4">Polaire — angle 1/N</h2>
  <div style="text-align:center;padding:20px 0 16px">
   <div style="font-size:10px;color:#556;letter-spacing:3px;text-transform:uppercase;margin-bottom:8px">Régler l'angle sur</div>
   <div style="font-size:56px;font-weight:bold;color:#4f4;letter-spacing:4px;font-variant-numeric:tabular-nums" id="polar-angle-val">0°</div>
   <div style="font-size:11px;color:#778;margin-top:8px">Ajustez mécaniquement puis validez</div>
  </div>
  <div style="display:flex;gap:10px;margin-bottom:12px">
   <div class="live-box"><div class="live-lbl">Force nette</div><div class="live-val" id="polar-live-F">— N</div></div>
   <div class="live-box"><div class="live-lbl">Vitesse</div><div class="live-val" id="polar-live-V">— m/s</div></div>
   <div class="live-box"><div class="live-lbl">RPM</div><div class="live-val" id="polar-live-rpm" style="color:#4f4">—</div></div>
  </div>
  <button class="btn-ready" id="btn-polar-confirm" onclick="confirmPolarAngle()">
   ✓ ANGLE RÉGLÉ — MESURER
  </button>
  <div class="progress-outer" id="polar-pg-outer" style="display:none;margin-top:8px">
   <div class="progress-inner" id="polar-pg-bar" style="width:0%"></div>
  </div>
  <div style="font-size:12px;min-height:18px;margin-top:6px;transition:color .3s" id="polar-st"></div>
  <!-- Résultats partiels -->
  <div id="polar-partial-wrap" style="display:none;margin-top:12px">
   <div style="font-size:9px;color:#556;letter-spacing:2px;text-transform:uppercase;margin-bottom:6px">Angles mesurés</div>
   <table class="rtable">
    <thead><tr><th>&#945;°</th><th>V m/s</th><th>F nette N</th><th>CL/CD</th><th>Re</th></tr></thead>
    <tbody id="polar-partial-tbody"></tbody>
   </table>
  </div>
 </div>
</div>

<!-- ===== STEP 4 : Fumee ===== -->
<div id="p-4" class="panel">
 <div class="card" style="padding:0;overflow:hidden;background:#000">
  <img src="/camera" style="width:100%;display:block;max-height:280px;object-fit:contain" alt="camera live">
 </div>

 <!-- Phase preparation -->
 <div class="card" id="smoke-prep-card">
  <h2>4 — Visualisation fumee</h2>
  <div class="instr-smoke">
   Ventilateur a <b id="smoke-duty-disp">15</b>% — ecoulement lent pour les lignes de courant.<br>
   <b>Preparez votre fumee</b> puis appuyez sur le bouton quand vous etes pret a enregistrer.
  </div>
  <div style="display:flex;gap:10px;margin-bottom:12px">
   <div class="live-box"><div class="live-lbl">Ventilo</div><div class="live-val" id="smoke-live-duty" style="color:#4f4">— %</div></div>
   <div class="live-box"><div class="live-lbl">Force</div><div class="live-val" id="smoke-live-F">— N</div></div>
  </div>
  <button class="btn-ready" id="btn-smoke-ready" onclick="startSmokeRec()">
   &#128168; JE SUIS PRET — LANCER L ENREGISTREMENT
  </button>
  <div style="text-align:center;margin-top:10px">
   <button onclick="skipSmoke()" style="background:none;border:none;color:#444;font-size:11px;cursor:pointer;text-decoration:underline">Passer cette etape</button>
  </div>
  <div class="status" id="st-smoke"></div>
 </div>

 <!-- Phase enregistrement -->
 <div class="card" id="smoke-rec-card" style="display:none;border-color:#2a6a3a">
  <h2 style="color:#4f4">&#9679; Enregistrement en cours</h2>
  <div class="smoke-cd" id="smoke-cd">20 s</div>
  <div class="progress-outer">
   <div class="progress-smoke" id="smoke-pg" style="width:0%"></div>
  </div>
  <div class="status st-wait" id="st-smoke-rec">Preparation...</div>
  <button class="btn-secondary" onclick="stopSmokeEarly()" style="width:100%;margin-top:10px;color:#fa0;border-color:#4a3a0a">
   Arreter maintenant
  </button>
 </div>

 <!-- Phase conversion -->
 <div class="card" id="smoke-conv-card" style="display:none">
  <div class="status st-wait" id="st-smoke-conv">Conversion video en cours...</div>
  <div class="progress-outer"><div class="progress-inner" id="smoke-conv-pg" style="width:30%;animation:pulse 1s infinite alternate"></div></div>
 </div>
</div>

<!-- ===== STEP 5 : Resultats ===== -->
<div id="p-5" class="panel">
 <div class="card">
  <h2>5 — Resultats mesure</h2>
  <div id="res-single" style="display:none">
   <table class="rtable"><thead><tr><th>Grandeur</th><th>Valeur</th><th>Unite</th></tr></thead>
    <tbody id="res-body"></tbody></table>
  </div>
  <div id="res-sweep" style="display:none">
   <table class="rtable"><thead><tr><th></th><th>Duty%</th><th>RPM</th><th>V m/s</th><th>F nette N</th><th>CD/CL</th><th>Re</th></tr></thead>
    <tbody id="res-sweep-body"></tbody></table>
  </div>
 </div>

 <!-- Fumee rapport -->
 <div class="card" id="smoke-results-card" style="display:none;border-color:#2a4a3a">
  <h2 style="color:#4f4">Fumee — capture &amp; video</h2>
  <div id="snap-loading" style="font-size:12px;color:#fa0;text-align:center;padding:12px">Extraction capture milieu video...</div>
  <img id="smoke-snap" class="snap-img" src="" alt="capture fumee" style="display:none">
  <div class="video-wrap" id="smoke-video-wrap" style="display:none">
   <video id="smoke-video" controls></video>
  </div>
  <div style="display:flex;gap:8px;margin-top:8px;flex-wrap:wrap">
   <a id="btn-dl-video" class="btn-secondary" href="#" download="" style="flex:1;text-align:center;text-decoration:none;display:inline-block;padding:8px">&#11015; Video</a>
   <button class="btn-secondary" onclick="openEFluxModal()" style="flex:1;border-color:#440;color:#ff8">&#9889; Analyse flux optique</button>
  </div>
  <div class="status" id="st-flux"></div>
 </div>

 <!-- Graphiques sweep -->
 <div class="card" id="res-charts-card" style="display:none">
  <h2>Courbes</h2>
  <canvas id="chart-f-v"   height="150" style="display:block;width:100%;border-radius:4px;margin-bottom:8px;background:#060616"></canvas>
  <canvas id="chart-cd-re" height="150" style="display:block;width:100%;border-radius:4px;background:#060616"></canvas>
  <div style="font-size:10px;color:#555;margin-top:5px">Bleu&nbsp;=&nbsp;montée &nbsp;&nbsp; Orange&nbsp;=&nbsp;descente</div>
 </div>

 <!-- Résultats polaire -->
 <div id="res-polar" style="display:none">
  <div class="card">
   <h2 style="color:#4f4">Polaire complète</h2>
   <table class="rtable">
    <thead><tr><th>&#945;°</th><th>Duty%</th><th>RPM</th><th>V m/s</th><th>F nette N</th><th>CL/CD</th><th>Re</th></tr></thead>
    <tbody id="res-polar-tbody"></tbody>
   </table>
  </div>
  <!-- Courbes polaires -->
  <div class="card">
   <h2>Courbes polaires</h2>
   <div style="font-size:9px;color:#4af;letter-spacing:1px;text-transform:uppercase;margin-bottom:6px">CL/CD en fonction de l'angle d'attaque</div>
   <canvas id="chart-polar-cl" height="160" style="display:block;width:100%;border-radius:4px;margin-bottom:8px;background:#060616"></canvas>
   <canvas id="chart-polar-cd" height="160" style="display:block;width:100%;border-radius:4px;margin-bottom:8px;background:#060616"></canvas>
   <div style="font-size:9px;color:#4f4;letter-spacing:1px;text-transform:uppercase;margin-bottom:6px">Polaire CL = f(CD)</div>
   <canvas id="chart-polar-clcd" height="160" style="display:block;width:100%;border-radius:4px;background:#060616"></canvas>
   <div style="display:flex;gap:16px;margin-top:6px;font-size:11px">
    <span style="color:#4af">&#9632; CL</span><span style="color:#fa0">&#9632; CD</span><span style="color:#4f4">&#9632; Polaire</span>
   </div>
  </div>
  <!-- Grille photos par angle -->
  <div class="card" id="polar-photos-card" style="display:none">
   <h2>Photos par angle</h2>
   <div id="polar-photos-grid" style="display:grid;grid-template-columns:repeat(3,1fr);gap:8px"></div>
  </div>
 </div>

 <div class="card">
  <div style="display:flex;gap:8px;flex-wrap:wrap">
   <button class="btn-primary"   onclick="exportCsv()" style="flex:1">Exporter CSV</button>
   <button class="btn-secondary" id="btn-rapport" onclick="openReport()" style="display:none;flex:1;border-color:#44a;color:#8af">📄 Rapport PDF</button>
   <button class="btn-secondary" onclick="goStep(1)" style="flex:1">Nouvelle étude</button>
  </div>
 </div>
</div>

</div><!-- /content -->

<div class="nav">
 <button class="btn-secondary" id="btn-prev" onclick="prevStep()" style="display:none">&larr; Precedent</button>
 <button class="btn-primary"   id="btn-next" onclick="nextStep()">Suivant &rarr;</button>
</div>

<style>
@keyframes pulse{from{opacity:.4}to{opacity:1}}
.sw-lbl{font-size:9px;color:#556;letter-spacing:2px;text-transform:uppercase;margin-bottom:3px}
#cfg-sweep input{background:#0d0d0d;color:#eee;border:1px solid #333;border-radius:3px;padding:4px 6px;font-family:inherit;font-size:13px}
</style>

<script>
let step=1;
let eMode='single'; // 'single'|'sweep'|'sweep_double'
let params={duty:50,duration:30,p1:0.1,p2:0.15,aoa:0,objType:'airfoil',smokeDuty:15,smokeDur:20};
let results={};
let sweepResults=[];
let tare2Triggered=false,tare2Stable=0,tare2Cd=null;
let smokeVideoFile='';
let smokePhotoFile='';
let _smokeTimer=null;

const EOBJ={
 airfoil: {name:'profil aerodynamique',p1:'Corde c',  p2:'Envergure b',angle:true, coef:'CL / CD',p1v:0.10,p2v:0.15},
 plate:   {name:'plaque plane',        p1:'Largeur w', p2:'Hauteur h',  angle:true, coef:'CD / CL',p1v:0.10,p2v:0.10},
 cube:    {name:'cube',                p1:'Cote a',    p2:null,         angle:false,coef:'CD (≈1.05)',p1v:0.05,p2v:0},
 sphere:  {name:'sphere',              p1:'Diametre d',p2:null,         angle:false,coef:'CD (≈0.47)',p1v:0.05,p2v:0},
 cylinder:{name:'cylindre',            p1:'Diametre d',p2:'Longueur L', angle:false,coef:'CD (≈1.2)', p1v:0.05,p2v:0.10},
};

const VS_DESCS={
 kalman:"Fusion Kalman : combine flux optique + ventilateur + BMP280. Meilleure précision globale.",
 fan:"Modèle ventilateur : V = KV × RPM. Rapide, ne nécessite pas BMP280.",
 pressure:"BMP280 seul : V = √(2ΔP/ρ). Mesure directe Bernoulli. Référence à prendre avant la mesure.",
 manual:"Valeur fixe saisie manuellement. Utile si la vitesse est connue par un autre moyen (anémomètre, tube de Pitot…).",
};
let _vsrc="kalman";
async function setVSrc(s){
 _vsrc=s;
 ["kalman","fan","pressure","manual"].forEach(k=>{
  const el=document.getElementById("vs-"+k);
  if(el) el.className="mode-chip"+(k===s?" sel":"");
 });
 const dd=document.getElementById("vs-desc");
 if(dd) dd.textContent=VS_DESCS[s]||"";
 const mr=document.getElementById("vs-manual-row");
 if(mr) mr.style.display=(s==="manual")?"flex":"none";
 await post("/etude/set_vsrc?src="+s);
}
async function applyManualV(){
 const v=parseFloat(document.getElementById("vs-manual-val").value)||0;
 await post("/etude/set_vsrc?src=manual&v="+v.toFixed(2));
}
setVSrc("kalman");

function _getVFromState(d){
 if(_vsrc==="fan")      return d.airspeed_fan_ms||0;
 if(_vsrc==="pressure") return d.airspeed_pressure_ms||0;
 if(_vsrc==="manual")   return d.airspeed_manual_ms||0;
 return d.airspeed_kalman_ms||d.airspeed_avg_ms||d.airspeed_fan_ms||0;
}

function setMode(m){
 eMode=m;
 ['single','sweep','sweep_double','polar'].forEach(k=>{
  const el=document.getElementById('mc-'+k);
  if(el) el.className='mode-chip'+(k===m?' sel':'');
 });
 document.getElementById('cfg-single').style.display=(m==='single')?'':'none';
 document.getElementById('cfg-sweep').style.display =(m==='sweep'||m==='sweep_double')?'':'none';
 const cp=document.getElementById('cfg-polar');
 if(cp) cp.style.display=(m==='polar')?'':'none';
 const dn=document.getElementById('double-note');
 if(dn) dn.style.display=(m==='sweep_double')?'':'none';
 const btn=document.getElementById('btn-measure');
 if(btn) btn.textContent=(m==='polar')?'DÉMARRER LA POLAIRE':'LANCER LA MESURE';
 if(m==='polar') updatePolarPreview();
}
setMode('single');

function etudeSetObj(t){
 params.objType=t;
 const d=EOBJ[t]||EOBJ.airfoil;
 document.querySelectorAll('.eobj').forEach(c=>{
  c.style.borderColor='#222';
  const lbl=c.querySelector('div');if(lbl)lbl.style.color='#666';
 });
 const sel=document.getElementById('eo-'+t);
 if(sel){
  sel.style.borderColor='#4af';
  const lbl=sel.querySelector('div');if(lbl)lbl.style.color='#4af';
 }
 document.getElementById('ecfg-title').textContent='Parametres — '+d.name;
 document.getElementById('ecfg-p1-lab').textContent=d.p1;
 document.getElementById('cfg-p1').value=d.p1v;
 const p2r=document.getElementById('ecfg-p2-row');
 p2r.style.display=d.p2?'':'none';
 if(d.p2){document.getElementById('ecfg-p2-lab').textContent=d.p2;document.getElementById('cfg-p2').value=d.p2v;}
 document.getElementById('ecfg-aoa-row').style.display=d.angle?'':'none';
 const nn=document.getElementById('obj-name-step2');if(nn)nn.textContent=d.name;
 post('/aero/set?type='+t);
}
etudeSetObj('airfoil');

// SSE live
const evs=new EventSource('/events');
let lastState={};
evs.onmessage=e=>{
 lastState=JSON.parse(e.data);
 const q=(id)=>document.getElementById(id);
 const fN=q('live-F'); if(fN)fN.textContent=(lastState.force_N||0).toFixed(4)+' N';
 const vE=q('live-V'); if(vE)vE.textContent=(_getVFromState(lastState)||0).toFixed(2)+' m/s';
 const dE=q('live-duty'); if(dE)dE.textContent=Math.round(lastState.fan_duty||0)+' %';
 const sF=q('smoke-live-F'); if(sF)sF.textContent=(lastState.force_N||0).toFixed(4)+' N';
 const sD=q('smoke-live-duty'); if(sD)sD.textContent=Math.round(lastState.fan_duty||0)+' %';
 if(step===1) updateDragStatus();
 if(step===2) _checkTare2();
};

function updateDragStatus(){
 const el=document.getElementById('proto-drag-status');if(!el)return;
 const tbl=lastState.fan_drag_table||[];
 if(tbl.length>0){
  el.style.color='#3f3';
  el.innerHTML='&#10003; Courbe trainee : <b>'+tbl.length+' points</b> — correction aero active';
 }else{
  el.style.color='#fa0';
  el.innerHTML='&#9888; Pas de courbe trainee — <a href="/calib" style="color:#4af">Calibrer d abord</a> (recommande)';
 }
}

function _checkTare2(){
 const s=lastState;
 const fanOk=(s.fan_duty||0)<=1;
 const sigOk=(s.force_std||1)<0.002;
 const fi=document.getElementById('ck-fan2-i');
 const si=document.getElementById('ck-sig2-i');
 if(fi){fi.className='check-icon '+(fanOk?'ok':'ko');fi.textContent=fanOk?'✓':'✗';}
 if(si){si.className='check-icon '+(sigOk?'ok':'wait');si.textContent=sigOk?'✓':'~';}
 if(!tare2Triggered){
  if(fanOk&&sigOk){
   tare2Stable++;
   if(tare2Stable===10){
    const cdEl=document.getElementById('auto-tare-cd2');if(cdEl)cdEl.style.display='block';
    let cnt=3; document.getElementById('cd2-secs').textContent=cnt;
    const tid=setInterval(()=>{
     cnt--; const el=document.getElementById('cd2-secs');if(el)el.textContent=cnt;
     if(cnt<=0){clearInterval(tid);tare2Cd=null;doTare2();}
    },1000);
    tare2Cd=tid;
   }
  }else{
   tare2Stable=0;
   if(tare2Cd){clearInterval(tare2Cd);tare2Cd=null;}
   const cdEl=document.getElementById('auto-tare-cd2');if(cdEl)cdEl.style.display='none';
  }
 }
}

function goStep(n){
 document.querySelectorAll('.panel').forEach(p=>p.classList.remove('on'));
 document.getElementById('p-'+n).classList.add('on');
 for(let i=1;i<=5;i++){
  const el=document.getElementById('si-'+i);
  if(el) el.className='stepitem'+(i===n?' active':i<n?' done':'');
 }
 // Bouton prev visible si pas step 1
 document.getElementById('btn-prev').style.display=n>1?'':'none';
 // Bouton next masque aux etapes 3 et 4 (avancement automatique)
 document.getElementById('btn-next').style.display=(n<5&&n!==3&&n!==4)?'':'none';
 document.getElementById('btn-next').disabled=false;
 step=n;

 if(n===1){
  updateDragStatus();
 }
 if(n===2){
  post('/fan/set?duty=0');
  if(tare2Cd){clearInterval(tare2Cd);tare2Cd=null;}
  tare2Stable=0; tare2Triggered=false;
  const cdEl=document.getElementById('auto-tare-cd2');if(cdEl)cdEl.style.display='none';
  const st=document.getElementById('st-tare2');if(st){st.className='status';st.textContent='';}
 }
 if(n===3){
  // Demarrer le ventilateur
  const duty=(eMode==='single')?(+document.getElementById('cfg-duty').value||50)
                                :(+document.getElementById('sw-dmin').value||20);
  post('/fan/set?duty='+duty);
  document.getElementById('sweep-live-card').style.display='none';
  document.getElementById('pg-meas').style.display='none';
  const st=document.getElementById('st-meas');
  st.className='status st-wait';
  st.textContent='Ventilateur a '+duty+'% — verifiez l objet, puis lancez la mesure.';
 }
 if(n===4){
  params.smokeDuty=+document.getElementById('cfg-smoke-duty').value||15;
  params.smokeDur =+document.getElementById('cfg-smoke-dur').value||20;
  document.getElementById('smoke-duty-disp').textContent=params.smokeDuty;
  // Afficher uniquement la phase preparation
  document.getElementById('smoke-prep-card').style.display='block';
  document.getElementById('smoke-rec-card').style.display='none';
  document.getElementById('smoke-conv-card').style.display='none';
  const btn=document.getElementById('btn-smoke-ready');if(btn)btn.disabled=false;
  const st=document.getElementById('st-smoke');st.className='status';st.textContent='';
  post('/fan/set?duty='+params.smokeDuty);
 }
 if(n===5){
  post('/fan/set?duty=0');
  _fillResults();
 }
}
function nextStep(){if(step<5)goStep(step+1);}
function prevStep(){
 if(step===3||step===4) post('/fan/set?duty=0');
 if(step>1) goStep(step-1);
}

async function post(u,body){return fetch(u,{method:'POST',body:body});}

async function doTare2(){
 if(tare2Cd){clearInterval(tare2Cd);tare2Cd=null;}
 tare2Triggered=true;
 const cdEl=document.getElementById('auto-tare-cd2');if(cdEl)cdEl.style.display='none';
 const btn=document.getElementById('btn-tare2');if(btn)btn.disabled=true;
 const st=document.getElementById('st-tare2');
 st.className='status st-wait';st.textContent='Tare en cours...';
 try{
  const r=await (await post('/tare')).json();
  st.className='status st-ok';st.textContent='&#10003; Offset = '+r.offset+' ADU — sauvegarde ✓';
  if(btn)btn.disabled=false;
  document.getElementById('si-2').className='stepitem done';
  params.p1=+document.getElementById('cfg-p1').value;
  params.p2=+document.getElementById('cfg-p2').value||0;
  params.aoa=+document.getElementById('cfg-aoa').value;
  await post('/aero/set?type='+params.objType+'&chord='+params.p1+'&span='+(params.p2||params.p1)+'&aoa='+params.aoa);
  setTimeout(()=>goStep(3),1500);
 }catch(e){
  st.className='status st-err';st.textContent='Erreur : '+e.message;
  if(btn)btn.disabled=false;
 }
}

// -------- MESURE --------
async function doMeasure(){
 if(eMode==='sweep'||eMode==='sweep_double'){await doSweep();return;}
 if(eMode==='polar'){await startPolar();return;}
 const duty=+document.getElementById('cfg-duty').value;
 const dur =+document.getElementById('cfg-dur').value;
 const st=document.getElementById('st-meas');
 const btn=document.getElementById('btn-measure');
 const pg=document.getElementById('pg-meas');
 const pgb=document.getElementById('pg-meas-bar');
 st.className='status st-wait';st.textContent='Acquisition ('+dur+'s)...';
 btn.disabled=true;pg.style.display='block';pgb.style.width='0%';
 await post('/etude/measure?duty='+duty+'&duration='+dur);
 const pid=setInterval(async()=>{
  const s=await fetch('/etude/status').then(r=>r.json()).catch(()=>null);
  if(!s)return;
  pgb.style.width=Math.round(s.progress*100)+'%';
  if(s.phase==='stabilizing') st.textContent='Stabilisation ('+duty+'%)...';
  else if(s.phase==='acquiring') st.textContent='Acquisition '+Math.round(s.progress*100)+'%';
  else if(s.phase==='done'){
   clearInterval(pid);pgb.style.width='100%';
   results=Object.assign(results,s);
   st.className='status st-ok';st.textContent='✓ Termine — '+s.n_samples+' echantillons';
   btn.disabled=false;
   document.getElementById('si-3').className='stepitem done';
   setTimeout(()=>goStep(4),800);
  }else if(s.phase==='error'){
   clearInterval(pid);st.className='status st-err';st.textContent='Erreur : '+s.msg;btn.disabled=false;
  }
 },600);
}

// ============================================================
// POLAIRE : état machine côté client
// ============================================================
let _polarResults=[];
let _polarAngles=[];
let _polarIdx=0;
let _polarDuty=60, _polarStab=6, _polarDur=15, _polarAutoTare=true;

function updatePolarPreview(){
 const amin=+document.getElementById("polar-amin").value;
 const amax=+document.getElementById("polar-amax").value;
 const step=Math.max(0.5, +(document.getElementById("polar-step").value)||5);
 const angles=[];
 for(let a=amin; a<=amax+0.001; a+=step) angles.push(Math.round(a*10)/10);
 const el=document.getElementById("polar-angles-preview");
 if(el) el.innerHTML=`<b>${angles.length} angles</b> : ${angles.map(a=>(a>=0?"+":"")+a+"°").join(" &nbsp; ")}`;
 return angles;
}

async function startPolar(){
 _polarAngles=updatePolarPreview();
 if(!_polarAngles.length){alert("Configurez les angles d'abord.");return;}
 _polarDuty =+document.getElementById("polar-duty").value||60;
 _polarStab =+document.getElementById("polar-stab").value||6;
 _polarDur  =+document.getElementById("polar-dur").value||15;
 _polarAutoTare=document.getElementById("polar-tare").checked;
 _polarIdx=0; _polarResults=[];
 // Démarrer le ventilateur
 await post('/fan/set?duty='+_polarDuty);
 goStep(3);
 // Afficher la UI polaire, cacher les autres
 document.getElementById("btn-measure").style.display="none";
 document.getElementById("sweep-live-card").style.display="none";
 document.getElementById("polar-angle-card").style.display="block";
 _showPolarAngle();
}

function _showPolarAngle(){
 if(_polarIdx>=_polarAngles.length){_polarFinish();return;}
 const a=_polarAngles[_polarIdx];
 const n=_polarAngles.length;
 document.getElementById("polar-angle-title").textContent="Polaire — angle "+(_polarIdx+1)+"/"+n;
 document.getElementById("polar-angle-val").textContent=(a>=0?"+":"")+a+"°";
 const st=document.getElementById("polar-st");
 st.textContent=""; st.style.color="#fa0";
 document.getElementById("polar-pg-outer").style.display="none";
 document.getElementById("btn-polar-confirm").disabled=false;
 if(_polarResults.length>0) _renderPolarPartial();
}

function _renderPolarPartial(){
 const tb=document.getElementById("polar-partial-tbody");
 if(!tb)return;
 tb.innerHTML="";
 _polarResults.forEach(r=>{
  const as=(r.angle>=0?"+":"")+r.angle;
  tb.innerHTML+=`<tr><td style="color:#4f4;font-weight:bold">${as}°</td><td>${r.V_ms.toFixed(2)}</td><td>${r.F_net_N.toFixed(4)}</td><td>${r.CL.toFixed(4)}</td><td>${Math.round(r.Re)}</td></tr>`;
 });
 document.getElementById("polar-partial-wrap").style.display="";
}

async function confirmPolarAngle(){
 const btn=document.getElementById("btn-polar-confirm");
 const pg =document.getElementById("polar-pg-outer");
 const pgb=document.getElementById("polar-pg-bar");
 const st =document.getElementById("polar-st");
 btn.disabled=true;

 if(_polarAutoTare){
  st.textContent="Arrêt ventilo pour tare..."; st.style.color="#fa0";
  await post('/fan/set?duty=0');
  await new Promise(r=>setTimeout(r,1800));
  await post('/tare');
  await new Promise(r=>setTimeout(r,500));
  await post('/fan/set?duty='+_polarDuty);
  st.textContent="Stabilisation ("+_polarStab+"s)...";
  await new Promise(r=>setTimeout(r,_polarStab*1000));
 } else {
  st.textContent="Stabilisation ("+_polarStab+"s)...";
  await new Promise(r=>setTimeout(r,_polarStab*1000));
 }

 // Lancer la mesure
 st.textContent="Acquisition "+_polarDur+"s..."; st.style.color="#4af";
 pg.style.display="block"; pgb.style.width="0%";
 await post('/etude/measure?duty='+_polarDuty+'&duration='+_polarDur);
 const angle=_polarAngles[_polarIdx];
 let measResult=null;

 await new Promise(resolve=>{
  const pid=setInterval(async()=>{
   const s=await fetch('/etude/status').then(r=>r.json()).catch(()=>null);
   if(!s)return;
   pgb.style.width=Math.round(s.progress*100)+"%";
   if(s.phase==="done"){
    clearInterval(pid); pgb.style.width="100%";
    measResult=s; resolve();
   } else if(s.phase==="error"){
    clearInterval(pid);
    st.textContent="Erreur mesure : "+s.msg; st.style.color="#f55";
    resolve();
   }
  },500);
 });

 // Photo live
 st.textContent="Photo..."; st.style.color="#4af";
 let snapUrl="";
 try{
  const sr=await fetch('/etude/photo').then(r=>r.json());
  if(sr.ok) snapUrl="/media/file?name="+encodeURIComponent(sr.photo);
 }catch(e){}

 if(measResult){
  _polarResults.push({
   angle,
   duty:Math.round(_polarDuty),
   rpm:measResult.fan_rpm||0,
   V_ms:measResult.V_ms||0,
   F_net_N:measResult.F_net_N||0,
   CL:measResult.CL||0,
   CD:measResult.CD||0,
   Re:measResult.Re||0,
   snapUrl,
  });
 }
 _polarIdx++;
 st.textContent="✓ Angle "+(angle>=0?"+":"")+angle+"° enregistré";st.style.color="#4f4";
 await new Promise(r=>setTimeout(r,600));
 _showPolarAngle();
}

function _polarFinish(){
 post('/fan/set?duty=0');
 document.getElementById("polar-angle-card").style.display="none";
 document.getElementById("btn-measure").style.display="";
 document.getElementById("si-3").className="stepitem done";
 document.getElementById("si-4").className="stepitem done";
 setTimeout(()=>{goStep(5);_showPolarResults();_saveEtude();},600);
}

function _showPolarResults(){
 document.getElementById("res-single").style.display="none";
 document.getElementById("res-sweep").style.display="none";
 document.getElementById("res-polar").style.display="";
 // Table
 const tb=document.getElementById("res-polar-tbody");
 tb.innerHTML="";
 _polarResults.forEach(r=>{
  const as=(r.angle>=0?"+":"")+r.angle;
  tb.innerHTML+=`<tr><td style="color:#4f4;font-weight:bold">${as}°</td><td>${r.duty}%</td><td>${r.rpm}</td><td>${r.V_ms.toFixed(2)}</td><td>${r.F_net_N.toFixed(5)}</td><td>${r.CL.toFixed(4)}</td><td>${Math.round(r.Re)}</td></tr>`;
 });
 // Photos
 const hasPh=_polarResults.some(r=>r.snapUrl);
 const grid=document.getElementById("polar-photos-grid");
 if(hasPh){
  grid.innerHTML="";
  _polarResults.forEach(r=>{
   const as=(r.angle>=0?"+":"")+r.angle;
   grid.innerHTML+=`<div style="background:#0a1018;border:1px solid #1a2a3a;border-radius:6px;overflow:hidden">${r.snapUrl?`<img src="${r.snapUrl}" loading="lazy" style="width:100%;display:block;aspect-ratio:4/3;object-fit:cover">`:'<div style="aspect-ratio:4/3;display:flex;align-items:center;justify-content:center;color:#334;font-size:11px">—</div>'}<div style="padding:6px;text-align:center"><div style="font-size:15px;font-weight:bold;color:#4f4">${as}°</div><div style="font-size:10px;color:#556">${r.V_ms.toFixed(1)}m/s · Re${Math.round(r.Re/1000)}k</div></div></div>`;
  });
  document.getElementById("polar-photos-card").style.display="";
 }
 // Charts
 _drawPolarCharts();
 document.getElementById("btn-rapport").style.display="";
}

function _drawPolarCharts(){
 if(_polarResults.length<2)return;
 const angles=_polarResults.map(r=>r.angle);
 const cls=_polarResults.map(r=>r.CL);
 const cds=_polarResults.map(r=>r.CD);
 // CL vs α
 _drawChart("chart-polar-cl",angles.map((a,i)=>[a,cls[i],"#4af"]),
  {xlabel:"α (°)",ylabel:"CL",dots:true,zeroline:true});
 // CD vs α
 _drawChart("chart-polar-cd",angles.map((a,i)=>[a,cds[i],"#fa0"]),
  {xlabel:"α (°)",ylabel:"CD",dots:true});
 // Polaire CL = f(CD)
 _drawChart("chart-polar-clcd",_polarResults.map(r=>[r.CD,r.CL,"#4f4"]),
  {xlabel:"CD",ylabel:"CL",dots:true,zeroline:true,connect:false});
}

let _sweepVar='duty';
function setSweepVar(v){
 _sweepVar=v;
 const isDuty=v==='duty';
 document.getElementById('sv-duty').style.borderColor=isDuty?'#5af':'#2a2a2a';
 document.getElementById('sv-duty').style.color=isDuty?'#5af':'#888';
 document.getElementById('sv-duty').style.background=isDuty?'#0a1e30':'#111';
 document.getElementById('sv-rpm').style.borderColor=isDuty?'#2a2a2a':'#5af';
 document.getElementById('sv-rpm').style.color=isDuty?'#888':'#5af';
 document.getElementById('sv-rpm').style.background=isDuty?'#111':'#0a1e30';
 document.getElementById('sw-row-dmin').style.display=isDuty?'':'none';
 document.getElementById('sw-row-dmax').style.display=isDuty?'':'none';
 document.getElementById('sw-row-rmin').style.display=isDuty?'none':'';
 document.getElementById('sw-row-rmax').style.display=isDuty?'none':'';
 document.getElementById('sv-hint').style.display=isDuty?'none':'';
}

async function doSweep(){
 const pts    =+document.getElementById('sw-pts').value;
 const stab   =+document.getElementById('sw-stab').value;
 const dur    =+document.getElementById('sw-dur').value||10;
 const dbl    =(eMode==='sweep_double')?1:0;
 const maxduty=+document.getElementById('sw-maxduty').value||95;
 let url;
 if(_sweepVar==='rpm'){
  const rmin=+document.getElementById('sw-rmin').value;
  const rmax=+document.getElementById('sw-rmax').value;
  url=`/etude/sweep?var=rpm&rmin=${rmin}&rmax=${rmax}&pts=${pts}&stab=${stab}&dur=${dur}&double=${dbl}&max_duty=${maxduty}`;
 }else{
  const dmin=+document.getElementById('sw-dmin').value;
  const dmax=+document.getElementById('sw-dmax').value;
  url=`/etude/sweep?var=duty&dmin=${dmin}&dmax=${dmax}&pts=${pts}&stab=${stab}&dur=${dur}&double=${dbl}&max_duty=${maxduty}`;
 }
 const st=document.getElementById('st-meas');
 const btn=document.getElementById('btn-measure');
 document.getElementById('sweep-live-card').style.display='block';
 document.getElementById('sw-live-tbody').innerHTML='';
 st.className='status st-wait';st.textContent='Sweep en cours...';
 btn.disabled=true; sweepResults=[];
 await post(url);
 const pid=setInterval(async()=>{
  const s=await fetch('/etude/status').then(r=>r.json()).catch(()=>null);
  if(!s)return;
  const msg=document.getElementById('sweep-live-msg');if(msg)msg.textContent=s.msg||'—';
  const bar=document.getElementById('sw-live-bar');if(bar)bar.style.width=Math.round(s.progress*100)+'%';
  if(s.sweep_partial&&s.sweep_partial.length>sweepResults.length){
   sweepResults=s.sweep_partial;
   const tb=document.getElementById('sw-live-tbody');
   if(tb){
    tb.innerHTML='';
    sweepResults.forEach(row=>{
     const dc=row.dir==='↓'?'dir-dn':'dir-up';
     tb.innerHTML+=`<tr><td class="${dc}">${row.dir||'↑'}</td><td>${row.duty}%</td><td>${row.V_ms.toFixed(2)}</td><td>${row.F_net_N.toFixed(4)}</td><td>${row.CL.toFixed(4)}</td><td>${Math.round(row.Re)}</td></tr>`;
    });
   }
  }
  if(s.phase==='done'){
   clearInterval(pid);
   sweepResults=s.sweep_partial||[];
   st.className='status st-ok';st.textContent='✓ Sweep '+sweepResults.length+' points';
   btn.disabled=false;
   document.getElementById('si-3').className='stepitem done';
   setTimeout(()=>goStep(4),800);
  }else if(s.phase==='error'){
   clearInterval(pid);st.className='status st-err';st.textContent='Erreur : '+s.msg;btn.disabled=false;
  }
 },600);
}

// -------- FUMEE --------
async function startSmokeRec(){
 const btn=document.getElementById('btn-smoke-ready');
 btn.disabled=true;
 const dur=params.smokeDur;
 const st=document.getElementById('st-smoke');
 st.className='status st-wait';st.textContent='Demarrage enregistrement...';
 const rr=await fetch('/record/toggle',{method:'POST'}).then(r=>r.json()).catch(()=>({recording:false}));
 if(!rr.recording){
  st.className='status st-err';st.textContent='Erreur : '+(rr.err||'enregistrement impossible');
  btn.disabled=false;return;
 }
 smokeVideoFile=rr.video?rr.video.split('/').pop():'';
 // Afficher la carte enregistrement
 document.getElementById('smoke-prep-card').style.display='none';
 document.getElementById('smoke-rec-card').style.display='block';
 const cd=document.getElementById('smoke-cd');
 const pg=document.getElementById('smoke-pg');
 const stRec=document.getElementById('st-smoke-rec');
 stRec.textContent='Enregistrement en cours...';
 let elapsed=0;
 _smokeTimer=setInterval(()=>{
  elapsed++;
  const rem=dur-elapsed;
  if(cd) cd.textContent=rem+' s';
  if(pg) pg.style.width=Math.round(elapsed/dur*100)+'%';
  if(elapsed>=dur){clearInterval(_smokeTimer);_smokeTimer=null;_doStopSmoke();}
 },1000);
}

function stopSmokeEarly(){
 if(_smokeTimer){clearInterval(_smokeTimer);_smokeTimer=null;}
 _doStopSmoke();
}

async function _doStopSmoke(){
 const stRec=document.getElementById('st-smoke-rec');
 if(stRec) stRec.textContent='Arret enregistrement...';
 await fetch('/record/toggle',{method:'POST'}).catch(()=>{});
 document.getElementById('smoke-rec-card').style.display='none';
 document.getElementById('smoke-conv-card').style.display='block';
 document.getElementById('si-4').className='stepitem done';
 // Attendre conversion puis snapshot
 await _waitAndSnapshot();
}

async function _waitAndSnapshot(){
 const st=document.getElementById('st-smoke-conv');
 let attempts=0;
 while(attempts<60){
  await _sleep(2000);
  attempts++;
  const d=await fetch('/media/list').then(r=>r.json()).catch(()=>({videos:[]}));
  const mine=(d.videos||[]).find(v=>v.name===smokeVideoFile&&!v.converting&&!v.analyzing);
  if(mine){
   st.textContent='Video prete — extraction capture milieu...';
   if(smokeVideoFile){
    const sr=await fetch('/etude/snapshot?video='+encodeURIComponent(smokeVideoFile))
     .then(r=>r.json()).catch(()=>({ok:false}));
    if(sr.ok) smokePhotoFile=sr.photo;
   }
   goStep(5);
   return;
  }
  if(st) st.textContent='Conversion en cours ('+attempts*2+'s)...';
 }
 // Timeout — on passe quand meme
 goStep(5);
}

function skipSmoke(){
 smokeVideoFile='';smokePhotoFile='';
 document.getElementById('si-4').className='stepitem done';
 goStep(5);
}

function _sleep(ms){return new Promise(r=>setTimeout(r,ms));}

// -------- RESULTATS --------
// ---- Moteur graphique canvas ----
function _drawChart(id, pts, opts){
 const c=document.getElementById(id); if(!c||!pts.length)return;
 const ctx=c.getContext('2d');
 c.width=c.offsetWidth||c.clientWidth||340;
 const W=c.width,H=c.height,pl=52,pr=10,pt=20,pb=28,w=W-pl-pr,h=H-pt-pb;
 ctx.clearRect(0,0,W,H);
 ctx.fillStyle='#060616';ctx.fillRect(0,0,W,H);
 const allX=pts.map(p=>p[0]),allY=pts.map(p=>p[1]);
 const xmin=Math.min(...allX),xmax=Math.max(...allX);
 const ymin=Math.min(...allY),ymax=Math.max(...allY);
 const xr=xmax-xmin||1,yr=ymax-ymin||1;
 const cx=x=>pl+(x-xmin)/xr*w, cy=y=>pt+h-(y-ymin)/yr*h;
 // grille
 ctx.strokeStyle='#12122a';ctx.lineWidth=1;
 for(let i=0;i<=4;i++){
  const yv=ymin+yr*i/4,py=cy(yv);
  ctx.beginPath();ctx.moveTo(pl,py);ctx.lineTo(pl+w,py);ctx.stroke();
  ctx.fillStyle='#555';ctx.font='9px monospace';ctx.textAlign='right';
  ctx.fillText(yv.toFixed(3),pl-3,py+3);
 }
 for(let i=0;i<=4;i++){
  const xv=xmin+xr*i/4,px=cx(xv);
  ctx.fillStyle='#555';ctx.font='9px monospace';ctx.textAlign='center';
  ctx.fillText(xv.toFixed(3),px,pt+h+20);
 }
 // axes
 ctx.strokeStyle='#2a2a4a';ctx.lineWidth=1;
 ctx.beginPath();ctx.moveTo(pl,pt);ctx.lineTo(pl,pt+h);ctx.lineTo(pl+w,pt+h);ctx.stroke();
 // ligne zéro horizontale (y=0)
 if(opts.zeroline && ymin<0 && ymax>0){
  const zy=cy(0);
  ctx.strokeStyle='#333';ctx.lineWidth=1;ctx.setLineDash([3,3]);
  ctx.beginPath();ctx.moveTo(pl,zy);ctx.lineTo(pl+w,zy);ctx.stroke();
  ctx.setLineDash([]);
 }
 // lignes et points par couleur
 const groups={};
 pts.forEach(p=>{const col=p[2]||opts.color||'#4af';(groups[col]=groups[col]||[]).push(p);});
 Object.entries(groups).forEach(([col,gp])=>{
  gp.sort((a,b)=>a[0]-b[0]);
  if(opts.connect!==false){
   ctx.strokeStyle=col;ctx.lineWidth=1.5;
   ctx.beginPath();gp.forEach((p,i)=>i?ctx.lineTo(cx(p[0]),cy(p[1])):ctx.moveTo(cx(p[0]),cy(p[1])));ctx.stroke();
  }
  gp.forEach(p=>{ctx.beginPath();ctx.arc(cx(p[0]),cy(p[1]),opts.dots?4:3,0,2*Math.PI);ctx.fillStyle=col;ctx.fill();});
 });
 // labels
 ctx.fillStyle='#888';ctx.font='10px monospace';ctx.textAlign='center';
 ctx.fillText(opts.title||'',pl+w/2,13);
 ctx.fillText(opts.xlabel||'',pl+w/2,H-4);
 ctx.save();ctx.translate(11,pt+h/2);ctx.rotate(-Math.PI/2);ctx.fillStyle='#777';ctx.font='9px monospace';ctx.fillText(opts.ylabel||'',0,0);ctx.restore();
}

function _drawCharts(){
 const card=document.getElementById('res-charts-card');
 if(eMode==='single'||!sweepResults.length){card.style.display='none';return;}
 card.style.display='';
 const pts1=sweepResults.map(r=>[r.V_ms, r.F_net_N, r.dir==='↓'?'#fa0':'#4af']);
 const pts2=sweepResults.map(r=>[r.Re,   r.CL,      r.dir==='↓'?'#fa0':'#f84']);
 requestAnimationFrame(()=>{
  _drawChart('chart-f-v',  pts1,{title:'Force nette vs Vitesse',xlabel:'V (m/s)',ylabel:'F (N)'});
  _drawChart('chart-cd-re',pts2,{title:'CL/CD vs Reynolds',    xlabel:'Re',      ylabel:'CL/CD',color:'#f84'});
 });
}

// ---- Sauvegarde étude & rapport PDF ----
let _currentEtudeId=null;

async function _saveEtude(){
 const payload={
  mode:eMode, object:params.objType, params:{...params},
  results:eMode==='single'?{...results}:{},
  sweep:(eMode==='sweep'||eMode==='sweep_double')?[...sweepResults]:[],
  polar:eMode==='polar'?[..._polarResults]:[],
  smoke_video:smokeVideoFile, smoke_photo:smokePhotoFile,
 };
 const r=await fetch('/etude/save',{method:'POST',
  headers:{'Content-Type':'application/json'},
  body:JSON.stringify(payload)
 }).then(r=>r.json()).catch(()=>({ok:false}));
 if(r.ok){
  _currentEtudeId=r.id;
  document.getElementById('btn-rapport').style.display='';
 }
}

function openReport(){
 if(!_currentEtudeId)return;
 window.open('/etude/report?id='+encodeURIComponent(_currentEtudeId),'_blank');
}

function _fillResults(){
 if(eMode==='polar'){ _showPolarResults(); return; }
 if(eMode==='single') _fillSingle();
 else _fillSweep();
 _drawCharts();
 _saveEtude();
 // Carte fumee
 const sc=document.getElementById('smoke-results-card');
 if(smokeVideoFile){
  sc.style.display='block';
  // Screenshot
  const snap=document.getElementById('smoke-snap');
  const snapLoading=document.getElementById('snap-loading');
  if(smokePhotoFile){
   snap.src='/media/file/'+encodeURIComponent(smokePhotoFile);
   snap.style.display='block';
   snapLoading.style.display='none';
  }else{
   snapLoading.style.display='block';snap.style.display='none';
  }
  // Video
  const vid=document.getElementById('smoke-video');
  const dl=document.getElementById('btn-dl-video');
  vid.src='/media/file/'+encodeURIComponent(smokeVideoFile);
  dl.href='/media/file/'+encodeURIComponent(smokeVideoFile);
  dl.download=smokeVideoFile;
  document.getElementById('smoke-video-wrap').style.display='block';
 }else{
  sc.style.display='none';
 }
}

function _fillSingle(){
 const r=results;
 const od=EOBJ[params.objType]||EOBJ.airfoil;
 const rows=[
  ['Force brute',   r.F_mean_N!=null?r.F_mean_N.toFixed(4):'—','N'],
  ['Support drag',  r.support_drag_N!=null?r.support_drag_N.toFixed(4):'—','N'],
  ['Force nette',   r.F_net_N!=null?r.F_net_N.toFixed(4):'—','N'],
  ['Vitesse air',   r.V_ms!=null?r.V_ms.toFixed(3):'—','m/s'],
  ['Reynolds Re',   r.Re!=null?Math.round(r.Re):'—',''],
  [od.coef,         r.CL!=null?r.CL.toFixed(4):'—',''],
  ['Strouhal St',   r.strouhal!=null?r.strouhal.toFixed(3):'—',''],
  ['Echantillons',  r.n_samples||'—',''],
 ];
 document.getElementById('res-single').style.display='';
 document.getElementById('res-sweep').style.display='none';
 const tb=document.getElementById('res-body');tb.innerHTML='';
 rows.forEach(([l,v,u])=>{
  tb.innerHTML+=`<tr><td style="color:#888">${l}</td><td class="val">${v}</td><td class="unit">${u}</td></tr>`;
 });
}

function _fillSweep(){
 document.getElementById('res-single').style.display='none';
 document.getElementById('res-sweep').style.display='';
 const tb=document.getElementById('res-sweep-body');tb.innerHTML='';
 sweepResults.forEach(row=>{
  const dc=row.dir==='↓'?'dir-dn':'dir-up';
  tb.innerHTML+=`<tr><td class="${dc}">${row.dir||'↑'}</td><td>${row.duty}%</td><td>${row.rpm||'—'}</td><td>${row.V_ms.toFixed(2)}</td><td>${row.F_net_N.toFixed(4)}</td><td>${row.CL.toFixed(4)}</td><td>${Math.round(row.Re)}</td></tr>`;
 });
}

const EAM_MODES={
 heatmap:    {label:'🌡 Chaleur',       desc:'Champ 2D complet, teinte=direction.'},
 arrows:     {label:'↗ Flèches',        desc:'Vecteurs décimés.'},
 vorticity:  {label:'🌀 Tourbillons',   desc:'Rouge=CCW, bleu=CW.'},
 streamlines:{label:'〜 Lignes courant',desc:'Trajectoires intégrées.'},
 flow_x:     {label:'↔ Axe X seul',    desc:'Horizontal uniquement. Élimine les perturbations Y.'},
 flow_y:     {label:'↕ Axe Y seul',    desc:'Vertical uniquement. Isole portance/déflexion.'},
};
const EAM_DIRS={all:{label:'± Toutes'},pos:{label:'+ Aval/Bas'},neg:{label:'− Retour/Haut'}};
let _eMode='flow_x',_eDir='all';
function openEFluxModal(){
 if(!smokeVideoFile){alert('Pas de vidéo fumée');return;}
 _eMode='flow_x';_eDir='all';
 _renderEModes();_renderEDirs();
 document.getElementById('eam').style.display='flex';
}
function closeEFluxModal(){document.getElementById('eam').style.display='none';}
function setEM(m){_eMode=m;_renderEModes();_renderEDirs();}
function setED(d){_eDir=d;_renderEDirs();}
function _ec(k,active,color,fn,label){return `<span onclick="${fn}('${k}')" style="padding:4px 9px;border-radius:4px;cursor:pointer;font-size:11px;border:1px solid ${active?color:'#2a2a2a'};color:${active?color:'#888'};background:#111;white-space:nowrap">${label}</span>`;}
function _renderEModes(){
 document.getElementById('eam-modes').innerHTML=Object.entries(EAM_MODES).map(([k,v])=>_ec(k,k===_eMode,'#5af','setEM',v.label)).join('');
 document.getElementById('eam-desc').textContent=EAM_MODES[_eMode]?.desc||'';
 const d=_eMode==='flow_x'||_eMode==='flow_y';
 document.getElementById('eam-dir-row').style.display=d?'':'none';
 if(!d)_eDir='all';
}
function _renderEDirs(){document.getElementById('eam-dirs').innerHTML=Object.entries(EAM_DIRS).map(([k,v])=>_ec(k,k===_eDir,'#fa0','setED',v.label)).join('');}
async function analyseFlux(){
 closeEFluxModal();
 const st=document.getElementById('st-flux');
 st.className='status st-wait';st.textContent='Analyse lancée — voir onglet Médias pour le progrès.';
 const r=await fetch('/media/analyze?name='+encodeURIComponent(smokeVideoFile)+'&mode='+_eMode+'&dir='+_eDir,{method:'POST'})
  .then(r=>r.json()).catch(()=>({ok:false}));
 if(r.ok){st.className='status st-ok';st.textContent='✓ En cours — consulter /media';}
 else{st.className='status st-err';st.textContent='Erreur : '+(r.err||'?');}
}

function exportCsv(){window.open('/export/csv','_blank');}

// Init
goStep(1);
window.addEventListener('beforeunload',()=>navigator.sendBeacon('/etude/cancel'));
document.addEventListener('visibilitychange',()=>{if(document.hidden)navigator.sendBeacon('/etude/cancel');});
</script>

<div id="eam" style="display:none;position:fixed;inset:0;background:rgba(0,0,0,.78);z-index:300;align-items:center;justify-content:center">
 <div style="background:#0e0e0e;border:1px solid #2a2a2a;border-radius:8px;padding:20px 22px;width:330px;max-width:94vw">
  <div style="font-size:13px;font-weight:600;color:#ccc;margin-bottom:14px">⚡ Analyse flux optique</div>
  <div style="font-size:10px;color:#555;margin-bottom:5px;text-transform:uppercase;letter-spacing:.06em">Mode</div>
  <div id="eam-modes" style="display:flex;flex-wrap:wrap;gap:5px;margin-bottom:12px"></div>
  <div id="eam-dir-row" style="display:none">
   <div style="font-size:10px;color:#555;margin-bottom:5px;text-transform:uppercase;letter-spacing:.06em">Direction</div>
   <div id="eam-dirs" style="display:flex;gap:5px;margin-bottom:12px"></div>
  </div>
  <div id="eam-desc" style="font-size:11px;color:#666;margin-bottom:16px;min-height:28px;line-height:1.4"></div>
  <div style="display:flex;gap:8px">
   <button onclick="analyseFlux()" style="flex:1;background:#1a3a1a;color:#8f8;border:1px solid #3a6a3a;padding:7px;border-radius:4px;cursor:pointer;font-weight:600">⚡ Lancer</button>
   <button onclick="closeEFluxModal()" style="background:#1a1a1a;color:#888;border:1px solid #333;padding:7px 12px;border-radius:4px;cursor:pointer">Annuler</button>
  </div>
 </div>
</div>
</body></html>"""


# ==================== SENSORS PAGE ====================

SENSORS_HTML = r"""<!DOCTYPE html>
<html lang="fr"><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Capteurs — Soufflerie ENSAM</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#080c10;color:#dde;font-family:ui-monospace,Menlo,monospace;min-height:100vh}
header{display:flex;align-items:center;gap:12px;padding:12px 16px;border-bottom:1px solid #1a2a3a;background:#0a1018}
header h1{flex:1;color:#4af;font-size:15px;letter-spacing:2px;text-transform:uppercase}
a.back{color:#4af;text-decoration:none;font-size:12px}
.content{padding:16px;max-width:900px;margin:auto}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(280px,1fr));gap:14px;margin-top:4px}
.card{background:#0f1720;border:1px solid #1a2a3a;border-radius:8px;padding:14px}
.card.ok   {border-left:3px solid #3f3}
.card.err  {border-left:3px solid #f55}
.card.warn {border-left:3px solid #fa0}
.card.off  {border-left:3px solid #444;opacity:.7}
.card-head{display:flex;align-items:center;gap:8px;margin-bottom:10px}
.badge{font-size:10px;font-weight:bold;padding:2px 8px;border-radius:10px;letter-spacing:1px}
.badge.ok  {background:#0a2a0a;color:#4f4}
.badge.err {background:#2a0a0a;color:#f55}
.badge.warn{background:#1a1400;color:#fa0}
.badge.off {background:#1a1a1a;color:#555}
.icon{font-size:22px;line-height:1}
.cname{font-size:13px;font-weight:bold;color:#dde;flex:1}
.row{display:flex;justify-content:space-between;align-items:center;padding:3px 0;font-size:12px;border-bottom:1px solid #111}
.row:last-child{border:none}
.lbl{color:#666}
.val{color:#4af;font-weight:bold}
.val.ok{color:#4f4}
.val.err{color:#f55}
.val.warn{color:#fa0}
.actions{display:flex;flex-wrap:wrap;gap:6px;margin-top:10px}
button{padding:5px 12px;background:#0d1520;color:#4af;border:1px solid #1a4a70;border-radius:3px;cursor:pointer;font-family:inherit;font-size:11px}
button:hover{background:#1a4a99;color:#fff}
button.danger{color:#f55;border-color:#4a1a1a}
button.success{color:#4f4;border-color:#1a4a1a}
.note{font-size:10px;color:#555;margin-top:6px;line-height:1.5}
.section-title{font-size:10px;color:#4af;letter-spacing:2px;text-transform:uppercase;margin:18px 0 8px;padding-bottom:4px;border-bottom:1px solid #1a2a3a}
.i2c-list{display:flex;flex-wrap:wrap;gap:6px;margin-top:8px}
.i2c-chip{padding:3px 8px;border-radius:3px;font-size:11px;background:#1a2030;color:#4af;border:1px solid #2a4a70}
.i2c-chip.found{background:#0a2a0a;color:#4f4;border-color:#2a6a3a}
.prog{height:4px;background:#1a2030;border-radius:2px;overflow:hidden;margin-top:4px}
.prog-fill{height:100%;background:#4af;transition:width .3s}
</style></head><body>
<header>
 <a class="back" href="/">&larr; Dashboard</a>
 <h1>&#9889; Diagnostics Capteurs</h1>
 <a class="back" href="/settings">Parametres</a>
</header>
<div class="content">

<div class="section-title">Capteurs actifs</div>
<div class="grid" id="grid-active">

 <!-- HX711 -->
 <div class="card" id="c-hx711">
  <div class="card-head">
   <span class="icon">&#9878;</span>
   <span class="cname">HX711 — Cellule de charge</span>
   <span class="badge" id="b-hx711">—</span>
  </div>
  <div class="row"><span class="lbl">GPIO DT / SCK</span><span class="val">23 / 24</span></div>
  <div class="row"><span class="lbl">Valeur brute ADU</span><span class="val" id="hx-raw">—</span></div>
  <div class="row"><span class="lbl">Force filtree</span><span class="val" id="hx-N">—</span></div>
  <div class="row"><span class="lbl">Bruit σ</span><span class="val" id="hx-std">—</span></div>
  <div class="row"><span class="lbl">Offset (tare)</span><span class="val" id="hx-off">—</span></div>
  <div class="row"><span class="lbl">Scale ADU/g</span><span class="val" id="hx-sc">—</span></div>
  <div class="prog"><div class="prog-fill" id="hx-prog" style="width:0%"></div></div>
  <div class="actions">
   <button onclick="doTare()">Tare zero</button>
   <button onclick="hxTest()">Test 5s (stabilite)</button>
  </div>
  <div class="note" id="hx-note"></div>
 </div>

 <!-- Fan -->
 <div class="card" id="c-fan">
  <div class="card-head">
   <span class="icon">&#128168;</span>
   <span class="cname">Ventilateur PWM</span>
   <span class="badge" id="b-fan">—</span>
  </div>
  <div class="row"><span class="lbl">PWM GPIO18 (pin 12)</span><span class="val">25 kHz</span></div>
  <div class="row"><span class="lbl">TACH GPIO17 (pin 11)</span><span class="val">open-collector</span></div>
  <div class="row"><span class="lbl">Duty actuel</span><span class="val" id="fan-duty">—</span></div>
  <div class="row"><span class="lbl">RPM mesure</span><span class="val" id="fan-rpm">—</span></div>
  <div class="row"><span class="lbl">Vitesse estimee</span><span class="val" id="fan-v">—</span></div>
  <div class="prog"><div class="prog-fill" id="fan-prog" style="width:0%"></div></div>
  <div class="actions">
   <button onclick="fanTest(30)">Test 30%</button>
   <button onclick="fanTest(50)">Test 50%</button>
   <button onclick="fanStop()" class="danger">Stop</button>
  </div>
  <div class="note" id="fan-note"></div>
 </div>

 <!-- Camera -->
 <div class="card" id="c-cam">
  <div class="card-head">
   <span class="icon">&#128247;</span>
   <span class="cname">Camera Pi v2 (IMX219)</span>
   <span class="badge" id="b-cam">—</span>
  </div>
  <div class="row"><span class="lbl">Connexion</span><span class="val">CSI-2 nappe 15 broches</span></div>
  <div class="row"><span class="lbl">Resolution</span><span class="val" id="cam-res">—</span></div>
  <div class="row"><span class="lbl">FPS reel</span><span class="val" id="cam-fps">—</span></div>
  <div class="row"><span class="lbl">AWB</span><span class="val" id="cam-awb">—</span></div>
  <div class="row"><span class="lbl">AE</span><span class="val" id="cam-ae">—</span></div>
  <div class="row"><span class="lbl">Flux JPEG actif</span><span class="val" id="cam-stream">—</span></div>
  <div class="actions">
   <button onclick="window.open('/export/snapshot','_blank')">Snapshot</button>
   <button onclick="window.open('/camera','_blank')">Live stream</button>
  </div>
 </div>

 <!-- LED -->
 <div class="card" id="c-led">
  <div class="card-head">
   <span class="icon">&#128161;</span>
   <span class="cname">Ruban LED WS2812B</span>
   <span class="badge" id="b-led">—</span>
  </div>
  <div class="row"><span class="lbl">GPIO10 / SPI0 MOSI (pin 19)</span><span class="val">8 MHz</span></div>
  <div class="row"><span class="lbl">SPI device</span><span class="val">/dev/spidev0.0</span></div>
  <div class="row"><span class="lbl">Animation</span><span class="val" id="led-anim">—</span></div>
  <div class="row"><span class="lbl">Luminosite</span><span class="val" id="led-bri">—</span></div>
  <div class="row"><span class="lbl">Nb LEDs config.</span><span class="val" id="led-n">—</span></div>
  <div class="row"><span class="lbl">Erreur SPI</span><span class="val" id="led-err-v">—</span></div>
  <div class="actions">
   <button onclick="ledFlash('white')">Test blanc</button>
   <button onclick="ledFlash('rgb')">Test RGB</button>
   <button onclick="ledFlash('off')">Eteindre</button>
  </div>
 </div>

</div><!-- /grid-active -->

<div class="section-title">Bus I2C — scan</div>
<div class="card" id="c-i2c">
 <div class="card-head">
  <span class="icon">&#128270;</span>
  <span class="cname">I2C bus 1 (GPIO2/3 — pins 3/5)</span>
  <span class="badge" id="b-i2c">—</span>
 </div>
 <div class="i2c-list" id="i2c-list"><span style="color:#555;font-size:11px">Scan en cours...</span></div>
 <div class="note">Adresses connues : 0x3A = Modulino Knob &nbsp;|&nbsp; 0x76/0x77 = BME280</div>
 <div class="actions"><button onclick="i2cScan()">Re-scanner</button></div>
</div>

<div class="section-title">Composants optionnels (desactives)</div>
<div class="grid" id="grid-optional">

 <!-- Modulino Knob -->
 <div class="card off" id="c-modulino">
  <div class="card-head">
   <span class="icon">&#127908;</span>
   <span class="cname">Modulino Knob (encodeur)</span>
   <span class="badge off" id="b-modulino">DESACTIVE</span>
  </div>
  <div class="row"><span class="lbl">I2C adresse</span><span class="val">0x3A</span></div>
  <div class="row"><span class="lbl">Bus</span><span class="val">I2C-1 (GPIO2/3)</span></div>
  <div class="row"><span class="lbl">Etat</span><span class="val" id="mod-state">—</span></div>
  <div class="note">Encodeur rotatif avec bouton. Temporairement debranche. L'activer dans les parametres si reconnecte.</div>
 </div>

 <!-- BME280 -->
 <div class="card off" id="c-bme">
  <div class="card-head">
   <span class="icon">&#127789;</span>
   <span class="cname">BME280 — T/P/Humidite</span>
   <span class="badge off">FUTUR</span>
  </div>
  <div class="row"><span class="lbl">I2C adresse</span><span class="val">0x76 ou 0x77</span></div>
  <div class="row"><span class="lbl">Bus</span><span class="val">I2C-1 (partage)</span></div>
  <div class="note">Non connecte. Permettrait de mesurer temperature et densite de l'air en temps reel pour corriger rho.</div>
 </div>

 <!-- Servo -->
 <div class="card off" id="c-servo">
  <div class="card-head">
   <span class="icon">&#9881;</span>
   <span class="cname">Servo SG90 — angle d'attaque</span>
   <span class="badge off">FUTUR</span>
  </div>
  <div class="row"><span class="lbl">GPIO12 / PWM hw (pin 32)</span><span class="val">50 Hz</span></div>
  <div class="row"><span class="lbl">Alimentation</span><span class="val">5V (pin 2)</span></div>
  <div class="note">Non connecte. Permettrait la polaire automatique (sweep angle alpha). A activer dans Parametres → Composants.</div>
 </div>

</div><!-- /grid-optional -->

</div><!-- /content -->
<script>
let lastState={};
const evs=new EventSource("/events");
evs.onmessage=e=>{lastState=JSON.parse(e.data);updateAll();};

async function post(u){return fetch(u,{method:"POST"});}

function badge(id,status,text){
 const el=document.getElementById(id);if(!el)return;
 el.className="badge "+status;el.textContent=text;
}
function cardClass(id,status){
 const el=document.getElementById(id);if(!el)return;
 el.className="card "+status;
}
function val(id,text,cls){
 const el=document.getElementById(id);if(!el)return;
 el.textContent=text;
 if(cls)el.className="val "+cls;
}

function updateAll(){
 const s=lastState;
 if(!s.raw&&s.raw!==0)return;

 // --- HX711 ---
 const hxOk=s.raw!==0||(s.force_std!==undefined&&s.force_std<10);
 const hxNoise=(s.force_std||0)*1000;
 const hxStable=hxNoise<5;
 badge("b-hx711",hxOk?"ok":"warn",hxOk?"OK":"VERIFIE");
 cardClass("c-hx711",hxOk?(hxStable?"ok":"warn"):"warn");
 val("hx-raw",(s.raw||0).toFixed(0)+" ADU");
 val("hx-N",(s.force_N||0).toFixed(4)+" N");
 val("hx-std",(s.force_std||0).toFixed(4)+" N ("+(hxNoise.toFixed(1))+" mN)",hxStable?"ok":"warn");
 val("hx-off",(s.offset||0).toFixed(0)+" ADU");
 val("hx-sc",(s.scale||1).toFixed(4)+" ADU/g");
 document.getElementById("hx-prog").style.width=Math.min(100,hxNoise*5)+"%";
 document.getElementById("c-hx711").querySelector(".prog-fill").style.background=hxStable?"#3f3":"#fa0";
 document.getElementById("hx-note").textContent=hxStable?"Signal stable — calibration utilisable":"Signal bruyant — verifier la cellule et les connexions DT/SCK";

 // --- Fan ---
 const fanRpm=s.fan_rpm||0;
 const fanDuty=s.fan_duty||0;
 const fanTachOk=fanDuty>5&&fanRpm>50;
 const fanOk=true; // PWM out toujours OK si dashboard tourne
 badge("b-fan",fanTachOk?"ok":(fanDuty>5?"warn":"ok"),fanTachOk?"OK":(fanDuty>5?"TACH?":"ARRETE"));
 cardClass("c-fan",fanTachOk?"ok":(fanDuty>5?"warn":"ok"));
 val("fan-duty",fanDuty.toFixed(1)+"%");
 val("fan-rpm",fanRpm+" RPM",fanTachOk?"ok":(fanDuty>5?"warn":""));
 const vEff=_getVFromState(s);
 val("fan-v",vEff.toFixed(2)+" m/s ("+_vsrc+")");
 document.getElementById("fan-prog").style.width=fanDuty+"%";
 document.getElementById("fan-note").textContent=fanDuty>5&&!fanTachOk
  ?"Ventilateur tourne mais pas de signal TACH — verifier fil blanc GPIO17"
  :(fanDuty>5?"Tachymetre OK":"");

 // --- Camera ---
 const camOk=(s.cam_fps||0)>5;
 badge("b-cam",camOk?"ok":"err",camOk?"OK":"ERREUR");
 cardClass("c-cam",camOk?"ok":"err");
 val("cam-res","1280 x 960");
 val("cam-fps",((s.cam_fps)||0).toFixed(1)+" fps",camOk?"ok":"err");
 val("cam-awb",s.cam_awb?"AUTO (actif)":"Manuel (gains fixes)",s.cam_awb?"warn":"ok");
 val("cam-ae",s.cam_ae?"AUTO":"Manuel");
 val("cam-stream",camOk?"Actif":"Inactif",camOk?"ok":"err");

 // --- LED ---
 const ledSt=s.led_ok!==false;
 badge("b-led",ledSt?"ok":"err",ledSt?"OK":"ERREUR SPI");
 cardClass("c-led",ledSt?"ok":"err");
}

// --- I2C scan ---
async function i2cScan(){
 document.getElementById("i2c-list").innerHTML='<span style="color:#fa0;font-size:11px">Scan...</span>';
 badge("b-i2c","warn","SCAN...");
 const d=await fetch("/sensors/i2c").then(r=>r.json()).catch(()=>null);
 if(!d){badge("b-i2c","err","ERREUR");return;}
 const known={"0x3a":"Modulino Knob","0x76":"BME280","0x77":"BME280","0x48":"ADS1115","0x68":"MPU6050"};
 const list=document.getElementById("i2c-list");
 if(!d.devices||d.devices.length===0){
  list.innerHTML='<span style="color:#555;font-size:11px">Aucun peripherique detecte</span>';
  badge("b-i2c","warn","VIDE");
  return;
 }
 list.innerHTML="";
 d.devices.forEach(addr=>{
  const lbl=known[addr.toLowerCase()]||"inconnu";
  list.innerHTML+=`<span class="i2c-chip found">${addr} — ${lbl}</span>`;
 });
 badge("b-i2c","ok",d.devices.length+" trouves");
 // update modulino status
 const modFound=d.devices.some(a=>a.toLowerCase()==="0x3a");
 document.getElementById("mod-state").textContent=modFound?"DETECTE sur I2C":"Non detecte";
 document.getElementById("mod-state").className="val "+(modFound?"ok":"warn");
 document.getElementById("c-modulino").className="card "+(modFound?"warn":"off");
}

// --- Actions ---
async function doTare(){
 await post("/tare");
 document.getElementById("hx-note").textContent="Tare effectuee ✓";
}
async function hxTest(){
 const note=document.getElementById("hx-note");
 note.textContent="Mesure stabilite 5s...";
 await new Promise(r=>setTimeout(r,5000));
 const s=lastState;
 const noise=(s.force_std||0)*1000;
 note.textContent="Stabilite : σ = "+noise.toFixed(2)+" mN — "+(noise<5?"Excellent":"Bruit important, verifier connexions");
}
async function fanTest(duty){
 await post("/fan/set?duty="+duty);
 document.getElementById("fan-note").textContent="Test "+duty+"% lance — verifier rotation...";
}
async function fanStop(){
 await post("/fan/set?duty=0");
 document.getElementById("fan-note").textContent="Arrete";
}
async function ledFlash(mode){
 if(mode==="white") await post("/led/set?anim=solid&r=255&g=255&b=255&bri=200");
 else if(mode==="rgb"){
  await post("/led/set?anim=rainbow&bri=150");
  setTimeout(()=>post("/led/set?anim=solid&r=0&g=120&b=255&bri=80"),3000);
 } else await post("/led/set?anim=off");
}

// LED status via polling
async function pollLed(){
 const j=await fetch("/led").then(r=>r.json()).catch(()=>null);
 if(!j)return;
 document.getElementById("led-anim").textContent=j.animation||"—";
 document.getElementById("led-bri").textContent=(j.brightness||0)+" / 255";
 document.getElementById("led-n").textContent=(j.n_leds||0)+" LEDs";
 const errEl=document.getElementById("led-err-v");
 errEl.textContent=j.error||"Aucune";
 errEl.className="val "+(j.error?"err":"ok");
 badge("b-led",j.ok?"ok":"err",j.ok?"OK":"ERREUR SPI");
 document.getElementById("c-led").className="card "+(j.ok?"ok":"err");
}

// Init
setInterval(pollLed,3000);
pollLed();
i2cScan();

// Expose cam_fps via state (add to health endpoint if missing)
evs.addEventListener("message",()=>{});
</script></body></html>"""


# ==================== MEDIA LIBRARY PAGE ====================

MEDIA_HTML = r"""<!DOCTYPE html>
<html lang="fr"><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Médias — Soufflerie ENSAM</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#0a0a0a;color:#e0e0e0;font-family:ui-monospace,Menlo,monospace;min-height:100vh}
header{display:flex;align-items:center;gap:10px;padding:8px 14px;background:#111;border-bottom:1px solid #222;height:44px}
.logo{font-size:14px;font-weight:bold;letter-spacing:3px;color:#4af;margin-right:8px}
.btn{padding:5px 11px;border:1px solid #333;background:#1a1a1a;color:#ccc;border-radius:4px;cursor:pointer;font-family:inherit;font-size:11px;letter-spacing:.5px;transition:border-color .2s;text-decoration:none;display:inline-block}
.btn:hover{border-color:#4af;color:#fff}
.btn-del{border-color:#500 !important;color:#f55 !important}
.btn-del:hover{border-color:#f44 !important;background:#300 !important}
.btn-dl{border-color:#250 !important;color:#5d5 !important}
.btn-dl:hover{border-color:#4f4 !important}
.btn-analyze{border-color:#440 !important;color:#ff8 !important}
.btn-analyze:hover{border-color:#ff8 !important;background:#221 !important}
.progress-bar{height:3px;background:#ff8;margin-top:5px;border-radius:2px;transition:width .3s}
.progress-wrap{background:#222;border-radius:2px;margin-top:5px;overflow:hidden;height:3px}
.converting-badge{font-size:9px;color:#fa0;letter-spacing:.5px;margin-top:4px;display:flex;align-items:center;gap:5px}
@keyframes spin{to{transform:rotate(360deg)}}
.spinner{display:inline-block;width:10px;height:10px;border:2px solid #fa0;border-top-color:transparent;border-radius:50%;animation:spin .7s linear infinite}
main{padding:16px;max-width:1400px;margin:0 auto}
.sec-title{font-size:9px;letter-spacing:2.5px;color:#c8f;text-transform:uppercase;margin:20px 0 10px;display:flex;align-items:center;gap:10px}
.sec-title span{color:#666;font-size:9px}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(210px,1fr));gap:10px}
.card{background:#141414;border:1px solid #222;border-radius:6px;overflow:hidden;transition:border-color .2s}
.card:hover{border-color:#3a1a5a}
.thumb{width:100%;height:126px;background:#0d0d0d;display:flex;align-items:center;justify-content:center;cursor:pointer;position:relative;overflow:hidden;border-bottom:1px solid #1a1a1a}
.thumb img{width:100%;height:100%;object-fit:cover;transition:opacity .2s}
.thumb:hover img{opacity:.85}
.play-overlay{position:absolute;inset:0;display:flex;align-items:center;justify-content:center;background:rgba(0,0,0,.35)}
.play-icon{width:42px;height:42px;background:rgba(200,130,255,.9);border-radius:50%;display:flex;align-items:center;justify-content:center;font-size:16px;padding-left:3px}
.meta{padding:8px 10px 9px}
.fname{font-size:11px;color:#ccc;white-space:nowrap;overflow:hidden;text-overflow:ellipsis;margin-bottom:3px}
.finfo{font-size:9px;color:#555;margin-bottom:7px}
.actions{display:flex;gap:5px}
.actions .btn{flex:1;text-align:center;padding:4px 0;font-size:10px}
.empty{color:#444;font-size:11px;padding:18px 0;letter-spacing:.5px}
/* Modal */
#modal{display:none;position:fixed;inset:0;background:rgba(0,0,0,.88);z-index:100;align-items:center;justify-content:center}
#modal.open{display:flex}
#modal-content{position:relative;max-width:92vw;max-height:88vh}
#modal video{max-width:90vw;max-height:82vh;border-radius:6px;display:block}
#modal img{max-width:90vw;max-height:82vh;border-radius:6px;display:block}
#modal-close{position:absolute;top:-30px;right:0;font-size:20px;color:#888;cursor:pointer;padding:4px 8px;background:#222;border-radius:4px}
#modal-close:hover{color:#fff;background:#333}
.storage-info{font-size:9px;color:#555;margin-left:auto;letter-spacing:.5px}
</style>
</head>
<body>
<header>
  <span class="logo">ENSAM</span>
  <a href="/" class="btn">← Dashboard</a>
  <a href="/sensors" class="btn" style="border-color:#1a4a1a;color:#4f4">&#9889; Capteurs</a>
  <button class="btn" onclick="loadMedia()" style="border-color:#3a1a5a;color:#c8f">↻ Actualiser</button>
  <span class="storage-info" id="storage-info"></span>
</header>
<main>
  <div class="sec-title">&#127916; VIDÉOS <span id="cnt-v"></span></div>
  <div class="grid" id="grid-v"><p class="empty">Chargement...</p></div>

  <div class="sec-title" style="margin-top:28px">&#128247; PHOTOS <span id="cnt-p"></span></div>
  <div class="grid" id="grid-p"><p class="empty">Chargement...</p></div>
</main>

<div id="modal">
  <div id="modal-content">
    <span id="modal-close" onclick="closeModal()">✕ Fermer</span>
    <video id="modal-video" controls style="display:none"></video>
    <img id="modal-img" style="display:none" alt="">
  </div>
</div>

<div id="analyze-modal" style="display:none;position:fixed;inset:0;background:rgba(0,0,0,.78);z-index:300;align-items:center;justify-content:center">
  <div style="background:#0e0e0e;border:1px solid #2a2a2a;border-radius:8px;padding:22px 24px;width:350px;max-width:94vw">
    <div style="font-size:14px;font-weight:600;color:#ccc;margin-bottom:16px">⚡ Analyse flux optique</div>
    <div style="font-size:11px;color:#666;margin-bottom:6px;text-transform:uppercase;letter-spacing:.06em">Mode de visualisation</div>
    <div id="am-modes" style="display:flex;flex-wrap:wrap;gap:6px;margin-bottom:14px"></div>
    <div id="am-dir-row" style="display:none">
      <div style="font-size:11px;color:#666;margin-bottom:6px;text-transform:uppercase;letter-spacing:.06em">Direction filtrée</div>
      <div id="am-dirs" style="display:flex;gap:6px;margin-bottom:14px"></div>
    </div>
    <div id="am-desc" style="font-size:11px;color:#777;margin-bottom:18px;min-height:30px;line-height:1.5"></div>
    <div style="display:flex;gap:8px">
      <button onclick="launchAnalyze()" style="flex:1;background:#1a3a1a;color:#8f8;border:1px solid #3a6a3a;padding:8px;border-radius:4px;cursor:pointer;font-weight:600">⚡ Lancer</button>
      <button onclick="closeAnalyzeModal()" style="background:#1a1a1a;color:#888;border:1px solid #333;padding:8px 14px;border-radius:4px;cursor:pointer">Annuler</button>
    </div>
  </div>
</div>

<script>
function fmtSize(b){
  if(b>1e6) return (b/1e6).toFixed(1)+' Mo';
  if(b>1e3) return (b/1e3).toFixed(0)+' Ko';
  return b+' o';
}

function renderGrid(gridId, cntId, files, type){
  const cntEl = document.getElementById(cntId);
  cntEl.textContent = '— '+files.length+' fichier'+(files.length>1?'s':'');
  const g = document.getElementById(gridId);
  if(!files.length){g.innerHTML='<p class="empty">Aucun fichier</p>';return;}
  g.innerHTML = files.map(f=>`
    <div class="card" id="card-${f.name.replace(/\W/g,'_')}">
      <div class="thumb" onclick="openMedia('${escQ(f.name)}','${type}')">
        ${type==='photo'
          ? `<img src="/media/file/${encodeURIComponent(f.name)}" loading="lazy" alt="">`
          : f.converting
            ? `<div class="play-overlay" style="background:rgba(0,0,0,.6)"><div class="spinner" style="width:32px;height:32px;border-width:3px"></div></div>`
            : `<div class="play-overlay"><div class="play-icon">▶</div></div>`}
      </div>
      <div class="meta">
        <div class="fname" title="${f.name}">${f.name}</div>
        <div class="finfo">${f.mtime} · ${fmtSize(f.size)}</div>
        ${f.converting ? `<div class="converting-badge"><span class="spinner"></span> Conversion MP4…</div>` : ''}
        ${f.analyzing  ? `<div class="converting-badge" style="color:#ff8"><span class="spinner"></span> Analyse flux… ${(f.analyzing.progress*100).toFixed(0)}%${f.analyzing.status?' — '+f.analyzing.status:''}</div>` : ''}
        ${f.conv_err ? `<div style="font-size:9px;color:#f55;margin-top:3px">${f.conv_err}</div>` : ''}
        ${f.analyzing ? `<div class="progress-wrap" style="display:block"><div class="progress-bar" style="width:${(f.analyzing.progress*100).toFixed(0)}%;background:#ff8"></div></div>` : ''}
        <div class="actions" style="margin-top:6px">
          ${type==='video' && !f.name.includes('_flow_') && !f.converting && !f.analyzing ? `<button class="btn btn-analyze" onclick="analyzeVideo('${escQ(f.name)}')">⚡ Flux</button>` : ''}
          ${!f.converting && !f.analyzing ? `<a class="btn btn-dl" href="/media/file/${encodeURIComponent(f.name)}" download="${f.name}">⬇ vidéo</a>` : ''}
          ${f.csv_name ? `<a class="btn btn-dl" style="border-color:#484;color:#8f8" href="/media/file/${encodeURIComponent(f.csv_name)}" download="${f.csv_name}" title="Télécharger données CSV">📊 CSV</a>` : ''}
          <button class="btn btn-del" onclick="delFile('${escQ(f.name)}','${type}')">🗑</button>
        </div>
      </div>
    </div>`).join('');
}

function escQ(s){ return s.replace(/'/g,"\\'"); }

function openMedia(name, type){
  const vid = document.getElementById('modal-video');
  const img = document.getElementById('modal-img');
  if(type==='video'){
    vid.src = '/media/file/'+encodeURIComponent(name);
    vid.style.display='block'; img.style.display='none';
    vid.play().catch(()=>{});
  } else {
    img.src = '/media/file/'+encodeURIComponent(name);
    img.style.display='block'; vid.style.display='none';
  }
  document.getElementById('modal').classList.add('open');
}

function closeModal(){
  const vid = document.getElementById('modal-video');
  vid.pause(); vid.src='';
  document.getElementById('modal').classList.remove('open');
}

document.getElementById('modal').addEventListener('click', e=>{
  if(e.target===document.getElementById('modal')) closeModal();
});

const AM_MODES = {
  heatmap:     {label:'🌡 Chaleur',         desc:'Teinte=direction, brillance=magnitude. Vue d\'ensemble du champ 2D.'},
  arrows:      {label:'↗ Flèches',          desc:'Vecteurs décimés. Lisible sur écoulements structurés.'},
  vorticity:   {label:'🌀 Tourbillons',     desc:'Rouge=CCW, bleu=CW. Détecte les tourbillons de sillage.'},
  streamlines: {label:'〜 Lignes courant',  desc:'Trajectoires intégrées RK2. Vue globale de l\'écoulement.'},
  flow_x:      {label:'↔ Axe X seul',      desc:'Composante horizontale uniquement. Rouge=aval (+X), bleu=retour (−X). Élimine les perturbations verticales.'},
  flow_y:      {label:'↕ Axe Y seul',      desc:'Composante verticale uniquement. Vert=bas (+Y), violet=haut (−Y). Isole les effets de portance/déflexion.'},
};
const AM_DIRS = {
  all: {label:'± Toutes',       desc:'Flux positif et négatif visibles.'},
  pos: {label:'+ Aval / Bas',   desc:'Affiche uniquement le flux dans la direction positive.'},
  neg: {label:'− Retour / Haut',desc:'Affiche uniquement le flux dans la direction négative.'},
};
let _analyzeTarget=null, _analyzeMode='heatmap', _analyzeDir='all';

function openAnalyzeModal(name){
  _analyzeTarget=name; _analyzeMode='heatmap'; _analyzeDir='all';
  _renderAmModes(); _renderAmDirs();
  document.getElementById('analyze-modal').style.display='flex';
}
function closeAnalyzeModal(){ document.getElementById('analyze-modal').style.display='none'; }
function setAMode(m){ _analyzeMode=m; _renderAmModes(); _renderAmDirs(); }
function setADir(d){ _analyzeDir=d; _renderAmDirs(); }

function _chip(k,active,color,onclick,label){
  return `<span onclick="${onclick}('${k}')" style="padding:5px 10px;border-radius:4px;cursor:pointer;font-size:12px;border:1px solid ${active?color:'#2a2a2a'};color:${active?color:'#888'};background:${active?'#111824':'#111'};white-space:nowrap">${label}</span>`;
}
function _renderAmModes(){
  document.getElementById('am-modes').innerHTML=Object.entries(AM_MODES).map(([k,v])=>_chip(k,k===_analyzeMode,'#5af','setAMode',v.label)).join('');
  document.getElementById('am-desc').textContent=AM_MODES[_analyzeMode]?.desc||'';
  const d=_analyzeMode==='flow_x'||_analyzeMode==='flow_y';
  document.getElementById('am-dir-row').style.display=d?'':'none';
  if(!d) _analyzeDir='all';
}
function _renderAmDirs(){
  document.getElementById('am-dirs').innerHTML=Object.entries(AM_DIRS).map(([k,v])=>_chip(k,k===_analyzeDir,'#fa0','setADir',v.label)).join('');
}
async function launchAnalyze(){
  if(!_analyzeTarget) return;
  closeAnalyzeModal();
  const url=`/media/analyze?name=${encodeURIComponent(_analyzeTarget)}&mode=${_analyzeMode}&dir=${_analyzeDir}`;
  const r=await fetch(url,{method:'POST'}).then(r=>r.json()).catch(()=>({ok:false}));
  if(!r.ok){ alert('Erreur: '+(r.err||'?')); return; }
  loadMedia();
}
// alias pour le bouton dans la grille
function analyzeVideo(name){ openAnalyzeModal(name); }

document.getElementById('analyze-modal').addEventListener('click',e=>{
  if(e.target===document.getElementById('analyze-modal')) closeAnalyzeModal();
});

async function delFile(name, type){
  if(!confirm('Supprimer '+name+' ?')) return;
  const r = await fetch('/media/delete?name='+encodeURIComponent(name)+'&type='+type,{method:'POST'})
    .then(r=>r.json()).catch(()=>({ok:false}));
  if(r.ok) loadMedia();
  else alert('Erreur : '+(r.err||'inconnue'));
}

document.addEventListener('keydown', e=>{ if(e.key==='Escape') closeModal(); });
document.addEventListener('visibilitychange', ()=>{ if(!document.hidden) loadMedia(); });

let _autoRefresh = null;
function scheduleRefresh(hasConverting, hasAnalyzing){
  clearInterval(_autoRefresh);
  const active = hasConverting || hasAnalyzing;
  if(!active) return;
  // Refresh rapide (~800ms) pendant une analyse, plus lent (2.5s) pour conversion seule
  const delay = hasAnalyzing ? 800 : 2500;
  _autoRefresh = setInterval(async ()=>{
    const d = await fetch('/media/list').then(r=>r.json()).catch(()=>({videos:[],photos:[]}));
    const vids = d.videos||[];
    renderGrid('grid-v','cnt-v', vids, 'video');
    renderGrid('grid-p','cnt-p', d.photos||[], 'photo');
    const stillActive = vids.some(v=>v.converting||v.analyzing);
    if(!stillActive){ clearInterval(_autoRefresh); _autoRefresh=null; }
  }, delay);
}

async function loadMedia(){
  document.getElementById('grid-v').innerHTML='<p class="empty">Chargement...</p>';
  document.getElementById('grid-p').innerHTML='<p class="empty">Chargement...</p>';
  const d = await fetch('/media/list').then(r=>r.json()).catch(()=>({videos:[],photos:[],storage:{}}));
  const vids = d.videos||[];
  renderGrid('grid-v','cnt-v', vids, 'video');
  renderGrid('grid-p','cnt-p', d.photos||[], 'photo');
  const si = d.storage||{};
  if(si.used_mb) document.getElementById('storage-info').textContent = `Stockage : ${si.used_mb} Mo`;
  scheduleRefresh(vids.some(v=>v.converting), vids.some(v=>v.analyzing));
}

loadMedia();
</script>
</body></html>"""


# ==================== HTTP ====================


def _save_config():
    """Ecrit offset, scale et autres params dans config.py. Retourne (ok, err)."""
    import re as _re
    cfg = os.path.join(BASE_DIR, "config.py")
    try:
        with open(cfg) as f:
            txt = f.read()
        with state_lock:
            o, s, p = state["offset"], state["scale"], state["px_per_m"]
            c, sp, rho = state["chord_m"], state["span_m"], state["rho"]
            kv, k0 = state["fan_kv"], state["fan_k0"]
            drag = state["support_drag_N"]
        txt = _re.sub(r"HX711_OFFSET\s*=\s*\S+", f"HX711_OFFSET = {int(o)}", txt)
        txt = _re.sub(r"HX711_SCALE\s*=\s*\S+",  f"HX711_SCALE = {s:.6f}", txt)
        txt = _re.sub(r"CAM_PX_PER_M\s*=\s*\S+", f"CAM_PX_PER_M = {p:.1f}", txt)
        txt = _re.sub(r"PROFILE_CHORD_M\s*=\s*\S+", f"PROFILE_CHORD_M = {c:.4f}", txt)
        txt = _re.sub(r"PROFILE_SPAN_M\s*=\s*\S+",  f"PROFILE_SPAN_M = {sp:.4f}", txt)
        txt = _re.sub(r"AIR_DENSITY\s*=\s*\S+",  f"AIR_DENSITY = {rho:.4f}", txt)
        txt = _re.sub(r"FAN_KV\s*=\s*\S+", f"FAN_KV = {kv:.6f}", txt)
        txt = _re.sub(r"FAN_K0\s*=\s*\S+", f"FAN_K0 = {k0:.4f}", txt)
        txt = _re.sub(r"SUPPORT_DRAG_N\s*=\s*\S+", f"SUPPORT_DRAG_N = {drag:.6f}", txt)
        with open(cfg, "w") as f:
            f.write(txt)
        return True, ""
    except Exception as e:
        return False, str(e)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a, **k): pass

    def _send(self, body, ct="text/html; charset=utf-8", code=200):
        if isinstance(body, str):
            body = body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ct)
        self.send_header("Content-Length", len(body))
        self.end_headers()
        self.wfile.write(body)

    def _json(self, obj, code=200):
        self._send(json.dumps(obj), "application/json", code)

    def _serve_file(self, filepath, content_type):
        """Sert un fichier avec support des Range requests (nécessaire pour la lecture vidéo)."""
        if not os.path.exists(filepath):
            self.send_response(404); self.end_headers(); return
        size = os.path.getsize(filepath)
        range_hdr = self.headers.get("Range", "")
        if range_hdr.startswith("bytes="):
            parts = range_hdr[6:].split("-")
            start = int(parts[0]) if parts[0] else 0
            end   = int(parts[1]) if len(parts) > 1 and parts[1] else size - 1
            end   = min(end, size - 1)
            length = end - start + 1
            self.send_response(206)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(length))
            self.send_header("Content-Range", f"bytes {start}-{end}/{size}")
            self.send_header("Accept-Ranges", "bytes")
            self.end_headers()
            with open(filepath, "rb") as f:
                f.seek(start)
                remaining = length
                while remaining > 0:
                    chunk = f.read(min(65536, remaining))
                    if not chunk:
                        break
                    self.wfile.write(chunk)
                    remaining -= len(chunk)
        else:
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(size))
            self.send_header("Accept-Ranges", "bytes")
            self.end_headers()
            with open(filepath, "rb") as f:
                while True:
                    chunk = f.read(65536)
                    if not chunk:
                        break
                    self.wfile.write(chunk)

    def do_GET(self):
        if not request_allowed(self.headers.get("Host"), self.headers.get("Origin"),
                               self.headers.get("Sec-Fetch-Site"), HTTP_BIND_HOST):
            self.send_error(403); return
        path = urlparse(self.path).path
        # Captive portal detection (iOS, Android, Windows) → redirect to dashboard
        if path in ("/hotspot-detect.html", "/library/test/success.html",
                    "/generate_204", "/connecttest.txt", "/ncsi.txt",
                    "/redirect", "/canonical.html"):
            self.send_response(302)
            self.send_header("Location", "/")
            self.end_headers()
            return
        if path == "/":
            self._send(HTML)
        elif path == "/settings":
            self._send(SETTINGS_HTML)
        elif path == "/camera":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    # Attend qu'un nouveau frame soit disponible (max 200ms timeout)
                    frame_event.wait(timeout=0.2)
                    frame_event.clear()
                    with frame_lock:
                        frame = latest_jpeg
                    if frame:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n")
                        self.wfile.write(frame)
                        self.wfile.write(b"\r\n")
            except Exception:
                pass
        elif path == "/events":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            try:
                while True:
                    with state_lock:
                        d = dict(state)
                    self.wfile.write(("data: " + json.dumps(d) + "\n\n").encode())
                    self.wfile.flush()
                    time.sleep(0.1)
            except Exception:
                pass

        elif path == "/export/csv":
            snap = list(data_buffer)
            buf = io.StringIO()
            # En-tete metadonnees (preserve la calibration et le contexte)
            with state_lock:
                meta = {
                    "exported_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                    "session_start": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(session_start)),
                    "samples": len(snap),
                    "sample_rate_hz": FORCE_SAMPLE_HZ,
                    "hx711_offset": state["offset"],
                    "hx711_scale_adu_per_g": state["scale"],
                    "chord_m": state["chord_m"],
                    "span_m": state["span_m"],
                    "aoa_deg": state["aoa_deg"],
                    "rho_kg_m3": state["rho"],
                    "px_per_m": state["px_per_m"],
                    "fan_kv_ms_per_rpm": state["fan_kv"],
                    "fan_k0_ms": state["fan_k0"],
                    "cam_mode": state["cam_mode"],
                }
            for k, v in meta.items():
                buf.write(f"# {k}: {v}\n")
            buf.write("#\n")
            w = csv.writer(buf)
            w.writerow(["t_s","duty_pct","fan_rpm","delta_adu","force_g","force_N","flow_mag_px","airspeed_flow_ms","airspeed_fan_ms"])
            for r in snap:
                w.writerow([r["t"],r["duty"],r["rpm"],r["delta_adu"],r["force_g"],r["force_N"],r["flow_mag"],r["airspeed_ms"],r.get("airspeed_fan_ms",0)])
            body = buf.getvalue().encode("utf-8")
            fname = time.strftime("soufflerie_%Y%m%d_%H%M%S.csv")
            self.send_response(200)
            self.send_header("Content-Type", "text/csv")
            self.send_header("Content-Disposition", f'attachment; filename="{fname}"')
            self.send_header("Content-Length", len(body))
            self.end_headers()
            self.wfile.write(body)

        elif path == "/export/json":
            # export JSON complet : metadonnees + buffer + dernier etat
            with state_lock:
                snapshot = dict(state)
            payload = {
                "meta": {
                    "exported_at": time.time(),
                    "session_start": session_start,
                    "sample_rate_hz": FORCE_SAMPLE_HZ,
                    "cam_fps": CAM_FPS,
                },
                "state": snapshot,
                "samples": list(data_buffer),
            }
            body = json.dumps(payload, indent=1).encode("utf-8")
            fname = time.strftime("soufflerie_%Y%m%d_%H%M%S.json")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Disposition", f'attachment; filename="{fname}"')
            self.send_header("Content-Length", len(body))
            self.end_headers()
            self.wfile.write(body)

        elif path == "/export/snapshot":
            with frame_lock:
                frame = bytes(latest_jpeg)
            fname = time.strftime("snap_%Y%m%d_%H%M%S.jpg")
            try:
                with open(os.path.join(DATA_DIR_PHOTOS, fname), "wb") as fp:
                    fp.write(frame)
            except Exception:
                pass
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Disposition", f'attachment; filename="{fname}"')
            self.send_header("Content-Length", len(frame))
            self.end_headers()
            self.wfile.write(frame)

        elif path == "/export/count":
            self._json({"n": len(data_buffer)})

        elif path == "/calib":
            self._send(CALIB_HTML)

        elif path == "/calib/status":
            with fan_calib_lock:
                d = dict(fan_calib_state)
            with state_lock:
                d["fan_drag_table"] = state.get("fan_drag_table", [])
            self._json(d)

        elif path == "/etude":
            self._send(ETUDE_HTML)

        elif path == "/etude/report":
            import json as _json
            eid = qs.get("id", [""])[0]
            if not eid or "/" in eid or ".." in eid:
                self._send(b"id invalide", ct="text/plain"); return
            fpath = os.path.join(DATA_DIR_ETUDES, f"{eid}.json")
            if not os.path.isfile(fpath):
                self._send(b"etude introuvable", ct="text/plain"); return
            with open(fpath) as f:
                data = _json.load(f)
            html_r = _build_etude_report(data)
            self._send(html_r.encode("utf-8"), ct="text/html; charset=utf-8")

        elif path == "/etude/list":
            import json as _json
            studies = []
            try:
                for fname in sorted(os.listdir(DATA_DIR_ETUDES), reverse=True):
                    if fname.endswith(".json"):
                        try:
                            with open(os.path.join(DATA_DIR_ETUDES, fname)) as f:
                                d = _json.load(f)
                            studies.append({"id": d.get("id", fname[:-5]),
                                            "saved_at": d.get("saved_at", ""),
                                            "mode": d.get("mode", ""),
                                            "object": d.get("object", "")})
                        except Exception:
                            pass
            except Exception:
                pass
            self._json({"ok": True, "studies": studies})

        elif path == "/sensors":
            self._send(SENSORS_HTML)

        elif path == "/media":
            self._send(MEDIA_HTML)

        elif path == "/media/list":
            videos, photos = [], []
            try:
                mp4_names = set()
                entries = sorted(os.listdir(DATA_DIR_VIDEO), reverse=True)
                for f in entries:
                    if f.endswith(".mp4"):
                        mp4_names.add(f)
                for f in entries:
                    fp = os.path.join(DATA_DIR_VIDEO, f)
                    if f.endswith(".mp4"):
                        with _conv_lock:
                            conv = dict(_conversions.get(f, {}))
                        converting = not conv.get("done", True) if conv else False
                        # Statut d'analyse en cours (source video -> job keyed by fname)
                        with _video_analysis_lock:
                            job = dict(_video_analysis_jobs.get(f, {}))
                        analyzing = None
                        if job and not job.get("done", True):
                            analyzing = {"progress": job.get("progress", 0.0),
                                         "status": job.get("status", "")}
                        # CSV d'analyse disponible (pour les vidéos _flow_)
                        csv_fname = f.replace(".mp4", ".csv")
                        csv_exists = os.path.isfile(os.path.join(DATA_DIR_VIDEO, csv_fname))
                        videos.append({
                            "name": f, "size": os.path.getsize(fp),
                            "mtime": time.strftime("%d/%m/%Y %H:%M",
                                                   time.localtime(os.path.getmtime(fp))),
                            "converting": converting,
                            "conv_err": conv.get("err", ""),
                            "analyzing": analyzing,
                            "csv_name": csv_fname if csv_exists else "",
                        })
                    elif f.endswith(".h264"):
                        # Fichier en cours ou en attente de conversion
                        mp4_equiv = f.replace(".h264", ".mp4")
                        if mp4_equiv not in mp4_names:
                            with _conv_lock:
                                conv = dict(_conversions.get(mp4_equiv, {}))
                            converting = not conv.get("done", True)
                            videos.append({
                                "name": mp4_equiv, "size": os.path.getsize(fp),
                                "mtime": time.strftime("%d/%m/%Y %H:%M",
                                                       time.localtime(os.path.getmtime(fp))),
                                "converting": True,
                                "conv_err": "",
                            })
            except Exception:
                pass
            try:
                for f in sorted(os.listdir(DATA_DIR_PHOTOS), reverse=True):
                    if f.endswith((".jpg", ".jpeg", ".png")):
                        fp = os.path.join(DATA_DIR_PHOTOS, f)
                        photos.append({
                            "name": f, "size": os.path.getsize(fp),
                            "mtime": time.strftime("%d/%m/%Y %H:%M",
                                                   time.localtime(os.path.getmtime(fp))),
                        })
            except Exception:
                pass
            # Calcul usage disque approximatif
            used_mb = 0
            try:
                for d_, files_ in [(DATA_DIR_VIDEO, videos), (DATA_DIR_PHOTOS, photos)]:
                    used_mb += sum(f["size"] for f in files_)
                used_mb = round(used_mb / 1e6, 1)
            except Exception:
                pass
            self._json({"videos": videos[:50], "photos": photos[:100],
                        "storage": {"used_mb": used_mb}})

        elif path == "/media/analyze/status":
            u = urlparse(self.path)
            fname = parse_qs(u.query).get("name", [""])[0]
            with _video_analysis_lock:
                job = dict(_video_analysis_jobs.get(fname, {}))
            self._json(job if job else {"err": "job inconnu"})

        elif path.startswith("/media/file/"):
            fname = path[len("/media/file/"):]
            # Cherche d'abord dans videos, puis photos
            vpath = os.path.join(DATA_DIR_VIDEO, fname)
            ppath = os.path.join(DATA_DIR_PHOTOS, fname)
            if os.path.isfile(vpath):
                if fname.endswith(".mp4"):
                    ct = "video/mp4"
                elif fname.endswith(".csv"):
                    ct = "text/csv"
                else:
                    ct = "application/octet-stream"
                self._serve_file(vpath, ct)
            elif os.path.isfile(ppath):
                ct = "image/jpeg" if fname.lower().endswith((".jpg", ".jpeg")) else "image/png"
                self._serve_file(ppath, ct)
            else:
                self.send_response(404); self.end_headers()

        elif path == "/sensors/i2c":
            try:
                r = subprocess.run(
                    ["i2cdetect", "-y", "1"],
                    capture_output=True, text=True, timeout=5
                )
                devices = []
                for line in r.stdout.splitlines():
                    parts = line.split(":")[1:]
                    for part in parts:
                        for tok in part.split():
                            if tok not in ("--", "UU") and len(tok) == 2:
                                try:
                                    int(tok, 16)
                                    # reconstruct full address from row
                                    row_prefix = line.split(":")[0].strip()
                                    col_idx = part.split().index(tok)
                                    # row_prefix is the MSB (e.g. "30" means 0x30-0x3f range)
                                    # col_idx is the LSB nibble offset
                                    addr = int(row_prefix, 16) + col_idx
                                    devices.append(f"0x{addr:02x}")
                                except Exception:
                                    pass
                self._json({"devices": devices, "raw": r.stdout})
            except Exception as ex:
                self._json({"devices": [], "error": str(ex)})

        elif path == "/etude/status":
            with etude_lock:
                self._json(dict(etude))

        elif path == "/led":
            self._json(led.status())

        elif path == "/health":
            info = {
                "uptime_s": round(time.time() - session_start, 1),
                "data_buffer": {"used": len(data_buffer), "max": data_buffer.maxlen,
                                "pct": round(100 * len(data_buffer) / data_buffer.maxlen, 1)},
                "force_buf_pct": round(100 * len(force_buf) / 256, 1),
                "spectro_pct": round(100 * len(spectro.cols) / spectro.n_cols, 1),
                "experiment_active": experiment["running"],
                "recording_active": recording["active"],
                "auto_tare_enabled": auto_tare["enabled"],
                "auto_tare_count": auto_tare["count"],
            }
            # CPU temperature
            try:
                with open("/sys/class/thermal/thermal_zone0/temp") as f:
                    info["cpu_temp_c"] = round(int(f.read().strip()) / 1000, 1)
            except Exception:
                info["cpu_temp_c"] = None
            # Memory
            try:
                with open("/proc/meminfo") as f:
                    lines = {l.split(":")[0]: l.split(":")[1].strip().split()[0]
                             for l in f.readlines()[:5]}
                mem_total = int(lines["MemTotal"])
                mem_avail = int(lines.get("MemAvailable", "0"))
                info["mem_used_pct"] = round(100 * (1 - mem_avail / mem_total), 1)
                info["mem_total_mb"] = round(mem_total / 1024, 0)
            except Exception:
                info["mem_used_pct"] = None
            # Camera FPS reel
            if len(_cam_fps_buf) > 5:
                dt = _cam_fps_buf[-1] - _cam_fps_buf[0]
                info["cam_fps"] = round((len(_cam_fps_buf) - 1) / dt, 1) if dt > 0 else 0
            else:
                info["cam_fps"] = 0
            # Disk
            try:
                import shutil
                u = shutil.disk_usage(DATA_ROOT)
                info["disk_free_gb"] = round(u.free / 1e9, 1)
                info["disk_used_pct"] = round(100 * u.used / u.total, 1)
            except Exception:
                pass
            self._json(info)

        elif path == "/allan":
            # Allan variance multi-tau — utilise data_buffer (jusqu'a 10 min)
            buf_snap = list(data_buffer)
            if len(buf_snap) < 64:
                self._json({"ok": False,
                            "err": f"buffer trop court ({len(buf_snap)} pts, attendre >= 64)"})
            else:
                import math as _math
                samples = [p["force_g"] for p in buf_snap]
                N = len(samples)
                fs = FORCE_SAMPLE_HZ
                # Taus logarithmiquement espaces sur 40 points de 1 a N//4
                max_tau = max(2, N // 4)
                seen = set()
                tau_list = []
                for i in range(40):
                    tm = max(1, int(round(10 ** (i / 39.0 * _math.log10(max_tau)))))
                    if tm not in seen:
                        seen.add(tm); tau_list.append(tm)
                result = []
                for tau_m in tau_list:
                    M = N // tau_m   # nb de clusters non-chevauchants
                    if M < 3:
                        continue
                    # Moyenne de chaque cluster de tau_m echantillons
                    clusters = [
                        sum(samples[k * tau_m:(k + 1) * tau_m]) / tau_m
                        for k in range(M)
                    ]
                    # AVAR = 1/(2(M-1)) * sum((X_{k+1} - X_k)^2)
                    avar = sum((clusters[k + 1] - clusters[k]) ** 2
                               for k in range(M - 1)) / (2 * (M - 1))
                    result.append({
                        "tau_s":       round(tau_m / fs, 4),
                        "allan_dev_g": round(avar ** 0.5, 7),
                        "n_clusters":  M,
                    })
                self._json({"ok": True, "fs_hz": fs, "n_samples": N,
                            "duration_s": round(N / fs, 1), "data": result})

        elif path == "/recordings":
            # Liste les fichiers MP4 enregistres avec leur CSV associe
            files = []
            try:
                for f in sorted(os.listdir(DATA_DIR_VIDEO), reverse=True):
                    if not f.endswith(".mp4"):
                        continue
                    vp = os.path.join(DATA_DIR_VIDEO, f)
                    cp = os.path.join(DATA_DIR_CSV, f.replace(".mp4", ".csv"))
                    files.append({
                        "name": f,
                        "video": vp,
                        "csv": cp if os.path.exists(cp) else None,
                        "size_mb": round(os.path.getsize(vp) / 1e6, 2),
                        "mtime": time.strftime("%Y-%m-%d %H:%M:%S",
                                               time.localtime(os.path.getmtime(vp))),
                    })
            except Exception:
                pass
            self._json({"files": files[:20]})

        elif path == "/spectrogram":
            cols, fmax = spectro.matrix()
            self._json({"cols": cols, "fmax": fmax, "n_cols": spectro.n_cols, "n_bins": spectro.n_bins})

        elif path == "/measure":
            # Mode "mesure certifiee" : moyenne sur N secondes + CI + Reynolds + St + CL
            try:
                dur = float(urlparse(self.path).query.split("dur=")[-1].split("&")[0]) if "dur=" in self.path else 10.0
            except Exception:
                dur = 10.0
            n_samples = max(20, int(dur * FORCE_SAMPLE_HZ))
            # Echantillonnage : on attend que steady puis on collecte
            t_end = time.time() + dur
            collected_F, collected_V = [], []
            while time.time() < t_end:
                with state_lock:
                    collected_F.append(state["force_N"])
                    collected_V.append(state["airspeed_kalman_ms"] or state["airspeed_avg_ms"])
                time.sleep(1.0 / FORCE_SAMPLE_HZ)
            f_mean, f_lo, f_hi = bootstrap_mean_ci(collected_F, n_boot=200)
            v_mean = sum(collected_V) / len(collected_V) if collected_V else 0.0
            with state_lock:
                c = state["chord_m"]; sp = state["span_m"]; rho = state["rho"]
                aoa = state["aoa_deg"]; rpm = state["fan_rpm"]
                fft = state["peak_freq_hz"]
            A = c * sp
            Re = reynolds(v_mean, c, rho, AIR_VISCOSITY)
            St = strouhal(fft, c, v_mean)
            CL = aero_coeff(f_mean, v_mean, A, rho)
            CL_lo = aero_coeff(f_lo, v_mean, A, rho)
            CL_hi = aero_coeff(f_hi, v_mean, A, rho)
            self._json({
                "duration_s": dur,
                "n_samples": len(collected_F),
                "F_mean_N": round(f_mean, 5),
                "F_ci95": [round(f_lo, 5), round(f_hi, 5)],
                "V_mean_ms": round(v_mean, 3),
                "rpm": rpm,
                "aoa_deg": aoa,
                "Re": round(Re, 0),
                "St": round(St, 4),
                "CL_or_CD": round(CL, 4),
                "CL_or_CD_ci95": [round(CL_lo, 4), round(CL_hi, 4)],
                "FFT_peak_Hz": fft,
                "chord_m": c, "span_m": sp, "rho": rho,
            })

        elif path == "/wake_survey":
            # Estime F_D depuis le champ de vitesse dans la ROI.
            # Necessite un mode flow actif et une ROI definie.
            try:
                with state_lock:
                    v_inf = state["airspeed_kalman_ms"] or state["airspeed_ms"]
                    rho = state["rho"]; span = state["span_m"]; pxm = state["px_per_m"]
                # Recupere flow du dernier frame via flow_proc (proxy : on n'a pas le flow stocke,
                # donc on utilise mag mean comme estimateur approximatif u(y))
                # Pour une vraie wake survey, il faudra stocker flow dans flow_proc.
                # Ici on retourne un placeholder + indication.
                self._json({
                    "ok": False,
                    "msg": "Wake survey necessite mode flow + ROI horizontale derriere le profil. Implementation simplifiee : utiliser mode arrows/heatmap, exporter video et integrer en post-traitement.",
                    "v_inf": v_inf, "rho": rho, "span": span,
                })
            except Exception as e:
                self._json({"ok": False, "err": str(e)})

        elif path == "/experiment/status":
            self._json({
                "running": experiment["running"],
                "progress": experiment["progress"],
                "msg": experiment["msg"],
                "results": experiment["results"],
            })

        elif path == "/export/pdf":
            # Genere un rapport PDF multi-pages : cover + time series + polar + snapshot
            try:
                import matplotlib
                matplotlib.use("Agg")
                import matplotlib.pyplot as plt
                from matplotlib.backends.backend_pdf import PdfPages
                import numpy as np
                pdf_buf = io.BytesIO()
                with state_lock:
                    s = dict(state)
                samples = list(data_buffer)
                with PdfPages(pdf_buf) as pdf:
                    # ---- Page 1 : Cover + parametres ----
                    fig, ax = plt.subplots(figsize=(8.27, 11.69))
                    ax.axis("off")
                    ax.text(0.5, 0.93, "RAPPORT DE MESURE",
                            ha="center", size=22, weight="bold", color="#1a4a99")
                    ax.text(0.5, 0.89, "Mini soufflerie ENSAM",
                            ha="center", size=13, color="#444")
                    ax.text(0.5, 0.86, time.strftime("%Y-%m-%d %H:%M:%S"),
                            ha="center", size=10, color="#888")
                    A_cm2 = s["chord_m"] * s["span_m"] * 1e4
                    cond = (
                        "\n[Profil]\n"
                        f"  Corde         L = {s['chord_m']*1000:6.1f} mm\n"
                        f"  Envergure     b = {s['span_m']*1000:6.1f} mm\n"
                        f"  AoA           a = {s['aoa_deg']:6.1f} deg\n"
                        f"  Surface       A = {A_cm2:6.2f} cm^2\n"
                        "\n[Air]\n"
                        f"  Densite       p = {s['rho']:.4f} kg/m^3\n"
                        "\n[Calibration]\n"
                        f"  HX711 offset    = {s['offset']}\n"
                        f"  HX711 scale     = {s['scale']:.3f} ADU/g\n"
                        f"  px/m            = {s['px_per_m']:.1f}\n"
                        f"  fan kv          = {s['fan_kv']:.6f} m/s/RPM\n"
                        "\n[Mesure courante]\n"
                        f"  Duty            = {s['fan_duty']:.0f} %\n"
                        f"  RPM             = {s['fan_rpm']}\n"
                        f"  V_flow          = {s['airspeed_ms']:.3f} m/s\n"
                        f"  V_fan           = {s['airspeed_fan_ms']:.3f} m/s\n"
                        f"  V_kalman        = {s['airspeed_kalman_ms']:.3f} +- {s['airspeed_kalman_sigma']:.3f} m/s\n"
                        f"  Force inst.     = {s['force_N']:.5f} N ({s['force_g']:.3f} g)\n"
                        f"  Force mu        = {s['force_mean']:.5f} N\n"
                        f"  Force sigma     = {s['force_std']:.5f} N\n"
                        f"  Force CI95      = [{s['force_ci_low']:.5f}, {s['force_ci_high']:.5f}] N\n"
                        f"  Reynolds        = {int(s['reynolds'])}\n"
                        f"  Strouhal        = {s['strouhal']:.4f}\n"
                        f"  C = F/qA        = {s['CL']:.4f}\n"
                        f"  Pic FFT         = {s['peak_freq_hz']:.2f} Hz\n"
                        f"  Regime steady   = {s['steady']}\n"
                        "\n[Donnees]\n"
                        f"  Echantillons    = {len(samples)}\n"
                        f"  Duree           = {(samples[-1]['t']-samples[0]['t']) if len(samples)>1 else 0:.1f} s"
                    )
                    ax.text(0.08, 0.80, cond, family="monospace", size=9.5, va="top")
                    pdf.savefig(fig, bbox_inches="tight")
                    plt.close(fig)

                    # ---- Page 2 : Time series ----
                    if len(samples) > 5:
                        fig, axes = plt.subplots(3, 1, figsize=(8.27, 9), sharex=True)
                        t = np.array([r["t"] for r in samples])
                        F_N = np.array([r["force_N"] for r in samples])
                        V = np.array([r["airspeed_ms"] for r in samples])
                        duty = np.array([r["duty"] for r in samples])
                        rpm = np.array([r["rpm"] for r in samples])
                        axes[0].plot(t, F_N, color="#1a4a99", linewidth=0.7)
                        axes[0].fill_between(t, F_N, alpha=0.2, color="#1a4a99")
                        axes[0].set_ylabel("Force [N]")
                        axes[0].set_title("Evolution temporelle")
                        axes[0].grid(alpha=0.3)
                        axes[1].plot(t, V, color="#2a8b2a", linewidth=0.7)
                        axes[1].set_ylabel("V_flow [m/s]")
                        axes[1].grid(alpha=0.3)
                        ax2 = axes[2].twinx()
                        axes[2].plot(t, duty, color="#cc6600", linewidth=0.8, label="duty %")
                        ax2.plot(t, rpm, color="#aa2222", linewidth=0.6, alpha=0.6, label="RPM")
                        axes[2].set_xlabel("temps [s]")
                        axes[2].set_ylabel("duty %", color="#cc6600")
                        ax2.set_ylabel("RPM", color="#aa2222")
                        axes[2].grid(alpha=0.3)
                        pdf.savefig(fig, bbox_inches="tight")
                        plt.close(fig)

                    # ---- Page 3 : Polaire F vs V^2 ----
                    if len(samples) > 5:
                        fig, ax = plt.subplots(figsize=(8.27, 6))
                        V_arr = np.array([r["airspeed_ms"] for r in samples])
                        F_arr = np.array([r["force_N"] for r in samples])
                        mask = V_arr > 0.2
                        if mask.any():
                            ax.scatter(V_arr[mask]**2, F_arr[mask], s=8,
                                       alpha=0.4, color="#cc6600", label="live")
                        if experiment["results"]:
                            V_sw = np.array([r["V_ms"] for r in experiment["results"]])
                            F_sw = np.array([r["F_N"] for r in experiment["results"]])
                            mask_sw = V_sw > 0.2
                            if mask_sw.any():
                                ax.plot(V_sw[mask_sw]**2, F_sw[mask_sw], "o-",
                                        color="#1a4a99", label="sweep auto", linewidth=2, markersize=7)
                                if mask_sw.sum() > 2:
                                    coef = np.polyfit(V_sw[mask_sw]**2, F_sw[mask_sw], 1)
                                    xs = np.linspace(0, (V_sw[mask_sw]**2).max(), 100)
                                    ax.plot(xs, coef[0]*xs + coef[1], "--", color="#2a8b2a",
                                            label=f"fit: F = {coef[0]:.4f} V^2 + {coef[1]:.4f}")
                                    Cd_est = 2*coef[0] / (s["rho"] * s["chord_m"]*s["span_m"])
                                    ax.text(0.04, 0.92, f"Cd estime = {Cd_est:.3f}",
                                            transform=ax.transAxes, fontsize=13,
                                            color="#2a8b2a", weight="bold")
                        ax.set_xlabel("V^2 [m^2/s^2]")
                        ax.set_ylabel("Force [N]")
                        ax.set_title("Loi quadratique F = 1/2 rho S Cd V^2")
                        ax.grid(alpha=0.3)
                        ax.legend()
                        pdf.savefig(fig, bbox_inches="tight")
                        plt.close(fig)

                    # ---- Page 4 : Snapshot ----
                    with frame_lock:
                        jpg = bytes(latest_jpeg)
                    if jpg:
                        arr = np.frombuffer(jpg, dtype=np.uint8)
                        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                        if img is not None:
                            img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                            fig, ax = plt.subplots(figsize=(8.27, 6))
                            ax.imshow(img_rgb)
                            ax.axis("off")
                            ax.set_title(f"Snapshot camera - mode: {s['cam_mode']}")
                            pdf.savefig(fig, bbox_inches="tight")
                            plt.close(fig)

                body = pdf_buf.getvalue()
                fname = time.strftime("rapport_soufflerie_%Y%m%d_%H%M%S.pdf")
                self.send_response(200)
                self.send_header("Content-Type", "application/pdf")
                self.send_header("Content-Disposition", f'attachment; filename="{fname}"')
                self.send_header("Content-Length", len(body))
                self.end_headers()
                self.wfile.write(body)
            except ImportError:
                self._json({"ok": False, "err": "matplotlib non installe : pip install matplotlib"})
            except Exception as e:
                self._json({"ok": False, "err": str(e)})

        elif path == "/experiment/export":
            # Export CSV des resultats de l'experience
            buf = io.StringIO()
            w = csv.writer(buf)
            w.writerow(["duty_pct", "rpm", "V_ms", "F_N", "F_ci_low", "F_ci_high", "steady"])
            for r in experiment["results"]:
                w.writerow([r["duty"], r["rpm"], r["V_ms"], r["F_N"], r["F_lo"], r["F_hi"], r["steady"]])
            body = buf.getvalue().encode("utf-8")
            fname = time.strftime("sweep_%Y%m%d_%H%M%S.csv")
            self.send_response(200)
            self.send_header("Content-Type", "text/csv")
            self.send_header("Content-Disposition", f'attachment; filename="{fname}"')
            self.send_header("Content-Length", len(body))
            self.end_headers()
            self.wfile.write(body)

        else:
            self.send_response(404); self.end_headers()

    def do_POST(self):
        if not request_allowed(self.headers.get("Host"), self.headers.get("Origin"),
                               self.headers.get("Sec-Fetch-Site"), HTTP_BIND_HOST):
            self.send_error(403); return
        u = urlparse(self.path)
        path, qs = u.path, parse_qs(u.query)

        def q(k, d=None): return qs.get(k, [d])[0] if k in qs else d

        if path == "/tare":
            samples = [raw_mean(max_age=1.0) for _ in range(10)
                       if time.sleep(0.08) is None]
            vals = [v for v in samples if v is not None]
            new_off = int(sum(vals) / len(vals)) if vals else state["offset"]
            force_avg.clear()
            force_buf.clear()
            with state_lock:
                state["offset"] = new_off
            _save_calib()
            self._json({"offset": new_off})

        elif path == "/calibrate":
            m = float(q("mass_g", "100"))
            samples = [raw_mean(max_age=1.0) for _ in range(10)
                       if time.sleep(0.08) is None]
            vals = [v for v in samples if v is not None]
            rm = sum(vals) / len(vals) if vals else 0
            with state_lock:
                off = state["offset"]
                scale = (rm - off) / m if m != 0 else 1.0
                if abs(scale) < 1e-9:
                    scale = 1.0
                state["scale"] = scale
            _save_calib()
            self._json({"scale": scale, "raw_mean": rm})

        elif path == "/led/set":
            kw = {}
            if "anim"   in qs: kw["animation"]  = q("anim")
            if "r"      in qs: kw["color"]       = (int(q("r")), int(q("g","0")), int(q("b","0")))
            if "bri"    in qs: kw["brightness"]  = int(q("bri"))
            if "speed"  in qs: kw["speed"]       = int(q("speed"))
            if "n"      in qs: kw["n_leds"]      = int(q("n"))
            led.set(**kw)
            _save_calib()
            self._json(led.status())

        elif path == "/cam/dist":
            v = max(0.05, min(20.0, float(q("v", "0.5"))))
            with state_lock:
                state["cam_dist_m"] = v
            _save_calib()
            self._json({"cam_dist_m": v})

        elif path == "/fan/set":
            duty = max(0.0, min(100.0, float(q("duty", "0"))))
            fan.set_speed(duty)
            with state_lock:
                state["fan_duty"] = duty
            led.set(animation="solid", color=(0, 200, 0) if duty > 0 else (200, 0, 0))
            self._json({"duty": duty})

        elif path == "/fan/set_v":
            v_ms = max(0.0, min(30.0, float(q("v", "0"))))
            if v_ms == 0.0:
                duty = 0.0
            else:
                with state_lock:
                    kv  = state.get("fan_kv", FAN_KV)
                    k0  = state.get("fan_k0", FAN_K0)
                    tbl = list(state.get("fan_drag_table", []))
                target_rpm = (v_ms - k0) / kv if kv > 0 else 0.0
                if tbl and len(tbl) >= 2:
                    tbl.sort(key=lambda x: x["rpm"])
                    if target_rpm <= tbl[0]["rpm"]:
                        duty = float(tbl[0]["duty"])
                    elif target_rpm >= tbl[-1]["rpm"]:
                        duty = float(tbl[-1]["duty"])
                    else:
                        duty = float(tbl[-1]["duty"])
                        for i in range(len(tbl) - 1):
                            r1, r2 = tbl[i]["rpm"], tbl[i+1]["rpm"]
                            if r1 <= target_rpm <= r2:
                                t = (target_rpm - r1) / max(r2 - r1, 1)
                                duty = tbl[i]["duty"] + t * (tbl[i+1]["duty"] - tbl[i]["duty"])
                                break
                else:
                    # estimation lineaire : RPM ≈ 1375 + 12.5 * duty (calibration OD1238)
                    duty = (target_rpm - 1375.0) / 12.5
                duty = max(0.0, min(100.0, duty))
            fan.set_speed(duty)
            with state_lock:
                state["fan_duty"] = duty
            led.set(animation="solid", color=(0, 200, 0) if duty > 0 else (200, 0, 0))
            self._json({"duty": round(duty, 1), "v_target": v_ms})

        elif path == "/settings/set":
            step = max(0.1, min(20.0, float(q("step", "1"))))
            with state_lock:
                state["encoder_step"] = step
            self._json({"encoder_step": step})

        elif path == "/auto_tare":
            if "enable" in qs:
                auto_tare["enabled"] = bool(int(q("enable")))
            if "idle_s" in qs:
                auto_tare["idle_s"] = max(2.0, min(60.0, float(q("idle_s"))))
            if "threshold_g" in qs:
                auto_tare["std_threshold_g"] = max(0.05, min(10.0, float(q("threshold_g"))))
            self._json({
                "enabled": auto_tare["enabled"],
                "idle_s": auto_tare["idle_s"],
                "threshold_g": auto_tare["std_threshold_g"],
                "count": auto_tare["count"],
            })

        elif path == "/force/filter":
            n = max(1, min(500, int(float(q("n", "10")))))
            with state_lock:
                state["filter_window"] = n
            _save_calib()
            self._json({"filter_window": n})

        elif path == "/cam/reset":
            with state_lock:
                state["cam_brightness"] = CAM_BRIGHTNESS
                state["cam_contrast"]   = CAM_CONTRAST
                state["cam_saturation"] = CAM_SATURATION
                state["cam_ae"]  = CAM_AE_ENABLE
                state["cam_awb"] = CAM_AWB_ENABLE
                state["px_per_m"] = CAM_PX_PER_M
            flow_proc.set_mode("raw")
            flow_proc.use_mask = False
            flow_proc.clear_roi()
            flow_proc.hsv_low  = __import__("numpy").array([0, 0, 180], dtype=__import__("numpy").uint8)
            flow_proc.hsv_high = __import__("numpy").array([180, 60, 255], dtype=__import__("numpy").uint8)
            self._json({"ok": True, "defaults": {
                "brightness": CAM_BRIGHTNESS, "contrast": CAM_CONTRAST,
                "saturation": CAM_SATURATION, "ae": CAM_AE_ENABLE,
                "awb": CAM_AWB_ENABLE, "px_per_m": CAM_PX_PER_M,
            }})

        elif path == "/cam/mode":
            m = q("mode", "raw")
            flow_proc.set_mode(m)
            with state_lock:
                state["cam_mode"] = flow_proc.mode
            self._json({"mode": flow_proc.mode})

        elif path == "/cam/controls":
            with state_lock:
                if "brightness" in qs: state["cam_brightness"] = float(q("brightness"))
                if "contrast" in qs: state["cam_contrast"] = float(q("contrast"))
                if "saturation" in qs: state["cam_saturation"] = float(q("saturation"))
                if "ae" in qs: state["cam_ae"] = bool(int(q("ae")))
                if "awb" in qs: state["cam_awb"] = bool(int(q("awb")))
            apply_cam_controls()
            self._json({"ok": True})

        elif path == "/cam/colour":
            with state_lock:
                if "r" in qs: state["cam_colour_gain_r"] = max(0.1, min(8.0, float(q("r"))))
                if "b" in qs: state["cam_colour_gain_b"] = max(0.1, min(8.0, float(q("b"))))
                rg = state["cam_colour_gain_r"]
                bg = state["cam_colour_gain_b"]
            apply_cam_controls()
            self._json({"r": rg, "b": bg})

        elif path == "/cam/smoke_enhance":
            en = bool(int(q("enable", "1")))
            flow_proc.smoke_enhance = en
            self._json({"smoke_enhance": en})

        elif path == "/cam/mask":
            en = bool(int(q("enable", "0")))
            flow_proc.use_mask = en
            with state_lock:
                state["cam_mask"] = en
            self._json({"mask": en})

        elif path == "/cam/hsv":
            args = {}
            for k in ("h_lo", "s_lo", "v_lo", "h_hi", "s_hi", "v_hi"):
                if k in qs: args[k] = int(q(k))
            flow_proc.set_hsv(**args)
            with state_lock:
                state["hsv_low"] = flow_proc.hsv_low.tolist()
                state["hsv_high"] = flow_proc.hsv_high.tolist()
            self._json({"ok": True})

        elif path == "/cam/pxm":
            v = max(1.0, float(q("v", "1000")))
            with state_lock:
                state["px_per_m"] = v
            _save_calib()
            self._json({"px_per_m": v})

        elif path == "/fan/calib":
            # 3 modes: from_flow=1 -> calibre depuis V_flow actuel + RPM actuel
            #          v=X -> V cible manuelle au RPM actuel
            #          kv=X & k0=X -> entrer K directement
            with state_lock:
                rpm = state["fan_rpm"]
                k0 = state["fan_k0"]
            if "from_flow" in qs:
                with state_lock:
                    v_target = state["airspeed_ms"]
                if rpm > 0 and v_target > 0.1:
                    kv = fan_calib_kv(rpm, v_target, k0)
                    with state_lock:
                        state["fan_kv"] = kv
                    _save_calib()
                    self._json({"ok": True, "kv": kv, "rpm": rpm, "v": v_target})
                else:
                    self._json({"ok": False, "err": "rpm ou flux trop faible"})
            elif "v" in qs:
                v_target = float(q("v"))
                if rpm > 0:
                    kv = fan_calib_kv(rpm, v_target, k0)
                    with state_lock:
                        state["fan_kv"] = kv
                    _save_calib()
                    self._json({"ok": True, "kv": kv, "rpm": rpm, "v": v_target})
                else:
                    self._json({"ok": False, "err": "ventilo a l arret"})
            elif "kv" in qs:
                kv = float(q("kv", "0.003"))
                k0v = float(q("k0", "0"))
                with state_lock:
                    state["fan_kv"] = kv
                    state["fan_k0"] = k0v
                _save_calib()
                self._json({"ok": True, "kv": kv, "k0": k0v})
            else:
                self._json({"ok": False, "err": "missing param"})

        elif path == "/cam/calib_chessboard":
            # Detecte un damier (cols x rows = nombre de coins intérieurs) dans la frame
            # courante et calcule px/m a partir de la taille de carre fournie [mm].
            cols = int(float(q("cols", "9")))
            rows = int(float(q("rows", "6")))
            sq_mm = float(q("square_mm", "20"))
            try:
                import numpy as np
                with frame_lock:
                    jpg = bytes(latest_jpeg)
                if not jpg:
                    self._json({"ok": False, "err": "pas de frame"}); return
                arr = np.frombuffer(jpg, dtype=np.uint8)
                img = cv2.imdecode(arr, cv2.IMREAD_GRAYSCALE)
                if img is None:
                    self._json({"ok": False, "err": "decode echec"}); return
                found, corners = cv2.findChessboardCorners(
                    img, (cols, rows),
                    flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE,
                )
                if not found:
                    self._json({"ok": False, "err": f"damier {cols}x{rows} non detecte"}); return
                # affinage subpixel
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners = cv2.cornerSubPix(img, corners, (11, 11), (-1, -1), criteria)
                # Distance moyenne entre voisins horizontaux (en pixels)
                pts = corners.reshape(rows, cols, 2)
                dx = []
                for r in range(rows):
                    for c in range(cols - 1):
                        d = pts[r, c + 1] - pts[r, c]
                        dx.append(float((d[0] ** 2 + d[1] ** 2) ** 0.5))
                dy = []
                for r in range(rows - 1):
                    for c in range(cols):
                        d = pts[r + 1, c] - pts[r, c]
                        dy.append(float((d[0] ** 2 + d[1] ** 2) ** 0.5))
                px_per_mm = (sum(dx) + sum(dy)) / (len(dx) + len(dy))
                px_per_m = px_per_mm / (sq_mm * 1e-3) / 1000.0   # = px_per_mm * 1000 / sq_mm
                px_per_m = px_per_mm * 1000.0 / sq_mm
                with state_lock:
                    state["px_per_m"] = px_per_m
                _save_calib()
                self._json({
                    "ok": True,
                    "px_per_m": round(px_per_m, 2),
                    "px_per_mm": round(px_per_mm, 3),
                    "square_mm": sq_mm,
                    "corners_found": cols * rows,
                })
            except Exception as e:
                self._json({"ok": False, "err": str(e)})

        elif path == "/noise_test":
            # Caracterisation bruit HX711 : moyenne, ecart-type, FFT pendant N secondes.
            # Le ventilateur doit etre arrete et la cellule au repos.
            try:
                dur = float(q("dur", "10"))
            except Exception:
                dur = 10.0
            samples = []
            t_end = time.time() + dur
            while time.time() < t_end:
                with state_lock:
                    samples.append(state["raw"])
                time.sleep(1.0 / FORCE_SAMPLE_HZ)
            if len(samples) < 10:
                self._json({"ok": False, "err": "pas assez de donnees"}); return
            # stats
            n = len(samples)
            mean = sum(samples) / n
            var = sum((x - mean) ** 2 for x in samples) / (n - 1)
            std = var ** 0.5
            # Allan variance (1-tau) : variance des differences successives /2
            diffs = [samples[i + 1] - samples[i] for i in range(n - 1)]
            allan = sum(d * d for d in diffs) / (2 * len(diffs))
            allan_std = allan ** 0.5
            # NSR effectif en grammes (selon scale)
            with state_lock:
                scale = state["scale"]
            noise_g = (std / scale) if scale > 1e-9 else 0.0
            allan_g = (allan_std / scale) if scale > 1e-9 else 0.0
            self._json({
                "ok": True, "n_samples": n, "duration_s": dur,
                "mean_adu": round(mean, 1),
                "std_adu": round(std, 2),
                "allan_std_adu": round(allan_std, 2),
                "noise_rms_g": round(noise_g, 4),
                "allan_g": round(allan_g, 4),
                "snr_db_if_force_100g": round(20 * math.log10(100.0 / noise_g) if noise_g > 0 else 0, 1),
            })

        elif path == "/cam/roi":
            # x, y, w, h normalises [0..1]. clear=1 pour annuler.
            if q("clear", "0") == "1":
                flow_proc.clear_roi()
                with state_lock:
                    state["roi"] = None
                self._json({"roi": None})
            else:
                x = float(q("x", "0")); y = float(q("y", "0"))
                w = float(q("w", "1")); h = float(q("h", "1"))
                flow_proc.set_roi(x, y, w, h)
                with state_lock:
                    state["roi"] = list(flow_proc.roi) if flow_proc.roi else None
                self._json({"roi": state["roi"]})

        elif path == "/aero/set":
            with state_lock:
                if "type" in qs: state["object_type"] = q("type")
                if "chord" in qs: state["chord_m"] = max(0.001, float(q("chord")))
                if "span" in qs: state["span_m"] = max(0.001, float(q("span")))
                if "aoa" in qs: state["aoa_deg"] = float(q("aoa"))
                if "rho" in qs: state["rho"] = max(0.1, float(q("rho")))
            _save_calib()
            self._json({"ok": True})

        elif path == "/experiment/start":
            if experiment["running"]:
                self._json({"ok": False, "err": "deja en cours"})
            else:
                d_from = int(float(q("from", "20")))
                d_to = int(float(q("to", "80")))
                d_step = int(float(q("step", "10")))
                dwell = float(q("dwell", "5"))
                threading.Thread(target=run_duty_sweep,
                                 args=(d_from, d_to, d_step, dwell),
                                 daemon=True).start()
                self._json({"ok": True})

        elif path == "/experiment/stop":
            experiment["running"] = False
            self._json({"ok": True})

        elif path == "/record/toggle":
            if not recording["active"]:
                # DEMARRAGE — FileOutput (H264 brut, toujours fiable)
                try:
                    stamp = time.strftime("%Y%m%d_%H%M%S")
                    h264_file = os.path.join(DATA_DIR_VIDEO, f"rec_{stamp}.h264")
                    mp4_file  = os.path.join(DATA_DIR_VIDEO, f"rec_{stamp}.mp4")
                    cfile     = os.path.join(DATA_DIR_CSV,   f"rec_{stamp}.csv")
                    encoder = H264Encoder(bitrate=6_000_000)
                    output  = FileOutput(h264_file)
                    picam.start_recording(encoder, output, name="main")
                    recording["h264"]       = encoder
                    recording["ffmpeg"]     = output
                    recording["h264_file"]  = h264_file
                    recording["video_file"] = mp4_file
                    recording["csv_file"]   = cfile
                    recording["start_time"] = time.time()
                    recording["start_t_rel"] = time.time() - session_start
                    recording["active"] = True
                    with state_lock:
                        state["recording"]    = True
                        state["record_file"]  = os.path.basename(mp4_file)
                    self._json({"recording": True, "video": mp4_file, "csv": cfile})
                except Exception as e:
                    self._json({"recording": False, "err": str(e)})
            else:
                # ARRET — stoppe seulement l'encodeur, puis conversion H264→MP4 en fond
                h264_f = recording.get("h264_file", "")
                mp4_f  = recording["video_file"]
                mp4_bn = os.path.basename(mp4_f)
                try:
                    recording["h264"].stop()
                except Exception as e:
                    with state_lock:
                        state["status"] = f"rec stop err: {str(e)[:30]}"

                def _convert_h264(src, dst, key):
                    try:
                        r = subprocess.run(
                            ["ffmpeg", "-y", "-framerate", str(CAM_FPS),
                             "-i", src, "-c:v", "copy", dst],
                            capture_output=True, timeout=300)
                        err = "" if r.returncode == 0 else r.stderr.decode()[-200:]
                        if r.returncode == 0:
                            try: os.remove(src)
                            except Exception: pass
                    except Exception as ex:
                        err = str(ex)
                    with _conv_lock:
                        _conversions[key] = {"done": True, "err": err}

                with _conv_lock:
                    _conversions[mp4_bn] = {"done": False, "err": ""}
                threading.Thread(target=_convert_h264,
                                 args=(h264_f, mp4_f, mp4_bn),
                                 daemon=True).start()
                # Sauvegarde CSV : echantillons depuis start_t_rel
                t_start = recording["start_t_rel"]
                samples = [s for s in list(data_buffer) if s["t"] >= t_start]
                try:
                    with open(recording["csv_file"], "w", newline="") as f:
                        w = csv.writer(f)
                        with state_lock:
                            meta = {
                                "video_file": recording["video_file"],
                                "started_at": time.strftime("%Y-%m-%d %H:%M:%S",
                                                            time.localtime(recording["start_time"])),
                                "duration_s": round(time.time() - recording["start_time"], 2),
                                "samples": len(samples),
                                "sample_rate_hz": FORCE_SAMPLE_HZ,
                                "hx711_offset": state["offset"],
                                "hx711_scale_adu_per_g": state["scale"],
                                "chord_m": state["chord_m"], "span_m": state["span_m"],
                                "aoa_deg": state["aoa_deg"], "rho_kg_m3": state["rho"],
                                "px_per_m": state["px_per_m"],
                                "fan_kv_ms_per_rpm": state["fan_kv"],
                                "cam_mode": state["cam_mode"],
                            }
                        for k, v in meta.items():
                            f.write(f"# {k}: {v}\n")
                        f.write("#\n")
                        w.writerow(["t_s","duty_pct","fan_rpm","delta_adu","force_g","force_N",
                                    "flow_mag_px","airspeed_flow_ms","airspeed_fan_ms"])
                        for r in samples:
                            t_rel = round(r["t"] - t_start, 3)
                            w.writerow([t_rel, r["duty"], r["rpm"], r["delta_adu"],
                                        r["force_g"], r["force_N"], r["flow_mag"],
                                        r["airspeed_ms"], r.get("airspeed_fan_ms", 0)])
                except Exception as e:
                    with state_lock:
                        state["status"] = f"csv save err: {str(e)[:30]}"
                vfile = recording["video_file"]
                cfile = recording["csv_file"]
                duration = time.time() - recording["start_time"]
                recording["active"] = False
                recording["h264"] = None
                recording["ffmpeg"] = None
                with state_lock:
                    state["recording"] = False
                    state["record_file"] = ""
                self._json({
                    "recording": False, "video": vfile, "csv": cfile,
                    "duration_s": round(duration, 2), "samples": len(samples),
                })

        elif path == "/calib/tare":
            samples = [raw_mean(max_age=1.0) for _ in range(10)
                       if time.sleep(0.08) is None]
            vals = [v for v in samples if v is not None]
            new_off = int(sum(vals) / len(vals)) if vals else state["offset"]
            force_avg.clear(); force_buf.clear()
            with state_lock:
                state["offset"] = new_off
            _save_calib()
            self._json({"ok": True, "offset": new_off})

        elif path == "/calib/scale":
            m = float(q("mass_g", "100"))
            samples = [raw_mean(max_age=1.0) for _ in range(10)
                       if time.sleep(0.08) is None]
            vals = [v for v in samples if v is not None]
            rm = sum(vals) / len(vals) if vals else 0
            with state_lock:
                off = state["offset"]
                scale = (rm - off) / m if m != 0 else 1.0
                if abs(scale) < 1e-9: scale = 1.0
                state["scale"] = scale
            _save_calib()
            self._json({"ok": True, "scale": scale, "raw_mean": rm})

        elif path == "/calib/fan_sweep":
            dmin  = max(10.0, min(80.0,  float(q("dmin",  "20"))))
            dmax  = max(30.0, min(100.0, float(q("dmax",  "80"))))
            pts   = max(3,    min(15,    int(q("pts",    "7"))))
            stab  = max(2.0,  min(15.0,  float(q("stab",  "5"))))

            def _run_sweep():
                import numpy as np_
                duties = [dmin + (dmax - dmin) * i / (pts - 1) for i in range(pts)]
                with fan_calib_lock:
                    fan_calib_state.update({"running": True, "phase": "waiting",
                                            "progress": 0.0, "table": [], "total_points": pts,
                                            "msg": "Demarrage sweep...", "current_duty": 0, "current_F": 0.0})
                table = []
                try:
                    for idx, d in enumerate(duties):
                        with fan_calib_lock:
                            if not fan_calib_state["running"]:
                                break
                            fan_calib_state["current_duty"] = int(d)
                            fan_calib_state["phase"] = "stabilizing"
                            fan_calib_state["msg"] = f"Point {idx+1}/{pts} — {int(d)}% — stabilisation {stab}s"
                        fan.set_speed(d)
                        with state_lock: state["fan_duty"] = d
                        time.sleep(stab)
                        with fan_calib_lock:
                            fan_calib_state["phase"] = "measuring"
                            fan_calib_state["msg"] = f"Point {idx+1}/{pts} — {int(d)}% — mesure..."
                        samples = []
                        for _ in range(20):
                            v = raw_mean(max_age=1.0)
                            if v is not None: samples.append(v)
                            time.sleep(0.1)
                        fan.set_speed(0)
                        with state_lock:
                            state["fan_duty"] = 0.0
                            off = state["offset"]
                            scale = state["scale"]
                            rpm = state["fan_rpm"]
                        if samples:
                            raw_avg = sum(samples) / len(samples)
                            delta = raw_avg - off
                            g = adu_to_grams(delta, scale)
                            F = round(grams_to_newtons(g), 6)
                        else:
                            F = 0.0
                        pt = {"duty": int(d), "F_N": F, "rpm": rpm}
                        table.append(pt)
                        with fan_calib_lock:
                            fan_calib_state["table"] = list(table)
                            fan_calib_state["current_F"] = F
                            fan_calib_state["progress"] = (idx + 1) / pts
                        time.sleep(1.5)  # pause entre points

                    fan.set_speed(0)
                    with state_lock:
                        state["fan_duty"] = 0.0
                        state["fan_drag_table"] = list(table)
                        state["support_drag_enabled"] = True
                    _save_calib()
                    with fan_calib_lock:
                        fan_calib_state["phase"] = "done"
                        fan_calib_state["progress"] = 1.0
                        fan_calib_state["running"] = False
                        fan_calib_state["msg"] = f"Sweep termine — {len(table)} points"
                except Exception as ex:
                    fan.set_speed(0)
                    with state_lock: state["fan_duty"] = 0.0
                    with fan_calib_lock:
                        fan_calib_state["phase"] = "error"
                        fan_calib_state["msg"] = str(ex)
                        fan_calib_state["running"] = False

            if not fan_calib_state["running"]:
                threading.Thread(target=_run_sweep, daemon=True).start()
                self._json({"ok": True, "started": True})
            else:
                self._json({"ok": False, "err": "sweep deja en cours"})

        elif path == "/calib/reset_sweep":
            with state_lock:
                state["fan_drag_table"] = []
                state["support_drag_enabled"] = False
            with fan_calib_lock:
                fan_calib_state["table"] = []
                fan_calib_state["phase"] = "idle"
            _save_calib()
            self._json({"ok": True})

        elif path == "/tare_support":
            # Tare aerodynamique du support (bloquant ~8s)
            duty = max(20.0, min(100.0, float(q("duty", "50"))))
            fan.set_speed(duty)
            with state_lock:
                state["fan_duty"] = duty
            time.sleep(5.0)   # stabilisation flux
            samples = []
            for _ in range(20):
                v = raw_mean(max_age=1.0)
                if v is not None:
                    samples.append(v)
                time.sleep(0.1)
            fan.set_speed(0)
            with state_lock:
                state["fan_duty"] = 0.0
            if samples:
                with state_lock:
                    off = state["offset"]
                    scale = state["scale"]
                raw_avg = sum(samples) / len(samples)
                delta = raw_avg - off
                g = adu_to_grams(delta, scale)
                drag_N = round(grams_to_newtons(g), 6)
                with state_lock:
                    state["support_drag_N"] = drag_N
                    state["support_drag_enabled"] = True
                _save_calib()
                self._json({"ok": True, "support_drag_N": drag_N})
            else:
                self._json({"ok": False, "err": "aucun echantillon HX711"})

        elif path == "/tare_support/enable":
            en = q("v", "true").lower() not in ("false", "0")
            with state_lock:
                state["support_drag_enabled"] = en
            self._json({"ok": True, "enabled": en})

        elif path == "/etude/measure":
            duty = max(20.0, min(100.0, float(q("duty", "50"))))
            duration = max(5.0, min(120.0, float(q("duration", "30"))))

            def _run_measure():
                with etude_lock:
                    etude["running"] = True
                    etude["phase"] = "stabilizing"
                    etude["progress"] = 0.0
                    etude["msg"] = ""
                try:
                    fan.set_speed(duty)
                    with state_lock:
                        state["fan_duty"] = duty
                    time.sleep(4.0)   # stabilisation

                    with etude_lock:
                        etude["phase"] = "acquiring"
                    samples_N = []
                    samples_v = []
                    t0 = time.time()
                    while (time.time() - t0) < duration:
                        with etude_lock:
                            if not etude["running"]:
                                break
                        with state_lock:
                            fn = state["force_N"]
                            asrc = state["airspeed_source"]
                            if asrc == "fan":
                                vm = state["airspeed_fan_ms"]
                            elif asrc == "pressure":
                                vm = state["airspeed_pressure_ms"] or state["airspeed_fan_ms"]
                            else:
                                vm = state["airspeed_kalman_ms"] or state["airspeed_avg_ms"] or state["airspeed_fan_ms"]
                        samples_N.append(fn)
                        samples_v.append(vm)
                        with etude_lock:
                            etude["progress"] = (time.time() - t0) / duration
                        time.sleep(0.1)

                    fan.set_speed(0)
                    with state_lock:
                        state["fan_duty"] = 0.0

                    F_mean = sum(samples_N) / len(samples_N) if samples_N else 0.0
                    V_mean = sum(samples_v) / len(samples_v) if samples_v else 0.0
                    with state_lock:
                        drag = state["support_drag_N"] if state["support_drag_enabled"] else 0.0
                        c = state["chord_m"]
                        sp = state["span_m"]
                        rho = state["rho"]
                        mu = AIR_VISCOSITY
                    F_net = F_mean - drag
                    Re = reynolds(V_mean, c, rho, mu) if V_mean > 0.1 else 0
                    A = c * sp
                    q_dyn = 0.5 * rho * V_mean ** 2
                    CL_CD = F_net / (q_dyn * A) if q_dyn > 0 and A > 0 else 0.0
                    with etude_lock:
                        etude["phase"] = "done"
                        etude["progress"] = 1.0
                        etude["F_mean_N"] = round(F_mean, 5)
                        etude["F_net_N"] = round(F_net, 5)
                        etude["V_ms"] = round(V_mean, 3)
                        etude["Re"] = int(Re)
                        etude["CL"] = round(CL_CD, 4)
                        etude["CD"] = round(CL_CD, 4)
                        etude["n_samples"] = len(samples_N)
                        etude["running"] = False
                except Exception as ex:
                    fan.set_speed(0)
                    with state_lock:
                        state["fan_duty"] = 0.0
                    with etude_lock:
                        etude["phase"] = "error"
                        etude["msg"] = str(ex)
                        etude["running"] = False

            threading.Thread(target=_run_measure, daemon=True).start()
            self._json({"ok": True, "started": True})

        elif path == "/etude/sweep":
            sweep_var = q("var", "duty")          # "duty" | "rpm"
            pts       = max(2,   min(12,   int(q("pts",  "6"))))
            stab_s    = max(2.0, min(15.0, float(q("stab", "5"))))
            dur_s     = max(5.0, min(60.0, float(q("dur",  "10"))))
            double    = q("double", "0") == "1"
            max_duty  = max(50.0, min(100.0, float(q("max_duty", "95"))))

            if sweep_var == "rpm":
                rpmmin = max(300,  min(3000, int(q("rmin", "800"))))
                rpmmax = max(500,  min(3800, int(q("rmax", "3000"))))
                targets_up   = [int(rpmmin + (rpmmax - rpmmin) * i / (pts - 1)) for i in range(pts)]
                targets_down = list(reversed(targets_up[:-1]))
                all_targets  = targets_up + (targets_down if double else [])
                label_range  = f"{rpmmin}–{rpmmax} RPM"
            else:
                dmin = max(20.0, min(80.0,  float(q("dmin", "20"))))
                dmax = max(30.0, min(max_duty, float(q("dmax", "80"))))
                targets_up   = [dmin + (dmax - dmin) * i / (pts - 1) for i in range(pts)]
                targets_down = list(reversed(targets_up[:-1]))
                all_targets  = targets_up + (targets_down if double else [])
                label_range  = f"{dmin:.0f}%–{dmax:.0f}%"

            total_pts = len(all_targets)

            def _run_sweep():
                with etude_lock:
                    etude["running"]       = True
                    etude["phase"]         = "stabilizing"
                    etude["progress"]      = 0.0
                    etude["msg"]           = f"Sweep {pts} pts {label_range} {'↑↓' if double else '↑'}"
                    etude["sweep_partial"] = []
                partial = []

                def _reach_rpm(target, timeout=10.0):
                    """Régulateur P : ajuste duty jusqu'à target RPM ± 60 RPM."""
                    # Estimation initiale (RPM ~linéaire avec duty entre 20% et max)
                    duty_est = max(20.0, min(max_duty, 20.0 + (target / 3800.0) * (max_duty - 20.0)))
                    fan.set_speed(duty_est)
                    with state_lock: state["fan_duty"] = duty_est
                    t0 = time.time()
                    while time.time() - t0 < timeout:
                        with etude_lock:
                            if not etude["running"]: return duty_est
                        with state_lock:
                            current = state["fan_rpm"]
                        error = target - current
                        if abs(error) < 60:
                            break
                        # P gain : 0.007 duty/RPM (conservateur)
                        duty_est = max(20.0, min(max_duty, duty_est + error * 0.007))
                        fan.set_speed(duty_est)
                        with state_lock: state["fan_duty"] = duty_est
                        time.sleep(0.25)
                    return duty_est

                try:
                    for idx, target in enumerate(all_targets):
                        dir_arrow = "↓" if (double and idx >= pts) else "↑"
                        with etude_lock:
                            if not etude["running"]: break
                            etude["phase"] = "stabilizing"
                            lbl = f"RPM {target}" if sweep_var == "rpm" else f"{target:.0f}%"
                            etude["msg"] = f"Pt {idx+1}/{total_pts} {dir_arrow} — {lbl}"

                        if sweep_var == "rpm":
                            duty_actual = _reach_rpm(target, timeout=stab_s + 5.0)
                        else:
                            duty_actual = min(target, max_duty)
                            fan.set_speed(duty_actual)
                            with state_lock: state["fan_duty"] = duty_actual
                            # Attente stabilisation
                            t_stab = time.time()
                            while time.time() - t_stab < stab_s:
                                with etude_lock:
                                    if not etude["running"]: break
                                time.sleep(0.1)

                        # Stabilisation additionnelle en RPM mode
                        if sweep_var == "rpm":
                            time.sleep(max(0.5, stab_s - 5.0))

                        with etude_lock:
                            etude["phase"] = "acquiring"
                        samples_N, samples_v, samples_rpm = [], [], []
                        t0 = time.time()
                        while time.time() - t0 < dur_s:
                            with etude_lock:
                                if not etude["running"]: break
                            with state_lock:
                                fn  = state["force_N"]
                                asrc = state["airspeed_source"]
                                if asrc == "fan":
                                    vm = state["airspeed_fan_ms"]
                                elif asrc == "pressure":
                                    vm = state["airspeed_pressure_ms"] or state["airspeed_fan_ms"]
                                else:
                                    vm = state["airspeed_kalman_ms"] or state["airspeed_avg_ms"] or state["airspeed_fan_ms"]
                                rpm = state["fan_rpm"]
                            samples_N.append(fn); samples_v.append(vm); samples_rpm.append(rpm)
                            time.sleep(0.1)

                        F_mean   = sum(samples_N)   / len(samples_N)   if samples_N   else 0.0
                        V_mean   = sum(samples_v)   / len(samples_v)   if samples_v   else 0.0
                        rpm_mean = sum(samples_rpm) / len(samples_rpm) if samples_rpm else 0.0
                        with state_lock:
                            drag = state["support_drag_N"] if state["support_drag_enabled"] else 0.0
                            c = state["chord_m"]; sp = state["span_m"]; rho = state["rho"]
                        F_net  = F_mean - drag
                        A      = c * sp
                        q_dyn  = 0.5 * rho * V_mean ** 2
                        CL_CD  = F_net / (q_dyn * A) if q_dyn > 0 and A > 0 else 0.0
                        Re     = reynolds(V_mean, c, rho, AIR_VISCOSITY) if V_mean > 0.1 else 0
                        with state_lock:
                            v_pres_snap = state["airspeed_pressure_ms"]
                            v_fan_snap  = state["airspeed_fan_ms"]
                        partial.append({
                            "duty":      round(duty_actual, 1),
                            "rpm":       int(rpm_mean),
                            "V_ms":      round(V_mean,    3),
                            "V_fan_ms":  round(v_fan_snap, 3),
                            "V_pres_ms": round(v_pres_snap, 3),
                            "F_net_N":   round(F_net,      5),
                            "CL":        round(CL_CD,      4),
                            "Re":        int(Re),
                            "dir":       dir_arrow,
                        })
                        with etude_lock:
                            etude["sweep_partial"] = list(partial)
                            etude["progress"]      = (idx + 1) / total_pts

                    fan.set_speed(0)
                    with state_lock: state["fan_duty"] = 0.0
                    with etude_lock:
                        etude["phase"]         = "done"
                        etude["progress"]      = 1.0
                        etude["running"]       = False
                        etude["sweep_partial"] = partial
                except Exception as ex:
                    fan.set_speed(0)
                    with state_lock: state["fan_duty"] = 0.0
                    with etude_lock:
                        etude["phase"] = "error"
                        etude["msg"]   = str(ex)
                        etude["running"] = False

            threading.Thread(target=_run_sweep, daemon=True).start()
            self._json({"ok": True, "started": True})

        elif path == "/etude/save":
            import json as _json
            body_len = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(body_len).decode("utf-8", errors="replace")
            try:
                payload = _json.loads(body)
            except Exception as ex:
                self._json({"ok": False, "err": f"JSON invalide: {ex}"}); return
            ts  = time.strftime("%Y%m%d_%H%M%S")
            eid = f"etude_{ts}"
            payload["id"]       = eid
            payload["saved_at"] = time.strftime("%Y-%m-%dT%H:%M:%S")
            fpath = os.path.join(DATA_DIR_ETUDES, f"{eid}.json")
            with open(fpath, "w", encoding="utf-8") as f:
                _json.dump(payload, f, ensure_ascii=False, indent=2)
            self._json({"ok": True, "id": eid})

        elif path == "/etude/photo":
            # Snapshot live depuis la caméra (pas d'extraction vidéo)
            with frame_lock:
                frame = bytes(latest_jpeg)
            if not frame:
                self._json({"ok": False, "err": "pas de frame camera"}); return
            fname = time.strftime("snap_%Y%m%d_%H%M%S.jpg")
            try:
                fpath = os.path.join(DATA_DIR_PHOTOS, fname)
                with open(fpath, "wb") as fp:
                    fp.write(frame)
                self._json({"ok": True, "photo": fname})
            except Exception as ex:
                self._json({"ok": False, "err": str(ex)})

        elif path == "/etude/snapshot":
            import json as _json
            fname = q("video", "")
            if not fname or "/" in fname or ".." in fname:
                self._json({"ok": False, "err": "nom invalide"}); return
            src = os.path.join(DATA_DIR_VIDEO, fname)
            if not os.path.isfile(src):
                self._json({"ok": False, "err": "video introuvable"}); return
            try:
                probe_r = subprocess.run(
                    ["ffprobe", "-v", "quiet", "-show_entries", "format=duration",
                     "-print_format", "json", src],
                    capture_output=True, text=True, timeout=15)
                info = _json.loads(probe_r.stdout)
                dur = float(info.get("format", {}).get("duration", 0) or 0)
                mid = max(dur / 2, 0.5)
                snap_name = "snap_" + fname.replace(".mp4", ".jpg")
                snap_path = os.path.join(DATA_DIR_PHOTOS, snap_name)
                r = subprocess.run(
                    ["ffmpeg", "-y", "-ss", f"{mid:.2f}", "-i", src,
                     "-frames:v", "1", "-q:v", "2", snap_path],
                    capture_output=True, timeout=30)
                if r.returncode == 0 and os.path.isfile(snap_path):
                    self._json({"ok": True, "photo": snap_name})
                else:
                    self._json({"ok": False, "err": r.stderr.decode()[-200:]})
            except Exception as ex:
                self._json({"ok": False, "err": str(ex)})

        elif path == "/calib/cancel":
            with fan_calib_lock:
                if fan_calib_state["running"]:
                    fan_calib_state["running"] = False
                    fan_calib_state["phase"] = "cancelled"
                    fan_calib_state["msg"] = "Annule"
            fan.set_speed(0)
            with state_lock: state["fan_duty"] = 0.0
            self._json({"ok": True})

        elif path == "/etude/cancel":
            with etude_lock:
                if etude["running"]:
                    etude["running"] = False
                    etude["phase"] = "cancelled"
                    etude["msg"] = "Annule"
            fan.set_speed(0)
            with state_lock: state["fan_duty"] = 0.0
            self._json({"ok": True})

        elif path == "/etude/set_vsrc":
            src = q("src", "kalman")
            if src not in ("kalman", "fan", "pressure", "manual"):
                src = "kalman"
            with state_lock:
                state["airspeed_source"] = src
                if src == "manual" and "v" in qs:
                    state["airspeed_manual_ms"] = max(0.0, float(q("v", "0")))
            _save_calib()
            self._json({"ok": True, "src": src, "v": state.get("airspeed_manual_ms", 0.0)})

        elif path == "/calibrate/pressure_ref":
            if not pressure.ok:
                self._json({"ok": False, "err": pressure.error}); return
            p = pressure.set_reference(n=40)
            if p is None:
                self._json({"ok": False, "err": "lecture echouee"}); return
            with state_lock:
                state["pressure_ref_pa"] = round(p, 2)
            self._json({"ok": True, "p_ref": round(p, 2)})

        elif path == "/calibrate/pressure_sweep":
            # SSE stream — sweep duty dmin→dmax, mesure ΔP à chaque point
            if not pressure.ok:
                self._json({"ok": False, "err": pressure.error}); return
            try:
                dmin  = max(20.0, min(80.0, float(q("dmin", "20"))))
                dmax  = max(30.0, min(95.0, float(q("dmax", "80"))))
                pts   = max(4, min(15, int(q("pts", "8"))))
                stab  = max(3, min(20, float(q("stab", "6"))))
            except Exception:
                self._json({"ok": False, "err": "params invalides"}); return

            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.end_headers()

            def _sse(obj):
                try:
                    self.wfile.write(("data: " + json.dumps(obj) + "\n\n").encode())
                    self.wfile.flush()
                except Exception:
                    pass

            try:
                # Référence ambiante
                p_ref = pressure.set_reference(n=40)
                with state_lock:
                    state["pressure_ref_pa"] = round(p_ref, 2) if p_ref else 0.0

                duties = [dmin + (dmax - dmin) * i / (pts - 1) for i in range(pts)]
                calib_table = []
                with state_lock:
                    kv = state["fan_kv"]
                    rho = state["rho"]

                for idx, duty in enumerate(duties):
                    fan.set_speed(duty)
                    with state_lock: state["fan_duty"] = duty
                    time.sleep(stab)
                    # Mesure ΔP (moyenne sur 20 samples × 0.1s = 2s)
                    dp_vals = []
                    rpm_vals = []
                    for _ in range(20):
                        dp_vals.append(pressure.delta_pa(n_avg=3))
                        rpm_vals.append(fan.rpm)
                        time.sleep(0.1)
                    dp_mean  = sum(dp_vals) / len(dp_vals) if dp_vals else 0.0
                    rpm_mean = sum(rpm_vals) / len(rpm_vals) if rpm_vals else 0.0
                    v_pres   = airspeed_from_pressure(dp_mean, rho)
                    v_fan    = airspeed_from_fan(rpm_mean, kv, 0.0)
                    pt = {
                        "idx":    idx + 1,
                        "duty":   round(duty, 1),
                        "rpm":    int(rpm_mean),
                        "dp":     round(dp_mean, 2),
                        "v_pres": round(v_pres, 3),
                        "v_fan":  round(v_fan, 3),
                        "kv_fan": kv,
                        "progress": (idx + 1) / pts,
                        "done":   False,
                    }
                    calib_table.append(pt)
                    _sse(pt)

                fan.set_speed(0)
                with state_lock:
                    state["fan_duty"] = 0.0
                    state["speed_calib_table"] = calib_table
                _save_calib()
                _sse({"done": True, "n": len(calib_table)})
            except Exception as ex:
                fan.set_speed(0)
                with state_lock: state["fan_duty"] = 0.0
                _sse({"error": str(ex)})

        elif path == "/media/analyze":
            fname     = q("name", "")
            mode      = q("mode", "heatmap")   # heatmap | arrows | vorticity | flow_x | flow_y | streamlines
            flow_dir  = q("dir", "all")         # all | pos | neg
            if not fname or "/" in fname or ".." in fname:
                self._json({"ok": False, "err": "nom invalide"}); return
            if mode not in ("heatmap", "arrows", "vorticity", "flow_x", "flow_y", "streamlines"):
                mode = "heatmap"
            if flow_dir not in ("all", "pos", "neg"):
                flow_dir = "all"
            src = os.path.join(DATA_DIR_VIDEO, fname)
            if not os.path.isfile(src):
                self._json({"ok": False, "err": "fichier introuvable"}); return
            dir_sfx  = f"_{flow_dir}" if flow_dir != "all" else ""
            out_name = fname.replace(".mp4", f"_flow_{mode}{dir_sfx}.mp4")
            out_path = os.path.join(DATA_DIR_VIDEO, out_name)
            with _video_analysis_lock:
                if fname in _video_analysis_jobs and not _video_analysis_jobs[fname].get("done"):
                    self._json({"ok": False, "err": "analyse deja en cours"}); return
                _video_analysis_jobs[fname] = {"progress": 0.0, "done": False, "err": "",
                                               "out": out_name}

            def _run_analysis(src_path, dst_path, job_key, flow_mode, flow_dir="all"):
                import numpy as _np
                import json as _json
                err      = ""
                csv_path = dst_path.replace(".mp4", ".csv")
                tmp_avi  = dst_path.replace(".mp4", "_tmp.avi")

                # Nettoie toute sortie précédente cassée
                for _p in (dst_path, tmp_avi, csv_path):
                    try:
                        if os.path.isfile(_p): os.remove(_p)
                    except Exception:
                        pass

                # Dimensions et fps via ffprobe (fiable pour tous codecs)
                try:
                    probe_r = subprocess.run(
                        ["ffprobe", "-v", "quiet", "-print_format", "json",
                         "-show_streams", src_path],
                        capture_output=True, text=True, timeout=30)
                    info = _json.loads(probe_r.stdout)
                    vs   = next(s for s in info["streams"] if s["codec_type"] == "video")
                    tw   = int(vs["width"])
                    th   = int(vs["height"])
                    _n, _d = vs.get("r_frame_rate", "25/1").split("/")
                    fps  = float(_n) / max(float(_d), 1)
                    total = int(vs.get("nb_frames", 0)) or 200
                except Exception as ex:
                    with _video_analysis_lock:
                        _video_analysis_jobs[job_key]["done"] = True
                        _video_analysis_jobs[job_key]["err"]  = f"ffprobe: {ex}"
                    return

                lores_w    = 320
                lores_h    = max(1, int(lores_w * th / tw)) if tw else 240
                frame_bytes = tw * th * 3

                writer = cv2.VideoWriter(tmp_avi,
                                         cv2.VideoWriter_fourcc(*"MJPG"),
                                         fps, (tw, th))
                pyr=0.5; levels=6; wsize=30; iters=5; poly_n=7; poly_s=1.5
                prev_gray = None
                n         = 0
                csv_rows  = []

                try:
                    # Décodage via pipe ffmpeg — contourne la limite H264 d'OpenCV sur Pi
                    proc = subprocess.Popen(
                        ["ffmpeg", "-i", src_path,
                         "-f", "rawvideo", "-pix_fmt", "bgr24", "-"],
                        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                    )
                    while True:
                        raw = proc.stdout.read(frame_bytes)
                        if len(raw) < frame_bytes:
                            break
                        frame_bgr = _np.frombuffer(raw, dtype=_np.uint8).reshape((th, tw, 3)).copy()
                        gray   = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
                        gray_s = cv2.resize(gray, (lores_w, lores_h))

                        flow_mag_mean = 0.0; flow_x_mean = 0.0
                        flow_y_mean   = 0.0; vort_max    = 0.0

                        if prev_gray is not None:
                            flow = cv2.calcOpticalFlowFarneback(
                                prev_gray, gray_s, None,
                                pyr, levels, wsize, iters, poly_n, poly_s, 0)
                            mag, ang = cv2.cartToPolar(flow[..., 0], flow[..., 1])
                            flow_mag_mean = float(mag.mean())
                            flow_x_mean   = float(flow[..., 0].mean())
                            flow_y_mean   = float(flow[..., 1].mean())
                            sx = tw / lores_w; sy = th / lores_h

                            if flow_mode == "heatmap":
                                hsv = _np.zeros((*mag.shape, 3), dtype=_np.uint8)
                                hsv[..., 0] = (ang * 90 / _np.pi).astype(_np.uint8)
                                hsv[..., 1] = 255
                                p95  = float(_np.percentile(mag, 95)) if mag.max() > 0 else 1.0
                                gain = min(255.0 / max(p95, 0.05), 120.0)
                                hsv[..., 2] = _np.clip(mag * gain, 0, 255).astype(_np.uint8)
                                color = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
                                color = cv2.resize(color, (tw, th))
                                frame_bgr = cv2.addWeighted(frame_bgr, 0.45, color, 0.55, 0)

                            elif flow_mode == "vorticity":
                                fy_, fx_ = _np.gradient(flow[..., 1]), _np.gradient(flow[..., 0])
                                vort = fy_[1] - fx_[0]
                                vm   = max(float(_np.abs(vort).max()), 1e-6)
                                vort_max = vm
                                norm = _np.clip(vort / vm, -1, 1)
                                color = _np.zeros((*vort.shape, 3), dtype=_np.uint8)
                                color[..., 0] = _np.clip(-norm * 255, 0, 255).astype(_np.uint8)
                                color[..., 2] = _np.clip( norm * 255, 0, 255).astype(_np.uint8)
                                color = cv2.resize(color, (tw, th))
                                frame_bgr = cv2.addWeighted(frame_bgr, 0.5, color, 0.5, 0)

                            elif flow_mode == "flow_x":
                                # Composante U (axe X = sens écoulement) uniquement
                                # rouge = flux aval (+X), bleu = retour (−X), Y ignoré
                                u = flow[..., 0].copy()
                                if flow_dir == "pos":   u = _np.clip(u, 0, None)
                                elif flow_dir == "neg": u = _np.clip(u, None, 0)
                                u_max = max(float(_np.abs(u).max()), 0.5)
                                norm  = _np.clip(u / u_max, -1.0, 1.0)
                                color = _np.zeros((*u.shape, 3), dtype=_np.uint8)
                                color[..., 0] = _np.clip(-norm * 255, 0, 255).astype(_np.uint8)
                                color[..., 2] = _np.clip( norm * 255, 0, 255).astype(_np.uint8)
                                color = cv2.resize(color, (tw, th))
                                frame_bgr = cv2.addWeighted(frame_bgr, 0.4, color, 0.6, 0)

                            elif flow_mode == "flow_y":
                                # Composante V (axe Y = transversal) uniquement
                                # vert = bas (+Y), violet = haut (−Y)
                                v = flow[..., 1].copy()
                                if flow_dir == "pos":   v = _np.clip(v, 0, None)
                                elif flow_dir == "neg": v = _np.clip(v, None, 0)
                                v_max = max(float(_np.abs(v).max()), 0.5)
                                norm  = _np.clip(v / v_max, -1.0, 1.0)
                                color = _np.zeros((*v.shape, 3), dtype=_np.uint8)
                                color[..., 1] = _np.clip( norm * 255, 0, 255).astype(_np.uint8)  # vert +Y
                                color[..., 0] = _np.clip(-norm * 255, 0, 255).astype(_np.uint8)  # violet -Y (B+R)
                                color[..., 2] = _np.clip(-norm * 255, 0, 255).astype(_np.uint8)
                                color = cv2.resize(color, (tw, th))
                                frame_bgr = cv2.addWeighted(frame_bgr, 0.4, color, 0.6, 0)

                            else:  # arrows
                                step = max(4, lores_w // 20)
                                hs, ws = mag.shape
                                for yy in range(step // 2, hs, step):
                                    for xx in range(step // 2, ws, step):
                                        fx_, fy_ = flow[yy, xx]
                                        if fx_*fx_ + fy_*fy_ < 0.05:
                                            continue
                                        p1 = (int(xx * sx), int(yy * sy))
                                        p2 = (int((xx + fx_ * 4) * sx),
                                              int((yy + fy_ * 4) * sy))
                                        cv2.arrowedLine(frame_bgr, p1, p2,
                                                        (0, 255, 255), 1, tipLength=0.35)

                            csv_rows.append((n, round(n / fps, 3),
                                             round(flow_mag_mean, 4), round(flow_x_mean, 4),
                                             round(flow_y_mean, 4), round(vort_max, 6)))

                        prev_gray = gray_s
                        writer.write(frame_bgr)
                        n += 1
                        with _video_analysis_lock:
                            _video_analysis_jobs[job_key]["progress"] = n / total * 0.8
                    try:
                        proc.wait(timeout=5)
                    except Exception:
                        proc.kill()
                except Exception as ex:
                    err = str(ex)
                finally:
                    writer.release()

                # Sauvegarde CSV par frame
                if csv_rows:
                    try:
                        with open(csv_path, "w", newline="") as cf:
                            cw = csv.writer(cf)
                            cw.writerow(["frame_idx", "time_s", "flow_mag",
                                         "flow_x", "flow_y", "vort_max"])
                            cw.writerows(csv_rows)
                    except Exception:
                        pass

                # Conversion MJPG AVI → H264 MP4 (lisible navigateur)
                if not err and os.path.isfile(tmp_avi):
                    with _video_analysis_lock:
                        _video_analysis_jobs[job_key]["status"] = "conversion H264…"
                    try:
                        r = subprocess.run(
                            ["ffmpeg", "-y", "-i", tmp_avi,
                             "-c:v", "libx264", "-crf", "22", "-preset", "fast",
                             "-movflags", "+faststart", dst_path],
                            capture_output=True, timeout=600)
                        if r.returncode != 0:
                            err = r.stderr.decode()[-300:]
                        else:
                            try: os.remove(tmp_avi)
                            except Exception: pass
                    except Exception as ex:
                        err = str(ex)

                with _video_analysis_lock:
                    _video_analysis_jobs[job_key]["done"]     = True
                    _video_analysis_jobs[job_key]["progress"] = 1.0
                    _video_analysis_jobs[job_key]["err"]      = err
                    if csv_rows:
                        _video_analysis_jobs[job_key]["csv"] = os.path.basename(csv_path)

            threading.Thread(target=_run_analysis,
                             args=(src, out_path, fname, mode, flow_dir),
                             daemon=True).start()
            self._json({"ok": True, "out": out_name})

        elif path == "/media/delete":
            fname = q("name", "")
            ftype = q("type", "")
            if not fname or "/" in fname or ".." in fname:
                self._json({"ok": False, "err": "nom invalide"}); return
            deleted = False
            if ftype == "video":
                fp = os.path.join(DATA_DIR_VIDEO, fname)
                # CSV de log (data/raw)
                cp = os.path.join(DATA_DIR_CSV, fname.replace(".mp4", ".csv"))
                # CSV d'analyse flux (data/videos, même nom .csv)
                ap = os.path.join(DATA_DIR_VIDEO, fname.replace(".mp4", ".csv"))
                if os.path.isfile(fp):
                    os.remove(fp); deleted = True
                if os.path.isfile(cp):
                    try: os.remove(cp)
                    except Exception: pass
                if os.path.isfile(ap):
                    try: os.remove(ap)
                    except Exception: pass
            elif ftype == "photo":
                fp = os.path.join(DATA_DIR_PHOTOS, fname)
                if os.path.isfile(fp):
                    os.remove(fp); deleted = True
            if deleted:
                self._json({"ok": True})
            else:
                self._json({"ok": False, "err": "fichier introuvable"})

        elif path == "/save":
            ok, err = _save_config()
            self._json({"ok": ok, "err": err})

        elif path == "/wifi/status":
            try:
                r = subprocess.run(
                    ["nmcli", "-t", "-f", "NAME,TYPE,DEVICE,STATE", "con", "show", "--active"],
                    capture_output=True, text=True, timeout=5)
                hotspot_active = False
                client_ssid = None
                for line in r.stdout.strip().split('\n'):
                    parts = line.split(':')
                    if len(parts) >= 4:
                        name, ctype, dev, st = parts[0], parts[1], parts[2], parts[3]
                        if name == "Soufflerie-AP" and "activated" in st:
                            hotspot_active = True
                        elif "wireless" in ctype and name != "Soufflerie-AP" and "activated" in st:
                            client_ssid = name
                ip_r = subprocess.run(["ip", "-4", "addr", "show", "wlan0"],
                                      capture_output=True, text=True, timeout=3)
                wlan0_ip = ""
                for line in ip_r.stdout.split('\n'):
                    if 'inet ' in line:
                        wlan0_ip = line.strip().split()[1].split('/')[0]; break
                self._json({"ok": True, "hotspot_active": hotspot_active,
                            "client_ssid": client_ssid, "wlan0_ip": wlan0_ip})
            except Exception as e:
                self._json({"ok": False, "err": str(e)})

        elif path == "/wifi/hotspot":
            if not WIFI_CONTROL_ENABLED:
                self._json({"ok": False, "err": "Commande Wi-Fi désactivée (SOUFFLERIE_ENABLE_WIFI_CONTROL=1)"}); return
            enable = qs.get("enable", ["0"])[0] == "1"
            try:
                if enable:
                    r = subprocess.run(["nmcli", "con", "up", "Soufflerie-AP"],
                                       capture_output=True, text=True, timeout=15)
                    if r.returncode != 0:
                        self._json({"ok": False, "err": (r.stderr or r.stdout).strip()}); return
                    subprocess.run(["nmcli", "con", "modify", "Soufflerie-AP",
                                    "connection.autoconnect", "yes"],
                                   capture_output=True, text=True, timeout=10)
                else:
                    r = subprocess.run(["nmcli", "con", "down", "Soufflerie-AP"],
                                       capture_output=True, text=True, timeout=10)
                    if r.returncode != 0:
                        self._json({"ok": False, "err": (r.stderr or r.stdout).strip()}); return
                    subprocess.run(["nmcli", "con", "modify", "Soufflerie-AP",
                                    "connection.autoconnect", "no"],
                                   capture_output=True, text=True, timeout=10)
                self._json({"ok": True})
            except Exception as e:
                self._json({"ok": False, "err": str(e)})

        elif path == "/wifi/connect":
            if not WIFI_CONTROL_ENABLED:
                self._json({"ok": False, "err": "Commande Wi-Fi désactivée (SOUFFLERIE_ENABLE_WIFI_CONTROL=1)"}); return
            content_length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(content_length).decode()
            params = parse_qs(body)
            ssid = params.get("ssid", [""])[0]
            password = params.get("password", [""])[0]
            if not ssid:
                self._json({"ok": False, "err": "ssid requis"}); return
            try:
                subprocess.run(["nmcli", "con", "down", "Soufflerie-AP"],
                               capture_output=True, text=True, timeout=10)
                cmd = ["nmcli", "device", "wifi", "connect", ssid]
                if password:
                    cmd += ["password", password]
                r = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
                if r.returncode == 0:
                    time.sleep(2)
                    ip_r = subprocess.run(["ip", "-4", "addr", "show", "wlan0"],
                                          capture_output=True, text=True, timeout=3)
                    ip = ""
                    for line in ip_r.stdout.split('\n'):
                        if 'inet ' in line:
                            ip = line.strip().split()[1].split('/')[0]; break
                    self._json({"ok": True, "ip": ip})
                else:
                    self._json({"ok": False, "err": (r.stderr or r.stdout).strip()})
            except Exception as e:
                self._json({"ok": False, "err": str(e)})

        elif path == "/wifi/scan":
            try:
                r = subprocess.run(
                    ["nmcli", "-t", "-f", "SSID,SIGNAL,SECURITY", "device", "wifi", "list"],
                    capture_output=True, text=True, timeout=15)
                networks = []
                seen = set()
                for line in r.stdout.strip().split('\n'):
                    parts = line.split(':')
                    if len(parts) >= 2:
                        ssid = parts[0].strip()
                        if ssid and ssid not in seen:
                            seen.add(ssid)
                            try:
                                sig = int(parts[1])
                                dbm = sig // 2 - 100
                            except Exception:
                                dbm = -100
                            security = len(parts) > 2 and bool(parts[2].strip())
                            networks.append({"ssid": ssid, "signal": dbm, "security": security})
                networks.sort(key=lambda x: -x["signal"])
                self._json({"ok": True, "networks": networks})
            except Exception as e:
                self._json({"ok": False, "err": str(e)})

        else:
            self.send_response(404); self.end_headers()


if __name__ == "__main__":
    print(f"Dashboard: http://{HTTP_BIND_HOST}:8080")
    try:
        ThreadingHTTPServer((HTTP_BIND_HOST, 8080), Handler).serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        fan.stop(); fan.cleanup()
        led.stop()
        picam.stop(); picam.close()
        GPIO.cleanup()
