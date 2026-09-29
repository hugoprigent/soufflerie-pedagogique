# GPIO (BCM)
PIN_FAN_PWM = 18
PIN_FAN_TACH = 17
PIN_HX711_DT = 23
PIN_HX711_SCK = 24

# Modulino Knob I2C
MODULINO_I2C_ADDR = 0x3A

# Capteur pression BMP280 (I2C, même bus que Modulino)
# Câblage : SDA→GPIO2/Pin3, SCL→GPIO3/Pin5, VCC→3.3V, GND→GND, SDO→GND
PRESSURE_I2C_BUS  = 1
PRESSURE_I2C_ADDR = 0x76     # SDO→GND=0x76 ; SDO→VCC=0x77

# PWM ventilateur
PWM_FREQUENCY = 25000
PWM_MIN_DUTY = 20
PWM_MAX_DUTY = 80

# HX711 — calibration (ADU/g). Tare = offset, scale = ADU par gramme.
HX711_OFFSET = 0
HX711_SCALE = 1.0
FORCE_FILTER_WINDOW = 10     # fenetre moyenne glissante (echantillons)
FORCE_SAMPLE_HZ = 20         # freq boucle force
SUPPORT_DRAG_N = 0.0         # trainee aero du support seul (tare aero, N)

# Geometrie tunnel (variables — modifier si la section change)
A_FAN_M2  = 0.0144    # m² — section ventilateur 120×120 mm (0.12*0.12)
A_TEST_M2 = 0.0100    # m² — section d'essai     100×100 mm (0.10*0.10)
# Rapport de contraction (continuité Q = V_fan*A_fan = V_test*A_test)
CONTRACTION = A_FAN_M2 / A_TEST_M2   # = 1.44

# Profil et aero (utilise pour CL/CD/Re/St)
AIR_DENSITY = 1.204          # kg/m^3 @ 20 C
AIR_VISCOSITY = 1.82e-5      # Pa.s
PROFILE_CHORD_M = 0.10       # corde (m)
PROFILE_SPAN_M = 0.10        # envergure (m) — section d'essai 100x100 mm
PROFILE_AOA_DEG = 0.0        # angle d attaque par defaut

# Vent estime depuis le ventilateur : V = FAN_KV * RPM + FAN_K0
#
# Calibration depuis fiche technique Orion Fans OD1238 VXC (12V) :
#   A_fan = 120x120 mm = 0.0144 m²   (1 CFM = 4.71947e-4 m³/s)
#   LB : 3000 RPM -> Q=0.06589 m³/s -> V_section=Q/A_TEST=6.589 m/s -> KV=0.002196
#   MB : 4000 RPM -> Q=0.08898 m³/s -> V_section=Q/A_TEST=8.898 m/s -> KV=0.002225
#   HB : 5000 RPM -> Q=0.10666 m³/s -> V_section=Q/A_TEST=10.666 m/s-> KV=0.002133
#   Moyenne : KV_section ≈ 0.00219 m/s/RPM = KV_fan * CONTRACTION (1.44)
#
#   Note : valeurs a Q_max (ΔP=0). En tunnel reel avec pertes η<1 → calibrer
#   empiriquement via /calibrate (section "Calibration pression BMP280").
FAN_KV = 0.00219             # m/s par RPM — section d'essai 100×100 mm, Q_max
FAN_K0 = 0.0                 # offset (m/s) : 0 car modele lineaire par l'origine

# Camera
CAM_WIDTH = 1280
CAM_HEIGHT = 960
CAM_FPS = 30
CAM_PX_PER_M = 1000.0        # calibration px/m (a regler avec une regle dans la scene)

# Camera controles (picamera2 set_controls)
CAM_BRIGHTNESS = 0.0         # [-1, 1]
CAM_CONTRAST = 1.0           # [0, 32] 1 = neutre
CAM_SATURATION = 1.0         # [0, 32] 1 = neutre
CAM_AE_ENABLE = True
CAM_AWB_ENABLE = True
CAM_EXPOSURE_US = 10000      # si AE off
CAM_GAIN = 1.0               # si AE off

# Flow (Farneback) — regle pour fumee lente
FLOW_PYR_SCALE = 0.5
FLOW_LEVELS = 5       # plus de niveaux = detecte les grands deplacements
FLOW_WINSIZE = 25     # fenetre plus large = plus sensible aux petits gradients
FLOW_ITERATIONS = 5   # plus d'iterations = plus precis
FLOW_POLY_N = 7       # voisinage polynomial plus grand
FLOW_POLY_SIGMA = 1.5
FLOW_FLAGS = 0

# Logging
DATA_DIR = "data/raw"
VIDEO_DIR = "data/videos"
CSV_HEADER = ["timestamp", "duty_pct", "rpm", "force_N", "airspeed_ms", "flow_mag", "frame_idx"]
