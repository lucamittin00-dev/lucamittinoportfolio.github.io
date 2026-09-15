import serial
import time
import tkinter as tk
from tkinter import messagebox, simpledialog
import re
import cv2
import numpy as np
from vmbpy import *
from PIL import Image, ImageTk
import threading
import os
import datetime
import logging

# --- CONFIGURACIÓN PRINCIPAL ---
PUERTO_ANET = "COM6"
PUERTO_LED = "COM7"
BAUDIOS = 115200

# Carpeta donde se guardan las capturas. Puede ser relativa (a la carpeta
# desde donde se ejecuta el script) o absoluta, por ejemplo:
#   CARPETA_CAPTURAS = "capturas"
#   CARPETA_CAPTURAS = r"C:\Usuarios\Luca\Documentos\Microscopio\capturas"
#   CARPETA_CAPTURAS = r"D:\Proyectos\Microscopio\capturas"
CARPETA_CAPTURAS = r"G:\30_Practica Luca\2.0 Codigo\Capturas Del Micro"

# --- LOGGING (reemplaza los print() sueltos, con niveles y timestamps) ---
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("microscopio")


class PanelControlAnetPro:
    def __init__(self, root):
        self.root = root
        self.root.title("ANET A8 - MASTER CONTROL (CON ILUMINACIÓN LED)")
        self.root.configure(bg="#121212")
        self.frame_crudo_actual = None  # Almacena la imagen en alta resolución

        ancho_v, alto_v = 1300, 950
        pos_x = int(self.root.winfo_screenwidth() / 2 - ancho_v / 2)
        pos_y = int(self.root.winfo_screenheight() / 2 - alto_v / 2)
        self.root.geometry(f"{ancho_v}x{alto_v}+{pos_x}+{pos_y}")

        # --- VARIABLES DE CONTROL ---
        self.camara_activa = True
        self.ultima_imagen = None

        # Incrementos independientes por eje
        self.paso_x = tk.DoubleVar(value=5.0)
        self.paso_y = tk.DoubleVar(value=5.0)
        self.paso_z = tk.DoubleVar(value=0.5)

        # Buzones de órdenes cámara (Hardware)
        # NOTA: usamos None como "sin cambios pendientes" y comprobamos con
        # "is not None" en vez de verdad/falsedad, porque 0.0 es un valor
        # válido para gamma, ganancia y balance de blancos, y antes se
        # perdía silenciosamente si el usuario arrastraba un slider a 0.
        self.cmd_exposicion = None
        self.cmd_ganancia = None
        self.cmd_balance_r = None
        self.cmd_balance_b = None
        self.cmd_gamma = None
        self.cmd_blacklevel = None

        # Foco asistido (varianza de Laplaciano sobre el frame actual)
        self.nitidez_actual = 0.0

        # --- AUTOENFOQUE (barrido de grueso a fino en 3 pasadas sobre Z) ---
        # Cada pasada recorre un rango alrededor de la posición actual,
        # mide la nitidez en cada parada, y al terminar vuelve al mejor
        # punto encontrado. Ese punto se usa como centro de la siguiente
        # pasada, con un rango y un paso más finos.
        self.autoenfoque_activo = False
        self._af_pasadas = [
            (0.60, 0.05),   # pasada 1: rango ±0.30 mm, paso 0.05 mm (barrido grueso)
            (0.15, 0.015),  # pasada 2: rango ±0.075 mm, paso 0.015 mm (refinar)
            (0.03, 0.003),  # pasada 3: rango ±0.015 mm, paso 0.003 mm (fino)
        ]
        self._af_pasada_idx = 0
        self._af_paso_actual = 0.0
        self._af_pasos_totales = 0
        self._af_paso_index = 0
        self._af_offset_actual = 0.0
        self._af_mejor_offset = 0.0
        self._af_mejor_nitidez = -1.0
        self._af_after_id = None

        # --- CALIBRACIÓN (µm por pixel, sobre el frame RAW, no el redimensionado) ---
        self.um_por_pixel = None  # None = sin calibrar
        self._calib_puntos = []   # puntos clicados (en coords del frame raw)

        # Jogging por teclado (igual que en el panel de impresión):
        # usamos un bucle after() con filtro de auto-repetición de Windows
        # y un JOG_STEP dinámico con factor de solape 1.3x para movimiento suave.
        self._teclas_activas = set()
        self._jog_after_id = None
        self.JOG_STEP_BASE = 0.5
        self.jog_step_actual = self.JOG_STEP_BASE

        # --- CONEXIÓN SERIAL IMPRESORA ---
        self.ser = None
        try:
            self.ser = serial.Serial(PUERTO_ANET, BAUDIOS, timeout=1)
            time.sleep(5)
            self.ser.reset_input_buffer()
            self.ser.write(b"G91\n")  # modo relativo una sola vez al inicio
            log.info(f"Impresora conectada en {PUERTO_ANET}")
        except Exception as e:
            log.warning(f"Impresora no detectada físicamente en {PUERTO_ANET}: {e}")

        # --- CONEXIÓN SERIAL LED (persistente, en vez de abrir/cerrar cada vez) ---
        self.ser_led = None
        try:
            self.ser_led = serial.Serial(PUERTO_LED, 9600, timeout=1)
            time.sleep(2)
            log.info(f"Relé LED conectado en {PUERTO_LED}")
        except Exception as e:
            log.warning(f"Relé LED no detectado en {PUERTO_LED}: {e}")

        # =========================================================
        # DISEÑO DE LA INTERFAZ GRÁFICA (GUI)
        # =========================================================

        # 1. PANEL IZQUIERDO: CNC, SENSORES Y LED
        self.frame_izq = tk.Frame(root, bg="#1a1a1a", padx=10, pady=10, highlightbackground="#333", highlightthickness=1)
        self.frame_izq.pack(side="left", fill="y", padx=10, pady=10)

        tk.Label(self.frame_izq, text="SISTEMA DE CONTROL", fg="#00FF41", bg="#1a1a1a", font=("Orbitron", 12, "bold")).pack(pady=10)
        tk.Button(self.frame_izq, text="📷 GUARDAR CAPTURA", bg="#1565C0", fg="white",
                  font=("Arial", 10, "bold"), height=2, command=self.guardar_imagen).pack(fill="x", pady=10)

        # --- CALIBRACIÓN ---
        tk.Button(self.frame_izq, text="📏 CALIBRAR (2 clics)", bg="#6A1B9A", fg="white",
                  font=("Arial", 9, "bold"), command=self.abrir_calibracion).pack(fill="x", pady=(0, 5))
        self.lbl_calib = tk.Label(self.frame_izq, text="SIN CALIBRAR", fg="#FF7043", bg="#1a1a1a", font=("Arial", 8, "bold"))
        self.lbl_calib.pack(pady=(0, 10))

        # --- SECCIÓN TEMPERATURA ---
        frame_temp = tk.Frame(self.frame_izq, bg="#121212", pady=10, padx=10, highlightbackground="#333", highlightthickness=1)
        frame_temp.pack(fill="x", pady=5)
        self.lbl_t = tk.Label(frame_temp, text="HOTEND: --°C", fg="#FF3D00", bg="#121212", font=("Consolas", 14, "bold"))
        self.lbl_t.pack()
        self.lbl_b = tk.Label(frame_temp, text="BED: --°C", fg="#00E5FF", bg="#121212", font=("Consolas", 14, "bold"))
        self.lbl_b.pack()
        tk.Button(frame_temp, text="LEER SENSORES (T)", bg="#333", fg="white", font=("Arial", 8, "bold"), command=self.pedir_temp).pack(pady=5)

        # --- SECCIÓN FOCO ASISTIDO ---
        frame_foco = tk.LabelFrame(self.frame_izq, text=" FOCO (nitidez) ", fg="#aaaaaa", bg="#1a1a1a", font=("Arial", 9, "bold"), pady=5)
        frame_foco.pack(fill="x", pady=5)
        self.lbl_nitidez = tk.Label(frame_foco, text="Nitidez: --", fg="#FFEB3B", bg="#1a1a1a", font=("Consolas", 12, "bold"))
        self.lbl_nitidez.pack(pady=3)
        self.btn_autofocus = tk.Button(frame_foco, text="🔍 AUTOENFOQUE", bg="#00838F", fg="white",
                                        font=("Arial", 9, "bold"), command=self.alternar_autoenfoque)
        self.btn_autofocus.pack(fill="x", padx=10, pady=(0, 5))

        # --- SECCIÓN: ILUMINACIÓN LED (RELÉ USB) ---
        frame_led = tk.LabelFrame(self.frame_izq, text=" ILUMINACIÓN LED ", fg="#aaaaaa", bg="#1a1a1a", font=("Arial", 9, "bold"), pady=10)
        frame_led.pack(fill="x", pady=10)

        self.led_encendido = False
        self.btn_led = tk.Button(frame_led, text="ENCENDER LED", bg="#2E7D32", fg="white",
                                  font=("Arial", 9, "bold"), command=self.alternar_led)
        self.btn_led.pack(fill="x", padx=10, pady=2)

        # --- MÓDULOS DE EJES INDEPENDIENTES ---
        self.crear_modulo_eje(self.frame_izq, "EJE X", self.paso_x, [0.1, 1, 5, 10], "X")
        self.crear_modulo_eje(self.frame_izq, "EJE Y", self.paso_y, [0.1, 1, 5, 10], "Y")
        self.crear_modulo_eje(self.frame_izq, "EJE Z", self.paso_z, [0.0025, 0.025, 0.1, 0.5, 1, 5], "Z")

        # Botones rápidos Z (Carga)
        frame_z_fast = tk.Frame(self.frame_izq, bg="#1a1a1a")
        frame_z_fast.pack(fill="x", pady=5)
        tk.Button(frame_z_fast, text="CARGA (+50)", bg="#1565C0", fg="white", font=("Arial", 8, "bold"), command=lambda: self.mover("Z", 50)).pack(side="left", expand=True, padx=2)
        tk.Button(frame_z_fast, text="BASE (-20)", bg="#455A64", fg="white", font=("Arial", 8, "bold"), command=lambda: self.mover("Z", -20)).pack(side="left", expand=True, padx=2)

        # Ayuda de teclado
        tk.Label(self.frame_izq, text="Teclado: flechas = X/Y   RePág/AvPág = Z",
                 fg="#666", bg="#1a1a1a", font=("Arial", 7)).pack(pady=(5, 0))

        # --- SERVICIO & CIERRE ---
        tk.Button(self.frame_izq, text="HOME (G28)", bg="#424242", fg="white", command=self.home).pack(fill="x", pady=20)
        tk.Button(self.frame_izq, text="APAGAR SISTEMA", bg="#b71c1c", fg="white", font=("Arial", 10, "bold"), command=self.cerrar).pack(side="bottom", fill="x")

        # 2. PANEL DERECHO: VISIÓN
        self.frame_der = tk.Frame(root, bg="#121212")
        self.frame_der.pack(side="right", expand=True, fill="both", padx=10, pady=10)

        self.frame_video = tk.Frame(self.frame_der, bg="black", highlightbackground="#444", highlightthickness=1)
        self.frame_video.pack(expand=True, fill="both")
        self.lbl_video = tk.Label(self.frame_video, bg="black", text="CONECTANDO CÁMARA...", fg="#00FF41")
        self.lbl_video.pack(expand=True)

        # --- SLIDERS CÁMARA ---
        self.frame_cam_ctrl = tk.Frame(self.frame_der, bg="#1f1f1f", pady=10, padx=20)
        self.frame_cam_ctrl.pack(fill="x", side="bottom")

        sliders_frame = tk.Frame(self.frame_cam_ctrl, bg="#1f1f1f")
        sliders_frame.pack(fill="x")
        sliders_frame.columnconfigure(0, weight=1)
        sliders_frame.columnconfigure(1, weight=1)

        s_cfg = {"orient": "horizontal", "bg": "#1f1f1f", "fg": "white", "troughcolor": "#333", "highlightthickness": 0, "font": ("Arial", 8)}

        self.slider_exp = tk.Scale(sliders_frame, from_=500, to=1000000, label="Exposición (µs)", command=self.upd_exposicion, **s_cfg)
        self.slider_exp.set(15000); self.slider_exp.grid(row=0, column=0, sticky="ew", padx=10)
        self.slider_gain = tk.Scale(sliders_frame, from_=0, to=24, resolution=0.1, label="Ganancia (dB)", command=self.upd_ganancia, **s_cfg)
        self.slider_gain.set(0); self.slider_gain.grid(row=0, column=1, sticky="ew", padx=10)
        self.slider_red = tk.Scale(sliders_frame, from_=0.5, to=3.0, resolution=0.05, label="Balance Rojo", command=self.upd_balance_r, **s_cfg)
        self.slider_red.set(1.0); self.slider_red.grid(row=1, column=0, sticky="ew", padx=10)
        self.slider_blue = tk.Scale(sliders_frame, from_=0.5, to=3.0, resolution=0.05, label="Balance Azul", command=self.upd_balance_b, **s_cfg)
        self.slider_blue.set(1.0); self.slider_blue.grid(row=1, column=1, sticky="ew", padx=10)
        self.slider_gamma = tk.Scale(sliders_frame, from_=0, to=3.0, resolution=0.05, label="Gamma (Contraste)", command=self.upd_gamma, **s_cfg)
        self.slider_gamma.set(1.0); self.slider_gamma.grid(row=2, column=0, sticky="ew", padx=10)
        self.slider_black = tk.Scale(sliders_frame, from_=0, to=64, label="Black Level (Nivel Negro)", command=self.upd_blacklevel, **s_cfg)
        self.slider_black.set(0); self.slider_black.grid(row=2, column=1, sticky="ew", padx=10)

        # --- BINDINGS DE TECLADO (jogging con flechas / RePág-AvPág) ---
        self.root.bind("<KeyPress>", self._tecla_abajo)
        self.root.bind("<KeyRelease>", self._tecla_arriba)

        # Hilo de Video
        self.hilo_video = threading.Thread(target=self.trabajador_camara, daemon=True)
        self.hilo_video.start()
        self.actualizar_pantalla()

    # --- FUNCIONES CONTROL LED (RELÉ USB, conexión persistente) ---
    def alternar_led(self):
        if self.led_encendido:
            self.led_off()
        else:
            self.led_on()

    def led_on(self):
        if not self.ser_led:
            log.warning("LED no disponible: puerto no conectado.")
            return
        try:
            self.ser_led.write(bytes([0xA0, 0x01, 0x01, 0xA2]))
            log.info("LED: Encendido")
            self.led_encendido = True
            self.btn_led.config(text="APAGAR LED", bg="#C62828")
        except Exception as e:
            log.error(f"Error al encender LED: {e}")

    def led_off(self):
        if not self.ser_led:
            return
        try:
            self.ser_led.write(bytes([0xA0, 0x01, 0x00, 0xA1]))
            log.info("LED: Apagado")
            self.led_encendido = False
            self.btn_led.config(text="ENCENDER LED", bg="#2E7D32")
        except Exception as e:
            log.error(f"Error al apagar LED: {e}")

    # --- FUNCIONES DE CÁMARA Y CNC ---
    # Corregido: "is not None" en vez de comprobación de verdad, para que
    # 0.0 (gamma, ganancia, balance de blancos) no se pierda silenciosamente.
    def upd_exposicion(self, val): self.cmd_exposicion = float(val)
    def upd_ganancia(self, val): self.cmd_ganancia = float(val)
    def upd_balance_r(self, val): self.cmd_balance_r = float(val)
    def upd_balance_b(self, val): self.cmd_balance_b = float(val)
    def upd_gamma(self, val): self.cmd_gamma = float(val)
    def upd_blacklevel(self, val): self.cmd_blacklevel = float(val)

    def crear_modulo_eje(self, parent, titulo, variable, pasos, eje):
        frame = tk.LabelFrame(parent, text=f" {titulo} ", fg="#aaaaaa", bg="#1a1a1a", font=("Arial", 9, "bold"), pady=5)
        frame.pack(fill="x", pady=5)
        sel_frame = tk.Frame(frame, bg="#1a1a1a")
        sel_frame.pack(fill="x")
        for p in pasos:
            tk.Radiobutton(sel_frame, text=str(p), variable=variable, value=p, bg="#1a1a1a", fg="#00FF41",
                           selectcolor="#333", font=("Arial", 8)).pack(side="left", expand=True)
        btn_frame = tk.Frame(frame, bg="#1a1a1a")
        btn_frame.pack(pady=5)
        b_std = {"width": 10, "bg": "#333", "fg": "white", "font": ("Arial", 9, "bold")}
        if eje == "X":
            tk.Button(btn_frame, text="X -", command=lambda: self.mover("X", -variable.get()), **b_std).pack(side="left", padx=2)
            tk.Button(btn_frame, text="X +", command=lambda: self.mover("X", variable.get()), **b_std).pack(side="left", padx=2)
        elif eje == "Y":
            tk.Button(btn_frame, text="Y -", command=lambda: self.mover("Y", -variable.get()), **b_std).pack(side="left", padx=2)
            tk.Button(btn_frame, text="Y +", command=lambda: self.mover("Y", variable.get()), **b_std).pack(side="left", padx=2)
        else:
            tk.Button(btn_frame, text="Z ↑ (Subir)", bg="#2E7D32", fg="white", font=("Arial", 9, "bold"), width=10, command=lambda: self.mover("Z", variable.get())).pack(side="left", padx=2)
            tk.Button(btn_frame, text="Z ↓ (Bajar)", bg="#C62828", fg="white", font=("Arial", 9, "bold"), width=10, command=lambda: self.mover("Z", -variable.get())).pack(side="left", padx=2)

    def trabajador_camara(self):
        try:
            with VmbSystem.get_instance() as vmb:
                cams = vmb.get_all_cameras()
                if not cams:
                    log.warning("No se detectó ninguna cámara Vimba.")
                    self.root.after(0, lambda: self.lbl_video.config(text="CÁMARA NO DETECTADA"))
                    self.root.after(0, lambda: messagebox.showwarning(
                        "Cámara no detectada", "No se encontró ninguna cámara Vimba conectada."))
                    return
                with cams[0] as cam:
                    try:
                        cam.get_feature_by_name('ExposureAuto').set('Off')
                        cam.get_feature_by_name('GainAuto').set('Off')
                        cam.get_feature_by_name('BalanceWhiteAuto').set('Off')
                        cam.get_feature_by_name('BlackLevelAuto').set('Off')
                        cam.get_feature_by_name('PixelFormat').set('Bgr8')
                    except Exception as e:
                        log.debug(f"No se pudieron fijar los modos automáticos de la cámara: {e}")

                    def procesar_frame(camara, stream, frame):
                        if frame.get_status() == FrameStatus.Complete:
                            img = frame.as_numpy_ndarray()
                            self.frame_crudo_actual = img.copy()

                            # Nitidez (foco asistido) sobre el frame completo, antes de reescalar
                            try:
                                gris = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                                self.nitidez_actual = cv2.Laplacian(gris, cv2.CV_64F).var()
                            except Exception:
                                pass

                            # Redimensiona conservando la relación de aspecto (antes deformaba la imagen)
                            img = self._resize_conservando_aspecto(img, 900, 600)
                            img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                            self.ultima_imagen = Image.fromarray(img_rgb)
                        camara.queue_frame(frame)

                    cam.start_streaming(handler=procesar_frame, buffer_count=5)

                    while self.camara_activa:
                        if self.cmd_exposicion is not None:
                            try: cam.get_feature_by_name('ExposureTime').set(self.cmd_exposicion)
                            except Exception as e: log.debug(f"ExposureTime: {e}")
                            self.cmd_exposicion = None
                        if self.cmd_ganancia is not None:
                            try: cam.get_feature_by_name('Gain').set(self.cmd_ganancia)
                            except Exception as e: log.debug(f"Gain: {e}")
                            self.cmd_ganancia = None
                        if self.cmd_balance_r is not None:
                            try:
                                cam.get_feature_by_name('BalanceRatioSelector').set('Red')
                                cam.get_feature_by_name('BalanceRatio').set(self.cmd_balance_r)
                            except Exception as e: log.debug(f"BalanceRatio Red: {e}")
                            self.cmd_balance_r = None
                        if self.cmd_balance_b is not None:
                            try:
                                cam.get_feature_by_name('BalanceRatioSelector').set('Blue')
                                cam.get_feature_by_name('BalanceRatio').set(self.cmd_balance_b)
                            except Exception as e: log.debug(f"BalanceRatio Blue: {e}")
                            self.cmd_balance_b = None
                        if self.cmd_gamma is not None:
                            try: cam.get_feature_by_name('Gamma').set(self.cmd_gamma)
                            except Exception as e: log.debug(f"Gamma: {e}")
                            self.cmd_gamma = None
                        if self.cmd_blacklevel is not None:
                            try: cam.get_feature_by_name('BlackLevel').set(self.cmd_blacklevel)
                            except Exception as e: log.debug(f"BlackLevel: {e}")
                            self.cmd_blacklevel = None
                        time.sleep(0.05)
                    cam.stop_streaming()
        except Exception as e:
            log.error(f"Error Vimba: {e}")
            self.root.after(0, lambda: messagebox.showwarning(
                "Error de cámara", f"No se pudo inicializar la cámara:\n{e}"))

    @staticmethod
    def _resize_conservando_aspecto(img, ancho_max, alto_max):
        """Reescala manteniendo la relación de aspecto y rellena (letterbox)
        en vez de deformar la imagen como hacía cv2.resize directo."""
        h, w = img.shape[:2]
        escala = min(ancho_max / w, alto_max / h)
        nuevo_w, nuevo_h = int(w * escala), int(h * escala)
        redim = cv2.resize(img, (nuevo_w, nuevo_h))
        lienzo = np.zeros((alto_max, ancho_max, 3), dtype=np.uint8)
        x_off = (ancho_max - nuevo_w) // 2
        y_off = (alto_max - nuevo_h) // 2
        lienzo[y_off:y_off + nuevo_h, x_off:x_off + nuevo_w] = redim
        return lienzo

    def actualizar_pantalla(self):
        if self.ultima_imagen:
            img_tk = ImageTk.PhotoImage(image=self.ultima_imagen)
            self.lbl_video.img_tk = img_tk
            self.lbl_video.config(image=img_tk)
        self.lbl_nitidez.config(text=f"Nitidez: {self.nitidez_actual:.1f}")
        self.root.after(30, self.actualizar_pantalla)

    # --- MOVIMIENTO CNC ---
    def home(self):
        if self.ser:
            self.ser.write(b"G28\n")
            self._esperar_ok()

    def mover(self, eje, dist):
        if not self.ser:
            return
        vel = 3000 if eje in ["X", "Y"] else 200
        # Nota: ya se fijó G91 (modo relativo) una sola vez en __init__,
        # así que no hace falta reenviarlo en cada movimiento.
        comando = f"G1 {eje}{dist} F{vel}\n"
        self.ser.write(comando.encode())
        self._esperar_ok()

    def _esperar_ok(self, timeout=0.5):
        """Espera brevemente una respuesta 'ok' de Marlin antes de continuar.
        Evita saturar el buffer serie si se pulsan botones de jog muy rápido
        (el bug original no esperaba nunca confirmación)."""
        if not self.ser:
            return
        fin = time.time() + timeout
        try:
            while time.time() < fin:
                if self.ser.in_waiting > 0:
                    linea = self.ser.readline().decode('utf-8', errors='ignore')
                    if 'ok' in linea.lower():
                        return
                else:
                    time.sleep(0.005)
        except Exception as e:
            log.debug(f"Error esperando 'ok': {e}")

    # --- AUTOENFOQUE (barrido de grueso a fino en 3 pasadas) ---
    def alternar_autoenfoque(self):
        if self.autoenfoque_activo:
            self._finalizar_autoenfoque(cancelado=True)
        else:
            self._iniciar_autoenfoque()

    def _iniciar_autoenfoque(self):
        if not self.ser:
            messagebox.showwarning("Autoenfoque", "No hay conexión con la impresora.")
            return
        self.autoenfoque_activo = True
        self._af_pasada_idx = 0
        self.btn_autofocus.config(text="⏹ DETENER", bg="#C62828")
        log.info("Autoenfoque iniciado (barrido grueso → fino, 3 pasadas).")
        self._af_iniciar_pasada()

    def _af_iniciar_pasada(self):
        if not self.autoenfoque_activo:
            return
        rango, paso = self._af_pasadas[self._af_pasada_idx]
        self._af_paso_actual = paso
        self._af_pasos_totales = max(1, int(round(rango / paso)))
        self._af_paso_index = 0
        self._af_offset_actual = 0.0
        self._af_mejor_offset = 0.0
        self._af_mejor_nitidez = -1.0
        log.info(f"Autoenfoque: pasada {self._af_pasada_idx + 1}/{len(self._af_pasadas)} "
                  f"(rango ±{rango/2:.3f} mm, paso {paso:.4f} mm, {self._af_pasos_totales} paradas)")
        # Nos movemos al extremo inferior del rango para empezar el barrido ahí.
        self.mover("Z", -rango / 2)
        self._af_offset_actual -= rango / 2
        self._af_after_id = self.root.after(200, self._af_medir_punto)

    def _af_medir_punto(self):
        if not self.autoenfoque_activo:
            return
        nitidez = self.nitidez_actual
        if nitidez > self._af_mejor_nitidez:
            self._af_mejor_nitidez = nitidez
            self._af_mejor_offset = self._af_offset_actual

        if self._af_paso_index >= self._af_pasos_totales:
            self._af_finalizar_pasada()
            return

        self._af_paso_index += 1
        self.mover("Z", self._af_paso_actual)
        self._af_offset_actual += self._af_paso_actual
        self._af_after_id = self.root.after(200, self._af_medir_punto)

    def _af_finalizar_pasada(self):
        # Volvemos al mejor punto encontrado durante esta pasada; ese punto
        # es el centro de la siguiente pasada, más fina.
        correccion = self._af_mejor_offset - self._af_offset_actual
        if abs(correccion) > 1e-6:
            self.mover("Z", correccion)
        log.info(f"Pasada {self._af_pasada_idx + 1} completa: mejor nitidez "
                  f"{self._af_mejor_nitidez:.1f} en offset {self._af_mejor_offset:+.4f} mm")

        self._af_pasada_idx += 1
        if self._af_pasada_idx >= len(self._af_pasadas):
            self._af_after_id = self.root.after(250, self._finalizar_autoenfoque)
        else:
            self._af_after_id = self.root.after(250, self._af_iniciar_pasada)

    def _finalizar_autoenfoque(self, cancelado=False):
        self.autoenfoque_activo = False
        if self._af_after_id is not None:
            try:
                self.root.after_cancel(self._af_after_id)
            except Exception:
                pass
            self._af_after_id = None
        self.btn_autofocus.config(text="🔍 AUTOENFOQUE", bg="#00838F")
        if cancelado:
            log.info("Autoenfoque cancelado por el usuario.")
        else:
            log.info(f"Autoenfoque finalizado. Nitidez final: {self.nitidez_actual:.1f}")

    # --- JOGGING POR TECLADO ---
    MAPA_TECLAS = {
        "Left": ("X", -1), "Right": ("X", 1),
        "Up": ("Y", 1), "Down": ("Y", -1),
        "Prior": ("Z", 1),   # Re Pág = subir
        "Next": ("Z", -1),   # Av Pág = bajar
    }

    def _tecla_abajo(self, event):
        if event.keysym not in self.MAPA_TECLAS:
            return
        # Filtro de auto-repetición de Windows: KeyPress y KeyRelease se
        # disparan repetidamente mientras se mantiene la tecla; solo
        # arrancamos el bucle de jog la primera vez.
        if event.keysym in self._teclas_activas:
            return
        self._teclas_activas.add(event.keysym)
        if self._jog_after_id is None:
            self.jog_step_actual = self.JOG_STEP_BASE
            self._bucle_jog()

    def _tecla_arriba(self, event):
        if event.keysym in self._teclas_activas:
            self._teclas_activas.discard(event.keysym)

    def _bucle_jog(self):
        if not self._teclas_activas:
            self._jog_after_id = None
            return
        for tecla in list(self._teclas_activas):
            eje, signo = self.MAPA_TECLAS[tecla]
            self.mover(eje, signo * self.jog_step_actual)
        # Factor de solape 1.3x: el paso crece mientras se mantiene la tecla,
        # para un movimiento que acelera suavemente en vez de a saltos fijos.
        self.jog_step_actual = min(self.jog_step_actual * 1.3, 20.0)
        self._jog_after_id = self.root.after(120, self._bucle_jog)

    def guardar_imagen(self):
        if self.frame_crudo_actual is not None:
            os.makedirs(CARPETA_CAPTURAS, exist_ok=True)
            nombre = os.path.join(
                CARPETA_CAPTURAS,
                f"cap_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.png")
            imagen_a_guardar = self.frame_crudo_actual
            if self.um_por_pixel:
                imagen_a_guardar = self._dibujar_barra_escala(imagen_a_guardar, self.um_por_pixel)
            cv2.imwrite(nombre, imagen_a_guardar)
            log.info(f"Captura guardada en: {os.path.abspath(nombre)}")
        else:
            log.error("No hay imagen de cámara disponible.")
            messagebox.showwarning("Sin imagen", "No hay imagen de cámara disponible para guardar.")

    # --- CALIBRACIÓN (la forma más simple: 2 clics + una distancia conocida) ---
    def abrir_calibracion(self):
        if self.frame_crudo_actual is None:
            messagebox.showwarning("Sin imagen", "Necesito una imagen de cámara para calibrar.")
            return

        raw = self.frame_crudo_actual
        h, w = raw.shape[:2]
        # Escalamos SOLO para mostrarlo en pantalla; guardamos el factor
        # para convertir los clics de vuelta a coordenadas del frame raw.
        self._calib_escala = min(900 / w, 650 / h)
        disp_w, disp_h = int(w * self._calib_escala), int(h * self._calib_escala)
        disp = cv2.resize(raw, (disp_w, disp_h))
        disp_rgb = cv2.cvtColor(disp, cv2.COLOR_BGR2RGB)
        self._calib_img_tk = ImageTk.PhotoImage(image=Image.fromarray(disp_rgb))

        self._calib_puntos = []
        self._calib_win = tk.Toplevel(self.root)
        self._calib_win.title("Calibración: haz clic en 2 puntos de distancia conocida")
        tk.Label(self._calib_win, text="Clic en el PRIMER punto y luego en el SEGUNDO",
                 fg="white", bg="#222", font=("Arial", 10, "bold")).pack(fill="x")
        self._calib_canvas = tk.Canvas(self._calib_win, width=disp_w, height=disp_h, cursor="cross")
        self._calib_canvas.pack()
        self._calib_canvas.create_image(0, 0, anchor="nw", image=self._calib_img_tk)
        self._calib_canvas.bind("<Button-1>", self._calib_clic)

    def _calib_clic(self, event):
        # Convertimos el clic (coords de pantalla) a coords del frame RAW
        x_raw = event.x / self._calib_escala
        y_raw = event.y / self._calib_escala
        self._calib_puntos.append((x_raw, y_raw))
        self._calib_canvas.create_oval(event.x - 4, event.y - 4, event.x + 4, event.y + 4,
                                        outline="#00FF41", width=2)

        if len(self._calib_puntos) == 2:
            self._calib_canvas.create_line(
                self._calib_puntos[0][0] * self._calib_escala, self._calib_puntos[0][1] * self._calib_escala,
                self._calib_puntos[1][0] * self._calib_escala, self._calib_puntos[1][1] * self._calib_escala,
                fill="#00FF41", width=2)
            self._calib_win.update()

            (x1, y1), (x2, y2) = self._calib_puntos
            dist_px = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
            self._calib_win.destroy()

            if dist_px < 1:
                messagebox.showwarning("Calibración", "Los dos puntos están demasiado juntos.")
                return

            distancia_um = simpledialog.askfloat(
                "Distancia conocida",
                "¿Cuántos micrómetros (µm) hay entre esos dos puntos?\n(1 mm = 1000 µm)",
                parent=self.root, minvalue=0.001)
            if not distancia_um:
                log.info("Calibración cancelada.")
                return

            self.um_por_pixel = distancia_um / dist_px
            self.lbl_calib.config(text=f"Calibrado: {self.um_por_pixel:.3f} µm/px", fg="#00E676")
            log.info(f"Calibración guardada: {self.um_por_pixel:.4f} µm/px ({dist_px:.1f} px = {distancia_um} µm)")

    @staticmethod
    def _dibujar_barra_escala(img, um_por_pixel):
        """Dibuja una barra de escala con una longitud 'redonda' (1,2,5,10,20,50...
        µm) elegida para ocupar aprox. el 20% del ancho de la imagen."""
        h, w = img.shape[:2]
        img = img.copy()
        objetivo_px = w * 0.20
        candidatos_um = [1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000, 10000]
        mejor_um = candidatos_um[0]
        for c in candidatos_um:
            if c / um_por_pixel <= objetivo_px:
                mejor_um = c
            else:
                break
        largo_px = int(mejor_um / um_por_pixel)
        etiqueta = f"{mejor_um} um" if mejor_um < 1000 else f"{mejor_um/1000:.0f} mm"

        x0, y0 = int(w * 0.05), int(h * 0.92)
        cv2.line(img, (x0, y0), (x0 + largo_px, y0), (255, 255, 255), 4)
        cv2.putText(img, etiqueta, (x0, y0 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        return img

    def pedir_temp(self):
        if not self.ser:
            return
        try:
            self.ser.reset_input_buffer()
            self.ser.write(b"M105\n")
            time.sleep(0.1)
            for _ in range(20):
                if self.ser.in_waiting > 0:
                    linea = self.ser.readline().decode('utf-8', errors='ignore')
                    t = re.search(r"T:([\d.]+)", linea)
                    b = re.search(r"B:([\d.]+)", linea)
                    if t and b:
                        self.lbl_t.config(text=f"HOTEND: {t.group(1)}°C")
                        self.lbl_b.config(text=f"BED: {b.group(1)}°C")
                        break
        except Exception as e:
            log.debug(f"Error leyendo temperatura: {e}")

    def cerrar(self):
        log.info("--- Iniciando apagado seguro del sistema ---")
        if self.autoenfoque_activo:
            self._finalizar_autoenfoque(cancelado=True)
        self.led_off()
        self.camara_activa = False
        if hasattr(self, 'hilo_video') and self.hilo_video.is_alive():
            self.hilo_video.join(timeout=1.0)
        if self.ser:
            try:
                self.ser.write(b"G90\n")
                self.ser.close()
            except Exception as e:
                log.debug(f"Error cerrando puerto impresora: {e}")
        if self.ser_led:
            try:
                self.ser_led.close()
            except Exception as e:
                log.debug(f"Error cerrando puerto LED: {e}")
        self.root.quit()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = PanelControlAnetPro(root)
    root.protocol("WM_DELETE_WINDOW", app.cerrar)
    root.mainloop()
