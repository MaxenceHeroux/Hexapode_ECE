"""Interface de pilotage d'une patte d'hexapode (3 Dynamixel AX-12A / AX-18F).

Se connecte a leg_server (sur la Raspberry Pi) en TCP et permet de :
  - voir la patte (vue de dessus + vue de cote) : position reelle et position cible ;
  - bouger chaque articulation (curseurs en degres) ;
  - regler chaque servo : couple ON/OFF, vitesse, limite de couple, compliance
    (marges / pentes = equivalent du PID sur les AX), punch, butees d'angle ;
  - jouer un mouvement de pas en boucle.

Lancement (PC, Python 3 avec Tkinter, rien a installer) :
    python leg_gui.py                # se connecte a 192.168.137.10:5555
    python leg_gui.py <ip> [port]

Cote Pi, lancer d'abord :  ~/Hexapode_ECE/Dyna_test/bin/leg_server
"""

import json
import math
import os
import queue
import socket
import sys
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

DEFAULT_HOST = "192.168.137.10"
DEFAULT_PORT = 5555
DEG_PER_UNIT = 300.0 / 1024.0          # AX : 0..1023 <-> 0..300 deg, 512 = milieu
POLL_PERIOD = 0.12                     # s entre deux lectures d'etat
SLOPES = [2, 4, 8, 16, 32, 64, 128]    # valeurs de pente conseillees (doc ROBOTIS)

# Bits d'erreur renvoyes par un AX (protocole 1.0) ; la plupart declenchent la mise en
# securite (couple coupe, limite de couple remise a 0) selon le registre Alarm Shutdown (18).
ERROR_BITS = {
    0x01: "tension",
    0x02: "butee d'angle",
    0x04: "surchauffe",
    0x08: "hors plage",
    0x10: "checksum",
    0x20: "surcharge",
    0x40: "instruction",
}


def safety_reason(s):
    """Texte de la mise en securite d'un servo, ou "" s'il est normal."""
    bits = [txt for b, txt in ERROR_BITS.items() if s.get("err", 0) & b]
    if bits:
        return ", ".join(bits)
    if s.get("tlimit") == 0:
        return "limite de couple a 0"
    return ""
CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "leg_gui_config.json")

# Configuration par defaut de la patte (dimensions approximatives du modele SolidWorks)
DEFAULT_CONFIG = {
    "host": DEFAULT_HOST,
    "port": DEFAULT_PORT,
    "lengths_mm": {"coxa": 50, "femur": 75, "tibia": 125},   # tibia = genou -> bout de patte
    # Courbure du tibia : angle (deg) entre l'axe du servo du genou (direction "droite" du
    # tibia) et la droite genou -> bout de patte. Le tibia du modele est cintre vers le corps.
    "tibia_bend_deg": -30,
    "joints": [
        # inverted : servo monte a l'envers (cf. Is_inverted() de mv_hard_code)
        # offset_deg : angle dessine quand le servo est a 512
        # 13 et 14 : sens verifie sur la vraie patte le 25/11/2026 (non inverses)
        {"name": "Coxa (hanche)", "id": 12, "inverted": True, "offset_deg": 0},
        {"name": "Femur",         "id": 13, "inverted": False, "offset_deg": 0},
        {"name": "Tibia",         "id": 14, "inverted": False, "offset_deg": -90},
    ],
    # Mouvement de pas : ECARTS (deg) coxa, femur, tibia ajoutes a la pose de depart
    # (la pose au moment ou on clique "Demarrer le pas")
    "gait": [
        [-15, 25, 20],   # lever la patte (en arriere)
        [15, 25, 20],    # avancer en l'air
        [15, 0, 0],      # poser
        [-15, 0, 0],     # pousser au sol (la patte recule)
    ],
}


def load_config():
    cfg = json.loads(json.dumps(DEFAULT_CONFIG))
    try:
        with open(CONFIG_FILE, encoding="utf-8") as f:
            user = json.load(f)
        cfg.update({k: v for k, v in user.items() if k in cfg})
    except (OSError, ValueError):
        pass
    return cfg


def save_config(cfg):
    try:
        with open(CONFIG_FILE, "w", encoding="utf-8") as f:
            json.dump(cfg, f, indent=2, ensure_ascii=False)
    except OSError:
        pass


def raw_to_deg(raw, inverted):
    d = (raw - 512) * DEG_PER_UNIT
    return -d if inverted else d


def deg_to_raw(deg, inverted):
    d = -deg if inverted else deg
    return int(round(512 + d / DEG_PER_UNIT))


# --------------------------------------------------------------------------- reseau
class LegClient:
    """Connexion TCP au leg_server. Un thread unique envoie les commandes en file
    et lit l'etat des servos entre deux commandes."""

    def __init__(self):
        self.sock = None
        self.rfile = None
        self.cmd_q = queue.Queue()
        self.goals = None            # [p1, p2, p3] a envoyer (derniere valeur seulement)
        self.goals_lock = threading.Lock()
        self.state = None            # dernier etat recu (dict)
        self.state_time = 0
        self.log_q = queue.Queue()   # messages pour la barre d'etat
        self.connected = False
        self._stop = threading.Event()
        self._thread = None

    def connect(self, host, port):
        self.close()
        s = socket.create_connection((host, port), timeout=3)
        s.settimeout(2)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock = s
        self.rfile = s.makefile("r", encoding="utf-8", newline="\n")
        self.connected = True
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def close(self):
        self._stop.set()
        if self._thread and self._thread is not threading.current_thread():
            self._thread.join(timeout=1)
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = None
        self.connected = False

    def send(self, line):
        """Met une commande en file (non bloquant)."""
        if self.connected:
            self.cmd_q.put(line)

    def set_goals(self, goals):
        with self.goals_lock:
            self.goals = list(goals)

    def _rpc(self, line):
        self.sock.sendall((line + "\n").encode())
        rep = self.rfile.readline()
        if not rep:
            raise ConnectionError("serveur deconnecte")
        return rep.strip()

    def _run(self):
        last_poll = 0
        while not self._stop.is_set():
            try:
                line = None
                try:
                    line = self.cmd_q.get(timeout=0.02)
                except queue.Empty:
                    pass
                if line:
                    rep = self._rpc(line)
                    if not rep.startswith("OK"):
                        self.log_q.put(f"{line} -> {rep}")
                    elif line.split()[0].upper() in ("SCAN", "ESTOP"):
                        self.log_q.put(f"{line} -> {rep}")
                    continue
                with self.goals_lock:
                    g, self.goals = self.goals, None
                if g:
                    rep = self._rpc("GOALS " + " ".join(str(int(v)) for v in g))
                    if not rep.startswith("OK"):
                        self.log_q.put("GOALS -> " + rep)
                    continue
                if time.time() - last_poll >= POLL_PERIOD:
                    last_poll = time.time()
                    rep = self._rpc("STATE")
                    if rep.startswith("OK "):
                        self.state = json.loads(rep[3:])
                        self.state_time = time.time()
            except (OSError, ConnectionError, ValueError) as e:
                self.log_q.put(f"Connexion perdue : {e}")
                self.connected = False
                break


# --------------------------------------------------------------------------- interface
class ServoPanel(ttk.LabelFrame):
    """Commande + reglages d'un servo."""

    def __init__(self, master, app, index):
        self.app = app
        self.i = index
        j = app.cfg["joints"][index]
        super().__init__(master, text=f"{j['name']}  -  ID {j['id']}", padding=6)
        self.id = j["id"]
        self.loaded = False

        # --- consigne
        self.target = tk.DoubleVar(value=0)
        self.scale = tk.Scale(self, from_=-150, to=150, orient="horizontal", resolution=1,
                              variable=self.target, length=260, label="Consigne (deg)",
                              command=lambda _v: app.on_slider())
        self.scale.grid(row=0, column=0, columnspan=4, sticky="we")

        self.torque = tk.IntVar(value=0)
        ttk.Checkbutton(self, text="Couple ON", variable=self.torque,
                        command=self.on_torque).grid(row=1, column=0, sticky="w")
        self.inv = tk.BooleanVar(value=j["inverted"])
        ttk.Checkbutton(self, text="Monte inverse", variable=self.inv,
                        command=app.on_geometry_change).grid(row=1, column=1, columnspan=2, sticky="w")

        # --- lecture
        # taille fixe (46 car. x 4 lignes) : un texte qui change ne redimensionne pas la fenetre
        self.readout = tk.Label(self, text="-", font=("Consolas", 9), justify="left", anchor="nw",
                                width=46, height=4)
        self.readout.grid(row=2, column=0, columnspan=4, sticky="w", pady=(2, 6))

        # --- reglages
        r = 3
        self.vars = {}

        def entry(label, key, row, col, width=6, tip=None):
            ttk.Label(self, text=label).grid(row=row, column=col, sticky="e")
            v = tk.StringVar()
            e = ttk.Entry(self, textvariable=v, width=width)
            e.grid(row=row, column=col + 1, sticky="w", padx=(2, 8))
            self.vars[key] = v
            return e

        entry("Vitesse", "mspeed", r, 0)
        entry("Lim. couple", "tlimit", r, 2)
        ttk.Label(self, text="(vitesse 0 = max ; couple 0..1023)", foreground="gray").grid(
            row=r + 1, column=0, columnspan=4, sticky="w")
        entry("Marge CW", "cw_margin", r + 2, 0)
        entry("Marge CCW", "ccw_margin", r + 2, 2)
        for k, row, col, lab in (("cw_slope", r + 3, 0, "Pente CW"), ("ccw_slope", r + 3, 2, "Pente CCW")):
            ttk.Label(self, text=lab).grid(row=row, column=col, sticky="e")
            v = tk.StringVar()
            ttk.Combobox(self, textvariable=v, values=SLOPES, width=5).grid(
                row=row, column=col + 1, sticky="w", padx=(2, 8))
            self.vars[k] = v
        entry("Punch", "punch", r + 4, 0)
        ttk.Label(self, text="marge = zone morte ; pente = souplesse\n(petite pente = raide) ; punch = courant mini",
                  foreground="gray").grid(row=r + 5, column=0, columnspan=4, sticky="w")
        bf = ttk.Frame(self)
        bf.grid(row=r + 6, column=0, columnspan=4, sticky="we", pady=4)
        ttk.Button(bf, text="Appliquer reglages", command=self.apply_settings).pack(side="left")
        ttk.Button(bf, text="Relire", command=self.reload).pack(side="left", padx=4)
        self.reset_btn = ttk.Button(bf, text="Reset securite", command=self.reset_safety)
        self.reset_btn.pack(side="left", padx=4)
        self.reset_btn.state(["disabled"])

        ttk.Separator(self).grid(row=r + 7, column=0, columnspan=4, sticky="we", pady=4)
        entry("Butee CW", "cw_limit", r + 8, 0)
        entry("Butee CCW", "ccw_limit", r + 8, 2)
        self.limit_lbl = ttk.Label(self, text="", foreground="gray", width=46)
        self.limit_lbl.grid(row=r + 9, column=0, columnspan=4, sticky="w")
        ttk.Button(self, text="Appliquer butees (EEPROM)", command=self.apply_limits).grid(
            row=r + 10, column=0, columnspan=4, sticky="w")

    # ----- actions
    def on_torque(self):
        self.app.client.send(f"TORQUE {self.id} {self.torque.get()}")

    def apply_settings(self):
        try:
            v = {k: int(float(self.vars[k].get())) for k in
                 ("mspeed", "tlimit", "cw_margin", "ccw_margin", "cw_slope", "ccw_slope", "punch")}
        except ValueError:
            messagebox.showerror("Reglages", "Valeur non numerique.")
            return
        c = self.app.client
        c.send(f"SPEED {self.id} {v['mspeed']}")
        c.send(f"TLIMIT {self.id} {v['tlimit']}")
        c.send(f"MARGIN {self.id} {v['cw_margin']} {v['ccw_margin']}")
        c.send(f"SLOPE {self.id} {v['cw_slope']} {v['ccw_slope']}")
        c.send(f"PUNCH {self.id} {v['punch']}")
        self.app.status(f"ID {self.id} : reglages envoyes")

    def apply_limits(self):
        try:
            cw, ccw = int(self.vars["cw_limit"].get()), int(self.vars["ccw_limit"].get())
        except ValueError:
            messagebox.showerror("Butees", "Valeur non numerique.")
            return
        if not 0 <= cw < ccw <= 1023:
            messagebox.showerror("Butees", "Il faut 0 <= CW < CCW <= 1023.")
            return
        if messagebox.askyesno("Butees", f"Ecrire les butees {cw} / {ccw} dans l'EEPROM du servo {self.id} ?\n"
                                         "(memorise meme apres coupure)"):
            self.app.client.send(f"LIMITS {self.id} {cw} {ccw}")

    def reload(self):
        self.loaded = False

    def reset_safety(self):
        self.app.reset_safety(self.id)

    # ----- mise a jour depuis l'etat
    def update_state(self, s):
        if not s or not s.get("ok"):
            self.readout.config(text="ne repond pas", foreground="red")
            return
        inv = self.inv.get()
        deg = raw_to_deg(s["pos"], inv)
        load = s["load"] * 100 / 1023
        txt = (f"pos {s['pos']:4d} ({deg:+6.1f} deg)  cible {s['goal']:4d}\n"
               f"charge {load:+5.0f} %  {s['temp']:2d} degC  {s['volt'] / 10:.1f} V"
               f"{'  en mouvement' if s['moving'] else ''}")
        color = "black"
        if s["temp"] >= 65 or abs(load) > 80:
            color = "red"
        safety = safety_reason(s)
        if safety:
            color = "red"
            txt += "\nSECURITE : " + safety + " -> Reset"
        self.readout.config(text=txt, foreground=color)
        self.reset_btn.state(["!disabled"] if safety else ["disabled"])
        self.torque.set(s["torque"])
        lo, hi = sorted((raw_to_deg(s["cw_limit"], inv), raw_to_deg(s["ccw_limit"], inv)))
        self.limit_lbl.config(text=f"butees actuelles : {s['cw_limit']} / {s['ccw_limit']}"
                                   f"  ({lo:+.0f} .. {hi:+.0f} deg)")
        if not self.loaded:
            for k in self.vars:
                if k in s:
                    self.vars[k].set(str(s[k]))
            self.loaded = True


class LegApp(tk.Tk):
    def __init__(self, host=None, port=None):
        super().__init__()
        self.title("Patte hexapode - Dynamixel 12 / 13 / 14")
        self.cfg = load_config()
        if host:
            self.cfg["host"] = host
        if port:
            self.cfg["port"] = int(port)
        self.client = LegClient()
        self.initialized = False
        self.gait_job = None
        self.gait_index = 0
        self.gait_base = [0, 0, 0]
        self._build()
        # Fige la taille de la fenetre : un changement de texte ne doit jamais la
        # redimensionner (sinon les curseurs bougent sous la souris).
        self.update_idletasks()
        self.geometry(f"{self.winfo_reqwidth()}x{self.winfo_reqheight()}")
        self.minsize(self.winfo_reqwidth(), self.winfo_reqheight())
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.after(100, self.tick)

    # ------------------------------------------------------------ construction
    def _build(self):
        top = ttk.Frame(self, padding=6)
        top.pack(fill="x")
        ttk.Label(top, text="Pi :").pack(side="left")
        self.host = tk.StringVar(value=self.cfg["host"])
        ttk.Entry(top, textvariable=self.host, width=16).pack(side="left")
        self.port = tk.StringVar(value=str(self.cfg["port"]))
        ttk.Entry(top, textvariable=self.port, width=6).pack(side="left", padx=2)
        ttk.Button(top, text="Connecter", command=self.connect).pack(side="left", padx=4)
        self.conn_lbl = ttk.Label(top, text="deconnecte", foreground="red", width=24)
        self.conn_lbl.pack(side="left", padx=6)
        tk.Button(top, text="ARRET COUPLE", bg="#c62828", fg="white", font=("Segoe UI", 10, "bold"),
                  command=self.estop).pack(side="right")
        tk.Button(top, text="RESET SECURITE (3)", bg="#ef6c00", fg="white", font=("Segoe UI", 10, "bold"),
                  command=lambda: self.reset_safety("ALL")).pack(side="right", padx=4)
        ttk.Button(top, text="Couple ON (3)", command=lambda: self.torque_all(1)).pack(side="right", padx=4)
        ttk.Button(top, text="Scanner", command=lambda: self.client.send("SCAN")).pack(side="right")

        mid = ttk.Frame(self, padding=(6, 0))
        mid.pack(fill="both", expand=True)

        # --- dessins
        draw = ttk.Frame(mid)
        draw.pack(side="left", fill="y")
        ttk.Label(draw, text="Vue de dessus (coxa)").pack(anchor="w")
        self.top_cv = tk.Canvas(draw, width=340, height=230, bg="white", highlightthickness=1,
                                highlightbackground="#ccc")
        self.top_cv.pack()
        ttk.Label(draw, text="Vue de cote (femur + tibia)").pack(anchor="w", pady=(6, 0))
        self.side_cv = tk.Canvas(draw, width=340, height=300, bg="white", highlightthickness=1,
                                 highlightbackground="#ccc")
        self.side_cv.pack()
        ttk.Label(draw, text="trait plein = position reelle, pointilles = consigne",
                  foreground="gray").pack(anchor="w")

        # --- geometrie
        geo = ttk.LabelFrame(draw, text="Geometrie (mm) et decalages (deg a 512)", padding=4)
        geo.pack(fill="x", pady=6)
        self.len_vars = {}
        for c, key in enumerate(("coxa", "femur", "tibia")):
            ttk.Label(geo, text=key).grid(row=0, column=2 * c, sticky="e")
            v = tk.StringVar(value=str(self.cfg["lengths_mm"][key]))
            ttk.Entry(geo, textvariable=v, width=5).grid(row=0, column=2 * c + 1, padx=(2, 6))
            self.len_vars[key] = v
        self.off_vars = []
        for c, j in enumerate(self.cfg["joints"]):
            ttk.Label(geo, text=f"dec. {j['id']}").grid(row=1, column=2 * c, sticky="e")
            v = tk.StringVar(value=str(j["offset_deg"]))
            ttk.Entry(geo, textvariable=v, width=5).grid(row=1, column=2 * c + 1, padx=(2, 6))
            self.off_vars.append(v)
        ttk.Label(geo, text="courbure tibia (deg)").grid(row=2, column=0, columnspan=3, sticky="e")
        self.bend_var = tk.StringVar(value=str(self.cfg["tibia_bend_deg"]))
        ttk.Entry(geo, textvariable=self.bend_var, width=5).grid(row=2, column=3, padx=(2, 6), sticky="w")
        ttk.Label(geo, text="axe genou -> bout de patte", foreground="gray").grid(
            row=2, column=4, columnspan=2, sticky="w")
        ttk.Button(geo, text="Appliquer / sauver", command=self.on_geometry_change).grid(
            row=3, column=0, columnspan=6, sticky="w", pady=(4, 0))

        # --- servos
        servos = ttk.Frame(mid)
        servos.pack(side="left", fill="both", expand=True, padx=(8, 0))
        self.panels = []
        for i in range(3):
            p = ServoPanel(servos, self, i)
            p.grid(row=0, column=i, sticky="n", padx=3)
            self.panels.append(p)

        # --- mouvements
        mv = ttk.LabelFrame(servos, text="Mouvements", padding=6)
        mv.grid(row=1, column=0, columnspan=3, sticky="we", pady=6)
        ttk.Button(mv, text="Position neutre (512)", command=self.neutral).grid(row=0, column=0, padx=2)
        ttk.Button(mv, text="Consigne = position actuelle", command=self.copy_present).grid(
            row=0, column=1, padx=2)
        self.gait_btn = ttk.Button(mv, text="Demarrer le pas", command=self.toggle_gait)
        self.gait_btn.grid(row=0, column=2, padx=10)
        ttk.Label(mv, text="Amplitude %").grid(row=0, column=3)
        self.amp = tk.Scale(mv, from_=10, to=150, orient="horizontal", length=120)
        self.amp.set(100)
        self.amp.grid(row=0, column=4)
        ttk.Label(mv, text="Duree etape (ms)").grid(row=0, column=5)
        self.step_ms = tk.Scale(mv, from_=150, to=2000, resolution=50, orient="horizontal", length=120)
        self.step_ms.set(600)
        self.step_ms.grid(row=0, column=6)
        ttk.Label(mv, text="Pas = lever, avancer en l'air, poser, pousser au sol, autour de la pose de "
                           "depart (ecarts dans leg_gui_config.json, cle \"gait\")", foreground="gray").grid(
            row=1, column=0, columnspan=7, sticky="w")

        self.status_var = tk.StringVar(value="Lancer leg_server sur la Pi, puis Connecter.")
        # width=1 : la barre d'etat prend la largeur de la fenetre sans jamais l'agrandir
        ttk.Label(self, textvariable=self.status_var, relief="sunken", anchor="w",
                  padding=3, width=1).pack(fill="x", side="bottom")

    # ------------------------------------------------------------ helpers
    def status(self, msg):
        self.status_var.set(time.strftime("%H:%M:%S  ") + msg)

    def joint_cfg(self, i):
        return self.cfg["joints"][i]

    def servo_state(self, i):
        st = self.client.state
        if not st:
            return None
        sid = self.joint_cfg(i)["id"]
        for s in st.get("servos", []):
            if s.get("id") == sid:
                return s
        return None

    def connect(self):
        try:
            self.client.connect(self.host.get().strip(), int(self.port.get()))
        except (OSError, ValueError) as e:
            self.status(f"Connexion impossible : {e}  (leg_server lance sur la Pi ?)")
            return
        self.cfg["host"], self.cfg["port"] = self.host.get().strip(), int(self.port.get())
        save_config(self.cfg)
        self.initialized = False
        for p in self.panels:
            p.loaded = False
        self.client.send("SCAN")
        self.status("Connecte")

    def reset_safety(self, who):
        """Sort les servos du mode securite (cf. RESET dans leg_server)."""
        self.stop_gait()
        self.client.send(f"RESET {who}")
        self.status(f"Reset securite envoye ({who}) : consigne = position actuelle, couple ON")

        def resync():
            # curseurs et reglages realignes sur l'etat reel apres le reset
            for i, p in enumerate(self.panels):
                if who == "ALL" or p.id == who:
                    s = self.servo_state(i)
                    if s and s.get("ok"):
                        p.target.set(round(raw_to_deg(s["pos"], p.inv.get())))
                    p.loaded = False
        self.after(500, resync)

    def estop(self):
        self.stop_gait()
        self.client.send("ESTOP")
        self.status("ARRET COUPLE envoye")

    def torque_all(self, on):
        for p in self.panels:
            self.client.send(f"TORQUE {p.id} {on}")

    def targets_raw(self):
        """Consignes des curseurs converties en positions brutes, limitees aux butees."""
        out = []
        for i, p in enumerate(self.panels):
            raw = deg_to_raw(p.target.get(), p.inv.get())
            s = self.servo_state(i)
            lo, hi = (s["cw_limit"], s["ccw_limit"]) if s and s.get("ok") else (0, 1023)
            out.append(max(lo, min(hi, raw)))
        return out

    def on_slider(self):
        if self.initialized:
            self.client.set_goals(self.targets_raw())

    def set_targets_deg(self, degs):
        for p, d in zip(self.panels, degs):
            p.target.set(max(-150, min(150, d)))
        self.on_slider()

    def neutral(self):
        if not messagebox.askyesno("Position neutre",
                                   "Envoyer les 3 servos a 512 (milieu) ?\n"
                                   "Le mouvement peut etre grand si la patte est repliee."):
            return
        self.stop_gait()
        self.set_targets_deg([0, 0, 0])

    def copy_present(self):
        for i, p in enumerate(self.panels):
            s = self.servo_state(i)
            if s and s.get("ok"):
                p.target.set(round(raw_to_deg(s["pos"], p.inv.get())))

    def on_geometry_change(self):
        try:
            for k, v in self.len_vars.items():
                self.cfg["lengths_mm"][k] = float(v.get())
            self.cfg["tibia_bend_deg"] = float(self.bend_var.get())
            for j, v, p in zip(self.cfg["joints"], self.off_vars, self.panels):
                j["offset_deg"] = float(v.get())
                j["inverted"] = bool(p.inv.get())
        except ValueError:
            messagebox.showerror("Geometrie", "Valeur non numerique.")
            return
        save_config(self.cfg)
        self.status("Geometrie sauvegardee dans leg_gui_config.json")

    # ------------------------------------------------------------ mouvement de pas
    def toggle_gait(self):
        if self.gait_job:
            self.stop_gait()
        else:
            if not self.initialized:
                self.status("Pas impossible : pas encore connecte")
                return
            self.gait_index = 0
            self.gait_base = [p.target.get() for p in self.panels]   # pose de depart
            self.gait_btn.config(text="Arreter le pas")
            self.gait_step()

    def stop_gait(self):
        if self.gait_job:
            self.after_cancel(self.gait_job)
            self.set_targets_deg(self.gait_base)   # retour a la pose de depart
        self.gait_job = None
        self.gait_btn.config(text="Demarrer le pas")

    def gait_step(self):
        poses = self.cfg["gait"]
        k = self.amp.get() / 100.0
        pose = poses[self.gait_index % len(poses)]
        self.set_targets_deg([b + a * k for b, a in zip(self.gait_base, pose)])
        self.gait_index += 1
        self.gait_job = self.after(int(self.step_ms.get()), self.gait_step)

    # ------------------------------------------------------------ dessin
    def joint_angles(self, source):
        """Angles dessines (deg) des 3 articulations : 'pos' (reel) ou 'target' (consigne)."""
        out = []
        for i, p in enumerate(self.panels):
            off = self.cfg["joints"][i]["offset_deg"]
            if source == "target":
                a = p.target.get()
            else:
                s = self.servo_state(i)
                if not s or not s.get("ok"):
                    return None
                a = raw_to_deg(s["pos"], p.inv.get())
            out.append(a + off)
        return out

    def draw(self):
        L = self.cfg["lengths_mm"]
        lc, lf, lt = L["coxa"], L["femur"], L["tibia"]
        reach = lc + lf + lt
        for cv in (self.top_cv, self.side_cv):
            cv.delete("all")

        # --- vue de dessus
        cv = self.top_cv
        w, h = 340, 230
        sc = (w - 60) / reach
        ox, oy = 30, h / 2
        cv.create_rectangle(0, oy - 60, ox, oy + 60, fill="#333", outline="")
        cv.create_text(ox + 4, 12, text="corps", anchor="w", fill="#777")
        cv.create_text(w - 6, h - 10, text=f"echelle : {reach:.0f} mm", anchor="e", fill="#999")

        def top(angles, style):
            g = self.leg_geometry(angles)
            c = math.radians(angles[0])

            def to_xy(r):
                return ox + r * sc * math.cos(c), oy - r * sc * math.sin(c)
            pts = [to_xy(r) for r in (0, lc, g["knee"][0], g["foot"][0])]
            self._poly(cv, pts, style, ctrl=to_xy(g["ctrl"][0]))

        # --- vue de cote
        sv = self.side_cv
        w2, h2 = 340, 300
        sc2 = (w2 - 50) / reach
        sx, sy = 25, 110
        sv.create_rectangle(0, sy - 25, sx, sy + 25, fill="#333", outline="")
        sv.create_line(0, sy + (lt * 0.85) * sc2, w2, sy + (lt * 0.85) * sc2, fill="#bbb", dash=(2, 4))
        sv.create_text(w2 - 6, sy + (lt * 0.85) * sc2 - 8, text="sol (indicatif)", anchor="e", fill="#aaa")

        def side(angles, style):
            g = self.leg_geometry(angles)

            def to_xy(rz):
                return sx + rz[0] * sc2, sy - rz[1] * sc2
            pts = [(sx, sy), to_xy((lc, 0)), to_xy(g["knee"]), to_xy(g["foot"])]
            self._poly(sv, pts, style, ctrl=to_xy(g["ctrl"]))
            if style == "real" or not real:
                r, z = g["foot"]
                sv.create_text(6, 10, anchor="w", fill="#333",
                               text=f"bout de patte : {r:.0f} mm de l'axe coxa, "
                                    f"z = {z:+.0f} mm" + ("" if style == "real" else " (consigne)"))

        tgt = self.joint_angles("target")
        real = self.joint_angles("pos")
        top(tgt, "ghost")
        side(tgt, "ghost")
        if real:
            top(real, "real")
            side(real, "real")

    def leg_geometry(self, angles):
        """Positions dans le plan de la patte (r = distance a l'axe de la coxa, z = hauteur,
        en mm) du genou, du bout de patte et du point de controle de la courbe du tibia.
        Le tibia est cintre : le bout de patte est a tibia_bend_deg de l'axe du genou."""
        L = self.cfg["lengths_mm"]
        lc, lf, lt = L["coxa"], L["femur"], L["tibia"]
        _, f, t = (math.radians(a) for a in angles)
        axis = f + t                                          # axe du servo du genou
        chord = axis + math.radians(self.cfg.get("tibia_bend_deg", 0))
        knee = (lc + lf * math.cos(f), lf * math.sin(f))
        foot = (knee[0] + lt * math.cos(chord), knee[1] + lt * math.sin(chord))
        # point de controle : depart de la courbe tangent a l'axe du genou
        ctrl = (knee[0] + 0.55 * lt * math.cos(axis), knee[1] + 0.55 * lt * math.sin(axis))
        return {"knee": knee, "foot": foot, "ctrl": ctrl}

    def _poly(self, cv, pts, style, ctrl=None):
        """Dessine la patte ; le dernier segment (tibia) est courbe via ctrl s'il est donne."""
        colors = ["#1565c0", "#2e7d32", "#ef6c00"]
        for k in range(len(pts) - 1):
            (x0, y0), (x1, y1) = pts[k], pts[k + 1]
            coords = (x0, y0, x1, y1)
            if ctrl and k == len(pts) - 2:
                coords = (x0, y0, ctrl[0], ctrl[1], x1, y1)
            if style == "ghost":
                cv.create_line(*coords, fill="#9e9e9e", width=2, dash=(4, 3), smooth=True)
            else:
                cv.create_line(*coords, fill=colors[k % 3], width=6, capstyle="round", smooth=True)
        if style == "real":
            for k, (x, y) in enumerate(pts[1:-1]):
                cv.create_oval(x - 6, y - 6, x + 6, y + 6, fill="white", outline="#333", width=2)
                cv.create_text(x, y - 13, text=str(self.cfg["joints"][k + 1]["id"]), fill="#333")
            x, y = pts[0]
            cv.create_oval(x - 6, y - 6, x + 6, y + 6, fill="white", outline="#333", width=2)
            x, y = pts[-1]
            cv.create_oval(x - 4, y - 4, x + 4, y + 4, fill="#333")

    # ------------------------------------------------------------ boucle d'affichage
    def tick(self):
        while True:
            try:
                self.status(self.client.log_q.get_nowait())
            except queue.Empty:
                break
        st = self.client.state
        if st:
            if st.get("port_error"):
                self.status("U2D2 : " + st["port_error"])
            rc = st.get("reconnects", 0)
            if rc != getattr(self, "last_reconnects", rc):
                self.status(f"U2D2 reconnecte ({st.get('port', '?')}), reconnexion n{rc} : "
                            "verifier masse / cable USB / alimentation des servos")
            self.last_reconnects = rc
        if self.client.connected and st and time.time() - self.client.state_time < 1.5:
            self.conn_lbl.config(text="connecte", foreground="green")
            if not self.initialized:
                # premiere lecture : curseurs = position actuelle (pas de saut au demarrage)
                self.copy_present()
                self.initialized = True
            for i, p in enumerate(self.panels):
                p.update_state(self.servo_state(i))
        elif self.client.connected:
            self.conn_lbl.config(text="connecte (attente etat)", foreground="orange")
        else:
            self.conn_lbl.config(text="deconnecte", foreground="red")
            self.initialized = False
            self.stop_gait()
        self.draw()
        self.after(100, self.tick)

    def on_close(self):
        self.stop_gait()
        self.client.close()
        self.destroy()


if __name__ == "__main__":
    app = LegApp(*(sys.argv[1:3]))
    app.mainloop()
