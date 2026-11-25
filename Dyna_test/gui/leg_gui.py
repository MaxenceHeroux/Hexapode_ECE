"""Interface de pilotage de l'hexapode (6 pattes x 3 Dynamixel AX-12A / AX-18F).

Se connecte a leg_server (sur la Raspberry Pi) en TCP et permet de :
  - voir le robot de dessus avec ses 6 pattes (position reelle) ; clic = choisir une patte ;
  - configurer chaque patte : nom, IDs coxa / femur / tibia, position et angle de montage
    sur le corps, sens et offset de chaque servo ;
  - piloter la patte choisie : curseurs par articulation ou cinematique inverse (position
    du bout de patte), vue de cote, butees en rouge ;
  - regler chaque servo : couple, vitesse, limite de couple, compliance (marges / pentes =
    equivalent du PID sur les AX), punch, butees ; reset du mode securite ;
  - jouer un mouvement de pas sur la patte choisie.

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

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hexapod_model as hm  # noqa: E402
from hexapod_model import deg_to_raw, load_config, raw_to_deg, save_config  # noqa: E402

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


# --------------------------------------------------------------------------- reseau
class LegClient:
    """Connexion TCP au leg_server. Un thread unique envoie les commandes en file
    et lit l'etat des servos entre deux commandes."""

    def __init__(self):
        self.sock = None
        self.rfile = None
        self.cmd_q = queue.Queue()
        self.goals = {}              # id -> position a envoyer (derniere valeur seulement)
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
        s.settimeout(3)
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
        """goals : dict id -> position brute."""
        with self.goals_lock:
            self.goals.update(goals)

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
                    elif line.split()[0].upper() in ("SCAN", "IDS", "ESTOP"):
                        self.log_q.put(f"{line.split()[0]} -> {rep}")
                    continue
                with self.goals_lock:
                    g, self.goals = self.goals, {}
                if g:
                    rep = self._rpc("GOALSID " + " ".join(f"{i} {int(v)}" for i, v in g.items()))
                    if not rep.startswith("OK"):
                        self.log_q.put("GOALSID -> " + rep)
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
    """Commande + reglages d'un servo de la patte selectionnee (k = 0 coxa, 1 femur, 2 tibia)."""

    def __init__(self, master, app, k):
        self.app = app
        self.i = k
        super().__init__(master, text="", padding=6)
        self.loaded = False

        # --- consigne
        self.target = tk.DoubleVar(value=0)       # angle de l'ARTICULATION (offset inclus)
        self.scale = tk.Scale(self, from_=-150, to=150, orient="horizontal", resolution=1,
                              variable=self.target, length=260,
                              label="Consigne articulation (deg)",
                              command=lambda _v: app.on_slider())
        self.scale.grid(row=0, column=0, columnspan=4, sticky="we")

        self.torque = tk.IntVar(value=0)
        ttk.Checkbutton(self, text="Couple ON", variable=self.torque,
                        command=self.on_torque).grid(row=1, column=0, sticky="w")
        self.inv = tk.BooleanVar(value=False)
        ttk.Checkbutton(self, text="Monte inverse", variable=self.inv,
                        command=self.on_inverted).grid(row=1, column=1, columnspan=2, sticky="w")

        # --- offset (zero de l'articulation) : ne fait jamais bouger le servo
        of = ttk.Frame(self)
        of.grid(row=2, column=0, columnspan=4, sticky="w", pady=(2, 0))
        ttk.Label(of, text="Offset (deg)").pack(side="left")
        self.off_var = tk.StringVar(value="0")
        oe = ttk.Entry(of, textvariable=self.off_var, width=6)
        oe.pack(side="left", padx=(2, 4))
        oe.bind("<Return>", lambda _e: self.apply_offset())
        for txt, delta in (("-90", -90), ("-1", -1), ("+1", 1), ("+90", 90)):
            ttk.Button(of, text=txt, width=3,
                       command=lambda d=delta: self.shift_offset(d)).pack(side="left")
        ttk.Button(of, text="Zero ici", command=self.zero_here).pack(side="left", padx=(4, 0))

        # --- lecture (taille fixe : un texte qui change ne redimensionne pas la fenetre)
        self.readout = tk.Label(self, text="-", font=("Consolas", 9), justify="left", anchor="nw",
                                width=46, height=4)
        self.readout.grid(row=3, column=0, columnspan=4, sticky="w", pady=(2, 6))

        # --- reglages
        r = 4
        self.vars = {}

        def entry(label, key, row, col, width=6):
            ttk.Label(self, text=label).grid(row=row, column=col, sticky="e")
            v = tk.StringVar()
            ttk.Entry(self, textvariable=v, width=width).grid(row=row, column=col + 1, sticky="w",
                                                              padx=(2, 8))
            self.vars[key] = v

        entry("Vitesse", "mspeed", r, 0)
        entry("Lim. couple", "tlimit", r, 2)
        ttk.Label(self, text="(vitesse 0 = max ; couple 0..1023)", foreground="gray").grid(
            row=r + 1, column=0, columnspan=4, sticky="w")
        entry("Marge CW", "cw_margin", r + 2, 0)
        entry("Marge CCW", "ccw_margin", r + 2, 2)
        for key, row, col, lab in (("cw_slope", r + 3, 0, "Pente CW"), ("ccw_slope", r + 3, 2, "Pente CCW")):
            ttk.Label(self, text=lab).grid(row=row, column=col, sticky="e")
            v = tk.StringVar()
            ttk.Combobox(self, textvariable=v, values=SLOPES, width=5).grid(
                row=row, column=col + 1, sticky="w", padx=(2, 8))
            self.vars[key] = v
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
        self.rebind()

    # ----- liaison a la patte selectionnee
    @property
    def leg(self):
        return self.app.leg()

    @property
    def id(self):
        return self.leg["ids"][self.i]

    def rebind(self):
        """(Re)lie le panneau au servo k de la patte selectionnee."""
        self.config(text=f"{hm.JOINT_NAMES[self.i]}  -  ID {self.id}")
        self.inv.set(bool(self.leg["inverted"][self.i]))
        self.off_var.set(f"{self.off():g}")
        self.set_range()
        self.loaded = False

    # ----- actions
    def on_torque(self):
        self.app.client.send(f"TORQUE {self.id} {self.torque.get()}")

    def on_inverted(self):
        # garde la meme position servo pour la consigne
        raw = self.to_raw(self.target.get())
        self.leg["inverted"][self.i] = bool(self.inv.get())
        self.set_range()
        self.target.set(round(self.to_deg(raw)))
        save_config(self.app.cfg)

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

    # ----- conversions servo <-> articulation (inversion + offset du montage)
    def off(self):
        return float(self.leg["offset_deg"][self.i])

    def to_deg(self, raw):
        return raw_to_deg(raw, self.inv.get()) + self.off()

    def to_raw(self, deg):
        return deg_to_raw(deg - self.off(), self.inv.get())

    # ----- offset
    def apply_offset(self):
        try:
            self.app.set_offset(self.i, float(self.off_var.get()))
        except ValueError:
            messagebox.showerror("Offset", "Valeur non numerique.")

    def shift_offset(self, delta):
        self.app.set_offset(self.i, self.off() + delta)

    def zero_here(self):
        """Offset tel que la position ACTUELLE du servo s'affiche 0 deg."""
        s = self.app.state_of(self.id)
        if not s or not s.get("ok"):
            messagebox.showerror("Zero ici", "Position du servo inconnue (non connecte ?).")
            return
        self.app.set_offset(self.i, round(-raw_to_deg(s["pos"], self.inv.get()), 1))

    def set_range(self):
        """Course du curseur = course du servo (0..1023) exprimee en angle d'articulation."""
        a, b = self.to_deg(0), self.to_deg(1023)
        self.scale.config(from_=round(min(a, b)), to=round(max(a, b)))

    def reset_safety(self):
        self.app.reset_safety(self.id)

    # ----- mise a jour depuis l'etat
    def update_state(self, s):
        if not s or not s.get("ok"):
            self.readout.config(text="ne repond pas" + (f" ({s.get('msg')})" if s else ""),
                                foreground="red")
            self.reset_btn.state(["disabled"])
            return
        deg = self.to_deg(s["pos"])
        # couple OFF : le registre de charge des AX garde une valeur figee -> non affichee
        load = s["load"] * 100 / 1023 if s["torque"] else 0
        ltxt = f"charge {load:+5.0f} %" if s["torque"] else "charge  -- (OFF)"
        txt = (f"pos {s['pos']:4d} ({deg:+6.1f} deg)  cible {s['goal']:4d}\n"
               f"{ltxt}  {s['temp']:2d} degC  {s['volt'] / 10:.1f} V"
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
        lo, hi = sorted((self.to_deg(s["cw_limit"]), self.to_deg(s["ccw_limit"])))
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
        self.title("Hexapode - pattes Dynamixel")
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
        self.robot_view = {}          # infos de mise a l'echelle de la vue robot (clic)
        self._build()
        # Fige la taille de la fenetre : un changement de texte ne doit jamais la
        # redimensionner (sinon les curseurs bougent sous la souris).
        self.update_idletasks()
        self.geometry(f"{self.winfo_reqwidth()}x{self.winfo_reqheight()}")
        self.minsize(self.winfo_reqwidth(), self.winfo_reqheight())
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.after(100, self.tick)

    # ------------------------------------------------------------ patte selectionnee
    def leg(self, k=None):
        return self.cfg["legs"][self.cfg["selected_leg"] if k is None else k]

    def leg_label(self, k):
        return f"{k + 1} - {self.cfg['legs'][k]['name']}"

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
        tk.Button(top, text="ARRET COUPLE (tous)", bg="#c62828", fg="white", font=("Segoe UI", 10, "bold"),
                  command=self.estop).pack(side="right")
        tk.Button(top, text="RESET SECURITE (tous)", bg="#ef6c00", fg="white", font=("Segoe UI", 10, "bold"),
                  command=lambda: self.reset_safety("ALL")).pack(side="right", padx=4)
        ttk.Button(top, text="Couple ON (patte)", command=lambda: self.torque_leg(1)).pack(side="right", padx=4)
        ttk.Button(top, text="Scanner", command=lambda: self.client.send("SCAN")).pack(side="right")

        mid = ttk.Frame(self, padding=(6, 0))
        mid.pack(fill="both", expand=True)

        # --- dessins
        draw = ttk.Frame(mid)
        draw.pack(side="left", fill="y")
        ttk.Label(draw, text="Robot vu de dessus (clic sur une patte = la choisir)").pack(anchor="w")
        self.robot_cv = tk.Canvas(draw, width=340, height=270, bg="white", highlightthickness=1,
                                  highlightbackground="#ccc")
        self.robot_cv.pack()
        self.robot_cv.bind("<Button-1>", self.on_robot_click)
        self.side_title = ttk.Label(draw, text="")
        self.side_title.pack(anchor="w", pady=(6, 0))
        self.side_cv = tk.Canvas(draw, width=340, height=250, bg="white", highlightthickness=1,
                                 highlightbackground="#ccc")
        self.side_cv.pack()
        lg = ttk.Frame(draw)
        lg.pack(fill="x")
        ttk.Label(lg, text="plein = reel, pointilles = consigne", foreground="gray").pack(side="left")
        self.show_limits = tk.BooleanVar(value=bool(self.cfg.get("show_limits", True)))
        ttk.Checkbutton(lg, text="Afficher butees (rouge)", variable=self.show_limits,
                        command=self.on_show_limits).pack(side="right")

        # --- geometrie (commune aux 6 pattes)
        geo = ttk.LabelFrame(draw, text="Geometrie des pattes (mm, deg)", padding=4)
        geo.pack(fill="x", pady=6)
        self.len_vars = {}
        for c, key in enumerate(("coxa", "femur", "tibia")):
            ttk.Label(geo, text=key).grid(row=0, column=2 * c, sticky="e")
            v = tk.StringVar(value=f"{self.cfg['lengths_mm'][key]:g}")
            ttk.Entry(geo, textvariable=v, width=5).grid(row=0, column=2 * c + 1, padx=(2, 6))
            self.len_vars[key] = v
        ttk.Label(geo, text="courbure tibia").grid(row=1, column=0, columnspan=2, sticky="e")
        self.bend_var = tk.StringVar(value=f"{self.cfg['tibia_bend_deg']:g}")
        ttk.Entry(geo, textvariable=self.bend_var, width=5).grid(row=1, column=2, padx=(2, 6), sticky="w")
        fo = self.cfg["foot_offset_mm"]
        ttk.Label(geo, text="bout : le long").grid(row=1, column=3, sticky="e")
        self.foot_along = tk.StringVar(value=f"{fo['along']:g}")
        ttk.Entry(geo, textvariable=self.foot_along, width=5).grid(row=1, column=4, padx=(2, 6), sticky="w")
        ttk.Label(geo, text="travers").grid(row=2, column=3, sticky="e")
        self.foot_perp = tk.StringVar(value=f"{fo['perp']:g}")
        ttk.Entry(geo, textvariable=self.foot_perp, width=5).grid(row=2, column=4, padx=(2, 6), sticky="w")
        ttk.Button(geo, text="Appliquer / sauver", command=self.on_geometry_change).grid(
            row=2, column=0, columnspan=3, sticky="w", pady=(4, 0))

        # --- servos de la patte selectionnee
        servos = ttk.Frame(mid)
        servos.pack(side="left", fill="both", expand=True, padx=(8, 0))
        self.panels = []
        for k in range(3):
            p = ServoPanel(servos, self, k)
            p.grid(row=0, column=k, sticky="n", padx=3)
            self.panels.append(p)

        # --- mouvements
        mv = ttk.LabelFrame(servos, text="Mouvements (patte selectionnee)", padding=6)
        mv.grid(row=1, column=0, columnspan=3, sticky="we", pady=(6, 0))
        ttk.Button(mv, text="Position neutre (512)", command=self.neutral).grid(row=0, column=0, padx=2)
        ttk.Button(mv, text="Consigne = position actuelle", command=self.copy_present).grid(
            row=0, column=1, padx=2)
        self.gait_btn = ttk.Button(mv, text="Demarrer le pas", command=self.toggle_gait)
        self.gait_btn.grid(row=0, column=2, padx=10)
        ttk.Label(mv, text="Amplitude %").grid(row=0, column=3)
        self.amp = tk.Scale(mv, from_=10, to=150, orient="horizontal", length=110)
        self.amp.set(100)
        self.amp.grid(row=0, column=4)
        ttk.Label(mv, text="Duree etape (ms)").grid(row=0, column=5)
        self.step_ms = tk.Scale(mv, from_=150, to=2000, resolution=50, orient="horizontal", length=110)
        self.step_ms.set(600)
        self.step_ms.grid(row=0, column=6)

        bottom = ttk.Frame(servos)
        bottom.grid(row=2, column=0, columnspan=3, sticky="we", pady=6)

        # --- configuration des pattes
        lc = ttk.LabelFrame(bottom, text="Configuration des pattes", padding=6)
        lc.pack(side="left", fill="y")
        ttk.Label(lc, text="Patte").grid(row=0, column=0, sticky="e")
        self.leg_choice = tk.StringVar()
        self.leg_combo = ttk.Combobox(lc, textvariable=self.leg_choice, state="readonly", width=22,
                                      values=[self.leg_label(k) for k in range(len(self.cfg["legs"]))])
        self.leg_combo.grid(row=0, column=1, columnspan=3, sticky="w", padx=4)
        self.leg_combo.bind("<<ComboboxSelected>>",
                            lambda _e: self.select_leg(self.leg_combo.current()))
        ttk.Label(lc, text="Nom").grid(row=1, column=0, sticky="e")
        self.leg_name = tk.StringVar()
        ttk.Entry(lc, textvariable=self.leg_name, width=24).grid(row=1, column=1, columnspan=3, sticky="w", padx=4)
        self.id_vars = []
        idf = ttk.Frame(lc)                     # IDs coxa / femur / tibia sur une ligne
        idf.grid(row=2, column=0, columnspan=4, sticky="w", pady=2)
        for k, lab in enumerate(("ID coxa", "femur", "tibia")):
            ttk.Label(idf, text=lab).pack(side="left", padx=(0 if k == 0 else 6, 2))
            v = tk.StringVar()
            ttk.Entry(idf, textvariable=v, width=4).pack(side="left")
            self.id_vars.append(v)
        mf = ttk.Frame(lc)
        mf.grid(row=3, column=0, columnspan=4, sticky="w", pady=2)
        self.mount_vars = {}
        for key, lab in (("x", "Montage x"), ("y", "y"), ("angle", "angle")):
            ttk.Label(mf, text=lab).pack(side="left", padx=(0 if key == "x" else 6, 2))
            v = tk.StringVar()
            ttk.Entry(mf, textvariable=v, width=5).pack(side="left")
            self.mount_vars[key] = v
        ttk.Label(lc, text="x avant, y gauche (mm) ; angle 0 = avant, 90 = gauche",
                  foreground="gray").grid(row=4, column=0, columnspan=4, sticky="w")
        ttk.Button(lc, text="Appliquer la patte", command=self.apply_leg).grid(
            row=5, column=0, columnspan=2, sticky="w", pady=(4, 0))

        # --- cinematique inverse de la patte selectionnee
        ik = ttk.LabelFrame(bottom, text="Cinematique : bout de patte (patte selectionnee)", padding=6)
        ik.pack(side="left", fill="both", expand=True, padx=(8, 0))
        self.ik_on = tk.BooleanVar(value=False)
        ttk.Checkbutton(ik, text="Piloter par la position du bout de patte (IK)",
                        variable=self.ik_on).grid(row=0, column=0, columnspan=2, sticky="w")
        self.ik_vars = {}
        for row, (key, lab, lo, hi) in enumerate((("coxa", "Coxa (deg)", -90, 90),
                                                  ("r", "Portee r (mm)", 40, 320),
                                                  ("z", "Hauteur z (mm)", -260, 120)), start=1):
            ttk.Label(ik, text=lab).grid(row=row, column=0, sticky="e")
            v = tk.DoubleVar(value=0)
            tk.Scale(ik, from_=lo, to=hi, orient="horizontal", resolution=1, length=230, variable=v,
                     showvalue=True, command=lambda _v: self.on_ik()).grid(row=row, column=1, sticky="w")
            self.ik_vars[key] = v
        ttk.Button(ik, text="Depuis la position actuelle", command=self.ik_from_present).grid(
            row=4, column=0, sticky="w", pady=(4, 0))
        self.ik_lbl = ttk.Label(ik, text="", width=44)
        self.ik_lbl.grid(row=4, column=1, sticky="w", pady=(4, 0))

        self.status_var = tk.StringVar(value="Lancer leg_server sur la Pi, puis Connecter.")
        # width=1 : la barre d'etat prend la largeur de la fenetre sans jamais l'agrandir
        ttk.Label(self, textvariable=self.status_var, relief="sunken", anchor="w",
                  padding=3, width=1).pack(fill="x", side="bottom")
        self.refresh_leg_widgets()

    # ------------------------------------------------------------ helpers
    def status(self, msg):
        self.status_var.set(time.strftime("%H:%M:%S  ") + msg)

    def state_of(self, sid):
        st = self.client.state
        if not st:
            return None
        for s in st.get("servos", []):
            if s.get("id") == sid:
                return s
        return None

    def servo_state(self, k):
        return self.state_of(self.leg()["ids"][k])

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
        self.send_ids()
        self.status("Connecte")

    def send_ids(self):
        """Donne au serveur la liste de tous les IDs des 6 pattes (il les scanne)."""
        self.client.send("IDS " + " ".join(str(i) for i in hm.all_ids(self.cfg)))

    def reset_safety(self, who):
        """Sort les servos du mode securite (cf. RESET dans leg_server)."""
        self.stop_gait()
        self.client.send(f"RESET {who}")
        self.status(f"Reset securite envoye ({who}) : consigne = position actuelle, couple ON")

        def resync():
            for k, p in enumerate(self.panels):
                if who == "ALL" or p.id == who:
                    s = self.servo_state(k)
                    if s and s.get("ok"):
                        p.target.set(round(p.to_deg(s["pos"])))
                    p.loaded = False
        self.after(500, resync)

    def estop(self):
        self.stop_gait()
        self.client.send("ESTOP")
        self.status("ARRET COUPLE envoye (tous les servos)")

    def torque_leg(self, on):
        for p in self.panels:
            self.client.send(f"TORQUE {p.id} {on}")

    def targets_raw(self):
        """Consignes des curseurs -> {id: position brute}, limitees aux butees."""
        out = {}
        for k, p in enumerate(self.panels):
            raw = p.to_raw(p.target.get())
            s = self.servo_state(k)
            lo, hi = (s["cw_limit"], s["ccw_limit"]) if s and s.get("ok") else (0, 1023)
            out[p.id] = max(lo, min(hi, raw))
        return out

    def on_slider(self):
        if self.initialized:
            self.client.set_goals(self.targets_raw())

    def set_targets_deg(self, degs):
        clipped = False
        for p, d in zip(self.panels, degs):
            lo, hi = p.scale.cget("from"), p.scale.cget("to")
            clipped |= not lo <= d <= hi
            p.target.set(max(lo, min(hi, d)))
        self.on_slider()
        return not clipped

    def neutral(self):
        if not messagebox.askyesno("Position neutre",
                                   "Envoyer les 3 servos de la patte a 512 (milieu) ?\n"
                                   "Le mouvement peut etre grand si la patte est repliee."):
            return
        self.stop_gait()
        self.set_targets_deg([p.to_deg(512) for p in self.panels])

    def copy_present(self):
        for k, p in enumerate(self.panels):
            s = self.servo_state(k)
            if s and s.get("ok"):
                p.target.set(round(p.to_deg(s["pos"])))

    def set_offset(self, k, off, save=True):
        """Change l'offset de l'articulation k de la patte selectionnee sans bouger le servo."""
        leg, p = self.leg(), self.panels[k]
        raw = deg_to_raw(p.target.get() - float(leg["offset_deg"][k]), bool(leg["inverted"][k]))
        leg["offset_deg"][k] = off
        p.set_range()
        p.target.set(round(p.to_deg(raw)))
        p.off_var.set(f"{off:g}")
        if save:
            save_config(self.cfg)
            self.status(f"ID {leg['ids'][k]} : offset {off:g} deg (sauvegarde)")

    def on_geometry_change(self):
        try:
            for k, v in self.len_vars.items():
                self.cfg["lengths_mm"][k] = float(v.get())
            self.cfg["tibia_bend_deg"] = float(self.bend_var.get())
            self.cfg["foot_offset_mm"] = {"along": float(self.foot_along.get()),
                                          "perp": float(self.foot_perp.get())}
        except ValueError:
            messagebox.showerror("Geometrie", "Valeur non numerique.")
            return
        save_config(self.cfg)
        self.status("Geometrie sauvegardee dans leg_gui_config.json")

    # ------------------------------------------------------------ choix / config des pattes
    def refresh_leg_widgets(self):
        k = self.cfg["selected_leg"]
        leg = self.leg()
        self.leg_combo.config(values=[self.leg_label(i) for i in range(len(self.cfg["legs"]))])
        self.leg_combo.current(k)
        self.leg_name.set(leg["name"])
        for v, i in zip(self.id_vars, leg["ids"]):
            v.set(str(i))
        for key, v in self.mount_vars.items():
            v.set(f"{leg['mount'][key]:g}")
        self.side_title.config(text=f"Patte {self.leg_label(k)} : vue de cote (femur + tibia)")
        self.title(f"Hexapode - patte {self.leg_label(k)} (IDs {' / '.join(map(str, leg['ids']))})")
        for p in self.panels:
            p.rebind()

    def select_leg(self, k):
        if k < 0 or k == self.cfg["selected_leg"]:
            return
        self.stop_gait()
        self.cfg["selected_leg"] = k
        save_config(self.cfg)
        self.refresh_leg_widgets()
        self.copy_present()          # curseurs = position reelle : aucun mouvement
        self.ik_from_present()
        self.status(f"Patte {self.leg_label(k)} selectionnee")

    def apply_leg(self):
        leg = self.leg()
        try:
            ids = [int(v.get()) for v in self.id_vars]
            mount = {key: float(v.get()) for key, v in self.mount_vars.items()}
        except ValueError:
            messagebox.showerror("Patte", "IDs et montage doivent etre numeriques.")
            return
        if len(set(ids)) != 3 or not all(0 <= i <= 252 for i in ids):
            messagebox.showerror("Patte", "3 IDs differents entre 0 et 252.")
            return
        others = {i: l["name"] for j, l in enumerate(self.cfg["legs"]) if j != self.cfg["selected_leg"]
                  for i in l["ids"]}
        dup = [f"{i} ({others[i]})" for i in ids if i in others]
        if dup and not messagebox.askyesno("Patte", "IDs deja utilises par une autre patte : "
                                                    + ", ".join(dup) + "\nContinuer quand meme ?"):
            return
        leg["name"] = self.leg_name.get().strip() or leg["name"]
        leg["ids"] = ids
        leg["mount"] = mount
        save_config(self.cfg)
        self.refresh_leg_widgets()
        self.send_ids()
        self.copy_present()
        self.status(f"Patte {self.leg_label(self.cfg['selected_leg'])} : configuration sauvegardee")

    def on_robot_click(self, ev):
        """Choisit la patte la plus proche du clic (hanche ou bout de patte)."""
        best, dist = None, 35.0
        for k, pts in self.robot_view.get("legs", {}).items():
            for (x, y) in pts:
                d = math.hypot(ev.x - x, ev.y - y)
                if d < dist:
                    best, dist = k, d
        if best is not None:
            self.select_leg(best)

    # ------------------------------------------------------------ cinematique inverse
    def on_ik(self):
        if not self.ik_on.get():
            return
        c, r, z = (self.ik_vars[k].get() for k in ("coxa", "r", "z"))
        sol = hm.leg_ik(self.cfg, c, r, z)
        if sol is None:
            self.ik_lbl.config(text="hors d'atteinte", foreground="red")
            return
        ok = self.set_targets_deg(sol) if self.initialized else True
        txt = f"coxa {sol[0]:+.0f}  femur {sol[1]:+.0f}  tibia {sol[2]:+.0f} deg"
        self.ik_lbl.config(text=txt + ("" if ok else "  (limite)"), foreground="black" if ok else "red")

    def ik_from_present(self):
        """Curseurs IK = position actuelle du bout de patte (sans bouger)."""
        ang = self.joint_angles("pos") or self.joint_angles("target")
        r, z = hm.leg_fk(self.cfg, ang)["foot"]
        was = self.ik_on.get()
        self.ik_on.set(False)
        self.ik_vars["coxa"].set(round(ang[0]))
        self.ik_vars["r"].set(round(r))
        self.ik_vars["z"].set(round(z))
        self.ik_on.set(was)
        self.ik_lbl.config(text=f"bout de patte : r {r:.0f} mm, z {z:+.0f} mm", foreground="black")

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
    def joint_angles(self, source, leg_index=None):
        """Angles (deg) des 3 articulations d'une patte : 'pos' (reel) ou 'target'."""
        if source == "target" and leg_index is None:
            return [p.target.get() for p in self.panels]
        leg = self.leg(leg_index)
        out = []
        for k, sid in enumerate(leg["ids"]):
            s = self.state_of(sid)
            if not s or not s.get("ok"):
                return None
            out.append(hm.raw_to_joint(leg, k, s["pos"]))
        return out

    def leg_geometry(self, angles):
        return hm.leg_fk(self.cfg, angles)

    def draw(self):
        self.draw_robot()
        self.draw_side()

    def draw_robot(self):
        cv = self.robot_cv
        cv.delete("all")
        w, h = 340, 270
        L = self.cfg["lengths_mm"]
        reach = L["coxa"] + L["femur"] + L["tibia"]
        legs = self.cfg["legs"]
        ext = max(max(abs(l["mount"]["x"]), abs(l["mount"]["y"])) for l in legs) + reach * 0.75
        sc = min(w, h) / 2 / ext
        cx, cy = w / 2, h / 2 + 5

        def to_s(x, y):                       # repere corps -> ecran (avant vers le haut)
            return cx - y * sc, cy - x * sc

        # corps : polygone des points de montage
        mounts = sorted(((l["mount"]["x"], l["mount"]["y"]) for l in legs),
                        key=lambda p: math.atan2(p[1], p[0]))
        cv.create_polygon(*[c for p in mounts for c in to_s(*p)], fill="#eceff1", outline="#607d8b", width=2)
        cv.create_text(cx, 10, text="avant", fill="#607d8b")
        cv.create_line(cx, 18, cx, 30, arrow="first", fill="#607d8b")

        sel = self.cfg["selected_leg"]
        self.robot_view = {"legs": {}}
        for k, leg in enumerate(legs):
            real = self.joint_angles("pos", k)
            ang = real or [0, 0, -60]
            pts = hm.leg_to_body(self.cfg, leg, ang)
            chain = [to_s(*pts[n]) for n in ("hip", "coxa_end", "knee", "foot")]
            alarm = any(self.state_of(i) and self.state_of(i).get("ok") and safety_reason(self.state_of(i))
                        for i in leg["ids"])
            if real is None:
                color, width, dash = "#bdbdbd", 2, (3, 3)
            elif alarm:
                color, width, dash = "#d32f2f", 4, None
            else:
                color, width, dash = ("#1565c0" if k == sel else "#455a64"), (5 if k == sel else 3), None
            cv.create_line(*[c for p in chain for c in p], fill=color, width=width, dash=dash,
                           capstyle="round", joinstyle="round")
            fx, fy = chain[-1]
            cv.create_oval(fx - 4, fy - 4, fx + 4, fy + 4, fill=color, outline="")
            hx, hy = chain[0]
            cv.create_oval(hx - 9, hy - 9, hx + 9, hy + 9, fill="white", outline=color, width=2)
            cv.create_text(hx, hy, text=str(k + 1), fill=color, font=("Segoe UI", 8, "bold"))
            self.robot_view["legs"][k] = [chain[0], chain[-1]]

        # consigne + butees de la coxa de la patte selectionnee
        leg = self.leg()
        tgt = self.joint_angles("target")
        pts = hm.leg_to_body(self.cfg, leg, tgt)
        chain = [to_s(*pts[n]) for n in ("hip", "coxa_end", "knee", "foot")]
        cv.create_line(*[c for p in chain for c in p], fill="#9e9e9e", width=2, dash=(4, 3))
        s = self.servo_state(0)
        if self.show_limits.get() and s and s.get("ok"):
            hx, hy = to_s(leg["mount"]["x"], leg["mount"]["y"])
            for raw, lab in ((s["cw_limit"], "CW"), (s["ccw_limit"], "CCW")):
                a = math.radians(leg["mount"]["angle"] + self.panels[0].to_deg(raw))
                ln = (L["coxa"] + L["femur"]) * sc
                x2, y2 = hx - ln * math.sin(a), hy - ln * math.cos(a)
                cv.create_line(hx, hy, x2, y2, fill="#d32f2f", width=2)
                cv.create_text(x2 - 10 * math.sin(a), y2 - 10 * math.cos(a), text=lab,
                               fill="#d32f2f", font=("Segoe UI", 7))
        cv.create_text(w - 4, h - 8, anchor="e", fill="#999",
                       text="gris pointille = servos absents, rouge = alarme")

    def draw_side(self):
        sv = self.side_cv
        sv.delete("all")
        L = self.cfg["lengths_mm"]
        lc, lt = L["coxa"], L["tibia"]
        reach = lc + L["femur"] + lt
        w2 = 340
        sc2 = (w2 - 50) / reach
        sx, sy = 25, 95
        sv.create_rectangle(0, sy - 25, sx, sy + 25, fill="#333", outline="")
        gy = sy + (lt * 0.85) * sc2
        sv.create_line(0, gy, w2, gy, fill="#bbb", dash=(2, 4))
        sv.create_text(w2 - 6, gy - 8, text="sol (indicatif)", anchor="e", fill="#aaa")

        tgt = self.joint_angles("target")
        real = self.joint_angles("pos")

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

        if self.show_limits.get():
            self.draw_limits_side(real or tgt, sv, (sx, sy, sc2))
        side(tgt, "ghost")
        if real:
            side(real, "real")

    def draw_limits_side(self, pose, cv, frame):
        """Butees CW / CCW du femur (depuis son axe) et du tibia (depuis le genou, direction
        du bout de patte, courbure incluse)."""
        red = "#d32f2f"
        L = self.cfg["lengths_mm"]
        sx, sy, sc2 = frame

        def ray(x, y, ang_deg, length, label):
            a = math.radians(ang_deg)
            x2, y2 = x + length * math.cos(a), y - length * math.sin(a)
            cv.create_line(x, y, x2, y2, fill=red, width=2)
            cv.create_text(x2 + 12 * math.cos(a), y2 - 12 * math.sin(a), text=label,
                           fill=red, font=("Segoe UI", 7))

        s1, s2 = self.servo_state(1), self.servo_state(2)
        if s1 and s1.get("ok"):
            for raw, lab in ((s1["cw_limit"], "CW"), (s1["ccw_limit"], "CCW")):
                ray(sx + L["coxa"] * sc2, sy, self.panels[1].to_deg(raw), L["femur"] * sc2, lab)
        if s2 and s2.get("ok"):
            g = self.leg_geometry(pose)
            kx, ky = sx + g["knee"][0] * sc2, sy - g["knee"][1] * sc2
            bend = float(self.cfg.get("tibia_bend_deg", 0))
            for raw, lab in ((s2["cw_limit"], "CW"), (s2["ccw_limit"], "CCW")):
                ray(kx, ky, pose[1] + self.panels[2].to_deg(raw) + bend, L["tibia"] * sc2, lab)

    def on_show_limits(self):
        self.cfg["show_limits"] = bool(self.show_limits.get())
        save_config(self.cfg)

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
            ids = self.leg()["ids"]
            for k, (x, y) in enumerate(pts[1:-1]):
                cv.create_oval(x - 6, y - 6, x + 6, y + 6, fill="white", outline="#333", width=2)
                cv.create_text(x, y - 13, text=str(ids[k + 1]), fill="#333")
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
        if self.client.connected and st and time.time() - self.client.state_time < 2.0:
            self.conn_lbl.config(text="connecte", foreground="green")
            if not self.initialized:
                # premiere lecture : curseurs = position actuelle (pas de saut au demarrage)
                self.copy_present()
                self.ik_from_present()
                self.initialized = True
            for k, p in enumerate(self.panels):
                p.update_state(self.servo_state(k))
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
