"""plot_torque - genere les graphiques de Torque_calcul.md a partir des mesures de
test_torque (gui/logs/) et les enregistre dans Dyna_test/graphs/.

    pip install matplotlib
    python plot_torque.py                         # dernier essai test_torque
    python plot_torque.py test_torque_20261125_111436
    python plot_torque.py --mass 2.0              # masse du robot pour le graphique theorique

Graphiques :
  lever_charge_vs_descente.png  charge des femurs / tibias a chaque pas du lever
  charge_femurs_temps.png       charge du femur de chaque patte pendant tout l'essai
  charge_par_patte_debout.png   charge coxa / femur / tibia de chaque patte, debout
  tension_temperature.png       tension d'alimentation et temperature max
  reglages_compliance.png       essai 1 : ecart et charge selon pente / punch
  couple_theorique.png          couple demande au femur selon le bras de levier
"""

import argparse
import csv
import glob
import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
LOGS = os.path.join(HERE, "logs")
OUT = os.path.join(os.path.dirname(HERE), "graphs")
sys.path.insert(0, HERE)
import hexapod_model as hm  # noqa: E402

PHASE_COLORS = {"rapprocher": "#fff3e0", "lever": "#e3f2fd", "tenir": "#e8f5e9",
                "deplacer": "#f3e5f5", "trepied": "#fce4ec", "redescendre": "#eceff1"}
LEG_COLORS = ["#1565c0", "#c62828", "#2e7d32", "#ef6c00", "#6a1b9a", "#00838f"]


def pct(v):
    return abs(int(v)) * 100 / 1023 if v not in ("", None) else None


def read_csv(path):
    with open(path, encoding="utf-8") as f:
        return list(csv.DictReader(f, delimiter=";"))


def save(fig, name):
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, name)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)
    print("  ", os.path.relpath(path, os.path.dirname(OUT)))


def shade_phases(ax, rows):
    """Fond colore par phase (echantillons)."""
    start, cur = None, None
    for r in rows + [{"phase": None, "t_s": rows[-1]["t_s"]}]:
        if r["phase"] != cur:
            if cur is not None:
                ax.axvspan(start, float(r["t_s"]), color=PHASE_COLORS.get(cur, "#fafafa"), zorder=0)
                ax.text((start + float(r["t_s"])) / 2, 1.0, cur, transform=ax.get_xaxis_transform(),
                        ha="center", va="bottom", fontsize=8, color="#555")
            cur, start = r["phase"], float(r["t_s"])


def plot_test(tag, cfg):
    pas = read_csv(os.path.join(LOGS, tag + "_pas.csv"))
    ech = read_csv(os.path.join(LOGS, tag + "_echantillons.csv"))
    info = json.load(open(os.path.join(LOGS, tag + ".json"), encoding="utf-8"))
    legs = info.get("config_pattes", cfg["legs"])
    params = info["parametres"]
    titre = f"essai {tag[12:]} - bouts de patte a {params.get('radius') or '?'} mm"

    # 1. charge par pas du lever
    lever = [p for p in pas if p["phase"] == "lever"]
    if lever:
        z = [float(p["z_vise_mm"]) for p in lever]
        fem = [float(p["femurs_moy_pct"]) for p in lever]
        fig, ax = plt.subplots(figsize=(8, 4.5))
        ax.plot(z, fem, "o-", color="#1565c0", label="femurs (moyenne des 6)")
        ax.plot(z, [float(p["tibias_moy_pct"]) for p in lever], "s-", color="#2e7d32", label="tibias (moyenne)")
        ax.plot(z, [float(p["charge_max_pct"]) for p in lever], "^--", color="#c62828", alpha=.7,
                label="charge max (un servo)")
        # pattes en l'air = charge minimale ; le contact au sol est a la FIN de ce palier
        # (z le plus bas du palier), la charge remonte ensuite
        fmin = min(fem)
        plateau = [z[i] for i in range(len(z)) if fem[i] <= fmin + 3]
        zc = min(plateau)
        ax.axvspan(max(plateau), zc, color="#eeeeee", zorder=0)
        ax.axvline(zc, color="#777", ls=":")
        ax.annotate(f"pattes en l'air\n(charge minimale)\ncontact au sol\nvers z = {zc:.0f} mm", (zc, max(fem)),
                    xytext=(6, 0), textcoords="offset points", fontsize=8, color="#555")
        ax.invert_xaxis()
        ax.set_xlabel("hauteur du bout de patte sous l'axe des hanches z (mm)  ->  le corps monte")
        ax.set_ylabel("charge (% du couple max)")
        ax.set_title("Lever a 6 pattes : charge a chaque pas\n" + titre, fontsize=10)
        ax.grid(alpha=.3)
        ax.legend(fontsize=8)
        save(fig, "lever_charge_vs_descente.png")

    # 2. charge des femurs dans le temps
    t = [float(r["t_s"]) for r in ech]
    fig, ax = plt.subplots(figsize=(10, 4.8))
    shade_phases(ax, ech)
    win = 8                                    # moyenne glissante ~1 s (8 lectures)
    for n, leg in enumerate(legs):
        fid = leg["ids"][1]
        pts = [(float(r["t_s"]), pct(r[f"charge_{fid}"])) for r in ech if r.get(f"charge_{fid}") not in ("", None)]
        if pts:
            ts, vs = zip(*pts)
            ax.plot(ts, vs, color=LEG_COLORS[n % 6], lw=0.6, alpha=0.18)
            sm = [sum(vs[max(0, i - win + 1):i + 1]) / len(vs[max(0, i - win + 1):i + 1]) for i in range(len(vs))]
            ax.plot(ts, sm, color=LEG_COLORS[n % 6], lw=1.8, label=f"patte {n + 1} ({leg['name']}, ID {fid})")
    ax.set_xlim(min(t), max(t))
    ax.set_xlabel("temps (s)")
    ax.set_ylabel("charge du femur (%)")
    ax.set_title("Charge du femur de chaque patte pendant l'essai (trait fort : moyenne glissante ~1 s, "
                 "pale : lectures brutes)\n" + titre, fontsize=10, pad=14)
    ax.grid(alpha=.3)
    ax.legend(fontsize=7, ncol=2, loc="upper left")
    save(fig, "charge_femurs_temps.png")

    # 3. charge par patte debout
    hold = [r for r in ech if r["phase"] == "tenir"]
    if hold:
        fig, ax = plt.subplots(figsize=(9, 4.5))
        w = 0.26
        for k, (lab, col) in enumerate((("coxa", "#9e9e9e"), ("femur", "#1565c0"), ("tibia", "#2e7d32"))):
            vals = []
            for leg in legs:
                sid = leg["ids"][k]
                v = [pct(r[f"charge_{sid}"]) for r in hold if r.get(f"charge_{sid}") not in ("", None)]
                vals.append(sum(v) / len(v) if v else 0)
            xs = [i + (k - 1) * w for i in range(len(legs))]
            bars = ax.bar(xs, vals, w, label=lab, color=col)
            for b, v in zip(bars, vals):
                if v >= 1:
                    ax.text(b.get_x() + b.get_width() / 2, v + .4, f"{v:.0f}", ha="center", fontsize=7)
        ax.set_xticks(range(len(legs)))
        ax.set_xticklabels([f"{n + 1} {l['name']}\n{'/'.join(map(str, l['ids']))}" for n, l in enumerate(legs)],
                           fontsize=8)
        ax.set_ylabel("charge moyenne debout (%)")
        ax.set_title(f"Repartition de la charge debout (z = {params.get('stand_z')} mm, {params.get('hold')} s)\n"
                     + titre, fontsize=10)
        ax.grid(axis="y", alpha=.3)
        ax.legend(fontsize=8)
        save(fig, "charge_par_patte_debout.png")

    # 4. tension et temperature
    fig, ax = plt.subplots(figsize=(10, 4))
    shade_phases(ax, ech)
    v = [(float(r["t_s"]), float(r["tension_V"])) for r in ech if r["tension_V"]]
    ax.plot(*zip(*v), color="#ef6c00", lw=1.2, label="tension min des servos (V)")
    ax.set_ylabel("tension (V)", color="#ef6c00")
    ax.set_xlabel("temps (s)")
    ax2 = ax.twinx()
    temps = []
    for r in ech:
        tv = [int(r[k]) for k in r if k.startswith("temp_") and r[k]]
        if tv:
            temps.append((float(r["t_s"]), max(tv)))
    ax2.plot(*zip(*temps), color="#c62828", lw=1.2, label="temperature max (degC)")
    ax2.set_ylabel("temperature max (degC)", color="#c62828")
    ax.set_xlim(min(t), max(t))
    ax.set_title("Tension d'alimentation et temperature des servos\n" + titre, fontsize=10, pad=14)
    ax.grid(alpha=.3)
    save(fig, "tension_temperature.png")
    return info


def plot_compliance():
    path = os.path.join(LOGS, "reglages_compliance_20261125.json")
    if not os.path.exists(path):
        return
    res = json.load(open(path, encoding="utf-8"))
    labels = [f"pente {r['slope']}\npunch {r['punch']}" for r in res]
    x = range(len(res))
    fig, ax = plt.subplots(figsize=(9, 4.5))
    w = 0.27
    ax.bar([i - w for i in x], [abs(r["e13"]) for r in res], w, color="#1565c0", label="ecart femur 13 (unites)")
    ax.bar(list(x), [abs(r["e14"]) for r in res], w, color="#2e7d32", label="ecart tibia 14 (unites)")
    ax.set_ylabel("ecart position / consigne (unites, 1 = 0,29 deg)")
    ax2 = ax.twinx()
    ax2.plot(list(x), [r["hold13"] for r in res], "o-", color="#c62828", label="charge de maintien 13 (%)")
    ax2.set_ylabel("charge de maintien femur 13 (%)", color="#c62828")
    for i, r in enumerate(res):
        if r["reached"] is None:
            ax.text(i - w, abs(r["e13"]) + .2, "non\natteint", ha="center", fontsize=7, color="#c62828")
        if r["slope"] == 8 and r["punch"] == 100:
            ax.text(i, max(abs(r["e13"]), abs(r["e14"])) + .6, "retenu", ha="center", fontsize=8,
                    color="#2e7d32", weight="bold")
    ax.set_xticks(list(x))
    ax.set_xticklabels(labels, fontsize=8)
    ax.set_title("Essai 1 : reglages de compliance (une patte, femur abaisse de ~15 deg sous charge)", fontsize=10)
    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, fontsize=8, loc="upper right")
    ax.grid(axis="y", alpha=.3)
    save(fig, "reglages_compliance.png")


def plot_theory(mass, measured):
    """Couple femur = m g / n x d, compare au couple disponible."""
    g = 9.81
    d = [x / 1000 for x in range(40, 241, 5)]
    fig, ax = plt.subplots(figsize=(8.5, 4.8))
    for n, col in ((6, "#2e7d32"), (3, "#ef6c00"), (1, "#c62828")):
        ax.plot([x * 1000 for x in d], [mass * g / n * x for x in d], color=col,
                label=f"{n} patte{'s' if n > 1 else ''} au sol")
    for tq, lab, ls in ((1.5, "couple max AX-12A a 12 V (1,5 N.m)", "-"),
                        (1.33, "couple max a 10,6 V (~1,33 N.m)", "--"),
                        (0.3 * 1.5, "~30 % (fonctionnement continu)", ":")):
        ax.axhline(tq, color="#555", ls=ls, lw=1)
        ax.text(242, tq, lab, fontsize=7, va="center")
    if measured:
        dm, tm = measured
        ax.plot([dm], [tm], "k*", ms=12, label=f"mesure debout ({tm:.2f} N.m a {dm:.0f} mm)")
    for dm, lab in ((120, "bouts a 170 mm"), (196, "pattes ecartees")):
        ax.axvline(dm, color="#bbb", ls=":")
        ax.text(dm, 0.05, lab, rotation=90, fontsize=7, color="#777", va="bottom", ha="right")
    ax.set_xlim(40, 240)
    ax.set_ylim(0, 3)
    ax.set_xlabel("bras de levier du femur d_f (mm)")
    ax.set_ylabel("couple demande au femur (N.m)")
    ax.set_title(f"Couple au femur = (m g / n) x d_f   (m = {mass} kg, a remplacer par la masse pesee)",
                 fontsize=10)
    ax.grid(alpha=.3)
    ax.legend(fontsize=8, loc="upper left")
    fig.subplots_adjust(right=0.78)
    save(fig, "couple_theorique.png")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("tag", nargs="?", help="essai (ex. test_torque_20261125_111436) ; defaut : le dernier")
    p.add_argument("--mass", type=float, default=2.0, help="masse du robot (kg) pour le graphique theorique")
    a = p.parse_args()
    cfg = hm.load_config()
    tags = sorted(os.path.basename(f)[:-8] for f in glob.glob(os.path.join(LOGS, "test_torque_*_pas.csv")))
    tag = a.tag or (tags[-1] if tags else None)
    print("Graphiques dans", OUT)
    measured = None
    if tag:
        info = plot_test(tag, cfg)
        # point mesure : charge moyenne des femurs debout x couple max a la tension de l'essai
        pas = read_csv(os.path.join(LOGS, tag + "_pas.csv"))
        hold = [r for r in pas if r["etape"].startswith("tenir")]
        if hold:
            fem = sum(float(r["femurs_moy_pct"]) for r in hold) / len(hold)
            volt = sum(float(r["tension_V"]) for r in hold) / len(hold)
            measured = (float(hold[0]["levier_femur_mm"]), fem / 100 * 1.5 * volt / 12)
    plot_compliance()
    plot_theory(a.mass, measured)


if __name__ == "__main__":
    main()
