# Couple des servos de l'hexapode : calculs, essais et résultats

Ce document rassemble le calcul du couple demandé aux servos Dynamixel pour
porter et lever l'hexapode, les essais faits le **25/11/2026** et leurs
résultats, et la façon de refaire les essais et les graphiques.

Programme d'essai : `Dyna_test/gui/test_torque.py` (README principal, §15).
Mesures brutes : `Dyna_test/gui/logs/` (versionnées, voir §5).

---

## 1. Le robot

| Élément | Valeur |
|---|---|
| Servos | 18 × **AX-12A** (modèle 12 lu sur les servos), protocole 1.0, 1 Mbaud |
| Couple de blocage AX-12A | **1,5 N·m à 12 V** (doc ROBOTIS), ~proportionnel à la tension |
| Pattes | 6 pattes × 3 servos : coxa (hanche), fémur, tibia |
| Longueurs (approx., modèle SolidWorks) | coxa 50 mm, fémur 75 mm, tibia 125 mm (genou → bout) |
| Tibia | cintré : courbure −45°, bout décalé de −20 mm (réglages de l'interface) |
| Disposition | patte 1 avant gauche 3/4/5, 2 avant droite 0/1/2, 3 milieu droite 15/16/17, 4 arrière droite 12/13/14, 5 arrière gauche 9/10/11, 6 milieu gauche 6/7/8 |
| Alimentation | 12,1 V au début de la journée, **9,8–10,8 V** pendant l'essai à 6 pattes |

> Le modèle SolidWorks prévoit des **AX-18F** (1,8 N·m à 12 V), mais les
> servos montés sont des AX-12A.

---

## 2. Calcul

### 2.1 Couple au fémur et au tibia

Robot immobile, poids réparti sur les `n` pattes au sol. Chaque bout de patte
reçoit une force verticale

```
F = m · g / n
```

Le couple qui s'exerce sur une articulation vaut cette force multipliée par la
**distance horizontale** entre l'axe de l'articulation et le bout de patte (le
bras de levier) :

```
couple fémur = F · d_f     d_f = distance horizontale axe du fémur → bout de patte
couple tibia = F · d_t     d_t = distance horizontale genou → bout de patte
```

La coxa (axe vertical) ne porte pas le poids. Elle ne travaille que pour les
déplacements horizontaux et les frottements.

**Conséquence :** à poids égal, le couple est proportionnel au bras de levier.
Rapprocher les bouts de patte du corps réduit le couple d'autant.
`test_torque.py` calcule `d_f` et `d_t` à chaque pas, par cinématique directe
(`levier_femur_mm` et `levier_tibia_mm` dans les mesures).

### 2.2 Couple disponible

```
couple max ≈ 1,5 N·m × (tension / 12 V)        AX-12A
           ≈ 1,33 N·m à 10,6 V (tension moyenne de l'essai à 6 pattes)
```

Le registre « charge » (40) d'un AX donne en gros la part du couple maximal
utilisée, en %. **Ce n'est pas un capteur de couple étalonné** : c'est
l'ordre de grandeur qu'il faut retenir. Quand le couple est coupé, ce registre
garde une valeur figée (souvent ±1023), qui est ignorée.

### 2.3 Couple demandé au fémur selon la configuration

Exemple pour une masse de **2,0 kg** (valeur à remplacer par la masse pesée,
voir §4.3), soit 19,6 N :

| Bras de levier fémur `d_f` | 6 pattes au sol | 3 pattes (trépied) | 1 patte seule |
|---|---|---|---|
| **120 mm** (bouts à 170 mm, essai réussi) | 0,39 N·m (**29 %** de 1,33) | 0,78 N·m (59 %) | 2,35 N·m (**177 %, impossible**) |
| **196 mm** (bouts à ~250 mm, pattes écartées) | 0,64 N·m (48 %) | 1,28 N·m (**96 %**) | 3,85 N·m (impossible) |

Pour une masse de 1,5 kg, ces valeurs sont à multiplier par 0,75.

**À retenir :**
- **Une seule patte ne peut pas lever le robot**, quel que soit le réglage.
  C'est ce qu'ont montré les essais du §3.2.
- **Pattes écartées, la marche en trépied (3 pattes au sol) est à la limite.**
  Il faut garder les bouts de patte près du corps quand le robot marche.
- **Pour un fonctionnement continu**, on vise en général une fraction modeste
  du couple de blocage (de l'ordre de 20 à 30 %) pour limiter l'échauffement.
  Debout à 6 pattes avec `d_f` = 120 mm, on y est (§3.3).

---

## 3. Essais du 25/11/2026

### 3.1 Essai 1 : réglages de compliance, une patte (12/13/14)

Fémur abaissé de 50 unités (≈ 15°) sous charge, 6 combinaisons de pente et de
punch, vitesse 150 (README §14.5) :

| Pente | Punch | Atteint | Écart 13 / 14 (unités) | Oscillation | Maintien fémur 13 |
|---|---|---|---|---|---|
| 32 (usine) | 32 | **non** | +7,9 / +4,1 | 0,3 | 24 % |
| 16 | 32 | 0,26 s | +3,0 / +4,0 | 0,4 | 16 % |
| 16 | 64 | 0,26 s | +3,0 / +3,4 | 0,5 | 19 % |
| 8 | 64 | 0,20 s | +2,6 / +1,0 | 0,8 | 30 % |
| **8** | **100** | **0,20 s** | **+1,0 / +1,0** | **0,0** | **22 %** |
| 4 | 64 | 0,19 s | +1,0 / +0,7 | 0,5 | 31 % |

→ **Retenu : pente 8, punch 100** pour les fémurs et les tibias
(`Dyna_test/leg_settings.txt`, réglages réappliqués par `test_torque`).

### 3.2 Essai 2 : une patte seule tient le robot levé → échec

Pose levée réglée à la main sur la patte 12/13/14 : fémur 13 à **60 %** et
tibia 14 à **35 %** de charge en continu, à 47 °C. Au bout de moins d'une
minute, le servo 13 se met en **sécurité pour surcharge** (erreur `0x20`,
limite de couple remise à 0) et le robot retombe. Ensuite, depuis la position
basse, le fémur ne remonte pas au-delà de 570 (consigne 505) avant de
re-décrocher. C'est cohérent avec le §2.3 : une patte seule ne peut pas lever
le robot. Les résultats de la série d'essais lancée juste après, avec cette
pose pour objectif, sont **invalides** : le servo était déjà en sécurité au
départ et le robot était retombé.

### 3.3 Essai 3 : lever complet à 6 pattes → **réussi**

`python test_torque.py --radius 170 --stand-z -90` — fichiers
`logs/test_torque_20261125_111436_*`.

| Paramètre | Valeur |
|---|---|
| Départ | robot posé, bouts de patte à 224–263 mm des hanches (`d_f` moyen 196 mm) |
| Rapprocher | par trépieds (levée 15 mm) → bouts de patte à **170 mm** (`d_f` = **120 mm**) |
| Lever | bouts de patte de z = −10 à **−90 mm** sous l'axe des hanches, pas de 4 mm, vitesse 80 |
| Critère par pas | les 18 servos à ≤ 6 unités de leur consigne (3 lectures de suite), sinon échec après 8 s |
| Réglages | fémurs et tibias pente 8 / punch 100, limite de couple 1023 |

**Résultat :**
- les **49 poses** ont été **atteintes**, chacune en 0,3 à 2,5 s, sans aucun
  délai dépassé ;
- aucune alarme, température **37–38 °C** ;
- le robot a tenu 5 s debout, puis est redescendu ; 237 lectures en 27,5 s.

**Contact avec le sol :** la charge moyenne des fémurs est au plus bas
(6–8 %) vers z = −33 à −37 mm, puis remonte régulièrement. Les bouts de patte
touchent donc le sol vers **z ≈ −38 mm** et **le corps s'est levé d'environ
52 mm** (les bouts de patte, eux, ont parcouru 80 mm).

| z bout de patte (mm) | −36,6 | −44,2 | −51,8 | −59,5 | −67,1 | −74,7 | −82,4 | −90,0 |
|---|---|---|---|---|---|---|---|---|
| Charge moyenne des fémurs | 6 % | 20 % | 27 % | 25 % | 30 % | 25 % | 25 % | 21 % |

**Debout (z = −90 mm, `d_f` = 120 mm), charge moyenne sur 5 s :**

| Patte | IDs | Coxa | Fémur | Tibia |
|---|---|---|---|---|
| 1 avant gauche | 3 / 4 / 5 | 0 % | 7,9 % | 3,7 % |
| 2 avant droite | 0 / 1 / 2 | 0 % | **22,3 %** | 5,0 % |
| 3 milieu droite | 15 / 16 / 17 | 0 % | 7,6 % | 0 % |
| 4 arrière droite | 12 / 13 / 14 | 0 % | **22,3 %** | **10,9 %** |
| 5 arrière gauche | 9 / 10 / 11 | 0 % | 17,7 % | 0 % |
| 6 milieu gauche | 6 / 7 / 8 | 0 % | **22,7 %** | 8,5 % |
| **Moyenne** | | | **16,7 %** | **4,7 %** |

Pics pendant tout l'essai (transitoires) : ID 13 **92 %**, ID 7 83 %,
ID 2 73 %, ID 1 72 %, ID 14 72 %, ID 8 72 %.

### 3.4 Analyse

- **Le lever à 6 pattes est largement possible** si les bouts de patte sont
  près du corps : 17 % de charge moyenne debout, avec une bonne marge.
- **La charge est mal répartie** : 22 % sur les pattes 2, 4 et 6 contre 8 %
  sur les pattes 1 et 3. Leurs bouts de patte ne sont donc pas exactement à
  la même hauteur : les **offsets ne sont pas parfaitement calés** (quelques
  millimètres suffisent). Pendant le lever, la patte 4 (12/13/14) porte
  souvent seule (jusqu'à 55–69 %), et c'est elle qui fait le pic de 92 %.
  → À recaler : robot posé sur un plan, mettre toutes les pattes dans la
  même pose et ajuster les offsets (« Zéro ici »).
- **La tension chute à 9,8 V** sous charge (12,1 V le matin) : batterie
  faible. Le couple disponible baisse d'autant (−19 % à 9,8 V). Recharger
  avant les essais de marche.
- **Tibias peu chargés** (5 %) : avec la courbure du tibia, le bout de patte
  est presque à l'aplomb du genou (petit `d_t`).

### 3.5 Estimation de la masse à partir des mesures (ordre de grandeur)

Debout : fémurs à 16,7 % de ≈ 1,33 N·m, soit 0,22 N·m par fémur.
Avec `d_f` = 0,120 m : F ≈ 1,85 N par patte, donc 11,1 N pour 6 pattes, soit
**≈ 1,1 kg**.

C'est **sous-estimé** : les 18 servos pèsent à eux seuls environ 1,0 kg
(54,6 g chacun), et la charge des AX n'est pas un couple étalonné.
**Peser le robot** permettra d'étalonner le registre de charge :
`facteur = (m·g·d_f / 6) / (charge% × couple max)`.

---

## 4. Refaire les essais

### 4.1 Commandes

```bash
cd Dyna_test/gui
python test_torque.py --dry-run --radius 170 --stand-z -90   # verifie le plan, ne bouge rien
python test_torque.py --radius 170 --stand-z -90             # essai reel (confirmation)
python test_torque.py --radius 170 --stand-z -90 --full      # + deplacer le corps + trepieds
```

Options utiles : `--radius` (distance bout de patte / hanche), `--stand-z`
(hauteur debout), `--step` (mm par pas), `--speed`, `--tol` (unités),
`--settle-max` (s avant échec), `--hold` (s debout), `--temp-max`.

### 4.2 Séries proposées pour les graphiques

1. **Charge en fonction du bras de levier** : `--radius` 150, 170, 190, 210
   avec `--stand-z -90` → charge moyenne des fémurs debout en fonction de
   `levier_femur_mm`. La théorie prévoit une droite (§2.1).
2. **Charge en fonction de la hauteur** : `--stand-z` −70, −90, −110 avec
   `--radius 170`.
3. **Trépieds** (`--full`) : charge à 3 pattes au sol, comparée au §2.3.
4. **Effet de la tension** : même essai batterie pleine / batterie faible.

### 4.3 Avant les prochains essais

- [ ] Peser le robot → masse réelle pour le §2.3 et l'étalonnage du §3.5
- [ ] Recaler les offsets pour équilibrer la charge entre les pattes (§3.4)
- [ ] Recharger la batterie (tension ≥ 12 V)

---

## 5. Fichiers de mesures

Dans `Dyna_test/gui/logs/` (séparateur `;`) :

| Fichier | Contenu |
|---|---|
| `essais.csv` | une ligne par essai réel : date, résultat, rayon, z debout, pas, vitesse, hauteur levée, charge max (et ID), température max, tension min, durée |
| `test_torque_<date>_pas.csv` | une ligne par pas : phase, z visé, atteint, durée, charge max (et ID), charge moyenne fémurs / tibias, écart max (et ID), bras de levier fémur / tibia, température, tension |
| `test_torque_<date>_echantillons.csv` | chaque lecture (~8 par seconde) : temps, phase, pas, z visé, puis position, consigne, charge (vide si couple coupé) et température des 18 servos, tension |
| `test_torque_<date>.json` | paramètres, résultat, pics de charge par servo, configuration des pattes et géométrie |

La charge est en unités brutes (−1023 à +1023, le signe donne le sens).
Pour l'avoir en % : `abs(charge) × 100 / 1023`.

### Exemple de graphique (Python + matplotlib)

```python
import csv, matplotlib.pyplot as plt

tag = "logs/test_torque_20261125_111436"
pas = list(csv.DictReader(open(tag + "_pas.csv", encoding="utf-8"), delimiter=";"))
lever = [p for p in pas if p["phase"] == "lever"]
z = [-float(p["z_vise_mm"]) for p in lever]
plt.plot(z, [float(p["femurs_moy_pct"]) for p in lever], "o-", label="femurs (moyenne)")
plt.plot(z, [float(p["tibias_moy_pct"]) for p in lever], "s-", label="tibias (moyenne)")
plt.plot(z, [float(p["charge_max_pct"]) for p in lever], "^--", label="charge max")
plt.xlabel("descente du bout de patte (mm)"); plt.ylabel("charge (% du couple max)")
plt.title("Lever a 6 pattes, bouts de patte a 170 mm"); plt.legend(); plt.grid(); plt.show()

# Charge d'un servo dans le temps
ech = list(csv.DictReader(open(tag + "_echantillons.csv", encoding="utf-8"), delimiter=";"))
t = [float(r["t_s"]) for r in ech if r["charge_13"]]
c = [abs(int(r["charge_13"])) * 100 / 1023 for r in ech if r["charge_13"]]
plt.plot(t, c); plt.xlabel("temps (s)"); plt.ylabel("charge ID 13 (%)"); plt.grid(); plt.show()
```
