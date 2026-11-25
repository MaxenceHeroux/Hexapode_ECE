# `test_torque` — lever l'hexapode et mesurer le couple

Script : [`gui/test_torque.py`](gui/test_torque.py). Il fait se lever le robot
complet (6 pattes, 18 AX-12A), le tient debout, le repose, et enregistre
charge, position, température et tension de chaque servo.

Documents liés : [dyna.md](dyna.md) §3 (pattes, `leg_server`),
[Torque_calcul.md](Torque_calcul.md) (calculs de couple et premiers essais).

---

## 1. Avant de lancer

1. **`leg_server` tourne sur la Pi** ([dyna.md](dyna.md) §2.1) :

   ```bash
   ssh hexapode "pgrep -a leg_server; tail /tmp/leg_server.log"
   # sinon, le lancer :
   ssh hexapode "cd ~/Hexapode_ECE/Dyna_test && make && (nohup ./bin/leg_server > /tmp/leg_server.log 2>&1 &)"
   ```

2. **La calibration des pattes est présente** : `gui/leg_gui_config.json`
   (IDs, sens, offsets, montage, tibia cintré −45°, décalage du pied −20 mm).
   Ce fichier est **ignoré par git** (`Dyna_test/.gitignore`) : il faut le
   copier à la main sur chaque PC. Sans lui, le script prend la configuration
   par défaut de `hexapod_model.py` (offsets à 0, tibia −30°) et la pose
   calculée est fausse (bouts de patte vus à z ≈ +40 mm au lieu du sol).

   Sur le PC Linux en dual boot, la copie se récupère depuis la partition
   Windows (montée en lecture seule) :

   ```bash
   udisksctl mount -b /dev/nvme0n1p3 -o ro
   cp "/media/$USER/8004DF2F04DF26C6/Données/A-Dossier ECE/MAKERS/Hexapode/Dyna_test/gui/leg_gui_config.json" \
      Dyna_test/gui/leg_gui_config.json
   ```

3. **Batterie chargée** (≥ 11 V à vide ; à 9,7 V le couple manque).
4. **Robot posé au sol, quelqu'un à côté de l'alimentation.**

---

## 2. Commandes

Depuis `Dyna_test/gui` :

```bash
# Toujours commencer par une simulation : lit le robot, calcule et verifie
# toutes les poses, ne bouge rien
python3 test_torque.py --dry-run --home

# Essai recommande : pattes en etoile de mer a 175 mm, puis lever
python3 test_torque.py --home

# Meme chose sans la question de confirmation
python3 test_torque.py --home --yes

# Etoile de mer a une autre distance
python3 test_torque.py --home --radius 180

# Rapprocher les pattes sans changer leur direction (ancien comportement)
python3 test_torque.py --radius 175

# Partir de la pose actuelle telle quelle (aucune mise en place)
python3 test_torque.py

# Essai complet : lever + deplacer le corps + trepieds
python3 test_torque.py --home --full

# Simulation sans robot ni leg_server
python3 test_torque.py --offline --home
```

Graphiques des mesures : `python3 plot_torque.py` (voir
[Torque_calcul.md](Torque_calcul.md)).

---

## 3. Options

| Option | Défaut | Rôle |
|---|---|---|
| `--dry-run` | | Lit le robot, calcule et vérifie le plan, **ne bouge rien** |
| `--offline` | | Simulation sans se connecter à `leg_server` |
| `--yes` | | Pas de confirmation avant de bouger |
| `--home` | | **Nouveau.** Avant le lever, met chaque patte en étoile de mer (voir §4) |
| `--radius` | inchangé (175 avec `--home`) | Distance bout de patte ↔ axe de hanche pendant le lever (mm) |
| `--clear` | 15 | Levée d'une patte pendant qu'on la déplace (mm) |
| `--start-z` | −10 | Hauteur de départ commune des bouts de patte, corps posé (mm) |
| `--stand-z` | −90 | Hauteur des bouts de patte debout (mm, < 0) |
| `--step` | 4 | Pas de descente des bouts de patte pendant le lever (mm) |
| `--tol` | 6 | Écart toléré position / consigne pour valider un pas (unités servo) |
| `--settle-max` | 8 | Attente max d'un pas avant échec (s) |
| `--hold` | 5 | Durée debout (s) |
| `--speed` | 80 | Vitesse des servos (1..1023) |
| `--temp-max` | 62 | Température d'arrêt (°C) |
| `--full` | | Ajoute « déplacer le corps » et « trépieds » |
| `--shift` | 20 | Déplacement du corps avec `--full` (mm) |
| `--host`, `--port` | ceux de `leg_gui_config.json` | Adresse de `leg_server` |

---

## 4. Déroulé

0. **Vérifications** : 18 servos présents, aucun en alarme, température
   < `--temp-max` ; **toutes les poses du plan sont calculées et vérifiées
   (atteignables, dans les butées) avant le moindre mouvement**.
1. **Réglages** : fémurs / tibias pente 8, punch 100, vitesse `--speed` ;
   consigne = position actuelle puis couple ON (aucun saut).
2. **Mise en place** (phase `rapprocher` dans les fichiers), seulement avec
   `--home` ou `--radius` : par trépieds, 3 pattes se lèvent de `--clear` mm,
   se déplacent et se reposent, puis les 3 autres. Le corps reste au sol.
   - `--radius R` : chaque patte garde **sa direction** et se met à R mm de
     sa hanche.
   - `--home` : chaque patte va **dans l'axe de son montage, coxa à 0°**
     (« étoile de mer »), à `--radius` mm (175 par défaut).
3. **Lever** : tous les bouts de patte descendent par pas de `--step` mm
   jusqu'à `--stand-z` ; à chaque pas, on attend que les 18 servos soient
   arrivés.
4. **Tenir** `--hold` s debout.
5. (`--full`) Déplacer le corps, puis trépieds.
6. **Redescendre** à la hauteur de départ. Le couple reste actif à la fin.

**Sécurité** : arrêt immédiat, consignes figées et **couple gardé** (le robot
n'est pas lâché) si un servo ne répond plus, passe en alarme, dépasse
`--temp-max`, ou sur Ctrl+C.

### Pourquoi `--home`

Sans mise en place, le script part de la pose où on a laissé le robot. Après
un changement de batterie (couple coupé, pattes bougées à la main), les coxas
des pattes du milieu (IDs 6 et 15) étaient tournées d'environ 30° :
`--radius` ne corrige que la distance, pas la direction. `--home` ramène
toutes les coxas à 512 (centre) : **même pose de départ à chaque essai**, donc
essais comparables.

---

## 5. Fichiers produits

Dans `gui/logs/` :

| Fichier | Contenu |
|---|---|
| `test_torque_<date>_echantillons.csv` | Chaque lecture : temps, phase, pas, z visé ; position, consigne, charge, température des 18 servos ; tension |
| `test_torque_<date>_pas.csv` | Un résumé par pas (atteint, temps, charges max / moyennes fémurs / tibias, bras de levier) |
| `test_torque_<date>.json` | Paramètres et résumé |
| `essais.csv` | Une ligne par essai (cumul) ; colonne rayon = 175 pour `--home` sans `--radius` |

---

## 6. Essais du 25/11/2026 (après-midi)

Batterie neuve (≈ 11,1 V), calibration Windows, lever de z −10 à −90 mm
(80 mm), 5 s debout.

| Essai | Commande | Résultat | Bras de levier fémur | Fémurs debout | Pic de charge (pendant le mouvement) |
|---|---|---|---|---|---|
| 152653 | `test_torque.py` | **Arrêt** au 1er pas : ID 6 ne répond plus (USB de l'U2D2 déconnecté) | — | — | — |
| 152908 | `test_torque.py` | Debout | 141 mm (pattes laissées écartées) | 17 % | 97 % (ID 1) |
| 153127 | `--radius 170` | **Refusé avant tout mouvement** : tibia ID 2 hors butée (234 < 244) | — | — | — |
| 153142 | `--radius 175` | Debout | 125 mm | 11–15 % | 86 % (ID 1) |
| 153300 | `--home` (175 mm) | Debout, coxas toutes à 512 | 125 mm | 19–21 % | 85 % (IDs 1, 13) |

Constats :

- **Rapprocher les pattes** (141 → 125 mm de bras de levier) fait baisser le
  pic de charge de 97 % à 85 %.
- **Les pics restent au seuil d'arrêt (85 %)** sur les fémurs avant droit
  (ID 1) et arrière droit (ID 13) : ils ne durent pas assez longtemps
  (> 0,6 s) pour arrêter l'essai, mais la marge est faible.
- **À 170 mm, le tibia avant droit touche sa butée** : 175 mm est le rayon
  minimal avec la calibration actuelle.
- **La température monte au fil des essais** (35 → 44 °C sur 4 essais
  rapprochés) : laisser refroidir entre deux séries.
- **L'USB de l'U2D2 décroche** (erreurs `urb status -71` puis déconnexion dans
  le journal du noyau de la Pi, jusqu'à 9 en 5 minutes). `leg_server` se
  reconnecte seul, mais un décrochage au mauvais moment arrête l'essai.
  À traiter avant la marche : câble USB court, ferrite, masse commune.
- La coxa de la patte avant droite (ID 0) est la seule du côté droit marquée
  « normal » dans `leg_gui_config.json` (les autres sont « inversé ») :
  à vérifier dans l'interface avant de faire tourner les coxas (marche).

Vérifier l'USB sur la Pi :

```bash
ssh hexapode 'grep -c rouvert /tmp/leg_server.log; journalctl -k --since "-10 min" --no-pager | grep -iE "usb disconnect|urb status"'
```
