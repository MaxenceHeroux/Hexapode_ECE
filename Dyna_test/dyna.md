# Dynamixel de l'hexapode — serveur, interface, essais

Pilotage des 18 servos Dynamixel AX-12A de l'hexapode depuis la Raspberry Pi : serveur `leg_server` (C++), interface graphique et cinématique (Python, sur le PC), réglages des servos et lever du robot (`test_torque`).

Connexion à la Pi, réseau et installation : [setup.md](../setup.md). Calculs de couple et résultats des essais : [Torque_calcul.md](Torque_calcul.md).

---

## 1. `Dyna_test` — programmes Dynamixel AX-12A

### 1.1 Contenu

```
Dyna_test/
├── Makefile              compile src/*.cpp → bin/
├── .gitignore            ignore bin/, la config locale de l'interface, __pycache__
├── dyna.md               ce document
├── Torque_calcul.md      calculs de couple, essais et résultats (avec graphiques)
├── graphs/               graphiques des essais (générés par gui/plot_torque.py)
├── leg_settings.txt      réglages des servos appliqués au démarrage de leg_server (§2.4)
├── udev/99-u2d2.rules    nom fixe /dev/u2d2 pour l'U2D2 (§1.7)
├── src/
│   └── leg_server.cpp    serveur TCP sur la Pi : pilote les 18 servos (§2, §3)
└── gui/                  sur le PC (Python 3 + Tkinter)
    ├── leg_gui.py        interface : robot, pattes, réglages, cinématique (§2, §3)
    ├── hexapod_model.py  config des 6 pattes, conversions, cinématique (§3)
    ├── test_torque.py    lever du robot et mesures de couple (§3, Torque_calcul.md)
    ├── plot_torque.py    graphiques des essais -> graphs/ (Torque_calcul.md §5)
    └── logs/             mesures des essais (CSV / JSON, versionnées)
```

### 1.2 Compiler

```bash
ssh hexapode
cd ~/Hexapode_ECE/Dyna_test
make                     # -> bin/leg_server
make clean               # supprime bin/
```

Le `Makefile` choisit la bibliothèque selon la machine :
`-ldxl_sbc_cpp` sur la Pi (ARM), `-ldxl_x64_cpp` sur un PC Linux x86_64.
Ne compile pas sous Windows tel quel (utilise `termios`/`unistd`).

Avant de lancer :
- brancher l'U2D2 sur **n'importe quel port USB** → il apparaît toujours en
  `/dev/u2d2` (vérifier : `ls -l /dev/u2d2`) ;
- alimenter les servos ;
- `hexa-pi` est dans le groupe `dialout` → pas besoin de `sudo`.

### 1.3 Paramètres et registres

| Paramètre | Valeur |
|---|---|
| Servos | AX-12A, **protocole 1.0** |
| Port | `/dev/u2d2` (`DEVICENAME`), nom fixe donné par la règle udev |
| Vitesse | `1000000` bauds (`BAUDRATE`) |
| Nombre de servos | 18 (IDs 0 à 17) |
| Position | 0–1023 (≈ 0–300°), 512 = milieu |

Registres de la table de contrôle AX-12A utilisés par `leg_server` :

| Registre | Adresse | Usage |
|---|---|---|
| `CW / CCW_ANGLE_LIMIT` | 6 / 8 (EEPROM) | butées |
| `MAX_TORQUE` | 14 (EEPROM) | valeur remise dans la limite de couple au reset |
| `TORQUE_ENABLE` | 24 | couple ON / OFF |
| `CW / CCW_COMPLIANCE_MARGIN` | 26 / 27 | zone morte |
| `CW / CCW_COMPLIANCE_SLOPE` | 28 / 29 | souplesse (« PID » des AX, §2.3) |
| `GOAL_POSITION` | 30 | consigne (SyncWrite pour plusieurs servos) |
| `MOVING_SPEED` | 32 | vitesse |
| `TORQUE_LIMIT` | 34 | limite de couple (RAM, mise à 0 en cas d'alarme) |
| `PRESENT_POSITION / SPEED / LOAD` | 36 / 38 / 40 | lecture |
| `PRESENT_VOLTAGE / TEMPERATURE` | 42 / 43 | lecture |
| `MOVING` | 46 | lecture |
| `PUNCH` | 48 | courant minimal |

Toute la table (adresses 0–49) est lue en un seul paquet par servo à chaque `STATE`.

### 1.4 Anciens programmes (supprimés le 25/11/2026)

`mv_hard_code.cpp` (réglage des registres et mouvement prédéfini des 18
servos, récupéré depuis l'Ubuntu du dual boot, version du 02/04/2026) et
`my_read_write.cpp` (aller-retour sur les servos 1 et 2) ont été retirés :
`leg_server`, l'interface et `test_torque` les remplacent. Ce qu'ils
apportaient est repris :
- les réglages de compliance et de punch (§2.3, §2.4) ;
- le sens des servos « inversés » (0-2, 12-14, 15-17) = pattes de droite (§3.1).

Ils restent disponibles :
- dans l'historique git : `git show daf52c3:Dyna_test/src/mv_hard_code.cpp`
  (version d'origine) et `git show 14d4b57:Dyna_test/src/mv_hard_code.cpp`
  (version corrigée, avec `/dev/u2d2`) ;
- dans la copie brute sur le PC : `Backup\Ubuntu_dualboot\Dyna_test`.

### 1.5 Réglages au démarrage

`leg_server` exécute au lancement les commandes de `Dyna_test/leg_settings.txt`
(une commande du protocole par ligne), car les réglages de compliance, punch,
vitesse et limite de couple des AX sont en RAM et perdus hors tension. Voir
§2.4.

### 1.6 DynamixelSDK sur la Pi

Bibliothèque C++ officielle ROBOTIS, compilée depuis la copie du PC
(`Backup\CODE_Dyna\DynamixelSDK`, dossier `c++/build/linux_sbc`) et installée :

| Élément | Emplacement |
|---|---|
| bibliothèque | `/usr/local/lib/libdxl_sbc_cpp.so` |
| en-têtes | `/usr/local/include/dynamixel_sdk/` |
| tables de contrôle | `/usr/local/share/dynamixel_sdk/control_table` |

Réinstaller (depuis le PC, dans `Backup\CODE_Dyna`) :

```bash
tar czf - --exclude="c++/example" DynamixelSDK/c++ DynamixelSDK/control_table DynamixelSDK/LICENSE \
  | ssh hexapode 'rm -rf /tmp/DynamixelSDK && tar xzf - -C /tmp'
ssh hexapode
cd /tmp/DynamixelSDK/c++/build/linux_sbc
make && sudo make install          # installe dans /usr/local + ldconfig
```

Sur un PC Linux x86_64 : même chose avec `c++/build/linux64`
(→ `libdxl_x64_cpp.so`).

### 1.7 Port fixe `/dev/u2d2` (règle udev)

Sans réglage, l'U2D2 prend le nom `/dev/ttyUSB0`, `/dev/ttyUSB1`… selon
l'ordre de branchement des adaptateurs USB-série. La règle
`/etc/udev/rules.d/99-u2d2.rules` (copie dans `Dyna_test/udev/`) :
- reconnaît l'U2D2 par sa puce FTDI FT232H (`0403:6014`) et crée le lien
  **`/dev/u2d2`**, quel que soit le port USB ;
- donne l'accès au groupe `dialout` ;
- règle la latence USB de l'adaptateur à **1 ms** au lieu de 16 ms
  (recommandation ROBOTIS, échanges plus rapides).

U2D2 vu sur la Pi : `FTDI USB <-> Serial Converter`, n° de série `FT8ISGAS`.

```bash
ls -l /dev/u2d2                                  # -> ttyUSB0 (ou autre)
cat /sys/bus/usb-serial/devices/*/latency_timer  # -> 1
```

Installer la règle sur une autre machine Linux :

```bash
sudo cp Dyna_test/udev/99-u2d2.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

> La règle vise tout adaptateur FT232H. Si un jour un autre appareil FT232H
> est branché en même temps, ajouter `ATTRS{serial}=="FT8ISGAS"` à la règle
> pour ne viser que cet U2D2.

---

## 2. Pilotage d'une patte : `leg_server` + interface graphique (voir aussi §3)

Une patte = 3 servos : **ID 12 = coxa** (hanche, rotation horizontale),
**ID 13 = fémur**, **ID 14 = tibia**. Servos détectés : AX-12A (modèle 12).

```
 PC Windows                                   Raspberry Pi
 gui/leg_gui.py  ── TCP 5555 (texte) ──►  bin/leg_server ── /dev/u2d2 ──► servos 12/13/14
 (dessin, curseurs, réglages, pas)        (registres Dynamixel)
```

### 2.1 Lancer

Sur la Pi (laisser tourner) :

```bash
ssh -i ~/.ssh/id_ed25519_hexapode hexa-pi@192.168.137.10
cd ~/Hexapode_ECE/Dyna_test && make && ./bin/leg_server
#   ./bin/leg_server 12 13 14 5555   (autres IDs / port)
```

Sur le PC (Python 3, Tkinter inclus, rien à installer) :

```bash
cd Dyna_test/gui
python leg_gui.py                 # se connecte à 192.168.137.10:5555
python leg_gui.py <ip> [port]
```

`leg_server` accepte **jusqu'à 4 clients en même temps** : l'interface peut
rester ouverte pendant `test_torque` (elle affiche le robot en direct). Les
commandes sont traitées une par une, donc jamais deux accès simultanés au bus.
Éviter quand même de bouger les curseurs pendant un essai.

`Ctrl+C` sur `leg_server` **coupe le couple** des 3 servos avant de quitter.

**Reconnexion automatique de l'U2D2.** Quand les servos bougent, l'U2D2 peut
décrocher de l'USB et revenir 0,3 s plus tard sous un autre nom
(`ttyUSB0` → `ttyUSB1`), constaté le 25/11/2026 :

```
ftdi_sio ttyUSB0: usb_serial_generic_write_bulk_callback - nonzero urb status: -71
usb 1-1.4: USB disconnect ... FT232H ... now attached to ttyUSB1
```

`leg_server` rouvre alors `/dev/u2d2` tout seul (si sa cible change, ou après
~1 s sans aucune réponse des servos) ; l'interface l'affiche dans sa barre
d'état. Testé en débranchant/rebranchant l'U2D2 logiciellement : servos
retrouvés sans intervention. Voir les déconnexions :
`journalctl -k | grep -E "ftdi|USB disconnect"`.

Cause probable : parasites quand les servos tirent du courant (masse / câble
USB). Pistes : câble USB court et de bonne qualité, U2D2 branché directement
sur la Pi (pas de hub), alimentation des servos via un U2D2 Power Hub ou une
alimentation qui tient les pics, masse commune propre, ferrite sur le câble USB.

### 2.2 L'interface

- **Vue de dessus** (rotation de la coxa) et **vue de côté** (fémur + tibia) :
  trait plein = position réelle lue sur les servos, pointillés = consigne.
  Dessin approximatif d'après le modèle SolidWorks *Hexapod Bas1ma Haz1r* :
  coxa ≈ 50 mm, fémur ≈ 75 mm, tibia ≈ 125 mm (modifiables).
- **Butées en rouge** (case « Afficher butées (rouge) » sous les dessins,
  mémorisée) : pour chaque servo, une ligne CW et une ligne CCW (registres
  6 / 8, lus sur le servo) partant de l'axe de l'articulation : coxa (12) en
  vue de dessus, fémur (13) et tibia (14) en vue de côté ; pour le tibia la
  ligne suit la direction du bout de patte (courbure incluse).
- **Par servo** : curseur de consigne en degrés, couple ON/OFF, lecture
  (position, charge %, température, tension, en mouvement), réglages, butées.
- **ARRÊT COUPLE** (rouge) : couple OFF sur les 3 servos, arrête le pas.
- **RESET SÉCURITÉ** (orange, + un bouton par servo, actif seulement quand le
  servo est en sécurité) : après une alarme (surcharge, surchauffe, tension…),
  un AX coupe son couple **et met sa limite de couple (reg. 34) à 0** — il
  reste mou même si on réactive le couple, et les AX n'ont pas d'instruction
  « reboot ». Le reset : consigne = position actuelle (pas de saut), limite de
  couple = couple max (reg. 14), couple ON. La cause est affichée en clair
  (`SECURITE : surcharge -> Reset`). Testé le 25/11/2026 sur le servo 14 en
  surcharge (erreur `0x20`, limite de couple 0) : remis en service sans mouvement.
- **Offset (décalage du montage)** = angle de l'articulation quand le servo
  est à 512. Il s'applique partout : curseur, lecture, butées et dessin
  affichent l'angle de l'**articulation**, pas celui du servo. Réglable dans
  chaque panneau de servo (ligne **Offset** : saisie + Entrée, boutons
  −90 / −1 / +1 / +90, **Zéro ici** = la position actuelle devient 0°) ou
  dans « dec. » du cadre Géométrie ; sauvegardé automatiquement.
  Servo 14 : offset **−181°** au 25/11/2026 (−82° alignement mesuré à 793,
  +9° moteur, −90° car il affichait 90° de trop).
  Changer un offset ne fait pas bouger le servo (la consigne est
  ré-exprimée avec le nouveau décalage).
- **Courbure du tibia** : le tibia du modèle est cintré, le bout de patte
  n'est pas dans l'axe du servo du genou. Réglage « courbure tibia (deg) » =
  angle entre l'axe du genou et la droite genou → bout de patte (défaut −30°,
  à ajuster sur la vraie patte). **Décalage du bout de patte** (Géométrie,
  « bout de patte (mm) ») : déplace le bout sans changer la courbure, dans le
  repère du tibia — « le long » = dans l'axe du genou (+ = plus loin),
  « travers » = perpendiculaire (+ = vers le haut quand la patte est
  horizontale). Le tibia est dessiné courbe et la position du
  bout de patte est affichée (distance à l'axe de la coxa, hauteur, en mm).
- **Mouvements** : position neutre (512, avec confirmation), « consigne =
  position actuelle », et **pas en boucle** (lever → avancer en l'air → poser
  → pousser au sol) **autour de la pose de départ**, avec amplitude (%) et durée
  d'étape réglables. À l'arrêt, la patte revient à la pose de départ.
- À la connexion, les curseurs prennent la position actuelle : **aucune
  consigne n'est envoyée** tant qu'on ne touche à rien.
- Réglages d'affichage (longueurs, décalages, servos inversés) et écarts du pas
  sauvegardés dans `gui/leg_gui_config.json` (non versionné).

> ⚠️ Sur les AX, **écrire une consigne active le couple** automatiquement :
> bouger un curseur suffit à mettre le servo sous couple.

### 2.3 « PID » sur les AX-12A / AX-18F

Les AX **n'ont pas de PID** (réservé aux séries MX/X). L'équivalent est la
**compliance** :

| Réglage | Registre | Effet |
|---|---|---|
| Marge CW / CCW | 26 / 27 | zone morte autour de la consigne (0–255) ; petite = précis |
| Pente CW / CCW | 28 / 29 | souplesse : 2, 4, 8, 16, 32, 64, 128 ; **petite = raide**, grande = souple |
| Punch | 48 | courant minimal appliqué dès qu'il y a un écart (défaut 32) |
| Limite de couple | 34 | couple max en RAM (0–1023), remis à « couple max » (reg. 14) à l'allumage |
| Vitesse | 32 | vitesse de déplacement (0 = maximum, 1–1023) |
| Butées CW / CCW | 6 / 8 | **EEPROM** : le servo refuse d'aller au-delà (conservé hors tension) |

À l'état actuel (lu le 25/11/2026) : marges 1/1, pentes 32/32, punch 32,
limite de couple 1023, vitesse 0 (max), **butées 0 / 1023 = aucune butée**.
→ Régler des butées adaptées à la mécanique avant les essais (bouton
« Appliquer butées (EEPROM) »), et une vitesse modérée (~150–300).

### 2.4 Réglages pour soulever le robot (fémur 13 / tibia 14)

Essais du 25/11/2026 : robot posé, bout de la patte 12/13/14 au sol ; pour
chaque combinaison (appliquée à 13 et 14 : vitesse 150, limite de couple
1023, marge 1), fémur abaissé de 50 unités (≈ 15°, 512 → 462) pour soulever
le corps, mesures pendant 2,5 s, retour à la pose de départ.

| Pente | Punch | Atteint | Écart 13 / 14 (unités) | Oscillation | Maintien 13 |
|---|---|---|---|---|---|
| 32 (usine) | 32 | **non** | +7,9 / +4,1 | 0,3 | 24 % |
| 16 | 32 | 0,26 s | +3,0 / +4,0 | 0,4 | 16 % |
| 16 | 64 | 0,26 s | +3,0 / +3,4 | 0,5 | 19 % |
| 8 | 64 | 0,20 s | +2,6 / +1,0 | 0,8 | 30 % |
| **8** | **100** | **0,20 s** | **+1,0 / +1,0** | **0,0** | **22 %** |
| 4 | 64 | 0,19 s | +1,0 / +0,7 | 0,5 | 31 % |

Aucune alarme, température 47–50 °C. **Retenu : pente 8 / punch 100** (même
précision que pente 4 mais sans oscillation et moins de charge, donc moins
de chauffe). Avec le réglage d'usine le poids fait plier la patte de ~2,3°.

Ces registres sont en **RAM** sur les AX (perdus hors tension) :
`leg_server` les réapplique à chaque démarrage depuis
**`Dyna_test/leg_settings.txt`** (une commande du protocole par ligne,
`#` = commentaire). Modifier ce fichier puis relancer `leg_server` pour
changer les réglages par défaut.

### 2.5 Protocole `leg_server` (TCP, une commande par ligne)

| Commande | Effet |
|---|---|
| `PING`, `SCAN` | test ; liste des servos qui répondent (`OK 0:1 1:1 … 17:1`) |
| `IDS id id …` | remplace la liste des servos gérés (défaut 0–17), puis SCAN |
| `STATE` | `OK {json}` : position, consigne, vitesse, charge, tension, température, couple et tous les réglages de chaque servo (adresses 0–49 en un paquet) ; les servos absents ne sont pas relus (un seul re-testé par STATE) |
| `GOAL id pos` | consigne 0–1023 d'un servo |
| `GOALS p0 p1 …` | consignes de tous les servos, dans l'ordre de la liste (SyncWrite) |
| `GOALSID id pos id pos …` | consignes d'un groupe de servos (SyncWrite) |
| `TORQUE id 0/1`, `ESTOP` | couple d'un servo / couple OFF partout |
| `RESET [id\|ALL]` | sortie du mode sécurité (consigne = position, limite de couple = couple max, couple ON) |
| `SPEED`, `TLIMIT`, `PUNCH id v` | vitesse, limite de couple, punch |
| `MARGIN`, `SLOPE id cw ccw` | compliance |
| `LIMITS id cw ccw` | butées (EEPROM), refuse cw ≥ ccw |
| `READ id addr len`, `WRITE id addr len val` | accès brut (ID et baudrate protégés) |

Test rapide depuis n'importe quel terminal : `nc 192.168.137.10 5555` puis
taper `SCAN` ou `STATE`.

> ⚠️ Sur les AX, quand le couple est **coupé**, le registre de charge (40)
> garde une valeur figée (souvent ±1023) : l'interface affiche alors
> « charge -- (OFF) » et `test_torque` l'ignore.

---

## 3. Robot complet : 6 pattes et `test_torque`

Depuis le 25/11/2026, `leg_server` gère **les 18 servos** (IDs 0–17 par
défaut ; `./bin/leg_server 12 13 14` pour n'en gérer que certains). Les 18
répondent sur le bus.

### 3.1 Configurer les pattes (interface)

- **Vue « Robot vu de dessus »** : le corps et les 6 pattes en position
  réelle (gris pointillé = servos absents, rouge = alarme, bleu = patte
  sélectionnée). **Clic sur une patte** = la sélectionner : ses 3 servos
  passent dans les panneaux, sans aucun mouvement.
- **Cadre « Configuration des pattes »** : pour chaque patte, nom, **IDs
  coxa / fémur / tibia** (modifiables), position du montage sur le corps
  (x vers l'avant, y vers la gauche, mm) et angle de montage (0 = avant,
  90 = gauche). Le sens et l'offset de chaque servo se règlent dans les
  panneaux (case « Monté inversé », ligne Offset) pour la patte sélectionnée.
- **Cadre « Cinématique »** : position du bout de patte (angle de coxa,
  portée r, hauteur z) ; avec « Piloter par la position du bout de patte »,
  les curseurs calculent les angles des 3 servos (cinématique inverse,
  tibia cintré compris). « Depuis la position actuelle » recale les curseurs
  sans bouger.

Disposition des pattes (IDs donnés le 25/11/2026 : triplets en tournant dans
le sens trigo à partir de l'avant droite) :

| Patte | Position | IDs coxa / fémur / tibia | Sens (coxa, fémur, tibia) |
|---|---|---|---|
| 1 | avant gauche | 3 / 4 / 5 | normal, inversé, inversé |
| 2 | avant droite | 0 / 1 / 2 | inversé, normal, normal |
| 3 | milieu droite | 15 / 16 / 17 | inversé, normal, normal |
| 4 | arrière droite | 12 / 13 / 14 (calibrée) | inversé, normal, normal |
| 5 | arrière gauche | 9 / 10 / 11 | normal, inversé, inversé |
| 6 | milieu gauche | 6 / 7 / 8 | normal, inversé, inversé |

Côté droit = 0-2, 12-14, 15-17 (les servos « inversés » de l'ancien `mv_hard_code`, §1.4) :
même sens que la patte 4 calibrée. Côté gauche : sens opposé — **hypothèse à
vérifier patte par patte** (case « Monté inversé ») ; offsets à 0 sauf
réglage.

Tout est sauvegardé dans `gui/leg_gui_config.json` (format 6 pattes ;
l'ancienne config une patte est reprise automatiquement). Le code commun
(config, conversions, cinématique directe / inverse) est dans
`gui/hexapod_model.py`.

### 3.2 `test_torque.py` : faire bouger tout le robot

```bash
cd Dyna_test/gui
python test_torque.py --dry-run        # lit le robot, calcule et verifie tout, ne bouge rien
python test_torque.py                  # essai reel (confirmation demandee)
python test_torque.py --lift 20 --skip-move --skip-tripod
```

Déroulé (à partir de la pose actuelle du robot, posé au sol) :

1. **Vérifications** : 18 servos présents, aucun en alarme, température
   < 62 °C ; **toutes les poses du plan sont calculées et vérifiées
   (atteignables, dans les butées) avant le moindre mouvement**.
2. **Réglages** : fémurs / tibias pente 8, punch 100 (§2.4), vitesse 100 ;
   consigne = position actuelle puis couple ON (aucun saut).
3. **Soulever** : tous les bouts de patte descendent par pas de 5 mm jusqu'à
   `--lift` (30 mm) → le corps monte ; arrêt de la montée si un servo dépasse
   `--max-load` (85 %) plus de 0,6 s.
4. **Tenir** `--hold` s (3 s).
5. **Déplacer le corps** de `--shift` mm (20) avant / arrière / gauche /
   droite (tous les bouts de patte glissent ensemble).
6. **Trépieds** : 3 pattes sur 2 (avant droite, arrière droite, milieu
   gauche, puis les 3 autres) se lèvent de `--tripod` mm (20), les 3 autres
   portent le robot.
7. **Reposer** progressivement à la pose de départ.

Sécurité : arrêt immédiat (consignes figées, **couple gardé** pour ne pas
lâcher le robot) si un servo passe en alarme, dépasse `--temp-max`, si une
pose est hors d'atteinte / hors butées, ou sur Ctrl+C. Mesures (charge max
et servo concerné, charge moyenne des fémurs, écart de position,
température, à chaque pas) enregistrées dans `gui/logs/test_torque_*.json`.

**Calculs de couple, essais et résultats : [`Torque_calcul.md`](Torque_calcul.md).** Le 25/11/2026, le robot s'est levé à 6 pattes (49 poses atteintes, 17 % de charge moyenne des fémurs debout).

Avant le premier essai réel : configurer les 6 pattes dans l'interface
(IDs, sens, offsets, montage), régler des butées, et vérifier le plan avec
`--dry-run`.
