# `leg_server_bridge` — pont ROS → vrai robot

Fait bouger les 18 servos à partir de `/joint_states` (publié par
[`gait_node`](gait_node.md)) en passant par le `leg_server` de la Pi
([`Dyna_test/dyna.md`](../../../../Dyna_test/dyna.md) §2). **Provisoire** : il sera
remplacé par le plugin ros2_control `hexapod_hardware` (phase P2 de
[`tps_archi.md`](../../../../tps_archi.md)).

Code : [`hexapod_control/leg_server_bridge.py`](../hexapod_control/leg_server_bridge.py).

```
/joint_states ─► leg_server_bridge ──TCP 5555──► leg_server (Pi) ──► U2D2 ──► 18 × AX-12A
  (rad)          radians → positions    GOALSID 25 Hz
                 (sens, offsets)        STATE 5 Hz (sécurité)
```

---

## Lancer

```bash
ros2 launch hexapod_control hexapod.launch.py robot:=dry    # vérifie, N'ENVOIE RIEN
ros2 launch hexapod_control hexapod.launch.py robot:=true   # LE ROBOT BOUGE
```

ou seul (avec `gait_node` déjà lancé) :

```bash
ros2 run hexapod_control leg_server_bridge                                        # simulation
ros2 run hexapod_control leg_server_bridge --ros-args -p send:=true -p ramp_time:=8.0
```

**Avant** : `leg_server` tourne sur la Pi (`ssh hexapode "pgrep -a leg_server"`),
batterie chargée (≥ 11 V), robot posé au sol, quelqu'un à côté de l'alimentation.
La calibration (`Dyna_test/gui/leg_gui_config.json` → `legs.yaml`) doit être à jour.

---

## Déroulé

1. **Connexion** au `leg_server`, lecture de l'état : les 18 servos doivent répondre,
   aucun en alarme, température < `temp_max`. Sinon le nœud s'arrête.
2. Avec `send:=true` : **réglages** (vitesse `speed`, limite de couple max, compliance
   pente 8 / punch 100 sur fémurs et tibias), consigne = **position actuelle**, puis
   **couple ON** (aucun saut).
3. **Rampe** : pendant `ramp_time` s, la consigne passe de la position réelle à la pose
   demandée par la marche (toutes les pattes ensemble : le robot se lève).
4. **Suivi** : chaque `/joint_states` est converti et envoyé à 25 Hz avec `GOALSPD` :
   position **et vitesse** de chaque servo, la vitesse étant celle qu'il faut pour arriver
   pile à la consigne suivante (× `speed_gain`). Le servo glisse d'une consigne à l'autre
   au lieu de foncer puis s'arrêter : mouvement fluide, et la vitesse demandée au robot est
   bien suivie. Toutes les 2 s : écart consigne / position, charge max, tension.
5. **Compliance selon la phase** (`/leg_phases` de `gait_node`), par articulation
   [coxa, fémur, tibia] :

   | Patte | Pente | Punch | Pourquoi |
   |---|---|---|---|
   | **au sol** (porte le robot) | 32 / 8 / 8 | 32 / 100 / 100 | fémur et tibia raides pour tenir le poids ; la coxa ne porte pas |
   | **en l'air** | 64 / 128 / 64 | 16 / 16 / 24 | souple, petit punch : pas de tremblement des servos du haut de la patte |

   Changement envoyé en 2 `SYNCW` seulement quand une patte change de phase.

Avec un ancien `leg_server` (sans `GOALSPD` / `SYNCW`), le pont prévient et revient à
`GOALSID` avec vitesse et compliance fixes.

Conversion (comme `hexapod_model.joint_to_raw`) :
`raw = 512 + (±(angle − offset)) / (300/1024 °)`, signe `−` si le servo est monté
inversé (`inverted` dans `legs.yaml`).

---

## Sécurité : réparer d'abord, geler en dernier recours

Toutes les 0,2 s, le pont lit l'état des servos (`STATE`).

**Remise en route automatique** (`RESET` du servo, puis il reprend la marche) quand un
servo :

| Cas | Seuil |
|---|---|
| est en alarme (surcharge, tension, hors plage), limite de couple à 0 ou couple coupé | immédiat |
| est **bloqué loin de sa consigne** (décalage simulation / réalité) | écart > `stuck_err` (60 unités ≈ 18°) pendant plus de `stuck_time` (1 s) |

Une charge > `max_load` (85 %) n'est plus qu'un avertissement : si elle dure, le servo se
met lui-même en sécurité et il est remis en route.

**Gel** (plus aucune consigne, **le couple reste actif** : le robot n'est pas lâché) :

| Cause | Seuil |
|---|---|
| trop de remises en route (le problème n'est pas passager) | plus de `max_resets` (6) en `reset_window` (30 s) |
| température | ≥ `temp_max` (62 °C) |
| servo qui ne répond plus (USB de l'U2D2 qui décroche : on attend d'abord) | plus de `absent_time` (2 s) |
| pose demandée par la marche hors des butées EEPROM | butées lues au démarrage |
| liaison TCP perdue | |
| Ctrl+C | |

Au démarrage, une position réelle un peu au-delà d'une butée (robot affaissé) est ramenée
dans les butées pour la rampe (message `positions hors butees au depart`), sans gel.
Après un gel : relancer (il repart de la position réelle, avec la rampe).

**Remise en route manuelle** (servos bloqués, en sécurité, couple coupé ou pont gelé) :
touche **`r`** de la fenêtre « Actions hexapode », ou

```bash
ros2 service call /reset_servos std_srvs/srv/Trigger
```

`RESET ALL` des servos, puis rampe douce (`reset_ramp_time` 3 s) de la position réelle vers
la pose de la marche ; le gel et les compteurs sont effacés.

**Robot porté en l'air** : si la charge moyenne des fémurs au sol reste sous `air_load`
(6 %) pendant `air_time` (0,6 s), toutes les pattes passent souples et sans punch
(`air_slope`, `air_punch`) et `/robot_lifted` passe à vrai (la marche met sa stabilisation
en pause) ; retour à la normale au-dessus de `ground_load` (12 %).

**Couper le couple** (robot relâché, il s'affaisse) :

```bash
nc 192.168.137.10 5555      # puis taper : ESTOP
```

Sortir un servo du mode sécurité : `RESET ALL` dans la même connexion
(voir [`dyna.md`](../../../../Dyna_test/dyna.md) §2.5).

---

## Paramètres (`config/bridge.yaml`)

| Paramètre | Défaut | Rôle |
|---|---|---|
| `send` | `false` | `true` = envoie aux servos (fixé par `robot:=true` au lancement) |
| `host`, `port` | `192.168.137.10`, 5555 | `leg_server` |
| `rate` | 25 Hz | fréquence d'envoi (lecture d'état à `rate` / 5) |
| `speed` | 100 | vitesse des servos au démarrage (et si `leg_server` ancien) |
| `speed_gain`, `min_speed` | 1,5 ; 30 | vitesse envoyée = vitesse nécessaire × gain, au moins `min_speed` |
| `stance_slope`, `stance_punch` | [32, 8, 8] ; [32, 100, 100] | compliance [coxa, fémur, tibia] d'une patte au sol |
| `swing_slope`, `swing_punch` | [64, 128, 64] ; [16, 16, 24] | compliance d'une patte en l'air |
| `stuck_err`, `stuck_time` | 60 ; 1 s | servo bloqué → remise en route |
| `max_resets`, `reset_window` | 6 ; 30 s | au-delà : gel |
| `absent_time` | 2 s | servo absent avant gel |
| `ramp_time` | 8 s | durée du lever au démarrage |
| `max_load` | 85 % | avertissement de charge |
| `temp_max` | 62 °C | gel en surchauffe |
| `legs_yaml` | (vide) | autre `legs.yaml` |

---

## Points à connaître

- **Un seul maître** : le pont et `test_torque` / `leg_gui` partagent le `leg_server` ;
  ne pas faire bouger le robot depuis deux programmes en même temps.
- **Sens des servos** : si une patte part à l'envers, c'est son `inverted` dans la
  calibration. Test du 25/11/2026 : les coxas des pattes 1 et 2 sont inversées, celles
  de 3 à 6 non (vérifié en faisant tourner chaque coxa de +20° et en regardant le pied).
- **L'USB de l'U2D2 décroche** parfois (voir `test_torque.md`) : `leg_server` se
  reconnecte, mais un `STATE` raté pendant ce temps provoque un gel.
