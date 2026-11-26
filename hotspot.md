# Sans câble : hotspot Wi-Fi de la Raspberry Pi

La **Pi crée un réseau Wi-Fi** (`Hexapode-ECE`) dès qu'elle démarre ; le PC s'y connecte
quand on veut piloter sans câble. Tout ce qui marche par le câble marche alors par le
Wi-Fi : `ssh`, ROS 2 (CycloneDDS), Foxglove, `hexapode.sh`, `test_torque`, `leg_gui`.

**Le câble Ethernet reste prioritaire** : quand il est branché, tout passe par lui,
exactement comme avant (le hotspot de la Pi reste allumé mais n'est pas utilisé).

**Pas besoin de clé Wi-Fi** : la puce Wi-Fi intégrée de la Pi 4 (Broadcom/Cypress) sait
faire point d'accès. Une clé USB ne servirait que côté **PC**, pour garder internet
pendant qu'il est sur le hotspot (§4).

Documents liés : [setup.md](setup.md) (câble, SSH), [ros2_ws/README.md](ros2_ws/README.md)
(lancement), [tps_archi.md](tps_archi.md) §7.1 (réseau ROS).

---

## 1. Résumé

| | Câble Ethernet (prioritaire) | Hotspot de la Pi |
|---|---|---|
| Réseau | `192.168.137.0/24` | `10.42.0.0/24` |
| PC | `192.168.137.1` (connexion NetworkManager `hexapode-share`, partage internet) | `10.42.0.1` fixe (connexion `hexapode-pi`, client Wi-Fi) |
| Pi | `192.168.137.10` (`eth0`) | `10.42.0.10` (`wlan0`, connexion `hexapode-ap`, point d'accès) |
| SSH | `ssh hexapode` | `ssh hexapode-wifi` |
| Wi-Fi | | SSID **`Hexapode-ECE`**, WPA2, canal 6 (2,4 GHz) |
| Internet | PC : Wi-Fi de l'école ; Pi : partagé par le PC | PC : **aucun** (sauf §4) ; Pi : aucun |

```
        ┌──────────── PC ─────────────┐                                 ┌──── Raspberry Pi ─────┐
        │ enx… : 192.168.137.1        │ ───── câble (prioritaire) ───── │ eth0 : 192.168.137.10 │
        │ wlp0s20f3 : client          │ ((((((( Wi-Fi "Hexapode-ECE" ((( │ wlan0 : point d'accès │
        │   10.42.0.1 (hexapode-pi)   │                                 │   10.42.0.10 (+DHCP)  │
        └─────────────────────────────┘                                 └───────────────────────┘
```

---

## 2. Utilisation

### En une commande

```bash
~/Hexapode2026/Hexapode_ECE/ros2_ws/hexapode.sh robot-distance     # (ou dry-distance pour vérifier)
```

1. si le **câble** est branché et que la Pi répond : liaison par câble, rien d'autre ;
2. sinon le PC **rejoint le hotspot de la Pi** (`nmcli connection up hexapode-pi`) et
   quitte le Wi-Fi de l'école ;
3. attend que la Pi réponde sur `10.42.0.10` (60 s max) ;
4. lance `leg_server` sur la Pi s'il ne tourne pas ;
5. force la liaison Wi-Fi pour ROS (`HEXAPODE_LINK=wifi`), puis lance tout comme
   `hexapode.sh robot` (marche, pont, Foxglove, Teleop).

**Ctrl+C** : tout s'arrête (robot figé, couple gardé), puis le PC **quitte le hotspot de
la Pi et revient sur le Wi-Fi utilisé avant** (mémorisé au départ), sinon sur un autre
réseau connu visible (du plus récent au plus ancien) ; le script attend
qu'internet réponde (30 s max) et affiche le réseau retrouvé. Seulement cette étape :
`./hexapode.sh internet`.

### À la main

```bash
nmcli connection up hexapode-pi            # PC -> hotspot de la Pi
ping -c 2 10.42.0.10
ssh hexapode-wifi hostname                 # -> hexapode
~/Hexapode2026/Hexapode_ECE/ros2_ws/hexapode.sh robot
nmcli connection down hexapode-pi          # retour au Wi-Fi de l'école
```

`hexapode.sh` (via `hexapode_env.sh`) choisit tout seul la liaison : câble s'il a une
adresse `192.168.137.x`, sinon Wi-Fi `10.42.0.x`, sinon ROS en local (simulation). Il
affiche `Liaison PC <-> Pi : …`. CycloneDDS n'utilise que les adresses de la liaison
active. **Relancer / ouvrir un nouveau terminal après avoir changé de liaison.**

Autres outils en Wi-Fi :

```bash
ssh hexapode-wifi "pgrep -a leg_server"
python3 Dyna_test/gui/test_torque.py --host 10.42.0.10 --dry-run
nc 10.42.0.10 5555                                           # puis ESTOP pour couper le couple
```

Dans `leg_gui`, mettre l'adresse `10.42.0.10` (enregistrée dans `leg_gui_config.json` :
la remettre à `192.168.137.10` pour le câble).

---

## 3. Ce qui a été configuré (26/11/2026)

### Raspberry Pi (point d'accès)

```bash
# pays Wi-Fi (sans lui le Wi-Fi reste bloque) et Wi-Fi active
echo "options cfg80211 ieee80211_regdom=FR" | sudo tee /etc/modprobe.d/cfg80211-fr.conf
sudo iw reg set FR
sudo rfkill unblock wifi
sudo nmcli radio wifi on

# point d'acces "Hexapode-ECE", demarre tout seul, adresse 10.42.0.10, DHCP pour les clients
sudo nmcli connection add type wifi ifname wlan0 con-name hexapode-ap autoconnect yes \
  connection.autoconnect-priority 20 connection.autoconnect-retries 0 \
  ssid Hexapode-ECE 802-11-wireless.mode ap 802-11-wireless.band bg 802-11-wireless.channel 6 \
  ipv4.method shared ipv4.addresses 10.42.0.10/24 ipv6.method disabled \
  wifi-sec.key-mgmt wpa-psk wifi-sec.psk '<mot de passe>' \
  wifi-sec.proto rsn wifi-sec.pairwise ccmp wifi-sec.group ccmp
```

- Les autres réseaux Wi-Fi enregistrés sur la Pi (FabLab Makers, Eceborg, freebox…) et
  l'ancien profil client `hexapode-wifi` sont en `autoconnect no` : la carte Wi-Fi reste
  en point d'accès.
- Le câble (`eth0`, `Wired connection 1`) n'est pas modifié.
- Conséquence : la Pi n'a internet que par le câble (`git pull`, `apt`).

### PC (client)

```bash
nmcli connection add type wifi ifname wlp0s20f3 con-name hexapode-pi autoconnect no \
  ssid Hexapode-ECE ipv4.method manual ipv4.addresses 10.42.0.1/24 ipv4.never-default yes \
  ipv6.method disabled wifi-sec.key-mgmt wpa-psk wifi-sec.psk '<mot de passe>'
```

- `autoconnect no` : le PC ne quitte **jamais tout seul** le Wi-Fi de l'école.
- Adresse fixe `10.42.0.1` (celle que la Pi et CycloneDDS attendent) et
  `ipv4.never-default` : le PC n'essaie pas de sortir sur internet par la Pi.
- `~/.ssh/config` : alias `hexapode-wifi` → `10.42.0.10`, même clé et même empreinte
  (`HostKeyAlias hexapode`) que `hexapode`.
- L'ancien hotspot du PC (`hexapode-hotspot`, 25/11/2026) est supprimé.

### Mot de passe

Le même sur les deux machines, il n'est pas dans le dépôt :

```bash
nmcli -s -g 802-11-wireless-security.psk connection show hexapode-pi          # le lire (PC)
nmcli connection modify hexapode-pi wifi-sec.psk 'nouveau'                    # le changer (PC)
ssh hexapode "sudo nmcli connection modify hexapode-ap wifi-sec.psk 'nouveau' && \
              sudo nmcli connection up hexapode-ap"                           # ... et sur la Pi
```

### Dépôt

| Fichier | Rôle |
|---|---|
| `ros2_ws/hexapode_env.sh` | choisit l'interface : câble (`192.168.137.x`), sinon Wi-Fi (`10.42.0.x`), sinon local ; exporte `HEXAPODE_PI` et les pairs CycloneDDS |
| `ros2_ws/src/hexapod_bringup/config/cyclonedds.xml` | pairs = ceux de la liaison active ; avertissements réseau masqués |
| `ros2_ws/hexapode.sh` | `robot-distance` / `dry-distance` : câble si branché, sinon rejoint `hexapode-pi`, attend la Pi, lance `leg_server` |

---

## 3 bis. Tout sans câble, dès l'allumage

Au démarrage de la Pi, **sans rien brancher** :

| Quoi | Comment |
|---|---|
| Hotspot `Hexapode-ECE` | profil `hexapode-ap` en connexion automatique (câble branché ou non ; le câble reste prioritaire côté ROS) |
| `leg_server` (servos) | service systemd `hexapode-leg-server` (relancé s'il s'arrête) |
| Nœud IMU `/imu/data` | service systemd `hexapode-imu` |
| Changement de liaison | dispatcher NetworkManager `90-hexapode-liaison` : câble branché / débranché → le nœud IMU est relancé sur la bonne liaison |

Il suffit donc d'allumer le robot, puis sur le PC : `./hexapode.sh robot-distance`.

Installation (une fois, déjà faite le 26/11/2026) et commandes utiles, sur la Pi :

```bash
sudo sh ~/Hexapode_ECE/ros2_ws/src/hexapod_bringup/systemd/install_pi_services.sh
systemctl status hexapode-leg-server hexapode-imu
journalctl -u hexapode-leg-server -f          # journal du leg_server
sudo systemctl restart hexapode-leg-server    # ATTENTION : coupe le couple un instant
```

Les fichiers sont dans [`ros2_ws/src/hexapod_bringup/systemd/`](ros2_ws/src/hexapod_bringup/systemd).

## 4. Internet sur le PC pendant le Wi-Fi ?

Le PC n'a qu'une carte Wi-Fi : sur le hotspot de la Pi, il n'est plus sur le Wi-Fi de
l'école. Pour garder internet :

1. **Câble** : s'il est branché, inutile de passer en Wi-Fi (il est prioritaire).
2. **Téléphone en partage de connexion USB** branché au PC.
3. **Clé Wi-Fi USB sur le PC** : une carte pour l'école, une pour la Pi
   (`nmcli connection modify hexapode-pi connection.interface-name <clé>`).

Sans internet, tout le pilotage du robot marche quand même ; seuls `git push / pull` et
`apt` attendent le retour sur le Wi-Fi de l'école.

---

## 5. Dépannage

| Symptôme | Vérifier |
|---|---|
| `Hexapode-ECE` n'apparaît pas | Pi allumée depuis > 30 s ; par le câble : `ssh hexapode "nmcli -t -f DEVICE,CONNECTION device"` doit montrer `wlan0:hexapode-ap`, sinon `ssh hexapode "sudo nmcli connection up hexapode-ap"` |
| `wlan0: unavailable` sur la Pi | `sudo rfkill unblock wifi` puis `sudo nmcli radio wifi on` ; pays : `iw reg get` doit afficher `country FR` |
| Le PC n'arrive pas à se connecter | mot de passe identique sur les deux profils (§3) ; `nmcli connection up hexapode-pi` affiche l'erreur |
| ROS : la Pi et le PC ne se voient pas | relancer `hexapode.sh` (ou un nouveau terminal) après le changement de liaison ; `echo $HEXAPODE_IFACE` : `wlp0s20f3` sur le PC, `wlan0` sur la Pi |
| Le pont ne joint pas le `leg_server` | `hexapode.sh` affiche l'adresse utilisée ; `nc -z 10.42.0.10 5555 && echo ok` |
| Saccades / gels plus fréquents en Wi-Fi | latence du Wi-Fi sur les 25 consignes/s : rester près du robot, ou lancer la marche **sur la Pi** (`ssh hexapode-wifi`, puis `ros2 launch hexapod_control hexapod.launch.py robot:=true host:=127.0.0.1`) et ne garder que Foxglove (`ws://10.42.0.10:8765`) et le clavier sur le PC |
| Câble : pertes de paquets (`ping` à 30-70 %), pont qui perd le `leg_server` | le lien était renégocié en 1 Gbit/s sur un câble trop faible : forcé à **100 Mbit/s** sur le PC (`nmcli connection modify hexapode-share 802-3-ethernet.auto-negotiate yes 802-3-ethernet.speed 100 802-3-ethernet.duplex full`, puis `nmcli connection up hexapode-share`) → 0 % de perte (26/11/2026) |
| Le PC n'a plus internet | normal (§4) : `nmcli connection down hexapode-pi` |
