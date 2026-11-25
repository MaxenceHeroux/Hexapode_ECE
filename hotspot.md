# Sans câble : hotspot Wi-Fi du PC → Raspberry Pi

Le PC (Ubuntu) crée un réseau Wi-Fi ; la Pi s'y connecte toute seule. Tout ce qui marche
par le câble marche alors par le Wi-Fi : `ssh`, `git pull`, ROS 2 (CycloneDDS),
Foxglove, `hexapode.sh robot`, `test_torque`, `leg_gui`.

Documents liés : [setup.md](setup.md) (câble, SSH), [ros2_ws/README.md](ros2_ws/README.md)
(lancement), [tps_archi.md](tps_archi.md) §7.1 (réseau ROS).

---

## 1. Résumé

| | Câble Ethernet (existant) | Hotspot Wi-Fi (nouveau) |
|---|---|---|
| Réseau | `192.168.137.0/24` | `10.42.0.0/24` |
| PC | `192.168.137.1` (connexion NetworkManager `hexapode-share`) | `10.42.0.1` (connexion `hexapode-hotspot`) |
| Pi | `192.168.137.10` (`eth0`) | `10.42.0.10` (`wlan0`, connexion `hexapode-wifi`) |
| SSH | `ssh hexapode` | `ssh hexapode-wifi` |
| Wi-Fi | | SSID **`Hexapode-ECE`**, WPA2, canal 6 (2,4 GHz) |
| Internet de la Pi | partagé par le PC | partagé par le PC (si le PC a internet, voir §4) |

Les deux liaisons peuvent coexister : le **câble est prioritaire** quand il est branché.

```
        ┌──────────── PC ─────────────┐      Wi-Fi "Hexapode-ECE"       ┌──── Raspberry Pi ────┐
        │ wlp0s20f3 : point d'accès   │ ))))))))))))))))))))))))))))))) │ wlan0 : 10.42.0.10   │
        │ 10.42.0.1 (DHCP + partage)  │                                 │ (fixe + DHCP)        │
        │ enx… : 192.168.137.1        │ ────────── câble (option) ───── │ eth0 : 192.168.137.10│
        └─────────────────────────────┘                                 └──────────────────────┘
```

---

## 2. Utilisation

### Allumer le hotspot (PC)

```bash
nmcli connection up hexapode-hotspot       # le PC devient point d'accès "Hexapode-ECE"
```

⚠️ **Le PC n'a qu'une carte Wi-Fi** : en mode point d'accès, elle **quitte le Wi-Fi de
l'école**. Le PC perd internet (et Claude Code, `git push`… aussi), sauf s'il a une autre
connexion (téléphone en partage USB, câble réseau, clé Wi-Fi USB) : voir §4.

La Pi s'y connecte toute seule en quelques secondes (profil `hexapode-wifi`, connexion
automatique). Vérifier :

```bash
ping -c 2 10.42.0.10
ssh hexapode-wifi hostname                 # -> hexapode
```

### Travailler comme avant

```bash
~/Hexapode2026/Hexapode_ECE/ros2_ws/hexapode.sh robot
```

`hexapode.sh` (via `hexapode_env.sh`) choisit tout seul la liaison : il affiche
`Liaison PC <-> Pi : wlp0s20f3, Pi (leg_server) en 10.42.0.10`. CycloneDDS connaît les
adresses de la liaison active (`ros2_ws/src/hexapod_bringup/config/cyclonedds.xml`). **Ouvrir un nouveau terminal / relancer après avoir changé de liaison** (câble ↔ Wi-Fi).

Autres outils, en remplaçant l'adresse :

```bash
ssh hexapode-wifi "pgrep -a leg_server"                       # leg_server tourne ?
python3 Dyna_test/gui/test_torque.py --host 10.42.0.10 --dry-run
nc 10.42.0.10 5555                                           # puis ESTOP pour couper le couple
```

Dans `leg_gui`, mettre l'adresse `10.42.0.10` (elle est enregistrée dans
`leg_gui_config.json` : la remettre à `192.168.137.10` pour le câble).

### Éteindre le hotspot

```bash
nmcli connection down hexapode-hotspot     # le PC revient sur le Wi-Fi de l'école
```

---

## 3. Ce qui a été configuré (25/11/2026)

### PC

```bash
nmcli connection add type wifi ifname wlp0s20f3 con-name hexapode-hotspot autoconnect no \
  ssid Hexapode-ECE 802-11-wireless.mode ap 802-11-wireless.band bg 802-11-wireless.channel 6 \
  ipv4.method shared ipv4.addresses 10.42.0.1/24 ipv6.method disabled \
  wifi-sec.key-mgmt wpa-psk wifi-sec.psk '<mot de passe>' \
  wifi-sec.proto rsn wifi-sec.pairwise ccmp wifi-sec.group ccmp
```

- `ipv4.method shared` : NetworkManager fait serveur DHCP et partage de connexion (NAT),
  comme `hexapode-share` sur le câble.
- `autoconnect no` : le hotspot ne démarre **jamais tout seul** (sinon le PC perdrait le
  Wi-Fi de l'école à chaque démarrage).
- Canal 6 en 2,4 GHz : portée meilleure qu'en 5 GHz, pas de canal radar (DFS).
- Mot de passe : `nmcli -s -g 802-11-wireless-security.psk connection show hexapode-hotspot`
  (il n'est pas écrit dans le dépôt). Le changer :
  `nmcli connection modify hexapode-hotspot wifi-sec.psk 'nouveau'`, puis faire de même
  sur la Pi (profil `hexapode-wifi`).
- `~/.ssh/config` : alias `hexapode-wifi` → `10.42.0.10`, même clé et même empreinte
  (`HostKeyAlias hexapode`) que `hexapode`.

### Raspberry Pi

```bash
# pays Wi-Fi (sans lui le Wi-Fi reste bloque) et Wi-Fi active
echo "options cfg80211 ieee80211_regdom=FR" | sudo tee /etc/modprobe.d/cfg80211-fr.conf
sudo iw reg set FR
sudo rfkill unblock wifi
sudo nmcli radio wifi on

# connexion au hotspot du PC : adresse fixe 10.42.0.10 + DHCP, connexion automatique
sudo nmcli connection add type wifi ifname wlan0 con-name hexapode-wifi autoconnect yes \
  ssid Hexapode-ECE wifi-sec.key-mgmt wpa-psk wifi-sec.psk '<mot de passe>' \
  ipv4.method auto ipv4.addresses 10.42.0.10/24 ipv4.may-fail yes ipv6.method link-local \
  connection.autoconnect-priority 10
```

Les **autres réseaux Wi-Fi enregistrés sur la Pi** (FabLab Makers, Eceborg, freebox…)
sont passés en `autoconnect no` : sinon la Pi s'y connecte dès que son Wi-Fi s'allume et
n'en change plus quand le hotspot apparaît (constaté au premier essai). `hexapode-wifi`
réessaie sans limite (`connection.autoconnect-retries 0`).

```bash
sudo nmcli connection modify "<autre reseau>" connection.autoconnect no   # pour chacun
sudo nmcli connection modify hexapode-wifi connection.autoconnect-retries 0
```

Le câble (`eth0`, `Wired connection 1`) n'est pas modifié.

### Dépôt

| Fichier | Changement |
|---|---|
| `ros2_ws/hexapode_env.sh` | choisit l'interface : câble (`192.168.137.x`) sinon hotspot (`10.42.0.x`) ; exporte `HEXAPODE_PI` (adresse de la Pi) |
| `ros2_ws/src/hexapod_bringup/config/cyclonedds.xml` | pairs CycloneDDS = ceux de la liaison active (`HEXAPODE_PEER_PC` / `_PI`, fixés par `hexapode_env.sh`) ; avertissements réseau masqués (sinon `ddsi_udp_conn_write … failed` remplit le terminal Teleop) |
| `ros2_ws/hexapode.sh` | passe `host:=$HEXAPODE_PI` au pont `leg_server_bridge` |

---

## 4. Internet en même temps ?

Une carte Wi-Fi ne peut pas être à la fois point d'accès et client d'un réseau qui
utilise un canal radar (le Wi-Fi de l'école est sur le canal 124 en 5 GHz). Solutions :

1. **Téléphone en partage de connexion USB** branché au PC : le PC garde internet par le
   téléphone et le partage à la Pi par le hotspot. Le plus simple.
2. **Clé Wi-Fi USB** sur le PC : une carte pour l'école, une pour le hotspot
   (`nmcli connection modify hexapode-hotspot connection.interface-name <clé>`).
3. **Câble réseau** vers le réseau de l'école, si une prise est disponible.

Sans internet, tout le pilotage du robot marche quand même (ROS, Foxglove, `ssh`) ; seuls
`git push / pull` vers GitHub et `apt` attendent le retour sur le Wi-Fi de l'école.

---

## 5. Dépannage

| Symptôme | Vérifier |
|---|---|
| La Pi n'apparaît pas | elle est peut-être connectée à un autre Wi-Fi : par le câble, `ssh hexapode "nmcli -t -f DEVICE,CONNECTION device"` doit montrer `wlan0:hexapode-wifi` ; sinon `ssh hexapode "sudo nmcli connection up hexapode-wifi"` (hotspot allumé) |
| La Pi n'apparaît toujours pas | `nmcli connection show --active` sur le PC (hotspot actif ?) ; Pi sous tension depuis > 30 s ; par le câble : `ssh hexapode "nmcli -t -f DEVICE,STATE device; rfkill list wifi"` |
| `wlan0: unavailable` sur la Pi | `sudo rfkill unblock wifi` puis `sudo nmcli radio wifi on` ; pays : `iw reg get` doit afficher `country FR` |
| Mauvais mot de passe | `ssh hexapode "sudo nmcli connection modify hexapode-wifi wifi-sec.psk '<mot de passe>'"` (par le câble) |
| ROS : la Pi et le PC ne se voient pas | `echo $HEXAPODE_IFACE` doit donner l'interface Wi-Fi sur le PC (`wlp0s20f3`) et `wlan0` sur la Pi ; ouvrir un nouveau terminal après avoir allumé le hotspot |
| Le pont ne joint pas le leg_server | `hexapode.sh` affiche l'adresse utilisée ; `nc -z 10.42.0.10 5555 && echo ok` |
| Saccades / gels plus fréquents | le Wi-Fi ajoute de la latence au pont (25 consignes/s) : rester près du robot, ou lancer la marche **sur la Pi** (`ssh hexapode-wifi`, puis `ros2 launch hexapod_control hexapod.launch.py robot:=true host:=127.0.0.1`) et ne garder que Foxglove (`ws://10.42.0.10:8765`) et le clavier sur le PC |
| Le PC n'a plus internet | normal (§4) : `nmcli connection down hexapode-hotspot` |
