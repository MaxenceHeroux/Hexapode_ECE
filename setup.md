# Raspberry Pi « hexapode » — Guide de connexion et d'installation

> Sans câble : hotspot Wi-Fi de la Pi, voir [hotspot.md](hotspot.md).

Ce document décrit tout ce qui a été mis en place le **23/11/2026** entre le PC
Windows et la Raspberry Pi de l'hexapode : connexion SSH, réseau, partage
d'internet, VS Code, mises à jour, remise à neuf, sauvegardes, synchronisation
git.

Les servos Dynamixel (serveur, interface, essais) sont décrits dans [dyna.md](Dyna_test/dyna.md) ; l'index du projet est le [README.md](README.md).

---

## 1. En bref

| Élément | Valeur |
|---|---|
| Carte | Raspberry Pi 4 Model B Rev 1.5 (pas une Pi 5) |
| Système | Ubuntu 22.04 LTS (jammy), noyau `5.15.0-1113-raspi` |
| ROS | ROS 2 **Humble** (`ros-humble-desktop`) |
| Nom de la machine | `hexapode` |
| Utilisateur | `hexa-pi` |
| Adresse IPv4 fixe (câble) | **`192.168.137.10`** |
| Adresse IPv6 lien local | `fe80::ba1c:8ef1:258:b472` |
| Adresse MAC Ethernet | `D8:3A:DD:9D:05:7C` |
| PC côté câble | `192.168.137.1` (adaptateur « Ethernet », Realtek USB GbE RTL8153) |

**Se connecter :**

```bash
ssh hexapode            # connexion normale (IP fixe 192.168.137.10)
ssh hexapode-ipv6       # secours si le partage de connexion Windows est coupé
```

**Ouvrir VS Code sur la Pi :**

```bash
code --remote ssh-remote+hexapode /home/hexa-pi
```

(ou dans VS Code : icône *Remote Explorer* → `hexapode`)

---

## 2. Schéma du réseau

```
   Internet
      │  (Wi-Fi)
┌─────┴──────────────────────┐        câble Ethernet        ┌──────────────────────┐
│ PC Windows                 │──────────────────────────────│ Raspberry Pi 4       │
│  Wi-Fi  : réseau de l'école│  adaptateur USB "Ethernet"   │  eth0                │
│  Ethernet : 192.168.137.1  │                              │   192.168.137.10 fixe│
│  (partage de connexion ICS)│                              │   192.168.137.x DHCP │
└────────────────────────────┘                              │   fe80::ba1c:…:b472  │
                                                            └──────────────────────┘
```

- Le PC partage sa connexion **Wi-Fi** vers le port **Ethernet** (partage de
  connexion Internet de Windows, « ICS »). Windows joue le rôle de box : il
  donne une adresse à la Pi (DHCP) et la fait sortir sur internet.
- La Pi a **deux adresses IPv4** sur le câble : une fixe (`192.168.137.10`,
  utilisée par `ssh hexapode`) et une automatique donnée par Windows.
- Elle garde aussi une adresse **IPv6 lien local** qui marche même sans
  partage de connexion (utilisée par `ssh hexapode-ipv6`).

---

## 3. Connexion SSH

### 3.1 Clé SSH (sans mot de passe)

- Clé privée sur le PC : `C:\Users\herou\.ssh\id_ed25519_hexapode`
  (type ed25519, sans passphrase)
- Clé publique installée sur la Pi dans `/home/hexa-pi/.ssh/authorized_keys` :
  ```
  ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIApgg4zOomN9TkrSmUhzLeKvn9bl5TFSpNzUV+SC8xfO pc-maxence->hexapode
  ```

### 3.2 Raccourcis dans `C:\Users\herou\.ssh\config`

```
# Raspberry Pi hexapode en cable direct (partage de connexion Windows actif)
Host hexapode
    HostName 192.168.137.10
    User hexa-pi
    IdentityFile ~/.ssh/id_ed25519_hexapode
    IdentitiesOnly yes
    HostKeyAlias hexapode
    ConnectTimeout 5

# Secours : IPv6 lien local (marche sans partage ; %%9 = numero de l'adaptateur USB)
Host hexapode-ipv6
    HostName fe80::ba1c:8ef1:258:b472%%9
    User hexa-pi
    IdentityFile ~/.ssh/id_ed25519_hexapode
    IdentitiesOnly yes
    HostKeyAlias hexapode
    ConnectTimeout 5
```

Remarques :
- Dans ce fichier, `%` doit s'écrire `%%`.
- `%9` est le numéro (ifIndex) de l'adaptateur USB Ethernet sous Windows
  (c'était `%30` le 23/11/2026, il est passé à `%9` après un changement de
  port USB). Il peut changer si l'adaptateur est branché sur un autre port
  USB. Pour le retrouver :
  ```powershell
  Get-NetAdapter | Where-Object InterfaceDescription -like '*Realtek USB*' | Select-Object Name, ifIndex
  ```
- `HostKeyAlias hexapode` : les deux raccourcis partagent la même empreinte
  de la Pi dans `known_hosts` (pas d'alerte en passant de l'un à l'autre).

### 3.3 Mot de passe

Le mot de passe de `hexa-pi` avait été oublié. Il a été **réinitialisé** avec
un mot de passe temporaire (volontairement non écrit ici).
**À changer** dès que possible :

```bash
ssh hexapode
passwd
```

Il est demandé pour chaque `sudo`.

#### Comment il a été réinitialisé (si ça se reproduit)

Un mot de passe Linux ne se « retrouve » pas, on le remplace. Méthode
utilisée, sans écran ni clavier sur la Pi, avec la carte SD dans le PC
Windows (seule la partition `system-boot` est lisible par Windows) :

1. Sauvegarde de `cmdline.txt` en `cmdline.txt.orig`.
2. Création d'un script `reset.sh` sur la partition `system-boot`, qui ajoute
   la clé SSH, change le mot de passe (`chpasswd`), remet `cmdline.txt`
   d'origine et redémarre.
3. Ajout à la fin de la ligne de `cmdline.txt` :
   `systemd.run="/bin/sh /boot/firmware/reset.sh" systemd.unit=kernel-command-line.service`
4. Carte SD remise dans la Pi, démarrage : le script s'exécute une seule fois.
5. Fichiers temporaires supprimés ensuite.

Alternative : avec un écran et un clavier, ajouter `init=/bin/sh` dans
`cmdline.txt`, puis `mount -o remount,rw /`, `passwd hexa-pi`, `sync`.

> ⚠️ Le lecteur de carte SD intégré du PC (Realtek PCIe) n'est **pas**
> compatible avec `wsl --mount` : la tentative avait mis le disque hors
> ligne. Pour le remettre : `Set-Disk -Number 1 -IsOffline $false` (admin).

---

## 4. Réseau

### 4.1 Partage de connexion Windows (Wi-Fi → Ethernet)

Activé par script (admin) avec l'API Windows `HNetCfg.HNetShare` :
- **Wi-Fi** = connexion publique partagée
- **Ethernet** = réseau privé qui reçoit la connexion → le PC prend
  automatiquement `192.168.137.1`

Activer / désactiver à la main :
*Paramètres réseau → Wi-Fi → Propriétés de l'adaptateur → onglet
« Partage » → cocher « Autoriser d'autres utilisateurs… » et choisir
« Ethernet ».*

Windows coupe parfois ce partage après une mise à jour. Symptôme :
`ssh hexapode` ne répond plus (mais `ssh hexapode-ipv6` marche) et la Pi n'a
plus internet.

Script de réactivation (à lancer en administrateur) :

```powershell
$m = New-Object -ComObject HNetCfg.HNetShare
foreach ($c in @($m.EnumEveryConnection)) {
    $n = $m.NetConnectionProps.Invoke($c).Name
    $cfg = $m.INetSharingConfigurationForINetConnection.Invoke($c)
    if ($cfg.SharingEnabled) { $cfg.DisableSharing() }
    if ($n -eq 'Wi-Fi')    { $wifi = $cfg }
    if ($n -eq 'Ethernet') { $eth  = $cfg }
}
$wifi.EnableSharing(0)   # 0 = public (internet)
$eth.EnableSharing(1)    # 1 = prive (vers la Pi)
```

Si tu changes de source d'internet (partage de téléphone USB par exemple), il
faut refaire le partage depuis cette nouvelle connexion.

### 4.2 Configuration réseau de la Pi (NetworkManager)

Connexion : `Wired connection 1` sur `eth0`.

| Réglage | Valeur | Pourquoi |
|---|---|---|
| `ipv4.method` | `auto` | DHCP : marche aussi sur une box / un routeur |
| `ipv4.addresses` | `192.168.137.10/24` | IP fixe en plus du DHCP, pour `ssh hexapode` |
| `ipv4.may-fail` | `yes` | |
| `ipv6.method` | `link-local` | **corrige les coupures toutes les 45 s** (voir ci-dessous) |

Commandes utilisées :

```bash
sudo nmcli con modify "Wired connection 1" ipv6.method link-local ipv4.may-fail yes
sudo nmcli con modify "Wired connection 1" ipv4.method auto ipv4.addresses 192.168.137.10/24
sudo nmcli dev reapply eth0
```

### 4.3 Problème de coupures (diagnostic)

**Cause n°1 (corrigée)** — en câble direct sans DHCP, NetworkManager attendait
une adresse pendant 45 s, échouait, puis **relançait la connexion** en boucle :

```
dhcp4 (eth0): activation: beginning transaction (timeout in 45 seconds)
device (eth0): Activation: failed for connection 'Wired connection 1'
```

Le passage de l'IPv6 en `link-local` (et maintenant le DHCP de Windows) rend la
connexion stable.

**Cause n°2 (non corrigée, matérielle)** — micro-coupures du lien physique de
quelques secondes (`bcmgenet … eth0: Link is Down / Link is Up` dans
`dmesg`). L'alimentation de la Pi a été vérifiée (`vcgencmd get_throttled` →
`0x0`, pas de sous-tension). Suspects, dans l'ordre :
1. l'adaptateur USB Ethernet du PC (essayer un autre port, éviter les hubs) ;
2. le câble / les connecteurs (câble court, ne pas tirer dessus quand le robot bouge) ;
3. la mise en veille USB de Windows (*Options d'alimentation → Paramètres USB
   → Suspension sélective → Désactivé*).

Pour vérifier côté Pi :

```bash
journalctl -b -o short-monotonic | grep -E "Link is (Up|Down)"
```

### 4.4 Trouver la Pi si rien ne marche

```powershell
ping hexapode.local                      # mDNS (pas toujours fiable sous Windows)
Get-NetNeighbor -InterfaceAlias Ethernet # chercher la MAC D8-3A-DD-9D-05-7C
```

### 4.5 Note : pas d'horloge RTC

La Pi 4 n'a pas de pile pour garder l'heure. Sans internet, l'heure (et donc
les dates du journal) peut être fausse. Avec le partage de connexion, elle se
synchronise toute seule (`timedatectl` → `System clock synchronized: yes`).

---

## 5. VS Code en SSH (Remote-SSH)

- Extension utilisée : **Remote - SSH** (`ms-vscode-remote.remote-ssh`), déjà installée.
- Dans `C:\Users\herou\AppData\Roaming\Code\User\settings.json`, ajouté dans
  `remote.SSH.remotePlatform` :
  ```json
  "hexapode": "linux",
  "hexapode-ipv6": "linux",
  ```
  Sans ça, VS Code reste bloqué sur une question « Select the platform of
  the remote host ».
- Le serveur VS Code s'installe tout seul dans `~/.vscode-server` sur la Pi à
  la première connexion (si la Pi n'a pas internet, VS Code le télécharge sur
  le PC et le copie).
- Le paquet `code` (VS Code de bureau) est aussi installé sur la Pi.

Ouvrir :

```bash
code --remote ssh-remote+hexapode /home/hexa-pi
```

---

## 6. Mises à jour du système

### 6.1 Ce qui a été fait

1. **Clé du dépôt ROS 2 expirée** (`EXPKEYSIG F42ED6FBAB17C654 Open Robotics`) :
   remplacée par la clé officielle à jour (valable jusqu'en 2030) :
   ```bash
   sudo curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
   ```
2. `apt upgrade` : **823 paquets** mis à jour (dont 289 paquets ROS Humble).
3. `apt full-upgrade` : 12 paquets restants, dont le **nouveau noyau**
   (`5.15.0-1113-raspi`), puis redémarrage.

Les fichiers de configuration existants ont été conservés
(`--force-confold`).

### 6.2 Refaire une mise à jour

```bash
sudo apt update
sudo apt full-upgrade
sudo reboot        # si /var/run/reboot-required existe
```

Si `apt` dit *« Could not get lock … held by process … (unattended-upgr) »* :
ce sont les mises à jour automatiques d'Ubuntu qui tournent. Attendre, ou
utiliser `sudo apt -o DPkg::Lock::Timeout=600 upgrade` qui patiente tout
seul. (Si c'est seulement l'étape de téléchargement `--download-only` qui
bloque, `sudo systemctl stop apt-daily.service` l'arrête sans risque.)

---

## 7. Remise à neuf de la Pi

### 7.1 Ce qui a été retiré

- **Données** : `hexaros_ws`, `ldlidar_ros2_ws`, `hexa_ws`, `HEXA_Dyna`,
  `Dynamixel_AX-12A`, `Documents`, `Téléchargements`, `test`.
- **Configuration perso** : `.vscode-server`, `.cache`, `.config`, `.local`
  (modules pip), `.thunderbird`, `.ros`, `.rviz2`, `.gitconfig`,
  `.cloudflared`, `.dotnet`, `.copilot`, historiques…
- **Paquets** : `zerotier-one` (+ son dépôt apt), `ffmpeg`, `fswebcam`,
  `v4l-utils`, `sl`.
- **Paquets ROS en plus du desktop** : `rosbridge-suite`, `v4l2-camera`,
  `image-transport-plugins` (compressed, theora…), `xacro`.
- Ancien noyau `5.15.0-1074`.

### 7.2 Ce qui reste

- Ubuntu 22.04 + **ROS 2 Humble desktop** (273 paquets ROS, testé avec
  `demo_nodes_cpp talker` → `/chatter` OK)
- Outils de dev : `git`, `build-essential` (gcc…), `python3-pip`, `vim`,
  `htop`, `net-tools`
- VS Code (`code`) + son dépôt
- DynamixelSDK C++ installé ensuite dans `/usr/local` (voir [dyna.md §1](Dyna_test/dyna.md))
- `.ssh` (clé de connexion), `.bashrc` / `.profile` / `.bash_logout`
  d'origine (`/etc/skel`)
- Les réglages réseau de la section 4.2 et la clé ROS de la section 6.1

`.bashrc` = celui d'origine + une seule ligne ajoutée :

```bash
# ROS 2 Humble
source /opt/ros/humble/setup.bash
```

> ZeroTier ayant été retiré, l'accès à distance via `10.242.43.116` ne
> fonctionne plus : seule la connexion par câble est configurée.

### 7.3 Sauvegardes (sur le PC)

Dossier : `C:\Données\A-Dossier ECE\MAKERS\Hexapode\Backup\Raspberry_Pi\`
(exclu de git par le `.gitignore`, donc jamais publié sur GitHub)

| Fichier | Contenu |
|---|---|
| `pi_home_2026-11-23.tar.gz` (368 Mo) | projets et documents — copie **exacte** (droits + liens symboliques) |
| dossiers extraits (`hexaros_ws`, `ldlidar_ros2_ws`, …) | les mêmes fichiers, lisibles depuis Windows (sans les 446 liens symboliques, voir `tar_err.txt`) |
| `pi_config_2026-11-23.tar.gz` (63 Mo) | toute la config perso + `paquets_manuels.txt` + `pip_user.txt` |

Chaque archive a été vérifiée (empreinte sha256 identique sur la Pi et sur le
PC) **avant** suppression.

Remettre un projet sur la Pi :

```bash
scp pi_home_2026-11-23.tar.gz hexapode:~
ssh hexapode "tar xzf pi_home_2026-11-23.tar.gz hexaros_ws"   # un seul dossier
```

---

## 8. Utiliser ROS 2 sur la Pi

```bash
# ROS est chargé automatiquement à chaque terminal (.bashrc)
echo $ROS_DISTRO                       # -> humble

# Test rapide
ros2 run demo_nodes_cpp talker         # terminal 1
ros2 run demo_nodes_py listener        # terminal 2

# Nouvel espace de travail
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws
colcon build
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

# rosdep (cache effacé lors de la remise à neuf)
rosdep update
rosdep install --from-paths src -y --ignore-src
```

Anciennes variables qui étaient dans `.bashrc` (à remettre si besoin) :

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

---

## 9. Dépannage rapide

| Symptôme | Cause probable | Solution |
|---|---|---|
| `ssh hexapode` → *Connection timed out* | partage de connexion Windows coupé | `ssh hexapode-ipv6` puis réactiver le partage (§4.1) |
| `ssh hexapode-ipv6` ne marche pas | adaptateur USB sur un autre port (`%9` a changé) | retrouver l'ifIndex (§3.2) et corriger `~/.ssh/config` |
| *Host key verification failed* | Pi réinstallée (nouvelle empreinte) | `ssh-keygen -R hexapode` puis se reconnecter |
| *Permission denied (publickey,password)* | clé absente de la Pi | réinstaller la clé (§3.3) |
| VS Code bloqué à « Setting up SSH host » | question de plateforme cachée / pas de réseau | vérifier `remote.SSH.remotePlatform`, tester `ssh hexapode` |
| Pi sans internet | partage coupé | §4.1 ; vérifier `ping 1.1.1.1` sur la Pi |
| `apt` bloqué par un verrou | mises à jour automatiques | §6.2 |
| Coupures de quelques secondes | lien physique | §4.3 |
| Heure fausse sur la Pi | pas de RTC + pas d'internet | rétablir internet, `timedatectl` |
| `Dyna_test` : *Failed to open the port* | U2D2 non branché ou règle udev absente | `ls -l /dev/u2d2` ; réinstaller la règle ([dyna.md §1.7](Dyna_test/dyna.md)) |
| `Dyna_test` : *cannot open shared object libdxl_sbc_cpp.so* | DynamixelSDK non installé | réinstaller le SDK ([dyna.md §1.6](Dyna_test/dyna.md)) |
| `Dyna_test` : servos sans réponse (`[TxRxResult] There is no status packet!`) | alimentation des servos, baudrate ou ID | vérifier l'alim 12 V, `BAUDRATE 1000000`, IDs 0 à 17 |

---

## 10. Dépôt Git du projet (PC)

- Dossier : `C:\Données\A-Dossier ECE\MAKERS\Hexapode`
- Distant : <https://github.com/MaxenceHeroux/Hexapode_ECE> (branche `main`)
- `.gitignore` :
  ```
  # Sauvegardes (non publiées)
  Backup/
  *-backups/

  # Fichiers temporaires / verrous
  ~$*
  fp-info-cache
  *.kicad_prl
  *.lck
  ```
- Premier commit : le dossier `PCB/` (projet KiCad, Gerbers, fichiers JLCPCB,
  import Altium, `BOM_PCB.xlsx`).
- Contenu actuel du dépôt :
  ```
  Hexapode_ECE/
  ├── README.md       ← index du projet
  ├── setup.md        ← ce document (Pi, réseau, SSH, git)
  ├── .gitignore
  ├── PCB/            ← carte d'alimentation / Dynamixel (KiCad)
  └── Dyna_test/      ← code Dynamixel + dyna.md, Torque_calcul.md, graphs/ ([dyna.md §1](Dyna_test/dyna.md))
  ```

---

## 11. Journal des modifications (23/11/2026)

**Sur le PC Windows**
- Dépôt git initialisé, `.gitignore`, premier commit, push sur GitHub.
- Clé SSH `id_ed25519_hexapode` créée ; raccourcis `hexapode` et
  `hexapode-ipv6` dans `~/.ssh/config` ; empreinte de la Pi dans `known_hosts`.
- VS Code : `hexapode` et `hexapode-ipv6` déclarés `linux` dans `settings.json`.
- Partage de connexion Internet activé : Wi-Fi → Ethernet (`192.168.137.1`).
- Lecteur SD remis en ligne après l'échec de `wsl --mount`.

**Sur la Raspberry Pi**
- Mot de passe de `hexa-pi` réinitialisé + clé SSH ajoutée (via la carte SD).
- NetworkManager : `ipv6.method link-local` + IP fixe `192.168.137.10/24`.
- Clé du dépôt ROS 2 renouvelée (ancienne gardée en `.gpg.old`).
- `apt upgrade` (823 paquets) + `apt full-upgrade` (noyau `5.15.0-1113`) + redémarrage.
- Sauvegarde complète vers le PC, puis remise à neuf (§7).
- Dépôt `Hexapode_ECE` cloné dans `~/Hexapode_ECE`, push via Deploy key (§12).
- DynamixelSDK C++ compilé pour ARM et installé dans `/usr/local` ;
  `Dyna_test` compilé ([dyna.md §1](Dyna_test/dyna.md)).
- Règle udev `99-u2d2.rules` : U2D2 toujours en `/dev/u2d2`, latence 1 ms ([dyna.md §1.7](Dyna_test/dyna.md)).
- `leg_server` + `gui/leg_gui.py` : pilotage d'une patte (IDs 12/13/14) ([dyna.md §2](Dyna_test/dyna.md)).

**Récupération du code Dynamixel**
- `Dyna_test` retrouvé sur la partition Ubuntu du dual boot du PC
  (`/home/maxou/Desktop/HEXAPODE/HEXAPODE/Dyna_test`), lue en lecture seule
  depuis Windows ; copie dans `Backup\Ubuntu_dualboot\Dyna_test`.
- Ajouté au dépôt avec un `Makefile` (commit `daf52c3`).

**À faire**
- [ ] Changer le mot de passe temporaire (`passwd`)
- [ ] Tester un autre câble / port USB si les micro-coupures gênent

---

## 12. Git synchronisé entre le PC et la Pi (via GitHub)

```
                 GitHub : MaxenceHeroux/Hexapode_ECE (branche main)
                 ↑ push        ↓ pull
   PC : C:\Données\A-Dossier ECE\MAKERS\Hexapode     Pi : ~/Hexapode_ECE
```

Chaque machine a sa propre copie du dépôt ; GitHub sert de référence commune.

### Configuration sur la Pi
- Dépôt cloné dans `~/Hexapode_ECE`
- Récupération (fetch/pull) en HTTPS : `https://github.com/MaxenceHeroux/Hexapode_ECE.git`
- Envoi (push) en SSH : `git@github.com:MaxenceHeroux/Hexapode_ECE.git`
- Clé SSH dédiée : `~/.ssh/id_ed25519_github` (déclarée dans `~/.ssh/config` pour `github.com`)
- Clé enregistrée sur GitHub comme **Deploy key avec accès en écriture**, nommée
  « Raspberry Pi hexapode » → la Pi ne peut écrire **que** dans ce dépôt.
  Gestion : https://github.com/MaxenceHeroux/Hexapode_ECE/settings/keys
- Identité git : `H-Maxence` / `maxence.heroux@edu.ece.fr` (identique au PC)
- `pull.rebase false` (git pull fusionne)
- L'ancienne clé `~/.ssh/id_ed25519` (herouxmaxence0@gmail.com) est toujours là
  mais n'est **pas** enregistrée sur GitHub.

### Utilisation au quotidien

```bash
# Avant de travailler (sur la machine où tu commences)
git pull

# Après avoir travaillé
git add <fichiers>
git commit -m "message"
git push
```

Règle d'or : **toujours `git pull` avant de commencer** sur une machine, et
**`git push` quand tu as fini**, pour ne pas avoir deux versions qui divergent.

Si `git push` est refusé (*rejected … fetch first*) : l'autre machine a poussé
entre-temps → `git pull`, régler les conflits éventuels, puis `git push`.

### Code ROS dans le dépôt
Le `.gitignore` exclut déjà `build/`, `install/` et `log/` (générés par
`colcon build`). Pour mettre un espace de travail ROS dans le dépôt, par
exemple : `~/Hexapode_ECE/ros2_ws/src/<tes_paquets>`, puis
`cd ~/Hexapode_ECE/ros2_ws && colcon build`.

### Vérifier l'accès GitHub depuis la Pi
```bash
ssh -T git@github.com
# -> Hi MaxenceHeroux/Hexapode_ECE! You've successfully authenticated...
```

Test fait le 23/11/2026 : commit `eb328d9` (ajout des dossiers colcon au
`.gitignore`) poussé depuis la Pi puis récupéré sur le PC avec `git pull`.
