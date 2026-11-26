#!/usr/bin/env bash
# Lance TOUT l'hexapode en une commande : marche + Foxglove (pont + application) +
# terminal de pilotage clavier, et le pont vers le vrai robot selon le mode.
#
#   ./hexapode.sh              # simulation (aucun servo ne bouge)
#   ./hexapode.sh dry          # + pont vers le robot SANS envoi (verifie servos, butees)
#   ./hexapode.sh robot        # + pont vers le robot : LE ROBOT SE LEVE PUIS MARCHE
#   ./hexapode.sh robot-distance  # comme robot, SANS CABLE : le PC se connecte au hotspot Wi-Fi
#                                 # de la Pi, lance leg_server si besoin (voir hotspot.md)
#   ./hexapode.sh dry-distance    # idem en verification (aucun envoi)
#                                 # -> Ctrl+C : quitte le hotspot de la Pi et revient sur un
#                                 #    Wi-Fi avec internet
#   ./hexapode.sh internet        # seulement ca : quitter le hotspot, retrouver internet
#   ./hexapode.sh robot rviz:=true      # arguments supplementaires passes au lancement
#
# Ctrl+C arrete tout (sur le robot : gel, couple garde ; ESTOP pour le relacher).
set -e
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

WIFI=false

# Wi-Fi utilise avant de rejoindre le hotspot de la Pi (pour y revenir ensuite)
PREV_WIFI_FILE="${XDG_CACHE_HOME:-$HOME/.cache}/hexapode_wifi_precedent"
active_wifi() {
  nmcli -t -f NAME,TYPE connection show --active | awk -F: '$2 == "802-11-wireless" && $1 != "hexapode-pi" {print $1; exit}'
}

# Quitte le hotspot de la Pi et revient sur un Wi-Fi avec internet : d'abord celui
# d'avant (memorise), sinon les reseaux connus visibles, du plus recent au plus ancien.
reconnect_internet() {
  trap '' INT                                   # un 2e Ctrl+C n'interrompt pas la reconnexion
  echo
  if nmcli -t -f NAME connection show --active | grep -qx hexapode-pi; then
    echo ">> Deconnexion du hotspot de la Pi..."
    nmcli connection down hexapode-pi >/dev/null 2>&1 || true
    sleep 1
  fi
  if ping -c 1 -W 2 1.1.1.1 >/dev/null 2>&1; then
    echo ">> Internet ok ($(active_wifi))."; return 0
  fi
  local cands=() prev seen
  prev="$(cat "$PREV_WIFI_FILE" 2>/dev/null)"
  [ -n "$prev" ] && cands+=("$prev")
  nmcli device wifi rescan >/dev/null 2>&1; sleep 2
  local visible; visible="$(nmcli -t -f SSID device wifi list 2>/dev/null)"
  # profils Wi-Fi connus dont le reseau est visible, du plus recemment utilise au plus ancien
  while IFS=: read -r ts name; do
    [ "$name" = hexapode-pi ] && continue
    ssid="$(nmcli -g 802-11-wireless.ssid connection show "$name" 2>/dev/null)"
    echo "$visible" | grep -qxF "$ssid" && cands+=("$name")
  done < <(nmcli -t -f TIMESTAMP,NAME,TYPE connection show | awk -F: '$3 == "802-11-wireless" {print $1":"$2}' | sort -rn)
  for name in "${cands[@]}"; do
    case " $seen " in *" $name "*) continue ;; esac; seen="$seen $name"
    echo -n ">> Connexion a '$name' "
    if nmcli connection up "$name" >/dev/null 2>&1; then
      for i in $(seq 1 15); do
        ping -c 1 -W 1 1.1.1.1 >/dev/null 2>&1 && { echo " ok, internet retrouve."; return 0; }
        echo -n "."; sleep 1
      done
    fi
    echo " echec"
  done
  echo ">> Pas d'internet : choisir un reseau a la main (nmcli device wifi connect <SSID> --ask)."
}

case "${1:-sim}" in
  internet)       reconnect_internet; exit 0 ;;
  sim|simulation) ROBOT=false ;;
  dry)            ROBOT=dry ;;
  robot)          ROBOT=true ;;
  dry-distance)   ROBOT=dry;  WIFI=true ;;
  robot-distance) ROBOT=true; WIFI=true ;;
  -h|--help)      sed -n '2,16p' "$0"; exit 0 ;;
  *) echo "mode inconnu : $1 (sim | dry | robot | dry-distance | robot-distance | internet)"; exit 1 ;;
esac
shift || true

# Sans cable : le PC rejoint le hotspot Wi-Fi de la Pi (profil NetworkManager hexapode-pi,
# hotspot.md). Si le cable est branche, il reste prioritaire : on ne touche pas au Wi-Fi.
if [ "$WIFI" = true ] && ping -c 1 -W 1 192.168.137.10 >/dev/null 2>&1; then
  echo ">> Cable branche et Pi joignable : liaison par CABLE (prioritaire), Wi-Fi inchange."
  WIFI=cable
fi
if [ "$WIFI" = true ]; then
  if ! nmcli -t -f NAME connection show --active | grep -qx hexapode-pi; then
    mkdir -p "$(dirname "$PREV_WIFI_FILE")"; active_wifi > "$PREV_WIFI_FILE"
    echo ">> Connexion au hotspot de la Pi 'Hexapode-ECE' (le PC quitte '$(cat "$PREV_WIFI_FILE")')."
    nmcli connection up hexapode-pi >/dev/null || {
      echo ">> Hotspot de la Pi introuvable : Pi allumee depuis plus de 30 s ? Voir hotspot.md."; exit 1; }
  fi
  echo -n ">> Attente de la Pi (10.42.0.10) "
  for i in $(seq 1 60); do
    ping -c 1 -W 1 10.42.0.10 >/dev/null 2>&1 && break
    echo -n "."; sleep 1
  done
  if ! ping -c 1 -W 1 10.42.0.10 >/dev/null 2>&1; then
    echo; echo ">> La Pi ne repond pas sur le hotspot (60 s). Voir hotspot.md, section Depannage."
    exit 1
  fi
  echo " ok"
  if ! nc -z -w 2 10.42.0.10 5555 2>/dev/null; then
    echo ">> leg_server ne tourne pas sur la Pi : lancement."
    ssh -o ConnectTimeout=5 hexapode-wifi \
      "cd ~/Hexapode_ECE/Dyna_test && (nohup ./bin/leg_server > /tmp/leg_server.log 2>&1 &)"
    for i in $(seq 1 10); do nc -z -w 1 10.42.0.10 5555 2>/dev/null && break; sleep 1; done
  fi
  export HEXAPODE_LINK=wifi
fi

# Environnement : ROS, CycloneDDS local (~/ros_local) si pas installe par apt, hexapode_env.sh
set +u
source /opt/ros/humble/setup.bash
if [ ! -e /opt/ros/humble/lib/librmw_cyclonedds_cpp.so ] && [ -d ~/ros_local/opt/ros/humble ]; then
  export AMENT_PREFIX_PATH=~/ros_local/opt/ros/humble:$AMENT_PREFIX_PATH
  export LD_LIBRARY_PATH=~/ros_local/opt/ros/humble/lib:~/ros_local/opt/ros/humble/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
fi
source "$WS/hexapode_env.sh"

# Compile si besoin (rapide, --symlink-install)
if [ ! -f "$WS/install/setup.bash" ]; then
  (cd "$WS" && colcon build --symlink-install)
  source "$WS/install/setup.bash"
fi

# Une session precedente oubliee = deux marches qui pilotent le robot en meme temps
# et le port Foxglove 8765 occupe : on l'arrete.
if pgrep -f "lib/hexapod_control/(gait_node|leg_server_bridge)" >/dev/null \
   || pgrep -x foxglove_bridge >/dev/null; then
  echo ">> Session precedente trouvee : arret."
  pkill -INT -f "ros2 launch hexapod_control" || true
  sleep 2
  pkill -f "lib/hexapod_control/(gait_node|leg_server_bridge)" || true
  pkill -x foxglove_bridge || true
  pkill -x robot_state_publisher || true
  sleep 1
fi

if [ "$ROBOT" = true ]; then
  echo ">> MODE ROBOT : le robot va se lever (rampe 8 s) puis suivre la marche."
  echo ">> Robot au sol, batterie >= 11 V, quelqu'un a cote de l'alimentation."
  read -r -p ">> Entree pour continuer, Ctrl+C pour annuler... "
fi

echo ">> Liaison PC <-> Pi : $HEXAPODE_IFACE, Pi (leg_server) en $HEXAPODE_PI"
if [ "$WIFI" = true ]; then
  # Ctrl+C arrete le lancement, puis le script continue : retour sur un Wi-Fi avec internet.
  # (gestionnaire 'true' et pas "ignorer" : ros2 launch recoit bien le Ctrl+C)
  set +e
  trap 'true' INT
  ros2 launch hexapod_control hexapod.launch.py robot:=$ROBOT teleop:=true app:=true host:=$HEXAPODE_PI "$@"
  reconnect_internet
  exit 0
fi
exec ros2 launch hexapod_control hexapod.launch.py robot:=$ROBOT teleop:=true app:=true host:=$HEXAPODE_PI "$@"
