#!/usr/bin/env bash
# Lance TOUT l'hexapode en une commande : marche + Foxglove (pont + application) +
# terminal de pilotage clavier, et le pont vers le vrai robot selon le mode.
#
#   ./hexapode.sh              # simulation (aucun servo ne bouge)
#   ./hexapode.sh dry          # + pont vers le robot SANS envoi (verifie servos, butees)
#   ./hexapode.sh robot        # + pont vers le robot : LE ROBOT SE LEVE PUIS MARCHE
#   ./hexapode.sh robot-distance  # comme robot, SANS CABLE : allume le hotspot Wi-Fi du PC,
#                                 # attend la Pi, lance leg_server si besoin (voir hotspot.md)
#   ./hexapode.sh dry-distance    # idem en verification (aucun envoi)
#   ./hexapode.sh robot rviz:=true      # arguments supplementaires passes au lancement
#
# Ctrl+C arrete tout (sur le robot : gel, couple garde ; ESTOP pour le relacher).
set -e
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

WIFI=false
case "${1:-sim}" in
  sim|simulation) ROBOT=false ;;
  dry)            ROBOT=dry ;;
  robot)          ROBOT=true ;;
  dry-distance)   ROBOT=dry;  WIFI=true ;;
  robot-distance) ROBOT=true; WIFI=true ;;
  -h|--help)      sed -n '2,13p' "$0"; exit 0 ;;
  *) echo "mode inconnu : $1 (sim | dry | robot | dry-distance | robot-distance)"; exit 1 ;;
esac
shift || true

# Sans cable : hotspot Wi-Fi du PC (connexion NetworkManager hexapode-hotspot, hotspot.md)
if [ "$WIFI" = true ]; then
  if ! nmcli -t -f NAME connection show --active | grep -qx hexapode-hotspot; then
    echo ">> Hotspot Wi-Fi 'Hexapode-ECE' : allumage (le PC quitte le Wi-Fi de l'ecole)."
    nmcli connection up hexapode-hotspot >/dev/null
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
  trap 'echo; echo ">> Hotspot toujours allume. Retour au Wi-Fi de l ecole : nmcli connection down hexapode-hotspot"' EXIT
  ros2 launch hexapod_control hexapod.launch.py robot:=$ROBOT teleop:=true app:=true host:=$HEXAPODE_PI "$@"
  exit $?
fi
exec ros2 launch hexapod_control hexapod.launch.py robot:=$ROBOT teleop:=true app:=true host:=$HEXAPODE_PI "$@"
