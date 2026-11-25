#!/usr/bin/env bash
# Lance TOUT l'hexapode en une commande : marche + Foxglove (pont + application) +
# terminal de pilotage clavier, et le pont vers le vrai robot selon le mode.
#
#   ./hexapode.sh              # simulation (aucun servo ne bouge)
#   ./hexapode.sh dry          # + pont vers le robot SANS envoi (verifie servos, butees)
#   ./hexapode.sh robot        # + pont vers le robot : LE ROBOT SE LEVE PUIS MARCHE
#   ./hexapode.sh robot rviz:=true      # arguments supplementaires passes au lancement
#
# Ctrl+C arrete tout (sur le robot : gel, couple garde ; ESTOP pour le relacher).
set -e
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

case "${1:-sim}" in
  sim|simulation) ROBOT=false ;;
  dry)            ROBOT=dry ;;
  robot)          ROBOT=true ;;
  -h|--help)      sed -n '2,10p' "$0"; exit 0 ;;
  *) echo "mode inconnu : $1 (sim | dry | robot)"; exit 1 ;;
esac
shift || true

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
exec ros2 launch hexapod_control hexapod.launch.py robot:=$ROBOT teleop:=true app:=true host:=$HEXAPODE_PI "$@"
