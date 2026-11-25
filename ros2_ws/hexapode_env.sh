# Environnement ROS 2 de l'hexapode (PC et Pi). A sourcer APRES /opt/ros/humble/setup.*
#   source ~/.../Hexapode_ECE/ros2_ws/hexapode_env.sh
# Voir tps_archi.md section 7.1.

_hexa_ws="$(cd "$(dirname "${BASH_SOURCE[0]:-${(%):-%x}}")" && pwd)"

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${_hexa_ws}/src/hexapod_bringup/config/cyclonedds.xml"

# Interface vers l'autre machine : le cable (192.168.137.x : eth0 sur la Pi, adaptateur USB
# sur le PC) en priorite, sinon le hotspot Wi-Fi du PC (10.42.0.x, voir hotspot.md).
_hexa_ips="$(ip -o -4 addr show 2>/dev/null)"
# (HEXAPODE_LINK=wifi : hotspot en priorite, utilise par hexapode.sh robot-distance)
HEXAPODE_IFACE=""
[ "$HEXAPODE_LINK" = wifi ] && HEXAPODE_IFACE="$(echo "$_hexa_ips" | awk '$4 ~ /^10\.42\.0\./ {print $2; exit}')"
[ -z "$HEXAPODE_IFACE" ] && HEXAPODE_IFACE="$(echo "$_hexa_ips" | awk '$4 ~ /^192\.168\.137\./ {print $2; exit}')"
[ -z "$HEXAPODE_IFACE" ] && HEXAPODE_IFACE="$(echo "$_hexa_ips" | awk '$4 ~ /^10\.42\.0\./ {print $2; exit}')"
# Aucune liaison (cable debranche, hotspot eteint) : ROS en local seulement (simulation)
export HEXAPODE_IFACE="${HEXAPODE_IFACE:-lo}"
# Adresse de la Pi (leg_server) vue de cette machine
case "$(echo "$_hexa_ips" | awk -v i="$HEXAPODE_IFACE" '$2 == i {print $4; exit}')" in
  10.42.0.*)    export HEXAPODE_PI=10.42.0.10 HEXAPODE_PEER_PC=10.42.0.1 HEXAPODE_PEER_PI=10.42.0.10 ;;
  192.168.137.*) export HEXAPODE_PI=192.168.137.10 HEXAPODE_PEER_PC=192.168.137.1 HEXAPODE_PEER_PI=192.168.137.10 ;;
  *)            export HEXAPODE_PI=192.168.137.10 HEXAPODE_PEER_PC=127.0.0.1 HEXAPODE_PEER_PI=127.0.0.1 ;;
esac    # HEXAPODE_PEER_* : pairs CycloneDDS de la liaison active (cyclonedds.xml)
unset _hexa_ips

# Espace de travail compile (colcon), s'il existe
if [ -n "$ZSH_VERSION" ] && [ -f "${_hexa_ws}/install/setup.zsh" ]; then
  source "${_hexa_ws}/install/setup.zsh"
elif [ -f "${_hexa_ws}/install/setup.bash" ]; then
  source "${_hexa_ws}/install/setup.bash"
fi
unset _hexa_ws
