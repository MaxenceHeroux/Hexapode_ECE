# Environnement ROS 2 de l'hexapode (PC et Pi). A sourcer APRES /opt/ros/humble/setup.*
#   source ~/.../Hexapode_ECE/ros2_ws/hexapode_env.sh
# Voir tps_archi.md section 7.1.

_hexa_ws="$(cd "$(dirname "${BASH_SOURCE[0]:-${(%):-%x}}")" && pwd)"

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${_hexa_ws}/src/hexapod_bringup/config/cyclonedds.xml"

# Interface reliee au cable (celle qui a une adresse 192.168.137.x) : eth0 sur la Pi,
# adaptateur USB Ethernet sur le PC. Repli sur eth0.
HEXAPODE_IFACE="$(ip -o -4 addr show 2>/dev/null | awk '$4 ~ /^192\.168\.137\./ {print $2; exit}')"
export HEXAPODE_IFACE="${HEXAPODE_IFACE:-eth0}"

# Espace de travail compile (colcon), s'il existe
if [ -n "$ZSH_VERSION" ] && [ -f "${_hexa_ws}/install/setup.zsh" ]; then
  source "${_hexa_ws}/install/setup.zsh"
elif [ -f "${_hexa_ws}/install/setup.bash" ]; then
  source "${_hexa_ws}/install/setup.bash"
fi
unset _hexa_ws
