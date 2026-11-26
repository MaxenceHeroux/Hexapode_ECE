#!/bin/sh
# A lancer SUR LA PI (sudo) : leg_server et IMU au demarrage, IMU relancee au changement de liaison.
#   sudo sh ~/Hexapode_ECE/ros2_ws/src/hexapod_bringup/systemd/install_pi_services.sh
set -e
D="$(cd "$(dirname "$0")" && pwd)"
install -m 644 "$D/hexapode-leg-server.service" /etc/systemd/system/
install -m 644 "$D/hexapode-imu.service" /etc/systemd/system/
install -m 755 "$D/90-hexapode-liaison" /etc/NetworkManager/dispatcher.d/
systemctl daemon-reload
systemctl enable hexapode-leg-server.service hexapode-imu.service
echo "Services installes et actives au demarrage :"
systemctl is-enabled hexapode-leg-server.service hexapode-imu.service
