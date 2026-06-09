#!/usr/bin/env bash
# set_px4_vision_params.sh — push PX4 vision params via mavros param service
# Usage: ./set_px4_vision_params.sh
set -euo pipefail

PARAMS=(
  "EKF2_EV_CTRL 3"
  "EKF2_EV_DELAY 0"
  "EKF2_EVP_NOISE 0.1"
  "EKF2_EV_NOISE_MD 0"
  "EKF2_EV_POS_X 0"
  "EKF2_EV_POS_Y 0"
  "EKF2_EV_POS_Z 0.09"
  "EKF2_GPS_POS_X -0.145"
  "EKF2_GPS_POS_Y -0.118"
  "EKF2_GPS_POS_Z -0.077"
  "EKF2_GPS_YAW_OFF 90"
  "EKF2_GPS_CTRL 15"
  "EKF2_MAG_TYPE 0"
)

# Wait for mavros
for i in {1..30}; do
  if ros2 service list 2>/dev/null | grep -q "/mavros/set_param"; then
    break
  fi
  echo "Waiting for mavros... ($i/30)"
  sleep 1
done

for entry in "${PARAMS[@]}"; do
  read -r key value <<< "$entry"
  echo "Setting $key = $value"
  ros2 param set /mavros "$(echo $key | tr '[:upper:]' '[:lower:]')" "$value"
done

echo "All params set. Run 'param save' on PX4 NSH to persist."
