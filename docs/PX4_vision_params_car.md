# PX4 EKF2 Vision Params — Small Car

Apply once via PX4 NSH console or QGroundControl. Per spec §5.2.

## Required

```bash
# Vision fusion
param set EKF2_EV_CTRL 3              # VPOS+HPOS (no VEL/YAW)
param set EKF2_EV_DELAY 0             # align_node handles timestamping
param set EKF2_EVP_NOISE 0.1          # 0.1m vision position noise
param set EKF2_EV_NOISE_MD 0          # use msg covariance

# Vision sensor offset (base_link ground projection → PX4 center, NED)
param set EKF2_EV_POS_X 0
param set EKF2_EV_POS_Y 0
param set EKF2_EV_POS_Z 0.09

# GPS offset (BG-620 → PX4 center, NED)
param set EKF2_GPS_POS_X -0.145
param set EKF2_GPS_POS_Y -0.118
param set EKF2_GPS_POS_Z -0.077

# Dual-antenna heading fusion
param set EKF2_GPS_YAW_OFF 90
param set EKF2_GPS_CTRL 15             # 1+2+4+8 (lat+lon+alt+vel+heading)

# Disable magnetometer
param set EKF2_MAG_TYPE 0
param save
```

## Verification

After param set + reboot, check:
- `ros2 topic echo /mavros/state --once` → `ekf2: ok`
- `ros2 topic echo /mavros/vision_pose/pose --once` → valid pose at 10Hz
- `ros2 topic echo /mavros/local_position/odom --once` → has covariance
