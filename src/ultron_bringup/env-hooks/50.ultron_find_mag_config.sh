ULTRON_MAG_CONFIG=$(catkin_find --etc --first-only ultron_bringup mag_config.yaml 2>/dev/null)
if [ -z "$ULTRON_MAG_CONFIG" ]; then
  ULTRON_MAG_CONFIG=$(catkin_find --share --first-only ultron_bringup config/mag_config_default.yaml 2>/dev/null)
fi

export ULTRON_MAG_CONFIG
