# Helpers the justfile's recipes source. Bash only; no side effects on source.
#
# Project settings (map, vehicle and sensor choices, rviz, scenario output, ...) are
# name:=value arguments of the recipes, as for a launch file. Host settings (displays, ports,
# install and data paths) come from the environment, kept in .env or .envrc.

# Environment variables the recipes used to read as project settings, each with the argument
# that replaced it. `csb_reject_env` refuses to run while any of them is set (even empty),
# so a stale shell or .env cannot change a run silently.
CSB_RETIRED_ENV=(
    "MAP_NAME|map_path:=<dir> (ego-av, bg-av, ego-isaac, isaac-world-gen) or the map argument (e2e)"
    "CSB_CONFIG_DIR|config_file:=<dir>/bridge_config.yaml (just run)"
    "CSB_BACKEND|backend:=carla|isaac (just run)"
    "WEATHER|weather:=<preset> (just run)"
    "GAP_TRACE_DIR|gap_trace_dir:=<dir> (just run)"
    "NPC_CONTACT_SENSORS|npc_contact_sensors:=<value> (just run)"
    "KEEP_STACKS|keep_stacks:=true (two-av, e2e)"
    "RECORD|record:=true (two-av)"
    "EGO_RVIZ_CONFIG|ego_rviz_config:=<file> (two-av)"
    "BG_RVIZ_CONFIG|bg_rviz_config:=<file> (two-av)"
    "EGO_OVERLAY|overlay:=<setup.bash> (ego-av, ego-isaac)"
    "EGO_GOAL_POSES_FILE|goal_poses_file:=<file> (ego-av)"
    "EGO_RELAY|relay:=<endpoint>, or relay:= for none (ego-av)"
    "BG_GOAL_POSES_FILE|goal_poses_file:=<file> (bg-av)"
    "BG_RELAY|relay:=<endpoint>, or relay:= for none (bg-av)"
    "ACB_PUBLISH_CLOCK|publish_clock:=true|false (ego-av)"
    "CONTROL_TRACE_PATH|control_trace_path:=<file> (ego-av)"
    "RVIZ_CONFIG|rviz_config:=<file> (ego-av, ego-isaac, bg-av)"
    "LAUNCH_RVIZ|rviz:=true (ego-av, ego-isaac) or launch_rviz:=true (bg-av)"
    "ACCEL_MAP_PATH|accel_map_path:=<csv>, or accel_map_path:= for none (ego-av)"
    "BRAKE_MAP_PATH|brake_map_path:=<csv>, or brake_map_path:= for none (ego-av)"
    "TRAFFIC_LIGHT_MAP_PATH|traffic_light_map_path:=<yaml>, or traffic_light_map_path:= for none (ego-av)"
    "REPORT_MEASURED_STEERING|report_measured_steering:=true (ego-av)"
    "STEERING_MULTIPLIER|steering_multiplier:=<x> (ego-av)"
    "STEER_RATE_LIMIT_DEG_S|steer_rate_limit_deg_s:=<deg/s> (ego-av)"
    "STEER_TIME_CONSTANT_S|steer_time_constant_s:=<s> (ego-av)"
    "GROUND_TRUTH_OBJECTS|publish_ground_truth_objects:=true (ego-av)"
    "SEED_LOCALIZATION|seed_localization_on_attach:=true (ego-av)"
    "GROUND_TRUTH_RANGE_M|ground_truth_range_m:=<m> (ego-av)"
    "LIDAR_DETECTION_MODEL|lidar_detection_model:=<model> (ego-av)"
    "EGO_LAUNCH_ARGS|the name:=value arguments themselves (ego-av, ego-isaac)"
    "EGO_CONTAINER_MODE|container_mode:=isolated|observable (ego-av, ego-isaac)"
    "EGO_PLAY_LAUNCH_ARGS|play_launch_args:=\"<options>\" (ego-av, ego-isaac)"
    "OUTPUT_DIR|output_directory:=<dir> (scenario, two-av, e2e)"
    "AUTOWARE_OVERLAY|autoware_overlay:=false (ego-av, ego-isaac, bg-av)"
)

csb_reject_env() {
    local e var bad=0
    for e in "${CSB_RETIRED_ENV[@]}"; do
        var="${e%%|*}"
        if [ -n "${!var+x}" ]; then
            echo "[just] $var is no longer read; pass ${e#*|} instead" >&2
            bad=1
        fi
    done
    if [ "$bad" = 1 ]; then
        echo "[just] Project settings are recipe arguments now (name:=value); unset the" \
             "variable(s) above, in this shell and in .env/.envrc, and pass the argument." >&2
        exit 2
    fi
}

# csb_with_defaults OUT DEFAULT... -- ARG...
# OUT=(each name:=value DEFAULT whose name no ARG names, then the ARGs). An argument with an
# empty value (`relay:=`) drops the default and is itself dropped -- ROS launch rejects an
# empty `name:=` -- so the launch file's own default applies.
csb_with_defaults() {
    local -n _out="$1"; shift
    local -a defaults=() given=()
    while [ $# -gt 0 ] && [ "$1" != -- ]; do defaults+=("$1"); shift; done
    [ "${1:-}" = -- ] && shift
    given=("$@")
    _out=()
    local d a named
    for d in "${defaults[@]}"; do
        named=false
        for a in "${given[@]}"; do [ "${a%%:=*}" = "${d%%:=*}" ] && named=true; done
        $named || _out+=("$d")
    done
    for a in "${given[@]}"; do
        [ -n "${a#*:=}" ] && _out+=("$a")
    done
    return 0
}
