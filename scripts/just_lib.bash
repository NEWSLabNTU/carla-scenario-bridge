# Helpers the justfile's recipes source. Bash only; no side effects on source.
#
# Project settings (map, vehicle and sensor choices, rviz, scenario output, ...) are
# name:=value arguments of the recipes, as for a launch file. Host settings (displays, ports,
# install and data paths, the log root) come from the environment, kept in .env or .envrc.

# Environment variables the recipes used to read as project settings, each with the argument
# that replaced it. `swb_reject_env` refuses to run while any of them is set (even empty),
# so a stale shell or .env cannot change a run silently.
SWB_RETIRED_ENV=(
    "MAP_NAME|map_path:=<dir> (ego-av, bg-av, ego-isaac, isaac-world-gen) or the map argument (e2e)"
    "CSB_CONFIG_DIR|config_file:=<dir>/bridge_config.yaml (just run)"
    "CSB_BACKEND|sim:=carla|isaac (just run)"
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

# Host settings renamed with the project (roadmap 046): the old name is refused, naming the
# new one, so a stale .env cannot leave a setting silently unread.
SWB_RENAMED_ENV=(
    "CSB_WEB_PORT|SWB_WEB_PORT"
    "CSB_VEHICLE_USD|SWB_VEHICLE_USD"
    "CSB_LOG_ROOT|SWB_LOG_ROOT"
    "CSB_LOG_ARCHIVE|SWB_LOG_ARCHIVE"
    "CSB_MAP_SOURCE|SWB_MAP_SOURCE"
    "CSB_TICK_GATE|SWB_TICK_GATE"
    "CSB_TM_SERVE|SWB_TM_SERVE"
    "CSB_ISAAC_DIR|SWB_ISAAC_DIR"
)

swb_reject_env() {
    local e var bad=0
    for e in "${SWB_RETIRED_ENV[@]}"; do
        var="${e%%|*}"
        if [ -n "${!var+x}" ]; then
            echo "[just] $var is no longer read; pass ${e#*|} instead" >&2
            bad=1
        fi
    done
    for e in "${SWB_RENAMED_ENV[@]}"; do
        var="${e%%|*}"
        if [ -n "${!var+x}" ]; then
            echo "[just] $var was renamed with the project; set ${e#*|} instead (.env/.envrc)" >&2
            bad=1
        fi
    done
    if [ "$bad" = 1 ]; then
        echo "[just] Project settings are recipe arguments now (name:=value); unset the" \
             "variable(s) above, in this shell and in .env/.envrc, and pass the argument." >&2
        exit 2
    fi
}

# swb_reject_args PROJECT ARG... -- refuse the launch arguments roadmap 051 renamed
# (carla_control:= -> sim_control:=, vehicle_model:=acb_vehicle -> tesla_model3, ...), which
# ROS launch would otherwise drop without a word. The list is the profile's
# (autoware_sim_profile's check_launch_args).
swb_reject_args() {
    local check="$1/install/autoware_sim_profile/lib/autoware_sim_profile/check_launch_args"
    shift
    [ -x "$check" ] || return 0   # not built yet: the launch itself will say so
    "$check" "$@" || exit 2
}

# swb_with_defaults OUT DEFAULT... -- ARG...
# OUT=(each name:=value DEFAULT whose name no ARG names, then the ARGs). An argument with an
# empty value (`relay:=`) drops the default and is itself dropped -- ROS launch rejects an
# empty `name:=` -- so the launch file's own default applies.
swb_with_defaults() {
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

# swb_log_dir PROJECT KIND: the directory play_launch logs a stack of this kind under,
# $SWB_LOG_ROOT/KIND (default PROJECT/play_log/KIND), created. When the log root is not the
# repo's play_log, PROJECT/play_log/KIND is made a symlink to it, so play_log/<kind>/latest
# keeps working from the repo.
swb_log_dir() {
    local project="$1" kind="$2"
    local root="${SWB_LOG_ROOT:-$project/play_log}"
    mkdir -p "$root/$kind"
    if [ "$(realpath -m "$root")" != "$(realpath -m "$project/play_log")" ]; then
        local link="$project/play_log/$kind"
        mkdir -p "$project/play_log"
        if [ -L "$link" ] || [ ! -e "$link" ]; then
            ln -sfn "$root/$kind" "$link"
        else
            echo "[just] note: $link is a directory, not a link to $root/$kind;" \
                 "this run logs to $root/$kind (move the old one with just prune-logs)" >&2
        fi
    fi
    echo "$root/$kind"
}
