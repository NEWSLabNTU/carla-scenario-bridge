# Autoware packages carried with fixes Autoware 1.5.0 does not have yet (roadmap 042).
#
# Sourced by the justfile, never run. Each source is one pinned commit of a NEWSLabNTU fork
# (autoware_universe, autoware_core), fetched sparse (only its package directories below) into
# third_party/autoware_overlay/src/<name> and built there against Autoware 1.5.0 alone. The
# ego stack sources the result right after Autoware, so these packages shadow Autoware's own
# and nothing else changes.
#
# To carry another fix: commit it on a csb/* branch of the fork based on the release's tag,
# push, set that source's pin to the commit and add the package directory (a new fork: a new
# source), `just build-autoware-overlay`. To retire one: drop its directory (or its source)
# when an Autoware release ships the fix.

# Each source: <name> <fork> <commit> <package dir>...
SWB_AUTOWARE_OVERLAY_SOURCES=(
    # Branch csb/blind-spot-right-turn-0.48.0: tag 0.48.0 (Autoware 1.5.0's package versions)
    # plus the blind_spot right-turn fix, autoware_universe#13509.
    "autoware_universe https://github.com/NEWSLabNTU/autoware_universe.git
        5c617b0a11df1854667563357fb04bd488bd8ce9
        planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module"
    # Branch csb/ndt-step-reversal-1.5.0: tag 1.5.0 plus NDT's step bound halved when the step
    # reverses, so a two-point cycle no longer runs to max_iterations and its WARN (an emergency
    # stop in Autoware 1.5's graph, docs/issues/002).
    "autoware_core https://github.com/NEWSLabNTU/autoware_core.git
        fe45f964d8e03b899e1c87593fe76e1d8e5f9952
        localization/autoware_ndt_scan_matcher"
)

# The pins, one line: what install/.swb_overlay_rev records
swb_autoware_overlay_revs() {
    local src
    for src in "${SWB_AUTOWARE_OVERLAY_SOURCES[@]}"; do
        read -r -a f <<< "$(echo $src)"
        printf '%s@%s ' "${f[0]}" "${f[2]}"
    done
}

# Fetch the pinned commits and build the packages. $1: overlay dir, $2: Autoware setup.bash
swb_autoware_overlay_build() {
    local dir="$1" autoware_setup="$2"
    local spec f name repo rev src
    mkdir -p "$dir/src"
    for spec in "${SWB_AUTOWARE_OVERLAY_SOURCES[@]}"; do
        read -r -a f <<< "$(echo $spec)"
        name=${f[0]} repo=${f[1]} rev=${f[2]}
        src="$dir/src/$name"
        if [ ! -d "$src/.git" ]; then
            git clone --filter=blob:none --sparse --no-checkout "$repo" "$src"
        fi
        git -C "$src" sparse-checkout set "${f[@]:3}"
        if ! git -C "$src" cat-file -e "$rev^{commit}" 2>/dev/null; then
            git -C "$src" fetch --filter=blob:none origin "$rev"
        fi
        git -C "$src" -c advice.detachedHead=false checkout --detach "$rev"
    done
    (
        set -e
        cd "$dir"
        source /opt/ros/humble/setup.bash
        source "$autoware_setup"
        # A fresh build each time: a stale package left over from an earlier package list
        # would keep shadowing Autoware's.
        rm -rf build install log
        colcon build --base-paths src --allow-overriding $(colcon list --base-paths src -n) \
            --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
    )
    swb_autoware_overlay_revs > "$dir/install/.swb_overlay_rev"
    echo "[autoware-overlay] built:"
    for spec in "${SWB_AUTOWARE_OVERLAY_SOURCES[@]}"; do
        read -r -a f <<< "$(echo $spec)"
        printf '    %s %s: %s\n' "${f[0]}" "${f[2]:0:7}" "${f[*]:3}"
    done
}

# Source the overlay into the calling shell, after Autoware's setup. On by default when
# built. $1: overlay dir; $2: true (default) or false, a recipe's autoware_overlay:= argument
# (false runs the stock packages).
swb_autoware_overlay_source() {
    local dir="$1"
    case "${2:-true}" in
        0|false|off)
            echo "[just] autoware_overlay:=false: stock Autoware packages, no overlay" >&2
            return 0 ;;
        1|true|on) ;;
        *) echo "[just] autoware_overlay:=$2 is not true or false" >&2; exit 2 ;;
    esac
    if [ ! -f "$dir/install/local_setup.bash" ]; then
        echo "[just] WARNING: Autoware overlay not built; running stock Autoware, which has" >&2
        echo "[just]   the blind_spot right-turn bug (autoware_universe#13509) and NDT's" >&2
        echo "[just]   two-point cycle (an emergency stop now and then, docs/issues/002)." >&2
        echo "[just]   Build it: just build-autoware-overlay (roadmap 042)" >&2
        return 0
    fi
    local built pinned
    built=$(cat "$dir/install/.swb_overlay_rev" 2>/dev/null || true)
    pinned=$(swb_autoware_overlay_revs)
    if [ "$built" != "$pinned" ]; then
        echo "[just] WARNING: Autoware overlay built from ${built:-unknown commits}," >&2
        echo "[just]   pinned $pinned: run just build-autoware-overlay" >&2
    fi
    # local_setup: the overlay alone, on top of the Autoware already sourced
    source "$dir/install/local_setup.bash"
}
