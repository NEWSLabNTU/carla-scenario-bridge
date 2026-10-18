# Autoware packages carried with fixes Autoware 1.5.0 does not have yet (roadmap 042).
#
# Sourced by the justfile, never run. One pinned commit of the NEWSLabNTU fork of
# autoware_universe, fetched sparse (only the package directories below) into
# third_party/autoware_overlay/src and built there against Autoware 1.5.0 alone. The ego
# stack sources the result right after Autoware, so these packages shadow Autoware's own
# and nothing else changes.
#
# To carry another fix: commit it on a csb/* branch of the fork based on the release's tag,
# push, set the pin to that commit, add the package directory, `just build-autoware-overlay`.
# To retire one: drop its directory when an Autoware release ships the fix.

CSB_AUTOWARE_OVERLAY_REPO=https://github.com/NEWSLabNTU/autoware_universe.git
# Branch csb/blind-spot-right-turn-0.48.0: tag 0.48.0 (Autoware 1.5.0's package versions)
# plus the blind_spot right-turn fix, autoware_universe#13509.
CSB_AUTOWARE_OVERLAY_REV=5c617b0a11df1854667563357fb04bd488bd8ce9
CSB_AUTOWARE_OVERLAY_PACKAGES=(
    planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module
)

# Fetch the pinned commit and build the packages. $1: overlay dir, $2: Autoware setup.bash
csb_autoware_overlay_build() {
    local dir="$1" autoware_setup="$2"
    local src="$dir/src/autoware_universe"
    if [ ! -d "$src/.git" ]; then
        mkdir -p "$dir/src"
        git clone --filter=blob:none --sparse --no-checkout "$CSB_AUTOWARE_OVERLAY_REPO" "$src"
    fi
    git -C "$src" sparse-checkout set "${CSB_AUTOWARE_OVERLAY_PACKAGES[@]}"
    if ! git -C "$src" cat-file -e "$CSB_AUTOWARE_OVERLAY_REV^{commit}" 2>/dev/null; then
        git -C "$src" fetch --filter=blob:none origin "$CSB_AUTOWARE_OVERLAY_REV"
    fi
    git -C "$src" -c advice.detachedHead=false checkout --detach "$CSB_AUTOWARE_OVERLAY_REV"
    (
        set -e
        cd "$dir"
        source /opt/ros/humble/setup.bash
        source "$autoware_setup"
        # A fresh build each time: a stale package left over from an earlier package list
        # would keep shadowing Autoware's.
        rm -rf build install log
        colcon build --base-paths src \
            --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
    )
    echo "$CSB_AUTOWARE_OVERLAY_REV" > "$dir/install/.csb_overlay_rev"
    echo "[autoware-overlay] built $CSB_AUTOWARE_OVERLAY_REV:"
    printf '    %s\n' "${CSB_AUTOWARE_OVERLAY_PACKAGES[@]}"
}

# Source the overlay into the calling shell, after Autoware's setup. On by default when
# built; AUTOWARE_OVERLAY=0 runs the stock packages. $1: overlay dir
csb_autoware_overlay_source() {
    local dir="$1"
    case "${AUTOWARE_OVERLAY:-1}" in
        0|false|off)
            echo "[just] AUTOWARE_OVERLAY=0: stock Autoware packages, no overlay" >&2
            return 0 ;;
    esac
    if [ ! -f "$dir/install/local_setup.bash" ]; then
        echo "[just] WARNING: Autoware overlay not built; running stock Autoware, which has" >&2
        echo "[just]   the blind_spot right-turn bug (autoware_universe#13509)." >&2
        echo "[just]   Build it: just build-autoware-overlay (roadmap 042)" >&2
        return 0
    fi
    local built
    built=$(cat "$dir/install/.csb_overlay_rev" 2>/dev/null || true)
    if [ "$built" != "$CSB_AUTOWARE_OVERLAY_REV" ]; then
        echo "[just] WARNING: Autoware overlay built from ${built:-an unknown commit}," >&2
        echo "[just]   pinned $CSB_AUTOWARE_OVERLAY_REV: run just build-autoware-overlay" >&2
    fi
    # local_setup: the overlay alone, on top of the Autoware already sourced
    source "$dir/install/local_setup.bash"
}
