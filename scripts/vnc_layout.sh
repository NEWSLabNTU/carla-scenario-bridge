#!/usr/bin/env bash
# Arrange a run's windows on the Isaac VNC display (`just isaac-vnc`, GNOME + VirtualGL).
#
# Usage: vnc_layout.sh [mode:=tile|ws] [wait:=SECS]   place the windows (default: tile)
#        vnc_layout.sh term title:=<t> cmd:=<command>  a titled gnome-terminal in the session
#                                                      running <command>, then a shell
#
# The display is the machine's: $ISAAC_DISPLAY (default :8), as `just isaac-vnc`.
#
#   tile  one workspace: Isaac Sim top-left (3/5 x 2/3), RViz top-right, terminals along
#         the bottom -- everything in one frame for an x11grab recording
#   ws    one window per workspace: 1 Isaac Sim, 2 RViz, 3 terminals (grid); switch with
#         Ctrl+Alt+1/2/3 (send a window: Ctrl+Alt+Shift+N). x11grab records only the
#         workspace on screen.
#   wait  keep placing for SECS while windows appear (Isaac's takes minutes to come up)
#
# Windows: Isaac Sim = title containing "Isaac Sim"; RViz = WM_CLASS rviz2 or title
# containing "RViz"; terminals = gnome-terminal.
set -euo pipefail
export DISPLAY=${ISAAC_DISPLAY:-:8}
WMCTRL=$(command -v wmctrl || echo "$HOME/.local/opt/wmctrl/usr/bin/wmctrl")
[ -x "$WMCTRL" ] || { echo "vnc_layout: wmctrl not found (apt-get download wmctrl; dpkg-deb -x)" >&2; exit 1; }

# The session's D-Bus, from gnome-shell's environment on this display.
session_bus() {
    local p env
    for p in $(pgrep -x gnome-shell -u "$(id -u)"); do
        env=$(tr '\0' '\n' < "/proc/$p/environ" 2>/dev/null) || continue
        if grep -qx "DISPLAY=$DISPLAY" <<< "$env"; then
            sed -n 's/^DBUS_SESSION_BUS_ADDRESS=//p' <<< "$env"
            return 0
        fi
    done
    return 1
}

if [ "${1:-}" = term ]; then
    title=""; cmd=true
    for a in "${@:2}"; do
        case $a in
            title:=*) title="csb: ${a#title:=}" ;;
            cmd:=*) cmd=${a#cmd:=} ;;
            *) echo "vnc_layout: term takes title:=<t> cmd:=<command>, not '$a'" >&2; exit 2 ;;
        esac
    done
    bus=$(session_bus) || { echo "vnc_layout: no GNOME session on $DISPLAY" >&2; exit 1; }
    # The title stays: an interactive bash whose prompt does not retitle the window.
    rc=${XDG_RUNTIME_DIR:-/tmp}/csb-vnc-term.rc
    cat > "$rc" <<'RC'
[ -f ~/.bashrc ] && . ~/.bashrc
PS1='\u@\h:\w\$ '
PROMPT_COMMAND='printf "\033]0;%s\007" "$CSB_TITLE"'
RC
    CSB_TITLE=$title DBUS_SESSION_BUS_ADDRESS=$bus gnome-terminal --title "$title" -- \
        bash -c 'printf "\033]0;%s\007" "$CSB_TITLE"; echo "\$ $1"; eval "$1"; exec bash --rcfile "$2"' \
        csb-term "$cmd" "$rc"
    exit 0
fi

mode=tile; wait=0
for a in "$@"; do
    case $a in
        mode:=*) mode=${a#mode:=} ;;
        wait:=*) wait=${a#wait:=} ;;
        tile|ws) mode=$a ;;
        *) echo "vnc_layout: unknown argument '$a' (mode:=tile|ws, wait:=SECS)" >&2; exit 2 ;;
    esac
done

# Work area (below the top bar): x y w h
read -r WX WY WW WH < <("$WMCTRL" -d | awk '$2=="*" {split($8,o,","); split($9,s,"x"); print o[1], o[2], s[1], s[2]; exit}')

place() {  # id desktop x y w h
    # Mutter maximizes a window as wide as the screen (Isaac opens at 1920x1080) and
    # ignores a resize until the un-maximize has landed.
    "$WMCTRL" -i -r "$1" -b remove,maximized_vert,maximized_horz,fullscreen
    sleep 0.3
    "$WMCTRL" -i -r "$1" -t "$2"
    "$WMCTRL" -i -r "$1" -e "0,$3,$4,$5,$6"
}

arrange() {
    local list isaac rviz terms n i
    list=$("$WMCTRL" -lx)
    isaac=$(awk '/Isaac Sim/ {print $1; exit}' <<< "$list")
    rviz=$(awk 'tolower($3) ~ /rviz/ || /RViz/ {print $1; exit}' <<< "$list")
    mapfile -t terms < <(awk -v a="$isaac" -v b="$rviz" \
        'tolower($3) ~ /gnome-terminal/ && $1 != a && $1 != b {print $1}' <<< "$list")
    n=${#terms[@]}
    case $mode in
    tile)
        local ih=$(( WH * 2 / 3 )) iw=$(( WW * 3 / 5 ))  # RViz is at least ~750 px wide
        [ -n "$isaac" ] && place "$isaac" 0 "$WX" "$WY" "$iw" "$ih"
        [ -n "$rviz" ] && place "$rviz" 0 $(( WX + iw )) "$WY" $(( WW - iw )) "$ih"
        for ((i = 0; i < n; i++)); do
            place "${terms[i]}" 0 $(( WX + i * WW / n )) $(( WY + ih )) $(( WW / n )) $(( WH - ih ))
        done ;;
    ws)
        [ -n "$isaac" ] && place "$isaac" 0 "$WX" "$WY" "$WW" "$WH"
        [ -n "$rviz" ] && place "$rviz" 1 "$WX" "$WY" "$WW" "$WH"
        local cols=$(( n > 2 ? 2 : (n > 0 ? n : 1) )) rows=$(( (n + 1) / 2 ))
        [ "$rows" -ge 1 ] || rows=1
        for ((i = 0; i < n; i++)); do
            place "${terms[i]}" 2 $(( WX + (i % cols) * WW / cols )) $(( WY + (i / cols) * WH / rows )) \
                $(( WW / cols )) $(( WH / rows ))
        done ;;
    *) echo "vnc_layout: mode tile|ws, not '$mode'" >&2; exit 2 ;;
    esac
    echo "vnc_layout[$mode] $DISPLAY: isaac=${isaac:--} rviz=${rviz:--} terminals=$n"
}

arrange
end=$(( SECONDS + wait ))
while [ "$SECONDS" -lt "$end" ]; do
    sleep 5
    arrange > /dev/null
done
