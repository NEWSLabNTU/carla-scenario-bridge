#!/usr/bin/env bash
# Build the README showreel GIFs from the 2026-10-12 screencasts (roadmap 027).
#
#   scripts/make_showreel.sh <screencast dir> [out dir]
#
# <screencast dir> holds town01_pedestrian.mp4 and town01_two_av.mp4 as recorded on display
# :5 (1920x1080, re-encoded 1280x720; layout in that directory's README). Crops below are in
# the 1280x720 frame: the CARLA chase view (left) and the RViz windows (right), without
# desktop, title bars or the terminal. Output: showreel_pedestrian.gif, showreel_two_av.gif.
set -euo pipefail
src="$1"
out="${2:-$(dirname "$0")/../docs/media}"
mkdir -p "$out"
font=/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf
label() { # text x y
    echo "drawtext=fontfile=$font:text='$1':x=$2:y=$3:fontsize=18:fontcolor=white:box=1:boxcolor=black@0.55:boxborderw=5"
}
gif() { # input start end filtergraph(ending in [v]) output fps
    local in="$1" from="$2" to="$3" graph="$4" dst="$5" fps="$6"
    ffmpeg -y -loglevel error -ss "$from" -to "$to" -i "$in" -filter_complex \
        "$graph;[v]fps=$fps,split[a][b];[a]palettegen=max_colors=96:stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=4:diff_mode=rectangle" \
        "$dst"
    echo "$dst: $(du -h "$dst" | cut -f1)"
}

# Kept small for the README (each loads before it plays): 640 px, 6 fps, trimmed.
# Pedestrian: driving, the stop for the crossing pedestrian (obstacle_stop), the restart.
# 0:24-1:48 at 8x -> ~10 s.
gif "$src/town01_pedestrian.mp4" 24 108 \
    "[0:v]setpts=PTS/8,split[c][r];\
[c]crop=493:250:47:42,scale=-2:360,$(label CARLA 10 10)[carla];\
[r]crop=540:540:640:100,scale=360:360,$(label Autoware 10 10)[rviz];\
[carla][rviz]hstack,scale=640:-2[v]" \
    "$out/showreel_pedestrian.gif" 6

# Two AVs: the ego and bg_av_1, each a full Autoware in its own ROS domain, one scenario.
# 0:35-2:05 at 6x -> ~15 s.
gif "$src/town01_two_av.mp4" 35 125 \
    "[0:v]setpts=PTS/6,split=3[c][e][b];\
[c]crop=493:250:47:42,scale=-2:360,$(label 'CARLA (ego)' 10 10)[carla];\
[e]crop=540:245:640:95,scale=-2:180,$(label 'Autoware - ego' 10 8)[ego];\
[b]crop=540:245:640:420,scale=-2:180,$(label 'Autoware - bg_av_1' 10 8)[bg];\
[ego][bg]vstack[right];[carla][right]hstack,scale=640:-2[v]" \
    "$out/showreel_two_av.gif" 6
