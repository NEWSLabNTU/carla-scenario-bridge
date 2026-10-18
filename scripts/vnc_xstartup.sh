#!/bin/sh
# TurboVNC xstartup for the Isaac Sim display (`just isaac-vnc`, :8).
#
# GNOME (Ubuntu session, X11) with VirtualGL: start vncserver with `-vgl` and this
# file as `-xstartup`; TurboVNC's own xstartup.turbovnc then runs the session under
# `vglrun +wm`, so gnome-shell and everything started inside it render OpenGL on the
# NVIDIA GPU through VirtualGL's EGL back end (no 3D X server needed).
#
# This session gets its own dconf database (~/.config/dconf/isaacvnc), so the settings
# below -- 3 static workspaces, no screen lock/blank, autohide dock, no desktop icons --
# do not leak into the GNOME sessions on other displays, which share ~/.config/dconf/user.

# The NVIDIA card (pci 0000:01:00.0); card1 is the AMD iGPU.
VGL_DISPLAY=${VGL_DISPLAY:-$(readlink -f /dev/dri/by-path/pci-0000:01:00.0-card)}
export VGL_DISPLAY
# Vulkan apps (Isaac Sim's Kit) on the NVIDIA ICD too.
VK_ICD_FILENAMES=${VK_ICD_FILENAMES:-/usr/share/vulkan/icd.d/nvidia_icd.json}
export VK_ICD_FILENAMES
case ":$PATH:" in *:/opt/VirtualGL/bin:*) ;; *) PATH=/opt/VirtualGL/bin:$PATH ;; esac
export PATH

D=$HOME/.config/dconf
mkdir -p "$D/isaacvnc.d"
cat > "$D/isaacvnc.d/00-isaacvnc" <<'KEYS'
[org/gnome/mutter]
dynamic-workspaces=false
workspaces-only-on-primary=true

[org/gnome/desktop/wm/preferences]
num-workspaces=3

[org/gnome/desktop/wm/keybindings]
switch-to-workspace-1=['<Control><Alt>1']
switch-to-workspace-2=['<Control><Alt>2']
switch-to-workspace-3=['<Control><Alt>3']
move-to-workspace-1=['<Control><Alt><Shift>1']
move-to-workspace-2=['<Control><Alt><Shift>2']
move-to-workspace-3=['<Control><Alt><Shift>3']

[org/gnome/desktop/session]
idle-delay=uint32 0

[org/gnome/desktop/screensaver]
lock-enabled=false
idle-activation-enabled=false

[org/gnome/desktop/lockdown]
disable-lock-screen=true

[org/gnome/settings-daemon/plugins/power]
sleep-inactive-ac-type='nothing'
idle-dim=false

[org/gnome/desktop/notifications]
show-banners=false

[org/gnome/shell]
disabled-extensions=['ding@rastersoft.com']
welcome-dialog-last-shown-version='999'

[org/gnome/shell/extensions/dash-to-dock]
dock-fixed=false
autohide=true
intellihide=false

[org/gnome/desktop/interface]
enable-animations=false
KEYS
# Defaults, rebuilt every start; changes made inside the session go to user-db:isaacvnc
# and win over them.
dconf compile "$D/isaacvnc-defaults" "$D/isaacvnc.d"
printf 'user-db:isaacvnc\nfile-db:%s\nsystem-db:ibus\n' "$D/isaacvnc-defaults" > "$D/isaacvnc.profile"
DCONF_PROFILE=$D/isaacvnc.profile
export DCONF_PROFILE

TVNC_WM=${TVNC_WM:-ubuntu}
export TVNC_WM
exec /opt/TurboVNC/bin/xstartup.turbovnc
