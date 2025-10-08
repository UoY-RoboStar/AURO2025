#!/bin/bash

# e.g. DISPLAY=:0.0 or :1 or localhost:0.0 or 192.168.1.2:0
disp="$DISPLAY"

# Strip off any hostname part (if present)
# e.g. "host:0.0" → "0.0", ":0.0" stays "0.0"
disp_nohost="${disp##*:}"

# Now disp_nohost might be "0", "0.0", "1", etc.
# Extract integer display number before any dot
dispnum="${disp_nohost%%.*}"

# Construct socket path
socket_path="/tmp/.X11-unix/X${dispnum}"

# For rootless docker ensure that permissions are set so that local UID has correct permissions
# irrespective of the mapping that Docker does on host system for these files. For rootless docker
# this is a must, as only root from within container is mapped to host user.
sudo setfacl -m u:$(id -u):r ~/.Xauthority
sudo setfacl -m u:$(id -u):rw ${socket_path}