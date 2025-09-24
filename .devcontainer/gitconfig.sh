#!/bin/bash
# Bash script to configure git user name/email within the Dev Container copying
# from the user's home directory on the host machine if it exists.

if [ -f ~/homedir/.gitconfig ]; then
    echo "[INFO]: Configuring git user/name within container"
    git config --global user.name "$(git config -f ~/homedir/.gitconfig --get user.name)"
    git config --global user.email "$(git config -f ~/homedir/.gitconfig --get user.email)"
else
    echo "[WARN]: No .gitconfig found in ~/homedir/.gitconfig, skipping automatic git user/name configuration"
fi