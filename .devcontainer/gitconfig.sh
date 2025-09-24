#!/bin/bash
# Bash script to configure git user name/email within the Dev Container based
# on either environment variables passed or .gitconfig under ~/homedir/AppData/Roaming/.gitconfig.

if [ -z "$(git config user.name)" ]; then
    if [ -f ~/homedir/AppData/Roaming/.gitconfig ]; then
        echo "[INFO]: Configuring git user within container based on that in '~/homedir/AppData/Roaming/.gitconfig'"
        git config --global user.name "$(git config -f ~/homedir/AppData/Roaming/.gitconfig --get user.name)"
    else
        if [ -n "$USERNAME" ]; then
            echo "[INFO]: Configuring git user.name within container to '$USERNAME'"
            git config --global user.name "$USERNAME"
        else
            echo "[WARN]: Unable to configure .gitconfig automatically for user.name. You may need to configure it manually inside the Dev Container."
        fi
    fi
fi

if [ -z "$(git config user.email)" ]; then
    if [ -f ~/homedir/AppData/Roaming/.gitconfig ]; then
        echo "[INFO]: Configuring git user within container based on that in '~/homedir/AppData/Roaming/.gitconfig'"
        git config --global user.email "$(git config -f ~/homedir/AppData/Roaming/.gitconfig --get user.email)"
    else
        if [ -n "$USERNAME" ]; then
           if [[ "$USERDNSDOMAIN" == "ITS.YORK.AC.UK" ]]; then
                echo "[INFO]: Configuring git user.email within container to '$USERNAME@york.ac.uk'"
                git config --global user.email "$USERNAME@york.ac.uk"
            else
                echo "[WARN]: Unable to configure user.email automatically as running outside York ITS network. You may need to configure it manually inside the Dev Container."
            fi
        else
            echo "[WARN]: Unable to configure .gitconfig automatically for user.email. You may need to configure it manually inside the Dev Container."
        fi
    fi
fi