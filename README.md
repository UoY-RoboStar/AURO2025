# AURO2025
This is the repository for the AURO course's practicals.

# Dev containers
If you have Docker and VS Code installed, you can use the following instructions to spin up
the Dev Container configuration contained in this repository to get started with ROS2.

1. Open VS Code
2. Ensure that the [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) has been installed.
3. In VS Code, open the command palette (eg. using the key combination `Shift+P` on Windows, or `Command+Shift+P`) and type `Dev Containers: Clone Repository in Container Volume`.
3. Then select `GitHub` and select this repository's name/URL.
4. Wait for the container to finish building.

## Dev Container Configurations available

* `.devcontainer/devcontainer.json`: Uses VNC to expose a desktop and allow interaction with GUI applications via a VNC client or web browser at http://localhost:6080/
* `.devcontainer/wsl/devcontainer.json`: Allows use of WSL2 to interact with GUI applications showing them natively on the Windows desktop. This option also takes advantage of GPU available on the host for rendering by leveraging support for translating OpenGL calls to Direct3D.
* `.devcontainer/other/devcontainer.json`: Allows use of X11 for natively running the container on Linux with Docker. GUI applications are shown on the host operating system. See section below for additional configuration that is required.

### auro-linux (Linux X11 native)

The configuration assumes that the X11 cookie is available at `~/.X11authority`. Therefore, to use this configuration you should configure a symlink from `~/.X11authority` to `${XAUTHORITY}` when the session is started. For GNOME, this can be achieved, for example, by adding a new file under `~/.config/autostart` with the following content:

```
[Desktop Entry]
Type=Application
Exec=/bin/bash -c "ln -fs $XAUTHORITY ~/.X11authority"
Hidden=false
NoDisplay=true
X-GNOME-Autostart-enabled=true
Name=CreateSymlink
Comment=Creates XAuthority symlink
```

Usually, this file is stored at `~/.Xauthority` when running under X11, however sessions running under Wayland, which is the default in recent versions of Ubuntu, expose an X Server via Xwayland, with the `XAuhtority` file being created at an unpredictable location, usually under `$XDG_RUNTIME_DIR/.mutter-Xwaylandauth.XXXXX`. Because any mounts configured for a Docker container become permanent, it becomes impractical to change them later on from the `devcontainer.json`, unless the container is deleted. For this reason, the Dev Container configuration for Linux requires that the `.XAuthority` file is available at `/.X11authority` instead.

Depending on your GPU, you may also need to change the device under `runArgs` to use the appropriate interface. For Intel/AMD, the default `/dev/dri`
should be adequate.

### auro-wsl
If using this configuration on a device that has an NVIDIA card, you may need to set the environment variable `MESA_D3D12_DEFAULT_ADAPTER_NAME` to `NVIDIA`, for example, using the command:
```
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
```
You could make this permanent by extending the [`containerEnv`](https://github.com/UoY-RoboStar/AURO2025/blob/e7f7ab9b6657aad4d8715cd908829bc9dbb09e73/.devcontainer/wsl/devcontainer.json#L28-L33) section of the `.devcontainer/wsl/devcontainer.json` file, though note that you may need to change it back to use the configuration in the CS labs. You could, alternatively, consider creating a new configuration in your own repository.

### auro-vnc
To use auro-vnc on a Mac with Apple Silicon (arm64) machine, because the underlying Docker image is for the amd64 architecture, the first time you use the Dev Container you will need to pull the Docker image from the command line using the following command:

```
docker pull --platform linux/amd64 ghcr.io/uoy-robostar/ros2-tb3/auro-dev:latest
```

Afterwards, if needed, you should be able to update the image using the Docker Desktop interface. If not, you can use the above command again.
