# ROS2 Workspace
Workspace for `CMPT720` running **ROS2 Humble**
- Fork of https://github.com/athackst/vscode_ros2_workspace/tree/humble

In this course, we use Docker together with VS Code Dev Containers to provide a consistent and reliable development environment for ROS 2. 
Robotics software is particularly sensitive to differences in system configuration, and this approach helps us avoid many common issues.

## Dev Container Setup
1. Make sure Docker, VS Code, and VS Code Remote Containers extension are installed. See [Docker Setup](#docker-setup)
2. Clone this the `cmpt720_sp26` branch of this repo

```bash
git clone -b cmpt720_sp26 git@github.com:SFU-MARS/ros2_ws.git
```

2. See [these instructions](https://github.com/SFU-MARS/ros2_tutorial/wiki/Building-and-using-the-dev-container)

3. See [this article](https://www.allisonthackston.com/articles/vscode-docker-ros2.html) for deeper insight

## Docker Setup
Before proceeding, ensure the following are installed on your system:
* [Docker](https://docs.docker.com/engine/install/)
* [Visual Studio Code](https://code.visualstudio.com/)
* [VS Code Remote Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)


### Docker Post-Installation (Linux)

To run Docker without sudo and allow VS Code to connect to containers, follow the official Docker post-installation steps:

https://docs.docker.com/engine/install/linux-postinstall/

Summary:
```bash
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker
```




# Usage