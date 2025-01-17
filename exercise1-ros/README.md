# Docker and ROS Setup Guide

## Run Docker Image

```sh
docker run -it kthfsdv/ros-intro
```
The `-it` flag is for an interactive session.

```sh
docker run -it \
    --mount type=bind,source="$(pwd)",target=/src \
    kthfsdv/ros-intro 
```
This binds a shared directory.

```sh
docker run -it \
    --mount type=bind,source="$(pwd)"/src,target=/src \
    --env="DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    kthfsdv/ros-intro
```
To enable visualization, you might need to use `xhost` to port the display. 

```sh
docker exec -it <image name> /bin/bash
```
Connect to a running container.

```sh
source /ros_entrypoint.sh
```

## Creating a Package

```sh
catkin create pkg <name>
```

## Make Your Workspace Visible to the File System

```sh
source devel/setup.bash
rospack find <package name>
```

## Python Script Setup

Create a script at `scripts/<name>.py` and add the following to your `CMakeLists.txt` to ensure the Python script gets installed properly and uses the correct interpreter:

```cmake
catkin_install_python(PROGRAMS scripts/talker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

## Run Packages

Deploy `roscore`, which is responsible for setting up connections between nodes:

```sh
roscore
rosrun <package name> <script name>
