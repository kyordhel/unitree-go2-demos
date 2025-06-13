# Quick Setup

ROS2 directory structure is quite strict.
Make sure to clone this repo inside a some_workspace/src directory.

We start at `ros2_ws/src`
Run the following commands:

```bash
git clone
cd go2_demo
./fetch-unitree-pkgs.sh
```

This will download all the packages needed to build.
Now let's build it. Building must be done in `ros2_ws`

```bash
git clone
cd ../..
colcon build
```
