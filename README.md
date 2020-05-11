## Installation

Download **realsense_ws** into ~
```shell
cd realsense_ws
catkin_make
echo "source ~/realsense_ws/devel/setup.zsh" >> ~/.zshrc
```

## Run

```shell
roslaunch realsense run.launch
python ~/realsense_ws/src/realsense/scripts/test.py
```