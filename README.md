## Installation

```shell
cd ~
git clone https://github.com/RaphaelCS/realsense

cd realsense_ws
catkin_make
echo "source ~/realsense_ws/devel/setup.zsh" >> ~/.zshrc
```

## Run

```shell
roslaunch realsense run.launch
python ~/realsense_ws/realsense/scripts/test.py
```