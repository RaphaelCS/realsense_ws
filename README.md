## Installation

```shell
cd ~
git clone https://github.com/RaphaelCS/realsense_ws.git

cd realsense_ws
catkin_make
```

If you use zsh:
```shell
echo "source ~/realsense_ws/devel/setup.zsh" >> ~/.zshrc
```

If you use bash:
```shell
echo "source ~/realsense_ws/devel/setup.bash" >> ~/.bashrc
```

## Run

```shell
roslaunch realsense run.launch
python ~/realsense_ws/src/realsense/scripts/test.py
```