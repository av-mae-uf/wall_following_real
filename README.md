# wall_following_real
Code to implemnet the wall following problem on the class vehicles

## Installation/Build
Create a development workspace by typing the following when in your home directory. Replace the text within the <> to be whatever you want.
```bash
mkdir -p <your_dev_ws>/src
```
Move into the **src** directory.
```bash
cd <your_dev_ws>/src
```

Download the repository using
```bash
git clone https://github.com/av-mae-uf/wall_following_real.git
```
The wall_following_sim package uses the ackermann_msgs package.  Add the following:
```bash
sudo apt install ros-humble-ackermann-msgs
```
Change your directory to your root workspace directory (*~/<your_dev_ws>*) and build the workspace.
```bash
colcon build
``` 
Source your workspace with

source install/setup.bash
```
## How to run code
hic window.
Source the .bashrc file.  Note: It will be sourced every time you open a new terminal window from here on.

Run the command below to see the simulation at work.
```bash
ros2 launch wall_following_sim wall_sim_launch.py
