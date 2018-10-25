# firebirdv-ros-package

**Note1**: The steps below are for **ROS Kinetic** installed on **Ubuntu Xenial (16.xx)**. To install ROS Kinetic follow [this](http://wiki.ros.org/kinetic/Installation/Ubuntu).

#### 1. Install the Arduino IDE
```
sudo apt-get update
sudo apt-get install arduino arduino-core
```
#### 2. Give Administrator privileges to the current user for the Arduino Permission Checker
```
sudo usermod -a -G dialout your_user_here
```

#### 3. Open and check the Arduino IDE
```
arduino
```

#### 4. Install ROSSerial for Kinetic
```
sudo apt-get install ros-kinetic-rosserial-arduino
sudo apt-get install ros-kinetic-rosserial
```

#### 5. Git clone this repository into your catkin workspace folder
```
cd ~/<catkin_ws>/src/git clone https://github.com/tushar-semwal/firebirdv-ros-package.git

```

#### 6. Install the ros_lib library
1. Go to the arduino sketchbook/libraries folder
`cd ~/sketchbook/libraries`
2. If not found, create one using `mkdir ~/sketchbook/libraries` and execute `cd ~/sketchbook/libraries`
3. Execute this command inside the sketchbook/libraries folder: `rosrun rosserial_arduino make_library.py .`
4. Note there is a DOT (.) after make_library.py in the above command.
5. Close the Arduino IDE and check the **Files->Examples->ros_lib**. If you can see it, congrats you have installed both ros_lib and FirebirdV libraries together.
