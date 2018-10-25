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
#### 3a. Setup the Arduino IDE for Firebird V 
1. Open the Arduino IDE and go to Tools->Boards. You will see a board for AtMega 2560. However, this board is set at F_CPU (cpu frequency) of 16MHz but the frequency of mega chip on Firebird robot is set to 14.7456MHz. Thus we need to create a new board configuration for the required frequency, in order to compile arduino sketches for Firebird V robot.
2. The new board configuration is required to be appended to **boards.text** file which can be found by following the steps below:
   1. Select an Arduino AVR Boards board from the Tools > Board menu.
   2. File > Examples > EEPROM > eeprom_clear
   3. Sketch > Show Sketch Folder
   4. Move up 4 folder levels and you will reach the location of boards.txt for the active version of Arduino AVR Boards.
3. Open the **boards.txt** file using your favourite text editor and append this to the file. Save it then.:
```
firebird.name=Firebird V Mega 2560 Robot

firebird.vid.0=0x2341
firebird.pid.0=0x0010
firebird.vid.1=0x2341
firebird.pid.1=0x0042
firebird.vid.2=0x2A03
firebird.pid.2=0x0010
firebird.vid.3=0x2A03
firebird.pid.3=0x0042
firebird.vid.4=0x2341
firebird.pid.4=0x0210
firebird.vid.5=0x2341
firebird.pid.5=0x0242

firebird.upload.tool=avrdude
firebird.upload.maximum_data_size=8192

firebird.bootloader.tool=avrdude
firebird.bootloader.low_fuses=0xFF
firebird.bootloader.unlock_bits=0x3F
firebird.bootloader.lock_bits=0x0F

firebird.build.f_cpu=14745600L

firebird.build.core=arduino
firebird.build.variant=mega
# default board may be overridden by the cpu menu
firebird.build.board=AVR_MEGA2560

## Arduino/Genuino Mega w/ ATmega2560
## -------------------------
firebird.menu.cpu.atmega2560=ATmega2560 (Mega 2560)

firebird.menu.cpu.atmega2560.upload.protocol=wiring
firebird.menu.cpu.atmega2560.upload.maximum_size=253952
firebird.menu.cpu.atmega2560.upload.speed=115200

firebird.menu.cpu.atmega2560.bootloader.high_fuses=0xD8
firebird.menu.cpu.atmega2560.bootloader.extended_fuses=0xFD
firebird.menu.cpu.atmega2560.bootloader.file=stk500v2/stk500boot_v2_mega2560.hex

firebird.menu.cpu.atmega2560.build.mcu=atmega2560
firebird.menu.cpu.atmega2560.build.board=AVR_MEGA2560
```
4. That's it! Close the IDE and open it again. You will be able to select your new board with name as Firebird V Mega 2560 Robot. You will be able to see the same as shown below:
![sample_ide](https://github.com/tushar-semwal/Arduino-FirebirdV/blob/master/images/sample.png)

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
5. Close and reopen the Arduino IDE and check the **Files->Examples->ros_lib**. If you can see it, congrats you have installed both ros_lib and FirebirdV libraries together.

#### 7. 
