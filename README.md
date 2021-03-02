# Ros_rasp
This is a ROS package wich receive/publish data for Robotran project. Runing this package alone have no interest. The objectif here is to first run the ros_rob package (see tutorial and informations [here](https://github.com/tuerlinckxt/ros_rob)),this is a ROS package that allow you to run a Robotran project and publish/receive data on ROS topics. Here specially is an example where you can drive a car with the left and right keyboard arrow and publish the torque in the steerwheel on a ROS topic. After that you can run the ros_rasp package on a other device (which have ROS on it and have a complete, bi-directional connectivity with the first device (SSH communication)). Now you are able to publish data to Robotran from an external device and also receive data from Robotran to this external device.  

[[_TOC_]]

## Installation and configuration of ROS 
For installing ROS I suggest you to follow the [official ROS Tutorials](http://wiki.ros.org/fr/ROS/Tutorials) : the [Installation and Configuration](http://wiki.ros.org/fr/ROS/Tutorials/InstallingandConfiguringROSEnvironment) is mandatory. After that, in order to have a better comprehension and management of ROS, you can follow all the beginner level and [Writing a simple publisher/sucriber](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29).  

Note that ROS only support the Linux distribution.

The ROS version used here is *Melodic* and the ROS compiler used is *catkin*. The nodes are written in C++. 

## Network Setup between the two devices 
This is one of the strengths of ROS, a ROS node makes no assumptions about where in the network it runs if there is a complete, bi-directional connectivity between all pairs of machines and if each machine can advertise itself by a name that all other machines can resolve. 

The example here show how to create an SSH communication between a computer with Ubuntu and a Raspberry Pi2 Model B with Ubuntu Mate connected by a ethernet cable. So this section may vary in function of your network configuration but you can do it by your own with some research on internet. The purpose is to obtain the complete, bi-directional connectivity between the devices. [This ROS tutorial](http://wiki.ros.org/ROS/NetworkSetup) will allow you to check that and gives you some issues if you have any problems with the connection. 


1)You must connect the Ethernet cable of the Raspberry to the port of the computer that you are going to use for the configuration of the SSH communication. If you don't have ethernet port on your computer you can use and adaptater ethernet to USB. 
2) Install SSH client and server on the two devices : `$ sudo apt-get install openssh-server openssh-client`
3) Create a fixe IP address for ethernet connection of the laptot.
  * Go to Parameters->Network
  * Go the parameters of the wired used. 
  

<img src="https://git.immc.ucl.ac.be/tuerlinckxt/ros_rasp/-/raw/CAN-develop/img/ipfixe_1.png" width="400">

  * Select *Make it accessible to other devices*
  

<img src="https://git.immc.ucl.ac.be/tuerlinckxt/ros_rasp/-/raw/CAN-develop/img/ipfixe_2.png" width="400">

  * Then in the IPv4 section, set the IP address to **192.168.100.1** (you can choose the value what you want but you need to keep it in mind) and set the netmask to **255.255.255.0** (this address must be the same for all the devices in the network).

<img src="https://git.immc.ucl.ac.be/tuerlinckxt/ros_rasp/-/raw/CAN-develop/img/ipfixe_3.png" width="400"> 

4) Create a fixe IP address for ethernet connection of the Raspberry.
   * You can edit the */etc/network/interfaces* file with : 
   
         auto lo
         iface lo inet loopback
    
          auto eth0
          iface eth0 inet static 
          address 192.168.100.2
          network 192.169.100.0
          netmask 255.255.255.0
          broadcast 192.168.200.255

          allow-hotplug wlan0
          iface wlan0 inet manual
          wpa-conf /etc/wpa_supplicant/wpa_supplicant.conf
    
   * Note that the netmask address is the same and keep in mind the IP address **192.168.100.2**
   * If you want to return to a classical ethernet communication juste change *static* by *dhcp* and put all the addresses in comment.
 5) If not done, enable SSH on the Raspberry. By typing `$ sudo raspi-config` and then go the *Interfacing Options* and enable SSH or with the command line `$ sudo update-rc.d ssh enable && invoke-rc.d ssh start`
 6)  You need to put a name on the IP address in order to use it in a more simple whay. 
   * Just add lines with the IP address and a name (you need to use the same names used here or you will have to make some changements in the *launch_haptic_simulator.py* file) in the */etc/hosts* file of the two devices (you need to have admin rights) :
    
    192.168.100.2   raspberry_ethernet
    192.168.100.1   laptop_ethernet

 7) Reboot the laptop and raspberry to take the changes in account.
 8) Reconfigure the SSH of the Raspberry : `$ sudo dpkg-reconfigure openssh-server`
 9) Sometimes the fire-wall of the computer can make obstruction in the communication between the two devices. You can desable the fire-wall with (need to be done each time the computer is turned on):
 
     `sudo iptables -F`

     `sudo iptables -X`
     
     `sudo iptables -t nat -F`
     
     `sudo iptables -t nat -X`
     
     `sudo iptables -t mangle -F`
     
     `sudo iptables -t mangle -X`
 
 Now the connection is established, you can verify that there is a full bi-directional connectivity between the two devices. [This ROS tutorial](http://wiki.ros.org/ROS/NetworkSetup) will allow you to do that and gives you some issues if you have any problems with the connection. 
   
## Global architecture


<img src="https://git.immc.ucl.ac.be/tuerlinckxt/ros_rasp/-/raw/master/img/globalarchi.JPG" width="600">


## ROS across multiple machines
Now that you have a SSH connection between your two devices (here a laptop and a Raspberry), we are going to setup the ROS environnement. When you run ROS across multiple machines you only need one master, all the other devices are slaves. But the nodes running on slave device must be configured to use the same master. In order to do that we are going to modify the *.bashrc* file of our device.

For this example :
* one master device, here the laptop (IP address = **192.168.100.1** named *laptop_ethernet*)
* one slave device, here the Raspeberry (IP address = **192.168.100.2** named *raspberry_ethernet*)

* Modify the *.bashrc* file of the master by adding this lines at the very beginning of the file : 

      source /opt/ros/melodic/setup.bash
      source $HOME/catkin_ws/devel/setup.bash

      export ROS_IP=laptop_ethernet 
      export ROS_MASTER_URI=http://laptop_ethernet:11311

      echo "ROS_IP :"$ROS_IP
      echo "ROS_MASTER_URI :"$ROS_MASTER_URI
      
   *echo* print the addresses and allows you to verify the addresses when you open a new terminal. *source* allows to take in account in the ROS environnement the modification of the *.bashrc* file when you open a new terminal.   
   
* Modify the *.bashrc* file of the slave by adding this lines at the very beginning of the file : 

      source /opt/ros/melodic/setup.bash
      source $HOME/catkin_ws/devel/setup.bash

      export ROS_IP=raspberry_ethernet 
      export ROS_MASTER_URI=http://laptop_ethernet:11311

      echo "ROS_IP :"$ROS_IP
      echo "ROS_MASTER_URI :"$ROS_MASTER_URI
      
     Note that the *ROS_MASTER_URI* address is the same for the two devices.
     
## Compilation and run instructions
Now that the network and the ROS environnement is setup here is the compilation and run instructions :

1) Follow [this tutorial](https://github.com/tuerlinckxt/ros_rob) for installing and compiling the *ros_rob* package on the computer (don't run it).
2) Download this package in your catkin_ws/src folder in the Rasberry.
3) Go to the catkin_ws folder et compile/build it with :`~/catkin_ws$ catkin_make`
4) Run `$ roscore` on the master device (here the computer).
5) On the Raspberry, run the *talker* node wich send the position and the velocity of the steerwheel to Robotran on the computer and run the *listener* node wich receive the torque of the steerwheel with `$ rosrun ros_rasp talker` and `$ rosrun ros_rasp listener`.
6) On the laptop, run the *exe_Car* node wich run the real-time simulation in Robotran, receive the position and velocity of the steerwheel from the *talker* node and publish the torque on a topic for the *listener* node. 
7) The *exe_Car* node close automatically. Close the *listener* and *talker* node and *roscore* by pressing ctrl+c in each terminal.

 
## CAN communication 

Now that you can send and receive your Robotran's data on an other device (here a Raspberry) it could be usefull to use this data in order to actuate a servomotor for example. Most a servomotor and servovariator use CANopen bus to communicate with other device. This section will explain how to provide CAN capabilty to a Raspberry and provide an example of ROS node with the package KaCanOpen.

### PICAN 2

Althoug the Raspberry runs Linux wich natively manages the SPI-CAN interface via the SocketCAN kernel module, it doesn't have any physical port for CAN communication. A *PiCAN2* from *SK Pang Electronics Ltd* available on [Elektorstore](https://www.elektor.fr/pican-2-can-bus-board-for-raspberry-pi) can be fixed on the Raspberry Pi's GPIO pins and provides a DB9 physical port.

#### Installation and configuration ok the PiCAN2 and CAN interface

In order to be able to send/receive data on the CAN interface you need to install *can-utils* : `$ sudo apt install can-utils`.

For the hardware and software installation of the PiCAN 2 you can follow the [user guide](http://skpang.co.uk/catalog/images/raspberrypi/pi_2/PICAN2UG13.pdf) until page 6 included. 

You can also bring the CAN interface up with `$ sudo /sbin/iplink set can0 up type can bitrate 500000` with *can0* and *500000* the name and the baudrate of the CAN bus but that will be also done in [lexium32A_CANopen.cpp](https://git.immc.ucl.ac.be/tuerlinckxt/ros_rasp/-/blob/CAN-develop/src/lexium32A_CANopen.cpp).

### KaCanOpen package

KaCanOpen is a package provided by *KITmedical* that allows CANopen communication and a brige to the ROS network

#### Installation and compilation of KaCanOpen

You can find all the informations and the instructions of installation and configuration on the [KaCanOpen GitHub](https://github.com/KITmedical/kacanopen).

Here the driver *SocketCAN* has been used. So the terminal command for the compilation is (with *_j1* in order to disable parallelization) : 

    catkin_make -DDRIVER=socket -j1

**Hint** : if you have difficulty to build the KaCanOpen package and your Raspberry "freezes", try to use the argument *-j1* for `catkin_make` (allow to disable parallelization in order to reduce the complexity of compilation). In order to restart the Raspberry after a "freeze" you need to delete the content of the *build* folder of the *catkin workspace* in the SDcard of the Raspberry.

#### EDS file

An EDS file (for Electronic Data Sheet) is a text file based on the CANopen standard profile (like DS301 for communication profile and DSP402 for device profile). It provides a lot of descriptive and communication data for the device. KaCanOpen allows to load a dictonnary structure from this EDS file and to use it in order to send/receive CAN communication. 

For example you can set the velocity of your motor with the function `mymotor.set_entry("Target velocity", 500)` in place of using hexadecimal data for the CAN communication. It allows to make the code easier to write and easier to understand. Moreover, as it is based on CANopen standard profiles, the same code can be used to control other motors (if they also respect the CANopen standard profiles and so only for the object index 6000h).

You can usually find the EDS file of your product on the official website and download it. After that you can simply load the dictonnary structure of your device from this EDS file. You can find an example of how to load and use it in [lexium32A_CANopen.cpp](https://git.immc.ucl.ac.be/tuerlinckxt/ros_rasp/-/blob/CAN-develop/src/lexium32A_CANopen.cpp).






  
   `
 



