# User manual and tutorial for the haptic feedback simulator with ROS-Robotran

This file will explain the different steps to be carried out for using the haptic feedback simulator of the UCL. The purpose of this application is to drive a vehicle (simulated on a computer) with a real steering wheel and receive the torque feedback back in the steering wheel in order to create a realistic driving-simulator.

First there is a small introduction and explication on the functioning of the simulator.

Then there will be some information on installing and using the following software : 

  * Robotran : this software take care about the simulation of a multi-body model (for this application it will be kart, car, ...)
  * ROS :  this software is used to carry on the communication between the between different devices. 

<!---
Mettre lien vers memoire pour plus d'informations sur Robotran et ROS ?
-->

Then some instructions on how to install and compile the *ros_rob* package on your computer. 

After that the tutorial will show you how to connect your computer to the Raspberry used for the simulator. 

The next step explains to you how to turn on the servo-drive and servo-motor of the simulator and how to launch and use the application.

Finally, the last part explains how to add its own multi-body vehicle model into the application.

**Note** : For this example you will need a computer runing Linux Ubuntu (the example here is done with Ubuntu 18.04.4 LTS).

## Introduction

<!--- Mettre les noms des gens ? -->
Here is a presentation of the haptic feedback simulator :

<img src="https://git.immc.ucl.ac.be/tuerlinckxt/ros_rasp/-/raw/CAN-develop/simu_complet.jpg" width="600">

With : 

 1. A seat *Sabelt*.
 2. A steering wheel connected to a sero-motor.
 3. A screen displaying a graphical interface of the simulation of the vehicle.
 4. A servo-drive used for the communication between the servo-motor and the Raspberry.
 5. A switch to turn on/off the servo-motor.
 6. A Raspberry with a PiCAN2 used for the communication between the servo-drive and the computer.
 7. A computer running the simulation of the vehicle.
 

The different steps of communication are the following : 

  1. The driver steers the car using the steering wheel. 
  2. A coder include in the servo-motor measure the position and velocity of the steering wheel. The servo-drive sends those data to the Raspberry through a CANopen protocol of communication. 
  3. The Raspberry receive the datas and sends it to the sofware Robotran (runing on the computer) through a ROS protocol of communication (called *topic*). 
  4. Robotran, which is simulating the dynamics of a vehicle, use the datas to compute the torque feeback in the steering wheel.
  5. It returns the value to the Raspberry through ROS communication. Then the Raspberry sends a CANopen command to the servo-drive which set the computed torque into the servo-motor. 
 

The driver is now able to steer the vehicle and feel the torque feedback.

### Hardware architecture

The following diagram shows the various hardware components, the physical connections between them as well as their necessary power supply :

<img src="https://git.immc.ucl.ac.be/tuerlinckxt/ros_rasp/-/raw/CAN-develop/archi_phy_hapt.JPG" width="750">

## Installation and configuration of Robotran
For using Robotran on Linux you need to install [MBsysPad](http://robotran-doc.git-page.immc.ucl.ac.be/Installation-Instruction/Linux/MBsysPad.html) and [MBsysC](http://robotran-doc.git-page.immc.ucl.ac.be/Installation-Instruction/Linux/MBsysC.html) in order to use the C/C++ version of Robotran. 

To understand the multibody system dynamics used here you can read the [Robotran Basics](https://www.robotran.be/images/files/Robotran_basics.pdf). 

In order to have a better understanding and management of Robotran you can follow the [Getting Started Tutorial in C/C++](http://robotran-doc.git-page.immc.ucl.ac.be/TutorialModellingFeatures/output_tuto/c-code/). If you want to go further you can also follow the [Equilibirum and Modal(not used here) module of Robotran Tutorial](http://robotran-doc.git-page.immc.ucl.ac.be/TutorialModellingFeatures/Modules/output_mod/c-code/). 

This package also use the real-time features of Robotran so please follow [this tutorial](https://www.robotran.be/tutorial/realtime/).

Finally, in order to link Robotran and ROS, we need to use multithreading. Update the package index `$ sudo apt-get update` and install the pthread library `$ sudo apt-get install libpthread-stubs0-dev`.


## Installation and configuration of ROS 

For installing ROS follow [this link](http://wiki.ros.org/hydro/Installation/Ubuntu). To configure your ROS environnement follow the [Installation and Configuration](http://wiki.ros.org/fr/ROS/Tutorials/InstallingandConfiguringROSEnvironment) with one constraint, you must create your catkin workspace (*catkin_ws*) in your */home/user_name/* folder (in order to allow Robotran to find the correct path to an executable *open_gl_exe* mandatory for the real-time simulation). 

After that, in order to have a better comprehension and management of ROS, you can follow all the beginner level from [official ROS Tutorials](http://wiki.ros.org/fr/ROS/Tutorials) and [Writing a simple publisher/sucriber](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29).  

Note that this ROS-Robotran application has only be written and tested on Linux distribution. 

The ROS version used here is *Melodic* and the ROS compiler used is *catkin*. The nodes are written in C++ except one in Python. 

## Installation and compilation of the *ros_rob* package

1) Download the [ros_rob package](https://git.immc.ucl.ac.be/tuerlinckxt/ros_rob/-/tree/master) in your catkin_ws/src folder.
2) Go to the CMakeLists of each Robotran project  (for example : /home/*user_name*/catkin_ws/src/ros_rob/src/Car/workR/CMakeLists.txt) and set the correct way to the MBsysC folder with /home/*user_name*/.robotran/mbsysc-dev/MBsysC/.
3) Go to the catkin_ws folder et compile/build it with :`~/catkin_ws$ catkin_make`

## Setup the communication between the Raspberry and the computer

In this application, the Raspberry is used to have CANopen communication capability (with an extension *PiCAN 2* fixed on the GPIO pins of the Raspberry). And in order to use the strength of the ROS communication, we need to create a complete, bi-directional connectivity between the Raspberry and the computer. 

### SSH communication

The example here show how to create an SSH communication between the computer with Ubuntu and the Raspberry Pi2 Model B with Ubuntu Mate connected by a ethernet cable. So this section may vary in function of your network configuration but you can do it by your own with some research on internet. The purpose is to obtain the complete, bi-directional connectivity between the devices. [This ROS tutorial](http://wiki.ros.org/ROS/NetworkSetup) will allow you to check that and gives you some issues if you have any problems with the connection. 

1) You must connect the Ethernet cable of the Raspberry to the port of the computer that you are going to use for the configuration of the SSH communication. If you don't have ethernet port on your computer you can use and adaptater ethernet to USB. 
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
 6) You need to put a name on the IP address in order to use it in a more simple whay. 
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
   
 (Optionnal) We can also increase the robustness of the connection and create an authorized-key on the laptop :
 
  1. Create an ssh-key on the laptop :
  
     `ssh-keygen -t rsa`

  2. Enter the file in wich save the key and a passphrase if you want.
  3. Copy the public key into the new machine's authorized_keys file, for our example it will be :
  
     `ssh-copy-id -i <path-to-the-file-created-at-point-2> tuerlinckxt@192.168.100.2`

### ROS across multiple machines

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
     
There is a *roslaunch* function of ROS wich allows to launch nodes on different devices at the same time is not used in this application because we need interaction and displaying with the Linux terminal of the two devices wich is not possible with *roslaunch*. However you can follow this [tutorial](https://github.com/pandora-auth-ros-pkg/pandora_docs/wiki/Remote-Machines-Running-ROS-nodes) if you want to use it in other project. 

## Setup the hardware and use of the haptic feedback simulator

This section show you how to connect your computer to the simulator and turn on the different devices. Next we will see how to launch a simulation. 

### Setup the devices

1. As explain at the **Setup the communication between the Raspberry and the computer** section. You must connect the Ethernet cable of the Raspberry to the port of the computer that you used for the configuration of the SSH communication. If you don't have ethernet port on your computer you can use and adaptater ethernet to USB. 
2. Turn on the Raspberry by plugging it. You will have to wait a few seconds before it is operational.
3. Check that the cable for the CAN communication (the purple one) is connected to PiCAN2 of the Raspberry. 
4. Turon on the screen and connect it to the computer.
5. Plug the cable for the ground.
6. Turn on the servo drive by plugging it. 
7. Plug the servo-motor power and than turn on the rotary switch. At this time the power stage (= the servo-motor) is not yet enable, it will be done by CAN communication during the running of the code. The message "rdy" (for "ready") is displayed on the servo-drive screen.

<img src="https://git.immc.ucl.ac.be/tuerlinckxt/ros_rasp/-/raw/CAN-develop/switch_on.jpg" width="200">

### Launch the simulation

First, open a terminal on the compture and enable the ROs environnement with : 

 `roscore`

Afte that, open a terminal on the computer and connect it to the Raspberry with :

 `ssh raspberry-ethernet`

Then run the node to enable the servo-drive with :

 `rosrun ros_rasp lexium32A_CANopen`
     
You will have to enter the password of the Raspberry, you can find it right next to him !

Finnaly open a new terminal again and launch the application with : 

 `roslaunch ros_rob haptic_simulator.launch`
        
1) So you have two main terminals : 
     * One for the ROS node on the Raspberry : it take care about the CAN communication with the servo-drive and display all the informations off the servo-drive setting. 
     * One for the two ROS nodes on the Laptop : one node runs the Robotran simulation and the second creates some interaction with the user for the simulation. 
     
2) On the laptop terminal, select the vehicle you want by typing the corresponding number in terminal and pressing "enter" : 
     
 <img src="https://git.immc.ucl.ac.be/tuerlinckxt/ros_rasp/-/raw/CAN-develop/terminal_1.png" width="400">
 
* **The car**

 <img src="https://git.immc.ucl.ac.be/tuerlinckxt/ros_rasp/-/raw/CAN-develop/car_presentation.png" width="300">
 
* **The kart **

 <img src="https://git.immc.ucl.ac.be/tuerlinckxt/ros_rasp/-/raw/CAN-develop/kart.png" width="300">
 
3) Next, select the simulation you want :
 
<img src="https://git.immc.ucl.ac.be/tuerlinckxt/ros_rasp/-/raw/CAN-develop/terminal_2.png" width="400">

* **Circuit 1**

 <img src="https://git.immc.ucl.ac.be/tuerlinckxt/ros_rasp/-/raw/CAN-develop/circuit_1.png" width="300">
 
* **Circuit 2**

 <img src="https://git.immc.ucl.ac.be/tuerlinckxt/ros_rasp/-/raw/CAN-develop/circuit_2.png" width="300">
 
* **Full speed turn** *not avaible with the kart* : drive at high speed then turn and feel the augmentation of the torque in the steeringwheel. Simulation done with random mechanic trail. 

* **Traffic circle** *not avaible with the kart* : take a traffic circle at constant speed. At a random time, the ground will be sliding during 2.5 seconds, try to stay on the traffic circle ! Simulation done with a random mechanic trail.

* **Obstacle avoid** *not avaible with the kart* : drive on a straigh road a constant speed. An obstacle is placed at a random distance, try to avoid it ! Simulation done with random mechanic trail. The "inside wiew" is advised.

4) Choose the view by typing the correct number *not avaible with the kart* : 

<img src="https://git.immc.ucl.ac.be/tuerlinckxt/ros_rasp/-/raw/CAN-develop/terminal_3.png" width="400">

* **The kart view**

 <img src="https://git.immc.ucl.ac.be/tuerlinckxt/ros_rasp/-/raw/CAN-develop/kart_vue.png" width="300">
 
* **The car outside vue**

 <img src="https://git.immc.ucl.ac.be/tuerlinckxt/ros_rasp/-/raw/CAN-develop/car_vue_1.png" width="300">
 
* **The car inside vue**

 <img src="https://git.immc.ucl.ac.be/tuerlinckxt/ros_rasp/-/raw/CAN-develop/car_vue_2.png" width="300">         
 
5) Then, select the initial speed of your vehicle in km/h and press "enter", the simulation of the vehicle will launch : ride it with the steerwheel ! 
    
 <img src="https://git.immc.ucl.ac.be/tuerlinckxt/ros_rasp/-/raw/CAN-develop/terminal_4.png" width="400">
        
4) At the end of the simulation, you can choose if you want to restart a new one. 
       
     * If yes you will return to the point 2.
     * If no the ROS node will shutdown. Then wait that the closing prodecure is done on the terminal of the Raspberry and that the message "Closing procedure successful !" is displayed. Turn off the switch of the motor. Turn off the Raspberry with `sudo shutdown` and then unplug everything.  
       
## Include your own Robotran project 
The examples here is made with a Robotran projects wich simulate the dynamics of a car and a kart. But you can also add your own Robotran project (other type of vehicles, other multibody system, ...). 

This tutorial assumes you followed the **Installation and configuration of ROS** and **Installation and configuration of Robotran** sections.

1) Download this package in your catkin_ws/src folder.

2) Copy all the files and folders of your Robotran project (userfctR, workR, ect) without *build* folder in *ros_rob/src/name_of_your_project*.

3) Then add the CMakeList of Robotran to the one of ROS by modifying this line in the ROS CMakeList 

      `add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/name_of_your_project/workR)`

4) Include the ROS and Pthread libraries in the Robotran CMakeList 

         include_directories(/opt/ros/melodic/include /opt/ros/melodic/lib)

         target_link_libraries(${Executable} -I/opt/ros/melodic/include -L/opt/ros/melodic/lib -lroscpp -lrostime -lrosconsole -lroscpp_serialization)
         
         target_link_libraries(${Executable} -lpthread)

5) Change the extension of the Robotran main.c by main.cpp. Change the content of the main with the template [main_robotran_template.cpp](https://git.immc.ucl.ac.be/tuerlinckxt/ros_rob/-/blob/CAN-develop_v2/.main_robotran_template_haptic_simulator.cpp) and follow the change instructions inside the template. 

6) As you can see at point 5) and 6) we need to use a user model structure from Robotran. Follow [the user model tutorial for structures](http://robotran-doc.git-page.immc.ucl.ac.be/usermodelstructurec/) in order to initialize it in *MBSysPad* and create an header file (also see [thread_struct.h](https://git.immc.ucl.ac.be/tuerlinckxt/ros_rob/-/blob/master/src/Car/userfctR/thread_struct.h) for an example of declaration of the structure). This structure contains pointeur to function like *give_torque_access* wich allows ROS publisher and suscriber to have access to *mbs_data* without segmentation fault cause of the multithreading. After that you can give the access to the ROS publisher and suscriber in all the user function with :
 
         (*mbs_data->user_model->thread.thread_struct->pointeur_<name_of_access_function>)();
         
 7) **Note** : the main of Robotran is now in C++ but the user functions that you way write are usually in C and call by this C++ function. You can have a look on [Name Mangling and extern “C” in C++](https://www.geeksforgeeks.org/extern-c-in-c/) to ensure that the C++ compiler behaves like a C compiler for this functions.  

 8) Go to the catkin_ws folder et compile/build it with :`~/catkin_ws$ catkin_make`   
 
### Deal with multiple Robotran projects 

If you need to deal with several Robotran projects, just follow the above instructions one more time with two exeptions :

  * At the point **3** you can have only one link with a Robotran CMakeLists (only one *add_sudirectory*). 
  * At the point **8**, before compile/build your workspace you need to delete the *ros_rob* folder in the *build* folder. 

This means that you can compile and build only one Robotran project at a time. But once the new ROS executbale is created (here the *exe_your_project_name*) you can run wich you want. 

### Use your own ROS message

The ROS messages here are used to vehicle data for the torque, position and velocity of a steerwheel wich are floating point values. It could be usefull to create your own ROS messages in order to deal with the values provided and needed by your Robotran project. You can consult this [tutorial](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv) and this [ROS wiki](http://wiki.ros.org/msg) for more informations. 


     
     