/*
 * Copyright (c) 2016, Thomas Keh
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <thread>
#include <chrono>
#include <cstdint>

#include "master.h"
#include "canopen_error.h"
#include "logger.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros_rasp/Torque_msg.h"
#include "ros_rasp/Pos_vit_msg.h"

#include <math.h>
#include <algorithm>

class Listener_torque
{
  public :

  double torque_rob;

  ros_rasp::Pos_vit_msg msg_pos_vit;

  ros::Publisher chatter_pub;

  void chatterCallback(const ros_rasp::Torque_msg& msg)
   {
   //   ROS_INFO("torque : [%f]", msg.value);
      torque_rob = -msg.value;
       chatter_pub.publish(msg_pos_vit);
   }
};



int main(int argc, char **argv) {

	// ----------- //
	// Preferences //
	// ----------- //

	// The node ID of the slave we want to communicate with.
	const uint8_t node_id = 32; //TO SET

	// Set the name of your CAN bus. "slcan0" is a common bus name
	// for the first SocketCAN device on a Linux system.
	const std::string busname = "can0"; 

	// Set the baudrate of your CAN bus. Most drivers support the values
	// "1M", "500K", "125K", "100K", "50K", "20K", "10K" and "5K".
	 const std::string baudrate_pican2 = "1000000"; //TO SET, replace K by 000
        const std::string baudrate = "1M"; //TO SET, same value as baudrate_pican2 


	// -------------- //
	// Initialization //
	// -------------- //

	std::cout << "CAN communication for the Lexium32A." << std::endl;

    std::cout << "Need the password to bring the CAN interface up on the Raspberry : " << std::endl;

	// Create Master (includes Core - accessible via master.core).
	kaco::Master master;

    std::string caninterface_up = "sudo /sbin/ip link set "+busname+" up type can bitrate "+baudrate_pican2;

    system(caninterface_up.c_str()); //bring the CAN interface up

	std::cout << "Starting Master (involves starting Core)..." << std::endl;
	if (!master.start(busname, baudrate)) {
		std::cout << "Starting Master failed." << std::endl;
		return EXIT_FAILURE;
	}

	bool found_device = false;
	size_t device_index;

	while (!found_device) {

		for (size_t i=0; i<master.num_devices(); ++i) {
			kaco::Device& device = master.get_device(i);
			if (device.get_node_id()==node_id) {
				found_device = true;
				device_index = i;
				break;
			}
		}

		std::cout << "Device with ID " << (unsigned) node_id
			<< " has not been found yet. Waiting one more second. Press Ctrl+C abort." << std::endl;
		std::this_thread::sleep_for(std::chrono::seconds(1));

	}

	// ------------------ //
	// Device Information //
	// ------------------ //
	
	kaco::Device& device = master.get_device(device_index);

	std::cout << "Starting device with ID " << (unsigned) node_id << "..." << std::endl;
	device.start();

	std::cout << "Loading object dictionary from the library." 
		<< " EDS file of the Lexium32A" << std::endl;
	std::string eds_path = "/home/steerhaptic/catkin_ws/src/ros_rasp/Lexium32A/SELXM32A_0126E.EDS";
	 device.load_dictionary_from_eds(eds_path);

	std::cout << "CiA-profile = " << device.get_device_profile_number() << std::endl;

	std::cout << "Vendor-ID = " << device.get_entry("Identity object/Vendor-ID") << std::endl;

	try {

		std::cout << "Manufacturer device name = " << device.get_entry("Manufacturer device name") << std::endl;

		std::cout << "Manufacturer hardware version = " << device.get_entry("manufact. hardware version") << std::endl;

		std::cout << "Manufacturer software version = " << device.get_entry("manufact. software version") << std::endl;

	} catch (const kaco::canopen_error& error) {
		std::cout << "Getting manufacturer information failed: " << error.what() << std::endl;
	}

 // --------------------- //
 // Device Initialisation //
 // --------------------- //

  //-----HOMING-----//

  try {

    std::cout << "Start Homing..." << std::endl;

    std::cout << "Velocity for research of limit switch set to 100" << std::endl;
    device.set_entry("Homing speeds/Homing speed during search for switch", (uint32_t) 100);

    std::cout << "Velocity for restart set to 10" << std::endl;
    device.set_entry("Homing speeds/speed during search for zero", (uint32_t) 10);

    std::cout << "NMT Start Remote Node" << std::endl;
    master.core.nmt.send_nmt_message(node_id,kaco::NMT::Command::start_node);

    std::cout << "Giving the device one second time to boot up ..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));

    //T_PDO1 avec mot d'état

    std::cout << "Enable power stage with R_PDO1" << std::endl;
    std::vector<uint8_t> enable_power_init = {0x00,0x00};
    master.core.pdo.send(0x200+node_id,enable_power_init);
    std::vector<uint8_t> enable_power_rdy = {0x06,0x00};
    master.core.pdo.send(0x200+node_id,enable_power_rdy);
    std::vector<uint8_t> enable_power_run = {0x0F,0x00};
    master.core.pdo.send(0x200+node_id,enable_power_run);
    // ou avec controlword ? DCOMcontrol index 6040 et voir profiles.cpp

    std::cout << "Giving the device one second time to enable the power stage ..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));

    //Operating state : 6->Operation Enabled

    std::cout << "Starting the operating mode : Homing" << std::endl;
    device.set_entry("Modes of operation", (int8_t) 6); // 6 : Homing

    std::cout << "Check operating mode : Homing ..." << std::endl;
   // kaco::Value homing_check = (int8_t) 6;
    kaco::Value homing_check((int8_t) 6);
    while(device.get_entry("Modes of operation display")!= homing_check)
    {
          std::cout << "Homing not yet enabled... Waiting one more second. Press Ctrl+C abort." << std::endl;
          std::this_thread::sleep_for(std::chrono::seconds(1)); 
	}
    std::cout << "Homing enabled !" << std::endl;

    std::cout << "Set Homing mode to : Immediate origin" << std::endl;
    device.set_entry("Homing method", (int8_t) 35); // 35 : Immediate origin
   
    std::cout << "Homing operation start" << std::endl;
    std::vector<uint8_t> homing_start = {0x1F,0x00};
    master.core.pdo.send(0x200+node_id,homing_start);
    std::this_thread::sleep_for(std::chrono::seconds(2)); 

    //Check if homing finished successfully

    std::cout << "Homing finish !" << std::endl;

    } catch (const kaco::canopen_error& error) {
		std::cout << "Lexium Homing failed: " << error.what() << std::endl;
	}
     
    //-----Torque Profile-----//

    try {

    std::cout << "Start setting profile torque..." << std::endl;

    std::cout << "Starting the operating mode : Profile Torque" << std::endl;
    device.set_entry("Modes of operation", (int8_t) 4); // 4 : Profile Torque

    std::cout << "Check operating mode : Profile Torque ..." << std::endl;
    //kaco::Value profile_torque_check = (int8_t) 4;
     kaco::Value profile_torque_check((int8_t) 4);
    while(device.get_entry("Modes of operation display")!= profile_torque_check)
  //while(device.get_entry("_DCOMopmd_act").operator!=(4)) //a tester
    {
          std::cout << "Profile Torque not yet enabled... Waiting one more second. Press Ctrl+C abort." << std::endl;
          std::this_thread::sleep_for(std::chrono::seconds(1)); 
	}
    std::cout << "Profile Torque enabled !" << std::endl;

    } catch (const kaco::canopen_error& error) {
		std::cout << "Set torque profil failed: " << error.what() << std::endl;
	}

    std::cout << "Set Torque to 0 for security" << std::endl;
    device.set_entry("Target torque",(int16_t) 0);

   // ----------------------- //
   // While Loop for Robotran //
   // ----------------------- //

   ros::init(argc,argv,"roscan");

   ros::NodeHandle n;

   uint16_t torque_init = device.get_entry("servo motor/_M_M_0").operator uint16_t();

   int16_t torque_can = 0;

   Listener_torque lis_tor;

   lis_tor.chatter_pub = n.advertise<ros_rasp::Pos_vit_msg>("chatter_pos_vit",1);

   lis_tor.torque_rob = 0.0;

   lis_tor.msg_pos_vit;

   ros::Subscriber chatter_sub = n.subscribe("chatter_torque", 1, &Listener_torque::chatterCallback, &lis_tor);  

  ros_rasp::Pos_vit_msg msg;

  ros::Rate loop_rate(2000);

    std::cout << "ROS publisher and suscriber set !" << std::endl;
     std::cout << "Running ..." << std::endl;

    int robotran_finish;
    int reset_flag = 1;

   while(ros::ok()) //voir quand la boucle s'arrête
    {
	ros::param::get("robotran_finish", robotran_finish);
	if(!robotran_finish)
	{
	    reset_flag = 1;
            //get position and velocity with CAN
            lis_tor.msg_pos_vit.pos_value = (double) (device.get_entry("Position actual value").operator int32_t())*2.0*M_PI/16384.0; 
            lis_tor.msg_pos_vit.vit_value = (double) (device.get_entry("Velocity actual value").operator int32_t())*2.0*M_PI/60.0;
	}
	else if(robotran_finish)
	{
	    //reset

	    if(reset_flag) {
    	        try
	        {
		    //Homing for re-initialize the steerwheel position
		    device.set_entry("Homing speeds/Homing speed during search for switch", (uint32_t) 100);
		    device.set_entry("Homing speeds/speed during search for zero", (uint32_t) 10);
		    master.core.nmt.send_nmt_message(node_id, kaco::NMT::Command::start_node);
		    std::this_thread::sleep_for(std::chrono::seconds(1));
		    std::vector<uint8_t> enable_power_init = {0x00,0x00};
		    master.core.pdo.send(0x200+node_id,enable_power_init);
		    std::vector<uint8_t> enable_power_rdy = {0x06,0x00};
		    master.core.pdo.send(0x200+node_id,enable_power_rdy);
		    std::vector<uint8_t> enable_power_run = {0x0F,0x00};
		    master.core.pdo.send(0x200+node_id,enable_power_run);
		    std::this_thread::sleep_for(std::chrono::seconds(1));

		    std::cout << "Starting the operating mode : Homing" << std::endl;
		    device.set_entry("Modes of operation", (int8_t) 6);

		    std::cout << "Check operating mode : Homing ..." << std::endl;
		    kaco::Value homing_check((int8_t) 6);
		    while(device.get_entry("Modes of operation display")!=homing_check)
		    {
			std::cout << "Homing not yet enabled... Wainting one more second. Press Ctrl+C abort." << std::endl;
			std::this_thread::sleep_for(std::chrono::seconds(1));
		    }
		    std::cout << "Homing enabled !" << std::endl;

	            std::cout << "Set Homing mode to : Immediate origin" << std::endl;
	            device.set_entry("Homing method", (int8_t) 35);

	            std::cout << "Homing operation start" << std::endl;
	            std::vector<uint8_t> homing_start = {0x1F,0x00};
	            master.core.pdo.send(0x200+node_id,homing_start);
	            std::this_thread::sleep_for(std::chrono::seconds(2));

		    std::cout << "Homing finish !" << std::endl;
	        }
    	        catch (const kaco::canopen_error& error) {
		    std::cout << "Lexium Homing failed: " << error.what() << std::endl;
                }

		try
		{
		    std::cout << "Starting the operating mode : Profile Torque" << std::endl;
		    device.set_entry("Modes of operation", (int8_t) 4);

		    std::cout << "Check operating mode : Profile Torque ..." << std::endl;
		    kaco::Value profile_torque_check((int8_t) 4);
		    while(device.get_entry("Modes of operation display")!=profile_torque_check)
		    {
			std::cout << "Profile Torque not yet enabled... Waiting one more second. Press Ctrl+C abort." << std::endl;
			std::this_thread::sleep_for(std::chrono::seconds(1));
		    }
		    std::cout << "Profile Torque enabled !" << std::endl;
		}
		catch (const kaco::canopen_error& error) {
		    std::cout << "Profile Torque failed: " << error.what() << std::endl;
		}

		std::cout << "Running ..." << std::endl;
		reset_flag = 0;
	    }

	}


        //receive torque and publish position and velocity
        ros::spinOnce();

        //set torque with CAN
        torque_can = (int16_t) std::min(300.0,(std::max(-300.0,(lis_tor.torque_rob*10000.0)/torque_init)));
        device.set_entry("Target torque",torque_can);

    }


   // ------------ //
   // Device Closing //
   // ------------ //

    std::cout << "Set Torque to 0 for security" << std::endl;
    device.set_entry("Target torque",(int16_t) 0);    

   try{

   std::cout << "Terminate operating mode \"Profile Torque\" with \"Quick Stop \" and R_PDO1" << std::endl;
   std::vector<uint8_t> profile_torque_end = {0x0B,0x00};
   master.core.pdo.send(0x200+node_id,profile_torque_end);

   std::cout << "Clear \"Quick Stop \" with R_PDO1" << std::endl;
   std::vector<uint8_t> clear_quick_stop = {0x0F,0x00};
   master.core.pdo.send(0x200+node_id,clear_quick_stop);

   } catch (const kaco::canopen_error& error) {
		std::cout << "Closing procedure failed: " << error.what() << std::endl;
	}

    std::cout << "Closing procedure successful !" << std::endl;

   master.stop();
}
