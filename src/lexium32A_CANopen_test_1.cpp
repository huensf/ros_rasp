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
//#include "ros_rasp/Torque_msg.h"
//#include "ros_rasp/Pos_vit_msg.h"

#include <math.h> 
#include <algorithm> 

class Listener_torque
{
  public :

  double torque_rob;

  void chatterCallback(const ros_rasp::Torque_msg& msg)
   {
      //ROS_INFO("torque : [%f]", msg.value);
      torque_rob = msg.value;      
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
	std::string eds_path = "/home/tuerlinckxt/catkin_ws/src/ros_rasp/Lexium32A/SELXM32A_0126E.EDS";
	 device.load_dictionary_from_eds(eds_path);

	std::cout << "CiA-profile = " << device.get_device_profile_number() << std::endl;

	std::cout << "Vendor-ID = " << device.get_entry("Identity object/Vendor-ID") << std::endl;

	try {

		std::cout << "Manufacturer device name = " << device.get_entry("manufacturer-device-name") << std::endl;

		std::cout << "Manufacturer hardware version = " << device.get_entry("manufact.-hardware-version") << std::endl;

		std::cout << "Manufacturer software version = " << device.get_entry("manufact.-software-version") << std::endl;

	} catch (const kaco::canopen_error& error) {
		std::cout << "Getting manufacturer information failed: " << error.what() << std::endl;
	}

   master.stop();
}
