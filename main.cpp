#include "ctrlGripper.hpp"
#include <iostream>
#include <vector>
#include <windows.h>

// Linear interpolation function
double interpolate(double x, double x0, double x1, double y0, double y1) {
	return x0 == x1 ? y0 : y0 + (x - x0) * (y1 - y0) / (x1 - x0);
}

double convert2domain(double input){
	std::vector<double> A = {  0,  2,  9, 15, 22, 29, 35, 42, 48, 54, 60, 66, 71, 77, 83, 85 };
	std::vector<double> B = { 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85 };

	// Find the two closest values in A
	double x0 = 0, x1 = 0;
	double y0 = 0, y1 = 0;
	for (size_t i = 0; i < A.size(); ++i) {
		if (A[i] <= input) {
			x0 = A[i];
			y0 = B[i];
		}
		if (A[i] >= input) {
			x1 = A[i];
			y1 = B[i];
			break;
		}
	}

	// Perform linear interpolation
	double result = interpolate(input, x0, x1, y0, y1);

	std::cout << "Mapped value in the domain of B: " << result << std::endl;

	return result;
}

int main() {
	// Set gripper parameter
	// Range 0 - 85  mm
	double pos_request =  0.0;
	double pos_home    = 85.0;

	// Convert to gripper domain
	double pos_request_ = convert2domain(pos_request);
	std::cout << "Press any key to start" << std::endl;
	system("pause");

	Active_on();
	setPos(pos_request_);
	moveGripper();

	system("pause");

	setPos(pos_home);
	moveGripper();

	Active_off();

	// Range 19-100 mm/s
	//double speed = 75.0; 

	//// Range 30-100 N
	//double force = 100.0; 

	//// System status parameter
	//int gACT;
	//int gGTO;
	//int gIMC;  //讀取數值為0-3，請轉成二進位，參照手冊解讀指令
	//int gOBJ;  //讀取數值為0-3，請轉成二進位，參照手冊解讀指令
	//int FAULT; //讀取數值為0-16，請轉成十六進位，參照手冊解讀指令

	//double current;
	//double pos_now;

	//std::string com_port = "COM7";
	//std::cout << "Connect port is : " << com_port << std::endl;

	//ROBOTIQ_function gripper(com_port);

	//system("pause");

	//// Activate the gripper.
	//gripper.ROBOTIQ_active_on();
	//std::cout << "Gripper is activate !!" << std::endl;
	//system("pause");

	//while (true) {
	//	double pos_request;
	//	std::cout << "Enter a value to set the position (or any negative value to exit): ";
	//	std::cin >> pos_request;

	//	// Check if the input is negative to exit the loop
	//	if (pos_request < 0) {
	//		std::cout << "Exiting the program." << std::endl;
	//		break;
	//	}

	//	pos_request_ = convert2domain(pos_request);

	//	gripper.ROBOTIQ_set_pos(pos_request_);
	//	std::cout << "Target Position is : " << pos_request << " mm" << std::endl;
	//	system("pause");

	//	std::cout << "Going to the tatget position" << std::endl;
	//	gripper.ROBOTIQ_goto();
	//}

	// Set the gripper's target position, velocity, and force.
	// gripper.ROBOTIQ_set_all(pos_request, speed, force);

	/*gripper.ROBOTIQ_set_pos(pos_request_);
	std::cout << "Target Position is : " << pos_request << " mm" << std::endl;
	system("pause");*/

	// Move the gripper to the target position.
	/*std::cout << "Going to the tatget position" << std::endl;
	gripper.ROBOTIQ_goto();
	system("pause");*/

	// Stop the gripper motion (if needed).
	// gripper.ROBOTIQ_stop();

	/*gripper.ROBOTIQ_action_status_read(pos_request, pos_now, current);
	gripper.ROBOTIQ_fault_read(FAULT);

	gripper.ROBOTIQ_gripper_status_read(gACT, gGTO, gIMC, gOBJ);*/

	//std::cout << "Going to the home position" << std::endl;
	//gripper.ROBOTIQ_set_pos(pos_home);
	//gripper.ROBOTIQ_goto();
	//system("pause");

	//// Deactivate the gripper when done.
	//gripper.ROBOTIQ_active_off();

	//system("pause");

	return 0;
}

