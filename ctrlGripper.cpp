#include "ctrlGripper.hpp"

ROBOTIQ_function gripper("COM7");

void iniGripper(int com_port) {
	char devname[64];
	sprintf_s(devname, "COM%d", com_port);
	std::cout << "Connect port is : " << std::string(devname) << std::endl;
	ROBOTIQ_function obj(devname);
	gripper = obj;
}

void Active_on() {
	gripper.ROBOTIQ_active_on();
}

void  Active_off() {
	gripper.ROBOTIQ_active_off();
}

void setPos(double pos) {
	std::cout << "Target position is " << pos << std::endl;
	pos = convert2domain(pos);
	gripper.ROBOTIQ_set_pos(pos);
}

void setPosVelForce(double pos, double vel, double force) {
	pos = convert2domain(pos);
	gripper.ROBOTIQ_set_all(pos, vel, force);
}

void moveGripper() {
	gripper.ROBOTIQ_goto();
}

void stopGripper() {
	gripper.ROBOTIQ_stop();
}

double interpolate(double x, double x0, double x1, double y0, double y1) {
	return x0 == x1 ? y0 : y0 + (x - x0) * (y1 - y0) / (x1 - x0);
}

double convert2domain(double input) {
	std::vector<double> A = { 0,  2,  9, 15, 22, 29, 35, 42, 48, 54, 60, 66, 71, 77, 83, 85 };
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