#define 	DEBUGSS	0

#include "ROBOTIQ_function.h"

#ifdef __cplusplus
extern "C" {
#endif
	void iniGripper(int);
	void Active_on();
	void Active_off();
	void setPos(double);
	void setPosVelForce(double, double, double);
	void moveGripper();
	void stopGripper(); 
#ifdef __cplusplus
}
#endif

double interpolate(double, double, double, double, double);
double convert2domain(double);