#ifndef WHEELS_H
#define WHEELS_H

#include "main.h"

#define TOP_LEFT_WHEEL 6
#define TOP_RIGHT_WHEEL 2
#define BOTTOM_LEFT_WHEEL 3
#define BOTTOM_RIGHT_WHEEL 5

class Wheels
{
public:
	Wheels();	// default constructor

	// set the wheel motors' voltage so that the hex bot travels in the corresponding direction at a proportional speed
	// given the vector of motion for the robot
	void drive(double, double);

	// if drive is called with a controller argument, get vector of motion from controller instead
	void drive(pros::Controller);

	// set the wheel motors' voltage so that the hex bot rotates in place in the corresponding direction at a proportional speed
	// given the direction and speed of rotation
	void rotate(double);

	// if rotate is called with a controller argument, get rotation speed and direction from controller instead
	void rotate(pros::Controller);
private:
	void set_voltage(pros::Motor*, double&, double);	// set the voltage if it would result in a change to prevent wheel jittering

	// pointers to wheel motors
	pros::Motor* top_left;			// points to top_left wheel motor
	pros::Motor* top_right;			// points to top_right wheel motor
	pros::Motor* bottom_left;		// points to bottom_left wheel motor
	pros::Motor* bottom_right;	// points to bottom_right wheel motor

	// contains voltages to be sent to motors
	double top_left_volt;
	double top_right_volt;
	double bottom_left_volt;
	double bottom_right_volt;
};

#endif
