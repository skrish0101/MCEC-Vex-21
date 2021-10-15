#include "main.h"
#include "wheels.h"

Wheels::Wheels()
{
  top_left = new pros::Motor(TOP_LEFT_WHEEL);
  top_right = new pros::Motor(TOP_RIGHT_WHEEL);
  bottom_left = new pros::Motor(BOTTOM_LEFT_WHEEL);
  bottom_right = new pros::Motor(BOTTOM_RIGHT_WHEEL);
}


// set the wheel motors' voltage so that the hex bot travels in the corresponding direction at a proportional speed
// x and y range from -127 to +127
void Wheels::drive(double move_x, double move_y)
{

  // calculate the angle and the magnitude of the controller joystick
  double move_angle = std::atan2(move_y, move_x);											// theta = atan(y/x)
  double move_magnitude = std::sqrt(pow(move_x, 2) + pow(move_y, 2));	// r = sqrt(x^2 + y^2)

  // set voltage for wheels
  set_voltage(top_left, top_left_volt, move_magnitude * std::sin(move_angle + M_PI/4));           // r*sin(theta+pi/4)
  set_voltage(top_right, top_right_volt, move_magnitude * std::sin(move_angle + 3*M_PI/4));       // r*sin(theta+3pi/4)
  set_voltage(bottom_left, bottom_left_volt, move_magnitude * std::sin(move_angle - M_PI/4));     // r*sin(theta-pi/4)
  set_voltage(bottom_right, bottom_right_volt, move_magnitude * std::sin(move_angle - 3*M_PI/4)); // r*sin(theta-3pi/4)
}

// if drive is called with a controller argument, get vector of motion from controller instead
void Wheels::drive(pros::Controller master)
{
  // pass in the x and y components of the left joystick as move_x and move_y in drive()
  drive(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X), master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
}

// set the wheel motors' voltage so that the hex bot rotates in place in the corresponding direction at a proportional speed
// rotation_factor ranges from -127 to +127 where -127 is max speed CCW and +127 is max speed CW
void Wheels::rotate(double rotation_factor)
{
  // ensure the voltage of each wheel is the rotation factor
  set_voltage(top_left, top_left_volt, rotation_factor);
  set_voltage(top_right, top_right_volt, rotation_factor);
  set_voltage(bottom_left, bottom_left_volt, rotation_factor);
  set_voltage(bottom_right, bottom_right_volt, rotation_factor);
}

// if rotate is called with a controller argument, get rotation speed and direction from controller instead
void Wheels::rotate(pros::Controller master)
{
  rotate(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));  // get the x of the left joystick and pass that as the rotation_factor
}

// set the voltage if it would result in a change to prevent wheel jittering
void Wheels::set_voltage(pros::Motor* wheel, double& old_voltage, double new_voltage)
{
  // if new voltage and old voltage are significantly different
  if (std::abs(new_voltage - old_voltage) > 1)
  {
    wheel->move(new_voltage);   // set voltage of wheel
    old_voltage = new_voltage;  // store new voltage for future comparisons
  }
}
