#include "main.h"
#include <cmath>

#define TOP_LEFT_WHEEL 6
#define TOP_RIGHT_WHEEL 2
#define BOTTOM_LEFT_WHEEL 3
#define BOTTOM_RIGHT_WHEEL 5

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void opcontrol() {
  pros::Controller master(pros::E_CONTROLLER_MASTER);

	// the motors for the four wheels
	pros::Motor top_left_wheel(TOP_LEFT_WHEEL);
	pros::Motor top_right_wheel(TOP_RIGHT_WHEEL);
	pros::Motor bottom_left_wheel(BOTTOM_LEFT_WHEEL);
	pros::Motor bottom_right_wheel(BOTTOM_RIGHT_WHEEL);

	bool rotating = false;
	// variables about moving the robot
	int move_x = 0;	// x component of robot movement / controller joystick
	int move_y = 0;	// y component of robot movement / controller joystick
	double move_angle = 0;	// angle of robot movement / controller joystick with respect to positive x
	int move_magnitude = 0;	// magnitude of robot movement / controller joystick
	double top_left_volt = 0;
	double top_right_volt = 0;
	double bottom_left_volt = 0;
	double bottom_right_volt = 0;

	double rotate_power = 0;

	while (true) {

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
		{
			if (rotating)
			{
				rotating = false;
			} else {
				rotating = true;
			}
		}

		if (rotating)
		{
			rotate_power = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);


			if (std::abs(rotate_power - top_left_volt) > 1)
			{
				top_left_wheel.move(rotate_power);
				top_right_wheel.move(rotate_power);
				bottom_left_wheel.move(rotate_power);
				bottom_right_wheel.move(rotate_power);
			}

		} else {
			//
			// set the powers to the wheel motors
			//

			// get the x and y components from the controller joystick
			move_x = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
			move_y = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);

			// calculate the angle and the magnitude of the controller joystick
			move_angle = std::atan2(move_y, move_x);											// theta = atan(y/x)
			move_magnitude = std::sqrt(pow(move_x, 2) + pow(move_y, 2));	// r = sqrt(x^2 + y^2)

			// set voltage for wheels

			// set voltage for top left wheel
			if (std::abs(move_magnitude * std::sin(move_angle + M_PI/4) - top_left_volt) > 10)	// only if significantly different voltage
			{
				top_left_volt = move_magnitude * std::sin(move_angle + M_PI/4);	// r*sin(theta + pi/4)
				top_left_wheel.move(top_left_volt); 														// set voltage
			}

			// set voltage for top right wheel
			if (std::abs(move_magnitude * std::sin(move_angle + 3*M_PI/4) - top_right_volt) > 10)	// only if significantly different voltage
			{
				top_right_volt = move_magnitude * std::sin(move_angle + 3*M_PI/4);	// r*sin(theta + pi/4)
				top_right_wheel.move(top_right_volt); 															// set voltage
			}

			// set voltage for bottom left wheel
			if (std::abs(move_magnitude * std::sin(move_angle - M_PI/4) - bottom_left_volt) > 10)	// only if significantly different voltage
			{
				bottom_left_volt = move_magnitude * std::sin(move_angle - M_PI/4);	// r*sin(theta + pi/4)
				bottom_left_wheel.move(bottom_left_volt); 													// set voltage
			}

			// set voltage for bottom right wheel
			if (std::abs(move_magnitude * std::sin(move_angle - 3*M_PI/4) - bottom_right_volt) > 10)	// only if significantly different voltage
			{
				bottom_right_volt = move_magnitude * std::sin(move_angle - 3*M_PI/4);	// r*sin(theta + pi/4)
				bottom_right_wheel.move(bottom_right_volt); 													// set voltage
			}
		}
	}
}
