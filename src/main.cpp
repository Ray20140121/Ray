#include "main.h"
#include <vector>
#include <string>

Motor leftFront(-11);
Motor leftBottom(-12);
Motor leftTop(13);
Motor rightFront(16);
Motor rightBottom(17);
Motor rightTop(-18);
Motor intake(-2);
Motor score(4);
Motor middle(-3);
Motor basket(9);
adi::Pneumatics roof('a',false);
adi::Pneumatics loader('b',false);
Imu imu(8);
void powerDriveMotor(int powerForward,int powerTurn);
void driveForward(int distance);
void turn(int turn);
int speedLimit(int speed);
// void update_lcd();
// void auton_selector_task();
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
int current_auton = 0;
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	// update_lcd();
	// pros::Task selector(auton_selector_task);
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
 * If the robot is disabled or communications is lost, the autonomous task`
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	//pros::lcd::set_text(3, "Running: " + auton_modes[current_auton]);
  	// Add your autonomous logic here based on current_auton
	intake.move (127);
	middle.move (127);
	driveForward(850);
	delay(100);
	turn(90);
	delay(1000);
	driveForward(900);
	delay(500);
	turn(70);
	delay(200);
	driveForward(1700);
	delay(200);
	turn(-50);
	delay(200);
	driveForward(1500);
	basket.move (127);
	score.move (127);
}

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
	while (true) {
		int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		powerDriveMotor(dir, turn);
		pros::delay(20);                               // Run for 20 ms then update

		if(master.get_digital(DIGITAL_L1)){
			intake.move (127);
			middle.move (127);
		}
		else if(master.get_digital(DIGITAL_L2)){
			intake.move (127);
			middle.move (127);
			basket.move (127);
			score.move (127);
		}
		else if(master.get_digital(DIGITAL_R1)){
			intake.move (127);
			middle.move (-127);
			basket.move (127);
		}
		else if(master.get_digital(DIGITAL_R2)){
			intake.move (-127);
			middle.move (-127);
			basket.move (-127);
			score.move (-127);
		}
		else{
			intake.move (0);
			middle.move (0);
			basket.move (0);
			score.move (0);
		}
		if(master.get_digital_new_press(DIGITAL_B)){
			roof.toggle();
		}
		if(master.get_digital_new_press(DIGITAL_DOWN)){
			loader.toggle();
		}
	}
}


void powerDriveMotor(int powerForward,int powerTurn){
	leftFront.move(powerForward + powerTurn);
	leftBottom.move(powerForward + powerTurn);
	leftTop.move(powerForward + powerTurn);
	rightFront.move(powerForward - powerTurn);
	rightBottom.move(powerForward - powerTurn);
	rightTop.move(powerForward - powerTurn);
}


void driveForward(int distance){
	leftFront.tare_position();
	int multiplierConstant = 1; 
	int power = 1;
	int drivingDistanceRemaining = distance - leftFront.get_position(); 
	while(drivingDistanceRemaining >= 3){ //distance left 
		drivingDistanceRemaining = distance - leftFront.get_position();
		power = drivingDistanceRemaining*multiplierConstant; 
		powerDriveMotor(speedLimit(power),0); 
	}
	powerDriveMotor(0,0);
}

void turn(int turn){
	imu.tare_rotation();
	int multiplierConstant = 1.5; 
	int power = 1; 
	int turningDistanceRemaining = turn - imu.get_rotation(); 
	while(turningDistanceRemaining >= 1){ //distance left 
		turningDistanceRemaining = turn - imu.get_rotation();
		power = turningDistanceRemaining*multiplierConstant; 
		powerDriveMotor(0,power);
	}
	powerDriveMotor(0,0);
}
pros::Controller master(pros::E_CONTROLLER_MASTER);

int speedLimit(int speed) {
	if (speed > 60){
		return 60;
	}
	else if(speed < -60){
		return -60;
	} 
	else {
		return speed;
	}
}


// std::vector<std::string> auton_modes = {
// 	"None",
//   	"Red - Front",
//   	"Red - Back",
//   	"Blue - Front",
//   	"Blue - Back",
//   	"Skills",
// };

// void update_lcd(){
//   	pros::lcd::clear();
// 	pros::lcd::set_text(1, "Selected Auton:");
// 	pros::lcd::set_text(2, auton_modes[current_auton]);
// }

// void auton_selector_task() {
//   	while (true) {
//     // Advance auton if R1 is held and A is newly pressed
//     	if (master.is_pressed(pros::E_CONTROLLER_DIGITAL_DOWN) &&
//         master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
//     		current_auton = (current_auton + 1) % auton_modes.size();
//     		update_lcd();
//     	}

//     	delay(100);
//   	}
// }
