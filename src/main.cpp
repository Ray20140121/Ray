#include "main.h"
#include <vector>
#include <string>

Motor leftFront(-11);
Motor leftBottom(-13);
Motor leftTop(14);
Motor rightFront(17);
Motor rightBottom(18);
Motor rightTop(-20);
Motor intake(5);
Motor score(-10);
adi::Pneumatics lift('a',false);
adi::Pneumatics loader('b',false);
Imu imu(9);
void powerDriveMotor(int powerForward,int powerTurn);
void driveForward(int distance, int timeout = 1500,int speed = 80);
void turn(int turn);
int speedLimit(int speed,int limit);

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		lcd::set_text(2, "I was pressed!");
	}
	else {
		lcd::clear_line(2);
	}
}
int current_auton = 0;

void initialize() {
	lcd::initialize();
	lcd::set_text(1, "Hello PROS User!");
	lcd::register_btn1_cb(on_center_button);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	intake.move (127);
	delay(200);
	driveForward(650,2800,40);
	//first three balls
	delay(1000);
	turn(123);
	delay(500);
	loader.extend();
	delay(200);
	leftBottom.tare_position(); // resetting the encoder that is being used 
	driveForward(400,1500,60);
	turn(45);
	delay(200);
	driveForward(500);
	//matchload
	delay(1000);
	leftBottom.tare_position(); // resetting the encoder that is being used 
	driveForward(-700,-80);
	loader.retract();
	delay(200);
	lift.extend();
	delay(200);
	driveForward(-1300, -80);
	//score
	delay(200);
	score.move_absolute(290,127);
	delay(500);
	score.move_absolute(-290,127);
	delay(500);
	score.move_absolute(350,127);
}
void opcontrol() {
	Controller master(pros::E_CONTROLLER_MASTER);
	while (true) {
		int dir = master.get_analog(ANALOG_LEFT_Y);
		int turn = master.get_analog(ANALOG_RIGHT_X);
		powerDriveMotor(dir, turn);

		if(master.get_digital(DIGITAL_L1)){
			intake.move (127);
		}
		else if(master.get_digital(DIGITAL_L2)){
			intake.move (-127);
		}
		else{
			intake.move (0);
		}
		if(master.get_digital(DIGITAL_R1)){
			score.move(-127);
		}
		else if(master.get_digital(DIGITAL_R2)){
			score.move(127);
		}
		else{
			score.move (0);
		}
		if(master.get_digital_new_press(DIGITAL_B)){
			lift.toggle();
		}
		if(master.get_digital_new_press(DIGITAL_A)){
			loader.toggle();
		}
		delay(20);
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

void driveForward(int distance, int timeout,int speed){
	leftTop.tare_position();
	int multiplierConstant = 1; 
	int power = 1;
	int drivingDistanceRemaining = distance - leftTop.get_position(); 
	int startTime = millis();
	while(std::abs(drivingDistanceRemaining) >= 3 && millis() - startTime < timeout){ //distance left 
		drivingDistanceRemaining = distance - leftTop.get_position();
		power = drivingDistanceRemaining*multiplierConstant; 
		powerDriveMotor(speedLimit(power,speed),0); 
		delay(20);
	}
	powerDriveMotor(0,0);
}
void turn(int turn){
	imu.tare_rotation();
	int multiplierConstant = 1; 
	int power = 1; 
	int timeOut = millis();
	int turningDistanceRemaining = turn - imu.get_rotation(); 
	while(std::abs(turningDistanceRemaining) >= 2 && millis() - timeOut < 1500){ //distance left 
		turningDistanceRemaining = turn - imu.get_rotation();
		power = turningDistanceRemaining*multiplierConstant; 
		powerDriveMotor(0,power);
		delay(20);
	}
	powerDriveMotor(0,0);
}
pros::Controller master(pros::E_CONTROLLER_MASTER);

int speedLimit(int speed, int limit) {
	if (speed > limit){
		return limit;
	}
	else if(speed < limit){
		return limit;
	} 
	else {
		return speed;
	}
}