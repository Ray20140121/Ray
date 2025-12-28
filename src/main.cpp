#include "main.h"
#include <vector>
#include <string>

pros::Controller master(pros::E_CONTROLLER_MASTER);
Motor leftFront(-11);
Motor leftBottom(-12);
Motor leftTop(13);
Motor rightFront(20);
Motor rightBottom(19);
Motor rightTop(-18);

MotorGroup leftMg({-11,-12,13}); 
MotorGroup rightMg({20,19,-18}); 
Motor intake(-1);
Motor score(9);
adi::Pneumatics lift('a',false);
adi::Pneumatics loader('b',false);
Imu imu(21);
void powerDriveMotor(int powerForward,int powerTurn);
void driveForward(double distance, int timeout,int speed, double Kp, double Kd, double TurningKp);
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
	imu.reset();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	leftMg.set_brake_mode(MOTOR_BRAKE_COAST);
	rightMg.set_brake_mode(MOTOR_BRAKE_COAST);

	driveForward(1165,2000,65,5,0.8,5); //this is for driving forward to balls
	intake.move (127);
	turn(90);
	driveForward(700,1500,20,1.4,3,5);//intake 3 balls
	delay(800);
	turn(90);
	driveForward(500,1200,60,3,0.5,5);//getting to the goal
	turn(-88);
	driveForward(1350,2000,60,3,2.2,5);//getting to the goal
	turn(91);
	lift.extend();
	delay(200);
	driveForward(-500,1000,40,1,2,5);//line up with goal
	delay(300);
	score.move(127);//score!!!!!
	delay(800);
	score.move(-127);
	delay(300);
	score.move(127);//score!!!!!
	delay(800);
	score.move(-127);
	delay(300);
	score.move(0);
	loader.extend();
	driveForward(1600,1500,70,3,5,5);//Match load
	delay(800);
	//driveForward(-1400,1000,70,3,5,5);//score!!!
	//loader.retract();
	//score.move(-127);
	//delay(500);
	//score.move(127);
	//delay(100);
	//score.move(-127);
	//delay(500);
	//score.move(127);

	/*Right
	leftMg.set_brake_mode(MOTOR_BRAKE_COAST);
	rightMg.set_brake_mode(MOTOR_BRAKE_COAST);

	driveForward(1165,2000,65,5,0.8,5); //this is for driving forward to balls
	intake.move (127);
	turn(90);
	driveForward(700,1500,20,1.4,3,5);//intake 3 balls
	delay(800);
	turn(90);
	driveForward(500,1200,60,3,0.5,5);//getting to the goal
	turn(-88);
	driveForward(1350,2000,60,3,3,5);//getting to the goal
	turn(91);
	lift.extend();
	delay(200);
	driveForward(-500,1000,40,1,2,5);//line up with goal
	delay(300);
	score.move(127);//score!!!!!
	delay(800);
	score.move(-127);
	delay(300);
	score.move(127);//score!!!!!
	delay(800);
	score.move(-127);
	delay(300);
	score.move(0);*/
}
void opcontrol() {
	leftMg.tare_position();
	rightMg.tare_position();
	score.tare_position();

	while (true) {
		lcd::print(1, "left: %f", leftFront.get_position()); // diasplay motor encoder
		//lcd::print(2, "leftTop: %f", leftTop.get_position());
		lcd::print(2, "right: %f", imu.get_rotation());
		//lcd::print(4, "rightFront: %f", rightFront.get_position());
		//lcd::print(5, "rightTop: %f", rightTop.get_position());
		//lcd::print(6, "rightBottom: %f", rightBottom.get_position());
		int dir = master.get_analog(ANALOG_LEFT_Y);
		int turn = master.get_analog(ANALOG_RIGHT_X);
		powerDriveMotor(dir, turn);

		if(master.get_digital(DIGITAL_L1)){
			intake.move (127);
		}
		else if(master.get_digital(DIGITAL_L2)){
			intake.move(-127);
		}
		else{
			intake.move (0);
		}
		if(master.get_digital(DIGITAL_R1)){
			score.move(127);
		}
		// else if(master.get_digital(DIGITAL_R2)){
		// 	score.move(-127);
		// }
		else{
			score.move_absolute(0,200);
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

void driveForward(double distance, int timeout,int speed, double Kp, double Kd, double TurningKp){
	leftMg.tare_position();
	rightMg.tare_position();
	imu.tare_rotation();
	double power;
	double lastError, derivative;
	double turningError; 
	double drivingDistanceRemaining = distance - ((leftMg.get_position() + rightMg.get_position())/2); 
	int startTime = millis();
	while(abs(drivingDistanceRemaining) >= 1 && millis() - startTime < timeout){ //distance left 
		drivingDistanceRemaining = distance - ((leftMg.get_position() + rightMg.get_position())/2);
		turningError = imu.get_rotation(); 
		derivative = lastError - drivingDistanceRemaining; 
		lastError = drivingDistanceRemaining; 
		power = drivingDistanceRemaining*Kp + derivative*Kd; 
		powerDriveMotor(speedLimit(power,speed),-turningError*TurningKp); 
		delay(8);
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

int speedLimit(int speed, int limit) {
	if (speed > limit){
		return limit;
	}
	else if(speed < limit && speed > 0){
		return speed;
	} 
	else if(speed < 0 && abs(speed) > limit){
		return -limit; 
	}
	else {
		return speed;
	}
}