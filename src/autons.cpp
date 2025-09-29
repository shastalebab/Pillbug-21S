#include "main.h"  // IWYU pragma: keep

// Commonly used speed constants
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 90;

///
// Constants
///
void default_constants() {
	// P, I, D, and Start I
	chassis.pid_drive_constants_set(13.25, 0.0, 61.25);	 // Straight driving constants, used for odom and non odom motions
	chassis.pid_heading_constants_set(11.75, 0.0,
									  31.25);  // Holds the robot straight while going forward without odom
	chassis.pid_turn_constants_set(2.75, 0.15, 27.00,
								   30.0);					   // Turn in place constants
	chassis.pid_swing_constants_set(9.25, 0.0, 78.25);		   // Swing constants
	chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);	   // Angular control for odom motions
	chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions
	chassis.pid_drive_constants_get();

	// Exit conditions
	chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms, false);
	chassis.pid_swing_exit_condition_set(70_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms, false);
	chassis.pid_drive_exit_condition_set(70_ms, 1.7_in, 180_ms, 4_in, 200_ms, 200_ms, false);
	chassis.pid_odom_turn_exit_condition_set(40_ms, 3_deg, 170_ms, 6_deg, 500_ms, 750_ms, false);
	chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms, false);
	chassis.pid_turn_chain_constant_set(4_deg);
	chassis.pid_swing_chain_constant_set(5_deg);
	chassis.pid_drive_chain_constant_set(4_in);
	chassis.drive_imu_scaler_set(1.00005);

	// Slew constants
	chassis.slew_turn_constants_set(3_deg, 70);
	chassis.slew_drive_constants_set(3_in, 70);
	chassis.slew_swing_constants_set(3_in, 80);

	// The amount that turns are prioritized over driving in odom motions
	// - if you have tracking wheels, you can run this higher.  1.0 is the max
	chassis.odom_turn_bias_set(0.5);

	chassis.odom_look_ahead_set(7_in);			 // This is how far ahead in the path the robot looks at
	chassis.odom_boomerang_distance_set(16_in);	 // This sets the maximum distance away from target that the carrot
												 // point can be
	chassis.odom_boomerang_dlead_set(0.625);	 // This handles how aggressive the end of boomerang motions are

	chassis.pid_angle_behavior_set(shortest);  // Changes the default behavior for turning, this defaults it to
											   // the shortest path there
}

//
// PID TUNING/TESTING ROUTINES
//

void drive_test(int inches) {
	chassis.pid_drive_set(inches, 127);
	chassis.pid_wait();
}

void turn_test(int degrees) {
	chassis.pid_turn_set(degrees, 127, raw);
	chassis.pid_wait();
}

void swing_test(int degrees) {
	chassis.pid_swing_set(LEFT_SWING, degrees, SWING_SPEED, raw);
	chassis.pid_wait();
}

void heading_test(int degrees) {
	chassis.pid_drive_set(12, 127);
	chassis.pid_wait_quick_chain();
	chassis.pid_turn_set(degrees, 127, raw);
	chassis.pid_wait_quick_chain();
	chassis.pid_drive_set(12, 127);
}

void constants_test() {
	setPosition(85.44, 20.86, 45);
	driveSet(12, 127);
	pidWait(WAIT);
	turnSet(90, 127);
	pidWait(WAIT);
	turnSet(0, 127);
	pidWait(WAIT);
	turnSet(45, 127);
	pidWait(WAIT);
}

//
// RIGHT AUTONS
//

void right_split() {
	setPosition(85.44, 20.86, 45);
	// Collect and score middle three blocks in mid goal
	driveSet(29.75, DRIVE_SPEED);
	pidWait(WAIT);
	swingSet(LEFT_SWING, -45, 90, -45, ccw);
	pidWait(WAIT);
	setIntake(127);
	driveSet(27.75, DRIVE_SPEED);
	pidWait(WAIT);
	delayMillis(200);
	setIntake(-100, -100, 100, false);
	delayMillis(1000);
	// Grab blocks under long goal
	setIntake(127);
	swingSet(RIGHT_SWING, 61, SWING_SPEED);
	pidWait(WAIT);
	driveSet(28, DRIVE_SPEED);
	pidWait(WAIT);
	setScraper(true);
	delayMillis(400);
	// Align to loader/long goal
	driveSet(-28, DRIVE_SPEED, true);
	pidWait(WAIT);
	moveToPoint({121.5, 25}, fwd, TURN_SPEED);
	pidWait(WAIT);
	turnSet(180, TURN_SPEED);
	pidWait(WAIT);
	// Grab blocks from loader and score on long goal
	driveSet(17, 60);
	delayMillis(800);
	driveSet(-8, DRIVE_SPEED);
	pidWait(WAIT);
	turnSet(1, TURN_SPEED);
	setScraper(false);
	pidWait(WAIT);
	driveSet(18.5, 60);
	pidWait(WAIT);
	setIntake(127, -127, 127, true);
}

void right_greed() {
	setPosition(85.44, 20.86, 45);
	// Collect and score middle three blocks in mid goal
	driveSet(29.75, DRIVE_SPEED, true);
	pidWait(WAIT);
	delayMillis(200);
	swingSet(LEFT_SWING, -45, 90, -45, ccw);
	pidWait(WAIT);
	driveSet(19, DRIVE_SPEED, true);
	delayMillis(200);
	setIntake(127);
	pidWait(WAIT);
	// Grab blocks under long goal
	turnSet(61, TURN_SPEED);
	pidWait(WAIT);
	driveSet(22, DRIVE_SPEED);
	pidWait(WAIT);
	setScraper(true);
	delayMillis(400);
	// Align to loader/long goal
	driveSet(-22, DRIVE_SPEED, true);
	pidWait(WAIT);
	moveToPoint({121.5, 25}, fwd, TURN_SPEED);
	pidWait(WAIT);
	delayMillis(200);
	turnSet(180, TURN_SPEED);
	pidWait(WAIT);
	// Grab blocks from loader and score on long goal
	driveSet(17, 60);
	delayMillis(800);
	driveSet(-8, DRIVE_SPEED);
	pidWait(WAIT);
	delayMillis(1000);
	turnSet(1, TURN_SPEED);
	setScraper(false);
	pidWait(WAIT);
	driveSet(18.5, 60);
	pidWait(WAIT);
	setIntake(127, -127, 127, true);
}

void right_awp() {
	setPosition(85.44, 20.86, 45);
	// Collect and score middle three blocks in mid goal
	driveSet(29.75, DRIVE_SPEED);
	pidWait(WAIT);
	swingSet(LEFT_SWING, -45, 90, -45, ccw);
	pidWait(WAIT);
	setIntake(127);
	driveSet(23.75, DRIVE_SPEED);
	pidWait(WAIT);
	delayMillis(200);
	setIntake(-100, -100, 100, false);
	delayMillis(1000);
	// Grab blocks on other side of field and score
	setIntake(127);
	driveSet(-7.75, DRIVE_SPEED, true);
	pidWait(WAIT);
	turnSet(-90, TURN_SPEED);
	pidWait(WAIT);
	driveSet(45, DRIVE_SPEED);
	pidWaitUntil(28_in);
	chassis.pid_speed_max_set(60);
	pidWait(WAIT);
	setIntake(127, 40, 0, 0);
	turnSet(45, TURN_SPEED);
	pidWait(WAIT);
	driveSet(12, DRIVE_SPEED);
	setIntake(90, 70, -60, false);
	pidWait(WAIT);
	delayMillis(1500);
	// Align to loader
	setIntake(127);
	driveSet(-45.5, DRIVE_SPEED, true);
	pidWait(WAIT);
	turnSet(180, TURN_SPEED);
	setScraper(true);
	pidWait(WAIT);
	// Grab blocks from loader and score on long goal
	driveSet(14, 80);
	delayMillis(900);
	driveSet(-8, DRIVE_SPEED);
	pidWait(CHAIN);
	turnSet(-1, TURN_SPEED);
	setScraper(false);
	pidWait(WAIT);
	driveSet(14, DRIVE_SPEED);
	pidWait(WAIT);
	setIntake(127, -127, 127, true);
}

//
// LEFT AUTONS
//

void left_split() {
	setPosition(58.56, 20.86, -45);
	// Collect and score middle three blocks in mid goal
	driveSet(30.5, DRIVE_SPEED, true);
	pidWait(WAIT);
	delayMillis(200);
	swingSet(RIGHT_SWING, 45, 90, -45, cw);
	pidWait(WAIT);
	driveSet(27.75, 70, true);
	delayMillis(200);
	setIntake(127);
	pidWait(WAIT);
	delayMillis(1200);
	setIntake(110, -90, -30, false);
	delayMillis(1000);
	// Grab blocks under long goal
	setIntake(127);
	swingSet(LEFT_SWING, -61, SWING_SPEED);
	pidWait(WAIT);
	driveSet(26, DRIVE_SPEED);
	pidWait(WAIT);
	setScraper(true);
	delayMillis(400);
	// Align to loader/long goal
	driveSet(-26, DRIVE_SPEED, true);
	pidWait(WAIT);
	moveToPoint({22.5, 25}, fwd, TURN_SPEED);
	pidWait(WAIT);
	turnSet(180, TURN_SPEED);
	pidWait(WAIT);
	// Grab blocks from loader and score on long goal
	driveSet(17, 60);
	delayMillis(800);
	driveSet(-8, DRIVE_SPEED);
	pidWait(WAIT);
	turnSet(-1, TURN_SPEED);
	setScraper(false);
	pidWait(WAIT);
	driveSet(18.5, 60);
	pidWait(WAIT);
	setIntake(127, -127, 127, true);
}

void left_greed() {
	setPosition(58.56, 20.86, -45);
	// Collect and score middle three blocks in mid goal
	driveSet(30.5, DRIVE_SPEED, true);
	pidWait(WAIT);
	delayMillis(200);
	swingSet(RIGHT_SWING, 45, 90, -45, cw);
	pidWait(WAIT);
	driveSet(19, DRIVE_SPEED, true);
	delayMillis(200);
	setIntake(127);
	pidWait(WAIT);
	// Grab blocks under long goal
	turnSet(-61, TURN_SPEED);
	pidWait(WAIT);
	driveSet(22, DRIVE_SPEED);
	pidWait(WAIT);
	setScraper(true);
	delayMillis(400);
	// Align to loader/long goal
	driveSet(-22, DRIVE_SPEED, true);
	pidWait(WAIT);
	moveToPoint({23, 25}, fwd, TURN_SPEED);
	pidWait(WAIT);
	delayMillis(200);
	turnSet(180, TURN_SPEED);
	pidWait(WAIT);
	// Grab blocks from loader and score on long goal
	driveSet(17, 60);
	delayMillis(800);
	driveSet(-8, DRIVE_SPEED);
	pidWait(WAIT);
	delayMillis(1000);
	turnSet(-1, TURN_SPEED);
	setScraper(false);
	pidWait(WAIT);
	driveSet(18.5, 60);
	pidWait(WAIT);
	setIntake(127, -127, 127, true);
}

void left_awp() {
	setPosition(58.56, 20.86, -45);
	// Collect and score middle three blocks in mid goal
	driveSet(29.75, DRIVE_SPEED);
	pidWait(WAIT);
	swingSet(RIGHT_SWING, 45, 90, -45, cw);
	pidWait(WAIT);
	setIntake(127);
	driveSet(28.75, 70);
	delayMillis(200);
	setIntake(110, 90, -60, false);
	pidWait(WAIT);
	delayMillis(1500);
	// Grab blocks on other side of field and score
	setIntake(127);
	driveSet(-12.75, DRIVE_SPEED, true);
	pidWait(WAIT);
	turnSet(90, TURN_SPEED);
	pidWait(WAIT);
	driveSet(47, DRIVE_SPEED);
	pidWaitUntil(28_in);
	chassis.pid_speed_max_set(60);
	pidWait(WAIT);
	setIntake(127, false);
	turnSet(-45, TURN_SPEED);
	pidWait(WAIT);
	driveSet(12.5, DRIVE_SPEED);
	pidWait(WAIT);
	setIntake(-100, -100, 100, false);
	delayMillis(1000);
	// Align to loader
	setIntake(127);
	driveSet(-47, DRIVE_SPEED, true);
	pidWait(WAIT);
	turnSet(180, TURN_SPEED);
	setScraper(true);
	pidWait(WAIT);
	// Grab blocks from loader and score on long goal
	driveSet(14, 80);
	delayMillis(900);
	driveSet(-8, DRIVE_SPEED);
	pidWait(CHAIN);
	turnSet(1, TURN_SPEED);
	setScraper(false);
	pidWait(WAIT);
	driveSet(14, DRIVE_SPEED);
	pidWait(WAIT);
	setIntake(127, -127, 127, true);
}

//
// SKILLS
//

void skills() {
	Colors userColor = allianceColor;
	setPosition(89.5, 9, -90);
	setAlliance(NEUTRAL);
	// Clear parking barrier
	if(autonMode != BRAIN) {
		driveSet(60, 80, true);
		setIntake(127);
		pidWait(WAIT);
		swingSet(LEFT_SWING, 90, SWING_SPEED, cw);
		pidWait(WAIT);
		setPosition(getDistanceActual(), 21.5, 90);
	} else {
		driveSet(48, 70, true);
		pidWait(WAIT);
	}
	// Score parking barrier blocks
	moveToPoint({24, 24}, fwd, DRIVE_SPEED);
	pidWait(WAIT);
	moveToPoint({24, 42}, fwd, DRIVE_SPEED);
	pidWait(WAIT);
	setIntake(127, -127, 127, true);
	delayMillis(1200);
	setIntake(127);
	// Intake and score matchloader blocks
	driveSet(-10, DRIVE_SPEED);
	pidWait(WAIT);
	turnSet(180, TURN_SPEED);
	setScraper(true);
	pidWait(WAIT);
	driveSet(17, 60);
	delayMillis(1800);
	driveSet(-8, DRIVE_SPEED);
	pidWait(WAIT);
	delayMillis(1000);
	turnSet(-1, TURN_SPEED);
	setScraper(false);
	pidWait(WAIT);
	driveSet(18.5, 60);
	pidWait(WAIT);
	setIntake(127, -127, 127, true);
	delayMillis(1200);
	setIntake(127);
	// Go to other side of field and repeat matchload scoring
	swingSet(RIGHT_SWING, 90, SWING_SPEED, 35, cw);
	pidWait(WAIT);
	moveToPoint({12, 104}, fwd, DRIVE_SPEED);
	pidWait(WAIT);
	moveToPoint({24, 120}, fwd, TURN_SPEED);
	pidWait(WAIT);
	turnSet(0, TURN_SPEED);
	setScraper(true);
	pidWait(WAIT);
	driveSet(17, 60);
	delayMillis(1200);
	driveSet(-8, DRIVE_SPEED);
	pidWait(WAIT);
	delayMillis(1000);
	turnSet(181, TURN_SPEED);
	setScraper(false);
	setAlliance(BLUE);
	pidWait(WAIT);
	driveSet(18.5, 60);
	pidWait(WAIT);
	setIntake(127, -127, 127, true);
	delayMillis(1200);
	// Collect other parking barrier blocks
	setIntake(127);
	driveSet(-12, DRIVE_SPEED);
	pidWait(WAIT);
	moveToPoint({40, 130}, fwd, DRIVE_SPEED);
	pidWait(WAIT);
	setAlliance(NEUTRAL);
	turnSet(45, TURN_SPEED);
	pidWait(WAIT);
	swingSet(LEFT_SWING, 90, SWING_SPEED, 30, cw);
	if(autonMode != BRAIN) {
		driveSet(60, 80, true);
		setIntake(127);
		pidWait(WAIT);
		swingSet(LEFT_SWING, 90, SWING_SPEED, cw);
		pidWait(WAIT);
		setPosition(144 - getDistanceActual(), 122.5, 90);
	} else {
		driveSet(48, 70, true);
		pidWait(WAIT);
	}
	pidWait(WAIT);
	moveToPoint({120, 120}, fwd, DRIVE_SPEED);
	pidWait(WAIT);
	moveToPoint({120, 102}, fwd, DRIVE_SPEED);
	pidWait(WAIT);
	setIntake(127, -127, 127, true);
	delayMillis(1200);
	setIntake(127);
	// Intake and score matchloader blocks
	driveSet(-10, DRIVE_SPEED);
	pidWait(WAIT);
	turnSet(0, TURN_SPEED);
	setScraper(true);
	pidWait(WAIT);
	driveSet(17, 60);
	delayMillis(1800);
	driveSet(-8, DRIVE_SPEED);
	pidWait(WAIT);
	delayMillis(1000);
	turnSet(181, TURN_SPEED);
	setScraper(false);
	pidWait(WAIT);
	driveSet(18.5, 60);
	pidWait(WAIT);
	setIntake(127, -127, 127, true);
	delayMillis(1200);
	setIntake(127);
	// Go to other side of field and repeat matchload scoring again, but only on low goals
	swingSet(RIGHT_SWING, 270, SWING_SPEED, 35, cw);
	pidWait(WAIT);
	moveToPoint({132, 40}, fwd, DRIVE_SPEED);
	pidWait(WAIT);
	moveToPoint({120, 24}, fwd, TURN_SPEED);
	pidWait(WAIT);
	turnSet(180, TURN_SPEED);
	setScraper(true);
	pidWait(WAIT);
	driveSet(17, 60);
	delayMillis(1800);
	driveSet(-8, DRIVE_SPEED);

	pidWait(WAIT);
	moveToPoint({72, 12}, fwd, DRIVE_SPEED);

	/*
	pidWait(WAIT);
	setAlliance(RED);
	setIntake(127, -127, 127, false);
	delayMillis(2000);
	// Score blue on mid goal
	setAlliance(BLUE);
	setIntake(127);
	moveToPoint({96, 48}, fwd, 70);
	pidWait(WAIT);
	moveToPoint({48, 48}, fwd, 70);
	pidWait(WAIT);
	moveToPoint({58, 58}, fwd, 70);
	pidWait(WAIT);
	setIntake(80, -90, -30, false);
	delayMillis(2500);
	setIntake(127);
	// Score red on low goal
	setAlliance(RED);
	moveToPoint({48, 48}, rev, DRIVE_SPEED);
	pidWait(WAIT);
	moveToPoint({96, 48}, rev, DRIVE_SPEED);
	pidWait(WAIT);
	moveToPoint({92, 18}, fwd, TURN_SPEED);
	pidWait(WAIT);
	moveToPoint({96, 96}, fwd, 90);
	pidWait(WAIT);
	moveToPoint({48, 96}, fwd, 70);
	pidWait(WAIT);
	moveToPoint({59, 85}, fwd, 70);
	pidWait(WAIT);
	setIntake(-70, -100, 127, false);
	delayMillis(3000);
	setAlliance(NEUTRAL);
	setIntake(127);
	moveToPoint({48, 96}, rev, DRIVE_SPEED);
	pidWait(WAIT);
	moveToPoint({72, 112}, fwd, DRIVE_SPEED);
	pidWait(WAIT);
	turnSet(0, TURN_SPEED);
	pidWait(WAIT);
	driveSet(24, DRIVE_SPEED);
	pidWait(CHAIN);
	if(autonMode != BRAIN) {
		chassis.drive_set(DRIVE_SPEED, DRIVE_SPEED);
		delayMillis(1000);
		driveSet(5, DRIVE_SPEED);
	}
	setAlliance(userColor);
	*/
}