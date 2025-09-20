#include "main.h"

// Chassis constructor
ez::Drive chassis(
	// These are your drive motors, the first motor is used for sensing!
	{-11, -12, -13},  // Left Chassis Ports (negative port will reverse it!)
	{14, 15, 16},	  // Right Chassis Ports (negative port will reverse it!)

	17,				 // IMU Port
	WHEEL_DIAMETER,	 // Wheel Diameter
	450);			 // Wheel RPM

void initialize() {
	pros::delay(500);  // Stop the user from doing anything while legacy ports configure

	// Configure chassis controls
	chassis.opcontrol_curve_buttons_toggle(false);	// Enables modifying the controller curve with controller buttons
	chassis.opcontrol_drive_activebrake_set(0.0);	// Sets the active brake kP
	chassis.opcontrol_curve_default_set(0.0,
										0.0);  // Set defaults for controller curve

	// Set default drive constants
	default_constants();

	// Add autons to auton selector
	auton_sel.selector_populate({{right_greed, "right_greed", "right side 9 in long goal", lv_color_darken(green, 60)},
								 {right_split, "right_split", "right side 4 + 5", gray},
								 {right_awp, "right_awp", "right side 4 + 3 + 3 solo AWP", violet},
								 {left_greed, "left_greed", "left side 9 in long goal", green},
								 {left_split, "left_split", "left side 4 + 5", lv_color_lighten(gray, 125)},
								 {left_awp, "left_awp", "left side 4 + 3 + 3 solo AWP", pink},
								 {constants_test, "constants_test", "drive and turn", blue},
								{skills, "skills", "skills route", lv_color_darken(blue, 60)}});

	// Initialize chassis, auton selector, and tasks
	chassis.initialize();
	uiInit();
	pros::Task ColorTask(colorTask, "color sort");
	pros::Task AntiJamTask(antiJamTask, "antijam");
	pros::Task ControllerTask(controllerTask, "controller printing");
	pros::Task PathViewerTask(pathViewerTask, "path viewer");
	pros::Task AngleCheckTask(angleCheckTask, "angle checker");
	pros::Task MotorUpdateTask(motorUpdateTask, "motor info updater");
	master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
}

void disabled() {}

void competition_initialize() { autonMode = BRAIN; }

void autonomous() {
	chassis.pid_targets_reset();   // Resets PID targets to 0
	chassis.drive_imu_reset();	   // Reset gyro position to 0
	chassis.drive_sensor_reset();  // Reset drive sensors to 0
	chassis.odom_xyt_set(0_in, 0_in,
						 0_deg);				// Reset current position to a default pose
	chassis.drive_brake_set(MOTOR_BRAKE_HOLD);	// Set motors to hold.  This helps
												// autonomous consistency

	autonMode = PLAIN;				// Sets which "mode" the auton is run in (between "PLAIN"
									// and "ODOM")
	autonPath = {};					// Clears saved auton path
	auton_sel.selector_callback();	// Calls selected auton from autonomous selector
}

void opcontrol() {
	chassis.drive_brake_set(pros::E_MOTOR_BRAKE_BRAKE);
	autonMode = PLAIN;	// Sets "mode" to "PLAIN", to ensure that any drive macros
						// in opcontrol only use PID

	while(true) {
		if(!probing) chassis.opcontrol_tank();	// Tank control

		setParkOp(); // Double park macros
		setIntakeOp();	 // Intake controls
		setScraperOp();	 // Scraper controls
		setDescoreOp();	 // Descore mech controls

		pros::delay(ez::util::DELAY_TIME);
	}
}
