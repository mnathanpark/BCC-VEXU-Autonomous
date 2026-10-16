#include "lemlib/api.hpp" // IWYU pragma: keep
#include "main.h"
#include "subsystems/clamp.hpp"
#include "subsystems/drive.hpp"
#include "subsystems/lift.hpp"

//gear ratio 48/60

// Applying positive voltage
pros::MotorGroup leftDrive({-1, -2, 3, -9}, pros::MotorGearset::blue);
pros::MotorGroup rightDrive({5, 6, -7, 8}, pros::MotorGearset::blue);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftDrive, // left motor group
                              &rightDrive, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

pros::Rotation x_sensor(19);

pros::Rotation y_sensor(20);

pros::Imu imu(17);

// CONFIGURE OFFSET WHEN MAKING BOT
lemlib::TrackingWheel horizontal_tracking_wheel(&x_sensor, lemlib::Omniwheel::NEW_2, -5.75);
lemlib::TrackingWheel vertical_tracking_wheel(&y_sensor, lemlib::Omniwheel::NEW_2, -2.5);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain,
                        lateral_controller,
                        angular_controller,
                        sensors,
                        &throttle_curve, 
                        &steer_curve
);

// this runs at the start of the program
void initialize() {
    // start the subsystem tasks before anything else
    pros::Task lift_task(liftControl, "Lift Task");
    pros::Task drive_task(driveControl, "Drive Task");
    pros::Task clamp_task(clampControl, "Clamp Task");

    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        }
    });
}

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
/*
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
}
*/

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
    // stop driver input so it can't fight autonomous motions later
    liftEnabled.store(false);
    driveEnabled.store(false);
    clampEnabled.store(false);
}

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
void auto_top_bot();
void auto_right_bot();

void autonomous() {
    // driver input off so the drive task doesn't override LemLib motions
    liftEnabled.store(false);
    driveEnabled.store(false);
    clampEnabled.store(false);

    clampMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
// Call auto route
auto_top_bot();
// auto_right_bot();

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

pros::Controller controller(pros::E_CONTROLLER_MASTER);

void opcontrol() {
    liftEnabled.store(true); // let the lift task take driver input
    driveEnabled.store(true); // let the drive task take joystick input
    clampEnabled.store(true); // let the clamp task take driver input

    while (true) {
        // drive, lift, and clamp run in their own tasks
        pros::delay(25);
    }















	/*while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		// Arcade control scheme
		int dir = controller.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = controller.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		left_motors.move(dir - turn);                      // Sets left motor voltage
		right_motors.move(dir + turn);                     // Sets right motor voltage
		pros::delay(20);                               // Run for 20 ms then update
	}
    */
}

// HARDWARE FOR AUTONOMOUS
// Ensure 'extern pros::Motor clampMotor;' is in include/subsytems/clamp.hpp
// PLACEHOLDER PORT NUMBER
constexpr int INTAKE_PORT = 15;
pros::Motor intakeMotor(INTAKE_PORT, pros::MotorGearset::green);

void runIntake(int direction) {
	// direction: 1 = intake in. -1 = outtake, 0 = stop
	if (direction == 0) {
		intakeMotor.brake();
	} else {
		intakeMotor.move_voltage(direction * 120000);
	}
}

void runClamp(int direction) {
	// direction: 1 = clamp grab, -1 = release, 0 = hold/brake
	if (direction == 0) {
		clampMotor.brake();
	} else {
		clampMotor.move_voltage(direction * 120000);
	}
}

// ===============================
// ROUTE 1: TOP WALL POSITION (Bot 1 - Upper Route)
// ===============================
void auto_top_bot() {
	// Start Pos: Purple box against top wall, 1 tile left of center, facing south (180 deg)
	chassis.setPose(-24, 60, 180);

	// --- 1. Hit toggle / Move to upper-left black (Arrow 1 - Purple) / Relative to autonomous route strategy 1
    runIntake(1); // Start spinning green intake motor inward
    chassis.turnToHeading(205, 800);
    chassis.moveToPoint(-36, 36, 1500);
    
    // Grab with red clamp motor and hold
    runClamp(1);
    pros::delay(250);
    runClamp(0); // Brake hold keeps grip

    // --- 2. Preload in far Goal from matchload / Center diagonal cycle (Arrow 2 - Blue)
    chassis.turnToHeading(115, 800);
    chassis.moveToPoint(12, 12, 1800); // Drive down-right toward center yellow pin
    
    // Return back up-left along Arrow 2
    chassis.moveToPoint(-36, 36, 1800, {.forwards = false});

    // --- 3. Pin cup field pair in far goal (Arrow 3 - Long Cyan Sweep to Top Right) ---
    chassis.turnToHeading(75, 800);
    chassis.moveToPoint(60, 60, 2500); // Sweep across top field to top-right goal
    
    // Score / Release in top-right goal
    runClamp(-1);
    runIntake(-1);
    pros::delay(300);
    runClamp(0);
    runIntake(0);

    // --- 4. Cycle matchload and close goal (Arrow 4 - Orange Loop) ---
    runIntake(1);
    chassis.turnToHeading(240, 800);
    chassis.moveToPoint(12, 36, 1500); // Grab blue object at (12, 36)
    runClamp(1);
    pros::delay(250);
    runClamp(0);

    // Return to top-right corner goal to score
    chassis.moveToPoint(60, 60, 1500, {.forwards = false});
    runClamp(-1);
    runIntake(-1);
    pros::delay(300);
    runClamp(0);
    runIntake(0);

    // --- 5. Sprint Mid (When black is far) ---
    chassis.turnToHeading(225, 800);
    chassis.moveToPoint(0, 0, 1500);
}

// ===========================================================================
// ROUTE 2: RIGHT WALL START POSITION (Bot 2 - Blue Alliance Wall Route)
// ===========================================================================
void auto_right_bot() {
    // Start Pos: Purple box on right alliance line, just below center, facing West (270 deg)
    chassis.setPose(60, -6, 270);

    // --- 1. Hit toggle / Move to initial blue object (Arrow 1 - Purple) ---
    runIntake(1);
    chassis.turnToHeading(305, 800);
    chassis.moveToPoint(36, 12, 1200);
    
    runClamp(1);
    pros::delay(250);
    runClamp(0);

    // --- 2. Cycle North-South to top-right pin and back (Arrow 2 - Blue) ---
    chassis.turnToHeading(0, 800);
    chassis.moveToPoint(36, 36, 1200);
    chassis.moveToPoint(36, 12, 1200, {.forwards = false});

    // --- 3. Pin cup field pair in far goal (Arrow 3 - Long Cyan Drive to Bottom Right) ---
    chassis.turnToHeading(160, 800);
    chassis.moveToPoint(60, -60, 2200); // Drive down to bottom-right corner goal
    
    runClamp(-1);
    runIntake(-1);
    pros::delay(300);
    runClamp(0);
    runIntake(0);

    // --- 4. Cycle matchload and close goal (Arrow 4 - Bottom-Right Orange Loop) ---
    runIntake(1);
    chassis.turnToHeading(325, 800);
    chassis.moveToPoint(36, -24, 1500); // Drive to bottom-right black object
    runClamp(1);
    pros::delay(250);
    runClamp(0);

    // Reverse back into bottom-right corner goal
    chassis.moveToPoint(60, -60, 1500, {.forwards = false});
    runClamp(-1);
    runIntake(-1);
    pros::delay(300);
    runClamp(0);
    runIntake(0);

    // --- 5. Top-Right Corner Cycle (Arrow 5 - Yellow Loop) ---
    runIntake(1);
    chassis.turnToHeading(350, 800);
    chassis.moveToPoint(48, 12, 1800);
    chassis.moveToPoint(60, 60, 1800); // Finish at top-right corner
    runIntake(0);
}
