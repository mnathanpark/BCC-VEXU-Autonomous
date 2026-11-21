#include "lemlib/api.hpp" // IWYU pragma: keep
#include "main.h"
#include "pros/abstract_motor.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "subsystems/claw.hpp"
#include "pros/motors.hpp"
#include "subsystems/drive.hpp"
#include "subsystems/lift.hpp"
#include "subsystems/toggle.hpp"
pros::MotorGroup leftDrive({-1, -2, 3, -4}, pros::MotorGearset::blue);
pros::MotorGroup rightDrive({5, 6, -7, 8}, pros::MotorGearset::blue);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftDrive, // left motor group
                              &rightDrive, // right motor group
                              12.452374, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // NEW 3.25 inches wheels
                              450, // rpm is 450
                              2 // horizontal drift is 2 (for now)
);

pros::Rotation x_sensor(-17);
//bot 1 negative
//bot 2 positive

pros::Rotation y_sensor(-20);
//bot 1 negative
//bot 2 positive

pros::Imu imu(19);

// CONFIGURE OFFSET WHEN MAKING BOT
lemlib::TrackingWheel horizontal_tracking_wheel(&x_sensor, lemlib::Omniwheel::NEW_2, -3);
lemlib::TrackingWheel vertical_tracking_wheel(&y_sensor, lemlib::Omniwheel::NEW_2, -2.25);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(19, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              100, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(6, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              40, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in degrees
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in degrees
                                              0, // large error range timeout, in milliseconds
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
    pros::Task claw_task(clawControl, "Claw Task");
    pros::Task toggle_task(toggleControl, "Toggle Task");

    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    chassis.setPose(0,0,0);

    move_claw(500, 1);
    move_claw(1000, -1);
    clawMotor.brake();

    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            pros::lcd::print(3, "Xraw: %d", x_sensor.get_position());
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
    clawEnabled.store(false);
    toggleEnabled.store(false);
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

void autonomous() {
    //bot 1
    liftEnabled.store(false);
    driveEnabled.store(false);
    clawEnabled.store(false);
    toggleEnabled.store(false);

    leftDrive.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    rightDrive.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    chassis.setPose(0,0,90);

    change_toggle();
    move_lift(500, 1);
    chassis.moveToPoint(27.5, 16, 3000, {.maxSpeed = 35});
    chassis.waitUntilDone();
    move_lift(600, -1);
    pros::delay(500);
    move_claw(600, -1);
    
    
    chassis.moveToPoint(4,8,1000, {.forwards = false, .maxSpeed = 70});
    chassis.swingToHeading(90, DriveSide::LEFT, 1000);
    chassis.moveToPoint(46, 6, 2000, {.maxSpeed = 60});
    chassis.turnToHeading(45, 500);

    pros::Task calibrate_claw_and_pick_up_yellow_pin([]() {
        move_claw(250, 1);
        move_claw(500, -1);
        clawMotor.brake();
        //pros::delay(700);
        move_claw(1000, 1);
    }, "Calibrate claw and pick up yellow pin");


    chassis.moveToPoint(64, 27, 2000, {.maxSpeed = 20});
    chassis.waitUntilDone();

    move_lift(1000, 1);
    chassis.turnToHeading(-75, 1000, {.maxSpeed = 50});
    chassis.waitUntilDone();
    chassis.moveToPoint(27,27, 1000, {.maxSpeed = 50});
    pros::delay(2000);

    move_lift(250, -1);
    pros::delay(1000);
    move_claw(250, -1);

    chassis.moveToPoint(45, 26, 1000, {.forwards = false});
    chassis.turnToHeading(-30, 1000);
    chassis.moveToPoint(10, 60, 5000, {.maxSpeed = 80});



    //bot 2
    /*liftEnabled.store(false);
    driveEnabled.store(false);
    clawEnabled.store(false);
    toggleEnabled.store(false);

    leftDrive.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    rightDrive.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    chassis.setPose(0,0,-90);

    change_toggle();
    move_lift(700, 1);
    chassis.moveToPoint(-27.5, 17, 3000, {.maxSpeed = 35});
    chassis.waitUntilDone();
    move_lift(700, -1);
    pros::delay(500);
    move_claw(600, -1);

    chassis.moveToPoint(0, 5, 1000, {.forwards = false, .maxSpeed = 80});
    chassis.moveToPoint(5, 45, 3000, {.maxSpeed = 80});*/

}


pros::Controller controller(pros::E_CONTROLLER_MASTER);

void opcontrol() {
    liftEnabled.store(true); // let the lift task take driver input
    driveEnabled.store(true); // let the drive task take joystick input
    clawEnabled.store(true); // let the claw task take driver input
    toggleEnabled.store(true); // let the toggle task take driver input

    leftDrive.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    rightDrive.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    while (true) {
        // drive, lift, claw, and toggle run in their own tasks
        pros::delay(25);
    }
}