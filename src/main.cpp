#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "liblvgl/display/lv_display.h"
#include "liblvgl/widgets/label/lv_label.h"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "main.h"

//gear ratio 48/60
// lift two motors diff direction
// clamp one motor 

// Applying positive voltage
pros::MotorGroup left_motors({-1, 2, -3, 4, -5}, pros::MotorGearset::blue);
pros::MotorGroup right_motors({-6, 7, -8, 9, -10}, pros::MotorGearset::blue);

// CONFIGURE PORT NUMBERS
// CHECK MOTOR GEAR SET COLOR
pros::Motor lift_pos(11, pros::MotorGearset::green);
pros::Motor lift_neg(-12, pros::MotorGearset::green);

// CONFIGURE PORT NUMBERS
// CHECK MOTOR GEAR SET COLOR
pros::Motor clamp(13, pros::MotorGearset::red);

pros::Motor intake(14, pros::MotorGearset::green);


// DRIVETRAIN SETUP NOT FINAL
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              12.79, // 14.5 NOT FINAL
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              480, // drivetrain rpm is 480
                              8 // Horizontal drift 
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

/*  0 - None
    1 - Auto for bigger bot
    2 - Auto for smaller bot
    3 - Skills
    4 - OpControl
*/
static int id_big = 1;
static int id_small = 2;
static int id_skills = 3;

lv_obj_t *btn_big;
lv_obj_t *btn_small;
lv_obj_t *btn_skills;

bool selected_routine_bool = false;

int selected_routine;
lv_obj_t* status_label;

static void button_click_brain(lv_event_t *e) {

    if (selected_routine_bool) {
        return;
    }

    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_CLICKED) {
        int *id = (int*) lv_event_get_user_data(e);
        if (id != nullptr) {
            selected_routine = *id;
            selected_routine_bool = true;

            switch(*id) {
                case 1: lv_label_set_text(status_label, "Selected: Big Auto"); break;
                case 2: lv_label_set_text(status_label, "Selected: Small Auto"); break;
                case 3: lv_label_set_text(status_label, "Skills"); break;
            }
        }
    }
}

// this runs at the start of the program
void initialize() {
    selected_routine = 0;

    lv_obj_t *title = lv_label_create(lv_screen_active());
    lv_label_set_text(title, "BCC AUTO AND SKILLS");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 15);

    status_label = lv_label_create(lv_screen_active());
    lv_label_set_text(status_label, "Selected: NONE");
    lv_obj_align(status_label, LV_ALIGN_TOP_MID, 0, 45);



    // Big Robot Button
    lv_obj_t *btn_big = lv_button_create(lv_screen_active());
    lv_obj_set_pos(btn_big, 30, 80);
    lv_obj_set_size(btn_big, 200, 50);
    // Attach event callback and pass 1 as user data
    lv_obj_add_event_cb(btn_big, button_click_brain, LV_EVENT_CLICKED, &id_big);

    lv_obj_t *label_big = lv_label_create(btn_big);
    lv_label_set_text(label_big, "Big Autonomous");
    lv_obj_center(label_big);



    // Small Robot Button
    lv_obj_t *btn_small = lv_button_create(lv_screen_active());
    lv_obj_set_pos(btn_small, 250, 80);
    lv_obj_set_size(btn_small, 200, 50);
    lv_obj_add_event_cb(btn_small, button_click_brain, LV_EVENT_CLICKED, &id_small);

    lv_obj_t *label_small = lv_label_create(btn_small);
    lv_label_set_text(label_small, "Small Autonomous");
    lv_obj_center(label_small);



    // Skills Button
    lv_obj_t *btn_skills = lv_button_create(lv_screen_active());
    lv_obj_set_pos(btn_skills, 30, 150);
    lv_obj_set_size(btn_skills, 420, 50);
    lv_obj_add_event_cb(btn_skills, button_click_brain, LV_EVENT_CLICKED, &id_skills);

    lv_obj_t *label_skills = lv_label_create(btn_skills);
    lv_label_set_text(label_skills, "Skills");
    lv_obj_center(label_skills);




    // maybe intefere with lvgl?
    //pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    /*pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        }
    });*/
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

void run_big_auto() {

    selected_routine_bool = false;
}

void run_small_auto() {

    selected_routine_bool = false;
}

void run_skills() {

    selected_routine_bool = false;
}

void autonomous() {
    switch (selected_routine) {
        case 1: run_big_auto(); break;
        case 2: run_small_auto(); break;
        case 3: run_skills(); break;
        default: break;
    }





    // voltage control
    // lift_neg.move_voltage(12000);

    chassis.setPose(0,0,0);

    // timeout is time in milliseconds to approximately complete the function
    // .forwards = false means face the point with the back of the robot
    // .maxSpeed is speed in voltage (127 is max)
    chassis.moveToPoint(0, 0, 0, {.forwards = false, .maxSpeed = 64}, false);
    
    chassis.turnToHeading(0, 0);
    chassis.swingToHeading(0, lemlib::DriveSide::RIGHT, 0);
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
    bool l1_pressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
    bool l2_pressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    bool r1_pressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    bool r2_pressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
    bool a_pressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
    bool b_pressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);
    bool x_pressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);
    bool y_pressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
    bool up_pressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
    bool down_pressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
    bool left_pressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT);
    bool right_pressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);



    while (true) {
        // get left y and right x positions
        int leftJoystick = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightJoystick = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        // move the robot
        chassis.arcade(leftJoystick, rightJoystick);

        // control lift
        // maybe make it quantized?
        if (l2_pressed) {
            lift_pos.move(127);
            lift_neg.move(127);
        } else if (r2_pressed) {
            lift_pos.move(-127);
            lift_neg.move(-127);
        } else {
            lift_pos.move(0);
            lift_neg.move(0);
        }

        // control clamp
        if (l1_pressed) {
            clamp.move(127);
        } else {
            clamp.move(0);
        }

        // control intake
        if (r1_pressed) {
            intake.move(127);
        } else {
            intake.move(0);
        }

        // delay to save resources
        pros::delay(25);
    }
}