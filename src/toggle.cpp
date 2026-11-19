#include "subsystems/toggle.hpp"

pros::Motor toggleMotor(12, pros::MotorGearset::green);

std::atomic<bool> toggleEnabled{false};
std::atomic<bool> toggleAutoActive{false};

void toggleControl() {
    toggleMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); // stay put when X is released

    while (true) {
        if (!toggleAutoActive.load()) {
            if (toggleEnabled.load() && controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
                toggleMotor.move(-127); // negative spins counterclockwise
            } else {
                toggleMotor.brake();
            }
        }

        // delay to save resources
        pros::delay(20);
    }
}

void change_toggle() {
    toggleAutoActive.store(true);
    toggleMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    toggleMotor.move(-127);
    pros::delay(1000); // Run for 1 second
    toggleMotor.brake();
    toggleAutoActive.store(false);
}