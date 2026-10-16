#include "subsystems/clamp.hpp"

pros::Motor clampMotor(12, pros::MotorGearset::red);

extern pros::Motor clampMotor;
extern std::atomic<bool> clampEnabled;

void clampControl() {
    clampMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); // keep grip when released

    while (true) {
        if (clampEnabled.load()) {
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                clampMotor.move_voltage(12000);
            } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
                clampMotor.move_voltage(-12000);
            } else {
                clampMotor.brake();
            }
        } else {
            clampMotor.brake();
        }

        pros::delay(20);
    }
}