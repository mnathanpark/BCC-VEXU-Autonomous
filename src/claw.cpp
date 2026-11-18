#include "subsystems/claw.hpp"

pros::Motor clawMotor(11, pros::MotorGearset::green);

std::atomic<bool> clawEnabled{false};

void clawControl() {
    clawMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); // keep the claw where it was left

    while (true) {
        if (clawEnabled.load()) {
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                clawMotor.move(127);
            } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
                clawMotor.move(-127);
            } else {
                clawMotor.brake();
            }
        } else {
            clawMotor.brake();
        }

        // delay to save resources
        pros::delay(20);
    }
}
