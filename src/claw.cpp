#include "subsystems/claw.hpp"

pros::Motor clawMotor(15, pros::MotorGearset::green);
//11 (bot 1)
//15 (bot 2)

std::atomic<bool> clawEnabled{false};
std::atomic<bool> clawAutoActive{false};

void clawControl() {
    clawMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); // keep the claw where it was left

    while (true) {
        if (!clawAutoActive.load()) {
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
        }

        // delay to save resources
        pros::delay(20);
    }
}

void move_claw(int duration_ms, int direction) {
    if (duration_ms <= 0) {
        return;
    }

    int speed = 0;
    if (direction > 0) {
        speed = 127;
    } else if (direction < 0) {
        speed = -127;
    }

    clawAutoActive.store(true);
    clawMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    clawMotor.move(speed);
    pros::delay(duration_ms);
    clawMotor.brake();
    clawAutoActive.store(false);
}