#include "subsystems/lift.hpp"

// port 10 is reversed so both motors drive the lift the same way
pros::MotorGroup liftMotors({9, -10}, pros::MotorGearset::red);

std::atomic<bool> liftEnabled{false};

void liftControl() {
    liftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); // keep the lift from sliding down

    while (true) {
        if (liftEnabled.load()) {
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
                liftMotors.move(127); // raise
            } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
                liftMotors.move(-127); // lower
            } else {
                liftMotors.brake();
            }
        } else {
            liftMotors.brake();
        }

        // delay to save resources
        pros::delay(20);
    }
}
