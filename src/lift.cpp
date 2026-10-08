#include "subsystems/lift.hpp"

// port 11 is reversed so both motors drive the cascade the same way
pros::MotorGroup liftMotors({10, -11}, pros::MotorGearset::green);

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
