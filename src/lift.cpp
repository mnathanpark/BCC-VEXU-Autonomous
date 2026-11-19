#include "subsystems/lift.hpp"

// port 10 is reversed so both motors drive the lift the same way
pros::MotorGroup liftMotors({9, -10}, pros::MotorGearset::red);

std::atomic<bool> liftEnabled{false};
std::atomic<bool> liftAutoActive{false};

void liftControl() {
    liftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); // keep the lift from sliding down

    while (true) {
        if (!liftAutoActive.load()) {
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
        }

        // delay to save resources
        pros::delay(20);
    }
}

void move_lift(int duration_ms, int direction) {
    if (duration_ms <= 0) {
        return;
    }

    int speed = 0;
    if (direction > 0) {
        speed = 127;
    } else if (direction < 0) {
        speed = -127;
    }

    liftAutoActive.store(true);
    liftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    liftMotors.move(speed);
    pros::delay(duration_ms);
    liftMotors.brake();
    liftAutoActive.store(false);
}