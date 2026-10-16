#include "subsystems/drive.hpp"

std::atomic<bool> driveEnabled{false};

void driveControl() {
    while (true) {
        // only touch the chassis in driver control so autonomous motions aren't overridden
        if (driveEnabled.load()) {
            // get left y and right x positions
            int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
            int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

            // move the robot
            chassis.arcade(leftY, leftX);
        }

        // delay to save resources
        pros::delay(25);
    }
}
