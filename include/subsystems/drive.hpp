#pragma once

#include "lemlib/api.hpp" // IWYU pragma: keep
#include "main.h"
#include <atomic>

// chassis and controller defined in main.cpp
extern lemlib::Chassis chassis;
extern pros::Controller controller;

// set true in opcontrol to let the drive task read the joysticks
extern std::atomic<bool> driveEnabled;

// FreeRTOS task body: arcade drive from the left stick
void driveControl();
