#pragma once

#include "main.h"
#include <atomic>

// Toggle motor: port 12 (green cartridge)
extern pros::Motor toggleMotor;

// controller defined in main.cpp
extern pros::Controller controller;

// set true in opcontrol to let the toggle task read driver input
extern std::atomic<bool> toggleEnabled;

// set true while autonomous is actively driving toggleMotor
extern std::atomic<bool> toggleAutoActive;

// FreeRTOS task body: hold X to spin the toggle motor counterclockwise
void toggleControl();

// Autonomous helper to run the toggle motor
void change_toggle();