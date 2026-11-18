#pragma once

#include "main.h"
#include <atomic>

// Lift motors: ports 9 and 10 (red cartridge), spun in opposite directions
extern pros::MotorGroup liftMotors;

// controller defined in main.cpp
extern pros::Controller controller;

// set true in opcontrol to let the lift task read driver input
extern std::atomic<bool> liftEnabled;

// FreeRTOS task body: R1 raises the lift, R2 lowers it
void liftControl();
