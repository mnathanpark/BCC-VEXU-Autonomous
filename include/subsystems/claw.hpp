#pragma once

#include "main.h"
#include <atomic>

// Ratchet claw motor: port 11 (green cartridge)
extern pros::Motor clawMotor;

// controller defined in main.cpp
extern pros::Controller controller;

// set true in opcontrol to let the claw task read driver input
extern std::atomic<bool> clawEnabled;

// set true while autonomous is actively driving clawMotor
extern std::atomic<bool> clawAutoActive;

// FreeRTOS task body: L1 spins the claw one way, L2 the other
void clawControl();

// Autonomous helper: move claw for duration_ms (ms); direction > 0 or < 0 sets direction
void move_claw(int duration_ms, int direction);