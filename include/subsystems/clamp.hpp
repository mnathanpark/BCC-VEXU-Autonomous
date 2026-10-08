#pragma once

#include "main.h"
#include <atomic>

// Roller clamp motor: port 12 (red cartridge)
extern pros::Motor clampMotor;

// controller defined in main.cpp
extern pros::Controller controller;

// set true in opcontrol to let the clamp task read driver input
extern std::atomic<bool> clampEnabled;

// FreeRTOS task body: L1 spins the roller one way, L2 the other
void clampControl();
