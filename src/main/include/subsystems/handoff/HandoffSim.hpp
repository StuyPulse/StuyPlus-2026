#pragma once

#include "Handoff.hpp"
#include <ctre/phoenix6/TalonFX.hpp>
#include "constants/Ports.hpp"
#include "constants/Settings.hpp"
#include <telemetrykit/TelemetryKit.h>

class HandoffSim : public Handoff
{
public:
    HandoffSim();

    void Periodic() override;

    void StopMotors() override {
        handoffMotor.StopMotor();
    }

private:
    ctre::phoenix6::hardware::TalonFX handoffMotor{Ports::Handoff::HANDOFF_MOTOR, Settings::CANBus};
};