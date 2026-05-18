#pragma once
#include "subsystems/feeder/Feeder.hpp"
#include <ctre/phoenix6/TalonFX.hpp>
#include "constants/Ports.hpp"
#include "constants/Settings.hpp"
#include <telemetrykit/TelemetryKit.h>

class FeederImpl : public Feeder
{
public:
    void Periodic() override;

    units::angular_velocity::revolutions_per_minute_t getCurrentAngularVelocity() override {
        return feederMotor.GetVelocity().GetValue();
    }

    void stopMotors() override {
        feederMotor.StopMotor();
    }
private:
    ctre::phoenix6::hardware::TalonFX feederMotor{Ports::Feeder::FEEDER_MOTOR, Settings::CANBus};
};