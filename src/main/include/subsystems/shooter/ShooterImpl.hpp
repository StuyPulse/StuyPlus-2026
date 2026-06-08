#pragma once
#include "subsystems/shooter/Shooter.hpp"
#include <ctre/phoenix6/TalonFX.hpp>
#include "constants/Ports.hpp"
#include "constants/Settings.hpp"
#include <telemetrykit/TelemetryKit.h>

class ShooterImpl : public Shooter
{
public:
    void Periodic() override;

    units::angular_velocity::revolutions_per_minute_t getCurrentAngularVelocity() override {
        return shooterMotorRight.GetVelocity().GetValue();
    }

    void StopMotors() override {
        shooterMotorRight.StopMotor();
    }
private:
    ctre::phoenix6::hardware::TalonFX shooterMotorRight{Ports::Feeder::FEEDER_MOTOR, Settings::CANBus};
};