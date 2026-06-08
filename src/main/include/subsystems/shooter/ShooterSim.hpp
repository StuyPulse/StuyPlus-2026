#pragma once
#include "subsystems/shooter/Shooter.hpp"
#include <ctre/phoenix6/TalonFX.hpp>
#include "constants/Ports.hpp"
#include "constants/Settings.hpp"
#include <telemetrykit/TelemetryKit.h>

using namespace ctre::phoenix6::hardware;

class ShooterSim : public Shooter
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
    TalonFX shooterMotorRight{Ports::Shooter::SHOOTER_MOTOR_RIGHT, Settings::CANBus};
    TalonFX shooterMotorCenter{Ports::Shooter::SHOOTER_MOTOR_CENTRE, Settings::CANBus};
    TalonFX shooterMotorLeft{Ports::Shooter::SHOOTER_MOTOR_LEFT, Settings::CANBus};
};