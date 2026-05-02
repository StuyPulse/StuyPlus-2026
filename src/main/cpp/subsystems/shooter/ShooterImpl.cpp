#include "subsystems/shooter/Shooter.hpp"
#include <ctre/phoenix6/TalonFX.hpp>
#include "constants/Ports.hpp"
#include "constants/Settings.hpp"
#include <telemetrykit/TelemetryKit.h>

class ShooterImpl : public Shooter
{
public:
    void Periodic() override {}
    ShooterImpl() {
        shooterMotorCentre.SetControl(ctre::phoenix6::controls::Follower(Ports::Shooter::SHOOTER_MOTOR_LEFT));
        shooterMotorRight.SetControl(ctre::phoenix6::controls::Follower(Ports::Shooter::SHOOTER_MOTOR_LEFT));
    }
private:
    ctre::phoenix6::hardware::TalonFX shooterMotorLeft{Ports::Shooter::SHOOTER_MOTOR_LEFT, Settings::CANBus};
    ctre::phoenix6::hardware::TalonFX shooterMotorCentre{Ports::Shooter::SHOOTER_MOTOR_CENTRE, Settings::CANBus};
    ctre::phoenix6::hardware::TalonFX shooterMotorRight{Ports::Shooter::SHOOTER_MOTOR_RIGHT, Settings::CANBus};
};

// TODO: follower control initialization

void ShooterImpl::Periodic() {
    // Control
    shooterMotorLeft.SetControl(ctre::phoenix6::controls::DutyCycleOut(getState().getTargetRPM()));

    // Logging
    tkit::Logger& logger = tkit::Logger::GetInstance();
    logger.RecordOutput("Handoff/Velocity", shooterMotorLeft.GetVelocity().GetValue());
    logger.RecordOutput("Handoff/Stator Current", shooterMotorLeft.GetStatorCurrent().GetValue());
    logger.RecordOutput("Handoff/Supply Current", shooterMotorLeft.GetSupplyCurrent().GetValue());

    Shooter::Periodic();
}