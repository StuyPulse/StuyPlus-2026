#include "subsystems/shooter/ShooterImpl.hpp"
#include <ctre/phoenix6/TalonFX.hpp>
#include "constants/Ports.hpp"
#include "constants/Settings.hpp"
#include <telemetrykit/TelemetryKit.h>

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