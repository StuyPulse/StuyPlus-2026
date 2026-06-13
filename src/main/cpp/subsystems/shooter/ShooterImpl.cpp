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
    logger.RecordOutput("Handoff/Velocity_RPS", shooterMotorLeft.GetVelocity().GetValueAsDouble());
    logger.RecordOutput("Handoff/Stator Current_Amps", shooterMotorLeft.GetStatorCurrent().GetValueAsDouble());
    logger.RecordOutput("Handoff/Supply Current_Amps", shooterMotorLeft.GetSupplyCurrent().GetValueAsDouble());

    Shooter::Periodic();
}