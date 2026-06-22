#include "subsystems/shooter/ShooterSim.hpp"
#include "subsystems/shooter/Shooter.hpp"
#include <ctre/phoenix6/TalonFX.hpp>
#include "constants/Ports.hpp"
#include "constants/Settings.hpp"
#include <telemetrykit/TelemetryKit.h>
#include "constants/Motors.hpp"

ShooterSim::ShooterSim() {
    Motors::Shooter::SHOOTER_MOTOR_LEFT.configure(shooterMotorLeft);
    Motors::Shooter::SHOOTER_MOTOR_CENTER.configure(shooterMotorCenter);
    Motors::Shooter::SHOOTER_MOTOR_RIGHT.configure(shooterMotorRight);

    shooterMotorLeft.SetControl(Shooter::followerControl);
    shooterMotorCenter.SetControl(Shooter::followerControl);
}
// TODO: follower control initialization

void ShooterSim::Periodic() {
    // Control
    shooterMotorRight.SetControl(ctre::phoenix6::controls::DutyCycleOut(getState().getTargetRPM()));

    // Logging
    tkit::Logger& logger = tkit::Logger::GetInstance();
    logger.RecordOutput("Shooter/Velocity_RPS", shooterMotorRight.GetVelocity().GetValueAsDouble());
    logger.RecordOutput("Shooter/Stator Current_Amps", shooterMotorRight.GetStatorCurrent().GetValueAsDouble());
    logger.RecordOutput("Shooter/Supply Current_Amps", shooterMotorRight.GetSupplyCurrent().GetValueAsDouble());

    Shooter::Periodic();
}