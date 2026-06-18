#include "subsystems/handoff/HandoffImpl.hpp"
#include <ctre/phoenix6/TalonFX.hpp>
#include "constants/Ports.hpp"
#include "constants/Settings.hpp"
#include <telemetrykit/TelemetryKit.h>
#include "constants/Motors.hpp"

HandoffImpl::HandoffImpl() {
    Motors::Handoff::HANDOFF_MOTOR_CONFIG.configure(handoffMotor);
}

void HandoffImpl::Periodic() {
    // Control
    handoffMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut(getState().getTargetDutyCycle()));

    // Logging
    tkit::Logger& logger = tkit::Logger::GetInstance();
    logger.RecordOutput("Handoff/Velocity_RPS", handoffMotor.GetVelocity().GetValueAsDouble());
    logger.RecordOutput("Handoff/Stator Current_Amps", handoffMotor.GetStatorCurrent().GetValueAsDouble());
    logger.RecordOutput("Handoff/Supply Current_Amps", handoffMotor.GetSupplyCurrent().GetValueAsDouble());

    Handoff::Periodic();
}