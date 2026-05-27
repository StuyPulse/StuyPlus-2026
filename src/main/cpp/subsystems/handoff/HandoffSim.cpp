#include "subsystems/handoff/HandoffSim.hpp"

void HandoffSim::Periodic() {
    // Control
    handoffMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut(getState().getTargetDutyCycle()));

    // Logging
    tkit::Logger& logger = tkit::Logger::GetInstance();
    logger.RecordOutput("Handoff/Velocity", handoffMotor.GetVelocity().GetValue());
    logger.RecordOutput("Handoff/Stator Current", handoffMotor.GetStatorCurrent().GetValue());
    logger.RecordOutput("Handoff/Supply Current", handoffMotor.GetSupplyCurrent().GetValue());

    Handoff::Periodic();
}