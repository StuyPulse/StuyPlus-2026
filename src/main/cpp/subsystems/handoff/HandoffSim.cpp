#include "subsystems/handoff/HandoffSim.hpp"
#include "subsystems/handoff/Handoff.hpp"

void HandoffSim::Periodic() {
    // Control
    handoffMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut(getState().getTargetDutyCycle()));

    // Logging
    tkit::Logger& logger = tkit::Logger::GetInstance();
    logger.RecordOutput("Handoff/Velocity_RPS", handoffMotor.GetVelocity().GetValueAsDouble());
    logger.RecordOutput("Handoff/Stator Current_Amps", handoffMotor.GetStatorCurrent().GetValueAsDouble());
    logger.RecordOutput("Handoff/Supply Current_Amps", handoffMotor.GetSupplyCurrent().GetValueAsDouble());

    Handoff::Periodic();
}