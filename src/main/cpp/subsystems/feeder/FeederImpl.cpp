#include "subsystems/feeder/FeederImpl.hpp"

void FeederImpl::Periodic() {
    // Control
    feederMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut(getState().getTargetDutyCycle()));

    // Logging
    tkit::Logger& logger = tkit::Logger::GetInstance();
    logger.RecordOutput("Feeder/Velocity", feederMotor.GetVelocity().GetValue());
    logger.RecordOutput("Feeder/Stator Current", feederMotor.GetStatorCurrent().GetValue());
    logger.RecordOutput("Feeder/Supply Current", feederMotor.GetSupplyCurrent().GetValue());

    Feeder::Periodic();
}