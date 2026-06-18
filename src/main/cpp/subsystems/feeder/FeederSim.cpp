#include "subsystems/feeder/FeederSim.hpp"
#include "subsystems/feeder/Feeder.hpp"
#include "constants/Motors.hpp"

FeederSim::FeederSim() {
    Motors::Feeder::LEADER_CONFIG.configure(feederMotor);
}

void FeederSim::Periodic() {
    // Control
    feederMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut(getState().getTargetDutyCycle()));

    // Logging
    tkit::Logger& logger = tkit::Logger::GetInstance();
    logger.RecordOutput("Feeder/Velocity_RPS", feederMotor.GetVelocity().GetValueAsDouble());
    logger.RecordOutput("Feeder/Stator Current_Amps", feederMotor.GetStatorCurrent().GetValueAsDouble());
    logger.RecordOutput("Feeder/Supply Current_Amps", feederMotor.GetSupplyCurrent().GetValueAsDouble());

    Feeder::Periodic();
}