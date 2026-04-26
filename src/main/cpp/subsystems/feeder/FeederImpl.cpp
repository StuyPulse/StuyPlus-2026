#include "subsystems/feeder/Feeder.hpp"
#include <ctre/phoenix6/TalonFX.hpp>
#include "constants/Ports.hpp"
#include "constants/Settings.hpp"
#include <telemetrykit/TelemetryKit.h>

class FeederImpl : public Feeder
{
public:
    void Periodic() override {}

private:
    ctre::phoenix6::hardware::TalonFX feederMotor{Ports::Feeder::FEEDER_MOTOR, Settings::CANBus};
};

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