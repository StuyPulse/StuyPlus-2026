#include "handoff/Handoff.hpp"
#include <ctre/phoenix6/TalonFX.hpp>
#include "constants/Ports.hpp"
#include "constants/Settings.hpp"
#include <telemetrykit/TelemetryKit.h>

class HandoffImpl : public Handoff
{
public:
    void Periodic() override {}

private:
    ctre::phoenix6::hardware::TalonFX handoffMotor{Ports::Handoff::HANDOFF_MOTOR, Settings::CANBus};
};

void HandoffImpl::Periodic() {
    // Control
    handoffMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut(getState().getTargetDutyCycle()));

    // Logging
    tkit::Logger& logger = tkit::Logger::GetInstance();
    logger.RecordOutput("Handoff/Velocity", handoffMotor.GetVelocity().GetValue());
    logger.RecordOutput("Handoff/Stator Current", handoffMotor.GetStatorCurrent().GetValue());
    logger.RecordOutput("Handoff/Supply Current", handoffMotor.GetSupplyCurrent().GetValue());

    Handoff::Periodic();
}