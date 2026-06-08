#include "Handoff.hpp"
#include <ctre/phoenix6/TalonFX.hpp>
#include "constants/Ports.hpp"
#include "constants/Settings.hpp"

class HandoffImpl : public Handoff
{
public:
    void Periodic() override {}

    void StopMotors() override {
        handoffMotor.StopMotor();
    }

private:
    ctre::phoenix6::hardware::TalonFX handoffMotor{Ports::Handoff::HANDOFF_MOTOR, Settings::CANBus};
};