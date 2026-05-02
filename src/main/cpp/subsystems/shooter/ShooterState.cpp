#include "subsystems/shooter/ShooterState.hpp"
#include <functional>

const ShooterState ShooterState::IDLE = ShooterState([]() -> double { return 0.0; });
const ShooterState ShooterState::SHOOT = ShooterState([]() -> double { return 100.0; });
const ShooterState ShooterState::FERRY = ShooterState([]() -> double { return 150.0; });
const ShooterState ShooterState::MANUAL_HUB = ShooterState([]() -> double { return 75.0; });

double ShooterState::getTargetRPM() const
{
    return targetRPM();
}