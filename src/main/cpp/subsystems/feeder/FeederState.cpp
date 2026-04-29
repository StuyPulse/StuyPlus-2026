#include "subsystems/feeder/FeederState.hpp"

const FeederState FeederState::IDLE = FeederState(0.0);
const FeederState FeederState::FORWARD = FeederState(1.0);
const FeederState FeederState::REVERSE = FeederState(-1.0);

double FeederState::getTargetDutyCycle() const
{
    return targetDutyCycle;
}