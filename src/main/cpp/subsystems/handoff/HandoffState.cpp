#include "handoff/HandoffState.hpp"

const HandoffState HandoffState::IDLE = HandoffState(0.0);
const HandoffState HandoffState::FORWARD = HandoffState(1.0);
const HandoffState HandoffState::REVERSE = HandoffState(-1.0);

double HandoffState::getTargetDutyCycle() const
{
    return targetDutyCycle;
}