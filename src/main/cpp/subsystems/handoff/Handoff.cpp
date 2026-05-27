#include "subsystems/handoff/Handoff.hpp"
#include "subsystems/handoff/HandoffState.hpp"
#include "HandoffImpl.cpp"
#include "HandoffSim.cpp"
#include <frc/RobotBase.h>

Handoff& Handoff::getInstance() {
    static Handoff* instance = frc::RobotBase::IsReal()
        ? static_cast<Handoff*>(new HandoffImpl())
        : static_cast<Handoff*>(new HandoffSim());
    return *instance;
}

Handoff::Handoff() : state(HandoffState::IDLE) {}

void Handoff::setState(HandoffState state)
{
    this -> state = state;
}

HandoffState Handoff::getState()
{
    return state;
}
