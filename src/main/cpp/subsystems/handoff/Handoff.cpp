#include "subsystems/handoff/Handoff.hpp"
#include "subsystems/handoff/HandoffState.hpp"
#include "HandoffImpl.cpp"
#include "HandoffSim.cpp"
#include <frc/RobotBase.h>

static Handoff* initInstance()
{
    if (frc::RobotBase::IsReal())
    {
        return new HandoffImpl();
    }
    else
    {
        return new HandoffSim();
    }
}

Handoff* instance = initInstance();
Handoff& Handoff::getInstance() {
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
