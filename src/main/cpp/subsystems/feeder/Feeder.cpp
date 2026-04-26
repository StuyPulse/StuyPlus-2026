#include "subsystems/feeder/Feeder.hpp"
#include "subsystems/feeder/FeederState.hpp"
#include "FeederImpl.cpp"
#include "FeederSim.cpp"
#include <frc/RobotBase.h>

static Feeder* initInstance()
{
    if (frc::RobotBase::IsReal())
    {
        return new FeederImpl();
    }
    else
    {
        return new FeederSim();
    }
}

Feeder* instance = initInstance();
Feeder& Feeder::getInstance() {
    return *instance;
}

Feeder::Feeder() : state(FeederState::IDLE) {}

void Feeder::setState(FeederState state)
{
    this -> state = state;
}

FeederState Feeder::getState()
{
    return state;
}
