#include "subsystems/shooter/Shooter.hpp"
#include "subsystems/shooter/ShooterState.hpp"
#include "ShooterImpl.cpp"
#include "ShooterSim.cpp"
#include <frc/RobotBase.h>

static Shooter* initInstance()
{
    if (frc::RobotBase::IsReal())
    {
        return new ShooterImpl();
    }
    else
    {
        return new ShooterSim();
    }
}

Shooter* instance = initInstance();
Shooter& Shooter::getInstance() {
    return *instance;
}

Shooter::Shooter() : state(ShooterState::IDLE) {}

void Shooter::setState(ShooterState state)
{
    this -> state = state;
}

ShooterState Shooter::getState()
{
    return state;
}
