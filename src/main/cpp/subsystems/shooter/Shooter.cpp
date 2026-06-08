#include "subsystems/shooter/Shooter.hpp"
#include "subsystems/shooter/ShooterState.hpp"
#include "ShooterImpl.cpp"
#include "ShooterSim.cpp"
#include <frc/RobotBase.h>

Shooter& Shooter::getInstance() {
    static Shooter* instance = frc::RobotBase::IsReal()
        ? static_cast<Shooter*>(new ShooterImpl())
        : static_cast<Shooter*>(new ShooterSim());
    return *instance;
}

Shooter::Shooter() : state(ShooterState::IDLE) {}

void Shooter::setState(ShooterState state)
{
    this -> state = state;
}

ShooterState Shooter::getState() const
{
    return state;
}