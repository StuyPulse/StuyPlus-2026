#include "subsystems/feeder/Feeder.hpp"
#include <frc/RobotBase.h>
#include "FeederImpl.cpp"
#include "FeederSim.cpp"
#include "FeederState.cpp"

Feeder& Feeder::getInstance() {
    static Feeder* instance = frc::RobotBase::IsReal()
        ? static_cast<Feeder*>(new FeederImpl())
        : static_cast<Feeder*>(new FeederSim());
    return *instance;
}

Feeder::Feeder() : state(FeederState::IDLE) {}

void Feeder::setState(FeederState newState) {
    state = newState;
}

FeederState Feeder::getState() const {
    return state;
}

void Feeder::Periodic() {
}