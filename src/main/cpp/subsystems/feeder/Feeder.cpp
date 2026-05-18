#include "subsystems/feeder/Feeder.hpp"
#include <frc/RobotBase.h>
#include "FeederImpl.cpp"
#include "FeederSim.cpp"
#include "FeederState.cpp"

Feeder& Feeder::getInstance() {
    static std::unique_ptr<Feeder> instance = frc::RobotBase::IsReal()
        ? std::unique_ptr<Feeder>(new FeederImpl())
        : std::unique_ptr<Feeder>(new FeederSim());
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