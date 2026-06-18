#include "subsystems/feeder/Feeder.hpp"
#include <frc/RobotBase.h>
#include "subsystems/feeder/FeederImpl.hpp"
#include "subsystems/feeder/FeederSim.hpp"
#include "subsystems/feeder/FeederState.hpp"
#include "constants/Motors.hpp"

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