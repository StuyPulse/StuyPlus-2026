#include "subsystems/intake/Intake.hpp"
#include "subsystems/intake/IntakeState.hpp"
#include "subsystems/intake/IntakeImpl.hpp"
#include "subsystems/intake/IntakeSim.hpp"
#include <frc/RobotBase.h>
#include <units/math.h>
#include "constants/Settings.hpp"

Intake& Intake::getInstance() {
    static Intake* instance = frc::RobotBase::IsReal()
        ? static_cast<Intake*>(new IntakeImpl())
        : static_cast<Intake*>(new IntakeSim());
    return *instance;
}

Intake::Intake() 
    : state(IntakeState::IDLE), 
    rollerController(getState().getTargetDutyCycle()), 
    positionController(getState().getTargetAngle()) 
{}

IntakeState Intake::getState() const
{
    return state;
}


void Intake::setState(IntakeState state) {
    this -> state = state;
}

bool Intake::atTargetAngle() {
    const units::degree_t distanceFromTarget = units::math::abs(this->getRelativePosition() - getState().getTargetAngle());
    return distanceFromTarget < Settings::Intake::Pivot::ANGLE_TOLERANCE;
}

void Intake::Periodic() { // todo: mirror java periodic
}