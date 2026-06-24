#include "subsystems/intake/IntakeState.hpp"
#include <units/angle.h>
#include "constants/Settings.hpp"

const IntakeState IntakeState::IDLE = IntakeState(Settings::Intake::Pivot::STOW_ANGLE, Settings::Intake::Roller::IDLE_DUTY_CYCLE);
const IntakeState IntakeState::DOWN = IntakeState(Settings::Intake::Pivot::DEPLOY_ANGLE, Settings::Intake::Roller::IDLE_DUTY_CYCLE);
const IntakeState IntakeState::INTAKE = IntakeState(Settings::Intake::Pivot::DEPLOY_ANGLE, Settings::Intake::Roller::INTAKE_DUTY_CYCLE);
const IntakeState IntakeState::OUTTAKE = IntakeState(Settings::Intake::Pivot::DEPLOY_ANGLE, Settings::Intake::Roller::OUTTAKE_DUTY_CYCLE);
const IntakeState IntakeState::AGITATE_UP = IntakeState(Settings::Intake::Pivot::AGITATE_UP_ANGLE, Settings::Intake::Roller::INTAKE_DUTY_CYCLE);
const IntakeState IntakeState::AGITATE_DOWN = IntakeState(Settings::Intake::Pivot::AGITATE_DOWN_ANGLE, Settings::Intake::Roller::INTAKE_DUTY_CYCLE);
const IntakeState IntakeState::HOMING_DOWN = IntakeState(Settings::Intake::Pivot::DEPLOY_ANGLE, Settings::Intake::Roller::IDLE_DUTY_CYCLE);

double IntakeState::getTargetDutyCycle() const {
    return targetDutyCycle;
}

units::degree_t IntakeState::getTargetAngle() const {
    return targetAngle;
}