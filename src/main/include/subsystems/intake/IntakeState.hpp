#pragma once

#include <units/angle.h>

struct IntakeState {
    public:
        static const IntakeState IDLE;
        static const IntakeState DOWN;
        static const IntakeState INTAKE;
        static const IntakeState OUTTAKE;
        static const IntakeState AGITATE_UP;
        static const IntakeState AGITATE_DOWN;
        static const IntakeState HOMING_DOWN;

        double getTargetDutyCycle() const;
        units::degree_t getTargetAngle() const;
    private:
        double targetDutyCycle;
        units::degree_t targetAngle;

        IntakeState(double targetDutyCycle, units::degree_t targetAngle) : targetDutyCycle(targetDutyCycle), targetAngle(targetAngle) {}
};