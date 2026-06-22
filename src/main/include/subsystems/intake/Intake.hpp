#pragma once

#include <frc2/command/SubsystemBase.h>
#include "IntakeState.hpp"
#include <units/angle.h>
#include <ctre/phoenix6/controls/Follower.hpp>
#include <ctre/phoenix6/controls/DutyCycleOut.hpp>
#include <ctre/phoenix6/controls/PositionTorqueCurrentFOC.hpp>
#include <ctre/phoenix6/controls/VoltageOut.hpp>
#include <ctre/phoenix6/controls/TorqueCurrentFOC.hpp>
#include "constants/Ports.hpp"
#include "constants/Settings.hpp"

class Intake : public frc2::SubsystemBase {
    public:
        static Intake& getInstance();

        IntakeState getState() const;
        void setState(IntakeState state);

        virtual units::degree_t getRelativePosition() const = 0;
        virtual void seedPivotAngle(units::degree_t angle);

        void Periodic() override;

    private:
        IntakeState state = IntakeState::IDLE;

    protected:
        // rollers
        const ctre::phoenix6::controls::Follower rollerFollowerControl{Ports::Intake::INTAKE_ROLLER_MOTOR_LEFT, ctre::phoenix6::signals::MotorAlignmentValue::Opposed};
        ctre::phoenix6::controls::DutyCycleOut rollerController;
        
        // pivot
        ctre::phoenix6::controls::PositionTorqueCurrentFOC positionController;
        const ctre::phoenix6::controls::VoltageOut homingController{Settings::Intake::Pivot::HOMING_DOWN_VOLTAGE};
        const ctre::phoenix6::controls::TorqueCurrentFOC pushdownController{Settings::Intake::Pivot::PUSHDOWN_CURRENT};
};