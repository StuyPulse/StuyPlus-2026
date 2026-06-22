#pragma once

#include <frc2/command/SubsystemBase.h>
#include <units/angular_velocity.h>
#include "ShooterState.hpp"
#include <ctre/phoenix6/controls/Follower.hpp>
#include "constants/Ports.hpp"
#include <ctre/phoenix6/signals/SpnEnums.hpp>

class Shooter : public frc2::SubsystemBase {
    public: 
        static Shooter& getInstance();

        ShooterState getState() const;
        void setState(ShooterState state);

        virtual units::angular_velocity::revolutions_per_minute_t getCurrentAngularVelocity() = 0;

        void Periodic() override;

    protected:
        Shooter();

        virtual void StopMotors() = 0;
        const ctre::phoenix6::controls::Follower followerControl{Ports::Shooter::SHOOTER_MOTOR_RIGHT, ctre::phoenix6::signals::MotorAlignmentValue::Opposed};

    private:
        ShooterState state;
};