#pragma once

#include <frc2/command/SubsystemBase.h>
#include <units/angular_velocity.h>
#include "FeederState.hpp"

class Feeder : public frc2::SubsystemBase
{
public:
    static Feeder &getInstance();

    void setState(FeederState state);
    FeederState getState() const;

    virtual units::angular_velocity::revolutions_per_minute_t getCurrentAngularVelocity() const = 0;

    void Periodic() override;

protected:
    Feeder();

    virtual void StopMotors() = 0;

private:
    FeederState state;
};