#pragma once

#include <frc2/command/SubsystemBase.h>
#include "ShooterState.hpp"

class Shooter : public frc2::SubsystemBase {
    public:
        static Shooter& getInstance();

        ShooterState getState();
        void setState(ShooterState state);
        virtual void Periodic();

    protected:
        Shooter();

    private:
        static Shooter* instance;
        ShooterState state;
};