#pragma once

#include <frc2/command/SubsystemBase.h>
#include "FeederState.hpp"

class Feeder : public frc2::SubsystemBase {
    public:
        static Feeder& getInstance();

        FeederState getState();
        void setState(FeederState state);
        virtual void Periodic();

    protected:
        Feeder();

    private:
        static Feeder* instance;
        FeederState state;
};