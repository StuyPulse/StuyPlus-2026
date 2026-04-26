#pragma once

#include <frc2/command/SubsystemBase.h>
#include "HandoffState.hpp"

class Handoff : public frc2::SubsystemBase {
    public:
        static Handoff& getInstance();

        HandoffState getState();
        void setState(HandoffState state);
        virtual void Periodic();

    protected:
        Handoff();

    private:
        static Handoff* instance;
        HandoffState state;
};