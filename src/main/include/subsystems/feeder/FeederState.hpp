#pragma once

struct FeederState {
    public:
        static const FeederState IDLE;
        static const FeederState FORWARD;
        static const FeederState REVERSE;

        double getTargetDutyCycle() const;

    private:
        double targetDutyCycle;

        FeederState(double targetDutyCycle) : targetDutyCycle(targetDutyCycle) {}
};