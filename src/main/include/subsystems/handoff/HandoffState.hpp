#pragma once

struct HandoffState {
    public:
        static const HandoffState IDLE;
        static const HandoffState FORWARD;
        static const HandoffState REVERSE;

        double getTargetDutyCycle() const;

    private:
        double targetDutyCycle;

        HandoffState(double targetDutyCycle) : targetDutyCycle(targetDutyCycle) {}
};