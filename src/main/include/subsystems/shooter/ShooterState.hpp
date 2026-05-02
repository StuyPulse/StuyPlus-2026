#pragma once
#include <functional>

struct ShooterState {
    public:
        static const ShooterState IDLE;
        static const ShooterState SHOOT;
        static const ShooterState FERRY;
        static const ShooterState MANUAL_HUB;

        double getTargetRPM() const;
    private:
        std::function<double()> targetRPM;

        ShooterState(std::function<double()> targetRPM) : targetRPM(targetRPM) {}
};