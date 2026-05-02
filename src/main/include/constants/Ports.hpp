#pragma once

namespace Ports {
    namespace Feeder {
        constexpr int FEEDER_MOTOR = 15;
    }

    namespace Handoff {
        constexpr int HANDOFF_MOTOR = 50;
    }

    namespace Shooter {
        constexpr int SHOOTER_MOTOR_LEFT = 22;
        constexpr int SHOOTER_MOTOR_CENTRE = 1; // TODO: get
        constexpr int SHOOTER_MOTOR_RIGHT = 2;
    }

    namespace Intake {
        constexpr int INTAKE_PIVOT_MOTOR = 10;
        constexpr int INTAKE_ROLLER_MOTOR_LEFT = 6;
        constexpr int INTAKE_ROLLER_MOTOR_RIGHT = 7;
    }
}