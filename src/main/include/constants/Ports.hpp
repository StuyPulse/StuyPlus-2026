#pragma once

namespace Ports {
    namespace Feeder {
        const int FEEDER_MOTOR = 15;
    }

    namespace Handoff {
        const int HANDOFF_MOTOR = 50;
    }

    namespace Shooter {
        const int SHOOTER_MOTOR_RIGHT = 2;
        const int SHOOTER_MOTOR_CENTRE = 1; // TODO: get
        const int SHOOTER_MOTOR_LEFT = 22;
    }

    namespace Intake {
        const int INTAKE_PIVOT_MOTOR = 10;
        const int INTAKE_ROLLER_MOTOR_LEFT = 6;
        const int INTAKE_ROLLER_MOTOR_RIGHT = 7;
    }
}