#include <pathplanner/lib/config/PIDConstants.h>
#include <units/current.h>

namespace Gains {

    namespace Shooter {
        constexpr double kP = 15.0;

        constexpr double kI = 0.0;

        constexpr double kD = 0.0;

        constexpr double kS = 2.5;

        constexpr double kV = 0.05;

        constexpr double kA = 0.0;

        namespace FirstShot {
            constexpr double kP = 20;
            constexpr double kI = 0;
            constexpr double kD = 0;
        }
    }

    namespace Intake {
        constexpr double kP = 300; // 300

        constexpr double kI = 0;

        constexpr double kD = 75;

        constexpr units::current::ampere_t kS = 0_A;

        constexpr units::current::ampere_t kV = 0_A;

        constexpr units::current::ampere_t kA = 0_A;

        constexpr units::current::ampere_t kG = -12_A;

        namespace Digestion {

            constexpr double kP = 325;

            constexpr double kI = 0;

            // TODO: tune
            constexpr double kD = 75;
        }
    }

    namespace Swerve {

        namespace Drive {

            constexpr double kS = 0.0;

            constexpr double kV = 0.124;

            constexpr double kA = 0.0;

            constexpr double kP = 0.1;

            constexpr double kI = 0.0;

            constexpr double kD = 0.0;
        }

        namespace Turn {

            constexpr double kS = 0.1;

            constexpr double kV = 2.66;

            constexpr double kA = 0.0;

            constexpr double kP = 100.0;

            constexpr double kI = 0.0;

            constexpr double kD = 0.5;
        }

        namespace Alignment {
            constexpr double akP = 8.8624;

            constexpr double akI = 0.0;

            constexpr double akD = 0.0;

            constexpr pathplanner::PIDConstants XY = pathplanner::PIDConstants(10, 0.0, 0.0);

            constexpr pathplanner::PIDConstants THETA = pathplanner::PIDConstants(10, 0.0, 0.0);
        }
    }
}