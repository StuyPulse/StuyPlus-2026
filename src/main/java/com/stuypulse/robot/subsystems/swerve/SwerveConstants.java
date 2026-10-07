package com.stuypulse.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.math.util.Units;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public interface SwerveConstants {
    public interface SwerveSettings {

        double MODULE_VELOCITY_DEADBAND_M_PER_S = 0.1;

        double ROTATIONAL_DEADBAND_RAD_PER_S = 0.1;

        public interface Constraints {

            double MAX_VELOCITY_M_PER_S = 4.3;

            // TODO: revert to 15.0
            double MAX_ACCEL_M_PER_S_SQUARED = 20.0;

            double MAX_ANGULAR_VEL_RAD_PER_S = Units.degreesToRadians(400.0);

            // TODO: revert to 900
            double MAX_ANGULAR_ACCEL_RAD_PER_S = Units.degreesToRadians(300.0);

            PathConstraints DEFAULT_CONSTRAINTS = new PathConstraints(MAX_VELOCITY_M_PER_S, MAX_ACCEL_M_PER_S_SQUARED,
                    MAX_ANGULAR_VEL_RAD_PER_S, MAX_ANGULAR_ACCEL_RAD_PER_S);
        }

        public interface Alignment {

            public interface Constraints {

                double DEFAULT_MAX_VELOCITY = 4.3;

                double DEFAULT_MAX_ACCELERATION = 15.0;

                double DEFAULT_MAX_ANGULAR_VELOCITY = Units.degreesToRadians(400.0);

                double DEFAULT_MAX_ANGULAR_ACCELERATION = Units.degreesToRadians(900.0);
            }

            public interface Tolerances {

                Distance X_TOLERANCE = Inches.of(2.0);

                Distance Y_TOLERANCE = Inches.of(2.0);

                Rotation2d THETA_TOLERANCE = Rotation2d.fromDegrees(8);

                Pose2d POSE_TOLERANCE = new Pose2d(X_TOLERANCE.in(Meters), Y_TOLERANCE.in(Meters), THETA_TOLERANCE);

                LinearVelocity MAX_VELOCITY_WHEN_ALIGNED = MetersPerSecond.of(0.15);

                Time ALIGNMENT_DEBOUNCE = Seconds.of(0.15);
            }

            public interface Targets {

                // TODO: Get actual angle
                Rotation2d HUB_LEFT_CORNER = Rotation2d.fromDegrees(45);

                Rotation2d HUB_RIGHT_CORNER = Rotation2d.fromDegrees(-45);
            }
        }
    }
    public interface SwerveGains {

        public interface Drive {

            double kS = 0.0;

            double kV = 0.124;

            double kA = 0.0;

            double kP = 0.1;

            double kI = 0.0;

            double kD = 0.0;
        }

        public interface Turn {

            double kS = 0.1;

            double kV = 2.66;

            double kA = 0.0;

            double kP = 100.0;

            double kI = 0.0;

            double kD = 0.5;
        }

        public interface Alignment {
            double akP = 8.8624;

            double akI = 0.0;

            double akD = 0.0;

            PIDConstants XY = new PIDConstants(10, 0.0, 0.0);

            PIDConstants THETA = new PIDConstants(10, 0.0, 0.0);
        }
    }
}
