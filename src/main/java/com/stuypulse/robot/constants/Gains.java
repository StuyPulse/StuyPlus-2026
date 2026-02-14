package com.stuypulse.robot.constants;

import com.pathplanner.lib.config.PIDConstants;

public class Gains {
    public interface Shooter {
        double kP = 0;
        double kI = 0;
        double kD = 0;

        double kS = 0;
        double kV = 0;
        double kA = 0;
        double kG = 0;
    }

    public interface Intake {
        double kP = 1.0; 
        double kI = 0.0;
        double kD = 0.0;
    }

    public interface Swerve {
        double kP = 0;
        double kI = 0;
        double kD = 0;

        double kS = 0;
        double kV = 0;
        double kA = 0;
        double kG = 0;

        public interface Alignment {
            PIDConstants XY = new PIDConstants(0, 0, 0);
            PIDConstants THETA = new PIDConstants(0, 0, 0);
        }

        public interface Drive {
            double kP = 0;
            double kI = 0;
            double kD = 0;

            double kS = 0;
            double kV = 0;
            double kA = 0;
            double kG = 0;
        }
        public interface Turn {
            double kP = 0;
            double kI = 0;
            double kD = 0;

            double kS = 0;
            double kV = 0;
            double kA = 0;
            double kG = 0;
        }
    }
}
