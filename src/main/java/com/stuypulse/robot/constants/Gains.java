package com.stuypulse.robot.constants;

import com.pathplanner.lib.config.PIDConstants;
import com.stuypulse.stuylib.network.SmartNumber;

public class Gains {
    public interface Shooter {
        double kP = 0.9;
        double kI = 0;
        double kD = 0;

        double kS = 0;
        double kV = 0;
        double kA = 0;
    }

    // public interface Feeder {
    //     double kP = 0.1;
    //     double kI = 0.01;
    //     double kD = 0;

    //     double kS = 0.0;
    //     SmartNumber kV = new SmartNumber("Feeder/kV", 5.0);
    //     SmartNumber kA = new SmartNumber("Feeder/kA", 0.5);
    //     double kG = 0.0;
    // }

    public interface Intake {
        SmartNumber kP = new SmartNumber("Intake/Pivot kP", 15);
        SmartNumber kI = new SmartNumber("Intake/Pivot kI", 0);
        SmartNumber kD = new SmartNumber("Intake/Pivot kD", 0);

        SmartNumber kV = new SmartNumber("Intake/Pivot kV", 0);
    }

    public interface Swerve {
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
            double kP = 1.0;
            double kI = 0.0;
            double kD = 0.0;
            double akP = 0.88624;
            double akI = 0.0;
            double akD = 0.0;

            double akS = 0.33753;
            double akV = 0.97475;
            double akA = 0.095509;

            PIDConstants XY = new PIDConstants(10, 0.0, 0.0);
            PIDConstants THETA = new PIDConstants(10, 0.0, 0.0);
        }
    }
}