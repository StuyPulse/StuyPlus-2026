package com.stuypulse.robot.constants;

public class Gains {
    public interface Intake {
        public interface Roller {
            double kP = 1.5;
            double kI = 0.0;
            double kD = 1.0;
        }
        public interface Pivot {
            double kP = 1.0; 
            double kI = 0.0;
            double kD = 0.0;
        }
    }
}
