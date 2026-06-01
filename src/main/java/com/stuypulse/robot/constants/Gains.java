/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.constants;

import static edu.wpi.first.units.Units.Amps;

import com.pathplanner.lib.config.PIDConstants;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.measure.Current;

public class Gains {

    public interface Shooter {

        DoubleSubscriber kP = DogLog.tunable("Shooter/kP", 13.0);

        DoubleSubscriber kI = DogLog.tunable("Shooter/kI", 0.0);

        DoubleSubscriber kD = DogLog.tunable("Shooter/kD", 0.0);

        DoubleSubscriber kS = DogLog.tunable("Shooter/kS", 2.5);

        DoubleSubscriber kV = DogLog.tunable("Shooter/kV", 0.05);

        DoubleSubscriber kA = DogLog.tunable("Shooter/kA", 0.0);
    }

    public interface Intake {

        // pivot gains
        double kP = 300;//300

        double kI = 0;

        double kD = 75;

        Current kS = Amps.of(0);

        Current kV = Amps.of(0);

        Current kA = Amps.of(0);

        Current kG = Amps.of(-12);

        public interface Digestion {

            double kP = 325;

            double kI = 0;

            // TODO: tune
            double kD = 75;
        }
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
            double akP = 8.8624;

            double akI = 0.0;

            double akD = 0.0;

            PIDConstants XY = new PIDConstants(10, 0.0, 0.0);

            PIDConstants THETA = new PIDConstants(10, 0.0, 0.0);
        }
    }
}
