/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {
    double DT = 0.020;

    public interface Feeder {
        double FEEDER_REVERSE = -3000;
        double FEEDER_FORWARD = 3000;
    }

    public interface EnabledSubsystems {
        SmartBoolean LED = new SmartBoolean("Enabled Subsystems/LED", true);
    }

    public interface LED {
        int LED_LENGTH = 68; // TODO: ask Plus-ME for LED Length
    }

    public interface Shooter {
        double BOTTOM_MOTOR_RPM = 3000;

        double kP = 0;
        double kI = 0;
        double kD = 0;

        double kS = 0;
        double kV = 0;
        double kA = 0;
        double kG = 0;
    }
}
