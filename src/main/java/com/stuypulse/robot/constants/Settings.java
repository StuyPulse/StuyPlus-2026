/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

//import com.stuypulse.stuylib.network.SmartBoolean;
//import com.stuypulse.stuylib.network.SmartNumber;

//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {
    double DT = 0.020;
    boolean DEBUG_MODE = true;

public interface Intake {
    double ARM_LENGTH = 1.0; // CHANGE
    double INTAKE_VOLTAGE = 1.0;
    double INTAKE_ANGLE = 67;
    double OUTTAKE_VOLTAGE = -1.0;
    double OUTTAKE_ANGLE = 90;
    double IDLE_ANGLE = 67;
    double IDLE_VOLTAGE = 0;
    double ROLLER_MAX_ACCEL = 0;
    double ROLLER_MAX_VEL = 0;
    double INITIAL_POSITION = 0;
    double AGITATE_ANGLE = 0;
    double JKgMetersSquared = 0.001;
    double PIVOT_MIN_ANGLE = 0.0;
    double PIVOT_MAX_ANGLE = Math.PI / 2;
    double GEAR_RATIO = 20.0;

    double ANGLE_TOLERANCE = 0.5; // degrees    

    double kP = 0;
    double kI = 0;
    double kD = 0;
    double DT = 0.020;

    public interface Feeder {
        double FEEDER_REVERSE = -41;
        double FEEDER_FORWARD = 41;

        double kP = 1.0;
        double kI = 1.0;
        double kD = 1.0;
    }

    public interface EnabledSubsystems {
        SmartBoolean LED = new SmartBoolean("Enabled Subsystems/LED", true);
    }

    public interface LED {
        int LED_LENGTH = 68; // TODO: ask Plus-ME for LED Length

        LEDPattern shooterShooting = LEDPattern.solid(Color.kOrange);
        LEDPattern feederForward= LEDPattern.solid(Color.kBlue);
        LEDPattern feederReverse = LEDPattern.solid(Color.kRed);
        LEDPattern ferrying = LEDPattern.solid(Color.kPurple);
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
