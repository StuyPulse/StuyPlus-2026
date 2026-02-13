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

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {
    double dT = 0.020;
    boolean DEBUG_MODE = true;

public interface Intake{
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
    double PIVOT_MAX_ANGLE = 90.0;
    double GEAR_RATIO = 20.0;
    }
}
