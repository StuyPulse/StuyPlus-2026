/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

/** This file contains the different ports of motors, solenoids and sensors */
public interface Ports {
    public interface Gamepad {
        int DRIVER = 0;
        int OPERATOR = 1;
        int DEBUGGER = 2;
    }

    // public interface LED {
    //     int PORT = 2;
    // }

    public interface Shooter {
        int SHOOTER_MOTOR_LEFT = 22;
        int SHOOTER_MOTOR_CENTER = 1; // TODO: get after champs
        int SHOOTER_MOTOR_RIGHT = 2;
    }

    public interface Handoff {
        int HANDOFF_MOTOR = 50;
    }

//TODO: Get ports from mech
    public interface Feeder {
        int FEEDER_MOTOR = 15;
    }

    public interface Intake {
        int MOTOR_INTAKE_ROLLER_LEFT = 6;
        int MOTOR_INTAKE_ROLLER_RIGHT = 7;
        int MOTOR_INTAKE_PIVOT = 10;
    }
    
}