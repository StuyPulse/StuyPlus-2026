/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

/** This file contains the different ports of motors, solenoids and sensors */
public interface Ports {
    public interface Gamepad {
        int DRIVER = 0; // TODO: ask Plus-ME for the port
        int OPERATOR = 1; // TODO: ask Plus-ME for the port
        int DEBUGGER = 2; // TODO: ask Plus-ME for the port
    }

    // public interface LED {
    //     int PORT = 2; // TODO: ask Plus-ME for the port
    // }

    // public interface ShooterPorts { // Temporary until the big mec-e people give us the ports
    //     int SHOOTER_MOTOR_1 = 0;
    //     int SHOOTER_MOTOR_2 = 1;
    //     int SHOOTER_MOTOR_3 = 2;

    //     int BOTTOM_MOTOR_1 = 3;
    //     int BOTTOM_MOTOR_2 = 4;
    // }

//TODO: Get ports from mech
    // public interface Feeder {
    //     int FEEDER_MOTOR_1 = 3;
    //     int FEEDER_MOTOR_2 = 4;
    // }

    public interface Intake {
        int MOTOR_INTAKE_ROLLER_LEFT = 6;
        int MOTOR_INTAKE_ROLLER_RIGHT = 7;
        int MOTOR_INTAKE_PIVOT = 10; //Reminder: Config numbers
    }
    
}