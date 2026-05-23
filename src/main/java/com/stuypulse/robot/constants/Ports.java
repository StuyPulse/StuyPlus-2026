/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.constants;

/** This file contains the different ports of motors, solenoids and sensors */
public interface Ports {

    public interface Gamepad {

        int DRIVER = 0;

        int OPERATOR = 1;

        int DEBUGGER = 2;
    }

    public interface LED {

        // TODO: Get actual port
        int LED_PWM_PORT = 0;
    }

    // public interface LED {
    // int PORT = 2;
    // }
    public interface Shooter {

        int SHOOTER_MOTOR_LEFT = 30;

        // TODO: get after champs
        int SHOOTER_MOTOR_CENTER = 54;

        int SHOOTER_MOTOR_RIGHT = 47;
    }

    public interface Handoff {

        int HANDOFF_MOTOR = 50;
    }

    // TODO: Get ports from mech
    public interface Feeder {

        int FEEDER_MOTOR = 15;
    }

    public interface Intake {

        int PIVOT_LIMIT_SWITCH = 0;

        int INTAKE_ROLLER_MOTOR_LEFT = 22;

        int INTAKE_ROLLER_MOTOR_RIGHT = 17;

        int INTAKE_PIVOT_MOTOR = 10;
    }
}
