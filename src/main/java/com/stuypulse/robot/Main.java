/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * <h2>Main Class</h2>
 * 
 * <p>This is the main class that instantiates the robot code.
 * <p>There is no need to edit this file, and it should not be edited unless you know what you are doing.
 */
public final class Main {

    private Main() {
    }

    /**
     * The main method that starts the robot code. This should not be edited unless you know what you are doing.
     */
    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}
