/**
 * ********************** PROJECT RON ************************
 */
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/**
 * ***********************************************************
 */
package com.stuypulse.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Main {

    private Main() {
    }

    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}
