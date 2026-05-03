/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.vision;

import com.stuypulse.robot.subsystems.vision.LimelightVision;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetIMUMode extends InstantCommand{
    private final LimelightVision vision;
    private final int mode;

    public SetIMUMode(int mode) {
        this.vision = LimelightVision.getInstance();
        this.mode = mode;
    }

    @Override
    public void initialize() {
        vision.setIMUMode(mode);
    }
}
