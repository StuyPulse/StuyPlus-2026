/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.vision;

import com.stuypulse.robot.subsystems.vision.Vision;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetPipeline extends InstantCommand {

    private final Vision vision;

    private int pipeline;

    public SetPipeline(int pipeline) {
        this.pipeline = pipeline;
        this.vision = Vision.getInstance();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void initialize() {
        vision.setPipeline(pipeline);
    }
}
