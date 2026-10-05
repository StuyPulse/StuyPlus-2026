/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.vision;

import com.stuypulse.robot.subsystems.vision.Vision;
import com.stuypulse.robot.subsystems.vision.VisionIO.MegaTagMode;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetMegaTagMode extends InstantCommand {

    private final Vision vision;

    private final MegaTagMode mode;

    public SetMegaTagMode(MegaTagMode mode) {
        this.vision = Vision.getInstance();
        this.mode = mode;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void initialize() {
        vision.setMegaTagMode(mode);
    }
}
