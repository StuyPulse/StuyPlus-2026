/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class IntakeAgitateOnce extends SequentialCommandGroup {
    public IntakeAgitateOnce() {
        addCommands(
            new IntakeSetDown(),
            new WaitCommand(0.25),
            new IntakeSetAgitateUp(),
            new WaitCommand(0.25),
            new IntakeSetDown()
        );
    }
}