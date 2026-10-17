/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.Intake;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class IntakeResetLoadCount extends InstantCommand {
    private Intake intake;

    public IntakeResetLoadCount() {
        this.intake = Intake.getInstance();
        // addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.resetLoadCounter();
    }
}
