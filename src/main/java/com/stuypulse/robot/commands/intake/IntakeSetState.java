/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.intake.Intake.IntakeState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class IntakeSetState extends InstantCommand{
    private Intake intake;
    private IntakeState intakeState;

    public IntakeSetState(IntakeState state) {
        this.intake = Intake.getInstance();
        this.intakeState = state;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setState(intakeState);
    }

}