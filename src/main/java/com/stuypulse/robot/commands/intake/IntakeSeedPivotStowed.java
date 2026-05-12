/**
 * ********************** PROJECT RON ************************
 */
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/**
 * ***********************************************************
 */
package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.intake.Intake.IntakeState;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class IntakeSeedPivotStowed extends InstantCommand {

    private final Intake intake;

    public IntakeSeedPivotStowed() {
        this.intake = Intake.getInstance();
        addRequirements(intake);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void initialize() {
        intake.seedPivotAngle(Settings.Intake.Pivot.STOW_ANGLE);
        intake.setState(IntakeState.IDLE);
    }
}
