/**
 * ********************** PROJECT RON ************************
 */
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/**
 * ***********************************************************
 */
package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ShooterSetState extends InstantCommand {

    private Shooter shooter;

    private ShooterState shooterState;

    public ShooterSetState(ShooterState shooterState) {
        this.shooter = Shooter.getInstance();
        this.shooterState = shooterState;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setState(shooterState);
    }
}
