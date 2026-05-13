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

import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;

public class ShooterSetShoot extends ShooterSetState {

    public ShooterSetShoot() {
        super(ShooterState.SHOOT);
    }
}
