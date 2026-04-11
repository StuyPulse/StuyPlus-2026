package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;

public class ShooterShoot extends ShooterSetState {
    public ShooterShoot() {
        super(ShooterState.SHOOTING);
    }
}