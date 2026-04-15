package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;

public class ShooterSetIdle extends ShooterSetState {
    public ShooterSetIdle() {
        super(ShooterState.IDLE);
    }
}