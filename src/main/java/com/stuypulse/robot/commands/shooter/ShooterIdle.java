package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;

public class ShooterIdle extends ShooterSetState {
    public ShooterIdle() {
        super(ShooterState.IDLE);
    }
}