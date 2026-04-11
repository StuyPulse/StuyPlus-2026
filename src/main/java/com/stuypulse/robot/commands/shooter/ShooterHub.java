package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;

public class ShooterHub extends ShooterSetState{
    public ShooterHub() {
        super(ShooterState.HUB);
    }
}
