package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;

public class ShooterSetManual extends ShooterSetState{
    public ShooterSetManual() {
        super(ShooterState.MANUAL_HUB);
    }
}
