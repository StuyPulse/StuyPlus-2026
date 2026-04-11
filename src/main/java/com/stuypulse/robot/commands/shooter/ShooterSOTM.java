package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;

public class ShooterSOTM extends ShooterSetState{
    public ShooterSOTM() {
        super(ShooterState.SOTM);
    }
}
