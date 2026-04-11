package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;

public class ShooterFOTM extends ShooterSetState{
    public ShooterFOTM() {
        super(ShooterState.FOTM);
    }
}
