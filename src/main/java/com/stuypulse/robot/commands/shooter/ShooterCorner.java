package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;

public class ShooterCorner extends ShooterSetState{
    public ShooterCorner() {
        super(ShooterState.CORNER);
    }
}
