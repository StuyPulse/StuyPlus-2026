package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;

public class ShooterFerry extends ShooterSetState {
    public ShooterFerry() {
        super(ShooterState.FERRYING);
    }
}