package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;

public class ShooterSetFerry extends ShooterSetState{
    public ShooterSetFerry() {
        super(ShooterState.FERRY);
    }

}
