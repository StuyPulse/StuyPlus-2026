package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;

public class ShooterSetShoot extends ShooterSetState{
    public ShooterSetShoot(){
        super(ShooterState.SHOOT);
    }
}
