package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;

public class ShooterSetFOTM extends ShooterSetState {
    public ShooterSetFOTM(){
        super(ShooterState.FOTM);
    }
}
