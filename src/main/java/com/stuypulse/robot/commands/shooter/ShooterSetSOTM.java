package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;

public class ShooterSetSOTM extends ShooterSetState {
    public ShooterSetSOTM(){
        super(ShooterState.SOTM);
    }
}