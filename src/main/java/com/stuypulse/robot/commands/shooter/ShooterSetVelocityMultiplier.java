package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.shooter.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ShooterSetVelocityMultiplier extends InstantCommand{
    private final Shooter shooter;
    private final double multiplier;

    public ShooterSetVelocityMultiplier(double multiplier) {
        this.shooter = Shooter.getInstance();
        this.multiplier = multiplier;
    }

    @Override
    public void initialize() {
        shooter.setVelocityMultiplier(multiplier);
    }
}
