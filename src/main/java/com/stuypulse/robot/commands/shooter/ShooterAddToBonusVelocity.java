package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.shooter.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ShooterAddToBonusVelocity extends InstantCommand{
    private final Shooter shooter;
    private final double velocity;

    public ShooterAddToBonusVelocity(double velocity) {
        shooter = Shooter.getInstance();
        this.velocity = velocity;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.addToBonusVelocity(velocity);
    }
}
