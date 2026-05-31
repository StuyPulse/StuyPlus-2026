package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.shooter.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class ShooterFirstShotIncrease extends Command{
    private final Shooter shooter;
    private final Timer timer;

    public ShooterFirstShotIncrease() {
        shooter = Shooter.getInstance();
        timer = new Timer();
    }

    @Override
    public void initialize() {
        timer.restart();
        shooter.addToBonusVelocity(150);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(1);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.resetBonusVelocity();
    }
}
