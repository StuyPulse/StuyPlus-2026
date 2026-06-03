package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.shooter.Shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterWaitForSpinUp extends Command {
    private static final Shooter shooter;

    public ShooterWaitForSpinUp() {
        addRequirements(shooter);
    }

    static {
        shooter = Shooter.getInstance();
    }

    @Override
    public boolean isFinished() {
        return shooter.shooterSpunUp();
    }
}
