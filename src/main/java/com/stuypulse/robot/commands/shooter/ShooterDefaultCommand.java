package com.stuypulse.robot.commands.shooter;


import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterDefaultCommand extends Command{
    private Shooter shooter;
    private CommandSwerveDrivetrain drivetrain;
    
    public ShooterDefaultCommand() {
        this.shooter = Shooter.getInstance();
        this.drivetrain = CommandSwerveDrivetrain.getInstance();

        addRequirements(shooter);
    }

    public void execute() {
        if (drivetrain.getPose().getX() < Field.allianceZone.getX()) {
            shooter.setState(ShooterState.SHOOTING);
        } else {
            shooter.setState(ShooterState.FERRYING);
        }
    }
}
