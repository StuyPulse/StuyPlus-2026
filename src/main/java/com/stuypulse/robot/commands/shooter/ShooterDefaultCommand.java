package com.stuypulse.robot.commands.shooter;


import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterDefaultCommand extends Command{
    private Shooter shooter;
    
    public ShooterDefaultCommand() {
        this.shooter = Shooter.getInstance();

        addRequirements(shooter);
    }

    public void execute() {
        if (Field.inAllianceZone()) {
            shooter.setState(ShooterState.SHOOTING);
        } else {
            shooter.setState(ShooterState.FERRYING);
        }
    }
}
