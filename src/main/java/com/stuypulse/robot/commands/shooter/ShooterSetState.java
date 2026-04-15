package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;
import com.stuypulse.robot.subsystems.shooter.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ShooterSetState extends InstantCommand {
    private Shooter shooter;
    private ShooterState shooterState; 
    
    public ShooterSetState(ShooterState shooterState){
        this.shooter = Shooter.getInstance();
        this.shooterState = shooterState;

        addRequirements(shooter);
    }
    
    @Override
    
    public void initialize(){
        shooter.setState(shooterState);
    }
}
