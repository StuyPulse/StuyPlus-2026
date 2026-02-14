package com.stuypulse.robot.commands.intake;
import com.stuypulse.robot.subsystems.intake.Intake.IntakeState;;

public class IntakeSetIntake extends IntakeSetState{
    public IntakeSetIntake(){
        super(IntakeState.INTAKE);
        
    }
}