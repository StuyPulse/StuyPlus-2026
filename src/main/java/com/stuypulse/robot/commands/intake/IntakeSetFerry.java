package com.stuypulse.robot.commands.intake;
import com.stuypulse.robot.subsystems.intake.Intake.IntakeState;;

public class IntakeSetFerry extends IntakeSetState{
    public IntakeSetFerry(){
        super(IntakeState.FERRY);
        
    }
}