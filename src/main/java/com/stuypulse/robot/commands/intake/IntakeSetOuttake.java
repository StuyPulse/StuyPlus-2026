package com.stuypulse.robot.commands.intake;
import com.stuypulse.robot.subsystems.intake.Intake.IntakeState;;

public class IntakeSetOuttake extends IntakeSetState {
    public IntakeSetOuttake(){
        super(IntakeState.OUTTAKE);
    }
}