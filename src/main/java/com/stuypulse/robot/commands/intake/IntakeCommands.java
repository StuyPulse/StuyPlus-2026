package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.intake.Intake.IntakeState;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class IntakeCommands {
    private static final Intake intake;

    static {
        intake = Intake.getInstance();
    }

    //States

    public static Command setIntake() {
        return Commands.runOnce(() -> intake.setState(IntakeState.INTAKE), intake).withName("IntakeSetIntake");
    }

    public static Command setOuttake() {
        return Commands.runOnce(() -> intake.setState(IntakeState.OUTTAKE), intake).withName("IntakeSetOuttake");
    }
    public static Command setIdle() {
        return Commands.runOnce(() -> intake.setState(IntakeState.IDLE), intake).withName("IntakeSetIdle");
    }

    public static Command setDown() {
        return Commands.runOnce(() -> intake.setState(IntakeState.DOWN), intake).withName("IntakeSetDown");
    }

    public static Command setHomingDown() {
        return Commands.runOnce(() -> intake.setState(IntakeState.HOMING_DOWN), intake).withName("IntakeSetHomingDown");
    }

    //Agitation
    
    public static Command agitateOnce() {
        return Commands.sequence(
            setDown(),
            new WaitCommand(0.25),
            setIdle(),
            new WaitCommand(0.25),
            setDown()
        ).withName("IntakeAgitateOnce");
    }

    //Zeroing

    public static Command setZero() {
        return Commands.runOnce(() -> intake.setPivotZero()).withName("IntakePivotSetZero");
    }

    public static Command setZeroAtBottom() {
        return Commands.runOnce(() -> intake.setPivotZeroAtBottom()).withName("IntakePivotSetZeroAtBottom");
    }
}
