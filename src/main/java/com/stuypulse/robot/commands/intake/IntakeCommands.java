package com.stuypulse.robot.commands.intake;

import static edu.wpi.first.units.Units.Degrees;

import com.stuypulse.robot.constants.Settings;
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

    public static Command setAgitateFastUp() {
        return Commands.runOnce(() -> intake.setState(IntakeState.AGITATE), intake).withName("IntakeSetAgitateFastUp");
    }

    public static Command setHomingDown() {
        return Commands.runOnce(() -> intake.setState(IntakeState.HOMING_DOWN), intake).withName("IntakeSetHomingDown");
    }

    public static Command setDigest() {
        return Commands.runOnce(() -> intake.setState(IntakeState.DIGEST), intake).withName("IntakeSetDigest");
    }

    //Agitation
    
    public static Command agitateFastOnce() {
        return Commands.sequence(
            setDown(),
            new WaitCommand(0.25),
            setAgitateFastUp(),
            new WaitCommand(0.25),
            setDown()
        ).withName("IntakeAgitateFastOnce");
    }

    //Zeroing

    public static Command zeroPivotNinety() {
        return Commands.runOnce(() -> intake.seedPivotAngle(Degrees.of(-90))).ignoringDisable(true).withName("IntakeSetPivotNinety");
    }

    public static Command zeroPivotStowed() {
        return Commands.runOnce(() -> {
            intake.seedPivotAngle(Settings.Intake.Pivot.STOW_ANGLE);
            intake.setState(IntakeState.IDLE);
        }).ignoringDisable(true).withName("IntakePivotSetZero");
    }

    public static Command zeroPivotDeployed() {
        return Commands.runOnce(() -> {
            intake.seedPivotAngle(Settings.Intake.Pivot.DEPLOY_ANGLE);
            intake.setState(IntakeState.DOWN);
        }).ignoringDisable(true).withName("IntakePivotSetZeroAtBottom");
    }
}
