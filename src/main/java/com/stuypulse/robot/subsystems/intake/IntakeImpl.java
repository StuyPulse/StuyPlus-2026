/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.EnabledSubsystems;
import com.stuypulse.robot.util.LoggedSignals;
import com.stuypulse.robot.util.SysId;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.Optional;
import java.util.function.BooleanSupplier;

public class IntakeImpl extends Intake {

    private final TalonFX pivotMotor;

    private final TalonFX rollerMotorLeft;

    private final TalonFX rollerMotorRight;

    private final PositionTorqueCurrentFOC positionController;

    private final VoltageOut homingController;

    private final TorqueCurrentFOC pushdownController;

    private final DutyCycleOut rollerController;

    private final Follower followerController;

    private final LoggedSignals pivotSignals;

    private final LoggedSignals rollerSignals;

    private final BooleanSupplier pivotStalling;

    private Optional<Voltage> pivotVoltageOverride;

    private final BStream leftRollerStalling;

    private final BStream rightRollerStalling;

    public IntakeImpl() {
        pivotMotor = new TalonFX(Ports.Intake.INTAKE_PIVOT_MOTOR, Settings.CANBUS);
        Motors.Intake.PIVOT_CONFIG.configure(pivotMotor);
        // zero it at the up pos
        pivotMotor.setPosition(Settings.Intake.Pivot.INITIAL_ANGLE);
        pivotSignals = new LoggedSignals(LoggedSignals.SignalLocation.CANIVORE, "Intake/Pivot/",
                pivotMotor.getSupplyCurrent(),
                pivotMotor.getStatorCurrent(),
                pivotMotor.getVelocity());
        // until rollers are fixed
        // leader
        rollerMotorLeft = new TalonFX(Ports.Intake.INTAKE_ROLLER_MOTOR_LEFT, Settings.CANBUS);
        rollerMotorRight = new TalonFX(Ports.Intake.INTAKE_ROLLER_MOTOR_RIGHT, Settings.CANBUS);
        Motors.Intake.LEFT_ROLLER_CONFIG.configure(rollerMotorLeft);
        Motors.Intake.RIGHT_ROLLER_CONFIG.configure(rollerMotorRight);
        // until rollers are fixed
        rollerSignals = new LoggedSignals(LoggedSignals.SignalLocation.CANIVORE, "Intake/Roller/").withMotor(
                "Intake Roller Left",
                rollerMotorLeft.getSupplyCurrent(),
                rollerMotorLeft.getStatorCurrent(),
                rollerMotorLeft.getVelocity())
                .withMotor(
                        "Intake Roller Right",
                        rollerMotorRight.getSupplyCurrent(),
                        rollerMotorRight.getStatorCurrent(),
                        rollerMotorRight.getVelocity());
        positionController = new PositionTorqueCurrentFOC(getState().getTargetAngle());
        homingController = new VoltageOut(Settings.Intake.Pivot.HOMING_DOWN_VOLTAGE).withEnableFOC(true);
        pushdownController = new TorqueCurrentFOC(Settings.Intake.Pivot.PUSHDOWN_CURRENT.getAsDouble());
        rollerController = new DutyCycleOut(getState().getTargetDutyCycle()).withEnableFOC(true);
        followerController = new Follower(Ports.Intake.INTAKE_ROLLER_MOTOR_LEFT, MotorAlignmentValue.Opposed);
        rollerMotorRight.setControl(followerController);
        pivotStalling = () -> pivotMotor.getStatorCurrent().getValue().gt(Settings.Intake.Pivot.STALL_CURRENT);
        // until rollers are fixed
        leftRollerStalling = BStream.create(
                () -> rollerMotorLeft.getStatorCurrent().getValueAsDouble() > Settings.Intake.Roller.STALL_CURRENT
                        .in(Amps))
                .filtered(new BDebounce.Both(Settings.Intake.Roller.STALL_DEBOUNCE_SEC.in(Seconds)));
        rightRollerStalling = BStream.create(
                () -> rollerMotorRight.getStatorCurrent()
                        .getValueAsDouble() > Settings.Intake.Roller.STALL_CURRENT.in(Amps))
                .filtered(new BDebounce.Both(Settings.Intake.Roller.STALL_DEBOUNCE_SEC.in(Seconds)));
        pivotVoltageOverride = Optional.empty();
    }

    // For sysid
    @Override
    public void setPivotVoltageOverride(Voltage voltage) {
        this.pivotVoltageOverride = Optional.of(voltage);
    }

    /** ****************** */
    /** Pivot Commands ** */
    /** ****************** */
    @Override
    public Angle getRelativePosition() {
        return pivotMotor.getPosition().getValue();
    }

    @Override
    public void seedPivotAngle(Angle angle) {
        pivotMotor.setPosition(angle);
    }

    private boolean pivotStalling() {
        return pivotStalling.getAsBoolean();
    }

    /** ******************* */
    /** Roller Commands ** */
    /** ******************* */
    // until rollers are fixed
    @Override
    public AngularVelocity getRollerVelocity() {
        return rollerMotorRight.getVelocity().getValue();
    }

    // until rollers are fixed
    private boolean leftRollerStalling() {
        return leftRollerStalling.get();
    }

    private boolean rightRollerStalling() {
        return rightRollerStalling.get();
    }

    @Override
    protected void stopMotors() {
        pivotMotor.stopMotor();
        // until rollers are fixed
        rollerMotorLeft.stopMotor();
        rollerMotorRight.stopMotor();
        // re-add the follow control after stopMotor removes it
        rollerMotorRight.setControl(followerController);
    }

    private ControlRequest getPivotControl(IntakeState currentState) {
        boolean pivotAboveThreshold = isPivotAboveThreshold();
        return switch (currentState) {
            case INTAKE, OUTTAKE, DOWN -> pivotAboveThreshold
                    ? pushdownController.withOutput(Settings.Intake.Pivot.PUSHDOWN_CURRENT.getAsDouble())
                    : positionController.withPosition(currentState.getTargetAngle()).withSlot(0);
            case HOMING_DOWN -> homingController;
            case DIGEST -> positionController.withPosition(currentState.getTargetAngle()).withSlot(1);
            default -> positionController.withPosition(currentState.getTargetAngle()).withSlot(0);
        };
    }

    @Override
    public void periodic() {
        if (!EnabledSubsystems.INTAKE.get()) {
            stopMotors();
            return;
        }
        if (pivotVoltageOverride.isPresent()) {
            pivotMotor.setVoltage(pivotVoltageOverride.get().in(Volts));
            return;
        }
        // Input
        final boolean pivotStalling = pivotStalling();
        final IntakeState currentState = getState();
        // State
        if (currentState == IntakeState.HOMING_DOWN && pivotStalling) {
            seedPivotAngle(Settings.Intake.Pivot.DEPLOY_ANGLE);
            setState(IntakeState.DOWN);
        }
        if ((currentState == IntakeState.DOWN) && pivotStalling) {
            seedPivotAngle(Settings.Intake.Pivot.DEPLOY_ANGLE);
        }
        // Output
        final ControlRequest pivotControl = getPivotControl(currentState);
        final DutyCycleOut rollerControl = rollerController.withOutput(currentState.getTargetDutyCycle());
        // Apply
        pivotMotor.setControl(pivotControl);
        rollerMotorLeft.setControl(rollerControl);
        // Logging
        this.pivotSignals.logAll();
        // until rollers are fixed
        this.rollerSignals.logAll();
        DogLog.forceNt.log("Intake/Rollers/Stalling", leftRollerStalling() || rightRollerStalling());
        DogLog.forceNt.log(
                "Intake/Pivot/Pushing Down", pivotMotor.getAppliedControl() == pushdownController);
        super.periodic();
    }

    @Override
    public SysIdRoutine getIntakeSysIdRoutine() {
        return SysId.getRoutine(
                Settings.Intake.Pivot.RAMP_RATE,
                Settings.Intake.Pivot.STEP_VOLTAGE,
                "Intake",
                this::setPivotVoltageOverride,
                () -> pivotMotor.getPosition().getValue(),
                () -> pivotMotor.getVelocity().getValue(),
                () -> pivotMotor.getMotorVoltage().getValue(),
                getInstance());
    }
}
