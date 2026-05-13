/**
 * ********************** PROJECT RON ************************
 */
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/**
 * ***********************************************************
 */
package com.stuypulse.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.EnabledSubsystems;
import com.stuypulse.robot.util.LoggedSignals;
import com.stuypulse.robot.util.SysId;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.filter.Debouncer;
import dev.doglog.DogLog;

public class IntakeImpl extends Intake {

    private final TalonFX intakePivotMotor;

    private final TalonFX intakeRollerMotorLeft;

    private final TalonFX intakeRollerMotorRight;

    private final PositionTorqueCurrentFOC positionController;

    private final VoltageOut homingController;

    private final TorqueCurrentFOC pushdownController;

    private final DutyCycleOut rollerController;

    private final Follower followerController;

    private final LoggedSignals pivotSignals;

    private final LoggedSignals rollerSignals;

    private final BooleanSupplier pivotStalling;

    private Optional<Voltage> pivotVoltageOverride;

    private final Debouncer rollerStallingDebouncer;

    public IntakeImpl() {
        intakePivotMotor = new TalonFX(Ports.Intake.INTAKE_PIVOT_MOTOR, Settings.CANBUS);
        Motors.Intake.PIVOT_CONFIG.configure(intakePivotMotor);
        // zero it at the up pos
        intakePivotMotor.setPosition(Settings.Intake.Pivot.INITIAL_ANGLE);
        pivotSignals = new LoggedSignals(intakePivotMotor.getSupplyCurrent(), intakePivotMotor.getStatorCurrent(), intakePivotMotor.getVelocity()).withLogPath("Intake/Pivot/").withSignalLocation(LoggedSignals.SignalLocation.CANIVORE);
        // until rollers are fixed
        // leader
        intakeRollerMotorLeft = new TalonFX(Ports.Intake.INTAKE_ROLLER_MOTOR_LEFT, Settings.CANBUS);
        intakeRollerMotorRight = new TalonFX(Ports.Intake.INTAKE_ROLLER_MOTOR_RIGHT, Settings.CANBUS);
        Motors.Intake.LEFT_ROLLER_CONFIG.configure(intakeRollerMotorLeft);
        Motors.Intake.RIGHT_ROLLER_CONFIG.configure(intakeRollerMotorRight);
        // until rollers are fixed
        rollerSignals = new LoggedSignals("Intake Roller Left", intakeRollerMotorLeft.getSupplyCurrent(), intakeRollerMotorLeft.getStatorCurrent(), intakeRollerMotorLeft.getVelocity()).withMotor("Intake Roller Right", intakeRollerMotorRight.getSupplyCurrent(), intakeRollerMotorRight.getStatorCurrent(), intakeRollerMotorRight.getVelocity()).withLogPath("Intake/Roller/").withSignalLocation(LoggedSignals.SignalLocation.CANIVORE);
        positionController = new PositionTorqueCurrentFOC(getState().getTargetAngle());
        homingController = new VoltageOut(Settings.Intake.Pivot.HOMING_DOWN_VOLTAGE).withEnableFOC(true);
        pushdownController = new TorqueCurrentFOC(Settings.Intake.Pivot.PUSHDOWN_CURRENT.getAsDouble());
        rollerController = new DutyCycleOut(getState().getTargetDutyCycle()).withEnableFOC(true);
        followerController = new Follower(Ports.Intake.INTAKE_ROLLER_MOTOR_LEFT, MotorAlignmentValue.Opposed);
        intakeRollerMotorRight.setControl(followerController);
        pivotStalling = () -> intakePivotMotor.getStatorCurrent().getValue().gt(Settings.Intake.Pivot.STALL_CURRENT);
        // until rollers are fixed
        rollerStallingDebouncer = new Debouncer(Settings.Intake.Roller.STALL_DEBOUNCE_SEC.in(Seconds), Debouncer.DebounceType.kBoth);
        pivotVoltageOverride = Optional.empty();
    }

    // For sysid
    @Override
    public void setPivotVoltageOverride(Voltage voltage) {
        this.pivotVoltageOverride = Optional.of(voltage);
    }

    /**
     * ******************
     */
    /**
     * Pivot Commands **
     */
    /**
     * ******************
     */
    @Override
    public Angle getRelativePosition() {
        return intakePivotMotor.getPosition().getValue();
    }

    @Override
    public void seedPivotAngle(Angle angle) {
        intakePivotMotor.setPosition(angle);
    }

    private boolean pivotStalling() {
        return pivotStalling.getAsBoolean();
    }

    /**
     * *******************
     */
    /**
     * Roller Commands **
     */
    /**
     * *******************
     */
    // until rollers are fixed
    @Override
    public AngularVelocity getRollerVelocity() {
        return intakeRollerMotorRight.getVelocity().getValue();
    }

    // until rollers are fixed
    private boolean leftRollerStalling() {
        boolean stalling = intakeRollerMotorLeft.getStatorCurrent().getValueAsDouble() > Settings.Intake.Roller.STALL_CURRENT.in(Amps);
        return rollerStallingDebouncer.calculate(stalling);
    }

    private boolean rightRollerStalling() {
        boolean stalling = intakeRollerMotorRight.getStatorCurrent().getValueAsDouble() > Settings.Intake.Roller.STALL_CURRENT.in(Amps);
        return rollerStallingDebouncer.calculate(stalling);
    }

    @Override
    protected void stopMotors() {
        intakePivotMotor.stopMotor();
        // until rollers are fixed
        intakeRollerMotorLeft.stopMotor();
        intakeRollerMotorRight.stopMotor();
        // re-add the follow control after stopMotor removes it
        intakeRollerMotorRight.setControl(followerController);
    }

    private ControlRequest getPivotControl(IntakeState currentState) {
        boolean pivotAboveThreshold = isPivotAboveThreshold();
        return switch(currentState) {
            case INTAKE, OUTTAKE, DOWN ->
                pivotAboveThreshold ? pushdownController.withOutput(Settings.Intake.Pivot.PUSHDOWN_CURRENT.getAsDouble()) : positionController.withPosition(currentState.getTargetAngle()).withSlot(0);
            case HOMING_DOWN ->
                homingController;
            case DIGEST ->
                positionController.withPosition(currentState.getTargetAngle()).withSlot(1);
            default ->
                positionController.withPosition(currentState.getTargetAngle()).withSlot(0);
        };
    }

    @Override
    public void periodic() {
        if (!EnabledSubsystems.INTAKE.get()) {
            stopMotors();
            return;
        }
        if (pivotVoltageOverride.isPresent()) {
            intakePivotMotor.setVoltage(pivotVoltageOverride.get().in(Volts));
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
        intakePivotMotor.setControl(pivotControl);
        intakeRollerMotorLeft.setControl(rollerControl);
        // Logging
        this.pivotSignals.logAll();
        // until rollers are fixed
        this.rollerSignals.logAll();
        DogLog.log("Intake/Rollers/Stalling", leftRollerStalling() || rightRollerStalling());
        DogLog.log("Intake/Pivot/Pushing Down", intakePivotMotor.getAppliedControl() == pushdownController);
        super.periodic();
    }

    @Override
    public SysIdRoutine getIntakeSysIdRoutine() {
        return SysId.getRoutine(Settings.Intake.Pivot.RAMP_RATE, Settings.Intake.Pivot.STEP_VOLTAGE, "Intake", this::setPivotVoltageOverride, () -> intakePivotMotor.getPosition().getValue(), () -> intakePivotMotor.getVelocity().getValue(), () -> intakePivotMotor.getMotorVoltage().getValue(), getInstance());
    }
}
