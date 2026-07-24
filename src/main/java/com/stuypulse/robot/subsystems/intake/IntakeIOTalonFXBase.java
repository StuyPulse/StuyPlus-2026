/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.intake;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.StatusSignal;
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

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;

import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeIOTalonFXBase implements IntakeIO {
    private final TalonFX pivotMotor;

    private final TalonFX rollerMotorLeft;
    private final TalonFX rollerMotorRight;

    private final PositionTorqueCurrentFOC positionController;
    private final VoltageOut homingController;
    private final TorqueCurrentFOC pushdownController;
    private final VoltageOut sysIdController;

    private final DutyCycleOut rollerController;
    private final Follower followerController;
    
    public final StatusSignal<Angle> pivotPosition;
    public final StatusSignal<AngularVelocity> pivotVelocity;
    public final StatusSignal<Voltage> pivotVoltage;
    public final StatusSignal<Current> pivotSupplyCurrent;
    public final StatusSignal<Current> pivotStatorCurrent;
    private final DigitalInput pivotLimitSwitch;
    private final BooleanSupplier pivotStalling;

    public final StatusSignal<AngularVelocity> rollerVelocity;
    public final StatusSignal<Voltage> rollerVoltage;
    public final StatusSignal<Current> rollerStatorCurrent;
    public final StatusSignal<Current> rollerSupplyCurrent;
    public final StatusSignal<Double> rollerDutyCycle;

    private final BooleanSupplier leftRollerStalling;
    private final BooleanSupplier rightRollerStalling;
    private final Debouncer leftRollerDebouncer;
    private final Debouncer rightRollerDebouncer;

    public IntakeIOTalonFXBase(TalonFX pivotMotor, TalonFX rollerMotorLeft, TalonFX rollerMotorRight) {
        this.pivotMotor = pivotMotor;
        Motors.Intake.PIVOT_CONFIG.configure(pivotMotor);
        pivotMotor.setPosition(Settings.Intake.Pivot.INITIAL_ANGLE);

        this.rollerMotorLeft = rollerMotorLeft;
        this.rollerMotorRight = rollerMotorRight;
        Motors.Intake.LEFT_ROLLER_CONFIG.configure(rollerMotorLeft);
        Motors.Intake.RIGHT_ROLLER_CONFIG.configure(rollerMotorRight);

        positionController = new PositionTorqueCurrentFOC(Settings.Intake.Pivot.INITIAL_ANGLE);
        homingController = new VoltageOut(Settings.Intake.Pivot.HOMING_DOWN_VOLTAGE).withEnableFOC(true);
        pushdownController = new TorqueCurrentFOC(Settings.Intake.Pivot.PUSHDOWN_CURRENT.getAsDouble());
        sysIdController = new VoltageOut(0).withEnableFOC(true);

        rollerController = new DutyCycleOut(0).withEnableFOC(true);
        followerController = new Follower(Ports.Intake.INTAKE_ROLLER_MOTOR_LEFT, MotorAlignmentValue.Opposed);
        rollerMotorRight.setControl(followerController);

        this.pivotPosition = pivotMotor.getPosition();
        this.pivotVelocity = pivotMotor.getVelocity();
        this.pivotVoltage = pivotMotor.getMotorVoltage();
        this.pivotSupplyCurrent = pivotMotor.getSupplyCurrent();
        this.pivotStatorCurrent = pivotMotor.getStatorCurrent();
        pivotLimitSwitch = new DigitalInput(Ports.Intake.PIVOT_LIMIT_SWITCH);
        pivotStalling = () -> pivotMotor.getStatorCurrent().getValue().gt(Settings.Intake.Pivot.STALL_CURRENT);

        this.rollerVelocity = rollerMotorLeft.getVelocity();
        this.rollerVoltage = rollerMotorLeft.getMotorVoltage();
        this.rollerStatorCurrent = rollerMotorLeft.getStatorCurrent();
        this.rollerSupplyCurrent = rollerMotorLeft.getSupplyCurrent();
        this.rollerDutyCycle = rollerMotorLeft.getDutyCycle();
        leftRollerStalling = () -> rollerMotorLeft.getStatorCurrent().getValue().gt(Settings.Intake.Roller.STALL_CURRENT);
        rightRollerStalling = () -> rollerMotorRight.getStatorCurrent().getValue().gt(Settings.Intake.Roller.STALL_CURRENT);

        leftRollerDebouncer = new Debouncer(Settings.Intake.Roller.STALL_DEBOUNCE_SEC.in(Seconds), DebounceType.kBoth);
        rightRollerDebouncer = new Debouncer(Settings.Intake.Roller.STALL_DEBOUNCE_SEC.in(Seconds), DebounceType.kBoth);
    }

    /*********************/
    /** Pivot Controls ***/
    /*********************/

    @Override
    public void setPivotPosition(Angle position) {
        pivotMotor.setControl(positionController.withPosition(position));
    }

    @Override
    public void setPivotPushdown(Current current) {
        pivotMotor.setControl(pushdownController.withOutput(current));
    }

    @Override
    public void setPivotHoming(Voltage voltage) {
        pivotMotor.setControl(homingController.withOutput(voltage));
    }

    @Override
    public void setVoltageOverride(Voltage voltage) {
        pivotMotor.setControl(sysIdController.withOutput(voltage));
    }

    /*********************/
    /** Roller Control ***/
    /*********************/

    @Override
    public void setRollerDutyCycle(double dutyCycle) {
        rollerMotorLeft.setControl(rollerController.withOutput(dutyCycle));
    }

    /*********************/
    /** Pivot Commands ***/
    /*********************/

    @Override
    public void seedPivotAngle(Angle angle) {
        pivotMotor.setPosition(angle);
    }

    @Override
    public void stopRollerMotors() {
        rollerMotorLeft.stopMotor();
        rollerMotorRight.stopMotor();
        // re-add the follow control after stopMotor removes it
        rollerMotorRight.setControl(followerController);
    }

    @Override
    public void stopPivotMotor() {
        pivotMotor.stopMotor();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        // if (pivotVoltageOverride.isPresent()) {
        //     pivotMotor.setVoltage(pivotVoltageOverride.get().in(Volts));
        //     return;
        // }

        // Inputs
        inputs.pivotPosition = pivotPosition.getValue();
        inputs.pivotVelocity = pivotVelocity.getValue();
        inputs.pivotVoltage = pivotVoltage.getValue();
        inputs.pivotStatorCurrent = pivotStatorCurrent.getValue();
        inputs.pivotSupplyCurrent = pivotSupplyCurrent.getValue();
        inputs.limitSwitchHit = !pivotLimitSwitch.get();
        inputs.pivotStalling = pivotStalling.getAsBoolean();
        inputs.pivotPushingDown = pivotMotor.getAppliedControl() == pushdownController;

        inputs.rollerVelocity = rollerVelocity.getValue();
        inputs.rollerVoltage = rollerVoltage.getValue();
        inputs.rollerStatorCurrent = rollerStatorCurrent.getValue();
        inputs.rollerSupplyCurrent = rollerSupplyCurrent.getValue();
        inputs.rollerDutyCycle = rollerDutyCycle.getValue();
        inputs.leftRollerStalling = leftRollerDebouncer.calculate(leftRollerStalling.getAsBoolean());
        inputs.rightRollerStalling = rightRollerDebouncer.calculate(rightRollerStalling.getAsBoolean());

        // State
        // if (limitSwitchHit()) {
        //     seedPivotAngle(Settings.Intake.Pivot.DEPLOY_ANGLE);
        // }

        // if (currentState == IntakeState.HOMING_DOWN && (pivotStalling || limitSwitchHit())) {
        //     seedPivotAngle(Settings.Intake.Pivot.DEPLOY_ANGLE);
        //     setState(IntakeState.INTAKE);
        // }
        // if ((currentState == IntakeState.DOWN) && (pivotStalling || limitSwitchHit())) {
        //     seedPivotAngle(Settings.Intake.Pivot.DEPLOY_ANGLE);
        // }
    }
}
