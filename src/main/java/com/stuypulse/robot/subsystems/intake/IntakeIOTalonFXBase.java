/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.intake;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.logged.LoggedTalonFX.LoggedTalonFX;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;

import edu.wpi.first.wpilibj.DigitalInput;

public abstract class IntakeIOTalonFXBase implements IntakeIO {
    private final LoggedTalonFX pivotMotor;

    private final LoggedTalonFX rollerMotorLeft;
    private final LoggedTalonFX rollerMotorRight;

    private final PositionTorqueCurrentFOC positionController;
    private final VoltageOut homingController;
    private final TorqueCurrentFOC pushdownController;

    private final DutyCycleOut rollerController;
    private final Follower followerController;
    
    private final DigitalInput pivotLimitSwitch;
    private final BooleanSupplier pivotStalling;

    private final BooleanSupplier leftRollerStalling;
    private final BooleanSupplier rightRollerStalling;
    private final Debouncer leftRollerDebouncer;
    private final Debouncer rightRollerDebouncer;

    public IntakeIOTalonFXBase(LoggedTalonFX pivotMotor, LoggedTalonFX rollerMotorLeft, LoggedTalonFX rollerMotorRight) {
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

        rollerController = new DutyCycleOut(0).withEnableFOC(true);
        followerController = new Follower(Ports.Intake.INTAKE_ROLLER_MOTOR_LEFT, MotorAlignmentValue.Opposed);
        rollerMotorRight.setControl(followerController);

        pivotLimitSwitch = new DigitalInput(Ports.Intake.PIVOT_LIMIT_SWITCH);
        pivotStalling = () -> pivotMotor.getStatorCurrent().getValue().gt(Settings.Intake.Pivot.STALL_CURRENT);

        leftRollerStalling = () -> rollerMotorLeft.getStatorCurrent().getValue().gt(Settings.Intake.Roller.STALL_CURRENT);
        rightRollerStalling = () -> rollerMotorRight.getStatorCurrent().getValue().gt(Settings.Intake.Roller.STALL_CURRENT);

        leftRollerDebouncer = new Debouncer(Settings.Intake.Roller.STALL_DEBOUNCE_SEC.in(Seconds), DebounceType.kBoth);
        rightRollerDebouncer = new Debouncer(Settings.Intake.Roller.STALL_DEBOUNCE_SEC.in(Seconds), DebounceType.kBoth);
    }

    @Override
    public void seedPivotAngle(Angle angle) {
        pivotMotor.setPosition(angle);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        pivotMotor.updateInputs(inputs.pivotMotorInputs);
        inputs.limitSwitchHit = !pivotLimitSwitch.get();
        inputs.pivotStalling = pivotStalling.getAsBoolean();
        inputs.pivotPushingDown = pivotMotor.getAppliedControl() == pushdownController;

        rollerMotorLeft.updateInputs(inputs.rollerMotorInputs);
        inputs.leftRollerStalling = leftRollerDebouncer.calculate(leftRollerStalling.getAsBoolean());
        inputs.rightRollerStalling = rightRollerDebouncer.calculate(rightRollerStalling.getAsBoolean());
    }

    @Override
    public void applyOutputs(IntakeIOOutputs outputs) {
        switch (outputs.pivot.outputMode) {
            case STOP -> pivotMotor.stopMotor();
            case POSITION -> pivotMotor.setControl(positionController.withPosition(outputs.pivot.position).withSlot(outputs.pivot.positionGainsSlot));
            case PUSHDOWN -> pivotMotor.setControl(pushdownController.withOutput(outputs.pivot.pushdown));
            case HOMING -> pivotMotor.setControl(homingController.withOutput(outputs.pivot.homing));
        }

        switch (outputs.roller.outputMode) {
            case STOP -> {
                rollerMotorLeft.stopMotor();
                rollerMotorRight.stopMotor();

                rollerMotorRight.setControl(followerController);
            }
            case DUTY_CYCLE -> rollerMotorLeft.setControl(rollerController.withOutput(outputs.roller.targetDutyCycle));
        }
    }
}
