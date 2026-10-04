/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.intake;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.util.logged.LoggedTalonFX.LoggedTalonFX;

import edu.wpi.first.units.measure.*;

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

    public IntakeIOTalonFXBase(LoggedTalonFX pivotMotor, LoggedTalonFX rollerMotorLeft, LoggedTalonFX rollerMotorRight) {
        this.pivotMotor = pivotMotor;
        IntakeConstants.IntakeMotorConfigs.PIVOT_CONFIG.configure(pivotMotor);
        pivotMotor.setPosition(IntakeConstants.IntakeSettings.Pivot.INITIAL_ANGLE);

        this.rollerMotorLeft = rollerMotorLeft;
        this.rollerMotorRight = rollerMotorRight;
        IntakeConstants.IntakeMotorConfigs.LEFT_ROLLER_CONFIG.configure(rollerMotorLeft);
        IntakeConstants.IntakeMotorConfigs.RIGHT_ROLLER_CONFIG.configure(rollerMotorRight);

        positionController = new PositionTorqueCurrentFOC(IntakeConstants.IntakeSettings.Pivot.INITIAL_ANGLE);
        homingController = new VoltageOut(IntakeConstants.IntakeSettings.Pivot.HOMING_DOWN_VOLTAGE).withEnableFOC(true);
        pushdownController = new TorqueCurrentFOC(IntakeConstants.IntakeSettings.Pivot.PUSHDOWN_CURRENT.getAsDouble());

        rollerController = new DutyCycleOut(0).withEnableFOC(true);
        followerController = new Follower(IntakeConstants.IntakeDeviceIds.INTAKE_ROLLER_MOTOR_LEFT, MotorAlignmentValue.Opposed);
        rollerMotorRight.setControl(followerController);

        pivotLimitSwitch = new DigitalInput(IntakeConstants.IntakeDeviceIds.PIVOT_LIMIT_SWITCH);
    }

    @Override
    public void seedPivotAngle(Angle angle) {
        pivotMotor.setPosition(angle);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {

        pivotMotor.updateInputs(inputs.pivotMotorInputs);
        inputs.limitSwitchHit = !pivotLimitSwitch.get();

        rollerMotorLeft.updateInputs(inputs.leftRollerMotorInputs);
        rollerMotorRight.updateInputs(inputs.rightRollerMotorInputs);
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
