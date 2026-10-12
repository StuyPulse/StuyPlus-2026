/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.util.logged.LoggedTalonFX.LoggedTalonFX;

public abstract class ShooterIOBase implements ShooterIO {
    private final LoggedTalonFX shooterMotorLeft;
    private final LoggedTalonFX shooterMotorCenter;
    private final LoggedTalonFX shooterMotorRight;

    private final VelocityTorqueCurrentFOC shooterController;
    private final Follower shooterFollowerController;

    public ShooterIOBase(LoggedTalonFX shooterMotorRight, LoggedTalonFX shooterMotorCenter, LoggedTalonFX shooterMotorLeft) {
        // leader
        this.shooterMotorRight = shooterMotorRight;
        this.shooterMotorCenter = shooterMotorCenter;
        this.shooterMotorLeft = shooterMotorLeft;

        // configure
        ShooterConstants.ShooterMotorConfigs.SHOOTER_MOTOR_RIGHT.configure(shooterMotorRight);
        ShooterConstants.ShooterMotorConfigs.SHOOTER_MOTOR_CENTER.configure(shooterMotorCenter);
        ShooterConstants.ShooterMotorConfigs.SHOOTER_MOTOR_LEFT.configure(shooterMotorLeft);

        this.shooterController = new VelocityTorqueCurrentFOC(0);

        shooterFollowerController = new Follower(shooterMotorRight.getDeviceID(), MotorAlignmentValue.Opposed);
        shooterMotorCenter.setControl(shooterFollowerController);
        shooterMotorLeft.setControl(shooterFollowerController);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        shooterMotorRight.updateInputs(inputs.shooterMotorRightInputs);
        shooterMotorLeft.updateInputs(inputs.shooterMotorLeftInputs);
        shooterMotorCenter.updateInputs(inputs.shooterMotorCenterInputs);
    }

    @Override
    public void applyOutputs(ShooterIOOutputs outputs) {
        switch (outputs.mode) {
            case VELOCITY -> shooterMotorRight.setControl(shooterController
                                                            .withVelocity(outputs.targetVelocity)
                                                            .withSlot(outputs.gainSlot));

            case STOP -> {
                shooterMotorRight.stopMotor();
                shooterMotorCenter.stopMotor();
                shooterMotorLeft.stopMotor();
                shooterMotorCenter.setControl(shooterFollowerController);
                shooterMotorLeft.setControl(shooterFollowerController);
            }
        }
    }
}
