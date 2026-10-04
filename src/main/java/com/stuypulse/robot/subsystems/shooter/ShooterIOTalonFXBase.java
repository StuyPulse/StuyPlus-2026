/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.subsystems.shooter.ShooterConstants.ShooterSettings;

import edu.wpi.first.units.measure.*;

public abstract class ShooterIOTalonFXBase implements ShooterIO {
    private final TalonFX shooterMotorLeft;
    private final TalonFX shooterMotorCenter;
    private final TalonFX shooterMotorRight;

    private final VelocityTorqueCurrentFOC shooterController;
    private final Follower shooterFollowerController;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> voltage;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> statorCurrent;

    public ShooterIOTalonFXBase(TalonFX shooterMotorRight, TalonFX shooterMotorCenter, TalonFX shooterMotorLeft) {
        // leader
        this.shooterMotorRight = shooterMotorRight;
        this.shooterMotorCenter = shooterMotorCenter;
        this.shooterMotorLeft = shooterMotorLeft;

        // configure
        ShooterConstants.ShooterMotorConfigs.SHOOTER_MOTOR_RIGHT.configure(shooterMotorRight);
        ShooterConstants.ShooterMotorConfigs.SHOOTER_MOTOR_CENTER.configure(shooterMotorCenter);
        ShooterConstants.ShooterMotorConfigs.SHOOTER_MOTOR_LEFT.configure(shooterMotorLeft);

        this.shooterController = new VelocityTorqueCurrentFOC(0);

        this.position = shooterMotorRight.getPosition();
        this.velocity = shooterMotorRight.getVelocity();
        this.voltage = shooterMotorRight.getMotorVoltage();
        this.torqueCurrent = shooterMotorRight.getTorqueCurrent();
        this.torqueCurrent.setUpdateFrequency(Hertz.of(1000));
        this.supplyCurrent = shooterMotorRight.getSupplyCurrent();
        this.statorCurrent = shooterMotorRight.getStatorCurrent();

        shooterFollowerController = new Follower(shooterMotorRight.getDeviceID(), MotorAlignmentValue.Opposed);
        shooterMotorCenter.setControl(shooterFollowerController);
        shooterMotorLeft.setControl(shooterFollowerController);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.position = this.position.refresh().getValue();
        inputs.velocity = this.velocity.refresh().getValue();
        inputs.voltage = this.voltage.refresh().getValue();
        inputs.torqueCurrent = this.torqueCurrent.refresh().getValue();
        inputs.supplyCurrent = this.supplyCurrent.refresh().getValue();
        inputs.statorCurrent = this.statorCurrent.refresh().getValue();
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
