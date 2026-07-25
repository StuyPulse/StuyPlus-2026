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
import edu.wpi.first.units.measure.*;

public abstract class ShooterIOTalonFXBase implements ShooterIO {
    private final TalonFX shooterMotorLeft;
    private final TalonFX shooterMotorCenter;
    private final TalonFX shooterMotorRight;

    private final VelocityTorqueCurrentFOC shooterController;
    private final VoltageOut sysIdController;
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
        Motors.Shooter.SHOOTER_MOTOR_RIGHT.configure(shooterMotorRight);
        Motors.Shooter.SHOOTER_MOTOR_CENTER.configure(shooterMotorCenter);
        Motors.Shooter.SHOOTER_MOTOR_LEFT.configure(shooterMotorLeft);

        this.shooterController = new VelocityTorqueCurrentFOC(0);
        this.sysIdController = new VoltageOut(0).withEnableFOC(true);

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
    public void stopMotors() {
        shooterMotorRight.stopMotor();
        shooterMotorCenter.stopMotor();
        shooterMotorLeft.stopMotor();
        shooterMotorCenter.setControl(shooterFollowerController);
        shooterMotorLeft.setControl(shooterFollowerController);
    }

    @Override
    public void setGainsSlot(int slot) {
        this.shooterController.withSlot(slot);
    }

    @Override
    public void setTargetVelocity(AngularVelocity targetVelocity) {
        shooterMotorRight.setControl(shooterController.withVelocity(targetVelocity));
    }

    @Override
    public void setTargetVoltage(Voltage voltage) {
        shooterMotorRight.setControl(sysIdController.withOutput(voltage));
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
}
