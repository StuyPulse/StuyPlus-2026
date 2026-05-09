/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.RobotVisualizer;
import com.stuypulse.robot.util.SysId;
import com.stuypulse.robot.util.simulation.TalonFXSimulation;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

public class ShooterSim extends Shooter {
    private final FlywheelSim shooterSim;
    private final TalonFXSimulation shooterMotorLeft;
    private final TalonFXSimulation shooterMotorCenter;
    private final TalonFXSimulation shooterMotorRight;
    private final VelocityTorqueCurrentFOC shooterController;
    private final Follower shooterFollowerController;

    private Optional<Voltage> voltageOverride;

    public ShooterSim() {
        shooterSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(
            DCMotor.getKrakenX60(3),
            Settings.Shooter.J.in(KilogramSquareMeters),
            Settings.Shooter.GEAR_RATIO),
            DCMotor.getKrakenX60(3)
        );
        shooterMotorRight = new TalonFXSimulation(Ports.Shooter.SHOOTER_MOTOR_RIGHT, shooterSim);
        shooterMotorCenter = new TalonFXSimulation(Ports.Shooter.SHOOTER_MOTOR_CENTER, shooterSim);
        shooterMotorLeft = new TalonFXSimulation(Ports.Shooter.SHOOTER_MOTOR_LEFT, shooterSim);

        Motors.Shooter.SHOOTER_MOTOR_RIGHT.configure(shooterMotorRight); // leader
        Motors.Shooter.SHOOTER_MOTOR_CENTER.configure(shooterMotorCenter);
        Motors.Shooter.SHOOTER_MOTOR_LEFT.configure(shooterMotorLeft);

        shooterController = new VelocityTorqueCurrentFOC(getState().getTargetAngularVelocity());
        shooterFollowerController = new Follower(shooterMotorRight.getDeviceID(), MotorAlignmentValue.Opposed);

        shooterMotorCenter.setControl(shooterFollowerController);
        shooterMotorLeft.setControl(shooterFollowerController);

        voltageOverride = Optional.empty();
    }

    @Override
    public AngularVelocity getCurrentAngularVelocity() {
        return shooterMotorRight.getVelocity().getValue();
    }

    @Override
    protected void stopMotors() {
        shooterMotorRight.stopMotor();
        shooterMotorCenter.stopMotor();
        shooterMotorLeft.stopMotor();

        shooterMotorCenter.setControl(shooterFollowerController);
        shooterMotorLeft.setControl(shooterFollowerController);
    }

    @Override
    public void setVoltageOverride(Voltage voltage) {
        this.voltageOverride = Optional.of(voltage);
    }

    @Override
    public void periodic() {
        if (!Settings.EnabledSubsystems.SHOOTER.get()) {
            stopMotors();
            return;
        }

        shooterMotorRight.setControl(shooterController.withVelocity(getState().getTargetAngularVelocity().in(RotationsPerSecond)));
        shooterMotorRight.update(Settings.DT); // leader first
        shooterMotorCenter.update(Settings.DT);
        shooterMotorLeft.update(Settings.DT);

        RobotVisualizer.getInstance().updateShooter(getCurrentAngularVelocity());

        super.periodic();
    }

    
    public SysIdRoutine getShooterSysIdRoutine() {
        return SysId.getRoutine(Settings.Shooter.RAMP_RATE,
                Settings.Shooter.STEP_VOLTAGE,
                "Shooter",
                voltage -> setVoltageOverride(voltage),
                () -> shooterMotorLeft.getPosition().getValue(),
                () -> shooterMotorLeft.getVelocity().getValue(),
                () -> shooterMotorLeft.getMotorVoltage().getValue(),
                getInstance());
    }
}
