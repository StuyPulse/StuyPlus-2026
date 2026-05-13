/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.LoggedSignals;
import com.stuypulse.robot.util.SysId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.Optional;

public class ShooterImpl extends Shooter {

private final TalonFX shooterMotorLeft;

private final TalonFX shooterMotorCenter;

private final TalonFX shooterMotorRight;

private final VelocityTorqueCurrentFOC shooterController;

private final Follower shooterFollowerController;

private final LoggedSignals signals;

private Optional<Voltage> voltageOverride;

public ShooterImpl() {
	// leader
	shooterMotorRight = new TalonFX(Ports.Shooter.SHOOTER_MOTOR_RIGHT, Settings.CANBUS);
	shooterMotorCenter = new TalonFX(Ports.Shooter.SHOOTER_MOTOR_CENTER, Settings.CANBUS);
	shooterMotorLeft = new TalonFX(Ports.Shooter.SHOOTER_MOTOR_LEFT, Settings.CANBUS);
	shooterController = new VelocityTorqueCurrentFOC(getState().getTargetAngularVelocity());
	signals =
		new LoggedSignals(
				"Right Motor",
				shooterMotorRight.getSupplyCurrent(),
				shooterMotorRight.getStatorCurrent(),
				shooterMotorRight.getVelocity())
			.withMotor(
				"Center Motor",
				shooterMotorCenter.getSupplyCurrent(),
				shooterMotorCenter.getStatorCurrent(),
				shooterMotorCenter.getVelocity())
			.withMotor(
				"Left Motor",
				shooterMotorLeft.getSupplyCurrent(),
				shooterMotorLeft.getStatorCurrent(),
				shooterMotorLeft.getVelocity())
			.withLogPath("Shooter/")
			.withSignalLocation(LoggedSignals.SignalLocation.CANIVORE);
	// configure
	Motors.Shooter.SHOOTER_MOTOR_RIGHT.configure(shooterMotorRight);
	Motors.Shooter.SHOOTER_MOTOR_CENTER.configure(shooterMotorCenter);
	Motors.Shooter.SHOOTER_MOTOR_LEFT.configure(shooterMotorLeft);
	// Set center and left to follow right
	shooterFollowerController =
		new Follower(shooterMotorRight.getDeviceID(), MotorAlignmentValue.Opposed);
	shooterMotorCenter.setControl(shooterFollowerController);
	shooterMotorLeft.setControl(shooterFollowerController);
	voltageOverride = Optional.empty();
}

@Override
public void setVoltageOverride(Voltage voltage) {
	this.voltageOverride = Optional.of(voltage);
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
public void periodic() {
	if (!Settings.EnabledSubsystems.SHOOTER.get()) {
	stopMotors();
	return;
	}
	if (voltageOverride.isPresent()) {
	shooterMotorRight.setVoltage(voltageOverride.get().in(Volts));
	return;
	}
	final AngularVelocity targetAngularVelocity = getState().getTargetAngularVelocity();
	final VelocityTorqueCurrentFOC shooterControl =
		shooterController.withVelocity(targetAngularVelocity);
	shooterMotorRight.setControl(shooterControl);
	this.signals.logAll();
	super.periodic();
}

@Override
public SysIdRoutine getShooterSysIdRoutine() {
	return SysId.getRoutine(
		Settings.Shooter.RAMP_RATE,
		Settings.Shooter.STEP_VOLTAGE,
		"Shooter",
		this::setVoltageOverride,
		() -> shooterMotorRight.getPosition().getValue(),
		() -> shooterMotorRight.getVelocity().getValue(),
		() -> shooterMotorRight.getMotorVoltage().getValue(),
		getInstance());
}
}
