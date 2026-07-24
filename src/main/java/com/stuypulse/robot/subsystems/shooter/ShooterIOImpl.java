package com.stuypulse.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.LoggedSignals;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

public class ShooterIOImpl implements ShooterIO {
    private final TalonFX shooterMotorLeft; // Follower
    private final TalonFX shooterMotorCenter; // Follower
    private final TalonFX shooterMotorRight; // Leader

    private final VelocityTorqueCurrentFOC shooterController;
    private final Follower shooterFollowerController;

    private final LoggedSignals signals;

    public ShooterIOImpl() {
        this.shooterMotorLeft = new TalonFX(Ports.Shooter.SHOOTER_MOTOR_LEFT, Settings.CANBUS);
        Motors.Shooter.SHOOTER_MOTOR_LEFT.configure(shooterMotorLeft);

        this.shooterMotorCenter = new TalonFX(Ports.Shooter.SHOOTER_MOTOR_CENTER, Settings.CANBUS);
        Motors.Shooter.SHOOTER_MOTOR_CENTER.configure(shooterMotorCenter);

        this.shooterMotorRight = new TalonFX(Ports.Shooter.SHOOTER_MOTOR_RIGHT, Settings.CANBUS);
        Motors.Shooter.SHOOTER_MOTOR_RIGHT.configure(shooterMotorRight);
        shooterMotorRight.getTorqueCurrent().setUpdateFrequency(Hertz.of(1000)); // TODO: test effect on CANBus usage (SystemStats/CANBus/Utilization)

        this.shooterController = new VelocityTorqueCurrentFOC(0.0);
        this.shooterFollowerController = new Follower(shooterMotorRight.getDeviceID(), MotorAlignmentValue.Opposed);
        
        this.shooterMotorLeft.setControl(shooterFollowerController);
        this.shooterMotorCenter.setControl(shooterFollowerController);

        this.signals = new LoggedSignals(LoggedSignals.SignalLocation.CANIVORE, "Shooter/")
            .withMotor(
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
                shooterMotorLeft.getVelocity());
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.position = shooterMotorRight.getPosition().getValue();
        inputs.voltage = shooterMotorRight.getMotorVoltage().getValue();
        inputs.angularVelocity = shooterMotorRight.getVelocity().getValue();

        inputs.leftMotor.voltage = shooterMotorLeft.getMotorVoltage().getValue();
        inputs.leftMotor.supplyCurrent = shooterMotorLeft.getSupplyCurrent().getValue();
        inputs.leftMotor.statorCurrent = shooterMotorLeft.getStatorCurrent().getValue();
        inputs.leftMotor.angularVelocity = shooterMotorLeft.getVelocity().getValue();

        inputs.centerMotor.voltage = shooterMotorCenter.getMotorVoltage().getValue();
        inputs.centerMotor.supplyCurrent = shooterMotorCenter.getSupplyCurrent().getValue();
        inputs.centerMotor.statorCurrent = shooterMotorCenter.getStatorCurrent().getValue();
        inputs.centerMotor.angularVelocity = shooterMotorCenter.getVelocity().getValue();

        inputs.rightMotor.voltage = shooterMotorRight.getMotorVoltage().getValue();
        inputs.rightMotor.supplyCurrent = shooterMotorRight.getSupplyCurrent().getValue();
        inputs.rightMotor.statorCurrent = shooterMotorRight.getStatorCurrent().getValue();
        inputs.rightMotor.angularVelocity = shooterMotorRight.getVelocity().getValue();
    }

    @Override
    public void stopMotors() {
        shooterMotorRight.stopMotor();
        shooterMotorCenter.stopMotor();
        shooterMotorLeft.stopMotor();
        shooterMotorCenter.setControl(this.shooterFollowerController);
        shooterMotorLeft.setControl(this.shooterFollowerController);
    }

    @Override
    public void setShooterAngularVelocity(AngularVelocity velocity, int gainSlot) {
        shooterMotorRight.setControl(shooterController.withVelocity(velocity).withSlot(gainSlot));
    }

    @Override
    public void logHardwareSignals() {
        this.signals.logAll();
    }

    @Override
    public void setVoltageOverride(Voltage voltage) {
        shooterMotorRight.setVoltage(voltage.in(Volts));
    }
}
