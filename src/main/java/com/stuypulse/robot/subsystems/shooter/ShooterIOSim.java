package com.stuypulse.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Settings;

import com.stuypulse.robot.util.simulation.TalonFXSimulation.SystemSim;
import com.stuypulse.robot.util.simulation.TalonFXSimulation.TalonFXSimulation;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

public class ShooterIOSim implements ShooterIO {
    private final TalonFXSimulation shooterMotorLeft; // Follower
    private final TalonFXSimulation shooterMotorCenter; // Follower
    private final TalonFXSimulation shooterMotorRight; // Leader

    private final SystemSim<FlywheelSim> shooterSim;

    private final VelocityTorqueCurrentFOC shooterController;
    private final Follower shooterFollowerController;

    public ShooterIOSim(int leftId, int centerId, int rightId, double gearRatio) {
        this.shooterSim = SystemSim.of(
            new FlywheelSim(
                LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(3),
                Settings.Shooter.J.in(KilogramSquareMeters),
                gearRatio),
                DCMotor.getKrakenX60(3))
        );

        this.shooterMotorLeft = new TalonFXSimulation(leftId, gearRatio, shooterSim);
        Motors.Shooter.SHOOTER_MOTOR_LEFT.configure(shooterMotorLeft);

        this.shooterMotorCenter = new TalonFXSimulation(centerId, gearRatio, shooterSim);
        Motors.Shooter.SHOOTER_MOTOR_CENTER.configure(shooterMotorCenter);

        this.shooterMotorRight = new TalonFXSimulation(rightId, gearRatio, shooterSim);
        Motors.Shooter.SHOOTER_MOTOR_RIGHT.configure(shooterMotorRight);
        shooterMotorRight.getTorqueCurrent().setUpdateFrequency(Hertz.of(1000));

        this.shooterController = new VelocityTorqueCurrentFOC(0.0);
        this.shooterFollowerController = new Follower(shooterMotorRight.getDeviceID(), MotorAlignmentValue.Opposed);
        
        this.shooterMotorLeft.setControl(shooterFollowerController);
        this.shooterMotorCenter.setControl(shooterFollowerController);
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
    public void updateSim() {
        this.shooterSim.update(Settings.DT);
        this.shooterMotorRight.refresh();
        this.shooterMotorCenter.refresh();
        this.shooterMotorLeft.refresh();
    }
}
