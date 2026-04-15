package com.stuypulse.robot.subsystems.shooter;

import java.util.Optional;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterImpl extends Shooter {
    private TalonFX shooterMotorLeft;
    private TalonFX shooterMotorCenter;
    private TalonFX shooterMotorRight;

    private TalonFX bottomMotorLeft;
    private TalonFX bottomMotorRight;

    private Optional<Double> voltageOveride;

    public ShooterImpl() {
        shooterMotorLeft = new TalonFX(Ports.ShooterPorts.SHOOTER_MOTOR_LEFT, Settings.CANIVORE);
        shooterMotorCenter = new TalonFX(Ports.ShooterPorts.SHOOTER_MOTOR_CENTER, Settings.CANIVORE);
        shooterMotorRight = new TalonFX(Ports.ShooterPorts.SHOOTER_MOTOR_RIGHT, Settings.CANIVORE);

        bottomMotorLeft = new TalonFX(Ports.ShooterPorts.BOTTOM_MOTOR_LEFT, Settings.CANIVORE);
        bottomMotorRight = new TalonFX(Ports.ShooterPorts.BOTTOM_MOTOR_RIGHT, Settings.CANIVORE);

        shooterMotorCenter = new TalonFX(Ports.ShooterPorts.SHOOTER_MOTOR_CENTER, Settings.CANIVORE);
        shooterMotorRight = new TalonFX(Ports.ShooterPorts.SHOOTER_MOTOR_RIGHT, Settings.CANIVORE);

        bottomMotorLeft = new TalonFX(Ports.ShooterPorts.BOTTOM_MOTOR_LEFT, Settings.CANIVORE);
        bottomMotorRight = new TalonFX(Ports.ShooterPorts.BOTTOM_MOTOR_RIGHT, Settings.CANIVORE);

        // configure
        Motors.Shooter.SHOOTER_MOTOR_CONFIG.configure(shooterMotorLeft);
        Motors.Shooter.SHOOTER_MOTOR_CONFIG.configure(shooterMotorCenter);
        Motors.Shooter.SHOOTER_MOTOR_CONFIG.configure(shooterMotorRight);
        Motors.Shooter.BOTTOM_MOTOR_CONFIG.configure(bottomMotorLeft);
        Motors.Shooter.BOTTOM_MOTOR_CONFIG.configure(bottomMotorRight);

        // Set shooter 2 and 3 motors to follow 1
        Follower shooter_follower = new Follower(shooterMotorLeft.getDeviceID(), MotorAlignmentValue.Opposed);
        Follower bottom_follower = new Follower(bottomMotorLeft.getDeviceID(), MotorAlignmentValue.Opposed);

        shooterMotorCenter.setControl(shooter_follower);
        shooterMotorRight.setControl(shooter_follower);

        // bottom motors
        bottomMotorRight.setControl(bottom_follower);
    }

    public void setVoltageOverride(double voltage) {
        this.voltageOveride = Optional.of(voltage);
    }

    public Optional<Double> getVoltageOverride() {
        return voltageOveride;
    }

    @Override
    public void periodic() {
        super.periodic();

        if (!Settings.EnabledSubsystems.SHOOTER.get())
            return;

        if (voltageOveride.isPresent()) {
            shooterMotorLeft.setVoltage(voltageOveride.get());
            return;
        }

        double targetRPS = getState().getRPM() / 60;
        VelocityVoltage control = new VelocityVoltage(targetRPS).withEnableFOC(true);
        DutyCycleOut dutyCycle = new DutyCycleOut(getState().getBottomMotorDutyCycle()).withEnableFOC(true);

        shooterMotorLeft.setControl(control);
        bottomMotorLeft.setControl(dutyCycle);

        SmartDashboard.putString("Shooter/State", getState().name());
        this.logMotor("ShooterLeft", shooterMotorLeft);
        this.logMotor("ShooterCenter", shooterMotorCenter);
        this.logMotor("ShooterRight", shooterMotorRight);

        this.logMotor("BottomLeft", bottomMotorLeft);
        this.logMotor("BottomRight", bottomMotorRight);
    }
}
