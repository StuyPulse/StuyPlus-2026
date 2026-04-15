package com.stuypulse.robot.subsystems.shooter;

import java.util.Optional;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterImpl extends Shooter {
    private TalonFX shooterMotor1;
    private TalonFX shooterMotor2;
    private TalonFX shooterMotor3;

    private TalonFX bottomMotor1;
    private TalonFX bottomMotor2;

    private Optional<Double> voltageOveride;

    public ShooterImpl() {
        shooterMotor1 = new TalonFX(Ports.ShooterPorts.SHOOTER_MOTOR_1, Settings.CANIVORE);
        shooterMotor2 = new TalonFX(Ports.ShooterPorts.BOTTOM_MOTOR_2, Settings.CANIVORE);
        shooterMotor3 = new TalonFX(Ports.ShooterPorts.SHOOTER_MOTOR_3, Settings.CANIVORE);

        bottomMotor1 = new TalonFX(Ports.ShooterPorts.BOTTOM_MOTOR_1, Settings.CANIVORE);
        bottomMotor2 = new TalonFX(Ports.ShooterPorts.BOTTOM_MOTOR_2, Settings.CANIVORE);

        // configure
        Motors.Shooter.MOTOR_CONFIG.configure(shooterMotor1);
        Motors.Shooter.MOTOR_CONFIG.configure(shooterMotor2);
        Motors.Shooter.MOTOR_CONFIG.configure(shooterMotor3);
        Motors.Shooter.MOTOR_CONFIG.configure(bottomMotor1);
        Motors.Shooter.MOTOR_CONFIG.configure(bottomMotor2);

        // Set shooter 2 and 3 motors to follow 1
        Follower _follower = new Follower(shooterMotor1.getDeviceID(), MotorAlignmentValue.Opposed);

        shooterMotor2.setControl(_follower);
        shooterMotor3.setControl(_follower);

        // bottom motors
        // we only need to apply a duty cycle lowk
    }

    public void setVoltageOverride(double voltage) {
        this.voltageOveride = Optional.of(voltage);
    }

    public Optional<Double> getVoltageOverride() {
        return voltageOveride;
    }

    @Override
    public void periodic() {

        if (Settings.EnabledSubsystems.SHOOTER.get()) {
            if (voltageOveride.isPresent()) {
                shooterMotor1.setVoltage(voltageOveride.get());
                return;
            }

            double targetRPS = getState().getRPM() / 60;
            VelocityVoltage control = new VelocityVoltage(targetRPS);

            shooterMotor1.setControl(control);
            bottomMotor1.setControl(control);
        }
        
        SmartDashboard.putString("Shooter/State", this.getState().name());
    }
}
