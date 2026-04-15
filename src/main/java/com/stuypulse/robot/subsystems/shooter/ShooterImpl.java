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
        Motors.Shooter.SHOOTER_MOTOR_CONFIG.configure(shooterMotor1);
        Motors.Shooter.SHOOTER_MOTOR_CONFIG.configure(shooterMotor2);
        Motors.Shooter.SHOOTER_MOTOR_CONFIG.configure(shooterMotor3);
        Motors.Shooter.SHOOTER_MOTOR_CONFIG.configure(bottomMotor1);
        Motors.Shooter.SHOOTER_MOTOR_CONFIG.configure(bottomMotor2);

        // Set shooter 2 and 3 motors to follow 1
        Follower shooter_follower = new Follower(shooterMotor1.getDeviceID(), MotorAlignmentValue.Opposed);
        Follower bottom_follower = new Follower(bottomMotor1.getDeviceID(), MotorAlignmentValue.Opposed);
        
        shooterMotor2.setControl(shooter_follower);
        shooterMotor3.setControl(shooter_follower);

        // bottom motors
        bottomMotor2.setControl(bottom_follower);
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
            DutyCycleOut dutyCycle = new DutyCycleOut(getState().getBottomMotorDutyCycle());

            shooterMotor1.setControl(control);
            bottomMotor1.setControl(dutyCycle.withEnableFOC(true));
        }
        
        SmartDashboard.putString("Shooter/State", getState().name());
    }
}
