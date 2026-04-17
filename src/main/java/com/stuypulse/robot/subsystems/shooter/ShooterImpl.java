package com.stuypulse.robot.subsystems.shooter;

import java.util.Optional;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
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
        VelocityTorqueCurrentFOC control = new VelocityTorqueCurrentFOC(targetRPS);
        DutyCycleOut dutyCycle = new DutyCycleOut(getState().getBottomMotorDutyCycle()).withEnableFOC(true);

        shooterMotorLeft.setControl(control);
        bottomMotorLeft.setControl(dutyCycle);

        SmartDashboard.putNumber("Shooter/Motors/ShooterLeft/MotorVoltage", shooterMotorLeft.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Motors/ShooterLeft/SupplyCurrent", shooterMotorLeft.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Motors/ShooterLeft/StatorCurrent", shooterMotorLeft.getStatorCurrent().getValueAsDouble());

        SmartDashboard.putNumber("Shooter/Motors/ShooterLeft/DutyCycle", shooterMotorLeft.getDutyCycle().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Motors/ShooterLeft/RPM", shooterMotorLeft.getVelocity().getValueAsDouble());

        SmartDashboard.putNumber("Shooter/Motors/ShooterCenter/MotorVoltage", shooterMotorCenter.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Motors/ShooterCenter/SupplyCurrent", shooterMotorCenter.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Motors/ShooterCenter/StatorCurrent", shooterMotorCenter.getStatorCurrent().getValueAsDouble());

        SmartDashboard.putNumber("Shooter/Motors/ShooterCenter/DutyCycle", shooterMotorCenter.getDutyCycle().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Motors/ShooterCenter/RPM", shooterMotorCenter.getVelocity().getValueAsDouble());

        SmartDashboard.putNumber("Shooter/Motors/ShooterRight/MotorVoltage", shooterMotorRight.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Motors/ShooterRight/SupplyCurrent", shooterMotorRight.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Motors/ShooterRight/StatorCurrent", shooterMotorRight.getStatorCurrent().getValueAsDouble());

        SmartDashboard.putNumber("Shooter/Motors/ShooterRight/DutyCycle", shooterMotorRight.getDutyCycle().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Motors/ShooterRight/RPM", shooterMotorRight.getVelocity().getValueAsDouble());

        SmartDashboard.putNumber("Shooter/Motors/BottomLeft/MotorVoltage", bottomMotorLeft.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Motors/BottomLeft/SupplyCurrent", bottomMotorLeft.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Motors/BottomLeft/StatorCurrent", bottomMotorLeft.getStatorCurrent().getValueAsDouble());

        SmartDashboard.putNumber("Shooter/Motors/BottomLeft/DutyCycle", bottomMotorLeft.getDutyCycle().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Motors/BottomLeft/RPM", bottomMotorLeft.getVelocity().getValueAsDouble());

        SmartDashboard.putNumber("Shooter/Motors/BottomRight/MotorVoltage", bottomMotorRight.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Motors/BottomRight/SupplyCurrent", bottomMotorRight.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Motors/BottomRight/StatorCurrent", bottomMotorRight.getStatorCurrent().getValueAsDouble());

        SmartDashboard.putNumber("Shooter/Motors/BottomRight/DutyCycle", bottomMotorRight.getDutyCycle().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Motors/BottomRight/RPM", bottomMotorRight.getVelocity().getValueAsDouble());
    }
}
