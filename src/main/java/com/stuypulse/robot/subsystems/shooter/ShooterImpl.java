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

    private TalonFX handoffMotor;

    private Optional<Double> voltageOveride;

    public ShooterImpl() {
        shooterMotorLeft = new TalonFX(Ports.ShooterPorts.SHOOTER_MOTOR_LEFT, Settings.CANIVORE);
        shooterMotorCenter = new TalonFX(Ports.ShooterPorts.SHOOTER_MOTOR_CENTER, Settings.CANIVORE);
        shooterMotorRight = new TalonFX(Ports.ShooterPorts.SHOOTER_MOTOR_RIGHT, Settings.CANIVORE);

        handoffMotor = new TalonFX(Ports.ShooterPorts.HANDOFF_MOTOR, Settings.CANIVORE);

        // configure
        Motors.Shooter.SHOOTER_MOTOR_CONFIG.configure(shooterMotorLeft);
        Motors.Shooter.SHOOTER_MOTOR_CONFIG.configure(shooterMotorCenter);
        Motors.Shooter.SHOOTER_MOTOR_CONFIG.configure(shooterMotorRight);
        Motors.Shooter.HANDOFF_MOTOR_CONFIG.configure(handoffMotor);

        // Set shooter 2 and 3 motors to follow 1
        Follower shooter_follower = new Follower(shooterMotorLeft.getDeviceID(), MotorAlignmentValue.Opposed);

        shooterMotorCenter.setControl(shooter_follower);
        shooterMotorRight.setControl(shooter_follower);
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
        DutyCycleOut dutyCycle = new DutyCycleOut(getState().getHandoffMotorDutyCycle()).withEnableFOC(true);

        shooterMotorLeft.setControl(control);
        handoffMotor.setControl(dutyCycle);

        this.logMotor("ShooterLeft", shooterMotorLeft);
        this.logMotor("ShooterCenter", shooterMotorCenter);
        this.logMotor("ShooterRight", shooterMotorRight);

        this.logMotor("Handoff", handoffMotor);
        SmartDashboard.putNumber("Shooter/Motors/Handoff/DutyCycle", handoffMotor.getDutyCycle().getValueAsDouble());
    }
}
