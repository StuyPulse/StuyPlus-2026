package com.stuypulse.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;

import java.util.Optional;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.SysId;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ShooterImpl extends Shooter {
    private final TalonFX shooterMotorLeft;
    private final TalonFX shooterMotorCenter;
    private final TalonFX shooterMotorRight;
    private final VelocityTorqueCurrentFOC shooterController;
    private final Follower shooterFollowerController;

    private final TalonFX handoffMotor;
    private final DutyCycleOut handoffController;

    private Optional<Double> voltageOveride;

    public ShooterImpl() {
        shooterMotorLeft = new TalonFX(Ports.ShooterPorts.SHOOTER_MOTOR_LEFT, Settings.CANIVORE); // leader
        shooterMotorCenter = new TalonFX(Ports.ShooterPorts.SHOOTER_MOTOR_CENTER, Settings.CANIVORE);
        shooterMotorRight = new TalonFX(Ports.ShooterPorts.SHOOTER_MOTOR_RIGHT, Settings.CANIVORE);
        shooterController = new VelocityTorqueCurrentFOC(getState().getRPM() / 60);

        handoffMotor = new TalonFX(Ports.ShooterPorts.HANDOFF_MOTOR, Settings.CANIVORE);
        handoffController = new DutyCycleOut(getState().getHandoffMotorDutyCycle()).withEnableFOC(true);

        // configure
        Motors.Shooter.SHOOTER_MOTOR_CONFIG.configure(shooterMotorLeft);
        Motors.Shooter.SHOOTER_MOTOR_CONFIG.configure(shooterMotorCenter);
        Motors.Shooter.SHOOTER_MOTOR_CONFIG.configure(shooterMotorRight);
        Motors.Shooter.HANDOFF_MOTOR_CONFIG.configure(handoffMotor);

        // Set shooter 2 and 3 motors to follow 1
        shooterFollowerController = new Follower(shooterMotorLeft.getDeviceID(), MotorAlignmentValue.Opposed);

        shooterMotorCenter.setControl(shooterFollowerController);
        shooterMotorRight.setControl(shooterFollowerController);

        voltageOveride = Optional.empty();
    }

    public void setVoltageOverride(double voltage) {
        this.voltageOveride = Optional.of(voltage);
    }

    public Optional<Double> getVoltageOverride() {
        return voltageOveride;
    }

    @Override
    public double getCurrentRPM() {
        return shooterMotorLeft.getVelocity().getValue().in(RPM);
    }

    private void stopMotors() {
        shooterMotorLeft.stopMotor();
        shooterMotorCenter.stopMotor();
        shooterMotorRight.stopMotor();

        shooterMotorCenter.setControl(shooterFollowerController);
        shooterMotorRight.setControl(shooterFollowerController);

        handoffMotor.stopMotor();
    }

    @Override
    public void periodic() {
        if (!Settings.EnabledSubsystems.SHOOTER.get()) {
            stopMotors();
            return;
        }

        if (voltageOveride.isPresent()) {
            shooterMotorLeft.setVoltage(voltageOveride.get());
            return;
        }

        final double targetRPS = getState().getRPM() / 60;
        final VelocityTorqueCurrentFOC shooterControl = shooterController.withVelocity(targetRPS);
        final DutyCycleOut handoffControl = handoffController.withOutput(getState().getHandoffMotorDutyCycle());

        shooterMotorLeft.setControl(shooterControl);
        handoffMotor.setControl(handoffControl);

        this.logMotor("ShooterLeft", shooterMotorLeft);
        this.logMotor("ShooterCenter", shooterMotorCenter);
        this.logMotor("ShooterRight", shooterMotorRight);

        this.logMotor("Handoff", handoffMotor);
        SmartDashboard.putNumber("Shooter/Motors/Handoff/DutyCycle", handoffMotor.getDutyCycle().getValueAsDouble());

        super.periodic();
    }

    public SysIdRoutine getShooterSysIdRoutine() {
        return SysId.getRoutine(Settings.Shooter.RAMP_RATE,
                Settings.Shooter.STEP_VOLTAGE,
                "Shooter",
                voltage -> setVoltageOverride(voltage),
                () -> shooterMotorLeft.getPosition().getValueAsDouble(),
                () -> shooterMotorLeft.getVelocity().getValueAsDouble(),
                () -> shooterMotorLeft.getMotorVoltage().getValueAsDouble(),
                getInstance());
    }
}
