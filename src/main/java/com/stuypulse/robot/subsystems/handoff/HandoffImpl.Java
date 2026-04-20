package com.stuypulse.robot.subsystems.handoff;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

public class HandoffImpl extends Handoff {
    private final TalonFX handoffMotorLeft;
    private final TalonFX handoffMotorRight;

    private final DutyCycleOut handoffController;
    private final Follower handoffFollowerController;

    // private Optional<Voltage> voltageOverride;

    public HandoffImpl() {
        handoffMotorLeft = new TalonFX(Ports.HandoffPorts.HANDOFF_MOTOR_LEFT, Settings.CANIVORE);
        handoffMotorRight = new TalonFX(Ports.HandoffPorts.HANDOFF_MOTOR_RIGHT, Settings.CANIVORE);
        handoffController = new DutyCycleOut(getState().getHandoffMotorDutyCycle()).withEnableFOC(true);

        // configure
        Motors.Handoff.HANDOFF_MOTOR_CONFIG.configure(handoffMotorLeft);
        Motors.Handoff.HANDOFF_MOTOR_CONFIG.configure(handoffMotorRight);

        // Set handoff right motors to follow the left one
        handoffFollowerController = new Follower(handoffMotorLeft.getDeviceID(), MotorAlignmentValue.Opposed);

        handoffMotorRight.setControl(handoffFollowerController);

        // voltageOverride = Optional.empty();
    }

    // public void setVoltageOverride(Voltage voltage) {
    //     this.voltageOverride = Optional.of(voltage);
    // }

    // public Optional<Voltage> getVoltageOverride() {
    //     return voltageOverride;
    // }

    @Override
    protected void stopMotors() {
        handoffMotorLeft.stopMotor();
        handoffMotorRight.stopMotor();

        handoffMotorRight.setControl(handoffFollowerController);
    }

    @Override
    public void periodic() {
        if (!Settings.EnabledSubsystems.HANDOFF.get()) {
            stopMotors();
            return;
        }
        // Stop shooting if not aligned  
        final CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();
        final Shooter shooter = Shooter.getInstance();

        if (!(swerve.isAlignedToTarget(Field.getHubPose())) && shooter.getState() == ShooterState.SHOOT) {
            setState(HandoffState.STOP);
        }

        if (!(swerve.isAlignedToTarget(Field.getFerryZonePose(swerve.getPose().getTranslation()))) && shooter.getState() == ShooterState.FERRY) {
            setState(HandoffState.STOP);
        }

        // if (voltageOverride.isPresent()) {
        //     HandoffMotorLeft.setVoltage(voltageOverride.get().in(Volts));
        //     return;
        // }

        final DutyCycleOut handoffControl = handoffController.withOutput(getState().getHandoffMotorDutyCycle());

        handoffMotorLeft.setControl(handoffControl);

        this.logMotor("HandoffLeft", handoffMotorLeft);
        this.logMotor("HandoffRight", handoffMotorRight);

        SmartDashboard.putNumber("Handoff/Motors/DutyCycle", handoffMotorLeft.getDutyCycle().getValueAsDouble());

        super.periodic();
    }
}