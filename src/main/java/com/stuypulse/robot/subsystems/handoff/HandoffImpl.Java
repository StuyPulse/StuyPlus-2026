package com.stuypulse.robot.subsystems.handoff;

import java.util.Optional;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

public class HandoffImpl extends Handoff {
    private final TalonFX handoffMotor;

    private final DutyCycleOut handoffController;

    private Optional<Voltage> voltageOverride;

    public HandoffImpl() {
        handoffMotor = new TalonFX(Ports.HandoffPorts.HANDOFF_MOTOR, Settings.CANIVORE);
        handoffController = new DutyCycleOut(getState().getHandoffMotorDutyCycle()).withEnableFOC(true);

        // configure
        Motors.Handoff.HANDOFF_MOTOR_CONFIG.configure(handoffMotor);
        
        voltageOverride = Optional.empty();
    }

    public void setVoltageOverride(Voltage voltage) {
        this.voltageOverride = Optional.of(voltage);
    }

    public Optional<Voltage> getVoltageOverride() {
        return voltageOverride;
    }

    @Override
    protected void stopMotors() {
        handoffMotor.stopMotor();
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

        if (voltageOverride.isPresent()) {
            handoffMotor.setVoltage(voltageOverride.get().in(Volts));
            return;
        }

        final DutyCycleOut handoffControl = handoffController.withOutput(getState().getHandoffMotorDutyCycle());

        handoffMotor.setControl(handoffControl);

        this.logMotor("HandoffLeft", handoffMotor);

        SmartDashboard.putNumber("Handoff/Motors/DutyCycle", handoffMotor.getDutyCycle().getValueAsDouble());

        super.periodic();
    }
}