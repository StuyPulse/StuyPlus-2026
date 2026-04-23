package com.stuypulse.robot.subsystems.handoff;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;

public class HandoffImpl extends Handoff {
    private final TalonFX handoffMotor;
    private final DutyCycleOut handoffController;
    private final BStream handoffStalling;

    public HandoffImpl() {
        handoffMotor = new TalonFX(Ports.ShooterPorts.HANDOFF_MOTOR, Settings.CANIVORE);
        handoffController = new DutyCycleOut(getState().getHandoffDutyCycle()).withEnableFOC(true);

        // Configuring
        Motors.Handoff.HANDOFF_MOTOR_CONFIG.configure(handoffMotor);

        handoffStalling = BStream.create(
            () -> Math.abs(handoffMotor.getStatorCurrent().getValueAsDouble()) > Settings.Handoff.STALL_CURRENT)
            .filtered(new BDebounce.Rising(Settings.Handoff.STALL_DEBOUNCE));
    }

    @Override
    protected void stopMotors() {
        handoffMotor.stopMotor();
    }

    @Override
    public void periodic() {
        // States
        final Shooter shooter = Shooter.getInstance();
        final CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();

        if (!(swerve.isAlignedToTarget(Field.getHubPose())) && shooter.getState() == ShooterState.SHOOT) {
            setState(HandoffState.IDLE);
        }
        // TODO: consider relaxing tolerances for ferrying
        if (!(swerve.isAlignedToTarget(Field.getFerryZonePose(swerve.getPose().getTranslation()))) && 
        shooter.getState() == ShooterState.FERRY) {
            setState(HandoffState.IDLE);
        }

        // Control
        final double dutyCycle = double dutyCycle = handoffStalling.get()
            ? Handoff.HandoffState.REVERSE.getHandoffDutyCycle()
            : getState().getHandoffDutyCycle();
        final DutyCycleOut handoffControl = handoffController.withOutput(dutyCycle);

        // Apply
        handoffMotor.setControl(handoffControl);

        // Logging
        this.logMotor("Handoff", handoffMotor);
        super.periodic();
    }
}
