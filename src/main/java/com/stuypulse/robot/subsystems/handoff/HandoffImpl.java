/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.handoff;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.LoggedSignals;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Voltage;

public class HandoffImpl extends Handoff {

    private final TalonFX handoffMotor;

    private final VoltageOut handoffController;

    private final LoggedSignals signals;

    private final BooleanSupplier handoffStalling;
    private final Debouncer handoffDebouncer;

    public HandoffImpl() {
        handoffMotor = new TalonFX(Ports.Handoff.HANDOFF_MOTOR, Settings.CANBUS);
        handoffController = new VoltageOut(getState().getTargetVoltage()).withEnableFOC(true);
        signals = new LoggedSignals(LoggedSignals.SignalLocation.CANIVORE, "Handoff/",
                handoffMotor.getSupplyCurrent(),
                handoffMotor.getStatorCurrent(),
                handoffMotor.getVelocity());
        // Configuring
        Motors.Handoff.HANDOFF_MOTOR_CONFIG.configure(handoffMotor);
        this.handoffStalling = () -> Math.abs(handoffMotor.getStatorCurrent().getValueAsDouble()) > Settings.Handoff.STALL_CURRENT;
        this.handoffDebouncer = new Debouncer(Settings.Handoff.STALL_DEBOUNCE, Debouncer.DebounceType.kRising);
    }

    @Override
    protected void stopMotors() {
        handoffMotor.stopMotor();
    }

    @Override
    protected boolean handoffStalling() {
        return handoffDebouncer.calculate(this.handoffStalling.getAsBoolean());
    }

    @Override
    public void periodic() {
        if (!Settings.EnabledSubsystems.HANDOFF.get()) {
            stopMotors();
            return;
        }

        // States
        final Shooter shooter = Shooter.getInstance();
        final CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();
        if (!(swerve.isAlignedToTarget(Field.getHubPose()))
                && shooter.getState() == ShooterState.SHOOT) {
            setState(HandoffState.IDLE);
        }
        // TODO: consider relaxing tolerances for ferrying
        if (!(swerve.isAlignedToTarget(Field.getFerryZonePose(swerve.getPose().getTranslation())))
                && shooter.getState() == ShooterState.FERRY) {
            setState(HandoffState.IDLE);
        }

        // Control
        final Voltage voltage = handoffStalling()
                ? Handoff.HandoffState.REVERSE.getTargetVoltage()
                : getState().getTargetVoltage();
        final VoltageOut handoffControl = handoffController.withOutput(voltage);
        // Apply
        handoffMotor.setControl(handoffControl);
        // Logging
        this.signals.logAll();
        super.periodic();
    }
}