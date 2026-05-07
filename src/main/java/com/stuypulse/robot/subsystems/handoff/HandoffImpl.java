/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.handoff;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.LoggedSignals;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class HandoffImpl extends Handoff {
    private final TalonFX handoffMotor;
    private final DutyCycleOut handoffController;
    private final LoggedSignals signals;
    private final BStream handoffStalling;

    public HandoffImpl() {
        handoffMotor = new TalonFX(Ports.Handoff.HANDOFF_MOTOR, Settings.CANBUS);
        handoffController = new DutyCycleOut(getState().getHandoffDutyCycle()).withEnableFOC(true);
        signals = new LoggedSignals(
            handoffMotor.getSupplyCurrent(),
            handoffMotor.getStatorCurrent(),
            handoffMotor.getVelocity()
        ).withLogPath("Handoff/").withSignalLocation(LoggedSignals.SignalLocation.CANIVORE);

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
    public Trigger handoffStalling() {
        return new Trigger(handoffStalling);
    }

    @Override
    public void periodic() {
        // Control
        final DutyCycleOut handoffControl = handoffController.withOutput(getState().getHandoffDutyCycle());

        // Apply
        handoffMotor.setControl(handoffControl);

        // Logging
        this.signals.logAll();
        super.periodic();
    }
}
