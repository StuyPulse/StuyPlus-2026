/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.handoff;

import static edu.wpi.first.units.Units.Amps;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.handoff.HandoffIO.HandoffIOOutputs;
import com.stuypulse.robot.util.FullSubsystem;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Voltage;

public class Handoff extends FullSubsystem {
    private static final Handoff instance;

    static {
        instance = Robot.isReal() ? new Handoff(new HandoffIOTalonFX()) : new Handoff(new HandoffIOSim());
    }

    public static Handoff getInstance() {
        return instance;
    }

    private final HandoffIO io;
    private final HandoffIOInputsAutoLogged inputs;
    private final HandoffIOOutputs outputs;

    @AutoLogOutput(key = "States/Handoff")
    private HandoffState state;

    private final BooleanSupplier handoffStalling;
    private final Debouncer handoffDebouncer;

    private Handoff(HandoffIO io) {
        this.io = io;
        this.inputs = new HandoffIOInputsAutoLogged();
        this.outputs = new HandoffIOOutputs();
        this.state = HandoffState.IDLE;

        this.handoffStalling = () -> inputs.handoffMotorInputs.statorCurrent.abs(Amps) > HandoffConstants.HandoffSettings.STALL_CURRENT;
        this.handoffDebouncer = new Debouncer(HandoffConstants.HandoffSettings.STALL_DEBOUNCE, DebounceType.kRising);
    }

    public void setState(HandoffState state) {
        this.state = state;
    }

    public HandoffState getState() {
        return this.state;
    }

    /** Enum representing the different possible states of the handoff. */
    public enum HandoffState {
        /** Handoff is stopped. */
        IDLE(HandoffConstants.HandoffSettings.IDLE_VOLTAGE),
        /** The handoff runs forward. */
        FORWARD(HandoffConstants.HandoffSettings.FORWARD_VOLTAGE),
        /** The handoff runs backward. */
        REVERSE(HandoffConstants.HandoffSettings.REVERSE_VOLTAGE);

        /** The target voltage of the handoff motor. */
        private Voltage targetVoltage;

        /**
         * Constructs a HandoffState with the given target voltage.
         * @param targetVoltage the target voltage of the handoff motor in the corresponding state.
         */
        private HandoffState(Voltage targetVoltage) {
            this.targetVoltage = targetVoltage;
        }

        /**
         * Gets the target voltage of the handoff motor in the corresponding state.
         * @return the target voltage of the handoff motor
         */
        public Voltage getTargetVoltage() {
            return targetVoltage;
        }
    }

    @AutoLogOutput(key = "Handoff/isStalling")
    public boolean handoffStalling() {
        return handoffDebouncer.calculate(this.handoffStalling.getAsBoolean());
    };

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Handoff", inputs);
        final HandoffState currentState = getState();
        
        if (Settings.EnabledSubsystems.HANDOFF.get()) {
            runVoltage(currentState.getTargetVoltage());
        } else {
            outputs.mode = HandoffIO.HandoffIOOutputMode.STOP;
        }
    }

    @Override
    public void periodicAfterScheduler() {
        io.applyOutputs(outputs);
    }

    private void runVoltage(Voltage voltage) {
        outputs.mode = HandoffIO.HandoffIOOutputMode.VOLTAGE;
        outputs.voltage = voltage;
    }
}
