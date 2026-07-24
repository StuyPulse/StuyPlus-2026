/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.handoff;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Handoff extends SubsystemBase {
    private static final Handoff instance;

    static {
        instance = Robot.isReal() ? new Handoff(new HandoffIOTalonFX()) : new Handoff(new HandoffIOSim());
    }

    public static Handoff getInstance() {
        return instance;
    }

    private final HandoffIO io;
    private final HandoffIOInputsAutoLogged inputs;
    private HandoffState state;

    protected Handoff(HandoffIO io) {
        this.io = io;
        this.inputs = new HandoffIOInputsAutoLogged();
        this.state = HandoffState.IDLE;
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
        IDLE(Settings.Handoff.IDLE_VOLTAGE),
        /** The handoff runs forward. */
        FORWARD(Settings.Handoff.FORWARD_VOLTAGE),
        /** The handoff runs backward. */
        REVERSE(Settings.Handoff.REVERSE_VOLTAGE);

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

    public boolean handoffStalling() {
        return inputs.isStalling;
    };

    @Override
    public void periodic() {
        final HandoffState currentState = getState();

        
        if (Settings.EnabledSubsystems.HANDOFF.get()) {
            io.setTargetVoltage(currentState.getTargetVoltage());
        } else {
            io.stopMotors();
        }
        io.updateInputs(inputs);

        DogLog.log("Handoff/State", currentState.name());
        DogLog.forceNt.log("States/Handoff", currentState.name());
        DogLog.log("Handoff/Handoff Target Voltage", currentState.getTargetVoltage());
        DogLog.forceNt.log("Handoff/Stalling", handoffStalling());
    }
}
