/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.feeder.FeederIO.FeederIOOutputMode;
import com.stuypulse.robot.subsystems.feeder.FeederIO.FeederIOOutputs;

import edu.wpi.first.units.measure.*;

import com.stuypulse.robot.util.FullSubsystem;

public class Feeder extends FullSubsystem {
    private static final Feeder instance;

    static {
        switch (Settings.CURRENT_MODE) {
            case REAL -> instance = new Feeder(new FeederIOReal());

            case SIM -> instance = new Feeder(new FeederIOSim());

            default -> instance = new Feeder(new FeederIO() {});
        }
    }

    public static Feeder getInstance() {
        return instance;
    }

    private final FeederIO io;
    private final FeederIOInputsAutoLogged inputs;
    private final FeederIOOutputs outputs;

    @AutoLogOutput(key = "States/Feeder")
    private FeederState state;

    private Feeder(FeederIO io) {
        this.io = io;
        this.inputs = new FeederIOInputsAutoLogged();
        this.outputs = new FeederIOOutputs();
        this.state = FeederState.IDLE;
    }

    public void setState(FeederState state) {
        this.state = state;
    }

    public FeederState getState() {
        return state;
    }

    public enum FeederState {
        IDLE(Volts.of(0.0)),
        FORWARD(FeederConstants.FeederSettings.FORWARD_VOLTAGE),
        REVERSE(FeederConstants.FeederSettings.REVERSE_VOLTAGE);

        private final Voltage targetVoltage;

        private FeederState(Voltage targetVoltage) {
            this.targetVoltage = targetVoltage;
        }

        public Voltage getTargetVoltage() {
            return this.targetVoltage;
        }
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Feeder", inputs);

        if (!Settings.EnabledSubsystems.FEEDER.get()) {
            stopMotor();
            return;
        }

        runVoltage(getState().getTargetVoltage());
    }

    @Override
    public void periodicAfterScheduler() {
        io.applyOutputs(outputs);
    }

    private void runVoltage(Voltage voltage) {
        outputs.mode = FeederIOOutputMode.VOLTAGE;
        outputs.voltage = voltage;
    }

    private void stopMotor() {
        outputs.mode = FeederIOOutputMode.STOP;
    }
}
