/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.*;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Feeder extends SubsystemBase {

    private static final Feeder instance;

    private FeederState state;

    static {
        if (Robot.isReal()) {
            instance = new FeederImpl();
        } else {
            instance = new FeederSim();
        }
    }

    public static Feeder getInstance() {
        return instance;
    }

    /** Enum representing the different possible states of the feeder. */
    public enum FeederState {
        /** Feeder is stopped. */
        IDLE(Volts.of(0.0)),
        /** Motors run forward to feed to the shooter. */
        FORWARD(Settings.Feeder.FORWARD_VOLTAGE),
        /** Motors run backward to work with the intake to outtake fuel from the robot. */
        REVERSE(Settings.Feeder.REVERSE_VOLTAGE);

        /** The target voltage of the feeder motors. */
        private Voltage targetVoltage;

        /**
         * Constructs a FeederState with the given target voltage.
         * @param targetVoltage the target voltage of the feeder motors in the corresponding state.
         */
        private FeederState(Voltage targetVoltage) {
            this.targetVoltage = targetVoltage;
        }

        /** 
         * Gets the target voltage of the feeder motors in the corresponding state.
         * @return the target voltage of the feeder motors
         */
        public Voltage getTargetVoltage() {
            return this.targetVoltage;
        }
    }

    protected Feeder() {
        this.state = FeederState.IDLE;
    }

    public void setState(FeederState state) {
        this.state = state;
    }

    public FeederState getState() {
        return state;
    }

    public abstract AngularVelocity getCurrentAngularVelocity();

    protected abstract void stopMotors();

    @Override
    public void periodic() {
        final FeederState currentState = getState();
        // Logging
        DogLog.log("Feeder/Target Voltage", currentState.getTargetVoltage());
        DogLog.log("Feeder/Current RPM", getCurrentAngularVelocity().in(RPM));
        DogLog.log("Feeder/State", currentState.name());
        DogLog.forceNt.log("States/Feeder", currentState.name());
    }
}
