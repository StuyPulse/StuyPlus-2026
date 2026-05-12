/**
 * ********************** PROJECT RON ************************
 */
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/**
 * ***********************************************************
 */
package com.stuypulse.robot.util;

import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Set;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.BaseStatusSignal;
import dev.doglog.DogLog;

/**
 * <h2>A container and handler for Phoenix's StatusSignals</h2>
 *
 * <p>Allows for compartmentalization of signals, which can be done by subsystem and by motor, and easy handling and logging.
 * <p>Usage example:
 * <pre>{@code
 * // ... imports
 * public class SubsystemImpl extends SubsystemBase {
 *     private final TalonFX motor1;
 *     private final TalonFX motor2;
 *     private final LoggedSignals signals;
 *
 *     public SubsystemImpl() {
 *         motor1 = new TalonFX(6);
 *         motor2 = new TalonFX(7);
 *         signals = new LoggedSignals(
 *             "Motor Number 1",
 *             motor1.getVelocity(),
 *             motor1.getStatorCurrent(),
 *             motor1.getSupplyCurrent()
 *         ).addMotor(
 *             "Motor Number 2",
 *             motor2.getVelocity(),
 *             motor2.getStatorCurrent(),
 *             motor2.getSupplyCurrent()
 *         );
 *     }
 *
 *     @Override
 *     public void periodic() {
 *         // ... other logic
 *         signals.logAll(); // call after the motor control logic is applied
 *     }
 * }
 * }</pre>
 */
public class LoggedSignals {

    /**
     * <h2>The CAN bus network that a group of signals is from</h2>
     *
     * <p>Each location maintains its own set of signals and a cached {@link List} used for
     * {@link BaseStatusSignal#refreshAll(List)} calls
     */
    public enum SignalLocation {

        RIO, CANIVORE;

        private final Set<BaseStatusSignal> signals;

        private List<BaseStatusSignal> signalList;

        private SignalLocation() {
            this.signals = new HashSet<>();
            cache();
        }

        /**
         * <h4>Converts and stores the set of signals as a list</h4>
         */
        private void cache() {
            this.signalList = List.copyOf(this.signals);
        }

        /**
         * <h4>Gets the set of signals registered to this location</h4>
         *
         * @return the set of registered {@link BaseStatusSignal}s
         */
        public Set<BaseStatusSignal> getSignals() {
            return signals;
        }

        /**
         * <h4>Gets the cached list of signals used for refreshAll calls</h4>
         *
         * <p>Exists because {@link BaseStatusSignal#refreshAll} doesn't accept sets
         * @return an immutable cache of the registered signals
         */
        public List<BaseStatusSignal> getSignalsList() {
            return signalList;
        }

        /**
         * <h4>Adds signals to this location</h4>
         *
         * @param signals signals to register
         */
        public void register(Set<BaseStatusSignal> signals) {
            this.signals.addAll(signals);
            cache();
        }

        /**
         * <h4>Removes signals from this location</h4>
         *
         * @param signals signals to deregister
         */
        public void deregister(Set<BaseStatusSignal> signals) {
            for (BaseStatusSignal signal : signals) this.signals.remove(signal);
            cache();
        }
    }

    /**
     * ************************
     */
    /**
     * INSTANCES & BUILDERS **
     */
    /**
     * ************************
     */
    private final LinkedHashMap<String, Set<BaseStatusSignal>> statusSignals;

    private String logPath;

    private SignalLocation signalLocation;

    /**
     * <h4>Creates a new {@code LoggedSignals} instance with a named motor and defaults to registering signals to {@link SignalLocation#RIO}</h4>
     *
     * <p>The logging path defaults to an empty string.
     * Use {@link #withLogPath(String)} and {@link #withSignalLocation(SignalLocation)} to change it.
     * It also registers the signals for you, so you don't have to call {@link #register()}
     *
     * @param motorName name of the motor whose signals are being registered
     * @param statusSignals the signals to manage, duplicates allowed but filtered out
     */
    public LoggedSignals(String motorName, BaseStatusSignal... statusSignals) {
        this.statusSignals = new LinkedHashMap<>();
        this.statusSignals.put(motorName, Set.of(statusSignals));
        this.logPath = "";
        this.signalLocation = SignalLocation.CANIVORE;
        register();
    }

    /**
     * <h4>Creates a new {@code LoggedSignals} instance with an unnamed motor and defaults to registering signals to {@link SignalLocation#RIO}</h4>
     *
     * <p>The logging path defaults to an empty string.
     * Use {@link #withLogPath(String)} and {@link #withSignalLocation(SignalLocation)} to change it.
     * It also registers the signals for you, so you don't have to call {@link #register()}
     *
     * @param statusSignals the signals to manage, duplicates allowed but filtered out
     */
    public LoggedSignals(BaseStatusSignal... statusSignals) {
        this("", statusSignals);
    }

    /**
     * <h4>Registers a named motor with its own set of signals</h4>
     *
     * <p>Adds to the internal set of the {@link LoggedSignals} instance it is called on,
     * so it uses the same logging path and signal location.
     *
     * @param motorName name of the motor whose signals are being registered
     * @param statusSignals the signals to manage, duplicates allowed but filtered out
     */
    public LoggedSignals withMotor(String motorName, BaseStatusSignal... statusSignals) {
        this.statusSignals.put(motorName, Set.of(statusSignals));
        register();
        return this;
    }

    /**
     * <h4>Moves all signals to a different {@link SignalLocation}</h4>
     *
     * @param location the target signal location
     * @return this instance for chaining
     */
    public LoggedSignals withSignalLocation(SignalLocation location) {
        if (location == this.signalLocation)
            return this;
        deregister();
        this.signalLocation = location;
        register();
        return this;
    }

    /**
     * <h4>Sets the SmartDashboard section to log values to</h4>
     *
     * <p>For example, the path {@code "Shooter/"} will log values like
     * {@code "Shooter/PositionRotations"} if no motor name was specified.
     *
     * @param logPath the prefix string for {@code SmartDashboard} signal logging
     * @return this instance for chaining
     */
    public LoggedSignals withLogPath(String logPath) {
        this.logPath = logPath;
        return this;
    }

    /**
     * ************
     */
    /**
     * REGISTRY **
     */
    /**
     * ************
     */
    /**
     * <h4>Registers all signals in this instance to the current {@link SignalLocation}</h4>
     *
     * <p>SHould be called whenever the internal signal set is added to.
     * Signal locations use a set internally, so no need to worry about duplicates
     */
    public void register() {
        for (final Set<BaseStatusSignal> signals : this.statusSignals.values()) this.signalLocation.register(signals);
    }

    /**
     * <h4>Deregisters all signals in this instance from the current {@link SignalLocation}</h4>
     *
     * <p>Should be called whenever stuff is removed from the internal signal set.
     * Signal locations use a set internally, so no need to worry about duplicates
     */
    public void deregister() {
        for (final Set<BaseStatusSignal> signals : this.statusSignals.values()) this.signalLocation.deregister(signals);
    }

    /**
     * ***********
     */
    /**
     * LOGGING **
     */
    /**
     * ***********
     */
    private void logSignals(String motorName, Set<BaseStatusSignal> signals) {
        for (final BaseStatusSignal signal : signals) {
            DogLog.log(// removes whitespace if motorName is empty
            logPath + String.join(" ", motorName, signal.getName(), signal.getUnits()).trim(), signal.getValueAsDouble());
        }
    }

    /**
     * <h4>Publishes the current value of each signal to SmartDashboard with the specified log path</h4>
     *
     * <p>Should be called after {@link #refreshAll()}. Iterates over the motors and logs the signal values to SmartDashboard
     */
    public void logAll() {
        this.statusSignals.forEach(this::logSignals);
    }

    /**
     * <h4>Refreshes all registered signals</h4>
     *
     * <p>Should be called once per robot periodic loop before reading any signal values or calling {@link #logAll()}.
     */
    public static void refreshAll() {
        for (SignalLocation location : SignalLocation.values()) if (location.getSignalsList().size() > 0)
            BaseStatusSignal.refreshAll(location.getSignalsList());
    }
}
