package com.stuypulse.robot.util;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

import com.ctre.phoenix6.BaseStatusSignal;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * <h2>A container and handler for Phoenix's StatusSignals</h2>
 * 
 * <p>Allows for compartmentalization of signals, usually by subsystem, and easy handling and logging.
 */
public class LoggedSignals {
    /**
     * <h2>The CAN bus network that a group of signals is from</h2>
     *
     * <p>Each location maintains its own set of signals and a cached {@link List} used for
     * {@link BaseStatusSignal#refreshAll(List)} calls
     */
    public enum SignalLocation {
        RIO,
        CANIVORE;

        private final Set<BaseStatusSignal> signals;
        private List<BaseStatusSignal> signalList;

        private SignalLocation() {
            this.signals = new HashSet<>();
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
            for (BaseStatusSignal signal : signals)
                this.signals.remove(signal);
            cache();
        }
    }

    private final Set<BaseStatusSignal> statusSignals;
    private String loggingPath;
    private SignalLocation signalLocation;

    /**
     * <h4>Creates a new {@code LoggedSignals} instance and defaults to registering signals to {@link SignalLocation#RIO}</h4>
     *
     * <p>The logging path defaults to an empty string. Use {@link #withLoggingPath(String)} and {@link #withSignalLocation(SignalLocation)}
     * to change it.
     * It also registers the signals for you, so you don't have to call {@link #register()}
     *
     * @param statusSignals the signals to manage, duplicates allowed but filtered out
     */
    public LoggedSignals(BaseStatusSignal... statusSignals) {
        this.statusSignals = Set.of(statusSignals);
        this.loggingPath = "";
        this.signalLocation = SignalLocation.RIO;
        register();
    }

    /**
     * <h4>Registers all signals in this instance in the current {@link SignalLocation}</h4>
     */
    public void register() {
        this.signalLocation.register(this.statusSignals);
    }

    /**
     * <h4>Deregisters all signals in this instance from the current {@link SignalLocation}</h4>
     */
    public void deregister() {
        this.signalLocation.deregister(this.statusSignals);
    }

    /**
     * <h4>Moves all signals to a different {@link SignalLocation}</h4>
     *
     * @param location the target signal location
     * @return this instance for chaining
     */
    public LoggedSignals withSignalLocation(SignalLocation location) {
        if (location == this.signalLocation) return this;
        deregister();
        this.signalLocation = location;
        register();
        return this;
    }

    /**
     * <h4>Sets the SmartDashboard section to log values to</h4>
     *
     * <p>For example, the path {@code "Shooter/"} will log values like
     * {@code "Shooter/PositionRotations"}.
     *
     * @param loggingPath the prefix string for {@code SmartDashboard} signal logging
     * @return this instance for chaining
     */
    public LoggedSignals withLoggingPath(String loggingPath) {
        this.loggingPath = loggingPath;
        return this;
    }

    /**
     * <h4>Publishes the current value of each signal to SmartDashboard with the specified log path</h4>
     *
     * <p>Should be called after {@link #refreshAll()}. Uses the units and name of the signal to determine the log key.
     */
    public void logAll() {
        for (final BaseStatusSignal signal : statusSignals) {
            SmartDashboard.putNumber(loggingPath + signal.getName() + signal.getUnits(), signal.getValueAsDouble());
        }
    }

    /**
     * <h4>Performs a batch refresh of all registered signals</h4>
     *
     * <p>Should be called once per robot periodic loop before reading any signal values or calling {@link #logAll()}.
     */
    public static void refreshAll() {
        for (SignalLocation location : SignalLocation.values())
            BaseStatusSignal.refreshAll(location.getSignalsList());
    }
}