/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.util;

import com.ctre.phoenix6.BaseStatusSignal;
import dev.doglog.DogLog;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Set;

public final class LoggedSignals {
    public enum SignalLocation {
        RIO,
        CANIVORE;

        private final Set<BaseStatusSignal> signalSet = new HashSet<>();

        private BaseStatusSignal[] signalArray = new BaseStatusSignal[0];

        public boolean register(BaseStatusSignal... signals) {
            boolean changed = false;
            for (final BaseStatusSignal s : signals)
                changed |= signalSet.add(s); // 67
            if (changed)
                cache();
            return changed;
        }

        public boolean deregister(BaseStatusSignal[] signals) {
            boolean changed = false;
            for (final BaseStatusSignal s : signals)
                changed |= signalSet.remove(s);
            if (changed)
                cache();
            return changed;
        }

        private void cache() {
            signalArray = signalSet.toArray(signalArray);
        }

        public BaseStatusSignal[] getSignals() {
            return signalArray;
        }
    }

    public static void refreshAll() {
        for (final SignalLocation loc : SignalLocation.values()) {
            final BaseStatusSignal[] arr = loc.getSignals();
            if (arr.length != 0)
                BaseStatusSignal.refreshAll(arr);
        }
    }

    private final ArrayList<BaseStatusSignal> signals;
    private final ArrayList<String> keys;

    private final String logPath;
    private final SignalLocation signalLocation;

    public LoggedSignals(
            SignalLocation location,
            String logPath,
            String motorName,
            BaseStatusSignal... statusSignals) {
        this.logPath = logPath.endsWith("/") ? logPath : logPath + "/";
        this.signalLocation = location;
        this.signals = new ArrayList<>();
        this.keys = new ArrayList<>();
        this.withMotor(motorName, statusSignals);
    }

    public LoggedSignals(
            SignalLocation location,
            String logPath,
            BaseStatusSignal... statusSignals) {
        this(location, logPath, "", statusSignals);
    }

    public LoggedSignals withMotor(String motorName, BaseStatusSignal... statusSignals) {
        for (final BaseStatusSignal s : statusSignals) {
            if (signalLocation.register(s)) { // yo this logic is genuinely evil
                signals.add(s);
                keys.add(getLogKey(motorName, s));
            }
        }
        return this;
    }

    public void logAll() {
        for (int i = 0; i < signals.size(); i++)
            DogLog.log(keys.get(i), signals.get(i).getValueAsDouble());
    }

    private String getLogKey(String motorName, BaseStatusSignal signal) {
        final String unit = signal.getUnits();
        return logPath
            + (motorName.isEmpty() ? "" : motorName + "_")
            + signal.getName()
            + (unit.isEmpty() ? "" : "_" + unit);
    }
}