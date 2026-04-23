package com.stuypulse.robot.util;

import java.util.ArrayList;
import java.util.Arrays;

import com.ctre.phoenix6.BaseStatusSignal;

public class PhoenixUtil {
    public static ArrayList<BaseStatusSignal> rioSignals, canivoreSignals;
    static {
        rioSignals = new ArrayList<BaseStatusSignal>();
        canivoreSignals = new ArrayList<BaseStatusSignal>();
    }

    public PhoenixUtil() {}

    public static void registerToRio(BaseStatusSignal... signals) {
        rioSignals.addAll(Arrays.asList(signals));
    }

    public static void registerToCanivore(BaseStatusSignal... signals) {
        canivoreSignals.addAll(Arrays.asList(signals));
    }


    public static void refreshAll() {
        BaseStatusSignal.refreshAll(rioSignals);
        BaseStatusSignal.refreshAll(canivoreSignals);
    }
}