package com.stuypulse.robot.util.logged.signal;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.ctre.phoenix6.hardware.TalonFX;

public class LoggedSignals implements LoggableInputs, Cloneable {
    private final NamedSignal[] signals;
    private TalonFX motor;

    @SafeVarargs
    protected LoggedSignals(NamedSignal... signals) {
        this.signals = signals;
        this.motor = new TalonFX(-1);
    }

    @Override
    public void toLog(LogTable table) {
        for (var signal : signals) {
            table.put(signal.name(), signal.getter().apply(this.motor).getValue());
        }
    }

    public void updateInputs(TalonFX motor) {
        if (!motor.equals(this.motor)) {
            this.motor = motor;
        }
    }

    @Override
    public void fromLog(LogTable table) {}

    public LoggedSignals clone() {
        return new LoggedSignals(signals);
    }
}
