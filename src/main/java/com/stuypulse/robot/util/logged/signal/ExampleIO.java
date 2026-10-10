package com.stuypulse.robot.util.logged.signal;

import com.ctre.phoenix6.hardware.TalonFX;

public interface ExampleIO {
    public static class ExampleIOInputs extends LoggedSignals {
        ExampleIOInputs() {
            super(
                new NamedSignal("Motor aura amount", TalonFX::getPosition)
            );
        }
    }

    public default void updateInputs(ExampleIOInputs inputs) {};
}
