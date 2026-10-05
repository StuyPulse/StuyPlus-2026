package com.stuypulse.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

import com.stuypulse.robot.util.logged.LoggedTalonFX.TalonFXInputs;

import edu.wpi.first.units.measure.*;

public interface FeederIO {
    @AutoLog
    public static class FeederIOInputs {
        public TalonFXInputs feederMotorInputs = new TalonFXInputs();
    }

    public default void updateInputs(FeederIOInputs inputs) {};

    public static enum FeederIOOutputMode {
        STOP,
        VOLTAGE
    }

    public static class FeederIOOutputs {
        @AutoLogOutput(key = "Feeder/Mode")
        public FeederIOOutputMode mode = FeederIOOutputMode.VOLTAGE;

        @AutoLogOutput(key = "Feeder/Target Voltage")
        public Voltage voltage = Volts.of(0);
    }

    public default void applyOutputs(FeederIOOutputs outputs) {}
}
