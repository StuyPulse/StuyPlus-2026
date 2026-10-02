package com.stuypulse.robot.util.logged.LoggedTalonFX;

import static edu.wpi.first.units.Units.*;

import com.stuypulse.robot.util.logged.LogTableUtil;
import edu.wpi.first.units.measure.*;
import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 *
 *
 * <h2>TalonFX Inputs</h2>
 *
 * <p>Contains logged data collected from a {@link LoggedTalonFX}.
 *
 * <p>A {@code TalonFXInputs} instance is typically stored as a field in a subsystem's {@code
 * IOInputs} class:
 *
 * <pre>{@code
 * @AutoLog
 * public static class IntakeIOInputs {
 *     public TalonFXInputsAutoLogged pivotSignals = new TalonFXInputsAutoLogged();
 *     public TalonFXInputsAutoLogged rollerSignals = new TalonFXInputsAutoLogged();
 * }
 * }</pre>
 *
 * <p>The values are populated by {@link LoggedTalonFX#updateInputs(TalonFXInputs)}.
 *
 * @see LoggedTalonFX
 * @see LogTableUtil
 * @author Faizaan (https://github.com/Faizaan-J)
 */
public class TalonFXInputs implements LoggableInputs, Cloneable {
    public Current supplyCurrent = Amps.zero();
    public Current statorCurrent = Amps.zero();
    public Temperature temperature = Celsius.zero();
    public Angle position = Degrees.zero();
    public Voltage appliedVoltage = Volts.zero();
    public AngularVelocity velocity = DegreesPerSecond.zero();

    private Map<String, Object> additionalSignalValues = new HashMap<>();

    /**
     * Records additional StatusSignals to be logged.
     *
     * @param signals The signals to log.
     */
    void recordAdditionalSignal(String name, Object value) {
        this.additionalSignalValues.put(name, value);
    }

    /**
     * Returns the value of an additional logged StatusSignal, or null if signal is not logged.
     *
     * @param <T> The type of the signal value.
     * @param name The name of the StatusSignal to retrieve.
     * @param type The type of the signal value.
     * @return The value of the signal, or null if not logged.
     */
    public <T> T getAdditionalSignal(String name, Class<T> type) {
        Object value = additionalSignalValues.get(name);
        if (value == null) {
            return null;
        }

        return type.cast(value);
    }

    @Override
    public void toLog(LogTable table) {
        table.put("SupplyCurrent", supplyCurrent);
        table.put("StatorCurrent", statorCurrent);
        table.put("Temperature", temperature);
        table.put("Position", position);
        table.put("AppliedVoltage", appliedVoltage);
        table.put("Velocity", velocity);

        for (Entry<String, Object> entry : additionalSignalValues.entrySet()) {
            LogTableUtil.put(table, entry.getKey(), entry.getValue());
        }
    }

    @Override
    public void fromLog(LogTable table) {
        supplyCurrent = table.get("SupplyCurrent", supplyCurrent);
        statorCurrent = table.get("StatorCurrent", statorCurrent);
        temperature = table.get("Temperature", temperature);
        position = table.get("Position", position);
        appliedVoltage = table.get("AppliedVoltage", appliedVoltage);
        velocity = table.get("Velocity", velocity);

        for (Entry<String, Object> entry : additionalSignalValues.entrySet()) {
            additionalSignalValues.put(
                    entry.getKey(), LogTableUtil.get(table, entry.getKey(), entry.getValue()));
        }
    }

    public TalonFXInputs clone() {
        TalonFXInputs copy = new TalonFXInputs();
        copy.supplyCurrent = this.supplyCurrent;
        copy.statorCurrent = this.statorCurrent;
        copy.temperature = this.temperature;
        copy.position = this.position;
        copy.appliedVoltage = this.appliedVoltage;
        copy.velocity = this.velocity;
        copy.additionalSignalValues = new HashMap<>(this.additionalSignalValues);
        return copy;
    }
}
