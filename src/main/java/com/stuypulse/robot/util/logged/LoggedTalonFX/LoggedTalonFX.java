package com.stuypulse.robot.util.logged.LoggedTalonFX;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.util.logged.LogTableUtil;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import java.util.ArrayList;
import java.util.List;

/**
 *
 *
 * <h2>Logged TalonFX</h2>
 *
 * <p>A {@link TalonFX} motor controller with built-in signal logging for AdvantageKit.
 *
 * <p>This class wraps the commonly used TalonFX status signals and provides a convenient method for
 * refreshing those signals and copying their values into a {@link TalonFXInputs} object.
 *
 * <h3>Usage</h3>
 *
 * <p>Include a {@link TalonFXInputs} field for each motor that should have its telemetry logged.
 *
 * <pre>{@code
 * @AutoLog
 * public static class IntakeIOInputs {
 *     public TalonFXInputs pivotInputs = new TalonFXInputs();
 *     public TalonFXInputsAutoLogged rollerInputs = new TalonFXInputs();
 * }
 * }</pre>
 *
 * <h3>Creating the Motor</h3>
 *
 * <p>Use {@code LoggedTalonFX} anywhere a normal {@link TalonFX} would be used:
 *
 * <pre>{@code
 * private final LoggedTalonFX pivotMotor;
 *
 * public IntakeIOReal() {
 *     pivotMotor = new LoggedTalonFX(
 *             IntakeDeviceIds.PIVOT,
 *             GlobalSettings.RIO);
 *
 *     // Use `withSignal` to log extra signals beyond the base six.
 *     pivotMotor
 *      .withSignal(pivotMotor.getReverseLimit());
 * }
 * }</pre>
 *
 * <h3>Updating Inputs</h3>
 *
 * <p>Call {@link #updateInputs(TalonFXInputs)} from the subsystem's IO implementation to refresh
 * the motor telemetry:
 *
 * <pre>{@code
 * @Override
 * public void updateInputs(IntakeIOInputs inputs) {
 *     pivotMotor.updateInputs(inputs.pivotInputs);
 *     rollerMotor.updateInputs(inputs.rollerInputs);
 * }
 * }</pre>
 *
 * @see TalonFXInputs
 * @author Faizaan (https://github.com/Faizaan-J)
 */
public class LoggedTalonFX extends TalonFX {
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> statorCurrent;
    private final StatusSignal<Temperature> temperature;
    private final StatusSignal<Angle> position;
    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<AngularVelocity> velocity;

    private final List<StatusSignal<?>> additionalSignals = new ArrayList<>();
    // keep references of alerts to stop java from complaining about resource leak
    private final List<Alert> additionalSignalAlerts = new ArrayList<>();

    public LoggedTalonFX(int deviceId, CANBus bus) {
        super(deviceId, bus);
        supplyCurrent = getSupplyCurrent();
        statorCurrent = getStatorCurrent();
        temperature = getDeviceTemp();
        position = getPosition();
        appliedVoltage = getMotorVoltage();
        velocity = getVelocity();
    }

    public LoggedTalonFX(int deviceId) {
        this(deviceId, new CANBus());
    }

    /**
     * Adds a status signal to be logged and replayed.
     *
     * @param signal The status signal to log.
     * @return This instance of the class, for method chaining.
     */
    public <T> LoggedTalonFX withSignal(StatusSignal<T> signal) {
        if (!LogTableUtil.isSupportedType(signal.getValue())) {
            Alert alert =
                    new Alert(
                            "LoggedTalonFX #" + getDeviceID(),
                            "Unsupported signal type for logging: " + signal.getName(),
                            AlertType.kError);
            alert.set(true);
            additionalSignalAlerts.add(alert);

            return this;
        }

        additionalSignals.add(signal);
        return this;
    }

    /**
     * Refreshes the motor status signals and updates the provided inputs.
     *
     * @param inputs The inputs object to update.
     */
    public void updateInputs(TalonFXInputs inputs) {
        BaseStatusSignal.refreshAll(
                supplyCurrent, statorCurrent, temperature, position, appliedVoltage, velocity);

        if (!additionalSignals.isEmpty()) {
            for (StatusSignal<?> signal : additionalSignals) {
                signal.refresh();
                inputs.recordAdditionalSignal(signal.getName(), signal.getValue());
            }
        }

        inputs.supplyCurrent = supplyCurrent.getValue();
        inputs.statorCurrent = statorCurrent.getValue();
        inputs.temperature = temperature.getValue();
        inputs.position = position.getValue();
        inputs.appliedVoltage = appliedVoltage.getValue();
        inputs.velocity = velocity.getValue();
    }
}
