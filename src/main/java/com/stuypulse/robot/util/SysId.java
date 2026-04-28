package com.stuypulse.robot.util;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.VoltageUnit;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 *<h2>A class that handles the Mechanism logging and unit conversions required to create a SysId routine</h2>
 */
public class SysId {
    /**
     * <h4><Creates a SysId routine from the parameters/h4>
     *
     * @oaram rampRate - Measured in volts per second, the voltage ramp rate used for quasistatic test routines.
     * @oaram stepVoltage - Measured in volts, the step voltage output used for dynamic test routines.
     * @oaram subsystemName - Name of the subsystem used for logging.
     * @oaram voltageSetter - Double consumer that sets the voltage of the subsystem.
     * @oaram positionSupplier - In rotations, a supplier for the position of the subsystem.
     * @oaram voltageSupplier - A supplier for the voltage of the subsystem's motors.
     * @oaram subsystemInstance - The subsystem containing the motor(s) that is (or are) being characterized.
     */
    public static SysIdRoutine getRoutine(
            double rampRate,
            double stepVoltage,
            String subsystemName,
            Consumer<Double> voltageSetter,
            Supplier<Double> positionSupplier,
            Supplier<Double> velocitySupplier,
            Supplier<Double> voltageSupplier,
            Subsystem subsystemInstance
    ) {
        return new SysIdRoutine(
                new SysIdRoutine.Config(
                        Units.Volts.of(rampRate).per(Second),
                        Units.Volts.of(stepVoltage),
                        null,
                        state -> SignalLogger.writeString(subsystemName + " SysId-State", state.toString())),
                new SysIdRoutine.Mechanism(
                        output -> voltageSetter.accept(output.in(Volts)),
                        state -> {
                            SignalLogger.writeDouble(subsystemName + " Position", positionSupplier.get());
                            SignalLogger.writeDouble(subsystemName + " Velocity", velocitySupplier.get());
                            SignalLogger.writeDouble(subsystemName + " Voltage", voltageSupplier.get());
                        },
                        subsystemInstance));
    }

    /**
     * <h4><Creates a SysId routine from the parameters/h4>
     *
     * <p>Uses WPILib unit classes so inputs can be in any unit.</p>
     *
     * @oaram rampRate - The voltage ramp rate used for quasistatic test routines.
     * @oaram stepVoltage - The step voltage output used for dynamic test routines.
     * @oaram subsystemName - Name of the subsystem used for logging.
     * @oaram voltageSetter - Double consumer that sets the voltage of the subsystem.
     * @oaram positionSupplier - A supplier for the position of the subsystem.
     * @oaram voltageSupplier - A supplier for the voltage of the subsystem's motors.
     * @oaram subsystemInstance - The subsystem containing the motor(s) that is (or are) being characterized.
     */
    public static SysIdRoutine getRoutine(
            Velocity<VoltageUnit> rampRate,
            Voltage stepVoltage,
            String subsystemName,
            Consumer<Voltage> voltageSetter,
            Supplier<Angle> positionSupplier,
            Supplier<AngularVelocity> velocitySupplier,
            Supplier<Voltage> voltageSupplier,
            Subsystem subsystemInstance
    ) {
        return new SysIdRoutine(
                new SysIdRoutine.Config(
                        rampRate,
                        stepVoltage,
                        null,
                        state -> SignalLogger.writeString(subsystemName + " SysId-State", state.toString())),
                new SysIdRoutine.Mechanism(
                        output -> voltageSetter.accept(output),
                        state -> {
                            SignalLogger.writeDouble(subsystemName + " Position", positionSupplier.get().in(Rotations));
                            SignalLogger.writeDouble(subsystemName + " Velocity", velocitySupplier.get().in(RotationsPerSecond));
                            SignalLogger.writeDouble(subsystemName + " Voltage", voltageSupplier.get().in(Volts));
                        },
                        subsystemInstance));
    }
}
