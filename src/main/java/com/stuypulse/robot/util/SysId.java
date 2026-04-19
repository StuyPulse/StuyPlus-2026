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

public class SysId {

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
