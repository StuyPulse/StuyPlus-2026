package com.stuypulse.robot.util.logged.signal;

import java.util.function.Function;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

public record NamedSignal (String name, Function<TalonFX, StatusSignal<? extends Measure<? extends Unit>>> getter) {
}
