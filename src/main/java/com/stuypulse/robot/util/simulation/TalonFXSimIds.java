package com.stuypulse.robot.util.simulation;

import java.util.ArrayList;
import java.util.List;

import java.lang.reflect.Field;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.subsystems.swerve.TunerConstants;

public final class TalonFXSimIds {
    private static final int MAX_SIM_DEVICES = 63;
    private static final List<Integer> idPool = new ArrayList<>();

    static {
        for (int i = 0; i < MAX_SIM_DEVICES; i++) {
            idPool.add(i);
        }

        reserveSwerveModule(TunerConstants.FrontLeft);
        reserveSwerveModule(TunerConstants.FrontRight);
        reserveSwerveModule(TunerConstants.BackLeft);
        reserveSwerveModule(TunerConstants.BackRight);
    }

    private static void reserveSwerveModule(SwerveModuleConstants<?, ?, ?> module) {
        Class<?> classObjectReference = module.getClass();
        Field[] publicFields = classObjectReference.getFields();

        for (Field field: publicFields) {
            if (field.getType() == int.class) {
                if (field.getName().endsWith("Id")) {
                    try {
                        int value = field.getInt(module);
                        idPool.remove(Integer.valueOf(value));
                    } catch (IllegalAccessException e) {
                        e.printStackTrace();
                    }
                }
            }
        }
    }

    public static int get() {
        if (Robot.isReal()) {
            throw new IllegalStateException(
                "TalonFXSimIds.get() was called on a real robot. This utility is used purely for simulation and should not be used on a real robot."
            );
        }

        if (idPool.isEmpty()) {
            throw new IllegalStateException(
                "Out of simulated CAN IDs (" + MAX_SIM_DEVICES + " max)");
        }

        return idPool.remove(0);
    }
}
