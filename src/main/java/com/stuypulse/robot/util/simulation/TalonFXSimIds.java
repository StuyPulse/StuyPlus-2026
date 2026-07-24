package com.stuypulse.robot.util.simulation;

import java.util.ArrayDeque;
import java.util.HashMap;
import java.util.Map;
import java.util.Queue;
import java.lang.reflect.Field;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.subsystems.swerve.TunerConstants;

import dev.doglog.DogLog;

public final class TalonFXSimIds {
    private static final int MAX_SIM_DEVICES = 63;

    private static final Map<String, Integer> assignedIds = new HashMap<>();
    private static final Queue<Integer> idPool = new ArrayDeque<>();

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
                if (field.getName().toLowerCase().endsWith("id")) {
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

    public static int get(String key) {
        if (Robot.isReal()) {
            throw new IllegalStateException(
                "TalonFXSimIds.get() was called on a real robot. This utility is used purely for simulation and should not be used on a real robot."
            );
        }

        Integer assigned = assignedIds.get(key);
        if (assigned != null) {
            return assigned;
        }

        if (idPool.isEmpty()) {
            throw new IllegalStateException(
                "Out of simulated CAN IDs (" + MAX_SIM_DEVICES + " max)");
        }

        return assignedIds.computeIfAbsent(key, k -> {
            int id = idPool.remove();

            DogLog.log("Simulation/CAN Assignments/" + key, id);

            return id;
        });
    }
}
