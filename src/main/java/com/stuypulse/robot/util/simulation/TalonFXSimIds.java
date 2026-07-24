package com.stuypulse.robot.util.simulation;

import java.lang.reflect.Field;
import java.util.ArrayDeque;
import java.util.HashMap;
import java.util.Map;
import java.util.Queue;

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

        reserveSwerveModule("Swerve (Reserved)/FrontLeft", TunerConstants.FrontLeft);
        reserveSwerveModule("Swerve (Reserved)/FrontRight", TunerConstants.FrontRight);
        reserveSwerveModule("Swerve (Reserved)/BackLeft", TunerConstants.BackLeft);
        reserveSwerveModule("Swerve (Reserved)/BackRight", TunerConstants.BackRight);
    }

    private static void reserveSwerveModule(String key, SwerveModuleConstants<?, ?, ?> module) {
        for (Field field : module.getClass().getFields()) {
            if (field.getType() == int.class
                    && field.getName().toLowerCase().endsWith("id")) {
                try {
                    reserve(key + "/" + field.getName(), field.getInt(module));
                } catch (IllegalAccessException e) {
                    e.printStackTrace();
                }
            }
        }
    }

    private static void assign(String key, int id) {
        assignedIds.put(key, id);
        DogLog.log("Simulation/CAN Assignments/" + key, id);
    }

    private static void reserve(String key, int id) {
        idPool.remove(Integer.valueOf(id));
        assign(key, id);
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

        int id = idPool.remove();
        assign(key, id);
        return id;
    }
}
