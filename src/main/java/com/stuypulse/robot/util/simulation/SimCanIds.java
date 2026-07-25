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

/**
 * Utility for assigning unique CAN IDs to simulated Phoenix 6 devices.
 *
 * <p>On the real robot, devices may exist on multiple CAN buses (for example,
 * the drivetrain on a CANivore and the rest of the robot on the roboRIO CAN
 * bus), allowing the same numerical CAN ID to be reused across buses.
 *
 * <p>However, Phoenix 6 simulation currently exposes every simulated device on
 * a single virtual CAN bus, regardless of which CAN bus they are configured to
 * use. This means IDs that are valid on the real robot may collide in
 * simulation, preventing devices from functioning correctly.
 *
 * <p>This class works around that limitation by:
 *
 * <ul>
 *   <li>Reserving the real CAN IDs used by the swerve modules (which rely on
 *       their configured IDs for Phoenix's drivetrain simulation).</li>
 *   <li>Assigning unique, unused CAN IDs to every other simulated TalonFX on
 *       demand.</li>
 * </ul>
 *
 * <p>The assigned IDs exist solely for simulation and have no effect on the
 * real robot. Calling {@link #get(String)} on real hardware will throw an
 * exception to prevent accidental misuse.
 *
 * <p>Every assignment is logged through DogLog under
 * {@code Simulation/CAN Assignments/...}, making it easy to determine which
 * simulated device received each CAN ID.
 *
 * <p>This class exists because Phoenix 6 currently does not support simulating
 * multiple independent CAN buses:
 *
 * <blockquote>
 * "Multiple CAN buses using the CANivore API is not supported at this time.
 * All CAN devices will appear on the same CAN bus. If you wish to run your
 * robot code in simulation, ensure devices have unique IDs across CAN buses."
 * </blockquote>
 */
public final class SimCanIds {
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

    /**
     * Reserves every CAN ID used by the given swerve module.
     *
     * <p>The module constants expose the IDs of the drive motor, steer motor,
     * encoder, etc. These IDs are removed from the available simulation pool so
     * they are never reassigned to another simulated device.
     */
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

    /**
     * Records a CAN ID assignment and publishes it to DogLog.
     */
    private static void assign(String key, int id) {
        assignedIds.put(key, id);
        DogLog.log("Simulation/CAN Assignments/" + key, id);
    }

    /**
    * Permanently reserves a CAN ID so it cannot be allocated later.
    */
    private static void reserve(String key, int id) {
        idPool.remove(Integer.valueOf(id));
        assign(key, id);
    }

    /**
     * Returns a unique simulated CAN ID for the given device.
     *
     * <p>The same key will always receive the same ID for the lifetime of the
     * program. New keys are assigned the next available unreserved CAN ID.
     *
     * @param key human-readable identifier used for logging (for example
     *            {@code "Handoff/Motor"})
     * @return a unique CAN ID for simulation
     *
     * @throws IllegalStateException if called on a real robot
     * @throws IllegalStateException if no simulated CAN IDs remain
     */
    public static int get(String key) {
        if (Robot.isReal()) {
            throw new IllegalStateException(
                "SimCanIds.get() was called on a real robot. This utility is used purely for simulation and should not be used on a real robot."
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
