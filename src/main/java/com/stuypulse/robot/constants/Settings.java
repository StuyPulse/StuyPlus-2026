/**
 * ********************** PROJECT RON ************************
 */
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/**
 * ***********************************************************
 */
package com.stuypulse.robot.constants;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotBase;
import com.ctre.phoenix6.CANBus;

import dev.doglog.DogLog;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use DogLog's tunables in order to have tunable
 * values that we can edit on whatever dashboard we
 * are using.
 */
public interface Settings {

    Time DT = Milliseconds.of(20);

    boolean DEBUG_MODE = true;

    CANBus CANBUS = new CANBus("rio");

    Mode SIMULATION_TASK = Mode.SIM; // What to do during simulation mode. Change this to REPLAY when replaying. Change to SIM when simulating code.
    Mode CURRENT_MODE = RobotBase.isReal() ? Mode.REAL : SIMULATION_TASK;
    
    VisionMode VISION_MODE = VisionMode.LIMELIGHT_VISION;

    enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    enum VisionMode {
        LIMELIGHT_VISION,
        PHOTON_VISION
    }

    public interface EnabledSubsystems {

        BooleanSubscriber FEEDER = DogLog.tunable("Enabled Subsystems/Feeder", true);

        BooleanSubscriber INTAKE = DogLog.tunable("Enabled Subsystems/Intake", true);

        // BooleanSubscriber INTAKE_ROLLERS = DogLog.tunable("Enabled
        // Subsystems/Intake/Rollers", true);

        // BooleanSubscriber INTAKE_PIVOT = DogLog.tunable("Enabled
        // Subsystems/Intake/Pivot", true);

        BooleanSubscriber LED = DogLog.tunable("Enabled Subsystems/LED", false);

        BooleanSubscriber HANDOFF = DogLog.tunable("Enabled Subsystems/Handoff", true);

        BooleanSubscriber SHOOTER = DogLog.tunable("Enabled Subsystems/Shooter", true);

        BooleanSubscriber VISION = DogLog.tunable("Enabled Subsystems/Vision", true);

        BooleanSubscriber SWERVE = DogLog.tunable("Enabled Subsystems/Swerve", true);
    }

    public interface Driver {

        double BUZZ_TIME = 1.0;

        double BUZZ_INTENSITY = 1.0;

        public interface Drive {

            double DEADBAND = 0.05;

            double RC = 0.05;

            double POWER = 2.0;
        }

        public interface Turn {

            double DEADBAND = 0.07;

            double RC = 0.05;

            double POWER = 2.0;
        }
    }
}
