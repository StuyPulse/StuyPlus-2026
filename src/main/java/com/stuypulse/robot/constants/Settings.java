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
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.util.Color;
import com.ctre.phoenix6.CANBus;
import dev.doglog.DogLog;

import com.pathplanner.lib.path.PathConstraints;
import com.stuypulse.robot.Robot;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use DogLog's tunables in order to have tunable
 * values that we can edit on whatever dashboard we
 * are using.
 */
public interface Settings {

    Time DT = Seconds.of(0.020);

    boolean DEBUG_MODE = true;

    CANBus CANBUS = new CANBus("rio");

    // Change to REPLAY during comp
    Mode SIM_MODE = Mode.SIM;

    Mode CURRENT_MODE = Robot.isReal() ? Mode.REAL : SIM_MODE;

    enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    public interface EnabledSubsystems {

        BooleanSubscriber FEEDER = DogLog.tunable("Enabled Subsystems/Feeder", true);

        BooleanSubscriber INTAKE = DogLog.tunable("Enabled Subsystems/Intake", true);

        // BooleanSubscriber INTAKE_ROLLERS = DogLog.tunable("Enabled Subsystems/Intake/Rollers", true);

        // BooleanSubscriber INTAKE_PIVOT = DogLog.tunable("Enabled Subsystems/Intake/Pivot", true);

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
