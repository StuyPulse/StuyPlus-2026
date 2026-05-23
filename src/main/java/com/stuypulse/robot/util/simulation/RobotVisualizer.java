/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.util.simulation;

import static edu.wpi.first.units.Units.*;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.intake.Intake.IntakeState;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import java.util.Arrays;

/**
 * <h2>Combined robot visualizer for all of the subsystems in one canvas.</h2>
 * 
 * <p>
 * This class employs the singleton pattern to ensure that only one visualizer
 * exists,
 * since only one robot instance should exist.
 * </p>
 */
public class RobotVisualizer {
    public static RobotVisualizer instance;

    static {
        instance = new RobotVisualizer();
    }

    public static RobotVisualizer getInstance() {
        return instance;
    }

    private static final double CANVAS_WIDTH = 67, CANVAS_HEIGHT = 67;

    /**
     * Number of spokes for wheels
     * 
     * @see #createSpokes(MechanismObject2d, String, double, double, Color8Bit)
     */
    private static final int NUM_SPOKES = 5;
    /**
     * Color of spokes for wheels
     * 
     * @see #createSpokes(MechanismObject2d, String, double, double, Color8Bit)
     */
    private static Color8Bit SPOKE_COLOR;

    /**
     * Sendable main canvas object for every drawn mechanism
     */
    private final Mechanism2d canvas;

    /**
     * Root of the {@link #bumper red} bumpers to show the silhouette of the robot
     */
    private final MechanismRoot2d bumperRoot;
    /**
     * Drawn representation of red bumpers to show the silhouette of the robot
     * 
     * @see #bumperRoot
     */
    private final MechanismLigament2d bumper;

    /**
     * Root for the drawn representation of the feeder
     * 
     * The actual feeder is a belt, but spokes are easier.
     * Additionally, the {@link #feederSpokes spokes} are placed where the handoff should be...
     */
    private final MechanismRoot2d feederRoot;

    /**
     * Spokes of the feeder
     * 
     * @see #feederRoot
     */
    private final MechanismLigament2d[] feederSpokes;

    /**
     * Root for the drawn representation of the intake
     * 
     * @see #intakePivot
     * @see #intakeSpokes
     */
    private final MechanismRoot2d intakeRoot;
    /**
     * Pivoting arm of the intake.
     * Parent of the {@link #intakeSpokes intake rollers}
     */
    private final MechanismLigament2d intakePivot;
    /**
     * Intake rollers, made into flattened array of every set of rollers for easy iteration
     * 
     * @see #RobotVisualizer()
     */
    private final MechanismLigament2d[] intakeSpokes;

    /**
     * Root for the drawn representation of the shooter
     */
    private final MechanismRoot2d shooterRoot;
    /**
     * Shooter rollers
     * 
     * @see #shooterRoot
     */
    private final MechanismLigament2d[] shooterSpokes;
    /**
     * <h4>Constructs the visualizer instance</h4>
     * <p>Uses specific internal logic, viewing the source code is recommended to better understand how everything is setup.</p>
     */
    private RobotVisualizer() {
        SPOKE_COLOR = new Color8Bit(Color.kWhite);
        canvas = new Mechanism2d(CANVAS_WIDTH, CANVAS_HEIGHT);
        // Silhouette
        bumperRoot = canvas.getRoot("Bumper Root", 10, 5);
        bumper = new MechanismLigament2d("Bumper", 50, 0, 90, new Color8Bit(Color.kRed));
        bumperRoot.append(bumper);
        // Feeder
        feederRoot = canvas.getRoot("Feeder Root", 45, 15);
        feederSpokes = createSpokes(feederRoot, "Feeder Spoke", 6.7, 2);
        // Shooter
        shooterRoot = canvas.getRoot("Shooter Root", 55, 45);
        shooterSpokes = createSpokes(shooterRoot, "Shooter Spoke", 6.7, 2);
        // Intake
        intakeRoot = canvas.getRoot("Intake Root", 15, 10);
        intakePivot = new MechanismLigament2d(
                "Intake Arm",
                9,
                IntakeState.IDLE.getTargetAngle().in(Degrees),
                4,
                new Color8Bit(Color.kGray));
        intakeRoot.append(intakePivot);
        MechanismLigament2d intakeTopRollers = new MechanismLigament2d("Intake Top Rollers", 4, -20, 4,
                new Color8Bit(Color.kGray));
        MechanismLigament2d intakeMiddleRollers = new MechanismLigament2d("Intake Middle Rollers", 4, 150, 4,
                new Color8Bit(Color.kGray));
        MechanismLigament2d intakeBottomRollers = new MechanismLigament2d("Intake Bottom Rollers", 8, 90, 4,
                new Color8Bit(Color.kGray));
        intakeBottomRollers.append(intakeMiddleRollers);
        intakeMiddleRollers.append(intakeTopRollers);
        {
            /* 
             * Draw the other lines in the intake structure.
             * We do this Within a separate scope using {} to ensure the _a and _b variables are temporary.
             */
            var _a = new MechanismLigament2d("_a", 8, 50, 4, new Color8Bit(Color.kGray));
            _a.append(intakeBottomRollers);
            var _b = new MechanismLigament2d("_b", 13.75, 95, 4, new Color8Bit(Color.kGray));
            intakeBottomRollers.append(_b);
            intakePivot.append(_a);
        }
        // Flatten all the sets of intake rollers into one big array
        intakeSpokes = Arrays.stream(
                new MechanismLigament2d[][] {
                        createSpokes(intakeTopRollers, "Intake Top Spoke", .67, 2),
                        createSpokes(intakeMiddleRollers, "Intake Middle Spoke", .67, 2),
                        createSpokes(intakeBottomRollers, "Intake Bottom Spoke", .67, 2)
                })
                .flatMap(Arrays::stream)
                .toArray(MechanismLigament2d[]::new);
        // Publish the canvas to Smartdashboard
        SmartDashboard.putData("Visualizers/Robot", canvas);
    }

    /**
     * 
     * <h4>Helper to create wheels</h4>
     * 
     * <p>Creates a simulacrum of a wheel or side view of a roller
     * by making a bunch of equally-spaced, outwards-radiating lines that like are spokes of a wheel</p>
     * @param target the object to append the spokes to
     * @param name name of each spoke, with name mangling applied internally
     * @param length length of the spokes
     * @param width width of the spokes
     * @param color color of the spokes
     * @return An array of the generated spokes
     */
    private MechanismLigament2d[] createSpokes(
            MechanismObject2d target,
            String name,
            double length,
            double width,
            Color8Bit color) {
        MechanismLigament2d[] spokes = new MechanismLigament2d[NUM_SPOKES];
        double spacing = 360 / NUM_SPOKES;
        for (int i = 0; i < NUM_SPOKES; i++) {
            MechanismLigament2d spoke = new MechanismLigament2d(name.trim() + " " + i, length, spacing * i, width, color);
            target.append(spoke);
            spokes[i] = spoke;
        }
        return spokes;
    }

    /**
     * 
     * <h4>Helper to create wheels</h4>
     * 
     * <p>Creates a simulacrum of a wheel or side view of a roller
     * by making a bunch of equally-spaced, outwards-radiating lines that like are spokes of a wheel.
     * Defaults to the statically specified color {@link #SPOKE_COLOR}</p>
     * @param target the object to append the spokes to
     * @param name name of each spoke, with name mangling applied internally
     * @param length length of the spokes
     * @param width width of the spokes
     * @return An array of the generated spokes
     */
    private MechanismLigament2d[] createSpokes(
            MechanismObject2d target,
            String name,
            double length,
            double width) {
        return createSpokes(target, name, length, width, SPOKE_COLOR);
    }

    /**
     * <h4>Update method for the feeder</h4>
     * 
     * <p>To be called by subsystems using motor measurements</p>
     * @param angularVelocity measured angular velocity of the feeder rollers
     */
    public void updateFeeder(AngularVelocity angularVelocity) {
        double rot = angularVelocity.in(RPM) * 6 * Settings.DT.in(Seconds);
        for (MechanismLigament2d spoke : feederSpokes)
            spoke.setAngle(spoke.getAngle() + rot);
    }

    /**
     * <h4>Update method for the shooter</h4>
     * 
     * <p>To be called by subsystems using motor measurements</p>
     * @param angularVelocity measured angular velocity of the shooter rollers
     */
    public void updateShooter(AngularVelocity angularVelocity) {
        double rot = angularVelocity.in(RPM) * 6 * Settings.DT.in(Seconds);
        for (MechanismLigament2d spoke : shooterSpokes)
            spoke.setAngle(spoke.getAngle() + rot);
    }

    /**
     * <h4>Update method for the intake</h4>
     * 
     * <p>To be called by subsystems using motor measurements</p>
     * @param pivotAngle measured position of the intake pivot
     * @param angularVelocity measured angular velocity of the intake rollers
     */
    public void updateIntake(Angle pivotAngle, AngularVelocity angularVelocity) {
        intakePivot.setAngle(pivotAngle.in(Degrees) + 102); // counteract the weird zeroing
        double rot = angularVelocity.in(RPM) * Settings.DT.in(Seconds);
        for (MechanismLigament2d spoke : intakeSpokes)
            spoke.setAngle(spoke.getAngle() + rot);
    }

    /**
     * <p>Publish the canvas to {@link SmartDashboard}</p>
     */
    public void update() {
        SmartDashboard.putData("Visualizers/Robot", canvas);
    }
}
