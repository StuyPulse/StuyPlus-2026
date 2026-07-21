/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.util.simulation;

import static edu.wpi.first.units.Units.*;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.handoff.Handoff;
import com.stuypulse.robot.subsystems.handoff.Handoff.HandoffState;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.intake.Intake.IntakeState;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Notifier;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

public class Simulation {

    private static final Simulation instance;

    public final Arena2026Rebuilt arenaInstance;

    private final Notifier shotLoop;

    private final SwerveDriveSimulation swerveMSim;

    private final IntakeSimulation intakeMSim;

    private final Intake intakeSim;

    private final Shooter shooterSim;

    private final Handoff handoffSim;

    private final StructArrayPublisher<Pose3d> fuel;

    private final StructPublisher<Pose3d> intakePivot;

    private final StructPublisher<Pose3d> hopper;

    private final StructArrayPublisher<Pose3d> fuelLayers;

    private final StructPublisher<Pose3d> shooter;

    static {
        if (CommandSwerveDrivetrain.getInstance().getMapleSimDrive() != null)
            instance = new Simulation();
        else
            // extra safeguarding to ensure NO overlap between sim code and actual code
            instance = null;
    }

    public static Simulation getInstance() {
        return instance;
    }

    private Simulation() {
        intakeSim = Intake.getInstance();
        shooterSim = Shooter.getInstance();
        handoffSim = Handoff.getInstance();
        swerveMSim = CommandSwerveDrivetrain.getInstance().getMapleSimDrive();
        arenaInstance = new Arena2026Rebuilt(false);
        configureArena(arenaInstance, swerveMSim);

        intakeMSim = createIntakeSimulation();
        intakeMSim.addGamePiecesToIntake(SimulationConstants.Hopper.FUEL_CAPACITY);

        shotLoop = new Notifier(this::updateSubsystemsBPSLoop);
        shotLoop.startPeriodic(1.0 / SimulationConstants.Shooter.BPS);

        NetworkTableInstance table = NetworkTableInstance.getDefault();
        fuel = table.getStructArrayTopic("AdvScope/FuelPoses", Pose3d.struct).publish();
        intakePivot = table.getStructTopic("AdvScope/IntakePose", Pose3d.struct).publish();
        hopper = table.getStructTopic("AdvScope/HopperPose", Pose3d.struct).publish();
        fuelLayers = table.getStructArrayTopic("AdvScope/FuelLayers", Pose3d.struct).publish();
        shooter = table.getStructTopic("AdvScope/ShooterPose", Pose3d.struct).publish();
    }

    private void configureArena(Arena2026Rebuilt arena, SwerveDriveSimulation drivetrain) {
        arena.setEfficiencyMode(SimulationConstants.SPAWN_GAMEPIECES_SPARSELY);
        arena.resetFieldForAuto();
        arena.addDriveTrainSimulation(drivetrain);
        SimulatedArena.overrideInstance(arena);
    }

    private IntakeSimulation createIntakeSimulation() {
        return IntakeSimulation.OverTheBumperIntake(
                "Fuel",
                swerveMSim,
                Meters.of(SimulationConstants.Intake.INTAKE_WIDTH),
                Meters.of(SimulationConstants.Intake.INTAKE_LENGTH),
                IntakeSimulation.IntakeSide.FRONT,
                SimulationConstants.Hopper.FUEL_CAPACITY);
    }

    private Pose3d getIntakePivotPose() {
        return SimulationConstants.Intake.PIVOT_OFFSETS.withRotation(
                new Rotation3d(
                        0, // inverts the angle
                        intakeSim.getRelativePosition().in(Radians),
                        0));
    }

    private double getIntakeArmEndX() {
        return SimulationConstants.Intake.PIVOT_END_X
                + // sin works because we're zeroed at horizontal
                Settings.Intake.Pivot.PIVOT_ARM_LENGTH.in(Meters)
                        * Math.sin(
                                intakeSim.getRelativePosition().in(Radians)
                                        + SimulationConstants.Intake.PIVOT_OFFSETS.toRotation3d().getX());
    }

    private void updateIntakeEnabled(boolean enabled) {
        if (enabled) {
            intakeMSim.startIntake();
        } else {
            intakeMSim.stopIntake();
        }
    }

    private void updateIntake() {
        boolean intakeEnabled = intakeSim.atTargetAngle()
                && (intakeSim.getState() == IntakeState.DOWN)
                && Settings.EnabledSubsystems.INTAKE.get();
        updateIntakeEnabled(intakeEnabled);
    }

    private void updateHopperFuel() {
        final double hopperPercentage = (double) intakeMSim.getGamePiecesAmount()
                / (double) SimulationConstants.Hopper.FUEL_CAPACITY;
        final int layers = SimulationConstants.Hopper.FUEL_LAYERS;
        final Pose3d visiblePose = SimulationConstants.Hopper.VISIBLE_POSE;
        final Pose3d hiddenPose = SimulationConstants.Hopper.HIDDEN_POSE;
        final Pose3d[] poses = new Pose3d[layers];
        for (int i = 0; i < layers; i++) {
            poses[i] = hopperPercentage >= (1 / (double) layers) * (i + 1) ? visiblePose : hiddenPose;
        }
        fuelLayers.set(poses);
        DogLog.log("Intake/hopperpercentage", hopperPercentage);
    }

    /**
     * <h4>Extension of {@link Arena2026Rebuilt#addPieceWithVariance} that uses
     * chassis speeds</h4>
     *
     * <p>
     * Adds a gamepiece too the arena with a certain random variance.
     *
     * @param piecePose the field relative position at which to spawn the gamepiece
     * @param yaw the initial yaw of the gamepiece
     * @param height the initial height of the gamepiece above the field
     * @param speed the initial speed of the gamepiece
     * @param pitch the initial pitch of the gamepiece
     * @param xVariance the maximum random offset applied to the x coordinate
     * @param yVariance the maximum random offset applied to the y coordinate
     * @param yawVariance the maximum random offset applied to the yaw, in degrees
     * @param speedVariance the maximum random offset applied to the speed, in m/s
     * @param pitchVariance the maximum random offset applied to the pitch, in degrees
     */
    private void robotRelativeAddPieceWithVariance(
            Translation2d piecePose,
            Rotation2d yaw,
            Distance height,
            LinearVelocity speed,
            Angle pitch,
            double xVariance,
            double yVariance,
            double yawVariance,
            double speedVariance,
            double pitchVariance) {
        arenaInstance.addGamePieceProjectile(
                new RebuiltFuelOnFly(
                        piecePose.plus(
                                new Translation2d(
                                        Arena2026Rebuilt.randomInRange(xVariance),
                                        Arena2026Rebuilt.randomInRange(yVariance))),
                        new Translation2d(),
                        swerveMSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                        yaw.plus(Rotation2d.fromDegrees(Arena2026Rebuilt.randomInRange(yawVariance))),
                        height,
                        speed.plus(MetersPerSecond.of(Arena2026Rebuilt.randomInRange(speedVariance))),
                        Degrees.of(pitch.in(Degrees) + Arena2026Rebuilt.randomInRange(pitchVariance))));
    }

    private void summonFuelAtIntake() {
        robotRelativeAddPieceWithVariance(
                swerveMSim
                        .getSimulatedDriveTrainPose()
                        .getTranslation()
                        .plus(
                                SimulationConstants.Intake.OUTTAKE_OFFSETS
                                        .applyToPose3dRobotRelative(
                                                new Pose3d(
                                                        getIntakeArmEndX(),
                                                        0,
                                                        0,
                                                        new Rotation3d(
                                                                swerveMSim.getSimulatedDriveTrainPose().getRotation())))
                                        .getTranslation()
                                        .toTranslation2d()),
                swerveMSim.getSimulatedDriveTrainPose().getRotation(),
                Meters.of(0),
                MetersPerSecond.of(2),
                Radians.of(0), // x
                SimulationConstants.Intake.INTAKE_WIDTH,
                0.0,
                0.0, // speed
                1.0,
                0.0);
    }

    private boolean canShoot() {
        // final ShooterState shooterState = shooterSim.getState();
        // final boolean shooterEnabled = (shooterState == ShooterState.SHOOT || shooterState == ShooterState.MANUAL_HUB || shooterState == ShooterState.FERRY) && Settings.EnabledSubsystems.SHOOTER.get();
        return handoffSim.getState() == HandoffState.FORWARD;
    }

    /**
     * <h4>Custom interval periodic function</h4>
     * <p>Runs at the speed of 1 over the balls per second constant {@link SimulationConstants.Shooter#BPS}</p>
     */
    private void updateSubsystemsBPSLoop() {
        if (intakeSim.getState() == IntakeState.OUTTAKE
                && Settings.EnabledSubsystems.INTAKE.get()
                && intakeMSim.obtainGamePieceFromIntake()) {
            summonFuelAtIntake();
        }
        if (this.canShoot()) {
            final Pose2d shooterPose = SimulationConstants.Shooter.OFFSETS.applyToPose2d(
                    swerveMSim.getSimulatedDriveTrainPose());
            final double launchAngle = 67.67; // random hood exit angle?
            this.robotRelativeAddPieceWithVariance(
                    shooterPose.getTranslation(),
                    swerveMSim.getSimulatedDriveTrainPose().getRotation(),
                    Meters.of(SimulationConstants.Shooter.OFFSETS.toPose3d().getZ()),
                    MetersPerSecond.of(
                            SimulationConstants.Shooter.angularVelocityToMps(
                                    shooterSim.getCurrentAngularVelocity())),
                    Degrees.of(launchAngle),
                    SimulationConstants.Intake.INTAKE_WIDTH,
                    0,
                    0,
                    0.5,
                    0);
        }
    }

    public synchronized void update() {
        if (swerveMSim == null)
            return;
        fuel.set(arenaInstance.getGamePiecesArrayByType("Fuel"));
        updateIntake();
        updateHopperFuel();
        double armEndX = getIntakeArmEndX();
        intakePivot.set(getIntakePivotPose());
        hopper.set(
                SimulationConstants.Hopper.OFFSETS.applyToPose3d(
                        new Pose3d(armEndX, 0, 0, new Rotation3d())));
        // Translation2d outtakeTranslationRobotRelative =
        // swerveMSim.getSimulatedDriveTrainPose().getTranslation().plus(
        // SimulationConstants.Intake.OUTTAKE_OFFSETS.applyToPose3dRobotRelative(
        // new Pose3d(getIntakeArmEndX(), 0, 0, new
        // Rotation3d(swerveMSim.getSimulatedDriveTrainPose().getRotation()))).getTranslation().toTranslation2d());
        // shooter.set(new Pose3d(tra.getX(), tra.getY(), 0, new Rotation3d()));
        // shooter.set(SimulationConstants.Shooter.OFFSETS.applyToPose3dRobotRelative(new
        // Pose3d(swerveMSim.getSimulatedDriveTrainPose())));
    }
}
