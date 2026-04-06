package com.stuypulse.robot.util.simulation;

import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.intake.Intake.IntakeState;
// import com.stuypulse.robot.subsystems.shooter.ShooterSim;
// import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.*;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

public class Simulation {
    private final static Simulation instance;

    public final Arena2026Rebuilt ARENA;
    private final Notifier SHOOT_LOOP;

    private final SwerveDriveSimulation swerveMSim;
    private final IntakeSimulation intakeMSim;
    private final Intake intakeSim;

    private final StructArrayPublisher<Pose3d> fuel;
    private final StructPublisher<Pose3d> intakePivot;
    private final StructPublisher<Pose3d> hopper;
    // private final StructPublisher<Pose3d> shooter;

    static {
        instance = new Simulation();
    }

    public static Simulation getInstance() {
        return instance;
    }

    private Simulation() {
        intakeSim = Intake.getInstance();
        swerveMSim = CommandSwerveDrivetrain.getInstance().getMapleSimDrive();

        ARENA = new Arena2026Rebuilt(false);
        configureDrivetrain(); // do this after creating arena
        intakeMSim = createIntakeSimulation();
        intakeMSim.addGamePiecesToIntake(SimulationConstants.Hopper.FUEL_CAPACITY);
        
        SHOOT_LOOP = new Notifier(this::updateShooting);
        SHOOT_LOOP.startPeriodic(1.0 / SimulationConstants.Shooter.BPS);

        NetworkTableInstance table = NetworkTableInstance.getDefault();
        fuel = table.getStructArrayTopic("AdvScope/FuelPoses", Pose3d.struct).publish();
        intakePivot = table.getStructTopic("AdvScope/IntakePose", Pose3d.struct).publish();
        hopper = table.getStructTopic("AdvScope/HopperPose", Pose3d.struct).publish();
        // shooter = table.getStructTopic("AdvScope/ShooterPose", Pose3d.struct).publish();
    }

    private void configureDrivetrain() {
        ARENA.resetFieldForAuto();
        ARENA.addDriveTrainSimulation(this.swerveMSim);
        SimulatedArena.overrideInstance(ARENA);
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
        return SimulationConstants.Intake.PIVOT_OFFSETS.withRotation(new Rotation3d(
                0,
                intakeSim.getRelativePosition().getRadians(), // inverts the angle
                0));
    }

    private double getIntakeArmEndX() {
        return SimulationConstants.Intake.PIVOT_END_X
                + SimulationConstants.Intake.PIVOT_ARM_LENGTH // sin works somehow??
                        * Math.sin(intakeSim.getRelativePosition().getRadians()
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
        boolean intakeEnabled = intakeSim.atAngle() && (intakeSim.getState() == IntakeState.INTAKE);

        SmartDashboard.putBoolean("Intake/MapleSimIntakeEnabled", intakeEnabled);
        SmartDashboard.putNumber("Intake/MapleSimIntakeCurrentAmount", intakeMSim.getGamePiecesAmount());
        updateIntakeEnabled(intakeEnabled);
    }

    /**
     * <h2>Extension of {@link Arena2026Rebuilt#addPieceWithVariance} that uses chassis speeds</h2>
     * <p>Adds a game piece too the arena with a certain random variance.
     * Param docs taken from {@link Arena2026Rebuilt#addPieceWithVariance}
     *
     * @param info the info of the game piece
     * @param robotPosition the position of the robot (not the shooter) at the time of launching the game piece
     * @param shooterPositionOnRobot the translation from the shooter's position to the robot's center, in the robot's
     *     frame of reference
     * @param chassisSpeedsFieldRelative the field-relative velocity of the robot chassis when launching the game piece,
     *     influencing the initial velocity of the game piece
     * @param shooterFacing the direction in which the shooter is facing at launch
     * @param initialHeight the initial height of the game piece when launched, i.e., the height of the shooter from the
     *     ground
     * @param launchingSpeed the speed at which the game piece is launch
     * @param shooterAngle the pitch angle of the shooter when launching
     * @param xVariance The max amount of variance that should be added too the x coordinate of the game piece.
     * @param yVariance The max amount of variance that should be added too the y coordinate of the game piece.
     * @param yawVariance The max amount of variance that should be added too the yaw of the game piece.
     * @param speedVariance The max amount of variance that should be added too the speed of the game piece.
     * @param pitchVariance The max amount of variance that should be added too the pitch of the game piece.
     * @param target The target of the gamepiece
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
        ARENA.addGamePieceProjectile(
            new RebuiltFuelOnFly(
                piecePose.plus(new Translation2d(Arena2026Rebuilt.randomInRange(xVariance), Arena2026Rebuilt.randomInRange(yVariance))),
                new Translation2d(),
                swerveMSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                yaw.plus(Rotation2d.fromDegrees(Arena2026Rebuilt.randomInRange(yawVariance))),
                height,
                speed.plus(MetersPerSecond.of(Arena2026Rebuilt.randomInRange(speedVariance))),
                Degrees.of(pitch.in(Degrees) + Arena2026Rebuilt.randomInRange(pitchVariance))
            )
        );
    }

    private void summonFuelAtIntake() {
        double intakeAngleRad = intakeSim.getRelativePosition().getRadians()
                + SimulationConstants.Intake.PIVOT_OFFSETS.toRotation3d().getX();

        robotRelativeAddPieceWithVariance(
            swerveMSim.getSimulatedDriveTrainPose().getTranslation().plus(
                SimulationConstants.Intake.OUTTAKE_OFFSETS.applyToPose3dRobotRelative(
                    new Pose3d(getIntakeArmEndX(), 0, 0, 
                    new Rotation3d(swerveMSim.getSimulatedDriveTrainPose().getRotation())
                )).getTranslation().toTranslation2d()),
            swerveMSim.getSimulatedDriveTrainPose().getRotation(),
            Meters.of(0),
            MetersPerSecond.of(2),
            Radians.of(0),
            SimulationConstants.Intake.INTAKE_WIDTH, // x
            0.0,
            0.0,
            1.0, // speed
            0.0
        );
    }

    private void updateShooting() {
        if (!intakeMSim.obtainGamePieceFromIntake())
            return;

        if (intakeSim.getState() == IntakeState.OUTTAKE) {
            summonFuelAtIntake();
        }// else if (ShooterSim.getInstance().getState() == ShooterState.SHOOTING || ShooterSim.getInstance().getState() == ShooterState.FERRYING) {
        //     final Pose2d shooterPose = SimulationConstants.Shooter.OFFSETS.applyToPose2d(swerveMSim.getSimulatedDriveTrainPose());
        //     final double launchAngle = 67.67; // ആറ് ഏഴ്

        //     robotRelativeAddPieceWithVariance(
        //         shooterPose.getTranslation(),
        //         swerveMSim.getSimulatedDriveTrainPose().getRotation(),
        //         Meters.of(SimulationConstants.Shooter.OFFSETS.toPose3d().getZ()),
        //         MetersPerSecond.of(SimulationConstants.Shooter.rpmToMps(ShooterSim.getInstance().getShootSpeed())),
        //         Degrees.of(launchAngle),
        //         SimulationConstants.Intake.INTAKE_WIDTH,
        //         0,
        //         0,
        //         0.5,
        //         0
        //     );
        // }
    }

    public synchronized void update() {
        if (swerveMSim == null)
            return;
        fuel.set(ARENA.getGamePiecesArrayByType("Fuel"));

        updateIntake();

        double armEndX = getIntakeArmEndX();
        intakePivot.set(getIntakePivotPose());
        hopper.set(SimulationConstants.Hopper.OFFSETS.applyToPose3d(new Pose3d(armEndX, 0, 0, new Rotation3d())));

        // Translation2d outtakeTranslationRobotRelative = swerveMSim.getSimulatedDriveTrainPose().getTranslation().plus(
        //         SimulationConstants.Intake.OUTTAKE_OFFSETS.applyToPose3dRobotRelative(
        //             new Pose3d(getIntakeArmEndX(), 0, 0, new Rotation3d(swerveMSim.getSimulatedDriveTrainPose().getRotation()))).getTranslation().toTranslation2d());
        // shooter.set(new Pose3d(tra.getX(), tra.getY(), 0, new Rotation3d()));
        // shooter.set(SimulationConstants.Shooter.OFFSETS.applyToPose3dRobotRelative(new Pose3d(swerveMSim.getSimulatedDriveTrainPose())));
    }
}