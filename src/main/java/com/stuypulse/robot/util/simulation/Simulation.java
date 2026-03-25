package com.stuypulse.robot.util.simulation;

import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.swerve.TunerConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;

import static edu.wpi.first.units.Units.*;

import java.util.Arrays;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.MapleMotorSim;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;

public class Simulation {
    private final static Simulation instance;

    public final Arena2026Rebuilt ARENA;

    private final SwerveDriveSimulation swerveSim;
    // private final IntakeSimulation mapleSimIntake;
    private final Intake intakeSim;

    private final StructPublisher<Pose2d> drivetrain;
    private final StructArrayPublisher<SwerveModuleState> swerve;
    private final StructPublisher<ChassisSpeeds> chassis;
    private final StructArrayPublisher<Pose3d> fuel;
    private final StructPublisher<Pose3d> intakePivot;
    private final StructPublisher<Pose3d> intakeRollers;


    static {
        instance = new Simulation();
   }

    public static Simulation getInstance() {
        return instance;
    }

    public Simulation() {
        intakeSim = Intake.getInstance();
        swerveSim = MapleSimSwerveDrivetrain.getInstance().mapleSimDrive;


        ARENA = new Arena2026Rebuilt();

        NetworkTableInstance table = NetworkTableInstance.getDefault();
        drivetrain = table.getStructTopic("AdvScope/DTPose", Pose2d.struct).publish();
        swerve = table.getStructArrayTopic("AdvScope/SwerveStates", SwerveModuleState.struct).publish();
        chassis = table.getStructTopic("AdvScope/ChassisSpeeds", ChassisSpeeds.struct).publish();
        fuel = table.getStructArrayTopic("AdvScope/FuelPoses", Pose3d.struct).publish();
        intakePivot = table.getStructTopic("AdvScope/IntakePose", Pose3d.struct).publish();
        intakeRollers = table.getStructTopic("AdvScope/IntakeRollerPose", Pose3d.struct).publish();
    }

    private void configureDrivetrain() {
        ARENA.resetFieldForAuto();
        ARENA.addDriveTrainSimulation(this.swerveSim);
        SimulatedArena.overrideInstance(ARENA);
    }

    private IntakeSimulation createIntakeSimulation() {
        return IntakeSimulation.OverTheBumperIntake(
            "Fuel",
            swerveSim,
            Meters.of(SimulationConstants.Intake.INTAKE_WIDTH),
            Meters.of(SimulationConstants.Intake.INTAKE_LENGTH),
            IntakeSimulation.IntakeSide.FRONT,
            SimulationConstants.Hopper.FUEL_CAPACITY
        );
    }


    

    private Pose3d getIntakePivotPose() {
        return SimulationConstants.Intake.PIVOT_OFFSETS.withRotation(new Rotation3d(
            0,
            Math.toRadians(180) - intakeSim.getRelativePosition().getRadians(), // inverts the angle
            0
        ));
    }
    private double getIntakeArmEndX() {
        return SimulationConstants.Intake.PIVOT_END_X
            + SimulationConstants.Intake.PIVOT_ARM_LENGTH
            * Math.cos(Math.toRadians(intakeSim.getRelativePosition().getDegrees()));
    }
    private Pose3d getHopperPose(double armEndX) {
        return SimulationConstants.Hopper.OFFSETS.applyToPose3d(new Pose3d(armEndX, 0, 0, new Rotation3d()));
    }
    private Pose3d getIntakeRollerPose(double armEndX) {
        return SimulationConstants.Intake.ROLLER_OFFSETS.applyToPose3d(new Pose3d(
            armEndX,
            0,
            SimulationConstants.Intake.PIVOT_ARM_LENGTH * Math.sin(Math.toRadians(intakeSim.getRelativePosition().getDegrees())),
            new Rotation3d()
        ));
    }

    public synchronized void update() {
        if (swerveSim == null) return;

        drivetrain.set(swerveSim.getSimulatedDriveTrainPose());
        swerve.set(Arrays.stream(swerveSim.getModules())
            .map(SwerveModuleSimulation::getCurrentState)
            .toArray(SwerveModuleState[]::new));
        chassis.set(swerveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative());

        fuel.set(ARENA.getGamePiecesArrayByType("Fuel"));

        double armEndX = getIntakeArmEndX();
        intakePivot.set(getIntakePivotPose());
        intakeRollers.set(getIntakeRollerPose(armEndX));
    }
}
